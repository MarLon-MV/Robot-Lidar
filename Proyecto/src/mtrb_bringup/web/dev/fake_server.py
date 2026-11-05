"""
Fake MTRB web bridge for developing the UI without the robot.

Usage (from Proyecto/src/mtrb_bringup/web/dev):
    python fake_server.py [--port 8080] [--token dev-token] [--mode map|mapping]
                          [--fail none|409|503|401|400|teleop|save409|save503|saveslow|savetimeout|
                                savebackup|savelimit|savedisk|saveratelimit|
                                initpose400|initpose409|initpose503|stop503]
                          [--no-dock] [--maps lab,casa] [--no-posegraph] [--stale]
--fail makes POST /api/goal, /api/dock and /api/cancel answer that error
(409 goal_active, 503 nav_unavailable, 401 unauthorized, 400 invalid_goal).
--mode map lets /api/initialpose succeed; in mapping it answers 409 wrong_mode.
--fail teleop makes every WS teleop answer {"type":"error","code":"teleop_rejected"}.
WS teleop/stop and POST /api/stop are accepted like the real server: seq must strictly
increase per connection, messages above 30 Hz are dropped, vx/wz are saturated; everything
received is printed (flush) so the client can be checked by eye.
The dock pose is published in status.dock.
Map saving (POST /api/map/save, GET /api/maps): only in mode mapping (409 wrong_mode otherwise),
name ^[A-Za-z0-9_][A-Za-z0-9_-]{0,39}$ (400 invalid_name), "map" reserved in any case
(400 reserved_name), an existing name answers 409 map_exists unless
"overwrite": true. --fail save409 -> 409 save_in_progress, save503 -> 503 slam_unavailable,
saveslow -> OK after 4 s (tests the "Guardando…" state), savetimeout -> 504 save_timeout after 2 s,
savebackup -> 409 backup_pending, savelimit -> 409 map_limit, savedisk -> 507
insufficient_storage, saveratelimit -> 429 save_rate_limited.
--maps sets the initial saved maps (comma separated; hostile names are allowed to test escaping),
--no-posegraph makes saves answer posegraph:false, --stale publishes scan_age/odom_age > 2 s.
--fail initpose400|initpose409|initpose503 make POST /api/initialpose answer 400 invalid_pose,
409 goal_active or 503 no_map (after body validation).
Token brute-force limit like the real server: 10 wrong tokens in 60 s from one IP lock that IP
for 60 s: REST answers 429 too_many_attempts (except POST /api/stop with a VALID token, which
always answers 200) and the WS handshake is rejected with HTTP 403 BEFORE accepting (a wrong
token or a locked IP never gets accept + close 1008). The WS also checks Origin against Host
(mismatch -> rejected before accept; no Origin -> allowed). Every response carries the real
security headers (X-Frame-Options, nosniff, Referrer-Policy, CSP).
--fail stop503 makes POST /api/stop answer 503 (saturated server) to test the PARAR message.
The status message carries nodes, scan_age and odom_age like the real server.
Then open http://localhost:8080/ and enter the token. Not installed with the package.
"""
import argparse
import asyncio
import json
import math
import os
import re
import secrets
import struct
import time
from urllib.parse import urlsplit
import zlib

from fastapi import FastAPI, Request, WebSocket
from fastapi.responses import FileResponse, JSONResponse
import uvicorn

WEB_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATIC = {
    '/': ('index.html', 'text/html'),
    '/app.js': ('app.js', 'application/javascript'),
    '/style.css': ('style.css', 'text/css'),
}
WIDTH, HEIGHT, RES = 200, 160, 0.05
ORIGIN = (-5.0, -4.0)
SECURITY_HEADERS = {
    'X-Frame-Options': 'DENY',
    'X-Content-Type-Options': 'nosniff',
    'Referrer-Policy': 'no-referrer',
    'Content-Security-Policy': (
        "default-src 'self'; script-src 'self'; style-src 'self'; "
        "connect-src 'self' ws: wss:; img-src 'self' data: blob:; "
        "frame-ancestors 'none'; base-uri 'none'; form-action 'self'"),
}


def make_map():
    """Return int8 cells (row-major, origin bottom-left): walls, a box, unknown strip."""
    cells = bytearray(WIDTH * HEIGHT)
    for j in range(HEIGHT):
        for i in range(WIDTH):
            v = 0
            if i < 3 or i >= WIDTH - 3 or j < 3 or j >= HEIGHT - 3:
                v = 100
            elif 80 <= i < 100 and 60 <= j < 90:
                v = 100
            elif i >= WIDTH - 25:
                v = -1
            cells[j * WIDTH + i] = v & 0xFF
    return bytes(cells)


AUTH_MAX_FAILS, AUTH_WINDOW_S, AUTH_LOCK_S = 10, 60.0, 60.0
INIT_FAILS = {
    'initpose400': (400, 'invalid_pose', 'Pose outside the map'),
    'initpose409': (409, 'goal_active', 'Navigation or teleop active'),
    'initpose503': (503, 'no_map', 'No map yet'),
}
FAILURES = {
    '409': (409, 'goal_active', 'A goal is already active'),
    '503': (503, 'nav_unavailable', 'Navigation is not available'),
    '401': (401, 'unauthorized', 'Missing or invalid token'),
    '400': (400, 'invalid_goal', 'Goal is not valid'),
}
MAX_VX, MAX_WZ = 0.2, 0.8   # INITIAL, as in web.yaml
MAX_RATE_HZ = 30.0
DOCK = {'x': -3.5, 'y': -2.5, 'yaw': 0.0}
NAME_RE = re.compile(r'^[A-Za-z0-9_][A-Za-z0-9_-]{0,39}$')
SAVE_FAILS = ('save409', 'save503', 'saveslow', 'savetimeout',
              'savebackup', 'savelimit', 'savedisk', 'saveratelimit')
SAVE_ERRORS = {
    'savebackup': (409, 'backup_pending', 'Unresolved backup'),
    'savelimit': (409, 'map_limit', 'Too many maps'),
    'savedisk': (507, 'insufficient_storage', 'Low disk space'),
    'saveratelimit': (429, 'save_rate_limited', 'Too fast'),
}


def _pose_ok(body):
    """Return True if body is exactly {x, y, yaw} with finite non-bool numbers."""
    return (isinstance(body, dict) and set(body) == {'x', 'y', 'yaw'}
            and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                    and math.isfinite(v) for v in body.values()))


class AuthLimiter:
    """Per-IP limit on wrong tokens: AUTH_MAX_FAILS in AUTH_WINDOW_S locks the IP."""

    def __init__(self):
        """Start with no recorded failures."""
        self.fails = {}
        self.locked = {}

    def is_locked(self, ip):
        """Return True while `ip` is locked out."""
        return self.locked.get(ip, 0.0) > time.monotonic()

    def record_failure(self, ip):
        """Count one wrong token from `ip`; lock it when the limit is reached."""
        now = time.monotonic()
        recent = [t for t in self.fails.get(ip, []) if now - t < AUTH_WINDOW_S] + [now]
        self.fails[ip] = recent
        if len(recent) >= AUTH_MAX_FAILS:
            self.locked[ip] = now + AUTH_LOCK_S
            self.fails[ip] = []
            log('auth lockout for', ip)


def _ip(conn):
    """Return the client IP of a request or websocket."""
    return conn.client.host if conn.client else '?'


def log(*parts):
    """Print one line immediately (the dev server is watched live)."""
    print('[fake]', *parts, flush=True)


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def handle_client(raw, conn, nav, fail, outbox):
    """Process one client text frame; errors are queued in `outbox` like the real server."""
    try:
        data = json.loads(raw)
    except ValueError:
        data = None
    kind = data.get('type') if isinstance(data, dict) else None
    if kind == 'stop':
        nav.update(state='canceled', distance=None)
        log('WS stop')
    elif kind == 'teleop':
        now = time.monotonic()
        seq = data.get('seq')
        if not (_num(data.get('vx')) and _num(data.get('wz'))
                and isinstance(seq, int) and not isinstance(seq, bool)):
            log('WS teleop INVALID', raw[:80])
            outbox.append({'type': 'error', 'code': 'invalid_teleop', 'detail': 'Bad teleop'})
        elif fail == 'teleop' or nav['state'] == 'active':
            log('WS teleop REJECTED seq=%s' % seq)
            outbox.append({'type': 'error', 'code': 'teleop_rejected', 'detail': 'Rejected'})
        elif seq <= conn['seq']:
            log('WS teleop DROPPED old/repeated seq=%d (last %d)' % (seq, conn['seq']))
        elif now - conn['t'] < 1.0 / MAX_RATE_HZ:
            log('WS teleop DROPPED rate seq=%d' % seq)
        else:
            conn.update(seq=seq, t=now)
            vx = max(-MAX_VX, min(MAX_VX, data['vx']))
            wz = max(-MAX_WZ, min(MAX_WZ, data['wz']))
            log('WS teleop seq=%d vx=%.3f wz=%.3f%s' % (
                seq, vx, wz, ' (saturated)' if (vx, wz) != (data['vx'], data['wz']) else ''))
    else:
        log('WS unsupported', raw[:80])
        outbox.append({'type': 'error', 'code': 'unsupported_message', 'detail': 'Unsupported'})


def create_app(token, mode='mapping', fail='none', with_dock=True, maps=None,
               posegraph=True, stale=False):
    """Build the fake app serving static files, /api/*, and /ws."""
    app = FastAPI()

    @app.middleware('http')
    async def security_headers(request: Request, call_next):
        response = await call_next(request)
        for key, value in SECURITY_HEADERS.items():
            response.headers[key] = value
        return response

    limiter = AuthLimiter()
    nav = {'state': 'idle', 'distance': None, 'log': []}
    saved = list(maps if maps is not None else ['lab', 'casa'])
    saving = {'busy': False}
    cells = make_map()
    payload = struct.pack('<I', 1) + zlib.compress(cells, 1)
    meta = {'type': 'map_meta', 'version': 1, 'width': WIDTH, 'height': HEIGHT,
            'resolution': RES, 'origin_x': ORIGIN[0], 'origin_y': ORIGIN[1]}

    def serve(name, media):
        async def handler():
            return FileResponse(os.path.join(WEB_DIR, name), media_type=media)
        return handler

    for route, (name, media) in STATIC.items():
        app.add_api_route(route, serve(name, media), methods=['GET'], include_in_schema=False)

    def guard(request):
        """Return an error response for a locked IP or a bad token, else None."""
        ip = _ip(request)
        if limiter.is_locked(ip):
            return JSONResponse({'error': 'too_many_attempts', 'detail': 'Locked'}, 429)
        auth = request.headers.get('authorization', '')
        if not secrets.compare_digest(auth, 'Bearer ' + token):
            limiter.record_failure(ip)
            return JSONResponse({'error': 'unauthorized', 'detail': 'Invalid token'}, 401)
        return None

    @app.get('/api/state')
    async def state(request: Request):
        bad = guard(request)
        if bad:
            return bad
        return {'mode': mode, 'pose': None, 'map': meta}

    def forced(path):
        if fail in FAILURES and path != '/api/initialpose':
            code, err, detail = FAILURES[fail]
            return JSONResponse({'error': err, 'detail': detail}, code)
        return None

    async def read_body(request):
        try:
            return await request.json()
        except Exception:  # empty or malformed body
            return None

    @app.post('/api/goal')
    async def goal(request: Request):
        bad = guard(request) or forced('/api/goal')
        if bad:
            return bad
        if not _pose_ok(await read_body(request)):
            return JSONResponse({'error': 'invalid_body', 'detail': 'Bad body'}, 400)
        if nav['state'] == 'active':
            return JSONResponse({'error': 'goal_active', 'detail': 'Goal active'}, 409)
        nav.update(state='active', distance=5.0)
        return {'ok': True, 'goal_id': 'fake'}

    @app.post('/api/dock')
    async def dock(request: Request):
        bad = guard(request) or forced('/api/dock')
        if bad:
            return bad
        if nav['state'] == 'active':
            return JSONResponse({'error': 'goal_active', 'detail': 'Goal active'}, 409)
        nav.update(state='active', distance=8.0)
        return {'ok': True, 'goal_id': 'fake-dock'}

    @app.post('/api/cancel')
    async def cancel(request: Request):
        bad = guard(request) or forced('/api/cancel')
        if bad:
            return bad
        nav.update(state='canceled', distance=None)
        return {'ok': True}

    @app.post('/api/stop')
    async def stop(request: Request):
        # Valid token: always 200, even from a locked IP (the stop must never be refused).
        auth = request.headers.get('authorization', '')
        if secrets.compare_digest(auth, 'Bearer ' + token):
            if fail == 'stop503':
                return JSONResponse({'error': 'nav_unavailable', 'detail': 'Saturated'}, 503)
        else:
            bad = guard(request)
            if bad:
                return bad
        nav.update(state='canceled', distance=None)
        log('POST /api/stop')
        return {'ok': True}

    @app.post('/api/initialpose')
    async def initialpose(request: Request):
        bad = guard(request)
        if bad:
            return bad
        if not _pose_ok(await read_body(request)):
            return JSONResponse({'error': 'invalid_body', 'detail': 'Bad body'}, 400)
        if fail in INIT_FAILS:
            code, err, detail = INIT_FAILS[fail]
            return JSONResponse({'error': err, 'detail': detail}, code)
        if mode != 'map':
            return JSONResponse({'error': 'wrong_mode', 'detail': 'Only in mode map'}, 409)
        return {'ok': True}

    @app.get('/api/maps')
    async def list_maps(request: Request):
        bad = guard(request)
        if bad:
            return bad
        return {'maps': sorted(saved)}

    @app.post('/api/map/save')
    async def save_map(request: Request):
        bad = guard(request)
        if bad:
            return bad
        body = await read_body(request)
        overwrite = body.get('overwrite') if isinstance(body, dict) else None
        if (not isinstance(body, dict) or not set(body) <= {'name', 'overwrite'}
                or not isinstance(body.get('name'), str)
                or (overwrite is not None and not isinstance(overwrite, bool))):
            return JSONResponse({'error': 'invalid_body', 'detail': 'Bad body'}, 400)
        name = body['name']
        if not NAME_RE.match(name):
            return JSONResponse({'error': 'invalid_name', 'detail': 'Bad name'}, 400)
        if name.lower() == 'map':
            return JSONResponse({'error': 'reserved_name', 'detail': 'Reserved'}, 400)
        log('POST /api/map/save name=%r overwrite=%r' % (name, overwrite))
        if mode != 'mapping':
            return JSONResponse({'error': 'wrong_mode', 'detail': 'Only in mode mapping'}, 409)
        if saving['busy'] or fail == 'save409':
            return JSONResponse({'error': 'save_in_progress', 'detail': 'Busy'}, 409)
        if fail == 'save503':
            return JSONResponse({'error': 'slam_unavailable', 'detail': 'No SLAM'}, 503)
        if fail in SAVE_ERRORS:
            code, err, detail = SAVE_ERRORS[fail]
            return JSONResponse({'error': err, 'detail': detail}, code)
        if name in saved and not overwrite:
            return JSONResponse({'error': 'map_exists', 'detail': 'Exists'}, 409)
        saving['busy'] = True
        try:
            if fail == 'savetimeout':
                await asyncio.sleep(2.0)
                return JSONResponse({'error': 'save_timeout', 'detail': 'Timeout'}, 504)
            if fail == 'saveslow':
                await asyncio.sleep(4.0)
            if name not in saved:
                saved.append(name)
        finally:
            saving['busy'] = False
        return {'ok': True, 'name': name, 'posegraph': posegraph}

    @app.websocket('/ws')
    async def ws(websocket: WebSocket):
        given = websocket.query_params.get('token', '')
        ip = _ip(websocket)
        # The real server rejects BEFORE accepting (HTTP 403 on the handshake); it never does
        # accept + close 1008. The client still handles a 1008 close (harmless), but it is no
        # longer the real path.
        origin = websocket.headers.get('origin')
        host = websocket.headers.get('host', '')
        if origin is not None and urlsplit(origin).netloc != host:
            log('WS rejected: origin mismatch', origin)
            await websocket.close(code=1008)
            return
        if limiter.is_locked(ip):
            await websocket.close(code=1008)
            return
        if not secrets.compare_digest(given, token):
            limiter.record_failure(ip)
            await websocket.close(code=1008)
            return
        await websocket.accept()
        await websocket.send_json(meta)
        await websocket.send_bytes(payload)
        await websocket.send_json({'type': 'plan', 'points': [
            [-3 + 0.1 * k, -2 + 0.04 * k] for k in range(60)]})
        t0 = time.time()
        tick = 0
        conn = {'seq': 0, 't': 0.0}   # per-connection teleop state (seq restarts here)
        outbox = []

        async def receive():
            while True:
                msg = await websocket.receive()
                if msg['type'] == 'websocket.disconnect':
                    return
                if msg.get('text') is not None:
                    handle_client(msg['text'], conn, nav, fail, outbox)

        receiver = asyncio.create_task(receive())
        try:
            while True:
                if receiver.done():
                    return
                while outbox:
                    await websocket.send_json(outbox.pop(0))
                t = time.time() - t0
                x, y = 2.0 * math.cos(0.3 * t), 1.5 * math.sin(0.3 * t)
                yaw = math.atan2(1.5 * math.cos(0.3 * t), -2.0 * math.sin(0.3 * t))
                await websocket.send_json({'type': 'pose', 'x': x, 'y': y, 'yaw': yaw,
                                           'stamp': time.time()})
                if tick % 2 == 0:
                    pts = []
                    for k in range(180):
                        a = yaw + (k / 180.0) * 2 * math.pi
                        r = 2.5 + 0.3 * math.sin(3 * a)
                        pts += [round(x + r * math.cos(a), 3), round(y + r * math.sin(a), 3)]
                    await websocket.send_json({'type': 'scan', 'stamp': time.time(),
                                               'points': pts})
                if tick % 10 == 0:
                    status = {
                        'type': 'status', 'battery_pct': 80.0, 'mode': mode,
                        'nodes': {'slam_toolbox': 'active' if mode == 'mapping' else 'inactive',
                                  'amcl': 'active' if mode == 'map' else 'unavailable',
                                  'bt_navigator': 'active', 'controller_server': 'unconfigured'},
                        'scan_age': 3.5 if stale else 0.1, 'odom_age': 2.6 if stale else 0.1}
                    if with_dock:
                        status['dock'] = DOCK
                    await websocket.send_json(status)
                if tick % 5 == 0:
                    if nav['state'] == 'active':
                        nav['distance'] = round(max(0.0, nav['distance'] - 0.25), 2)
                        if nav['distance'] == 0.0:
                            nav.update(state='succeeded', distance=None)
                    await websocket.send_json({'type': 'nav', 'state': nav['state'],
                                               'distance_remaining': nav['distance']})
                tick += 1
                await asyncio.sleep(0.1)
        except Exception:  # client went away
            return
        finally:
            receiver.cancel()
            log('WS closed')

    return app


def main():
    """Run the fake server."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--token', default='dev-token')
    parser.add_argument('--mode', choices=('map', 'mapping'), default='mapping')
    parser.add_argument(
        '--fail', choices=('none', 'teleop') + tuple(FAILURES) + SAVE_FAILS + tuple(INIT_FAILS)
        + ('stop503',),
        default='none')
    parser.add_argument('--no-dock', action='store_true')
    parser.add_argument('--maps', default='lab,casa', help='comma separated saved map names')
    parser.add_argument('--no-posegraph', action='store_true')
    parser.add_argument('--stale', action='store_true')
    args = parser.parse_args()
    print('Fake MTRB server on http://127.0.0.1:%d  token=%s' % (args.port, args.token))
    maps = [m for m in args.maps.split(',') if m]
    app = create_app(args.token, args.mode, args.fail, not args.no_dock, maps,
                     not args.no_posegraph, args.stale)
    uvicorn.run(app, host='127.0.0.1', port=args.port, log_level='warning')


if __name__ == '__main__':
    main()
