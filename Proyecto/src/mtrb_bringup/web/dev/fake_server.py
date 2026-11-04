"""
Fake MTRB web bridge for developing the UI without the robot.

Usage (from Proyecto/src/mtrb_bringup/web/dev):
    python fake_server.py [--port 8080] [--token dev-token] [--mode map|mapping]
                          [--fail none|409|503|401|400] [--no-dock]
--fail makes POST /api/goal, /api/dock and /api/cancel answer that error
(409 goal_active, 503 nav_unavailable, 401 unauthorized, 400 invalid_goal).
--mode map lets /api/initialpose succeed; in mapping it answers 409 wrong_mode.
The dock pose is published in status.dock.
Then open http://localhost:8080/ and enter the token. Not installed with the package.
"""
import argparse
import asyncio
import math
import os
import secrets
import struct
import time
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


FAILURES = {
    '409': (409, 'goal_active', 'A goal is already active'),
    '503': (503, 'nav_unavailable', 'Navigation is not available'),
    '401': (401, 'unauthorized', 'Missing or invalid token'),
    '400': (400, 'invalid_goal', 'Goal is not valid'),
}
DOCK = {'x': -3.5, 'y': -2.5, 'yaw': 0.0}


def _pose_ok(body):
    """Return True if body is exactly {x, y, yaw} with finite non-bool numbers."""
    return (isinstance(body, dict) and set(body) == {'x', 'y', 'yaw'}
            and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                    and math.isfinite(v) for v in body.values()))


def create_app(token, mode='mapping', fail='none', with_dock=True):
    """Build the fake app serving static files, /api/*, and /ws."""
    app = FastAPI()
    nav = {'state': 'idle', 'distance': None, 'log': []}
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

    @app.get('/api/state')
    async def state(request: Request):
        auth = request.headers.get('authorization', '')
        if not secrets.compare_digest(auth, 'Bearer ' + token):
            return JSONResponse({'error': 'unauthorized', 'detail': 'Invalid token'}, 401)
        return {'mode': mode, 'pose': None, 'map': meta}

    def guard(request):
        """Return an error response for a bad token or the forced failure, else None."""
        auth = request.headers.get('authorization', '')
        if not secrets.compare_digest(auth, 'Bearer ' + token):
            return JSONResponse({'error': 'unauthorized', 'detail': 'Invalid token'}, 401)
        return None

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

    @app.post('/api/initialpose')
    async def initialpose(request: Request):
        bad = guard(request)
        if bad:
            return bad
        if not _pose_ok(await read_body(request)):
            return JSONResponse({'error': 'invalid_body', 'detail': 'Bad body'}, 400)
        if mode != 'map':
            return JSONResponse({'error': 'wrong_mode', 'detail': 'Only in mode map'}, 409)
        return {'ok': True}

    @app.websocket('/ws')
    async def ws(websocket: WebSocket):
        given = websocket.query_params.get('token', '')
        if not secrets.compare_digest(given, token):
            await websocket.close(code=1008)
            return
        await websocket.accept()
        await websocket.send_json(meta)
        await websocket.send_bytes(payload)
        await websocket.send_json({'type': 'plan', 'points': [
            [-3 + 0.1 * k, -2 + 0.04 * k] for k in range(60)]})
        t0 = time.time()
        tick = 0
        try:
            while True:
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
                        'nodes': {'slam_toolbox': 'active'}, 'scan_age': 0.1, 'odom_age': 0.1}
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

    return app


def main():
    """Run the fake server."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--token', default='dev-token')
    parser.add_argument('--mode', choices=('map', 'mapping'), default='mapping')
    parser.add_argument('--fail', choices=('none',) + tuple(FAILURES), default='none')
    parser.add_argument('--no-dock', action='store_true')
    args = parser.parse_args()
    print('Fake MTRB server on http://127.0.0.1:%d  token=%s' % (args.port, args.token))
    app = create_app(args.token, args.mode, args.fail, not args.no_dock)
    uvicorn.run(app, host='127.0.0.1', port=args.port, log_level='warning')


if __name__ == '__main__':
    main()
