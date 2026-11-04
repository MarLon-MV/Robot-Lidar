"""
FastAPI application of the web bridge (no ROS imports, testable on macOS).

The ROS node writes into a `StateStore`; HTTP handlers only read snapshots of it.
"""

import asyncio
import copy
import hmac
import json
import math
import os
import threading
import time

from fastapi import Depends, FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from mtrb_bringup.web_utils import validate_goal
from starlette.websockets import WebSocketState
import yaml

NODE_SECTION = 'mtrb_web_bridge'
TOKEN_ENV_VAR = 'MTRB_WEB_TOKEN'
LOCAL_PARAM_FILE = 'web.local.yaml'
DEFAULT_PARAM_FILE = 'web.yaml'
PLACEHOLDER_TOKEN_PREFIX = 'CHANGE-ME'
STATIC_FILES = {'/': 'index.html', '/app.js': 'app.js', '/style.css': 'style.css'}
STATIC_MEDIA_TYPES = {'/': 'text/html', '/app.js': 'text/javascript', '/style.css': 'text/css'}

WS_PATH = '/ws'
WS_CLOSE_UNAUTHORIZED = 1008  # policy violation (only reachable after accept)
WS_CLOSE_TOO_BIG = 1009
WS_CLOSE_TRY_AGAIN_LATER = 1013  # too many clients
WS_CLOSE_INTERNAL_ERROR = 1011
DEFAULT_MAX_CLIENTS = 5
DEFAULT_MAX_WS_BYTES = 4096
WS_SEND_TIMEOUT_S = 5.0  # a client that cannot take one message in this time is dropped
# Latest-value slots per client, in the order they are sent. Each slot holds ONE pending
# item: a newer value replaces an unsent older one (queue of size 1, stale data dropped).
SLOT_ORDER = ('map', 'error', 'plan', 'nav', 'status', 'pose', 'scan')

MAX_BODY_BYTES = 1024            # REST bodies are tiny ({x, y, yaw}); anything bigger is refused
MAX_ABS_COORD_M = 10000.0        # sanity bound for a pose given by the client (metres)
MAP_MODE = 'map'                 # the only mode in which /api/initialpose is accepted
POSE_FIELDS = frozenset(('x', 'y', 'yaw'))

_SNAPSHOT_KEYS = ('mode', 'pose', 'scan_age', 'map', 'plan_len', 'nav', 'battery_pct',
                  'dock')


class StateStore:
    """Thread-safe latest-value store shared by the ROS callbacks and the HTTP handlers."""

    def __init__(self, clock=time.time):
        """Create an empty store; every value starts as None (unknown)."""
        self._lock = threading.Lock()
        self._clock = clock
        self._state = {key: None for key in _SNAPSHOT_KEYS}
        self._state['stamp'] = None
        self._grid = None

    def set_grid(self, grid):
        """
        Keep the latest map WITH its cells for goal validation (not part of the snapshot).

        `grid` is a dict with width, height, resolution, origin_x, origin_y and data. It is
        stored by reference and must not be mutated afterwards (the node never does).
        """
        with self._lock:
            self._grid = grid

    def grid(self):
        """Return the latest map dict given to `set_grid`, or None if there is none."""
        with self._lock:
            return self._grid

    def update(self, **values):
        """Set known keys atomically and refresh the stamp (unix seconds)."""
        unknown = set(values) - set(_SNAPSHOT_KEYS)
        if unknown:
            raise KeyError(f'Unknown state keys: {sorted(unknown)}')
        with self._lock:
            self._state.update(copy.deepcopy(values))
            self._state['stamp'] = self._clock()

    def snapshot(self):
        """Return a deep copy of the state, safe to serialize outside the lock."""
        with self._lock:
            return copy.deepcopy(self._state)


class NavUnavailableError(Exception):
    """Nav2 (or ROS) cannot take the command right now: HTTP 503."""


class NavBusyError(Exception):
    """There already is an active goal: HTTP 409."""


class NavPort:
    """
    Interface between the HTTP handlers and ROS (implemented by the node).

    Tests inject a double with the same methods. All methods must return quickly (never
    block on ROS) and may be called from any thread.
    """

    def send_goal(self, x_m, y_m, yaw_rad):
        """
        Send a NavigateToPose goal in frame `map` and return its goal_id (str).

        Raise NavBusyError if a goal is active and NavUnavailableError if Nav2 is not ready.
        """
        raise NotImplementedError

    def cancel(self):
        """Cancel the active goal, if any. Idempotent; never raises for 'nothing to cancel'."""
        raise NotImplementedError

    def publish_initial_pose(self, x_m, y_m, yaw_rad):
        """Publish the pose estimate (frame `map`) on `initialpose`."""
        raise NotImplementedError


def json_message(message_type, **fields):
    """Serialize a WebSocket message as compact JSON; NaN/inf raise ValueError."""
    return json.dumps({'type': message_type, **fields}, separators=(',', ':'),
                      allow_nan=False)


class ClientChannel:
    """
    Pending messages of one WebSocket client: one slot per message kind.

    `put` may be called from any thread; `take_all` returns what is pending, oldest kinds
    first, and empties the slots. Memory per client is bounded by the number of kinds.
    """

    def __init__(self, wakeup):
        """Store the thread-safe callable that tells the client's sender there is data."""
        self._lock = threading.Lock()
        self._slots = {}
        self._wakeup = wakeup
        self.closed = False

    def put(self, kind, payload):
        """Replace the pending item of `kind` (str, or (str, bytes) for the map)."""
        if kind not in SLOT_ORDER:
            raise KeyError(f'Unknown message kind: {kind}')
        with self._lock:
            if self.closed:
                return
            self._slots[kind] = payload
        try:
            self._wakeup()
        except RuntimeError:  # the client's event loop is gone: the client is dead
            with self._lock:  # wakeup already returned: the lock is never held during it
                self.closed = True

    def discard(self, kind):
        """Drop the pending item of `kind`, if any."""
        with self._lock:
            self._slots.pop(kind, None)

    def take_all(self):
        """Return the pending items as [str | bytes, ...] and clear the slots."""
        with self._lock:
            slots, self._slots = self._slots, {}
        items = []
        for kind in SLOT_ORDER:
            payload = slots.get(kind)
            if payload is None:
                continue
            if isinstance(payload, tuple):
                items.extend(payload)
            else:
                items.append(payload)
        return items

    def close(self):
        """Mark the channel as dead and release pending data."""
        with self._lock:
            self.closed = True
            self._slots = {}


class BroadcastHub:
    """
    Fan-out of the latest values to all connected clients (thread-safe, no ROS).

    The ROS thread calls `publish`; each client drains its own `ClientChannel`. A slow client
    only ever loses its own old data; it never blocks the publisher or other clients.
    """

    def __init__(self, max_clients=DEFAULT_MAX_CLIENTS):
        """Create an empty hub that accepts at most `max_clients` clients."""
        if not isinstance(max_clients, int) or isinstance(max_clients, bool) or max_clients < 1:
            raise ValueError('max_clients must be a positive integer')
        self._lock = threading.Lock()
        self._max_clients = max_clients
        self._channels = []
        self._latest = {}

    @property
    def client_count(self):
        """Number of connected clients."""
        with self._lock:
            return len(self._channels)

    def add_client(self, wakeup):
        """Register a client seeded with the latest values; None if the hub is full."""
        channel = ClientChannel(wakeup)
        with self._lock:
            if len(self._channels) >= self._max_clients:
                return None
            self._channels.append(channel)
            seed = dict(self._latest)
        for kind, payload in seed.items():
            channel.put(kind, payload)
        return channel

    def remove_client(self, channel):
        """Unregister a client (idempotent)."""
        channel.close()
        with self._lock:
            if channel in self._channels:
                self._channels.remove(channel)

    def publish(self, kind, payload):
        """Remember `payload` as the latest of `kind` and queue it for every client."""
        if kind not in SLOT_ORDER:
            raise KeyError(f'Unknown message kind: {kind}')
        with self._lock:
            self._latest[kind] = payload
            channels = list(self._channels)
        for channel in channels:
            channel.put(kind, payload)

    def discard(self, kind):
        """Forget the latest value of `kind` (data became invalid, e.g. TF lost)."""
        with self._lock:
            self._latest.pop(kind, None)
            channels = list(self._channels)
        for channel in channels:
            channel.discard(kind)


class ApiError(Exception):
    """Error with an HTTP status, rendered as {"error": code, "detail": text}."""

    def __init__(self, status_code, code, detail, headers=None):
        """Store the status, machine-readable code, human text and extra headers."""
        super().__init__(detail)
        self.status_code = status_code
        self.code = code
        self.detail = detail
        self.headers = headers


def _token_from_yaml(path):
    """Return auth.token from a bridge YAML file, or None if absent or malformed."""
    try:
        with open(path, encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
        token = data[NODE_SECTION]['ros__parameters']['auth']['token']
    except (OSError, yaml.YAMLError, KeyError, TypeError):
        return None
    return token if isinstance(token, str) else None


def resolve_token(param_dir, environ=None):
    """
    Pick the token: env MTRB_WEB_TOKEN, else web.local.yaml, else web.yaml.

    Returns '' when nothing usable is found (the caller must refuse to start).
    """
    environ = os.environ if environ is None else environ
    token = environ.get(TOKEN_ENV_VAR, '')
    if token:
        return token
    for name in (LOCAL_PARAM_FILE, DEFAULT_PARAM_FILE):
        token = _token_from_yaml(os.path.join(param_dir, name))
        if token:
            return token
    return ''


def is_placeholder_token(token):
    """Return True if the token is the public example value from the repository."""
    return token.startswith(PLACEHOLDER_TOKEN_PREFIX)


def _extract_bearer(request):
    """Return the Bearer credential of the request, or '' if missing or malformed."""
    scheme, _, credential = request.headers.get('authorization', '').partition(' ')
    return credential.strip() if scheme.lower() == 'bearer' else ''


def _message_size(message):
    """Size in bytes of a received WebSocket message (text or binary)."""
    text = message.get('text')
    if text is not None:
        return len(text.encode('utf-8'))
    return len(message.get('bytes') or b'')


def _consume_result(task):
    """Retrieve a finished task's outcome so asyncio does not warn about it."""
    if not task.cancelled():
        task.exception()


async def _send_loop(websocket, channel, event):
    """Send pending items to one client; give up if it cannot keep up."""
    while True:
        await event.wait()
        event.clear()
        for item in channel.take_all():
            if isinstance(item, bytes):
                await asyncio.wait_for(websocket.send_bytes(item), WS_SEND_TIMEOUT_S)
            else:
                await asyncio.wait_for(websocket.send_text(item), WS_SEND_TIMEOUT_S)


async def _receive_loop(websocket, channel, max_ws_bytes):
    """Read client messages; phase 2 accepts none, so each one is answered with an error."""
    while True:
        message = await websocket.receive()
        if message['type'] == 'websocket.disconnect':
            return
        if _message_size(message) > max_ws_bytes:
            await websocket.close(code=WS_CLOSE_TOO_BIG)
            return
        channel.put('error', json_message(
            'error', code='unsupported_message',
            detail='No client messages are supported yet'))


def _pose_from_body(body):
    """Strictly validate {x, y, yaw}: exact keys, real finite numbers (no bool), bounded."""
    if not isinstance(body, dict) or set(body) != POSE_FIELDS:
        raise ApiError(400, 'invalid_body', 'Body must be a JSON object with x, y and yaw')
    values = []
    for name in ('x', 'y', 'yaw'):
        value = body[name]
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or abs(value) > MAX_ABS_COORD_M):
            raise ApiError(400, 'invalid_body', f'Field {name} must be a finite number')
        values.append(float(value))
    return tuple(values)


async def _read_json(request):
    """Read a JSON body of at most MAX_BODY_BYTES; None if the body is empty."""
    declared = request.headers.get('content-length', '')
    if declared.isdigit() and int(declared) > MAX_BODY_BYTES:
        raise ApiError(400, 'body_too_large', f'Body larger than {MAX_BODY_BYTES} bytes')
    chunks, size = [], 0
    async for chunk in request.stream():
        size += len(chunk)
        if size > MAX_BODY_BYTES:
            raise ApiError(400, 'body_too_large', f'Body larger than {MAX_BODY_BYTES} bytes')
        chunks.append(chunk)
    raw = b''.join(chunks)
    if not raw.strip():
        return None
    try:
        return json.loads(raw.decode('utf-8'), parse_constant=_reject_constant)
    except (UnicodeDecodeError, ValueError, RecursionError) as err:
        raise ApiError(400, 'invalid_json', 'Body is not valid JSON') from err


def _reject_constant(name):
    """json.loads hook: refuse NaN / Infinity / -Infinity literals."""
    raise ValueError(f'Non-finite constant {name}')


def create_app(store, token, static_dir=None, hub=None, max_ws_bytes=DEFAULT_MAX_WS_BYTES,
               nav_port=None, dock=None):
    """
    Build the FastAPI app. An empty token is rejected (fail closed).

    `nav_port` (a NavPort) executes the navigation commands; without it they answer 503.
    `dock` is the (x_m, y_m, yaw_rad) of the dock; without it /api/dock answers 503.
    """
    if not token:
        raise ValueError('An authentication token is required')
    expected = token.encode('utf-8')
    app = FastAPI(title='MTRB web bridge', docs_url=None, redoc_url=None, openapi_url=None)

    @app.exception_handler(ApiError)
    async def _api_error(_request, err):
        return JSONResponse(status_code=err.status_code, headers=err.headers,
                            content={'error': err.code, 'detail': err.detail})

    def require_token(request: Request):
        given = _extract_bearer(request).encode('utf-8')
        if not hmac.compare_digest(given, expected):
            raise ApiError(401, 'unauthorized', 'Missing or invalid token',
                           {'WWW-Authenticate': 'Bearer'})

    hub = BroadcastHub() if hub is None else hub

    @app.websocket(WS_PATH)
    async def ws_endpoint(websocket: WebSocket):
        # Authenticate before accepting: a bad token never takes a client slot.
        given = websocket.query_params.get('token', '').encode('utf-8')
        if not hmac.compare_digest(given, expected):
            await websocket.close(code=WS_CLOSE_UNAUTHORIZED)
            return
        await websocket.accept()
        loop = asyncio.get_running_loop()
        event = asyncio.Event()
        channel = hub.add_client(lambda: loop.call_soon_threadsafe(event.set))
        if channel is None:
            await websocket.close(code=WS_CLOSE_TRY_AGAIN_LATER)
            return
        sender = asyncio.ensure_future(_send_loop(websocket, channel, event))
        receiver = asyncio.ensure_future(_receive_loop(websocket, channel, max_ws_bytes))
        for task in (sender, receiver):
            task.add_done_callback(_consume_result)
        event.set()  # flush the values seeded at registration
        try:
            await asyncio.wait({sender, receiver}, return_when=asyncio.FIRST_COMPLETED)
        finally:
            # Synchronous cleanup first: it must run even if this handler is cancelled.
            hub.remove_client(channel)
            sender.cancel()
            receiver.cancel()
        if (websocket.client_state == WebSocketState.CONNECTED
                and websocket.application_state == WebSocketState.CONNECTED):
            try:
                await websocket.close()
            except (RuntimeError, WebSocketDisconnect):
                pass  # the peer vanished meanwhile

    @app.get('/api/state', dependencies=[Depends(require_token)])
    def get_state():
        state = store.snapshot()
        if state.get('dock') is None and dock is not None:
            # Same {x, y, yaw} shape as the WS status message; None when not configured.
            state['dock'] = dict(zip(('x', 'y', 'yaw'), dock))
        return state

    async def body_json(request: Request):
        return await _read_json(request)

    def require_nav():
        if nav_port is None:
            raise ApiError(503, 'nav_unavailable', 'Navigation is not available')
        return nav_port

    def run_nav(action, *args):
        """Call a NavPort method and translate its failures into API errors."""
        try:
            return action(*args)
        except NavBusyError as err:
            raise ApiError(409, 'goal_active', str(err) or 'A goal is already active') from err
        except NavUnavailableError as err:
            raise ApiError(503, 'nav_unavailable',
                           str(err) or 'Navigation is not available') from err

    def send_checked_goal(port, pose):
        """Validate `pose` against the latest map (503 if none) and send it."""
        grid = store.grid()
        if grid is None:
            raise ApiError(503, 'no_map', 'No map available yet')
        try:
            x, y, yaw = validate_goal(*pose, grid)
        except ValueError as err:
            raise ApiError(400, 'invalid_goal', str(err)) from err
        return {'ok': True, 'goal_id': run_nav(port.send_goal, x, y, yaw)}

    @app.post('/api/goal', dependencies=[Depends(require_token)])
    def post_goal(body=Depends(body_json)):
        pose = _pose_from_body(body)
        return send_checked_goal(require_nav(), pose)

    @app.post('/api/dock', dependencies=[Depends(require_token)])
    def post_dock():
        # The dock pose comes from configuration, not from the client: the body is ignored.
        port = require_nav()
        if dock is None:
            raise ApiError(503, 'dock_unavailable', 'Dock pose is not configured')
        if store.grid() is None:
            # No map to check against: the configured pose is sent as is (Nav2 will reject
            # it if it is not reachable).
            return {'ok': True, 'goal_id': run_nav(port.send_goal, *dock)}
        return send_checked_goal(port, dock)

    @app.post('/api/cancel', dependencies=[Depends(require_token)])
    def post_cancel():
        run_nav(require_nav().cancel)
        return {'ok': True}

    @app.post('/api/initialpose', dependencies=[Depends(require_token)])
    def post_initialpose(body=Depends(body_json)):
        pose = _pose_from_body(body)
        mode = store.snapshot()['mode']
        if mode != MAP_MODE:
            raise ApiError(409, 'wrong_mode',
                           f'Initial pose is only accepted in mode map (mode: {mode})')
        run_nav(require_nav().publish_initial_pose, *pose)
        return {'ok': True}

    def serve_static(path):
        def handler():
            if static_dir is not None:
                candidate = os.path.join(static_dir, STATIC_FILES[path])
                if os.path.isfile(candidate):
                    return FileResponse(candidate, media_type=STATIC_MEDIA_TYPES[path])
            raise ApiError(404, 'not_found', 'Web UI not installed')
        return handler

    for route in STATIC_FILES:
        app.add_api_route(route, serve_static(route), methods=['GET'],
                          include_in_schema=False)
    return app
