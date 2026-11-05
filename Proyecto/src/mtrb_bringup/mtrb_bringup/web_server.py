"""
FastAPI application of the web bridge (no ROS imports, testable on macOS).

The ROS node writes into a `StateStore`; HTTP handlers only read snapshots of it.
"""

import asyncio
import copy
import hmac
import itertools
import json
import math
import os
import shutil
import threading
import time

from fastapi import Depends, FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from mtrb_bringup.web_utils import (
    existing_map_files, list_maps, MAP_ALL_EXTS, MAP_BACKUP_EXT, map_image_saved, MAP_LIST_LIMIT,
    MapPathError, POSEGRAPH_EXTS, posegraph_saved, ReservedMapNameError, resolve_map_base,
    valid_map_name, validate_goal)
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
MAPPING_MODE = 'mapping'         # the only mode in which /api/map/save is accepted
POSE_FIELDS = frozenset(('x', 'y', 'yaw'))
MAP_SAVE_FIELDS = frozenset(('name', 'overwrite'))
# Map saving. slam_toolbox runs `map_saver_cli` inside the save_map service (a few seconds
# on a large map) and the serialization is in-process: both get a bound well above that.
MAP_SAVE_TIMEOUT_S = 15.0
MAP_SERIALIZE_TIMEOUT_S = 15.0
# After a service timeout slam_toolbox may still be writing: no new save for this long.
MAP_SAVE_COOLDOWN_S = 30.0
# Map storage quotas (a flooded disk would stop the robot's logging and the OS itself).
MAP_MAX_SAVED = 200                 # saved maps (valid name, yaml + image) in the map dir
MAP_MIN_FREE_MB_DEFAULT = 200       # free space required before a save (MB)
MAP_MIN_INTERVAL_S_DEFAULT = 5.0    # minimum time between two accepted saves (s)
BYTES_PER_MB = 1024 * 1024
# Security headers on every HTTP response. The UI has no inline <script>/<style>, no style=""
# attributes and no event handlers (web/index.html, web/app.js): the only style change done by
# JS is the CSSOM property `element.style.transform`, which CSP does not block. So both
# script-src and style-src can be 'self' only, without 'unsafe-inline'. `ws:`/`wss:` are
# needed for the /ws WebSocket (connect-src 'self' does not cover it on every browser).
SECURITY_HEADERS = {
    'X-Frame-Options': 'DENY',
    'X-Content-Type-Options': 'nosniff',
    'Referrer-Policy': 'no-referrer',
    'Content-Security-Policy': (
        "default-src 'self'; script-src 'self'; style-src 'self'; "
        "connect-src 'self' ws: wss:; img-src 'self' data: blob:; "
        "frame-ancestors 'none'; base-uri 'none'; form-action 'self'"),
}
WS_TELEOP = 'teleop'
WS_STOP = 'stop'

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

    def teleop(self, client_id, message):
        """
        Handle a validated-by-the-node teleop `message` (dict) of WebSocket client `client_id`.

        Return None when it was accepted (or silently dropped) or an (code, detail) error to
        send back to that client. Never raises.
        """
        raise NotImplementedError

    def teleop_disconnect(self, client_id):
        """Stop the teleop of `client_id` at once because its WebSocket closed; never raises."""
        raise NotImplementedError

    def stop(self):
        """
        Emergency-style stop of the web: zero Twist FIRST, then cancel any navigation.

        Must not depend on Nav2, the map or the mode; never raises.
        """
        raise NotImplementedError


class SlamUnavailableError(Exception):
    """slam_toolbox (or ROS) cannot take the map request right now: HTTP 503."""


class SlamTimeoutError(Exception):
    """slam_toolbox did not answer a map request in time: HTTP 504."""


class MapPort:
    """
    Interface between the map saver and slam_toolbox (implemented by the node).

    Both save methods BLOCK the calling thread up to `timeout_s` (they run in the HTTP
    thread pool, never on the executor or the event loop) and must never be called from a
    ROS callback. Tests inject a double with the same methods.
    """

    def ready(self):
        """Return True if the slam_toolbox save_map service is available (non-blocking)."""
        raise NotImplementedError

    def save_map(self, base_path, timeout_s):
        """
        Ask slam_toolbox to write base_path + .yaml/.pgm; return True on success.

        Raise SlamUnavailableError if the service is not ready and SlamTimeoutError if it
        does not answer within `timeout_s`. `base_path` is an absolute, validated path.
        """
        raise NotImplementedError

    def serialize_posegraph(self, base_path, timeout_s):
        """
        Ask slam_toolbox to write base_path + .posegraph/.data; return True on success.

        Same errors as save_map.
        """
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
    return token.strip().upper().startswith(PLACEHOLDER_TOKEN_PREFIX)


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


def _parse_ws_text(message):
    """Return the JSON object of a text WebSocket message, or None if it is not one."""
    text = message.get('text')
    if text is None:
        return None
    try:
        data = json.loads(text, parse_constant=_reject_constant)
    except (ValueError, RecursionError):
        return None
    return data if isinstance(data, dict) else None


def _handle_client_message(data, nav_port, client_id):
    """
    Dispatch one parsed client message; return an (code, detail) error reply or None.

    Only `teleop` and `stop` are supported. Teleop content is never logged.
    """
    kind = None if data is None else data.get('type')
    if kind not in (WS_TELEOP, WS_STOP):
        return ('unsupported_message', 'Unsupported client message')
    if nav_port is None:
        if kind == WS_STOP:
            return ('stop_unavailable', 'Navigation is not available: use the physical stop')
        return ('teleop_rejected', 'Teleoperation is not available')
    if kind == WS_STOP:
        nav_port.stop()  # never lease- or rate-limited
        return None
    return nav_port.teleop(client_id, data)


async def _receive_loop(websocket, channel, max_ws_bytes, nav_port=None, client_id=None):
    """Read client messages: `teleop` and `stop` are handled, the rest gets an error."""
    while True:
        message = await websocket.receive()
        if message['type'] == 'websocket.disconnect':
            return
        if _message_size(message) > max_ws_bytes:
            await websocket.close(code=WS_CLOSE_TOO_BIG)
            return
        reply = _handle_client_message(_parse_ws_text(message), nav_port, client_id)
        if reply is not None:
            channel.put('error', json_message('error', code=reply[0], detail=reply[1]))


def _map_save_from_body(body):
    """Strictly validate {name, overwrite?}: exact keys, valid name, real bool (no extras)."""
    if (not isinstance(body, dict) or 'name' not in body
            or not set(body) <= MAP_SAVE_FIELDS):
        raise ApiError(400, 'invalid_body',
                       'Body must be a JSON object with name and optional overwrite')
    try:
        name = valid_map_name(body['name'])
    except ReservedMapNameError as err:
        raise ApiError(400, 'reserved_name', 'That map name is not allowed') from err
    except ValueError as err:
        raise ApiError(400, 'invalid_name', str(err)) from err
    overwrite = body.get('overwrite', False)
    if not isinstance(overwrite, bool):
        raise ApiError(400, 'invalid_body', 'Field overwrite must be a boolean')
    return name, overwrite


class MapSaver:
    """
    Save the map and the pose-graph of slam_toolbox into one directory, safely.

    Pure orchestration (no ROS): slam_toolbox is reached through a `MapPort`. Rules:

    - the target is `<real map_dir>/<name>` with `name` validated by the caller; the
      directory is a configured value, never client data; existing symlinks are refused;
    - one save at a time (own lock, unrelated to the teleop and navigation locks) and none
      during the cooldown that follows a slam_toolbox timeout; at least `min_interval_s`
      between two saves (429 save_rate_limited);
    - quotas checked before slam_toolbox is called: at most MAP_MAX_SAVED maps (409
      map_limit, only for a NEW map) and `min_free_mb` of free disk (507
      insufficient_storage);
    - overwriting never touches an existing `*.bak` (409 backup_pending);
    - without `overwrite` an existing map (any of its files) answers 409 map_exists;
    - with `overwrite` the previous files are first renamed to `*.bak`; if the save fails they
      are restored, if it succeeds they are deleted. After a TIMEOUT nothing is touched
      (slam_toolbox may still be writing) and the `*.bak` files stay for manual recovery;
    - the map (yaml + image) is mandatory; the pose-graph is best effort: if only it fails
      the answer is still success with `posegraph: false` (the map works with mode:=map but
      cannot be extended with `posegraph:=`).
    """

    def __init__(self, map_dir, port, clock=time.monotonic, on_event=None,
                 min_free_mb=MAP_MIN_FREE_MB_DEFAULT,
                 min_interval_s=MAP_MIN_INTERVAL_S_DEFAULT):
        """
        Store the configured map directory (str), the MapPort and a monotonic clock.

        `min_free_mb` is the free disk space required to save and `min_interval_s` the
        minimum time between two saves.

        `on_event(level, text)` (level 'info' | 'warning' | 'error') receives the safety
        relevant events; the text holds only validated names and fixed phrases. This module
        never writes to a log itself.
        """
        self._map_dir = map_dir
        self._port = port
        self._clock = clock
        self._on_event = on_event
        self._lock = threading.Lock()
        self._blocked_until_s = float('-inf')
        self._last_save_s = float('-inf')
        self._min_free_bytes = min_free_mb * BYTES_PER_MB
        self._min_interval_s = min_interval_s

    def _event(self, level, text):
        """Report an event to the owner (no-op without a listener)."""
        if self._on_event is not None:
            self._on_event(level, text)

    def names(self):
        """Return the sorted saved map names, at most MAP_LIST_LIMIT of them."""
        try:
            return list_maps(self._map_dir, MAP_LIST_LIMIT)
        except (MapPathError, OSError) as err:
            self._event('error', f'Cannot list maps: {err}')
            raise ApiError(500, 'map_dir_invalid', 'The map directory is not usable') from err

    def save(self, name, overwrite):
        """
        Save map and pose-graph as `name` and return the response body.

        Raise ApiError: 409 save_in_progress / map_exists / backup_pending / map_limit,
        429 save_rate_limited, 507 insufficient_storage, 503 slam_unavailable,
        504 save_timeout, 500 map_dir_invalid / save_failed.
        """
        if not self._lock.acquire(blocking=False):
            raise ApiError(409, 'save_in_progress', 'Another map save is in progress')
        try:
            if self._clock() < self._blocked_until_s:
                raise ApiError(409, 'save_in_progress',
                               'A previous save timed out: wait before saving again')
            if self._clock() - self._last_save_s < self._min_interval_s:
                raise ApiError(429, 'save_rate_limited', 'Saving too often: wait a few seconds')
            return self._save_locked(name, overwrite)
        finally:
            self._lock.release()

    def _block_after_timeout(self):
        """Refuse new saves for MAP_SAVE_COOLDOWN_S (slam_toolbox may still be writing)."""
        self._blocked_until_s = self._clock() + MAP_SAVE_COOLDOWN_S

    def _prepare(self, name):
        """Create the map directory if needed and return the validated base path."""
        try:
            if not isinstance(self._map_dir, str) or not os.path.isabs(self._map_dir):
                raise MapPathError('The map directory is not configured')
            os.makedirs(self._map_dir, mode=0o755, exist_ok=True)
            return resolve_map_base(self._map_dir, name)
        except (MapPathError, OSError) as err:
            self._event('error', f'Map directory not usable: {err}')
            raise ApiError(500, 'map_dir_invalid', 'The map directory is not usable') from err

    def _save_locked(self, name, overwrite):
        """Body of save() with the lock held."""
        try:
            ready = self._port.ready()
        except Exception as err:  # noqa: B902 - a broken port must give a controlled answer
            self._event('error', f'slam_toolbox readiness check failed: {type(err).__name__}')
            raise ApiError(503, 'slam_unavailable',
                           'slam_toolbox save_map is not available') from err
        if not ready:
            raise ApiError(503, 'slam_unavailable', 'slam_toolbox save_map is not available')
        base = self._prepare(name)
        existing = existing_map_files(base)
        if existing and not overwrite:
            raise ApiError(409, 'map_exists', f'Map {name} already exists')
        if existing and any(os.path.lexists(path + MAP_BACKUP_EXT) for path in
                            [base + ext for ext in MAP_ALL_EXTS]):
            self._event('error', f'Map {name}: an unresolved backup blocks the overwrite')
            raise ApiError(409, 'backup_pending',
                           'A previous backup is unresolved: review or delete <name>.*.bak')
        self._check_quotas(name, bool(existing))
        self._last_save_s = self._clock()
        backups = self._backup(existing)
        try:
            saved = self._port.save_map(base, MAP_SAVE_TIMEOUT_S)
        except SlamTimeoutError as err:
            self._block_after_timeout()
            self._event('error', f'Map save {name} timed out; files left as they are')
            raise ApiError(504, 'save_timeout', 'slam_toolbox did not answer in time') from err
        except SlamUnavailableError as err:
            self._restore(base, backups)
            raise ApiError(503, 'slam_unavailable',
                           'slam_toolbox save_map is not available') from err
        except Exception as err:  # noqa: B902 - the port must never leave a half state
            self._restore(base, backups)
            self._event('error', f'Map save {name} failed: {err}')
            raise ApiError(500, 'save_failed', 'Saving the map failed') from err
        if not saved or not map_image_saved(base):
            self._restore(base, backups)
            self._event('error', f'Map save {name} failed: slam_toolbox reported an error')
            raise ApiError(500, 'save_failed', 'slam_toolbox could not save the map')
        posegraph = self._serialize(base, name)
        for _original, backup in backups:
            self._remove(backup)
        self._event('info', f'Map {name} saved (posegraph: {posegraph}, overwrite: {overwrite})')
        return {'ok': True, 'name': name, 'posegraph': posegraph}

    def _check_quotas(self, name, exists):
        """Refuse the save if the map count or the free disk space would be exceeded."""
        try:
            if not exists and len(list_maps(self._map_dir, MAP_MAX_SAVED)) >= MAP_MAX_SAVED:
                self._event('warning', f'Map {name} refused: map limit reached')
                raise ApiError(409, 'map_limit', 'Too many saved maps: delete some first')
            free_bytes = shutil.disk_usage(self._map_dir).free
        except (MapPathError, OSError) as err:
            self._event('error', f'Cannot check the map storage: {err}')
            raise ApiError(500, 'map_dir_invalid', 'The map directory is not usable') from err
        if free_bytes < self._min_free_bytes:
            self._event('error', f'Map {name} refused: insufficient free disk space')
            raise ApiError(507, 'insufficient_storage', 'Not enough free disk space')

    def _serialize(self, base, name):
        """Best effort pose-graph serialization; return True only if both files exist."""
        try:
            done = self._port.serialize_posegraph(base, MAP_SERIALIZE_TIMEOUT_S)
        except SlamTimeoutError:
            self._block_after_timeout()
            self._event('warning', f'Pose-graph of {name} timed out')
            return False
        except SlamUnavailableError:
            self._event('warning', f'Pose-graph of {name} not saved: service not available')
            self._discard_partial_posegraph(base)
            return False
        except Exception as err:  # noqa: B902
            self._event('warning', f'Pose-graph of {name} failed: {err}')
            self._discard_partial_posegraph(base)
            return False
        if not done or not posegraph_saved(base):
            self._event('warning', f'Pose-graph of {name} was not written')
            self._discard_partial_posegraph(base)
            return False
        return True

    def _discard_partial_posegraph(self, base):
        """
        Delete partial `.posegraph`/`.data` files left by a failed (not timed out) serialization.

        At this point any previous files were renamed to `*.bak`, so whatever sits at `base` is
        new. Only regular files (no symlinks) inside the map directory are removed; the `*.bak`
        files are never touched. Not called after a timeout: slam_toolbox may still be writing.
        """
        map_dir = os.path.realpath(self._map_dir)
        for ext in POSEGRAPH_EXTS:
            path = base + ext
            if (os.path.islink(path) or not os.path.isfile(path)
                    or os.path.dirname(os.path.realpath(path)) != map_dir):
                continue
            self._remove(path)

    def _backup(self, existing):
        """Rename existing files to `*.bak`; return [(original, backup)]. Undo on error."""
        backups = []
        try:
            for path in existing:
                os.replace(path, path + MAP_BACKUP_EXT)
                backups.append((path, path + MAP_BACKUP_EXT))
        except OSError as err:
            for original, backup in backups:
                self._rename(backup, original)
            self._event('error', f'Cannot back up the previous map: {err}')
            raise ApiError(500, 'map_dir_invalid', 'The map directory is not usable') from err
        return backups

    def _restore(self, base, backups):
        """Delete files written by a failed save and put the backups back."""
        for original in existing_map_files(base):
            self._remove(original)
        for original, backup in backups:
            self._rename(backup, original)

    def _remove(self, path):
        """Delete a file, logging (never raising) on failure."""
        try:
            os.remove(path)
        except FileNotFoundError:
            pass
        except OSError as err:
            self._event('error', f'Cannot remove {os.path.basename(path)}: {err}')

    def _rename(self, source, target):
        """Atomic rename, logging (never raising) on failure."""
        try:
            os.replace(source, target)
        except OSError as err:
            self._event('error', f'Cannot restore {os.path.basename(target)}: {err}')


def _pose_from_body(body):
    """Strictly validate {x, y, yaw}: exact keys, real finite numbers (no bool), bounded."""
    if not isinstance(body, dict) or set(body) != POSE_FIELDS:
        raise ApiError(400, 'invalid_body', 'Body must be a JSON object with x, y and yaw')
    values = []
    for name in ('x', 'y', 'yaw'):
        value = body[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            valid = False
        else:
            try:  # ints too big for a float overflow: invalid, never a 500
                valid = math.isfinite(value) and abs(value) <= MAX_ABS_COORD_M
            except (OverflowError, ValueError):
                valid = False
        if not valid:
            raise ApiError(400, 'invalid_body', f'Field {name} must be a finite number')
        values.append(float(value))
    return tuple(values)


async def _read_json(request):
    """Read a JSON body of at most MAX_BODY_BYTES; None if the body is empty."""
    declared = request.headers.get('content-length', '')
    try:
        too_large = declared.isascii() and declared.isdigit() and int(declared) > MAX_BODY_BYTES
    except ValueError:  # absurdly long digit string: refuse it
        too_large = True
    if too_large:
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
               nav_port=None, dock=None, map_port=None, map_dir=None, on_event=None,
               map_min_free_mb=MAP_MIN_FREE_MB_DEFAULT,
               map_min_interval_s=MAP_MIN_INTERVAL_S_DEFAULT):
    """
    Build the FastAPI app. An empty token is rejected (fail closed).

    `nav_port` (a NavPort) executes the navigation commands; without it they answer 503.
    `map_port` (a MapPort) and `map_dir` (absolute directory) enable /api/map/save and
    /api/maps; without them they answer 503. `on_event(level, text)` receives the map-saving
    events; `map_min_free_mb` / `map_min_interval_s` are the save quotas.
    `dock` is the (x_m, y_m, yaw_rad) of the dock; without it /api/dock answers 503.
    """
    if not token:
        raise ValueError('An authentication token is required')
    expected = token.encode('utf-8')
    app = FastAPI(title='MTRB web bridge', docs_url=None, redoc_url=None, openapi_url=None)

    @app.middleware('http')
    async def _security_headers(request, call_next):
        response = await call_next(request)
        response.headers.update(SECURITY_HEADERS)
        return response

    @app.exception_handler(Exception)
    async def _unexpected_error(_request, _err):
        # Neutral body: never a trace or a path. Outside the middleware, so add the headers.
        return JSONResponse(status_code=500, headers=SECURITY_HEADERS,
                            content={'error': 'internal_error', 'detail': 'Internal error'})

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
    client_ids = itertools.count(1)  # one unique id per WebSocket connection

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
        client_id = next(client_ids)
        receiver = asyncio.ensure_future(
            _receive_loop(websocket, channel, max_ws_bytes, nav_port, client_id))
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
            if nav_port is not None:
                nav_port.teleop_disconnect(client_id)  # zero Twist now, not at the watchdog
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

    @app.post('/api/stop', dependencies=[Depends(require_token)])
    async def post_stop():
        # Zero Twist first, then cancel; independent of Nav2, map and mode. No body.
        # `async def` on purpose: the handler body runs on the event loop. The sync
        # `require_token` dependency still runs in the thread pool (pure CPU, no I/O), so a
        # saturated pool could delay the request; NavPort.stop() itself must never block.
        require_nav().stop()
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

    map_saver = None
    if map_port is not None and map_dir is not None:
        map_saver = MapSaver(map_dir, map_port, on_event=on_event,
                             min_free_mb=map_min_free_mb, min_interval_s=map_min_interval_s)

    def require_map_saver():
        if map_saver is None:
            raise ApiError(503, 'slam_unavailable', 'Map saving is not available')
        return map_saver

    @app.post('/api/map/save', dependencies=[Depends(require_token)])
    def post_map_save(body=Depends(body_json)):
        # Sync handler: runs in the thread pool and may wait for slam_toolbox (bounded).
        name, overwrite = _map_save_from_body(body)
        mode = store.snapshot()['mode']
        if mode != MAPPING_MODE:
            raise ApiError(409, 'wrong_mode',
                           f'Maps are only saved in mode mapping (mode: {mode})')
        return require_map_saver().save(name, overwrite)

    @app.get('/api/maps', dependencies=[Depends(require_token)])
    def get_maps():
        return {'maps': require_map_saver().names()}

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
