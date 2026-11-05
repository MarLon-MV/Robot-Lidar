"""
FastAPI application of the web bridge (no ROS imports, testable on macOS).

The ROS node writes into a `StateStore`; HTTP handlers only read snapshots of it.
"""

import asyncio
import collections
import copy
import hmac
import itertools
import json
import math
import os
import shutil
import stat
import threading
import time
from urllib.parse import urlsplit

from fastapi import Depends, FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from mtrb_bringup.web_utils import (
    existing_map_files, list_maps, MAP_ALL_EXTS, MAP_BACKUP_EXT, map_image_saved, MAP_LIST_LIMIT,
    MapPathError, POSEGRAPH_EXTS, posegraph_saved, ReservedMapNameError, resolve_map_base,
    valid_map_name, validate_goal)
from starlette.websockets import WebSocketState

TOKEN_ENV_VAR = 'MTRB_WEB_TOKEN'
DEFAULT_TOKEN_FILE = '~/.config/mtrb/web_token'
TOKEN_FILE_MAX_BYTES = 1024         # a token file bigger than this is refused
TOKEN_MIN_LENGTH = 20               # characters
TOKEN_FILE_FORBIDDEN_MODE = 0o077   # any group/other permission bit refuses the file
PLACEHOLDER_TOKEN_PREFIX = 'CHANGE-ME'
# Brute-force protection (per client IP, only the failed-token path feeds it).
AUTH_MAX_FAILURES = 10              # failed tokens tolerated inside AUTH_WINDOW_S
AUTH_WINDOW_S = 60.0
AUTH_BLOCK_S = 60.0                 # time an IP stays blocked after AUTH_MAX_FAILURES
AUTH_MAX_TRACKED_IPS = 1024         # bound of the failure table (memory)
AUTH_UNKNOWN_CLIENT = 'unknown'
EVENT_SUMMARY_PERIOD_S = 1.0        # at most one summarized log line per period
AUTH_LOG_PERIOD_S = 5.0
STATIC_FILES = {'/': 'index.html', '/app.js': 'app.js', '/style.css': 'style.css'}
STATIC_MEDIA_TYPES = {'/': 'text/html', '/app.js': 'text/javascript', '/style.css': 'text/css'}

WS_PATH = '/ws'
WS_CLOSE_UNAUTHORIZED = 1008  # policy violation; ONLY before accept (browser sees 1006)
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
        """
        Publish the pose estimate (frame `map`) on `initialpose`.

        Raise NavBusyError if a goal, an unconfirmed goal, a pending cancel-all or a teleop
        command may be moving the robot (the pose jump would corrupt the localization).
        """
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


class TokenError(Exception):
    """The token source is unusable (never carries the token or its file content)."""


def is_placeholder_token(token):
    """Return True if the token is the public example value from the repository."""
    return token.strip().upper().startswith(PLACEHOLDER_TOKEN_PREFIX)


def is_weak_token(token):
    """
    Return True if the token is a placeholder or has too little entropy.

    Weak means: example marker, shorter than TOKEN_MIN_LENGTH, blank, or a single
    repeated character.
    """
    stripped = token.strip()
    return (is_placeholder_token(stripped) or len(stripped) < TOKEN_MIN_LENGTH
            or len(set(stripped)) < 2)


def read_token_file(path):
    """
    Read the token from a private file; return None if the file does not exist.

    The file must be a REGULAR file (a symlink is refused: O_NOFOLLOW), owned by the user
    running the node, with no group/other permission (mode & 0o077 == 0). It is opened once
    and checked through the descriptor (no race between check and read). Raise TokenError
    with a fixed message otherwise; the content is never put in a message.
    """
    expanded = os.path.expanduser(path) if isinstance(path, str) else ''
    if not expanded or expanded.startswith('~') or not os.path.isabs(expanded):
        raise TokenError('auth.token_file must be an absolute path (or start with ~/)')
    flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | getattr(os, 'O_NONBLOCK', 0)
    try:
        fd = os.open(expanded, flags)
    except FileNotFoundError:
        return None
    except OSError as err:  # includes ELOOP for a symlink
        raise TokenError(
            f'Cannot open the token file ({type(err).__name__}): it must be a regular file, '
            'not a symlink') from err
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode):
            raise TokenError('The token file must be a regular file')
        if info.st_uid != os.geteuid():
            raise TokenError('The token file must be owned by the user running the node')
        if stat.S_IMODE(info.st_mode) & TOKEN_FILE_FORBIDDEN_MODE:
            raise TokenError('The token file must not be accessible by group or others '
                             '(chmod 600)')
        raw = os.read(fd, TOKEN_FILE_MAX_BYTES + 1)
    except OSError as err:
        raise TokenError(f'Cannot read the token file ({type(err).__name__})') from err
    finally:
        os.close(fd)
    if len(raw) > TOKEN_FILE_MAX_BYTES:
        raise TokenError('The token file is too large')
    try:
        return raw.decode('utf-8').strip()
    except UnicodeDecodeError:
        raise TokenError('The token file is not valid UTF-8') from None


def load_token(token_file, environ=None):
    """
    Return the token: env MTRB_WEB_TOKEN first, else the private file `token_file`.

    Neither source is a ROS parameter (a parameter is readable by anyone in the ROS domain).
    The value is stripped and never logged. Return '' if no source has a token; raise
    TokenError if the file exists but is unsafe.
    """
    environ = os.environ if environ is None else environ
    token = environ.get(TOKEN_ENV_VAR, '').strip()
    if token:
        return token
    return read_token_file(token_file) or ''


class EventSummary:
    """
    Turn a flood of identical events into at most one counted log line per period.

    `note()` counts an event and returns the text to log when the period has elapsed (the
    first event is returned at once); `poll()` returns the pending count when the period has
    elapsed without new events. Thread-safe; the clock is injectable (monotonic seconds).
    """

    def __init__(self, label, period_s=EVENT_SUMMARY_PERIOD_S, clock=time.monotonic):
        """Store the plural `label` used in the text and the minimum period in seconds."""
        self._label = label
        self._period_s = period_s
        self._clock = clock
        self._lock = threading.Lock()
        self._count = 0
        self._last_emit_s = None

    def _due_locked(self, now_s):
        """Return True if a line may be emitted now (lock held)."""
        return (self._last_emit_s is None or now_s < self._last_emit_s
                or now_s - self._last_emit_s >= self._period_s)

    def _render_locked(self, now_s):
        """Reset the counter and return the text for it (lock held)."""
        text = f'{self._count} {self._label} in the last window'
        self._count = 0
        self._last_emit_s = now_s
        return text

    def note(self):
        """Count one event; return the summary text if one is due, else None."""
        now_s = self._clock()
        with self._lock:
            self._count += 1
            return self._render_locked(now_s) if self._due_locked(now_s) else None

    def poll(self):
        """Return the summary text of pending events once the period elapsed, else None."""
        now_s = self._clock()
        with self._lock:
            if self._count and self._due_locked(now_s):
                return self._render_locked(now_s)
            return None


class AuthFailureLimiter:
    """
    Per-client-IP limit of failed tokens (anti brute force), with a bounded table.

    After `max_failures` failures inside `window_s` the IP is blocked for `block_s`. Only
    failures feed it; a valid token never creates an entry. The table keeps at most
    `max_ips` entries: expired ones are dropped first and then the oldest. The clock is
    injectable (monotonic seconds). Thread-safe.
    """

    def __init__(self, clock=time.monotonic, max_failures=AUTH_MAX_FAILURES,
                 window_s=AUTH_WINDOW_S, block_s=AUTH_BLOCK_S, max_ips=AUTH_MAX_TRACKED_IPS):
        """Store the limits and an empty table."""
        self._clock = clock
        self._max_failures = max_failures
        self._window_s = window_s
        self._block_s = block_s
        self._max_ips = max_ips
        self._lock = threading.Lock()
        self._table = collections.OrderedDict()  # ip -> [failures, window_start, blocked_until]

    def __len__(self):
        """Return the number of tracked IPs."""
        with self._lock:
            return len(self._table)

    def blocked(self, ip):
        """Return True while `ip` is blocked."""
        now_s = self._clock()
        with self._lock:
            entry = self._table.get(ip)
            if entry is None:
                return False
            if self._expired(entry, now_s):
                del self._table[ip]
                return False
            return entry[2] is not None and now_s < entry[2]

    def record_failure(self, ip):
        """Count a failed token of `ip`; return True if it is (now) blocked."""
        now_s = self._clock()
        with self._lock:
            entry = self._table.get(ip)
            if entry is not None and self._expired(entry, now_s):
                del self._table[ip]
                entry = None
            if entry is None:
                self._make_room_locked(now_s)
                entry = [0, now_s, None]
                self._table[ip] = entry
            if entry[2] is not None:
                return True  # already blocked: the block is not extended
            entry[0] += 1
            if entry[0] >= self._max_failures:
                entry[2] = now_s + self._block_s
                return True
            return False

    def _expired(self, entry, now_s):
        """Return True if the entry carries no information any more."""
        if entry[2] is not None:
            return now_s >= entry[2] or now_s < entry[2] - self._block_s  # clock went back
        return now_s - entry[1] >= self._window_s or now_s < entry[1]

    def _make_room_locked(self, now_s):
        """Free one slot if the table is full: expired entries first, else the oldest."""
        if len(self._table) < self._max_ips:
            return
        for key in [k for k, e in self._table.items() if self._expired(e, now_s)]:
            del self._table[key]
        while len(self._table) >= self._max_ips:
            self._table.popitem(last=False)


def _hostport(netloc, scheme):
    """Return (host, port) of a netloc, filling in the default port of `scheme`, or None."""
    try:
        parts = urlsplit('//' + netloc)
        host, port = parts.hostname, parts.port
    except ValueError:
        return None
    if not host or '@' in netloc:
        return None
    default = {'http': 80, 'ws': 80, 'https': 443, 'wss': 443}.get(scheme)
    return host.lower(), port if port is not None else default


def origin_matches_host(origin, host_header):
    """
    Return True if the Origin header names the same host:port as the Host header.

    Used against cross-site WebSocket hijacking. `origin` None (no header: a non-browser
    client) is accepted by the caller before calling this; here a missing or unparsable
    Host, the literal Origin `null` or another scheme than http/https is a mismatch.
    """
    if not host_header:
        return False
    try:
        parsed = urlsplit(origin)
    except ValueError:
        return False
    if parsed.scheme not in ('http', 'https') or not parsed.netloc:
        return False
    origin_hp = _hostport(parsed.netloc, parsed.scheme)
    host_hp = _hostport(host_header, parsed.scheme)
    return origin_hp is not None and origin_hp == host_hp


def client_key(client):
    """Return the IP string of a Starlette `client` (host, port) tuple, or a fixed key."""
    host = getattr(client, 'host', None)
    return host if isinstance(host, str) and host else AUTH_UNKNOWN_CLIENT


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
    except ReservedMapNameError:
        raise ApiError(400, 'reserved_name', 'That map name is not allowed') from None
    except ValueError as err:
        raise ApiError(400, 'invalid_name', str(err)) from None
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
    except (UnicodeDecodeError, ValueError, RecursionError):
        raise ApiError(400, 'invalid_json', 'Body is not valid JSON') from None


def _reject_constant(name):
    """json.loads hook: refuse NaN / Infinity / -Infinity literals."""
    raise ValueError(f'Non-finite constant {name}')


def create_app(store, token, static_dir=None, hub=None, max_ws_bytes=DEFAULT_MAX_WS_BYTES,
               nav_port=None, dock=None, map_port=None, map_dir=None, on_event=None,
               map_min_free_mb=MAP_MIN_FREE_MB_DEFAULT,
               map_min_interval_s=MAP_MIN_INTERVAL_S_DEFAULT, auth_limiter=None):
    """
    Build the FastAPI app. An empty token is rejected (fail closed).

    `auth_limiter` (AuthFailureLimiter) limits the failed tokens per client IP: an IP that
    is blocked gets 429 `too_many_attempts` (REST) or a rejected handshake (WebSocket), even
    with a valid token, EXCEPT on POST /api/stop, which a valid token always reaches. The
    client IP is the socket peer (X-Forwarded-For is never trusted). A WebSocket whose Origin
    header does not match its Host header is rejected before accept (anti CSWSH).

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

    limiter = AuthFailureLimiter() if auth_limiter is None else auth_limiter
    fail_log = EventSummary('failed authentication attempt(s)', AUTH_LOG_PERIOD_S)
    lock_log = EventSummary('request(s) refused for too many failed tokens',
                            AUTH_LOG_PERIOD_S)
    origin_log = EventSummary('WebSocket(s) refused for a foreign Origin', AUTH_LOG_PERIOD_S)

    def report(summary):
        """Count an event and log the summary line if one is due (never any credential)."""
        text = summary.note()
        if text is not None and on_event is not None:
            on_event('warning', text)

    def token_decision(given, client, stop_exempt=False):
        """
        Return None (allowed), 'invalid' (401) or 'locked' (429) for a presented token.

        The limiter is only fed by failures. A blocked IP is refused even with a valid
        token (else the block would not stop guessing) unless `stop_exempt`: the stop
        endpoint is never blocked for a valid token. An invalid token from a blocked IP is
        'locked' and is not counted again.
        """
        ip = client_key(client)
        if hmac.compare_digest(given, expected):
            if stop_exempt or not limiter.blocked(ip):
                return None
            report(lock_log)
            return 'locked'
        if limiter.blocked(ip):
            report(lock_log)
            return 'locked'
        report(fail_log)
        limiter.record_failure(ip)
        return 'invalid'

    def check_request_token(request, stop_exempt=False):
        decision = token_decision(_extract_bearer(request).encode('utf-8'), request.client,
                                  stop_exempt)
        if decision == 'locked':
            raise ApiError(429, 'too_many_attempts',
                           'Too many failed attempts: wait a minute',
                           {'Retry-After': str(int(AUTH_BLOCK_S))})
        if decision == 'invalid':
            raise ApiError(401, 'unauthorized', 'Missing or invalid token',
                           {'WWW-Authenticate': 'Bearer'})

    async def require_token(request: Request):
        # `async def`: pure CPU, runs on the event loop (never waits for the thread pool).
        check_request_token(request)

    async def require_token_stop(request: Request):
        check_request_token(request, stop_exempt=True)

    hub = BroadcastHub() if hub is None else hub
    client_ids = itertools.count(1)  # one unique id per WebSocket connection

    @app.websocket(WS_PATH)
    async def ws_endpoint(websocket: WebSocket):
        # Everything is decided BEFORE accepting: a refused client never takes a slot and a
        # browser sees a failed handshake (1006), never an accepted socket closed with 1008.
        origin = websocket.headers.get('origin')
        if origin is not None and not origin_matches_host(origin, websocket.headers.get('host')):
            report(origin_log)
            await websocket.close(code=WS_CLOSE_UNAUTHORIZED)
            return
        given = websocket.query_params.get('token', '').encode('utf-8')
        if token_decision(given, websocket.client) is not None:
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
            raise ApiError(400, 'invalid_goal', str(err)) from None
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

    @app.post('/api/stop', dependencies=[Depends(require_token_stop)])
    async def post_stop():
        # Zero Twist first, then cancel; independent of Nav2, map and mode. No body.
        # `async def` on purpose: handler and token check both run on the event loop (no
        # thread pool); NavPort.stop() itself must never block. A valid token is never
        # refused here, not even for an IP blocked for failed tokens.
        require_nav().stop()
        return {'ok': True}

    @app.post('/api/initialpose', dependencies=[Depends(require_token)])
    def post_initialpose(body=Depends(body_json)):
        pose = _pose_from_body(body)
        mode = store.snapshot()['mode']
        if mode != MAP_MODE:
            raise ApiError(409, 'wrong_mode',
                           f'Initial pose is only accepted in mode map (mode: {mode})')
        port = require_nav()
        grid = store.grid()
        if grid is None:  # fail safe: a pose that cannot be checked is not published
            raise ApiError(503, 'no_map', 'No map available yet')
        try:  # inside the map is enough: the cell may be free, unknown or occupied
            validate_goal(*pose, {key: value for key, value in grid.items() if key != 'data'})
        except ValueError as err:
            raise ApiError(400, 'invalid_pose', str(err).replace('Goal', 'Pose')) from None
        # The port answers NavBusyError (409 goal_active) while Nav2 may be driving (goal,
        # orphan goal, pending cancel-all) or a teleop controller is active.
        run_nav(port.publish_initial_pose, *pose)
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
