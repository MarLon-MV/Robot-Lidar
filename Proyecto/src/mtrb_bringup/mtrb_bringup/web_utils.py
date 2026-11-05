"""Pure helpers for the web bridge (no ROS imports, easy to test)."""

import array
import collections
import math
import os
import re
import struct
import time
import zlib

from mtrb_bringup.utils import yaw_to_quaternion

ZLIB_LEVEL = 1
OCCUPIED_THRESHOLD = 50  # Cells with a value >= this are obstacles (nav_msgs convention).
UNKNOWN_CELL = -1
SCAN_MIN_RANGE_M = 0.15
SCAN_MAX_RANGE_M = 12.0
# First character is never '-' (a name must not look like a command-line option).
MAP_NAME_PATTERN = re.compile(r'[A-Za-z0-9_][A-Za-z0-9_-]{0,39}')
# Reserved names (compared case-insensitively): `map` is the default map of the launch.
MAP_RESERVED_NAMES = frozenset(('map',))
COORD_DECIMALS = 3
# --- Saved maps (files written by slam_toolbox inside the configured map directory) ------
MAP_LIST_LIMIT = 200                # max entries returned by GET /api/maps
MAP_SCAN_MAX_ENTRIES = 10000        # max directory entries inspected while listing
# Same character set slam_toolbox accepts for the path it hands to a shell (see
# map_saver.cpp): the absolute base path we send must match it entirely.
MAP_PATH_PATTERN = re.compile(r'[A-Za-z0-9_./-]+')
MAP_YAML_EXT = '.yaml'
MAP_IMAGE_EXTS = ('.pgm', '.png')
POSEGRAPH_EXTS = ('.posegraph', '.data')
MAP_BACKUP_EXT = '.bak'             # suffix of the previous files while overwriting
MAP_ALL_EXTS = (MAP_YAML_EXT,) + MAP_IMAGE_EXTS + POSEGRAPH_EXTS
MAP_VERSION_MODULO = 2 ** 32  # the version travels as uint32

# --- Teleop gate constants (safety limits live here and in the node parameters) ---------
TELEOP_MAX_RATE_HZ = 30.0           # messages above this rate per client are dropped
TELEOP_ZERO_BURST = 3               # zero Twists published on every stop (robustness)
TELEOP_NOTIFY_PERIOD_S = 1.0        # min time between two error replies to one client
TELEOP_MAX_SEQ = 2 ** 53            # largest seq a JSON client can represent exactly
TELEOP_MAX_TRACKED_CLIENTS = 64     # bound of the per-client tables
# Result kinds of TeleopGate (see TeleopAction).
TELEOP_PUBLISH = 'publish'    # publish `twist` (saturated)
TELEOP_ZERO = 'zero'          # publish `zeros` zero Twists
TELEOP_REJECT = 'reject'      # not executed; `error` may carry the reply for the client
TELEOP_DROP = 'drop'          # silently ignored (stale seq or rate limit)
TELEOP_IDLE = 'idle'          # nothing to do


def _is_real_int(value):
    """Return True for an int that is not a bool."""
    return isinstance(value, int) and not isinstance(value, bool)


def _is_real_number(value):
    """
    Return True for a finite int/float that is not a bool.

    Huge ints (e.g. 10**400) overflow the float conversion: they are not valid numbers.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, ValueError):
        return False


def occupancy_to_bytes(data, width, height):
    """Compress occupancy values (-1, 0..100) row-major into zlib bytes (level 1)."""
    if not (_is_real_int(width) and _is_real_int(height)):
        raise ValueError('Map width and height must be integers')
    if width <= 0 or height <= 0:
        raise ValueError(f'Map size must be positive (got {width}x{height})')
    try:
        cell_count = len(data)
    except TypeError as err:
        raise ValueError('Map data must be a sequence') from err
    if cell_count != width * height:
        raise ValueError(f'Map data has {len(data)} cells, expected {width * height}')
    raw = _int8_buffer_to_bytes(data)
    if raw is None and isinstance(data, (list, tuple)):
        try:  # C-speed conversion of a plain sequence; values outside int8 use the slow path
            raw = array.array('b', data).tobytes()
        except (TypeError, ValueError, OverflowError):
            raw = None
    if raw is None:
        try:
            raw = bytes(int(v) & 0xFF for v in data)  # int8 two's complement: -1 -> 0xFF
        except (TypeError, ValueError, OverflowError) as err:
            raise ValueError(f'Invalid occupancy value: {err}') from err
    return zlib.compress(raw, ZLIB_LEVEL)


def _int8_buffer_to_bytes(data):
    """Fast path: raw bytes of a 1-D int8 buffer (array('b'), numpy int8), else None."""
    try:
        view = memoryview(data)
    except TypeError:
        return None
    if view.format != 'b' or view.ndim != 1:
        return None
    return view.tobytes()


def pack_map_binary(version, compressed):
    """Binary map message: uint32 little-endian `version` followed by the zlib payload."""
    if not _is_real_int(version) or version < 0:
        raise ValueError('Map version must be a non-negative integer')
    return struct.pack('<I', version % MAP_VERSION_MODULO) + bytes(compressed)


def yaw_from_quaternion(x, y, z, w):
    """Yaw in radians, in (-pi, pi], of a quaternion (inverse of quaternion_from_yaw)."""
    if not all(_is_real_number(v) for v in (x, y, z, w)):
        raise ValueError('Quaternion components must be finite numbers')
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.pi if yaw <= -math.pi else yaw


def quaternion_from_yaw(yaw):
    """Quaternion (x, y, z, w) for `yaw` radians; reuses utils.yaw_to_quaternion."""
    if not _is_real_number(yaw):
        raise ValueError('Yaw must be a finite number')
    return yaw_to_quaternion(yaw)


def downsample_scan(ranges, angle_min, angle_increment, max_points,
                    r_min=SCAN_MIN_RANGE_M, r_max=SCAN_MAX_RANGE_M):
    """
    Return at most `max_points` valid (range_m, angle_rad) pairs, evenly spread.

    Beams that are inf/nan or outside [r_min, r_max] are dropped.
    """
    if not _is_real_int(max_points) or max_points <= 0:
        raise ValueError('max_points must be a positive integer')
    if not all(_is_real_number(v) for v in (angle_min, angle_increment, r_min, r_max)):
        raise ValueError('Scan angles and range limits must be finite numbers')
    try:
        iterator = iter(ranges)
    except TypeError as err:
        raise ValueError('ranges must be a sequence') from err
    valid = []
    for index, r in enumerate(iterator):
        if isinstance(r, bool) or not isinstance(r, (int, float)):
            raise ValueError('Scan range is not a number')
        # Non-finite (also an int too big for float) beams are dropped like inf/nan.
        if _is_real_number(r) and r_min <= r <= r_max:
            valid.append((float(r), angle_min + index * angle_increment))
    count = len(valid)
    if count <= max_points:
        return valid
    return [valid[i * count // max_points] for i in range(max_points)]


def scan_to_map_points(points, pose):
    """
    Project (range_m, angle_rad) points from the laser frame to the map frame.

    `pose` is (x, y, yaw) of the laser in `map`. Returns [x0, y0, x1, y1, ...] rounded
    to 3 decimals.
    """
    try:
        px, py, pyaw = pose
    except (TypeError, ValueError) as err:
        raise ValueError('Pose must be (x, y, yaw)') from err
    if not all(_is_real_number(v) for v in (px, py, pyaw)):
        raise ValueError('Pose must be finite numbers')
    try:
        pairs = [(r, angle) for r, angle in points]
    except (TypeError, ValueError) as err:
        raise ValueError('points must be a sequence of (range, angle) pairs') from err
    out = []
    for r, angle in pairs:
        if not (_is_real_number(r) and _is_real_number(angle)):
            raise ValueError('Scan point range and angle must be finite numbers')
        a = pyaw + angle
        out.append(round(px + r * math.cos(a), COORD_DECIMALS))
        out.append(round(py + r * math.sin(a), COORD_DECIMALS))
    return out


def simplify_path(points, max_points):
    """Decimate a path to at most `max_points` [x, y] pairs keeping first and last."""
    if not _is_real_int(max_points) or max_points < 2:
        raise ValueError('max_points must be an integer of at least 2')
    try:
        path = [[float(p[0]), float(p[1])] for p in points]
    except (TypeError, ValueError, IndexError, KeyError, OverflowError) as err:
        raise ValueError(f'Invalid path point: {err}') from err
    count = len(path)
    if count <= max_points:
        return path
    last = max_points - 1
    return [path[i * (count - 1) // last] for i in range(max_points)]


def validate_goal(x, y, yaw, map_info):
    """
    Check a navigation goal and return (x, y, yaw) as floats, or raise ValueError.

    `map_info` is a dict with width, height, resolution, origin_x, origin_y and,
    optionally, data (row-major int8 values). The goal must be finite, inside the map
    and not on an occupied cell (>= 50); unknown cells (-1) are accepted.
    """
    if not all(_is_real_number(v) for v in (x, y, yaw)):
        raise ValueError('Goal x, y and yaw must be finite numbers')
    if map_info is None:
        raise ValueError('No map available')
    try:
        width = int(map_info['width'])
        height = int(map_info['height'])
        resolution = float(map_info['resolution'])
        origin_x = float(map_info['origin_x'])
        origin_y = float(map_info['origin_y'])
    except (KeyError, TypeError, ValueError, OverflowError) as err:
        raise ValueError(f'Invalid map info: {err}') from err
    if width <= 0 or height <= 0 or not resolution > 0.0:
        raise ValueError('Invalid map info: size and resolution must be positive')
    try:
        col = math.floor((x - origin_x) / resolution)
        row = math.floor((y - origin_y) / resolution)
    except (OverflowError, ValueError) as err:
        raise ValueError('Goal is outside the map (non-finite cell index)') from err
    if not (0 <= col < width and 0 <= row < height):
        raise ValueError('Goal is outside the map')
    data = map_info.get('data')
    if data is not None:
        if isinstance(data, (str, bytes, dict)):
            raise ValueError('Invalid map info: data is not a sequence')
        try:
            data_len = len(data)
        except TypeError as err:
            raise ValueError('Invalid map info: data is not a sequence') from err
        if data_len != width * height:
            raise ValueError('Invalid map info: data size does not match width x height')
        cell = data[row * width + col]
        if not _is_real_number(cell):
            raise ValueError('Invalid map info: data cell is not a finite number')
        if cell >= OCCUPIED_THRESHOLD:
            raise ValueError('Goal is on an occupied cell')
    return float(x), float(y), float(yaw)


def clamp_twist(vx, wz, max_vx, max_wz):
    """Saturate a velocity command to +-max_vx (m/s) and +-max_wz (rad/s)."""
    if not all(_is_real_number(v) for v in (vx, wz, max_vx, max_wz)):
        raise ValueError('Twist and limits must be finite')
    if max_vx < 0.0 or max_wz < 0.0:
        raise ValueError('Twist limits must not be negative')
    return max(-max_vx, min(max_vx, vx)), max(-max_wz, min(max_wz, wz))


class ReservedMapNameError(ValueError):
    """The map name is syntactically valid but reserved (cannot be saved)."""


def is_reserved_map_name(name):
    """Return True if `name` is reserved (case-insensitive)."""
    return isinstance(name, str) and name.lower() in MAP_RESERVED_NAMES


def listable_map_name(name):
    """Return True if `name` is valid and not reserved (it may appear in the map list)."""
    return (isinstance(name, str) and MAP_NAME_PATTERN.fullmatch(name) is not None
            and not is_reserved_map_name(name))


def valid_map_name(name):
    """
    Return `name` if it matches [A-Za-z0-9_][A-Za-z0-9_-]{0,39}, else raise ValueError.

    Raise ReservedMapNameError (a ValueError) for reserved names such as `map`.
    """
    if not isinstance(name, str) or MAP_NAME_PATTERN.fullmatch(name) is None:
        raise ValueError('Map name must match [A-Za-z0-9_][A-Za-z0-9_-]{0,39}')
    if is_reserved_map_name(name):
        raise ReservedMapNameError('Map name is reserved')
    return name


class MapPathError(ValueError):
    """A map directory or file path is unsafe or unusable (never built from raw client data)."""


def safe_map_dir(map_dir):
    """
    Return the real (symlink-free) absolute path of the existing map directory.

    Raise MapPathError if it is not an absolute path to a directory, is the filesystem root
    or has characters that slam_toolbox would not accept in a path.
    """
    if not isinstance(map_dir, str) or not map_dir or not os.path.isabs(map_dir):
        raise MapPathError('The map directory must be an absolute path')
    real_dir = os.path.realpath(map_dir)
    if real_dir == os.sep or not os.path.isdir(real_dir):
        raise MapPathError('The map directory does not exist or is the filesystem root')
    if MAP_PATH_PATTERN.fullmatch(real_dir) is None:
        raise MapPathError('The map directory path has characters outside [A-Za-z0-9_./-]')
    return real_dir


def resolve_map_base(map_dir, name):
    """
    Return the absolute base path (no extension) of map `name` inside `map_dir`.

    The only client data used is `name`, validated with valid_map_name (ValueError). Every
    related file (yaml, image, pose-graph, backups) that already exists must be a plain file
    located directly in the real map directory: a symlink, a directory or anything resolving
    outside it raises MapPathError.
    """
    valid_map_name(name)
    real_dir = safe_map_dir(map_dir)
    base = os.path.join(real_dir, name)
    for ext in MAP_ALL_EXTS + tuple(e + MAP_BACKUP_EXT for e in MAP_ALL_EXTS):
        path = base + ext
        if not os.path.lexists(path):
            continue
        if os.path.islink(path) or not os.path.isfile(path):
            raise MapPathError(f'Existing map file {name}{ext} is not a plain file')
        if os.path.dirname(os.path.realpath(path)) != real_dir:
            raise MapPathError(f'Map file {name}{ext} resolves outside the map directory')
    return base


def existing_map_files(base):
    """Return the paths among base + (yaml, images, pose-graph) that exist (symlinks too)."""
    return [base + ext for ext in MAP_ALL_EXTS if os.path.lexists(base + ext)]


def _plain_nonempty_file(path):
    """Return True for a regular, non-symlink file with at least one byte."""
    try:
        return (not os.path.islink(path) and os.path.isfile(path)
                and os.path.getsize(path) > 0)
    except OSError:
        return False


def map_image_saved(base):
    """Return True if base.yaml and an image (pgm or png) exist as non-empty plain files."""
    return (_plain_nonempty_file(base + MAP_YAML_EXT)
            and any(_plain_nonempty_file(base + ext) for ext in MAP_IMAGE_EXTS))


def posegraph_saved(base):
    """Return True if base.posegraph and base.data exist as non-empty plain files."""
    return all(_plain_nonempty_file(base + ext) for ext in POSEGRAPH_EXTS)


def list_maps(map_dir, limit=MAP_LIST_LIMIT):
    """
    Return the sorted names of the saved maps (yaml + image) of `map_dir`, at most `limit`.

    A missing directory means no maps. Names that are not valid map names, symlinks and
    entries that are not plain files are ignored. Raises MapPathError for an unusable path.
    """
    if not isinstance(map_dir, str) or not os.path.isdir(map_dir):
        return []
    real_dir = safe_map_dir(map_dir)
    names = []
    with os.scandir(real_dir) as entries:
        for index, entry in enumerate(entries):
            if index >= MAP_SCAN_MAX_ENTRIES:
                break
            stem, ext = os.path.splitext(entry.name)
            if ext != MAP_YAML_EXT or not listable_map_name(stem):
                continue
            base = os.path.join(real_dir, stem)
            if (entry.is_file(follow_symlinks=False) and not entry.is_symlink()
                    and any(_plain_nonempty_file(base + e) for e in MAP_IMAGE_EXTS)):
                names.append(stem)
    return sorted(names)[:limit]


TeleopAction = collections.namedtuple('TeleopAction', ('kind', 'twist', 'zeros', 'error'))
TeleopAction.__doc__ = """
What the node must do after a TeleopGate decision.

`twist` is (vx_mps, wz_radps) or None; `zeros` is how many zero Twists to publish (before
`twist`, if both are set); `error` is (code, detail) to send to the client, or None.
"""


def _teleop_action(kind, twist=None, zeros=0, error=None):
    """Build a TeleopAction with defaults."""
    return TeleopAction(kind, twist, zeros, error)


class TeleopGate:
    """
    Pure decision logic of the web teleoperation (no ROS, injectable clock).

    It decides what the node publishes on `cmd_vel_nav`; the node only executes the returned
    TeleopAction. It is NOT thread-safe: the node serializes the calls (and the publishing)
    with one lock. Fail-safe rules:

    - one controller at a time (lease): while a client sent a valid command within
      `timeout_s`, other clients are rejected;
    - a command is accepted only if its fields are finite numbers, its `seq` is greater than
      the last one of that client, and it is not faster than TELEOP_MAX_RATE_HZ;
    - velocities are saturated with clamp_twist to the configured limits;
    - if no valid command arrives within `timeout_s` the watchdog orders zero Twists once
      (a short burst) and then nothing; a busy navigation, a stop or the disconnect of
      the owner also end the teleop with zeros.

    State machine: IDLE --valid teleop--> ACTIVE --valid teleop--> ACTIVE;
    ACTIVE --timeout | stop | owner disconnect | navigation busy--> IDLE (zero burst).
    """

    def __init__(self, max_vx, max_wz, timeout_s, clock=time.monotonic, enabled=True,
                 max_rate_hz=TELEOP_MAX_RATE_HZ, zero_burst=TELEOP_ZERO_BURST,
                 notify_period_s=TELEOP_NOTIFY_PERIOD_S):
        """
        Store the limits (m/s, rad/s), the timeout (s) and the clock (callable -> s).

        The default clock is monotonic on purpose: the watchdog must not depend on the ROS or
        wall clock, which can jump backwards (NTP step) and freeze the timeout. The clock
        stays injectable for tests.
        """
        numbers = (max_vx, max_wz, timeout_s, max_rate_hz, notify_period_s)
        if not all(_is_real_number(v) for v in numbers):
            raise ValueError('Teleop limits, timeout and rates must be finite numbers')
        if max_vx <= 0.0 or max_wz <= 0.0 or timeout_s <= 0.0 or max_rate_hz <= 0.0:
            raise ValueError('Teleop limits, timeout and rate must be positive')
        if notify_period_s < 0.0 or not _is_real_int(zero_burst) or zero_burst < 1:
            raise ValueError('Invalid teleop notify period or zero burst')
        if not isinstance(enabled, bool) or not callable(clock):
            raise ValueError('enabled must be a bool and clock a callable')
        self._max_vx = float(max_vx)
        self._max_wz = float(max_wz)
        self._timeout_s = float(timeout_s)
        self._min_interval_s = 1.0 / max_rate_hz
        self._zero_burst = zero_burst
        self._notify_period_s = float(notify_period_s)
        self._enabled = enabled
        self._clock = clock
        self._owner = None          # client id holding the lease (None = nobody)
        self._last_valid_s = None   # clock time of the last valid command of the owner
        self._last_seq = {}         # client id -> last accepted seq
        self._last_accept_s = {}    # client id -> clock time of the last accepted command
        self._last_notify_s = {}    # client id -> clock time of the last error reply

    @property
    def active(self):
        """True while a controller holds the lease and its command has not expired."""
        return self._owner is not None and self._fresh(self._clock())

    @property
    def owner_age_s(self):
        """Seconds since the last valid command of the owner, or None if nobody owns it."""
        if self._owner is None:
            return None
        return self._clock() - self._last_valid_s

    def _fresh(self, now_s):
        """Return True if the owner's last command is recent (a backwards clock is stale)."""
        age_s = now_s - self._last_valid_s
        return 0.0 <= age_s <= self._timeout_s

    def _end_teleop(self):
        """Release the lease; return the zero-burst action."""
        self._owner = None
        self._last_valid_s = None
        return _teleop_action(TELEOP_ZERO, zeros=self._zero_burst)

    def _reject(self, client_id, now_s, code, detail, kind=TELEOP_REJECT, zeros=0):
        """Rejected command; the error reply is rate-limited per client."""
        last = self._last_notify_s.get(client_id)
        error = None
        if last is None or now_s - last >= self._notify_period_s or now_s < last:
            self._remember(self._last_notify_s, client_id, now_s)
            error = (code, detail)
        return _teleop_action(kind, zeros=zeros, error=error)

    def _remember(self, table, client_id, value):
        """Store a per-client value keeping the table bounded (drops the oldest client)."""
        if client_id not in table and len(table) >= TELEOP_MAX_TRACKED_CLIENTS:
            for old in list(table):
                if old != self._owner:
                    del table[old]
                    break
        table[client_id] = value

    @staticmethod
    def _parse(message):
        """Return (vx, wz, seq) from a teleop message dict, or raise ValueError."""
        if not isinstance(message, dict):
            raise ValueError('Teleop message must be an object')
        vx, wz, seq = message.get('vx'), message.get('wz'), message.get('seq')
        if not (_is_real_number(vx) and _is_real_number(wz)):
            raise ValueError('vx and wz must be finite numbers')
        if not _is_real_int(seq) or not 0 <= seq <= TELEOP_MAX_SEQ:
            raise ValueError('seq must be a non-negative integer')
        return float(vx), float(wz), seq

    def on_teleop(self, client_id, message, nav_busy):
        """
        Decide what to do with a teleop `message` (dict) from `client_id`.

        `nav_busy` is True while Nav2 may be driving (goal, orphan or cancel-all pending).
        """
        now_s = self._clock()
        if not self._enabled:
            return self._reject(client_id, now_s, 'teleop_disabled', 'Teleop is disabled')
        if nav_busy:
            # A goal may be driving: reject, never mix two controllers. If we were the
            # active controller, stop it too.
            was_owned = self._owner is not None
            if was_owned:
                self._end_teleop()
            return self._reject(client_id, now_s, 'teleop_rejected',
                                'A navigation goal is active: stop it first',
                                kind=TELEOP_ZERO if was_owned else TELEOP_REJECT,
                                zeros=self._zero_burst if was_owned else 0)
        if (self._owner is not None and self._owner != client_id
                and self._fresh(now_s)):
            return self._reject(client_id, now_s, 'teleop_rejected',
                                'Another client is controlling the robot')
        try:
            vx, wz, seq = self._parse(message)
        except ValueError as err:
            return self._reject(client_id, now_s, 'invalid_teleop', str(err))
        last_seq = self._last_seq.get(client_id)
        if last_seq is not None and seq <= last_seq:
            return _teleop_action(TELEOP_DROP)
        last_accept = self._last_accept_s.get(client_id)
        if (last_accept is not None and 0.0 <= now_s - last_accept < self._min_interval_s):
            return _teleop_action(TELEOP_DROP)
        vx, wz = clamp_twist(vx, wz, self._max_vx, self._max_wz)
        self._owner = client_id
        self._last_valid_s = now_s
        self._remember(self._last_seq, client_id, seq)
        self._remember(self._last_accept_s, client_id, now_s)
        return _teleop_action(TELEOP_PUBLISH, twist=(vx, wz))

    def watchdog_tick(self):
        """Periodic check: zero burst once when the owner's command expired, else idle."""
        if self._owner is None:
            return _teleop_action(TELEOP_IDLE)
        if self._fresh(self._clock()):
            return _teleop_action(TELEOP_IDLE)
        return self._end_teleop()

    def stop(self):
        """Stop request (always honoured, no lease or rate checks): zero burst, no owner."""
        return self._end_teleop()

    def client_disconnected(self, client_id):
        """Forget `client_id`; if it was the owner, order the zero burst immediately."""
        self._last_seq.pop(client_id, None)
        self._last_accept_s.pop(client_id, None)
        self._last_notify_s.pop(client_id, None)
        if self._owner is not None and self._owner == client_id:
            return self._end_teleop()
        return _teleop_action(TELEOP_IDLE)
