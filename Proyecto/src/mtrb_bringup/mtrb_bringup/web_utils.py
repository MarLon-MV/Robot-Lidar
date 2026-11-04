"""Pure helpers for the web bridge (no ROS imports, easy to test)."""

import math
import re
import struct
import zlib

from mtrb_bringup.utils import yaw_to_quaternion

ZLIB_LEVEL = 1
OCCUPIED_THRESHOLD = 50  # Cells with a value >= this are obstacles (nav_msgs convention).
UNKNOWN_CELL = -1
SCAN_MIN_RANGE_M = 0.15
SCAN_MAX_RANGE_M = 12.0
MAP_NAME_PATTERN = re.compile(r'[A-Za-z0-9_-]{1,40}')
COORD_DECIMALS = 3
MAP_VERSION_MODULO = 2 ** 32  # the version travels as uint32


def _is_real_int(value):
    """Return True for an int that is not a bool."""
    return isinstance(value, int) and not isinstance(value, bool)


def _is_real_number(value):
    """Return True for a finite int/float that is not a bool."""
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


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
    if raw is None:
        try:
            raw = bytes(int(v) & 0xFF for v in data)  # int8 two's complement: -1 -> 0xFF
        except (TypeError, ValueError) as err:
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
        if math.isfinite(r) and r_min <= r <= r_max:
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
    except (TypeError, ValueError, IndexError, KeyError) as err:
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
    except (KeyError, TypeError, ValueError) as err:
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


def valid_map_name(name):
    """Return `name` if it matches ^[A-Za-z0-9_-]{1,40}$, else raise ValueError."""
    if not isinstance(name, str) or MAP_NAME_PATTERN.fullmatch(name) is None:
        raise ValueError('Map name must match [A-Za-z0-9_-]{1,40}')
    return name
