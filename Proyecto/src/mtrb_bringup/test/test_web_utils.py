"""Unit tests for mtrb_bringup.web_utils (pure Python, no ROS needed)."""

import math
import zlib

from mtrb_bringup.web_utils import (clamp_twist, downsample_scan, occupancy_to_bytes,
                                    quaternion_from_yaw, scan_to_map_points,
                                    simplify_path, valid_map_name,
                                    validate_goal, yaw_from_quaternion)
import pytest

NAN = float('nan')
INF = float('inf')


# --- occupancy_to_bytes -------------------------------------------------------------

def test_occupancy_roundtrip_preserves_unknown_free_occupied():
    packed = occupancy_to_bytes([-1, 0, 100, 0], 2, 2)
    raw = zlib.decompress(packed)
    assert list(raw) == [0xFF, 0, 100, 0], f'unexpected bytes: {list(raw)}'
    signed = [b - 256 if b > 127 else b for b in raw]
    assert signed == [-1, 0, 100, 0]


@pytest.mark.parametrize('size', [3, 5, 0])
def test_occupancy_wrong_data_size_raises(size):
    with pytest.raises(ValueError):
        occupancy_to_bytes([0] * size, 2, 2)


@pytest.mark.parametrize('width,height', [(0, 2), (2, 0), (-1, 2), (2, -3)])
def test_occupancy_non_positive_size_raises(width, height):
    with pytest.raises(ValueError):
        occupancy_to_bytes([], width, height)


def test_occupancy_invalid_value_raises_value_error():
    with pytest.raises(ValueError):
        occupancy_to_bytes([0, 'x', 0, 0], 2, 2)


# --- yaw / quaternion ---------------------------------------------------------------

@pytest.mark.parametrize('yaw', [0.0, math.pi / 2, -math.pi / 2, 1.0, -2.5])
def test_yaw_roundtrip(yaw):
    x, y, z, w = quaternion_from_yaw(yaw)
    assert yaw_from_quaternion(x, y, z, w) == pytest.approx(yaw, abs=1e-9)


@pytest.mark.parametrize('yaw', [math.pi, -math.pi])
def test_yaw_at_pi_is_returned_as_plus_pi(yaw):
    x, y, z, w = quaternion_from_yaw(yaw)
    result = yaw_from_quaternion(x, y, z, w)
    assert result == pytest.approx(math.pi, abs=1e-9)
    assert result > 0.0


def test_yaw_from_quaternion_exact_minus_pi_boundary():
    # atan2(-0.0, -1) == -pi; must be mapped to +pi.
    assert yaw_from_quaternion(0.0, 0.0, 1.0, -0.0) == math.pi


@pytest.mark.parametrize('bad', [NAN, INF, -INF])
def test_yaw_from_quaternion_non_finite_raises(bad):
    with pytest.raises(ValueError):
        yaw_from_quaternion(0.0, 0.0, bad, 1.0)


@pytest.mark.parametrize('bad', [NAN, INF, -INF])
def test_quaternion_from_yaw_non_finite_raises(bad):
    with pytest.raises(ValueError):
        quaternion_from_yaw(bad)


# --- downsample_scan ----------------------------------------------------------------

def test_scan_drops_inf_and_nan():
    result = downsample_scan([1.0, INF, NAN, 2.0, -INF], 0.0, 0.1, 10)
    assert [r for r, _ in result] == [1.0, 2.0]
    assert result[1][1] == pytest.approx(0.3)


def test_scan_keeps_range_extremes_inclusive():
    result = downsample_scan([0.15, 12.0], 0.0, 1.0, 10)
    assert [r for r, _ in result] == [0.15, 12.0]


@pytest.mark.parametrize('value', [0.0, 0.1499, 12.0001, 50.0, -1.0])
def test_scan_drops_out_of_range(value):
    assert downsample_scan([value, 1.0], 0.0, 1.0, 10) == [(1.0, 1.0)]


def test_scan_limits_points_when_more_beams_than_max():
    ranges = [1.0 + i * 0.01 for i in range(100)]
    result = downsample_scan(ranges, 0.0, 0.01, 10)
    assert len(result) == 10
    assert result[0][0] == pytest.approx(1.0)  # first beam kept
    angles = [a for _, a in result]
    assert angles == sorted(angles)


def test_scan_max_points_equal_to_valid_returns_all():
    result = downsample_scan([1.0, 2.0, 3.0], 0.0, 1.0, 3)
    assert len(result) == 3


def test_scan_max_points_one():
    assert len(downsample_scan([1.0, 2.0, 3.0], 0.0, 1.0, 1)) == 1


@pytest.mark.parametrize('max_points', [0, -1, -100])
def test_scan_non_positive_max_points_raises(max_points):
    with pytest.raises(ValueError):
        downsample_scan([1.0], 0.0, 1.0, max_points)


def test_scan_empty_input_returns_empty():
    assert downsample_scan([], 0.0, 1.0, 5) == []


# --- simplify_path ------------------------------------------------------------------

def test_simplify_path_empty():
    assert simplify_path([], 5) == []


def test_simplify_path_one_point():
    assert simplify_path([(1, 2)], 5) == [[1.0, 2.0]]


def test_simplify_path_two_points():
    assert simplify_path([(0, 0), (1, 1)], 2) == [[0.0, 0.0], [1.0, 1.0]]


def test_simplify_path_keeps_first_and_last_and_limits_count():
    points = [(float(i), 0.0) for i in range(101)]
    result = simplify_path(points, 10)
    assert len(result) == 10
    assert result[0] == [0.0, 0.0]
    assert result[-1] == [100.0, 0.0]


def test_simplify_path_two_max_points_keeps_endpoints():
    points = [(float(i), 0.0) for i in range(7)]
    assert simplify_path(points, 2) == [[0.0, 0.0], [6.0, 0.0]]


def test_simplify_path_shorter_than_max_unchanged():
    points = [(0, 0), (1, 1), (2, 2)]
    assert simplify_path(points, 10) == [[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]]


@pytest.mark.parametrize('max_points', [1, 0, -5])
def test_simplify_path_max_points_below_two_raises(max_points):
    with pytest.raises(ValueError):
        simplify_path([(0, 0), (1, 1)], max_points)


# --- validate_goal ------------------------------------------------------------------

def make_map(data=None, **overrides):
    info = {'width': 4, 'height': 4, 'resolution': 0.5, 'origin_x': -1.0, 'origin_y': -1.0}
    if data is not None:
        info['data'] = data
    info.update(overrides)
    return info


def test_goal_inside_free_map_is_accepted():
    assert validate_goal(0.0, 0.0, 1, make_map([0] * 16)) == (0.0, 0.0, 1.0)


def test_goal_without_data_is_accepted_when_inside():
    assert validate_goal(0.0, 0.0, 0.0, make_map()) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('x,y', [(-1.0, -1.0), (0.99, 0.99), (-1.0, 0.99), (0.99, -1.0)])
def test_goal_on_map_borders_is_accepted(x, y):
    assert validate_goal(x, y, 0.0, make_map([0] * 16))[:2] == (x, y)


@pytest.mark.parametrize('x,y', [(-1.01, 0.0), (0.0, -1.01), (1.0, 0.0), (0.0, 1.0),
                                 (10.0, 10.0), (-10.0, 0.0)])
def test_goal_outside_map_raises(x, y):
    with pytest.raises(ValueError):
        validate_goal(x, y, 0.0, make_map([0] * 16))


@pytest.mark.parametrize('value', [50, 51, 100])
def test_goal_on_occupied_cell_raises(value):
    data = [0] * 16
    data[2 * 4 + 2] = value  # cell containing (0.0, 0.0)
    with pytest.raises(ValueError):
        validate_goal(0.0, 0.0, 0.0, make_map(data))


def test_goal_on_cell_just_below_threshold_is_accepted():
    data = [0] * 16
    data[2 * 4 + 2] = 49
    assert validate_goal(0.0, 0.0, 0.0, make_map(data)) == (0.0, 0.0, 0.0)


def test_goal_on_unknown_cell_is_accepted_per_d3_proposal():
    # D-3 (unknown cells accepted) is a proposal not yet confirmed by the human.
    data = [0] * 16
    data[2 * 4 + 2] = -1
    assert validate_goal(0.0, 0.0, 0.0, make_map(data)) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('goal', [(NAN, 0.0, 0.0), (0.0, INF, 0.0), (0.0, 0.0, -INF),
                                  (0.0, 0.0, NAN), ('a', 0.0, 0.0), (None, 0.0, 0.0)])
def test_goal_non_finite_or_non_numeric_raises(goal):
    with pytest.raises(ValueError):
        validate_goal(*goal, make_map([0] * 16))


def test_goal_without_map_raises():
    with pytest.raises(ValueError):
        validate_goal(0.0, 0.0, 0.0, None)


@pytest.mark.parametrize('missing', ['width', 'height', 'resolution', 'origin_x', 'origin_y'])
def test_goal_incomplete_map_info_raises_value_error(missing):
    info = make_map([0] * 16)
    del info[missing]
    with pytest.raises(ValueError):
        validate_goal(0.0, 0.0, 0.0, info)


@pytest.mark.parametrize('overrides', [{'width': 0}, {'height': -1}, {'resolution': 0.0},
                                       {'resolution': -0.5}, {'resolution': NAN},
                                       {'width': 'abc'}, {'origin_x': None}])
def test_goal_invalid_map_values_raise(overrides):
    with pytest.raises(ValueError):
        validate_goal(0.0, 0.0, 0.0, make_map([0] * 16, **overrides))


@pytest.mark.parametrize('size', [0, 15, 17])
def test_goal_data_size_mismatch_raises(size):
    with pytest.raises(ValueError):
        validate_goal(0.0, 0.0, 0.0, make_map([0] * size))


def test_goal_returns_floats():
    result = validate_goal(0, 0, 1, make_map())
    assert all(isinstance(v, float) for v in result)


# --- clamp_twist --------------------------------------------------------------------

def test_clamp_twist_within_limits_unchanged():
    assert clamp_twist(0.1, -0.2, 0.3, 1.0) == (0.1, -0.2)


def test_clamp_twist_saturates_positive_and_negative():
    assert clamp_twist(5.0, 9.0, 0.3, 1.0) == (0.3, 1.0)
    assert clamp_twist(-5.0, -9.0, 0.3, 1.0) == (-0.3, -1.0)


def test_clamp_twist_zero_limits_force_zero():
    assert clamp_twist(1.0, -1.0, 0.0, 0.0) == (0.0, 0.0)


@pytest.mark.parametrize('args', [(NAN, 0.0, 1.0, 1.0), (0.0, INF, 1.0, 1.0),
                                  (0.0, 0.0, NAN, 1.0), (0.0, 0.0, 1.0, INF),
                                  (-INF, 0.0, 1.0, 1.0)])
def test_clamp_twist_non_finite_raises(args):
    with pytest.raises(ValueError):
        clamp_twist(*args)


@pytest.mark.parametrize('args', [(0.0, 0.0, -0.1, 1.0), (0.0, 0.0, 1.0, -0.1)])
def test_clamp_twist_negative_limits_raise(args):
    with pytest.raises(ValueError):
        clamp_twist(*args)


# --- valid_map_name -----------------------------------------------------------------

@pytest.mark.parametrize('name', ['a', 'lab_1', 'Map-2', 'x' * 40, '0'])
def test_map_name_valid(name):
    assert valid_map_name(name) == name


@pytest.mark.parametrize('name', ['../x', '', 'x' * 41, 'my map', 'abc\n', '\nabc',
                                  'a/b', 'a.b', 'ñandú', ' a', 'a '])
def test_map_name_invalid_string_raises(name):
    with pytest.raises(ValueError):
        valid_map_name(name)


@pytest.mark.parametrize('name', [None, 5, b'abc', ['a'], 1.5])
def test_map_name_non_str_raises(name):
    with pytest.raises(ValueError):
        valid_map_name(name)


# --- scan_to_map_points -------------------------------------------------------------

def test_scan_to_map_points_identity_pose():
    out = scan_to_map_points([(1.0, 0.0), (2.0, math.pi / 2)], (0.0, 0.0, 0.0))
    assert out == [1.0, 0.0, 0.0, 2.0]


def test_scan_to_map_points_applies_translation_and_yaw():
    out = scan_to_map_points([(1.0, 0.0)], (1.0, 2.0, math.pi / 2))
    assert out == [1.0, 3.0]


def test_scan_to_map_points_empty_returns_empty():
    assert scan_to_map_points([], (0.0, 0.0, 0.0)) == []


def test_scan_to_map_points_rounds_to_three_decimals():
    out = scan_to_map_points([(1.0, 0.0)], (0.123456, 0.0, 0.0))
    assert out == [1.123, 0.0]


@pytest.mark.parametrize('pose', [(NAN, 0.0, 0.0), (0.0, INF, 0.0), (0.0, 0.0, NAN)])
def test_scan_to_map_points_non_finite_pose_raises(pose):
    with pytest.raises(ValueError):
        scan_to_map_points([(1.0, 0.0)], pose)


# --- validate_goal: celdas de datos no numericas / no finitas (regresion) -----------

def test_goal_non_numeric_data_raises():
    with pytest.raises(ValueError):
        validate_goal(0.0, 0.0, 0.0, make_map(['a'] * 16))


@pytest.mark.parametrize('bad', [NAN, INF, -INF])
def test_goal_non_finite_cell_at_goal_raises(bad):
    data = [0] * 16
    data[10] = bad  # goal (0, 0) -> col 2, row 2 -> index 10
    with pytest.raises(ValueError):
        validate_goal(0.0, 0.0, 0.0, make_map(data))


@pytest.mark.parametrize('bad', [NAN, INF])
def test_goal_non_finite_cell_elsewhere_is_ignored(bad):
    data = [0] * 16
    data[0] = bad
    assert validate_goal(0.0, 0.0, 0.0, make_map(data)) == (0.0, 0.0, 0.0)


# --- Phase 1: wrong-type inputs always raise ValueError (never TypeError) -----------

MAP_OK = {'width': 4, 'height': 4, 'resolution': 1.0, 'origin_x': 0.0, 'origin_y': 0.0}


@pytest.mark.parametrize('x', [True, '1', None])
def test_validate_goal_wrong_type_x_raises_value_error(x):
    with pytest.raises(ValueError):
        validate_goal(x, 1.0, 0.0, MAP_OK)


@pytest.mark.parametrize('bad', [True, '1', None])
def test_validate_goal_wrong_type_y_yaw_raises_value_error(bad):
    with pytest.raises(ValueError):
        validate_goal(1.0, bad, 0.0, MAP_OK)
    with pytest.raises(ValueError):
        validate_goal(1.0, 1.0, bad, MAP_OK)


@pytest.mark.parametrize('data', [5, 'abcd', {'a': 1}, b'abcd'])
def test_validate_goal_invalid_data_container_raises_value_error(data):
    with pytest.raises(ValueError):
        validate_goal(1.0, 1.0, 0.0, dict(MAP_OK, data=data))


@pytest.mark.parametrize('cell', [True, 'x', None])
def test_validate_goal_invalid_cell_raises_value_error(cell):
    data = [0] * 16
    data[1 * 4 + 1] = cell
    with pytest.raises(ValueError):
        validate_goal(1.5, 1.5, 0.0, dict(MAP_OK, data=data))


@pytest.mark.parametrize('make', [list, tuple])
def test_validate_goal_valid_sequences_and_ints_work(make):
    result = validate_goal(1, 2, 0, dict(MAP_OK, data=make([0] * 16)))
    assert result == (1.0, 2.0, 0.0)
    assert all(isinstance(v, float) for v in result)


def test_validate_goal_data_none_is_valid():
    assert validate_goal(1.5, 1.5, 0.5, dict(MAP_OK, data=None)) == (1.5, 1.5, 0.5)


def test_validate_goal_accepts_array_like_data():
    array = pytest.importorskip('array').array('b', [0] * 16)
    assert validate_goal(1.5, 1.5, 0.0, dict(MAP_OK, data=array)) == (1.5, 1.5, 0.0)


def test_validate_goal_occupied_cell_still_rejected():
    data = [0] * 16
    data[1 * 4 + 1] = 100
    with pytest.raises(ValueError):
        validate_goal(1.5, 1.5, 0.0, dict(MAP_OK, data=data))


@pytest.mark.parametrize('args', [
    (None, 0.0, 1.0, 1.0), ('1', 0.0, 1.0, 1.0), (True, 0.0, 1.0, 1.0),
    (0.0, None, 1.0, 1.0), (0.0, 'a', 1.0, 1.0), (0.0, True, 1.0, 1.0),
    (0.0, 0.0, None, 1.0), (0.0, 0.0, 1.0, 'a'), (0.0, 0.0, True, 1.0),
])
def test_clamp_twist_wrong_type_raises_value_error(args):
    with pytest.raises(ValueError):
        clamp_twist(*args)


def test_clamp_twist_valid_int_and_float_still_work():
    assert clamp_twist(5, -5, 1, 2) == (1, -2)
    assert clamp_twist(0.1, 0.2, 0.5, 0.5) == (0.1, 0.2)


@pytest.mark.parametrize('ranges', [None, 5])
def test_downsample_scan_non_iterable_ranges_raises_value_error(ranges):
    with pytest.raises(ValueError):
        downsample_scan(ranges, 0.0, 0.1, 10)


@pytest.mark.parametrize('ranges', [['a', 1.0], [None, 1.0], [True, 1.0]])
def test_downsample_scan_non_numeric_elements_raise_value_error(ranges):
    with pytest.raises(ValueError):
        downsample_scan(ranges, 0.0, 0.1, 10)


@pytest.mark.parametrize('max_points', [2.5, True, 0, -1, None])
def test_downsample_scan_bad_max_points_raises_value_error(max_points):
    with pytest.raises(ValueError):
        downsample_scan([1.0, 2.0], 0.0, 0.1, max_points)


@pytest.mark.parametrize('angle_min', [None, 'a', True, NAN])
def test_downsample_scan_bad_angle_min_raises_value_error(angle_min):
    with pytest.raises(ValueError):
        downsample_scan([1.0], angle_min, 0.1, 10)


def test_downsample_scan_valid_inputs_still_work():
    out = downsample_scan((1, 2.0, INF, 0.01), -1, 0.5, 10)
    assert out == [(1.0, -1.0), (2.0, -0.5)]


@pytest.mark.parametrize('data', [None, 5])
def test_occupancy_non_sequence_data_raises_value_error(data):
    with pytest.raises(ValueError):
        occupancy_to_bytes(data, 1, 1)


@pytest.mark.parametrize('width', ['3', True, None, 2.0])
def test_occupancy_bad_width_type_raises_value_error(width):
    with pytest.raises(ValueError):
        occupancy_to_bytes([0] * 3, width, 1)


def test_occupancy_valid_inputs_tuple_and_array():
    from array import array
    assert zlib.decompress(occupancy_to_bytes((0, 100), 2, 1)) == bytes([0, 100])
    assert zlib.decompress(occupancy_to_bytes(array('b', [-1, 0]), 1, 2)) == bytes([255, 0])


# --- hardening: validate_goal extreme coordinates -----------------------------------

_GOAL_MAP = {'width': 10, 'height': 10, 'resolution': 0.05,
             'origin_x': 0.0, 'origin_y': 0.0}


@pytest.mark.parametrize('huge', [1e308, -1e308, 1e300])
@pytest.mark.parametrize('axis', ['x', 'y'])
def test_validate_goal_huge_coordinate_raises_value_error(huge, axis):
    coords = {'x': 0.1, 'y': 0.1}
    coords[axis] = huge
    with pytest.raises(ValueError):
        validate_goal(coords['x'], coords['y'], 0.0, dict(_GOAL_MAP))


@pytest.mark.parametrize('huge', [1e308, -1e308, 1e300])
@pytest.mark.parametrize('axis', ['x', 'y'])
def test_validate_goal_tiny_resolution_huge_coordinate_raises_value_error(huge, axis):
    info = dict(_GOAL_MAP, resolution=1e-300)
    coords = {'x': 0.0, 'y': 0.0}
    coords[axis] = huge
    with pytest.raises(ValueError):
        validate_goal(coords['x'], coords['y'], 0.0, info)


def test_validate_goal_valid_goal_unchanged():
    assert validate_goal(0.1, 0.2, 0.5, dict(_GOAL_MAP)) == (0.1, 0.2, 0.5)


# --- hardening: quaternion helpers --------------------------------------------------

_BAD_NUMBERS = [None, 'a', True, NAN, INF]


@pytest.mark.parametrize('bad', _BAD_NUMBERS)
@pytest.mark.parametrize('index', [0, 1, 2, 3])
def test_yaw_from_quaternion_bad_component_raises_value_error(bad, index):
    comps = [0.0, 0.0, 0.0, 1.0]
    comps[index] = bad
    with pytest.raises(ValueError):
        yaw_from_quaternion(*comps)


@pytest.mark.parametrize('bad', _BAD_NUMBERS)
def test_quaternion_from_yaw_bad_yaw_raises_value_error(bad):
    with pytest.raises(ValueError):
        quaternion_from_yaw(bad)


def test_quaternion_roundtrip_valid_unchanged():
    x, y, z, w = quaternion_from_yaw(1.0)
    assert yaw_from_quaternion(x, y, z, w) == pytest.approx(1.0)


# --- hardening: scan_to_map_points --------------------------------------------------

@pytest.mark.parametrize('pose', [None, (0.0, 0.0), (0.0, 0.0, 0.0, 0.0),
                                  (True, 0.0, 0.0), (0.0, 'a', 0.0), (0.0, 0.0, True),
                                  ('1', '2', '3'), 5])
def test_scan_to_map_points_bad_pose_raises_value_error(pose):
    with pytest.raises(ValueError):
        scan_to_map_points([(1.0, 0.0)], pose)


@pytest.mark.parametrize('points', [None, 5, 1.5, True])
def test_scan_to_map_points_non_iterable_points_raises_value_error(points):
    with pytest.raises(ValueError):
        scan_to_map_points(points, (0.0, 0.0, 0.0))


@pytest.mark.parametrize('point', [(1.0,), (1.0, 0.0, 0.0), 5, None])
def test_scan_to_map_points_bad_point_shape_raises_value_error(point):
    with pytest.raises(ValueError):
        scan_to_map_points([point], (0.0, 0.0, 0.0))


@pytest.mark.parametrize('bad', [None, True, 'a', NAN, INF])
def test_scan_to_map_points_bad_range_raises_value_error(bad):
    with pytest.raises(ValueError):
        scan_to_map_points([(bad, 0.0)], (0.0, 0.0, 0.0))


@pytest.mark.parametrize('bad', ['a', NAN, None, True, INF])
def test_scan_to_map_points_bad_angle_raises_value_error(bad):
    with pytest.raises(ValueError):
        scan_to_map_points([(1.0, bad)], (0.0, 0.0, 0.0))


def test_scan_to_map_points_valid_input_unchanged():
    out = scan_to_map_points([(1.0, 0.0), (2.0, math.pi / 2)], (1.0, 1.0, 0.0))
    assert out == [2.0, 1.0, 1.0, 3.0]
    assert scan_to_map_points([], (0.0, 0.0, 0.0)) == []


# --- hardening: simplify_path -------------------------------------------------------

@pytest.mark.parametrize('max_points', [None, 'a', True, 1.5, 0, 1, -1])
def test_simplify_path_bad_max_points_raises_value_error(max_points):
    with pytest.raises(ValueError):
        simplify_path([[0, 0], [1, 1]], max_points)


@pytest.mark.parametrize('points', [None, 5, 1.5])
def test_simplify_path_non_iterable_points_raises_value_error(points):
    with pytest.raises(ValueError):
        simplify_path(points, 5)


@pytest.mark.parametrize('point', [None, [1], ['a', 'b'], 5])
def test_simplify_path_bad_point_raises_value_error(point):
    with pytest.raises(ValueError):
        simplify_path([[0, 0], point], 5)


def test_simplify_path_valid_input_unchanged():
    pts = [[float(i), 0.0] for i in range(10)]
    assert simplify_path(pts, 20) == pts
    out = simplify_path(pts, 3)
    assert out == [[0.0, 0.0], [4.0, 0.0], [9.0, 0.0]]
    assert simplify_path([], 2) == []


# --- regression: valid inputs of other helpers --------------------------------------

def test_regression_occupancy_to_bytes_valid():
    assert zlib.decompress(occupancy_to_bytes([-1, 0, 50, 100], 4, 1)) == bytes(
        [255, 0, 50, 100])


def test_regression_downsample_scan_valid():
    out = downsample_scan([1.0, INF, 0.01, 2.0, 20.0, 3.0], 0.0, 0.5, 10)
    assert out == [(1.0, 0.0), (2.0, 1.5), (3.0, 2.5)]
    assert len(downsample_scan([1.0] * 100, 0.0, 0.01, 10)) == 10


def test_regression_clamp_twist_valid():
    assert clamp_twist(5.0, -5.0, 1.0, 2.0) == (1.0, -2.0)
    assert clamp_twist(0.5, 0.5, 1.0, 2.0) == (0.5, 0.5)


# --- binary map: fast path and version packing (phase 2, I-1/I-2) -----------------------

def test_occupancy_array_int8_fast_path_matches_list():
    from array import array
    values = [-1, 0, 100, 50, -1, 0]
    assert (occupancy_to_bytes(array('b', values), 3, 2)
            == occupancy_to_bytes(values, 3, 2))


def test_occupancy_array_int8_decompresses_to_twos_complement():
    from array import array
    raw = zlib.decompress(occupancy_to_bytes(array('b', [-1, 0, 100, 0]), 2, 2))
    assert list(raw) == [0xFF, 0, 100, 0]


def test_occupancy_array_wrong_size_raises():
    from array import array
    with pytest.raises(ValueError):
        occupancy_to_bytes(array('b', [0, 0, 0]), 2, 2)


@pytest.mark.parametrize('version', [0, 1, 2 ** 32 - 1])
def test_pack_map_binary_version_is_uint32_little_endian(version):
    import struct
    from mtrb_bringup.web_utils import pack_map_binary
    packed = pack_map_binary(version, b'xyz')
    assert struct.unpack('<I', packed[:4])[0] == version
    assert packed[4:] == b'xyz'


def test_pack_map_binary_version_wraps_modulo_2_32():
    import struct
    from mtrb_bringup.web_utils import pack_map_binary
    assert struct.unpack('<I', pack_map_binary(2 ** 32, b'')[:4])[0] == 0


@pytest.mark.parametrize('version', [-1, True, False, 1.0, '1', None])
def test_pack_map_binary_rejects_invalid_version(version):
    from mtrb_bringup.web_utils import pack_map_binary
    with pytest.raises(ValueError):
        pack_map_binary(version, b'')
