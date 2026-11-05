"""Unit tests for mtrb_bringup.web_utils (pure Python, no ROS needed)."""

import math
import os
import types
import zlib

from mtrb_bringup import web_utils
from mtrb_bringup.web_utils import (clamp_twist, downsample_scan, existing_map_files,
                                    is_reserved_map_name, list_maps, listable_map_name,
                                    map_image_saved, MAP_PATH_PATTERN, MAP_SCAN_MAX_ENTRIES,
                                    MapPathError, occupancy_to_bytes, posegraph_saved,
                                    quaternion_from_yaw, ReservedMapNameError,
                                    resolve_map_base, safe_map_dir, scan_to_map_points,
                                    simplify_path, valid_map_name, validate_goal,
                                    yaw_from_quaternion)
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
                                  'a/b', 'a.b', 'ñandú', ' a', 'a ', '-x', '-', '--help',
                                  '.', '..', 'ñ'])
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


# --- phase 4: huge integers (10**400 overflow float()) ---------------------------------------
# Every helper must refuse them with ValueError (or drop the beam) and never leak an
# OverflowError, which would become an HTTP 500 or kill a ROS callback.

HUGE = 10 ** 400


@pytest.mark.parametrize('huge', [HUGE, -HUGE])
def test_clamp_twist_huge_velocity_raises_value_error(huge):
    with pytest.raises(ValueError):
        clamp_twist(huge, 0.0, 0.2, 0.8)
    with pytest.raises(ValueError):
        clamp_twist(0.0, huge, 0.2, 0.8)


def test_clamp_twist_huge_limit_raises_value_error():
    with pytest.raises(ValueError):
        clamp_twist(0.1, 0.1, HUGE, 0.8)
    with pytest.raises(ValueError):
        clamp_twist(0.1, 0.1, 0.2, HUGE)


def test_clamp_twist_still_saturates_normal_values():
    assert clamp_twist(9.0, -9.0, 0.2, 0.8) == (0.2, -0.8)


MAP_INFO = {'width': 2, 'height': 2, 'resolution': 1.0, 'origin_x': 0.0, 'origin_y': 0.0,
            'data': [0, 0, 0, 0]}


@pytest.mark.parametrize('coords', [(HUGE, 0.5, 0.0), (0.5, HUGE, 0.0), (0.5, 0.5, HUGE),
                                    (-HUGE, 0.5, 0.0)])
def test_validate_goal_huge_x_y_yaw_raises_value_error(coords):
    with pytest.raises(ValueError):
        validate_goal(*coords, MAP_INFO)


@pytest.mark.parametrize('field', ['width', 'height', 'resolution', 'origin_x', 'origin_y'])
def test_validate_goal_huge_map_info_raises_value_error(field):
    info = dict(MAP_INFO, **{field: HUGE})
    with pytest.raises(ValueError):
        validate_goal(0.5, 0.5, 0.0, info)


def test_validate_goal_huge_cell_value_raises_value_error():
    info = dict(MAP_INFO, data=[0, 0, HUGE, 0])
    with pytest.raises(ValueError):
        validate_goal(0.5, 1.5, 0.0, info)       # cell index 2


def test_validate_goal_huge_cell_elsewhere_is_ignored():
    info = dict(MAP_INFO, data=[0, 0, HUGE, 0])
    assert validate_goal(0.5, 0.5, 0.0, info) == (0.5, 0.5, 0.0)


def test_downsample_scan_drops_a_huge_integer_range():
    result = downsample_scan([1.0, HUGE, -HUGE, 2.0], 0.0, 0.5, 10)
    assert [r for r, _ in result] == [1.0, 2.0]
    assert result[1][1] == pytest.approx(1.5)    # the angle keeps the original beam index


@pytest.mark.parametrize('field', ['angle_min', 'angle_increment', 'r_min', 'r_max'])
def test_downsample_scan_huge_parameter_raises_value_error(field):
    kwargs = {'angle_min': 0.0, 'angle_increment': 0.1, 'r_min': 0.15, 'r_max': 12.0}
    kwargs[field] = HUGE
    with pytest.raises(ValueError):
        downsample_scan([1.0], kwargs['angle_min'], kwargs['angle_increment'], 10,
                        r_min=kwargs['r_min'], r_max=kwargs['r_max'])


def test_scan_to_map_points_huge_range_or_angle_raises_value_error():
    with pytest.raises(ValueError):
        scan_to_map_points([(HUGE, 0.0)], (0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        scan_to_map_points([(1.0, HUGE)], (0.0, 0.0, 0.0))


@pytest.mark.parametrize('pose', [(HUGE, 0.0, 0.0), (0.0, HUGE, 0.0), (0.0, 0.0, HUGE)])
def test_scan_to_map_points_huge_pose_raises_value_error(pose):
    with pytest.raises(ValueError):
        scan_to_map_points([(1.0, 0.0)], pose)


@pytest.mark.parametrize('point', [[HUGE, 0.0], [0.0, HUGE], [-HUGE, 1.0]])
def test_simplify_path_huge_coordinate_raises_value_error(point):
    with pytest.raises(ValueError):
        simplify_path([[0.0, 0.0], point, [2.0, 2.0]], 10)


def test_simplify_path_huge_coordinate_in_a_decimated_path_raises_value_error():
    points = [[float(i), 0.0] for i in range(10)]
    points[5] = [HUGE, 0.0]
    with pytest.raises(ValueError):
        simplify_path(points, 3)


@pytest.mark.parametrize('width,height', [(HUGE, 2), (2, HUGE)])
def test_occupancy_huge_size_raises_value_error(width, height):
    with pytest.raises(ValueError):
        occupancy_to_bytes([0, 0, 0, 0], width, height)


def test_occupancy_huge_cell_value_never_leaks_an_overflow_error():
    # Python ints do not overflow on `& 0xFF`, so the cell wraps like any out-of-range int8
    # (existing behaviour); what must never happen is an OverflowError (or any other type).
    try:
        packed = occupancy_to_bytes([0, HUGE, 0, 0], 2, 2)
    except ValueError:
        return
    assert len(zlib.decompress(packed)) == 4


@pytest.mark.parametrize('yaw', [HUGE, -HUGE])
def test_quaternion_from_yaw_huge_raises_value_error(yaw):
    with pytest.raises(ValueError):
        quaternion_from_yaw(yaw)


@pytest.mark.parametrize('index', range(4))
def test_yaw_from_quaternion_huge_component_raises_value_error(index):
    components = [0.0, 0.0, 0.0, 1.0]
    components[index] = HUGE
    with pytest.raises(ValueError):
        yaw_from_quaternion(*components)


# --- Phase 5: saved-map path helpers -------------------------------------------------------
# Everything below uses real files in pytest's tmp_path (no mocks of the filesystem), except
# the 10000-entries bound of list_maps, which uses a lazy fake scandir.

@pytest.fixture
def root(tmp_path):
    """Return a real directory whose path slam_toolbox accepts (else skip: test is moot)."""
    real = os.path.realpath(str(tmp_path))
    if MAP_PATH_PATTERN.fullmatch(real) is None:
        pytest.skip(f'tmp_path has characters outside [A-Za-z0-9_./-]: {real}')
    return real


def _touch(path, content=b'x'):
    with open(path, 'wb') as handle:
        handle.write(content)


def _make_map(directory, name, image='.pgm'):
    _touch(os.path.join(directory, name + '.yaml'))
    _touch(os.path.join(directory, name + image))


# safe_map_dir

def test_safe_map_dir_returns_the_real_path_of_an_existing_directory(root):
    assert safe_map_dir(root) == root


@pytest.mark.parametrize('bad', [None, '', 5, 1.5, True, [], b'/tmp', 'maps', './maps',
                                 '../maps', 'a/b'])
def test_safe_map_dir_rejects_non_absolute_or_non_string(bad):
    with pytest.raises(MapPathError):
        safe_map_dir(bad)


def test_safe_map_dir_rejects_a_missing_directory(root):
    with pytest.raises(MapPathError):
        safe_map_dir(os.path.join(root, 'missing'))


def test_safe_map_dir_rejects_a_file(root):
    _touch(os.path.join(root, 'plain'))
    with pytest.raises(MapPathError):
        safe_map_dir(os.path.join(root, 'plain'))


def test_safe_map_dir_rejects_the_filesystem_root():
    with pytest.raises(MapPathError):
        safe_map_dir('/')


def test_safe_map_dir_rejects_a_root_reached_through_dots():
    with pytest.raises(MapPathError):
        safe_map_dir('/..')


@pytest.mark.parametrize('dirname', [
    'with space', 'semi;colon', 'dollar$x', 'quote"d', "single'q", 'back`tick', 'pipe|x',
    'amp&x', 'ñandú', 'tab\there', 'new\nline', 'star*', 'paren(x)', 'plus+x', 'eq=x'])
def test_safe_map_dir_rejects_characters_outside_the_slam_toolbox_set(root, dirname):
    path = os.path.join(root, dirname)
    os.mkdir(path)
    with pytest.raises(MapPathError):
        safe_map_dir(path)


def test_safe_map_dir_accepts_the_whole_allowed_character_set(root):
    path = os.path.join(root, 'aZ09_-.x')
    os.mkdir(path)
    assert safe_map_dir(path) == path


def test_safe_map_dir_resolves_dot_dot_so_the_result_has_no_dots(root):
    os.mkdir(os.path.join(root, 'a'))
    os.mkdir(os.path.join(root, 'b'))
    result = safe_map_dir(os.path.join(root, 'a', '..', 'b'))
    assert result == os.path.join(root, 'b')
    assert '..' not in result.split(os.sep)


def test_safe_map_dir_collapses_double_slashes(root):
    os.mkdir(os.path.join(root, 'sub'))
    result = safe_map_dir(root + '//sub//')
    assert result == os.path.join(root, 'sub')
    assert '//' not in result


def test_safe_map_dir_returns_the_target_of_a_symlinked_directory(root):
    os.mkdir(os.path.join(root, 'real'))
    os.symlink(os.path.join(root, 'real'), os.path.join(root, 'link'))
    assert safe_map_dir(os.path.join(root, 'link')) == os.path.join(root, 'real')


def test_safe_map_dir_rejects_a_symlink_whose_target_has_bad_characters(root):
    os.mkdir(os.path.join(root, 'bad dir'))
    os.symlink(os.path.join(root, 'bad dir'), os.path.join(root, 'link'))
    with pytest.raises(MapPathError):
        safe_map_dir(os.path.join(root, 'link'))


def test_safe_map_dir_rejects_a_dangling_symlink(root):
    os.symlink(os.path.join(root, 'nowhere'), os.path.join(root, 'link'))
    with pytest.raises(MapPathError):
        safe_map_dir(os.path.join(root, 'link'))


def test_map_path_error_is_a_value_error_but_not_every_value_error_is_one():
    assert issubclass(MapPathError, ValueError)
    with pytest.raises(ValueError) as caught:
        valid_map_name('bad name')
    assert not isinstance(caught.value, MapPathError)


# resolve_map_base

def test_resolve_map_base_returns_an_absolute_path_inside_the_real_directory(root):
    base = resolve_map_base(root, 'lab')
    assert base == os.path.join(root, 'lab')
    assert os.path.isabs(base)
    assert MAP_PATH_PATTERN.fullmatch(base) is not None


def test_resolve_map_base_follows_a_symlinked_map_dir_to_the_real_one(root):
    os.mkdir(os.path.join(root, 'real'))
    os.symlink(os.path.join(root, 'real'), os.path.join(root, 'link'))
    assert resolve_map_base(os.path.join(root, 'link'), 'lab') == os.path.join(
        root, 'real', 'lab')


@pytest.mark.parametrize('name', [
    '../x', '..', '.', 'a/b', '/abs', '', ' ', 'a b', 'a.b', 'x' * 41, 'ñ', 'a\x00b', 'a\n',
    None, 5, True, ['a'], b'abc'])
def test_resolve_map_base_rejects_invalid_names_with_a_plain_value_error(root, name):
    with pytest.raises(ValueError) as caught:
        resolve_map_base(root, name)
    assert not isinstance(caught.value, MapPathError)


@pytest.mark.parametrize('name', ['_', '0', 'x' * 40, 'A-b_9', '_-x', 'a-'])
def test_resolve_map_base_valid_names_stay_absolute_and_confined(root, name):
    base = resolve_map_base(root, name)
    assert base.startswith('/')
    assert os.path.dirname(base) == root
    assert not os.path.basename(base).startswith('-')


@pytest.mark.parametrize('name', ['-h', '--help', '-', '-x', '-rf', '--', '-' + 'a' * 39])
def test_resolve_map_base_rejects_names_starting_with_a_dash(root, name):
    # A leading '-' could be read as an option by a shell: the name is refused outright
    # (plain ValueError, not a reserved name) and no path is ever built from it.
    with pytest.raises(ValueError) as caught:
        resolve_map_base(root, name)
    assert not isinstance(caught.value, (MapPathError, ReservedMapNameError))
    with pytest.raises(ValueError):
        valid_map_name(name)


@pytest.mark.parametrize('name', ['a', 'lab_1', 'x', '_x', '9-9', 'z' * 40, 'A-b_9'])
def test_no_valid_map_name_yields_a_path_component_starting_with_a_dash(root, name):
    assert not os.path.basename(resolve_map_base(root, name)).startswith('-')


@pytest.mark.parametrize('name', ['map', 'MAP', 'Map', 'mAp', 'mAP'])
def test_resolve_map_base_and_valid_map_name_reject_reserved_names_case_insensitively(
        root, name):
    for call in (lambda: valid_map_name(name), lambda: resolve_map_base(root, name)):
        with pytest.raises(ReservedMapNameError) as caught:
            call()
        assert isinstance(caught.value, ValueError)
    assert is_reserved_map_name(name)
    assert not listable_map_name(name)


@pytest.mark.parametrize('name', ['maps', 'map1', 'my_map', 'map-', '_map', 'mapa'])
def test_names_merely_containing_map_are_not_reserved(root, name):
    assert valid_map_name(name) == name
    assert not is_reserved_map_name(name)
    assert listable_map_name(name)


def test_reserved_name_error_is_a_value_error_but_other_invalid_names_are_not_reserved():
    assert issubclass(ReservedMapNameError, ValueError)
    with pytest.raises(ValueError) as caught:
        valid_map_name('-x')
    assert not isinstance(caught.value, ReservedMapNameError)


@pytest.mark.parametrize('name', ['', 'x' * 41, '.', '..', 'a/b', 'ñ', '-', '--help'])
def test_listable_map_name_is_false_for_invalid_names(name):
    assert not listable_map_name(name)


@pytest.mark.parametrize('bad', [None, 5, b'map', ['map']])
def test_is_reserved_and_listable_are_false_for_non_strings(bad):
    assert not is_reserved_map_name(bad)
    assert not listable_map_name(bad)


@pytest.mark.parametrize('bad_dir', [None, '', 'relative', '/nonexistent-dir-for-tests'])
def test_resolve_map_base_rejects_an_unusable_directory(bad_dir):
    with pytest.raises(MapPathError):
        resolve_map_base(bad_dir, 'lab')


@pytest.mark.parametrize('ext', ['.yaml', '.pgm', '.png', '.posegraph', '.data',
                                 '.yaml.bak', '.pgm.bak', '.png.bak', '.posegraph.bak',
                                 '.data.bak'])
def test_resolve_map_base_refuses_a_symlink_among_the_files(root, ext):
    _touch(os.path.join(root, 'elsewhere'))
    os.symlink(os.path.join(root, 'elsewhere'), os.path.join(root, 'lab' + ext))
    with pytest.raises(MapPathError):
        resolve_map_base(root, 'lab')


def test_resolve_map_base_refuses_a_dangling_symlink(root):
    os.symlink(os.path.join(root, 'nowhere'), os.path.join(root, 'lab.yaml'))
    with pytest.raises(MapPathError):
        resolve_map_base(root, 'lab')


def test_resolve_map_base_refuses_a_symlink_pointing_inside_the_same_directory(root):
    _touch(os.path.join(root, 'other.yaml'))
    os.symlink(os.path.join(root, 'other.yaml'), os.path.join(root, 'lab.yaml'))
    with pytest.raises(MapPathError):
        resolve_map_base(root, 'lab')


@pytest.mark.parametrize('ext', ['.yaml', '.pgm', '.data', '.yaml.bak', '.posegraph.bak'])
def test_resolve_map_base_refuses_a_directory_instead_of_a_file(root, ext):
    os.mkdir(os.path.join(root, 'lab' + ext))
    with pytest.raises(MapPathError):
        resolve_map_base(root, 'lab')


def test_resolve_map_base_accepts_existing_plain_files_and_backups(root):
    for ext in ('.yaml', '.pgm', '.posegraph', '.data', '.yaml.bak', '.pgm.bak'):
        _touch(os.path.join(root, 'lab' + ext))
    assert resolve_map_base(root, 'lab') == os.path.join(root, 'lab')


def test_resolve_map_base_ignores_symlinks_of_other_maps(root):
    _touch(os.path.join(root, 'elsewhere'))
    os.symlink(os.path.join(root, 'elsewhere'), os.path.join(root, 'other.yaml'))
    assert resolve_map_base(root, 'lab') == os.path.join(root, 'lab')


def test_resolve_map_base_does_not_create_anything(root):
    resolve_map_base(root, 'lab')
    assert os.listdir(root) == []


# existing_map_files

def test_existing_map_files_lists_every_related_file_and_nothing_else(root):
    base = os.path.join(root, 'lab')
    for ext in ('.yaml', '.pgm', '.png', '.posegraph', '.data', '.yaml.bak', '.txt'):
        _touch(base + ext)
    _touch(os.path.join(root, 'lab2.yaml'))
    assert existing_map_files(base) == [base + e for e in (
        '.yaml', '.pgm', '.png', '.posegraph', '.data')]


def test_existing_map_files_reports_a_lone_data_file(root):
    base = os.path.join(root, 'lab')
    _touch(base + '.data')
    assert existing_map_files(base) == [base + '.data']


def test_existing_map_files_is_empty_for_a_new_name(root):
    assert existing_map_files(os.path.join(root, 'new')) == []


def test_existing_map_files_includes_dangling_symlinks(root):
    base = os.path.join(root, 'lab')
    os.symlink(os.path.join(root, 'nowhere'), base + '.yaml')
    assert existing_map_files(base) == [base + '.yaml']


# map_image_saved / posegraph_saved

@pytest.mark.parametrize('image', ['.pgm', '.png'])
def test_map_image_saved_true_with_yaml_and_either_image(root, image):
    _make_map(root, 'lab', image)
    assert map_image_saved(os.path.join(root, 'lab')) is True


def test_map_image_saved_false_with_only_the_yaml(root):
    _touch(os.path.join(root, 'lab.yaml'))
    assert map_image_saved(os.path.join(root, 'lab')) is False


def test_map_image_saved_false_with_only_the_image(root):
    _touch(os.path.join(root, 'lab.pgm'))
    assert map_image_saved(os.path.join(root, 'lab')) is False


@pytest.mark.parametrize('empty', ['.yaml', '.pgm'])
def test_map_image_saved_false_when_a_file_is_empty(root, empty):
    _make_map(root, 'lab')
    _touch(os.path.join(root, 'lab' + empty), b'')
    assert map_image_saved(os.path.join(root, 'lab')) is False


def test_map_image_saved_false_for_a_symlinked_yaml(root):
    _touch(os.path.join(root, 'real.yaml'))
    _touch(os.path.join(root, 'lab.pgm'))
    os.symlink(os.path.join(root, 'real.yaml'), os.path.join(root, 'lab.yaml'))
    assert map_image_saved(os.path.join(root, 'lab')) is False


def test_map_image_saved_false_for_a_symlinked_image(root):
    _touch(os.path.join(root, 'lab.yaml'))
    _touch(os.path.join(root, 'real.pgm'))
    os.symlink(os.path.join(root, 'real.pgm'), os.path.join(root, 'lab.pgm'))
    assert map_image_saved(os.path.join(root, 'lab')) is False


def test_map_image_saved_false_for_a_directory_named_like_the_yaml(root):
    os.mkdir(os.path.join(root, 'lab.yaml'))
    _touch(os.path.join(root, 'lab.pgm'))
    assert map_image_saved(os.path.join(root, 'lab')) is False


def test_posegraph_saved_true_with_both_files(root):
    _touch(os.path.join(root, 'lab.posegraph'))
    _touch(os.path.join(root, 'lab.data'))
    assert posegraph_saved(os.path.join(root, 'lab')) is True


@pytest.mark.parametrize('present', ['.posegraph', '.data'])
def test_posegraph_saved_false_with_only_one_file(root, present):
    _touch(os.path.join(root, 'lab' + present))
    assert posegraph_saved(os.path.join(root, 'lab')) is False


def test_posegraph_saved_false_when_one_file_is_empty(root):
    _touch(os.path.join(root, 'lab.posegraph'))
    _touch(os.path.join(root, 'lab.data'), b'')
    assert posegraph_saved(os.path.join(root, 'lab')) is False


def test_posegraph_saved_false_for_a_symlink(root):
    _touch(os.path.join(root, 'real'))
    _touch(os.path.join(root, 'lab.data'))
    os.symlink(os.path.join(root, 'real'), os.path.join(root, 'lab.posegraph'))
    assert posegraph_saved(os.path.join(root, 'lab')) is False


# list_maps

def test_list_maps_returns_sorted_names_of_complete_maps(root):
    _make_map(root, 'zeta')
    _make_map(root, 'alpha', '.png')
    _make_map(root, 'Beta')
    assert list_maps(root) == ['Beta', 'alpha', 'zeta']


def test_list_maps_is_empty_for_a_missing_directory(root):
    assert list_maps(os.path.join(root, 'missing')) == []


@pytest.mark.parametrize('bad', [None, 5, b'/tmp', ['x']])
def test_list_maps_is_empty_for_a_non_string_directory(bad):
    assert list_maps(bad) == []


def test_list_maps_is_empty_for_an_empty_directory(root):
    assert list_maps(root) == []


def test_list_maps_returns_at_most_200_sorted_names(root):
    names = [f'm{i:03d}' for i in range(250)]
    for name in reversed(names):
        _make_map(root, name)
    result = list_maps(root)
    assert len(result) == web_utils.MAP_LIST_LIMIT == 200
    assert result == names[:200]


def test_list_maps_honours_an_explicit_limit(root):
    for name in ('c', 'a', 'b', 'd'):
        _make_map(root, name)
    assert list_maps(root, 2) == ['a', 'b']


def test_list_maps_ignores_yaml_without_image_and_image_without_yaml(root):
    _touch(os.path.join(root, 'onlyyaml.yaml'))
    _touch(os.path.join(root, 'onlyimage.pgm'))
    _make_map(root, 'good')
    assert list_maps(root) == ['good']


def test_list_maps_never_lists_reserved_or_dash_names_even_if_the_files_exist(root):
    for name in ('map', 'Map', 'MAP', '-x', '-', '--help'):
        _make_map(root, name)
    _make_map(root, 'lab')
    assert list_maps(root) == ['lab']


def test_list_maps_ignores_invalid_names(root):
    for name in ('with space', 'dots.in.name', 'x' * 41, 'ñandú', 'semi;colon'):
        _make_map(root, name)
    _make_map(root, 'x' * 40)
    assert list_maps(root) == ['x' * 40]


def test_list_maps_ignores_other_extensions(root):
    _touch(os.path.join(root, 'a.yml'))
    _touch(os.path.join(root, 'a.pgm'))
    _touch(os.path.join(root, 'b.YAML'))
    _touch(os.path.join(root, 'b.pgm'))
    assert list_maps(root) == []


def test_list_maps_ignores_symlinked_yaml_and_symlinked_images(root):
    _make_map(root, 'real')
    os.symlink(os.path.join(root, 'real.yaml'), os.path.join(root, 'linkyaml.yaml'))
    _touch(os.path.join(root, 'linkyaml.pgm'))
    _touch(os.path.join(root, 'linkimg.yaml'))
    os.symlink(os.path.join(root, 'real.pgm'), os.path.join(root, 'linkimg.pgm'))
    assert list_maps(root) == ['real']


def test_list_maps_ignores_a_directory_named_like_a_yaml(root):
    os.mkdir(os.path.join(root, 'dirmap.yaml'))
    _touch(os.path.join(root, 'dirmap.pgm'))
    assert list_maps(root) == []


def test_list_maps_ignores_a_map_whose_image_is_empty(root):
    # Spec: the image must be a regular, non-empty file (an empty yaml is not checked).
    _touch(os.path.join(root, 'empty.yaml'))
    _touch(os.path.join(root, 'empty.pgm'), b'')
    assert list_maps(root) == []


def test_list_maps_ignores_backups(root):
    _touch(os.path.join(root, 'old.yaml.bak'))
    _touch(os.path.join(root, 'old.pgm.bak'))
    assert list_maps(root) == []


def test_list_maps_refuses_a_relative_existing_directory(root, monkeypatch):
    os.mkdir(os.path.join(root, 'rel'))
    monkeypatch.chdir(root)
    with pytest.raises(MapPathError):
        list_maps('rel')


def test_list_maps_refuses_a_directory_with_bad_characters(root):
    path = os.path.join(root, 'bad dir')
    os.mkdir(path)
    with pytest.raises(MapPathError):
        list_maps(path)


def test_list_maps_never_inspects_more_than_10000_directory_entries(root, monkeypatch):
    _make_map(root, 'inside')    # entry index 9999: still inspected
    _make_map(root, 'outside')   # entry index 10000: must not be reached
    consumed = []

    def entry(name):
        return types.SimpleNamespace(name=name, is_file=lambda follow_symlinks=True: True,
                                     is_symlink=lambda: False)

    def entries():
        for index in range(MAP_SCAN_MAX_ENTRIES + 50):
            consumed.append(index)
            if index == MAP_SCAN_MAX_ENTRIES - 1:
                yield entry('inside.yaml')
            elif index == MAP_SCAN_MAX_ENTRIES:
                yield entry('outside.yaml')
            else:
                yield entry(f'junk{index}.txt')

    class FakeScandir:
        def __enter__(self):
            return entries()

        def __exit__(self, *exc):
            return False

    monkeypatch.setattr(web_utils.os, 'scandir', lambda path: FakeScandir())
    assert MAP_SCAN_MAX_ENTRIES == 10000
    assert list_maps(root) == ['inside']
    assert len(consumed) <= MAP_SCAN_MAX_ENTRIES + 1


# --- occupancy_to_bytes: list/tuple go through array('b') (audit, task 21) ----------------------

def _raw(packed):
    return zlib.decompress(packed)


def test_list_and_tuple_cells_equal_the_array_b_result_byte_for_byte():
    from array import array as typed
    values = [-1, 0, 100, 50, 1, 99]
    expected = occupancy_to_bytes(typed('b', values), 3, 2)
    assert occupancy_to_bytes(values, 3, 2) == expected
    assert occupancy_to_bytes(tuple(values), 3, 2) == expected
    assert _raw(expected) == typed('b', values).tobytes()


def test_cells_outside_int8_take_the_slow_path_masked_to_a_byte_without_raising():
    packed = occupancy_to_bytes([0, 300, -129, 128], 2, 2)
    assert _raw(packed) == bytes([0, 300 & 0xFF, (-129) & 0xFF, 128])


def test_array_b_input_is_used_as_raw_bytes_without_conversion():
    from array import array as typed
    data = typed('b', [-1, 0, 100, 0])
    assert _raw(occupancy_to_bytes(data, 2, 2)) == bytes([255, 0, 100, 0])


@pytest.mark.parametrize('cells', [['a', 0, 0, 0], [None, 0, 0, 0], [[1], 0, 0, 0],
                                   [float('nan'), 0, 0, 0], [float('inf'), 0, 0, 0]])
def test_unconvertible_cells_raise_value_error_never_another_exception(cells):
    with pytest.raises(ValueError):
        occupancy_to_bytes(cells, 2, 2)


@pytest.mark.parametrize('cells', [[True, False, True, False], [1.0, 2.0, 3.0, 4.0]])
def test_bool_and_integral_float_cells_keep_working_through_the_slow_path(cells):
    assert _raw(occupancy_to_bytes(cells, 2, 2)) == bytes(int(v) & 0xFF for v in cells)
