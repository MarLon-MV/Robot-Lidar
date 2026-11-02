"""Unit tests for mtrb_bringup.utils (pure Python, no ROS needed)."""

import math
import os

from mtrb_bringup.utils import (absolute_path, absolute_topic, is_true, validate_mode,
                                validate_namespace, yaw_to_quaternion)
import pytest


@pytest.mark.parametrize('value', ['true', 'True', ' 1 ', 'yes', 'YES', True])
def test_is_true_accepts_truthy_strings(value):
    assert is_true(value)


@pytest.mark.parametrize('value', ['false', '0', 'no', '', 'on', False])
def test_is_true_rejects_everything_else(value):
    assert not is_true(value)


def test_absolute_topic_prefixes_namespace():
    assert absolute_topic('r2', 'scan') == '/r2/scan'
    assert absolute_topic('/r2/', 'scan') == '/r2/scan'


def test_absolute_topic_keeps_absolute_names():
    assert absolute_topic('r2', '/scan') == '/scan'


def test_absolute_path_expands_home():
    home = os.path.expanduser('~')
    assert absolute_path('~/maps/a.yaml') == os.path.join(home, 'maps', 'a.yaml')
    assert os.path.isabs(absolute_path('relative.yaml'))


@pytest.mark.parametrize('yaw', [0.0, math.pi / 2, -math.pi / 3, math.pi])
def test_yaw_to_quaternion_is_unit_and_round_trips(yaw):
    x, y, z, w = yaw_to_quaternion(yaw)
    assert (x, y) == (0.0, 0.0)
    assert math.isclose(x * x + y * y + z * z + w * w, 1.0)
    assert math.isclose(math.atan2(2 * w * z, 1 - 2 * z * z), yaw, abs_tol=1e-9) or \
        math.isclose(abs(yaw), math.pi)


def test_validate_mode():
    assert validate_mode(' MAP ') == 'map'
    assert validate_mode('mapping') == 'mapping'
    with pytest.raises(ValueError):
        validate_mode('slam')


def test_validate_namespace():
    assert validate_namespace('/r2/') == 'r2'
    with pytest.raises(ValueError):
        validate_namespace(' / ')
