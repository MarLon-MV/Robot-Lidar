"""Unit tests for the initial pose message (needs geometry_msgs from a ROS install)."""

import math

import pytest

pytest.importorskip('geometry_msgs')

from builtin_interfaces.msg import Time  # noqa: E402, I100
from mtrb_bringup.initial_pose_publisher import (  # noqa: E402, I100, I201
    build_initial_pose, COVARIANCE_XY, COVARIANCE_YAW)


def test_build_initial_pose():
    stamp = Time(sec=10, nanosec=5)
    msg = build_initial_pose(1.5, -2, math.pi / 2, 'map', stamp)

    assert msg.header.frame_id == 'map'
    assert msg.header.stamp == stamp
    assert (msg.pose.pose.position.x, msg.pose.pose.position.y) == (1.5, -2.0)
    q = msg.pose.pose.orientation
    assert math.isclose(q.z, math.sqrt(0.5)) and math.isclose(q.w, math.sqrt(0.5))
    assert msg.pose.covariance[0] == msg.pose.covariance[7] == COVARIANCE_XY
    assert msg.pose.covariance[35] == COVARIANCE_YAW
