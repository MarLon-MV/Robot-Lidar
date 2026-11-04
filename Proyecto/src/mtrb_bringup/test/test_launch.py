"""
Integration check: the launch file loads and declares its public arguments.

Needs the workspace built and sourced (it looks up the installed package share).
"""

import importlib.util
import os

import pytest

pytest.importorskip('launch_ros')

from ament_index_python.packages import (  # noqa: E402, I100
    get_package_share_directory, PackageNotFoundError)

REQUIRED_ARGS = {
    'mode', 'namespace', 'map', 'posegraph', 'navigation', 'use_ekf', 'scan_topic',
    'odom_topic', 'imu_topic', 'lidar_frame', 'dock_x', 'dock_y', 'dock_yaw',
    'lidar', 'lidar_port', 'lidar_params_file',
}


def _load_launch_module():
    try:
        share = get_package_share_directory('mtrb_bringup')
    except PackageNotFoundError:
        pytest.skip('mtrb_bringup is not installed (run colcon build first)')
    path = os.path.join(share, 'launch', 'mtrb.launch.py')
    spec = importlib.util.spec_from_file_location('mtrb_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_declares_public_arguments():
    description = _load_launch_module().generate_launch_description()
    declared = {a.name for a in description.get_launch_arguments()}
    assert REQUIRED_ARGS <= declared, f'missing: {REQUIRED_ARGS - declared}'
    assert 'rviz' not in declared
