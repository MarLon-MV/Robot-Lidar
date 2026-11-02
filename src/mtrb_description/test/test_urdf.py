"""Render mtrb.urdf.xacro and check the frames it produces."""

import os
import xml.etree.ElementTree as ET

import pytest

xacro = pytest.importorskip('xacro')

URDF = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'mtrb.urdf.xacro')

# Frames of the Create 3 model (same names the robot publishes on /r2/tf_static).
CREATE3_LINKS = {
    'base_link', 'bumper', 'front_caster_link', 'imu_link', 'optical_mouse', 'ir_omni',
    'left_wheel', 'right_wheel', 'wheel_drop_left', 'wheel_drop_right',
    'bump_front_center', 'bump_front_left', 'bump_front_right', 'bump_left', 'bump_right',
    'button_1', 'button_2', 'button_power',
    'cliff_front_left', 'cliff_front_right', 'cliff_side_left', 'cliff_side_right',
    'ir_intensity_front_center_left', 'ir_intensity_front_center_right',
    'ir_intensity_front_left', 'ir_intensity_front_right', 'ir_intensity_left',
    'ir_intensity_right', 'ir_intensity_side_left',
}


def render(**mappings):
    xml = xacro.process_file(URDF, mappings={k: str(v) for k, v in mappings.items()}).toxml()
    return ET.fromstring(xml)


def links(robot):
    return {link.get('name') for link in robot.findall('link')}


def test_lidar_only_has_base_link_and_lidar():
    robot = render(lidar_only='true', lidar_frame='laser', lidar_z=0.2)
    assert links(robot) == {'base_link', 'laser'}
    joint = robot.find("joint[@name='laser_joint']")
    assert joint.find('parent').get('link') == 'base_link'
    assert joint.find('origin').get('xyz').split()[2] == '0.2'


def test_lidar_frame_is_configurable():
    assert 'my_lidar' in links(render(lidar_only='true', lidar_frame='my_lidar'))


def test_full_model_has_every_create3_frame_and_no_simulation_tags():
    robot = render(lidar_only='false')
    assert links(robot) == CREATE3_LINKS | {'laser'}
    assert robot.findall('gazebo') == []
    assert robot.findall('ros2_control') == []


def test_lidar_frame_is_on_the_scan_plane_of_the_ld19():
    robot = render(lidar_only='true')
    joint = robot.find("joint[@name='laser_joint']")
    assert float(joint.find('origin').get('rpy').split()[2]) == 0.0
    base, head = robot.find("link[@name='laser']").findall('visual')
    # Bottom of the base is 24.5 mm below the frame (scan plane, approximate).
    base_z = float(base.find('origin').get('xyz').split()[2])
    assert abs(base_z - (0.016 / 2 - 0.0245)) < 1e-9
