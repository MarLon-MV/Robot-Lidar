"""Integration check: every node the launch starts has a section in its YAML file."""

import os

import pytest

yaml = pytest.importorskip('yaml')

PARAM_DIR = os.path.join(os.path.dirname(__file__), '..', 'param')

# Node names used in launch/mtrb.launch.py.
EXPECTED = {
    'nav2.yaml': ['map_server', 'amcl', 'planner_server', 'controller_server',
                  'behavior_server', 'bt_navigator', 'velocity_smoother',
                  'global_costmap', 'local_costmap'],
    'ekf.yaml': ['ekf_filter_node'],
    'slam_toolbox.yaml': ['slam_toolbox'],
    'lidar.yaml': ['ldlidar_node'],
    'web.yaml': ['mtrb_web_bridge'],
}


@pytest.mark.parametrize('file_name,nodes', EXPECTED.items())
def test_param_file_has_a_section_per_node(file_name, nodes):
    with open(os.path.join(PARAM_DIR, file_name)) as f:
        data = yaml.safe_load(f)
    missing = [n for n in nodes if n not in data]
    assert not missing, f'{file_name} is missing sections: {missing}'


def test_lidar_matches_ld19_datasheet():
    with open(os.path.join(PARAM_DIR, 'lidar.yaml')) as f:
        params = yaml.safe_load(f)['ldlidar_node']['ros__parameters']
    assert params['product_name'] == 'LDLiDAR_LD19'
    assert params['port_baudrate'] == 230400
    assert params['range_max'] == 12.0


def _web_params(file_name):
    with open(os.path.join(PARAM_DIR, file_name)) as f:
        return yaml.safe_load(f)['mtrb_web_bridge']['ros__parameters']


def _flatten(d, prefix=''):
    keys = set()
    for k, v in d.items():
        path = f'{prefix}{k}'
        if isinstance(v, dict):
            keys |= _flatten(v, path + '.')
        else:
            keys.add(path)
    return keys


def test_web_token_is_the_placeholder():
    assert _web_params('web.yaml')['auth']['token'] == 'CHANGE-ME-example-token'


def test_web_and_example_have_same_keys():
    assert _flatten(_web_params('web.yaml')) == _flatten(_web_params('web.example.yaml'))


def test_web_teleop_limits_are_positive():
    teleop = _web_params('web.yaml')['teleop']
    assert teleop['max_vx'] > 0
    assert teleop['max_wz'] > 0


def test_web_max_ws_bytes_is_4096():
    assert _web_params('web.yaml')['limits']['max_ws_bytes'] == 4096


# --- Phase 1: static checks (no launch_ros import) ----------------------------------

def test_launch_web_params_file_argument_has_description():
    import os
    import re
    path = os.path.join(os.path.dirname(__file__), '..', 'launch', 'mtrb.launch.py')
    with open(path, encoding='utf-8') as handle:
        text = handle.read()
    match = re.search(r"arg\(\s*'web_params_file'(.*?)\)\s*,\s*\n", text, re.S)
    assert match is not None, 'web_params_file argument not declared'
    assert re.search(r"description\s*=\s*'[^']+'", match.group(1)), \
        'web_params_file has no description'


def test_requirements_web_includes_httpx():
    import os
    path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'requirements-web.txt')
    with open(path, encoding='utf-8') as handle:
        names = [line.strip().lower() for line in handle
                 if line.strip() and not line.startswith('#')]
    assert any(n.startswith('httpx') for n in names), f'httpx missing: {names}'
