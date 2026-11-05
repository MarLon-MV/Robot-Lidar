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


# --- Phase 5: rates.lifecycle, status.nodes, map_dir ---------------------------------------

import re  # noqa: E402, I100

PHASE_5_KEYS = ('rates.lifecycle', 'status.nodes', 'map_dir')
NODE_NAME = re.compile(r'[A-Za-z_][A-Za-z0-9_]{0,63}')
MAP_PATH = re.compile(r'[A-Za-z0-9_./-]+')
WEB_FILES = ['web.yaml', 'web.example.yaml']


@pytest.mark.parametrize('file_name', WEB_FILES)
def test_web_files_have_the_phase_5_keys(file_name):
    keys = _flatten(_web_params(file_name))
    assert set(PHASE_5_KEYS) <= keys


def test_web_files_have_the_same_phase_5_values_shape():
    web, example = _web_params('web.yaml'), _web_params('web.example.yaml')
    assert type(web['rates']['lifecycle']) is type(example['rates']['lifecycle'])
    assert isinstance(web['status']['nodes'], list)
    assert isinstance(example['status']['nodes'], list)
    assert isinstance(web['map_dir'], str) and isinstance(example['map_dir'], str)


@pytest.mark.parametrize('file_name', WEB_FILES)
def test_web_lifecycle_rate_is_within_the_node_limits(file_name):
    rate = _web_params(file_name)['rates']['lifecycle']
    assert isinstance(rate, (int, float)) and not isinstance(rate, bool)
    assert 0.05 <= rate <= 0.5


@pytest.mark.parametrize('file_name', WEB_FILES)
def test_web_status_nodes_are_at_most_16_unique_valid_names(file_name):
    nodes = _web_params(file_name)['status']['nodes']
    assert isinstance(nodes, list)
    assert len(nodes) <= 16
    assert len(set(nodes)) == len(nodes)
    assert all(isinstance(n, str) and NODE_NAME.fullmatch(n) for n in nodes), nodes


@pytest.mark.parametrize('file_name', WEB_FILES)
def test_web_status_nodes_are_nav2_nodes_of_nav2_yaml(file_name):
    with open(os.path.join(PARAM_DIR, 'nav2.yaml')) as f:
        nav2 = yaml.safe_load(f)
    nodes = _web_params(file_name)['status']['nodes']
    assert [n for n in nodes if n not in nav2] == []


def test_the_localization_nodes_added_by_mode_have_a_yaml_section():
    # The node adds map_server + amcl (mode map) or slam_toolbox (mode mapping) by itself.
    with open(os.path.join(PARAM_DIR, 'nav2.yaml')) as f:
        nav2 = yaml.safe_load(f)
    with open(os.path.join(PARAM_DIR, 'slam_toolbox.yaml')) as f:
        slam = yaml.safe_load(f)
    assert 'map_server' in nav2 and 'amcl' in nav2
    assert 'slam_toolbox' in slam


@pytest.mark.parametrize('file_name', WEB_FILES)
def test_web_map_dir_is_empty_or_a_safe_absolute_path(file_name):
    map_dir = _web_params(file_name)['map_dir']
    assert isinstance(map_dir, str)
    if map_dir:
        assert os.path.isabs(map_dir) and MAP_PATH.fullmatch(map_dir), map_dir


# --- Security fixes of phase 5: map quotas and allow_example_token --------------------------

SECURITY_KEYS = ('maps.min_free_mb', 'maps.min_interval_s', 'auth.allow_example_token')
# YAML numbers the node declares with dynamic typing (int or float both accepted).
DYNAMIC_NUMERIC_TYPES = (int, float)


@pytest.mark.parametrize('file_name', WEB_FILES)
def test_web_files_have_the_security_keys(file_name):
    assert set(SECURITY_KEYS) <= _flatten(_web_params(file_name))


@pytest.mark.parametrize('file_name', WEB_FILES)
def test_web_allow_example_token_is_false_and_a_real_bool(file_name):
    value = _web_params(file_name)['auth']['allow_example_token']
    assert value is False


@pytest.mark.parametrize('file_name', WEB_FILES)
@pytest.mark.parametrize('key', ['min_free_mb', 'min_interval_s'])
def test_web_map_quotas_are_non_negative_finite_numbers(file_name, key):
    import math
    value = _web_params(file_name)['maps'][key]
    assert isinstance(value, DYNAMIC_NUMERIC_TYPES) and not isinstance(value, bool)
    assert math.isfinite(value) and value >= 0


def test_web_files_have_the_same_security_values():
    web, example = _web_params('web.yaml'), _web_params('web.example.yaml')
    assert web['auth']['allow_example_token'] is example['auth']['allow_example_token'] is False
    assert web['maps'] == example['maps']
    assert web['auth']['token'] == example['auth']['token']
    assert web['host'] == example['host']


@pytest.mark.parametrize('file_name', WEB_FILES)
def test_web_map_quota_defaults_are_the_documented_ones(file_name):
    maps = _web_params(file_name)['maps']
    assert maps['min_free_mb'] == 200 and maps['min_interval_s'] == 5.0
