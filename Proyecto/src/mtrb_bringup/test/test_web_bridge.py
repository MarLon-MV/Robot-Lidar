"""Unit tests for mtrb_web_bridge token/host/port selection (need rclpy; skipped without)."""

from array import array
import json
import logging
import math
import os
import pwd
import threading
from types import SimpleNamespace as NS
from unittest import mock

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('uvicorn')

from mtrb_bringup import mtrb_web_bridge as bridge  # noqa: E402, I100
from mtrb_bringup.web_server import (  # noqa: E402, I100
    BroadcastHub, StateStore, TOKEN_ENV_VAR)

SECRET = 'S3cr3t-token-value-0123456789'


class FakeParam:
    def __init__(self, value):
        self.value = value


def make_node(params=None):
    """Build a bridge without running Node.__init__ (no ROS context needed)."""
    node = object.__new__(bridge.MtrbWebBridge)
    values = {'auth.token_file': '/home/robot/.config/mtrb/web_token',
              'auth.allow_example_token': False, 'host': '127.0.0.1', 'port': 8080}
    values.update(params or {})
    node.get_parameter = lambda name: FakeParam(values[name])
    node.logger = mock.Mock()
    node.get_logger = lambda: node.logger
    return node


@pytest.fixture(autouse=True)
def clean_env(monkeypatch):
    monkeypatch.delenv(TOKEN_ENV_VAR, raising=False)


# --- _load_token: the token comes ONLY from MTRB_WEB_TOKEN or the private file ----------------

def test_load_token_environment_has_priority_over_the_file(monkeypatch, tmp_path):
    path = tmp_path / 'tok'
    path.write_text('from-the-file-0123456789')
    path.chmod(0o600)
    monkeypatch.setenv(TOKEN_ENV_VAR, '  from-env-0123456789  ')
    node = make_node({'auth.token_file': str(path)})
    assert node._load_token() == 'from-env-0123456789'


def test_load_token_reads_the_private_file_and_strips_it(tmp_path):
    path = tmp_path / 'tok'
    path.write_text('from-the-file-0123456789\n')
    path.chmod(0o600)
    assert make_node({'auth.token_file': str(path)})._load_token() == 'from-the-file-0123456789'


def test_load_token_without_any_source_is_a_config_error_with_instructions(tmp_path):
    node = make_node({'auth.token_file': str(tmp_path / 'missing')})
    with pytest.raises(bridge.ConfigError) as caught:
        node._load_token()
    text = str(caught.value)
    assert 'MTRB_WEB_TOKEN' in text and str(tmp_path / 'missing') in text
    assert '0600' in text and '20' in text


def test_load_token_empty_file_is_a_config_error(tmp_path):
    path = tmp_path / 'tok'
    path.write_text('\n')
    path.chmod(0o600)
    with pytest.raises(bridge.ConfigError):
        make_node({'auth.token_file': str(path)})._load_token()


def test_load_token_unsafe_file_is_a_config_error_that_never_shows_the_content(tmp_path):
    path = tmp_path / 'tok'
    path.write_text(SECRET)
    path.chmod(0o644)
    with pytest.raises(bridge.ConfigError) as caught:
        make_node({'auth.token_file': str(path)})._load_token()
    assert SECRET not in str(caught.value) and SECRET not in repr(caught.value)
    assert 'chmod 600' in str(caught.value)


def test_load_token_symlink_is_a_config_error(tmp_path):
    real = tmp_path / 'real'
    real.write_text(SECRET)
    real.chmod(0o600)
    link = tmp_path / 'link'
    link.symlink_to(real)
    with pytest.raises(bridge.ConfigError) as caught:
        make_node({'auth.token_file': str(link)})._load_token()
    assert SECRET not in str(caught.value)


@pytest.mark.parametrize('path', ['relative/token', '~nonexistent_user_zzz9/token', ''])
def test_load_token_relative_or_unresolved_path_is_a_config_error(path):
    with pytest.raises(bridge.ConfigError, match='must be an absolute path'):
        make_node({'auth.token_file': path})._load_token()


@pytest.mark.parametrize('value', [None, 5, True, ['/x']])
def test_load_token_non_string_token_file_is_a_config_error(value):
    with pytest.raises(bridge.ConfigError, match='auth.token_file'):
        make_node({'auth.token_file': value})._load_token()


def test_the_token_is_not_a_ros_parameter_any_more():
    assert 'auth.token' not in {name for name, _ in bridge.PARAMETERS}
    assert 'auth.token' not in bridge.DYNAMIC_PARAMETERS
    assert dict(bridge.PARAMETERS)['auth.token_file'] == '~/.config/mtrb/web_token'
    assert not [n for n, _ in bridge.PARAMETERS
                if n.split('.')[-1] in ('token', 'secret', 'password')]


# Parameter defaults of the teleop section (mirror param/web.yaml and PARAMETERS).
TELEOP_DEFAULTS = {'teleop.max_vx': 0.2, 'teleop.max_wz': 0.8, 'teleop.timeout_ms': 300,
                   'teleop.enabled': True}


def _fake_settings(self):
    """Stand-in for _read_settings (token/port tests only; see the _read_settings tests below)."""
    self._max_clients = 5
    self._max_ws_bytes = 4096
    self._max_connections = 64
    self._dock = (0.0, 0.0, 0.0)
    self._map_dir = '/share/map'  # phase 5: __init__ hands it to create_app
    self._map_min_free_mb = 200   # security fixes: quotas handed to create_app
    self._map_min_interval_s = 5.0


def _init_node(params, token_result=SECRET, real_loader=False):
    """Run the real __init__ with ROS pieces replaced by doubles (token loader included)."""
    values = {'auth.token_file': '/x/web_token', 'host': '127.0.0.1', 'port': 8080,
              'auth.allow_example_token': False}
    values.update(TELEOP_DEFAULTS)  # _read_teleop_settings runs for real in __init__
    values.update(params)
    logger = mock.Mock()
    create_app_mock = mock.Mock(return_value=object())
    with mock.patch.object(bridge.Node, '__init__', lambda self, name: None), \
            mock.patch.object(bridge.MtrbWebBridge, 'declare_parameter',
                              create=True, new=lambda self, *a, **k: None), \
            mock.patch.object(bridge.MtrbWebBridge, 'get_parameter', create=True,
                              new=lambda self, n: FakeParam(values[n])), \
            mock.patch.object(bridge.MtrbWebBridge, 'get_logger', create=True,
                              new=lambda self: logger), \
            mock.patch.object(bridge, 'get_package_share_directory', return_value='/share'), \
            mock.patch.object(bridge, 'load_token', new=(
                bridge.load_token if real_loader else (lambda *a, **k: token_result))), \
            mock.patch.object(bridge.MtrbWebBridge, '_read_settings', new=_fake_settings), \
            mock.patch.object(bridge.MtrbWebBridge, '_init_ros_io', new=lambda self: None), \
            mock.patch.object(bridge, 'create_app', new=create_app_mock):
        node = bridge.MtrbWebBridge()
    node.create_app_mock = create_app_mock   # kept off the logger: it receives the token
    return node, logger


@pytest.mark.parametrize('port', [0, 1023, 65536, 'abc'])
def test_invalid_port_raises_config_error(port):
    with pytest.raises(bridge.ConfigError):
        _init_node({'port': port})


@pytest.mark.parametrize('host', ['', None])
def test_empty_host_raises_config_error(host):
    with pytest.raises(bridge.ConfigError):
        _init_node({'host': host})


def test_no_token_anywhere_raises_config_error():
    with pytest.raises(bridge.ConfigError, match='MTRB_WEB_TOKEN'):
        _init_node({}, token_result='')


def test_real_token_does_not_warn_and_is_never_logged(caplog):
    with caplog.at_level(logging.DEBUG):
        node, logger = _init_node({}, token_result=SECRET)
    assert not logger.warning.called
    assert SECRET not in ' '.join(str(c) for c in logger.method_calls)
    assert SECRET not in caplog.text


# --- Broadcast logic with test doubles (phase 2, I-1/I-2) ------------------------------
# These tests drive the node's callbacks/timers on an instance built without Node.__init__
# (fake clock, fake TF buffer, real StateStore and BroadcastHub). The integration with real
# publishers (rclpy topics, TF, uvicorn) is NOT covered here: it runs on the Raspberry Pi.


class TransformError(Exception):
    pass


def make_bridge(now=100.0):
    node = object.__new__(bridge.MtrbWebBridge)
    node.clock = [now]
    node._now_s = lambda: node.clock[0]
    # Only the PUBLIC rclpy clock API (get_clock().now().nanoseconds) backs _now_ns, and it
    # follows the fake clock of the tests.
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(round(node.clock[0] * 1e9))))
    node.logger = mock.Mock()
    node.get_logger = lambda: node.logger
    node.hub = BroadcastHub()
    node.store = StateStore(clock=node._now_s)
    node._init_state()
    node._plan_max_points = 200
    node._scan_max_points = 360
    node._scan_min_range_m = 0.15
    node._scan_max_range_m = 12.0
    node._mode = 'map'
    node._dock = (0.0, 0.0, 0.0)
    # Phase 5: _broadcast_status reads the lifecycle snapshot. No watched nodes here (so
    # `nodes` is {}); the lifecycle behaviour has its own tests further down.
    node._status_nodes = ()
    node._lifecycle_period_s = 2.0
    node.create_subscription = node.create_timer = mock.Mock()
    node.create_client = mock.Mock()
    node._init_status_io()
    node.tf = None  # (x, y, yaw, stamp_s) or exception
    node._lookup = lambda target, source: node.tf
    node.channel = node.hub.add_client(lambda: None)
    return node


def drain(node):
    return [json.loads(i) if isinstance(i, str) else i for i in node.channel.take_all()]


def grid(width=2, height=2, data=None, frame='map', resolution=0.05):
    cells = array('b', [0, -1, 100, 0] if data is None else data)
    info = NS(width=width, height=height, resolution=resolution,
              origin=NS(position=NS(x=0.0, y=0.0)))
    return NS(header=NS(frame_id=frame), info=info, data=cells)


def test_map_repeated_identical_does_not_bump_version():
    node = make_bridge()
    node._on_map(grid())
    node._broadcast_map()
    node._on_map(grid())
    node._broadcast_map()
    assert node._map_version == 1
    assert node.store.snapshot()['map']['version'] == 1


def test_map_changed_content_bumps_version_and_publishes_meta_and_binary():
    node = make_bridge()
    node._on_map(grid())
    node._broadcast_map()
    first = drain(node)
    node._on_map(grid(data=[0, 0, 0, 0]))
    node._broadcast_map()
    second = drain(node)
    assert first[0]['type'] == 'map_meta' and first[0]['version'] == 1
    assert isinstance(first[1], bytes)
    assert second[0]['version'] == 2


@pytest.mark.parametrize('msg', [
    grid(data=[0, 0, 0]),             # len(data) != width * height
    grid(frame='odom'),               # wrong frame
    grid(width=0, data=[]),           # empty size
    grid(resolution=0.0),             # invalid resolution
])
def test_map_invalid_is_ignored(msg):
    node = make_bridge()
    node._on_map(msg)
    node._broadcast_map()
    assert drain(node) == []
    assert node._map_version == 0
    assert node.store.snapshot()['map'] is None


def test_map_not_dirty_is_not_reprocessed():
    node = make_bridge()
    node._on_map(grid())
    node._broadcast_map()
    drain(node)
    node._broadcast_map()
    assert drain(node) == []


def _path(frame, points):
    poses = [NS(pose=NS(position=NS(x=x, y=y))) for x, y in points]
    return NS(header=NS(frame_id=frame), poses=poses)


def test_plan_in_other_frame_is_ignored():
    node = make_bridge()
    node._on_plan(_path('odom', [(0, 0), (1, 1)]))
    assert drain(node) == []
    assert node.store.snapshot()['plan_len'] is None


def test_plan_in_map_frame_is_published_and_unchanged_plan_is_not_repeated():
    node = make_bridge()
    node._on_plan(_path('map', [(0, 0), (1, 1)]))
    node._on_plan(_path('/map', [(0, 0), (1, 1)]))
    messages = drain(node)
    assert len(messages) == 1
    assert messages[0]['type'] == 'plan'
    assert node.store.snapshot()['plan_len'] == 2


@pytest.mark.parametrize('pct', [float('nan'), float('inf'), -0.1, 1.5])
def test_battery_invalid_becomes_null(pct):
    node = make_bridge()
    node._on_battery(NS(percentage=pct))
    node._broadcast_status()
    assert drain(node)[0]['battery_pct'] is None
    assert node.store.snapshot()['battery_pct'] is None


def test_battery_valid_fraction_becomes_percentage():
    node = make_bridge()
    node._on_battery(NS(percentage=0.756))
    node._broadcast_status()
    assert drain(node)[0]['battery_pct'] == 75.6


def test_status_without_scan_reports_null_age():
    node = make_bridge()
    node._broadcast_status()
    message = drain(node)[0]
    assert message['scan_age'] is None
    assert message['mode'] == 'map'


def test_status_includes_the_dock_pose_with_finite_x_y_yaw():
    node = make_bridge()
    node._dock = (1.5, -2.0, 0.25)
    node._broadcast_status()
    dock = drain(node)[0]['dock']
    assert dock == {'x': 1.5, 'y': -2.0, 'yaw': 0.25}
    assert all(math.isfinite(v) for v in dock.values())


def test_status_dock_defaults_are_finite_numbers():
    node = make_bridge()
    node._broadcast_status()
    dock = drain(node)[0]['dock']
    assert set(dock) == {'x', 'y', 'yaw'}
    assert all(isinstance(v, float) and math.isfinite(v) for v in dock.values())


def test_pose_without_tf_is_not_published_and_state_pose_is_null():
    node = make_bridge()
    node.store.update(pose={'x': 1})
    node.hub.publish('pose', '{"type":"pose"}')
    node.tf = None
    node._broadcast_pose()
    assert drain(node) == []
    assert node.store.snapshot()['pose'] is None


def test_pose_with_tf_is_published():
    node = make_bridge()
    node.tf = (1.0, 2.0, 0.5, 99.9)
    node._broadcast_pose()
    message = drain(node)[0]
    assert (message['type'], message['x'], message['y'], message['yaw']) == ('pose', 1.0, 2.0, 0.5)


def _scan(ranges=(1.0, 2.0)):
    return NS(header=NS(frame_id='laser'), ranges=list(ranges), angle_min=0.0,
              angle_increment=0.1, range_min=0.1, range_max=10.0)


def test_scan_without_tf_is_not_published():
    node = make_bridge()
    node._on_scan(_scan())
    node.tf = None
    node._broadcast_scan()
    assert drain(node) == []


def test_scan_with_tf_is_published_once_per_received_scan():
    node = make_bridge()
    node.tf = (0.0, 0.0, 0.0, 100.0)
    node._on_scan(_scan())
    node._broadcast_scan()
    node._broadcast_scan()
    messages = drain(node)
    assert len(messages) == 1
    assert messages[0]['type'] == 'scan'
    assert len(messages[0]['points']) == 4


def test_stale_scan_is_discarded_not_published():
    node = make_bridge()
    node.tf = (0.0, 0.0, 0.0, 100.0)
    node._on_scan(_scan())
    node.hub.publish('scan', '{"type":"scan"}')
    node.channel.take_all()
    node.clock[0] += bridge.SCAN_MAX_AGE_S + 0.5
    node._broadcast_scan()
    assert drain(node) == []
    assert node.hub.add_client(lambda: None).take_all() == []


def _real_lookup_node(stamp_s, now=100.0):
    """Node using the real _lookup with a fake TF buffer."""
    node = make_bridge(now)
    del node._lookup  # fall back to the class implementation
    transform = NS(
        transform=NS(translation=NS(x=1.0, y=2.0, z=0.0),
                     rotation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
        header=NS(stamp=stamp_s))
    node._tf_buffer = NS(lookup_transform=lambda *a: transform)
    return node


class FakeTime:
    """Replacement for rclpy Time: the header 'stamp' is already seconds."""

    @staticmethod
    def from_msg(stamp):
        return NS(nanoseconds=int(stamp * 1e9))


@pytest.fixture
def fake_time():
    with mock.patch.object(bridge, 'Time', FakeTime):
        yield


def test_lookup_stale_tf_returns_none(fake_time):
    node = _real_lookup_node(stamp_s=100.0 - bridge.TF_MAX_AGE_S - 1.0)
    assert node._lookup('map', 'base_link') is None


def test_lookup_fresh_tf_returns_pose(fake_time):
    node = _real_lookup_node(stamp_s=99.9)
    x, y, yaw, _ = node._lookup('map', 'base_link')
    assert (x, y, yaw) == (1.0, 2.0, 0.0)


def test_lookup_transform_exception_returns_none(fake_time):
    node = _real_lookup_node(stamp_s=99.9)

    def boom(*args):
        raise bridge.TransformException('no tf')

    node._tf_buffer = NS(lookup_transform=boom)
    assert node._lookup('map', 'base_link') is None


# --- Scan staleness and stamp (review findings, phase 2) --------------------------------


def _node_with_sent_scan(now=100.0):
    """Node that has already broadcast one fresh scan (client drained)."""
    node = make_bridge(now)
    node.tf = (0.0, 0.0, 0.0, now)
    node._on_scan(_scan())
    node._broadcast_scan()
    assert [m['type'] for m in drain(node)] == ['scan']
    return node


def _late_client_messages(node):
    channel = node.hub.add_client(lambda: None)
    return [json.loads(i) for i in channel.take_all() if isinstance(i, str)]


def test_scan_already_sent_then_silent_lidar_is_discarded_for_new_clients():
    node = _node_with_sent_scan()
    node.clock[0] += bridge.SCAN_MAX_AGE_S + 0.1
    node._broadcast_scan()
    assert [m for m in _late_client_messages(node) if m['type'] == 'scan'] == []


def test_scan_silent_lidar_discards_pending_scan_of_connected_clients():
    node = _node_with_sent_scan()
    node.hub.publish('scan', '{"type":"scan"}')  # pending, not yet drained
    node.clock[0] += bridge.SCAN_MAX_AGE_S + 0.1
    node._broadcast_scan()
    assert drain(node) == []


def test_scan_silent_lidar_store_scan_age_exceeds_limit():
    node = _node_with_sent_scan()
    node.clock[0] += bridge.SCAN_MAX_AGE_S + 0.5
    node._broadcast_scan()
    assert node.store.snapshot()['scan_age'] > bridge.SCAN_MAX_AGE_S


def test_scan_age_exactly_at_limit_is_not_discarded():
    node = _node_with_sent_scan()
    node.clock[0] += bridge.SCAN_MAX_AGE_S
    node._broadcast_scan()
    assert [m['type'] for m in _late_client_messages(node)] == ['scan']
    assert drain(node) == []


def test_scan_none_message_neither_raises_nor_discards():
    node = make_bridge()
    node.hub.publish('scan', '{"type":"scan"}')
    node.channel.take_all()
    node._broadcast_scan()
    assert [m['type'] for m in _late_client_messages(node)] == ['scan']
    assert node.store.snapshot()['scan_age'] is None


def test_scan_fresh_and_already_sent_is_not_resent_nor_discarded():
    node = _node_with_sent_scan()
    node.clock[0] += bridge.SCAN_MAX_AGE_S / 2
    node._broadcast_scan()
    assert drain(node) == []
    assert [m['type'] for m in _late_client_messages(node)] == ['scan']


def test_scan_recovers_after_new_scan_following_a_silent_period():
    node = _node_with_sent_scan()
    node.clock[0] += bridge.SCAN_MAX_AGE_S + 1.0
    node._broadcast_scan()
    node.tf = (0.0, 0.0, 0.0, node.clock[0])
    node._on_scan(_scan())
    node._broadcast_scan()
    messages = drain(node)
    assert [m['type'] for m in messages] == ['scan']
    assert [m['type'] for m in _late_client_messages(node)] == ['scan']


def test_scan_published_stamp_is_the_tf_stamp():
    node = make_bridge()
    node.tf = (0.0, 0.0, 0.0, 99.4567)
    node._on_scan(_scan())
    node._broadcast_scan()
    assert drain(node)[0]['stamp'] == 99.457


# --- _read_settings / _number ------------------------------------------------------------

VALID_SETTINGS = {
    'rates.pose': 10.0, 'rates.scan': 5.0, 'rates.status': 1.0,
    'scan.max_points': 360, 'scan.min_range': 0.15, 'scan.max_range': 12.0,
    'plan.max_points': 200, 'limits.max_clients': 5, 'limits.max_ws_bytes': 4096,
    'limits.max_connections': 64, 'mode': 'map', 'dock.x': 0.0, 'dock.y': 0.0, 'dock.yaw': 0.0,
    # Phase 5 parameters (mirror param/web.yaml).
    'rates.lifecycle': 0.5, 'map_dir': '/maps',
    # Security fixes (mirror param/web.yaml).
    'maps.min_free_mb': 200, 'maps.min_interval_s': 5.0, 'auth.allow_example_token': False,
    'status.nodes': ['controller_server', 'planner_server', 'bt_navigator',
                     'behavior_server', 'velocity_smoother'],
}


def _settings_node(**overrides):
    values = dict(VALID_SETTINGS)
    values.update(overrides)
    node = object.__new__(bridge.MtrbWebBridge)
    node.get_parameter = lambda name: FakeParam(values[name])
    node.logger = mock.Mock()
    node.get_logger = lambda: node.logger
    return node


def test_read_settings_valid_values_set_periods_and_limits():
    node = _settings_node()
    node._read_settings()
    assert node._pose_period_s == pytest.approx(0.1)
    assert node._scan_period_s == pytest.approx(0.2)
    assert node._status_period_s == pytest.approx(1.0)
    assert node._scan_max_points == 360
    assert node._max_clients == 5
    assert node._max_ws_bytes == 4096
    assert node._mode == 'map'


@pytest.mark.parametrize('name', ['rates.pose', 'rates.scan', 'rates.status'])
@pytest.mark.parametrize('bad', [0, 0.0, -1.0, 0.05, True, float('nan'), float('inf'), 'x'])
def test_read_settings_invalid_rate_raises(name, bad):
    with pytest.raises(bridge.ConfigError):
        _settings_node(**{name: bad})._read_settings()


@pytest.mark.parametrize('name', [
    'scan.max_points', 'plan.max_points', 'limits.max_clients', 'limits.max_ws_bytes'])
@pytest.mark.parametrize('bad', [0, -3, 1.5, True, float('nan'), '5', None])
def test_read_settings_invalid_integer_limit_raises(name, bad):
    with pytest.raises(bridge.ConfigError):
        _settings_node(**{name: bad})._read_settings()


def test_read_settings_plan_max_points_below_two_raises():
    with pytest.raises(bridge.ConfigError):
        _settings_node(**{'plan.max_points': 1})._read_settings()


@pytest.mark.parametrize('bad', [-0.1, True, float('nan'), float('inf')])
def test_read_settings_invalid_min_range_raises(bad):
    with pytest.raises(bridge.ConfigError):
        _settings_node(**{'scan.min_range': bad})._read_settings()


@pytest.mark.parametrize('min_range, max_range', [(5.0, 5.0), (6.0, 5.0)])
def test_read_settings_min_range_not_below_max_range_raises(min_range, max_range):
    node = _settings_node(**{'scan.min_range': min_range, 'scan.max_range': max_range})
    with pytest.raises(bridge.ConfigError):
        node._read_settings()


def test_number_error_message_names_the_parameter():
    node = _settings_node(**{'rates.pose': -1.0})
    with pytest.raises(bridge.ConfigError, match='rates.pose'):
        node._number('rates.pose', 0.1)


@pytest.mark.parametrize('raw, expected', [
    ('map', 'map'), ('mapping', 'mapping'), (' MAPPING ', 'mapping'),
    ('', None), ('nav', None), ('maps', None)])
def test_read_settings_mode(raw, expected):
    node = _settings_node(mode=raw)
    node._read_settings()
    assert node._mode == expected


def test_read_settings_dock_is_a_tuple_of_floats():
    node = _settings_node(**{'dock.x': 1, 'dock.y': -2.5, 'dock.yaw': 0.5})
    node._read_settings()
    assert node._dock == (1.0, -2.5, 0.5)
    assert all(type(v) is float for v in node._dock)


@pytest.mark.parametrize('name', ['dock.x', 'dock.y', 'dock.yaw'])
@pytest.mark.parametrize('bad', [float('nan'), float('inf'), True, '1', None, 10001.0,
                                 -10001.0])
def test_read_settings_invalid_dock_raises(name, bad):
    node = _settings_node(**{name: bad})
    with pytest.raises(bridge.ConfigError):
        node._read_settings()


def test_read_settings_dock_at_the_limit_is_accepted():
    node = _settings_node(**{'dock.x': 10000.0, 'dock.y': -10000.0})
    node._read_settings()
    assert node._dock[:2] == (10000.0, -10000.0)


# --- _init_ros_io with doubles ------------------------------------------------------------


def _io_node():
    node = object.__new__(bridge.MtrbWebBridge)
    node._pose_period_s, node._scan_period_s, node._status_period_s = 0.1, 0.2, 1.0
    node._status_nodes = ('map_server', 'amcl')   # phase 5: lifecycle clients to create
    node._lifecycle_period_s = 2.0
    node.create_subscription = mock.Mock()
    node.create_timer = mock.Mock()
    node.created_publishers = {}  # topic -> the object create_publisher returned for it

    def create_publisher(msg_type, topic, depth):
        node.created_publishers[topic] = mock.Mock(name=f'publisher-{topic}')
        return node.created_publishers[topic]

    node.create_publisher = mock.Mock(side_effect=create_publisher)
    node.created_clients = {}  # service name -> the client create_client returned for it

    def create_client(srv_type, name, **kwargs):
        node.created_clients[name] = mock.Mock(name=f'client-{name}')
        return node.created_clients[name]

    node.create_client = mock.Mock(side_effect=create_client)
    node._publish_nav = mock.Mock()  # the initial `idle` message is checked separately
    with mock.patch.object(bridge, 'Buffer'), mock.patch.object(bridge, 'TransformListener'), \
            mock.patch.object(bridge, 'ActionClient') as action_client:
        node._init_ros_io()
    node.action_client_cls = action_client
    return node


def test_init_ros_io_map_subscription_is_reliable_transient_local_depth_1():
    node = _io_node()
    calls = [c for c in node.create_subscription.call_args_list
             if c.args[0] is bridge.OccupancyGrid]
    assert len(calls) == 1
    assert calls[0].args[1] == 'map'
    qos = calls[0].args[3]
    assert qos.depth == 1
    assert qos.reliability == bridge.ReliabilityPolicy.RELIABLE
    assert qos.durability == bridge.DurabilityPolicy.TRANSIENT_LOCAL


def test_init_ros_io_subscribes_to_the_five_expected_topics_only():
    node = _io_node()
    topics = sorted(c.args[1] for c in node.create_subscription.call_args_list)
    # Phase 5 adds `odom` (only its arrival time is used for status.odom_age).
    assert topics == ['battery_state', 'map', 'odom', 'plan', 'scan']


def test_init_ros_io_scan_uses_sensor_data_qos():
    node = _io_node()
    call = [c for c in node.create_subscription.call_args_list if c.args[1] == 'scan'][0]
    assert call.args[3] is bridge.qos_profile_sensor_data


def test_init_ros_io_timer_periods_and_callbacks():
    node = _io_node()
    timers = {c.args[1].__name__: c.args[0] for c in node.create_timer.call_args_list}
    assert timers == {
        '_broadcast_pose': pytest.approx(0.1),      # 10 Hz
        '_broadcast_scan': pytest.approx(0.2),      # 5 Hz
        '_broadcast_status': pytest.approx(1.0),    # 1 Hz
        '_broadcast_map': pytest.approx(bridge.MAP_PERIOD_S),
        '_nav_watchdog': pytest.approx(bridge.NAV_WATCHDOG_PERIOD_S),
        '_teleop_watchdog': pytest.approx(0.02),    # 20 ms << teleop.timeout_ms
        '_poll_lifecycle': pytest.approx(2.0),      # phase 5: rates.lifecycle 0.5 Hz
    }
    assert bridge.MAP_PERIOD_S == 1.0
    assert bridge.TELEOP_WATCHDOG_PERIOD_S == 0.02


def test_init_ros_io_teleop_watchdog_has_its_own_callback_group():
    node = _io_node()
    by_name = {c.args[1].__name__: c for c in node.create_timer.call_args_list}
    group = by_name['_teleop_watchdog'].kwargs.get('callback_group')
    assert isinstance(group, bridge.MutuallyExclusiveCallbackGroup)
    for name, call in by_name.items():
        # The others stay in the node's default group, except the lifecycle polling (phase 5)
        # which has a group of its own (tested in the lifecycle section).
        if name not in ('_teleop_watchdog', '_poll_lifecycle'):
            assert 'callback_group' not in call.kwargs, name


def test_init_ros_io_creates_exactly_the_initialpose_and_cmd_vel_nav_publishers():
    node = _io_node()
    created = {c.args[1]: c.args[0] for c in node.create_publisher.call_args_list}
    assert len(node.create_publisher.call_args_list) == 2
    assert set(created) == {'initialpose', 'cmd_vel_nav'}
    assert created['initialpose'] is bridge.PoseWithCovarianceStamped
    assert created['cmd_vel_nav'] is bridge.Twist
    assert bridge.Twist.__name__ == 'Twist'  # geometry_msgs/Twist, NOT TwistStamped
    assert not hasattr(bridge, 'TwistStamped')


def test_init_ros_io_never_creates_a_bare_cmd_vel_publisher():
    node = _io_node()
    topics = [c.args[1] for c in node.create_publisher.call_args_list]
    assert 'cmd_vel' not in topics
    assert all(not t.lstrip('/').rstrip('/').endswith('/cmd_vel') for t in topics)
    assert all(t == 'cmd_vel_nav' or t == 'initialpose' for t in topics)


def test_init_ros_io_keeps_the_cmd_vel_nav_publisher_for_the_teleop():
    node = _io_node()
    assert node._cmd_vel_nav_pub is node.created_publishers['cmd_vel_nav']
    assert node._initial_pose_pub is node.created_publishers['initialpose']
    assert node._cmd_vel_nav_pub is not node._initial_pose_pub


def test_init_ros_io_cmd_vel_nav_name_is_relative_so_it_lives_in_the_node_namespace():
    node = _io_node()
    call = [c for c in node.create_publisher.call_args_list if 'cmd_vel_nav' in c.args][0]
    assert call.args[1] == 'cmd_vel_nav'
    assert not call.args[1].startswith('/')


def test_init_ros_io_creates_the_navigate_to_pose_action_client_with_relative_name():
    node = _io_node()
    args = node.action_client_cls.call_args.args
    assert args[1] is bridge.NavigateToPose
    assert args[2] == 'navigate_to_pose'


def test_init_ros_io_creates_the_cancel_all_client_with_type_and_exact_service_name():
    node = _io_node()
    # Phase 5: create_client is also used for the lifecycle and slam_toolbox clients, so the
    # cancel-all client is looked up by its service name (exactly one such client).
    calls = [c for c in node.create_client.call_args_list
             if c.args[1] == 'navigate_to_pose/_action/cancel_goal']
    assert len(calls) == 1
    assert calls[0].args[0] is bridge.CancelGoal
    assert node._cancel_all_client is node.created_clients['navigate_to_pose/_action/cancel_goal']


def test_cancel_all_client_is_none_before_ros_io_is_created():
    assert make_bridge()._cancel_all_client is None


def test_init_ros_io_publishes_idle_nav_state():
    node = object.__new__(bridge.MtrbWebBridge)
    node._pose_period_s, node._scan_period_s, node._status_period_s = 0.1, 0.2, 1.0
    node._status_nodes, node._lifecycle_period_s = (), 2.0  # phase 5 attributes
    node.create_subscription = node.create_timer = node.create_publisher = mock.Mock()
    node.create_client = mock.Mock()
    node._publish_nav = mock.Mock()
    with mock.patch.object(bridge, 'Buffer'), mock.patch.object(bridge, 'TransformListener'), \
            mock.patch.object(bridge, 'ActionClient'):
        node._init_ros_io()
    node._publish_nav.assert_called_once_with('idle', None)


def test_default_rates_give_the_specified_timer_periods():
    node = _settings_node(**{'rates.pose': 10.0, 'rates.scan': 5.0, 'rates.status': 1.0})
    node._read_settings()
    assert (node._pose_period_s, node._scan_period_s, node._status_period_s) == (
        pytest.approx(0.1), pytest.approx(0.2), pytest.approx(1.0))


# rclpy creates its own publishers in Node.__init__ (/parameter_events, /rosout); they are
# not the bridge's. Compare on the last path segment so namespaces do not matter.
RCLPY_INTERNAL_TOPICS = {'parameter_events', 'rosout'}


def _own_publisher_topics(node):
    """Topic names of the node's publishers, without rclpy's internal ones."""
    topics = [pub.topic_name for pub in node.publishers]
    return [t for t in topics if t.rstrip('/').split('/')[-1] not in RCLPY_INTERNAL_TOPICS]


def test_real_rclpy_node_publishes_only_initialpose_and_cmd_vel_nav_with_timers(monkeypatch):
    """
    Real rclpy node (SOLO Pi/ROS: only meaningful with the package installed).

    Skipped where the package share directory or a ROS context is unavailable.
    """
    rclpy = pytest.importorskip('rclpy')
    from ament_index_python.packages import PackageNotFoundError
    monkeypatch.setenv(TOKEN_ENV_VAR, 'integration-token-xyz')
    try:
        rclpy.init()
    except Exception as err:  # noqa: B902 - no usable ROS context
        pytest.skip(f'rclpy cannot be initialised: {err}')
    node = None
    try:
        try:
            node = bridge.MtrbWebBridge()
        except PackageNotFoundError:
            pytest.skip('mtrb_bringup is not installed')
        own = _own_publisher_topics(node)
        names = {t.rstrip('/').split('/')[-1] for t in own}
        assert names == {'initialpose', 'cmd_vel_nav'}, (
            f'only initialpose and cmd_vel_nav may be published, found {own}')
        assert len(own) == 2
        for topic in own:  # explicit guard: the web NEVER publishes on a bare cmd_vel
            assert topic.rstrip('/').split('/')[-1] != 'cmd_vel', topic
        nav_topics = [t for t in own if t.rstrip('/').split('/')[-1] == 'cmd_vel_nav']
        assert len(nav_topics) == 1 and nav_topics[0].endswith('/cmd_vel_nav')
        # Node.timers: rclpy Jazzy exposes Timer.timer_period_ns (int); fall back to the
        # handle's period if a distro lacks the property (defensive, API varies by distro).
        periods = sorted(round(t.timer_period_ns / 1e9, 3) for t in node.timers)
        # Phase 5 adds the lifecycle polling timer: rates.lifecycle 0.5 Hz = 2.0 s.
        assert periods == [0.02, 0.1, 0.2, 0.5, 1.0, 1.0, 2.0]
        # Node.subscriptions: Subscription.topic_name (fully qualified) and .qos_profile.
        by_topic = {s.topic_name.rstrip('/').split('/')[-1]: s for s in node.subscriptions}
        assert {'map', 'scan', 'plan', 'battery_state'} <= set(by_topic)
        qos = by_topic['map'].qos_profile
        assert qos.depth == 1
        assert qos.reliability == bridge.ReliabilityPolicy.RELIABLE
        assert qos.durability == bridge.DurabilityPolicy.TRANSIENT_LOCAL
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


# --- Integration with real rclpy (SOLO Pi/ROS; skipped without rclpy / ROS context) -----------

INTEGRATION_NS = '/mtrb_it_ns'
INTEGRATION_TIMEOUT_S = 10.0


def test_integration_late_node_receives_latched_map_pose_and_scan(monkeypatch):
    """
    Task 12 (SOLO Pi/ROS): fake map/tf/scan publishers -> real node -> hub, no uvicorn.

    The map is published BEFORE the node exists: it must still arrive (TRANSIENT_LOCAL).
    Spinning is bounded by a monotonic deadline; there is no time.sleep.
    """
    import time
    import zlib

    rclpy = pytest.importorskip('rclpy')
    from ament_index_python.packages import PackageNotFoundError
    TransformStamped = pytest.importorskip('geometry_msgs.msg').TransformStamped
    TFMessage = pytest.importorskip('tf2_msgs.msg').TFMessage
    OccupancyGrid = pytest.importorskip('nav_msgs.msg').OccupancyGrid
    LaserScan = pytest.importorskip('sensor_msgs.msg').LaserScan
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import (
        DurabilityPolicy, QoSProfile, qos_profile_sensor_data, ReliabilityPolicy)

    monkeypatch.setenv(TOKEN_ENV_VAR, 'integration-token-xyz')
    try:
        rclpy.init(args=['--ros-args', '-r', f'__ns:={INTEGRATION_NS}'])
    except Exception as err:  # noqa: B902 - no usable ROS context
        pytest.skip(f'rclpy cannot be initialised: {err}')
    fake = node = executor = None
    try:
        fake = rclpy.create_node('mtrb_it_fake_sources')
        map_pub = fake.create_publisher(
            OccupancyGrid, 'map', QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        tf_pub = fake.create_publisher(TFMessage, '/tf', 100)
        scan_pub = fake.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)

        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = 'map'
        grid_msg.info.width, grid_msg.info.height = 3, 2
        grid_msg.info.resolution = 0.05
        grid_msg.data = [0, -1, 100, 0, 50, -1]
        map_pub.publish(grid_msg)  # BEFORE the bridge node exists

        try:
            node = bridge.MtrbWebBridge()
        except PackageNotFoundError:
            pytest.skip('mtrb_bringup is not installed')
        channel = node.hub.add_client(lambda: None)
        assert channel is not None
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(fake)

        def publish_fresh():
            now = fake.get_clock().now().to_msg()
            transforms = []
            for parent, child in (('map', 'odom'), ('odom', 'base_link'),
                                  ('base_link', 'laser')):
                tf = TransformStamped()
                tf.header.stamp, tf.header.frame_id, tf.child_frame_id = now, parent, child
                tf.transform.rotation.w = 1.0
                transforms.append(tf)
            transforms[1].transform.translation.x = 1.5
            tf_pub.publish(TFMessage(transforms=transforms))
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = now, 'laser'
            scan.angle_min, scan.angle_increment = 0.0, 0.1
            scan.range_min, scan.range_max = 0.15, 12.0
            scan.ranges = [1.0] * 10
            scan_pub.publish(scan)

        seen = {}
        deadline = time.monotonic() + INTEGRATION_TIMEOUT_S
        while time.monotonic() < deadline and not {'pose', 'scan', 'map_meta'} <= set(seen):
            publish_fresh()
            executor.spin_once(timeout_sec=0.1)
            for item in channel.take_all():
                if isinstance(item, bytes):
                    seen['map_bin'] = item
                else:
                    seen[json.loads(item)['type']] = json.loads(item)
        assert {'pose', 'scan', 'map_meta'} <= set(seen), f'timed out, got {sorted(seen)}'
        assert seen['pose']['x'] == pytest.approx(1.5, abs=1e-3)
        assert len(seen['scan']['points']) > 0
        meta = seen['map_meta']
        assert (meta['width'], meta['height']) == (3, 2)
        assert meta['resolution'] == pytest.approx(0.05)
        binary = seen['map_bin']
        assert int.from_bytes(binary[:4], 'little') == meta['version']
        assert list(zlib.decompress(binary[4:])) == [0, 255, 100, 0, 50, 255]
        assert node.store.snapshot()['map']['version'] == meta['version']
    finally:
        if executor is not None:
            executor.shutdown()
        for n in (node, fake):
            if n is not None:
                n.destroy_node()
        rclpy.shutdown()


# --- Navigation goal state machine with doubles (task 15, U-4) --------------------------------
# The ROS action client, futures and goal handles are replaced by doubles; the node is built
# without Node.__init__. NOT covered here (needs real ROS / the Pi): the NavigateToPose goal
# message content (_make_goal) and the real Nav2 action server behaviour.

ACCEPTED, SUCCEEDED, CANCELED, ABORTED = 1, 4, 5, 6


class FakeFuture:
    def __init__(self):
        self.callbacks = []
        self.value = None
        self.error = None

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def result(self):
        if self.error is not None:
            raise self.error
        return self.value

    def resolve(self, value=None, error=None):
        self.value, self.error = value, error
        for callback in self.callbacks:
            callback(self)


class FakeHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancel_calls = 0
        self.cancel_error = None
        self.cancel_futures = []
        self.result_future = FakeFuture()
        self.result_error = None

    def get_result_async(self):
        if self.result_error:
            raise self.result_error
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        if self.cancel_error:
            raise self.cancel_error
        future = FakeFuture()
        self.cancel_futures.append(future)
        return future


class FakeCancelClient:
    """Fake of a PUBLIC rclpy service client: service_is_ready() and call_async() only."""

    def __init__(self):
        self.ready = True
        self.requests = []
        self.futures = []
        self.call_error = None
        self.lock_probe = None      # callable run at every call_async (lock spy)

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        if self.lock_probe is not None:
            self.lock_probe()
        if self.call_error:
            raise self.call_error
        self.requests.append(request)
        future = FakeFuture()
        self.futures.append(future)
        return future

    def answer(self, index, code):
        self.futures[index].resolve(NS(return_code=code))


class FakeActionClient:
    def __init__(self):
        self.ready = True
        self.futures = []
        self.feedback_callbacks = []
        self.send_error = None

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, feedback_callback=None):
        if self.send_error:
            raise self.send_error
        future = FakeFuture()
        self.futures.append(future)
        self.feedback_callbacks.append(feedback_callback)
        return future


def cancel_client(node):
    return node._cancel_all_client


def make_nav_bridge(now=100.0):
    node = make_bridge(now)
    node._nav_client = FakeActionClient()
    node._cancel_all_client = FakeCancelClient()  # what create_client() returns in production
    node._make_goal = lambda x, y, yaw: ('goal', x, y, yaw)
    drain(node)
    return node


def nav_messages(node):
    return [m for m in drain(node) if isinstance(m, dict) and m.get('type') == 'nav']


def accept(node, index=0):
    """Nav2 accepts the goal number `index`; returns the handle."""
    handle = FakeHandle()
    node._nav_client.futures[index].resolve(handle)
    return handle


def feedback(distance):
    return NS(feedback=NS(distance_remaining=distance))


def test_send_goal_returns_id_and_publishes_active_with_null_distance():
    node = make_nav_bridge()
    goal_id = node.send_goal(1.0, 2.0, 0.5)
    assert isinstance(goal_id, str) and goal_id
    assert nav_messages(node) == [
        {'type': 'nav', 'state': 'active', 'distance_remaining': None}]
    assert node.store.snapshot()['nav'] == {'state': 'active', 'distance_remaining': None}


def test_send_goal_returns_a_distinct_id_each_time():
    node = make_nav_bridge()
    first = node.send_goal(1.0, 2.0, 0.5)
    accept(node).result_future.resolve(NS(status=SUCCEEDED))
    assert node.send_goal(1.0, 2.0, 0.5) != first


def test_send_goal_without_action_server_raises_unavailable_and_stays_idle():
    node = make_nav_bridge()
    node._nav_client.ready = False
    with pytest.raises(bridge.NavUnavailableError):
        node.send_goal(1.0, 2.0, 0.5)
    assert node._nav_client.futures == []
    assert node._nav_token is None
    assert nav_messages(node) == []


def test_send_goal_while_active_raises_busy_and_sends_nothing_more():
    node = make_nav_bridge()
    node.send_goal(1.0, 2.0, 0.5)
    drain(node)
    with pytest.raises(bridge.NavBusyError):
        node.send_goal(3.0, 4.0, 0.0)
    assert len(node._nav_client.futures) == 1
    assert nav_messages(node) == []


def test_send_goal_async_failure_marks_failed_releases_and_raises_unavailable():
    node = make_nav_bridge()
    node._nav_client.send_error = RuntimeError('boom')
    with pytest.raises(bridge.NavUnavailableError):
        node.send_goal(1.0, 2.0, 0.5)
    # the client slot keeps only the latest value: 'active' was replaced by 'failed'
    assert [m['state'] for m in nav_messages(node)] == ['failed']
    assert node.store.snapshot()['nav']['state'] == 'failed'
    assert node._nav_token is None
    node._nav_client.send_error = None
    node.send_goal(1.0, 2.0, 0.5)  # not left busy


def test_goal_rejected_by_nav2_becomes_failed_and_is_released():
    node = make_nav_bridge()
    node.send_goal(1.0, 2.0, 0.5)
    drain(node)
    node._nav_client.futures[0].resolve(FakeHandle(accepted=False))
    assert [m['state'] for m in nav_messages(node)] == ['failed']
    assert node.store.snapshot()['nav']['state'] == 'failed'
    assert node._nav_token is None
    node.send_goal(1.0, 2.0, 0.5)  # a new goal is accepted after the rejection


def test_goal_response_with_exception_becomes_failed():
    node = make_nav_bridge()
    node.send_goal(1.0, 2.0, 0.5)
    drain(node)
    node._nav_client.futures[0].resolve(error=RuntimeError('lost'))
    assert [m['state'] for m in nav_messages(node)] == ['failed']
    assert node._nav_token is None


def test_goal_response_none_becomes_failed():
    node = make_nav_bridge()
    node.send_goal(1.0, 2.0, 0.5)
    drain(node)
    node._nav_client.futures[0].resolve(None)
    assert [m['state'] for m in nav_messages(node)] == ['failed']


@pytest.mark.parametrize('status, expected', [
    (SUCCEEDED, 'succeeded'), (CANCELED, 'canceled'), (ABORTED, 'failed'),
    (0, 'failed'), (2, 'failed'), (99, 'failed')])
def test_action_result_status_maps_to_final_state_and_releases_goal(status, expected):
    node = make_nav_bridge()
    node.send_goal(1.0, 2.0, 0.5)
    handle = accept(node)
    drain(node)
    handle.result_future.resolve(NS(status=status))
    assert [m['state'] for m in nav_messages(node)] == [expected]
    assert node.store.snapshot()['nav'] == {'state': expected, 'distance_remaining': None}
    assert node._nav_token is None


def test_result_future_error_becomes_failed():
    node = make_nav_bridge()
    node.send_goal(1.0, 2.0, 0.5)
    handle = accept(node)
    drain(node)
    handle.result_future.resolve(error=RuntimeError('gone'))
    assert [m['state'] for m in nav_messages(node)] == ['failed']
    assert node._nav_token is None
    # the accepted goal may still be running: it stays tracked and the node stays busy
    assert len(node._nav_orphans) == 1
    with pytest.raises(bridge.NavBusyError):
        node.send_goal(3.0, 4.0, 0.0)


def test_new_goal_is_accepted_after_a_final_state():
    node = make_nav_bridge()
    node.send_goal(1.0, 2.0, 0.5)
    accept(node).result_future.resolve(NS(status=SUCCEEDED))
    node.send_goal(5.0, 6.0, 0.0)
    assert len(node._nav_client.futures) == 2


# --- 5 s without an answer / server lost ---

def test_no_answer_within_5s_fails_the_goal_and_frees_the_node():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    drain(node)
    node.clock[0] = 100.0 + bridge.NAV_RESPONSE_TIMEOUT_S + 0.1
    node._nav_watchdog()
    assert [m['state'] for m in nav_messages(node)] == ['failed']
    assert node._nav_token is None
    node.send_goal(3.0, 4.0, 0.0)


def test_answer_timeout_is_5_seconds():
    assert bridge.NAV_RESPONSE_TIMEOUT_S == 5.0


def test_watchdog_does_not_fail_before_the_5s_deadline():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    drain(node)
    node.clock[0] = 104.9
    node._nav_watchdog()
    assert nav_messages(node) == []
    assert node._nav_token is not None


def test_watchdog_does_nothing_without_a_goal():
    node = make_nav_bridge()
    node.clock[0] = 10000.0
    node._nav_watchdog()
    assert nav_messages(node) == []


def test_accepted_goal_clears_the_answer_deadline():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    accept(node)
    drain(node)
    node.clock[0] = 100.0 + 600.0  # a long navigation is not a timeout
    node._nav_watchdog()
    assert nav_messages(node) == []
    assert node._nav_token is not None


def test_late_acceptance_after_timeout_cancels_the_orphan_goal_and_keeps_node_busy():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    node.clock[0] = 106.0
    node._nav_watchdog()
    drain(node)
    handle = accept(node)
    assert handle.cancel_calls == 1
    assert node._nav_token is None
    assert nav_messages(node) == []
    # fail safe: the node stays busy until Nav2 confirms that the late goal ended
    assert len(node._nav_orphans) == 1
    with pytest.raises(bridge.NavBusyError):
        node.send_goal(3.0, 4.0, 0.0)


def test_server_lost_for_more_than_3s_fails_the_active_goal():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    accept(node)
    drain(node)
    node._nav_client.ready = False
    node._nav_watchdog()          # first sight of the loss: starts the timer
    assert nav_messages(node) == []
    node.clock[0] = 103.0
    node._nav_watchdog()          # exactly 3 s: not yet
    assert nav_messages(node) == []
    node.clock[0] = 103.1
    node._nav_watchdog()
    assert [m['state'] for m in nav_messages(node)] == ['failed']
    assert node._nav_token is None
    assert len(node._nav_orphans) == 1  # accepted goal: not released without a result


def test_server_back_before_3s_resets_the_loss_timer():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    accept(node)
    drain(node)
    node._nav_client.ready = False
    node._nav_watchdog()
    node.clock[0] = 102.0
    node._nav_client.ready = True
    node._nav_watchdog()
    node._nav_client.ready = False
    node.clock[0] = 104.0
    node._nav_watchdog()          # loss restarts at 104
    node.clock[0] = 106.0
    node._nav_watchdog()
    assert nav_messages(node) == []
    assert node._nav_token is not None


# --- cancel ---

def test_cancel_without_goal_is_a_silent_noop():
    node = make_nav_bridge()
    node.cancel()
    node.cancel()
    assert nav_messages(node) == []
    assert node._nav_token is None


def test_cancel_after_acceptance_requests_cancel_and_waits_for_the_result():
    node = make_nav_bridge()
    node.send_goal(1.0, 2.0, 0.5)
    handle = accept(node)
    drain(node)
    node.cancel()
    assert handle.cancel_calls == 1
    assert nav_messages(node) == []        # the final state comes with the action result
    assert node._nav_token is not None
    handle.result_future.resolve(NS(status=CANCELED))
    assert [m['state'] for m in nav_messages(node)] == ['canceled']
    assert node._nav_token is None


def test_cancel_before_acceptance_is_applied_when_nav2_accepts():
    node = make_nav_bridge()
    node.send_goal(1.0, 2.0, 0.5)
    node.cancel()
    drain(node)
    handle = accept(node)
    assert handle.cancel_calls == 1
    handle.result_future.resolve(NS(status=CANCELED))
    assert [m['state'] for m in nav_messages(node)] == ['canceled']
    assert node._nav_token is None


def test_cancel_before_acceptance_but_goal_rejected_ends_failed():
    node = make_nav_bridge()
    node.send_goal(1.0, 2.0, 0.5)
    node.cancel()
    drain(node)
    node._nav_client.futures[0].resolve(FakeHandle(accepted=False))
    assert [m['state'] for m in nav_messages(node)] == ['failed']


def test_cancel_request_failure_keeps_the_goal_busy_and_is_retried_by_the_watchdog():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    handle = accept(node)
    handle.cancel_error = RuntimeError('x')
    drain(node)
    node.cancel()
    # failing to ASK for the cancel must neither fail nor release a moving goal
    assert nav_messages(node) == []
    assert node._nav_token is not None
    assert node.store.snapshot()['nav']['state'] == 'active'
    node.logger.error.assert_called()
    with pytest.raises(bridge.NavBusyError):
        node.send_goal(3.0, 4.0, 0.0)
    assert handle.cancel_calls == 1
    node.clock[0] = 100.9
    node._nav_watchdog()
    assert handle.cancel_calls == 1          # retried at most every NAV_CANCEL_RETRY_S
    node.clock[0] = 101.0
    node._nav_watchdog()
    assert handle.cancel_calls == 2
    node.clock[0] = 101.5
    node._nav_watchdog()
    assert handle.cancel_calls == 2
    node.clock[0] = 102.0
    node._nav_watchdog()
    assert handle.cancel_calls == 3
    # once the request works and Nav2 reports the result, the goal ends normally
    handle.cancel_error = None
    handle.result_future.resolve(NS(status=CANCELED))
    assert [m['state'] for m in nav_messages(node)] == ['canceled']
    assert node._nav_token is None


def test_cancel_without_answer_in_5s_fails_the_goal_but_keeps_it_as_orphan():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    handle = accept(node)
    node.clock[0] = 1000.0
    node.cancel()
    drain(node)
    node.clock[0] = 1004.9
    node._nav_watchdog()
    assert nav_messages(node) == []
    node.clock[0] = 1005.1
    node._nav_watchdog()
    assert [m['state'] for m in nav_messages(node)] == ['failed']
    assert node.store.snapshot()['nav']['state'] == 'failed'
    assert node._nav_token is None
    # the accepted goal is NOT released: still tracked, still being cancelled, node busy
    assert len(node._nav_orphans) == 1
    calls = handle.cancel_calls
    with pytest.raises(bridge.NavBusyError):
        node.send_goal(3.0, 4.0, 0.0)
    assert len(node._nav_client.futures) == 1
    node.clock[0] = 1006.2
    node._nav_watchdog()
    assert handle.cancel_calls == calls + 1


def test_repeated_cancel_requests_do_not_raise_and_goal_stays_single():
    node = make_nav_bridge()
    node.send_goal(1.0, 2.0, 0.5)
    handle = accept(node)
    node.cancel()
    node.cancel()
    assert handle.cancel_calls >= 1
    assert len(node._nav_client.futures) == 1


# --- stale callbacks ---

def test_result_of_an_old_goal_does_not_touch_the_new_goal():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    old_handle = accept(node, 0)
    node.clock[0] = 200.0
    node.cancel()
    node.clock[0] = 206.0
    node._nav_watchdog()               # cancel not answered: failed, kept as orphan
    assert node.store.snapshot()['nav']['state'] == 'failed'
    with pytest.raises(bridge.NavBusyError):
        node.send_goal(3.0, 4.0, 0.0)   # still busy until Nav2 confirms the end
    assert len(node._nav_client.futures) == 1
    drain(node)
    old_handle.result_future.resolve(NS(status=SUCCEEDED))  # Nav2 confirms the old goal
    assert nav_messages(node) == []     # an orphan result is not published
    assert node.store.snapshot()['nav']['state'] == 'failed'
    assert node._nav_orphans == {}
    node.send_goal(3.0, 4.0, 0.0)       # now the node is free
    assert node.store.snapshot()['nav']['state'] == 'active'
    assert len(node._nav_client.futures) == 2


def test_result_of_a_released_old_goal_does_not_touch_the_new_goal():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)       # never accepted by Nav2
    node.clock[0] = 106.0
    node._nav_watchdog()                # no handle: safe to release
    new_id = node.send_goal(3.0, 4.0, 0.0)
    old_handle = accept(node, 0)        # late acceptance of the old goal -> orphan
    drain(node)
    old_handle.result_future.resolve(NS(status=SUCCEEDED))
    assert nav_messages(node) == []
    assert node.store.snapshot()['nav']['state'] == 'active'
    assert node._nav_token == new_id
    assert node._nav_orphans == {}


def test_response_of_an_old_goal_does_not_become_the_current_handle():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    node.clock[0] = 106.0
    node._nav_watchdog()
    node.send_goal(3.0, 4.0, 0.0)
    old_handle = accept(node, 0)       # late acceptance of goal 1
    assert node._nav_handle is None
    assert old_handle.cancel_calls == 1
    assert node.store.snapshot()['nav']['state'] == 'active'


def test_feedback_of_an_old_goal_is_ignored():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    node.clock[0] = 106.0
    node._nav_watchdog()
    node.send_goal(3.0, 4.0, 0.0)
    drain(node)
    node._nav_client.feedback_callbacks[0](feedback(1.0))
    assert nav_messages(node) == []
    assert node.store.snapshot()['nav']['distance_remaining'] is None


def test_finish_with_unknown_token_is_ignored():
    node = make_nav_bridge()
    node.send_goal(1.0, 2.0, 0.5)
    drain(node)
    node._finish_goal('not-the-current-token', 'failed', 'x')
    assert nav_messages(node) == []
    assert node._nav_token is not None


# --- feedback: <= 2 Hz and NaN -> null ---

def test_feedback_publishes_distance_at_most_every_half_second():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    accept(node)
    drain(node)
    callback = node._nav_client.feedback_callbacks[0]
    sent = []
    for step in range(20):              # 10 Hz of feedback during 2 s
        node.clock[0] = 100.0 + step * 0.1
        callback(feedback(10.0 - step))
        sent.extend(nav_messages(node))
    assert 3 <= len(sent) <= 4          # 0, 0.5, 1.0, 1.5 s
    assert all(m['state'] == 'active' for m in sent)
    assert [m['distance_remaining'] for m in sent][:2] == [10.0, 5.0]


def test_feedback_exactly_at_half_second_is_published_and_just_before_is_not():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    drain(node)
    callback = node._nav_client.feedback_callbacks[0]
    seen = []
    callback(feedback(5.0))
    seen += nav_messages(node)
    node.clock[0] = 100.49
    callback(feedback(4.0))
    seen += nav_messages(node)
    node.clock[0] = 100.5
    callback(feedback(3.0))
    seen += nav_messages(node)
    assert [m['distance_remaining'] for m in seen] == [5.0, 3.0]


@pytest.mark.parametrize('value', [float('nan'), float('inf'), float('-inf')])
def test_feedback_non_finite_distance_becomes_null(value):
    node = make_nav_bridge()
    node.send_goal(1.0, 2.0, 0.5)
    drain(node)
    node._nav_client.feedback_callbacks[0](feedback(value))
    messages = nav_messages(node)
    assert len(messages) == 1
    assert messages[0]['distance_remaining'] is None
    assert node.store.snapshot()['nav']['distance_remaining'] is None


def test_feedback_distance_is_rounded_to_centimetres():
    node = make_nav_bridge()
    node.send_goal(1.0, 2.0, 0.5)
    drain(node)
    node._nav_client.feedback_callbacks[0](feedback(1.23456))
    assert nav_messages(node)[0]['distance_remaining'] == 1.23


def test_feedback_after_the_goal_ended_is_ignored():
    node = make_nav_bridge()
    node.send_goal(1.0, 2.0, 0.5)
    handle = accept(node)
    handle.result_future.resolve(NS(status=SUCCEEDED))
    drain(node)
    node._nav_client.feedback_callbacks[0](feedback(1.0))
    assert nav_messages(node) == []


def test_final_state_resets_distance_to_null():
    node = make_nav_bridge()
    node.send_goal(1.0, 2.0, 0.5)
    handle = accept(node)
    node._nav_client.feedback_callbacks[0](feedback(2.0))
    handle.result_future.resolve(NS(status=SUCCEEDED))
    assert node.store.snapshot()['nav'] == {'state': 'succeeded', 'distance_remaining': None}


# --- initial pose and map for validation ---

def _pose_msg_factory():
    msg = NS(header=NS(frame_id='', stamp=None),
             pose=NS(pose=NS(position=NS(x=0.0, y=0.0, z=0.0),
                             orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
                     covariance=[0.0] * 36))
    return lambda: msg


def test_publish_initial_pose_fills_frame_pose_and_covariance_and_publishes_once():
    node = make_nav_bridge()
    node._initial_pose_pub = mock.Mock()
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: 'STAMP'))
    with mock.patch.object(bridge, 'PoseWithCovarianceStamped', _pose_msg_factory()):
        node.publish_initial_pose(1.5, -2.0, 0.0)
    node._initial_pose_pub.publish.assert_called_once()
    msg = node._initial_pose_pub.publish.call_args.args[0]
    assert msg.header.frame_id == 'map'
    assert msg.header.stamp == 'STAMP'
    assert (msg.pose.pose.position.x, msg.pose.pose.position.y) == (1.5, -2.0)
    assert msg.pose.pose.orientation.w == pytest.approx(1.0)
    assert msg.pose.covariance[0] == bridge.INITIAL_COVARIANCE_XY
    assert msg.pose.covariance[7] == bridge.INITIAL_COVARIANCE_XY
    assert msg.pose.covariance[35] == bridge.INITIAL_COVARIANCE_YAW
    assert sum(1 for v in msg.pose.covariance if v) == 3


def test_publish_initial_pose_orientation_follows_yaw():
    node = make_nav_bridge()
    node._initial_pose_pub = mock.Mock()
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: 'STAMP'))
    with mock.patch.object(bridge, 'PoseWithCovarianceStamped', _pose_msg_factory()):
        node.publish_initial_pose(0.0, 0.0, 1.0)
    q = node._initial_pose_pub.publish.call_args.args[0].pose.pose.orientation
    assert (q.z, q.w) == (pytest.approx(0.479426, abs=1e-5), pytest.approx(0.877583, abs=1e-5))


def test_initial_pose_does_not_send_goals_or_touch_nav_state():
    node = make_nav_bridge()
    node._initial_pose_pub = mock.Mock()
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: 'STAMP'))
    with mock.patch.object(bridge, 'PoseWithCovarianceStamped', _pose_msg_factory()):
        node.publish_initial_pose(0.0, 0.0, 0.0)
    assert node._nav_client.futures == []
    assert nav_messages(node) == []


def test_on_map_stores_the_grid_with_cells_for_goal_validation():
    node = make_nav_bridge()
    node._on_map(grid())
    stored = node.store.grid()
    assert (stored['width'], stored['height'], stored['resolution']) == (2, 2, 0.05)
    assert list(stored['data']) == [0, -1, 100, 0]
    assert (stored['origin_x'], stored['origin_y']) == (0.0, 0.0)


@pytest.mark.parametrize('bad', [
    grid(frame='odom'), grid(width=3), grid(width=0), grid(resolution=0.0),
    grid(resolution=float('nan')), grid(data=[0, 0, 0])])
def test_on_map_invalid_map_clears_the_stored_grid(bad):
    node = make_nav_bridge()
    node._on_map(grid())
    assert node.store.grid() is not None
    node._on_map(bad)
    assert node.store.grid() is None


def _publisher_topics_in_source():
    """Topic argument (literal) of every create_publisher(...) call found in the source."""
    import ast
    import inspect
    tree = ast.parse(inspect.getsource(bridge))
    topics = []
    for call in ast.walk(tree):
        if isinstance(call, ast.Call) and getattr(call.func, 'attr', None) == 'create_publisher':
            topic = call.args[1]
            assert isinstance(topic, ast.Constant), 'publisher topic must be a literal'
            topics.append(topic.value)
    return topics


def test_node_source_never_publishes_on_a_bare_cmd_vel():
    import inspect
    import re
    source = inspect.getsource(bridge)
    assert re.search(r"create_publisher\([^)]*['\"]cmd_vel['\"]", source) is None
    assert re.search(r"create_publisher\([^)]*['\"]/?cmd_vel['\"]", source) is None


def test_the_only_cmd_vel_publisher_in_the_source_is_cmd_vel_nav():
    topics = _publisher_topics_in_source()
    assert sorted(topics) == ['cmd_vel_nav', 'initialpose']
    assert [t for t in topics if t.startswith('cmd_vel')] == ['cmd_vel_nav']


def test_node_source_has_no_literal_cmd_vel_topic_and_no_twist_stamped():
    import ast
    import inspect
    source = inspect.getsource(bridge)
    tree = ast.parse(source)
    literals = [n.value for n in ast.walk(tree) if isinstance(n, ast.Constant)
                and isinstance(n.value, str)]
    assert 'cmd_vel' not in literals and '/cmd_vel' not in literals
    names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    names |= {a.name for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)
              for a in n.names}
    assert 'TwistStamped' not in names and 'Twist' in names


# --- Fail-safe handling of accepted goals without a Nav2 result (phase 3 safety fixes) --------
# Rule under test: "the safe failure is to stay busy". An ACCEPTED goal that left the current
# slot without a Nav2 result (orphan) is cancelled again and again, no new goal is accepted
# until Nav2 confirms its end (result, or cancel return code 2/3).

def make_orphan(node):
    """Send, accept and let an unanswered cancel expire: returns the orphaned handle."""
    node.send_goal(1.0, 2.0, 0.5)
    handle = accept(node)
    node.cancel()
    node.clock[0] += bridge.NAV_RESPONSE_TIMEOUT_S + 0.1
    node._nav_watchdog()
    drain(node)
    assert node._nav_token is None and len(node._nav_orphans) == 1
    return handle


def assert_busy_with_orphans(node):
    sent = len(node._nav_client.futures)
    with pytest.raises(bridge.NavBusyError) as info:
        node.send_goal(9.0, 9.0, 0.0)
    assert str(info.value) == (
        'A previous goal has not been confirmed finished by Nav2; cancel and wait')
    assert len(node._nav_client.futures) == sent
    assert nav_messages(node) == []


def test_orphan_constants_are_the_specified_ones():
    assert bridge.NAV_CANCEL_RETRY_S == 1.0
    assert bridge.MAX_ORPHAN_GOALS == 8
    assert (bridge.CANCEL_ERROR_NONE, bridge.CANCEL_ERROR_REJECTED,
            bridge.CANCEL_ERROR_UNKNOWN_GOAL_ID, bridge.CANCEL_ERROR_GOAL_TERMINATED) == (
        0, 1, 2, 3)


def test_send_goal_with_orphans_raises_busy_sends_nothing_and_publishes_nothing():
    node = make_nav_bridge(now=100.0)
    make_orphan(node)
    assert_busy_with_orphans(node)
    assert node.store.snapshot()['nav']['state'] == 'failed'


# cancel_goal_async raising -> retried by the watchdog every second (orphan)

def test_orphan_cancel_failure_does_not_release_and_is_retried_every_second():
    node = make_nav_bridge(now=100.0)
    handle = make_orphan(node)
    handle.cancel_error = RuntimeError('bus down')
    base = handle.cancel_calls
    node.clock[0] += 0.9
    node._nav_watchdog()
    assert handle.cancel_calls == base           # not before NAV_CANCEL_RETRY_S
    node.clock[0] += 0.2
    node._nav_watchdog()
    assert handle.cancel_calls == base + 1       # raised, but still tracked
    node.clock[0] += 1.0
    node._nav_watchdog()
    assert handle.cancel_calls == base + 2
    assert len(node._nav_orphans) == 1
    assert_busy_with_orphans(node)
    node.logger.error.assert_called()


def test_cancel_goal_async_future_that_never_resolves_is_retried_by_time_not_by_ticks():
    node = make_nav_bridge(now=100.0)
    handle = make_orphan(node)
    base = handle.cancel_calls
    for _ in range(6):                           # 3 s of 0.5 s watchdog ticks
        node.clock[0] += bridge.NAV_WATCHDOG_PERIOD_S
        node._nav_watchdog()
    assert handle.cancel_calls == base + 3
    assert len(node._nav_orphans) == 1
    assert_busy_with_orphans(node)


# _on_cancel_response return codes

def test_cancel_return_code_0_keeps_the_orphan_and_waits_for_the_result():
    node = make_nav_bridge(now=100.0)
    handle = make_orphan(node)
    base = handle.cancel_calls
    handle.cancel_futures[-1].resolve(NS(return_code=0))
    assert handle.cancel_calls == base           # no immediate retry
    assert len(node._nav_orphans) == 1
    assert_busy_with_orphans(node)
    handle.result_future.resolve(NS(status=CANCELED))
    assert node._nav_orphans == {}
    assert nav_messages(node) == []
    node.send_goal(3.0, 4.0, 0.0)


@pytest.mark.parametrize('code', [1, 4, 99])
def test_cancel_return_code_rejected_or_unknown_keeps_the_orphan_and_retries(code):
    node = make_nav_bridge(now=100.0)
    handle = make_orphan(node)
    base = handle.cancel_calls
    handle.cancel_futures[-1].resolve(NS(return_code=code))
    assert len(node._nav_orphans) == 1
    assert_busy_with_orphans(node)
    node.clock[0] += bridge.NAV_CANCEL_RETRY_S
    node._nav_watchdog()
    assert handle.cancel_calls == base + 1


@pytest.mark.parametrize('code', [2, 3])
def test_cancel_return_code_unknown_goal_or_terminated_retires_the_orphan(code):
    node = make_nav_bridge(now=100.0)
    handle = make_orphan(node)
    base = handle.cancel_calls
    handle.cancel_futures[-1].resolve(NS(return_code=code))
    assert node._nav_orphans == {}
    assert nav_messages(node) == []              # confirmation is not a nav event
    node.clock[0] += 5.0
    node._nav_watchdog()
    assert handle.cancel_calls == base           # nothing left to cancel
    node.send_goal(3.0, 4.0, 0.0)                # the node is free again
    assert node.store.snapshot()['nav']['state'] == 'active'


@pytest.mark.parametrize('code', [2, 3])
def test_cancel_return_code_2_or_3_does_not_release_the_current_goal(code):
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    handle = accept(node)
    node.cancel()
    drain(node)
    handle.cancel_futures[-1].resolve(NS(return_code=code))
    assert node._nav_token is not None           # only the action result ends a current goal
    assert nav_messages(node) == []
    with pytest.raises(bridge.NavBusyError):
        node.send_goal(3.0, 4.0, 0.0)
    handle.result_future.resolve(NS(status=CANCELED))
    assert [m['state'] for m in nav_messages(node)] == ['canceled']


def test_cancel_response_exception_keeps_the_orphan_and_retries():
    node = make_nav_bridge(now=100.0)
    handle = make_orphan(node)
    base = handle.cancel_calls
    handle.cancel_futures[-1].resolve(error=RuntimeError('lost answer'))
    assert len(node._nav_orphans) == 1
    assert_busy_with_orphans(node)
    node.clock[0] += bridge.NAV_CANCEL_RETRY_S
    node._nav_watchdog()
    assert handle.cancel_calls == base + 1


def test_cancel_return_code_for_an_already_retired_orphan_is_harmless():
    node = make_nav_bridge(now=100.0)
    handle = make_orphan(node)
    handle.result_future.resolve(NS(status=CANCELED))
    assert node._nav_orphans == {}
    handle.cancel_futures[-1].resolve(NS(return_code=3))
    node.send_goal(3.0, 4.0, 0.0)


# deadlines and server loss with an ACCEPTED goal vs. without handle

def test_cancel_unanswered_5s_with_accepted_goal_fails_and_keeps_orphan_with_retries():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    handle = accept(node)
    node.clock[0] = 200.0
    node.cancel()
    drain(node)
    node.clock[0] = 205.1
    node._nav_watchdog()
    assert [m['state'] for m in nav_messages(node)] == ['failed']
    assert len(node._nav_orphans) == 1
    assert_busy_with_orphans(node)
    calls = handle.cancel_calls
    node.clock[0] = 206.2
    node._nav_watchdog()
    assert handle.cancel_calls == calls + 1


def test_server_lost_3s_with_accepted_goal_fails_and_keeps_orphan_with_retries():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    handle = accept(node)
    drain(node)
    node._nav_client.ready = False
    node._nav_watchdog()
    node.clock[0] = 103.1
    node._nav_watchdog()
    assert [m['state'] for m in nav_messages(node)] == ['failed']
    assert node._nav_token is None
    assert len(node._nav_orphans) == 1
    assert handle.cancel_calls >= 1              # cancelled as soon as it is orphaned
    node._nav_client.ready = True
    assert_busy_with_orphans(node)
    calls = handle.cancel_calls
    node.clock[0] = 104.2
    node._nav_watchdog()
    assert handle.cancel_calls == calls + 1


def test_cancel_unanswered_5s_without_handle_fails_and_releases_the_node():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)               # Nav2 never answered: no handle
    node.clock[0] = 103.0
    node.cancel()
    drain(node)
    node.clock[0] = 107.9
    node._nav_watchdog()
    assert nav_messages(node) == []
    node.clock[0] = 108.1
    node._nav_watchdog()
    assert [m['state'] for m in nav_messages(node)] == ['failed']
    assert node._nav_token is None
    assert node._nav_orphans == {}
    node.send_goal(3.0, 4.0, 0.0)


def test_server_lost_3s_without_handle_fails_and_releases_the_node():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    drain(node)
    node._nav_client.ready = False
    node._nav_watchdog()
    node.clock[0] = 103.1
    node._nav_watchdog()
    assert [m['state'] for m in nav_messages(node)] == ['failed']
    assert node._nav_orphans == {}
    node._nav_client.ready = True
    node.send_goal(3.0, 4.0, 0.0)


# cancel() (/api/cancel)

def test_cancel_without_current_goal_but_with_orphans_cancels_each_orphan():
    node = make_nav_bridge(now=100.0)
    first = make_orphan(node)
    second = FakeHandle()
    with node._nav_lock:
        node._add_orphan_locked('second-token', second)
    base = first.cancel_calls
    node.cancel()
    assert first.cancel_calls == base + 1
    assert second.cancel_calls == 1
    assert nav_messages(node) == []
    assert node._nav_token is None
    assert len(node._nav_orphans) == 2           # cancelling does not release them


def assert_cancel_all_request(request, stamp_ns=None):
    """goal_id all zeros + stamp = decision instant (>= 1 ns), as sec/nanosec."""
    assert list(request.goal_info.goal_id.uuid) == [0] * 16
    stamp = request.goal_info.stamp
    assert 0 <= stamp.nanosec < 10**9
    total = stamp.sec * 10**9 + stamp.nanosec
    assert total >= 1
    if stamp_ns is not None:
        assert total == stamp_ns


def test_cancel_without_goal_and_without_orphans_sends_one_cancel_all():
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    node.cancel()                                 # must not raise NavBusy/NavUnavailable
    assert len(client.requests) == 1
    assert_cancel_all_request(client.requests[0])
    assert nav_messages(node) == []
    node.logger.info.assert_called()
    node.logger.error.assert_not_called()
    assert node._nav_token is None and node._nav_orphans == {}
    assert node._cancel_all is not None


def test_cancel_all_cancel_without_goal_is_idempotent_and_never_raises():
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    node.cancel()
    node.cancel()
    assert len(client.requests) == 2
    for request in client.requests:
        assert_cancel_all_request(request)
    assert node._cancel_all['attempts'] == 1
    assert node._nav_token is None and node._nav_orphans == {}
    node.logger.error.assert_not_called()


def test_cancel_with_orphan_whose_cancel_raises_does_not_propagate():
    node = make_nav_bridge(now=100.0)
    handle = make_orphan(node)
    handle.cancel_error = RuntimeError('boom')
    node.cancel()                                # must not raise (HTTP handler)
    assert len(node._nav_orphans) == 1


def test_cancel_also_cancels_orphans_while_a_current_goal_is_active():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    current = accept(node)
    extra = FakeHandle()
    with node._nav_lock:
        node._add_orphan_locked('old-token', extra)
    node.cancel()
    assert current.cancel_calls == 1
    assert extra.cancel_calls == 1


# results, feedback and late acceptance of orphans

@pytest.mark.parametrize('status', [SUCCEEDED, CANCELED, ABORTED])
def test_orphan_result_retires_it_without_publishing_and_new_goals_are_accepted(status):
    node = make_nav_bridge(now=100.0)
    handle = make_orphan(node)
    handle.result_future.resolve(NS(status=status))
    assert node._nav_orphans == {}
    assert nav_messages(node) == []
    assert node.store.snapshot()['nav']['state'] == 'failed'
    node.send_goal(3.0, 4.0, 0.0)
    accept(node, 1).result_future.resolve(NS(status=SUCCEEDED))
    assert node.store.snapshot()['nav']['state'] == 'succeeded'


def test_orphan_result_error_keeps_the_orphan_tracked():
    node = make_nav_bridge(now=100.0)
    handle = make_orphan(node)
    handle.result_future.resolve(error=RuntimeError('gone'))
    assert len(node._nav_orphans) == 1
    assert_busy_with_orphans(node)


def test_feedback_of_an_orphan_triggers_a_cancel_limited_to_once_per_second():
    node = make_nav_bridge(now=100.0)
    handle = make_orphan(node)
    callback = node._nav_client.feedback_callbacks[0]
    base = handle.cancel_calls
    callback(feedback(1.0))                      # retry time not reached yet
    assert handle.cancel_calls == base
    node.clock[0] += bridge.NAV_CANCEL_RETRY_S
    callback(feedback(1.0))
    callback(feedback(1.0))                      # same instant: limited
    assert handle.cancel_calls == base + 1
    node.clock[0] += 0.5
    callback(feedback(1.0))
    assert handle.cancel_calls == base + 1
    node.clock[0] += 0.5
    callback(feedback(1.0))
    assert handle.cancel_calls == base + 2
    assert nav_messages(node) == []              # orphan feedback is never published
    assert node.store.snapshot()['nav']['distance_remaining'] is None


def test_late_acceptance_after_timeout_keeps_the_node_busy_until_nav2_confirms():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    node.clock[0] = 106.0
    node._nav_watchdog()
    drain(node)
    handle = accept(node)
    assert handle.cancel_calls == 1
    assert_busy_with_orphans(node)
    handle.cancel_futures[-1].resolve(NS(return_code=0))
    assert_busy_with_orphans(node)               # accepted cancel is not a confirmation
    handle.result_future.resolve(NS(status=CANCELED))
    assert nav_messages(node) == []
    node.send_goal(3.0, 4.0, 0.0)


def test_late_acceptance_confirmed_by_cancel_code_terminated_frees_the_node():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    node.clock[0] = 106.0
    node._nav_watchdog()
    handle = accept(node)
    handle.cancel_futures[-1].resolve(NS(return_code=3))
    node.send_goal(3.0, 4.0, 0.0)


def test_late_acceptance_when_get_result_async_fails_still_cancels_and_stays_busy():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    node.clock[0] = 106.0
    node._nav_watchdog()
    drain(node)
    handle = FakeHandle()
    handle.result_error = RuntimeError('no result channel')
    node._nav_client.futures[0].resolve(handle)
    assert handle.cancel_calls >= 1
    assert_busy_with_orphans(node)


def test_get_result_async_error_makes_the_goal_failed_orphan_and_cancels_it():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    drain(node)
    handle = FakeHandle()
    handle.result_error = RuntimeError('no result channel')
    node._nav_client.futures[0].resolve(handle)
    assert [m['state'] for m in nav_messages(node)] == ['failed']
    assert node.store.snapshot()['nav']['state'] == 'failed'
    assert node._nav_token is None
    assert len(node._nav_orphans) == 1
    assert handle.cancel_calls == 1
    node.logger.error.assert_called()
    assert_busy_with_orphans(node)


def test_result_future_error_orphan_is_cancelled_by_the_watchdog():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    handle = accept(node)
    handle.result_future.resolve(error=RuntimeError('gone'))
    node._nav_watchdog()
    assert handle.cancel_calls == 1
    node.clock[0] += bridge.NAV_CANCEL_RETRY_S
    node._nav_watchdog()
    assert handle.cancel_calls == 2


# message order

def test_late_feedback_never_republishes_active_after_a_final_state():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    handle = accept(node)
    callback = node._nav_client.feedback_callbacks[0]
    handle.result_future.resolve(NS(status=SUCCEEDED))
    drain(node)
    node.clock[0] += 10.0
    callback(feedback(2.0))
    assert nav_messages(node) == []
    assert node.store.snapshot()['nav'] == {'state': 'succeeded', 'distance_remaining': None}


# orphan table bound

def test_orphan_table_full_drops_the_oldest_and_logs_an_error():
    node = make_nav_bridge(now=100.0)
    with node._nav_lock:
        for index in range(bridge.MAX_ORPHAN_GOALS):
            node._add_orphan_locked(f'orphan-{index}', FakeHandle())
    assert len(node._nav_orphans) == bridge.MAX_ORPHAN_GOALS
    node.logger.error.assert_not_called()
    with node._nav_lock:
        node._add_orphan_locked('orphan-new', FakeHandle())
    assert len(node._nav_orphans) == bridge.MAX_ORPHAN_GOALS
    assert 'orphan-0' not in node._nav_orphans
    assert 'orphan-1' in node._nav_orphans and 'orphan-new' in node._nav_orphans
    node.logger.error.assert_called_once()
    assert 'orphan-0' in node.logger.error.call_args[0][0]


def test_orphan_table_full_does_not_drop_anything_when_re_adding_a_known_token():
    node = make_nav_bridge(now=100.0)
    with node._nav_lock:
        for index in range(bridge.MAX_ORPHAN_GOALS):
            node._add_orphan_locked(f'orphan-{index}', FakeHandle())
        node._add_orphan_locked('orphan-3', FakeHandle())
    assert sorted(node._nav_orphans) == sorted(
        f'orphan-{index}' for index in range(bridge.MAX_ORPHAN_GOALS))
    node.logger.error.assert_not_called()


# nav messages are published under _nav_lock

def test_every_nav_publication_happens_while_holding_the_nav_lock():
    node = make_nav_bridge(now=100.0)
    original = node._publish_nav
    held = []

    def spy(state, distance):
        held.append((state, node._nav_lock.locked()))
        original(state, distance)

    node._publish_nav = spy
    node.send_goal(1.0, 2.0, 0.5)                                    # active
    callback = node._nav_client.feedback_callbacks[0]
    accept(node)
    callback(feedback(1.5))                                          # feedback
    node._nav_client.futures[0].resolve(FakeHandle(accepted=False))  # late: ignored
    node.cancel()
    node.clock[0] += bridge.NAV_RESPONSE_TIMEOUT_S + 0.1
    node._nav_watchdog()                                             # detach -> failed
    node._nav_orphans.clear()
    node.send_goal(3.0, 4.0, 0.0)
    node.clock[0] += bridge.NAV_RESPONSE_TIMEOUT_S + 0.1
    node._nav_watchdog()                                             # unaccepted -> failed
    node.send_goal(5.0, 6.0, 0.0)
    node._nav_client.futures[-1].resolve(FakeHandle(accepted=False))  # rejected -> failed
    node._nav_client.send_error = RuntimeError('x')
    with pytest.raises(bridge.NavUnavailableError):
        node.send_goal(7.0, 8.0, 0.0)                                # send failure -> failed
    states = [state for state, _ in held]
    assert states.count('active') >= 4 and states.count('failed') >= 4
    assert all(locked for _, locked in held)


# --- real threads: races between HTTP threads and executor callbacks ---

def run_threads(*functions):
    """Run the callables at the same time; every join has a timeout (no sleeps)."""
    errors = []
    barrier = threading.Barrier(len(functions), timeout=5.0)

    def wrap(function):
        def run():
            try:
                barrier.wait()
                function()
            except Exception as err:  # noqa: B902
                errors.append(err)
        return run

    threads = [threading.Thread(target=wrap(f), daemon=True) for f in functions]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=5.0)
    assert not any(thread.is_alive() for thread in threads), 'thread did not finish'
    assert errors == []


RACE_ROUNDS = 40


def test_threads_cancel_result_and_watchdog_race_ends_consistent():
    for _ in range(RACE_ROUNDS):
        node = make_nav_bridge(now=100.0)
        node.send_goal(1.0, 2.0, 0.5)
        handle = accept(node)
        node.clock[0] = 200.0                    # past any deadline cancel() may set
        drain(node)
        run_threads(node.cancel, _watchdog_after(node, 10.0),
                    lambda h=handle: h.result_future.resolve(NS(status=CANCELED)))
        assert node._nav_token is None
        assert node._nav_orphans == {}           # the result retires an orphan, if any
        assert node.store.snapshot()['nav']['state'] in ('canceled', 'failed')
        states = [m['state'] for m in nav_messages(node)]
        assert states in (['canceled'], ['failed'])  # one final state, never both
        node.send_goal(3.0, 4.0, 0.0)            # the node ends free


def _watchdog_after(node, seconds):
    def run():
        node.clock[0] += seconds
        node._nav_watchdog()
    return run


def test_threads_feedback_and_result_never_leave_active_after_a_final_state():
    for _ in range(RACE_ROUNDS):
        node = make_nav_bridge(now=100.0)
        node.send_goal(1.0, 2.0, 0.5)
        handle = accept(node)
        callback = node._nav_client.feedback_callbacks[0]
        drain(node)
        run_threads(lambda c=callback: c(feedback(1.0)),
                    lambda h=handle: h.result_future.resolve(NS(status=SUCCEEDED)))
        assert node.store.snapshot()['nav']['state'] == 'succeeded'
        assert nav_messages(node)[-1]['state'] == 'succeeded'


def test_threads_concurrent_send_goal_accepts_exactly_one():
    node = make_nav_bridge(now=100.0)
    outcomes = []

    def attempt():
        try:
            outcomes.append(node.send_goal(1.0, 2.0, 0.5))
        except bridge.NavBusyError:
            outcomes.append('busy')

    run_threads(*[attempt] * 8)
    assert len([o for o in outcomes if o != 'busy']) == 1
    assert outcomes.count('busy') == 7
    assert len(node._nav_client.futures) == 1


def test_threads_cancel_and_orphan_result_race_never_leaves_a_stuck_orphan():
    for _ in range(RACE_ROUNDS):
        node = make_nav_bridge(now=100.0)
        handle = make_orphan(node)
        run_threads(node.cancel, _watchdog_after(node, 2.0),
                    lambda h=handle: h.result_future.resolve(NS(status=CANCELED)))
        assert node._nav_orphans == {}
        node.send_goal(3.0, 4.0, 0.0)


# --- cancel-all safety net (goal without handle released by timeout / lost server) ---

def release_by_timeout(node):
    """Send a goal Nav2 never answers and let the watchdog release it (tick 1 sends)."""
    node.send_goal(1.0, 2.0, 0.5)
    node.clock[0] += bridge.NAV_RESPONSE_TIMEOUT_S + 0.1
    node._nav_watchdog()
    assert node._nav_token is None


def tick(node, seconds):
    node.clock[0] += seconds
    node._nav_watchdog()


def test_goal_released_by_timeout_arms_and_sends_one_cancel_all():
    node = make_nav_bridge(now=100.0)
    release_by_timeout(node)
    client = cancel_client(node)
    assert len(client.requests) == 1
    assert_cancel_all_request(client.requests[0])
    assert node._nav_orphans == {}


def test_goal_released_by_server_lost_arms_and_sends_cancel_all():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    node._nav_client.ready = False
    node._nav_watchdog()                          # starts the lost timer
    assert cancel_client(node).requests == []
    tick(node, bridge.NAV_SERVER_LOST_TIMEOUT_S + 0.1)
    assert node._nav_token is None
    assert len(cancel_client(node).requests) == 1
    assert_cancel_all_request(cancel_client(node).requests[0])


@pytest.mark.parametrize('code', [
    bridge.CANCEL_ERROR_NONE, bridge.CANCEL_ERROR_UNKNOWN_GOAL_ID,
    bridge.CANCEL_ERROR_GOAL_TERMINATED])
def test_cancel_all_answered_is_not_repeated(code):
    node = make_nav_bridge(now=100.0)
    release_by_timeout(node)
    client = cancel_client(node)
    client.answer(0, code)
    assert node._cancel_all is None
    for _ in range(bridge.MAX_CANCEL_ALL_ATTEMPTS + 2):
        tick(node, bridge.NAV_CANCEL_RETRY_S + 0.1)
    assert len(client.requests) == 1
    node.logger.error.assert_not_called()


def test_cancel_all_is_not_repeated_before_the_retry_period():
    node = make_nav_bridge(now=100.0)
    release_by_timeout(node)
    tick(node, bridge.NAV_CANCEL_RETRY_S / 2)
    tick(node, bridge.NAV_CANCEL_RETRY_S / 4)
    assert len(cancel_client(node).requests) == 1
    tick(node, bridge.NAV_CANCEL_RETRY_S)
    assert len(cancel_client(node).requests) == 2


def _answer_rejected(client, index):
    client.answer(index, bridge.CANCEL_ERROR_REJECTED)


def _answer_error(client, index):
    client.futures[index].resolve(error=RuntimeError('service died'))


def _answer_none_at_all(client, index):
    return None


@pytest.mark.parametrize('responder', [_answer_rejected, _answer_error, _answer_none_at_all])
def test_cancel_all_retries_until_the_limit_then_gives_up_with_manual_stop_error(responder):
    node = make_nav_bridge(now=100.0)
    release_by_timeout(node)
    client = cancel_client(node)
    responder(client, 0)
    for index in range(1, bridge.MAX_CANCEL_ALL_ATTEMPTS):
        tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
        assert len(client.requests) == index + 1
        responder(client, index)
    node.logger.error.assert_not_called()          # still inside the allowed attempts
    tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert len(client.requests) == bridge.MAX_CANCEL_ALL_ATTEMPTS
    assert node._cancel_all is None
    node.logger.error.assert_called_once()
    assert 'STOP THE ROBOT MANUALLY' in node.logger.error.call_args[0][0]
    for _ in range(3):
        tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert len(client.requests) == bridge.MAX_CANCEL_ALL_ATTEMPTS
    node.logger.error.assert_called_once()


def test_cancel_all_call_async_exception_is_retried_then_given_up():
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    client.call_error = RuntimeError('service gone')
    release_by_timeout(node)                       # must not raise
    node.logger.error.assert_called()              # the failure is logged
    for _ in range(bridge.MAX_CANCEL_ALL_ATTEMPTS + 2):
        tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert node._cancel_all is None
    messages = [call[0][0] for call in node.logger.error.call_args_list]
    assert any('STOP THE ROBOT MANUALLY' in m for m in messages)
    assert client.requests == []


def test_cancel_all_call_async_exception_recovers_when_service_comes_back():
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    client.call_error = RuntimeError('service gone')
    release_by_timeout(node)
    client.call_error = None
    tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert len(client.requests) == 1
    assert_cancel_all_request(client.requests[0])


def test_cancel_all_service_not_ready_sends_nothing_and_consumes_no_attempts():
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    client.ready = False
    release_by_timeout(node)
    for _ in range(bridge.MAX_CANCEL_ALL_ATTEMPTS + 3):
        tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert client.requests == []
    assert node._cancel_all is not None                  # still pending, not given up
    assert node._cancel_all['attempts'] == 0
    node.logger.error.assert_not_called()
    client.ready = True
    tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert len(client.requests) == 1
    assert_cancel_all_request(client.requests[0])


def test_cancel_with_service_not_ready_sends_nothing_and_does_not_raise():
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    client.ready = False
    node.cancel()
    assert client.requests == []
    node.logger.error.assert_not_called()
    client.ready = True
    tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert len(client.requests) == 1


def test_send_goal_clears_pending_cancel_all_and_it_never_cancels_the_new_goal():
    node = make_nav_bridge(now=100.0)
    release_by_timeout(node)
    client = cancel_client(node)
    assert len(client.requests) == 1               # unanswered, still pending
    node.send_goal(3.0, 4.0, 0.0)
    assert node._cancel_all is None
    for _ in range(bridge.MAX_CANCEL_ALL_ATTEMPTS + 2):
        tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
        if node._nav_token is None:
            break
        node._nav_deadline_s = None                # keep the new goal alive
    assert len(client.requests) == 1


def test_watchdog_never_sends_cancel_all_while_a_goal_is_active():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    node._cancel_all = {'retry_s': 0.0, 'attempts': 0, 'stamp_ns': 1}   # stale pending request
    node._nav_watchdog()
    assert cancel_client(node).requests == []
    assert node._cancel_all is None


def test_cancel_all_not_sent_with_active_goal_with_handle():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    handle = accept(node)
    tick(node, 0.5)
    node.cancel()
    assert cancel_client(node).requests == []
    assert handle.cancel_calls == 1


def test_cancel_all_not_armed_when_orphans_exist():
    node = make_nav_bridge(now=100.0)
    handle = make_orphan(node)
    node.cancel()
    for _ in range(4):
        tick(node, bridge.NAV_CANCEL_RETRY_S + 0.1)
    assert cancel_client(node).requests == []
    assert node._cancel_all is None
    assert handle.cancel_calls >= 2                 # the orphan is managed by cancel()/watchdog


def test_no_nav2_call_is_made_while_holding_the_nav_lock():
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    held = []
    client.lock_probe = lambda: held.append(('call_async', node._nav_lock.locked()))
    node.cancel()                                           # cancel-all from cancel()
    release_by_timeout(node)                                # cancel-all from the watchdog
    tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)            # retry
    # handle cancels (current goal and orphan)
    node.send_goal(5.0, 6.0, 0.0)
    handle = accept(node, index=len(node._nav_client.futures) - 1)
    original = handle.cancel_goal_async

    def spy():
        held.append(('cancel_goal_async', node._nav_lock.locked()))
        return original()

    handle.cancel_goal_async = spy
    node.cancel()
    tick(node, bridge.NAV_RESPONSE_TIMEOUT_S + 0.1)         # detach -> orphan
    tick(node, bridge.NAV_CANCEL_RETRY_S + 0.1)
    kinds = {kind for kind, _ in held}
    assert kinds == {'call_async', 'cancel_goal_async'}
    assert held and not any(locked for _, locked in held)


def test_goal_released_by_timeout_and_accepted_late_goes_to_orphan_path():
    node = make_nav_bridge(now=100.0)
    release_by_timeout(node)
    client = cancel_client(node)
    late = accept(node)                                     # Nav2 accepts after the release
    assert len(node._nav_orphans) == 1
    assert late.cancel_calls == 1                           # cancelled through its own handle
    assert node._nav_token is None
    late.result_future.resolve(NS(status=CANCELED))
    assert node._nav_orphans == {}
    client.answer(0, bridge.CANCEL_ERROR_NONE)
    assert node._cancel_all is None
    tick(node, bridge.NAV_CANCEL_RETRY_S + 0.1)
    assert len(client.requests) == 1
    assert late.cancel_calls == 1
    assert node.send_goal(7.0, 8.0, 0.0)                    # free again


# --- cancel-all: attempts counted only for requests really sent (safe behaviour) ---

def _errors(node):
    return [call[0][0] for call in node.logger.error.call_args_list]


def _warnings(node):
    return [call[0][0] for call in node.logger.warning.call_args_list]


def _pending_warnings(node):
    return [m for m in _warnings(node) if 'Cancel-all still pending' in m]


def test_cancel_arms_cancel_all_and_counts_the_attempt_when_sent():
    node = make_nav_bridge(now=100.0)
    node.cancel()
    assert len(cancel_client(node).requests) == 1
    assert node._cancel_all['attempts'] == 1
    assert node._cancel_all['retry_s'] == pytest.approx(100.0 + bridge.NAV_CANCEL_RETRY_S)


def test_cancel_arms_cancel_all_with_zero_attempts_when_service_not_ready():
    node = make_nav_bridge(now=100.0)
    cancel_client(node).ready = False
    node.cancel()
    assert node._cancel_all['attempts'] == 0
    assert node._cancel_all['retry_s'] == 100.0
    assert cancel_client(node).requests == []


def test_cancel_all_pending_after_more_than_max_ticks_down_then_sent_on_recovery():
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    client.ready = False
    release_by_timeout(node)
    for _ in range(bridge.MAX_CANCEL_ALL_ATTEMPTS * 3):
        tick(node, 1.0)
        assert node._cancel_all is not None
        assert node._cancel_all['attempts'] == 0
    assert client.requests == []
    assert not any('STOP THE ROBOT MANUALLY' in m for m in _errors(node))
    client.ready = True
    tick(node, 1.0)
    assert len(client.requests) == 1
    assert_cancel_all_request(client.requests[0])
    assert node._cancel_all['attempts'] == 1


def test_cancel_all_permanently_down_is_never_abandoned_without_sending():
    node = make_nav_bridge(now=100.0)
    cancel_client(node).ready = False
    release_by_timeout(node)
    for _ in range(100):
        tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert cancel_client(node).requests == []
    assert node._cancel_all is not None
    assert node._cancel_all['attempts'] == 0
    assert _errors(node) == []


def test_cancel_all_intermittent_service_counts_only_real_sends_then_gives_up():
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    client.ready = False
    release_by_timeout(node)                     # armed, nothing sent
    sent = 0
    for _ in range(bridge.MAX_CANCEL_ALL_ATTEMPTS):
        tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)   # service down: no attempt used
        assert node._cancel_all['attempts'] == sent
        client.ready = True
        tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)   # service up: real send, unanswered
        sent += 1
        assert len(client.requests) == sent
        assert node._cancel_all['attempts'] == sent
        client.ready = False
    assert not any('STOP THE ROBOT MANUALLY' in m for m in _errors(node))
    tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert node._cancel_all is None
    assert len(client.requests) == bridge.MAX_CANCEL_ALL_ATTEMPTS
    manual = [m for m in _errors(node) if 'STOP THE ROBOT MANUALLY' in m]
    assert len(manual) == 1


def test_cancel_all_call_async_exception_counts_as_a_real_attempt():
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    client.call_error = RuntimeError('boom')
    release_by_timeout(node)
    assert node._cancel_all['attempts'] == 1
    tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert node._cancel_all['attempts'] == 2
    for _ in range(bridge.MAX_CANCEL_ALL_ATTEMPTS):
        tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert node._cancel_all is None
    assert sum('STOP THE ROBOT MANUALLY' in m for m in _errors(node)) == 1
    assert sum('Cancel-all request failed' in m for m in _errors(node)) == (
        bridge.MAX_CANCEL_ALL_ATTEMPTS)


def _without_client(node):
    node._cancel_all_client = None
    return node


def test_missing_cancel_client_cancel_logs_visible_error_returns_ok_and_stays_pending():
    node = _without_client(make_nav_bridge(now=100.0))
    node.cancel()                                 # HTTP handler: must not raise, returns ok
    errors = _errors(node)
    assert len(errors) == 1 and 'STOP THE ROBOT MANUALLY' in errors[0]
    assert node._cancel_all is not None
    assert node._cancel_all['attempts'] == 0      # nothing left the node: no attempt used


def test_missing_cancel_client_watchdog_release_logs_error_and_never_raises():
    node = _without_client(make_nav_bridge(now=100.0))
    release_by_timeout(node)                      # watchdog: must not raise
    assert any('STOP THE ROBOT MANUALLY' in m for m in _errors(node))
    assert node._cancel_all is not None


def test_missing_cancel_client_error_is_rate_limited_and_never_consumes_attempts():
    node = _without_client(make_nav_bridge(now=100.0))
    node.cancel()
    assert len(_errors(node)) == 1
    for _ in range(9):
        tick(node, 1.0)                           # t=109: still inside the warn period
    assert len(_errors(node)) == 1
    assert node._cancel_all['attempts'] == 0
    tick(node, 1.0)                               # t=110: period elapsed -> one more
    assert len(_errors(node)) == 2
    for _ in range(100):
        tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert node._cancel_all is not None           # never abandoned
    assert node._cancel_all['attempts'] == 0
    assert all('STOP THE ROBOT MANUALLY' in m for m in _errors(node))
    assert len(_errors(node)) < 100               # limited, not one per tick


def test_missing_cancel_client_send_goal_still_clears_the_pending_cancel_all():
    node = _without_client(make_nav_bridge(now=100.0))
    node.cancel()
    node.send_goal(1.0, 2.0, 0.5)
    assert node._cancel_all is None


def test_send_goal_during_cancel_all_pending_with_service_down_clears_it():
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    client.ready = False
    node.cancel()
    assert node._cancel_all is not None
    node.send_goal(3.0, 4.0, 0.0)
    assert node._cancel_all is None
    client.ready = True
    node._nav_deadline_s = None
    tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert client.requests == []


def test_cancel_all_warn_period_is_ten_seconds():
    assert bridge.CANCEL_ALL_WARN_PERIOD_S == 10.0


def test_cancel_all_not_ready_warning_is_emitted_once_in_9_seconds_of_ticks():
    node = make_nav_bridge(now=100.0)
    cancel_client(node).ready = False
    node.cancel()                                 # first warning at t=100
    for _ in range(9):
        tick(node, 1.0)                           # up to t=109
    assert len(_pending_warnings(node)) == 1


def test_cancel_all_not_ready_warning_is_emitted_twice_in_11_seconds_of_ticks():
    node = make_nav_bridge(now=100.0)
    cancel_client(node).ready = False
    node.cancel()
    for _ in range(11):
        tick(node, 1.0)                           # up to t=111
    assert len(_pending_warnings(node)) == 2
    assert 'retrying' in _pending_warnings(node)[0]


# --- cancel-all: request content, stamp semantics and public-API-only client ---------------
# The server-side filtering by stamp (goals accepted after the stamp are NOT cancelled) is done
# by the Nav2 action server and can only be verified on the Raspberry Pi. Here only what the
# node controls is checked: the content of the request and when the stamp is taken.

def ns(seconds):
    """Convert fake-clock seconds to the integer nanoseconds the node clock reports."""
    return int(round(seconds * 1e9))


def test_cancel_all_request_has_zero_goal_id_and_stamp_equal_to_the_decision_instant():
    node = make_nav_bridge(now=100.25)
    node.cancel()
    request = cancel_client(node).requests[0]
    assert list(request.goal_info.goal_id.uuid) == [0] * 16
    assert request.goal_info.stamp.sec == 100
    assert request.goal_info.stamp.nanosec == 250000000
    assert node._cancel_all['stamp_ns'] == ns(100.25)


def test_cancel_all_stamp_is_never_zero_even_with_a_clock_at_zero():
    node = make_nav_bridge(now=0.0)
    node.cancel()
    stamp = cancel_client(node).requests[0].goal_info.stamp
    assert stamp.sec * 10**9 + stamp.nanosec == 1


def test_cancel_all_stamp_splits_seconds_and_nanoseconds_correctly():
    node = make_nav_bridge(now=1234.5)
    node.cancel()
    stamp = cancel_client(node).requests[0].goal_info.stamp
    assert (stamp.sec, stamp.nanosec) == (1234, 500000000)


def test_cancel_all_armed_by_the_watchdog_uses_the_release_instant_as_stamp():
    node = make_nav_bridge(now=100.0)
    release_by_timeout(node)                    # released at t = 100 + timeout + 0.1
    expected = ns(100.0 + bridge.NAV_RESPONSE_TIMEOUT_S + 0.1)
    assert_cancel_all_request(cancel_client(node).requests[0], expected)
    assert node._cancel_all['stamp_ns'] == expected


def test_cancel_all_retries_reuse_the_same_stamp_and_never_recompute_it():
    node = make_nav_bridge(now=100.0)
    release_by_timeout(node)
    client = cancel_client(node)
    expected = client.requests[0].goal_info.stamp
    first_total = expected.sec * 10**9 + expected.nanosec
    for _ in range(bridge.MAX_CANCEL_ALL_ATTEMPTS - 1):
        tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)    # the clock moves on
    assert len(client.requests) == bridge.MAX_CANCEL_ALL_ATTEMPTS
    for request in client.requests:
        assert_cancel_all_request(request, first_total)
    assert node._now_ns() > first_total               # the clock did advance


def test_retry_after_service_recovers_still_uses_the_original_decision_stamp():
    node = make_nav_bridge(now=100.0)
    cancel_client(node).ready = False
    node.cancel()                                     # decision at t=100
    for _ in range(30):
        tick(node, 1.0)
    cancel_client(node).ready = True
    tick(node, 1.0)
    assert len(cancel_client(node).requests) == 1
    assert_cancel_all_request(cancel_client(node).requests[0], ns(100.0))


def test_a_new_cancel_rearms_the_cancel_all_with_a_new_later_stamp():
    node = make_nav_bridge(now=100.0)
    node.cancel()
    node.clock[0] = 103.0
    node.cancel()
    client = cancel_client(node)
    assert len(client.requests) == 2
    assert_cancel_all_request(client.requests[0], ns(100.0))
    assert_cancel_all_request(client.requests[1], ns(103.0))
    assert node._cancel_all['stamp_ns'] == ns(103.0)
    assert node._cancel_all['attempts'] == 1          # rearmed from zero


def test_send_goal_clears_cancel_all_and_the_new_goal_is_later_than_its_stamp():
    # The node cannot make the server filter by stamp (that is Nav2's job, only verifiable on
    # the Pi). What it guarantees: the pending cancel-all is dropped by send_goal, so no
    # request (not even a late retry) is issued once the new goal exists, and the new goal's
    # instant is strictly after the stamp of the cancel-all.
    node = make_nav_bridge(now=100.0)
    node.cancel()
    stamp = node._cancel_all['stamp_ns']
    node.clock[0] += 0.001
    node.send_goal(1.0, 2.0, 0.5)
    assert node._now_ns() > stamp
    assert node._cancel_all is None
    sent = len(cancel_client(node).requests)
    node._nav_deadline_s = None
    for _ in range(5):
        tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert len(cancel_client(node).requests) == sent == 1


def test_cancel_all_with_an_active_goal_is_not_sent_by_cancel():
    node = make_nav_bridge(now=100.0)
    node.send_goal(1.0, 2.0, 0.5)
    node.cancel()                                     # token set: handle path, not cancel-all
    assert cancel_client(node).requests == []
    assert node._cancel_all is None


@pytest.mark.parametrize('code', [
    bridge.CANCEL_ERROR_REJECTED, 99, -1])
def test_cancel_all_rejected_or_unknown_code_keeps_pending_and_retries_same_stamp(code):
    node = make_nav_bridge(now=100.0)
    node.cancel()
    client = cancel_client(node)
    client.answer(0, code)
    assert node._cancel_all is not None
    tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert len(client.requests) == 2
    assert_cancel_all_request(client.requests[1], ns(100.0))


def test_cancel_all_response_exception_keeps_pending_and_retries_same_stamp():
    node = make_nav_bridge(now=100.0)
    node.cancel()
    client = cancel_client(node)
    client.futures[0].resolve(error=RuntimeError('lost'))
    assert node._cancel_all is not None
    tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert len(client.requests) == 2
    assert_cancel_all_request(client.requests[1], ns(100.0))


@pytest.mark.parametrize('code', [
    bridge.CANCEL_ERROR_NONE, bridge.CANCEL_ERROR_UNKNOWN_GOAL_ID,
    bridge.CANCEL_ERROR_GOAL_TERMINATED])
def test_cancel_all_final_codes_clear_pending_from_cancel(code):
    node = make_nav_bridge(now=100.0)
    node.cancel()
    cancel_client(node).answer(0, code)
    assert node._cancel_all is None
    for _ in range(3):
        tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert len(cancel_client(node).requests) == 1


def test_cancel_all_gives_up_after_exactly_five_real_sends_with_manual_stop_message():
    assert bridge.MAX_CANCEL_ALL_ATTEMPTS == 5
    node = make_nav_bridge(now=100.0)
    node.cancel()
    for _ in range(20):
        tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert len(cancel_client(node).requests) == 5
    assert node._cancel_all is None
    manual = [m for m in _errors(node) if 'STOP THE ROBOT MANUALLY' in m]
    assert len(manual) == 1


def test_cancel_all_service_not_ready_warns_with_the_pending_message_and_does_not_raise():
    node = make_nav_bridge(now=100.0)
    cancel_client(node).ready = False
    node.cancel()
    assert len(_pending_warnings(node)) == 1
    assert _errors(node) == []


def test_production_module_never_references_the_private_cancel_goal_client():
    import inspect
    source = inspect.getsource(bridge)
    assert '_cancel_goal_client' not in source


def test_test_doubles_do_not_imitate_the_private_cancel_goal_client():
    node = make_nav_bridge()
    assert not hasattr(node._nav_client, '_cancel_goal_client')
    assert not hasattr(node, '_cancel_goal_client')


# --- cancel-all: answers are attributed to the stamp they were sent for ----------------------
# Regression: a late answer to an OLDER cancel-all must not clear the pending cancel-all of a
# newer decision (it would stop the retries and leave a possibly running goal uncancelled).

def _arm_t2_after_t1_was_sent(node):
    """cancel() at t1 (sent), send_goal, service lost, goal released by timeout -> pending t2.

    Returns (t1_stamp_ns, t2_stamp_ns). The cancel-all of t2 is armed but NOT sent.
    """
    client = cancel_client(node)
    node.cancel()                                   # t1: sent (futures[0])
    t1 = node._cancel_all['stamp_ns']
    node.clock[0] += 0.5
    node.send_goal(1.0, 2.0, 0.5)                   # clears the pending cancel-all of t1
    assert node._cancel_all is None
    client.ready = False                            # the service goes down
    node.clock[0] += bridge.NAV_RESPONSE_TIMEOUT_S + 0.1
    node._nav_watchdog()                            # no handle: released, arms cancel-all t2
    assert node._nav_token is None
    t2 = node._cancel_all['stamp_ns']
    assert t2 > t1
    assert len(client.requests) == 1                # t2 not sent: service not ready
    return t1, t2


def test_ca_late_answer_of_an_older_cancel_all_does_not_clear_the_newer_pending():
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    t1, t2 = _arm_t2_after_t1_was_sent(node)
    client.answer(0, bridge.CANCEL_ERROR_NONE)      # LATE answer to t1
    assert node._cancel_all is not None
    assert node._cancel_all['stamp_ns'] == t2
    assert node._cancel_all['attempts'] == 0
    client.ready = True                             # service back: t2 must still be sent
    tick(node, 1.0)
    assert len(client.requests) == 2
    assert_cancel_all_request(client.requests[1], t2)


@pytest.mark.parametrize('code', [
    bridge.CANCEL_ERROR_UNKNOWN_GOAL_ID, bridge.CANCEL_ERROR_GOAL_TERMINATED,
    bridge.CANCEL_ERROR_REJECTED, 99])
def test_ca_stale_answer_with_any_code_leaves_the_newer_pending_untouched(code):
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    _, t2 = _arm_t2_after_t1_was_sent(node)
    before = dict(node._cancel_all)
    client.answer(0, code)
    assert node._cancel_all == before
    assert node._cancel_all['stamp_ns'] == t2


@pytest.mark.parametrize('code', [bridge.CANCEL_ERROR_REJECTED, 99])
def test_ca_stale_rejected_answer_gives_no_retry_warning_and_no_noise(code):
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    _arm_t2_after_t1_was_sent(node)
    warnings_before = len(_warnings(node))
    errors_before = len(_errors(node))
    client.answer(0, code)
    assert len(_warnings(node)) == warnings_before
    assert len(_errors(node)) == errors_before
    assert not [m for m in _warnings(node) if 'rejected' in m or 'will retry' in m]


def test_ca_stale_answer_with_exception_is_ignored_without_warning():
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    _, t2 = _arm_t2_after_t1_was_sent(node)
    warnings_before = len(_warnings(node))
    client.futures[0].resolve(error=RuntimeError('lost'))
    assert node._cancel_all is not None and node._cancel_all['stamp_ns'] == t2
    assert len(_warnings(node)) == warnings_before
    assert not [m for m in _warnings(node) if 'No cancel-all answer' in m]


def test_ca_stale_answer_is_logged_at_debug_and_not_as_info():
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    t1, _ = _arm_t2_after_t1_was_sent(node)
    info_before = len(node.logger.info.call_args_list)
    client.answer(0, bridge.CANCEL_ERROR_NONE)
    assert len(node.logger.info.call_args_list) == info_before
    assert node.logger.debug.call_count == 1
    assert 'stale' in node.logger.debug.call_args[0][0].lower()
    assert str(t1) in node.logger.debug.call_args[0][0]


def test_ca_stale_answer_does_not_read_the_future_result():
    node = make_nav_bridge(now=100.0)
    _arm_t2_after_t1_was_sent(node)
    future = mock.Mock()
    future.result.side_effect = AssertionError('a stale answer must not be inspected')
    node._on_cancel_all_response(1, future)         # stamp that is not the pending one
    future.result.assert_not_called()
    assert node._cancel_all is not None


@pytest.mark.parametrize('code', [
    bridge.CANCEL_ERROR_NONE, bridge.CANCEL_ERROR_REJECTED, 99])
def test_ca_answer_arriving_when_nothing_is_pending_neither_raises_nor_creates_one(code):
    node = make_nav_bridge(now=100.0)
    node.cancel()
    client = cancel_client(node)
    client.answer(0, bridge.CANCEL_ERROR_NONE)      # clears the pending
    assert node._cancel_all is None
    node._on_cancel_all_response(node._now_ns(), NS(result=lambda: NS(return_code=code)))
    late = FakeFuture()
    late.value = NS(return_code=code)
    node._on_cancel_all_response(node._now_ns(), late)
    assert node._cancel_all is None
    # a second (duplicate) answer to the same stamp is harmless too
    client.answer(0, code)
    assert node._cancel_all is None
    assert len(client.requests) == 1


def test_ca_answer_with_exception_when_nothing_is_pending_neither_raises_nor_warns():
    node = make_nav_bridge(now=100.0)
    node.cancel()
    client = cancel_client(node)
    client.answer(0, bridge.CANCEL_ERROR_NONE)
    warnings_before = len(_warnings(node))
    client.futures[0].resolve(error=RuntimeError('late'))
    assert node._cancel_all is None
    assert len(_warnings(node)) == warnings_before


@pytest.mark.parametrize('code', [
    bridge.CANCEL_ERROR_NONE, bridge.CANCEL_ERROR_UNKNOWN_GOAL_ID,
    bridge.CANCEL_ERROR_GOAL_TERMINATED])
def test_ca_current_stamp_final_codes_clear_the_pending_after_a_stale_answer(code):
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    _, t2 = _arm_t2_after_t1_was_sent(node)
    client.ready = True
    tick(node, 1.0)                                 # t2 sent: futures[1]
    assert_cancel_all_request(client.requests[1], t2)
    client.answer(0, code)                          # stale: ignored
    assert node._cancel_all is not None
    client.answer(1, code)                          # current: clears
    assert node._cancel_all is None
    for _ in range(3):
        tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert len(client.requests) == 2


@pytest.mark.parametrize('failure', ['rejected', 'exception'])
def test_ca_current_stamp_failure_keeps_pending_and_retries_with_the_same_stamp(failure):
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    _, t2 = _arm_t2_after_t1_was_sent(node)
    client.ready = True
    tick(node, 1.0)
    if failure == 'rejected':
        client.answer(1, bridge.CANCEL_ERROR_REJECTED)
    else:
        client.futures[1].resolve(error=RuntimeError('lost'))
    assert node._cancel_all is not None and node._cancel_all['stamp_ns'] == t2
    assert any('will retry' in m for m in _warnings(node))
    tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)
    assert len(client.requests) == 3
    assert_cancel_all_request(client.requests[2], t2)


def test_ca_each_attempt_registers_a_callback_bound_to_its_own_stamp():
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    node.cancel()                                   # t1, attempt 1
    t1 = node._cancel_all['stamp_ns']
    tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)    # t1, attempt 2 (same stamp)
    assert len(client.requests) == 2
    assert [len(f.callbacks) for f in client.futures] == [1, 1]
    # both answers belong to t1: the first (rejected) keeps it, the second clears it
    client.answer(0, bridge.CANCEL_ERROR_REJECTED)
    assert node._cancel_all is not None and node._cancel_all['stamp_ns'] == t1
    client.answer(1, bridge.CANCEL_ERROR_NONE)
    assert node._cancel_all is None


def test_ca_answers_of_attempt_one_and_two_of_t1_do_not_touch_a_newer_t2():
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    node.cancel()                                   # t1 attempt 1
    tick(node, bridge.NAV_CANCEL_RETRY_S + 0.01)    # t1 attempt 2
    node.clock[0] += 1.0
    node.cancel()                                   # t2 rearmed (no goal): futures[2]
    t2 = node._cancel_all['stamp_ns']
    assert len(client.requests) == 3
    client.answer(0, bridge.CANCEL_ERROR_NONE)
    client.answer(1, bridge.CANCEL_ERROR_UNKNOWN_GOAL_ID)
    assert node._cancel_all is not None and node._cancel_all['stamp_ns'] == t2
    client.answer(2, bridge.CANCEL_ERROR_NONE)
    assert node._cancel_all is None


def test_ca_race_pending_rearmed_between_the_check_and_the_delete_is_not_deleted():
    # Deterministic: future.result() runs between the first check (under the lock) and the
    # delete (under the lock again); its side effect rearms the pending like cancel() would.
    node = make_nav_bridge(now=100.0)
    client = cancel_client(node)
    node.cancel()                                   # t1
    t1 = node._cancel_all['stamp_ns']

    class RearmingFuture(FakeFuture):
        def result(self):
            node.clock[0] += 2.0
            node.cancel()                           # rearms: pending is now t3 != t1
            return NS(return_code=bridge.CANCEL_ERROR_NONE)

    node._on_cancel_all_response(t1, RearmingFuture())
    assert node._cancel_all is not None
    t3 = node._cancel_all['stamp_ns']
    assert t3 > t1
    assert len(client.requests) == 2                # t3 was sent by the rearming cancel()
    assert_cancel_all_request(client.requests[1], t3)
    assert not [m for m in node.logger.info.call_args_list
                if 'Cancel-all answered' in m[0][0]]
    assert node.logger.debug.call_count == 1


def test_ca_race_pending_cleared_between_the_check_and_the_delete_does_not_raise():
    node = make_nav_bridge(now=100.0)
    node.cancel()
    t1 = node._cancel_all['stamp_ns']

    class ClearingFuture(FakeFuture):
        def result(self):
            with node._nav_lock:
                node._cancel_all = None             # e.g. send_goal or the watchdog
            return NS(return_code=bridge.CANCEL_ERROR_NONE)

    node._on_cancel_all_response(t1, ClearingFuture())
    assert node._cancel_all is None


def test_ca_the_pending_is_only_modified_while_holding_the_nav_lock():
    node = make_nav_bridge(now=100.0)
    node.cancel()
    t1 = node._cancel_all['stamp_ns']
    seen = []

    class SpyLock:
        def __init__(self, inner):
            self.inner = inner
            self.depth = 0

        def __enter__(self):
            self.inner.acquire()
            self.depth += 1
            return self

        def __exit__(self, *args):
            self.depth -= 1
            self.inner.release()

        def locked(self):
            return self.inner.locked()

    spy = SpyLock(node._nav_lock)
    node._nav_lock = spy

    class WatchedFuture(FakeFuture):
        def result(self):
            seen.append(spy.depth)                  # result() is read OUTSIDE the lock
            return NS(return_code=bridge.CANCEL_ERROR_NONE)

    node._on_cancel_all_response(t1, WatchedFuture())
    assert seen == [0]
    assert node._cancel_all is None


@pytest.mark.parametrize('goals', [(), ('a',), ('a', 'b', 'c')])
def test_ca_info_log_includes_the_number_of_goals_canceling(goals):
    node = make_nav_bridge(now=100.0)
    node.cancel()
    cancel_client(node).futures[0].resolve(
        NS(return_code=bridge.CANCEL_ERROR_NONE, goals_canceling=list(goals)))
    assert node._cancel_all is None
    messages = [c[0][0] for c in node.logger.info.call_args_list
                if 'Cancel-all answered' in c[0][0]]
    assert len(messages) == 1
    assert f'goals_canceling={len(goals)}' in messages[0]
    assert 'code 0' in messages[0]


def test_ca_result_without_goals_canceling_still_clears_the_pending():
    node = make_nav_bridge(now=100.0)
    node.cancel()
    cancel_client(node).futures[0].resolve(NS(return_code=bridge.CANCEL_ERROR_NONE))
    assert node._cancel_all is None
    messages = [c[0][0] for c in node.logger.info.call_args_list
                if 'Cancel-all answered' in c[0][0]]
    assert len(messages) == 1 and 'goals_canceling=0' in messages[0]


def test_ca_goals_canceling_is_not_logged_for_a_rejected_answer():
    node = make_nav_bridge(now=100.0)
    node.cancel()
    cancel_client(node).futures[0].resolve(
        NS(return_code=bridge.CANCEL_ERROR_REJECTED, goals_canceling=['a']))
    assert node._cancel_all is not None
    assert not [c for c in node.logger.info.call_args_list
                if 'Cancel-all answered' in c[0][0]]


# === Web teleop on cmd_vel_nav, stop and watchdog (task 18, criteria I-3 and A-4) =============
# SAFETY-CRITICAL: first time the web moves the robot. The node is built without Node.__init__;
# the Twist class and the cmd_vel_nav publisher are doubles of the PUBLIC rclpy/geometry_msgs
# API only (Twist() with linear/angular x,y,z; publisher.publish(msg)). A shared list of
# events gives the exact order of publications vs. Nav2 calls. Time is the fake clock of
# make_bridge (node.clock[0]); nothing sleeps. NOT covered here (needs the Pi): the real
# publisher/QoS, the real executor running the 20 ms timer and velocity_smoother.

from mtrb_bringup.web_utils import TeleopGate  # noqa: E402, I100

ZERO_BURST = 3
LEAK_TOKEN = 'LEAK-S3cr3t-token'


class FakeTwist:
    """Public shape of geometry_msgs/Twist: linear/angular with x, y, z = 0.0; no header."""

    def __init__(self):
        self.linear = NS(x=0.0, y=0.0, z=0.0)
        self.angular = NS(x=0.0, y=0.0, z=0.0)


class FakeCmdVelPublisher:
    """Public rclpy Publisher API: publish(msg). Every ATTEMPT is logged in node.events."""

    def __init__(self, node):
        self.node = node
        self.msgs = []              # messages whose publish() succeeded
        self.attempts = 0
        self.fail_on = set()        # attempt numbers (0-based) that raise
        self.always_fail = False
        self.hook = None            # callable(vx, wz) run inside publish()

    def publish(self, msg):
        number = self.attempts
        self.attempts += 1
        self.node.events.append(('pub', msg.linear.x, msg.angular.z, self.node.clock[0]))
        if self.hook is not None:
            self.hook(msg.linear.x, msg.angular.z)
        if self.always_fail or number in self.fail_on:
            raise RuntimeError('publish failed')
        self.msgs.append(msg)


class EventHandle(FakeHandle):
    def __init__(self, events):
        super().__init__()
        self.events = events

    def cancel_goal_async(self):
        self.events.append(('cancel',))
        return super().cancel_goal_async()


class EventActionClient(FakeActionClient):
    def __init__(self, events):
        super().__init__()
        self.events = events

    def send_goal_async(self, goal, feedback_callback=None):
        self.events.append(('send_goal',))
        return super().send_goal_async(goal, feedback_callback)


def make_teleop_bridge(monkeypatch, now=100.0, enabled=True, gate=True):
    monkeypatch.setattr(bridge, 'Twist', FakeTwist)
    node = make_nav_bridge(now)
    node.events = []
    node._nav_client = EventActionClient(node.events)
    node._cancel_all_client.lock_probe = lambda: node.events.append(('cancel_all',))
    node._cmd_vel_nav_pub = FakeCmdVelPublisher(node)
    node._initial_pose_pub = mock.Mock()
    if gate:
        node._teleop_gate = TeleopGate(0.2, 0.8, 0.3, node._now_s, enabled=enabled)
    return node


def tmsg(seq, vx=0.1, wz=0.2):
    return {'type': 'teleop', 'vx': vx, 'wz': wz, 'seq': seq}


def published(node):
    """[(vx, wz)] of every publication attempt, in order."""
    return [(e[1], e[2]) for e in node.events if e[0] == 'pub']


def zero_count(node):
    return sum(1 for p in published(node) if p == (0.0, 0.0))


def accept_goal(node):
    """Send a goal and let Nav2 accept it with a handle that logs its cancel."""
    node.send_goal(1.0, 2.0, 0.5)
    handle = EventHandle(node.events)
    node._nav_client.futures[-1].resolve(handle)
    return handle


def lock_test_events(node):
    return [e for e in node.events if e[0] in ('send_goal', 'cancel', 'cancel_all')]


# --- published message ---------------------------------------------------------------------

def test_teleop_publishes_one_twist_with_only_linear_x_and_angular_z(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    assert node.teleop(1, tmsg(0, vx=0.1, wz=-0.3)) is None
    [msg] = node._cmd_vel_nav_pub.msgs
    assert isinstance(msg, bridge.Twist)
    assert (msg.linear.x, msg.angular.z) == (0.1, -0.3)
    assert (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y) == (0.0, 0.0, 0.0, 0.0)
    assert not hasattr(msg, 'header')  # Twist, not TwistStamped: there is no stamp to fill


@pytest.mark.parametrize('vx, wz, expected', [
    (9.0, 9.0, (0.2, 0.8)), (-9.0, -9.0, (-0.2, -0.8)), (0.2, 0.8, (0.2, 0.8))])
def test_teleop_publishes_the_saturated_velocity(monkeypatch, vx, wz, expected):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0, vx=vx, wz=wz))
    assert published(node) == [expected]


def test_published_velocities_are_floats(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0, vx=0, wz=1))
    [msg] = node._cmd_vel_nav_pub.msgs
    assert type(msg.linear.x) is float and type(msg.angular.z) is float
    assert (msg.linear.x, msg.angular.z) == (0.0, 0.8)


def test_zero_twist_has_every_field_at_zero(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.stop()
    assert len(node._cmd_vel_nav_pub.msgs) == ZERO_BURST
    for msg in node._cmd_vel_nav_pub.msgs:
        assert isinstance(msg, bridge.Twist)
        assert [msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z] == [0.0] * 6


def test_nothing_is_published_for_rejected_dropped_or_invalid_messages(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    node.events.clear()
    assert node.teleop(1, tmsg(0)) is None                        # replayed seq: dropped
    node.clock[0] += 0.01
    assert node.teleop(2, tmsg(0))[0] == 'teleop_rejected'        # lease held by client 1
    assert node.teleop(1, tmsg(1, vx=float('nan')))[0] == 'invalid_teleop'
    node.clock[0] += 1.0                                          # past the reply throttle
    assert node.teleop(1, None)[0] == 'invalid_teleop'
    assert published(node) == []


def test_teleop_never_touches_the_initial_pose_publisher(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    node.stop()
    node.teleop_disconnect(1)
    node.clock[0] += 1.0
    node._teleop_watchdog()
    node._initial_pose_pub.publish.assert_not_called()


# --- I-3: watchdog (fake clock, no sleep) --------------------------------------------------

def test_i3_silence_for_300_ms_publishes_a_zero_twist_and_then_stops_publishing(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    start = node.clock[0]
    assert node.teleop(1, tmsg(0)) is None
    for k in range(1, 151):                      # 3 s of 20 ms watchdog ticks
        node.clock[0] = start + k * 0.02
        node._teleop_watchdog()
    zero_times = [round(e[3] - start, 6) for e in node.events
                  if e[0] == 'pub' and (e[1], e[2]) == (0.0, 0.0)]
    assert published(node) == [(0.1, 0.2)] + [(0.0, 0.0)] * ZERO_BURST
    assert len(zero_times) == ZERO_BURST
    assert min(zero_times) > 0.3 - 1e-6          # never before 300 ms of silence
    assert max(zero_times) <= 0.32 + 1e-6        # at the first 20 ms tick after 300 ms


def test_i3_watchdog_does_not_publish_at_exactly_300_ms_and_does_just_after(monkeypatch):
    node = make_teleop_bridge(monkeypatch, now=100.0)
    node.teleop(1, tmsg(0))
    node.clock[0] = 100.3
    node._teleop_watchdog()
    assert published(node) == [(0.1, 0.2)]
    node.clock[0] = 100.3001
    node._teleop_watchdog()
    assert published(node) == [(0.1, 0.2)] + [(0.0, 0.0)] * ZERO_BURST
    node.clock[0] = 100.32
    node._teleop_watchdog()
    node.clock[0] = 105.0
    node._teleop_watchdog()
    assert zero_count(node) == ZERO_BURST


def test_i3_watchdog_reads_only_the_injected_clock(monkeypatch):
    class NoWallClock:
        def __getattr__(self, name):
            raise AssertionError(f'time.{name} used by the teleop path')

    node = make_teleop_bridge(monkeypatch)
    monkeypatch.setattr(bridge, 'time', NoWallClock())
    node.teleop(1, tmsg(0))
    node.clock[0] += 0.4
    node._teleop_watchdog()
    assert zero_count(node) == ZERO_BURST


def test_i3_steady_stream_of_commands_keeps_the_robot_driving_without_zeros(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    for seq in range(40):                         # 10 Hz for 4 s, watchdog ticks in between
        node.clock[0] = 100.0 + seq * 0.1
        assert node.teleop(1, tmsg(seq)) is None
        for tick in range(5):
            node.clock[0] += 0.02
            node._teleop_watchdog()
    assert zero_count(node) == 0
    assert len(published(node)) == 40


def test_i3_after_silence_the_stream_can_restart(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    node.clock[0] += 0.4
    node._teleop_watchdog()
    node.clock[0] += 0.1
    assert node.teleop(1, tmsg(1)) is None
    assert published(node)[-1] == (0.1, 0.2)
    node.clock[0] += 0.4
    node._teleop_watchdog()
    assert zero_count(node) == 2 * ZERO_BURST


def test_watchdog_with_nobody_controlling_never_publishes(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    for k in range(500):
        node.clock[0] += 0.02
        node._teleop_watchdog()
    assert node.events == []


def test_watchdog_without_a_gate_is_a_noop(monkeypatch):
    node = make_teleop_bridge(monkeypatch, gate=False)
    node._teleop_watchdog()
    assert node.events == []


def test_watchdog_failure_publishes_zeros_and_logs_an_error(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node._teleop_gate.watchdog_tick = mock.Mock(side_effect=RuntimeError('gate broke'))
    node._teleop_watchdog()
    assert published(node) == [(0.0, 0.0)] * ZERO_BURST
    assert any('watchdog' in m.lower() for m in _errors(node))


def test_watchdog_does_not_publish_zeros_again_after_the_burst_even_if_late(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    node.clock[0] += 10.0                        # the executor was stalled for 10 s
    node._teleop_watchdog()
    node._teleop_watchdog()
    assert zero_count(node) == ZERO_BURST


# --- A-4: stop ----------------------------------------------------------------------------

def test_stop_publishes_the_zero_burst_before_anything_else(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.stop()
    assert published(node) == [(0.0, 0.0)] * ZERO_BURST
    assert [e[0] for e in node.events][:ZERO_BURST] == ['pub'] * ZERO_BURST


def test_a4_stop_publishes_the_first_zero_in_less_than_500_ms_of_the_fake_clock(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    requested = node.clock[0]
    node.stop()
    first_zero = [e for e in node.events if e[0] == 'pub' and (e[1], e[2]) == (0.0, 0.0)][0]
    assert first_zero[3] - requested < 0.5
    assert first_zero[3] - requested == 0.0      # synchronous chain: no waiting at all


def test_stop_with_a_goal_publishes_zeros_first_and_then_cancels_the_goal(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    accept_goal(node)
    node.events.clear()
    node.stop()
    kinds = [e[0] for e in node.events]
    assert kinds[:ZERO_BURST] == ['pub'] * ZERO_BURST
    assert kinds[ZERO_BURST:] == ['cancel']


def test_stop_with_nav2_down_still_publishes_zeros_and_arms_the_cancel_all(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node._nav_client.ready = False
    cancel_client(node).ready = False
    node.stop()
    assert published(node) == [(0.0, 0.0)] * ZERO_BURST
    assert lock_test_events(node) == []
    assert node._cancel_all is not None           # the cancel stays pending for the retries


def test_stop_without_a_goal_sends_the_cancel_all_after_the_zeros(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.stop()
    kinds = [e[0] for e in node.events]
    assert kinds == ['pub'] * ZERO_BURST + ['cancel_all']


def test_stop_zeros_precede_a_cancel_that_raises(monkeypatch):
    node = make_teleop_bridge(monkeypatch)

    def failing_cancel():
        node.events.append(('cancel',))
        raise RuntimeError('cancel exploded')

    node.cancel = failing_cancel
    node.stop()                                   # must not raise
    kinds = [e[0] for e in node.events]
    assert kinds == ['pub'] * ZERO_BURST + ['cancel']
    assert any('Cancel during stop failed' in m for m in _errors(node))


def test_stop_does_not_depend_on_the_nav_lock(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node._nav_lock.acquire()                      # a stuck Nav2 callback holds the nav lock
    try:
        done = threading.Thread(target=node._stop_teleop_only)
        done.start()
        done.join(2.0)
        assert not done.is_alive()
    finally:
        node._nav_lock.release()
    assert zero_count(node) == ZERO_BURST


def test_stop_releases_the_lease_so_that_another_client_can_drive(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    node.stop()
    node._nav_client.ready = False                # nothing may keep the node "busy"
    cancel_client(node).ready = False
    node.clock[0] += 0.1
    assert node.teleop(2, tmsg(0)) is None
    assert published(node)[-1] == (0.1, 0.2)


def test_stop_then_watchdog_does_not_publish_a_second_burst(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    node.stop()
    node.clock[0] += 1.0
    node._teleop_watchdog()
    assert zero_count(node) == ZERO_BURST


def test_stop_without_a_gate_still_publishes_the_zero_burst(monkeypatch):
    node = make_teleop_bridge(monkeypatch, gate=False)
    node.stop()
    assert published(node) == [(0.0, 0.0)] * ZERO_BURST


def test_stop_with_the_teleop_disabled_still_publishes_zeros(monkeypatch):
    node = make_teleop_bridge(monkeypatch, enabled=False)
    node.stop()
    assert zero_count(node) == ZERO_BURST


def _clocked_summaries(node, clock):
    """Give every log summary of the node an injected clock (no real time, no sleeping)."""
    node._log_summaries = {
        key: (bridge.EventSummary(summary._label, bridge.LOG_SUMMARY_PERIOD_S,
                                  clock=lambda: clock[0]), level)
        for key, (summary, level) in node._log_summaries.items()}


def _stop_lines(node):
    return [m for m in _warnings(node) if 'stop(s)' in m]


def test_stop_is_logged_without_any_velocity_data(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.stop()
    assert _stop_lines(node) == ['1 stop(s) in the last window']
    text = _all_log_text(node)
    for forbidden in ('vx', 'wz', 'linear', 'angular', 'Twist', 'velocity'):
        assert forbidden not in text


def test_a_flood_of_stops_publishes_every_zero_burst_but_logs_one_line_per_second(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    clock = [50.0]
    _clocked_summaries(node, clock)
    for _ in range(40):
        node.stop()
    assert zero_count(node) == 40 * ZERO_BURST           # SAFETY: no zero is ever coalesced
    assert _stop_lines(node) == ['1 stop(s) in the last window']   # first one at once
    clock[0] += 1.0
    node._flush_log_summaries()
    assert _stop_lines(node) == ['1 stop(s) in the last window',
                                 '39 stop(s) in the last window']
    node._flush_log_summaries()                           # nothing pending any more
    assert len(_stop_lines(node)) == 2


def test_stops_inside_the_period_are_counted_and_logged_when_the_period_ends(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    clock = [10.0]
    _clocked_summaries(node, clock)
    for _ in range(10):                                   # t = 10.0 .. 10.9
        node.stop()
        clock[0] += 0.1
    assert len(_stop_lines(node)) == 1
    clock[0] = 11.0
    node.stop()                                           # the period elapsed: summary line
    assert _stop_lines(node)[-1] == '10 stop(s) in the last window'
    assert len(_stop_lines(node)) == 2
    assert zero_count(node) == 11 * ZERO_BURST


def test_the_watchdog_tick_releases_the_pending_log_summaries(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    clock = [5.0]
    _clocked_summaries(node, clock)
    for _ in range(6):
        node.stop()
    assert len(_stop_lines(node)) == 1
    clock[0] += 0.5
    node._teleop_watchdog()
    assert len(_stop_lines(node)) == 1                    # not yet due
    clock[0] += 0.5
    node._teleop_watchdog()
    assert _stop_lines(node)[-1] == '5 stop(s) in the last window'


def test_flush_log_summaries_without_pending_events_logs_nothing(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    clock = [0.0]
    _clocked_summaries(node, clock)
    clock[0] += 100.0
    node._flush_log_summaries()
    assert not node.logger.warning.called and not node.logger.info.called


def test_a_clock_that_goes_back_does_not_swallow_the_stop_log(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    clock = [100.0]
    _clocked_summaries(node, clock)
    node.stop()
    clock[0] = 20.0
    node.stop()
    assert len(_stop_lines(node)) == 2


@pytest.mark.parametrize('fail_on', [{0}, {1}, {2}, {0, 1}, {0, 2}, {1, 2}])
def test_stop_keeps_publishing_the_remaining_zeros_when_a_publish_raises(monkeypatch, fail_on):
    node = make_teleop_bridge(monkeypatch)
    node._cmd_vel_nav_pub.fail_on = fail_on
    node.stop()                                   # never raises
    assert node._cmd_vel_nav_pub.attempts >= ZERO_BURST
    assert len(node._cmd_vel_nav_pub.msgs) == ZERO_BURST - len(fail_on)
    assert len(_errors(node)) >= len(fail_on)


def test_stop_attempts_the_full_burst_even_if_every_publish_raises(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node._cmd_vel_nav_pub.always_fail = True
    node.stop()
    assert node._cmd_vel_nav_pub.attempts == ZERO_BURST
    assert len(node.events) == ZERO_BURST + 1     # + the cancel-all of stop()
    assert node.events[-1] == ('cancel_all',)


def test_a_publish_error_during_teleop_does_not_propagate_and_is_logged(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node._cmd_vel_nav_pub.always_fail = True
    assert node.teleop(1, tmsg(0)) is None
    assert any('cmd_vel_nav' in m for m in _errors(node))


def test_a_publish_error_does_not_leave_the_watchdog_disarmed(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node._cmd_vel_nav_pub.fail_on = {0}
    node.teleop(1, tmsg(0))                       # the command failed to publish
    node.clock[0] += 0.4
    node._teleop_watchdog()                       # the zeros still go out
    assert len(node._cmd_vel_nav_pub.msgs) == ZERO_BURST


# --- teleop vs goals -----------------------------------------------------------------------

def test_send_goal_with_active_teleop_publishes_zeros_before_send_goal_async(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    node.events.clear()
    node.send_goal(1.0, 2.0, 0.5)
    kinds = [e[0] for e in node.events]
    assert kinds == ['pub'] * ZERO_BURST + ['send_goal']
    assert published(node) == [(0.0, 0.0)] * ZERO_BURST


def test_send_goal_releases_the_teleop_lease_and_the_watchdog_stays_quiet(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    node.send_goal(1.0, 2.0, 0.5)
    node.clock[0] += 1.0
    node._teleop_watchdog()
    assert zero_count(node) == ZERO_BURST
    assert node._teleop_gate.active is False


def test_teleop_after_a_goal_started_is_rejected_and_publishes_nothing(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    node.send_goal(1.0, 2.0, 0.5)
    node.events.clear()
    node.clock[0] += 0.1
    reply = node.teleop(1, tmsg(1))
    assert reply is not None and reply[0] == 'teleop_rejected'
    assert published(node) == []


def test_send_goal_without_teleop_publishes_no_zeros(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.send_goal(1.0, 2.0, 0.5)
    assert published(node) == []
    assert node.events == [('send_goal',)]


def test_send_goal_after_the_teleop_already_ended_publishes_no_extra_zeros(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    node.clock[0] += 0.4
    node._teleop_watchdog()
    node.events.clear()
    node.send_goal(1.0, 2.0, 0.5)
    assert published(node) == []


def test_send_goal_refused_without_nav2_does_not_disturb_the_teleop(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    node.events.clear()
    node._nav_client.ready = False
    with pytest.raises(bridge.NavUnavailableError):
        node.send_goal(1.0, 2.0, 0.5)
    assert published(node) == []
    node.clock[0] += 0.1
    assert node.teleop(1, tmsg(1)) is None
    assert published(node) == [(0.1, 0.2)]


def test_send_goal_zeros_failure_does_not_prevent_the_goal(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    node._cmd_vel_nav_pub.always_fail = True
    goal_id = node.send_goal(1.0, 2.0, 0.5)
    assert goal_id and ('send_goal',) in node.events
    assert node._cmd_vel_nav_pub.attempts == 1 + ZERO_BURST


def test_teleop_is_rejected_with_an_active_goal(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.send_goal(1.0, 2.0, 0.5)
    node.events.clear()
    reply = node.teleop(1, tmsg(0))
    assert reply is not None and reply[0] == 'teleop_rejected'
    assert published(node) == []


def test_teleop_is_rejected_with_an_accepted_goal(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    accept_goal(node)
    node.events.clear()
    assert node.teleop(1, tmsg(0))[0] == 'teleop_rejected'
    assert published(node) == []


def test_teleop_is_rejected_while_an_orphan_goal_is_unconfirmed(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    make_orphan(node)
    node.events.clear()
    assert node.teleop(1, tmsg(0))[0] == 'teleop_rejected'
    assert published(node) == []


def test_teleop_is_rejected_with_a_pending_cancel_all_if_the_action_server_is_ready(
        monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    cancel_client(node).ready = False             # the cancel-all stays pending
    node.cancel()
    assert node._cancel_all is not None
    node._nav_client.ready = True
    assert node.teleop(1, tmsg(0))[0] == 'teleop_rejected'
    assert published(node) == []


def test_teleop_is_not_rejected_with_a_pending_cancel_all_if_the_server_is_not_ready(
        monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    cancel_client(node).ready = False
    node.cancel()
    assert node._cancel_all is not None
    node._nav_client.ready = False                # Nav2 absent (e.g. mode mapping)
    assert node.teleop(1, tmsg(0)) is None
    assert published(node) == [(0.1, 0.2)]


def test_stop_does_not_block_the_teleop_for_good_when_nav2_is_absent(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node._nav_client.ready = False
    cancel_client(node).ready = False
    node.stop()
    node.clock[0] += 0.1
    assert node.teleop(1, tmsg(0)) is None


def test_stop_with_nav2_up_blocks_the_teleop_until_the_cancel_all_is_answered(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.stop()                                   # cancel-all sent, not answered yet
    node.clock[0] += 0.1
    assert node.teleop(1, tmsg(0))[0] == 'teleop_rejected'
    cancel_client(node).answer(0, bridge.CANCEL_ERROR_NONE)
    node.clock[0] += 1.5                          # past the error-reply throttle
    assert node.teleop(1, tmsg(1)) is None


def test_nav_busy_is_false_when_idle_and_true_for_each_busy_cause(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    assert node._nav_busy() is False
    node.send_goal(1.0, 2.0, 0.5)
    assert node._nav_busy() is True


def test_cancel_all_becoming_pending_while_driving_stops_the_teleop(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    cancel_client(node).ready = False
    node.teleop(1, tmsg(0))
    node.cancel()                                 # idle cancel: arms a pending cancel-all
    node.events.clear()
    node.clock[0] += 0.1
    reply = node.teleop(1, tmsg(1))
    assert reply[0] == 'teleop_rejected'
    assert published(node) == [(0.0, 0.0)] * ZERO_BURST


def test_busy_check_happens_inside_the_teleop_lock(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    seen = []
    original = node._nav_busy

    def spy():
        seen.append(node._teleop_lock.locked())
        return original()

    node._nav_busy = spy
    node.teleop(1, tmsg(0))
    assert seen == [True]


def test_disabled_teleop_answers_teleop_disabled_and_publishes_nothing(monkeypatch):
    node = make_teleop_bridge(monkeypatch, enabled=False)
    reply = node.teleop(1, tmsg(0))
    assert reply[0] == 'teleop_disabled'
    assert published(node) == []


# --- disconnect ----------------------------------------------------------------------------

def test_disconnect_of_the_owner_publishes_zeros_at_once(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    node.events.clear()
    node.teleop_disconnect(1)
    assert published(node) == [(0.0, 0.0)] * ZERO_BURST


def test_disconnect_of_another_client_publishes_nothing_and_keeps_the_owner(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    node.events.clear()
    node.teleop_disconnect(2)
    assert published(node) == []
    assert node._teleop_gate.active is True


def test_disconnect_of_a_client_that_never_drove_publishes_nothing(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop_disconnect(7)
    assert node.events == []


def test_disconnect_then_watchdog_does_not_repeat_the_burst(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    node.teleop_disconnect(1)
    node.clock[0] += 1.0
    node._teleop_watchdog()
    assert zero_count(node) == ZERO_BURST


def test_disconnect_without_a_gate_is_a_noop(monkeypatch):
    node = make_teleop_bridge(monkeypatch, gate=False)
    node.teleop_disconnect(1)
    assert node.events == []


def test_disconnect_failure_in_the_gate_stops_the_robot_and_logs(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node._teleop_gate.client_disconnected = mock.Mock(side_effect=RuntimeError('boom'))
    node.teleop_disconnect(1)
    assert published(node) == [(0.0, 0.0)] * ZERO_BURST
    assert any('disconnect' in m.lower() for m in _errors(node))


# --- fail closed ---------------------------------------------------------------------------

def test_teleop_without_a_gate_is_rejected_and_publishes_nothing(monkeypatch):
    node = make_teleop_bridge(monkeypatch, gate=False)
    assert node._teleop_gate is None
    reply = node.teleop(1, tmsg(0))
    assert reply is not None and reply[0] == 'teleop_rejected'
    assert node.events == []


def test_gate_exception_stops_the_robot_logs_and_replies_with_an_error(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node._teleop_gate.on_teleop = mock.Mock(side_effect=RuntimeError('gate broke'))
    reply = node.teleop(1, tmsg(0))
    assert reply is not None and reply[0] == 'teleop_rejected'
    assert published(node) == [(0.0, 0.0)] * ZERO_BURST
    assert any('Teleop failed' in m for m in _errors(node))


def test_gate_exception_releases_the_lease(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    node._teleop_gate.on_teleop = mock.Mock(side_effect=RuntimeError('gate broke'))
    node.teleop(1, tmsg(1))
    assert node._teleop_gate.owner_age_s is None


def test_nav_busy_exception_is_handled_as_a_failure_with_zeros(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node._nav_busy = mock.Mock(side_effect=RuntimeError('lock broke'))
    reply = node.teleop(1, tmsg(0))
    assert reply[0] == 'teleop_rejected'
    assert published(node) == [(0.0, 0.0)] * ZERO_BURST


def test_the_teleop_lock_is_released_after_a_failure(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node._teleop_gate.on_teleop = mock.Mock(side_effect=RuntimeError('gate broke'))
    node.teleop(1, tmsg(0))
    assert node._teleop_lock.locked() is False
    node.stop()
    assert node._teleop_lock.locked() is False


# --- lock order and race with stop ---------------------------------------------------------

class SpyLock:
    """Context-manager lock that records acquisitions and which locks were already held."""

    def __init__(self, name, held, log):
        self.name, self.held, self.log = name, held, log
        self._real = threading.Lock()

    def __enter__(self):
        self.log.append((self.name, tuple(self.held)))
        self._real.acquire()
        self.held.append(self.name)
        return self

    def __exit__(self, *exc):
        self.held.remove(self.name)
        self._real.release()
        return False

    def locked(self):
        return self._real.locked()


def test_lock_order_is_teleop_lock_then_nav_lock_never_the_reverse(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    held, log = [], []
    node._teleop_lock = SpyLock('teleop', held, log)
    node._nav_lock = SpyLock('nav', held, log)
    node.teleop(1, tmsg(0))                       # busy check under the teleop lock
    node.teleop(2, tmsg(0))                       # rejected by lease
    node.send_goal(1.0, 2.0, 0.5)                 # nav lock, then the teleop stop
    node.teleop(1, tmsg(1))                       # rejected: goal active
    node.stop()
    node.cancel()
    node.clock[0] += 1.0
    node._teleop_watchdog()
    node._nav_watchdog()
    node.teleop_disconnect(1)
    node._teleop_stop_if_owned()
    taken_teleop_under_nav = [e for e in log if e[0] == 'teleop' and 'nav' in e[1]]
    assert taken_teleop_under_nav == []
    # the spy really sees the allowed nesting (busy check inside the teleop lock)
    assert any(e[0] == 'nav' and 'teleop' in e[1] for e in log)
    assert held == []


def test_a_command_accepted_just_before_a_stop_is_never_published_after_the_zeros(
        monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    inside, release = threading.Event(), threading.Event()

    def hook(vx, wz):
        if (vx, wz) != (0.0, 0.0) and not inside.is_set():
            inside.set()
            release.wait(5.0)                     # the command is "in flight" in publish()

    node._cmd_vel_nav_pub.hook = hook
    sender = threading.Thread(target=node.teleop, args=(1, tmsg(0)))
    stopper = threading.Thread(target=node.stop)
    sender.start()
    assert inside.wait(5.0)
    stopper.start()
    stopper.join(0.05)
    blocked = stopper.is_alive()                  # the stop must wait for the in-flight command
    release.set()
    sender.join(5.0)
    stopper.join(5.0)
    assert blocked, 'stop() overtook a command that was already accepted'
    assert published(node) == [(0.1, 0.2)] + [(0.0, 0.0)] * ZERO_BURST


def test_two_clients_cannot_publish_concurrently(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    inside, release = threading.Event(), threading.Event()
    active = []
    overlap = []

    def hook(vx, wz):
        active.append(1)
        overlap.append(len(active))
        if not inside.is_set():
            inside.set()
            release.wait(5.0)
        active.pop()

    node._cmd_vel_nav_pub.hook = hook
    first = threading.Thread(target=node.teleop, args=(1, tmsg(0)))
    second = threading.Thread(target=node.teleop, args=(2, tmsg(0)))
    first.start()
    assert inside.wait(5.0)
    second.start()
    second.join(0.05)
    release.set()
    first.join(5.0)
    second.join(5.0)
    assert max(overlap) == 1                      # publishes are serialized by one lock
    assert published(node) == [(0.1, 0.2)]        # client 2 lost the lease


# --- chain through the real FastAPI app (doubles for ROS) ----------------------------------

def _app_client(node):
    pytest.importorskip('httpx')
    from fastapi.testclient import TestClient
    from mtrb_bringup.web_server import create_app
    return TestClient(create_app(StateStore(), 'tok-123', nav_port=node))


def test_post_api_stop_publishes_the_zeros_within_500_ms_of_the_fake_clock(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.teleop(1, tmsg(0))
    requested = node.clock[0]
    response = _app_client(node).post('/api/stop', headers={'Authorization': 'Bearer tok-123'})
    assert response.status_code == 200 and response.json() == {'ok': True}
    assert published(node) == [(0.1, 0.2)] + [(0.0, 0.0)] * ZERO_BURST
    assert node.events[1][3] - requested < 0.5


def test_post_api_stop_without_token_publishes_nothing(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    assert _app_client(node).post('/api/stop').status_code == 401
    assert node.events == []


def test_ws_teleop_and_stop_reach_the_node_and_the_disconnect_stops_the_robot(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    disconnected = threading.Event()
    original = node.teleop_disconnect

    def disconnect_and_signal(client_id):
        original(client_id)
        disconnected.set()

    node.teleop_disconnect = disconnect_and_signal
    client = _app_client(node)
    with client, client.websocket_connect('/ws?token=tok-123') as ws:
        ws.send_text(json.dumps(tmsg(0, vx=0.15, wz=0.4)))
        ws.send_text('{"type":"probe"}')
        assert json.loads(ws.receive_text())['code'] == 'unsupported_message'
        assert published(node) == [(0.15, 0.4)]
    assert disconnected.wait(3.0), 'the closed WebSocket did not stop the teleop'
    assert published(node) == [(0.15, 0.4)] + [(0.0, 0.0)] * ZERO_BURST


def test_ws_stop_message_publishes_the_zero_burst(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    client = _app_client(node)
    with client, client.websocket_connect('/ws?token=tok-123') as ws:
        ws.send_text(json.dumps(tmsg(0)))
        ws.send_text('{"type":"stop"}')
        ws.send_text('{"type":"probe"}')
        ws.receive_text()
        assert published(node) == [(0.1, 0.2)] + [(0.0, 0.0)] * ZERO_BURST


def test_ws_second_client_cannot_drive_while_the_first_one_does(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    replies, handled = [], [threading.Event(), threading.Event()]
    original = node.teleop

    def teleop_and_record(client_id, message):
        reply = original(client_id, message)
        replies.append((client_id, reply))
        handled[len(replies) - 1].set()
        return reply

    node.teleop = teleop_and_record
    client = _app_client(node)
    with client, client.websocket_connect('/ws?token=tok-123') as first, \
            client.websocket_connect('/ws?token=tok-123') as second:
        first.send_text(json.dumps(tmsg(0)))
        assert handled[0].wait(3.0)
        second.send_text(json.dumps(tmsg(0, vx=0.2)))
        assert handled[1].wait(3.0)
        assert replies[0][1] is None
        assert replies[1][1] is not None and replies[1][1][0] == 'teleop_rejected'
        reply = json.loads(second.receive_text())  # only reached when an error was produced
        assert reply['code'] == 'teleop_rejected'
        assert published(node) == [(0.1, 0.2)]


# --- _read_teleop_settings -----------------------------------------------------------------

def _teleop_settings_node(clock=None, **overrides):
    values = dict(TELEOP_DEFAULTS)
    values.update({f'teleop.{k}': v for k, v in overrides.items()})
    node = object.__new__(bridge.MtrbWebBridge)
    node.get_parameter = lambda name: FakeParam(values[name])
    node.clock = clock if clock is not None else [0.0]   # the ROS clock of the node
    node._now_s = lambda: node.clock[0]
    return node


def _patch_monotonic(monkeypatch, start=0.0):
    """Replace the `time` module seen by the bridge: time.monotonic() reads mono[0].

    Only the bridge module is patched (not the global time module), so pytest and the
    test client keep their real clocks. Returns the mutable fake monotonic clock.
    """
    mono = [start]
    monkeypatch.setattr(bridge, 'time', NS(monotonic=lambda: mono[0]))
    return mono


def test_teleop_parameter_defaults_are_0_2_0_8_and_300_ms():
    defaults = dict(bridge.PARAMETERS)
    assert defaults['teleop.max_vx'] == 0.2
    assert defaults['teleop.max_wz'] == 0.8
    assert defaults['teleop.timeout_ms'] == 300
    assert defaults['teleop.enabled'] is True


def test_hard_ceilings_are_the_create3_limits():
    assert bridge.TELEOP_HARD_MAX_VX_MPS == 0.46
    assert bridge.TELEOP_HARD_MAX_WZ_RADPS == 1.9
    assert (bridge.TELEOP_MIN_TIMEOUT_MS, bridge.TELEOP_MAX_TIMEOUT_MS) == (50, 1000)


def test_default_settings_build_a_gate_with_the_default_limits_and_timeout(monkeypatch):
    mono = _patch_monotonic(monkeypatch)
    node = _teleop_settings_node()
    node._read_teleop_settings()
    gate = node._teleop_gate
    assert gate.on_teleop('a', tmsg(0, vx=9, wz=-9), False).twist == (0.2, -0.8)
    mono[0] += 0.3
    assert gate.watchdog_tick().kind == 'idle'
    mono[0] += 0.0001
    assert gate.watchdog_tick().kind == 'zero'


def test_the_gate_uses_time_monotonic_by_default_and_not_the_ros_clock(monkeypatch):
    mono = _patch_monotonic(monkeypatch)
    node = _teleop_settings_node()
    node._read_teleop_settings()
    node._teleop_gate.on_teleop('a', tmsg(0), False)
    assert node._teleop_gate.owner_age_s == 0.0
    node.clock[0] += 5000.0          # the ROS clock moves: the gate must not notice
    assert node._teleop_gate.owner_age_s == 0.0
    mono[0] += 0.125                 # the monotonic clock moves: the gate follows it
    assert node._teleop_gate.owner_age_s == pytest.approx(0.125)


def test_the_gate_clock_is_the_real_time_monotonic_when_not_patched():
    import time
    node = _teleop_settings_node()
    node._read_teleop_settings()
    assert node._teleop_gate._clock is time.monotonic


def test_a_ros_clock_jump_backwards_does_not_freeze_the_gate_timeout(monkeypatch):
    node = make_teleop_bridge(monkeypatch, gate=False)
    node.get_parameter = lambda name: FakeParam(TELEOP_DEFAULTS[name])
    mono = _patch_monotonic(monkeypatch)
    node._read_teleop_settings()
    assert node.teleop(1, tmsg(0)) is None
    node.clock[0] -= 1.0e6           # NTP step backwards of the ROS/system clock
    mono[0] += 0.3001                # but the command is older than the 300 ms timeout
    node._teleop_watchdog()
    assert published(node) == [(0.1, 0.2)] + [(0.0, 0.0)] * ZERO_BURST


def test_a_ros_clock_jump_forwards_does_not_expire_a_fresh_command(monkeypatch):
    node = make_teleop_bridge(monkeypatch, gate=False)
    node.get_parameter = lambda name: FakeParam(TELEOP_DEFAULTS[name])
    mono = _patch_monotonic(monkeypatch)
    node._read_teleop_settings()
    assert node.teleop(1, tmsg(0)) is None
    node.clock[0] += 1.0e6           # forward jump of the ROS clock
    mono[0] += 0.1                   # real time: the command is still fresh
    node._teleop_watchdog()
    assert published(node) == [(0.1, 0.2)]


def test_the_teleop_watchdog_timer_uses_a_steady_clock():
    node = _io_node()
    calls = [c for c in node.create_timer.call_args_list
             if c.args[1].__name__ == '_teleop_watchdog']
    assert len(calls) == 1
    clock = calls[0].kwargs.get('clock')
    assert isinstance(clock, bridge.Clock)
    assert clock.clock_type == bridge.ClockType.STEADY_TIME


@pytest.mark.parametrize('max_vx, max_wz', [(0.46, 1.9), (0.05, 0.1), (1e-6, 1e-6), (0.1, 1)])
def test_valid_limits_up_to_the_hard_ceilings_are_accepted(max_vx, max_wz):
    node = _teleop_settings_node(max_vx=max_vx, max_wz=max_wz)
    node._read_teleop_settings()
    twist = node._teleop_gate.on_teleop('a', tmsg(0, vx=99, wz=99), False).twist
    assert twist == (float(max_vx), float(max_wz))


@pytest.mark.parametrize('bad', [0, 0.0, -0.1, 0.4601, 0.47, 1, 5.0, float('nan'),
                                 float('inf'), -float('inf'), True, False, '0.2', None, [0.2]])
def test_invalid_max_vx_raises_config_error_naming_the_parameter(bad):
    with pytest.raises(bridge.ConfigError, match='teleop.max_vx'):
        _teleop_settings_node(max_vx=bad)._read_teleop_settings()


@pytest.mark.parametrize('bad', [0, 0.0, -0.1, 1.9001, 2, 10.0, float('nan'), float('inf'),
                                 True, False, '0.8', None])
def test_invalid_max_wz_raises_config_error_naming_the_parameter(bad):
    with pytest.raises(bridge.ConfigError, match='teleop.max_wz'):
        _teleop_settings_node(max_wz=bad)._read_teleop_settings()


@pytest.mark.parametrize('timeout_ms', [50, 100, 300, 1000])
def test_valid_timeouts_are_accepted_and_applied(monkeypatch, timeout_ms):
    mono = _patch_monotonic(monkeypatch)
    node = _teleop_settings_node(timeout_ms=timeout_ms)
    node._read_teleop_settings()
    node._teleop_gate.on_teleop('a', tmsg(0), False)
    mono[0] += timeout_ms / 1000.0
    assert node._teleop_gate.watchdog_tick().kind == 'idle'
    mono[0] += 0.0001
    assert node._teleop_gate.watchdog_tick().kind == 'zero'


@pytest.mark.parametrize('bad', [49, 1001, 0, -300, 300.0, 300.5, 0.3, True, False, '300',
                                 None, float('nan'), float('inf'), 10 ** 9])
def test_invalid_timeout_ms_raises_config_error_naming_the_parameter(bad):
    with pytest.raises(bridge.ConfigError, match='teleop.timeout_ms'):
        _teleop_settings_node(timeout_ms=bad)._read_teleop_settings()


@pytest.mark.parametrize('bad', ['true', 'yes', 1, 0, None, 1.0, [True]])
def test_enabled_that_is_not_a_bool_raises_config_error(bad):
    with pytest.raises(bridge.ConfigError, match='teleop.enabled'):
        _teleop_settings_node(enabled=bad)._read_teleop_settings()


def test_enabled_false_builds_a_gate_that_refuses_everything():
    node = _teleop_settings_node(enabled=False)
    node._read_teleop_settings()
    assert node._teleop_gate.on_teleop('a', tmsg(0), False).error[0] == 'teleop_disabled'


def test_a_failed_read_leaves_no_gate_behind():
    node = _teleop_settings_node(max_vx=9.0)
    with pytest.raises(bridge.ConfigError):
        node._read_teleop_settings()
    assert node._teleop_gate is None


def test_init_raises_config_error_for_an_invalid_teleop_parameter():
    with pytest.raises(bridge.ConfigError):
        _init_node({'teleop.max_vx': 0.5})


def test_init_builds_the_gate_with_the_default_parameters():
    node, _ = _init_node({})
    assert node._teleop_gate is not None
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=0))  # public rclpy clock API
    assert node._teleop_gate.on_teleop('a', tmsg(0, vx=9), False).twist[0] == 0.2


def test_the_shipped_param_files_hold_valid_teleop_values():
    import os
    import yaml
    base = os.path.join(os.path.dirname(__file__), '..', 'param')
    for name in ('web.yaml', 'web.example.yaml'):
        with open(os.path.join(base, name), encoding='utf-8') as handle:
            params = yaml.safe_load(handle)['mtrb_web_bridge']['ros__parameters']['teleop']
        node = _teleop_settings_node(**params)
        node._read_teleop_settings()                 # raises ConfigError if out of range
        assert params['max_vx'] <= bridge.TELEOP_HARD_MAX_VX_MPS
        assert params['max_wz'] <= bridge.TELEOP_HARD_MAX_WZ_RADPS
        assert params['timeout_ms'] <= 300 and isinstance(params['enabled'], bool)


# --- no sensitive data in the logs ---------------------------------------------------------

def _all_log_text(node):
    return ' '.join(str(call) for call in node.logger.method_calls)


def test_logs_never_contain_teleop_values_payloads_or_tokens(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    secret_msg = {**tmsg(987654321, vx=0.123456, wz=-0.654321), 'token': LEAK_TOKEN}
    node.teleop(41, secret_msg)
    node.clock[0] += 0.5
    node.teleop(41, {**tmsg(987654322), 'vx': 'PAYLOAD-' + LEAK_TOKEN})   # invalid
    node.teleop(42, {**secret_msg, 'seq': 5})                             # lease held
    node.teleop(43, secret_msg)                                           # rejected
    node.send_goal(1.0, 2.0, 0.5)
    node.teleop(41, secret_msg)                                           # busy
    node._cmd_vel_nav_pub.always_fail = True
    node.stop()
    node.teleop_disconnect(42)
    text = _all_log_text(node)
    assert text                                    # the logs are not empty: the test can fail
    for forbidden in ('0.123456', '0.654321', '987654321', '987654322', 'PAYLOAD',
                      LEAK_TOKEN):
        assert forbidden not in text, forbidden


def test_rejection_warnings_are_throttled_and_show_only_the_error_code(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.send_goal(1.0, 2.0, 0.5)
    for seq in range(50):
        node.teleop(1, tmsg(seq))
    rejections = [m for m in _warnings(node) if 'Teleop rejected' in m]
    assert rejections == ['Teleop rejected (teleop_rejected)']


def test_teleop_logging_in_the_source_never_references_message_or_velocity_variables():
    import ast
    import inspect
    tree = ast.parse(inspect.getsource(bridge))
    cls = [n for n in tree.body if isinstance(n, ast.ClassDef)
           and n.name == 'MtrbWebBridge'][0]
    teleop_methods = {'teleop', 'teleop_disconnect', 'stop', '_stop_teleop_only',
                      '_teleop_stop_if_owned', '_teleop_execute', '_teleop_watchdog',
                      '_publish_cmd_vel_nav', '_nav_busy', '_read_teleop_settings'}
    forbidden = {'message', 'vx_mps', 'wz_radps', 'twist', 'vx', 'wz', 'seq', 'msg', 'token'}
    checked = 0
    for method in cls.body:
        if not (isinstance(method, ast.FunctionDef) and method.name in teleop_methods):
            continue
        for call in ast.walk(method):
            if not isinstance(call, ast.Call):
                continue
            name = getattr(call.func, 'attr', None)
            if name not in ('debug', 'info', 'warning', 'error', 'fatal', '_warn_throttled'):
                continue
            checked += 1
            used = {n.id for arg in call.args for n in ast.walk(arg) if isinstance(n, ast.Name)}
            assert not used & forbidden, (method.name, used & forbidden)
    assert checked >= 6


# --- repeated stops: the cancel-all is coalesced, the zeros never are (phase 4) ---------------
# SAFETY: every stop MUST publish its zero burst; only the cancel-all (which cancels every
# goal of Nav2) is limited to one per STOP_CANCEL_ALL_MIN_PERIOD_S. Tracked goals and orphans
# are cancelled on every stop. The coalescing period is measured with time.monotonic, which the
# tests replace in the bridge module (nothing sleeps).

def _stop_node(monkeypatch, start=10.0):
    mono = _patch_monotonic(monkeypatch, start=start)
    return make_teleop_bridge(monkeypatch), mono


def cancel_all_count(node):
    return len(cancel_client(node).requests)


def test_the_stop_cancel_all_period_is_half_a_second():
    assert bridge.STOP_CANCEL_ALL_MIN_PERIOD_S == 0.5


def test_repeated_stops_within_the_period_send_a_single_cancel_all(monkeypatch):
    node, mono = _stop_node(monkeypatch)
    node.stop()
    mono[0] += 0.1
    node.stop()
    mono[0] += 0.25
    node.stop()
    assert cancel_all_count(node) == 1
    assert_cancel_all_request(cancel_client(node).requests[0])


def test_every_repeated_stop_still_publishes_its_zero_burst(monkeypatch):
    node, mono = _stop_node(monkeypatch)
    for _ in range(4):
        node.stop()
        mono[0] += 0.05
    assert published(node) == [(0.0, 0.0)] * (4 * ZERO_BURST)
    assert cancel_all_count(node) == 1


def test_a_stop_publishes_its_zeros_before_the_cancel_all_even_when_coalesced(monkeypatch):
    node, mono = _stop_node(monkeypatch)
    node.stop()
    del node.events[:]
    mono[0] += 0.1
    node.stop()
    assert [e[0] for e in node.events] == ['pub'] * ZERO_BURST


def test_a_stop_just_before_the_period_ends_is_still_coalesced(monkeypatch):
    node, mono = _stop_node(monkeypatch)
    node.stop()
    mono[0] += 0.4999
    node.stop()
    assert cancel_all_count(node) == 1


def test_a_stop_after_the_period_sends_another_cancel_all(monkeypatch):
    node, mono = _stop_node(monkeypatch)
    node.stop()
    mono[0] += bridge.STOP_CANCEL_ALL_MIN_PERIOD_S
    node.stop()
    assert cancel_all_count(node) == 2
    mono[0] += 0.4
    node.stop()                      # coalesced against the second one
    assert cancel_all_count(node) == 2
    mono[0] += 0.1
    node.stop()
    assert cancel_all_count(node) == 3


def test_the_ros_clock_does_not_decide_the_coalescing(monkeypatch):
    node, mono = _stop_node(monkeypatch)
    node.stop()
    node.clock[0] += 1000.0          # ROS time far ahead, monotonic time unchanged
    node.stop()
    assert cancel_all_count(node) == 1
    node.clock[0] -= 5000.0          # ROS time jumps backwards, monotonic advances
    mono[0] += 0.6
    node.stop()
    assert cancel_all_count(node) == 2


def test_a_stop_with_a_tracked_goal_cancels_it_every_time_and_sends_no_cancel_all(monkeypatch):
    node, mono = _stop_node(monkeypatch)
    handle = accept_goal(node)
    for _ in range(3):
        node.stop()
        mono[0] += 0.05
    assert [e for e in node.events if e[0] == 'cancel'] == [('cancel',)] * 3
    assert handle.cancel_calls == 3
    assert cancel_all_count(node) == 0
    assert zero_count(node) == 3 * ZERO_BURST


def test_a_tracked_goal_is_cancelled_by_a_stop_right_after_a_coalesced_cancel_all(monkeypatch):
    node, mono = _stop_node(monkeypatch)
    node.stop()                      # arms and sends the cancel-all at t = 10.0
    assert cancel_all_count(node) == 1
    handle = accept_goal(node)       # a goal appears 0 s later
    mono[0] += 0.1
    node.stop()
    assert handle.cancel_calls == 1  # the goal is NOT left driving by the coalescing
    assert cancel_all_count(node) == 1


def test_a_stop_with_orphans_cancels_them_every_time_even_inside_the_period(monkeypatch):
    node, mono = _stop_node(monkeypatch)
    node.stop()                      # a cancel-all was armed at t = 10.0
    handle = make_orphan(node)       # then a goal became an orphan (no handle result)
    base = handle.cancel_calls
    for _ in range(3):
        mono[0] += 0.05
        node.stop()
    assert handle.cancel_calls == base + 3
    assert cancel_all_count(node) == 1    # no new cancel-all while orphans exist
    assert zero_count(node) == 4 * ZERO_BURST


def test_an_api_cancel_is_never_coalesced(monkeypatch):
    node, mono = _stop_node(monkeypatch)
    node.stop()
    mono[0] += 0.1
    node.cancel()
    mono[0] += 0.1
    node.cancel()
    assert cancel_all_count(node) == 3


def test_an_api_cancel_does_not_publish_zeros(monkeypatch):
    node, _ = _stop_node(monkeypatch)
    node.cancel()
    assert published(node) == []


def test_the_stop_context_does_not_leak_to_other_threads(monkeypatch):
    node, _ = _stop_node(monkeypatch)
    seen = []
    original = node._request_cancel_all

    def request_with_a_cancel_from_another_thread(stamp_ns):
        if not seen:
            seen.append(True)
            other = threading.Thread(target=node.cancel)   # API cancel, same instant
            other.start()
            other.join()
        return original(stamp_ns)

    node._request_cancel_all = request_with_a_cancel_from_another_thread
    node.stop()
    assert cancel_all_count(node) == 2   # the other thread's cancel was not coalesced


def test_a_stop_in_another_thread_does_not_coalesce_a_later_api_cancel(monkeypatch):
    node, _ = _stop_node(monkeypatch)
    worker = threading.Thread(target=node.stop)
    worker.start()
    worker.join()
    node.cancel()
    assert cancel_all_count(node) == 2


def test_the_stop_context_is_cleared_after_an_exception_in_cancel(monkeypatch):
    node, mono = _stop_node(monkeypatch)
    node.stop()                                    # armed at t = 10.0
    real_cancel = node.cancel

    def failing_cancel():
        raise RuntimeError('boom')

    node.cancel = failing_cancel
    mono[0] += 0.1
    node.stop()                                    # must not raise, zeros still published
    assert zero_count(node) == 2 * ZERO_BURST
    node.cancel = real_cancel
    mono[0] += 0.1
    node.cancel()                                  # API cancel: would be coalesced if leaked
    assert cancel_all_count(node) == 2


def test_the_stop_context_is_cleared_after_a_failure_inside_the_real_cancel(monkeypatch):
    node, mono = _stop_node(monkeypatch)
    real_request = node._request_cancel_all

    def failing_request(stamp_ns):
        raise RuntimeError('service exploded')

    node._request_cancel_all = failing_request
    node.stop()                                    # logged, never raised
    assert zero_count(node) == ZERO_BURST
    node._request_cancel_all = real_request
    mono[0] += 0.1
    node.cancel()
    assert cancel_all_count(node) == 1


# --- main(): shutdown tries the last zero burst ---------------------------------------------

class FakeRclpy:
    """Public rclpy module API used by main(): init, ok, shutdown, logging.get_logger."""

    def __init__(self, ok=True):
        self.ok_value = ok
        self.calls = []
        self.errors = []
        self.logging = NS(get_logger=lambda name: NS(error=self.errors.append))

    def init(self, args=None):
        self.calls.append('init')

    def ok(self):
        return self.ok_value

    def shutdown(self):
        self.calls.append('shutdown')


def _main_setup(monkeypatch, spin=None, ok=True, build_error=None):
    """Run main() against a node built like the teleop tests; returns (node, rclpy, result)."""
    node = make_teleop_bridge(monkeypatch)
    rclpy_double = FakeRclpy(ok=ok)

    class Executor:
        def __init__(self, num_threads=None):
            pass

        def add_node(self, added):
            assert added is node

        def spin(self):
            node.events.append(('spin',))
            if spin is not None:
                raise spin

    def build():
        if build_error is not None:
            raise build_error
        return node

    node.start_server = lambda: node.events.append(('start_server',))
    node.stop_server = lambda: node.events.append(('stop_server',))
    node.destroy_node = lambda: node.events.append(('destroy_node',))
    monkeypatch.setattr(bridge, 'rclpy', rclpy_double)
    monkeypatch.setattr(bridge, 'MtrbWebBridge', build)
    monkeypatch.setattr(bridge, 'MultiThreadedExecutor', Executor)
    return node, rclpy_double


def _kinds(node):
    return [e[0] for e in node.events]


def test_main_publishes_the_last_zero_burst_at_shutdown_before_stopping_the_server(
        monkeypatch):
    node, rclpy_double = _main_setup(monkeypatch, spin=KeyboardInterrupt())
    assert bridge.main() == 0
    assert _kinds(node) == ['start_server', 'spin'] + ['pub'] * ZERO_BURST + [
        'stop_server', 'destroy_node']
    assert published(node) == [(0.0, 0.0)] * ZERO_BURST
    assert rclpy_double.calls == ['init', 'shutdown']


def test_main_publishes_the_last_zero_burst_also_when_the_spin_dies_with_an_error(monkeypatch):
    node, rclpy_double = _main_setup(monkeypatch, spin=RuntimeError('executor died'))
    with pytest.raises(RuntimeError, match='executor died'):
        bridge.main()
    assert published(node) == [(0.0, 0.0)] * ZERO_BURST
    assert _kinds(node)[-2:] == ['stop_server', 'destroy_node']
    assert rclpy_double.calls == ['init', 'shutdown']


def test_main_a_failing_publisher_does_not_break_the_shutdown(monkeypatch):
    node, rclpy_double = _main_setup(monkeypatch, spin=KeyboardInterrupt())
    node._cmd_vel_nav_pub.always_fail = True        # e.g. the context is already shut down
    assert bridge.main() == 0
    assert node._cmd_vel_nav_pub.attempts == ZERO_BURST   # it still tried every zero
    assert _kinds(node)[-2:] == ['stop_server', 'destroy_node']
    assert rclpy_double.calls == ['init', 'shutdown']


def test_main_with_the_context_already_shut_down_still_stops_server_and_node(monkeypatch):
    node, rclpy_double = _main_setup(monkeypatch, spin=KeyboardInterrupt(), ok=False)
    node._cmd_vel_nav_pub.always_fail = True
    assert bridge.main() == 0
    assert _kinds(node)[-2:] == ['stop_server', 'destroy_node']
    assert rclpy_double.calls == ['init']           # nothing to shut down twice


def test_main_an_exception_in_the_final_stop_is_logged_and_does_not_break_the_shutdown(
        monkeypatch):
    node, rclpy_double = _main_setup(monkeypatch, spin=KeyboardInterrupt())

    def exploding_stop():
        raise RuntimeError('publisher gone')

    node._stop_teleop_only = exploding_stop
    assert bridge.main() == 0
    assert _kinds(node)[-2:] == ['stop_server', 'destroy_node']
    assert rclpy_double.calls == ['init', 'shutdown']
    assert len(rclpy_double.errors) == 1
    assert 'publisher gone' in rclpy_double.errors[0]


def test_main_with_an_invalid_configuration_returns_1_and_touches_no_node(monkeypatch):
    node, rclpy_double = _main_setup(monkeypatch, build_error=bridge.ConfigError('bad port'))
    assert bridge.main() == 1
    assert node.events == []                         # no zeros, nothing to stop
    assert rclpy_double.errors == ['bad port']
    assert rclpy_double.calls == ['init', 'shutdown']


# =============================================================================================
# Phase 5: odometry age, lifecycle polling, map services (MapPort) and new settings.
#
# The service clients and futures below are fakes that expose ONLY the public rclpy API that
# the node may use (checked against the rclpy Jazzy sources, rclpy/client.py and task.py):
#   Client.service_is_ready(), Client.call_async(request) -> Future,
#   Client.remove_pending_request(future);
#   Future.add_done_callback(cb), done(), result(), exception(), set_result(), set_exception().
# Any other attribute access on them raises AttributeError, so a call to an API that does not
# exist (the lesson of the earlier phases) fails here instead of on the robot. NOT covered
# (needs real ROS / the Pi): the real DDS discovery, the executor scheduling the callbacks and
# slam_toolbox writing the files (risk A-2 of the spec).
# =============================================================================================

WAIT_S = 10.0   # upper bound while waiting for another thread (never reached when all is well)


class RclFuture:
    """Public-API model of rclpy.task.Future (callbacks run at once when already done)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._done = False
        self._result = None
        self._exception = None
        self._callbacks = []

    def done(self):
        return self._done

    def result(self):
        if self._exception is not None:
            raise self._exception
        return self._result

    def exception(self):
        return self._exception

    def add_done_callback(self, callback):
        with self._lock:
            if not self._done:
                self._callbacks.append(callback)
                return
        callback(self)

    def _finish(self):
        with self._lock:
            self._done = True
            callbacks, self._callbacks = self._callbacks, []
        for callback in callbacks:
            callback(self)

    def set_result(self, result):
        self._result = result
        self._finish()

    def set_exception(self, exception):
        self._exception = exception
        self._finish()


class SrvClient:
    """Public-API model of rclpy.client.Client (see the block comment above)."""

    def __init__(self, srv_type, name=''):
        self.srv_type = srv_type
        self.name = name
        self.ready = True
        self.call_error = None
        self.requests = []
        self.futures = []
        self.pending = []           # futures that can still receive an answer
        self.removed = []           # futures passed to remove_pending_request, in order
        self.ready_error = None
        self.on_call = None         # callable(future) run inside call_async
        self.probe = None           # callable run at the start of every public method
        self.auto_answer = None     # value: call_async returns an already answered future

    def _probe(self):
        if self.probe is not None:
            self.probe()

    def service_is_ready(self):
        self._probe()
        if self.ready_error is not None:
            raise self.ready_error
        return self.ready

    def call_async(self, request):
        self._probe()
        if not isinstance(request, self.srv_type.Request):   # as the real Client does
            raise TypeError()
        if self.call_error is not None:
            raise self.call_error
        future = RclFuture()
        self.requests.append(request)
        self.futures.append(future)
        self.pending.append(future)
        if self.auto_answer is not None:
            self.answer(future, self.auto_answer)
        if self.on_call is not None:
            self.on_call(future)
        return future

    def remove_pending_request(self, future):
        self._probe()
        self.removed.append(future)
        if future in self.pending:
            self.pending.remove(future)

    def answer(self, future, response):
        """Deliver an answer like the executor does; dropped if the request was removed."""
        if future not in self.pending:
            return False
        self.pending.remove(future)
        future.set_result(response)
        return True

    def fail(self, future, error):
        if future in self.pending:
            self.pending.remove(future)
            future.set_exception(error)


def lifecycle_state(label):
    return NS(current_state=NS(label=label))


def make_lifecycle_node(names=('amcl', 'planner_server'), now=100.0):
    """Node built like production: _init_status_io with a fake create_client."""
    node = make_bridge(now)
    node._status_nodes = tuple(names)
    node.created = {}

    def create_client(srv_type, name, callback_group=None):
        node.created[name] = SrvClient(srv_type, name)
        return node.created[name]

    node.create_client = create_client
    node.create_subscription = mock.Mock()
    node.create_timer = mock.Mock()
    node._init_status_io()
    return node


def lifecycle_client(node, name):
    return node._lifecycle_clients[name]


def states(node):
    return node._lifecycle_snapshot()


def run_in_thread(function, *args):
    """Run `function` in a daemon thread; return (thread, box) with value/error in box."""
    box = {}

    def target():
        try:
            box['value'] = function(*args)
        except BaseException as err:  # noqa: B902 - handed to the test
            box['error'] = err

    thread = threading.Thread(target=target, daemon=True)
    thread.start()
    return thread, box


# --- odometry age ------------------------------------------------------------------------

def test_status_odom_age_is_null_until_an_odometry_message_arrives():
    node = make_bridge()
    node._broadcast_status()
    message = drain(node)[0]
    assert 'odom_age' in message and message['odom_age'] is None


def test_status_odom_age_is_the_seconds_since_the_last_odometry_message():
    node = make_bridge(100.0)
    node._on_odom(object())             # the content is never read
    node.clock[0] = 102.5
    node._broadcast_status()
    assert drain(node)[0]['odom_age'] == 2.5


def test_status_odom_age_is_zero_right_after_a_message_and_resets_with_a_newer_one():
    node = make_bridge(100.0)
    node._on_odom(None)
    node._broadcast_status()
    assert drain(node)[0]['odom_age'] == 0.0
    node.clock[0] = 110.0
    node._on_odom(None)
    node.clock[0] = 110.25
    node._broadcast_status()
    assert drain(node)[0]['odom_age'] == 0.25


def test_status_odom_age_is_rounded_to_two_decimals():
    node = make_bridge(100.0)
    node._on_odom(None)
    node.clock[0] = 101.23456
    node._broadcast_status()
    assert drain(node)[0]['odom_age'] == 1.23


def test_status_odom_age_is_never_negative_when_the_clock_goes_back():
    node = make_bridge(100.0)
    node._on_odom(None)
    node.clock[0] = 90.0
    node._broadcast_status()
    assert drain(node)[0]['odom_age'] == 0.0


def test_status_odom_age_is_independent_of_the_scan_age():
    node = make_bridge(100.0)
    node._on_odom(None)
    node.clock[0] = 103.0
    node._on_scan(_scan())
    node.clock[0] = 104.0
    node._broadcast_status()
    message = drain(node)[0]
    assert (message['odom_age'], message['scan_age']) == (4.0, 1.0)


def test_status_message_has_exactly_the_contracted_fields():
    node = make_bridge()
    node._broadcast_status()
    assert set(drain(node)[0]) == {'type', 'battery_pct', 'mode', 'nodes', 'scan_age',
                                   'odom_age', 'dock'}


def test_init_ros_io_subscribes_to_odom_with_sensor_data_qos_and_reads_only_its_arrival():
    node = _io_node()
    calls = [c for c in node.create_subscription.call_args_list if c.args[1] == 'odom']
    assert len(calls) == 1
    assert calls[0].args[0] is bridge.Odometry
    assert calls[0].args[2] == node._on_odom
    assert calls[0].args[3] is bridge.qos_profile_sensor_data
    assert not calls[0].args[1].startswith('/')      # relative: /<ns>/odom


# --- lifecycle clients and timer ---------------------------------------------------------

def test_init_ros_io_creates_one_get_state_client_per_node_with_relative_names():
    node = _io_node()
    calls = {c.args[1]: c.args[0] for c in node.create_client.call_args_list
             if c.args[1].endswith('/get_state')}
    assert calls == {'map_server/get_state': bridge.GetState, 'amcl/get_state': bridge.GetState}
    assert set(node._lifecycle_clients) == {'map_server', 'amcl'}
    for name, client in node._lifecycle_clients.items():
        assert client is node.created_clients[f'{name}/get_state']


def test_init_ros_io_creates_the_two_slam_toolbox_service_clients():
    node = _io_node()
    by_name = {c.args[1]: c for c in node.create_client.call_args_list}
    assert by_name['slam_toolbox/save_map'].args[0] is bridge.SaveMap
    assert by_name['slam_toolbox/serialize_map'].args[0] is bridge.SerializePoseGraph
    assert node._save_map_client is node.created_clients['slam_toolbox/save_map']
    assert node._serialize_client is node.created_clients['slam_toolbox/serialize_map']
    assert all(not name.startswith('/') for name in by_name)
    groups = [by_name[n].kwargs.get('callback_group')
              for n in ('slam_toolbox/save_map', 'slam_toolbox/serialize_map')]
    assert all(isinstance(g, bridge.ReentrantCallbackGroup) for g in groups)


def test_init_ros_io_creates_exactly_the_expected_clients():
    node = _io_node()
    names = sorted(c.args[1] for c in node.create_client.call_args_list)
    assert names == sorted([
        'navigate_to_pose/_action/cancel_goal', 'map_server/get_state', 'amcl/get_state',
        'slam_toolbox/save_map', 'slam_toolbox/serialize_map'])


def test_lifecycle_polling_has_its_own_mutually_exclusive_callback_group():
    node = _io_node()
    timer = [c for c in node.create_timer.call_args_list
             if c.args[1].__name__ == '_poll_lifecycle'][0]
    group = timer.kwargs['callback_group']
    assert isinstance(group, bridge.MutuallyExclusiveCallbackGroup)
    for name in ('map_server', 'amcl'):
        call = [c for c in node.create_client.call_args_list
                if c.args[1] == f'{name}/get_state'][0]
        assert call.kwargs['callback_group'] is group
    assert timer.args[0] == pytest.approx(2.0)


def test_lifecycle_states_start_unknown_for_every_watched_node():
    node = make_lifecycle_node(('a', 'b', 'c'))
    assert states(node) == {'a': 'unknown', 'b': 'unknown', 'c': 'unknown'}
    assert list(states(node)) == ['a', 'b', 'c']


def test_no_watched_nodes_gives_empty_nodes_in_status():
    node = make_lifecycle_node(())
    node._poll_lifecycle()
    node._broadcast_status()
    assert drain(node)[0]['nodes'] == {}


# --- _poll_lifecycle ---------------------------------------------------------------------

def test_poll_asks_every_ready_node_once_with_a_get_state_request():
    node = make_lifecycle_node(('amcl', 'planner_server'))
    node._poll_lifecycle()
    for name in ('amcl', 'planner_server'):
        client = lifecycle_client(node, name)
        assert len(client.requests) == 1
        assert isinstance(client.requests[0], bridge.GetState.Request)
    assert states(node) == {'amcl': 'unknown', 'planner_server': 'unknown'}   # not answered yet


@pytest.mark.parametrize('label', sorted(bridge.LIFECYCLE_LABELS))
def test_poll_stores_every_valid_lifecycle_label(label):
    node = make_lifecycle_node(('amcl',))
    node._poll_lifecycle()
    client = lifecycle_client(node, 'amcl')
    client.answer(client.futures[0], lifecycle_state(label))
    assert states(node) == {'amcl': label}


def test_poll_results_reach_the_status_message_nodes_field():
    node = make_lifecycle_node(('amcl', 'bt_navigator'))
    node._poll_lifecycle()
    client = lifecycle_client(node, 'amcl')
    client.answer(client.futures[0], lifecycle_state('active'))
    lifecycle_client(node, 'bt_navigator').ready = False
    node._poll_lifecycle()      # amcl is asked again (its label stays until the next tick)
    node._broadcast_status()
    assert drain(node)[0]['nodes'] == {'amcl': 'active', 'bt_navigator': 'unavailable'}


@pytest.mark.parametrize('label', ['ACTIVE', ' Active ', 'Inactive\n'])
def test_poll_normalises_case_and_whitespace_of_the_label(label):
    node = make_lifecycle_node(('amcl',))
    node._poll_lifecycle()
    client = lifecycle_client(node, 'amcl')
    client.answer(client.futures[0], lifecycle_state(label))
    assert states(node)['amcl'] == label.strip().lower()


@pytest.mark.parametrize('label', ['weird', '', 'running', 'active; DROP', None, 5, 'x' * 500,
                                   'actív'])
def test_poll_maps_an_unknown_label_to_unknown(label):
    node = make_lifecycle_node(('amcl',))
    node._poll_lifecycle()
    client = lifecycle_client(node, 'amcl')
    client.answer(client.futures[0], lifecycle_state(label))
    assert states(node) == {'amcl': 'unknown'}


@pytest.mark.parametrize('response', [None, NS(), NS(current_state=None), 'garbage', 5])
def test_poll_maps_a_malformed_response_to_unknown(response):
    node = make_lifecycle_node(('amcl',))
    node._poll_lifecycle()
    client = lifecycle_client(node, 'amcl')
    client.answer(client.futures[0], response)
    assert states(node) == {'amcl': 'unknown'}


def test_poll_maps_an_answer_with_an_exception_to_unknown():
    node = make_lifecycle_node(('amcl',))
    node._poll_lifecycle()
    client = lifecycle_client(node, 'amcl')
    client.answer(client.futures[0], lifecycle_state('active'))
    assert states(node)['amcl'] == 'active'
    node._poll_lifecycle()
    client.fail(client.futures[1], RuntimeError('service died'))
    assert states(node)['amcl'] == 'unknown'


def test_poll_a_service_that_is_not_ready_is_unavailable_and_is_not_called():
    node = make_lifecycle_node(('amcl',))
    lifecycle_client(node, 'amcl').ready = False
    node._poll_lifecycle()
    assert states(node) == {'amcl': 'unavailable'}
    assert lifecycle_client(node, 'amcl').requests == []
    assert node._lifecycle_pending == {}


def test_poll_a_node_that_disappears_becomes_unavailable_and_returns_when_it_is_back():
    node = make_lifecycle_node(('amcl',))
    client = lifecycle_client(node, 'amcl')
    node._poll_lifecycle()
    client.answer(client.futures[0], lifecycle_state('active'))
    client.ready = False
    node._poll_lifecycle()
    assert states(node)['amcl'] == 'unavailable'
    client.ready = True
    node._poll_lifecycle()
    client.answer(client.futures[-1], lifecycle_state('inactive'))
    assert states(node)['amcl'] == 'inactive'


@pytest.mark.parametrize('error', [RuntimeError('boom'), TypeError(), OSError('closed')])
def test_poll_a_call_async_that_raises_gives_unknown_and_never_stops_the_timer(error):
    node = make_lifecycle_node(('amcl', 'planner_server'))
    lifecycle_client(node, 'amcl').call_error = error
    node._poll_lifecycle()                       # must not raise
    assert states(node)['amcl'] == 'unknown'
    assert len(lifecycle_client(node, 'planner_server').requests) == 1   # the others go on
    assert node._lifecycle_pending.keys() == {'planner_server'}
    assert node.logger.warning.called


def test_poll_a_service_is_ready_that_raises_is_handled_like_a_failed_query():
    node = make_lifecycle_node(('amcl',))
    lifecycle_client(node, 'amcl').ready_error = RuntimeError('context gone')
    node._poll_lifecycle()
    assert states(node) == {'amcl': 'unknown'}


def test_poll_failure_warnings_are_throttled():
    node = make_lifecycle_node(('amcl',))
    lifecycle_client(node, 'amcl').call_error = RuntimeError('boom')
    for _ in range(5):
        node._poll_lifecycle()
    assert node.logger.warning.call_count == 1
    node.clock[0] += bridge.LOG_THROTTLE_S + 1
    node._poll_lifecycle()
    assert node.logger.warning.call_count == 2


def test_poll_a_request_still_pending_is_removed_and_the_state_becomes_unknown():
    node = make_lifecycle_node(('amcl',))
    client = lifecycle_client(node, 'amcl')
    node._poll_lifecycle()
    client.answer(client.futures[0], lifecycle_state('active'))
    node._poll_lifecycle()                      # second request: never answered
    assert states(node)['amcl'] == 'active'     # the old label stays until a tick says otherwise
    node._poll_lifecycle()                      # third tick finds the second still pending
    assert client.removed == [client.futures[1]]   # real remove_pending_request, once
    assert states(node)['amcl'] == 'unknown'
    assert len(client.requests) == 3


def test_poll_pending_requests_do_not_accumulate_when_the_service_never_answers():
    node = make_lifecycle_node(('amcl', 'bt_navigator'))
    for _ in range(50):
        node._poll_lifecycle()
    for name in ('amcl', 'bt_navigator'):
        client = lifecycle_client(node, name)
        assert len(client.pending) == 1             # only the latest request is alive
        assert len(client.removed) == 49
    assert len(node._lifecycle_pending) == 2


def test_poll_a_late_answer_to_a_removed_request_is_dropped_by_rclpy_and_changes_nothing():
    node = make_lifecycle_node(('amcl',))
    client = lifecycle_client(node, 'amcl')
    node._poll_lifecycle()
    node._poll_lifecycle()                       # drops request 0
    assert client.answer(client.futures[0], lifecycle_state('active')) is False
    assert states(node)['amcl'] == 'unknown'


def test_poll_a_late_answer_with_an_old_sequence_never_overwrites_a_newer_state():
    # Defence in depth: even if an answer slipped through remove_pending_request (done
    # callback already scheduled on the executor), its sequence number is old.
    node = make_lifecycle_node(('amcl',))
    client = lifecycle_client(node, 'amcl')
    node._poll_lifecycle()
    old = client.futures[0]
    node._poll_lifecycle()
    node._poll_lifecycle()                       # state 'unknown', request 2 pending
    old.set_result(lifecycle_state('active'))    # bypasses the client: a late delivery
    node._on_lifecycle('amcl', 1, old)
    assert states(node)['amcl'] == 'unknown'
    client.answer(client.futures[2], lifecycle_state('inactive'))
    assert states(node)['amcl'] == 'inactive'
    node._on_lifecycle('amcl', 1, old)           # older again after a newer answer
    assert states(node)['amcl'] == 'inactive'


def test_poll_an_answer_clears_the_pending_entry_so_the_next_tick_removes_nothing():
    node = make_lifecycle_node(('amcl',))
    client = lifecycle_client(node, 'amcl')
    node._poll_lifecycle()
    client.answer(client.futures[0], lifecycle_state('active'))
    assert node._lifecycle_pending == {}
    node._poll_lifecycle()
    assert client.removed == []
    assert states(node)['amcl'] == 'active'


def test_poll_an_already_answered_future_is_handled_without_deadlock():
    node = make_lifecycle_node(('amcl',))
    lifecycle_client(node, 'amcl').auto_answer = lifecycle_state('active')
    thread, box = run_in_thread(node._poll_lifecycle)
    thread.join(WAIT_S)
    assert not thread.is_alive(), '_poll_lifecycle deadlocked on its own done callback'
    assert 'error' not in box
    assert states(node) == {'amcl': 'active'}
    assert node._lifecycle_pending == {}


def test_poll_never_holds_the_lifecycle_lock_while_it_talks_to_the_clients():
    node = make_lifecycle_node(('amcl', 'bt_navigator'))
    seen = []
    for name in ('amcl', 'bt_navigator'):
        lifecycle_client(node, name).probe = lambda: seen.append(node._lifecycle_lock.locked())
    node._poll_lifecycle()
    node._poll_lifecycle()      # the second tick also calls remove_pending_request
    assert seen and not any(seen)


def test_poll_never_holds_the_teleop_or_navigation_locks():
    node = make_lifecycle_node(('amcl',))
    seen = []
    lifecycle_client(node, 'amcl').probe = lambda: seen.append(
        (node._teleop_lock.locked(), node._nav_lock.locked()))
    node._poll_lifecycle()
    node._poll_lifecycle()
    assert seen and set(seen) == {(False, False)}


def test_poll_only_uses_the_public_client_api():
    # SrvClient has nothing but the public methods: any other call raises AttributeError.
    public = {name for name in dir(SrvClient) if not name.startswith('_')}
    assert {'service_is_ready', 'call_async', 'remove_pending_request'} <= public
    node = make_lifecycle_node(('amcl',))
    node._poll_lifecycle()
    node._poll_lifecycle()
    assert states(node)['amcl'] == 'unknown'


def test_the_lifecycle_snapshot_is_a_copy():
    node = make_lifecycle_node(('amcl',))
    snapshot = node._lifecycle_snapshot()
    snapshot['amcl'] = 'hacked'
    snapshot['extra'] = 'x'
    assert states(node) == {'amcl': 'unknown'}


def test_a_status_broadcast_does_not_wait_for_a_pending_lifecycle_request():
    node = make_lifecycle_node(('amcl',))
    node._poll_lifecycle()                       # pending forever
    thread, box = run_in_thread(node._broadcast_status)
    thread.join(WAIT_S)
    assert not thread.is_alive() and 'error' not in box
    assert drain(node)[0]['nodes'] == {'amcl': 'unknown'}


# --- _call_service -----------------------------------------------------------------------

call_service = bridge.MtrbWebBridge._call_service


def test_call_service_not_ready_raises_unavailable_without_sending():
    client = SrvClient(bridge.SaveMap)
    client.ready = False
    with pytest.raises(bridge.SlamUnavailableError):
        call_service(client, bridge.SaveMap.Request(), 1.0)
    assert client.requests == []


@pytest.mark.parametrize('error', [RuntimeError('x'), TypeError(), ValueError('y'),
                                   OSError('z')])
def test_call_service_a_call_async_that_raises_is_unavailable(error):
    client = SrvClient(bridge.SaveMap)
    client.call_error = error
    with pytest.raises(bridge.SlamUnavailableError):
        call_service(client, bridge.SaveMap.Request(), 1.0)


def test_call_service_returns_the_response_when_it_is_already_there():
    client = SrvClient(bridge.SaveMap)
    response = NS(result=0)
    client.auto_answer = response
    assert call_service(client, bridge.SaveMap.Request(), 0.0) is response


def test_call_service_returns_the_response_delivered_by_another_thread():
    client = SrvClient(bridge.SaveMap)
    sent = threading.Event()
    client.on_call = lambda future: sent.set()
    response = NS(result=0)
    thread, box = run_in_thread(call_service, client, bridge.SaveMap.Request(), WAIT_S)
    assert sent.wait(WAIT_S)
    client.answer(client.futures[0], response)
    thread.join(WAIT_S)
    assert not thread.is_alive()
    assert box == {'value': response}


def test_call_service_timeout_raises_timeout_and_removes_the_pending_request_once():
    client = SrvClient(bridge.SaveMap)
    with pytest.raises(bridge.SlamTimeoutError):
        call_service(client, bridge.SaveMap.Request(), 0.0)
    assert client.removed == [client.futures[0]]
    assert client.pending == []


def test_call_service_a_late_answer_after_the_timeout_is_dropped():
    client = SrvClient(bridge.SaveMap)
    with pytest.raises(bridge.SlamTimeoutError):
        call_service(client, bridge.SaveMap.Request(), 0.0)
    assert client.answer(client.futures[0], NS(result=0)) is False
    assert not client.futures[0].done()


def test_call_service_an_answer_with_an_exception_returns_none():
    client = SrvClient(bridge.SaveMap)
    client.on_call = lambda future: future.set_exception(RuntimeError('server died'))
    assert call_service(client, bridge.SaveMap.Request(), 1.0) is None


def test_call_service_removes_nothing_when_the_answer_arrives_in_time():
    client = SrvClient(bridge.SaveMap)
    client.auto_answer = NS(result=0)
    call_service(client, bridge.SaveMap.Request(), 1.0)
    assert client.removed == []


def test_call_service_does_not_need_the_node_and_takes_no_lock():
    # A staticmethod: nothing of the node (no _teleop_lock, _nav_lock) can be involved.
    assert isinstance(bridge.MtrbWebBridge.__dict__['_call_service'], staticmethod)


# --- MapPort of the node -----------------------------------------------------------------

def make_map_node(monkeypatch=None, teleop=False):
    node = make_teleop_bridge(monkeypatch) if teleop else make_bridge()
    node._save_map_client = SrvClient(bridge.SaveMap, 'slam_toolbox/save_map')
    node._serialize_client = SrvClient(bridge.SerializePoseGraph, 'slam_toolbox/serialize_map')
    return node


def test_the_node_is_a_map_port_that_overrides_every_method():
    assert issubclass(bridge.MtrbWebBridge, bridge.MapPort)
    for name in ('ready', 'save_map', 'serialize_posegraph'):
        assert getattr(bridge.MtrbWebBridge, name) is not getattr(bridge.MapPort, name)


@pytest.mark.parametrize('ready', [True, False])
def test_map_port_ready_follows_the_save_map_service(ready):
    node = make_map_node()
    node._save_map_client.ready = ready
    assert node.ready() is ready
    assert node._save_map_client.requests == []


def test_map_port_save_map_sends_the_base_path_as_a_string_message():
    node = make_map_node()
    node._save_map_client.auto_answer = NS(result=bridge.SaveMap.Response.RESULT_SUCCESS)
    assert node.save_map('/maps/lab', 1.0) is True
    [request] = node._save_map_client.requests
    assert request.name.data == '/maps/lab'
    assert node._serialize_client.requests == []


def test_map_port_save_map_false_when_slam_toolbox_reports_an_error():
    node = make_map_node()
    node._save_map_client.auto_answer = NS(result=1)
    assert node.save_map('/maps/lab', 1.0) is False


def test_map_port_save_map_false_when_the_answer_failed():
    node = make_map_node()
    node._save_map_client.on_call = lambda f: f.set_exception(RuntimeError('x'))
    assert node.save_map('/maps/lab', 1.0) is False


def test_map_port_save_map_propagates_unavailable_and_timeout():
    node = make_map_node()
    with pytest.raises(bridge.SlamTimeoutError):
        node.save_map('/maps/lab', 0.0)
    node._save_map_client.ready = False
    with pytest.raises(bridge.SlamUnavailableError):
        node.save_map('/maps/lab', 0.0)


def test_map_port_serialize_posegraph_sends_the_filename_and_maps_the_result():
    node = make_map_node()
    node._serialize_client.auto_answer = NS(
        result=bridge.SerializePoseGraph.Response.RESULT_SUCCESS)
    assert node.serialize_posegraph('/maps/lab', 1.0) is True
    [request] = node._serialize_client.requests
    assert request.filename == '/maps/lab'
    assert node._save_map_client.requests == []
    node._serialize_client.auto_answer = NS(result=1)
    assert node.serialize_posegraph('/maps/lab', 1.0) is False


def test_map_port_serialize_posegraph_propagates_unavailable_and_timeout():
    node = make_map_node()
    with pytest.raises(bridge.SlamTimeoutError):
        node.serialize_posegraph('/maps/lab', 0.0)
    node._serialize_client.ready = False
    with pytest.raises(bridge.SlamUnavailableError):
        node.serialize_posegraph('/maps/lab', 0.0)


def test_map_port_a_blocked_save_holds_no_node_lock_and_stop_still_publishes_zeros(
        monkeypatch):
    node = make_map_node(monkeypatch, teleop=True)
    sent = threading.Event()
    node._save_map_client.on_call = lambda future: sent.set()
    thread, box = run_in_thread(node.save_map, '/maps/lab', WAIT_S)
    stop_thread = None
    try:
        assert sent.wait(WAIT_S)
        # The HTTP thread is now waiting inside the service call: it must hold no lock.
        for lock in (node._teleop_lock, node._nav_lock, node._lifecycle_lock):
            assert lock.acquire(blocking=False), 'a map save holds a node lock'
            lock.release()
        stop_thread, stop_box = run_in_thread(node.stop)
        stop_thread.join(WAIT_S)
        assert not stop_thread.is_alive(), 'stop() waited for the map save'
        assert 'error' not in stop_box
        assert published(node)[:ZERO_BURST] == [(0.0, 0.0)] * ZERO_BURST
        assert thread.is_alive()                  # the save is still waiting for slam_toolbox
    finally:
        node._save_map_client.answer(node._save_map_client.futures[0], NS(result=0))
        thread.join(WAIT_S)
        if stop_thread is not None:
            stop_thread.join(WAIT_S)
    assert box == {'value': True}


def test_map_port_a_teleop_command_is_not_delayed_by_a_blocked_save(monkeypatch):
    node = make_map_node(monkeypatch, teleop=True)
    sent = threading.Event()
    node._serialize_client.on_call = lambda future: sent.set()
    thread, box = run_in_thread(node.serialize_posegraph, '/maps/lab', WAIT_S)
    teleop_thread = None
    try:
        assert sent.wait(WAIT_S)
        teleop_thread, teleop_box = run_in_thread(node.teleop, 1, tmsg(0))
        teleop_thread.join(WAIT_S)
        assert not teleop_thread.is_alive(), 'teleop waited for the map save'
        assert teleop_box == {'value': None}
        assert published(node) == [(0.1, 0.2)]
    finally:
        node._serialize_client.answer(node._serialize_client.futures[0], NS(result=0))
        thread.join(WAIT_S)
        if teleop_thread is not None:
            teleop_thread.join(WAIT_S)
    assert box == {'value': True}


def test_init_map_clients_uses_relative_service_names_and_a_reentrant_group():
    node = object.__new__(bridge.MtrbWebBridge)
    node.create_client = mock.Mock()
    node._init_map_clients()
    names = [c.args[1] for c in node.create_client.call_args_list]
    assert names == [bridge.SAVE_MAP_SERVICE, bridge.SERIALIZE_MAP_SERVICE]
    assert names == ['slam_toolbox/save_map', 'slam_toolbox/serialize_map']
    groups = [c.kwargs['callback_group'] for c in node.create_client.call_args_list]
    assert isinstance(groups[0], bridge.ReentrantCallbackGroup) and groups[0] is groups[1]


def test_log_map_event_routes_levels_to_the_node_logger():
    node = make_bridge()
    node._log_map_event('info', 'saved')
    node._log_map_event('warning', 'careful')
    node._log_map_event('error', 'bad')
    node._log_map_event('weird', 'default')
    assert [c.args[0] for c in node.logger.info.call_args_list] == ['saved', 'default']
    assert [c.args[0] for c in node.logger.warning.call_args_list] == ['careful']
    assert [c.args[0] for c in node.logger.error.call_args_list] == ['bad']


# --- settings: rates.lifecycle, status.nodes, map_dir, union by mode ---------------------

@pytest.fixture
def share(monkeypatch):
    monkeypatch.setattr(bridge, 'get_package_share_directory', lambda name: '/share')


@pytest.mark.parametrize('rate, period', [(0.5, 2.0), (0.05, 20.0), (0.25, 4.0), (0.1, 10.0),
                                          (0.2, 5.0)])
def test_read_settings_lifecycle_rate_sets_the_polling_period(rate, period):
    node = _settings_node(**{'rates.lifecycle': rate})
    node._read_settings()
    assert node._lifecycle_period_s == pytest.approx(period)


@pytest.mark.parametrize('bad', [0.51, 0.5001, 1, 1.0, 2.0, 10.0, 100, 0.0499, 0.04, 0.01,
                                 0, 0.0, -0.5, True, False, float('nan'), float('inf'),
                                 -float('inf'), 'x', '0.5', None, [0.5]])
def test_read_settings_invalid_lifecycle_rate_raises_naming_the_parameter(bad):
    with pytest.raises(bridge.ConfigError, match='rates.lifecycle'):
        _settings_node(**{'rates.lifecycle': bad})._read_settings()


def test_the_lifecycle_rate_limits_are_the_specified_ones():
    assert (bridge.LIFECYCLE_MIN_RATE_HZ, bridge.LIFECYCLE_MAX_RATE_HZ) == (0.05, 0.5)
    assert bridge.LIFECYCLE_MAX_NODES == 16


def test_default_lifecycle_rate_and_nodes_in_parameters_are_valid():
    defaults = dict(bridge.PARAMETERS)
    assert defaults['rates.lifecycle'] == 0.5
    node = _settings_node(**{'rates.lifecycle': defaults['rates.lifecycle'],
                             'status.nodes': defaults['status.nodes']})
    node._read_settings()
    assert node._status_nodes[-5:] == tuple(defaults['status.nodes'])


def test_read_settings_status_nodes_valid_list_is_kept_in_order():
    node = _settings_node(mode='', **{'status.nodes': ['b', 'a', 'c_1', '_x', 'A9']})
    node._read_settings()
    assert node._status_nodes == ('b', 'a', 'c_1', '_x', 'A9')


@pytest.mark.parametrize('nodes', [['a', 'a'], ['a', 'b', 'a'], ['x'] * 3])
def test_read_settings_duplicate_status_nodes_raise(nodes):
    with pytest.raises(bridge.ConfigError, match='status.nodes'):
        _settings_node(**{'status.nodes': nodes})._read_settings()


@pytest.mark.parametrize('name', ['', ' ', '1abc', 'a-b', 'a b', 'a/b', '/a', 'a.b', 'ñ', 'a\n',
                                  'a;b', 'a' * 65, '../x', 'a$', '-x'])
def test_read_settings_invalid_node_names_raise(name):
    with pytest.raises(bridge.ConfigError, match='status.nodes'):
        _settings_node(**{'status.nodes': ['ok', name]})._read_settings()


@pytest.mark.parametrize('name', [1, 1.5, None, True, ['a'], {'a': 1}, b'abc'])
def test_read_settings_non_string_node_names_raise(name):
    with pytest.raises(bridge.ConfigError, match='status.nodes'):
        _settings_node(**{'status.nodes': ['ok', name]})._read_settings()


def test_read_settings_longest_valid_node_name_is_64_characters():
    node = _settings_node(mode='', **{'status.nodes': ['a' * 64]})
    node._read_settings()
    assert node._status_nodes == ('a' * 64,)


def test_read_settings_exactly_16_status_nodes_are_accepted_and_17_are_not():
    names = [f'n{i}' for i in range(17)]
    node = _settings_node(mode='', **{'status.nodes': names[:16]})
    node._read_settings()
    assert len(node._status_nodes) == 16
    with pytest.raises(bridge.ConfigError, match='status.nodes'):
        _settings_node(**{'status.nodes': names})._read_settings()


@pytest.mark.parametrize('value', ['controller_server', None, 5, 1.5, True, {'a': 1},
                                   {'a'}, b'abc'])
def test_read_settings_status_nodes_that_is_not_a_list_raises(value):
    with pytest.raises(bridge.ConfigError, match='status.nodes'):
        _settings_node(**{'status.nodes': value})._read_settings()


def test_read_settings_empty_status_nodes_keeps_only_the_mode_nodes():
    node = _settings_node(mode='map', **{'status.nodes': []})
    node._read_settings()
    assert node._status_nodes == ('map_server', 'amcl')
    node = _settings_node(mode='', **{'status.nodes': []})
    node._read_settings()
    assert node._status_nodes == ()


def test_read_settings_a_tuple_of_node_names_is_accepted():
    node = _settings_node(mode='', **{'status.nodes': ('a', 'b')})
    node._read_settings()
    assert node._status_nodes == ('a', 'b')


@pytest.mark.parametrize('mode, expected', [
    ('map', ('map_server', 'amcl', 'controller_server', 'planner_server')),
    ('mapping', ('slam_toolbox', 'controller_server', 'planner_server')),
    ('', ('controller_server', 'planner_server')),
    ('nav', ('controller_server', 'planner_server'))])
def test_read_settings_unites_the_localization_nodes_of_the_mode_with_the_configured(
        mode, expected):
    node = _settings_node(mode=mode, **{'status.nodes': ['controller_server',
                                                         'planner_server']})
    node._read_settings()
    assert node._status_nodes == expected


@pytest.mark.parametrize('mode, configured, expected', [
    ('map', ['amcl', 'bt_navigator'], ('map_server', 'amcl', 'bt_navigator')),
    ('map', ['map_server', 'amcl'], ('map_server', 'amcl')),
    ('mapping', ['x', 'slam_toolbox'], ('slam_toolbox', 'x')),
    ('mapping', ['amcl'], ('slam_toolbox', 'amcl')),       # configured explicitly: kept
    ('map', ['slam_toolbox'], ('map_server', 'amcl', 'slam_toolbox'))])
def test_read_settings_union_has_no_duplicates_and_puts_the_mode_nodes_first(
        mode, configured, expected):
    node = _settings_node(mode=mode, **{'status.nodes': configured})
    node._read_settings()
    assert node._status_nodes == expected
    assert len(set(node._status_nodes)) == len(node._status_nodes)


def test_the_mode_nodes_are_the_ones_the_launch_starts():
    assert bridge.MODE_NODES == {'map': ('map_server', 'amcl'), 'mapping': ('slam_toolbox',)}


def test_a_node_built_with_the_union_polls_the_mode_nodes_too():
    node = _settings_node(mode='mapping', **{'status.nodes': ['planner_server']})
    node._read_settings()
    watched = make_lifecycle_node(node._status_nodes)
    assert set(watched.created) == {'slam_toolbox/get_state', 'planner_server/get_state'}


# map_dir

def test_read_settings_map_dir_absolute_is_kept():
    node = _settings_node(map_dir='/home/pi/maps')
    node._read_settings()
    assert node._map_dir == '/home/pi/maps'


@pytest.mark.parametrize('raw, expected', [('/maps/', '/maps'), ('/a/./b', '/a/b'),
                                           ('/a/b/../c', '/a/c'), ('  /maps  ', '/maps')])
def test_read_settings_map_dir_is_normalised(raw, expected):
    node = _settings_node(map_dir=raw)
    node._read_settings()
    assert node._map_dir == expected


@pytest.mark.parametrize('raw', ['', '   ', '\t'])
def test_read_settings_empty_map_dir_means_mtrb_maps_in_the_home_directory(
        monkeypatch, raw):
    monkeypatch.setenv('HOME', '/home/robot')
    node = _settings_node(map_dir=raw)
    node._read_settings()
    assert node._map_dir == '/home/robot/mtrb_maps'
    assert not node.logger.warning.called


@pytest.mark.parametrize('raw', ['', '~/mtrb_maps'])
def test_read_settings_without_a_resolvable_home_warns_and_keeps_an_unusable_map_dir(
        monkeypatch, raw):
    monkeypatch.delenv('HOME', raising=False)
    monkeypatch.setattr(pwd, 'getpwuid', _no_such_user)
    node = _settings_node(map_dir=raw)
    node._read_settings()                     # the node still starts
    assert node.logger.warning.called
    assert node._map_dir.startswith('~')      # not absolute: every save answers 500
    assert 'home' in str(node.logger.warning.call_args).lower()


def _no_such_user(uid):
    raise KeyError(uid)


def test_read_settings_map_dir_expands_the_home_directory(monkeypatch):
    monkeypatch.setenv('HOME', '/home/robot')
    node = _settings_node(map_dir='~/mtrb_maps')
    node._read_settings()
    assert node._map_dir == '/home/robot/mtrb_maps'


def test_read_settings_unknown_user_map_dir_starts_with_a_warning_and_is_unusable():
    node = _settings_node(map_dir='~nosuchuser_zzz/maps')
    node._read_settings()
    assert node.logger.warning.called
    assert node._map_dir == '~nosuchuser_zzz/maps'
    assert not os.path.isabs(node._map_dir)


@pytest.mark.parametrize('raw', ['maps', './maps', '../maps', 'a/b'])
def test_read_settings_relative_map_dir_raises(raw):
    with pytest.raises(bridge.ConfigError, match='map_dir'):
        _settings_node(map_dir=raw)._read_settings()


@pytest.mark.parametrize('raw', [None, 5, 1.5, True, False, ['/maps'], {'a': 1}, b'/maps'])
def test_read_settings_non_string_map_dir_raises(raw):
    with pytest.raises(bridge.ConfigError, match='map_dir'):
        _settings_node(map_dir=raw)._read_settings()


# shipped parameter files

def _shipped_params(file_name):
    import os
    import yaml
    path = os.path.join(os.path.dirname(__file__), '..', 'param', file_name)
    with open(path, encoding='utf-8') as handle:
        return yaml.safe_load(handle)['mtrb_web_bridge']['ros__parameters']


def _flat(tree, prefix=''):
    flat = {}
    for key, value in tree.items():
        if isinstance(value, dict):
            flat.update(_flat(value, f'{prefix}{key}.'))
        else:
            flat[f'{prefix}{key}'] = value
    return flat


@pytest.mark.parametrize('file_name', ['web.yaml', 'web.example.yaml'])
def test_the_shipped_files_pass_read_settings_in_both_modes(file_name, share):
    flat = _flat(_shipped_params(file_name))
    for mode in ('map', 'mapping'):
        values = {k: v for k, v in flat.items() if k in VALID_SETTINGS}
        node = _settings_node(**{**values, 'mode': mode})
        node._read_settings()                       # raises ConfigError if invalid
        assert node._lifecycle_period_s == pytest.approx(1.0 / flat['rates.lifecycle'])
        assert set(flat['status.nodes']) <= set(node._status_nodes)


@pytest.mark.parametrize('file_name', ['web.yaml', 'web.example.yaml'])
def test_every_key_of_the_shipped_files_is_a_declared_node_parameter(file_name):
    declared = {name for name, _ in bridge.PARAMETERS}
    assert set(_flat(_shipped_params(file_name))) <= declared


@pytest.mark.parametrize('file_name', ['web.yaml', 'web.example.yaml'])
def test_the_node_defaults_of_the_phase_5_parameters_match_the_shipped_files(file_name):
    flat = _flat(_shipped_params(file_name))
    defaults = dict(bridge.PARAMETERS)
    for key in ('rates.lifecycle', 'status.nodes', 'map_dir'):
        assert defaults[key] == flat[key], key


# =========================================================================================
# Security fixes of phase 5: token strength, allow_example_token, map quotas, ~/mtrb_maps.
# The tests fix the SAFE behaviour: the node refuses to start with a weak token unless the
# development switch is a real bool AND the host is loopback; the token is never printed.
# =========================================================================================

SENTINEL_TOKEN = 'SENTINEL-tok-7f3a9c1e'          # 21 characters: strong, unique, greppable
PLACEHOLDER = 'CHANGE-ME-example-token'           # the value shipped in the repository
LOOPBACK = ['127.0.0.1', '::1', 'localhost']
NOT_LOOPBACK = ['0.0.0.0', '::', '192.168.1.50', '10.0.0.5', '172.16.3.4', 'fe80::1',
                'robot.local', 'localhost.evil.com', '127.0.0.1.evil.com', '127.0.0.2']


def _everything_printed(logger, caplog=None):
    text = ' '.join(str(c) for c in logger.method_calls)
    return text + (caplog.text if caplog is not None else '')


def _refused(params, **kwargs):
    """Run __init__ expecting ConfigError; return the exception text."""
    with pytest.raises(bridge.ConfigError) as caught:
        _init_node(params, **kwargs)
    return str(caught.value)


def test_token_constants_are_the_documented_ones():
    assert bridge.TOKEN_MIN_LENGTH == 20
    assert tuple(bridge.LOOPBACK_HOSTS) == ('127.0.0.1', '::1', 'localhost')
    assert dict(bridge.PARAMETERS)['auth.allow_example_token'] is False


@pytest.mark.parametrize('host', ['127.0.0.1', '0.0.0.0', '192.168.1.50'])
def test_the_example_token_is_refused_by_default_on_every_host(host):
    text = _refused({'host': host}, token_result=PLACEHOLDER)
    assert PLACEHOLDER not in text


@pytest.mark.parametrize('token', ['CHANGE-ME', 'CHANGE-ME-', 'CHANGE-ME-' + 'x' * 40,
                                   'change-me-' + 'q' * 30])
def test_any_token_with_the_placeholder_prefix_is_refused_even_if_long(token):
    assert token not in _refused({}, token_result=token)


@pytest.mark.parametrize('token', ['a1B2c3D4e5F6g7H', 'a1B2c3D4e5F6g7H8i9J'])
def test_a_token_shorter_than_20_characters_is_refused(token):
    assert len(token) < 20
    assert token not in _refused({}, token_result=token)


@pytest.mark.parametrize('length', [20, 21, 32, 64])
def test_a_token_of_20_or_more_characters_starts_without_a_warning(length):
    token = 'k9' * (length // 2) + 'z' * (length % 2)
    assert len(token) == length
    node, logger = _init_node({}, token_result=token)
    assert node.host == '127.0.0.1'
    assert not logger.warning.called and not logger.error.called


@pytest.mark.parametrize('token', ['', '1', 'short', 'x' * 8, '123456', '   '])
def test_short_tokens_are_refused_by_default(token):
    assert 'token' in _refused({}, token_result=token).lower()


@pytest.mark.parametrize('token', ['a' * 20, 'a' * 64, ' ' * 20, '7' * 30])
def test_a_single_repeated_character_is_refused_whatever_the_length(token):
    text = _refused({}, token_result=token)
    assert 'token' in text.lower()
    assert token.strip() == '' or token not in text


def test_the_length_is_measured_after_stripping():
    token = ' ' + 'a1' * 9 + ' '          # 20 characters with spaces, 18 once stripped
    assert len(token) == 20
    _refused({}, token_result=token)


@pytest.mark.parametrize('token', [PLACEHOLDER, 'short-tok', '123456', 'a' * 20])
@pytest.mark.parametrize('host', LOOPBACK)
def test_allow_example_token_true_on_loopback_starts_with_a_warning(token, host):
    node, logger = _init_node({'host': host, 'auth.allow_example_token': True},
                              token_result=token)
    assert node.host == host
    assert logger.warning.called
    assert token not in _everything_printed(logger)
    assert 'DEVELOPMENT' in str(logger.warning.call_args)


@pytest.mark.parametrize('host', NOT_LOOPBACK)
@pytest.mark.parametrize('token', [PLACEHOLDER, 'short-tok', 'a' * 20])
def test_allow_example_token_true_is_not_honoured_on_a_non_loopback_host(host, token):
    text = _refused({'host': host, 'auth.allow_example_token': True}, token_result=token)
    assert token not in text


@pytest.mark.parametrize('allow', ['true', 'True', 'yes', 1, 1.0, 0, None, [], 'false'])
@pytest.mark.parametrize('host', ['127.0.0.1', '0.0.0.0'])
def test_allow_example_token_must_be_a_real_bool(allow, host):
    text = _refused({'host': host, 'auth.allow_example_token': allow},
                    token_result=PLACEHOLDER)
    assert PLACEHOLDER not in text


def test_allow_example_token_false_is_the_default_and_refuses_the_placeholder_on_loopback():
    assert dict(bridge.PARAMETERS)['auth.allow_example_token'] is False
    _refused({'host': '127.0.0.1', 'auth.allow_example_token': False},
             token_result=PLACEHOLDER)


def test_a_strong_token_ignores_allow_example_token_and_logs_no_warning():
    node, logger = _init_node({'host': '0.0.0.0', 'auth.allow_example_token': True},
                              token_result=SENTINEL_TOKEN)
    assert node.host == '0.0.0.0'
    assert not logger.warning.called


@pytest.mark.parametrize('token', [SENTINEL_TOKEN, 'CHANGE-ME-' + SENTINEL_TOKEN,
                                   'S-' + SENTINEL_TOKEN[:12]])
def test_the_token_never_appears_in_logs_or_in_exception_messages(token, caplog):
    with caplog.at_level(logging.DEBUG):
        try:
            node, logger = _init_node({}, token_result=token)
        except bridge.ConfigError as err:
            assert token not in str(err) and token not in repr(err)
            assert token not in ''.join(str(a) for a in err.args)
        else:
            assert token not in _everything_printed(logger, caplog)
    assert token not in caplog.text


@pytest.mark.parametrize('host', ['0.0.0.0', '192.168.1.50'])
def test_refused_startup_does_not_print_the_token_in_the_error_for_any_host(host, caplog):
    with caplog.at_level(logging.DEBUG):
        text = _refused({'host': host, 'auth.allow_example_token': True},
                        token_result='CHANGE-ME-' + SENTINEL_TOKEN)
    assert SENTINEL_TOKEN not in text and SENTINEL_TOKEN not in caplog.text


def test_a_refused_token_never_builds_the_web_app():
    created = []
    with mock.patch.object(bridge, 'create_app', side_effect=created.append):
        with pytest.raises(bridge.ConfigError):
            _init_node({}, token_result=PLACEHOLDER)
    assert created == []


@pytest.mark.parametrize('env_token', ['xyz', 'a' * 19, PLACEHOLDER, 'a' * 40])
def test_a_weak_token_in_the_environment_is_refused_without_falling_back_to_the_file(
        monkeypatch, tmp_path, env_token):
    good = tmp_path / 'tok'
    good.write_text(SENTINEL_TOKEN)
    good.chmod(0o600)
    monkeypatch.setenv(TOKEN_ENV_VAR, env_token)
    text = _refused({'auth.token_file': str(good)}, real_loader=True)
    assert env_token not in text and SENTINEL_TOKEN not in text


def test_a_20_character_environment_token_starts(monkeypatch, tmp_path):
    monkeypatch.setenv(TOKEN_ENV_VAR, 'e1' * 10)
    node, logger = _init_node({'auth.token_file': str(tmp_path / 'missing')},
                              real_loader=True)
    assert not logger.warning.called
    assert node.create_app_mock.call_args.args[1] == 'e1' * 10


def test_a_weak_environment_token_with_allow_and_loopback_starts_with_a_warning(
        monkeypatch, tmp_path):
    monkeypatch.setenv(TOKEN_ENV_VAR, 'dev')
    node, logger = _init_node({'auth.allow_example_token': True,
                               'auth.token_file': str(tmp_path / 'missing')},
                              real_loader=True)
    assert logger.warning.called and 'dev' not in _everything_printed(logger)


def test_a_token_from_the_private_file_reaches_the_web_app(tmp_path):
    path = tmp_path / 'tok'
    path.write_text(SENTINEL_TOKEN + '\n')
    path.chmod(0o600)
    node, logger = _init_node({'auth.token_file': str(path)}, real_loader=True)
    assert node.create_app_mock.call_args.args[1] == SENTINEL_TOKEN
    assert SENTINEL_TOKEN not in _everything_printed(logger)


def test_a_weak_token_in_the_file_is_refused(tmp_path):
    path = tmp_path / 'tok'
    path.write_text('abc\n')
    path.chmod(0o600)
    _refused({'auth.token_file': str(path)}, real_loader=True)


def test_an_unsafe_token_file_stops_the_startup_without_building_the_app(tmp_path):
    path = tmp_path / 'tok'
    path.write_text(SENTINEL_TOKEN)
    path.chmod(0o640)
    created = []
    with mock.patch.object(bridge, 'create_app', side_effect=created.append):
        text = _refused({'auth.token_file': str(path)}, real_loader=True)
    assert created == [] and SENTINEL_TOKEN not in text


def test_no_token_by_any_route_is_a_config_error(tmp_path):
    text = _refused({'auth.token_file': str(tmp_path / 'missing')}, real_loader=True)
    assert 'MTRB_WEB_TOKEN' in text


def test_the_app_is_built_with_the_token_and_no_limiter_override():
    node, _ = _init_node({}, token_result=SENTINEL_TOKEN)
    [call] = node.create_app_mock.call_args_list
    assert call.args[1] == SENTINEL_TOKEN


# --- maps.min_free_mb / maps.min_interval_s -----------------------------------------------

def test_the_quota_parameters_have_the_documented_defaults():
    defaults = dict(bridge.PARAMETERS)
    assert defaults['maps.min_free_mb'] == 200
    assert defaults['maps.min_interval_s'] == 5.0


def test_read_settings_reads_the_quota_parameters():
    node = _settings_node(**{'maps.min_free_mb': 350, 'maps.min_interval_s': 12})
    node._read_settings()
    assert node._map_min_free_mb == 350
    assert node._map_min_interval_s == 12.0 and isinstance(node._map_min_interval_s, float)


@pytest.mark.parametrize('value', [0, 1, 200, 10 ** 6, 0.5])
def test_read_settings_accepts_valid_min_free_mb(value):
    node = _settings_node(**{'maps.min_free_mb': value})
    node._read_settings()
    assert node._map_min_free_mb == value


@pytest.mark.parametrize('bad', [-1, -0.1, True, False, 'x', '200', None, float('nan'),
                                 float('inf'), [], {}])
def test_read_settings_invalid_min_free_mb_raises(bad):
    with pytest.raises(bridge.ConfigError, match='maps.min_free_mb'):
        _settings_node(**{'maps.min_free_mb': bad})._read_settings()


@pytest.mark.parametrize('value', [0, 0.0, 0.5, 5, 3600])
def test_read_settings_accepts_valid_min_interval_s(value):
    node = _settings_node(**{'maps.min_interval_s': value})
    node._read_settings()
    assert node._map_min_interval_s == float(value)


@pytest.mark.parametrize('bad', [-1, -0.1, True, False, 'x', '5', None, float('nan'),
                                 float('inf'), []])
def test_read_settings_invalid_min_interval_s_raises(bad):
    with pytest.raises(bridge.ConfigError, match='maps.min_interval_s'):
        _settings_node(**{'maps.min_interval_s': bad})._read_settings()


def test_init_hands_the_map_quotas_and_the_map_dir_to_the_web_app():
    node, _ = _init_node({}, token_result=SENTINEL_TOKEN)
    [call] = node.create_app_mock.call_args_list
    assert call.kwargs['map_dir'] == '/share/map'
    assert call.kwargs['map_min_free_mb'] == 200
    assert call.kwargs['map_min_interval_s'] == 5.0
    assert call.kwargs['map_port'] is node and call.kwargs['on_event'] == node._log_map_event


# --- ~/mtrb_maps end to end (node settings + real MapSaver on a temporary HOME) -----------

class _WritingPort:
    """MapPort double that writes a tiny map."""

    def __init__(self):
        self.saves = []

    def ready(self):
        return True

    def save_map(self, base_path, timeout_s):
        self.saves.append(base_path)
        for ext in ('.yaml', '.pgm'):
            with open(base_path + ext, 'wb') as handle:
                handle.write(b'x')
        return True

    def serialize_posegraph(self, base_path, timeout_s):
        return False


def _client_for(node, port):
    from fastapi.testclient import TestClient
    from mtrb_bringup.web_server import create_app
    store = StateStore()
    store.update(mode='mapping')
    app = create_app(store, SENTINEL_TOKEN, map_port=port, map_dir=node._map_dir,
                     map_min_free_mb=0, map_min_interval_s=0.0)
    return TestClient(app)


AUTH_HEADERS = {'Authorization': f'Bearer {SENTINEL_TOKEN}'}


def test_the_first_save_with_an_empty_map_dir_creates_home_mtrb_maps_with_mode_0755(
        tmp_path, monkeypatch):
    home = os.path.join(os.path.realpath(str(tmp_path)), 'home')
    os.mkdir(home)
    monkeypatch.setenv('HOME', home)
    node = _settings_node(map_dir='')
    node._read_settings()
    assert node._map_dir == os.path.join(home, 'mtrb_maps')
    assert not os.path.exists(node._map_dir)          # created lazily, at the first save
    previous = os.umask(0o022)
    try:
        port = _WritingPort()
        response = _client_for(node, port).post('/api/map/save', json={'name': 'lab'},
                                                headers=AUTH_HEADERS)
    finally:
        os.umask(previous)
    assert response.status_code == 200, response.text
    assert (os.stat(node._map_dir).st_mode & 0o777) == 0o755
    assert port.saves == [os.path.join(node._map_dir, 'lab')]
    assert os.path.isfile(os.path.join(node._map_dir, 'lab.yaml'))


def test_without_a_home_the_save_answers_500_map_dir_invalid_and_creates_nothing(
        tmp_path, monkeypatch):
    monkeypatch.delenv('HOME', raising=False)
    monkeypatch.setattr(pwd, 'getpwuid', _no_such_user)
    monkeypatch.chdir(tmp_path)
    node = _settings_node(map_dir='')
    node._read_settings()
    assert node.logger.warning.called                 # warned at startup
    port = _WritingPort()
    client = _client_for(node, port)
    response = client.post('/api/map/save', json={'name': 'lab'}, headers=AUTH_HEADERS)
    assert (response.status_code, response.json()['error']) == (500, 'map_dir_invalid')
    assert port.saves == []
    assert os.listdir(str(tmp_path)) == []            # no literal '~' directory in the cwd
    assert str(tmp_path) not in response.text and '~' not in response.text


def test_a_symlink_among_the_files_of_the_default_map_dir_is_rejected(tmp_path, monkeypatch):
    home = os.path.join(os.path.realpath(str(tmp_path)), 'home')
    maps = os.path.join(home, 'mtrb_maps')
    os.makedirs(maps)
    victim = os.path.join(home, 'victim')
    with open(victim, 'wb') as handle:
        handle.write(b'do-not-touch')
    os.symlink(victim, os.path.join(maps, 'lab.yaml'))
    monkeypatch.setenv('HOME', home)
    node = _settings_node(map_dir='')
    node._read_settings()
    port = _WritingPort()
    response = _client_for(node, port).post(
        '/api/map/save', json={'name': 'lab', 'overwrite': True}, headers=AUTH_HEADERS)
    assert (response.status_code, response.json()['error']) == (500, 'map_dir_invalid')
    assert port.saves == []
    with open(victim, 'rb') as handle:
        assert handle.read() == b'do-not-touch'


# --- the files shipped in param/ ----------------------------------------------------------

def _shipped_init_values(file_name):
    return dict(_flat(_shipped_params(file_name)))


@pytest.mark.parametrize('file_name', ['web.yaml', 'web.example.yaml'])
def test_the_repository_ships_no_token_and_the_node_cannot_start_without_one(file_name):
    values = _shipped_init_values(file_name)
    assert 'auth.token' not in values                 # the token is never in a YAML file
    assert values['auth.token_file'] == '~/.config/mtrb/web_token'
    assert values['auth.allow_example_token'] is False
    assert 'MTRB_WEB_TOKEN' in _refused(values, token_result='')


@pytest.mark.parametrize('file_name', ['web.yaml', 'web.example.yaml'])
def test_the_shipped_values_start_with_a_real_token(file_name):
    values = _shipped_init_values(file_name)
    node, logger = _init_node(values, token_result=SENTINEL_TOKEN)
    assert node.host == values['host']
    assert not logger.warning.called


@pytest.mark.parametrize('file_name', ['web.yaml', 'web.example.yaml'])
def test_the_shipped_values_with_allow_example_token_need_a_loopback_host(file_name):
    values = _shipped_init_values(file_name)
    values['auth.allow_example_token'] = True
    _refused(values, token_result=PLACEHOLDER)        # shipped host is 0.0.0.0
    values['host'] = '127.0.0.1'
    node, logger = _init_node(values, token_result=PLACEHOLDER)
    assert logger.warning.called and PLACEHOLDER not in _everything_printed(logger)


def test_the_two_shipped_files_have_exactly_the_same_keys_and_security_values():
    web, example = _flat(_shipped_params('web.yaml')), _flat(_shipped_params('web.example.yaml'))
    assert set(web) == set(example)
    for key in ('auth.allow_example_token', 'auth.token_file', 'limits.max_connections',
                'map_dir', 'maps.min_free_mb', 'maps.min_interval_s', 'host'):
        assert web[key] == example[key], key
    assert web['limits.max_connections'] == 64


@pytest.mark.parametrize('file_name', ['web.yaml', 'web.example.yaml'])
def test_the_shipped_files_declare_exactly_the_node_parameters_but_the_launch_injected(
        file_name):
    declared = {name for name, _ in bridge.PARAMETERS}
    assert set(_flat(_shipped_params(file_name))) == declared - {'mode'}


@pytest.mark.parametrize('file_name', ['web.yaml', 'web.example.yaml'])
def test_the_shipped_security_values_equal_the_node_defaults(file_name):
    flat = _flat(_shipped_params(file_name))
    defaults = dict(bridge.PARAMETERS)
    for key in ('auth.allow_example_token', 'maps.min_free_mb', 'maps.min_interval_s',
                'map_dir'):
        assert flat[key] == defaults[key], key
    assert flat['maps.min_free_mb'] == 200 and flat['maps.min_interval_s'] == 5.0


# =========================================================================================
# Parameter declaration (phase 5, security/startup). Until now _init_node replaced
# declare_parameter by a no-op, so the declaration path had no coverage. The fake below
# imitates rclpy Jazzy (Node.declare_parameters / _set_parameters_atomically_common): the
# declared type is inferred from the default; an override of another type raises
# InvalidParameterTypeException unless the descriptor has dynamic_typing=True.
# =========================================================================================

import copy  # noqa: E402, I100


class RealisticDescriptor:
    """Same public attribute as rcl_interfaces/msg/ParameterDescriptor."""

    def __init__(self, dynamic_typing=False, **_ignored):
        self.dynamic_typing = dynamic_typing


def _rclpy_type(value):
    """Return the Parameter.Type name rclpy infers from a Python value."""
    if isinstance(value, bool):
        return 'bool'
    if isinstance(value, int):
        return 'integer'
    if isinstance(value, float):
        return 'double'
    if isinstance(value, str):
        return 'string'
    if isinstance(value, (list, tuple)):
        if all(isinstance(v, str) for v in value) and value:
            return 'string_array'
        return 'byte_array' if not value else 'other_array'
    return 'other'


class RealisticParameters:
    """Declared-parameter store with the type rules of rclpy (overrides = YAML values)."""

    def __init__(self, overrides):
        self.overrides = dict(overrides)
        self.values = {}
        self.declared = []           # (name, dynamic_typing)

    def declare_parameter(self, name, value=None, descriptor=None, ignore_override=False):
        assert name not in self.values, f'{name} declared twice'
        dynamic = descriptor is not None and descriptor.dynamic_typing is True
        actual = self.overrides[name] if name in self.overrides else value
        if not dynamic and _rclpy_type(actual) != _rclpy_type(value):
            # The real exception text embeds the offending value: the node must not repeat it.
            raise bridge.InvalidParameterTypeException(
                f'{name}={actual!r}', _rclpy_type(value))
        self.values[name] = actual
        self.declared.append((name, dynamic))

    def get_parameter(self, name):
        return FakeParam(self.values[name])


STRONG_TOKEN = 'declared-tok-0123456789'


def _init_declared(overrides, token=STRONG_TOKEN, host=None, space_cls=None):
    """Run the REAL __init__ (declaration, validation, teleop) with the realistic fake."""
    overrides = dict(overrides)
    if host is not None:
        overrides['host'] = host
    space = (space_cls or RealisticParameters)(overrides)
    logger = mock.Mock()
    create_app_mock = mock.Mock(return_value=object())
    with mock.patch.object(bridge.Node, '__init__', lambda self, name: None), \
            mock.patch.object(bridge, 'ParameterDescriptor', RealisticDescriptor), \
            mock.patch.object(bridge.MtrbWebBridge, 'declare_parameter', create=True,
                              new=lambda self, *a, **k: space.declare_parameter(*a, **k)), \
            mock.patch.object(bridge.MtrbWebBridge, 'get_parameter', create=True,
                              new=lambda self, n: space.get_parameter(n)), \
            mock.patch.object(bridge.MtrbWebBridge, 'get_logger', create=True,
                              new=lambda self: logger), \
            mock.patch.object(bridge, 'get_package_share_directory', return_value='/share'), \
            mock.patch.object(bridge, 'load_token', return_value=token), \
            mock.patch.object(bridge.MtrbWebBridge, '_init_ros_io', new=lambda self: None), \
            mock.patch.object(bridge, 'create_app', new=create_app_mock):
        node = bridge.MtrbWebBridge()
    node.create_app_mock = create_app_mock
    node.space = space
    node.logger_double = logger
    return node


def _declared_refusal(overrides, **kwargs):
    with pytest.raises(bridge.ConfigError) as caught:
        _init_declared(overrides, **kwargs)
    return str(caught.value)


EXPECTED_DYNAMIC = {
    'rates.pose', 'rates.scan', 'rates.status', 'rates.lifecycle',
    'teleop.max_vx', 'teleop.max_wz', 'scan.min_range', 'scan.max_range',
    'dock.x', 'dock.y', 'dock.yaw', 'maps.min_free_mb', 'maps.min_interval_s',
    'status.nodes'}
# Integer or bool or str BY DESIGN: _number(integer=True) / strict bool / text. A float here is
# a configuration error, so they keep static typing on purpose.
STATIC_BY_DESIGN = {
    'port', 'teleop.timeout_ms', 'teleop.enabled', 'scan.max_points', 'plan.max_points',
    'limits.max_clients', 'limits.max_ws_bytes', 'limits.max_connections',
    'auth.allow_example_token'}


def test_dynamic_parameters_are_exactly_the_fourteen_expected():
    assert set(bridge.DYNAMIC_PARAMETERS) == EXPECTED_DYNAMIC
    assert len(bridge.DYNAMIC_PARAMETERS) == 14
    assert 'auth.token' not in bridge.DYNAMIC_PARAMETERS


def test_every_dynamic_parameter_is_declared_in_parameters():
    assert set(bridge.DYNAMIC_PARAMETERS) <= {name for name, _ in bridge.PARAMETERS}


def test_every_numeric_default_is_dynamic_or_int_bool_by_design():
    """Guards against a new float/number parameter that rclpy would reject for `10` vs `10.0`."""
    unclassified = []
    for name, default in bridge.PARAMETERS:
        numeric = isinstance(default, (int, float)) and not isinstance(default, bool)
        if numeric and name not in bridge.DYNAMIC_PARAMETERS and name not in STATIC_BY_DESIGN:
            unclassified.append(name)
    assert unclassified == []


def test_float_defaults_are_always_dynamic():
    floats = [n for n, d in bridge.PARAMETERS if isinstance(d, float)]
    assert floats and all(n in bridge.DYNAMIC_PARAMETERS for n in floats)


def test_the_node_declares_dynamic_parameters_with_dynamic_typing_and_the_rest_without():
    node = _init_declared({})
    flags = dict(node.space.declared)
    assert set(flags) == {name for name, _ in bridge.PARAMETERS}
    assert {n for n, dynamic in flags.items() if dynamic} == EXPECTED_DYNAMIC


def test_the_realistic_fake_rejects_a_type_change_of_a_static_parameter():
    space = RealisticParameters({'port': 8080.0})
    with pytest.raises(bridge.InvalidParameterTypeException):
        space.declare_parameter('port', 8080, RealisticDescriptor())
    space.declare_parameter('port', 8080, RealisticDescriptor(dynamic_typing=True))
    assert space.get_parameter('port').value == 8080.0


def test_defaults_alone_start_through_the_real_declaration_path():
    node = _init_declared({})
    assert node.port == 8080 and node._map_min_interval_s == 5.0


@pytest.mark.parametrize('overrides,check', [
    ({'maps.min_interval_s': 0}, lambda n: (n._map_min_interval_s == 0.0
                                            and isinstance(n._map_min_interval_s, float))),
    ({'maps.min_interval_s': 0.0}, lambda n: n._map_min_interval_s == 0.0),
    ({'maps.min_free_mb': 0}, lambda n: n._map_min_free_mb == 0),
    ({'maps.min_free_mb': 150.5}, lambda n: n._map_min_free_mb == 150.5),
    ({'maps.min_free_mb': 150}, lambda n: n._map_min_free_mb == 150),
    ({'rates.pose': 10}, lambda n: n._pose_period_s == pytest.approx(0.1)),
    ({'rates.scan': 5}, lambda n: n._scan_period_s == pytest.approx(0.2)),
    ({'rates.status': 2}, lambda n: n._status_period_s == pytest.approx(0.5)),
    ({'rates.lifecycle': 0.25}, lambda n: n._lifecycle_period_s == pytest.approx(4.0)),
    ({'teleop.max_wz': 1}, lambda n: n._teleop_gate is not None),
    ({'dock.x': 1}, lambda n: n._dock == (1.0, 0.0, 0.0)),
    ({'dock.y': 2, 'dock.yaw': -3}, lambda n: n._dock == (0.0, 2.0, -3.0)),
    ({'scan.min_range': 0, 'scan.max_range': 12}, lambda n: n._scan_max_range_m == 12),
])
def test_integer_or_float_yaml_values_of_dynamic_parameters_start(overrides, check):
    node = _init_declared(overrides)
    assert check(node)


def test_max_quota_values_reach_create_app_as_declared():
    node = _init_declared({'maps.min_free_mb': 150.5, 'maps.min_interval_s': 0})
    kwargs = node.create_app_mock.call_args.kwargs
    assert kwargs['map_min_free_mb'] == 150.5 and kwargs['map_min_interval_s'] == 0.0


def test_teleop_max_vx_as_an_integer_is_declared_but_rejected_by_the_hard_ceiling():
    # `teleop.max_vx: 1` is an int (declaration OK, dynamic) but 1 m/s > 0.46 m/s (Create 3).
    text = _declared_refusal({'teleop.max_vx': 1})
    assert 'teleop.max_vx' in text and 'wrong type' not in text and '0.46' in text


@pytest.mark.parametrize('name,value', [
    ('port', 8080.0), ('port', 48123.75), ('teleop.timeout_ms', 300.0),
    ('teleop.timeout_ms', 612.75), ('teleop.enabled', 'yes'),
    ('teleop.enabled', 'SENTINEL-yes'), ('auth.allow_example_token', 'true'),
    ('auth.allow_example_token', 'SENTINEL-true'), ('host', 1), ('host', 7654321),
    ('map_dir', 3), ('map_dir', 7654321), ('scan.max_points', 360.5),
    ('plan.max_points', '200'), ('limits.max_clients', 5.5), ('limits.max_ws_bytes', None),
    ('mode', 1), ('teleop.enabled', 1), ('port', True),
])
def test_wrong_type_of_a_static_parameter_is_a_config_error_naming_it_without_the_value(
        name, value):
    text = _declared_refusal({name: value})
    assert text.startswith(f'Invalid parameter {name}: wrong type or value (')
    assert 'InvalidParameterTypeException' in text
    assert repr(value) not in text
    if not isinstance(value, (bool, int, type(None))):
        assert str(value) not in text
    elif isinstance(value, int) and not isinstance(value, bool):
        assert str(value) not in text.replace(name, '')


@pytest.mark.parametrize('exc', [bridge.InvalidParameterTypeException('p', 't'),
                                 bridge.InvalidParameterValueException('p', 'LEAK-v', 'r'),
                                 TypeError('LEAK-v'), ValueError('LEAK-v')])
def test_every_declaration_error_class_becomes_a_config_error_without_its_text(exc):
    class Raising(RealisticParameters):
        def declare_parameter(self, name, value=None, descriptor=None, ignore_override=False):
            raise exc

    text = _declared_refusal({}, space_cls=Raising)
    assert text.startswith('Invalid parameter host: wrong type or value (')
    assert type(exc).__name__ in text and 'LEAK-v' not in text


PARAM_SECRET = 'SECRET-override-value-7f3a'


def _declaration_error(overrides, space_cls=None):
    with pytest.raises(bridge.ConfigError) as caught:
        _init_declared(overrides, space_cls=space_cls)
    return caught.value


def _assert_config_error_chain_is_clean(err, name):
    import traceback
    assert f'Invalid parameter {name}' in str(err)
    assert err.__cause__ is None
    assert err.__suppress_context__ is True
    shown = ' '.join([str(err), repr(err), repr(err.args), ''.join(
        traceback.format_exception(type(err), err, err.__traceback__))])
    assert PARAM_SECRET not in shown


def _raising_space(exc_factory):
    class Raising(RealisticParameters):
        def declare_parameter(self, name, value=None, descriptor=None, ignore_override=False):
            raise exc_factory(name)

    return Raising


DECLARATION_ERRORS = {
    'realistic-type-error': None,      # the realistic rclpy double: its text embeds the value
    'InvalidParameterTypeException': lambda n: bridge.InvalidParameterTypeException(
        f'{n}={PARAM_SECRET!r}', 'string'),
    'InvalidParameterValueException': lambda n: bridge.InvalidParameterValueException(
        n, PARAM_SECRET, 'rejected'),
    'TypeError': lambda n: TypeError(PARAM_SECRET),
    'ValueError': lambda n: ValueError(PARAM_SECRET),
}


def _declaration_site(kind):
    if kind == 'realistic-type-error':
        return _declaration_error({'port': PARAM_SECRET}), 'port'
    return _declaration_error({}, _raising_space(DECLARATION_ERRORS[kind])), 'host'


@pytest.mark.parametrize('kind', list(DECLARATION_ERRORS))
def test_declaration_config_error_has_no_cause_and_never_prints_the_override_value(kind):
    err, name = _declaration_site(kind)
    _assert_config_error_chain_is_clean(err, name)


@pytest.mark.xfail(strict=True, reason=(
    'BUG (backend, low): "raise ConfigError(...) from None" inside the except block leaves '
    'the rclpy exception in err.__context__; its args/repr include the override value '
    '(hidden when printing, still reachable). Fix: raise outside the except block.'))
@pytest.mark.parametrize('kind', list(DECLARATION_ERRORS))
def test_declaration_config_error_context_attribute_does_not_carry_the_override_value(kind):
    err, _ = _declaration_site(kind)
    link = err.__context__
    assert link is None or PARAM_SECRET not in repr(link) + repr(link.args) + str(link)


def test_a_declaration_error_stops_the_startup_before_the_token_and_the_app():
    node_text = _declared_refusal({'port': 1.5})
    assert 'port' in node_text


def test_a_declaration_error_never_builds_the_app():
    created = []
    with mock.patch.object(bridge, 'create_app', side_effect=created.append):
        with pytest.raises(bridge.ConfigError):
            _init_declared({'host': 1})
    assert created == []


MAPS_BAD = [True, float('nan'), float('inf'), -float('inf'), -1, 'x']
NUMERIC_CASES = (
    [(name, bad) for name in ('maps.min_free_mb', 'maps.min_interval_s') for bad in MAPS_BAD]
    + [(name, bad) for name in ('rates.pose', 'rates.scan', 'rates.status', 'rates.lifecycle')
       for bad in MAPS_BAD + [0]]
    # dock.*: -1 is a legal coordinate; only non-numbers, non finite and out of range fail.
    + [(name, bad) for name in ('dock.x', 'dock.y', 'dock.yaw')
       for bad in [True, float('nan'), float('inf'), -float('inf'), 'x']]
    + [(name, bad) for name in ('teleop.max_vx', 'teleop.max_wz')
       for bad in MAPS_BAD + [0, 100]]
    + [(name, bad) for name in ('scan.min_range', 'scan.max_range') for bad in MAPS_BAD]
)


@pytest.mark.parametrize('name,bad', NUMERIC_CASES)
def test_invalid_numeric_values_of_dynamic_parameters_are_a_config_error_naming_it(name, bad):
    text = _declared_refusal({name: bad})
    assert f'Invalid parameter {name}' in text


@pytest.mark.parametrize('name', ['dock.x', 'dock.y', 'dock.yaw'])
@pytest.mark.parametrize('bad', [1e5, -1e5])
def test_a_dock_pose_beyond_the_sanity_bound_is_a_config_error(name, bad):
    text = _declared_refusal({name: bad})
    assert 'dock.x/y/yaw' in text or f'Invalid parameter {name}' in text


def test_a_zero_minimum_interval_and_zero_free_space_are_valid():
    node = _init_declared({'maps.min_interval_s': 0, 'maps.min_free_mb': 0})
    assert (node._map_min_free_mb, node._map_min_interval_s) == (0, 0.0)


def test_negative_dock_coordinates_are_valid():
    assert _init_declared({'dock.x': -1, 'dock.y': -1.5})._dock == (-1.0, -1.5, 0.0)


def test_scan_min_range_must_stay_below_max_range():
    assert 'scan' in _declared_refusal({'scan.min_range': 5, 'scan.max_range': 5})


def test_an_empty_status_nodes_list_is_accepted_and_only_the_mode_nodes_remain():
    node = _init_declared({'status.nodes': [], 'mode': 'map'})
    assert node._status_nodes == ('map_server', 'amcl')
    assert _init_declared({'status.nodes': []})._status_nodes == ()


def test_a_valid_status_nodes_list_is_kept_in_order():
    node = _init_declared({'status.nodes': ['planner_server', 'bt_navigator']})
    assert node._status_nodes == ('planner_server', 'bt_navigator')


@pytest.mark.parametrize('nodes', ['controller_server', 5, ['bad name'], ['a', 'a'],
                                   ['n%d' % i for i in range(17)], [1, 2]])
def test_an_invalid_status_nodes_value_is_a_config_error(nodes):
    assert 'status.nodes' in _declared_refusal({'status.nodes': nodes})


# --- auth.allow_example_token is validated as a bool whatever the token strength -----------

NOT_BOOL = ['true', 'True', 'yes', 1, 0, 1.0, None, [], 'false']


@pytest.mark.parametrize('allow', NOT_BOOL)
@pytest.mark.parametrize('token', [STRONG_TOKEN, 'weak', PLACEHOLDER, 'change-me-' + 'x' * 20])
@pytest.mark.parametrize('host', ['127.0.0.1', '0.0.0.0'])
def test_check_token_strength_requires_a_real_bool_even_with_a_strong_token(
        allow, token, host):
    node = make_node({'auth.allow_example_token': allow})
    node.host = host
    with pytest.raises(bridge.ConfigError) as caught:
        node._check_token_strength(token)
    assert 'auth.allow_example_token' in str(caught.value)
    assert token not in str(caught.value)
    assert not node.logger.warning.called


@pytest.mark.parametrize('token', [STRONG_TOKEN, 'weak'])
@pytest.mark.parametrize('allow', ['true', 'SENTINEL-true', 1])
def test_a_non_bool_allow_example_token_is_a_config_error_through_the_real_declaration(
        token, allow):
    text = _declared_refusal({'auth.allow_example_token': allow}, token=token)
    assert 'auth.allow_example_token' in text and token not in text
    assert repr(allow) not in text


@pytest.mark.parametrize('allow', [True, False])
def test_a_bool_allow_example_token_with_a_strong_token_starts_without_a_warning(allow):
    node = _init_declared({'auth.allow_example_token': allow}, host='0.0.0.0')
    assert not node.logger_double.warning.called


# --- placeholder detection in the node ------------------------------------------------------

LOWER_PLACEHOLDER = 'change-me-' + 'x' * 20


@pytest.mark.parametrize('host', LOOPBACK + NOT_LOOPBACK)
def test_a_lowercase_placeholder_token_of_16_plus_chars_is_refused_without_override(host):
    assert len(LOWER_PLACEHOLDER) >= 20
    text = _refused({'host': host}, token_result=LOWER_PLACEHOLDER)
    assert LOWER_PLACEHOLDER not in text


@pytest.mark.parametrize('token', ['Change-Me-' + 'y' * 20, '  change-me-' + 'z' * 20,
                                   'cHaNgE-mE-' + 'q' * 20])
def test_mixed_case_placeholder_tokens_are_refused_through_the_real_declaration(token):
    text = _declared_refusal({}, token=token, host='127.0.0.1')
    assert token.strip() not in text


def test_a_lowercase_placeholder_is_accepted_only_with_the_override_on_loopback():
    node = _init_declared({'auth.allow_example_token': True}, token=LOWER_PLACEHOLDER,
                          host='127.0.0.1')
    assert node.logger_double.warning.called
    assert LOWER_PLACEHOLDER not in str(node.logger_double.method_calls)
    _declared_refusal({'auth.allow_example_token': True}, token=LOWER_PLACEHOLDER,
                      host='0.0.0.0')


# --- the YAML files shipped in param/ declare without error --------------------------------

def _yaml_variant(file_name, variant):
    flat = _flat(copy.deepcopy(_shipped_params(file_name)))
    out = {}
    for name, value in flat.items():
        if name in bridge.DYNAMIC_PARAMETERS and isinstance(value, (int, float)) \
                and not isinstance(value, bool):
            if variant == 'to_float':
                value = float(value)
            elif variant == 'to_int' and float(value).is_integer():
                value = int(value)
        out[name] = value
    return out


@pytest.mark.parametrize('variant', ['as_is', 'to_float', 'to_int'])
@pytest.mark.parametrize('file_name', ['web.yaml', 'web.example.yaml'])
def test_the_shipped_yaml_files_declare_and_start_with_the_realistic_fake(
        file_name, variant):
    values = _yaml_variant(file_name, variant)
    assert 'auth.token' not in values
    node = _init_declared(values)
    assert node.port == 8080 and node.host == '0.0.0.0'
    assert node._map_min_free_mb == 200 and node._map_min_interval_s == 5.0
    assert not node.logger_double.warning.called


@pytest.mark.parametrize('file_name', ['web.yaml', 'web.example.yaml'])
def test_the_shipped_yaml_files_declare_only_known_parameters(file_name):
    known = {name for name, _ in bridge.PARAMETERS}
    assert set(_flat(_shipped_params(file_name))) <= known


@pytest.mark.parametrize('file_name', ['web.yaml', 'web.example.yaml'])
def test_the_shipped_yaml_types_match_the_declared_defaults_for_static_parameters(file_name):
    defaults = dict(bridge.PARAMETERS)
    for name, value in _flat(_shipped_params(file_name)).items():
        if name not in bridge.DYNAMIC_PARAMETERS:
            assert _rclpy_type(value) == _rclpy_type(defaults[name]), name


# =========================================================================================
# Audit (task 21): limits.max_connections, uvicorn options, initialpose gating, map arrays.
# =========================================================================================

# --- limits.max_connections ------------------------------------------------------------------

def test_max_connections_default_is_64_and_static_int():
    assert dict(bridge.PARAMETERS)['limits.max_connections'] == 64
    assert 'limits.max_connections' not in bridge.DYNAMIC_PARAMETERS
    assert bridge.MAX_CONNECTIONS_LIMIT == 1024


def test_read_settings_max_connections_default_is_valid():
    node = _settings_node()
    node._read_settings()
    assert node._max_connections == 64


@pytest.mark.parametrize('value', [1, 2, 64, 1000, 1024])
def test_read_settings_accepts_max_connections_from_1_to_1024(value):
    node = _settings_node(**{'limits.max_connections': value})
    node._read_settings()
    assert node._max_connections == value


@pytest.mark.parametrize('bad', [0, -1, 1025, 10 ** 6, True, False, 64.0, 1.5, '64', None,
                                 [], float('nan')])
def test_read_settings_invalid_max_connections_is_a_config_error(bad):
    with pytest.raises(bridge.ConfigError, match='limits.max_connections'):
        _settings_node(**{'limits.max_connections': bad})._read_settings()


def test_max_connections_of_1025_names_the_ceiling():
    with pytest.raises(bridge.ConfigError, match='at most 1024'):
        _settings_node(**{'limits.max_connections': 1025})._read_settings()


@pytest.mark.parametrize('bad', ['64', 64.5, 0])
def test_a_wrong_max_connections_override_is_a_config_error_through_the_declaration(bad):
    text = _declared_refusal({'limits.max_connections': bad})
    assert 'limits.max_connections' in text
    assert str(bad) not in text.replace('limits.max_connections', '') or bad == 0


# --- the options handed to uvicorn ---------------------------------------------------------------

class FakeUvicornConfig:
    def __init__(self, app, **kwargs):
        self.app, self.kwargs = app, kwargs


class FakeUvicornServer:
    def __init__(self, config):
        self.config = config
        self.started = True
        self.should_exit = False
        self.sockets = None

    def run(self, sockets=None):
        self.sockets = sockets


def _server_node(max_connections=64, max_ws_bytes=4096):
    node = make_node({'host': '127.0.0.1'})
    node.host, node.port = '127.0.0.1', 0
    node.app = object()
    node._max_ws_bytes = max_ws_bytes
    node._max_connections = max_connections
    node.server = node.thread = None
    return node


def _start_with_fake_uvicorn(node):
    fake = NS(Config=FakeUvicornConfig, Server=FakeUvicornServer)
    with mock.patch.object(bridge, 'uvicorn', fake):
        node.start_server()
    node.thread.join(5)
    return node.server.config


def test_start_server_passes_the_slow_client_limits_to_uvicorn():
    config = _start_with_fake_uvicorn(_server_node(max_connections=37, max_ws_bytes=2048))
    kwargs = config.kwargs
    assert kwargs['limit_concurrency'] == 37
    assert kwargs['timeout_keep_alive'] == 5 == bridge.UVICORN_KEEP_ALIVE_S
    assert kwargs['h11_max_incomplete_event_size'] == 16384 == bridge.UVICORN_MAX_HEADER_BYTES
    assert kwargs['ws_max_size'] == 2048
    assert kwargs['access_log'] is False and kwargs['log_level'] == 'warning'


def test_start_server_hands_over_the_bound_socket_and_the_app():
    node = _server_node()
    config = _start_with_fake_uvicorn(node)
    assert config.app is node.app
    [sock] = node.server.sockets
    try:
        assert sock.getsockname()[0] == '127.0.0.1'
    finally:
        sock.close()


def _real_uvicorn_config_parameters():
    """Parameter names of the REAL uvicorn.Config.__init__, or None if only a stub exists."""
    import ast
    import inspect
    real = getattr(bridge.uvicorn, 'Config', None)
    try:
        names = set(inspect.signature(real.__init__).parameters)
    except (TypeError, ValueError):
        names = set()
    if 'limit_concurrency' in names:
        return names
    source_dir = os.environ.get('MTRB_UVICORN_SOURCE_DIR')      # unpacked uvicorn, no deps
    if source_dir and os.path.isfile(os.path.join(source_dir, 'config.py')):
        with open(os.path.join(source_dir, 'config.py'), encoding='utf-8') as handle:
            tree = ast.parse(handle.read())
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == 'Config':
                for item in node.body:
                    if isinstance(item, ast.FunctionDef) and item.name == '__init__':
                        return {a.arg for a in item.args.args + item.args.kwonlyargs}
    return None


def test_the_uvicorn_options_exist_in_the_real_config_signature():
    names = _real_uvicorn_config_parameters()
    if names is None:
        pytest.skip('only a uvicorn stub is importable (set MTRB_UVICORN_SOURCE_DIR)')
    used = set(_start_with_fake_uvicorn(_server_node()).kwargs)
    assert {'limit_concurrency', 'timeout_keep_alive', 'h11_max_incomplete_event_size',
            'ws_max_size', 'access_log', 'log_level'} <= used
    assert used <= names, sorted(used - names)


def test_start_server_without_limit_concurrency_would_be_caught():
    kwargs = _start_with_fake_uvicorn(_server_node()).kwargs
    assert 'limit_concurrency' in kwargs and 'timeout_keep_alive' in kwargs


# --- initialpose: the node refuses while Nav2 may be driving or a teleop lease is held ----------

def _pose_node(monkeypatch):
    node = make_teleop_bridge(monkeypatch)
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: 'STAMP',
                                               nanoseconds=int(node.clock[0] * 1e9)))
    monkeypatch.setattr(bridge, 'PoseWithCovarianceStamped', _pose_msg_factory())
    return node


def _pose_published(node):
    return node._initial_pose_pub.publish.call_count


def test_initial_pose_is_published_when_nothing_is_moving(monkeypatch):
    node = _pose_node(monkeypatch)
    node.publish_initial_pose(1.0, 2.0, 0.5)
    assert _pose_published(node) == 1


def test_initial_pose_is_refused_with_an_active_goal(monkeypatch):
    node = _pose_node(monkeypatch)
    node.send_goal(1.0, 2.0, 0.5)
    with pytest.raises(bridge.NavBusyError):
        node.publish_initial_pose(1.0, 2.0, 0.5)
    assert _pose_published(node) == 0


def test_initial_pose_is_refused_with_an_orphan_goal(monkeypatch):
    node = _pose_node(monkeypatch)
    make_orphan(node)
    with pytest.raises(bridge.NavBusyError):
        node.publish_initial_pose(1.0, 2.0, 0.5)
    assert _pose_published(node) == 0


def test_initial_pose_is_refused_with_a_pending_cancel_all_while_nav2_is_up(monkeypatch):
    node = _pose_node(monkeypatch)
    cancel_client(node).ready = False
    node.cancel()
    assert node._cancel_all is not None
    node._nav_client.ready = True
    with pytest.raises(bridge.NavBusyError):
        node.publish_initial_pose(1.0, 2.0, 0.5)
    assert _pose_published(node) == 0


def test_initial_pose_is_allowed_with_a_pending_cancel_all_if_nav2_is_absent(monkeypatch):
    node = _pose_node(monkeypatch)
    cancel_client(node).ready = False
    node.cancel()
    node._nav_client.ready = False
    node.publish_initial_pose(1.0, 2.0, 0.5)
    assert _pose_published(node) == 1


def test_initial_pose_is_refused_while_a_teleop_controller_holds_the_lease(monkeypatch):
    node = _pose_node(monkeypatch)
    assert node.teleop(1, tmsg(0)) is None
    with pytest.raises(bridge.NavBusyError):
        node.publish_initial_pose(1.0, 2.0, 0.5)
    assert _pose_published(node) == 0


def test_initial_pose_is_allowed_again_once_the_teleop_lease_expired(monkeypatch):
    node = _pose_node(monkeypatch)
    node.teleop(1, tmsg(0))
    node.clock[0] += 5.0                  # far beyond the 300 ms teleop timeout
    node.publish_initial_pose(1.0, 2.0, 0.5)
    assert _pose_published(node) == 1


def test_initial_pose_is_allowed_after_the_controller_disconnects(monkeypatch):
    node = _pose_node(monkeypatch)
    node._nav_client.ready = False
    node.teleop(1, tmsg(0))
    node.teleop_disconnect(1)
    node.publish_initial_pose(1.0, 2.0, 0.5)
    assert _pose_published(node) == 1


def test_initial_pose_without_a_teleop_gate_depends_on_nav_only(monkeypatch):
    node = _pose_node(monkeypatch)
    node._teleop_gate = None
    node.publish_initial_pose(1.0, 2.0, 0.5)
    assert _pose_published(node) == 1
    node.send_goal(1.0, 2.0, 0.5)
    with pytest.raises(bridge.NavBusyError):
        node.publish_initial_pose(1.0, 2.0, 0.5)


def test_initial_pose_busy_check_holds_the_teleop_lock(monkeypatch):
    node = _pose_node(monkeypatch)
    seen = []
    original = node._nav_busy

    def spy():
        seen.append(node._teleop_lock.locked())
        return original()

    node._nav_busy = spy
    node.publish_initial_pose(1.0, 2.0, 0.5)
    assert seen == [True]


# --- _broadcast_map: array('b') conversion -------------------------------------------------------

def _raw_map(data, width=2, height=2):
    info = NS(width=width, height=height, resolution=0.05,
              origin=NS(position=NS(x=0.0, y=0.0)))
    return NS(header=NS(frame_id='map'), info=info, data=data)


def _published_map(node):
    messages = drain(node)
    return [m for m in messages if isinstance(m, bytes)]


@pytest.mark.parametrize('factory', [list, tuple])
def test_a_list_or_tuple_map_is_published_identically_to_an_array_b(factory):
    cells = [0, -1, 100, 50]
    reference = make_bridge()
    reference._on_map(_raw_map(array('b', cells)))
    reference._broadcast_map()
    expected = _published_map(reference)
    node = make_bridge()
    node._on_map(_raw_map(factory(cells)))
    node._broadcast_map()
    assert _published_map(node) == expected and len(expected) == 1
    assert node._map_version == 1 and not node.logger.warning.called


def test_an_array_b_map_is_not_converted_again(monkeypatch):
    data = array('b', [0, -1, 100, 0])
    node = make_bridge()
    node._on_map(_raw_map(data))
    seen = []
    real = bridge.occupancy_to_bytes

    def spy(arg, width, height):
        seen.append(arg)
        return real(arg, width, height)

    monkeypatch.setattr(bridge, 'occupancy_to_bytes', spy)
    node._broadcast_map()
    assert seen == [data] and seen[0] is data


@pytest.mark.parametrize('data', [[0, 300, -1, 0], [0, -129, 0, 0], [0, 128, 0, 0]])
def test_cells_outside_int8_are_ignored_with_a_warning_and_never_raise(data):
    node = make_bridge()
    node._on_map(_raw_map(data))
    node._broadcast_map()                      # must not raise
    assert drain(node) == [] and node._map_version == 0
    assert node.store.snapshot()['map'] is None
    assert any('Invalid map ignored' in m for m in _warnings(node))


@pytest.mark.parametrize('data', [['a', 'b', 'c', 'd'], [None, 0, 0, 0], [1.5, 0, 0, 0],
                                  [[0], [0], [0], [0]]])
def test_a_map_whose_cells_cannot_be_converted_is_ignored_with_a_warning(data):
    node = make_bridge()
    node._on_map(_raw_map(data))
    node._broadcast_map()
    assert drain(node) == [] and node._map_version == 0
    assert any('Invalid map ignored' in m for m in _warnings(node))


def test_the_invalid_cell_warning_is_throttled_and_carries_no_cell_data():
    node = make_bridge()
    for _ in range(5):
        node._on_map(_raw_map(['SECRET-cell', 0, 0, 0]))
        node._broadcast_map()
    warnings = [m for m in _warnings(node) if 'Invalid map ignored' in m]
    assert len(warnings) == 1 and 'SECRET' not in warnings[0]
    node.clock[0] += bridge.LOG_THROTTLE_S
    node._on_map(_raw_map(['SECRET-cell', 0, 0, 0]))
    node._broadcast_map()
    assert len([m for m in _warnings(node) if 'Invalid map ignored' in m]) == 2


def test_a_valid_map_after_an_invalid_one_is_published():
    node = make_bridge()
    node._on_map(_raw_map(['x', 0, 0, 0]))
    node._broadcast_map()
    node._on_map(_raw_map([0, 0, 0, 0]))
    node._broadcast_map()
    assert len(_published_map(node)) == 1 and node._map_version == 1
