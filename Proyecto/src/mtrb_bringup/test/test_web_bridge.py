"""Unit tests for mtrb_web_bridge token/host/port selection (need rclpy; skipped without)."""

from array import array
import json
import logging
import math
import threading
from types import SimpleNamespace as NS
from unittest import mock

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('uvicorn')

from mtrb_bringup import mtrb_web_bridge as bridge  # noqa: E402, I100
from mtrb_bringup.web_server import (  # noqa: E402, I100
    BroadcastHub, StateStore, TOKEN_ENV_VAR)

SECRET = 'S3cr3t-token-value'


class FakeParam:
    def __init__(self, value):
        self.value = value


def make_node(params=None):
    """Build a bridge without running Node.__init__ (no ROS context needed)."""
    node = object.__new__(bridge.MtrbWebBridge)
    values = {'auth.token': '', 'host': '127.0.0.1', 'port': 8080}
    values.update(params or {})
    node.get_parameter = lambda name: FakeParam(values[name])
    node.logger = mock.Mock()
    node.get_logger = lambda: node.logger
    return node


@pytest.fixture(autouse=True)
def clean_env(monkeypatch):
    monkeypatch.delenv(TOKEN_ENV_VAR, raising=False)


def test_select_token_env_has_priority(monkeypatch):
    monkeypatch.setenv(TOKEN_ENV_VAR, 'from-env')
    node = make_node({'auth.token': 'from-param'})
    with mock.patch.object(bridge, 'resolve_token', return_value='from-file'):
        assert node._select_token('/x') == 'from-env'


def test_select_token_parameter_over_files():
    node = make_node({'auth.token': ' from-param '})
    with mock.patch.object(bridge, 'resolve_token', return_value='from-file'):
        assert node._select_token('/x') == 'from-param'


def test_select_token_numeric_parameter_becomes_string():
    node = make_node({'auth.token': 123456})
    assert node._select_token('/x') == '123456'


@pytest.mark.parametrize('value', ['', '   ', None, True, False])
def test_select_token_empty_or_bool_falls_back_to_resolve_token(value):
    node = make_node({'auth.token': value})
    with mock.patch.object(bridge, 'resolve_token', return_value='from-file') as res:
        assert node._select_token('/dir') == 'from-file'
    res.assert_called_once_with('/dir')


def test_select_token_all_empty_returns_empty():
    node = make_node()
    with mock.patch.object(bridge, 'resolve_token', return_value=''):
        assert node._select_token('/x') == ''


def _fake_settings(self):
    """Stand-in for _read_settings (token/port tests only; see the _read_settings tests below)."""
    self._max_clients = 5
    self._max_ws_bytes = 4096
    self._dock = (0.0, 0.0, 0.0)


def _init_node(params, token_result=SECRET):
    """Run the real __init__ with ROS pieces replaced by doubles."""
    values = {'auth.token': '', 'host': '127.0.0.1', 'port': 8080}
    values.update(params)
    logger = mock.Mock()
    with mock.patch.object(bridge.Node, '__init__', lambda self, name: None), \
            mock.patch.object(bridge.MtrbWebBridge, 'declare_parameter',
                              create=True, new=lambda self, *a, **k: None), \
            mock.patch.object(bridge.MtrbWebBridge, 'get_parameter', create=True,
                              new=lambda self, n: FakeParam(values[n])), \
            mock.patch.object(bridge.MtrbWebBridge, 'get_logger', create=True,
                              new=lambda self: logger), \
            mock.patch.object(bridge, 'get_package_share_directory', return_value='/share'), \
            mock.patch.object(bridge, 'resolve_token', return_value=token_result), \
            mock.patch.object(bridge.MtrbWebBridge, '_read_settings', new=_fake_settings), \
            mock.patch.object(bridge.MtrbWebBridge, '_init_ros_io', new=lambda self: None), \
            mock.patch.object(bridge, 'create_app', return_value=object()):
        node = bridge.MtrbWebBridge()
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
    with pytest.raises(bridge.ConfigError):
        _init_node({}, token_result='')


def test_placeholder_token_logs_warning_without_leaking_token():
    placeholder = 'CHANGE-ME-abc123'
    node, logger = _init_node({'auth.token': placeholder})
    assert logger.warning.called
    logged = ' '.join(str(c) for c in logger.method_calls)
    assert placeholder not in logged


def test_real_token_does_not_warn_and_is_never_logged(caplog):
    with caplog.at_level(logging.DEBUG):
        node, logger = _init_node({'auth.token': SECRET})
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
    'mode': 'map', 'dock.x': 0.0, 'dock.y': 0.0, 'dock.yaw': 0.0,
}


def _settings_node(**overrides):
    values = dict(VALID_SETTINGS)
    values.update(overrides)
    node = object.__new__(bridge.MtrbWebBridge)
    node.get_parameter = lambda name: FakeParam(values[name])
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
    node.create_subscription = mock.Mock()
    node.create_timer = mock.Mock()
    node.create_publisher = mock.Mock()
    node.create_client = mock.Mock()
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


def test_init_ros_io_subscribes_to_the_four_expected_topics_only():
    node = _io_node()
    topics = sorted(c.args[1] for c in node.create_subscription.call_args_list)
    assert topics == ['battery_state', 'map', 'plan', 'scan']


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
    }
    assert bridge.MAP_PERIOD_S == 1.0


def test_init_ros_io_creates_only_the_initialpose_publisher():
    node = _io_node()
    topics = [c.args[1] for c in node.create_publisher.call_args_list]
    assert topics == ['initialpose']
    assert node.create_publisher.call_args.args[0] is bridge.PoseWithCovarianceStamped


def test_init_ros_io_creates_the_navigate_to_pose_action_client_with_relative_name():
    node = _io_node()
    args = node.action_client_cls.call_args.args
    assert args[1] is bridge.NavigateToPose
    assert args[2] == 'navigate_to_pose'


def test_init_ros_io_creates_the_cancel_all_client_with_type_and_exact_service_name():
    node = _io_node()
    node.create_client.assert_called_once()
    args = node.create_client.call_args.args
    assert args[0] is bridge.CancelGoal
    assert args[1] == 'navigate_to_pose/_action/cancel_goal'
    assert node._cancel_all_client is node.create_client.return_value


def test_cancel_all_client_is_none_before_ros_io_is_created():
    assert make_bridge()._cancel_all_client is None


def test_init_ros_io_publishes_idle_nav_state():
    node = object.__new__(bridge.MtrbWebBridge)
    node._pose_period_s, node._scan_period_s, node._status_period_s = 0.1, 0.2, 1.0
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


def test_real_rclpy_node_only_publishes_initialpose_and_has_expected_timers(monkeypatch):
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
        assert names == {'initialpose'}, f'only initialpose may be published, found {own}'
        assert len(own) == 1
        for topic in own:  # explicit guard for the safety-critical topics
            assert not topic.lstrip('/').startswith(('cmd_vel', 'cmd_vel_nav'))
        # Node.timers: rclpy Jazzy exposes Timer.timer_period_ns (int); fall back to the
        # handle's period if a distro lacks the property (defensive, API varies by distro).
        periods = sorted(round(t.timer_period_ns / 1e9, 3) for t in node.timers)
        assert periods == [0.1, 0.2, 0.5, 1.0, 1.0]
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


def test_node_exposes_no_cmd_vel_publisher_in_source():
    import inspect
    source = inspect.getsource(bridge)
    assert 'cmd_vel' not in source


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
