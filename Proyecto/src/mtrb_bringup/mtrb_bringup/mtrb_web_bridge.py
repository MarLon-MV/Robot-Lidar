"""ROS 2 node that hosts the web server (FastAPI + uvicorn) next to rclpy."""

import array
import functools
import hashlib
import math
import os
import re
import socket
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from mtrb_bringup.utils import MODES, validate_web_port
from mtrb_bringup.web_server import (
    BroadcastHub, create_app, DEFAULT_TOKEN_FILE, EventSummary,
    is_weak_token, json_message, load_token, MapPort, NavBusyError, NavPort,
    NavUnavailableError, SlamTimeoutError, SlamUnavailableError, StateStore, TOKEN_MIN_LENGTH,
    TokenError)
from mtrb_bringup.web_utils import (
    downsample_scan, occupancy_to_bytes, pack_map_binary, quaternion_from_yaw,
    scan_to_map_points, simplify_path, TELEOP_PUBLISH, TELEOP_ZERO, TELEOP_ZERO_BURST,
    TeleopAction, TeleopGate, yaw_from_quaternion)
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.exceptions import InvalidParameterTypeException, InvalidParameterValueException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, LaserScan
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import uvicorn

PACKAGE_NAME = 'mtrb_bringup'
EXIT_FAILURE = 1
SERVER_START_TIMEOUT_S = 5.0
SERVER_STOP_TIMEOUT_S = 5.0
SOCKET_BACKLOG = 16
EXECUTOR_THREADS = 4
MAP_FRAME = 'map'
BASE_FRAME = 'base_link'
TF_BUFFER_S = 10.0
TF_MAX_AGE_S = 1.0       # a pose/scan transform older than this is treated as a failure
SCAN_MAX_AGE_S = 1.0     # a scan older than this is not broadcast
MAP_PERIOD_S = 1.0       # the map is re-sent at most once per period
LOG_THROTTLE_S = 5.0
NS_PER_S = 1e9
NAV_FEEDBACK_PERIOD_S = 0.5     # distance_remaining is published at most at 2 Hz
NAV_RESPONSE_TIMEOUT_S = 5.0    # max wait for Nav2 to accept/reject a goal or a cancel
NAV_SERVER_LOST_TIMEOUT_S = 3.0  # an active goal whose action server vanished is failed
NAV_WATCHDOG_PERIOD_S = 0.5
NAV_CANCEL_RETRY_S = 1.0         # min time between two cancel requests of the same goal
MAX_ORPHAN_GOALS = 8             # bound of goals kept until Nav2 confirms they ended
STOP_CANCEL_ALL_MIN_PERIOD_S = 0.5  # min time between cancel-all requests armed by stops
CANCEL_ALL_WARN_PERIOD_S = 10.0  # min time between 'service not ready' warnings
MAX_CANCEL_ALL_ATTEMPTS = 5      # bound of cancel-all requests per release/cancel event
# action_msgs/srv/CancelGoal return codes (stable values of the ROS 2 interface).
CANCEL_ERROR_NONE = 0
CANCEL_ERROR_REJECTED = 1
CANCEL_ERROR_UNKNOWN_GOAL_ID = 2
CANCEL_ERROR_GOAL_TERMINATED = 3
# Initial pose covariance (INITIAL, tune on the robot): same values as
# initial_pose_publisher.py (variances: m^2 for x/y, rad^2 for yaw).
INITIAL_COVARIANCE_XY = 0.25
INITIAL_COVARIANCE_YAW = 0.0685
DOCK_LIMIT = 10000.0            # sanity bound of the configured dock pose (m / rad)
# Teleop. Hard ceilings = Create 3 limits (0.46 m/s, 1.9 rad/s): a parameter above them is a
# configuration error, never silently accepted. Watchdog period << teleop.timeout_ms.
TELEOP_WATCHDOG_PERIOD_S = 0.02
TELEOP_HARD_MAX_VX_MPS = 0.46
TELEOP_HARD_MAX_WZ_RADPS = 1.9
TELEOP_MIN_TIMEOUT_MS = 50
TELEOP_MAX_TIMEOUT_MS = 1000
MS_PER_S = 1000.0
# Status: lifecycle polling (Hz) is deliberately slow; it must never load the Pi or the executor.
LIFECYCLE_MAX_RATE_HZ = 0.5
LIFECYCLE_MIN_RATE_HZ = 0.05
LIFECYCLE_MAX_NODES = 16
NODE_NAME_PATTERN = re.compile(r'[A-Za-z_][A-Za-z0-9_]{0,63}')
# State labels of lifecycle_msgs/State (primary and transition states). Anything else is
# reported as 'unknown'; a missing service is 'unavailable'.
LIFECYCLE_LABELS = frozenset((
    'unknown', 'unconfigured', 'inactive', 'active', 'finalized', 'configuring',
    'cleaningup', 'shuttingdown', 'activating', 'deactivating', 'errorprocessing'))
STATE_UNKNOWN = 'unknown'
STATE_UNAVAILABLE = 'unavailable'
# Localization node that the launch starts in each mode (mtrb.launch.py _localization/_slam).
MODE_NODES = {'map': ('map_server', 'amcl'), 'mapping': ('slam_toolbox',)}
# slam_toolbox services (relative: /<ns>/slam_toolbox/...; both created by the node
# `slam_toolbox` with these names in slam_toolbox_common.cpp / map_saver.cpp, Jazzy).
SAVE_MAP_SERVICE = 'slam_toolbox/save_map'
SERIALIZE_MAP_SERVICE = 'slam_toolbox/serialize_map'

NAV_IDLE = 'idle'
NAV_ACTIVE = 'active'
NAV_SUCCEEDED = 'succeeded'
NAV_CANCELED = 'canceled'
NAV_FAILED = 'failed'
# Final state for each action status; anything else (unknown, aborted) is a failure.
_FINAL_STATE = {GoalStatus.STATUS_SUCCEEDED: NAV_SUCCEEDED,
                GoalStatus.STATUS_CANCELED: NAV_CANCELED}

# (name, default) of every parameter; defaults mirror param/web.yaml.
PARAMETERS = (
    ('host', '0.0.0.0'),
    ('port', 8080),
    ('rates.pose', 10.0),             # Hz
    ('rates.scan', 5.0),              # Hz
    ('rates.status', 1.0),            # Hz
    ('teleop.max_vx', 0.2),           # m/s
    ('teleop.max_wz', 0.8),           # rad/s
    ('teleop.timeout_ms', 300),       # ms
    ('teleop.enabled', True),
    ('mode', ''),                     # injected by the launch ('map' | 'mapping')
    ('scan.max_points', 360),
    ('scan.min_range', 0.15),         # m
    ('scan.max_range', 12.0),         # m
    ('plan.max_points', 200),
    ('limits.max_clients', 5),
    ('limits.max_ws_bytes', 4096),
    ('limits.max_connections', 64),   # uvicorn limit_concurrency (HTTP + WebSocket)
    ('auth.token_file', DEFAULT_TOKEN_FILE),  # path of the private token file (not the token)
    ('dock.x', 0.0),                  # m
    ('dock.y', 0.0),                  # m
    ('dock.yaw', 0.0),                # rad
    ('rates.lifecycle', 0.5),         # Hz, at most LIFECYCLE_MAX_RATE_HZ
    ('status.nodes', ['controller_server', 'planner_server', 'bt_navigator',
                      'behavior_server', 'velocity_smoother']),
    ('map_dir', ''),                  # '' = ~/mtrb_maps
    ('auth.allow_example_token', False),  # development only; loopback host required
    ('maps.min_free_mb', 200),        # MB of free disk required to save a map
    ('maps.min_interval_s', 5.0),     # s between two map saves
)


# Parameters that accept both int and float from YAML (e.g. `teleop.max_vx: 1`): declared
# with dynamic typing so rclpy does not raise at declaration; _number() validates the type.
# `status.nodes` is dynamic too so that an empty list is not rejected by the static typing.
# There is NO token parameter: a ROS parameter is readable by the whole ROS domain.
# Every other parameter keeps static typing:
# a wrong type is reported as ConfigError by _declare_parameters.
DYNAMIC_PARAMETERS = frozenset((
    'rates.pose', 'rates.scan', 'rates.status', 'rates.lifecycle',
    'teleop.max_vx', 'teleop.max_wz', 'scan.min_range', 'scan.max_range',
    'dock.x', 'dock.y', 'dock.yaw', 'maps.min_free_mb', 'maps.min_interval_s',
    'status.nodes'))

UVICORN_KEEP_ALIVE_S = 5             # idle keep-alive connections are closed after this
UVICORN_MAX_HEADER_BYTES = 16384     # h11_max_incomplete_event_size (HTTP request head)
MAX_CONNECTIONS_LIMIT = 1024         # ceiling of limits.max_connections
LOG_SUMMARY_PERIOD_S = 1.0           # stop/cancel floods are logged at most once per period
# uvicorn is bound through an AF_INET socket (start_server), so in practice only IPv4 hosts
# can be listened on; '::1' is accepted here but cannot be bound (the bind fails: ConfigError).
LOOPBACK_HOSTS = ('127.0.0.1', '::1', 'localhost')
DEFAULT_MAP_DIR = '~/mtrb_maps'


class ConfigError(Exception):
    """Invalid or missing configuration: the node must not start."""


class MtrbWebBridge(Node, NavPort, MapPort):
    """Hold the shared StateStore, the NavPort implementation and the web server thread."""

    _teleop_gate = None  # TeleopGate; None (never configured) = teleop refused, fail closed
    _odom_rx_s = None    # ROS time of the last odometry message (None = never received)

    def __init__(self):
        """Declare parameters, validate them and prepare (not start) the web app."""
        super().__init__('mtrb_web_bridge')
        self._declare_parameters()
        self.host = self.get_parameter('host').value
        try:
            self.port = validate_web_port(self.get_parameter('port').value)
        except ValueError as err:
            raise ConfigError(f'Invalid port: {err}') from err
        if not self.host:
            raise ConfigError('Invalid host: empty')

        share = get_package_share_directory(PACKAGE_NAME)
        token = self._load_token()
        self._check_token_strength(token)

        self._read_settings()
        self._read_teleop_settings()
        self.store = StateStore(clock=self._now_s)
        self.hub = BroadcastHub(self._max_clients)
        self.app = create_app(self.store, token, share + '/web', self.hub, self._max_ws_bytes,
                              nav_port=self, dock=self._dock, map_port=self,
                              map_dir=self._map_dir, on_event=self._log_map_event,
                              map_min_free_mb=self._map_min_free_mb,
                              map_min_interval_s=self._map_min_interval_s)
        self.server = None
        self.thread = None
        self._init_state()
        self._init_ros_io()

    # --- Configuration ---------------------------------------------------------------

    def _declare_parameters(self):
        """
        Declare every parameter; a wrong override type or value raises ConfigError.

        rclpy raises on declaration when an override has a different type than the default
        (statically typed parameter). The message names the parameter, never its value.
        """
        for name, default in PARAMETERS:
            try:
                if name in DYNAMIC_PARAMETERS:
                    self.declare_parameter(
                        name, default, ParameterDescriptor(dynamic_typing=True))
                else:
                    self.declare_parameter(name, default)
            except (InvalidParameterTypeException, InvalidParameterValueException,
                    TypeError, ValueError) as err:
                raise ConfigError(
                    f'Invalid parameter {name}: wrong type or value ({type(err).__name__})'
                ) from None  # the rclpy cause message includes the value

    def _load_token(self):
        """
        Read the token from MTRB_WEB_TOKEN or the private file auth.token_file; ConfigError.

        The token is never a ROS parameter. Neither the token nor the file content is ever
        printed or logged.
        """
        token_file = self.get_parameter('auth.token_file').value
        if not isinstance(token_file, str):
            raise ConfigError('Invalid parameter auth.token_file: string expected')
        try:
            token = load_token(token_file)
        except TokenError as err:
            raise ConfigError(f'Unusable auth token file: {err}') from err
        if not token:
            raise ConfigError(
                'No auth token: set the environment variable MTRB_WEB_TOKEN or create the '
                f'file {token_file} (mode 0600, owned by this user, regular file) with a '
                f'random token of at least {TOKEN_MIN_LENGTH} characters')
        return token

    def _check_token_strength(self, token):
        """
        Refuse the example token and weak tokens (see is_weak_token); raise ConfigError.

        The only exception is auth.allow_example_token=true (a strict bool read from the node
        parameters, never from a client) together with a loopback host: a warning is logged.
        The token itself is never printed or logged.
        """
        allow = self.get_parameter('auth.allow_example_token').value
        if not isinstance(allow, bool):  # validated always, whatever the token strength
            raise ConfigError(
                f'Invalid parameter auth.allow_example_token: {allow!r} (bool)')
        if not token.strip():
            raise ConfigError('The auth token is blank')
        if not is_weak_token(token):
            return
        if not (allow and str(self.host).strip().lower() in LOOPBACK_HOSTS):
            raise ConfigError(
                f'The auth token is the example one, blank, a single repeated character or '
                f'shorter than {TOKEN_MIN_LENGTH} characters: use a random one in '
                'MTRB_WEB_TOKEN or the token file (auth.allow_example_token is only honoured '
                'for a loopback host)')
        self.get_logger().warning(
            'DEVELOPMENT ONLY: weak or example auth token accepted because '
            'auth.allow_example_token is true and the host is loopback')

    def _number(self, name, minimum, integer=False):
        """Read a numeric parameter and require a finite value >= minimum."""
        value = self.get_parameter(name).value
        valid_type = isinstance(value, int) if integer else isinstance(value, (int, float))
        if (not valid_type or isinstance(value, bool) or not math.isfinite(value)
                or value < minimum):
            raise ConfigError(f'Invalid parameter {name}: {value!r} (minimum {minimum})')
        return value

    def _read_settings(self):
        """Validate the rates and limits used by the broadcast; raise ConfigError."""
        self._pose_period_s = 1.0 / self._number('rates.pose', 0.1)
        self._scan_period_s = 1.0 / self._number('rates.scan', 0.1)
        self._status_period_s = 1.0 / self._number('rates.status', 0.1)
        self._scan_max_points = self._number('scan.max_points', 1, integer=True)
        self._scan_min_range_m = self._number('scan.min_range', 0.0)
        self._scan_max_range_m = self._number('scan.max_range', 0.0)
        if self._scan_min_range_m >= self._scan_max_range_m:
            raise ConfigError('scan.min_range must be lower than scan.max_range')
        self._plan_max_points = self._number('plan.max_points', 2, integer=True)
        self._max_clients = self._number('limits.max_clients', 1, integer=True)
        self._max_ws_bytes = self._number('limits.max_ws_bytes', 1, integer=True)
        self._max_connections = self._number('limits.max_connections', 1, integer=True)
        if self._max_connections > MAX_CONNECTIONS_LIMIT:
            raise ConfigError(f'Invalid parameter limits.max_connections: '
                              f'{self._max_connections!r} (at most {MAX_CONNECTIONS_LIMIT})')
        self._dock = tuple(float(self._number(f'dock.{axis}', -DOCK_LIMIT))
                           for axis in ('x', 'y', 'yaw'))
        if any(abs(v) > DOCK_LIMIT for v in self._dock):
            raise ConfigError(f'dock.x/y/yaw must be within +-{DOCK_LIMIT}')
        mode = str(self.get_parameter('mode').value).strip().lower()
        self._mode = mode if mode in MODES else None  # None = unknown, reported as null
        self._read_status_settings()
        self._read_map_dir()
        self._map_min_free_mb = self._number('maps.min_free_mb', 0)
        self._map_min_interval_s = float(self._number('maps.min_interval_s', 0.0))

    def _read_status_settings(self):
        """Validate the lifecycle polling rate and node list; raise ConfigError."""
        rate_hz = self._number('rates.lifecycle', LIFECYCLE_MIN_RATE_HZ)
        if rate_hz > LIFECYCLE_MAX_RATE_HZ:
            raise ConfigError(f'Invalid parameter rates.lifecycle: {rate_hz!r} '
                              f'(at most {LIFECYCLE_MAX_RATE_HZ} Hz)')
        self._lifecycle_period_s = 1.0 / rate_hz
        configured = self.get_parameter('status.nodes').value
        if (not isinstance(configured, (list, tuple)) or len(configured) > LIFECYCLE_MAX_NODES
                or not all(isinstance(n, str) and NODE_NAME_PATTERN.fullmatch(n)
                           for n in configured)
                or len(set(configured)) != len(configured)):
            raise ConfigError(
                f'Invalid parameter status.nodes: {configured!r} (at most {LIFECYCLE_MAX_NODES} '
                'unique node names [A-Za-z_][A-Za-z0-9_]*)')
        names = list(MODE_NODES.get(self._mode, ())) + [
            n for n in configured if n not in MODE_NODES.get(self._mode, ())]
        self._status_nodes = tuple(names)

    def _read_map_dir(self):
        """Resolve the map directory: parameter map_dir or the default ~/mtrb_maps."""
        value = self.get_parameter('map_dir').value
        if not isinstance(value, str):
            raise ConfigError(f'Invalid parameter map_dir: {value!r} (string)')
        value = os.path.expanduser(value.strip() or DEFAULT_MAP_DIR)
        if value.startswith('~'):
            # HOME is not defined: start anyway, but every save answers 500 map_dir_invalid.
            self.get_logger().warning(
                'Cannot resolve the home directory: saving maps is disabled (set map_dir)')
            self._map_dir = value
            return
        if not os.path.isabs(value):
            raise ConfigError(f'Invalid parameter map_dir: {value!r} (absolute path)')
        self._map_dir = os.path.normpath(value)  # created with mode 0o755 at the first save

    def _read_teleop_settings(self):
        """Validate the teleop parameters and build the TeleopGate; raise ConfigError."""
        limits = {}
        for name, ceiling in (('max_vx', TELEOP_HARD_MAX_VX_MPS),
                              ('max_wz', TELEOP_HARD_MAX_WZ_RADPS)):
            value = self.get_parameter(f'teleop.{name}').value
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not 0.0 < value <= ceiling):
                raise ConfigError(
                    f'Invalid parameter teleop.{name}: {value!r} (must be in (0, {ceiling}])')
            limits[name] = float(value)
        timeout_ms = self.get_parameter('teleop.timeout_ms').value
        if (isinstance(timeout_ms, bool) or not isinstance(timeout_ms, int)
                or not TELEOP_MIN_TIMEOUT_MS <= timeout_ms <= TELEOP_MAX_TIMEOUT_MS):
            raise ConfigError(
                f'Invalid parameter teleop.timeout_ms: {timeout_ms!r} (integer between '
                f'{TELEOP_MIN_TIMEOUT_MS} and {TELEOP_MAX_TIMEOUT_MS})')
        enabled = self.get_parameter('teleop.enabled').value
        if not isinstance(enabled, bool):
            raise ConfigError(f'Invalid parameter teleop.enabled: {enabled!r} (bool)')
        self._teleop_gate = TeleopGate(
            limits['max_vx'], limits['max_wz'], timeout_ms / MS_PER_S, time.monotonic,
            enabled=enabled)  # monotonic, NOT the ROS clock: see _init_ros_io watchdog timer

    def _now_s(self):
        """Return the ROS time in seconds (sim time aware); never the wall clock."""
        return self._now_ns() / NS_PER_S

    def _now_ns(self):
        """Return the ROS time in nanoseconds (sim time aware); never the wall clock."""
        return self.get_clock().now().nanoseconds

    # --- ROS inputs and periodic broadcast -------------------------------------------

    def _init_state(self):
        """Create the mutable state touched by the callbacks (all on the executor)."""
        self._scan_msg = None
        self._scan_rx_s = None       # ROS time at which the last scan arrived
        self._scan_sent_rx_s = None  # arrival time of the last scan already broadcast
        self._map_msg = None
        self._map_dirty = False
        self._map_signature = None
        self._map_version = 0
        self._plan_points = None
        self._battery_pct = None
        self._last_log_s = {}
        # Navigation goal tracking. HTTP threads and executor callbacks both touch it:
        # every access goes through _nav_lock. _nav_token identifies the current goal so
        # that late callbacks of an old goal are ignored.
        self._nav_lock = threading.Lock()
        self._nav_token = None          # None = no goal in flight
        self._nav_handle = None         # ClientGoalHandle once Nav2 accepted the goal
        self._nav_cancel_requested = False
        self._nav_deadline_s = None     # max ROS time to get an answer from Nav2
        self._nav_server_lost_s = None  # ROS time since the action server is missing
        self._nav_feedback_sent_s = None
        self._nav_cancel_retry_s = None  # earliest ROS time of the next cancel retry
        # Accepted goals that left the "current" slot WITHOUT a Nav2 result (token ->
        # {'handle', 'retry_s'}). Fail safe: they may still be driving the robot, so they
        # are cancelled again and again until Nav2 confirms their end, and no new goal is
        # accepted meanwhile (the node stays "busy", never "free").
        self._nav_orphans = {}
        # Pending "cancel all" (goal_id zeros + stamp zero = every goal of the server):
        # safety net for a goal Nav2 may have accepted whose acceptance never reached us, so
        # there is no handle to cancel. None = nothing pending; else {'retry_s', 'attempts'}.
        # 'stamp_ns' is the ROS time of the DECISION (see _request_cancel_all); retries reuse it.
        self._cancel_all = None
        self._cancel_all_warn_s = float('-inf')  # last 'service not ready' warning
        # Floods (a client can send stops at the WebSocket rate) become one counted line per
        # LOG_SUMMARY_PERIOD_S; the first event is logged at once. Behavior is unchanged.
        self._log_summaries = {
            'stop': (EventSummary('stop(s)', LOG_SUMMARY_PERIOD_S), 'warning'),
            'cancel': (EventSummary('navigation cancel request(s)', LOG_SUMMARY_PERIOD_S),
                       'info'),
            'zero': (EventSummary('teleop zero burst(s) published', LOG_SUMMARY_PERIOD_S),
                     'info')}
        self._stop_context = threading.local()  # per-thread: True while stop() runs
        self._last_cancel_all_armed_s = None  # monotonic time of the last armed cancel-all
        self._cancel_all_client = None  # public CancelGoal service client (see _init_ros_io)
        # Teleop: one lock serializes gate decisions AND the publishing that follows, so a
        # stop can never be overtaken by a command that was accepted just before it.
        # Lock order: _teleop_lock -> _nav_lock (never the reverse).
        self._teleop_lock = threading.Lock()
        self._cmd_vel_nav_pub = None    # created in _init_ros_io

    def _init_ros_io(self):
        """Create the TF buffer, subscriptions (read-only) and timers."""
        self._tf_buffer = Buffer(cache_time=Duration(seconds=TF_BUFFER_S))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        # Same QoS as nav2 map_server / slam_toolbox, else the late-joining bridge never
        # receives the latched map.
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, map_qos)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Path, 'plan', self._on_plan, 1)
        self.create_subscription(
            BatteryState, 'battery_state', self._on_battery, qos_profile_sensor_data)
        self.create_timer(self._pose_period_s, self._broadcast_pose)
        self.create_timer(self._scan_period_s, self._broadcast_scan)
        self.create_timer(self._status_period_s, self._broadcast_status)
        self.create_timer(MAP_PERIOD_S, self._broadcast_map)
        # Relative names: resolved under the node namespace (/r2).
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Own PUBLIC client of the action's cancel service (rclpy has no public cancel
        # without a goal handle). Relative name -> /r2/navigate_to_pose/_action/cancel_goal.
        self._cancel_all_client = self.create_client(
            CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        # SAFETY: the only direct-velocity path of the web. Relative name -> /r2/cmd_vel_nav
        # (input of velocity_smoother), geometry_msgs/Twist (NOT TwistStamped). The web NEVER
        # publishes on `cmd_vel` (only velocity_smoother does). Second allowed publisher
        # next to `initialpose`; commands come only from TeleopGate decisions.
        self._cmd_vel_nav_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.create_timer(NAV_WATCHDOG_PERIOD_S, self._nav_watchdog)
        # Own callback group: a slow callback of the default group (map encoding...) must not
        # delay the teleop watchdog.
        # The timer uses a STEADY (monotonic) clock on purpose: with use_sim_time=false the
        # default ROS clock follows the system clock, and a backwards jump (NTP step) would
        # leave the 20 ms timer silent for the size of the jump, so the last command could
        # stay active. The gate's clock is monotonic for the same reason.
        self.create_timer(TELEOP_WATCHDOG_PERIOD_S, self._teleop_watchdog,
                          callback_group=MutuallyExclusiveCallbackGroup(),
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        # No lock needed: the executor is not spinning yet, nothing else touches the state.
        self._publish_nav(NAV_IDLE, None)
        self._init_status_io()
        self._init_map_clients()

    def _init_status_io(self):
        """Odometry age and lifecycle polling for the `status` message (read-only)."""
        # Relative name -> /<ns>/odom (Create 3). Only the arrival time is kept.
        self.create_subscription(Odometry, 'odom', self._on_odom, qos_profile_sensor_data)
        # Own callback group: slow default-group callbacks (map encoding) never delay the
        # polling, and the polling (tiny callbacks) never touches teleop or goals.
        group = MutuallyExclusiveCallbackGroup()
        self._lifecycle_lock = threading.Lock()  # guards the three dicts below only
        self._lifecycle_states = {name: STATE_UNKNOWN for name in self._status_nodes}
        self._lifecycle_seq = {name: 0 for name in self._status_nodes}
        self._lifecycle_pending = {}   # node name -> Future of the request in flight
        # Relative names -> /<ns>/<node>/get_state (every lifecycle node serves it).
        self._lifecycle_clients = {
            name: self.create_client(GetState, f'{name}/get_state', callback_group=group)
            for name in self._status_nodes}
        self.create_timer(self._lifecycle_period_s, self._poll_lifecycle,
                          callback_group=group)

    def _init_map_clients(self):
        """Public clients of the slam_toolbox map services (used from HTTP threads)."""
        # Reentrant group of its own: the answers are processed by any free executor thread,
        # independently of the default group (a busy map callback cannot delay them).
        group = ReentrantCallbackGroup()
        self._save_map_client = self.create_client(
            SaveMap, SAVE_MAP_SERVICE, callback_group=group)
        self._serialize_client = self.create_client(
            SerializePoseGraph, SERIALIZE_MAP_SERVICE, callback_group=group)

    def _warn_throttled(self, key, text):
        """Log a warning at most once per LOG_THROTTLE_S for each key (no sensitive data)."""
        now = self._now_s()
        last = self._last_log_s.get(key)
        if last is None or now - last >= LOG_THROTTLE_S:
            self._last_log_s[key] = now
            self.get_logger().warning(text)

    def _publish(self, kind, **fields):
        """Serialize and fan out one message; invalid numbers are dropped, not sent."""
        try:
            self.hub.publish(kind, json_message(kind, **fields))
        except ValueError as err:
            self._warn_throttled(f'publish-{kind}', f'Dropped {kind} message: {err}')

    def _lookup(self, target, source):
        """Fresh (x, y, yaw, stamp_s) of `source` in `target` from TF, or None on failure."""
        try:
            transform = self._tf_buffer.lookup_transform(target, source, Time())
            tr = transform.transform.translation
            rot = transform.transform.rotation
            stamp_s = Time.from_msg(transform.header.stamp).nanoseconds / NS_PER_S
            yaw = yaw_from_quaternion(rot.x, rot.y, rot.z, rot.w)
            values = (tr.x, tr.y, yaw)
        except (TransformException, ValueError) as err:
            self._warn_throttled(f'tf-{target}-{source}', f'No TF {target}->{source}: {err}')
            return None
        age_s = self._now_s() - stamp_s
        if not all(math.isfinite(v) for v in values) or age_s > TF_MAX_AGE_S:
            self._warn_throttled(f'tf-{target}-{source}',
                                 f'TF {target}->{source} invalid or stale ({age_s:.2f} s)')
            return None
        return values[0], values[1], values[2], min(stamp_s, self._now_s())

    def _on_scan(self, msg):
        """Keep the latest scan; processing happens in the 5 Hz timer."""
        self._scan_msg = msg
        self._scan_rx_s = self._now_s()

    def _on_odom(self, _msg):
        """Keep only the arrival time of the odometry (its content is not used)."""
        self._odom_rx_s = self._now_s()

    def _poll_lifecycle(self):
        """
        Ask every watched node for its lifecycle state without blocking.

        A request still pending from the previous tick is dropped (the state becomes
        'unknown'), so nothing accumulates. Answers arrive in `_on_lifecycle`.
        """
        for name, client in self._lifecycle_clients.items():
            with self._lifecycle_lock:
                stale = self._lifecycle_pending.pop(name, None)
                self._lifecycle_seq[name] += 1
                seq = self._lifecycle_seq[name]
                if stale is not None:
                    self._lifecycle_states[name] = STATE_UNKNOWN
            if stale is not None:
                client.remove_pending_request(stale)
            try:
                if not client.service_is_ready():
                    self._set_lifecycle_state(name, seq, STATE_UNAVAILABLE)
                    continue
                future = client.call_async(GetState.Request())
            except Exception as err:  # noqa: B902 - polling must never stop the timer
                self._warn_throttled(f'lifecycle-{name}', f'Lifecycle query failed: {err}')
                self._set_lifecycle_state(name, seq, STATE_UNKNOWN)
                continue
            with self._lifecycle_lock:
                if self._lifecycle_seq[name] == seq:
                    self._lifecycle_pending[name] = future
            future.add_done_callback(functools.partial(self._on_lifecycle, name, seq))

    def _set_lifecycle_state(self, name, seq, state):
        """Store a node state unless a newer request replaced the one with number `seq`."""
        with self._lifecycle_lock:
            if self._lifecycle_seq[name] == seq:
                self._lifecycle_states[name] = state
                self._lifecycle_pending.pop(name, None)

    def _on_lifecycle(self, name, seq, future):
        """Store the lifecycle label of `name` from a GetState answer (invalid -> unknown)."""
        try:
            label = str(future.result().current_state.label).strip().lower()
        except Exception:  # noqa: B902 - a failed call is just an unknown state
            label = STATE_UNKNOWN
        self._set_lifecycle_state(
            name, seq, label if label in LIFECYCLE_LABELS else STATE_UNKNOWN)

    def _lifecycle_snapshot(self):
        """Return {node: state} (a copy, safe to serialize)."""
        with self._lifecycle_lock:
            return dict(self._lifecycle_states)

    def _on_battery(self, msg):
        """Store the battery charge as a percentage; NaN or out of range becomes unknown."""
        fraction = msg.percentage
        valid = math.isfinite(fraction) and 0.0 <= fraction <= 1.0
        self._battery_pct = round(fraction * 100.0, 1) if valid else None

    def _on_plan(self, msg):
        """Simplify the plan and broadcast it only when it changed."""
        if msg.header.frame_id.lstrip('/') != MAP_FRAME:
            self._warn_throttled('plan-frame', f'Plan in frame {msg.header.frame_id!r}, '
                                 f'expected {MAP_FRAME!r}: ignored')
            return
        try:
            points = simplify_path(
                [(p.pose.position.x, p.pose.position.y) for p in msg.poses],
                self._plan_max_points) if msg.poses else []
        except ValueError as err:
            self._warn_throttled('plan-invalid', f'Invalid plan ignored: {err}')
            return
        if points == self._plan_points:
            return
        self._plan_points = points
        self._publish('plan', points=points)
        self.store.update(plan_len=len(points))

    def _on_map(self, msg):
        """Keep the latest map; encoding is done by the 1 Hz timer (max 1 map per second)."""
        self._map_msg = msg
        self._map_dirty = True
        self._store_grid(msg)

    def _store_grid(self, msg):
        """Keep the map with its cells for goal validation; invalid maps clear it."""
        info = msg.info
        values = (info.resolution, info.origin.position.x, info.origin.position.y)
        if (msg.header.frame_id.lstrip('/') != MAP_FRAME or info.width <= 0
                or info.height <= 0 or not all(math.isfinite(v) for v in values)
                or info.resolution <= 0.0 or len(msg.data) != info.width * info.height):
            self.store.set_grid(None)  # never validate a goal against a bad map
            return
        self.store.set_grid({
            'width': info.width, 'height': info.height, 'resolution': info.resolution,
            'origin_x': info.origin.position.x, 'origin_y': info.origin.position.y,
            'data': msg.data})

    # --- Map saving (MapPort) --------------------------------------------------------

    def _log_map_event(self, level, text):
        """Log an event of the map saver ('info' | 'warning' | 'error'; no sensitive data)."""
        log = self.get_logger()
        {'warning': log.warning, 'error': log.error}.get(level, log.info)(text)

    def ready(self):
        """Return True if the slam_toolbox save_map service is available (non-blocking)."""
        return self._save_map_client.service_is_ready()

    def save_map(self, base_path, timeout_s):
        """Call slam_toolbox/save_map for `base_path`; True if it reported success."""
        request = SaveMap.Request()
        request.name = String(data=base_path)
        response = self._call_service(self._save_map_client, request, timeout_s)
        return response is not None and response.result == SaveMap.Response.RESULT_SUCCESS

    def serialize_posegraph(self, base_path, timeout_s):
        """Call slam_toolbox/serialize_map for `base_path`; True if it reported success."""
        request = SerializePoseGraph.Request()
        request.filename = base_path
        response = self._call_service(self._serialize_client, request, timeout_s)
        return (response is not None
                and response.result == SerializePoseGraph.Response.RESULT_SUCCESS)

    @staticmethod
    def _call_service(client, request, timeout_s):
        """
        Send `request` and wait at most `timeout_s` for the answer; None if it failed.

        Only for HTTP (thread pool) threads: it blocks the caller, never an executor
        thread or the event loop. The answer is processed by the executor, which sets the
        event. Raise SlamUnavailableError (service not ready) or SlamTimeoutError.
        """
        if not client.service_is_ready():
            raise SlamUnavailableError('Service not ready')
        done = threading.Event()
        try:
            future = client.call_async(request)
        except Exception as err:  # noqa: B902 - any rclpy failure means "not available"
            raise SlamUnavailableError(f'Request could not be sent: {err}') from err
        future.add_done_callback(lambda _future: done.set())
        # Check done() too: the callback may have been added after completion.
        if not future.done() and not done.wait(timeout_s):
            client.remove_pending_request(future)  # frees the pending entry; late answer dropped
            raise SlamTimeoutError('No answer in time')
        if future.exception() is not None:
            return None
        return future.result()

    # --- Navigation (NavPort) --------------------------------------------------------

    def _publish_nav(self, state, distance_m):
        """
        Publish the `nav` message and keep it in the store; callers hold `_nav_lock`.

        Publishing under `_nav_lock` serializes nav messages, so a late message of an old
        goal can never overwrite a newer one. Lock order: `_nav_lock` -> hub lock / store
        lock. Neither the hub nor the store ever takes `_nav_lock` (no cycle).
        """
        if distance_m is not None and not math.isfinite(distance_m):
            distance_m = None
        distance_m = None if distance_m is None else round(distance_m, 2)
        self._publish('nav', state=state, distance_remaining=distance_m)
        self.store.update(nav={'state': state, 'distance_remaining': distance_m})

    def _make_goal(self, x_m, y_m, yaw_rad):
        """Build a NavigateToPose goal in frame `map`."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x_m
        goal.pose.pose.position.y = y_m
        (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = quaternion_from_yaw(
            yaw_rad)
        return goal

    def send_goal(self, x_m, y_m, yaw_rad):
        """Send the goal without blocking; reject it while another one is not finished."""
        if not self._nav_client.server_is_ready():  # non-blocking check
            raise NavUnavailableError('Nav2 action server navigate_to_pose is not available')
        goal = self._make_goal(x_m, y_m, yaw_rad)
        goal_id = uuid.uuid4().hex
        with self._nav_lock:
            if self._nav_token is not None:
                raise NavBusyError('A goal is already active: cancel it first')
            if self._nav_orphans:
                # Safest choice: a previous goal may still be moving the robot because Nav2
                # has not confirmed its end. Stay busy until the cancel is confirmed.
                raise NavBusyError(
                    'A previous goal has not been confirmed finished by Nav2; cancel and wait')
            self._nav_token = goal_id
            self._nav_handle = None
            self._nav_cancel_requested = False
            self._nav_deadline_s = self._now_s() + NAV_RESPONSE_TIMEOUT_S
            self._nav_server_lost_s = None
            self._nav_feedback_sent_s = None
            self._nav_cancel_retry_s = None
            # A pending cancel-all must never hit the goal that is being sent now.
            self._cancel_all = None
            self._publish_nav(NAV_ACTIVE, None)
        # Goal vs teleop: the goal wins and the teleop is stopped FIRST (zero Twist, lease
        # released) instead of answering 409, so the operator's explicit request is honoured
        # and two controllers never drive together. The goal is already tracked above, so any
        # teleop message arriving from now on is rejected as "navigation busy".
        # The teleop is therefore already stopped if send_goal_async fails below (safe side:
        # the error path never leaves the robot driven by the teleop of an unsent goal).
        self._teleop_stop_if_owned()
        try:
            future = self._nav_client.send_goal_async(
                goal, feedback_callback=lambda fb: self._on_nav_feedback(goal_id, fb))
            future.add_done_callback(lambda fut: self._on_goal_response(goal_id, fut))
        except Exception as err:  # noqa: B902 - any rclpy failure must release the goal
            self._finish_goal(goal_id, NAV_FAILED, f'send_goal_async failed: {err}')
            raise NavUnavailableError('Could not send the goal to Nav2') from err
        self.get_logger().info(f'Navigation goal {goal_id} sent')
        return goal_id

    def cancel(self):
        """
        Cancel the current goal and every unconfirmed (orphan) goal; idempotent.

        The final state arrives with the action result. Calling it again extends the
        Nav2 answer deadline of the current goal by NAV_RESPONSE_TIMEOUT_S. If nothing is
        tracked (no current goal, no orphan) a "cancel all" is sent instead: it cancels EVERY
        goal of navigate_to_pose, also those of other clients (same as the manual
        `ros2 service call` fallback), because the goal may be running without a handle.
        When called from stop() (repeated stops) that cancel-all is not armed again if
        another one was armed less than STOP_CANCEL_ALL_MIN_PERIOD_S ago (it is still
        pending and retried); tracked goals and orphans are always canceled.
        """
        now = self._now_s()
        send_all = False
        stamp_ns = 0
        with self._nav_lock:
            token, handle = self._nav_token, self._nav_handle
            targets = []
            if token is None and not self._nav_orphans and not self._cancel_all_coalesced():
                # Decision instant, read under the lock: a goal sent later gets a later stamp.
                stamp_ns = self._now_ns()
                self._cancel_all = {'retry_s': now, 'attempts': 0, 'stamp_ns': stamp_ns}
                send_all = True
            if token is not None:
                self._nav_cancel_requested = True
                self._nav_deadline_s = now + NAV_RESPONSE_TIMEOUT_S
                if handle is not None:  # else the cancel is applied when Nav2 accepts it
                    self._nav_cancel_retry_s = now + NAV_CANCEL_RETRY_S
                    targets.append((token, handle))
            for orphan_token, entry in self._nav_orphans.items():
                entry['retry_s'] = now + NAV_CANCEL_RETRY_S
                targets.append((orphan_token, entry['handle']))
        for target_token, target_handle in targets:
            self._request_cancel(target_token, target_handle)
        if send_all and self._request_cancel_all(stamp_ns):
            self._count_cancel_all_sent(now)
        if token is not None or targets or send_all:
            self._log_summarized('cancel')

    def _cancel_all_coalesced(self):
        """Return True if a stop-armed cancel-all is skipped as too close to the last one."""
        coalesce = getattr(self._stop_context, 'active', False)  # set only by stop()
        mono_s = time.monotonic()  # monotonic: immune to clock jumps
        last_s = self._last_cancel_all_armed_s
        if coalesce and last_s is not None and 0.0 <= mono_s - last_s < \
                STOP_CANCEL_ALL_MIN_PERIOD_S:
            return True
        self._last_cancel_all_armed_s = mono_s
        return False

    def _request_cancel(self, token, handle):
        """
        Ask Nav2 to cancel `handle`; never ends the goal.

        A failure to ask is only logged: the goal stays tracked (current or orphan) and the
        watchdog retries. The goal ends only when Nav2 reports a result or confirms (by the
        cancel return code) that the goal is terminated or unknown.
        """
        try:
            future = handle.cancel_goal_async()
            future.add_done_callback(lambda fut: self._on_cancel_response(token, fut))
        except Exception as err:  # noqa: B902
            self.get_logger().error(
                f'cancel_goal_async failed for goal {token}: {err}; will retry')

    def _count_cancel_all_sent(self, now):
        """
        Count one cancel-all really sent and delay the next retry.

        Only requests that left the node consume an attempt: while the service is down the
        cancel-all stays pending and is retried at every tick without using attempts.
        """
        with self._nav_lock:
            pending = self._cancel_all
            if pending is not None:
                pending['attempts'] += 1
                pending['retry_s'] = now + NAV_CANCEL_RETRY_S

    def _request_cancel_all(self, stamp_ns):
        """
        Ask Nav2 to cancel the goals accepted up to `stamp_ns` (non-blocking); never raises.

        Returns True if an attempt was made (request issued, or the call itself failed) and
        False if nothing was tried because the service is missing or not ready: the caller
        counts an attempt only on True.

        CancelGoal request with goal_id all zeros and stamp = decision instant: the server
        cancels only goals accepted with stamp <= that instant (rcl_action cancel semantics),
        so a goal sent after the decision is never hit, also by a late retry. Retries reuse
        the original stamp. Assumes bridge and Nav2 share the ROS clock (same use_sim_time).
        It may still cancel goals of other clients accepted before the decision.
        A failure to ask is only logged and the watchdog retries (bounded by
        MAX_CANCEL_ALL_ATTEMPTS).
        """
        try:
            client = self._cancel_all_client
            if client is None or not client.service_is_ready():
                now = self._now_s()
                if now - self._cancel_all_warn_s >= CANCEL_ALL_WARN_PERIOD_S:
                    self._cancel_all_warn_s = now
                    if client is None:
                        self.get_logger().error(
                            'Cancel-all CANNOT be sent: cancel service client was not '
                            'created; STOP THE ROBOT MANUALLY if a goal may be running')
                    else:
                        self.get_logger().warning(
                            'Cancel-all still pending: cancel service not ready; retrying')
                return False
            request = CancelGoal.Request()  # goal_id stays all zeros
            request.goal_info.stamp = Time(nanoseconds=max(1, stamp_ns)).to_msg()
            future = client.call_async(request)
            future.add_done_callback(
                lambda fut, stamp=stamp_ns: self._on_cancel_all_response(stamp, fut))
            return True
        except Exception as err:  # noqa: B902
            self.get_logger().error(f'Cancel-all request failed: {err}; will retry')
            return True  # a real attempt that failed: it counts, so the retries stay bounded

    def _on_cancel_all_response(self, stamp_ns, future):
        """
        Handle the answer to the cancel-all sent for `stamp_ns`; stale answers are ignored.

        Only an answer whose stamp matches the pending cancel-all may clear it: a late answer
        to an older cancel-all must not cancel the retries of a newer one. A rejection or a
        failed call leaves the pending entry untouched, so the watchdog retries it.
        """
        with self._nav_lock:
            pending = self._cancel_all
            current = pending is not None and pending['stamp_ns'] == stamp_ns
        if not current:
            self.get_logger().debug(
                f'Ignoring stale cancel-all answer (stamp_ns={stamp_ns})')
            return
        try:
            result = future.result()
            code = result.return_code
            canceling = len(getattr(result, 'goals_canceling', ()))  # for the log only
        except Exception as err:  # noqa: B902
            self.get_logger().warning(f'No cancel-all answer: {err}; will retry')
            return
        if code == CANCEL_ERROR_REJECTED or code not in (
                CANCEL_ERROR_NONE, CANCEL_ERROR_UNKNOWN_GOAL_ID, CANCEL_ERROR_GOAL_TERMINATED):
            self.get_logger().warning(f'Cancel-all rejected (code {code}); will retry')
            return
        with self._nav_lock:
            pending = self._cancel_all
            if pending is None or pending['stamp_ns'] != stamp_ns:
                self.get_logger().debug(
                    f'Ignoring stale cancel-all answer (stamp_ns={stamp_ns})')
                return
            self._cancel_all = None  # answered: goals canceled or none was running
        self.get_logger().info(
            f'Cancel-all answered by Nav2 (code {code}, goals_canceling={canceling})')

    def _on_cancel_response(self, token, future):
        """Check the return code of a cancel request; anything but success is retried."""
        try:
            code = future.result().return_code
        except Exception as err:  # noqa: B902
            self.get_logger().warning(f'No cancel answer for goal {token}: {err}; will retry')
            return
        if code == CANCEL_ERROR_NONE:
            return  # accepted: the action result will confirm the end
        if code in (CANCEL_ERROR_GOAL_TERMINATED, CANCEL_ERROR_UNKNOWN_GOAL_ID):
            with self._nav_lock:
                confirmed = self._nav_orphans.pop(token, None) is not None
            if confirmed:  # Nav2 says the goal is over (or unknown): nothing left to stop
                self.get_logger().info(f'Orphan goal {token} confirmed finished (code {code})')
            return  # a current goal ends with its result or via the watchdog
        # ERROR_REJECTED or an unknown code: the goal may still run; the watchdog retries.
        self.get_logger().warning(f'Cancel of goal {token} rejected (code {code}); will retry')

    def publish_initial_pose(self, x_m, y_m, yaw_rad):
        """
        Publish the pose estimate on `initialpose` (the only pose publisher here).

        Refused with NavBusyError (409 goal_active) while Nav2 may be driving (`_nav_busy`)
        or a teleop controller holds a fresh lease.
        """
        gate = self._teleop_gate
        with self._teleop_lock:  # lock order: _teleop_lock -> _nav_lock
            busy = self._nav_busy() or (gate is not None and gate.active)
        if busy:
            raise NavBusyError('Robot is moving or navigating: stop it before setting the pose')
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x_m
        msg.pose.pose.position.y = y_m
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = quaternion_from_yaw(
            yaw_rad)
        msg.pose.covariance[0] = INITIAL_COVARIANCE_XY
        msg.pose.covariance[7] = INITIAL_COVARIANCE_XY
        msg.pose.covariance[35] = INITIAL_COVARIANCE_YAW
        self._initial_pose_pub.publish(msg)
        self.get_logger().info('Initial pose published from the web')

    # --- Teleop (NavPort) ------------------------------------------------------------
    # All decisions come from TeleopGate; these methods only execute them. No message
    # content is logged.

    def _nav_busy(self):
        """
        Return True if Nav2 may be driving: goal, unconfirmed (orphan) goal or pending cancel-all.

        A pending cancel-all only counts while the navigate_to_pose action server is up:
        with Nav2 absent (e.g. mode mapping) nothing can be driving, and a stop must not
        block the teleop for good. Goals and orphans always count.
        """
        with self._nav_lock:
            if self._nav_token is not None or self._nav_orphans:
                return True
            cancel_all_pending = self._cancel_all is not None
        return cancel_all_pending and self._nav_client.server_is_ready()

    def _publish_cmd_vel_nav(self, vx_mps, wz_radps):
        """Publish one Twist (only linear.x and angular.z) on cmd_vel_nav; never raises."""
        try:
            msg = Twist()
            msg.linear.x = float(vx_mps)
            msg.angular.z = float(wz_radps)
            self._cmd_vel_nav_pub.publish(msg)
        except Exception as err:  # noqa: B902 - keep trying the remaining zeros
            self.get_logger().error(f'Could not publish on cmd_vel_nav: {err}')

    def _log_summarized(self, key):
        """Count an event of a flood-prone kind and log the summary line if one is due."""
        summary, level = self._log_summaries[key]
        text = summary.note()
        if text is not None:
            getattr(self.get_logger(), level)(text)

    def _flush_log_summaries(self):
        """Log the counts that are still pending once their period elapsed (watchdog tick)."""
        for summary, level in self._log_summaries.values():
            text = summary.poll()
            if text is not None:
                getattr(self.get_logger(), level)(text)

    def _teleop_execute(self, action):
        """Execute a TeleopAction; `_teleop_lock` must be held. Zeros go first."""
        for _ in range(action.zeros):
            self._publish_cmd_vel_nav(0.0, 0.0)
        if action.zeros:
            self._log_summarized('zero')
        if action.kind == TELEOP_PUBLISH and action.twist is not None:
            self._publish_cmd_vel_nav(*action.twist)

    def teleop(self, client_id, message):
        """Handle a teleop message; return None or an (code, detail) error for the client."""
        gate = self._teleop_gate
        if gate is None:
            return ('teleop_rejected', 'Teleop is not configured')
        try:
            with self._teleop_lock:
                # Read busy INSIDE the lock: send_goal tracks the goal first and stops the
                # teleop later, so either we see the goal or the stop runs after us.
                action = gate.on_teleop(client_id, message, self._nav_busy())
                self._teleop_execute(action)
        except Exception as err:  # noqa: B902 - fail safe: stop and tell the client
            self.get_logger().error(f'Teleop failed, stopping: {err}')
            self._stop_teleop_only()
            return ('teleop_rejected', 'Internal teleop error')
        if action.error is not None:
            self._warn_throttled('teleop-reject', f'Teleop rejected ({action.error[0]})')
        return action.error

    def _stop_teleop_only(self):
        """Zero Twist burst and release the lease; works without a gate. Never raises."""
        try:
            with self._teleop_lock:
                gate = self._teleop_gate
                action = gate.stop() if gate is not None else TeleopAction(
                    TELEOP_ZERO, None, TELEOP_ZERO_BURST, None)
                self._teleop_execute(action)
        except Exception as err:  # noqa: B902
            self.get_logger().error(f'Could not stop the teleop: {err}')

    def _teleop_stop_if_owned(self):
        """Stop the teleop only if a controller holds the lease (used before a goal)."""
        gate = self._teleop_gate
        if gate is None:
            return
        try:
            with self._teleop_lock:
                if gate.owner_age_s is not None:
                    self._teleop_execute(gate.stop())
        except Exception as err:  # noqa: B902
            self.get_logger().error(f'Could not stop the teleop: {err}')

    def teleop_disconnect(self, client_id):
        """Zero Twist at once if the WebSocket of the controlling client closed."""
        gate = self._teleop_gate
        if gate is None:
            return
        try:
            with self._teleop_lock:
                self._teleop_execute(gate.client_disconnected(client_id))
        except Exception as err:  # noqa: B902
            self.get_logger().error(f'Teleop disconnect handling failed: {err}')
            self._stop_teleop_only()

    def stop(self):
        """
        Stop: zero Twist burst first, then cancel the goals; needs no Nav2, map or mode.

        The zero Twist is always issued before (and independently of) the cancel, which
        never raises by design; a failure there is only logged.
        """
        self._log_summarized('stop')
        self._stop_teleop_only()
        self._stop_context.active = True
        try:
            self.cancel()
        except Exception as err:  # noqa: B902
            self.get_logger().error(f'Cancel during stop failed: {err}')
        finally:
            self._stop_context.active = False

    def _teleop_watchdog(self):
        """Periodic: zero Twist burst (once) when the controlling client went silent."""
        self._flush_log_summaries()
        gate = self._teleop_gate
        if gate is None:
            return
        try:
            with self._teleop_lock:
                self._teleop_execute(gate.watchdog_tick())
        except Exception as err:  # noqa: B902
            self.get_logger().error(f'Teleop watchdog failed: {err}')
            self._stop_teleop_only()

    def _clear_current_locked(self):
        """Empty the current-goal slot; `_nav_lock` must be held."""
        self._nav_token = None
        self._nav_handle = None
        self._nav_cancel_requested = False
        self._nav_deadline_s = None
        self._nav_server_lost_s = None
        self._nav_cancel_retry_s = None

    def _finish_goal(self, token, state, reason):
        """End the goal `token` (ignored if it is not the current one) and publish `state`."""
        with self._nav_lock:
            if self._nav_token != token:
                return
            self._clear_current_locked()
            self._publish_nav(state, None)
        self.get_logger().info(f'Navigation goal {token} ended: {state} ({reason})')

    def _add_orphan_locked(self, token, handle):
        """Track an accepted goal whose end is unconfirmed; `_nav_lock` must be held."""
        if len(self._nav_orphans) >= MAX_ORPHAN_GOALS and token not in self._nav_orphans:
            oldest = next(iter(self._nav_orphans))
            del self._nav_orphans[oldest]  # bounded memory; practically unreachable
            self.get_logger().error(f'Orphan goal table full: dropped tracking of {oldest}')
        self._nav_orphans[token] = {'handle': handle, 'retry_s': self._now_s()}

    def _detach_current_locked(self, reason):
        """
        Report the current accepted goal as failed WITHOUT releasing it.

        It becomes an orphan that keeps being cancelled; `_nav_lock` must be held.
        """
        token = self._nav_token
        self._add_orphan_locked(token, self._nav_handle)
        self._clear_current_locked()
        self._publish_nav(NAV_FAILED, None)
        self.get_logger().error(
            f'Navigation goal {token} not confirmed finished ({reason}): still being cancelled')

    def _on_goal_response(self, token, future):
        """Nav2 accepted or rejected the goal."""
        try:
            handle = future.result()
        except Exception as err:  # noqa: B902
            self._finish_goal(token, NAV_FAILED, f'no response: {err}')
            return
        if handle is None or not handle.accepted:
            self._finish_goal(token, NAV_FAILED, 'rejected by Nav2')
            return
        now = self._now_s()
        with self._nav_lock:
            stale = self._nav_token != token
            if stale:  # released meanwhile (timeout): Nav2 runs it, so keep and cancel it
                self._add_orphan_locked(token, handle)
                cancel_now = True
            else:
                self._nav_handle = handle
                cancel_now = self._nav_cancel_requested
                if cancel_now:
                    self._nav_cancel_retry_s = now + NAV_CANCEL_RETRY_S
                else:
                    self._nav_deadline_s = None
        try:
            handle.get_result_async().add_done_callback(
                lambda fut: self._on_goal_result(token, fut))
        except Exception as err:  # noqa: B902
            # Without a result channel the end can only be known through the cancel.
            self.get_logger().error(f'get_result_async failed for goal {token}: {err}')
            with self._nav_lock:
                if self._nav_token == token:
                    self._detach_current_locked('no result channel')
            cancel_now = True
        if cancel_now:
            self._request_cancel(token, handle)

    def _on_goal_result(self, token, future):
        """Map the final action status to a nav state; an orphan result closes the orphan."""
        try:
            status = future.result().status
        except Exception as err:  # noqa: B902
            # The end of an accepted goal is unknown: keep it tracked and cancel it.
            with self._nav_lock:
                if self._nav_token == token:
                    self._detach_current_locked(f'no result: {err}')
            return
        with self._nav_lock:
            was_orphan = self._nav_orphans.pop(token, None) is not None
        if was_orphan:
            self.get_logger().info(f'Orphan goal {token} ended with status {status}')
            return
        self._finish_goal(token, _FINAL_STATE.get(status, NAV_FAILED), f'status {status}')

    def _on_nav_feedback(self, token, feedback_msg):
        """Publish distance_remaining at most at 2 Hz; cancel feedback of an orphan goal."""
        now = self._now_s()
        orphan_handle = None
        with self._nav_lock:
            if self._nav_token != token:
                entry = self._nav_orphans.get(token)
                if entry is not None and entry['retry_s'] <= now:
                    entry['retry_s'] = now + NAV_CANCEL_RETRY_S
                    orphan_handle = entry['handle']
            else:
                last = self._nav_feedback_sent_s
                if last is None or now - last >= NAV_FEEDBACK_PERIOD_S:
                    self._nav_feedback_sent_s = now
                    self._publish_nav(
                        NAV_ACTIVE, feedback_msg.feedback.distance_remaining)
        if orphan_handle is not None:  # an unconfirmed goal is still running: stop it
            self._request_cancel(token, orphan_handle)

    def _nav_watchdog(self):
        """
        Release unaccepted goals on timeout and keep cancelling accepted unconfirmed ones.

        A goal Nav2 has not accepted yet is failed after NAV_RESPONSE_TIMEOUT_S or when the
        action server is lost for NAV_SERVER_LOST_TIMEOUT_S (a late acceptance is cancelled).
        An ACCEPTED goal is never released without a Nav2 result: it is reported as failed
        but kept as an orphan and its cancel is retried every NAV_CANCEL_RETRY_S.
        """
        now = self._now_s()
        targets = []
        send_all = False
        stamp_ns = 0
        with self._nav_lock:
            token = self._nav_token
            if token is not None:
                expired = self._nav_deadline_s is not None and now > self._nav_deadline_s
                if self._nav_client.server_is_ready():
                    self._nav_server_lost_s = None
                    lost = False
                else:
                    if self._nav_server_lost_s is None:
                        self._nav_server_lost_s = now
                    lost = now - self._nav_server_lost_s > NAV_SERVER_LOST_TIMEOUT_S
                reason = ('Nav2 did not answer in time' if expired
                          else 'Nav2 action server lost')
                if self._nav_handle is None:
                    if expired or lost:  # never accepted: safe to release
                        self._clear_current_locked()
                        self._publish_nav(NAV_FAILED, None)
                        self.get_logger().info(
                            f'Navigation goal {token} ended: failed ({reason})')
                        # Nav2 may have accepted it without us hearing: no handle to cancel,
                        # so arm a cancel-all (sent below, retried by later ticks).
                        self._cancel_all = {
                            'retry_s': now, 'attempts': 0, 'stamp_ns': self._now_ns()}
                elif expired or lost:
                    self._detach_current_locked(reason)
                elif (self._nav_cancel_requested and self._nav_cancel_retry_s is not None
                      and now >= self._nav_cancel_retry_s):
                    self._nav_cancel_retry_s = now + NAV_CANCEL_RETRY_S
                    targets.append((token, self._nav_handle))
            for orphan_token, entry in self._nav_orphans.items():
                if entry['retry_s'] <= now:
                    entry['retry_s'] = now + NAV_CANCEL_RETRY_S
                    targets.append((orphan_token, entry['handle']))
            pending = self._cancel_all
            if pending is not None:
                if self._nav_token is not None:
                    self._cancel_all = None  # a new goal exists: never cancel it by mistake
                elif pending['attempts'] >= MAX_CANCEL_ALL_ATTEMPTS:
                    self._cancel_all = None
                    self.get_logger().error(
                        'Cancel-all not confirmed by Nav2: giving up; STOP THE ROBOT MANUALLY')
                elif pending['retry_s'] <= now:
                    send_all = True  # attempt counted only if really sent (below)
                    stamp_ns = pending['stamp_ns']  # original decision instant, never renewed
        for target_token, target_handle in targets:
            self._request_cancel(target_token, target_handle)
        if send_all and self._request_cancel_all(stamp_ns):
            self._count_cancel_all_sent(now)

    def _broadcast_pose(self):
        """10 Hz: pose of base_link in map; on any TF problem the pose becomes unknown."""
        found = self._lookup(MAP_FRAME, BASE_FRAME)
        if found is None:
            self.hub.discard('pose')
            self.store.update(pose=None)
            return
        x, y, yaw, stamp_s = found
        self._publish('pose', x=round(x, 4), y=round(y, 4), yaw=round(yaw, 4),
                      stamp=round(stamp_s, 3))
        self.store.update(pose={'x': x, 'y': y, 'yaw': yaw, 'stamp': stamp_s})

    def _broadcast_scan(self):
        """5 Hz: latest scan projected to map with the latest TF; skipped if stale or no TF."""
        msg, rx_s = self._scan_msg, self._scan_rx_s
        if msg is None or rx_s is None:
            return
        # Staleness is checked BEFORE the "already sent" shortcut: if the LiDAR goes
        # silent, the last scan must not stay available to new clients.
        scan_age_s = self._now_s() - rx_s
        if scan_age_s > SCAN_MAX_AGE_S:
            self.hub.discard('scan')
            self.store.update(scan_age=round(max(0.0, scan_age_s), 2))
            return
        if rx_s == self._scan_sent_rx_s:
            return
        laser = self._lookup(MAP_FRAME, msg.header.frame_id.lstrip('/'))
        if laser is None:
            self.hub.discard('scan')
            return
        r_min, r_max = self._scan_min_range_m, self._scan_max_range_m
        if math.isfinite(msg.range_min) and msg.range_min > r_min:
            r_min = msg.range_min
        if math.isfinite(msg.range_max) and 0.0 < msg.range_max < r_max:
            r_max = msg.range_max
        try:
            points = downsample_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                                     self._scan_max_points, r_min, r_max)
            flat = scan_to_map_points(points, laser[:3])
        except ValueError as err:
            self._warn_throttled('scan-invalid', f'Invalid scan ignored: {err}')
            self.hub.discard('scan')
            return
        self._scan_sent_rx_s = rx_s
        # The stamp is that of the TF used for the projection, not the scan header stamp.
        self._publish('scan', stamp=round(laser[3], 3), points=flat)

    def _broadcast_status(self):
        """1 Hz: battery, mode, node states and data ages (null = unknown)."""
        scan_age = None if self._scan_rx_s is None else max(0.0, self._now_s() - self._scan_rx_s)
        scan_age = None if scan_age is None else round(scan_age, 2)
        dock = dict(zip(('x', 'y', 'yaw'), self._dock))  # validated finite in _read_settings
        odom_rx_s = self._odom_rx_s
        odom_age = None if odom_rx_s is None else round(max(0.0, self._now_s() - odom_rx_s), 2)
        self._publish('status', battery_pct=self._battery_pct, mode=self._mode,
                      nodes=self._lifecycle_snapshot(), scan_age=scan_age,
                      odom_age=odom_age, dock=dock)
        self.store.update(mode=self._mode, scan_age=scan_age, battery_pct=self._battery_pct)

    def _broadcast_map(self):
        """At most 1 Hz: validate, and send map_meta + binary only if the map changed."""
        msg = self._map_msg
        if msg is None or not self._map_dirty:
            return
        self._map_dirty = False
        info = msg.info
        values = (info.resolution, info.origin.position.x, info.origin.position.y)
        if (msg.header.frame_id.lstrip('/') != MAP_FRAME or info.width <= 0
                or info.height <= 0 or not all(math.isfinite(v) for v in values)
                or info.resolution <= 0.0 or len(msg.data) != info.width * info.height):
            self._warn_throttled('map-invalid', 'Invalid map ignored (frame, size or origin)')
            return
        meta = {'width': info.width, 'height': info.height, 'resolution': info.resolution,
                'origin_x': info.origin.position.x, 'origin_y': info.origin.position.y}
        data = msg.data
        if not isinstance(data, (array.array, memoryview, bytes, bytearray)):
            try:  # defensive: rclpy delivers array('b'); a list is converted once, in C
                data = array.array('b', data)
            except (TypeError, OverflowError, ValueError):
                self._warn_throttled('map-invalid', 'Invalid map ignored (cell values)')
                return
        digest = hashlib.blake2b(bytes(memoryview(data).cast('B')), digest_size=16)
        signature = (tuple(meta.values()), digest.digest())
        if signature == self._map_signature:
            return
        try:
            compressed = occupancy_to_bytes(data, info.width, info.height)
            version = (self._map_version + 1) % (2 ** 32)
            meta_json = json_message('map_meta', version=version, **meta)
            binary = pack_map_binary(version, compressed)
        except ValueError as err:
            self._warn_throttled('map-encode', f'Map could not be encoded: {err}')
            return
        self._map_signature = signature
        self._map_version = version
        self.hub.publish('map', (meta_json, binary))
        self.store.update(map={'version': version, **meta})

    def start_server(self):
        """Bind the port (fail if busy) and run uvicorn in a daemon thread."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.host, self.port))
            sock.listen(SOCKET_BACKLOG)
        except OSError as err:
            sock.close()
            raise ConfigError(f'Cannot listen on {self.host}:{self.port}: {err}') from err
        # Access log off: later WebSocket URLs carry the token in the query string.
        # access_log=False relies on log_level='warning' too: at 'info' uvicorn would log
        # the WebSocket route including the ?token= query string.
        # Slow-client limits (real uvicorn.Config options): limit_concurrency answers 503 to
        # new connections beyond the bound, timeout_keep_alive closes idle keep-alive
        # connections, h11_max_incomplete_event_size bounds the request head (h11 only).
        # uvicorn has NO timeout for a request head sent slowly (slowloris): residual risk.
        config = uvicorn.Config(self.app, log_level='warning', access_log=False,
                                ws_max_size=self._max_ws_bytes,
                                limit_concurrency=self._max_connections,
                                timeout_keep_alive=UVICORN_KEEP_ALIVE_S,
                                h11_max_incomplete_event_size=UVICORN_MAX_HEADER_BYTES)
        self.server = uvicorn.Server(config)
        self.thread = threading.Thread(
            target=self.server.run, kwargs={'sockets': [sock]}, name='uvicorn', daemon=True)
        self.thread.start()
        deadline = time.monotonic() + SERVER_START_TIMEOUT_S
        while not self.server.started:
            if not self.thread.is_alive() or time.monotonic() > deadline:
                raise ConfigError('Web server failed to start')
            time.sleep(0.05)
        self.get_logger().info(f'Web server listening on {self.host}:{self.port}')

    def stop_server(self):
        """Ask uvicorn to exit and wait a bounded time for its thread."""
        if self.server is not None:
            self.server.should_exit = True
        if self.thread is not None:
            self.thread.join(SERVER_STOP_TIMEOUT_S)


def main(args=None):
    """Run the bridge; return 1 if configuration or the port is invalid."""
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = MtrbWebBridge()
        node.start_server()
        executor = MultiThreadedExecutor(num_threads=EXECUTOR_THREADS)
        executor.add_node(node)
        executor.spin()
    except ConfigError as err:
        rclpy.logging.get_logger('mtrb_web_bridge').error(str(err))
        exit_code = EXIT_FAILURE
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            # Last-chance zeros before the publisher disappears (SIGINT / shutdown). The
            # context may already be shut down by the signal handler: then the publish
            # fails and is only logged.
            try:
                node._stop_teleop_only()
            except Exception as err:  # noqa: B902
                rclpy.logging.get_logger('mtrb_web_bridge').error(
                    f'Final zero Twist burst failed: {err}')
            node.stop_server()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
