"""ROS 2 node that hosts the web server (FastAPI + uvicorn) next to rclpy."""

import hashlib
import math
import os
import socket
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mtrb_bringup.utils import MODES, validate_web_port
from mtrb_bringup.web_server import (
    BroadcastHub, create_app, is_placeholder_token, json_message, NavBusyError, NavPort,
    NavUnavailableError, resolve_token, StateStore, TOKEN_ENV_VAR)
from mtrb_bringup.web_utils import (
    downsample_scan, occupancy_to_bytes, pack_map_binary, quaternion_from_yaw,
    scan_to_map_points, simplify_path, yaw_from_quaternion)
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, LaserScan
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
    ('mode', ''),                     # injected by the launch ('map' | 'mapping')
    ('scan.max_points', 360),
    ('scan.min_range', 0.15),         # m
    ('scan.max_range', 12.0),         # m
    ('plan.max_points', 200),
    ('limits.max_clients', 5),
    ('limits.max_ws_bytes', 4096),
    ('auth.token', ''),
    ('dock.x', 0.0),                  # m
    ('dock.y', 0.0),                  # m
    ('dock.yaw', 0.0),                # rad
)


class ConfigError(Exception):
    """Invalid or missing configuration: the node must not start."""


class MtrbWebBridge(Node, NavPort):
    """Hold the shared StateStore, the NavPort implementation and the web server thread."""

    def __init__(self):
        """Declare parameters, validate them and prepare (not start) the web app."""
        super().__init__('mtrb_web_bridge')
        for name, default in PARAMETERS:
            if name == 'auth.token':
                # Dynamic typing: YAML may deliver a numeric token as int/float.
                self.declare_parameter(
                    name, default, ParameterDescriptor(dynamic_typing=True))
            else:
                self.declare_parameter(name, default)
        self.host = self.get_parameter('host').value
        try:
            self.port = validate_web_port(self.get_parameter('port').value)
        except ValueError as err:
            raise ConfigError(f'Invalid port: {err}') from err
        if not self.host:
            raise ConfigError('Invalid host: empty')

        share = get_package_share_directory(PACKAGE_NAME)
        token = self._select_token(share + '/param')
        if not token:
            raise ConfigError(
                'No auth token: set MTRB_WEB_TOKEN or auth.token in param/web.local.yaml')
        if is_placeholder_token(token):
            self.get_logger().warning(
                'Using the example token from the repository: set a real one in '
                'param/web.local.yaml or MTRB_WEB_TOKEN')

        self._read_settings()
        self.store = StateStore(clock=self._now_s)
        self.hub = BroadcastHub(self._max_clients)
        self.app = create_app(self.store, token, share + '/web', self.hub, self._max_ws_bytes,
                              nav_port=self, dock=self._dock)
        self.server = None
        self.thread = None
        self._init_state()
        self._init_ros_io()

    # --- Configuration ---------------------------------------------------------------

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
        self._dock = tuple(float(self._number(f'dock.{axis}', -DOCK_LIMIT))
                           for axis in ('x', 'y', 'yaw'))
        if any(abs(v) > DOCK_LIMIT for v in self._dock):
            raise ConfigError(f'dock.x/y/yaw must be within +-{DOCK_LIMIT}')
        mode = str(self.get_parameter('mode').value).strip().lower()
        self._mode = mode if mode in MODES else None  # None = unknown, reported as null

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
        self._cancel_all_client = None  # public CancelGoal service client (see _init_ros_io)

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
        self.create_timer(NAV_WATCHDOG_PERIOD_S, self._nav_watchdog)
        # No lock needed: the executor is not spinning yet, nothing else touches the state.
        self._publish_nav(NAV_IDLE, None)

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
        """
        now = self._now_s()
        send_all = False
        stamp_ns = 0
        with self._nav_lock:
            token, handle = self._nav_token, self._nav_handle
            targets = []
            if token is None and not self._nav_orphans:
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
            self.get_logger().info(
                f'Navigation cancel requested ({len(targets)} handle(s), '
                f'cancel-all {"yes" if send_all else "no"})')

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
            future.add_done_callback(self._on_cancel_all_response)
            return True
        except Exception as err:  # noqa: B902
            self.get_logger().error(f'Cancel-all request failed: {err}; will retry')
            return True  # a real attempt that failed: it counts, so the retries stay bounded

    def _on_cancel_all_response(self, future):
        """Stop retrying the cancel-all once Nav2 answered; a rejection is retried."""
        try:
            code = future.result().return_code
        except Exception as err:  # noqa: B902
            self.get_logger().warning(f'No cancel-all answer: {err}; will retry')
            return
        if code == CANCEL_ERROR_REJECTED or code not in (
                CANCEL_ERROR_NONE, CANCEL_ERROR_UNKNOWN_GOAL_ID, CANCEL_ERROR_GOAL_TERMINATED):
            self.get_logger().warning(f'Cancel-all rejected (code {code}); will retry')
            return
        with self._nav_lock:
            self._cancel_all = None  # answered: goals canceled or none was running
        self.get_logger().info(f'Cancel-all answered by Nav2 (code {code})')

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
        """Publish the pose estimate on `initialpose` (the only pose publisher here)."""
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
        """1 Hz: battery, mode and data ages (null = unknown)."""
        scan_age = None if self._scan_rx_s is None else max(0.0, self._now_s() - self._scan_rx_s)
        scan_age = None if scan_age is None else round(scan_age, 2)
        dock = dict(zip(('x', 'y', 'yaw'), self._dock))  # validated finite in _read_settings
        self._publish('status', battery_pct=self._battery_pct, mode=self._mode, nodes={},
                      scan_age=scan_age, odom_age=None, dock=dock)
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
        digest = hashlib.blake2b(bytes(memoryview(msg.data).cast('B')), digest_size=16)
        signature = (tuple(meta.values()), digest.digest())
        if signature == self._map_signature:
            return
        try:
            compressed = occupancy_to_bytes(msg.data, info.width, info.height)
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

    def _select_token(self, param_dir):
        """
        Token priority: env MTRB_WEB_TOKEN > parameter auth.token > share files.

        Never logged. Returns '' if nothing usable is found.
        """
        env_token = os.environ.get(TOKEN_ENV_VAR, '')
        if env_token:
            return env_token
        value = self.get_parameter('auth.token').value
        if value is not None and not isinstance(value, bool):
            param_token = str(value).strip()
            if param_token:
                return param_token
        return resolve_token(param_dir)

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
        config = uvicorn.Config(self.app, log_level='warning', access_log=False,
                                ws_max_size=self._max_ws_bytes)
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
            node.stop_server()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
