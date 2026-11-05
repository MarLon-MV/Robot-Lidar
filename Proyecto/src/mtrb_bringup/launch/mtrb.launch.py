"""
Single entry point for the MTRB robot (Create 3 + 2D LiDAR).

  ros2 launch mtrb_bringup mtrb.launch.py mode:=map     map:=<map.yaml>
  ros2 launch mtrb_bringup mtrb.launch.py mode:=mapping [posegraph:=<base path>]

mode:=map      map_server + AMCL on an existing map. The dock pose is published once on
               /<ns>/initialpose when AMCL is active.
mode:=mapping  slam_toolbox. Without posegraph the map origin is the robot's pose at
               start (the dock). With posegraph the saved map is extended and the robot
               starts at the dock pose given by dock_x/dock_y/dock_yaw.

Both modes also start the EKF and Nav2. Exactly one of AMCL / slam_toolbox runs, since
both publish map -> odom. The Create 3 publishes odom -> base_link; nothing here does.

Every node runs in the robot namespace and remaps /tf and /tf_static to the relative
names, so it uses /<ns>/tf and /<ns>/tf_static.

Layout: one function per subsystem (_state_publisher, _lidar, _ekf, _localization, _slam,
_navigation). _launch_setup only reads the arguments and picks which ones to start.
"""

from dataclasses import dataclass
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from mtrb_bringup.utils import (absolute_path, absolute_topic, is_true, validate_mode,
                                validate_namespace, validate_web_port)
from nav2_common.launch import RewrittenYaml
import xacro

PKG = 'mtrb_bringup'
DESCRIPTION_PKG = 'mtrb_description'
TF_REMAPS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


@dataclass(frozen=True)
class Settings:
    """Launch arguments, already parsed. Built once and passed to every subsystem."""

    get: callable  # name -> raw string value of any launch argument
    mode: str
    ns: str
    use_sim_time: bool
    use_ekf: bool
    scan_topic: str
    lidar_frame: str
    dock: tuple
    web: bool
    web_port: int

    @classmethod
    def from_context(cls, context):
        def get(name):
            return LaunchConfiguration(name).perform(context).strip()
        return cls(
            get=get,
            mode=validate_mode(get('mode')),
            ns=validate_namespace(get('namespace')),
            use_sim_time=is_true(get('use_sim_time')),
            use_ekf=is_true(get('use_ekf')),
            scan_topic=get('scan_topic'),
            lidar_frame=get('lidar_frame'),
            dock=(float(get('dock_x')), float(get('dock_y')), float(get('dock_yaw'))),
            web=is_true(get('web')),
            web_port=validate_web_port(get('web_port')))


# --- Helpers -------------------------------------------------------------------------

def _node(s, package, executable, name, parameters=(), remappings=(), cls=Node):
    """Node in the robot namespace, with TF remapped and use_sim_time set."""
    return cls(
        package=package,
        executable=executable,
        name=name,
        namespace=s.ns,
        output='screen',
        parameters=list(parameters) + [{'use_sim_time': s.use_sim_time}],
        remappings=TF_REMAPS + list(remappings))


def _lifecycle_manager(s, name, node_names):
    return _node(s, 'nav2_lifecycle_manager', 'lifecycle_manager', name,
                 parameters=[{'autostart': True, 'node_names': list(node_names)}])


def _param_file(s, arg_name, rewrites=None):
    """YAML from a launch argument, rooted at the namespace, with optional overrides."""
    return RewrittenYaml(
        source_file=s.get(arg_name),
        root_key=s.ns,
        param_rewrites=dict(rewrites or {}, use_sim_time=str(s.use_sim_time).lower()),
        convert_types=True)


def _nav_params(s):
    return _param_file(s, 'params_file', {
        'odom_topic': 'odometry/filtered' if s.use_ekf else s.get('odom_topic'),
        'topic': s.scan_topic,
        'sensor_frame': s.lidar_frame,
    })


# --- Subsystems ----------------------------------------------------------------------

def _state_publisher(s):
    """base_link -> LiDAR on /<ns>/tf_static (the Create 3 publishes the rest)."""
    urdf = os.path.join(get_package_share_directory(DESCRIPTION_PKG), 'urdf', 'mtrb.urdf.xacro')
    description = xacro.process_file(urdf, mappings={
        'lidar_only': 'false' if is_true(s.get('full_model')) else 'true',
        'lidar_frame': s.lidar_frame,
        'lidar_x': s.get('lidar_x'), 'lidar_y': s.get('lidar_y'),
        'lidar_z': s.get('lidar_z'), 'lidar_yaw': s.get('lidar_yaw'),
    }).toxml()
    # Remapped so it does not clash with the robot's own /<ns>/robot_description.
    return [_node(s, 'robot_state_publisher', 'robot_state_publisher', 'mtrb_state_publisher',
                  parameters=[{'robot_description': description}],
                  remappings=[('robot_description', 'mtrb_description')])]


def _lidar(s):
    """LD19 driver (ldlidar_stl_ros2) -> /<ns>/<scan_topic> in frame lidar_frame."""
    if not is_true(s.get('lidar')):
        return []
    return [_node(s, 'ldlidar_stl_ros2', 'ldlidar_stl_ros2_node', 'ldlidar_node',
                  parameters=[_param_file(s, 'lidar_params_file'),
                              {'frame_id': s.lidar_frame, 'port_name': s.get('lidar_port')}],
                  remappings=[('scan', s.scan_topic)])]


def _ekf(s):
    """Create 3 odometry + IMU -> /<ns>/odometry/filtered."""
    if not s.use_ekf:
        return []
    return [_node(s, 'robot_localization', 'ekf_node', 'ekf_filter_node',
                  parameters=[_param_file(s, 'ekf_params_file'),
                              {'publish_tf': is_true(s.get('ekf_publish_tf'))}],
                  remappings=[('odom', s.get('odom_topic')), ('imu', s.get('imu_topic'))])]


def _localization(s, nav_params):
    """mode:=map -> map_server + AMCL, plus the dock pose as initial pose."""
    map_yaml = absolute_path(
        s.get('map') or os.path.join(get_package_share_directory(PKG), 'map', 'map.yaml'))
    if not os.path.isfile(map_yaml):
        raise RuntimeError(
            f'Map file not found: {map_yaml}. Pass map:=<map.yaml> or save one with '
            'mode:=mapping first (see README)')
    x, y, yaw = s.dock
    return [
        _node(s, 'nav2_map_server', 'map_server', 'map_server',
              parameters=[nav_params, {'yaml_filename': map_yaml}]),
        _node(s, 'nav2_amcl', 'amcl', 'amcl',
              parameters=[nav_params, {'scan_topic': s.scan_topic}]),
        _lifecycle_manager(s, 'lifecycle_manager_localization', ['map_server', 'amcl']),
        _node(s, PKG, 'initial_pose_publisher', 'initial_pose_publisher',
              parameters=[{'x': x, 'y': y, 'yaw': yaw}]),
    ]


def _slam(s):
    """mode:=mapping -> slam_toolbox (lifecycle node in Jazzy: configure, then activate)."""
    # slam_toolbox subscribes with the name given, so pass the absolute topic.
    overrides = {'scan_topic': absolute_topic(s.ns, s.scan_topic)}
    posegraph = s.get('posegraph')
    if posegraph:
        # Base path of the serialized pose-graph (no .posegraph/.data extension).
        overrides['map_file_name'] = absolute_path(posegraph)
        overrides['map_start_pose'] = list(s.dock)

    slam = _node(s, 'slam_toolbox', 'async_slam_toolbox_node', 'slam_toolbox',
                 parameters=[_param_file(s, 'slam_params_file'), overrides],
                 cls=LifecycleNode)

    def transition(transition_id):
        return EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam), transition_id=transition_id))

    activate_when_configured = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam,
        start_state='configuring',
        goal_state='inactive',
        entities=[LogInfo(msg='[mtrb] slam_toolbox configured, activating'),
                  transition(Transition.TRANSITION_ACTIVATE)]))
    return [slam, activate_when_configured, transition(Transition.TRANSITION_CONFIGURE)]


def _navigation(s, nav_params):
    """
    Nav2: planner, controller, behaviors, bt_navigator and velocity smoother.

    cmd_vel_nav -> velocity_smoother -> /<ns>/cmd_vel (geometry_msgs/Twist in Jazzy).
    """
    to_smoother = [('cmd_vel', 'cmd_vel_nav')]
    # (package, executable/name, extra params, remappings)
    servers = [
        ('nav2_controller', 'controller_server', {}, to_smoother),
        # Paths may cross unknown space only while mapping.
        ('nav2_planner', 'planner_server', {'GridBased.allow_unknown': s.mode == 'mapping'}, []),
        ('nav2_behaviors', 'behavior_server', {}, to_smoother),
        ('nav2_bt_navigator', 'bt_navigator', {}, []),
        ('nav2_velocity_smoother', 'velocity_smoother', {},
         to_smoother + [('cmd_vel_smoothed', 'cmd_vel')]),
    ]
    nodes = [_node(s, pkg, name, name, parameters=[nav_params, extra], remappings=remaps)
             for pkg, name, extra, remaps in servers]
    names = [name for _, name, _, _ in servers]
    return nodes + [_lifecycle_manager(s, 'lifecycle_manager_navigation', names)]


def _web(s):
    """Web bridge (FastAPI + rclpy), only with web:=true. Dock pose comes from the launch."""
    if not s.web:
        return []
    x, y, yaw = s.dock
    return [_node(s, PKG, 'mtrb_web_bridge', 'mtrb_web_bridge',
                  parameters=[_param_file(s, 'web_params_file'),
                              {'port': s.web_port,
                               'mode': s.mode,
                               'dock.x': x, 'dock.y': y, 'dock.yaw': yaw}])]


# --- Entry point ---------------------------------------------------------------------

def _launch_setup(context):
    s = Settings.from_context(context)
    nav_params = _nav_params(s)

    actions = _state_publisher(s) + _lidar(s) + _ekf(s)
    # map -> odom: AMCL (map) or slam_toolbox (mapping), never both.
    actions += _localization(s, nav_params) if s.mode == 'map' else _slam(s)
    if is_true(s.get('navigation')):
        actions += _navigation(s, nav_params)
    actions += _web(s)
    return actions


def generate_launch_description():
    pkg = get_package_share_directory(PKG)

    def param(name):
        return os.path.join(pkg, 'param', name)

    # web.local.yaml (git-ignored, optional parameter overrides, no token) wins over web.yaml.
    web_params = param('web.local.yaml')
    if not os.path.isfile(web_params):
        web_params = param('web.yaml')

    arg = DeclareLaunchArgument
    return LaunchDescription([
        arg('mode', choices=['map', 'mapping'],
            description="'map': map_server + AMCL on an existing map; "
                        "'mapping': slam_toolbox (new map or extend one)"),
        arg('namespace', default_value='r2', description='Robot namespace (Create 3)'),
        arg('use_sim_time', default_value='false'),

        arg('map', default_value='', description='Map .yaml for mode:=map '
            '(default: map/map.yaml of mtrb_bringup)'),
        arg('posegraph', default_value='',
            description='mode:=mapping: base path of a serialized pose-graph to extend; '
                        'empty = new map'),
        arg('navigation', default_value='true',
            description='Start Nav2 (false: map/localize only)'),

        arg('params_file', default_value=param('nav2.yaml')),
        arg('ekf_params_file', default_value=param('ekf.yaml')),
        arg('slam_params_file', default_value=param('slam_toolbox.yaml')),

        # Dock pose in the map frame: initial pose (map) or start pose (mapping+posegraph).
        arg('dock_x', default_value='0.0'),
        arg('dock_y', default_value='0.0'),
        arg('dock_yaw', default_value='0.0', description='rad'),

        # Topics, relative to the namespace unless they start with '/'.
        arg('scan_topic', default_value='scan'),
        arg('odom_topic', default_value='odom'),
        arg('imu_topic', default_value='imu'),
        arg('use_ekf', default_value='true'),
        arg('ekf_publish_tf', default_value='false',
            description='EKF publishes odom -> base_link. Only if the Create 3 does not.'),

        # LiDAR: LDROBOT LD19 driver (ldlidar_stl_ros2).
        arg('lidar', default_value='true',
            description='Start the LD19 driver (false: run your own driver)'),
        arg('lidar_port', default_value='/dev/ttyUSB0',
            description='Serial port of the LiDAR (/dev/ldlidar with the udev rule)'),
        arg('lidar_params_file', default_value=param('lidar.yaml')),
        arg('lidar_frame', default_value='laser',
            description='frame_id of the scans (set on the driver and in the URDF)'),

        # LiDAR mount relative to base_link, at the scan plane. TODO: measure x/y/z.
        # z = height of the mounting surface + 0.0298 m (LD19 scan plane above its base).
        arg('lidar_x', default_value='0.0'),
        arg('lidar_y', default_value='0.0'),
        arg('lidar_z', default_value='0.1'),
        # ldlidar_stl_ros2 publishes the device angle as is (0 = LD19 front), so with the
        # LD19 mounted facing the robot's front no rotation is needed.
        arg('lidar_yaw', default_value='0.0', description='rad'),
        arg('full_model', default_value='false',
            description='Publish the full Create 3 model too (duplicates its tf_static; '
                        'a visualizer only needs the description topic)'),

        # Web UI / REST API (LAN only). Token: MTRB_WEB_TOKEN or ~/.config/mtrb/web_token (0600).
        arg('web', default_value='false', description='Start mtrb_web_bridge'),
        arg('web_port', default_value='8080', description='Web bridge port (1024-65535)'),
        arg('web_params_file', default_value=web_params,
            description='web.local.yaml if present, else web.yaml'),

        OpaqueFunction(function=_launch_setup),
    ])
