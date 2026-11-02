"""Publish the dock pose once on /initialpose when AMCL is active, then exit."""

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from mtrb_bringup.utils import yaw_to_quaternion
import rclpy
from rclpy.node import Node

# Initial uncertainty: 0.5 m std in x/y, ~15 deg std in yaw.
COVARIANCE_XY = 0.25
COVARIANCE_YAW = 0.0685


def build_initial_pose(x, y, yaw, frame_id, stamp):
    """Build the PoseWithCovarianceStamped of a 2D pose (x, y, yaw) in `frame_id`."""
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    q = msg.pose.pose.orientation
    q.x, q.y, q.z, q.w = yaw_to_quaternion(yaw)
    msg.pose.covariance[0] = COVARIANCE_XY    # x
    msg.pose.covariance[7] = COVARIANCE_XY    # y
    msg.pose.covariance[35] = COVARIANCE_YAW  # yaw
    return msg


class InitialPosePublisher(Node):
    """Wait for AMCL to be active, then publish the dock pose `repeat` times."""

    def __init__(self):
        """Declare parameters, the publisher, the AMCL state client and the timer."""
        super().__init__('initial_pose_publisher')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('amcl_node', 'amcl')
        self.declare_parameter('timeout', 60.0)
        self.declare_parameter('repeat', 3)

        # Relative names: they resolve under the node namespace (/r2/initialpose ...).
        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        amcl = self.get_parameter('amcl_node').value
        self.client = self.create_client(GetState, f'{amcl}/get_state')

        self.start = self.get_clock().now()
        self.sent = 0
        self.pending = None
        self.done = False
        self.failed = False
        self.create_timer(0.5, self.tick)

    def amcl_active(self):
        """Ask AMCL for its lifecycle state without blocking the executor."""
        if self.pending is None:
            if self.client.service_is_ready():
                self.pending = self.client.call_async(GetState.Request())
            return False
        if not self.pending.done():
            return False
        result = self.pending.result()
        self.pending = None
        return result is not None and result.current_state.id == State.PRIMARY_STATE_ACTIVE

    def tick(self):
        """Timer callback: wait for AMCL (with timeout), then publish."""
        waited = (self.get_clock().now() - self.start).nanoseconds / 1e9
        if self.sent == 0:
            if waited > self.get_parameter('timeout').value:
                self.get_logger().error('AMCL did not become active in time; pose not sent')
                self.failed = self.done = True
                return
            if not self.amcl_active() or self.pub.get_subscription_count() == 0:
                return
        self.publish()
        self.sent += 1
        if self.sent >= self.get_parameter('repeat').value:
            self.done = True

    def publish(self):
        """Publish the dock pose once."""
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        yaw = self.get_parameter('yaw').value
        self.pub.publish(build_initial_pose(
            x, y, yaw, self.get_parameter('frame_id').value,
            self.get_clock().now().to_msg()))
        self.get_logger().info(
            f'Initial pose (dock) published: x={x:.3f} y={y:.3f} yaw={yaw:.3f}')


def main(args=None):
    """Run until the pose is sent; exit code 1 if AMCL never became active."""
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    failed = node.failed
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(1 if failed else 0)


if __name__ == '__main__':
    main()
