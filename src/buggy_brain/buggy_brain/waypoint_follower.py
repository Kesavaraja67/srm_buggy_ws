import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math


class WaypointFollower(Node):
    """
    ROS 2 node that follows a planned path by publishing velocity commands.

    Subscribes : /planned_path   (nav_msgs/Path)
                 /odom            (nav_msgs/Odometry)
    Publishes  : /cmd_vel         (geometry_msgs/Twist)
                 /waypoint_status (std_msgs/String)
    """

    def __init__(self):
        super().__init__('waypoint_follower')

        # --- Parameters ---
        self.goal_tolerance    = 2.0   # metres — how close = "reached waypoint"
        self.linear_speed      = 2.0   # m/s forward speed
        self.angular_gain      = 1.5   # proportional gain for turning
        self.max_angular_speed = 1.0   # rad/s cap

        # --- State ---
        self.waypoints     = []        # list of (x, y) tuples from planned path
        self.current_wp    = 0         # index of active waypoint
        self.robot_x       = 0.0
        self.robot_y       = 0.0
        self.robot_yaw     = 0.0
        self.active        = False     # True when a path is loaded and running
        self.stopped       = False     # True when safety monitor says stop

        # --- Subscribers ---
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.safety_sub = self.create_subscription(
            String,
            '/safety_status',
            self.safety_callback,
            10
        )

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub  = self.create_publisher(String, '/waypoint_status', 10)

        # --- Control loop: runs at 10 Hz ---
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Waypoint follower started.')

    # ------------------------------------------------------------------
    def path_callback(self, msg):
        """Load a new path when published by path_planner_node."""
        self.waypoints  = [(p.pose.position.x, p.pose.position.y)
                           for p in msg.poses]
        self.current_wp = 0
        self.active     = True
        self.stopped    = False
        self.get_logger().info(
            f'New path received — {len(self.waypoints)} waypoints.')

    def odom_callback(self, msg):
        """Update robot position and orientation from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Convert quaternion to yaw angle
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def safety_callback(self, msg):
        """Listen to safety monitor — stop or resume based on status."""
        if msg.data == 'STOP':
            if not self.stopped:
                self.get_logger().warn('Safety stop received — halting buggy.')
            self.stopped = True
        elif msg.data == 'CLEAR':
            if self.stopped:
                self.get_logger().info('Path clear — resuming navigation.')
            self.stopped = False

    # ------------------------------------------------------------------
    def control_loop(self):
        """Proportional controller — runs at 10 Hz."""
        cmd = Twist()  # default is all zeros = stopped

        if not self.active or self.stopped or not self.waypoints:
            self.cmd_vel_pub.publish(cmd)
            return

        if self.current_wp >= len(self.waypoints):
            self.get_logger().info('All waypoints reached — mission complete.')
            self.active = False
            self.cmd_vel_pub.publish(cmd)
            self._publish_status('MISSION_COMPLETE')
            return

        # Current target waypoint
        tx, ty = self.waypoints[self.current_wp]

        # Distance to target
        dx       = tx - self.robot_x
        dy       = ty - self.robot_y
        distance = math.sqrt(dx * dx + dy * dy)

        # Check if waypoint reached
        if distance < self.goal_tolerance:
            self.get_logger().info(
                f'Waypoint {self.current_wp} reached — '
                f'({tx:.1f}, {ty:.1f})')
            self.current_wp += 1
            self._publish_status(f'WAYPOINT_{self.current_wp}_REACHED')
            return

        # Angle to target
        angle_to_target = math.atan2(dy, dx)
        angle_error     = angle_to_target - self.robot_yaw

        # Normalise angle error to [-pi, pi]
        while angle_error >  math.pi: angle_error -= 2 * math.pi
        while angle_error < -math.pi: angle_error += 2 * math.pi

        # Proportional controller
        angular_vel = self.angular_gain * angle_error
        angular_vel = max(-self.max_angular_speed,
                          min(self.max_angular_speed, angular_vel))

        # Slow down when turning sharply
        if abs(angle_error) > 0.5:
            linear_vel = self.linear_speed * 0.4
        else:
            linear_vel = self.linear_speed

        cmd.linear.x  = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)

    def _publish_status(self, text):
        msg      = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
