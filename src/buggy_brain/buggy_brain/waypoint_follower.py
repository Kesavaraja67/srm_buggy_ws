#!/usr/bin/env python3
"""
waypoint_follower.py  —  Team Bravo  (v2 — clean rewrite)
Master Plan §6.3
Subscribes: /planned_path, /odom, /buggy_state
Publishes:  /nav_cmd_vel, /navigation_command
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Float32

# Tuning constants
MAX_LINEAR_SPEED  = 2.00   # m/s
MIN_LINEAR_SPEED  = 0.30   # m/s
ANGULAR_P_GAIN    = 1.20   # proportional gain on heading error
MAX_ANGULAR_SPEED = 1.00   # rad/s cap
ARRIVAL_RADIUS    = 2.00   # metres
SLOW_TURN_SPEED   = 0.30   # m/s linear speed during curves
HEADING_THRESHOLD = 0.40   # rad — threshold for slowing down (~23°)
HEADING_DEADBAND  = 0.05   # rad
TURN_IN_PLACE_THR = 0.90   # rad — stop and spin if error > ~51°
CONTROL_RATE_HZ   = 10

# Spawn offset — Gazebo diff_drive odom starts at (0,0)
# but our map graph has the hub at (-13, 0)
SPAWN_X   = -13.0
SPAWN_Y   =   0.0
SPAWN_YAW =   0.0


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointFollowerNode(Node):

    def __init__(self):
        super().__init__('waypoint_follower_node')

        self._cmd_pub = self.create_publisher(Twist, '/nav_cmd_vel', 10)
        self._nav_pub = self.create_publisher(String, '/navigation_command', 10)

        self.create_subscription(Path,     '/planned_path', self._path_callback,  10)
        self.create_subscription(Odometry, '/odom',         self._odom_callback,  10)
        self.create_subscription(String,   '/buggy_state',  self._state_callback, 10)
        self.create_subscription(Float32,  '/avoidance_steering', self._avoidance_callback, 10)

        self._waypoints    = []
        self._wp_index     = 0
        self._pos_x        = 0.0
        self._pos_y        = 0.0
        self._yaw          = 0.0
        self._buggy_state  = 'WAITING_FOR_DESTINATION'
        self._odom_ready   = False
        self._avoidance_offset = 0.0

        self.create_timer(1.0 / CONTROL_RATE_HZ, self._control_loop)
        self.get_logger().info('WaypointFollowerNode v2 started.')

    def _path_callback(self, msg: Path):
        self._waypoints = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self._wp_index = 0
        self.get_logger().info(
            f'New path received: {len(self._waypoints)} waypoints: {self._waypoints}'
        )

    def _odom_callback(self, msg: Odometry):
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        # On first message, decide if odom is relative (starts ~0) or absolute (starts ~-13)
        if not hasattr(self, '_odom_offset_set'):
            if abs(raw_x) < 2.0 and abs(raw_y) < 2.0:
                # Relative odom — need to add spawn offset
                self._offset_x = SPAWN_X
                self._offset_y = SPAWN_Y
            else:
                # Absolute odom — no offset needed
                self._offset_x = 0.0
                self._offset_y = 0.0
            self._odom_offset_set = True
            self.get_logger().info(
                f'Odom offset set: ({self._offset_x}, {self._offset_y}), '
                f'first raw: ({raw_x:.2f}, {raw_y:.2f})')

        self._pos_x = raw_x + self._offset_x
        self._pos_y = raw_y + self._offset_y
        self._yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._odom_ready = True

    def _state_callback(self, msg: String):
        self._buggy_state = msg.data

    def _avoidance_callback(self, msg: Float32):
        self._avoidance_offset = msg.data

    def _control_loop(self):
        cmd = Twist()

        # Only move when state machine says NAVIGATING
        if self._buggy_state != 'NAVIGATING':
            self._cmd_pub.publish(cmd)   # zero velocity
            return

        if not self._odom_ready:
            self.get_logger().warn('Waiting for /odom...', throttle_duration_sec=5.0)
            return

        if not self._waypoints:
            return   # no path yet — stay still

        if self._wp_index >= len(self._waypoints):
            return   # all waypoints reached

        target_x, target_y = self._waypoints[self._wp_index]
        dx = target_x - self._pos_x
        dy = target_y - self._pos_y
        distance = math.hypot(dx, dy)

        # Arrival check — advance to next waypoint
        if distance < ARRIVAL_RADIUS:
            self.get_logger().info(
                f'Waypoint {self._wp_index} reached '
                f'(target={target_x:.1f},{target_y:.1f}, '
                f'pos={self._pos_x:.1f},{self._pos_y:.1f})'
            )
            self._wp_index += 1

            if self._wp_index >= len(self._waypoints):
                self.get_logger().info('Final destination reached!')
                self._cmd_pub.publish(Twist())  # stop
                nav_msg = String()
                nav_msg.data = 'DESTINATION_REACHED'
                self._nav_pub.publish(nav_msg)
                self._waypoints = []
            return

        # Periodic log of progress
        if self._wp_index < len(self._waypoints):
            self.get_logger().info(f'Distance to target: {distance:.2f}m', throttle_duration_sec=2.0)

        # P-controller for heading
        desired_heading = math.atan2(dy, dx)
        heading_error   = desired_heading - self._yaw

        # Normalise to [-π, π]
        while heading_error > math.pi:
            heading_error -= 2.0 * math.pi
        while heading_error < -math.pi:
            heading_error += 2.0 * math.pi

        if abs(heading_error) < HEADING_DEADBAND:
            heading_error = 0.0

        angular_z = max(-MAX_ANGULAR_SPEED,
                        min(MAX_ANGULAR_SPEED, ANGULAR_P_GAIN * heading_error))

        if abs(heading_error) > TURN_IN_PLACE_THR:
            linear_x = 0.0              # Stop and spin in place for > ~70°
        elif abs(heading_error) > HEADING_THRESHOLD:
            linear_x = SLOW_TURN_SPEED  # Slow down during sharp turns
        else:
            linear_x = min(MAX_LINEAR_SPEED, max(MIN_LINEAR_SPEED, distance * 0.8))

        cmd.linear.x  = linear_x
        cmd.angular.z = angular_z + self._avoidance_offset
        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            # Shutdown errors are usually benign (e.g., already shut down)
            print(f'Shutdown warning (WaypointFollowerNode): {e}')


if __name__ == '__main__':
    main()
