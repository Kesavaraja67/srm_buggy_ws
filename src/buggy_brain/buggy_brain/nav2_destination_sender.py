#!/usr/bin/env python3
"""
nav2_destination_sender.py — SRM Autonomous Buggy
──────────────────────────────────────────────────
Shows a terminal menu (A/B/C/D) and sends the selected
destination as a Nav2 NavigateToPose action goal.

The buggy uses the LiDAR map + Nav2 stack for
path planning and obstacle avoidance.
"""
import time
import threading
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math

# ── Destination coordinates (MAP frame) ──────────────────────
# Coordinates adjusted to land EXACTLY on known free white road pixels 
DESTINATIONS = {
    'A': ('SRM Institute (North)',   -12.25,  49.90),
    'B': ('SRM Hospital (East)',      50.25,  11.90),
    'C': ('SRM Temple (South)',       11.15, -50.80),
    'D': ('Buggy Hub (Home)',        -11.00,   0.00),
}

MENU = (
    "\n"
    "========================================\n"
    "  SRM Autonomous Buggy — Nav2 Command  \n"
    "========================================\n"
    "  [A] SRM Institute (North)\n"
    "  [B] SRM Hospital (East)\n"
    "  [C] SRM Temple (South)\n"
    "  [D] Buggy Hub (Home)\n"
    "  [Q] Quit\n"
    "========================================\n"
    "Select Destination: "
)

# Spawn pose matches Gazebo starting position exactly
SPAWN_X   = -13.0
SPAWN_Y   =  0.0
SPAWN_YAW =  0.0


from rclpy.parameter import Parameter

class Nav2DestinationSender(Node):

    def __init__(self):
        super().__init__('nav2_destination_sender', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self._navigating = False
        self._goal_handle = None
        self._nav2_ready = False

        self.get_logger().info('Nav2 Destination Sender started.')

        # Background thread: wait for Nav2 then show menu
        self._thread = threading.Thread(
            target=self._setup_and_run, daemon=True)
        self._thread.start()

    # ── Setup ─────────────────────────────────────────────────

    def _setup_and_run(self):
        """Wait for Nav2 with timeout, then ALWAYS show menu."""
        self.get_logger().info('Waiting for Nav2 action server (max 30s)...')

        # Try to connect to the action server
        self._nav2_ready = self._nav_client.wait_for_server(timeout_sec=30.0)

        if self._nav2_ready:
            self.get_logger().info('Nav2 action server connected!')
        else:
            self.get_logger().warning(
                'Nav2 action server not found after 30s. '
                'Menu will show but navigation may not work.')

        # Publish initial pose regardless
        time.sleep(1.0)
        self._publish_initial_pose()
        time.sleep(1.0)

        # ALWAYS show the menu
        self._input_loop()

    # ── Initial pose ──────────────────────────────────────────

    def _publish_initial_pose(self):
        """Set AMCL initial pose to match the buggy spawn location."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = SPAWN_X
        msg.pose.pose.position.y = SPAWN_Y
        msg.pose.pose.position.z = 0.0

        # Yaw to quaternion
        msg.pose.pose.orientation.z = math.sin(SPAWN_YAW / 2.0)
        msg.pose.pose.orientation.w = math.cos(SPAWN_YAW / 2.0)

        # Small covariance
        msg.pose.covariance[0]  = 0.25  # x
        msg.pose.covariance[7]  = 0.25  # y
        msg.pose.covariance[35] = 0.07  # yaw

        self._initial_pose_pub.publish(msg)
        self.get_logger().info(
            f'Published initial pose: x={SPAWN_X}, y={SPAWN_Y}, yaw={SPAWN_YAW}')

    # ── Menu loop ─────────────────────────────────────────────

    def _input_loop(self):
        """Terminal menu loop — ALWAYS shows, regardless of Nav2 state."""
        while rclpy.ok():
            if self._navigating:
                time.sleep(0.5)
                continue

            print(MENU, end='', flush=True)
            try:
                raw = input()
            except EOFError:
                break

            choice = raw.strip().upper()

            if choice == 'Q':
                print("\n  Shutting down...")
                break

            if choice not in DESTINATIONS:
                print(f"  Invalid choice '{choice}'. Enter A, B, C, or D.")
                continue

            if not self._nav2_ready:
                print("  ⚠ Nav2 is not ready yet. Try again in a few seconds.")
                continue

            name, goal_x, goal_y = DESTINATIONS[choice]
            print(f"\n  Navigating to: {name} ({goal_x}, {goal_y})")
            print("  Using Nav2 + LiDAR map for path planning...\n")
            self._send_goal(goal_x, goal_y, name)

    # ── Navigation ────────────────────────────────────────────

    def _send_goal(self, x: float, y: float, name: str, attempt: int = 1):
        """Send a NavigateToPose goal to Nav2 with auto-retry."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self._navigating = True
        self._current_goal = (x, y, name, attempt)
        future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        future.add_done_callback(
            lambda f: self._goal_response_cb(f, name))

    def _goal_response_cb(self, future, name: str):
        goal_handle = future.result()
        if not goal_handle.accepted:
            x, y, gname, attempt = self._current_goal
            if attempt < 10:
                print(f"  ⏳ Nav2 not ready yet (attempt {attempt}/10). Retrying in 3s...")
                time.sleep(3.0)
                self._send_goal(x, y, gname, attempt + 1)
            else:
                self.get_logger().error(f'Goal to {name} REJECTED after 10 attempts.')
                print(f"  ❌ Nav2 could not accept the goal. Try again later.")
                self._navigating = False
            return


        self.get_logger().info(f'Goal to {name} ACCEPTED. Navigating...')
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._result_cb(f, name))

    def _feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        remaining = feedback.distance_remaining
        self.get_logger().info(
            f'Distance remaining: {remaining:.1f}m',
            throttle_duration_sec=3.0)

    def _result_cb(self, future, name: str):
        status = future.result().status
        if status == 4:  # SUCCEEDED
            print("\n" + "*" * 50)
            print(f"  ARRIVED AT {name}!")
            print("*" * 50 + "\n")
        else:
            print(f"\n  Navigation ended with status: {status}\n")

        self._navigating = False
        self._goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = Nav2DestinationSender()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
