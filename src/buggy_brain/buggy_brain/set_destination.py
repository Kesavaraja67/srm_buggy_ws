#!/usr/bin/env python3
"""
set_destination.py — Team Bravo interactive destination selector.
Run this in a separate terminal while the buggy system is running:
    ros2 run buggy_brain set_destination
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from buggy_brain.map_graph import VALID_DESTINATIONS, MENU


def main(args=None):
    rclpy.init(args=args)
    node = Node('destination_cli')
    pub = node.create_publisher(String, '/destination_request', 10)

    # Wait briefly for publisher to connect
    time.sleep(1.0)

    print("\n[Destination CLI] Connected to buggy system.")
    print("[Destination CLI] Publishing to /destination_request topic.\n")

    try:
        while rclpy.ok():
            try:
                raw = input(MENU)
            except EOFError:
                break

            choice = raw.strip().upper()

            if choice == 'Q':
                print("[Destination CLI] Quitting.")
                break

            if choice not in VALID_DESTINATIONS:
                print(f"  Invalid choice '{raw}'. Enter A, B, C, or D (or Q to quit).")
                continue

            dest = VALID_DESTINATIONS[choice]
            msg = String()
            msg.data = choice
            pub.publish(msg)
            print(f"  [Destination CLI] Sent destination: {dest}")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
