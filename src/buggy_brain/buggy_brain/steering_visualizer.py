#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math

class SteeringVisualizer(Node):
    def __init__(self):
        super().__init__('steering_visualizer')
        
        # Subscribes to cmd_vel to get steering commands
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # Publishes to steering_joints (consumed by joint_state_publisher)
        self.publisher = self.create_publisher(JointState, 'steering_joints', 10)
        
        # Joint names from URDF
        self.joint_names = ['stwheel_jt']
        
        # Configuration ratios
        self.STEERING_WHEEL_RATIO = -4.2  # Real steering wheel turns ~6:1; negative because of rotation axis
        
        self.get_logger().info('Steering Visualizer Node Started')

    def cmd_vel_callback(self, msg):
        # Create a new JointState message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        
        # Steering wheel visual rotation (tilted on column)
        # We multiply by ratio to make the steering wheel turn much more than the wheels
        stwheel_angle = msg.angular.z * self.STEERING_WHEEL_RATIO
        
        joint_state.position = [stwheel_angle]
        
        # Publish it! The joint_state_publisher + robot_state_publisher 
        # will merge this with other states.
        self.publisher.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = SteeringVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
