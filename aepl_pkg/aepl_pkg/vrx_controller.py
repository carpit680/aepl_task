#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class ThrusterController(Node):
    def __init__(self):
        super().__init__('thruster_controller')
        self.left_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.left_pos_pub = self.create_publisher(Float64, '/wamv/thrusters/left/pos', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.right_pos_pub = self.create_publisher(Float64, '/wamv/thrusters/right/pos', 10)
        
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.max_input_velocity = 1.0
        self.max_output_thrust = 1000.0
        self.max_output_pos = 1.0

    def cmd_vel_callback(self, msg):
        # Assuming a differential drive model
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Scale velocity to be within the maximum input velocity
        scaled_linear_vel = min(max(linear_vel, -self.max_input_velocity), self.max_input_velocity)
        scaled_angular_vel = min(max(angular_vel, -self.max_input_velocity), self.max_input_velocity)

        # Convert scaled velocity to thrust and position for left thruster
        left_thrust = (scaled_linear_vel - scaled_angular_vel) * (self.max_output_thrust / self.max_input_velocity)
        left_pos = -(scaled_linear_vel + scaled_angular_vel) / (2.0 * self.max_input_velocity) * self.max_output_pos

        # Convert scaled velocity to thrust and position for right thruster
        right_thrust = (scaled_linear_vel + scaled_angular_vel) * (self.max_output_thrust / self.max_input_velocity)
        right_pos = (scaled_linear_vel - scaled_angular_vel) / (2.0 * self.max_input_velocity) * self.max_output_pos

        # Publish values to topics
        self.left_thrust_pub.publish(Float64(data=left_thrust))
        self.left_pos_pub.publish(Float64(data=left_pos))
        self.right_thrust_pub.publish(Float64(data=right_thrust))
        self.right_pos_pub.publish(Float64(data=right_pos))

def main(args=None):
    rclpy.init(args=args)
    thruster_controller = ThrusterController()
    rclpy.spin(thruster_controller)
    thruster_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
