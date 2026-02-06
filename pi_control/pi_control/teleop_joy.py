#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time

class TeleopJoy(Node):
    def __init__(self):
        super().__init__('teleop_joy')
        self.sub_joy_ = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pub_vel_ = self.create_publisher(Twist, "/cmd_vel", 10)
        

        self.get_logger().info("TeleopJoy Node Initialized")

    def joy_callback(self, msg):
        twist = Twist()
        if msg.buttons[4] == 1:

            twist.linear.x = msg.axes[0] * 1.0
            twist.linear.y = msg.axes[1] * 1.0
        else:
            twist.linear.x = 0.0
            twist.linear.y = 0.0

        self.pub_vel_.publish(twist)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()