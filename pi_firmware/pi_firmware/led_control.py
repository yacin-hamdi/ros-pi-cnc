#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial

class LedControlNode(Node):
    def __init__(self):
        super().__init__("led_control_node")

        self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
        self.get_logger().info("Serial connection established on /dev/ttyACM1 at 115200 baud.")
        self.led_sub_ = self.create_subscription(
            Bool, 
            "led_control" ,
            self.led_callback, 
            10
        )


    def led_callback(self, msg: Bool):
        if msg.data:
            self.ser.write(b'1')
            self.get_logger().info("LED ON command send.")
        else:
            self.ser.write(b'0')
            self.get_logger().info("LED OFF command send.")


def main(args=None):
    rclpy.init(args=args)
    led_control_node = LedControlNode()
    rclpy.spin(led_control_node)
    led_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()