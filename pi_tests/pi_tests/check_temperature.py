#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from gpiozero import CPUTemperature
from example_interfaces.msg import Float64

class CheckTemperatureNode(Node):
    def __init__(self):
        super().__init__('check_temperature_node')
        self.get_logger().info(f"rpi temperature {CPUTemperature().temperature}")
        self.get_logger().info('Check Temperature Node has been started.')
        self.pub = self.create_publisher(Float64, "/temperature", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        msg.data = CPUTemperature().temperature
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CheckTemperatureNode()
    rclpy.spin(node)
    rclpy.shutdown()