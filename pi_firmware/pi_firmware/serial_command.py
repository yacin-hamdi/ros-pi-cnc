#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time 

class SerialCommand(Node):
    def __init__(self):
        super().__init__("SerialCommand")

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.get_logger().info("Serial connection established on /dev/ttyACM0 at 115200 baud.")
        self.led_sub_ = self.create_subscription(
            String, 
            "serial_command" ,
            self.led_callback, 
            10
        )


    def led_callback(self, msg: String):
        self.get_logger().info(f"Received command: {msg.data}")
        self.ser.write(f"{msg.data}\n".encode('utf-8'))
        time.sleep(1.5)  # Give Arduino time to respond
        if self.ser.in_waiting > 0:
            reply = self.ser.readline().decode('utf-8').strip()
            self.get_logger().info(f"Arduino replied: {reply}")


def main(args=None):
    rclpy.init(args=args)
    serial_command = SerialCommand()
    rclpy.spin(serial_command)
    serial_command.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()