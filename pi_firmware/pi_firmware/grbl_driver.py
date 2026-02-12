#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import serial
import time
import re

class GrblDriver(Node):
    def __init__(self):
        super().__init__('GrblDriver')

        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200

        self.feed_rate = 100    # Speed in mm/min
        self.loop_rate = 0.1
        self.step_scaler = 1.0

        self.sub_vel_ = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        self.sub_abs_ = self.create_subscription(Point, "cnc/move_to", self.abs_callback, 10)


        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            time.sleep(2)  
            self.ser.flushInput()
            time.sleep(0.5)
            self.ser.write(b"G91 G21\n") 
            self.get_logger().info(f"Connected to GRBL on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Could not connect to GRBL: {e}")
            return 

        self.latest_twist = Twist()
        self.moving = False

        self.timer = self.create_timer(self.loop_rate, self.control_loop)


    def vel_callback(self, msg):
        self.latest_twist = msg

    def abs_callback(self, msg):
        if self.moving:
            self.get_logger().info("Ignoring LLM command: Joystick is active.")
            return
        
        target_x = msg.x * 1.0
        target_y = msg.y * 1.0

        command = f"G90 G0 X{target_x:.3f} Y{target_y:.3f}\n"

        self.get_logger().info(f"LLM command: {command.strip()}")
        self.ser.write(command.encode('utf-8'))

    def control_loop(self):
        while self.ser.in_waiting:
            try: 
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                

                if line.startswith('<'):
                    self.parse_and_publish_joints(line)

            except Exception as e:
                        self.get_logger().error(f"CRASH in parser: {e}")
        
        x_vel = self.latest_twist.linear.x 
        y_vel = self.latest_twist.linear.y 

        if abs(x_vel) > 0.05 or abs(y_vel) > 0.05:

            step_x = x_vel * self.step_scaler
            step_y = y_vel * self.step_scaler

            command = f"$J=G91 X{step_x:.3f} Y{step_y:.3f} F{self.feed_rate}\n"
            self.get_logger().info("[command]: " + command)
            self.ser.write(command.encode('utf-8'))
            self.moving = True
            self.ser.write(b"?")
        else:
            if self.moving:
                self.ser.write(b"\x85")
                self.ser.flushInput()
                self.moving = False
                # self.get_logger().info("Stopping")
                self.ser.write(b"?")

        

    def parse_and_publish_joints(self, data):
        # Regex to find MPos:1.23,4.56,7.89
        match = re.search(r'MPos:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+)', data)
        
        if match:
            # Convert Millimeters (GRBL) to Meters (ROS)
            x_m = -float(match.group(2)) / 500.0
            y_m = -float(match.group(1)) / 500.0

            self.get_logger().info(f"[jointState]: {x_m}, {y_m}")

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['joint_x', 'joint_y'] # Must match your URDF
            
            msg.position = [x_m, y_m]
            
            
            self.pub_joints.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GrblDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()