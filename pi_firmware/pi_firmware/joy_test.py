#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time

class CncSerialTeleop(Node):
    def __init__(self):
        super().__init__('cnc_serial_teleop')
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # Open Serial Port (Match your Arduino baud rate)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        
        time.sleep(2)  # Give it 2 seconds
    
        # Clear the startup text from the buffer
        self.ser.flushInput()
        
        # Now send the modes
        self.ser.write(b"\r\n\r\n") # Wake up characters
        time.sleep(0.5)
        self.ser.write(b"G91 G21\n") 
        self.get_logger().info("CNC Set to Relative Mode (G91)")
        self.last_command_time = self.get_clock().now()
        self.min_interval = 0.02  # Max 20 commands per second (50ms)

        self.get_logger().info("CNC Serial Teleop Node Initialized")
        self.moving = True

    def joy_callback(self, msg):
        
        now = self.get_clock().now()
        
        # 1. Time-based limit
        if (now - self.last_command_time).nanoseconds / 1e9 < self.min_interval:
            return

        
        x_val = msg.axes[1]
        
        if abs(x_val) > 0.2:  # Smaller deadzone for better feel
            # Map stick to a small relative distance (e.g., 2mm)
            step_x = x_val * 1.0 
            
            # Use the $J= Jogging command (Real-time and interruptible)
            # This is much smoother than G1
            command = f"$J=G91 X{step_x:.2f} F800\n"
            
            self.ser.write(command.encode('utf-8'))
            self.get_logger().info(f"Sent: {command.strip()}")
            self.last_command_time = now
        else:
            if self.moving:
            # 3. IF JOYSTICK IS RELEASED: Send immediate stop character
            # '\x85' is the GRBL jog cancel character. 
            # It clears the jog buffer immediately so it doesn't "keep moving"
                self.ser.write(b"\x85")
                time.sleep(0.01)
                self.ser.reset_input_buffer()
                self.moving = False
            return
        self.moving = True

def main(args=None):
    rclpy.init(args=args)
    node = CncSerialTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()