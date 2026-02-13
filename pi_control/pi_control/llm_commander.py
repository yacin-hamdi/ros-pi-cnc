#!/user/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
import json 
import time

class LLMCommander(Node):
    def __init__(self):
        super().__init__("llm_commander")

        self.pub_gcode_ = self.create_publisher(Point, "/cnc/move_to", 10)
        self.cli_home = self.create_client(Trigger, "cnc/set_home")

        while not self.cli_home.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Home Service...")
        
        self.set_home_position()

        self.MAX_X = 85.0
        self.MAX_Y = 52.0
        self.get_logger().info("LLM Commander Ready. Type 'draw a box' to test.")

        self.run_interactive_mode()

    def set_home_position(self):
        req = Trigger.Request()
        future = self.cli_home.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info("Startup SUCCESS: Home set to (0, 0).")
        else:
            self.get_logger().info("ERROR: Could not set home position!")

    def run_interactive_mode(self):

        while rclpy.ok():
            try:
                user_input = input("Enter command: ")
                if user_input == "exit": break

                points = self.get_points_from_llm(user_input)
                self.execute_sequence(points)

            except KeyboardInterrupt:
                break
    
    def get_points_from_llm(self, text):
        self.get_logger().info(f"Sending to LLM: {text}")

        if "box" in text or "square" in text:

            return [
                {"x": 0, "y": 0}, # Start
                {"x": 30, "y": 0}, # Right
                {"x": 30, "y": 15}, # Up
                {"x": 0, "y": 15}, # Left
                {"x": 0, "y": 0}
            ]
        elif "center" in text:
            return [{"x": 42.5, "y": 26.0}]
        else:
            self.get_logger().warn("Mock LLM didn't understand. Try 'draw a box'.")
            return []

    
    def execute_sequence(self, points_list):
        for pt in points_list:
            # 1. Validate against Limits (Double Safety)
            x = float(pt['x'])
            y = float(pt['y'])
            
            x = max(0.0, min(x, self.MAX_X))
            y = max(0.0, min(y, self.MAX_Y))
            
            # 2. Convert Millimeters to Meters (ROS Standard)
            msg = Point()
            msg.x = x / 1.0
            msg.y = y / 1.0
            msg.z = 0.0
            
            self.pub_gcode_.publish(msg)
            self.get_logger().info(f"Moving to: X={x}mm, Y={y}mm")
            
            # 3. Wait for move to finish? 
            # In a simple version, we just sleep. 
            # Ideally, we should listen to 'joint_states' to know when we arrived.
            time.sleep(2.0) 

def main(args=None):
    rclpy.init(args=args)
    node = LLMCommander()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()