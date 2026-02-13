#!/user/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
from openai import AsyncOpenAI
from agents import Agent, Runner, OpenAIChatCompletionsModel, function_tool
from pydantic import BaseModel, Field
import logging
import asyncio
from dotenv import load_dotenv
import json 
import time
import os

logging.basicConfig(level=logging.DEBUG)

logging.getLogger("openai").setLevel(logging.ERROR)
logging.getLogger("httpcore").setLevel(logging.ERROR)
logging.getLogger("httpx").setLevel(logging.ERROR)

GOOGLE_BASE_URL = "https://generativelanguage.googleapis.com/v1beta/openai/"
OPENROUTER_BASE_URL = "https://openrouter.ai/api/v1"
load_dotenv(dotenv_path="/home/pi/.env", override=True)
google_api_key = os.getenv("GOOGLE_API_KEY")
openrouter_api_key = os.getenv("OPENROUTER_API_KEY")



class Coordinate(BaseModel):
    x: float = Field(..., description="The X position in millimeters (0-85)")
    y: float = Field(..., description="The Y position in millimeters (0-52)")

class ListCoordinates(BaseModel):
    points: list[Coordinate] = Field(..., description="The complete list of sequential coordinates to draw")




class LLMCommander(Node):
    def __init__(self):
        super().__init__("llm_commander")
        
        self.pub_gcode_ = self.create_publisher(Point, "/cnc/move_to", 10)
        self.cli_home = self.create_client(Trigger, "cnc/set_home")

        while not self.cli_home.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Home Service...")
        
        self.set_home_position()
        self.get_logger().info("Set home position to (0, 0)")

        self.MAX_X = 85.0
        self.MAX_Y = 52.0
        

        # Define the Agent
        self.get_logger().info("Define the cnc agent")
        client = AsyncOpenAI(base_url=GOOGLE_BASE_URL, api_key=google_api_key)
        model = OpenAIChatCompletionsModel(model="gemini-3-flash-preview", openai_client=client)
        openrouter_client = AsyncOpenAI(base_url=OPENROUTER_BASE_URL, api_key=openrouter_api_key)
        openai_model = OpenAIChatCompletionsModel(model="openai/gpt-oss-120b:free", openai_client=openrouter_client)

        instructions = f"""
            You are the geometric brain of a 2-axis CNC Plotter.
            
            HARDWARE CONSTRAINTS:
            - X-Axis: 0 to {self.MAX_X} mm
            - Y-Axis: 0 to {self.MAX_Y} mm
            - Origin: (0,0) is Bottom-Left.
            
            YOUR JOB:
            1. Receive a natural language request (e.g., "Draw a 20mm box in the center").
            2. Calculate the specific coordinates. 
               - Center of workspace is ({self.MAX_X/2}, {self.MAX_Y/2}).
            3. Call the `draw_shape` tool with the list of points.
            
            CRITICAL RULES:
            - DO NOT chat with the user.
            - DO NOT output JSON text or Markdown.
            - ONLY call the tool.
            - If asking for a Circle, approximate it with 16-20 small line segments.
            
            EXAMPLE 1:
            User: "Draw a 10mm square at 10,10"
            Thought: I need 5 points: (10,10) -> (20,10) -> (20,20) -> (10,20) -> (10,10).
            Action: call draw_shape(shape_name="square", points=[...])
            
            EXAMPLE 2:
            User: "Draw a triangle in the middle"
            Thought: Center is ~42,26. I will calculate 4 points around that center.
            Action: call draw_shape(...)
            """
        self.cnc_agent = Agent(
            name="CNC Agent", 
            instructions=instructions,
            model=openai_model
        )
        
        self.get_logger().info("LLM Commander Ready")


    def set_home_position(self):
        req = Trigger.Request()
        future = self.cli_home.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info("Startup SUCCESS: Home set to (0, 0).")
        else:
            self.get_logger().info("ERROR: Could not set home position!")

    
    @function_tool
    async def draw_shape(self, path: ListCoordinates):
        """
        Moves the robot through a list of points.
        """

        print(f"\n[DEBUG] Agent called tool with: {path.points}")
        count = len(path.points)
        self.get_logger().info(f"🎨 Drawing {path.shape_name} ({count} points)...")

        for i, pt in enumerate(path.points):
            # 1. Validate against Limits (Double Safety)
            x = float(pt.x)
            y = float(pt.y)
            
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
            await asyncio.sleep(1.5)
        
        return f"Finished drawing"
    

async def run_node():
    rclpy.init()
    node = LLMCommander()
    
    # Register tool manually to instance
    node.cnc_agent.tools = [node.draw_shape]
    
    print("\n--- CNC AGENT ONLINE ---")
    
    while rclpy.ok():
        try:
            user_input = await asyncio.to_thread(input, "User: ")
            if user_input.lower() in ["exit", "quit"]: break
            
            response = await Runner.run(
                node.cnc_agent,
                input=user_input,
            )
            print(f"Agent: {response.final_output}")
            
        except KeyboardInterrupt:
            break
            
    node.destroy_node()
    rclpy.shutdown()

def main():
    asyncio.run(run_node())

if __name__ == "__main__":
    main()