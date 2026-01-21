#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pi_interfaces.srv import CreateFolder
import os

class CreateFolderServer(Node):
    def __init__(self):
        super().__init__("create_folder_server")
        self.folder_server = self.create_service(
            CreateFolder, 
            "create_folder",
            self.create_folder_callback
        )

        self.get_logger().info("Create Folder Service is ready.")

    def create_folder_callback(self, request, response):
        folder_name = request.folder_name
        try: 
            os.mkdir(folder_name)
            response.success = True 
            response.message = "Folder created successfully"
            self.get_logger().info(f"Folder {folder_name} created successfully.")
        except FileExistsError:
            response.success = False
            response.message = "Folder already exists"
            self.get_logger().error(f"Folder already exists: {folder_name}")

        return response

def main(args=None):
    rclpy.init(args=args)
    node = CreateFolderServer()
    rclpy.spin(node)
    rclpy.shutdown()