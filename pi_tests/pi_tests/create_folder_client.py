#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from pi_interfaces.srv import CreateFolder

class CreateFolderClient(Node):
    def __init__(self):
        super().__init__('create_folder_client')
        self.folder_client = self.create_client(
            CreateFolder,
            "create_folder"
        )

        while not self.folder_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    
    def create_folder_request(self, folder_name):
        req = CreateFolder.Request()
        req.folder_name = folder_name

        future = self.folder_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = CreateFolderClient()
    future_result = node.create_folder_request("my_test")
    print(f"success: {future_result.success}, message:{future_result.message}")
    node.destroy_node()
    rclpy.shutdown()
    