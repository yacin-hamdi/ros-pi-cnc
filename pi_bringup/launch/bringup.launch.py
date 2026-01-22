from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    temperature_node = Node(
        package="pi_tests", 
        executable="check_temperature"
    )

    create_folder_server = Node(
        package="pi_tests", 
        executable="create_folder_server"
    )


    return LaunchDescription([
        temperature_node, 
        create_folder_server
    ])