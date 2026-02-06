from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
import os 
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy = Node(
        package="joy", 
        executable="joy_node",
        name="joy_node",
        output="screen"
    )
    

    teleop_joy = Node(
        package= "pi_control", 
        executable="teleop_joy",
        name="teleop_joy",
        output="screen"
    )

    return LaunchDescription([
        joy, 
        teleop_joy
    ])