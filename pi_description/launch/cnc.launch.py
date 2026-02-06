from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
import os 
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    robot_description_arg = DeclareLaunchArgument(
        name="robot_description", 
        default_value=os.path.join(get_package_share_directory("pi_description"), "urdf", "cnc.urdf.xacro"),
        description="cnc urdf path"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("robot_description")]), value_type=str)

    cnc_state_publisher = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher", 
        parameters=[{"robot_description": robot_description}]
    )

    return LaunchDescription([
        robot_description_arg, 
        cnc_state_publisher
    ])