from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    grbl_driver = Node(
        package="pi_firmware", 
        executable="grbl_driver.py", 
        output="screen"
    )

    cnc_description = IncludeLaunchDescription([
        PathJoinSubstitution([
            FindPackageShare("pi_description"),
            "launch",
            "cnc.launch.py"
        ])
    ])

    joy = IncludeLaunchDescription([
        PathJoinSubstitution([
            FindPackageShare("pi_control"), 
            "launch", 
            "joy.launch.py"
        ])
    ])


    


    return LaunchDescription([
        grbl_driver, 
        cnc_description, 
        joy
    ])