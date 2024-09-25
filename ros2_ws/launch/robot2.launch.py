import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package="robot2",
            executable="tfgh",
        ),
        Node(
            package="robot2",
            executable="ijkl",
        ),
        Node(
            package="peripheral",
            executable="robot2_driver",
        ),
    ])