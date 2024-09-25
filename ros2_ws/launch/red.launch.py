import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # 子launchファイルのパスを取得
    # child_launch_file_path = os.path.join(
    #     get_package_share_directory('realsense2_camera'),
    #     'launch',
    #     'rs_launch.py'
    # )
    # パラメータファイルのパスを取得
    path_can_sender = os.path.expanduser('~/NHKRobocon2024B/ros2_ws/config/htmd_can_sender.yaml')
    path_udp_receiver = os.path.expanduser('~/NHKRobocon2024B/ros2_ws/config/htmd_can_receiver.yaml')
    path_robot1_red = os.path.expanduser('~/NHKRobocon2024B/ros2_ws/config/robot1_navi.yaml')
    return LaunchDescription([
        Node(
            package="robot1",
            executable="robot1_key"
        ),
        Node(
            package="robot1",
            executable="robot1_navi",
            parameters=[path_robot1_red]
        ),
        Node(
            package="robot1",
            executable="robot1_auto",
        ),
        Node(
            package="peripheral",
            executable="solenoid_driver",
        ),
        Node(
            package="htmd_manager",
            executable="htmd_udp_sender",
            parameters=[path_can_sender]
        ),
        # Node(
        #     package="htmd_manager",
        #     executable="htmd_udp_receiver",
        #     parameters=[path_udp_receiver]
        # ),
        Node(
            package="peripheral",
            executable="robot1_sensor"
        )
        # # 子launchファイルを含める
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(child_launch_file_path)
        # ),
    ])