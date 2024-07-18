import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='Lrobot_code',
            name='robot_control',
            output='screen',
        ),
        Node(
            package='robot_control',
            executable='robot_control_ui',
            name='robot_control_ui',
            output='screen',
        ),
    ])