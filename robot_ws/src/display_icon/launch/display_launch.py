# Fichier : robot_ws/src/display_icon/launch/display_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='display_icon',
            executable='display_farthest_node',
            name='display_farthest_node',
            output='screen'
        ),
    ])
