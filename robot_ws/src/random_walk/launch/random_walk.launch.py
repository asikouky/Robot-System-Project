from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='random_walk',
            executable='random_walk',
            name='random_walk',
            output='screen'
        ),
    ])
