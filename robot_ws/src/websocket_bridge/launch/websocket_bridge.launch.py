from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='websocket_bridge',
            executable='websocket_ros_bridge',
            name='websocket_bridge',
            output='screen'
        ),
    ])
