from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='p2p_comm',
            executable='p2p_node',
            name='p2p_node',
            output='screen',
            env={
                'ROBOT_NAME': 'robot1',
                'OTHER_ROBOT_NAME': 'robot2'
            }
        ),
    ])
