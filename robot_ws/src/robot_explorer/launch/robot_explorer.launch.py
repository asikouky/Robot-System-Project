import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    package_share = get_package_share_directory('robot_explorer')
    rviz_config = os.path.join(package_share, 'rviz', 'default.rviz')

    return LaunchDescription([
        Node(
            package='robot_explorer',
            executable='random_walk_node',
            name='random_walk'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen'
        )
    ])
