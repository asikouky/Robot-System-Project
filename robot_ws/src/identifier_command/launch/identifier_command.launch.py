from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'topic_prefix',
            default_value='robot1_104',
            description='Préfixe des topics du robot'
        ),
        DeclareLaunchArgument(
            'cmd_vel_remap',
            default_value='/cmd_vel',
            description='Remap vers le vrai /cmd_vel du robot'
        ),

        Node(
            package='identifier_command',
            executable='identifier_command',
            name='identifier_node',
            parameters=[{
                'topic_prefix': LaunchConfiguration('topic_prefix')
            }],
            remappings=[
                (PathJoinSubstitution([
                    TextSubstitution(text='/'),
                    LaunchConfiguration('topic_prefix'),
                    TextSubstitution(text='cmd_vel')
                ]), LaunchConfiguration('cmd_vel_remap'))
            ]
        )
    ])
