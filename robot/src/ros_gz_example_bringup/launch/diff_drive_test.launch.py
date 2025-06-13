#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo')
    pkg_project_description = get_package_share_directory('ros_gz_example_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # load the sdf file for first differential drive robot
    sdf_file = os.path.join(pkg_project_description, 'models', 'limo_diff_drive1', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    # load the sdf file for second differential drive robot
    sdf_file2 = os.path.join(pkg_project_description, 'models', 'limo_diff_drive2', 'model.sdf')
    with open(sdf_file2, 'r') as infp:
        robot_desc2 = infp.read()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'diff_drive_world_source.sdf'
        ])}.items(),
    )

    robot_state_publisher1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher1',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )
    
    robot_state_publisher2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher2',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc2},
        ]
    )

    # bridge ros topics and gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_example_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    test_script_path = os.path.join(pkg_project_bringup, 'scripts', 'test_diffdrive_movement_limo1.py')
    test_script = TimerAction(
        period=10.0,  # wait 10 seconds for the simulation to initialize
        actions=[
            ExecuteProcess(
                cmd=['python3', test_script_path],
                output='screen'
            )
        ]
    )
    
    test_script_path2 = os.path.join(pkg_project_bringup, 'scripts', 'test_diffdrive_movement_limo2.py')
    test_script2 = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', test_script_path2],
                output='screen'
            )
        ]
    )

    launch_components = [
        gz_sim,
        bridge,
        robot_state_publisher1,
        robot_state_publisher2,
        test_script,
        test_script2
    ]

    return LaunchDescription(launch_components)