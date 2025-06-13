# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo')
    pkg_project_description = get_package_share_directory('ros_gz_example_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # load the sdf file from "description" package - using ackerman model
    sdf_file = os.path.join(pkg_project_description, 'models', 'limo_ackerman1', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # load the sdf file from "description" package - using ackerman2 model
    sdf_file = os.path.join(pkg_project_description, 'models', 'limo_ackerman2', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc2 = infp.read()

    # setup to launch the simulator and gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'ackerman_drive_generated.sdf'
        ])}.items(),
    )

    # generate the gazebo world file
    generate_simulation_world = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(pkg_project_gazebo, 'scripts', 'generate_simulation_world.py'),
            '--sdf-source',
            os.path.join(pkg_project_gazebo, 'worlds', 'ackerman_world_source.sdf'),
            '--sdf-dest',
            os.path.join(pkg_project_gazebo, 'worlds', 'ackerman_drive_generated.sdf'),
        ],
        output='screen'
    )

    robot_state_publisher1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher1',
        namespace='limo1',
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
        namespace='limo2',
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

    identifier_node = Node(
        package='ros_gz_example_application',
        executable='identifier_command',
        name='identifier_command',
        output='screen',
    )

    debug_log_limo1 = Node(
        package='debug_log',
        executable='debug_log',
        name='debug_log1',
        namespace='limo1',
        output='screen',
    )

    debug_log_limo2 = Node(
        package='debug_log',
        executable='debug_log',
        name='debug_log2',
        namespace='limo2',
        output='screen',
    )

    websocket_bridge_node = Node(
        package='websocket_bridge',
        executable='websocket_ros_bridge',
        name='websocket_bridge',
        output='screen',
    )

    mission = Node(
        package='mission',
        executable='mission',
        name='mission',
        output='screen',
    )

    mission_state = Node(
    package='mission_state',  # Remplace par le bon package si différent
        executable='mission_state',  # Le nom défini dans setup.py
        name='mission_state',
        output='screen',
    )

    map_merger_node = Node(
        package='map_merger',
        executable='map_merger',
        name='map_merger',
        output='screen'
    )

    random_nav_limo1 = Node(
        package='random_walk',
        executable='random_walk',
        name='random_walk1',
        namespace='limo1',
        output='screen',
        parameters=[{'enable_ackerman': True}]
    )

    random_nav_limo2 = Node(
        package='random_walk',
        executable='random_walk',
        name='random_walk2',
        namespace='limo2',
        output='screen',
        parameters=[{'enable_ackerman': True}]
    )

    return LaunchDescription([
        generate_simulation_world,  # needs to be executed before gazebo
        gz_sim,
        bridge,
        robot_state_publisher1,
        robot_state_publisher2,
        identifier_node,
        websocket_bridge_node,
        mission,
        debug_log_limo1,
        debug_log_limo2,
        random_nav_limo1,
        random_nav_limo2,
        mission_state,
        map_merger_node
    ])