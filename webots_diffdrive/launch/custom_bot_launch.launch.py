#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
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

"""Launch Webots TurtleBot3 Burger driver."""

import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():
    package_dir = get_package_share_directory('webots_diffdrive')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam = LaunchConfiguration('slam', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )

    robot_state_publishers = []
    footprint_publishers = []
    lidar_publishers = []
    turtlebot_drivers = []
    waiting_nodes = []
    for i in range(1,3):

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=f'robot{i}',
            output='screen',
            parameters=[{
                'robot_description': '<robot name=""><link name=""/></robot>'
            }],
            remappings=[('/tf', f'/robot{i}/tf'), ('/tf_static', f'/robot{i}/tf_static')],
        )
        robot_state_publishers.append(robot_state_publisher)

        footprint_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=f'robot{i}',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', f'robot{i}/base_link', f'robot{i}/base_footprint'],
            remappings=[('/tf', f'/robot{i}/tf'), ('/tf_static', f'/robot{i}/tf_static')],
        )
        footprint_publishers.append(footprint_publisher)

        lidar_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=f'robot{i}',
            output='screen',
            arguments=['0', '0', '0.19', '0', '0', '0', f'robot{i}/base_link', f'robot{i}/LDS-01'],
            remappings=[('/tf', f'/robot{i}/tf'), ('/tf_static', f'/robot{i}/tf_static')],
        )
        lidar_publishers.append(lidar_publisher)

        # ROS control spawners
        controller_manager_timeout = ['--controller-manager-timeout', '50']
        controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
        diffdrive_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            name=f'diffdrive_controller_spawner_robot{i}',
            namespace=f'robot{i}',
            output='screen',
            prefix=controller_manager_prefix,
            arguments=['diffdrive_controller'] + controller_manager_timeout,
        )
        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            name=f'joint_state_broadcaster_spawner_robot{i}',
            namespace=f'robot{i}',
            output='screen',
            prefix=controller_manager_prefix,
            arguments=['joint_state_broadcaster'] + controller_manager_timeout,
        )
        ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

        robot_description_path = os.path.join(package_dir, 'resource', f'custom_bot_{i}.urdf')
        ros2_control_params = os.path.join(package_dir, 'resource', f'custom_bot_{i}_ros2control.yml')
        use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy'])
        if use_twist_stamped:
            mappings = [('/diffdrive_controller/cmd_vel', f'/robot{i}/cmd_vel'), ('/diffdrive_controller/odom', f'/robot{i}/odom'), ('/tf', f'/robot{i}/tf')]
        else:
            mappings = [('/diffdrive_controller/cmd_vel_unstamped', f'/robot{i}/cmd_vel'), ('/diffdrive_controller/odom', f'/robot{i}/odom'), ('/tf', f'/robot{i}/tf')]
        turtlebot_driver = WebotsController(
        robot_name=f'robot{i}',
        namespace=f'robot{i}',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True,
             'update_rate': 50},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
        )
        turtlebot_drivers.append(turtlebot_driver)

        # Navigation
        navigation_nodes = []
        os.environ['TURTLEBOT3_MODEL'] = 'burger'
        nav2_map = os.path.join(package_dir, 'resource', 'turtlebot_example_map.yaml')
        nav2_params = os.path.join(package_dir, 'resource', f'custom_bot_{i}_nav2_params.yaml')
        if 'turtlebot3_navigation2_custom' in get_packages_with_prefixes():
            turtlebot_navigation = GroupAction([
                PushRosNamespace(f'robot{i}'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('turtlebot3_navigation2_custom'), 'launch', 'navigation2.launch.py')),
                    launch_arguments=[
                        ('map', nav2_map),
                        ('params_file', nav2_params),
                        ('use_sim_time', use_sim_time),
                        ('robot_number', i)
                    ],
                    condition=launch.conditions.IfCondition(use_nav))
            ])
            navigation_nodes.append(turtlebot_navigation)

        # SLAM
        if 'turtlebot3_cartographer_custom' in get_packages_with_prefixes():
            turtlebot_slam = GroupAction([
                PushRosNamespace(f'robot{i}'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('turtlebot3_cartographer_custom'), 'launch', 'cartographer.launch.py')),
                    launch_arguments=[
                        ('use_sim_time', use_sim_time),
                        ('robot_number', i)
                    ],
                    condition=launch.conditions.IfCondition(use_slam))
            ])
            navigation_nodes.append(turtlebot_slam)

        # Wait for the simulation to be ready to start navigation nodes
        waiting_node = WaitForControllerConnection(
            target_driver=turtlebot_driver,
            nodes_to_start=navigation_nodes + ros_control_spawners
        )
        waiting_nodes.append(waiting_node)

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='custom_world.wbt',
            description='Choose one of the world files from `/webots_ros2_turtlebot/world` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        webots,
        webots._supervisor,

        *robot_state_publishers,
        *footprint_publishers,
        *lidar_publishers,

        *turtlebot_drivers,
        *waiting_nodes,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),
    ])