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
import shutil
import tempfile

from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def _make_robot_def(i, y_pos):
    """Return a Webots VRML CustomBot block for robot number i at the given y position."""
    return (
        f'CustomBot {{\n'
        f'  name "robot{i}"\n'
        f'  translation 6.36 {y_pos} 0\n'
        f'  controller "<extern>"\n'
        f'  controllerArgs [\n'
        f'    ""\n'
        f'  ]\n'
        f'  extensionSlot [\n'
        f'    Solid {{\n'
        f'      name "imu_link"\n'
        f'    }}\n'
        f'    GPS {{\n'
        f'    }}\n'
        f'    InertialUnit {{\n'
        f'      name "inertial_unit"\n'
        f'    }}\n'
        f'    RobotisLds01 {{\n'
        f'    }}\n'
        f'  ]\n'
        f'}}'
    )


def launch_setup(context, *args, **kwargs):
    num_robots = int(context.launch_configurations['num_robots'])
    world_name = context.launch_configurations['world']
    mode = context.launch_configurations['mode']
    use_nav = context.launch_configurations['nav'].lower() == 'true'
    use_slam = context.launch_configurations['slam'].lower() == 'true'
    use_sim_time_str = context.launch_configurations['use_sim_time']
    use_sim_time = use_sim_time_str.lower() == 'true'

    package_dir = get_package_share_directory('webots_diffdrive')

    # Generate world file: inject N robots spaced 0.5 m apart on the y-axis
    world_template = os.path.join(package_dir, 'worlds', world_name)
    with open(world_template) as f:
        world_content = f.read()

    robot_defs = [
        _make_robot_def(i, (i - 1) * 0.5)
        for i in range(1, num_robots + 1)
    ]
    world_content = world_content.replace('# ROBOT_DEFINITIONS', '\n'.join(robot_defs))

    # Write the world file into a temp directory and copy any local proto files
    # alongside it so that relative EXTERNPROTO paths resolve correctly.
    worlds_dir = os.path.join(package_dir, 'worlds')
    tmp_world_dir = tempfile.mkdtemp(prefix='webots_world_')
    for proto in os.listdir(worlds_dir):
        if proto.endswith('.proto'):
            shutil.copy(os.path.join(worlds_dir, proto), tmp_world_dir)
    meshes_src = os.path.join(worlds_dir, 'meshes')
    if os.path.isdir(meshes_src):
        shutil.copytree(meshes_src, os.path.join(tmp_world_dir, 'meshes'))

    tmp_world_path = os.path.join(tmp_world_dir, world_name)
    with open(tmp_world_path, 'w') as f:
        f.write(world_content)

    webots = WebotsLauncher(
        world=tmp_world_path,
        mode=mode,
        ros2_supervisor=True,
    )

    # Per-robot nodes
    robot_state_publishers = []
    footprint_publishers = []
    lidar_publishers = []
    turtlebot_drivers = []
    waiting_nodes = []

    use_twist_stamped = 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] in ['rolling', 'jazzy']
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    urdf_template = os.path.join(package_dir, 'resource', 'custom_bot.urdf')
    ros2ctrl_template = os.path.join(package_dir, 'resource', 'custom_bot_ros2control.yml')
    nav2_template = os.path.join(package_dir, 'resource', 'custom_bot_nav2_params.yaml')
    nav2_map = os.path.join(package_dir, 'resource', 'turtlebot_example_map.yaml')

    for i in range(1, num_robots + 1):
        robot_state_publishers.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=f'robot{i}',
            output='screen',
            parameters=[{'robot_description': '<robot name=""><link name=""/></robot>'}],
            remappings=[('/tf', f'/robot{i}/tf'), ('/tf_static', f'/robot{i}/tf_static')],
        ))

        footprint_publishers.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=f'robot{i}',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', f'/robot{i}/base_link', f'/robot{i}/base_footprint'],
            remappings=[('/tf', f'/robot{i}/tf'), ('/tf_static', f'/robot{i}/tf_static')],
        ))

        lidar_publishers.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=f'robot{i}',
            output='screen',
            arguments=['0', '0', '0.19', '0', '0', '0', f'robot{i}/base_link', f'robot{i}/LDS-01'],
            remappings=[('/tf', f'/robot{i}/tf'), ('/tf_static', f'/robot{i}/tf_static')],
        ))

        ros_control_spawners = [
            Node(
                package='controller_manager',
                executable='spawner',
                name=f'diffdrive_controller_spawner_robot{i}',
                namespace=f'robot{i}',
                output='screen',
                prefix=controller_manager_prefix,
                arguments=['diffdrive_controller'] + controller_manager_timeout,
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                name=f'joint_state_broadcaster_spawner_robot{i}',
                namespace=f'robot{i}',
                output='screen',
                prefix=controller_manager_prefix,
                arguments=['joint_state_broadcaster'] + controller_manager_timeout,
            ),
        ]

        # Generate URDF from single template
        with open(urdf_template) as f:
            urdf_content = f.read().replace('ROBOT_NS', f'robot{i}')
        tmp_urdf = tempfile.NamedTemporaryFile(
            prefix=f'urdf_robot{i}_', suffix='.urdf', delete=False, mode='w')
        tmp_urdf.write(urdf_content)
        robot_description_path = tmp_urdf.name
        tmp_urdf.close()

        # Generate ros2_control params from single template
        with open(ros2ctrl_template) as f:
            ros2ctrl_content = f.read().replace('ROBOT_NS', f'robot{i}')
        tmp_ros2ctrl = tempfile.NamedTemporaryFile(
            prefix=f'ros2ctrl_robot{i}_', suffix='.yml', delete=False, mode='w')
        tmp_ros2ctrl.write(ros2ctrl_content)
        ros2_control_params = tmp_ros2ctrl.name
        tmp_ros2ctrl.close()

        if use_twist_stamped:
            mappings = [
                (f'/robot{i}/diffdrive_controller/cmd_vel', f'/robot{i}/cmd_vel'),
                (f'/robot{i}/diffdrive_controller/odom', f'/robot{i}/odom'),
                ('/tf', f'/robot{i}/tf'),
            ]
        else:
            mappings = [
                (f'/robot{i}/diffdrive_controller/cmd_vel_unstamped', f'/robot{i}/cmd_vel'),
                (f'/robot{i}/diffdrive_controller/odom', f'/robot{i}/odom'),
                ('/tf', f'/robot{i}/tf'),
            ]

        turtlebot_driver = WebotsController(
            robot_name=f'robot{i}',
            namespace=f'robot{i}',
            parameters=[
                {
                    'robot_description': robot_description_path,
                    'use_sim_time': use_sim_time,
                    'set_robot_state_publisher': True,
                    'update_rate': 50,
                },
                ros2_control_params,
            ],
            remappings=mappings,
            respawn=True,
        )
        turtlebot_drivers.append(turtlebot_driver)

        # Generate nav2 params from single template
        with open(nav2_template) as f:
            nav2_content = f.read().replace('ROBOT_NS', f'robot{i}')
        tmp_nav2 = tempfile.NamedTemporaryFile(
            prefix=f'nav2_robot{i}_', suffix='.yaml', delete=False, mode='w')
        tmp_nav2.write(nav2_content)
        nav2_params = tmp_nav2.name
        tmp_nav2.close()

        navigation_nodes = []
        os.environ['TURTLEBOT3_MODEL'] = 'burger'

        if use_nav and 'turtlebot3_navigation2_custom' in get_packages_with_prefixes():
            navigation_nodes.append(GroupAction([
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('turtlebot3_navigation2_custom'),
                        'launch', 'navigation2.launch.py')),
                    launch_arguments=[
                        ('map', nav2_map),
                        ('params_file', nav2_params),
                        ('use_sim_time', use_sim_time_str),
                        ('robot_number', str(i)),
                    ],
                )
            ]))

        if use_slam and 'turtlebot3_cartographer_custom' in get_packages_with_prefixes():
            navigation_nodes.append(GroupAction([
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('turtlebot3_cartographer_custom'),
                        'launch', 'cartographer.launch.py')),
                    launch_arguments=[
                        ('use_sim_time', use_sim_time_str),
                        ('robot_number', str(i)),
                    ],
                )
            ]))

        waiting_nodes.append(WaitForControllerConnection(
            target_driver=turtlebot_driver,
            nodes_to_start=navigation_nodes + ros_control_spawners,
        ))

    return [
        webots,
        webots._supervisor,
        *robot_state_publishers,
        *footprint_publishers,
        *lidar_publishers,
        *turtlebot_drivers,
        *waiting_nodes,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='custom_world.wbt',
            description='World file from the worlds/ directory',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode',
        ),
        DeclareLaunchArgument(
            'num_robots',
            default_value='1',
            description='Number of robots to spawn, spaced 0.5 m apart on the y-axis',
        ),
        DeclareLaunchArgument(
            'nav',
            default_value='false',
            description='Launch Nav2',
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='false',
            description='Launch Cartographer SLAM',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Webots simulation clock',
        ),
        OpaqueFunction(function=launch_setup),
    ])
