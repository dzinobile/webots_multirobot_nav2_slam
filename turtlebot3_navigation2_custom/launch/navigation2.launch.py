# Copyright 2019 Open Source Robotics Foundation, Inc.
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
#
# Author: Darby Lim

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO')


def launch_setup(context, *args, **kwargs):
    robot_number = context.launch_configurations['robot_number']
    use_sim_time = context.launch_configurations['use_sim_time']
    params_file = context.launch_configurations['params_file']
    map_dir = context.launch_configurations['map']

    robot_ns = f'robot{robot_number}'
    pkg_dir = get_package_share_directory('turtlebot3_navigation2_custom')

    # If params_file not explicitly provided, generate from single template
    if not params_file:
        if ROS_DISTRO == 'humble':
            template = os.path.join(pkg_dir, 'param', 'humble', 'burger.yaml')
        else:
            template = os.path.join(pkg_dir, 'param', 'burger.yaml')
        with open(template) as f:
            content = f.read().replace('ROBOT_NS', robot_ns)
        tmp = tempfile.NamedTemporaryFile(
            prefix=f'nav2_{robot_ns}_', suffix='.yaml', delete=False, mode='w')
        tmp.write(content)
        params_file = tmp.name
        tmp.close()

    # Generate RViz config from single template
    rviz_template = os.path.join(pkg_dir, 'rviz', 'tb3_navigation2.rviz')
    with open(rviz_template) as f:
        rviz_content = f.read().replace('ROBOT_NS', robot_ns)
    tmp_rviz = tempfile.NamedTemporaryFile(
        prefix=f'nav2_{robot_ns}_', suffix='.rviz', delete=False, mode='w')
    tmp_rviz.write(rviz_content)
    rviz_path = tmp_rviz.name
    tmp_rviz.close()

    use_sim_time_bool = use_sim_time.lower() == 'true'
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # nav2_bringup uses PythonExpression to evaluate 'slam', 'use_composition', etc.
    # These must be Python-compatible ('True'/'False'), not YAML-style ('true'/'false').
    use_sim_time_py = 'True' if use_sim_time_bool else 'False'

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time_py,
                'params_file': params_file,
                'namespace': robot_ns,
                'use_namespace': 'True',
                'slam': 'False',
                'use_composition': 'False',
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=robot_ns,
            arguments=['-d', rviz_path],
            parameters=[{'use_sim_time': use_sim_time_bool}],
            remappings=[
                ('/tf', f'/{robot_ns}/tf'),
                ('/tf_static', f'/{robot_ns}/tf_static'),
            ],
            output='screen'),
    ]


def generate_launch_description():
    default_map = os.path.join(
        get_package_share_directory('turtlebot3_navigation2_custom'), 'map', 'map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Full path to param file to load (auto-generated from robot_number if empty)'),
        DeclareLaunchArgument(
            'robot_number',
            default_value='1',
            description='integer robot number'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        OpaqueFunction(function=launch_setup),
    ])
