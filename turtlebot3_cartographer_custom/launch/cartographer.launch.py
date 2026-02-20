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


def launch_setup(context, *args, **kwargs):
    robot_number = context.launch_configurations['robot_number']
    use_sim_time = context.launch_configurations['use_sim_time']
    use_rviz = context.launch_configurations['use_rviz']
    resolution = context.launch_configurations['resolution']
    publish_period_sec = context.launch_configurations['publish_period_sec']

    robot_ns = f'robot{robot_number}'
    pkg_dir = get_package_share_directory('turtlebot3_cartographer_custom')

    # Generate Lua config from single template
    lua_template = os.path.join(pkg_dir, 'config', 'turtlebot3_lds_2d.lua')
    with open(lua_template) as f:
        lua_content = f.read().replace('ROBOT_NS', robot_ns)
    tmp_lua = tempfile.NamedTemporaryFile(
        prefix=f'cartographer_{robot_ns}_', suffix='.lua', delete=False, mode='w')
    tmp_lua.write(lua_content)
    lua_path = tmp_lua.name
    tmp_lua.close()

    # Generate RViz config from single template
    rviz_template = os.path.join(pkg_dir, 'rviz', 'tb3_cartographer.rviz')
    with open(rviz_template) as f:
        rviz_content = f.read().replace('ROBOT_NS', robot_ns)
    tmp_rviz = tempfile.NamedTemporaryFile(
        prefix=f'cartographer_{robot_ns}_', suffix='.rviz', delete=False, mode='w')
    tmp_rviz.write(rviz_content)
    rviz_path = tmp_rviz.name
    tmp_rviz.close()

    use_sim_time_bool = use_sim_time.lower() == 'true'

    nodes = [
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            namespace=robot_ns,
            remappings=[
                ('/tf', f'/{robot_ns}/tf'),
                ('/tf_static', f'/{robot_ns}/tf_static'),
                ('scan', f'/{robot_ns}/scan'),
            ],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_bool}],
            arguments=['-configuration_directory', os.path.dirname(lua_path),
                       '-configuration_basename', os.path.basename(lua_path)],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(os.path.dirname(__file__), 'occupancy_grid.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec,
                'robot_number': robot_number,
            }.items(),
        ),
    ]

    if use_rviz.lower() == 'true':
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
            parameters=[{'use_sim_time': use_sim_time_bool}],
            remappings=[
                ('/tf', f'/{robot_ns}/tf'),
                ('/tf_static', f'/{robot_ns}/tf_static'),
            ],
            output='screen',
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_number',
            default_value='1',
            description='robot integer number'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz'),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Resolution of a grid cell in the published occupancy grid'),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='OccupancyGrid publishing period'),
        OpaqueFunction(function=launch_setup),
    ])
