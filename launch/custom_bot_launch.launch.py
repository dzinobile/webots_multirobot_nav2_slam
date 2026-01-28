import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node


def generate_launch_description():
    
    package_dir = get_package_share_directory('webots_diffdrive')
    robot_description_path = os.path.join(package_dir, 'resource', 'custom_robot.urdf')
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'custom_world.wbt')
    )

    robot_drivers = []
    robot_state_publishers = []
    static_tf_publishers = []
    lidar_tf_publishers = []

    for i in range (1,3):
        custom_robot_driver = WebotsController(
            robot_name=f'robot{i}',
            parameters=[
                {'robot_description': robot_description_path},
            ],
            remappings=[
                ('cmd_vel', f'robot{i}/cmd_vel'),
                ('test_vel', f'robot{i}/test_vel')
            ],
            # namespace=f'robot{i}'

        )
        robot_drivers.append(custom_robot_driver)

        # robot_state_pub = Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     namespace=f'robot{i}',
        #     output='screen',
        #     parameters=[{
        #         'robot_description': robot_description_path,
        #         'frame_prefix': f'robot{i}/'
        #     }]
        # )
        # robot_state_publishers.append(robot_state_pub)

        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'world_to_robot{i}',
            arguments=[
                '0', str((i-1)*0.5), '0',
                '0', '0', '0',
                'world',
                f'robot{i}/base_link'
            ]
        )
        static_tf_publishers.append(static_tf)

        lidar_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'robot{i}_to_lidar',
            arguments=['0', '0', '0', '0', '0', '0', f'robot{i}/base_link', 'lidar_frame']
        )
        lidar_tf_publishers.append(lidar_tf)


    return LaunchDescription([
        webots,
        *robot_drivers,
        # *robot_state_publishers,
        *lidar_tf_publishers,
        *static_tf_publishers,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])