import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_control')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/maps/rover_room.yaml'),
        description='Path to a saved map .yaml (from map_saver_cli).',
    )

    # map_server is a lifecycle node - it won't publish /map until configured
    # and activated. lifecycle_manager below does that automatically. Nothing
    # else needs to be running for this: no sim, no SLAM, no Nav2 bringup.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': False}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'view_map.rviz')],
        output='screen',
    )

    return LaunchDescription([map_arg, map_server, lifecycle_manager, rviz])
