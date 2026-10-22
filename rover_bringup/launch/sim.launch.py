"""Gazebo side of the sim: world, robot spawn, ROS <-> gz bridge.

Included by rover.launch.py, which also runs
robot_state_publisher - this spawns from its /robot_description topic.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('rover_bringup')

    world_arg = DeclareLaunchArgument(
        'world', default_value=os.path.join(bringup, 'worlds', 'rover_world.sdf'))

    # "-r" runs immediately, "-v 3" gives useful log output.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [LaunchConfiguration('world'), ' -r -v 3']}.items(),
    )

    # Spawn from the /robot_description topic rather than a file path: xacro
    # has already expanded it, so gz never has to resolve ROS package paths.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'rover', '-z', '0.17'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(bringup, 'config', 'gz_bridge.yaml'),
            'use_sim_time': True,
        }],
        output='screen',
    )

    return LaunchDescription([world_arg, gz_sim, spawn_robot, bridge])
