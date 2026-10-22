"""The one entry point for the single rover in Gazebo.

    ros2 launch rover_bringup rover.launch.py                  # drive only
    ... slam:=true nav:=true autonomy:=true                     # add the stack
    ... foxglove:=true      # Foxglove bridge on ws://localhost:8765 (layout: foxglove/rover.json)

What always runs:
    robot_state_publisher      URDF -> /tf_static, /robot_description
    sim.launch.py              Gazebo: world, robot, drive + lidar (/odom, /tf, /scan)
    twist_mux + lidar_safety_node    the velocity command pipeline:

        /autonomy/cmd_vel (FSM, prio 100) ┐
        /cmd_vel          (teleop, 50)    ├─ twist_mux -> /cmd_vel_mux
        /nav2/cmd_vel     (Nav2, 10)      ┘
            -> lidar_safety_node -> /cmd_vel_safe -> drive

Every node runs on Gazebo's clock (use_sim_time).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('rover_bringup')
    description = get_package_share_directory('rover_description')

    use_sim_time = True

    args = [
        DeclareLaunchArgument('slam', default_value='false',
                              description='Run slam_toolbox (online mapping, provides map->odom).'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='Run Nav2. Needs slam:=true for the map frame.'),
        DeclareLaunchArgument('autonomy', default_value='false',
                              description='Run the mission FSM. Needs nav:=true to go anywhere.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        DeclareLaunchArgument('foxglove', default_value='false',
                              description='Run foxglove_bridge (ws://<host>:8765) for Foxglove.'),
        DeclareLaunchArgument('world',
                              default_value=os.path.join(bringup, 'worlds', 'rover_world.sdf'),
                              description='World SDF file.'),
    ]

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(description, 'urdf', 'rover.urdf.xacro')]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        parameters=[os.path.join(bringup, 'config', 'twist_mux.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', 'cmd_vel_mux')],
    )

    lidar_safety = Node(
        package='rover_autonomy',
        executable='lidar_safety_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # slam_toolbox is a lifecycle node on Jazzy; its own launch file does the
    # configure/activate transitions (a plain Node would sit unconfigured and
    # never publish a map).
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(bringup, 'config', 'slam.yaml'),
            'use_sim_time': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'nav.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('nav')),
    )

    autonomy = Node(
        package='rover_autonomy',
        executable='autonomous_decision_maker',
        output='screen',
        parameters=[os.path.join(bringup, 'config', 'autonomy.yaml'),
                    {'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('autonomy')),
    )

    # View from `map` when SLAM runs (the frame fixed to the world - odom is
    # allowed to drift), otherwise from `odom`, since `map` won't exist.
    fixed_frame = PythonExpression(
        ["'map' if '", LaunchConfiguration('slam'), "'.lower() == 'true' else 'odom'"])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(bringup, 'rviz', 'rover.rviz'), '-f', fixed_frame],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Lets Foxglove (desktop or browser) see every topic, publish teleop and
    # goals, and call the /autonomy services. Import foxglove/rover.json.
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        output='screen',
        parameters=[{'port': 8765, 'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('foxglove')),
    )

    return LaunchDescription(args + [
        robot_state_publisher,
        sim_launch,
        twist_mux,
        lidar_safety,
        slam,
        nav2,
        autonomy,
        rviz,
        foxglove,
    ])
