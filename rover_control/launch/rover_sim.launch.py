import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_control')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'rover_world.sdf'),
        description='Path to the world SDF file',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Launch RViz2'
    )
    safety_arg = DeclareLaunchArgument(
        'safety', default_value='false',
        description='Route cmd_vel through lidar_safety_node. When true, '
                    'point your teleop/nav source at /cmd_vel_teleop instead '
                    'of /cmd_vel.',
    )
    slam_arg = DeclareLaunchArgument(
        'slam', default_value='false',
        description='Start slam_toolbox (async, online mapping) alongside the sim.',
    )
    autonomy_arg = DeclareLaunchArgument(
        'autonomy', default_value='false',
        description='Start the autonomous_decision_maker FSM (high-level '
                    'explore / avoid / return-home supervisor). Needs Nav2 '
                    '(and usually slam:=true or a saved map) already running '
                    'for EXPLORE/RETURN_HOME to actually go anywhere - '
                    'AVOID_OBSTACLE and EMERGENCY_STOP work regardless.',
    )

    xacro_file = os.path.join(pkg_share, 'urdf', 'rover.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Gazebo Harmonic server + GUI. "-r" runs immediately, "-v 3" gives useful
    # log output while you're still debugging the lidar / bridge.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [LaunchConfiguration('world'), ' -r -v 3']}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
    )

    # Spawn from the /robot_description TOPIC (not a file path). This is the
    # deliberate fix from the earlier $(find ...) substitution problem: xacro
    # has already fully expanded the URDF by the time it hits this topic, so
    # gz-sim never needs to resolve any ROS package paths itself.
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
            'config_file': os.path.join(pkg_share, 'config', 'rover_bridge.yaml'),
            'use_sim_time': True,
        }],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'rover.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    lidar_safety = Node(
        package='rover_control',
        executable='lidar_safety_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_cmd_topic': '/cmd_vel_teleop',
            'output_cmd_topic': '/cmd_vel',
        }],
        condition=IfCondition(LaunchConfiguration('safety')),
    )

    # Plain Node (not slam_toolbox's own LifecycleNode-based launch file) -
    # simpler to reason about here, no manual configure/activate transitions
    # needed, and it picks up config/slam_params.yaml as-is.
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'slam_params.yaml'),
            {'use_sim_time': True},
        ],
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    autonomous_decision_maker = Node(
        package='rover_control',
        executable='autonomous_decision_maker',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('autonomy')),
    )

    return LaunchDescription([
        world_arg,
        rviz_arg,
        safety_arg,
        slam_arg,
        autonomy_arg,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        rviz,
        lidar_safety,
        slam_toolbox,
        autonomous_decision_maker,
    ])
