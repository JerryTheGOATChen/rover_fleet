"""A fleet of rovers exploring the Gazebo house over a short-range radio.

    ros2 launch rover_bringup fleet.launch.py                          # 3 robots, learned policy
    ros2 launch rover_bringup fleet.launch.py policy:=greedy robots:=2 comm_range:=8.0
    ros2 launch rover_bringup fleet.launch.py gui:=false foxglove:=true # headless + Foxglove

Per robot, in namespace robotN with frames robotN/... on one shared /tf:
    Gazebo model + bridge, robot_state_publisher, static world -> robotN/map
    (its spawn pose), twist_mux + lidar_safety_node, slam_toolbox, Nav2, and
    rover_fleet's fleet_agent (the policy).
Shared:
    Gazebo, the clock bridge (throttled to 100 Hz), rover_fleet's comms_sim
    (radio + base station), and optionally RViz / Foxglove.

The single-robot stack (rover.launch.py) is unchanged; this reuses its
config files, rewritten per robot (rover_fleet/launch_utils.py).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition

from fleet_rl.houses import GAZEBO_BASE, GAZEBO_SPAWNS, load_sdf_house
from rover_fleet.launch_utils import namespaced_params

SECONDS_BETWEEN_ROBOTS = 4.0   # stagger bring-up so WSL isn't hit by everything at once


def robot(name, index, x, y, args, bringup, description, grid):
    """Everything one robot runs, in namespace `name`."""
    config = os.path.join(bringup, 'config')
    sim_time = {'use_sim_time': True}

    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', namespace=name,
        parameters=[{'robot_description': ParameterValue(Command([
            'xacro ', os.path.join(description, 'urdf', 'rover.urdf.xacro'),
            ' ns:=', name, ' odometry:=', args['odometry']]), value_type=str),
            'frame_prefix': f'{name}/', **sim_time}])
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', f'/{name}/robot_description', '-name', name,
                   '-x', str(x), '-y', str(y), '-z', '0.17'])
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', namespace=name,
        parameters=[{'config_file': os.path.join(config, 'gz_bridge_fleet.yaml'),
                     'expand_gz_topic_names': True, **sim_time}])
    # SLAM's map frame starts at the robot's odom frame. With odometry:=truth
    # that is the world itself; with wheel odometry it is where the robot spawned.
    truth = args['odometry'] == 'truth'
    world_to_map = Node(
        package='tf2_ros', executable='static_transform_publisher', namespace=name,
        arguments=['--x', '0' if truth else str(x), '--y', '0' if truth else str(y),
                   '--frame-id', 'world', '--child-frame-id', f'{name}/map'])
    twist_mux = Node(
        package='twist_mux', executable='twist_mux', namespace=name,
        parameters=[namespaced_params(os.path.join(config, 'twist_mux.yaml'), name), sim_time],
        remappings=[('cmd_vel_out', 'cmd_vel_mux')])
    safety = Node(package='rover_autonomy', executable='lidar_safety_node', namespace=name,
                  parameters=[sim_time])
    # Wheel joint states for robot_state_publisher (only the wheels' drawn angle
    # depends on them) at 10 Hz, instead of Gazebo's ~1 kHz per robot.
    wheels = Node(package='joint_state_publisher', executable='joint_state_publisher',
                  namespace=name, parameters=[{'rate': 10, **sim_time}])

    # slam_toolbox is a lifecycle node: configure it, then activate it once
    # configured (the same thing slam_toolbox's own launch file does).
    # Add a scan every 0.2 m / 0.2 rad (single robot: 0.5), so a robot's first
    # map grows as soon as it starts moving.
    slam_fleet = {'slam_toolbox': {'ros__parameters': {
        'minimum_travel_distance': 0.2, 'minimum_travel_heading': 0.2}}}
    slam = LifecycleNode(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
        namespace=name, output='screen',
        parameters=[namespaced_params(os.path.join(config, 'slam.yaml'), name, slam_fleet),
                    sim_time])
    slam_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam), transition_id=Transition.TRANSITION_CONFIGURE))
    slam_activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_ACTIVATE))]))

    # Same Nav2 config as the single robot, plus two fleet-only changes:
    # - the fleet's behavior tree (no spin-in-place recovery; see
    #   behavior_trees/fleet_navigate_to_pose.xml)
    # - teammates as obstacles in the local costmap: fleet_agent publishes the
    #   teammates it hears from as a point cloud. Local only: it is the costmap
    #   that steers around moving things, and in the global costmap the marks
    #   outlived the robots (scans that would clear them get dropped whenever
    #   SLAM's transform lags), walling robots in where teammates had stood.
    teammates = {'obstacle_layer': {
        'observation_sources': 'scan teammates',
        'teammates': {'topic': f'/{name}/fleet/teammates', 'data_type': 'PointCloud2',
                      'marking': True, 'clearing': False, 'min_obstacle_height': 0.0,
                      'max_obstacle_height': 2.0, 'obstacle_min_range': 0.0,
                      'obstacle_max_range': 50.0}}}
    # - BackUp without the costmap look-ahead (simulate_ahead_time 0): a robot
    #   wedged against a door post or a teammate already overlaps the inflated
    #   obstacle, so the check refused every back-up, the one move that frees it.
    #   It only ever reverses 0.3 m at 0.15 m/s, into space it just drove through.
    fleet_nav = {
        'bt_navigator': {'ros__parameters': {'default_nav_to_pose_bt_xml': os.path.join(
            bringup, 'behavior_trees', 'fleet_navigate_to_pose.xml')}},
        'local_costmap': {'local_costmap': {'ros__parameters': teammates}},
        'behavior_server': {'ros__parameters': {'simulate_ahead_time': 0.0}},
    }
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'nav.launch.py')),
        launch_arguments={'use_sim_time': 'true', 'namespace': name,
                          'params_file': namespaced_params(os.path.join(config, 'nav2.yaml'),
                                                           name, fleet_nav)}.items())

    agent = Node(
        package='rover_fleet', executable='fleet_agent', namespace=name, output='screen',
        parameters=[{**sim_time, **grid,
                     'robots': args['names'], 'policy': args['policy'], 'model': args['model'],
                     'comm_range': args['comm_range'], 'wall_penalty': args['wall_penalty'],
                     'beacons': args['beacons'], 'mission_seconds': args['mission_seconds'],
                     'base_x': GAZEBO_BASE[0], 'base_y': GAZEBO_BASE[1],
                     'home_x': float(x), 'home_y': float(y)}])

    actions = [state_publisher, wheels, spawn, bridge, world_to_map, twist_mux, safety,
               slam, slam_activate, slam_configure, nav2, agent]
    return [TimerAction(period=2.0 + SECONDS_BETWEEN_ROBOTS * index, actions=actions)]


def launch_setup(context):
    bringup = get_package_share_directory('rover_bringup')
    description = get_package_share_directory('rover_description')
    world = LaunchConfiguration('world').perform(context)
    n = int(LaunchConfiguration('robots').perform(context))
    if not 1 <= n <= len(GAZEBO_SPAWNS):
        raise RuntimeError(f'robots must be 1..{len(GAZEBO_SPAWNS)}')
    args = {
        'names': [f'robot{i + 1}' for i in range(n)],
        'policy': LaunchConfiguration('policy').perform(context),
        'model': LaunchConfiguration('model').perform(context),
        'comm_range': float(LaunchConfiguration('comm_range').perform(context)),
        'wall_penalty': float(LaunchConfiguration('wall_penalty').perform(context)),
        'beacons': int(LaunchConfiguration('beacons').perform(context)),
        'mission_seconds': float(LaunchConfiguration('mission_seconds').perform(context)),
        'odometry': LaunchConfiguration('odometry').perform(context),
    }
    # The fleet's shared world grid. Agents only get its extent - never the walls.
    house = load_sdf_house(world, base_xy=GAZEBO_BASE)
    grid = {'grid_origin_x': float(house.origin[0]), 'grid_origin_y': float(house.origin[1]),
            'grid_width': int(house.shape[1]), 'grid_height': int(house.shape[0]),
            'cell': float(house.cell)}

    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{world} -r -v 2' + ('' if gui else ' -s'),
                          'on_exit_shutdown': 'true'}.items())
    # Gazebo publishes the clock every physics step (1 kHz). Every sim-time node
    # in the fleet (~45 of them) would wake up for each one, so only one node
    # sees the raw clock and republishes it at 100 Hz.
    clock_hz = float(LaunchConfiguration('clock_hz').perform(context))
    clock = Node(package='ros_gz_bridge', executable='parameter_bridge',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                 remappings=[('/clock', '/clock_gz')] if clock_hz > 0 else [])
    clock_throttle = Node(package='topic_tools', executable='throttle',
                          arguments=['messages', '/clock_gz', str(clock_hz), '/clock'],
                          condition=IfCondition(str(clock_hz > 0)))
    comms = Node(
        package='rover_fleet', executable='comms_sim', output='screen',
        parameters=[{'use_sim_time': True, 'robots': args['names'], 'world': world,
                     'base_x': GAZEBO_BASE[0], 'base_y': GAZEBO_BASE[1],
                     'comm_range': args['comm_range'], 'wall_penalty': args['wall_penalty']}])

    actions = [gazebo, clock, clock_throttle, comms]
    for i, name in enumerate(args['names']):
        x, y = GAZEBO_SPAWNS[i]
        actions += robot(name, i, x, y, args, bringup, description, grid)
    return actions


def generate_launch_description():
    bringup = get_package_share_directory('rover_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='3', description='Fleet size, 1-6.'),
        DeclareLaunchArgument('policy', default_value='learned',
                              description='learned | greedy | return | breadcrumb'),
        DeclareLaunchArgument('model', default_value='',
                              description='Policy weights (.npz); empty = the trained model '
                                          'shipped in fleet_rl/models.'),
        DeclareLaunchArgument('comm_range', default_value='5.0',
                              description='Radio range in open space (m).'),
        DeclareLaunchArgument('wall_penalty', default_value='3.0',
                              description='Radio range lost per wall (m).'),
        DeclareLaunchArgument('beacons', default_value='0',
                              description='Relay beacons each robot carries.'),
        DeclareLaunchArgument('mission_seconds', default_value='240.0',
                              description='Mission length; robots go home after it.'),
        DeclareLaunchArgument('odometry', default_value='truth',
                              description='truth: Gazebo\'s exact pose as odometry. wheels: '
                                          'the skid-steer wheel odometry (drifts a lot).'),
        DeclareLaunchArgument('clock_hz', default_value='100',
                              description='Republish the sim clock at this rate (0 = raw).'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo window (false = headless server).'),
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz.'),
        DeclareLaunchArgument('foxglove', default_value='false',
                              description='foxglove_bridge on ws://localhost:8765 '
                                          '(layout: foxglove/fleet.json).'),
        DeclareLaunchArgument('world', default_value=os.path.join(
            bringup, 'worlds', 'rover_world.sdf')),
        OpaqueFunction(function=launch_setup),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(bringup, 'rviz', 'fleet.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='foxglove_bridge', executable='foxglove_bridge', output='screen',
             parameters=[{'port': 8765, 'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('foxglove'))),
    ])
