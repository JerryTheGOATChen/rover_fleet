import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hw_share = get_package_share_directory('rover_hardware')
    control_share = get_package_share_directory('rover_control')

    xacro_file = os.path.join(hw_share, 'urdf', 'rover_real.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    controllers_yaml = os.path.join(control_share, 'config', 'controllers.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False, 'robot_description': robot_description}],
    )

    # controller_manager loads the RoverSystemHardware plugin named in the
    # URDF's <ros2_control> block - robot_description is passed here too so
    # it doesn't need a separate parameter file just to see the same URDF.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': False},
            controllers_yaml,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # diff_drive_controller only after joint_state_broadcaster is up - avoids
    # a race where the controller manager hasn't finished discovering state
    # interfaces yet.
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    # controllers.yaml has use_stamped_vel: true, so diff_drive_controller
    # expects TwistStamped on its cmd_vel topic. This converts plain Twist
    # from teleop/Nav2 to that - see src/rover_control_node.cpp for why it
    # exists at all (it's the sim-vs-real difference: the Gazebo DiffDrive
    # plugin took plain Twist, this real ros2_control path needs stamped).
    twist_to_stamped = Node(
        package='rover_control',
        executable='rover_control_node',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/diff_drive_controller/cmd_vel',
            'frame_id': 'body_link',
        }],
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        delayed_diff_drive_spawner,
        twist_to_stamped,
    ])
