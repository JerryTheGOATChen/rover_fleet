"""Nav2, trimmed to what this rover uses. Included by rover.launch.py (nav:=true).

Jazzy's stock nav2_bringup/navigation_launch.py also starts docking, route,
waypoint, smoother, velocity-smoother and collision-monitor servers, none of
which this robot uses - they roughly doubled the topic count and each needed
config just to let bringup succeed. The default navigate_to_pose behavior
tree only needs these four:

    planner_server     global path (NavFn) on the SLAM map
    controller_server  follows it (DWB) -> /nav2/cmd_vel
    behavior_server    Spin / BackUp / Wait recoveries -> /nav2/cmd_vel
    bt_navigator       the navigate_to_pose action the autonomy FSM calls

Acceleration limits are enforced downstream by the drive (gz DiffDrive or
diff_drive_controller), and lidar_safety_node is the one obstacle stop, so
velocity_smoother and collision_monitor added nothing.

fleet.launch.py includes this once per robot with namespace:=robotN and a
params_file rewritten for that robot (rover_fleet/launch_utils.py).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('rover_bringup'), 'config', 'nav2.yaml')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    common = [LaunchConfiguration('params_file'), {'use_sim_time': use_sim_time}]
    to_mux = [('cmd_vel', 'nav2/cmd_vel')]

    servers = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
    executables = {
        'planner_server': ('nav2_planner', []),
        'controller_server': ('nav2_controller', to_mux),
        'behavior_server': ('nav2_behaviors', to_mux),
        'bt_navigator': ('nav2_bt_navigator', []),
    }
    nodes = [
        Node(package=pkg, executable=name, name=name, namespace=namespace, output='screen',
             parameters=common, remappings=remap)
        for name, (pkg, remap) in ((n, executables[n]) for n in servers)
    ]

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': servers}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Robot namespace (fleet only).'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Nav2 parameters (fleet: rewritten per robot).'),
        *nodes,
        lifecycle_manager,
    ])
