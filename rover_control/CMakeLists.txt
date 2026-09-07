cmake_minimum_required(VERSION 3.8)
project(rover_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(lidar_safety_node src/lidar_safety_node.cpp)
ament_target_dependencies(lidar_safety_node rclcpp geometry_msgs sensor_msgs)

# Autonomous decision maker: high-level FSM (IDLE/EXPLORE/AVOID_OBSTACLE/
# RETURN_HOME/EMERGENCY_STOP). Delegates actual driving to Nav2's
# navigate_to_pose action except for the two reactive states (AVOID_OBSTACLE,
# EMERGENCY_STOP), which publish cmd_vel directly.
add_executable(autonomous_decision_maker src/autonomous/autonomous_decision_maker.cpp)
target_include_directories(autonomous_decision_maker PRIVATE include)
ament_target_dependencies(autonomous_decision_maker
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  nav2_msgs
  tf2
  tf2_ros
)

# Twist -> TwistStamped converter, needed because controllers.yaml's
# diff_drive_controller runs with use_stamped_vel: true. Used by
# rover_hardware.launch.py (in the separate rover_hardware package) for the
# real robot; the sim doesn't need it since the native DiffDrive plugin
# takes plain Twist directly.
add_executable(rover_control_node src/rover_control_node.cpp)
target_include_directories(rover_control_node PRIVATE include)
ament_target_dependencies(rover_control_node rclcpp geometry_msgs)

install(TARGETS
  lidar_safety_node
  autonomous_decision_maker
  rover_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  worlds
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
