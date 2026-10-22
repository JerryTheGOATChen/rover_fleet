cmake_minimum_required(VERSION 3.8)
project(rover_autonomy)

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
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# Last stop for every velocity command (after twist_mux): clamps forward
# motion to zero while the lidar sees something close ahead.
add_executable(lidar_safety_node src/lidar_safety_node.cpp)
ament_target_dependencies(lidar_safety_node rclcpp geometry_msgs sensor_msgs)

# Mission FSM (IDLE/EXPLORE/AVOID_OBSTACLE/RETURN_HOME/EMERGENCY_STOP).
# EXPLORE picks frontiers from /map (include/rover_autonomy/frontier.hpp).
# Delegates driving to Nav2's navigate_to_pose action except in the two
# reactive states, which publish on /autonomy/cmd_vel (top twist_mux priority).
add_executable(autonomous_decision_maker src/autonomous_decision_maker.cpp)
ament_target_dependencies(autonomous_decision_maker
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  nav2_msgs
  nav_msgs
  tf2
  tf2_ros
)

install(TARGETS
  lidar_safety_node
  autonomous_decision_maker
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
