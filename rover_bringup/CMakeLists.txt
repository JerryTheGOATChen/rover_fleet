cmake_minimum_required(VERSION 3.8)
project(rover_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds rviz foxglove behavior_trees DESTINATION share/${PROJECT_NAME})

ament_package()
