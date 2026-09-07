cmake_minimum_required(VERSION 3.8)
project(rover_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# lgpio is a plain system C library (not an ament/ROS package) - install it
# on the Pi first, e.g.:
#   git clone https://github.com/joan2937/lg && cd lg && make && sudo make install
# or via your distro's package if it ships one (liblgpio-dev on some).
find_library(LGPIO_LIB lgpio)
find_path(LGPIO_INCLUDE_DIR lgpio.h)
if(NOT LGPIO_LIB OR NOT LGPIO_INCLUDE_DIR)
  message(FATAL_ERROR
    "lgpio not found. Install it on the Pi first (see comment above this "
    "line in CMakeLists.txt), then re-run colcon build.")
endif()

add_library(rover_system_hardware SHARED
  src/rover_system_hardware.cpp
  src/motor_channel.cpp
  src/quadrature_encoder.cpp
)
target_include_directories(rover_system_hardware PRIVATE
  include
  ${LGPIO_INCLUDE_DIR}
)
target_link_libraries(rover_system_hardware ${LGPIO_LIB})
ament_target_dependencies(rover_system_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface rover_hardware_plugin.xml)

install(TARGETS rover_system_hardware
  DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES rover_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(rover_system_hardware)
ament_package()
