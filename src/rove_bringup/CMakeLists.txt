cmake_minimum_required(VERSION 3.8)
project(rove_bringup)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Executables
add_executable(rove_controller_node src/rove_controller_node.cpp)
add_executable(joy_remapper_node src/joy_remapper_node.cpp)

# Dependencies per target
ament_target_dependencies(rove_controller_node
  rclcpp
  rclcpp_action
  std_srvs
  control_msgs
)

ament_target_dependencies(joy_remapper_node
  rclcpp
  sensor_msgs
  geometry_msgs
  control_msgs
  rclcpp_action
)

# Installation
install(TARGETS
  rove_controller_node
  joy_remapper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/wifi_roamer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
