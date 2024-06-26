cmake_minimum_required(VERSION 3.8)
project(rove_bringup)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(rove_controller_node src/rove_controller_node.cpp)

# Link dependencies
ament_target_dependencies(rove_controller_node rclcpp sensor_msgs geometry_msgs control_msgs rclcpp_action)

# Install executable
install(TARGETS
  rove_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
