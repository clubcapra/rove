cmake_minimum_required(VERSION 3.8)
project(rove_bringup)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(rove_controller_node src/rove_controller_node.cpp)

# Link dependencies
ament_target_dependencies(rove_controller_node rclcpp sensor_msgs)

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
