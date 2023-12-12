cmake_minimum_required(VERSION 3.8)
project(rove_bringup)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(rove_controller src/rove_controller.cpp)

# Link dependencies
ament_target_dependencies(rove_controller rclcpp sensor_msgs)

# Install executable
install(TARGETS
  rove_controller
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
