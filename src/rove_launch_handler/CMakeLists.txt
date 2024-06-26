cmake_minimum_required(VERSION 3.5)
project(rove_launch_handler)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)

# Convert message files to be used in python
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LaunchRequest.srv"
  "srv/LaunchListRequest.srv"
)

# Install Python executables
install(PROGRAMS
  scripts/launch_handler.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch srv
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
