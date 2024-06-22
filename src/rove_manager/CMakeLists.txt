cmake_minimum_required(VERSION 3.5)
project(rove_manager)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Create executable
add_executable(${PROJECT_NAME}_node src/rove_manager_node.cpp)

# Link libraries
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
)

# Install executable
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
