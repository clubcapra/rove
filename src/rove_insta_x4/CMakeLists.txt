cmake_minimum_required(VERSION 3.5)
project(rove_insta_x4)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Define executable
add_executable(image_splitter src/image_splitter.cpp)

# Link libraries
ament_target_dependencies(image_splitter rclcpp sensor_msgs cv_bridge OpenCV)

# Install executable
install(TARGETS
  image_splitter
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

# Ament package
ament_package()
