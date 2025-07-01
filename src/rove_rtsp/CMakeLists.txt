cmake_minimum_required(VERSION 3.5)
project(rove_rtsp)

# Use Python
find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(
  PROGRAMS
  rove_rtsp/rtsp_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
