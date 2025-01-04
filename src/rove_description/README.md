# Rove Description

This package contains the URDF description of the Rove robot and related simulation files.

## Launch Files

- **launch.py**: Launches the robot description and visualization in RViz.

## URDF Files

- **rove.urdf.xacro**: Main URDF file for the Rove robot, defining its physical structure and joints.
- **gazebo.urdf.xacro**: Gazebo-specific URDF extensions for simulation purposes.
- **velodyne.urdf.xacro**: URDF for the Velodyne sensor, used for 3D mapping and obstacle detection.

## Usage

Once everything has been installed and compiled, you can use the following command to launch the simulation:

```bash
ros2 launch rove_description sim.launch.py
