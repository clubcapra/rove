# Radiation Mapping

This package simulates radiation mapping.

## Overview

The system listens to:
- `/dose_rate`: Radiation intensity readings (`std_msgs/Float32`)
- `/map`: The environment occupancy grid (`nav_msgs/OccupancyGrid`)
- `/odometry/local`: The robot's position (`nav_msgs/Odometry`)

It publishes:
- A radiation heatmap as a modified occupancy grid on `/radiation_map`
- Visual markers to RViz on `/radiation_marker`

## How to Run

### 1. Launch the robot simulation

```bash
ros2 launch rove_bringup sim.launch.py
```

### 2. Launch the SLAM system
```bash
ros2 launch rove_slam slam2d.launch.py
```

### 3. (Optional) Build after making code changes
```bash

colcon build --packages-select rove_radiation
```

Run this if you modified any files in the rove_radiation package.

### 4. Launch the radiation mapping

```bash
ros2 launch rove_radiation radiation.launch.py max_intensity:=10.0
```

You can change the value of max_intensity (must be a double) to set the maximum radiation intensity you'd like to detect. 

## Dependencies

    ROS 2 (Humble)

    RViz2 for visualization

