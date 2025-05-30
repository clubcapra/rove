# Rove SLAM

This package contains SLAM (Simultaneous Localization and Mapping) related nodes and configurations for the Rove robot.

## Launch Files

- **icp_zed.launch.py**: Uses lidar to localize the robot in the map. The 2D map is created from both the lidar and the Zed camera. The Zed camera is used to create a 3D map, which is photorealistic and can be seen in RViz.
- **slam2d.launch.py**: Uses SLAM Toolbox native SLAM to perform SLAM using laser scan (3D lidar converted to 2D). This method is the least accurate but the fastest (lowest resource usage).
- **slam3d_full.launch.py**: Uses RTABMAP to create a 3D map from the lidar and the Zed camera. This method is the most accurate but the slowest (highest resource usage). The 3D map can't be seen in RViz with color; you need to use RTABMAP viewer to see the 3D color map (coupled with lidar point).
- **velodyne_3d.launch.py**: Uses RTABMAP to create a 3D map from the lidar only. This method doesn't use a lot of resources and is more accurate than slam2d. The 3D map isn't colored.

## Nodes

- **icp_odometry_node**: Handles odometry using ICP (Iterative Closest Point) algorithm.
- **point_cloud_assembler_node**: Assembles point clouds from multiple sensors.
- **rtabmap_node**: Main node for RTABMAP, handling SLAM and mapping.
- **rtabmap_viz_node**: Visualization node for RTABMAP, displaying the 3D map.