# SLAM options

**icp_zed.launch.py** : Use lidar to localize the robot in the map. The 2d map is created from both the lidar and the zed camera. The zed camera is used to create a 3d map. The 3d map is photorealistic and can be seen in rviz.

**slam2d.launch.py** : Use slamtool box native slam to perform slam using laser scan (3d lidar converted to 2d). This method is the least accurate but the fastest. (lowest resource usage)

**slam3d_full.launch.py** : Use RTABMAP and create a 3d map from the lidar and the zed camera. This method is the most accurate but the slowest. (highest resource usage). The 3d map can't be seen in rviz with color, you need to use RTABMAP viewer to see the 3d color map. (coupled with lidar point).

**velodyne_3d.launch.py** : Use RTABMAP and create a 3d map from the lidar only. Don't use a lot of resources and more accurate than slam2d. The 3d map isn't colored.

