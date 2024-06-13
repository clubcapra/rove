import os
from sympy import false
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    parameters=[
            ['database_path', '/media/SSD/stable/rove/src/rove_rtabmap/maps/map.db' ],
            ['rgb_topic', '/zed/zed_node/rgb/image_rect_color' ],
            ['depth_topic', '/zed/zed_node/depth/depth_registered' ],
            ['camera_info_topic', '/zed/zed_node/depth/camera_info'],
            ['odom_topic', '/zed/zed_node/odom' ],
            # ['odom_frame_id', '/zed/zed_node/odom'],
            ['imu_topic', '/zed/zed_node/imu/data' ],
            ['visual_odometry', 'false' ],
            ['frame_id', 'zed_camera_link' ],
            ['approx_sync', 'false' ],
            ['rgbd_sync', 'true' ],
            ['approx_rgbd_sync', 'false'],
            ['delete_db_on_start', 'true'],
            # ['subscribe_scan_cloud', 'true'],
            # ['scan_cloud_topic', '/velodyne_points'],
            # ['RGBD/NeighborLinkRefining', 'true'],
            # ['RGBD/ProximityBySpace', 'true'],
            # ['RGBD/AngularUpdate', '0.01'],
            # ['RGBD/LinearUpdate', '0.01'],
            # ['RGBD/OptimizeFromGraphEnd', 'false'],
            # ['Grid/FromDepth', 'false'],
            # ['Reg/Force3DoF', 'true'],
            # ['Reg/Strategy', '1'],
            # ['Icp/VoxelSize', '0.05'],
            # ['Icp/MaxCorrespondenceDistance', '0.1'],
            # ['Optimizer/Slam2D', 'true'],
          ]

    pkg_rtabmap = get_package_share_directory('rtabmap_launch')
    launch_file_path = os.path.join(pkg_rtabmap, 'launch', 'rtabmap.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            launch_description_source=launch_file_path,
            launch_arguments=parameters,
        )
    ])
