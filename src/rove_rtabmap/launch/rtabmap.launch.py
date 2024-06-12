# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true
#   $ ros2 param set /camera/camera depth_module.emitter_enabled 0
#
#   $ ros2 launch rtabmap_examples realsense_d435i_stereo.launch.py

from sympy import false
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    parameters=[{
            "database_path": "/media/SSD/stable/rove/src/rove_rtabmap/maps/map.db" ,
            "rgb_topic": '/zed/zed_node/rgb/image_rect_color' ,
            "depth_topic": "/zed/zed_node/depth/depth_registered" ,
            "camera_info_topic" : "/zed/zed_node/depth/camera_info",
            "odom_topic" : "/zed/zed_node/odom" ,
            "imu_topic" : "/zed/zed_node/imu/data" ,
            "visual_odometry" : False ,
            "frame_id":"zed_camera_link" ,
            "approx_sync" : False ,
            "rgbd_sync": True ,
            "approx_rgbd_sync" : False,
            "delete_db_on_start" : True,
          }]

    return LaunchDescription([

        # Nodes to launch       
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=parameters),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters),
                
        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera/imu')]),
        
        # The IMU frame is missing in TF tree, add it:
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']),
    ])