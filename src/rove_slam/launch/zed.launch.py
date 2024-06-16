from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_rove_zed = get_package_share_directory('rove_zed')

    parameters=[{
        'odom_frame_id': 'odom',
        'publish_tf': False,
        'frame_id':'base_link',
        'subscribe_depth': True,
        'subscribe_odom_info': False,
        'approx_sync': True,
        'Grid/MaxGroundHeight': '0.1',
        'Grid/MaxObstacleHeight': '1.5',
        'Grid/RayTracing': True,
        'Grid/3D': False,
        'wait_imu_to_init':True}]

    remappings=[
        ('imu', '/imu'),
        ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
        ('rgb/camera_info', '/zed/zed_node/depth/camera_info'),
        ('/depth/image', '/zed/zed_node/depth/depth_registered'),
        ]

    rtabmap_slam = Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d'])
    
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=parameters,
        remappings=remappings
    )



    odom =  Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings)

    

    return LaunchDescription([
        odom,
        rtabmap_slam,
        rtabmap_viz_node,
    ])
