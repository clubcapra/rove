from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    parameters=[{
        'frame_id':'base_link',
        'subscribe_depth':True,
        'subscribe_odom_info':False,
        'subscribe_rgbd':True,
        'publish_tf':True,
        'Odom/Strategy':'0',
        'Odom/ResetCountdown':'15',
        'Odom/GuessSmoothingDelay':'0',
        'Rtabmap/StartNewMapOnLoopClosure':'true',
        'RGBD/CreateOccupancyGrid':'false',
        'RGBD/LinearUpdate':'0',
        'Icp/VoxelSize': '0.5',
        'Icp/PointToPlaneK': '20',
        'Icp/PointToPlaneRadius': '0',
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '10',
        'Icp/Epsilon': '0.001',
        'Icp/MaxTranslation': '3',
        'Icp/MaxCorrespondenceDistance': '1',
        'Icp/Strategy': '1',
        'Icp/OutlierRatio': '0.7',
        'Icp/CorrespondenceRatio': '0.2',
        'Grid/MaxGroundHeight': '0.1',
        'Grid/MaxObstacleHeight': '1.5',
        'wait_for_transform': 0.2,
        'use_sim_time': use_sim_time,
        'RGBD/AngularUpdate':'0'}]


    remappings=[
        ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
        ('rgb/camera_info', '/zed/zed_node/depth/camera_info'),
        ('/depth/image', '/zed/zed_node/depth/depth_registered'),
        ('/odom', '/odometry/local'),
        ]



    rtabmap_slam = Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d', 
                       'Grid/MaxGroundHeight', '0.1',])
    
    rtabmap_viz_node = Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings)

    camera_sync = Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=parameters,
            remappings=remappings)
    
    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'odom_frame_id': '/odom',
            'wait_for_transform': 0.2,
            'publish_tf': False,
            'expected_update_rate': 50.0,
            'deskewing': False,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('scan_cloud', '/velodyne_points'),
            ('odom', '/icp_odom')
        ],
    )


    return LaunchDescription([
        icp_odometry_node,
        camera_sync,
        rtabmap_slam,
        rtabmap_viz_node,
    ])
