from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = False

    remappings=[
        ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
        ('rgb/camera_info', '/zed/zed_node/depth/camera_info'),
        ('/depth/image', '/zed/zed_node/depth/depth_registered'),
        ('/odom', '/odometry/local'),
        ('scan_cloud', '/assembled_cloud'),
        ]

    parameters=[{
        'frame_id':'base_link',
        'subscribe_depth':True,
        'subscribe_odom_info':False,
        'subscribe_scan_cloud': True,
        'subscribe_rgb':True,
        'approx_sync': True,
        'publish_tf':True,
        'Odom/Strategy':'0',
        'Odom/ResetCountdown':'15',
        'Odom/GuessSmoothingDelay':'0',
        'Rtabmap/StartNewMapOnLoopClosure':'true',
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
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
        'Grid/RayTracing': 'true',
        'wait_for_transform': 0.2,
        'use_sim_time': True,}]


    # Nodes
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
            'deskewing': deskewing,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('scan_cloud', '/assembled_cloud'),
            ('odom', '/icp_odom')
        ],
    )

    

    point_cloud_assembler_node = Node(
        package='rtabmap_util',
        executable='point_cloud_assembler',
        name='point_cloud_assembler',
        output='screen',
        parameters=[{
            'max_clouds': 10,
            'fixed_frame_id':'sensor_link',
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('cloud', '/velodyne_points'),
            ('odom', '/odometry/local')
        ]
    )

    camera_sync = Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{
              'approx_sync':True
            }],
            remappings=remappings)

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=parameters,
        remappings=remappings,
        arguments=['-d']
    )

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=parameters,
        remappings=remappings,
    )

    return LaunchDescription([
        icp_odometry_node,
        camera_sync,
        point_cloud_assembler_node,
        rtabmap_node,
        rtabmap_viz_node
    ])
