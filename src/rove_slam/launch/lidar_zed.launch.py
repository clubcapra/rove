from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_rove_zed = get_package_share_directory('rove_zed')

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = False

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rove_zed, 'launch', 'zed_mapping.launch.py'),
        )
    )

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
            ('scan_cloud', '/velodyne_points'),
            ('odom', '/icp_odom')
        ],
        arguments=[
            'Icp/PointToPlane', 'true',
            'Icp/Iterations', '10',
            'Icp/VoxelSize', '0.1',
            'Icp/Epsilon', '0.001',
            'Icp/PointToPlaneK', '20',
            'Icp/PointToPlaneRadius', '0',
            'Icp/MaxTranslation', '2',
            'Icp/MaxCorrespondenceDistance', '1',
            'Icp/Strategy', '1',
            'Icp/OutlierRatio', '0.7',
            'Icp/CorrespondenceRatio', '0.01',
            'Odom/ScanKeyFrameThr', '0.4',
            'OdomF2M/ScanSubtractRadius', '0.1',
            'OdomF2M/ScanMaxSize', '15000',
            'OdomF2M/BundleAdjustment', 'false',
        ]
    )

    point_cloud_assembler_node = Node(
        package='rtabmap_util',
        executable='point_cloud_assembler',
        name='point_cloud_assembler',
        output='screen',
        parameters=[{
            'max_clouds': 10,
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
            remappings=[
              ('/depth/image', '/zed/zed_node/depth/depth_registered'),
              ('/rgb/camera_info' , '/zed/zed_node/depth/camera_info'),
              ('/rgb/image', '/zed/zed_node/rgb/image_rect_color')
            ])
    

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'wait_for_transform': 0.2,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('scan_cloud', '/assembled_cloud'),
            ('odom', '/odometry/local')

        ],
        arguments=[
            '-d',  # This will delete the previous database (~/.ros/rtabmap.db)
            'RGBD/ProximityMaxGraphDepth', '0',
            'RGBD/ProximityPathMaxNeighbors', '1',
            'RGBD/AngularUpdate', '0.05',
            'RGBD/LinearUpdate', '0.05',
            'RGBD/CreateOccupancyGrid', 'true',
            'Mem/NotLinkedNodesKept', 'false',
            'Mem/STMSize', '30',
            'Mem/LaserScanNormalK', '20',
            'Reg/Strategy', '1',
            'Icp/VoxelSize', '0.5',
            'Icp/PointToPlaneK', '20',
            'Icp/PointToPlaneRadius', '0',
            'Icp/PointToPlane', 'true',
            'Icp/Iterations', '10',
            'Icp/Epsilon', '0.001',
            'Icp/MaxTranslation', '3',
            'Icp/MaxCorrespondenceDistance', '1',
            'Icp/Strategy', '1',
            'Icp/OutlierRatio', '0.7',
            'Icp/CorrespondenceRatio', '0.2',
            'Grid/MaxGroundHeight', '0.1',
            'Grid/MaxObstacleHeight', '1.5',
            'Grid/RayTracing', 'true',
            'Grid/3D', 'false',
            'Reg/Force3DoF', 'true',
            'Optimizer/Slam2D', 'true',
        ]
    )

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'odom_frame_id': 'odom',
            'subscribe_rgbd':True,
            'subscribe_odom_info': True,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('scan_cloud', '/assembled_cloud'),
            ('odom', '/odometry/local')
        ]
    )

    return LaunchDescription([
        icp_odometry_node,
        point_cloud_assembler_node,
        rtabmap_node,
        rtabmap_viz_node,
        zed,
        camera_sync,
    ])
