from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    deskewing = False

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
        #rtabmap_viz_node
    ])
