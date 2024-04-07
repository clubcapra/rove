import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Configure ROS nodes for launch

    # Get the launch directory
    pkg_rove_description = get_package_share_directory('rove_description')
    pkg_rove_slam = get_package_share_directory('rove_slam')
    pkg_rove_nav = get_package_share_directory('rove_navigation')
    slam_pkg_path = get_package_share_directory("slam_toolbox")
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    bringup_pkg_path = get_package_share_directory('rove_bringup')

    # Get the URDF file
    urdf_path = os.path.join(pkg_rove_description, 'urdf', 'rove.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    world_file_name = 'worlds/base_world.world'
    world = os.path.join(pkg_rove_description, world_file_name)

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': "-v 4 -r " + world}.items(),
    )

    # Spawn robot
    create = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'rove',
                   '-topic', 'robot_description',
                   '-x', '0',
                   '-y', '0',
                   '-z', '0.1',
                   ],
        output='screen',
    )

    # Takes the description and joint angles as inputs and publishes
    # the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {"use_sim_time": True, }
        ]
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_rove_description, 'config',
                                     'basic.rviz')],
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_rove_description, 'config',
                                        'default_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            "use_sim_time": True,
        }],
        output='screen'
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg_path, "launch", "online_async_launch.py"),
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": os.path.join(
                pkg_rove_slam, "config", "slam_config.yaml"
            )
        }.items(),
    )

    slam3d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rove_slam, "launch", "slam3d.launch.py"),
        ),
        launch_arguments={
            "use_sim_time": "true",
            "deskewing": "true",
        }.items(),
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rove_nav, "navigation.launch.py"),
        )
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_rove_slam, 'config/ekf.yaml'),
                   {'use_sim_time': True}]
                   )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg_path, "launch", "rove_controller_usb.launch.py"),
        ),
    )

    return LaunchDescription([
            gz_sim,
            DeclareLaunchArgument('rviz', default_value='true',
                                  description='Open RViz.'),
            bridge,
            robot_state_publisher,
            robot_localization_node,
            rviz,
            slam,
            #slam3d,
            create,
            nav,
            teleop,
            ])
