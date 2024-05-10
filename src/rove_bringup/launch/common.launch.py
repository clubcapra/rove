import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the launch directory
    pkg_rove_description = get_package_share_directory('rove_description')
    pkg_rove_slam = get_package_share_directory('rove_slam')
    bringup_pkg_path = get_package_share_directory('rove_bringup')

    # Get the URDF file
    urdf_path = os.path.join(pkg_rove_description, 'urdf', 'rove.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

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

    # used tutorial: https://navigation.ros.org/tutorials/docs/navigation2_with_gps.html

    robot_localization_node_local = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node_local',
       output='screen',
       parameters=[os.path.join(pkg_rove_slam, 'config/ekf.yaml'),
                   {'use_sim_time': True}]
                   )

    robot_localization_node_global = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node_global',
       output='screen',
       parameters=[os.path.join(pkg_rove_slam, 'config/ekf.yaml'),
                   {'use_sim_time': True}],
       remappings=[("odometry/filtered", "odometry/filtered/global")],
    )

    navsat_transform = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[os.path.join(pkg_rove_description, 'config/navsat_transform.yaml'),
                   {'use_sim_time': True}],
        remappings=[
            # Subscribed Topics
                    ("imu/data", "imu"),
                    ("gps/fix", "gps"),
                    ("odometry/filtered", "odometry/filtered/global"),
            # Published Topics
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                ],
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg_path, "launch", "rove_controller_usb.launch.py"),
        ),
    )

    return LaunchDescription([
            robot_state_publisher,
            robot_localization_node_local,
            robot_localization_node_global,
            navsat_transform,
            rviz,
            teleop,
            ])
