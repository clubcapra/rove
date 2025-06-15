import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch_ros.actions import SetRemap


def generate_launch_description():
    pkg_rove_navigation = get_package_share_directory("rove_navigation")

    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
    )

    nav2_config_path = os.path.join(
        pkg_rove_navigation, "config", "rove_nav_params.yaml"
    )

    bt_xml_path = PathJoinSubstitution(
        [pkg_rove_navigation, "config", "follow_dynamic_point.xml"]
    )

    person_following_node = Node(
        package="rove_navigation",
        executable="person_following",
        name="person_following",
        output="screen",
    )

    frontier_publisher_node = Node(
        package="rove_navigation",
        executable="frontier_publisher",
        name="frontier_publisher",
        output="screen",
    )

    explorer_node = Node(
        package="rove_navigation",
        executable="exploration",
        name="exploration",
        output="screen",
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": nav2_config_path,
        }.items(),
    )

    return LaunchDescription(
        [
            # person_following_node,
            frontier_publisher_node,
            GroupAction(
                actions=[
                    SetRemap(src="cmd_vel", dst="nav_vel"),
                    nav,
                ]
            ),
        ]
    )
