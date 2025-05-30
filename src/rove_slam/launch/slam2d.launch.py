from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_rove_slam = get_package_share_directory("rove_slam")

    config_file = os.path.join(pkg_rove_slam, "config", "slam_config.yaml")

    return LaunchDescription(
        [
            Node(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[config_file],
            )
        ]
    )
