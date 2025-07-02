import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions.node import Node


def generate_launch_description():
    pkg_rove_bringup = get_package_share_directory("rove_bringup")

    roamer = Node(
        package="rove_bringup",
        executable="wifi_roamer_node.py",
        name="wifi_roamer_node",
        parameters=[os.path.join(pkg_rove_bringup, "config", "roamer.yaml")],
        output="both",
    )

    return LaunchDescription([roamer])
