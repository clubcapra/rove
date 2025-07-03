from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.event_handlers import OnShutdown, OnProcessStart

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="rove_logging",
            executable="poi_markers_node.py",
            name="poi_markers_node",
            output="screen",),
    ])
