from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_rove_radiation = get_package_share_directory('rove_radiation')

    return LaunchDescription([
        Node(
            package='rove_radiation',
            executable='radiation_publisher',
            name='radiation_publisher',
            output='screen'
        ),
        Node(
            package='rove_radiation',
            executable='radiation_position_tracker',
            name='radiation_position_tracker',
            output='screen'
        ),
        Node(
            package='rove_radiation',
            executable='radiation_position_tracker_3d',
            name='radiation_position_tracker_3d',
            output='screen'
        ),
        
    ])

