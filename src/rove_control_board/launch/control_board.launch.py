import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_rove_control_board = get_package_share_directory("rove_control_board")
    config_file = os.path.join(pkg_rove_control_board, 'config', 'base.yaml')
    
    return LaunchDescription([
        Node(
            package='rove_control_board',
            # namespace='rove_control_board',
            executable='main',
            name='control_board',
            parameters=[
                config_file,
            ]
        ),
    ])