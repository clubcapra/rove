from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    camera_model = 'zed2i'

    pkg_zed_wrapper = get_package_share_directory('zed_wrapper')
    launch_file_path = os.path.join(pkg_zed_wrapper, 'launch', 'zed_camera.launch.py')

    # Parse yml config file to iterable array
    pkg_rove_zed = get_package_share_directory('rove_zed')
    config_override_path = os.path.join(pkg_rove_zed, 'config', 'zed_body_trck.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            launch_description_source=launch_file_path,
            launch_arguments=[ # `ros2 launch rove_zed zed_body_trck.launch.py --show-arguments`
                ['camera_model', camera_model],
                ['camera_name', 'zed'],
                ['ros_params_override_path', config_override_path]
            ]
        )
    ])
