import os
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_kinova_arm_controller = get_package_share_directory("arm_controller")
    kac_launch_file_path = os.path.join(pkg_kinova_arm_controller, "launch", "arm_launch.py")
    
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                launch_description_source=kac_launch_file_path,
            ),
            Node(
                package='rove_arm_control',
                executable='rove_arm_control',
                name='arm_control_node',
                output='screen',
            ),
            Node(
                package="capra_spacemouse_package",
                executable="spacemouse_joy_publisher",
                name="spacemouse_joy_publisher_node",
            ),
        ]
    )