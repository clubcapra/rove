import os
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # pkg_kinova_arm_controller = get_package_share_directory("KinovaArmController")
    # kac_launch_file_path = os.path.join(pkg_kinova_arm_controller, "launch", "arm_launch.py")
    # sj_launch_file_path = os.path.join(pkg_spacemouse_joy, "launch", "pkg_kinova_arm_controller.launch.py")
    
    return LaunchDescription(
        [
            # IncludeLaunchDescription(
            #     launch_description_source=kac_launch_file_path,
            # ),
            Node(
                package="capra_spacemouse_package",
                executable="joy_listener",
                name="joy_listener_node",
            ),
            Node(
                package="capra_spacemouse_package",
                executable="spacemouse_joy_publisher",
                name="spacemouse_joy_publisher_node",
            ),
        ]
    )