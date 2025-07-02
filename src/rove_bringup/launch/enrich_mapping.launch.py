import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import yaml


def generate_launch_description():

    # Radiation mapping
    pkg_rove_radiation = get_package_share_directory("rove_radiation")
    launch_radiation_path = os.path.join(pkg_rove_radiation, "launch", "radiation.launch.py")
    radiation_mapping = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_radiation_path),)

    # Velodyne
    pkg_rove_bringup = get_package_share_directory("rove_bringup")
    launch_velodyne_path = os.path.join(pkg_rove_bringup, "launch", "velodyne.launch.py")
    velodyne = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_velodyne_path),)
    
    # Slam
    launch_slam_path = os.path.join(pkg_rove_bringup, "launch", "autonomy.launch.py")
    slam = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_slam_path),)
    
    return LaunchDescription(
        [
            velodyne,
            slam,
            radiation_mapping,
        ]
    )
