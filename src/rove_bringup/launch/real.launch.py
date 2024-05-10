import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the launch directory
    pkg_rove_bringup = get_package_share_directory('rove_bringup')

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rove_bringup, "launch", "common.launch.py"),
        ),
    )


    ###### Sensor ######
    vectornav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rove_bringup, "launch", "vectornav.launch.py"),
        ),
    )

    return LaunchDescription([
            common,
            vectornav,
            ])
