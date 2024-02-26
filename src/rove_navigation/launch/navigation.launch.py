import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_rove_navigation = get_package_share_directory('rove_navigation')

    params_file = os.path.join(pkg_rove_navigation, 'config',
                               'rove_nav_params.yaml')

    nav=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
        launch_arguments={
        'params_file': params_file}.items(),

    )

    return LaunchDescription([
        nav
    ])
