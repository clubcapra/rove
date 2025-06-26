from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_rove_radiation = get_package_share_directory("rove_radiation")

    pkg_rove_radiation = get_package_share_directory("radiacode_driver")
    launch_file_path = os.path.join(pkg_rove_radiation, "radiacode_driver.launch.py")
    print(launch_file_path)


    max_intensity = LaunchConfiguration("max_intensity", default=10.0)

    return LaunchDescription(
        [
            Node(
                package="rove_radiation",
                executable="radiation_position_tracker",
                name="radiation_position_tracker",
                output="screen",
                parameters=[max_intensity],
            ),
            Node(
                package="rove_radiation",
                executable="radiation_map_data_viewer",
                name="radiation_map_data_viewer",
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file_path)
            )
        ]
    )
