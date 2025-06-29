import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Get the launch directory
    pkg_rove_slam = get_package_share_directory("rove_slam")
    pkg_rove_nav = get_package_share_directory("rove_navigation")
    slam_pkg_path = get_package_share_directory("slam_toolbox")

    use_slam3d = LaunchConfiguration("use_slam3d")
    use_sim_time = LaunchConfiguration("use_sim_time")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg_path, "launch", "online_async_launch.py"),
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": os.path.join(
                pkg_rove_slam, "config", "slam_config.yaml"
            ),
        }.items(),
        condition=UnlessCondition(use_slam3d),
    )

    slam3d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rove_slam, "launch", "velodyne_3d.launch.py"),
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "deskewing": "true",
        }.items(),
        condition=IfCondition(use_slam3d),
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rove_nav, "navigation.launch.py"),
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            slam,
            slam3d,
            # nav,
        ]
    )
