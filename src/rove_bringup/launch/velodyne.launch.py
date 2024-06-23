import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import yaml

# from https://github.com/ros-drivers/velodyne/tree/ros2
def generate_launch_description():
    package_name = 'rove_bringup'
    dir = get_package_share_directory(package_name)
    # Get params files
    params = os.path.join(dir, 'config', 'VLP16_velodyne_params.yaml')
    with open(params, 'r') as f:
        transform_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    transform_params['calibration'] = os.path.join(dir, 'config', 'VLP16db.yaml')

    driver_node = Node(package='velodyne_driver',
        executable='velodyne_driver_node',
        output='both',
        parameters=[params])

    driver_exit_event = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=driver_node,
            on_exit=[launch.actions.EmitEvent(
                event=launch.events.Shutdown())],
        ))

    # laserscan
    laserscan_node = Node(package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        name='scan',
        output='both',
        parameters=[params])

    # pointcloud
    transform_node = Node(package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_points',
        output='both',
        parameters=[transform_params])

    return LaunchDescription([
        driver_node,
        driver_exit_event,
        laserscan_node,
        transform_node,
        ])