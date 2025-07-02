from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.event_handlers import OnShutdown, OnProcessStart

def generate_launch_description():
    if (not os.path.isdir('./output/bags')): os.mkdir('./output/bags')
    folder_number = len(next(os.walk('./output/bags'))[1])
    if(folder_number > 0): os.rename("./output/bags/rosbag_latest", "./output/bags/rosbag_{:02}".format(folder_number-1))

    return LaunchDescription([
        Node(
            package="rove_logging",
            executable="rosbag_state_node.py",
            name="rosbag_state_node",
            output="screen",),
        ExecuteProcess(
            cmd=[
                'ros2', 
                'bag', 
                'record', 
                '-a', 
                '--output', './output/bags/rosbag_latest', 
                '--use-sim-time', 
                '--compression-mode', 'file',
                '--compression-format', 'zstd'],
            output='screen'
        )
    ])
