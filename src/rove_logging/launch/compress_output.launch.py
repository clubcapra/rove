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

    # This deletes then creates a compressed version of the output folder without the older rosbag records or the readme
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'rm -f output.zip && zip -r output.zip ./output -x "./output/bags/*" "./output/README.md" && zip -r output.zip ./output/bags/rosbag_latest'
            ],
            shell=True,
            output='screen'

        )
    ])
