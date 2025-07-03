from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.event_handlers import OnShutdown, OnProcessStart

def generate_launch_description():
    ws_folder = os.path.abspath(os.path.join(
        get_package_share_directory("rove_logging"), '../../../../'
    ))

    # This deletes then creates a compressed version of the output folder without the older rosbag records or the readme
    # im atheist and even I think this is an afront to god
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'''
                rtabmap-export --scan ~/.ros/rtabmap.db && \
                mv ~/.ros/rtabmap_cloud.ply {ws_folder}/output && \
                rm -f {ws_folder}/output.zip && \
                cd {ws_folder} && \
                zip -r output.zip output -x "output/bags/*" "output/README.md" && \
                zip -r output.zip output/bags/rosbag_latest && \
                ros2 topic pub /is_compression_finished std_msgs/Bool '{{data: true}}'
                '''
            ],

            shell=True,
            output='screen',
            log_cmd=True,
        )
    ])
