from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rosbridge_launch_file = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch',
        'foxglove_bridge_launch.xml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(rosbridge_launch_file),
            launch_arguments=[
                ("include_hidden", "true"),
            ],
        ),
        Node(
            package='rove_launch_handler',
            executable='launch_handler.py',
            name='launch_handler',
            output='screen',
        )
    ])
