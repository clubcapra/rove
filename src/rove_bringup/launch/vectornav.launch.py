import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'rove_bringup'
    dir = get_package_share_directory(package_name)
    # Get params files
    vn_param_file = os.path.join(dir, 'config', 'vn_300.yaml')

    return LaunchDescription([
        Node(
            package='vectornav', 
            executable='vectornav',
            output='screen',
            parameters=[vn_param_file],
        ),
        Node(
            package='vectornav', 
            executable='vn_sensor_msgs',
            output='screen',
            parameters=[vn_param_file],
            remappings=[
                ('/vectornav/gnss', '/gnss'),
            ]
        ),
    ])