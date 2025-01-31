from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    package_name = 'rove_bringup'
    dir = get_package_share_directory(package_name)
    # Get params files
    
    insta360_x4 = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node_exe',
            output='screen',
            parameters=[os.path.join(dir, 'config', 'insta360_x4.yaml')],
        )

    return LaunchDescription([
        insta360_x4
    ])
