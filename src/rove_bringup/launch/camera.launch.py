from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    package_name = 'rove_bringup'
    dir = get_package_share_directory(package_name)
    # Get params files
    
    insta360_x4_wide = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node_exe',
        output='screen',
        parameters=[os.path.join(dir, 'config', 'insta360_x4_wide.yaml')],
        remappings = [
            ('image_raw', f'insta360/image_raw'),
            ('image_raw/compressed', f'insta360/image_compressed'),
            ('image_raw/compressedDepth', f'insta360/compressedDepth'),
            ('image_raw/theora', f'insta360/image_raw/theora'),
            ('camera_info', f'insta360/camera_info'),
        ]
    )
    
    insta360_x4_split = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node_exe',
        output='screen',
        parameters=[os.path.join(dir, 'config', 'insta360_x4_split.yaml')],
        remappings = [
            ('image_raw', f'insta360/image_raw'),
            ('image_raw/compressed', f'insta360/image_compressed'),
            ('image_raw/compressedDepth', f'insta360/compressedDepth'),
            ('image_raw/theora', f'insta360/image_raw/theora'),
            ('camera_info', f'insta360/camera_info'),
        ]
    )

    return LaunchDescription([
        insta360_x4_wide,
        # insta360_x4_split
    ])
