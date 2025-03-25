from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    package_name = 'rove_insta_x4'
    dir = get_package_share_directory(package_name)
    # Get params files
    
    wide = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node_exe',
        output='screen',
        parameters=[os.path.join(dir, 'config', 'wide.yaml')],
        remappings = [
            ('image_raw', f'insta360/image_raw'),
            ('image_raw/compressed', f'insta360/image_compressed'),
            ('image_raw/compressedDepth', f'insta360/compressedDepth'),
            ('image_raw/theora', f'insta360/image_raw/theora'),
            ('camera_info', f'insta360/camera_info'),
        ]
    )
    
    split = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node_exe',
        output='screen',
        parameters=[os.path.join(dir, 'config', 'split.yaml')],
        remappings = [
            ('image_raw', f'insta360/image_raw'),
            ('image_raw/compressed', f'insta360/image_compressed'),
            ('image_raw/compressedDepth', f'insta360/compressedDepth'),
            ('image_raw/theora', f'insta360/image_raw/theora'),
            ('camera_info', f'insta360/camera_info'),
        ]
    )
    
    image_splitter = Node(
        package=package_name,
        executable='image_splitter',
        name='image_splitter',
        output='screen',
        remappings = [
            ('input_image/compressed', f'insta360/image_compressed'),
        ]
    )

    return LaunchDescription([
        wide,
        #split,
        image_splitter
    ])
