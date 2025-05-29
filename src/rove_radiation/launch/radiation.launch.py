from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument  
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    pkg_rove_radiation = get_package_share_directory('rove_radiation')

    max_intensity_arg = DeclareLaunchArgument(
        'max_intensity',
        description='Intensité maximale pour l’échelle de couleurs'
    )

    max_intensity = LaunchConfiguration('max_intensity')
    
    return LaunchDescription([
        Node(
            package='rove_radiation',
            executable='radiation_publisher',
            name='radiation_publisher',
            output='screen'
        ),
        Node(
            package='rove_radiation',
            executable='radiation_position_tracker',
            name='radiation_position_tracker',
            output='screen',
            parameters=[{'max_intensity': max_intensity}]
        ),
        Node(
            package='rove_radiation',
            executable='radiation_map_data_viewer',
            name='radiation_map_data_viewer',
            output='screen'
        )
    ])

