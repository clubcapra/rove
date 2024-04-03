from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rove_opi',
            namespace='rove_opi',
            executable='test',
            name='opi',
        ),
    ])