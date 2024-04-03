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
        Node(
            package='rove_opi_publisher',
            namespace='rove_opi',
            executable='test',
            name='opi_pub',
        ),
    ])