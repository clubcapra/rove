from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rove_control_board',
            # namespace='rove_control_board',
            executable='main',
            name='control_board',
        ),
    ])