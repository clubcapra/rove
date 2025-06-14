from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bitrate = LaunchConfiguration('bitrate', default=500000)
    sim = LaunchConfiguration('sim', default=False)
    
    
    return LaunchDescription([
        Node(
            package='rove_control_board',
            # namespace='rove_control_board',
            executable='main',
            name='control_board',
            ros_arguments=[
                ('mock_servos', 'true' if sim else 'false'),
                ('bitrate', str(bitrate)),
            ],
        ),
    ])