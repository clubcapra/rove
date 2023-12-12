from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'rove_bringup'
    # Get params files
    joy_params_file = get_package_share_directory(package_name) + '/config/joy_params.yaml'
    teleope_joy_params_file = get_package_share_directory(package_name) + '/config/teleop_joy_params.yaml'

    return LaunchDescription([
        Node(
            package='joy',
            executable='game_controller_node',
            name='game_controller_node',
            output='screen',
            parameters=[joy_params_file],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[teleope_joy_params_file],
            remappings=[
                ('/joy', '/rove/joy'),
                ('/cmd_vel', '/model/rove/cmd_vel')
            ],
        ),
        Node(
            package=package_name,
            executable='rove_controller',
            name='rove_controller',
            output='screen',
        ),
    ])