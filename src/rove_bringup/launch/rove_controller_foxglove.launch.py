from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'rove_bringup'
    dir = get_package_share_directory(package_name)
    # Get params files
    joy_params_file = dir + '/config/joy_params.yaml'
    teleop_joy_params_file = dir + '/config/teleop_joy_params.yaml'
    usb_mapping_file = dir + '/config/usb_mapping.yaml'
    cvt_mapping_file = dir + '/config/foxglove_to_rove_mapping.yaml'

    return LaunchDescription([
        Node(
            package=package_name,
            executable='joy_remapper_node',
            name='joy_remapper_node',
            parameters=[cvt_mapping_file],
            output='screen',
            remappings=[
                ("/joy_out", "/rove/joy"),
            ],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[teleop_joy_params_file],
            remappings=[
                ("/joy", "/rove/joy"),
                ('/cmd_vel', '/joy_vel'),
            ],
        ),
        # Node(
        #     package=package_name,
        #     executable='rove_controller_node',
        #     name='rove_controller_node',
        #     parameters=[usb_mapping_file],
        #     output='screen',
        #     remappings=[
        #         ("/joy_out", "/joy"),
        #     ],
        # ),
    ])