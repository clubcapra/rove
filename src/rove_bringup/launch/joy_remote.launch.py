from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "rove_bringup"
    dir = get_package_share_directory(package_name)
    # Get params files
    joy_params_file = dir + "/config/joy_params.yaml"

    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="game_controller_node",
                output="screen",
                parameters=[joy_params_file],
                remappings=[
                    ("/joy", "/rove/joy"),
                ]
            ),
        ]
    )
