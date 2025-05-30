import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf_file_name = "urdf/rove.urdf.xacro"
    urdf = os.path.join(get_package_share_directory("rove_description"), urdf_file_name)

    doc = xacro.process_file(urdf)
    robot_desc = doc.toxml()

    return LaunchDescription(
        [
            # A GUI to manipulate the joint state values
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc}],
                arguments=[urdf],
            ),
            Node(
                package="rviz2",
                namespace="",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d"
                    + os.path.join(
                        get_package_share_directory("rove_description"),
                        "rviz",
                        "conf.rviz",
                    )
                ],
            ),
        ]
    )
