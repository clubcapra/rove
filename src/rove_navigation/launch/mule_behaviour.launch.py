import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch_ros.actions import SetRemap

def generate_launch_description():
    
    mule_behavior = Node(
            package="rove_navigation",
            executable='mule_behavior',
            name='mule_behavior',
            output='screen',
        )

    person_following = Node(
            package="rove_navigation",
            executable='person_following',
            name='navigate_to_person',
            output='screen',
        )
    
    return LaunchDescription([
        #person_following_node,
        mule_behavior,
        person_following
    ])
