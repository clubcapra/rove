import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare launch arguments
    declare_small_house_arg = DeclareLaunchArgument(
        "small_house",
        default_value="false",
        description='Set to "true" to use the small house world.',
    )

    # Get the launch directory
    pkg_rove_bringup = get_package_share_directory("rove_bringup")
    pkg_rove_description = get_package_share_directory("rove_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_ovis = get_package_share_directory("ovis_bringup")
    pkg_aws_house = get_package_share_directory("aws_robomaker_small_house_world")

    # Get simulation file
    walls_file_path = os.path.join(pkg_rove_description, "worlds", "walls.sdf")
    actor_file_path = os.path.join(pkg_rove_description, "worlds", "actor.sdf")
    world_file_path = os.path.join(pkg_rove_description, "worlds", "base_world.world")
    small_house_file_path = os.path.join(pkg_aws_house, "worlds", "small_house.world")

    model_path = os.path.join(pkg_aws_house, "models")

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-v 4 -r " + world_file_path}.items(),
        condition=UnlessCondition(LaunchConfiguration("small_house")),
    )

    gz_sim_house = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-v 4 -r " + small_house_file_path}.items(),
        condition=IfCondition(LaunchConfiguration("small_house")),
    )

    spawn_walls = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file",
            walls_file_path,
            "-name",
            "walls",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0",
        ],
        condition=UnlessCondition(LaunchConfiguration("small_house")),
        output="screen",
    )

    spawn_actor = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file",
            actor_file_path,
            "-name",
            "actor",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.06",
        ],
        condition=UnlessCondition(LaunchConfiguration("small_house")),
        output="screen",
    )

    # Spawn robot
    spawn_rove = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "rove",
            "-topic",
            "robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.1",
        ],
        output="screen",
    )

    # fake human tracker
    human_tracker = Node(
        package="rove_navigation",
        executable="green_person_tracker",
        name="green_person_tracker",
        output="screen",
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_rove_description, "config", "default_bridge.yaml"
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rove_bringup, "launch", "common.launch.py"),
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    ovis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ovis, "launch", "sim.launch.py"),
        ),
        launch_arguments={
            "with_rove": "true",
            "with_joy": "false",
            "ovis_base_origin": "0.22 0 0.38 0 0 3.14",
        }.items(),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path),
            declare_small_house_arg,
            gz_sim,
            gz_sim_house,
            bridge,
            spawn_walls,
            spawn_actor,
            spawn_rove,
            common,
            # human_tracker,
            # ovis,
        ]
    )
