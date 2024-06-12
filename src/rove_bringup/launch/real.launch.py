import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit, OnShutdown

def generate_launch_description():
    # Get the launch directory
    pkg_rove_bringup = get_package_share_directory('rove_bringup')
    pkg_rove_description = get_package_share_directory('rove_description')

    # Get the URDF file
    urdf_path = os.path.join(pkg_rove_description, 'urdf', 'rove.urdf.xacro')
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [urdf_path]
            ),
#            " ",
#            "use_mock_hardware:=",
#            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    # Controllers
    controller_nodes = ["diff_drive_controller"]

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rove_bringup, "launch", "common.launch.py"),
        ),
        launch_arguments={
            "use_sim_time": "false",
        }.items(),
    )

    ###### ROS2 control ######
    controllers = PathJoinSubstitution(
        [
            pkg_rove_description,
            "config",
            "tracks_controllers.yaml",
        ]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers,
        ],
        output="both",
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    
    def create_controller_node(node_name:str):
        robot_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[node_name, "--controller-manager", "/controller_manager"],
        )
        
        # Delay start of robot_controller after `joint_state_broadcaster`
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
        return delay_robot_controller_spawner_after_joint_state_broadcaster_spawner
        
    delayed_controller_nodes = list([create_controller_node(node_name) for node_name in controller_nodes])
    
    # start_can_cmd = ExecuteProcess(
    #     cmd=[[
    #         'ip link set can0 type can bitrate 250000; ip link set up can0'
    #     ]],
    #     shell=True
    # )

    # stop_can_cmd = ExecuteProcess(
    #     cmd=[[
    #         'ip link set down can0'
    #     ]],
    #     shell=True
    # )

    # shutdown = RegisterEventHandler(
    #     event_handler=OnShutdown(
    #         on_shutdown=[stop_can_cmd]
    #     )
    # )

    ###### Sensor ######
    vectornav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rove_bringup, "launch", "vectornav.launch.py"),
        ),
    )

    velodyne = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rove_bringup, "launch", "velodyne.launch.py"),
        ),
    )

    return LaunchDescription([
            control_node,
            common,
            joint_state_broadcaster_spawner,
            *delayed_controller_nodes,
            vectornav,
            velodyne,
            ])
