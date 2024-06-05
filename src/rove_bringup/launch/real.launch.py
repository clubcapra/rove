import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Get the launch directory
    pkg_rove_bringup = get_package_share_directory('rove_bringup')
    pkg_rove_description = get_package_share_directory('rove_description')

    # Get the URDF file
    urdf_path = os.path.join(pkg_rove_description, 'urdf', 'rove.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
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
    
    # Takes the description and joint angles as inputs and publishes
    # the 3D poses of the robot links
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {"use_sim_time": True, }
        ],
        remappings=[
            # ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            # ("/diff_drive_controller/cmd_vel", "/cmd_vel"),
            # ("/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped"),
            # ("/cmd_vel", "/diff_drive_controller/cmd_vel"),
        ],
    )
    
    # Controllers
    # controller_nodes = ["left_track_controller", "right_track_controller", "diff_drive_controller"]
    controller_nodes = ["diff_drive_controller"]
    # controller_nodes = ["velocity_controller"]
    # controller_nodes = ["forward_controller"]

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rove_bringup, "launch", "common.launch.py"),
        ),
    )

    ###### ROS2 control ######
    controllers = PathJoinSubstitution(
        [
            pkg_rove_description,
            "config",
            "tracks_controllers.yaml",
            # "test_controllers.yaml",
        ]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            # robot_desc,
            robot_description,
            controllers,
        ],
        # remappings=[
        #     ('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'),
        # ],
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
    

    ###### Sensor ######
    vectornav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rove_bringup, "launch", "vectornav.launch.py"),
        ),
    )
    

    return LaunchDescription([
            robot_state_pub_node,
            control_node,
            common,
            joint_state_broadcaster_spawner,
            *delayed_controller_nodes,
            # vectornav,
            ])
