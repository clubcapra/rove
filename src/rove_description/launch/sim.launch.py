from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Package directories
    pkg_rove_description = get_package_share_directory('rove_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Add ign path to environment
    ign_gazebo_resource_path = "/workspace/rove/install/rove_description/share"
    existing_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    if existing_path:
        ign_gazebo_resource_path += ':' + existing_path

    # World file
    world_file_name = 'worlds/base_world.world'
    world = os.path.join(pkg_rove_description, world_file_name)

    # robot file
    rove_urdf_path  =  os.path.join(pkg_rove_description, 'urdf', 'rove.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', rove_urdf_path]), value_type=str)


    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': "-v 4 -r " + world}.items(),
    )

    # Robot state publisher
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
            arguments=[])
    
    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_rove_description, 'config', 'basic.rviz')],
    )

    # Bridge between ROS and Gazebo
    bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': os.path.join(pkg_rove_description, 'config', 'robot_bridge.yaml'),
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }],
            output='screen'
        )
    
    # Spawn robot in Gazebo
    spawn_rove = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'rove',
                   '-topic', 'robot_description',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '1.0',
                ],
        output='screen',
    )
    
    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_gazebo_resource_path),
        gz_sim,
        bridge,
        robot_state_publisher,
        spawn_rove,
        rviz,
    ])