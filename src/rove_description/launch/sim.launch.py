from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from launch.actions import SetEnvironmentVariable



def generate_launch_description():
    world_file_name = 'worlds/base_world.world'
    urdf_file_name = 'urdf/rove.urdf.xacro'
    world = os.path.join(get_package_share_directory('rove_description'),world_file_name)
    urdf = os.path.join(get_package_share_directory('rove_description'),urdf_file_name)
    
    ign_gazebo_resource_path = "/workspace/rove/install/rove_description/share"

    existing_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    if existing_path:
        ign_gazebo_resource_path += ':' + existing_path

    doc = xacro.process_file(urdf)
    robot_desc = doc.toxml()

    # Gazebo Sim
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_gazebo_resource_path),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r  ' + world + ' --headless-rendering -v4'}.items(),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc , 'use_sim_time': True}],
            arguments=[urdf]),

        Node(package='ros_gz_sim', executable='create', arguments=[
            '-name', 'rove',
            '-topic', 'robot_description',
            '-z', '0.1',],
            output='screen', ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/rove/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/rove/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
            output='screen'
        )
        
    ])