import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_dir = get_package_share_directory('f1tenth_gazebo')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    world_file = os.path.join(pkg_dir, 'worlds', 'demo_track.world')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'racecar.xacro')
    
    # Process xacro
    robot_description_config = xacro.process_file(xacro_file, mappings={'package_name': 'f1tenth_gazebo'})
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        
        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'racecar',
                '-x', '0.0',
                '-y', '-5.0',
                '-z', '0.1'
            ],
            output='screen',
        ),
    ])
