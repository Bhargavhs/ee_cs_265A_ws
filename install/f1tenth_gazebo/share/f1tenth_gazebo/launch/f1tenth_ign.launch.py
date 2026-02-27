import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_dir = get_package_share_directory('f1tenth_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Process xacro for each colored car
    xacro_red = os.path.join(pkg_dir, 'urdf', 'racecar_red.xacro')
    xacro_blue = os.path.join(pkg_dir, 'urdf', 'racecar_blue.xacro')
    xacro_green = os.path.join(pkg_dir, 'urdf', 'racecar_green.xacro')
    
    robot_desc_red = {'robot_description': xacro.process_file(xacro_red).toxml()}
    robot_desc_blue = {'robot_description': xacro.process_file(xacro_blue).toxml()}
    robot_desc_green = {'robot_description': xacro.process_file(xacro_green).toxml()}
    
    # World file
    world_file = os.path.join(pkg_dir, 'worlds', 'track.sdf')
    
    return LaunchDescription([
        # Launch Ignition Gazebo with track
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        ),
        
        # Robot State Publisher for RED car
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_car1',
            namespace='car1',
            output='screen',
            parameters=[robot_desc_red],
        ),
        
        # Robot State Publisher for BLUE car
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_car2',
            namespace='car2',
            output='screen',
            parameters=[robot_desc_blue],
        ),
        
        # Robot State Publisher for GREEN car
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_car3',
            namespace='car3',
            output='screen',
            parameters=[robot_desc_green],
        ),
        
        # Spawn RED car - left position on track
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'car1_red',
                '-topic', '/car1/robot_description',
                '-x', '0.0',
                '-y', '5.0',
                '-z', '0.1',
                '-Y', '0.0'
            ],
            output='screen',
        ),
        
        # Spawn BLUE car - center position on track
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'car2_blue',
                '-topic', '/car2/robot_description',
                '-x', '0.0',
                '-y', '6.0',
                '-z', '0.1',
                '-Y', '0.0'
            ],
            output='screen',
        ),
        
        # Spawn GREEN car - right position on track
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'car3_green',
                '-topic', '/car3/robot_description',
                '-x', '0.0',
                '-y', '7.0',
                '-z', '0.1',
                '-Y', '0.0'
            ],
            output='screen',
        ),
        
        # Bridge for RED car
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_car1',
            arguments=[
                '/model/car1_red/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/model/car1_red/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            ],
            remappings=[
                ('/model/car1_red/cmd_vel', '/red/cmd_vel'),
                ('/model/car1_red/odometry', '/red/odometry'),
            ],
            output='screen',
        ),

        # Bridge for BLUE car
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_car2',
            arguments=[
                '/model/car2_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/model/car2_blue/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            ],
            remappings=[
                ('/model/car2_blue/cmd_vel', '/blue/cmd_vel'),
                ('/model/car2_blue/odometry', '/blue/odometry'),
            ],
            output='screen',
        ),

        # Bridge for GREEN car
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_car3',
            arguments=[
                '/model/car3_green/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/model/car3_green/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            ],
            remappings=[
                ('/model/car3_green/cmd_vel', '/green/cmd_vel'),
                ('/model/car3_green/odometry', '/green/odometry'),
            ],
            output='screen',
        ),
        
        # Clock bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_clock',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            ],
            output='screen',
        ),
    ])
