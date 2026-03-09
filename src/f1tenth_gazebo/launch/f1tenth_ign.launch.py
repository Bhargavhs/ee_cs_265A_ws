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
                '-y', '9.2',
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
                '-y', '10.0',
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
                '-y', '10.8',
                '-z', '0.1',
                '-Y', '0.0'
            ],
            output='screen',
        ),
        
        # Bridge for RED car (cmd_vel, odometry, lidar scan, TF)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_car1',
            arguments=[
                '/model/car1_red/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/model/car1_red/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                '/red_scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/model/car1_red/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            ],
            remappings=[
                ('/model/car1_red/cmd_vel', '/red/cmd_vel'),
                ('/model/car1_red/odometry', '/red/odometry'),
                ('/red_scan', '/red/scan'),
                ('/model/car1_red/tf', '/tf'),
            ],
            output='screen',
        ),

        # NOTE: map -> odom is published by AMCL in planning.launch.py

        # Static TF: laser -> car1_red/base_link/lidar (lidar frame alias)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_frame_car1',
            arguments=['0', '0', '0', '0', '0', '0', 'laser', 'car1_red/base_link/lidar'],
            parameters=[{'use_sim_time': True}],
        ),

        # Fallback static TFs in case Ackermann plugin ignores frame_id params
        # odom -> car1_red/odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_frame_car1',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'car1_red/odom'],
            parameters=[{'use_sim_time': True}],
        ),

        # car1_red/base_link -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_frame_car1',
            arguments=['0', '0', '0', '0', '0', '0', 'car1_red/base_link', 'base_link'],
            parameters=[{'use_sim_time': True}],
        ),

        # Bridge for BLUE car (cmd_vel, odometry only - scan shared topic issue)
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

        # Bridge for GREEN car (cmd_vel, odometry only - scan shared topic issue)
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
