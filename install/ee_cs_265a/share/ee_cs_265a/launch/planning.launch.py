import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ee_cs_265a')
    map_yaml = os.path.join(pkg_dir, 'maps', 'ground_truth_map.yaml')

    return LaunchDescription([
        # Global Planner - A* on inflated map
        Node(
            package='ee_cs_265a',
            executable='global_planner',
            name='global_planner',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'map_yaml': map_yaml,
                'inflation_radius': 0.2,
                'proximity_cost_max': 1.0,
                'proximity_cost_radius': 0.5,
                'publish_rate': 1.0,
                'waypoint_spacing': 0.2,
                'waypoints': [
                    # Config 1: A* finds shortest path from start to goal
                    # Car faces +x, so goal is to the right
                    0.0, 9.2,        # START (top corridor, red car spawn)
                    7.5, -10.0,      # GOAL (bottom-right corridor)
                ],
            }],
        ),

        # Local Planner - RRT* for dynamic obstacle avoidance
        Node(
            package='ee_cs_265a',
            executable='local_planner',
            name='local_planner',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'scan_topic': '/red/scan',
                'obstacle_radius': 0.5,
                'path_check_distance': 5.0,
                'path_block_threshold': 0.8,
                'rrt_step_size': 0.3,
                'rrt_max_iter': 800,
                'rrt_goal_bias': 0.3,
                'rrt_search_radius': 6.0,
                'rrt_rewire_radius': 1.5,
                'replan_rate': 5.0,
                'rejoin_distance': 3.0,
            }],
        ),

        # Pure Pursuit Controller (follows /local_path from local planner)
        Node(
            package='ee_cs_265a',
            executable='pure_pursuit',
            name='pure_pursuit',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'lookahead_distance': 0.5,
                'min_lookahead': 0.4,
                'max_lookahead': 0.5,
                'lookahead_speed_gain': 0.3,
                'target_speed': 0.4,
                'min_speed': 0.1,
                'speed_curvature_gain': 0.9,
                'max_omega': 5.0,
                'goal_tolerance': 0.1,
            }],
        ),

        # Map Server - serves ground truth map for AMCL localization
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': map_yaml,
            }],
        ),

        # AMCL - corrects map->odom using lidar scan matching
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'global_frame_id': 'map',
                'scan_topic': '/red/scan',
                'odom_topic': '/red/odometry',
                'set_initial_pose': True,
                'initial_pose.x': 0.0,
                'initial_pose.y': 9.2,
                'initial_pose.z': 0.0,
                'initial_pose.yaw': 0.0,
                'max_particles': 2000,
                'min_particles': 500,
                'transform_tolerance': 0.5,
            }],
        ),

        # Lifecycle Manager - auto-activates map_server and amcl
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),

        # Dynamic Agent - BLUE car (clockwise outer loop)
        Node(
            package='ee_cs_265a',
            executable='dynamic_agent',
            name='blue_agent',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'cmd_vel_topic': '/blue/cmd_vel',
                'odom_topic': '/blue/odometry',
                'spawn_x': 0.0,
                'spawn_y': 10.0,
                'spawn_yaw': 0.0,
                'speed': 0.6,
                'waypoint_tolerance': 1.5,
                'kp_angular': 2.0,
                'max_omega': 3.0,
                'loop': True,
                'waypoints': [
                    # Clockwise outer loop
                    7.5, 10.0,       # east on north corridor
                    7.5, 6.0,        # turn south into east corridor
                    7.5, 0.0,        # mid east corridor
                    7.5, -6.0,       # continue south
                    7.5, -10.0,      # south-east corner
                    0.0, -10.0,      # west on south corridor
                    -7.5, -10.0,     # south-west corner
                    -7.5, -6.0,      # turn north into west corridor
                    -7.5, 0.0,       # mid west corridor
                    -7.5, 6.0,       # continue north
                    -7.5, 10.0,      # north-west corner
                    0.0, 10.0,       # back to start
                ],
            }],
        ),

        # Dynamic Agent - GREEN car (counter-clockwise outer loop)
        Node(
            package='ee_cs_265a',
            executable='dynamic_agent',
            name='green_agent',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'cmd_vel_topic': '/green/cmd_vel',
                'odom_topic': '/green/odometry',
                'spawn_x': 0.0,
                'spawn_y': 10.8,
                'spawn_yaw': 0.0,
                'speed': 0.5,
                'waypoint_tolerance': 1.5,
                'kp_angular': 2.0,
                'max_omega': 3.0,
                'loop': True,
                'waypoints': [
                    # Counter-clockwise outer loop
                    -7.5, 10.0,      # west on north corridor
                    -7.5, 6.0,       # turn south into west corridor
                    -7.5, 0.0,       # mid west corridor
                    -7.5, -6.0,      # continue south
                    -7.5, -10.0,     # south-west corner
                    0.0, -10.0,      # east on south corridor
                    7.5, -10.0,      # south-east corner
                    7.5, -6.0,       # turn north into east corridor
                    7.5, 0.0,        # mid east corridor
                    7.5, 6.0,        # continue north
                    7.5, 10.0,       # north-east corner
                    0.0, 10.0,       # back to start
                ],
            }],
        ),

        # Trajectory Plotter - records and plots robot path
        Node(
            package='ee_cs_265a',
            executable='trajectory_plotter',
            name='trajectory_plotter',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'record_interval': 0.2,
                'save_path': '/home/bhargav/ee_cs_265A_ws/src/ee_cs_265a/trajectory_plot.png',
            }],
        ),
    ])
