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

        # Pure Pursuit Controller
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

        # Trajectory Plotter - records and plots robot path
        Node(
            package='ee_cs_265a',
            executable='trajectory_plotter',
            name='trajectory_plotter',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'record_interval': 0.2,
                'save_path': '/tmp/trajectory_plot.png',
            }],
        ),
    ])
