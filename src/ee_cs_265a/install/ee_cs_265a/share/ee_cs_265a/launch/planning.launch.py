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
                'inflation_radius': 0.30,
                'publish_rate': 1.0,
                'waypoint_spacing': 0.2,
                'waypoints': [
                    # Figure-8: start top-center, right loop then left loop
                    0.0, 7.25,      # start (top corridor, starting line)
                    8.0, 7.25,      # top corridor heading right
                    13.0, 5.5,      # entering right corridor
                    13.0, 0.0,      # right corridor mid
                    13.0, -5.5,     # right corridor lower
                    8.0, -7.25,     # bottom corridor heading left
                    0.0, -7.25,     # bottom center
                    # Cross through center (going up)
                    0.0, -3.5,      # center gap lower
                    0.0, 0.0,       # center crossing
                    0.0, 3.5,       # center gap upper
                    # Left loop
                    -8.0, 7.25,     # top corridor heading left
                    -13.0, 5.5,     # entering left corridor
                    -13.0, 0.0,     # left corridor mid
                    -13.0, -5.5,    # left corridor lower
                    -8.0, -7.25,    # bottom corridor heading right
                    0.0, -7.25,     # bottom center again
                    # Cross through center again (going up)
                    0.0, -3.5,      # center gap lower
                    0.0, 0.0,       # center crossing
                    0.0, 3.5,       # center gap upper
                    0.0, 7.25,      # back to start (loop)
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
                'lookahead_distance': 1.5,
                'min_lookahead': 0.8,
                'max_lookahead': 3.0,
                'lookahead_speed_gain': 1.0,
                'target_speed': 0.7,
                'min_speed': 0.2,
                'speed_curvature_gain': 0.9,
                'max_omega': 3.0,
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
