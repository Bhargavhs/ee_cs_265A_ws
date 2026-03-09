import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ee_cs_265a')
    slam_config = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')

    return LaunchDescription([
        # SLAM Toolbox for mapping
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config,
                {'use_sim_time': True},
            ],
        ),

    ])
