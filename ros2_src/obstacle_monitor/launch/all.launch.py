# -*- coding: utf-8 -*-
"""Obstacle monitorとレーザスキャンシミュレータを同時に起動するlaunch."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Obstacle monitor関連ノードをまとめて起動するLaunchDescriptionを生成する."""

    pkg_share = FindPackageShare('obstacle_monitor')
    map_path = PathJoinSubstitution([pkg_share, 'map', 'map.bmp'])
    param_file = PathJoinSubstitution([pkg_share, 'params', 'default.yaml'])

    laser_scan_simulator_node = Node(
        package='obstacle_monitor',
        executable='laser_scan_simulator',
        name='laser_scan_simulator',
        output='screen',
        parameters=[
            {
                'map_image_path': map_path,
                'map_resolution_m': 0.2,
                'publish_rate_hz': 40.0,
                'enable_debug_view': True,
                'debug_view_rate_hz': 10.0,
                'angle_min_deg': -135.0,
                'angle_max_deg': 135.0,
                'angle_increment_deg': 0.25,
                'range_min': 0.05,
                'range_max': 30.0,
            }
        ],
    )

    obstacle_monitor_node = Node(
        package='obstacle_monitor',
        executable='obstacle_monitor',
        name='obstacle_monitor',
        output='screen',
        parameters=[param_file],
    )

    return LaunchDescription([
        laser_scan_simulator_node,
        obstacle_monitor_node,
    ])
