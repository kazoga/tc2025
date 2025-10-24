# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """LaserScanSimulatorノードを起動するためのLaunchDescriptionを生成する."""

    pkg_share = FindPackageShare('obstacle_monitor')
    default_map_path = PathJoinSubstitution([pkg_share, 'map', 'map.bmp'])

    # --- launch引数の宣言 ---
    map_path_arg = DeclareLaunchArgument(
        'map_image_path',
        default_value=default_map_path,
        description='マップ画像（BMP）のパス (デフォルト: <pkg_share>/map/map.bmp)'
    )

    # --- ノード定義 ---
    node = Node(
        package='obstacle_monitor',
        executable='laser_scan_simulator',
        name='laser_scan_simulator',
        output='screen',
        parameters=[
            {
                # Launch引数を使用してパスを渡す
                'map_image_path': LaunchConfiguration('map_image_path'),
                'map_resolution_m': 0.2,
                'publish_rate_hz': 40.0,
                'enable_debug_view': True,
                'debug_view_rate_hz': 10.0,
            },
        ],
    )

    # --- LaunchDescriptionに登録 ---
    return LaunchDescription([
        map_path_arg,
        node,
    ])

