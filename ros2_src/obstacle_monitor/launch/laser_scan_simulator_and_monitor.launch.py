from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('obstacle_monitor')
    default_map_path = PathJoinSubstitution([pkg_share, 'map', 'map.bmp'])

    map_path_arg = DeclareLaunchArgument(
        'map_image_path',
        default_value=default_map_path,
        description='マップ画像（BMP）のパス (デフォルト: <pkg_share>/map/map.bmp)'
    )

    simulator_node = Node(
        package='obstacle_monitor',
        executable='laser_scan_simulator',
        name='laser_scan_simulator',
        output='screen',
        parameters=[
            {
                'map_image_path': LaunchConfiguration('map_image_path'),
                'map_resolution_m': 0.2,
                'publish_rate_hz': 40.0,
                'enable_debug_view': True,
                'debug_view_rate_hz': 10.0,
            }
        ],
    )

    monitor_node = Node(
        package='obstacle_monitor',
        executable='obstacle_monitor',
        name='obstacle_monitor',
        output='screen',
        parameters=[
            {
                'pub_rate_hz': 10.0,
                'front_half_deg': 12.5,
                'stop_dist_m': 1.5,
                'robot_width_m': 0.45,
                'safety_margin_m': 0.10,
                'enable_viewer': True,
                'viewer_image_size': 500,
                'viewer_scale_px_per_m': 50.0,
            }
        ],
    )

    return LaunchDescription([
        map_path_arg,
        simulator_node,
        monitor_node,
    ])
