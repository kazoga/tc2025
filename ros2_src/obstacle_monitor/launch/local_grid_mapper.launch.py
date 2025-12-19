# -*- coding: utf-8 -*-
"""local_grid_mapperノードを起動するlaunchファイル."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """local_grid_mapperノードのLaunchDescriptionを生成する."""

    # --- launch引数の宣言 ---
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='base_link',
        description='変換先フレームID (デフォルト: base_link)',
    )
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LaserScanトピック名',
    )
    mid_topic_arg = DeclareLaunchArgument(
        'mid_topic',
        default_value='/mid360/points2d',
        description='Mid-360のPointCloud2トピック名',
    )
    viewer_topic_arg = DeclareLaunchArgument(
        'viewer_topic',
        default_value='/grid_viewer',
        description='可視化画像トピック名',
    )
    viewer_x_range_arg = DeclareLaunchArgument(
        'viewer_x_range_m',
        default_value='20.0',
        description='描画X方向範囲[m] (デフォルト: 20.0)',
    )
    viewer_y_range_arg = DeclareLaunchArgument(
        'viewer_y_range_m',
        default_value='5.0',
        description='描画Y方向範囲[m] (デフォルト: 5.0)',
    )
    viewer_pixel_pitch_arg = DeclareLaunchArgument(
        'viewer_pixel_pitch',
        default_value='40',
        description='ピクセル密度[pix/m] (デフォルト: 40)',
    )
    viewer_grid_interval_arg = DeclareLaunchArgument(
        'viewer_grid_interval_m',
        default_value='5.0',
        description='補助線間隔[m] (デフォルト: 5.0)',
    )
    robot_width_arg = DeclareLaunchArgument(
        'robot_width_m',
        default_value='0.8',
        description='ロボット幅[m] (デフォルト: 0.8)',
    )

    # --- LaunchConfiguration ---
    target_frame = LaunchConfiguration('target_frame')
    scan_topic = LaunchConfiguration('scan_topic')
    mid_topic = LaunchConfiguration('mid_topic')
    viewer_topic = LaunchConfiguration('viewer_topic')
    viewer_x_range = LaunchConfiguration('viewer_x_range_m')
    viewer_y_range = LaunchConfiguration('viewer_y_range_m')
    viewer_pixel_pitch = LaunchConfiguration('viewer_pixel_pitch')
    viewer_grid_interval = LaunchConfiguration('viewer_grid_interval_m')
    robot_width = LaunchConfiguration('robot_width_m')

    # --- ノード定義 ---
    node = Node(
        package='obstacle_monitor',
        executable='local_grid_mapper',
        name='local_grid_mapper',
        output='screen',
        parameters=[
            {
                'target_frame': target_frame,
                'scan_topic': scan_topic,
                'mid_topic': mid_topic,
                'viewer_topic': viewer_topic,
                'viewer_x_range_m': viewer_x_range,
                'viewer_y_range_m': viewer_y_range,
                'viewer_pixel_pitch': viewer_pixel_pitch,
                'viewer_grid_interval_m': viewer_grid_interval,
                'robot_width_m': robot_width,
            }
        ],
    )

    # --- LaunchDescriptionに登録 ---
    return LaunchDescription([
        target_frame_arg,
        scan_topic_arg,
        mid_topic_arg,
        viewer_topic_arg,
        viewer_x_range_arg,
        viewer_y_range_arg,
        viewer_pixel_pitch_arg,
        viewer_grid_interval_arg,
        robot_width_arg,
        node,
    ])

