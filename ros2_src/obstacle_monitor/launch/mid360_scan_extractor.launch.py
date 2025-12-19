# -*- coding: utf-8 -*-
"""mid360_scan_extractorノードを起動するlaunchファイル."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """mid360_scan_extractorノードのLaunchDescriptionを生成する."""

    # --- launch引数の宣言 ---
    height_min_arg = DeclareLaunchArgument(
        'height_min_m',
        default_value='0.1',
        description='Z軸下限[m] (デフォルト: 0.1)',
    )
    height_max_arg = DeclareLaunchArgument(
        'height_max_m',
        default_value='1.0',
        description='Z軸上限[m] (デフォルト: 1.0)',
    )
    sensor_height_arg = DeclareLaunchArgument(
        'sensor_height_m',
        default_value='1.005',
        description='センサ高さ[m] (デフォルト: 1.005)',
    )
    max_radius_arg = DeclareLaunchArgument(
        'max_radius_m',
        default_value='20.0',
        description='水平距離の上限[m] (デフォルト: 20.0)',
    )
    range_min_arg = DeclareLaunchArgument(
        'range_min_m',
        default_value='0.05',
        description='水平距離の下限[m] (デフォルト: 0.05)',
    )
    angle_min_arg = DeclareLaunchArgument(
        'angle_min_deg',
        default_value='-180.0',
        description='LaserScanの開始角度[deg] (デフォルト: -180.0)',
    )
    angle_max_arg = DeclareLaunchArgument(
        'angle_max_deg',
        default_value='180.0',
        description='LaserScanの終了角度[deg] (デフォルト: 180.0)',
    )
    angle_increment_arg = DeclareLaunchArgument(
        'angle_increment_deg',
        default_value='0.5',
        description='LaserScanの角度分解能[deg] (デフォルト: 0.5)',
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='mid360_frame',
        description='出力フレームID (デフォルト: mid360_frame)',
    )
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/mid360/livox/lidar',
        description='入力PointCloud2トピック名',
    )
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/mid360/scan',
        description='出力LaserScanトピック名',
    )
    viewer_topic_arg = DeclareLaunchArgument(
        'viewer_topic',
        default_value='/mid360/scan_viewer',
        description='画像トピック名',
    )
    viewer_x_range_arg = DeclareLaunchArgument(
        'viewer_x_range_m',
        default_value='20.0',
        description='画像化する前方距離範囲[m] (デフォルト: 20.0)',
    )
    viewer_y_range_arg = DeclareLaunchArgument(
        'viewer_y_range_m',
        default_value='5.0',
        description='画像化する左右距離範囲[m] (デフォルト: 5.0)',
    )
    viewer_pixel_pitch_arg = DeclareLaunchArgument(
        'viewer_pixel_pitch',
        default_value='40',
        description='画像化の解像度[pix/m] (デフォルト: 40)',
    )
    viewer_grid_interval_arg = DeclareLaunchArgument(
        'viewer_grid_interval_m',
        default_value='5.0',
        description='進行方向の補助線間隔[m] (デフォルト: 5.0)',
    )
    robot_width_arg = DeclareLaunchArgument(
        'robot_width_m',
        default_value='0.8',
        description='ロボット幅ライン描画用の幅[m] (デフォルト: 0.8)',
    )

    # --- LaunchConfiguration ---
    height_min = LaunchConfiguration('height_min_m')
    height_max = LaunchConfiguration('height_max_m')
    sensor_height = LaunchConfiguration('sensor_height_m')
    max_radius = LaunchConfiguration('max_radius_m')
    range_min = LaunchConfiguration('range_min_m')
    angle_min = LaunchConfiguration('angle_min_deg')
    angle_max = LaunchConfiguration('angle_max_deg')
    angle_increment = LaunchConfiguration('angle_increment_deg')
    frame_id = LaunchConfiguration('frame_id')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    viewer_topic = LaunchConfiguration('viewer_topic')
    viewer_x_range = LaunchConfiguration('viewer_x_range_m')
    viewer_y_range = LaunchConfiguration('viewer_y_range_m')
    viewer_pixel_pitch = LaunchConfiguration('viewer_pixel_pitch')
    viewer_grid_interval = LaunchConfiguration('viewer_grid_interval_m')
    robot_width = LaunchConfiguration('robot_width_m')

    # --- ノード定義 ---
    node = Node(
        package='obstacle_monitor',
        executable='mid360_scan_extractor',
        name='mid360_scan_extractor',
        output='screen',
        parameters=[
            {
                'height_min_m': height_min,
                'height_max_m': height_max,
                'sensor_height_m': sensor_height,
                'max_radius_m': max_radius,
                'range_min_m': range_min,
                'angle_min_deg': angle_min,
                'angle_max_deg': angle_max,
                'angle_increment_deg': angle_increment,
                'frame_id': frame_id,
                'input_topic': input_topic,
                'output_topic': output_topic,
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
        height_min_arg,
        height_max_arg,
        sensor_height_arg,
        max_radius_arg,
        range_min_arg,
        angle_min_arg,
        angle_max_arg,
        angle_increment_arg,
        frame_id_arg,
        input_topic_arg,
        output_topic_arg,
        viewer_topic_arg,
        viewer_x_range_arg,
        viewer_y_range_arg,
        viewer_pixel_pitch_arg,
        viewer_grid_interval_arg,
        robot_width_arg,
        node,
    ])
