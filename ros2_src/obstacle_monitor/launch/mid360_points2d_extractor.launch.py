# -*- coding: utf-8 -*-
"""mid360_points2d_extractorノードを起動するlaunchファイル."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """mid360_points2d_extractorノードのLaunchDescriptionを生成する."""

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
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='/mid360_frame',
        description='出力フレームID (デフォルト: /mid360_frame)',
    )
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/mid360/livox/lidar',
        description='入力PointCloud2トピック名',
    )
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/mid360/points2d',
        description='出力PointCloud2トピック名',
    )

    # --- LaunchConfiguration ---
    height_min = LaunchConfiguration('height_min_m')
    height_max = LaunchConfiguration('height_max_m')
    max_radius = LaunchConfiguration('max_radius_m')
    range_min = LaunchConfiguration('range_min_m')
    frame_id = LaunchConfiguration('frame_id')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')

    # --- ノード定義 ---
    node = Node(
        package='obstacle_monitor',
        executable='mid360_points2d_extractor',
        name='mid360_points2d_extractor',
        output='screen',
        parameters=[
            {
                'height_min_m': height_min,
                'height_max_m': height_max,
                'max_radius_m': max_radius,
                'range_min_m': range_min,
                'frame_id': frame_id,
                'input_topic': input_topic,
                'output_topic': output_topic,
            }
        ],
    )

    # --- LaunchDescriptionに登録 ---
    return LaunchDescription([
        height_min_arg,
        height_max_arg,
        max_radius_arg,
        range_min_arg,
        frame_id_arg,
        input_topic_arg,
        output_topic_arg,
        node,
    ])
