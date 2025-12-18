# -*- coding: utf-8 -*-
"""rolling grid デバッグノードを起動する launch ファイル."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """obstacle_monitor_grid_debug ノードを起動する LaunchDescription を生成する."""

    # --- launch 引数の宣言 ---
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='odom',
        description='tf 変換の対象フレーム（固定グリッドの基準座標）',
    )
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='UTM-30LX LaserScan 入力トピック名',
    )
    mid_topic_arg = DeclareLaunchArgument(
        'mid_topic',
        default_value='/mid360/points2d',
        description='Mid-360 2D PointCloud2 入力トピック名',
    )
    amcl_topic_arg = DeclareLaunchArgument(
        'amcl_topic',
        default_value='/amcl_pose',
        description='ロボット平行移動を追従する AMCL Pose トピック名',
    )
    window_name_arg = DeclareLaunchArgument(
        'window_name',
        default_value='grid_debug',
        description='OpenCV 表示ウィンドウ名',
    )

    target_frame = LaunchConfiguration('target_frame')
    scan_topic = LaunchConfiguration('scan_topic')
    mid_topic = LaunchConfiguration('mid_topic')
    amcl_topic = LaunchConfiguration('amcl_topic')
    window_name = LaunchConfiguration('window_name')

    node = Node(
        package='obstacle_monitor',
        executable='obstacle_monitor_grid_debug',
        name='obstacle_monitor_grid_debug',
        output='screen',
        parameters=[
            {
                'target_frame': target_frame,
                'scan_topic': scan_topic,
                'mid_topic': mid_topic,
                'amcl_topic': amcl_topic,
                'window_name': window_name,
            }
        ],
    )

    return LaunchDescription([
        target_frame_arg,
        scan_topic_arg,
        mid_topic_arg,
        amcl_topic_arg,
        window_name_arg,
        node,
    ])
