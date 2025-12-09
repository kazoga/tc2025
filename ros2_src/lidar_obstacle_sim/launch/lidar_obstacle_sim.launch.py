# -*- coding: utf-8 -*-
"""Gazebo 上に LiDAR 障害物シミュレータ環境を立ち上げる launch ファイル."""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Gazebo を起動し、前後が判別できるシンプルなロボットモデルを配置する."""

    pkg_share = FindPackageShare('lidar_obstacle_sim')
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'lidar_obstacle_sim.world'])

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='起動する SDF ワールドファイルのパス'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    return LaunchDescription([
        world_arg,
        gazebo_launch,
    ])


__all__: List[str] = ['generate_launch_description']
