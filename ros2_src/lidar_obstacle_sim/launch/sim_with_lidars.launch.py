#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""LiDAR + パイロン環境の Gazebo シミュレーションを起動する launch ファイル."""

from __future__ import annotations

import os
from typing import List, Dict

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """LaunchDescription を生成する."""
    pkg_share = get_package_share_directory('lidar_obstacle_sim')

    world_path = os.path.join(pkg_share, 'worlds', 'road_course.world')
    pylon_model_path = os.path.join(pkg_share, 'models', 'pylon', 'model.sdf')

    # Gazebo model path に本パッケージの models ディレクトリを追加
    models_path = os.path.join(pkg_share, 'models')
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if gazebo_model_path:
        gazebo_model_path = gazebo_model_path + os.pathsep + models_path
    else:
        gazebo_model_path = models_path

    # Gazebo plugin path に gazebo_ros のライブラリを明示的に追加
    gazebo_ros_prefix = get_package_prefix('gazebo_ros')
    gazebo_ros_plugin_path = os.path.join(gazebo_ros_prefix, 'lib')
    gazebo_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    if gazebo_plugin_path:
        gazebo_plugin_path = gazebo_plugin_path + os.pathsep + gazebo_ros_plugin_path
    else:
        gazebo_plugin_path = gazebo_ros_plugin_path

    gazebo_cmd: List[str] = [
        'gazebo',
        '--verbose',
        world_path,
        '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so',
    ]

    gazebo_env: Dict[str, str] = {
        'GAZEBO_MODEL_PATH': gazebo_model_path,
        'GAZEBO_PLUGIN_PATH': gazebo_plugin_path,
    }

    gazebo_process = ExecuteProcess(
        cmd=gazebo_cmd,
        output='screen',
        additional_env=gazebo_env,
    )

    # ランダムパイロンスポーナ
    spawn_pylons_node = Node(
        package='lidar_obstacle_sim',
        executable='spawn_random_pylons.py',
        name='spawn_random_pylons',
        output='screen',
        parameters=[
            {'model_path': pylon_model_path},
            {'min_longitudinal_spacing': 5.0},
            {'longitudinal_margin': 1.0},
        ],
    )

    # Gazebo /odom → /amcl_pose へ変換する簡易AMCLノード
    fake_amcl_node = Node(
        package='lidar_obstacle_sim',
        executable='fake_amcl_pose.py',
        name='fake_amcl_pose',
        output='screen',
        parameters=[
            {'odom_topic': '/odom'},
            {'amcl_topic': '/amcl_pose'},
        ],
    )

    return LaunchDescription([
        gazebo_process,
        spawn_pylons_node,
        fake_amcl_node,
    ])
