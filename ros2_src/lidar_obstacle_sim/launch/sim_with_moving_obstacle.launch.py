#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""移動障害物付きの Gazebo シミュレーションを起動する launch ファイル."""

from __future__ import annotations

import os
from typing import Dict, List, Tuple

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _compute_contact_range(
    spawn_position: float, obstacle_speed: float
) -> Tuple[float, float]:
    """移動障害物とロボットの接触想定区間を算出する.

    ロボット速度 1.07 m/s、停止距離 1 m、走行開始距離 20 m を既存プラグイン
    設定と同条件で仮定し、さらにロボット側へ 5 m の余裕を持って禁止区間を
    拡張する。

    Args:
        spawn_position (float): 障害物の初期 x 座標[m]。
        obstacle_speed (float): 障害物の走行速度[m/s]。符号は考慮せず絶対値を使用。

    Returns:
        Tuple[float, float]: (禁止区間開始位置, spawn 位置)。
    """

    robot_speed = 1.07
    start_distance = 20.0
    stop_distance = 1.0

    closing_speed = robot_speed + abs(obstacle_speed)
    if closing_speed <= 0.0:
        return spawn_position, spawn_position

    travel = max(start_distance - stop_distance, 0.0) * abs(obstacle_speed) / closing_speed
    contact_position = spawn_position - travel
    safety_start = contact_position - 5.0

    return safety_start, spawn_position


def generate_launch_description() -> LaunchDescription:
    """移動障害物シナリオ用の LaunchDescription を生成する."""

    enable_pylons_arg = DeclareLaunchArgument(
        'enable_pylons',
        default_value='true',
        description='Enable random pylon spawning: true / false',
    )

    enable_pylons = LaunchConfiguration('enable_pylons')

    pkg_share = get_package_share_directory('lidar_obstacle_sim')
    pkg_prefix = get_package_prefix('lidar_obstacle_sim')

    world_path = os.path.join(pkg_share, 'worlds', 'road_course_moving_obstacle.world')

    models_path = os.path.join(pkg_share, 'models')
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if gazebo_model_path:
        gazebo_model_path = os.pathsep.join([gazebo_model_path, models_path])
    else:
        gazebo_model_path = models_path

    gazebo_ros_prefix = get_package_prefix('gazebo_ros')
    gazebo_ros_plugin_path = os.path.join(gazebo_ros_prefix, 'lib')
    plugin_path = os.path.join(pkg_prefix, 'lib')
    gazebo_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    plugin_paths = [
        path for path in [gazebo_plugin_path, gazebo_ros_plugin_path, plugin_path] if path
    ]
    gazebo_plugin_path = os.pathsep.join(plugin_paths)

    gazebo_cmd: List[str] = [
        'gazebo',
        '--verbose',
        world_path,
        '-s',
        'libgazebo_ros_init.so',
        '-s',
        'libgazebo_ros_factory.so',
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

    simple_robot_path = os.path.join(pkg_share, 'models', 'simple_robot', 'model.sdf')
    pylon_model_path = os.path.join(pkg_share, 'models', 'pylon', 'model.sdf')

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity',
            'simple_robot',
            '-file',
            simple_robot_path,
            '-x',
            '1.4',
            '-y',
            '0',
            '-z',
            '0.5',
        ],
        output='screen',
    )

    contact_range_front = _compute_contact_range(40.0, 0.4)
    contact_range_back = _compute_contact_range(90.0, 1.0)

    spawn_pylons_node = Node(
        package='lidar_obstacle_sim',
        executable='random_pylon_spawner.py',
        name='spawn_random_pylons',
        output='screen',
        condition=IfCondition(enable_pylons),
        parameters=[
            {'model_path': pylon_model_path},
            {'road_type': 'straight'},
            {'road_width': 3.0},
            {'min_longitudinal_spacing': 5.0},
            {'longitudinal_margin': 1.0},
            {'restricted_start_s_list': [contact_range_front[0], contact_range_back[0]]},
            {'restricted_end_s_list': [contact_range_front[1], contact_range_back[1]]},
            {'restricted_half_width_list': [0.4, 0.4]},
        ],
    )

    fake_amcl_node = Node(
        package='lidar_obstacle_sim',
        executable='fake_amcl_pose.py',
        name='fake_amcl_pose',
        output='screen',
        parameters=[
            {'odom_topic': '/ypspur_ros/odom'},
            {'amcl_topic': '/amcl_pose'},
        ],
    )

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    base_to_mid360_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_mid360',
        arguments=['-0.425', '0', '1.005', '0', '0', '0', 'base_link', 'mid360_frame'],
    )

    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_laser',
        arguments=['0.075', '0', '0.49', '0', '0', '0', 'base_link', 'laser'],
    )

    return LaunchDescription([
        enable_pylons_arg,
        gazebo_process,
        spawn_robot_node,
        spawn_pylons_node,
        fake_amcl_node,
        map_to_odom_tf,
        base_to_mid360_tf,
        base_to_laser_tf,
    ])

