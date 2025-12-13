#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""移動障害物付きの Gazebo シミュレーションを起動する launch ファイル."""

from __future__ import annotations

import os
from typing import Dict, List

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """移動障害物シナリオ用の LaunchDescription を生成する."""
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

    simple_robot_path = os.path.join(pkg_share, 'models', 'simple_robot', 'model.sdf')

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
        gazebo_process,
        spawn_robot_node,
        fake_amcl_node,
        map_to_odom_tf,
        base_to_mid360_tf,
        base_to_laser_tf,
    ])

