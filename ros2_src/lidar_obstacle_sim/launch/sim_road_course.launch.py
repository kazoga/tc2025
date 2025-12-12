#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
汎用 LiDAR + パイロン（任意）付き Gazebo シミュレーション launch ファイル。

パラメータ:
  road_type: straight / crank / scurve
  road_width: 2.0 / 3.0 / 5.0
  enable_pylons: true / false
"""

from __future__ import annotations

import os
from typing import Dict

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # ============================================================
    # Launch Arguments
    # ============================================================
    road_type_arg = DeclareLaunchArgument(
        'road_type',
        default_value='crank',
        description='Road shape: straight / crank / scurve',
    )

    road_width_arg = DeclareLaunchArgument(
        'road_width',
        default_value='5.0',
        description='Road width (meters): 2.0 / 3.0 / 5.0',
    )

    enable_pylons_arg = DeclareLaunchArgument(
        'enable_pylons',
        default_value='true',
        description='Enable random pylon spawning: true / false',
    )

    road_type = LaunchConfiguration('road_type')
    road_width = LaunchConfiguration('road_width')
    enable_pylons = LaunchConfiguration('enable_pylons')

    pkg_share = get_package_share_directory('lidar_obstacle_sim')

    # ============================================================
    # World file selection
    # ============================================================
    # world ファイル名： road_{type}_w{width}.world
    #
    # 有効組み合わせ：
    #   straight: w2, w3, w5
    #   crank:    w3, w5
    #   scurve:   w3, w5
    #
    world_file_name = PythonExpression([
        "'road_' + '",
        road_type,
        "' + '_w' + '",
        road_width,
        "' + '.world'",
    ])

    world_path = PathJoinSubstitution([
        FindPackageShare('lidar_obstacle_sim'),
        'worlds',
        world_file_name,
    ])

    # ============================================================
    # Gazebo environment settings
    # ============================================================
    models_path = os.path.join(pkg_share, 'models')
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if gazebo_model_path:
        gazebo_model_path = gazebo_model_path + os.pathsep + models_path
    else:
        gazebo_model_path = models_path

    gazebo_ros_prefix = get_package_prefix('gazebo_ros')
    gazebo_ros_plugin_path = os.path.join(gazebo_ros_prefix, 'lib')
    gazebo_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    if gazebo_plugin_path:
        gazebo_plugin_path = gazebo_plugin_path + os.pathsep + gazebo_ros_plugin_path
    else:
        gazebo_plugin_path = gazebo_ros_plugin_path

    gazebo_env: Dict[str, str] = {
        'GAZEBO_MODEL_PATH': gazebo_model_path,
        'GAZEBO_PLUGIN_PATH': gazebo_plugin_path,
    }

    gazebo_cmd = [
        'gazebo',
        '--verbose',
        world_path,
        '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so',
    ]

    gazebo_process = ExecuteProcess(
        cmd=gazebo_cmd,
        output='screen',
        additional_env=gazebo_env,
    )

    # ============================================================
    # Optional: Random Pylon Spawner
    # ============================================================
    pylon_model_path = os.path.join(pkg_share, 'models', 'pylon', 'model.sdf')

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

    spawn_pylons_node = Node(
        package='lidar_obstacle_sim',
        executable='random_pylon_spawner.py',
        name='spawn_random_pylons',
        output='screen',
        condition=IfCondition(enable_pylons),
        parameters=[
            {'model_path': pylon_model_path},
            {'road_type': road_type},
            {'road_width': road_width},
            {'min_longitudinal_spacing': 5.0},
            {'longitudinal_margin': 1.0},
        ],
    )

    # ============================================================
    # Fake AMCL (odom → amcl_pose conversion)
    # ============================================================
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

    # ============================================================
    # Required Static TF
    # ============================================================
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

    # ============================================================
    # Final LaunchDescription
    # ============================================================
    return LaunchDescription([
        road_type_arg,
        road_width_arg,
        enable_pylons_arg,

        gazebo_process,
        spawn_robot_node,
        spawn_pylons_node,
        fake_amcl_node,
        map_to_odom_tf,
        base_to_mid360_tf,
        base_to_laser_tf,
    ])

