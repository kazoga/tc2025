# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_monitor',
            executable='obstacle_monitor',
            name='obstacle_monitor',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('obstacle_monitor'),
                    'params',
                    'default.yaml',
                ])
            ],
        )
    ])
