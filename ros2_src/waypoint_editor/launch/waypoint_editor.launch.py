from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_editor',
            executable='waypoint_editor_node',
            name='waypoint_editor_node',
            output='screen',
        ),
        Node(
            package='waypoint_editor',
            executable='waypoint_editor_gui',
            name='waypoint_editor_gui',
            output='screen',
        ),
    ])
