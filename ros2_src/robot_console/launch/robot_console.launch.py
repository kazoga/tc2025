"""robot_console を起動するための launch スクリプト。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """robot_console ノードを単独起動する。"""

    return LaunchDescription(
        [
            Node(
                package='robot_console',
                executable='robot_console',
                name='robot_console',
                output='screen',
            ),
        ]
    )
