"""robot_console を起動するための launch スクリプト。"""

import os
from pathlib import Path
from typing import List

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch_ros.actions import Node


def _resolve_log_directory(context: LaunchContext) -> str:
    """ROS2 の規約に従い、ログディレクトリを決定する。"""

    for key in ('log_dir', 'ros_log_dir'):
        value = context.launch_configurations.get(key)
        if value:
            return str(Path(value).expanduser())
    env_log_dir = os.environ.get('ROS_LOG_DIR')
    if env_log_dir:
        return str(Path(env_log_dir).expanduser())
    ros_home = os.environ.get('ROS_HOME', str(Path.home() / '.ros'))
    return str(Path(ros_home).expanduser() / 'log')


def _launch_setup(context: LaunchContext, *args, **kwargs) -> List[Node]:
    """ノード定義を生成する内部関数。"""

    log_dir = _resolve_log_directory(context)
    node = Node(
        package='robot_console',
        executable='robot_console',
        name='robot_console',
        output='screen',
        parameters=[{'console_log_directory': log_dir}],
    )
    return [node]


def generate_launch_description() -> LaunchDescription:
    """robot_console ノードを単独起動する。"""

    return LaunchDescription([OpaqueFunction(function=_launch_setup)])
