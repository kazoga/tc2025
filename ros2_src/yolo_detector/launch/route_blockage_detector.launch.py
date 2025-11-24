from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('yolo_detector')
    default_param = PathJoinSubstitution([pkg_share, 'params', 'route_blockage_detector.yaml'])

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param,
        description='route_blockage_detectorのパラメータファイル',
    )

    route_blockage_node = Node(
        package='yolo_detector',
        executable='route_blockage_detector',
        name='route_blockage_detector',
        output='screen',
        parameters=[LaunchConfiguration('param_file')],
    )

    return LaunchDescription([param_file_arg, route_blockage_node])
