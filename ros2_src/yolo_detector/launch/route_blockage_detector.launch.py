from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('yolo_detector')
    default_param = PathJoinSubstitution([pkg_share, 'params', 'route_blockage_detector.yaml'])

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='シミュレーション時間の利用有無'
    )
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
        parameters=[
            LaunchConfiguration('param_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([use_sim_time_arg, param_file_arg, route_blockage_node])
