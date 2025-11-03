from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('route_planner')
    default_param = PathJoinSubstitution([pkg_share, 'params', 'default.yaml'])
    default_csv_dir = PathJoinSubstitution([pkg_share, 'routes'])

    param_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param,
        description='route_plannerノードの既定パラメータファイル',
    )
    csv_base_dir_arg = DeclareLaunchArgument(
        'csv_base_dir',
        default_value=default_csv_dir,
        description='CSVファイルの基準ディレクトリ (デフォルト: <pkg_share>/routes)',
    )
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='route_planner',
        description='ノード名',
    )
    get_service_arg = DeclareLaunchArgument(
        'get_route_service',
        default_value='/get_route',
        description='提供するGetRouteサービス名',
    )
    update_service_arg = DeclareLaunchArgument(
        'update_route_service',
        default_value='/update_route',
        description='提供するUpdateRouteサービス名',
    )

    param_file = LaunchConfiguration('param_file')
    csv_base_dir = LaunchConfiguration('csv_base_dir')
    node_name = LaunchConfiguration('node_name')
    get_route_service = LaunchConfiguration('get_route_service')
    update_route_service = LaunchConfiguration('update_route_service')

    route_planner_node = Node(
        package='route_planner',
        executable='route_planner',
        name=node_name,
        output='screen',
        emulate_tty=True,
        parameters=[
            param_file,
            {
                'csv_base_dir': csv_base_dir,
            },
        ],
        remappings=[
            ('get_route', get_route_service),
            ('update_route', update_route_service),
        ],
    )

    return LaunchDescription([
        param_arg,
        csv_base_dir_arg,
        node_name_arg,
        get_service_arg,
        update_service_arg,
        route_planner_node,
    ])
