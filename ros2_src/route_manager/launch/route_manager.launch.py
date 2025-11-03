from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('route_manager')
    default_param = PathJoinSubstitution([pkg_share, 'params', 'default.yaml'])

    param_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param,
        description='route_managerノードの既定パラメータファイル',
    )
    start_label_arg = DeclareLaunchArgument('start_label', default_value='', description='始点ラベル')
    goal_label_arg = DeclareLaunchArgument('goal_label', default_value='', description='終点ラベル')

    active_route_topic_arg = DeclareLaunchArgument(
        'active_route_topic', default_value='/active_route', description='配信するルートトピック名'
    )
    route_state_topic_arg = DeclareLaunchArgument(
        'route_state_topic', default_value='/route_state', description='RouteState配信トピック名'
    )
    mission_info_topic_arg = DeclareLaunchArgument(
        'mission_info_topic', default_value='/mission_info', description='MissionInfo配信トピック名'
    )
    manager_status_topic_arg = DeclareLaunchArgument(
        'manager_status_topic', default_value='/manager_status', description='ManagerStatus配信トピック名'
    )
    report_stuck_service_arg = DeclareLaunchArgument(
        'report_stuck_service', default_value='/report_stuck', description='ReportStuckサービス名'
    )
    planner_get_service_arg = DeclareLaunchArgument(
        'planner_get_service', default_value='/get_route', description='planner側GetRouteサービス名'
    )
    planner_update_service_arg = DeclareLaunchArgument(
        'planner_update_service', default_value='/update_route', description='planner側UpdateRouteサービス名'
    )

    param_file = LaunchConfiguration('param_file')
    start_label = LaunchConfiguration('start_label')
    goal_label = LaunchConfiguration('goal_label')

    active_route_topic = LaunchConfiguration('active_route_topic')
    route_state_topic = LaunchConfiguration('route_state_topic')
    mission_info_topic = LaunchConfiguration('mission_info_topic')
    manager_status_topic = LaunchConfiguration('manager_status_topic')
    report_stuck_service = LaunchConfiguration('report_stuck_service')
    planner_get_service = LaunchConfiguration('planner_get_service')
    planner_update_service = LaunchConfiguration('planner_update_service')

    node = Node(
        package='route_manager',
        executable='route_manager',
        name='route_manager',
        output='screen',
        emulate_tty=True,
        parameters=[
            param_file,
            {
                'start_label': start_label,
                'goal_label': goal_label,
            },
        ],
        remappings=[
            ('active_route', active_route_topic),
            ('route_state', route_state_topic),
            ('mission_info', mission_info_topic),
            ('manager_status', manager_status_topic),
            ('report_stuck', report_stuck_service),
            ('get_route', planner_get_service),
            ('update_route', planner_update_service),
        ],
    )

    return LaunchDescription([
        param_arg,
        start_label_arg,
        goal_label_arg,
        active_route_topic_arg,
        route_state_topic_arg,
        mission_info_topic_arg,
        manager_status_topic_arg,
        report_stuck_service_arg,
        planner_get_service_arg,
        planner_update_service_arg,
        node,
    ])
