from typing import Any, List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _parse_checkpoint_labels(raw_value: str) -> List[str]:
    """DeclareLaunchArgumentで受け取った文字列をチェックポイント配列へ変換する。"""

    text = raw_value.replace("\n", ",").strip()
    if not text:
        return []
    if text.startswith("[") and text.endswith("]"):
        text = text[1:-1]
    labels: List[str] = []
    for chunk in text.split(","):
        label = chunk.strip().strip("\"'")
        if label:
            labels.append(label)
    return labels


def _get_config_value(context: Any, name: str) -> str:
    """LaunchContext から指定名の設定値を取得するユーティリティ."""

    # LaunchConfiguration.perform() で取得を試み、失敗した場合は直接辞書を参照する。
    try:
        value = LaunchConfiguration(name).perform(context)
    except Exception:  # pragma: no cover - context 実装差異へのフォールバック
        value = None
    if value is None:
        launch_configs = getattr(context, 'launch_configurations', None)
        if isinstance(launch_configs, dict):
            value = launch_configs.get(name, '')
    if value is None:
        return ''
    return str(value)


def _create_node(context, *args, **kwargs):
    """LaunchDescription生成時に評価されるノード設定を構築する。"""

    param_file_value = _get_config_value(context, 'param_file')
    start_label_value = _get_config_value(context, 'start_label').strip()
    goal_label_value = _get_config_value(context, 'goal_label').strip()
    checkpoint_value = _get_config_value(context, 'checkpoint_labels')

    parameters: List[object] = [param_file_value]
    overrides = {}
    if start_label_value:
        overrides['start_label'] = start_label_value
    if goal_label_value:
        overrides['goal_label'] = goal_label_value
    checkpoint_labels = _parse_checkpoint_labels(checkpoint_value)
    if checkpoint_labels:
        overrides['checkpoint_labels'] = checkpoint_labels
    if overrides:
        parameters.append(overrides)

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
        parameters=parameters,
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
    return [node]


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
    checkpoint_labels_arg = DeclareLaunchArgument(
        'checkpoint_labels', default_value='', description='チェックポイントラベル（カンマ区切り）'
    )

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

    return LaunchDescription([
        param_arg,
        start_label_arg,
        goal_label_arg,
        checkpoint_labels_arg,
        active_route_topic_arg,
        route_state_topic_arg,
        mission_info_topic_arg,
        manager_status_topic_arg,
        report_stuck_service_arg,
        planner_get_service_arg,
        planner_update_service_arg,
        OpaqueFunction(function=_create_node),
    ])
