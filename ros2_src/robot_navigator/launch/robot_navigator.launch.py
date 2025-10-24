from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('robot_navigator')
    default_param = PathJoinSubstitution([pkg_share, 'params', 'default.yaml'])

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param,
        description='robot_navigator ノードの既定パラメータファイル',
    )
    node_name_arg = DeclareLaunchArgument(
        'node_name', default_value='robot_navigator', description='ノード名'
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic', default_value='/scan', description='LaserScan 入力トピック'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odom', description='オドメトリ入力トピック'
    )
    amcl_pose_topic_arg = DeclareLaunchArgument(
        'amcl_pose_topic', default_value='/amcl_pose', description='現在姿勢入力トピック'
    )
    active_target_topic_arg = DeclareLaunchArgument(
        'active_target_topic',
        default_value='/active_target',
        description='PoseStamped 目標入力トピック',
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/cmd_vel', description='Twist 出力トピック'
    )
    marker_topic_arg = DeclareLaunchArgument(
        'marker_topic',
        default_value='/direction_marker',
        description='可視化 Marker 出力トピック',
    )
    obstacle_hint_topic_arg = DeclareLaunchArgument(
        'obstacle_hint_topic',
        default_value='/obstacle_avoidance_hint',
        description='障害物ヒント入力トピック',
    )

    param_file = LaunchConfiguration('param_file')
    node_name = LaunchConfiguration('node_name')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    amcl_pose_topic = LaunchConfiguration('amcl_pose_topic')
    active_target_topic = LaunchConfiguration('active_target_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    marker_topic = LaunchConfiguration('marker_topic')
    obstacle_hint_topic = LaunchConfiguration('obstacle_hint_topic')

    navigator_node = Node(
        package='robot_navigator',
        executable='robot_navigator',
        name=node_name,
        output='screen',
        emulate_tty=True,
        parameters=[param_file],
        remappings=[
            ('scan', scan_topic),
            ('odom', odom_topic),
            ('amcl_pose', amcl_pose_topic),
            ('active_target', active_target_topic),
            ('cmd_vel', cmd_vel_topic),
            ('direction_marker', marker_topic),
            ('obstacle_avoidance_hint', obstacle_hint_topic),
        ],
    )

    return LaunchDescription(
        [
            param_file_arg,
            node_name_arg,
            scan_topic_arg,
            odom_topic_arg,
            amcl_pose_topic_arg,
            active_target_topic_arg,
            cmd_vel_topic_arg,
            marker_topic_arg,
            obstacle_hint_topic_arg,
            navigator_node,
        ]
    )
