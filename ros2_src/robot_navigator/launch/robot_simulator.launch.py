from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/ypspur_ros/odom',
        description='シミュレータが配信するオドメトリトピック',
    )

    odom_topic = LaunchConfiguration('odom_topic')

    simulator_node = Node(
        package='robot_navigator',
        executable='robot_simulator',
        name='robot_simulator',
        output='screen',
        parameters=[
            {
                'cycle_hz': 10.0,
                'publish_tf': True,
                'pose_noise_std_m': 0.0,
                'yaw_noise_std_deg': 0.0,
            }
        ],
        remappings=[
            ('odom', odom_topic),
            ('/odom', odom_topic),
        ],
    )

    return LaunchDescription([
        odom_topic_arg,
        simulator_node,
    ])
