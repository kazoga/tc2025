from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_navigator',
            executable='robot_simulator',
            name='robot_simulator',
            output='screen',
            parameters=[{
                'cycle_hz': 10.0,
                'publish_tf': True,
                'pose_noise_std_m': 0.0,
                'yaw_noise_std_deg': 0.0
            }],
        ),
    ])

