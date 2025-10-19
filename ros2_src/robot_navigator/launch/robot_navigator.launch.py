from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_navigator',
            executable='robot_navigator',
            name='robot_navigator',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('robot_navigator'),
                    'params',
                    'default.yaml'
                ])
            ],
        ),
    ])

