from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/usb_cam/image_raw', description='購読する画像トピック'
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='', description='NCNNモデルディレクトリのパス'
    )
    detection_interval_arg = DeclareLaunchArgument(
        'detection_interval',
        default_value='1.0',
        description='推論を行うインターバル（秒）',
    )
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.5', description='検出の信頼度閾値'
    )

    yolo_ncnn_node = Node(
        package='yolo_detector',
        executable='yolo_ncnn_node',
        name='yolo_ncnn_node',
        output='screen',
        parameters=[
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'model_path': LaunchConfiguration('model_path'),
                'detection_interval': LaunchConfiguration('detection_interval'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            }
        ],
    )

    return LaunchDescription(
        [
            image_topic_arg,
            model_path_arg,
            detection_interval_arg,
            confidence_threshold_arg,
            yolo_ncnn_node,
        ]
    )
