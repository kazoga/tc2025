from setuptools import setup
import os
from glob import glob

package_name = 'yolo_detector'

# modelsディレクトリ内のファイルのみを取得（ディレクトリは除外）
model_files = [f for f in glob('models/*') if os.path.isfile(f)]

# scriptsディレクトリのファイルも含める
script_files = glob('scripts/*.py')

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), model_files),
        (os.path.join('share', package_name, 'scripts'), script_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='YOLO11n object detection for ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_detector.yolo_node:main',
            'route_blockage_detector = yolo_detector.route_blockage_detector_node:main',
            'yolo_ncnn_node = yolo_detector.yolo_ncnn_node:main',
            'camera_simulator_node = yolo_detector.camera_simulator_node:main',
        ],
    },
)
