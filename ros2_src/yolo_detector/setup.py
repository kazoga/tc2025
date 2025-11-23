import os
from glob import glob
from typing import List, Tuple

from setuptools import setup

package_name = 'yolo_detector'


def list_data_files(target_dir: str) -> List[Tuple[str, List[str]]]:
    entries = []
    for root, _, files in os.walk(target_dir):
        if not files:
            continue
        install_dir = os.path.join('share', package_name, os.path.relpath(root, '.'))
        src_files = [os.path.join(root, file_name) for file_name in files]
        entries.append((install_dir, src_files))
    return entries


# modelsディレクトリ内のファイルのみを取得（ディレクトリは除外）
model_files = [f for f in glob('models/*') if os.path.isfile(f)]

# scriptsディレクトリのファイルも含める
script_files = glob('scripts/*.py')

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

if model_files:
    data_files.append((os.path.join('share', package_name, 'models'), model_files))

if script_files:
    data_files.append((os.path.join('share', package_name, 'scripts'), script_files))

if os.path.isdir('params'):
    data_files.extend(list_data_files('params'))

if os.path.isdir('launch'):
    data_files.extend(list_data_files('launch'))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
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
