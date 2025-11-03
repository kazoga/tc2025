import os
from glob import glob

from setuptools import setup
from typing import List, Tuple

package_name = 'obstacle_monitor'

def list_data_files(target_dir: str) -> List[Tuple[str, List[str]]]:
    entries = []
    for root, _, files in os.walk(target_dir):
        if not files:
            continue
        install_dir = os.path.join('share', package_name, os.path.relpath(root, '.'))
        src_files = [os.path.join(root, file_name) for file_name in files]
        entries.append((install_dir, src_files))
    return entries

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

map_files = glob('map/*')
if map_files:
    data_files.append((os.path.join('share', package_name, 'map'), map_files))

if os.path.isdir('launch'):
    data_files.extend(list_data_files('launch'))

if os.path.isdir('params'):
    data_files.extend(list_data_files('params'))

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auto',
    maintainer_email='auto@example.com',
    description='Obstacle monitor node for 2D LiDAR (Phase2).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_monitor = obstacle_monitor.obstacle_monitor_node:main',
            'laser_scan_simulator = obstacle_monitor.laser_scan_simulator_node:main',
        ],
    },
)
