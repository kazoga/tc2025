from glob import glob
import os
from setuptools import setup

package_name = 'lidar_obstacle_sim'


def list_data_files(target_dir: str):
    data_entries = []
    for root, _, files in os.walk(target_dir):
        if not files:
            continue
        src_files = [os.path.join(root, f) for f in files]
        rel_path = os.path.relpath(root, '.')
        install_dir = os.path.join('share', package_name, rel_path)
        data_entries.append((install_dir, src_files))
    return data_entries


static_dirs = ['launch', 'worlds', 'models']


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

for subdir in static_dirs:
    if os.path.isdir(subdir):
        data_files.extend(list_data_files(subdir))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tc2025 team',
    maintainer_email='example@example.com',
    description='Gazebo 上に LiDAR 障害物シミュレータ環境を提供するパッケージ',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
