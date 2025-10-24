import os
from glob import glob

from setuptools import setup

package_name = 'obstacle_monitor'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
    ],
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
