from setuptools import setup
import os
from glob import glob

package_name = 'robot_navigator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kazuki Ogata',
    maintainer_email='kaz.ogata1988@gmail.com',
    description='robot_navigator: ROS2 node for navigation control and robot simulation.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_navigator = robot_navigator.robot_navigator_node:main',
            'robot_simulator = robot_navigator.robot_simulator_node:main',
        ],
    },
)

