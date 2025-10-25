from glob import glob
from setuptools import setup

package_name = 'robot_console'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.ui_components', f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config/node_params', glob('config/node_params/**/*.yaml', recursive=True)),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot_console maintainers',
    maintainer_email='maintainer@example.com',
    description='Route monitoring console with tkinter GUI and ROS2 integrations.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_console = robot_console.robot_console_node:main',
        ],
    },
)
