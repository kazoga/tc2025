from setuptools import setup

package_name = 'waypoint_editor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/waypoint_editor.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/waypoint_editor.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='developer@example.com',
    description='Waypoint editor with CSV IO, coordinate transforms, and RViz integration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_editor_node = waypoint_editor.waypoint_editor_node:main',
            'waypoint_editor_gui = waypoint_editor.waypoint_editor_gui:main',
        ],
    },
)
