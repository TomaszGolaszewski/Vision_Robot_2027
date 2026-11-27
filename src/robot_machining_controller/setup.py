from glob import glob

from setuptools import setup

package_name = 'robot_machining_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    maintainer='Tomasz Golaszewski',
    maintainer_email='tomasz.golaszewski@ezut.edu.pl',
    description='Workpiece position detection and robot position forwarding',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'workpiece_position_detector = robot_machining_controller.workpiece_position_detector:main',
            'trajectory_generator = robot_machining_controller.trajectory_generator:main',
            'robot_controller = robot_machining_controller.robot_controller:main',
            'calibrate_camera = robot_machining_controller.calibrate_camera:main',
            'mock_home = robot_machining_controller.mock_home:main',
        ],
    },
)
