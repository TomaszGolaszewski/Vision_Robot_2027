from setuptools import setup

package_name = 'robot_machining_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tomasz Golaszewski',
    maintainer_email='tomasz.golaszewski@ezut.edu.pl',
    description='Workpiece position detection and robot position forwarding',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'workpiece_position_detector = robot_machining_controller.workpiece_position_detector:main',
            'trajectory_generator = robot_machining_controller.trajectory_generator:main',
        ],
    },
)
