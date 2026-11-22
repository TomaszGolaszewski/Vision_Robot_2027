import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'robot_machining_controller'


def generate_launch_description():
    """Starts the whole QR-following chain: camera mount TF, detector, trajectory generator, robot controller."""
    # all node parameters come from the installed config/params.yaml
    params = os.path.join(get_package_share_directory(PKG), 'config', 'params.yaml')
    return LaunchDescription([
        # `ros2 launch ... mode:=circle` overrides the path mode from params.yaml
        DeclareLaunchArgument('mode', default_value='direct', description='direct | circle'),
        # camera position on the tool: static transform tool0 -> camera_optical_frame
        # shortcut: placeholder camera mount, replace with the hand-eye calibration result
        Node(package='tf2_ros', executable='static_transform_publisher', name='camera_mount_tf',
             arguments=['--z', '0.1', '--frame-id', 'tool0', '--child-frame-id', 'camera_optical_frame']),
        # image -> code pose in base_link (topic workpiece_position)
        Node(package=PKG, executable='workpiece_position_detector', parameters=[params]),
        # code pose -> filtered, desired tool0 pose on the path (topic robot_position)
        Node(package=PKG, executable='trajectory_generator',
             parameters=[params, {'mode': LaunchConfiguration('mode')}]),
        # desired pose -> safety-checked, speed-limited command for MoveIt Servo
        Node(package=PKG, executable='robot_controller', parameters=[params]),
    ])
