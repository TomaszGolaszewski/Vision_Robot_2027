import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'robot_machining_controller'


def generate_launch_description():
    """Starts the whole marker-following chain: camera, camera mount TF, detector, trajectory generator, controller."""
    # all node parameters come from the installed config/params.yaml
    share = get_package_share_directory(PKG)
    params = os.path.join(share, 'config', 'params.yaml')
    return LaunchDescription([
        # `ros2 launch ... mode:=circle` overrides the path mode from params.yaml
        DeclareLaunchArgument('mode', default_value='direct', description='direct | circle'),
        # USB camera + camera mount TF + detector (topic workpiece_position); robot TF from the real driver;
        # its arguments (camera, device_id, width, height, dpmm, show_image) can be passed here as well
        IncludeLaunchDescription(os.path.join(share, 'launch', 'detect_marker.launch.py'),
                                 launch_arguments={'fake_robot': 'false'}.items()),
        # code pose -> filtered, desired tool0 pose on the path (topic robot_position)
        Node(package=PKG, executable='trajectory_generator',
             parameters=[params, {'mode': LaunchConfiguration('mode')}]),
        # desired pose -> safety-checked, speed-limited command for MoveIt Servo
        Node(package=PKG, executable='robot_controller', parameters=[params]),
    ])
