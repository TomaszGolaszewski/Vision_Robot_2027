import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
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
        # true = whole chain without the robot driver: fixed fake_tcp pose instead of its TF
        DeclareLaunchArgument('fake_robot', default_value='false'),
        # only together with robot.launch.py use_mock:=true: the FANUC mock publishes no robot status
        DeclareLaunchArgument('mock_robot', default_value='false',
                              description='true = do not require the FANUC driver status (mock hardware only)'),
        # USB camera + camera mount TF + detector with Kalman (topic workpiece_position); robot TF from the driver;
        # its arguments (camera, device_id, width, height, dpmm, show_image, fake_tcp) can be passed here as well
        IncludeLaunchDescription(os.path.join(share, 'launch', 'detect_marker.launch.py'),
                                 launch_arguments={'fake_robot': LaunchConfiguration('fake_robot')}.items()),
        # filtered code pose -> desired tool (fanuc_flange) pose on the path (topic robot_position)
        Node(package=PKG, executable='trajectory_generator',
             parameters=[params, {'mode': LaunchConfiguration('mode')}]),
        # desired pose -> safety-checked, speed-limited command for MoveIt Servo
        Node(package=PKG, executable='robot_controller',
             parameters=[params, {'require_robot_status': PythonExpression(
                 ["'", LaunchConfiguration('mock_robot'), "'.lower() != 'true'"])}]),
    ])
