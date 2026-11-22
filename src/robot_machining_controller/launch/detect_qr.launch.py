import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'robot_machining_controller'


def generate_launch_description():
    """QR detection only: USB camera + WorkpiecePositionDetector, optionally without the robot."""
    params = os.path.join(get_package_share_directory(PKG), 'config', 'params.yaml')
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('device_id', default_value='0', description='camera index, /dev/videoN'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        # DPMM is valid only for the camera and resolution it was measured with
        DeclareLaunchArgument('dpmm', default_value='210.0', description='px per 100 mm at nominal distance'),
        DeclareLaunchArgument('fake_robot', default_value='true',
                              description='publish a fixed robot pose instead of the robot driver TF'),

        # USB camera -> /image_raw (OpenCV VideoCapture inside image_tools)
        Node(package='image_tools', executable='cam2image', name='camera',
             remappings=[('image', 'image_raw')],
             parameters=[{'device_id': arg('device_id'), 'width': arg('width'), 'height': arg('height'),
                          'frequency': 30.0, 'reliability': 'best_effort',
                          'frame_id': 'camera_optical_frame'}]),

        # camera position on the tool: static transform tool0 -> camera_optical_frame
        # shortcut: placeholder camera mount (same as follow_qr.launch.py), replace with hand-eye calibration
        Node(package='tf2_ros', executable='static_transform_publisher', name='camera_mount_tf',
             arguments=['--z', '0.1', '--frame-id', 'tool0', '--child-frame-id', 'camera_optical_frame']),

        # no robot: fixed tool0 pose chosen so that camera_optical_frame == base_link,
        # i.e. workpiece_position reads directly as the code pose relative to the camera
        Node(package='tf2_ros', executable='static_transform_publisher', name='fake_robot_tf',
             condition=IfCondition(arg('fake_robot')),
             arguments=['--z', '-0.1', '--frame-id', 'base_link', '--child-frame-id', 'tool0']),

        # image -> code pose in base_link (topic workpiece_position)
        Node(package=PKG, executable='workpiece_position_detector', parameters=[params, {'dpmm': arg('dpmm')}]),
    ])
