import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'robot_machining_controller'
# the USB camera in use; its /dev/videoN number changes on every replug, this path does not
CAMERA = '/dev/v4l/by-id/usb-FLH-201016_Integrated_Camera-video-index0'


def camera(context):
    """USB camera -> /image_raw (OpenCV VideoCapture inside image_tools)."""
    device_id = LaunchConfiguration('device_id').perform(context)
    if device_id == 'auto':
        # no silent fallback to another camera: with the robot its mount TF would be wrong
        if not os.path.exists(CAMERA):
            raise RuntimeError(f'USB camera not connected ({CAMERA}); pass device_id:=N to use /dev/videoN')
        device_id = os.path.realpath(CAMERA).removeprefix('/dev/video')
    arg = LaunchConfiguration
    return [Node(package='image_tools', executable='cam2image', name='camera',
                 remappings=[('image', 'image_raw')],
                 parameters=[{'device_id': int(device_id), 'width': arg('width'), 'height': arg('height'),
                              'frequency': 30.0, 'reliability': 'best_effort',
                              'frame_id': 'camera_optical_frame'}])]


def generate_launch_description():
    """USB camera + camera mount TF + WorkpiecePositionDetector; without the robot by default (fake_robot).

    Also included by follow_marker.launch.py with fake_robot:=false.
    """
    params = os.path.join(get_package_share_directory(PKG), 'config', 'params.yaml')
    # written by calibrate_camera into src/.../config, installed by colcon build
    calibration = os.path.join(get_package_share_directory(PKG), 'config', 'camera_calibration.yaml')
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('device_id', default_value='auto',
                              description='N of /dev/videoN; auto = the USB camera, found by its by-id path'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        # DPMM is valid only for the camera and resolution it was measured with
        DeclareLaunchArgument('dpmm', default_value='210.0', description='px per 100 mm at nominal distance'),
        DeclareLaunchArgument('camera_calibration_file', default_value=calibration if os.path.exists(calibration) else '',
                              description="camera intrinsics YAML; '' = DPMM fallback"),
        DeclareLaunchArgument('show_image', default_value='false',
                              description='cv2.imshow window with the detection (also on topic image_debug)'),
        DeclareLaunchArgument('fake_robot', default_value='true',
                              description='publish a fixed robot pose instead of the robot driver TF'),

        OpaqueFunction(function=camera),

        # camera position on the tool: static transform tool0 -> camera_optical_frame
        # shortcut: placeholder camera mount, replace with the hand-eye calibration result
        Node(package='tf2_ros', executable='static_transform_publisher', name='camera_mount_tf',
             arguments=['--z', '0.1', '--frame-id', 'tool0', '--child-frame-id', 'camera_optical_frame']),

        # no robot: fixed tool0 pose chosen so that camera_optical_frame == base_link,
        # i.e. workpiece_position reads directly as the code pose relative to the camera
        Node(package='tf2_ros', executable='static_transform_publisher', name='fake_robot_tf',
             condition=IfCondition(arg('fake_robot')),
             arguments=['--z', '-0.1', '--frame-id', 'base_link', '--child-frame-id', 'tool0']),

        # image -> code pose in base_link (topic workpiece_position)
        Node(package=PKG, executable='workpiece_position_detector',
             parameters=[params, {'dpmm': arg('dpmm'), 'show_image': arg('show_image'),
                                  'camera_calibration_file': arg('camera_calibration_file')}]),
    ])
