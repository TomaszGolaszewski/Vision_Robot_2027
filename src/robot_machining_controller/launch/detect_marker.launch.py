import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'robot_machining_controller'
# USB cameras of the same model differ only in the product string (FLH-<date>) of their by-id path;
# the /dev/videoN number changes on every replug, this path does not
FIXED_CAMERA_FRAME = 'fixed_camera_optical_frame'  # see fixed_camera()
CAMERA = '/dev/v4l/by-id/usb-FLH-{}_Integrated_Camera-video-index0'


def camera(context):
    """USB camera -> /image_raw (OpenCV VideoCapture inside image_tools) + WorkpiecePositionDetector.

    Each camera unit has its own intrinsics: config/camera_calibration_<camera>.yaml.
    """
    arg = LaunchConfiguration
    name = arg('camera').perform(context)
    device_id = arg('device_id').perform(context)
    if device_id == 'auto':
        # no silent fallback to another camera: with the robot its mount TF would be wrong
        path = CAMERA.format(name)
        if not os.path.exists(path):
            raise RuntimeError(f'USB camera not connected ({path}); pass camera:=<FLH id> or device_id:=N')
        device_id = os.path.realpath(path).removeprefix('/dev/video')
    calibration = arg('camera_calibration_file').perform(context)
    if calibration == 'auto':
        # written by calibrate_camera into src/.../config, installed by colcon build; '' = DPMM fallback
        calibration = os.path.join(get_package_share_directory(PKG), 'config', f'camera_calibration_{name}.yaml')
        calibration = calibration if os.path.exists(calibration) else ''
    params = os.path.join(get_package_share_directory(PKG), 'config', 'params.yaml')
    frame = FIXED_CAMERA_FRAME if arg('fixed_camera').perform(context) else 'camera_optical_frame'
    return [Node(package='image_tools', executable='cam2image', name='camera',
                 remappings=[('image', 'image_raw')],
                 parameters=[{'device_id': int(device_id), 'width': arg('width'), 'height': arg('height'),
                              'frequency': 30.0, 'reliability': 'best_effort',
                              'frame_id': frame}]),
            # image -> code pose in base_link (topic workpiece_position)
            Node(package=PKG, executable='workpiece_position_detector',
                 parameters=[params, {'dpmm': arg('dpmm'), 'show_image': arg('show_image'),
                                      'camera_calibration_file': calibration}])]


def static_pose_tf(name, value, parent, child):
    """static_transform_publisher from a launch argument "x y z roll pitch yaw" [m, rad]."""
    values = value.split()
    if len(values) != 6:
        raise RuntimeError(f'{name} needs 6 numbers "x y z roll pitch yaw", got {values}')
    pose = [a for key, v in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), values) for a in (f'--{key}', v)]
    return Node(package='tf2_ros', executable='static_transform_publisher', name=f'{name}_tf',
                arguments=pose + ['--frame-id', parent, '--child-frame-id', child])


def fake_robot(context):
    """No robot: fixed tool pose `fake_tcp` (x y z roll pitch yaw, in base_link, m and rad) instead of the driver TF."""
    if LaunchConfiguration('fake_robot').perform(context).lower() != 'true':
        return []
    return [static_pose_tf('fake_robot', LaunchConfiguration('fake_tcp').perform(context), 'base_link', 'fanuc_flange')]


def fixed_camera(context):
    """Simulation only: the real camera stands on the desk, not on the (mock) robot.

    Images are stamped with this fixed frame instead of camera_optical_frame, so moving the virtual robot does not
    move the marker in base_link (otherwise the robot chases its own motion).
    """
    value = LaunchConfiguration('fixed_camera').perform(context)
    return [static_pose_tf('fixed_camera', value, 'base_link', FIXED_CAMERA_FRAME)] if value else []


def generate_launch_description():
    """USB camera + camera mount TF + WorkpiecePositionDetector; without the robot by default (fake_robot).

    Also included by follow_marker.launch.py with fake_robot:=false.
    """
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='201016',
                              description='USB camera unit: FLH id from /dev/v4l/by-id (201016 | 200917)'),
        DeclareLaunchArgument('device_id', default_value='auto',
                              description='N of /dev/videoN; auto = the camera unit, found by its by-id path'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        # DPMM is valid only for the camera and resolution it was measured with
        DeclareLaunchArgument('dpmm', default_value='210.0', description='px per 100 mm at nominal distance'),
        DeclareLaunchArgument('camera_calibration_file', default_value='auto',
                              description="camera intrinsics YAML; auto = config/camera_calibration_<camera>.yaml "
                                          "if present; '' = DPMM fallback"),
        DeclareLaunchArgument('show_image', default_value='false',
                              description='cv2.imshow window with the detection (also on topic image_debug)'),
        DeclareLaunchArgument('fake_robot', default_value='true',
                              description='publish a fixed robot pose instead of the robot driver TF'),
        # default chosen so that camera_optical_frame == base_link (with the placeholder mount below),
        # i.e. workpiece_position reads directly as the code pose relative to the camera
        DeclareLaunchArgument('fake_tcp', default_value='0 0 -0.1 0 0 0',
                              description='fake_robot tool pose in base_link: "x y z roll pitch yaw" [m, rad]'),
        # simulation with the mock robot only (follow_marker.launch.py refuses it otherwise), README: Simulation in RViz
        DeclareLaunchArgument('fixed_camera', default_value='',
                              description='simulation only: camera on the desk at this pose in base_link, '
                                          '"x y z roll pitch yaw" [m, rad]; "" = camera on the robot'),

        OpaqueFunction(function=camera),

        # camera position on the tool: static transform fanuc_flange -> camera_optical_frame
        # shortcut: placeholder camera mount, replace with the hand-eye calibration result
        Node(package='tf2_ros', executable='static_transform_publisher', name='camera_mount_tf',
             arguments=['--z', '0.1', '--frame-id', 'fanuc_flange', '--child-frame-id', 'camera_optical_frame']),

        OpaqueFunction(function=fake_robot),
        OpaqueFunction(function=fixed_camera),
    ])
