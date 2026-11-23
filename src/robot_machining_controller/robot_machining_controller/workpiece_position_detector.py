import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from robot_machining_controller.pose_math import (aruco_detector, camera_matrix_from_dpmm, compose,
                                                  find_marker_corners, from_transform, marker_pose_from_corners,
                                                  to_pose)


class WorkpiecePositionDetector(Node):
    """Detects the ArUco marker in camera images and publishes its pose in the robot base frame.

    in:  image_raw (Image), camera_info (CameraInfo, optional), TF base_link -> camera_optical_frame
    out: workpiece_position (PoseStamped, code pose in base_link, stamped with the image time)
         image_debug (Image, camera image with the code outline and axes; drawn only when subscribed)
    """

    def __init__(self):
        super().__init__('workpiece_position_detector')

        # --- parameters ---
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value       # output frame
        self.side = self.declare_parameter('marker_side_m', 0.1).value                  # outer black edge [m]
        self.dictionary = self.declare_parameter('aruco_dictionary', 'DICT_4X4_50').value  # cv2.aruco name
        self.marker_id = self.declare_parameter('marker_id', 0).value                   # other ids are ignored
        self.dpmm = self.declare_parameter('dpmm', 210.0).value                         # px per 100 mm ...
        self.nominal_distance_mm = self.declare_parameter('nominal_distance_mm', 440.0).value  # ... at this distance
        self.show_image = self.declare_parameter('show_image', False).value             # cv2.imshow window

        # --- camera intrinsics: set from camera_info when available, otherwise estimated from dpmm ---
        self.K = None      # 3x3 camera matrix
        self.dist = None   # lens distortion coefficients

        # --- tools: ROS image <-> OpenCV conversion, ArUco detector, TF buffer ---
        self.bridge = CvBridge()
        self.detector = aruco_detector(self.dictionary)
        self.tf_buffer = Buffer()
        # separate internal node + thread, so the blocking lookup below can wait for TF at the image stamp
        self.tf_listener = TransformListener(self.tf_buffer, None, spin_thread=True)

        # --- rates for the debug image: wall times of the last second of processed frames / accepted detections ---
        self.frame_times = deque()
        self.detection_times = deque()

        # --- topics ---
        self.create_subscription(CameraInfo, 'camera_info', self.camera_info_callback, 10)
        # depth 1: if detection is slower than the camera, always take the newest frame
        self.create_subscription(Image, 'image_raw', self.image_callback,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.workpiece_position_publisher = self.create_publisher(PoseStamped, 'workpiece_position', 10)
        self.image_debug_publisher = self.create_publisher(Image, 'image_debug', 1)

    def camera_info_callback(self, msg):
        # take the calibrated intrinsics from the camera driver; k[0] == 0 means the camera is not calibrated
        if msg.k[0] > 0:
            self.K = np.array(msg.k).reshape(3, 3)
            self.dist = np.array(msg.d) if len(msg.d) else None

    def image_callback(self, msg):
        # ROS image -> grayscale OpenCV image (marker detection does not need colour)
        gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        K, dist = self.K, self.dist
        if K is None:
            # shortcut: nominal pinhole from DPMM, no distortion; publish a calibrated camera_info for real accuracy
            K = camera_matrix_from_dpmm(gray.shape[1], gray.shape[0], self.dpmm, self.nominal_distance_mm)

        # marker corners in the image, then marker pose relative to the camera; None = no (plausible) marker
        corners = find_marker_corners(gray, self.detector, self.marker_id)
        found = corners is not None
        code_in_camera = marker_pose_from_corners(corners, K, dist, self.side) if found else None
        now = time.monotonic()
        for times, hit in ((self.frame_times, True), (self.detection_times, code_in_camera is not None)):
            if hit:
                times.append(now)
            while times and times[0] < now - 1.0:
                times.popleft()
        if self.show_image or self.image_debug_publisher.get_subscription_count() > 0:
            self.show_debug(msg, gray, corners if found else None, code_in_camera, K, dist)
        if code_in_camera is None:
            return

        # camera pose in the base frame (robot pose + camera mount from TF)
        try:
            # robot pose at the moment the image was taken, so robot motion does not shift the result
            t = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id,
                                                Time.from_msg(msg.header.stamp), Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().warn(f'no TF {self.base_frame}<-{msg.header.frame_id}: {e}',
                                   throttle_duration_sec=1.0)
            return

        # code in base = (camera in base) * (code in camera); keep the image stamp for later latency handling
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.base_frame
        out.pose = to_pose(*compose(from_transform(t.transform), code_in_camera))
        self.workpiece_position_publisher.publish(out)

    def show_debug(self, msg, gray, corners, code_in_camera, K, dist):
        """Draw the detection: green outline + marker axes (x red, y green, z blue), red outline = pose rejected."""
        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        # processed frames can be fewer than camera frames: the subscription keeps only the newest one
        text = f'{len(self.frame_times)} fps, {len(self.detection_times)} markers/s'
        cv2.putText(img, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(img, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        if corners is not None:
            color = (0, 255, 0) if code_in_camera is not None else (0, 0, 255)
            cv2.polylines(img, [corners.reshape(-1, 1, 2).astype(np.int32)], True, color, 2)
        if code_in_camera is not None:
            position, rotation = code_in_camera
            cv2.drawFrameAxes(img, K, dist, rotation.as_rotvec(), position, self.side / 2)
        if self.image_debug_publisher.get_subscription_count() > 0:
            self.image_debug_publisher.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8', msg.header))
        if self.show_image:
            # shortcut: GUI in the executor thread, needs a display; use image_debug on a headless machine
            cv2.imshow('workpiece_position_detector', img)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = WorkpiecePositionDetector()
    try:
        rclpy.spin(node)  # process callbacks until Ctrl+C
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
