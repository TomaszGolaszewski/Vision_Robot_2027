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

from robot_machining_controller.pose_math import (camera_matrix_from_dpmm, compose, detect_qr_pose,
                                                  from_transform, to_pose)


class WorkpiecePositionDetector(Node):
    """Detects the QR code in camera images and publishes its pose in the robot base frame.

    in:  image_raw (Image), camera_info (CameraInfo, optional), TF base_link -> camera_optical_frame
    out: workpiece_position (PoseStamped, code pose in base_link, stamped with the image time)
    """

    def __init__(self):
        super().__init__('workpiece_position_detector')

        # --- parameters ---
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value       # output frame
        self.side = self.declare_parameter('qr_side_m', 0.05).value                     # code side length [m]
        self.dpmm = self.declare_parameter('dpmm', 210.0).value                         # px per 100 mm ...
        self.nominal_distance_mm = self.declare_parameter('nominal_distance_mm', 440.0).value  # ... at this distance

        # --- camera intrinsics: set from camera_info when available, otherwise estimated from dpmm ---
        self.K = None      # 3x3 camera matrix
        self.dist = None   # lens distortion coefficients

        # --- tools: ROS image <-> OpenCV conversion, QR detector, TF buffer ---
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.tf_buffer = Buffer()
        # separate internal node + thread, so the blocking lookup below can wait for TF at the image stamp
        self.tf_listener = TransformListener(self.tf_buffer, None, spin_thread=True)

        # --- topics ---
        self.create_subscription(CameraInfo, 'camera_info', self.camera_info_callback, 10)
        # depth 1: detection (~65 ms at 1280x960) is slower than the camera, always take the newest frame
        self.create_subscription(Image, 'image_raw', self.image_callback,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.workpiece_position_publisher = self.create_publisher(PoseStamped, 'workpiece_position', 10)

    def camera_info_callback(self, msg):
        # take the calibrated intrinsics from the camera driver; k[0] == 0 means the camera is not calibrated
        if msg.k[0] > 0:
            self.K = np.array(msg.k).reshape(3, 3)
            self.dist = np.array(msg.d) if len(msg.d) else None

    def image_callback(self, msg):
        # ROS image -> grayscale OpenCV image (QR detection does not need colour)
        gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        K, dist = self.K, self.dist
        if K is None:
            # shortcut: nominal pinhole from DPMM, no distortion; publish a calibrated camera_info for real accuracy
            K = camera_matrix_from_dpmm(gray.shape[1], gray.shape[0], self.dpmm, self.nominal_distance_mm)

        # code pose relative to the camera; None when there is no (plausible) code in the image
        code_in_camera = detect_qr_pose(gray, K, dist, self.side, self.detector)
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


def main(args=None):
    rclpy.init(args=args)
    node = WorkpiecePositionDetector()
    try:
        rclpy.spin(node)  # process callbacks until Ctrl+C
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
