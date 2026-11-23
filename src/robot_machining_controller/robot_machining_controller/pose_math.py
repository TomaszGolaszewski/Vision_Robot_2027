"""Geometry and filtering shared by the nodes, kept free of node logic so it is unit tested offline.

Conventions used everywhere:
- a pose is a tuple (position as np.array [m], scipy Rotation),
- quaternions are in ROS / scipy order (x, y, z, w),
- the code frame: origin in the ArUco marker centre, x right, y up, z out of the board (towards the viewer),
- the camera frame is the ROS optical frame: x right, y down, z forward (along the viewing direction).
"""
import cv2
import numpy as np
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

# Paths the TrajectoryGenerator can produce (value of its `mode` parameter).
MODES = ('direct', 'circle')

# Camera facing the code with an upright image: optical z against the code normal (180 deg about x).
FACING = Rotation.from_euler('x', np.pi)


def camera_matrix_from_dpmm(width, height, dpmm, distance_mm):
    """Pinhole K from image scale: `dpmm` pixels per 100 mm, seen at `distance_mm`."""
    # pinhole model: size_px = f * size_mm / distance_mm  ->  f = (px per mm) * distance_mm
    f = dpmm / 100.0 * distance_mm
    # principal point assumed in the image centre (no calibration available yet)
    return np.array([[f, 0, (width - 1) / 2], [0, f, (height - 1) / 2], [0, 0, 1]], dtype=float)


def marker_object_points(side):
    """Marker corners in the code frame (x right, y up, z out of the board), in ArUco order TL, TR, BR, BL."""
    h = side / 2
    # all corners lie in the board plane z = 0; this order is also the one SOLVEPNP_IPPE_SQUARE requires
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=float)


def aruco_detector(dictionary_name):
    """(dictionary, detector parameters) for cv2.aruco.detectMarkers, with subpixel corner refinement."""
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    params = cv2.aruco.DetectorParameters_create()
    # subpixel corners: the orientation accuracy of a small marker depends on them
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return dictionary, params


def find_marker_corners(gray, detector, marker_id):
    """4 image corners (TL, TR, BR, BL) of the marker with `marker_id`, or None when it is not visible."""
    dictionary, params = detector
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None:
        return None
    for c, i in zip(corners, ids.ravel()):
        if i == marker_id:
            return c.reshape(4, 2)
    return None


def detect_marker_pose(gray, K, dist, side, detector, marker_id, max_reproj_px=2.0):
    """Marker pose in the camera frame as (position, Rotation), or None when no plausible marker is found."""
    # 1. find the 4 outer corners of the marker in the image (pixels)
    corners = find_marker_corners(gray, detector, marker_id)
    if corners is None:
        return None
    return marker_pose_from_corners(corners, K, dist, side, max_reproj_px)


def marker_pose_from_corners(corners, K, dist, side, max_reproj_px=2.0):
    """Marker pose in the camera frame from its 4 image corners (TL, TR, BR, BL), or None when implausible."""
    # 2. 3D corners (known marker size) + 2D corners + camera intrinsics -> pose of the marker relative to the camera
    obj, img = marker_object_points(side), corners.reshape(4, 2).astype(float)
    n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    # 3. IPPE returns two mirror solutions (best first); keep one that reprojects well and faces the camera
    for rvec, tvec in zip(rvecs, tvecs):
        if not np.all(np.isfinite(rvec)):
            continue  # IPPE sometimes returns NaN for the second solution
        # IPPE alone is ~9 deg off at exactly frontal views; Levenberg-Marquardt refinement removes that
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, dist, rvec.copy(), tvec.copy())
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        err = np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - img) ** 2, axis=1)))  # RMS, as solvePnPGeneric
        r = Rotation.from_rotvec(rvec.ravel())
        # r.as_matrix()[2, 2] is the z component of the marker normal in the camera frame; < 0 = towards camera
        if err < max_reproj_px and r.as_matrix()[2, 2] < 0:
            return tvec.ravel(), r
    return None


def camera_target(position, rotation, standoff, mode, t=0.0, radius=0.05, period=10.0):
    """Desired camera pose facing the code at `standoff` along its normal; `circle` moves it in the code plane."""
    axes = rotation.as_matrix()  # columns: code x, y, z in the parent frame
    # direct: camera sits on the code normal, `standoff` metres in front of the code centre
    offset = standoff * axes[:, 2]
    if mode == 'circle':
        # circle: additionally move along a circle spanned by the code x and y axes (i.e. parallel to the board),
        # one full turn every `period` seconds
        a = 2 * np.pi * t / period
        offset = offset + radius * (np.cos(a) * axes[:, 0] + np.sin(a) * axes[:, 1])
    elif mode != 'direct':
        raise ValueError(f'unknown mode {mode!r}')
    # orientation: code orientation turned 180 deg about x, so the camera looks straight at the code
    return position + offset, rotation * FACING


def limit_step(prev_p, prev_r, target_p, target_r, max_dist, max_angle):
    """Move from prev towards target by at most max_dist [m] and max_angle [rad]."""
    # translation: shorten the step vector to max_dist, keeping its direction
    d = target_p - prev_p
    n = np.linalg.norm(d)
    if n > max_dist:
        d = d * (max_dist / n)
    # rotation: relative rotation prev -> target as axis * angle, angle clamped to max_angle
    rel = (target_r * prev_r.inv()).as_rotvec()
    a = np.linalg.norm(rel)
    if a > max_angle:
        rel = rel * (max_angle / a)
    return prev_p + d, Rotation.from_rotvec(rel) * prev_r


class PoseKalman:
    """cv2.KalmanFilter on state [position, velocity, quaternion]: constant velocity, random-walk orientation.

    State (10): x, y, z, vx, vy, vz, qx, qy, qz, qw.   Measurement (7): x, y, z, qx, qy, qz, qw.
    """

    def __init__(self, accel_noise, rot_noise, pos_meas_noise, rot_meas_noise):
        self.accel_noise, self.rot_noise = accel_noise, rot_noise
        self.kf = cv2.KalmanFilter(10, 7)
        # measurement matrix H: we measure position (state 0..2) and quaternion (state 6..9), not velocity
        H = np.zeros((7, 10), np.float32)
        H[0:3, 0:3] = np.eye(3)
        H[3:7, 6:10] = np.eye(4)
        self.kf.measurementMatrix = H
        # measurement noise R: how much a single detection is trusted (variances)
        self.kf.measurementNoiseCov = np.diag([pos_meas_noise] * 3 + [rot_meas_noise] * 4).astype(np.float32)
        self.initialized = False

    def reset(self):
        # next measurement re-initializes the filter (used after the code was lost)
        self.initialized = False

    def update(self, dt, measurement=None):
        """Predict by dt and correct with (position, Rotation) if given. Returns the filtered pose or None."""
        kf = self.kf
        if measurement is not None:
            p, q = measurement[0], measurement[1].as_quat()
        # first measurement: start the state directly from it, with zero velocity and large uncertainty
        if not self.initialized:
            if measurement is None:
                return None
            kf.statePost = np.r_[p, np.zeros(3), q].astype(np.float32).reshape(10, 1)
            kf.errorCovPost = np.eye(10, dtype=np.float32)
            self.initialized = True
            return self._estimate()
        # transition matrix F for the actual time step: position += velocity * dt, rest unchanged
        F = np.eye(10, dtype=np.float32)
        F[0:3, 3:6] = np.eye(3) * dt
        kf.transitionMatrix = F
        # process noise Q: how fast the true pose may change between steps
        # discrete white-noise acceleration (variance accel_noise [m^2/s^4]), diagonal only; orientation random walk
        kf.processNoiseCov = np.diag([self.accel_noise * dt ** 4 / 4] * 3 + [self.accel_noise * dt ** 2] * 3
                                     + [self.rot_noise * dt] * 4).astype(np.float32)
        kf.predict()  # also copies the prediction into statePost, so a missing measurement is fine
        # correction step, only when a new detection arrived since the last call
        if measurement is not None:
            if np.dot(q, kf.statePre[6:10, 0]) < 0:
                q = -q  # q and -q are the same rotation; keep the measurement on the state's side
            kf.correct(np.r_[p, q].astype(np.float32).reshape(7, 1))
        return self._estimate()

    def _estimate(self):
        # the linear filter does not keep |q| = 1, so renormalize the quaternion in the state
        s = self.kf.statePost
        s[6:10] /= np.linalg.norm(s[6:10])
        self.kf.statePost = s
        return s[0:3, 0].astype(float), Rotation.from_quat(s[6:10, 0])


# --- conversions between ROS messages and (position, Rotation) poses ---

def to_pose(p, r):
    """(position, Rotation) -> geometry_msgs/Pose."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in p)
    o = pose.orientation
    o.x, o.y, o.z, o.w = (float(v) for v in r.as_quat())
    return pose


def from_pose(pose):
    """geometry_msgs/Pose -> (position, Rotation)."""
    p, o = pose.position, pose.orientation
    return np.array([p.x, p.y, p.z]), Rotation.from_quat([o.x, o.y, o.z, o.w])


def from_transform(tf):
    """geometry_msgs/Transform (from a TF lookup) -> (position, Rotation)."""
    t, o = tf.translation, tf.rotation
    return np.array([t.x, t.y, t.z]), Rotation.from_quat([o.x, o.y, o.z, o.w])


def compose(a, b):
    """Pose a * pose b, both as (position, Rotation)."""
    # e.g. compose(camera in base, code in camera) = code in base
    return a[0] + a[1].apply(b[0]), a[1] * b[1]
