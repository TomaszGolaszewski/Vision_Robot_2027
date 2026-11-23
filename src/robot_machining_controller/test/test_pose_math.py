import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from robot_machining_controller.pose_math import (FACING, PoseKalman, aruco_detector, calibrate_charuco_robust,
                                                  camera_matrix_from_dpmm, camera_target, charuco_board,
                                                  detect_marker_pose, find_charuco_corners, limit_step,
                                                  load_camera_info_yaml, marker_object_points,
                                                  marker_pose_from_corners, save_camera_info_yaml)

SIDE = 0.05
# arbitrary tilted code pose in the base frame
CODE_P = np.array([0.8, 0.1, 0.5])
CODE_R = Rotation.from_euler('xyz', [20, -30, 45], degrees=True)


def angle_deg(a, b):
    return np.degrees((a * b.inv()).magnitude())


def test_dpmm_focal_length():
    K = camera_matrix_from_dpmm(1280, 960, 210, 440)
    assert K[0, 0] == pytest.approx(924.0)
    # a 50 mm code at 440 mm spans 105 px, i.e. 210 px per 100 mm
    assert K[0, 0] * SIDE / 0.44 == pytest.approx(105.0)


def test_circle_radius_and_plane():
    axes = CODE_R.as_matrix()
    for t in np.linspace(0, 10, 101):
        p, r = camera_target(CODE_P, CODE_R, 0.44, 'circle', t, 0.05, 10.0)
        in_plane = p - 0.44 * axes[:, 2] - CODE_P  # project back onto the code plane
        assert abs(np.linalg.norm(in_plane) - 0.05) < 1e-4
        assert abs(in_plane @ axes[:, 2]) < 1e-4
        assert angle_deg(r, CODE_R * FACING) < 1e-6


def test_direct_faces_code_at_standoff():
    p, r = camera_target(CODE_P, CODE_R, 0.44, 'direct')
    z_code, z_cam = CODE_R.as_matrix()[:, 2], r.as_matrix()[:, 2]
    assert np.allclose(p, CODE_P + 0.44 * z_code)
    assert np.allclose(z_cam, -z_code)  # camera looks straight at the code


def test_unknown_mode():
    with pytest.raises(ValueError):
        camera_target(CODE_P, CODE_R, 0.44, 'zigzag')


DETECTOR = aruco_detector('DICT_4X4_50')
MARKER_ID = 7


def render_marker(K, p, r, size=(1280, 960), marker_id=MARKER_ID, side=SIDE):
    marker = cv2.aruco.drawMarker(DETECTOR[0], marker_id, 400)  # side = outer edge of the black border
    n = marker.shape[0]
    src = np.float32([[0, 0], [n, 0], [n, n], [0, n]]) - 0.5  # pixel centres sit at integers
    dst, _ = cv2.projectPoints(marker_object_points(side), r.as_rotvec(), p, K, None)
    H = cv2.getPerspectiveTransform(src, dst.reshape(4, 2).astype(np.float32))
    return cv2.warpPerspective(marker, H, size, flags=cv2.INTER_AREA, borderValue=255)


FRONTAL_640 = pytest.mark.xfail(strict=True, reason='50 mm marker is only 52 px at 640x480, see README')


# frontal views (yaw 0/45/135) were the failing cases with QRCodeDetector corners
@pytest.mark.parametrize('size, dpmm, tilt_deg, yaw_deg', [
    *[((1280, 960), 210, t, y) for t, y in [(0, 0), (0, 10), (25, 10), (0, 45), (0, 135), (40, -60)]],
    # 0.1 px corner noise on a 52 px marker: near-frontal orientation is ambiguous by 5-7 deg
    pytest.param((640, 480), 105, 0, 0, marks=FRONTAL_640),
    pytest.param((640, 480), 105, 0, 10, marks=FRONTAL_640),
    *[((640, 480), 105, t, y) for t, y in [(25, 10), (0, 45), (0, 135), (40, -60)]],
])
def test_detect_marker_pose_synthetic(size, dpmm, tilt_deg, yaw_deg):
    K = camera_matrix_from_dpmm(*size, dpmm, 440)
    p = np.array([0.03, -0.02, 0.44])
    r = FACING * Rotation.from_euler('yz', [tilt_deg, yaw_deg], degrees=True)
    result = detect_marker_pose(render_marker(K, p, r, size), K, None, SIDE, DETECTOR, MARKER_ID)
    assert result is not None
    assert np.linalg.norm(result[0] - p) < 0.003 * 210 / dpmm  # half the px per mm -> twice the depth error
    assert angle_deg(result[1], r) < 3.0


@pytest.mark.parametrize('tilt_deg, yaw_deg', [(0, 0), (0, 10), (0, 45)])
def test_100mm_marker_at_640x480_frontal(tilt_deg, yaw_deg):
    # the marker actually used with the 640x480 camera: 105 px, so near-frontal views are accurate again
    K = camera_matrix_from_dpmm(640, 480, 105, 440)
    p = np.array([0.03, -0.02, 0.44])
    r = FACING * Rotation.from_euler('yz', [tilt_deg, yaw_deg], degrees=True)
    result = detect_marker_pose(render_marker(K, p, r, (640, 480), side=0.1), K, None, 0.1, DETECTOR, MARKER_ID)
    assert result is not None
    assert np.linalg.norm(result[0] - p) < 0.003
    assert angle_deg(result[1], r) < 1.0


def test_detect_ignores_other_marker_id():
    K = camera_matrix_from_dpmm(1280, 960, 210, 440)
    img = render_marker(K, np.array([0.03, 0, 0.44]), FACING, marker_id=MARKER_ID + 1)
    assert detect_marker_pose(img, K, None, SIDE, DETECTOR, MARKER_ID) is None


def test_rejects_implausible_solution():
    # corners in mirrored order (TL, BL, BR, TR): the only fitting pose has its normal facing away from the camera
    K = camera_matrix_from_dpmm(1280, 960, 210, 440)
    dst, _ = cv2.projectPoints(marker_object_points(SIDE), FACING.as_rotvec(), np.array([0.03, 0, 0.44]), K, None)
    assert marker_pose_from_corners(dst.reshape(4, 2)[[0, 3, 2, 1]], K, None, SIDE) is None


def test_no_code_returns_none():
    K = camera_matrix_from_dpmm(640, 480, 210, 440)
    assert detect_marker_pose(np.full((480, 640), 255, np.uint8), K, None, SIDE, DETECTOR, MARKER_ID) is None


BOARD = charuco_board(7, 5, 0.025, 0.018, 'DICT_6X6_250')  # the printed board


def render_board(K, rotvec, tvec, size=(640, 480), px_per_m=10000):
    """Board seen by camera K; board frame (OpenCV 4.6): origin top-left, x right, y down, z away from the camera."""
    w, h = 7 * 0.025, 5 * 0.025
    flat = BOARD.draw((round(w * px_per_m), round(h * px_per_m)))  # drawn image covers exactly the board
    src = np.float32([[0, 0], [w, 0], [w, h], [0, h]]) * px_per_m - 0.5  # pixel centres sit at integers
    dst, _ = cv2.projectPoints(np.float32([[0, 0, 0], [w, 0, 0], [w, h, 0], [0, h, 0]]), rotvec, tvec, K, None)
    H = cv2.getPerspectiveTransform(src, dst.reshape(4, 2).astype(np.float32))
    return cv2.warpPerspective(flat, H, size, flags=cv2.INTER_AREA, borderValue=255)


def test_charuco_calibration_recovers_intrinsics():
    K = np.array([[600.0, 0, 330], [0, 600, 235], [0, 0, 1]])  # a different camera than the DPMM default
    views = []
    # (tilt about x, tilt about y [deg], board centre direction x/z, y/z): tilted views in the centre and the image
    # corners; without the corner views the distortion is unconstrained there (18 px error in the synthetic test)
    for i, (ax, ay, cx, cy) in enumerate([(0, 0, 0, 0), (30, 0, 0, 0), (-30, 0, 0, 0), (0, 30, 0, 0), (0, -30, 0, 0),
                                          (25, 25, 0, 0), (20, 10, 0.3, 0.2), (-20, 10, -0.3, 0.2),
                                          (10, -20, 0.3, -0.2), (-10, -20, -0.3, -0.2), (15, 15, 0.3, 0),
                                          (-15, 15, -0.3, 0)]):
        r = Rotation.from_euler('xyz', [ax, ay, 5 * i], degrees=True)
        t = np.array([cx, cy, 1]) * 0.3 - r.apply([0.0875, 0.0625, 0])  # board centre 30 cm away
        view = find_charuco_corners(render_board(K, r.as_rotvec(), t), BOARD)
        assert view is not None, i
        views.append(view)
    # one blurred-like view: corners shifted by noise must be dropped as an outlier
    noisy = views[0][0] + np.random.default_rng(0).normal(0, 3, views[0][0].shape).astype(np.float32)
    K_est, dist, rms, dropped = calibrate_charuco_robust(views + [(noisy, views[0][1])], BOARD, (640, 480))
    assert dropped == [len(views)]
    assert rms < 0.5
    assert abs(K_est[0, 0] - 600) < 6 and abs(K_est[1, 1] - 600) < 6  # focal length within 1 %
    assert abs(K_est[0, 2] - 330) < 5 and abs(K_est[1, 2] - 235) < 5
    # no distortion in the synthetic images: the estimated one may move no image point by more than 1 px
    grid = np.array([[x, y] for x in np.linspace(0, 639, 9) for y in np.linspace(0, 479, 7)], np.float32)
    undistorted = cv2.undistortPoints(grid.reshape(-1, 1, 2), K_est, dist, P=K_est).reshape(-1, 2)
    assert np.abs(undistorted - grid).max() < 1.0


def test_no_board_returns_none():
    assert find_charuco_corners(np.full((480, 640), 255, np.uint8), BOARD) is None


def test_camera_info_yaml_round_trip(tmp_path):
    K = np.array([[600.0, 0, 330], [0, 601, 235], [0, 0, 1]])
    dist = np.array([0.1, -0.2, 0.001, 0.002, 0.05])
    save_camera_info_yaml(tmp_path / 'cam.yaml', K, dist, (640, 480))
    K2, dist2, size = load_camera_info_yaml(tmp_path / 'cam.yaml')
    assert np.allclose(K2, K) and np.allclose(dist2, dist) and size == (640, 480)


def test_kalman_smooths_noise_and_handles_quaternion_sign():
    rng = np.random.default_rng(0)
    kf = PoseKalman(0.25, 0.001, 1e-6, 1e-4)
    assert kf.update(0.033) is None  # nothing before the first measurement
    raw_err, est_err, raw_ang, est_ang = [], [], [], []
    for i in range(300):
        noisy_p = CODE_P + rng.normal(0, 0.001, 3)
        noisy_r = Rotation.from_rotvec(rng.normal(0, 0.01, 3)) * CODE_R
        q = noisy_r.as_quat() * (-1 if i % 2 else 1)  # alternate sign, same rotation
        p, r = kf.update(0.033, (noisy_p, Rotation.from_quat(q)))
        if i > 100:
            raw_err.append(np.linalg.norm(noisy_p - CODE_P))
            est_err.append(np.linalg.norm(p - CODE_P))
            raw_ang.append(angle_deg(noisy_r, CODE_R))
            est_ang.append(angle_deg(r, CODE_R))
    assert np.mean(est_err) < 0.8 * np.mean(raw_err)
    assert np.mean(est_ang) < 0.8 * np.mean(raw_ang)


def test_kalman_follows_motion():
    kf = PoseKalman(0.25, 0.001, 1e-6, 1e-4)
    v = np.array([0.1, 0, 0])  # 100 mm/s, the operator moving the board
    for i in range(150):
        p, _ = kf.update(0.033, (CODE_P + v * i * 0.033, CODE_R))
    assert np.linalg.norm(p - (CODE_P + v * 149 * 0.033)) < 0.002


def test_kalman_predicts_without_measurement():
    kf = PoseKalman(0.25, 0.001, 1e-6, 1e-4)
    kf.update(0.033, (CODE_P, CODE_R))
    p, _ = kf.update(0.033)
    assert np.allclose(p, CODE_P, atol=1e-6)


def test_limit_step():
    p0, r0 = np.zeros(3), Rotation.identity()
    p, r = limit_step(p0, r0, np.array([1.0, 0, 0]), Rotation.from_euler('z', 1.0), 0.002, 0.01)
    assert np.linalg.norm(p - p0) == pytest.approx(0.002)
    assert (r * r0.inv()).magnitude() == pytest.approx(0.01)
    # small steps pass through unchanged
    p, r = limit_step(p0, r0, np.array([0.001, 0, 0]), Rotation.from_euler('z', 0.005), 0.002, 0.01)
    assert np.allclose(p, [0.001, 0, 0]) and angle_deg(r, Rotation.from_euler('z', 0.005)) < 1e-6
