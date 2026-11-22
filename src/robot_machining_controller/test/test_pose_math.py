import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from robot_machining_controller.pose_math import (FACING, PoseKalman, camera_matrix_from_dpmm, camera_target,
                                                  detect_qr_pose, limit_step, qr_object_points)

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


def render_qr(K, p, r, size=(1280, 960)):
    code = cv2.QRCodeEncoder.create().encode('VR2027')
    ys, xs = np.where(code == 0)
    code = code[ys.min():ys.max() + 1, xs.min():xs.max() + 1]  # crop quiet zone: side = finder edges
    code = cv2.resize(code, None, fx=20, fy=20, interpolation=cv2.INTER_NEAREST)
    n = code.shape[0]
    src = np.float32([[0, 0], [n, 0], [n, n], [0, n]]) - 0.5  # pixel centres sit at integers
    dst, _ = cv2.projectPoints(qr_object_points(SIDE), r.as_rotvec(), p, K, None)
    H = cv2.getPerspectiveTransform(src, dst.reshape(4, 2).astype(np.float32))
    return cv2.warpPerspective(code, H, size, flags=cv2.INTER_AREA, borderValue=255)


@pytest.mark.parametrize('tilt_deg, yaw_deg', [
    (0, 10), (25, 10),
    # QRCodeDetector corners are ~1 px off (not subpixel): at 105 px code size that is 7-14 deg of orientation
    pytest.param(0, 45, marks=pytest.mark.xfail(strict=True, reason='QR corner accuracy, see README')),
    pytest.param(0, 135, marks=pytest.mark.xfail(strict=True, reason='QR corner accuracy, see README')),
])
def test_detect_qr_pose_synthetic(tilt_deg, yaw_deg):
    K = camera_matrix_from_dpmm(1280, 960, 210, 440)
    p = np.array([0.03, -0.02, 0.44])
    r = FACING * Rotation.from_euler('yz', [tilt_deg, yaw_deg], degrees=True)
    result = detect_qr_pose(render_qr(K, p, r), K, None, SIDE, cv2.QRCodeDetector())
    assert result is not None
    assert np.linalg.norm(result[0] - p) < 0.003
    assert angle_deg(result[1], r) < 3.0


def test_detect_rejects_implausible_solution():
    # axis-aligned frontal view: IPPE fails here (74 px reprojection, flipped normal); must not be published
    K = camera_matrix_from_dpmm(1280, 960, 210, 440)
    img = render_qr(K, np.array([0.03, 0, 0.44]), FACING)
    assert detect_qr_pose(img, K, None, SIDE, cv2.QRCodeDetector()) is None


def test_no_code_returns_none():
    K = camera_matrix_from_dpmm(640, 480, 210, 440)
    assert detect_qr_pose(np.full((480, 640), 255, np.uint8), K, None, SIDE, cv2.QRCodeDetector()) is None


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
