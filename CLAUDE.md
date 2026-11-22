# CLAUDE.md

PhD research project: a FANUC CRX-10iA/L with an eye-in-hand camera detects a QR code on a board held by an
operator and follows it. The user writes in Polish; answer in Polish. Code, comments, README and CHANGELOG are in English.

## Environment

- ROS 2 Jazzy, Python only (`ament_python`), OpenCV 4.6 (with `cv2.aruco`), numpy, scipy.
- Single package: `src/robot_machining_controller`. Workspace root = repo root (`build/ install/ log/` are git-ignored).
- The only camera on the dev machine is the laptop webcam (`/dev/video0`, Lenovo EasyCamera, 640x480 @ 30 fps,
  1280x720 @ 10 fps). It is NOT the target camera. No `usb_cam`/`v4l2_camera`; images come from `image_tools/cam2image`.

## Commands

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_machining_controller && source install/setup.bash
colcon test --packages-select robot_machining_controller && colcon test-result --verbose
cd src/robot_machining_controller && python3 -m pytest test -q        # fast, no build needed
ros2 launch robot_machining_controller detect_qr.launch.py             # webcam + detector, fake robot TF
ros2 launch robot_machining_controller follow_qr.launch.py mode:=circle  # full chain (needs image + robot TF)
```

Expected test result: all pass, 2 `xfail` (documented QR corner accuracy limit, `strict=True`).

## Architecture

```
image_raw, camera_info -> workpiece_position_detector -> workpiece_position -> trajectory_generator
                                                                                   | robot_position
                                    /servo_node/pose_target_cmds <- robot_controller <-+
```

- `pose_math.py`: all geometry, PnP, Kalman filter, step limiter, ROS<->pose conversions. No node logic, unit tested.
  Put new math here with a test; keep nodes thin.
- `workpiece_position_detector`: QRCodeDetector + `solvePnPGeneric(IPPE_SQUARE)`, transforms the code pose to
  `base_link` with TF **at the image stamp** (robot motion compensation). Intrinsics from `camera_info`, else from DPMM.
- `trajectory_generator`: OpenCV Kalman (state: position, velocity, quaternion), `mode` = `direct` | `circle`
  (switchable live via `ros2 param set`), stops publishing after `lost_timeout_s`.
  Publishes the desired **tool0** pose (camera target composed with the static mount TF).
- `robot_controller`: safety gate, workspace box, linear/angular speed limit, command timeout, ramps from the
  actual tool0 pose (TF). Output to MoveIt Servo pose topic.

Conventions: `geometry_msgs/PoseStamped` in `base_link`, SI units (m, rad), quaternion (x, y, z, w).
Code frame: x right, y up, z out of the board. Camera: ROS optical frame (z forward, y down).
Frames: `base_link -> tool0 -> camera_optical_frame`.

Parameter defaults live in two places: `declare_parameter(...)` in the node and `config/params.yaml`. Keep them in sync.

## Gotchas (learned the hard way)

- Jazzy `TransformListener(buffer, node, spin_thread=True)` spins the WHOLE node in a second executor. Use
  `TransformListener(buffer, None, spin_thread=True)` when a callback needs a blocking TF lookup.
- `QRCodeDetector` corners are ~1 px off (not subpixel). For a 50 mm code at 440 mm (~105 px) this gives 7-14 deg
  orientation error near frontal views, about 100 mm lateral target error at 440 mm standoff. Position error is a few mm.
  `cornerSubPix` makes it worse. **Open decision for the user**: ArUco/AprilTag next to the QR (recommended),
  a bigger code, or a position-only `direct` mode.
- IPPE returns two solutions and sometimes garbage (74 px reprojection, flipped normal). `detect_qr_pose` rejects
  reprojection > 2 px and normals facing away. Do not remove that check.
- Kalman on quaternions: flip the measurement sign to the state's hemisphere and renormalize after each step.
- Synthetic test images: pixel centres are at integer coordinates (`src - 0.5` in `render_qr`). Without that,
  pose tests are off by a half-pixel bias.
- Detection takes ~65 ms at 1280x960, so the image subscription has depth 1 (newest frame only).
- DPMM (210 px / 100 mm at 440 mm) belongs to the target camera at its resolution. It is wrong for the webcam or at
  another resolution. Correction: `dpmm_new = dpmm * true_distance / measured_z`.

## Placeholders (must be replaced before the real robot)

- Camera mount TF `tool0 -> camera_optical_frame` (`--z 0.1`) is duplicated in `launch/follow_qr.launch.py` and
  `launch/detect_qr.launch.py`. Replace both with the hand-eye calibration result.
- Workspace box `workspace_min/max` in `config/params.yaml`.
- Kalman noise values were tuned on synthetic data only.
- Robot driver not chosen yet (FANUC ROS 2 driver / RMI); `robot_controller` targets MoveIt Servo.

## Plan and rules

Stages: (1) code + offline tests (done), (2) rosbag + RViz, Kalman tuning, hand-eye calibration script,
(3) simulation with mock hardware, (4) real robot in T1 at low speed.

- Never run anything that moves the real robot without the user's explicit approval for that step.
- Safety checks (workspace box, speed limits, timeouts, PnP plausibility) are not optional. Do not weaken them to
  make something work.
- Add an entry to `CHANGELOG.md` under `[Unreleased]` for user-visible changes.
- Known limits get a `# shortcut: <limit>, <when to upgrade>` comment.
- Throwaway scripts (smoke tests, snapshots) go outside the repo.
