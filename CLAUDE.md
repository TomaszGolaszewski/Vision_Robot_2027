# CLAUDE.md

PhD research project: a FANUC CRX-10iA/L with an eye-in-hand camera detects an ArUco marker on a board held by an
operator and follows it. The user writes in Polish; answer in Polish. Code, comments, README and CHANGELOG are in English.

## Environment

- ROS 2 Jazzy, Python only (`ament_python`), OpenCV 4.6 (with `cv2.aruco`), numpy, scipy.
- Single package: `src/robot_machining_controller`. Workspace root = repo root (`build/ install/ log/` are git-ignored).
- Cameras on the dev machine: laptop webcam (`/dev/video0`, Lenovo EasyCamera) and the USB camera in use
  (`0c45:64ab` "FLH-201016 Integrated Camera", `/dev/v4l/by-id/usb-FLH-201016_Integrated_Camera-video-index0`).
  The USB camera is stable only at 640x480 (30 fps MJPG); at 1280x720 it drops off the bus (`error -71`, cable/power).
  This camera is also the one on the robot. Its `/dev/videoN` number changes on every replug; `detect_marker.launch.py`
  resolves it from the by-id path (`device_id:=auto`) and fails when it is unplugged (no fallback: wrong camera = wrong
  mount TF on the robot). No `usb_cam`/`v4l2_camera`; images come from `image_tools/cam2image`.

## Commands

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_machining_controller && source install/setup.bash
colcon test --packages-select robot_machining_controller && colcon test-result --verbose
cd src/robot_machining_controller && python3 -m pytest test -q        # fast, no build needed
ros2 launch robot_machining_controller detect_marker.launch.py         # camera + detector, fake robot TF
ros2 launch robot_machining_controller follow_marker.launch.py mode:=circle  # full chain: includes detect_marker (fake_robot:=false), needs robot TF
```

Expected test result: all pass, 2 `xfail` (50 mm marker at 640x480 is too small for near-frontal orientation, `strict=True`).

## Architecture

```
image_raw, camera_info -> workpiece_position_detector -> workpiece_position -> trajectory_generator
                                                                                   | robot_position
                                    /servo_node/pose_target_cmds <- robot_controller <-+
```

- `pose_math.py`: all geometry, PnP, Kalman filter, step limiter, ROS<->pose conversions. No node logic, unit tested.
  Put new math here with a test; keep nodes thin.
- `workpiece_position_detector`: `cv2.aruco.detectMarkers` (subpixel corners, one `marker_id`) +
  `solvePnPGeneric(IPPE_SQUARE)` + `solvePnPRefineLM`, transforms the code pose to
  `base_link` with TF **at the image stamp** (robot motion compensation). Intrinsics from `camera_calibration_file`
  (`config/camera_calibration.yaml`, written by `calibrate_camera`) or `camera_info`, else from DPMM. Images of
  another resolution than the calibrated one are refused.
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
- QR was replaced by ArUco (OpenCV 4.6 API: `detectMarkers(gray, dict, parameters=...)`, keyword is required).
  `QRCodeDetector` corners were ~1 px off and gave 7-14 deg orientation error; ArUco subpixel corners are ~0.1 px.
- IPPE alone is ~9 deg off at exactly frontal views even with perfect corners; `solvePnPRefineLM` fixes it.
- Orientation accuracy is set by the marker size in pixels: 50 mm at 440 mm is 105 px at 1280x960 (< 1 deg), but only
  52 px at 640x480 (5-7 deg near frontal). The marker in use is 100 mm: ~105 px at 640x480, < 0.5 deg.
- IPPE returns two solutions and sometimes garbage (NaN, flipped normal). `marker_pose_from_corners` rejects
  reprojection > 2 px and normals facing away. Do not remove that check.
- Kalman on quaternions: flip the measurement sign to the state's hemisphere and renormalize after each step.
- Synthetic test images: pixel centres are at integer coordinates (`src - 0.5` in `render_marker`). Without that,
  pose tests are off by a half-pixel bias.
- Detection takes ~2.5 ms at 640x480, ~11 ms at 1280x960; the image subscription has depth 1 (newest frame only).
- Calibration board: ChArUco 7x5 squares, 25 mm square, 18 mm marker, `DICT_6X6_250` (ids 0-16, top-left square black).
  OpenCV 4.6 board frame: origin top-left, x right, y down (same as `board.draw`).
- USB camera intrinsics (`config/camera_calibration.yaml`, 2026-11-23, 25 views, RMS 0.44 px): fx ~597 px, strong barrel
  distortion (k1 ~-0.36, ~60 px in the image corners; consistent across calibrations, so real). Split-half check:
  fx +-3 %, cx +-30 px; more strongly tilted views (> 30 deg) would tighten it. The DPMM fallback (f = 924) is ~55 % off.
- Camera calibration uses `CALIB_FIX_K3`; with a small board k3 overfits. Views in the image corners are essential:
  without them the synthetic test had 18 px distortion error in the corners (0.4 px with them).
- DPMM (210 px / 100 mm at 440 mm) belongs to the target camera at its resolution. It is wrong for the webcam or at
  another resolution. Correction: `dpmm_new = dpmm * true_distance / measured_z`.

## Placeholders (must be replaced before the real robot)

- Camera mount TF `tool0 -> camera_optical_frame` (`--z 0.1`) in `launch/detect_marker.launch.py` (included by
  `follow_marker.launch.py`). Replace it with the hand-eye calibration result.
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
