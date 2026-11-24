# Calibration procedure (stage 2)

Repeat this procedure when the robot gets a new camera, a new tool, or a new camera mount.
The table below says which steps you must redo. Each step lists what to prepare, what to measure, and where the result goes.

| Change | Steps to redo |
|---|---|
| New camera (model, lens or resolution) | 1, 2, 3, 4, 5, 6 |
| Same camera, new mount or moved on the wrist | 5, 6 |
| New tool on the flange (camera mount unchanged) | 6 (check clearance), 5 only if the camera was removed |
| New cell or new obstacles | 6 |
| New marker or board | 3, 4 |

Safety: steps 5 and 6 move the real robot. Use T1 at low speed. Each motion step needs the explicit approval of the person responsible for the robot.

---

## 1. Camera and driver

- Write down the camera model, resolution, frame rate and interface (USB / GigE).
- Current camera: USB `0c45:64ab` (FLH-201016), 640x480 @ 30 fps via `image_tools/cam2image`, selected by its
  `/dev/v4l/by-id/` path in `launch/detect_marker.launch.py`. It publishes no `camera_info`; intrinsics come from a file (step 2).
- Set the image `frame_id` to `camera_optical_frame`.
- Check that the detection time at the chosen resolution fits the frame rate (about 2.5 ms at 640x480).

## 2. Camera intrinsics

Prepare:
- The ChArUco board: 7x5 squares, square 25 mm, marker 18 mm, `DICT_6X6_250`, on a rigid, flat backing.
  Measure the square on the print, not in the PDF; pass other boards with `--squares --square --marker --dictionary`.
- Stop `detect_marker`/`follow_marker` first: the script opens the camera itself.

Procedure:
1. From the repo root run
   `ros2 run robot_machining_controller calibrate_camera --output src/robot_machining_controller/config/camera_calibration_<camera>.yaml --save-dir ~/calib_views`.
   `--from-dir ~/calib_views` recalibrates from the saved PNGs later, without the camera.
2. Save (SPACE) 20-30 views: board in the centre and in **all four image corners and edges**, tilted up to about 45 deg
   in different directions, at different distances. Keep it still when saving (motion blur). Blue dots show the
   coverage so far. Without views in the image corners the distortion there is a guess (18 px error in a synthetic test).
3. Press `c`. Views with an error above 2x the median are dropped and listed. The RMS should be below about 0.5 px;
   above 1 px the file is not written. Causes seen: a paper sheet that bends (glue it to a rigid board), motion blur
   (auto exposure is about 16 ms indoors: hold still or add light). The camera has fixed focus.
4. `colcon build` installs the file (deleting it from `src` does not remove the installed copy: delete both); the launch files pass it to the detector (`camera_calibration_file`). The detector
   refuses images of another resolution than the calibrated one.
5. Check: marker at a distance measured with a tape, `ros2 topic echo /workpiece_position` with `detect_marker`
   (`z` = distance). Expect an error below about 1 %.

Result: `config/camera_calibration_<camera>.yaml` (ROS camera_info format), committed with the code. Redo it after any change of
the camera, lens, focus or resolution.
Fallback only: `dpmm`, `nominal_distance_mm` in `config/params.yaml`. Correct them with `dpmm_new = dpmm * true_distance / measured_z`.

## 3. Marker / board

- ArUco marker, `DICT_4X4_50`, id 0 by default (print command in `README.md`). The marker in use has a 100 mm side (minimum at 640x480).
- Print it without scaling on a rigid, flat backing, with a white margin around. Measure the outer black edge.

Result: `marker_side_m` (and `marker_id`, `aruco_dictionary` if changed) in `config/params.yaml`, in both the
`declare_parameter` default and the yaml.

## 4. Kalman tuning (rosbag)

The camera can stand on a tripod for this step. The robot is not needed.

Recordings (`ros2 bag record /image_raw /camera_info /tf /tf_static`):
1. **Static:** the operator holds the board still for about 30 s at the working distance (`standoff_m`).
   Gives the measurement noise: `kalman_pos_meas_noise` and `kalman_rot_meas_noise` = variance of `workpiece_position`.
2. **Motion:** slow and fast movements typical for the task, about 60 s each.
   Gives the process noise: `kalman_accel_noise` and `kalman_rot_noise`. Tune until the filter follows without lag and without jitter.
3. Replay with `ros2 bag play` and compare `workpiece_position` with `robot_position` in `rqt_plot` / RViz.

Result: the `kalman_*` parameters in `config/params.yaml` and in the `declare_parameter` defaults.
Keep the bags. They are the test set for the next retune.

## 5. Hand-eye calibration (`tool0 -> camera_optical_frame`)

Prepare:
- The camera in its final mount. Any later move of the camera on the wrist invalidates the result.
- A calibration board (as in step 2), fixed so that it does not move during the whole session.

Procedure:
1. On the pendant set UFRAME = 0, UTOOL = 0.
2. Jog to 15-20 poses with the board fully visible. Use different orientations, with rotations of at least 30 deg about several axes. Pure translations give no information.
3. At each pose save the image and the pendant pose (X, Y, Z [mm], W, P, R [deg]). Prefer an export from the controller. Hand-typed poses are a common source of silent errors.
4. Compute with `cv2.calibrateHandEye` (eye-in-hand), using the script `TODO: not written yet`.
5. Check the result: the board pose in `base_link` should be the same for all poses. A spread of a few mm is acceptable.

Result: replace the placeholder `--z 0.1` static transform in `launch/detect_marker.launch.py` (`follow_marker.launch.py` includes it).

## 6. Workspace box (`workspace_min`, `workspace_max`)

### What the code checks

`robot_controller` checks the **target position of `tool0`** (flange centre) against an axis-aligned box in `base_link`.
It does not check orientation, and it does not check the camera, the tool or the robot arm.
If the target is outside the box, the robot does not move and a warning is logged.

### What to measure: 6 values

| Value | Limited by |
|---|---|
| `x_min` | Closeness to the robot base: the camera hitting the arm or J1, and the singularity near the J1 axis. |
| `x_max` | Reach (CRX-10iA/L: 1418 mm, singular when fully stretched) **and where the operator stands**. |
| `y_min`, `y_max` | Side obstacles: walls, fences, other stations. |
| `z_min` | Table or floor + how far the camera, mount, tool and cable stick out below the flange + margin. |
| `z_max` | Ceiling, lamps, guards, or the reach upwards. |

### How to measure

1. Mount the final camera and tool. They usually hit first.
2. On the pendant set **UFRAME = 0, UTOOL = 0**. X, Y, Z then show the flange in the World frame, which corresponds to `tool0`.
3. In T1 at low speed, jog to each of the 6 limits with the wrist orientation used during tracking (camera facing the operator). Write down the coordinate.
4. Subtract a margin of 50-100 mm towards the inside of the box.
5. Convert mm to m.

Also write down:
- **What limits each value** (e.g. "z_min: table top at 0.72 m"). After a change in the cell this shows what to recheck.
- **2-3 control points** (full pendant poses). They are used to check that the FANUC World frame matches `base_link` in the URDF. The origin may be shifted, e.g. in Z. Verify this in simulation (stage 3) before using the box on the real robot.
- **The operator zone**: the approximate X, Y, Z range where the operator holds the board.
  The robot places `tool0` about `standoff_m` + camera offset (now 0.44 + 0.1 = 0.54 m) in front of the board.
  The operator zone shifted by that distance towards the robot must fit inside the box. Otherwise the robot will not move.

### Limits

- The box protects only the flange. The elbow and the arm can leave it. If there are obstacles inside, shrink the box to exclude them.
- This is a software layer only. The primary protection is a **DCS** position check zone on the FANUC controller.

Result: `workspace_min`, `workspace_max` in `config/params.yaml` and the `declare_parameter` defaults in `robot_controller.py`.

### Measurement record (copy for each calibration)

```
Date:            Robot:            Camera:            Tool / mount:
Margin used [mm]:
x_min =      m   limited by:
x_max =      m   limited by:
y_min =      m   limited by:
y_max =      m   limited by:
z_min =      m   limited by:
z_max =      m   limited by:
Control points (X Y Z W P R):
  1.
  2.
  3.
Operator zone:  x [   ,   ]  y [   ,   ]  z [   ,   ]
```
