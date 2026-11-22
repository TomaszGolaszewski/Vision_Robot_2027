# Vision Robot 2027

## About
This project is part of my PhD research. 
Its goal is to enhance the capabilities of an industrial robot by integrating machine vision.

**Project under development!!!**

## Package `robot_machining_controller` (ROS 2 Jazzy)

```
camera ─image_raw, camera_info─▶ workpiece_position_detector ─workpiece_position─▶ trajectory_generator
                                   (QR + solvePnP, TF at image stamp)  (Kalman, path)     │ robot_position
                                                                                         ▼
                              /servo_node/pose_target_cmds ◀─ robot_controller (workspace box, speed limits, timeout)
```

All poses are `geometry_msgs/PoseStamped` in `base_link` (m, quaternion).
`robot_position` is the desired pose of `tool0`; the camera mount `tool0 → camera_optical_frame` comes from TF.

| Node | Main parameters |
|---|---|
| `workpiece_position_detector` | `qr_side_m` 0.05; `dpmm` 210 / `nominal_distance_mm` 440 build the fallback intrinsics (f = 924 px) until a calibrated `camera_info` arrives |
| `trajectory_generator` | `mode` `direct` (camera faces the code at `standoff_m` 0.44) or `circle` (`radius_m` 0.05 in the code plane, `period_s`); `lost_timeout_s`; `kalman_*` noises |
| `robot_controller` | `workspace_min/max`, `max_linear_speed` 0.1 m/s, `max_angular_speed`, `command_timeout_s`, `output_topic` |

Full list with defaults: `src/robot_machining_controller/config/params.yaml`.

### Build, test, run

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_machining_controller
colcon test --packages-select robot_machining_controller && colcon test-result --verbose
source install/setup.bash
ros2 launch robot_machining_controller follow_qr.launch.py mode:=circle
ros2 param set /trajectory_generator mode direct   # switch path live

# QR detection only (USB camera, no robot): workpiece_position = code pose relative to the camera
ros2 launch robot_machining_controller detect_qr.launch.py device_id:=0 width:=640 height:=480 dpmm:=210.0
ros2 topic echo /workpiece_position

# visualization: green outline + code axes = pose accepted, red outline = pose rejected (implausible)
ros2 launch robot_machining_controller detect_qr.launch.py show_image:=true  # cv2.imshow window (needs a display)
ros2 run rqt_image_view rqt_image_view /image_debug                          # or view the debug topic
```

### Known limitations (stage 1)

- **Camera mount TF is a placeholder** (`launch/follow_qr.launch.py`): replace it with a hand-eye calibration result.
- **Workspace box is a placeholder**: measure the safe zone in the cell.
- **QR orientation accuracy**: OpenCV `QRCodeDetector` corners are ~1 px off. For a 50 mm code at 440 mm (105 px)
  this is 7–14° of orientation error near a frontal view, i.e. up to ~100 mm lateral target error at 440 mm standoff.
  Position error is a few mm. The detector rejects solutions with reprojection error > 2 px or facing away.
  Tests marked `xfail` document this.
- QR detection takes ~65 ms at 1280×960 (≈15 Hz max); the detector always processes the newest frame.
