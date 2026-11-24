# Vision Robot 2027

## About
This project is part of my PhD research. 
Its goal is to enhance the capabilities of an industrial robot by integrating machine vision.

**Project under development!!!**

## Package `robot_machining_controller` (ROS 2 Jazzy)

```
camera ─image_raw, camera_info─▶ workpiece_position_detector ─workpiece_position─▶ trajectory_generator
                                   (ArUco + solvePnP, TF at image stamp)  (Kalman, path)     │ robot_position
                                                                                         ▼
                              /servo_node/pose_target_cmds ◀─ robot_controller (workspace box, speed limits, timeout)
```

All poses are `geometry_msgs/PoseStamped` in `base_link` (m, quaternion).
`robot_position` is the desired pose of `tool0`; the camera mount `tool0 → camera_optical_frame` comes from TF.

| Node | Main parameters |
|---|---|
| `workpiece_position_detector` | `marker_side_m` 0.1, `aruco_dictionary` DICT_4X4_50, `marker_id` 0; `camera_calibration_file` (set by the launch files to `config/camera_calibration_<camera>.yaml` if present); `dpmm` 210 / `nominal_distance_mm` 440 build the fallback intrinsics (f = 924 px) without it |
| `trajectory_generator` | `mode` `direct` (camera faces the code at `standoff_m` 0.44) or `circle` (`radius_m` 0.05 in the code plane, `period_s`); `lost_timeout_s`; `kalman_*` noises |
| `robot_controller` | `workspace_min/max`, `max_linear_speed` 0.1 m/s, `max_angular_speed`, `command_timeout_s`, `output_topic` |

Full list with defaults: `src/robot_machining_controller/config/params.yaml`.

### Build, test, run

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_machining_controller
colcon test --packages-select robot_machining_controller && colcon test-result --verbose
source install/setup.bash
ros2 launch robot_machining_controller follow_marker.launch.py mode:=circle  # camera + detector + robot chain
ros2 param set /trajectory_generator mode direct   # switch path live

# marker detection only (USB camera, no robot): workpiece_position = marker pose relative to the camera
# camera:=201016 (default) | 200917 = USB camera unit (FLH id in /dev/v4l/by-id/), selects config/camera_calibration_<camera>.yaml
# device_id:=auto (default) = that unit, found by its /dev/v4l/by-id/ name; error if unplugged; device_id:=N = /dev/videoN
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_machining_controller detect_marker.launch.py width:=640 height:=480 dpmm:=210.0
ros2 topic echo /workpiece_position

# visualization: green outline + marker axes = pose accepted, red outline = pose rejected (implausible)
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_machining_controller detect_marker.launch.py show_image:=true  # cv2.imshow window (needs a display)
ros2 run rqt_image_view rqt_image_view /image_debug                          # or view the debug topic

# camera intrinsics with the ChArUco board (7x5, 25/18 mm, DICT_6X6_250), see docs/calibration_procedure.md step 2;
# then colcon build: the launch files load config/camera_calibration_<camera>.yaml automatically
ros2 run robot_machining_controller calibrate_camera --output src/robot_machining_controller/config/camera_calibration_201016.yaml

# printable marker (id 0, DICT_4X4_50); print without scaling, then measure the outer black edge -> marker_side_m
python3 -c "import cv2; cv2.imwrite('marker_0.png', cv2.copyMakeBorder(cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 0, 600), 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255))"
```

### Known limitations (stage 1)

- **Camera mount TF is a placeholder** (`launch/detect_marker.launch.py`, also used by `follow_marker.launch.py`): replace it with a hand-eye calibration result.
- **Workspace box is a placeholder**: measure the safe zone in the cell.
- **Marker size in pixels limits orientation accuracy**: ArUco corners (subpixel) are ~0.1 px accurate. A 50 mm marker
  at 440 mm spans 105 px at 1280×960: orientation error < 1°. At 640×480 it spans only ~52 px and near-frontal views
  are 5–7° off (tests marked `xfail`). The 100 mm marker in use spans ~105 px at 640×480: < 0.5°, < 2 mm.
  The detector rejects solutions with reprojection error > 2 px or facing away.
- Detection takes ~2.5 ms at 640×480 and ~11 ms at 1280×960; the detector always processes the newest frame.
