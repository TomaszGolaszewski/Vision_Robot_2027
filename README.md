# Vision Robot 2027

## About
This project is part of my PhD research. 
Its goal is to enhance the capabilities of an industrial robot by integrating machine vision.

**Project under development!!!**

## Package `robot_machining_controller` (ROS 2 Jazzy)

```
camera ─image_raw, camera_info─▶ workpiece_position_detector ─workpiece_position─▶ trajectory_generator
                     (ArUco + solvePnP, TF at image stamp, Kalman)                   (direct / circle path)
                                                                                         │ robot_position
                                                                                         ▼
   FANUC driver ◀─ MoveIt Servo ◀─ /servo_node/pose_target_cmds ◀─ robot_controller (robot status, workspace box,
   (forward_position_controller)                                                     speed limits, timeouts)
```

The robot pose comes from TF: from the FANUC driver (`robot.launch.py`), or a fixed `fake_tcp` pose without it.

All poses are `geometry_msgs/PoseStamped` in `base_link` (m, quaternion).
`robot_position` is the desired pose of `fanuc_flange`; the camera mount `fanuc_flange → camera_optical_frame` comes from TF.

| Node | Main parameters |
|---|---|
| `workpiece_position_detector` | `lost_timeout_s`, `kalman_*` noises; `marker_side_m` 0.1, `aruco_dictionary` DICT_4X4_50, `marker_id` 0; `camera_calibration_file` (set by the launch files to `config/camera_calibration_<camera>.yaml` if present); `dpmm` 210 / `nominal_distance_mm` 440 build the fallback intrinsics (f = 924 px) without it |
| `trajectory_generator` | `mode` `direct` (camera faces the code at `standoff_m` 0.44) or `circle` (`radius_m` 0.05 in the code plane, `period_s`); `lost_timeout_s` |
| `robot_controller` | `workspace_min/max`, `max_linear_speed` 0.1 m/s, `max_angular_speed`, `command_timeout_s`, `status_timeout_s`, `output_topic` |

Full list with defaults: `src/robot_machining_controller/config/params.yaml`.

### FANUC ROS 2 driver (separate workspace `~/ws_fanuc`)

Official driver, see the [FANUC docs](https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/quick_start/quick_start.html).
Controller options J519 + R912 (or S636), robot IP 192.168.1.100.

```bash
sudo apt install git-lfs && git lfs install            # robot meshes are in Git LFS
mkdir -p ~/ws_fanuc/src && cd ~/ws_fanuc/src
git clone https://github.com/FANUC-CORPORATION/fanuc_description.git
git clone --branch main --single-branch --recurse-submodules https://github.com/FANUC-CORPORATION/fanuc_driver.git
cd ~/ws_fanuc && rosdep install --ignore-src --from-paths src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Build, test, run

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_fanuc/install/setup.bash   # FANUC driver underlay (fanuc_msgs, robot model)
colcon build --packages-select robot_machining_controller
colcon test --packages-select robot_machining_controller && colcon test-result --verbose
source install/setup.bash
ros2 launch robot_machining_controller robot.launch.py use_mock:=true       # FANUC driver (mock) + MoveIt Servo
# the mock starts at all joints 0 = wrist singularity (Servo stops); move it away first, mock only:
ros2 topic pub --times 5 -r 10 /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0, 0.3, -0.3, 0, -0.8, 0]}"
ros2 launch robot_machining_controller follow_marker.launch.py mock_robot:=true  # mock publishes no robot status
ros2 launch robot_machining_controller follow_marker.launch.py mode:=circle  # camera + detector + robot chain
ros2 launch robot_machining_controller follow_marker.launch.py fake_robot:=true  # whole chain without the driver
ros2 param set /trajectory_generator mode direct   # switch path live

# marker detection only (USB camera, no robot): workpiece_position = marker pose relative to the camera
# camera:=201016 (default) | 200917 = USB camera unit (FLH id in /dev/v4l/by-id/), selects config/camera_calibration_<camera>.yaml
# device_id:=auto (default) = that unit, found by its /dev/v4l/by-id/ name; error if unplugged; device_id:=N = /dev/videoN
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_machining_controller detect_marker.launch.py width:=640 height:=480 dpmm:=210.0
ros2 launch robot_machining_controller detect_marker.launch.py fake_tcp:="0.5 0 0.6 0 0 0"  # other fixed robot pose
ros2 topic echo /workpiece_position

# visualization: green outline + marker axes = pose accepted, red outline = pose rejected (implausible),
# blue outline = Kalman-filtered pose
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
