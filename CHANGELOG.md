# Changelog

## [0.1.7] - 2026-11-27

### Added

- Added the `mock_home` node and `robot.launch.py` argument (`"J1 .. J6"` in degrees) to set the initial mock robot joint pose when `use_mock:=true`.

## [0.1.6] - 2026-11-26

### Added

- Added a blue outline of the Kalman-filtered marker pose to `image_debug`.
- Added the `workpiece_position_raw` topic for the unfiltered marker pose and Kalman filter tuning.
- Added the `fake_tcp` launch argument (`"x y z roll pitch yaw"`) for use with `fake_robot:=true`.
- Added the `fake_robot` argument to `follow_marker.launch.py`.
- Added `robot.launch.py` with the FANUC ROS 2 driver (`forward_position_controller`, mock or real) and MoveIt Servo.
- Added robot status checks to `robot_controller`; command transmission stops on driver disconnect, alarm, e-stop, teach mode, or unavailable motion.
- Added the `mock_robot` argument to `follow_marker.launch.py` (`require_robot_status:=false`).

### Changed

- Moved the Kalman filter from `trajectory_generator` to `workpiece_position_detector`; `workpiece_position` is now filtered and predicted through short detection dropouts.
- Moved the `kalman_*` and `lost_timeout_s` parameters to `workpiece_position_detector`.
- Renamed the tool frame from `tool0` to `fanuc_flange`.
- Changed `robot_controller` to publish Servo targets as the `flange` pose.

## [0.1.5] - 2026-11-24

### Added
 
- Added the `camera` launch argument to select one of the two USB cameras (`201016` or `200917`).
 
### Changed
 
- Renamed `config/camera_calibration.yaml` to `config/camera_calibration_201016.yaml`.
- Updated launch files to load `config/camera_calibration_<camera>.yaml` automatically.

## [0.1.4] - 2026-11-23

### Added

- Added the `calibrate_camera` script for camera calibration using a ChArUco board.
- Added `config/camera_calibration.yaml` with USB camera intrinsics.
- Added the `camera_calibration_file` parameter to `workpiece_position_detector`.

## [0.1.3] - 2026-11-23

### Added

- Added automatic USB camera selection in `detect_marker.launch.py`.
- Added camera startup to `follow_marker.launch.py`.
- Added `aruco_dictionary` and `marker_id` parameters to `workpiece_position_detector`.
- Added `docs/calibration_procedure.md` with stage 2 calibration instructions.

### Changed

- Replaced QR code-based workpiece detection with ArUco marker detection.
- Renamed `qr_side_m` to `marker_side_m` and updated its default value.
- Renamed `detect_qr.launch.py` to `detect_marker.launch.py`.
- Renamed `follow_qr.launch.py` to `follow_marker.launch.py`.

## [0.1.2] - 2026-11-22

### Added

- Added the `image_debug` topic to `workpiece_position_detector`.
- Added frame rate and detection rate overlays to the debug image.
- Added the `show_image` parameter to display the debug image in a window.

## [0.1.1] - 2026-11-22

### Added

- Added QR pose detection in `workpiece_position_detector`.
- Added the `trajectory_generator` node.
- Added the `robot_controller` node.
- Added safety checks to `robot_controller`.
- Added launch and parameter files.
- Added offline unit tests.

### Changed

- Changed the `workpiece_position` and `robot_position` topics to use `geometry_msgs/PoseStamped`.

## [0.1.0] - 2026-11-12

### Added
- Initialized the project repository.
- Prepared the initial project file structure.
