# Changelog

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
