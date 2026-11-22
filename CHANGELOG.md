# Changelog

## [0.1.1] - 2026-11-22

### Added

- Added QR pose detection based on OpenCV and the IPPE `solvePnP` method, publishing poses as `geometry_msgs/PoseStamped` in the `base_link` frame using a TF transform evaluated at the image timestamp.
- Added a trajectory generator node (`trajectory_generator`) based on the OpenCV Kalman filter, supporting runtime-switchable direct and circular motion paths.
- Added the `robot_controller` node responsible for direct communication with the robot.
- Added a safety layer to the `robot_controller` node, enforcing workspace boundaries, speed limits, and command timeouts before forwarding commands to MoveIt Servo.
- Added launch and parameter files.
- Added offline unit tests.

### Changed

- Changed the `workpiece_position` and `robot_position` topics to use `geometry_msgs/PoseStamped` instead of `std_msgs/Float32MultiArray`.

## [0.1.0] - 2026-11-12

### Added
- Initialized the project repository.
- Prepared the initial project file structure.
