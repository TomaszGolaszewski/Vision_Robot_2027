import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from robot_machining_controller.pose_math import (MODES, PoseKalman, camera_target, compose, from_pose,
                                                  from_transform, to_pose)


class TrajectoryGenerator(Node):
    """Filters the code pose and publishes the tool0 pose that puts the camera on the selected path.

    in:  workpiece_position (PoseStamped, code pose in base_link)
    out: robot_position     (PoseStamped, desired tool0 pose in base_link), published at rate_hz
    """

    def __init__(self):
        super().__init__('trajectory_generator')
        p = self.declare_parameter

        # --- path selection; reject unknown modes at start-up and on `ros2 param set` ---
        p('mode', 'direct')  # read every tick, so `ros2 param set` switches the path live
        if self.get_parameter('mode').value not in MODES:
            raise ValueError(f'mode must be one of {MODES}')
        self.add_on_set_parameters_callback(lambda ps: SetParametersResult(
            successful=all(x.name != 'mode' or x.value in MODES for x in ps)))

        # --- path geometry and timing ---
        self.standoff = p('standoff_m', 0.44).value          # camera distance from the code along its normal
        self.radius = p('radius_m', 0.05).value              # circle radius in the code plane
        self.period = p('period_s', 10.0).value              # time of one full circle
        self.lost_timeout = p('lost_timeout_s', 0.5).value   # no detection for longer -> stop commanding
        self.camera_frame = p('camera_frame', 'camera_optical_frame').value
        self.tool_frame = p('tool_frame', 'tool0').value

        # --- Kalman filter smoothing the detected code pose (noise values: see config/params.yaml) ---
        self.kalman = PoseKalman(p('kalman_accel_noise', 0.25).value, p('kalman_rot_noise', 0.001).value,
                                 p('kalman_pos_meas_noise', 1e-6).value, p('kalman_rot_meas_noise', 1e-4).value)

        # --- TF: needed for the camera mount (tool0 relative to the camera) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- topics and the fixed-rate output loop ---
        self.workpiece_position_subscriber = self.create_subscription(
            PoseStamped, 'workpiece_position', self.workpiece_callback, 10)
        self.robot_position_publisher = self.create_publisher(PoseStamped, 'robot_position', 10)
        self.timer = self.create_timer(1.0 / p('rate_hz', 30.0).value, self.timer_callback)

        # --- runtime state ---
        self.measurement = None             # newest detection not yet fed to the filter
        self.frame_id = None                # frame of the detections (base_link), reused for the output
        self.last_measurement_time = None   # arrival time of the newest detection, for lost_timeout
        self.last_tick = None               # time of the previous timer tick, for the filter dt
        self.tool_in_camera = None          # cached static mount transform camera -> tool0

    def workpiece_callback(self, msg):
        # only store the detection; filtering and publishing happen in the timer at a constant rate
        self.measurement = from_pose(msg.pose)
        self.frame_id = msg.header.frame_id
        self.last_measurement_time = self.get_clock().now()

    def timer_callback(self):
        # time since the previous tick = prediction step of the Kalman filter
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds * 1e-9 if self.last_tick else 0.0
        self.last_tick = now

        # code never seen or lost for too long: publish nothing (RobotController then stops on its own timeout)
        if (self.last_measurement_time is None
                or (now - self.last_measurement_time).nanoseconds * 1e-9 > self.lost_timeout):
            self.kalman.reset()  # code lost: stop commanding, start fresh when it comes back
            return

        # filter: predict to `now`, correct with the new detection if one arrived since the last tick
        code = self.kalman.update(dt, self.measurement)
        self.measurement = None
        if code is None:
            return

        # camera mount transform, looked up once when TF first has it
        if self.tool_in_camera is None:
            try:
                t = self.tf_buffer.lookup_transform(self.camera_frame, self.tool_frame, Time())
            except TransformException as e:
                self.get_logger().warn(f'no TF {self.camera_frame}<-{self.tool_frame}: {e}',
                                       throttle_duration_sec=1.0)
                return
            self.tool_in_camera = from_transform(t.transform)  # static camera mount, look it up once

        # where the camera should be on the selected path (direct / circle) ...
        camera = camera_target(*code, self.standoff, self.get_parameter('mode').value,
                               now.nanoseconds * 1e-9, self.radius, self.period)
        # ... converted to the tool0 pose the robot has to reach so the camera ends up there
        out = PoseStamped()
        out.header.stamp = now.to_msg()
        out.header.frame_id = self.frame_id
        out.pose = to_pose(*compose(camera, self.tool_in_camera))
        self.robot_position_publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)  # process callbacks until Ctrl+C
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
