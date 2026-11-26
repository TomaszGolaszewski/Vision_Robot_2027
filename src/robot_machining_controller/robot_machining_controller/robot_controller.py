import numpy as np
import rclpy
from fanuc_msgs.msg import ConnectionStatus, RobotStatus
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from robot_machining_controller.pose_math import compose, from_pose, from_transform, limit_step, to_pose


class RobotController(Node):
    """Safety gate between robot_position and MoveIt Servo: robot status, workspace box, speed limits, timeouts.

    in:  robot_position (PoseStamped, desired tool pose in base_link), TF base_link -> fanuc_flange (actual robot pose)
         /fanuc_gpio_controller/connection_status, robot_status (FANUC driver, ~500 Hz)
    out: output_topic   (PoseStamped, speed-limited servo_frame pose for MoveIt Servo), published at rate_hz
    """

    def __init__(self):
        super().__init__('robot_controller')
        p = self.declare_parameter

        # --- frames and loop rate ---
        self.base_frame = p('base_frame', 'base_link').value
        self.tool_frame = p('tool_frame', 'fanuc_flange').value
        self.servo_frame = p('servo_frame', 'flange').value  # tip of the Servo planning group (srdf)
        rate = p('rate_hz', 50.0).value
        self.dt = 1.0 / rate

        # --- safety limits ---
        self.max_linear_speed = p('max_linear_speed', 0.1).value     # [m/s]
        self.max_angular_speed = p('max_angular_speed', 0.5).value   # [rad/s]
        self.command_timeout = p('command_timeout_s', 0.5).value     # older target -> stop sending
        self.status_timeout = p('status_timeout_s', 0.2).value       # older driver status -> stop sending
        # false only for the FANUC mock hardware, which publishes no status (its fanuc_gpio_controller does not start);
        # as soon as any driver status arrives, it is enforced anyway
        self.require_robot_status = p('require_robot_status', True).value
        # axis-aligned box in base_link the tool target must stay inside
        self.workspace_min = np.array(p('workspace_min', [0.3, -0.6, 0.1]).value)
        self.workspace_max = np.array(p('workspace_max', [1.2, 0.6, 1.2]).value)

        # --- TF: actual robot pose, used as the starting point of the speed ramp ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- topics and the fixed-rate command loop ---
        self.create_subscription(PoseStamped, 'robot_position', self.robot_position_callback, 10)
        self.command_publisher = self.create_publisher(
            PoseStamped, p('output_topic', '/servo_node/pose_target_cmds').value, 10)
        self.create_subscription(ConnectionStatus, '/fanuc_gpio_controller/connection_status',
                                 lambda m: setattr(self, 'connection', (m, self.get_clock().now())), 1)
        self.create_subscription(RobotStatus, '/fanuc_gpio_controller/robot_status',
                                 lambda m: setattr(self, 'status', (m, self.get_clock().now())), 1)
        # Servo starts in joint-jog mode; it must be switched to pose commands once
        self.servo_mode_client = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # --- runtime state ---
        self.target = None        # newest desired pose (position, Rotation)
        self.target_time = None   # its arrival time, for command_timeout
        self.last_sent = None     # pose sent in the previous tick; None = restart the ramp from the robot pose
        self.connection = None    # (ConnectionStatus, arrival time)
        self.status = None        # (RobotStatus, arrival time)
        self.servo_mode = None    # future of the switch to pose mode; done + success = Servo accepts poses
        self.servo_in_tool = None  # cached static transform tool_frame -> servo_frame

    def robot_position_callback(self, msg):
        # accept only targets in our base frame, mixing frames would send the robot somewhere else
        if msg.header.frame_id != self.base_frame:
            self.get_logger().warn(f'robot_position in {msg.header.frame_id!r}, expected {self.base_frame!r}',
                                   throttle_duration_sec=1.0)
            return
        self.target = from_pose(msg.pose)
        self.target_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()

        # 0. robot not ready (driver down, disconnected, alarm, e-stop, pendant enabled) or Servo not in pose mode
        problem = self.robot_problem(now)
        if problem:
            if self.target is not None:
                self.get_logger().warn(f'not moving: {problem}', throttle_duration_sec=1.0)
            self.last_sent = None
            return

        # 1. no target or target too old (code lost, generator stopped): send nothing
        if self.target is None or (now - self.target_time).nanoseconds * 1e-9 > self.command_timeout:
            self.last_sent = None  # no fresh command: send nothing, Servo halts on its own timeout
            return

        # 2. target outside the allowed box: refuse it and stay where we are
        if not (np.all(self.target[0] >= self.workspace_min) and np.all(self.target[0] <= self.workspace_max)):
            self.get_logger().warn(f'target {self.target[0].round(3)} outside workspace, not moving',
                                   throttle_duration_sec=1.0)
            self.last_sent = None
            return

        # 3. (re)start: begin the ramp at the current tool pose, never jump to the target
        if self.last_sent is None:
            try:  # ramp from where the robot actually is
                t = self.tf_buffer.lookup_transform(self.base_frame, self.tool_frame, Time())
            except TransformException as e:
                self.get_logger().warn(f'no TF {self.base_frame}<-{self.tool_frame}: {e}', throttle_duration_sec=1.0)
                return
            self.last_sent = from_transform(t.transform)

        # 4. move the commanded pose towards the target by at most (speed limit * dt) per tick
        self.last_sent = limit_step(*self.last_sent, *self.target,
                                    self.max_linear_speed * self.dt, self.max_angular_speed * self.dt)

        # 5. send the limited pose to MoveIt Servo, as the pose of its tip link (flange, not fanuc_flange)
        out = PoseStamped()
        out.header.stamp = now.to_msg()
        out.header.frame_id = self.base_frame
        out.pose = to_pose(*compose(self.last_sent, self.servo_in_tool))
        self.command_publisher.publish(out)

    def robot_problem(self, now):
        """Why the robot must not be commanded now, or '' when it may."""
        def fresh(x):
            return x is not None and (now - x[1]).nanoseconds * 1e-9 <= self.status_timeout
        if self.require_robot_status or self.connection is not None or self.status is not None:
            problem = self.status_problem(fresh)
            if problem:
                return problem
        else:
            self.get_logger().warn('require_robot_status is false: driver status not checked (mock only)',
                                   throttle_duration_sec=10.0)
        return self.servo_problem()

    def status_problem(self, fresh):
        if not fresh(self.connection) or not self.connection[0].is_connected:
            return 'no connection to the robot (fanuc_gpio_controller/connection_status)'
        if not fresh(self.status):
            return 'no robot status (fanuc_gpio_controller/robot_status)'
        s = self.status[0]
        if s.in_error or s.e_stopped or s.tp_enabled or not s.motion_possible:
            return (f'robot status in_error={s.in_error} e_stopped={s.e_stopped} tp_enabled={s.tp_enabled} '
                    f'motion_possible={s.motion_possible}')
        return ''

    def servo_problem(self):
        if self.servo_in_tool is None:
            try:
                t = self.tf_buffer.lookup_transform(self.tool_frame, self.servo_frame, Time())
            except TransformException as e:
                return f'no TF {self.tool_frame}<-{self.servo_frame}: {e}'
            self.servo_in_tool = from_transform(t.transform)  # static, look it up once
        if self.servo_mode is None or (self.servo_mode.done() and not self.servo_mode.result().success):
            if self.servo_mode_client.service_is_ready():
                self.servo_mode = self.servo_mode_client.call_async(
                    ServoCommandType.Request(command_type=ServoCommandType.Request.POSE))
            return 'MoveIt Servo not in pose mode yet (/servo_node/switch_command_type)'
        if not self.servo_mode.done():
            return 'waiting for MoveIt Servo to switch to pose mode'
        return ''


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)  # process callbacks until Ctrl+C
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
