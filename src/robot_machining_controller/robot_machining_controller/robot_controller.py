import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from robot_machining_controller.pose_math import from_pose, from_transform, limit_step, to_pose


class RobotController(Node):
    """Safety gate between robot_position and MoveIt Servo: workspace box, speed limits, command timeout.

    in:  robot_position (PoseStamped, desired tool0 pose in base_link), TF base_link -> tool0 (actual robot pose)
    out: output_topic   (PoseStamped, speed-limited tool0 pose for MoveIt Servo), published at rate_hz
    """

    def __init__(self):
        super().__init__('robot_controller')
        p = self.declare_parameter

        # --- frames and loop rate ---
        self.base_frame = p('base_frame', 'base_link').value
        self.tool_frame = p('tool_frame', 'tool0').value
        rate = p('rate_hz', 50.0).value
        self.dt = 1.0 / rate

        # --- safety limits ---
        self.max_linear_speed = p('max_linear_speed', 0.1).value     # [m/s]
        self.max_angular_speed = p('max_angular_speed', 0.5).value   # [rad/s]
        self.command_timeout = p('command_timeout_s', 0.5).value     # older target -> stop sending
        # axis-aligned box in base_link the tool0 target must stay inside
        self.workspace_min = np.array(p('workspace_min', [0.3, -0.6, 0.1]).value)
        self.workspace_max = np.array(p('workspace_max', [1.2, 0.6, 1.2]).value)

        # --- TF: actual robot pose, used as the starting point of the speed ramp ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- topics and the fixed-rate command loop ---
        self.create_subscription(PoseStamped, 'robot_position', self.robot_position_callback, 10)
        self.command_publisher = self.create_publisher(
            PoseStamped, p('output_topic', '/servo_node/pose_target_cmds').value, 10)
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # --- runtime state ---
        self.target = None        # newest desired pose (position, Rotation)
        self.target_time = None   # its arrival time, for command_timeout
        self.last_sent = None     # pose sent in the previous tick; None = restart the ramp from the robot pose

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

        # 3. (re)start: begin the ramp at the current tool0 pose, never jump to the target
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

        # 5. send the limited pose to MoveIt Servo
        out = PoseStamped()
        out.header.stamp = now.to_msg()
        out.header.frame_id = self.base_frame
        out.pose = to_pose(*self.last_sent)
        self.command_publisher.publish(out)


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
