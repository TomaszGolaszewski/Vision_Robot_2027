import math

import rclpy
from fanuc_msgs.msg import ConnectionStatus
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

JOINTS = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']  # forward_position_controller order


class MockHome(Node):
    """FANUC mock hardware only: put the robot into a start joint pose, then exit.

    The mock always starts at all zeros (J5 = 0, wrist singularity: MoveIt Servo refuses to move). This pauses Servo
    (a halted Servo keeps publishing the current pose and would overwrite ours), jumps the mock to `positions_deg`
    (pendant joint angles, the driver converts deg <-> rad 1:1) and resumes Servo.
    Never on the real robot: a jump bypasses every ramp. It aborts as soon as the driver reports a connection.
    """

    def __init__(self):
        super().__init__('mock_home')
        # typed, no default: unset -> ParameterUninitializedException at startup
        positions = self.declare_parameter('positions_deg', Parameter.Type.DOUBLE_ARRAY).value
        if len(positions) != len(JOINTS):
            raise ValueError(f'positions_deg needs {len(JOINTS)} joint angles [deg], got {positions}')
        self.target = [math.radians(a) for a in positions]
        self.tolerance = math.radians(0.01)

        self.publisher = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 1)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)
        self.create_subscription(ConnectionStatus, '/fanuc_gpio_controller/connection_status',
                                 self.connection_callback, 1)
        self.pause_client = self.create_client(SetBool, '/servo_node/pause_servo')
        self.create_timer(0.1, self.timer_callback)

        self.joints = None   # newest positions in JOINTS order
        self.paused = None   # future of the pause call
        self.done = False

    def connection_callback(self, msg):
        # the mock's fanuc_gpio_controller does not run: any status means a real driver, do not touch it
        self.get_logger().fatal('FANUC driver connection status received: not mock hardware, aborting')
        self.done = True

    def joint_state_callback(self, msg):
        names = dict(zip(msg.name, msg.position))
        if all(j in names for j in JOINTS):
            self.joints = [names[j] for j in JOINTS]

    def timer_callback(self):
        if self.done or self.joints is None or not self.pause_client.service_is_ready():
            return
        if self.paused is None:
            self.paused = self.pause_client.call_async(SetBool.Request(data=True))
            return
        if not self.paused.done():
            return
        if max(abs(a - b) for a, b in zip(self.joints, self.target)) > self.tolerance:
            self.publisher.publish(Float64MultiArray(data=self.target))
            return
        self.pause_client.call_async(SetBool.Request(data=False))
        self.get_logger().info(f'mock robot at start pose {[round(math.degrees(a), 2) for a in self.target]} deg')
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = MockHome()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
        rclpy.spin_once(node, timeout_sec=0.5)  # let the resume call go out
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
