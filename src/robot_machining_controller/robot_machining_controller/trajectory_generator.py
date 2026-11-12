import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.workpiece_position_subscriber = self.create_subscription(
            Float32MultiArray, 'workpiece_position', self.workpiece_callback, 10)
        self.robot_position_publisher = self.create_publisher(Float32MultiArray, 'robot_position', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.last_position = None

    def workpiece_callback(self, msg):
        self.last_position = msg

    def timer_callback(self):
        # nothing to forward until the first workpiece position arrives
        if self.last_position is not None:
            self.robot_position_publisher.publish(self.last_position)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
