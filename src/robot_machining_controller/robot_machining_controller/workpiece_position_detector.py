import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class WorkpiecePositionDetector(Node):
    def __init__(self):
        super().__init__('workpiece_position_detector')
        self.workpiece_position_publisher = self.create_publisher(Float32MultiArray, 'workpiece_position', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.t = 0.0

    def timer_callback(self):
        # shortcut: synthetic motion instead of real detection, replace with camera/sensor data
        self.t += 1.0 / 30.0
        x, y, z = math.cos(self.t), math.sin(self.t), 0.5
        roll, pitch, yaw = 0.0, 0.0, self.t % (2 * math.pi)
        msg = Float32MultiArray(data=[x, y, z, roll, pitch, yaw])
        self.workpiece_position_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WorkpiecePositionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
