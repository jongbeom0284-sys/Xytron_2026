import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray


class FigureEight(Node):
    def __init__(self):
        super().__init__('kobot_figure_eight')
        self.declare_parameter('armed', False)
        self.declare_parameter('speed', 12.0)
        self.publisher = self.create_publisher(Float32MultiArray, '/xycar_motor', 10)
        self.angle = 0.0
        self.step = 5.0
        self.timer = self.create_timer(0.1, self.tick)
        if not self.get_parameter('armed').value:
            self.get_logger().warning('Figure-eight test is DISARMED; no commands will be published')

    def tick(self):
        if not self.get_parameter('armed').value:
            return
        self.angle += self.step
        if self.angle >= 100.0 or self.angle <= -100.0:
            self.angle = max(-100.0, min(self.angle, 100.0))
            self.step *= -1.0
        msg = Float32MultiArray()
        msg.data = [self.angle, float(self.get_parameter('speed').value)]
        self.publisher.publish(msg)

    def stop(self):
        if self.get_parameter('armed').value:
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0]
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FigureEight()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
