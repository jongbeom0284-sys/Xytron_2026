import serial
from rclpy.node import Node
import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Int32MultiArray


def parse_ultrasonic_line(line: str):
    tokens = [token.strip() for token in line.strip().split(',')]
    if len(tokens) < 8:
        return None
    values = []
    for token in tokens[-8:]:
        try:
            value = int(token)
        except ValueError:
            value = 0
        values.append(value if 0 <= value <= 140 else 0)
    return values


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('xycar_ultrasonic')
        self.declare_parameter('port', '/dev/ttySonic')
        self.declare_parameter('baudrate', 115200)
        self.publisher = self.create_publisher(Int32MultiArray, '/xycar_ultrasonic', 10)
        self.serial_port = None
        self.buffer = bytearray()
        self.last_retry_ns = 0
        self.timer = self.create_timer(1.0 / 30.0, self.tick)

    def connect(self):
        now = self.get_clock().now().nanoseconds
        if now - self.last_retry_ns < 3_000_000_000:
            return
        self.last_retry_ns = now
        try:
            self.serial_port = serial.Serial(
                self.get_parameter('port').value,
                int(self.get_parameter('baudrate').value),
                timeout=0,
            )
            self.get_logger().info('Ultrasonic connected')
        except serial.SerialException as exc:
            self.get_logger().warning(f'Ultrasonic unavailable: {exc}')

    def tick(self):
        if self.serial_port is None or not self.serial_port.is_open:
            self.connect()
            return
        try:
            waiting = self.serial_port.in_waiting
            if waiting:
                self.buffer.extend(self.serial_port.read(waiting))
            while b'\n' in self.buffer:
                raw, _, self.buffer = self.buffer.partition(b'\n')
                values = parse_ultrasonic_line(raw.decode('ascii', errors='ignore'))
                if values is not None:
                    msg = Int32MultiArray()
                    msg.data = values
                    self.publisher.publish(msg)
            if len(self.buffer) > 2048:
                self.buffer = self.buffer[-512:]
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Ultrasonic disconnected: {exc}')
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
