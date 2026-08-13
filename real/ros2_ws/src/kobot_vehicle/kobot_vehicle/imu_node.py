import math
import serial
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu


DEG_TO_RAD = math.pi / 180.0


def quaternion_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def parse_imu_line(line, yaw_calibration=0.0):
    if not line.startswith('#YPRAG='):
        return None
    words = line[len('#YPRAG='):].strip().split(',')
    if len(words) < 9:
        return None
    try:
        yaw_deg = -float(words[2]) + yaw_calibration
        if yaw_deg > 180.0:
            yaw_deg -= 360.0
        if yaw_deg < -180.0:
            yaw_deg += 360.0
        roll = -float(words[0]) * DEG_TO_RAD
        pitch = float(words[1]) * DEG_TO_RAD
        yaw = yaw_deg * DEG_TO_RAD
        accel_factor = 9.806 / 256.0
        return {
            'quaternion': quaternion_from_euler(roll, pitch, yaw),
            'accel': (-float(words[3]) * accel_factor,
                      float(words[4]) * accel_factor,
                      float(words[5]) * accel_factor),
            'gyro': (float(words[6]), -float(words[7]), -float(words[8])),
        }
    except ValueError:
        return None


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('port', '/dev/ttyIMU')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('frame_id', 'base_imu_link')
        self.declare_parameter('yaw_calibration', 0.0)
        self.publisher = self.create_publisher(Imu, '/imu', 10)
        self.serial_port = None
        self.last_retry_ns = 0
        self.timer = self.create_timer(0.01, self.tick)

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
            # Select the proven text format. Calibration values are not rewritten.
            self.serial_port.write(b'#o0\r#ox\r#o1\r')
            self.get_logger().info('IMU connected; calibration left unchanged')
        except serial.SerialException as exc:
            self.get_logger().warning(f'IMU unavailable: {exc}')

    def tick(self):
        if self.serial_port is None or not self.serial_port.is_open:
            self.connect()
            return
        try:
            while self.serial_port.in_waiting:
                line = self.serial_port.readline().decode('utf-8', errors='ignore')
                parsed = parse_imu_line(line, float(self.get_parameter('yaw_calibration').value))
                if parsed is None:
                    continue
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.get_parameter('frame_id').value
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = parsed['quaternion']
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = parsed['accel']
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = parsed['gyro']
                msg.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
                msg.angular_velocity_covariance = [0.002, 0.0, 0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.002]
                msg.linear_acceleration_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]
                self.publisher.publish(msg)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'IMU disconnected: {exc}')
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
