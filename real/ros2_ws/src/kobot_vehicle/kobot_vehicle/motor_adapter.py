import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped


class MotorAdapter(Node):
    def __init__(self):
        super().__init__('kobot_motor_adapter')
        self.declare_parameter('enabled', False)
        self.declare_parameter('angle_scale', 0.0068)
        self.declare_parameter('speed_scale', 0.08)
        self.declare_parameter('speed_limit_mps', 2.16)
        self.declare_parameter('acceleration_limit_mps2', 2.5)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.target_angle = 0.0
        self.target_speed = 0.0
        self.current_speed = 0.0
        self.last_command_ns = None
        self.publisher = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray, '/xycar_motor', self.on_command, 10)
        self.timer = self.create_timer(0.05, self.publish_command)
        if not self.get_parameter('enabled').value:
            self.get_logger().warning('Motor adapter is DISARMED; commands will be ignored')

    def on_command(self, msg):
        if not self.get_parameter('enabled').value or len(msg.data) < 2:
            return
        self.target_angle = max(-50.0, min(float(msg.data[0]), 100.0))
        speed = max(-50.0, min(float(msg.data[1]), 100.0))
        speed_mps = speed * float(self.get_parameter('speed_scale').value)
        speed_limit = float(self.get_parameter('speed_limit_mps').value)
        self.target_speed = max(-speed_limit, min(speed_mps, speed_limit))
        self.last_command_ns = self.get_clock().now().nanoseconds

    def publish_command(self):
        if not self.get_parameter('enabled').value:
            return
        if self.last_command_ns is None:
            return

        now = self.get_clock().now()
        timeout = float(self.get_parameter('command_timeout_sec').value)
        if self.last_command_ns is None or (now.nanoseconds - self.last_command_ns) / 1e9 > timeout:
            self.target_speed = 0.0

        # A zero command or command timeout is an immediate stop. Non-zero
        # commands are slew-limited centrally so every /xycar_motor publisher
        # gets the same safe start and direction-change behavior.
        if self.target_speed == 0.0:
            self.current_speed = 0.0
        else:
            step = float(self.get_parameter('acceleration_limit_mps2').value) * 0.05
            if self.current_speed * self.target_speed < 0.0:
                if self.current_speed > 0.0:
                    self.current_speed = max(0.0, self.current_speed - step)
                else:
                    self.current_speed = min(0.0, self.current_speed + step)
            elif self.current_speed < self.target_speed:
                self.current_speed = min(self.current_speed + step, self.target_speed)
            elif self.current_speed > self.target_speed:
                self.current_speed = max(self.current_speed - step, self.target_speed)

        out = AckermannDriveStamped()
        out.header.stamp = now.to_msg()
        out.drive.steering_angle = -(
            self.target_angle * float(self.get_parameter('angle_scale').value))
        out.drive.speed = self.current_speed
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MotorAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
