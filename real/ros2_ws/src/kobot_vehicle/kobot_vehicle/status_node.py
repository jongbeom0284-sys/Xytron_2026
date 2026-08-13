import glob
import json
import os
import socket
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String


class StatusNode(Node):
    def __init__(self):
        super().__init__('kobot_status')
        self.publisher = self.create_publisher(String, '/kobot/status', 10)
        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        devices = {name: os.path.exists(path) for name, path in {
            'camera': '/dev/videoCAM', 'lidar': '/dev/ttyLIDAR',
            'ultrasonic': '/dev/ttySonic', 'imu': '/dev/ttyIMU',
            'motor': '/dev/ttyMOTOR',
        }.items()}
        temperatures = {}
        for path in glob.glob('/sys/class/thermal/thermal_zone*/temp'):
            try:
                zone = os.path.basename(os.path.dirname(path))
                with open(path, encoding='ascii') as handle:
                    temperatures[zone] = round(float(handle.read().strip()) / 1000.0, 1)
            except (OSError, ValueError):
                pass
        try:
            mode = subprocess.run(['nvpmodel', '-q'], capture_output=True, text=True, timeout=1).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            mode = 'unknown'
        msg = String()
        msg.data = json.dumps({
            'hostname': socket.gethostname(), 'devices': devices,
            'temperatures_c': temperatures, 'power_mode': mode,
        }, ensure_ascii=False)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StatusNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
