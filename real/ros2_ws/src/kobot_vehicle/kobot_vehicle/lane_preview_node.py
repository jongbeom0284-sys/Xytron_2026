import json
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def _fit_lane(points, y_top, y_bottom):
    if len(points) < 4:
        return None
    values = np.asarray(points, dtype=np.float32)
    ys = values[:, 1]
    xs = values[:, 0]
    if float(np.ptp(ys)) < 35.0:
        return None
    slope, intercept = np.polyfit(ys, xs, 1)
    return (
        (int(np.clip(slope * y_bottom + intercept, 0, 10000)), y_bottom),
        (int(np.clip(slope * y_top + intercept, 0, 10000)), y_top),
    )


def analyze_lane(frame):
    """Return an annotated camera frame and lane status; never controls the car."""
    output = frame.copy()
    height, width = frame.shape[:2]
    y_top = int(height * 0.40)
    y_bottom = height - 1
    center_x = width // 2

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, np.array([0, 0, 140]), np.array([179, 125, 255]))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 45, 140)
    color_band = cv2.dilate(white, np.ones((7, 7), np.uint8), iterations=1)
    binary = cv2.bitwise_and(edges, color_band)

    roi = np.zeros_like(binary)
    polygon = np.array([[
        (0, y_bottom),
        (width - 1, y_bottom),
        (int(width * 0.82), y_top),
        (int(width * 0.18), y_top),
    ]], dtype=np.int32)
    cv2.fillPoly(roi, polygon, 255)
    binary = cv2.bitwise_and(binary, roi)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    lines = cv2.HoughLinesP(
        binary,
        1,
        np.pi / 180.0,
        threshold=24,
        minLineLength=max(24, width // 18),
        maxLineGap=max(18, width // 24),
    )
    left_points = []
    right_points = []
    segment_count = 0
    if lines is not None:
        for raw in lines[:, 0]:
            x1, y1, x2, y2 = map(int, raw)
            dx = x2 - x1
            dy = y2 - y1
            if abs(dy) < 18:
                continue
            if max(y1, y2) < int(height * 0.58):
                continue
            slope = dy / float(dx if dx else 0.001)
            if abs(slope) < 0.35:
                continue
            midpoint = (x1 + x2) * 0.5
            segment_count += 1
            if midpoint < center_x and slope < 0:
                left_points.extend(((x1, y1), (x2, y2)))
            elif midpoint > center_x and slope > 0:
                right_points.extend(((x1, y1), (x2, y2)))

    left = _fit_lane(left_points, y_top, y_bottom)
    right = _fit_lane(right_points, y_top, y_bottom)
    lane_width = int(width * 0.46)
    state = 'no_lane'
    confidence = 0.0

    if left and right:
        state = 'both'
        confidence = min(1.0, 0.65 + segment_count * 0.035)
    elif left:
        state = 'left_only'
        confidence = min(0.65, 0.35 + segment_count * 0.025)
        right = ((left[0][0] + lane_width, y_bottom), (left[1][0] + lane_width // 2, y_top))
    elif right:
        state = 'right_only'
        confidence = min(0.65, 0.35 + segment_count * 0.025)
        left = ((right[0][0] - lane_width, y_bottom), (right[1][0] - lane_width // 2, y_top))

    offset_px = 0
    if left and right:
        left = tuple((int(np.clip(x, 0, width - 1)), y) for x, y in left)
        right = tuple((int(np.clip(x, 0, width - 1)), y) for x, y in right)
        lane_center_bottom = (left[0][0] + right[0][0]) // 2
        lane_center_top = (left[1][0] + right[1][0]) // 2
        offset_px = lane_center_bottom - center_x
        fill = output.copy()
        area = np.array([[left[0], left[1], right[1], right[0]]], dtype=np.int32)
        cv2.fillPoly(fill, area, (35, 170, 70))
        output = cv2.addWeighted(fill, 0.22, output, 0.78, 0)
        cv2.line(output, left[0], left[1], (0, 255, 255), 6, cv2.LINE_AA)
        cv2.line(output, right[0], right[1], (255, 210, 0), 6, cv2.LINE_AA)
        cv2.line(
            output,
            (lane_center_bottom, y_bottom),
            (lane_center_top, y_top),
            (0, 255, 80),
            4,
            cv2.LINE_AA,
        )

    cv2.polylines(output, polygon, True, (255, 120, 40), 2, cv2.LINE_AA)
    cv2.line(output, (center_x, y_bottom), (center_x, int(height * 0.72)), (40, 40, 255), 3)
    status_text = {
        'both': 'LANE OK',
        'left_only': 'LEFT ONLY',
        'right_only': 'RIGHT ONLY',
        'no_lane': 'NO LANE',
    }[state]
    cv2.rectangle(output, (12, 12), (305, 66), (15, 23, 42), -1)
    cv2.putText(output, status_text, (24, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2)
    cv2.putText(
        output,
        f'confidence {confidence:.2f}  offset {offset_px:+d}px',
        (24, 59),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (210, 225, 240),
        1,
    )
    return output, {
        'state': state,
        'confidence': round(float(confidence), 3),
        'offset_px': int(offset_px),
        'segments': int(segment_count),
        'left_detected': bool(left_points),
        'right_detected': bool(right_points),
    }


class LanePreviewNode(Node):
    def __init__(self):
        super().__init__('lane_preview')
        self.bridge = CvBridge()
        self.last_frame_at = 0.0
        self.image_pub = self.create_publisher(Image, '/lane_preview/image', 2)
        self.status_pub = self.create_publisher(String, '/lane_preview/status', 10)
        self.create_subscription(Image, '/image_raw', self.on_image, qos_profile_sensor_data)
        self.get_logger().info('Lane preview started; motor output is disabled')

    def on_image(self, message):
        now = time.monotonic()
        if now - self.last_frame_at < 1.0 / 15.0:
            return
        self.last_frame_at = now
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            overlay, status = analyze_lane(frame)
            image = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            image.header = message.header
            self.image_pub.publish(image)
            status_message = String()
            status_message.data = json.dumps(status, ensure_ascii=False)
            self.status_pub.publish(status_message)
        except Exception as exc:
            self.get_logger().error(f'Lane preview failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = LanePreviewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
