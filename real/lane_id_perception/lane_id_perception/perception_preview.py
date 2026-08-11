from __future__ import annotations

import json
import time
from typing import Any

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .lane_config import lane_vision_config_from_getter
from .lane_vision import LaneDetector


def stamp_to_float(stamp: Any) -> float:
    sec = float(getattr(stamp, "sec", 0))
    nanosec = float(getattr(stamp, "nanosec", 0))
    value = sec + nanosec * 1e-9
    return value if value > 0.0 else time.time()


class LanePerceptionPreviewNode(Node):
    """Publish lane perception diagnostics without publishing motor commands."""

    def __init__(self) -> None:
        super().__init__(
            "lane_perception_preview",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.bridge = CvBridge()
        self.frame_id = 0
        self.last_processing_ms = 0.0

        self.image_topic = str(self._param("image_topic", "/image_raw"))
        self.debug_topic = str(self._param("debug_topic", "/lane_id/debug"))
        self.debug_image_topic = str(self._param("debug_image_topic", "/lane_id/debug_image"))
        self.front_detection_image_topic = str(
            self._param("front_detection_image_topic", "/lane_id/front_detection_image")
        )
        self.bev_image_topic = str(self._param("bev_image_topic", "/lane_id/bev_image"))
        self.mask_image_topic = str(self._param("mask_image_topic", "/lane_id/mask_image"))
        self.publish_debug_image = bool(self._param("publish_debug_image", True))
        self.show_debug_window = bool(self._param("show_debug_window", False))

        self.detector = LaneDetector(lane_vision_config_from_getter(self._param))

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, self.debug_image_topic, 2)
        self.front_detection_image_pub = self.create_publisher(Image, self.front_detection_image_topic, 2)
        self.bev_image_pub = self.create_publisher(Image, self.bev_image_topic, 2)
        self.mask_image_pub = self.create_publisher(Image, self.mask_image_topic, 2)
        self.create_subscription(Image, self.image_topic, self.image_callback, sensor_qos)
        self.get_logger().info(
            "lane perception preview ready: "
            f"image={self.image_topic}, debug={self.debug_topic}, "
            f"debug_image={self.debug_image_topic}, front_detection_image={self.front_detection_image_topic}, "
            f"bev_image={self.bev_image_topic}, "
            f"mask_image={self.mask_image_topic}; motor publishing disabled"
        )

    def _param(self, name: str, default: Any) -> Any:
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        value = self.get_parameter(name).value
        return default if value is None else value

    def image_callback(self, msg: Image) -> None:
        capture_stamp = stamp_to_float(msg.header.stamp)
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"image conversion failed: {exc}")
            return

        started = time.monotonic()
        self.frame_id += 1
        result = self.detector.process(
            bgr,
            capture_timestamp=capture_stamp,
            frame_id=self.frame_id,
            make_debug=True,
        )
        self.last_processing_ms = (time.monotonic() - started) * 1000.0

        if self.publish_debug_image and result.debug_image is not None:
            self._publish_image(self.debug_image_pub, result.debug_image, "front_camera_debug")
        if self.publish_debug_image and result.front_detection_image is not None:
            self._publish_image(
                self.front_detection_image_pub,
                result.front_detection_image,
                "front_detection",
            )
        if self.publish_debug_image and result.bev_debug_image is not None:
            self._publish_image(self.bev_image_pub, result.bev_debug_image, "lane_bev")
        if self.publish_debug_image and result.mask_debug_image is not None:
            self._publish_image(self.mask_image_pub, result.mask_debug_image, "lane_mask")
        if self.show_debug_window and result.debug_image is not None:
            cv2.imshow("lane_id_perception preview", result.debug_image)
            cv2.waitKey(1)

        payload = {
            "preview_only": True,
            "frame_id": result.frame_id,
            "lane_state": result.state,
            "confidence": round(result.overall_lane_confidence, 3),
            "intersection_like": bool(result.intersection_like),
            "lane_width_m": round(result.estimated_lane_width, 3),
            "lateral_error_near_m": round(result.lateral_error_near, 3),
            "heading_error_rad": round(result.heading_error, 4),
            "curvature": round(result.estimated_curvature, 4),
            "processing_ms": round(self.last_processing_ms, 3),
            "fit_reject_reasons": result.debug.get("fit_reject_reasons", []),
            "tracking_reject_reasons": result.debug.get("tracking", {}).get("reject_reasons", []),
        }
        debug = String()
        debug.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self.debug_pub.publish(debug)

    def _publish_image(self, publisher: Any, bgr: Any, frame_id: str) -> None:
        try:
            out = self.bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = frame_id
            publisher.publish(out)
        except Exception as exc:
            self.get_logger().warn(f"{frame_id} image publish failed: {exc}")

    def destroy_node(self) -> bool:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LanePerceptionPreviewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
