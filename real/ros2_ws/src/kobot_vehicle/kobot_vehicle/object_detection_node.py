#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 YOLOv8 전용 객체 탐지(Object Detection) 디버깅 노드
- 카메라 토픽(/image_raw/compressed)을 수신하여 YOLOv8 추론 수행
- 특정 클래스(예: ID 28 suitcase) 제외 필터링 지원
- 디버깅 창(OpenCV GUI)에 객체 BBox, Class ID, Class Label, Confidence 실시간 출력
- 순수 객체 인지 및 디버깅 전용 (주행/모터 제어 로직 배제)
"""

import os
import sys

# ==============================================================================
# [중요] 최신 numpy 및 가상환경 패키지를 시스템 기본보다 최우선으로 로드
# (cv2, numpy, torch, ultralytics import 전에 반드시 수행해야 함)
# ==============================================================================
VENV_SITE_PACKAGES = [
    '/home/xytron/clrernet_eval/venv/lib/python3.10/site-packages',
    '/home/xytron/2026_Xytron/real/.venv/lib/python3.10/site-packages'
]
for p in VENV_SITE_PACKAGES:
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

import time
import cv2
import numpy as np

# YOLOv8 로드
YOLO_AVAILABLE = False
YOLO_IMPORT_ERROR = ""
try:
    from ultralytics import YOLO
    import torch
    YOLO_AVAILABLE = True
except Exception as err:
    YOLO_AVAILABLE = False
    YOLO_IMPORT_ERROR = str(err)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


def find_model_file(custom_path: str = "") -> str:
    """ 워크스페이스 내 yolov8n.pt 모델 가중치 파일 경로를 안전하게 탐색 """
    candidate_paths = [
        custom_path,
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "yolov8n.pt"),
        "/home/xytron/2026_Xytron/real/ros2_ws/src/kobot_vehicle/kobot_vehicle/yolov8n.pt",
        "/home/xytron/2026_Xytron/real/ros2_ws/src/kobot_lane_drive/kobot_lane_drive/yolov8n.pt",
        os.path.expanduser("~/2026_Xytron/real/ros2_ws/src/kobot_vehicle/kobot_vehicle/yolov8n.pt"),
    ]
    for path in candidate_paths:
        if path and os.path.exists(path):
            return os.path.abspath(path)
    return "yolov8n.pt"


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("object_detection_node")
        self.bridge = CvBridge()

        # 파라미터 선언
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("model_path", "")
        self.declare_parameter("exclude_classes", [28])  # 기본 28번(suitcase) 제외
        self.declare_parameter("publish_debug_topic", True)
        self.declare_parameter("show_gui", True)

        self.conf_thres = float(self.get_parameter("conf_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        custom_model = str(self.get_parameter("model_path").value)
        self.exclude_classes = set(self.get_parameter("exclude_classes").value)
        self.publish_debug_topic = bool(self.get_parameter("publish_debug_topic").value)
        self.show_gui = bool(self.get_parameter("show_gui").value)

        # 모델 파일 경로 탐색
        self.model_path = find_model_file(custom_model)
        self.get_logger().info(f"YOLOv8 모델 경로: {self.model_path}")
        self.get_logger().info(f"제외 대상 Class ID 목록: {list(self.exclude_classes)}")

        # YOLO 모델 로드
        self.model = None
        if not YOLO_AVAILABLE:
            self.get_logger().error(f"YOLO 라이브러리 로드 실패: {YOLO_IMPORT_ERROR}")
        else:
            try:
                device = "cuda:0" if torch.cuda.is_available() else "cpu"
                self.get_logger().info(f"YOLOv8 모델 로드 중 (Device: {device}) ...")
                self.model = YOLO(self.model_path)
                self.device = device
                self.get_logger().info("YOLOv8 모델이 성공적으로 로드되었습니다.")
            except Exception as e:
                self.get_logger().error(f"모델 초기화 오류: {e}")

        # 클래스별 고유 BGR 색상 팔레트 생성 (COCO 80개 클래스)
        np.random.seed(42)
        self.colors = np.random.randint(50, 255, size=(100, 3), dtype=np.uint8).tolist()

        # 카메라 토픽 구독 설정 (최신 프레임 1개 유지)
        qos_cam = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.camera_sub = self.create_subscription(
            CompressedImage,
            "/image_raw/compressed",
            self.image_callback,
            qos_cam
        )

        # 디버그 이미지 토픽 발행자
        if self.publish_debug_topic:
            self.debug_pub = self.create_publisher(Image, "/yolo/debug_image", 10)

        self.fps = 0.0
        self.get_logger().info("Object Detection Node 초기화 완료. 카메라 프레임(/image_raw/compressed) 대기 중...")

    def image_callback(self, msg: CompressedImage):
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Compressed Image 디코딩 오류: {e}")
            return

        start_time = time.time()
        out_img = cv_img.copy()
        img_h, img_w = cv_img.shape[:2]

        detections = []

        if self.model is not None:
            try:
                results = self.model(
                    cv_img,
                    imgsz=self.imgsz,
                    conf=self.conf_thres,
                    device=self.device,
                    verbose=False
                )

                if len(results) > 0 and results[0].boxes is not None:
                    boxes = results[0].boxes
                    names = results[0].names

                    for box in boxes:
                        cls_id = int(box.cls[0].item())
                        
                        # [제외 필터링] 28번 등 제외 목록에 포함된 Class ID는 무시
                        if cls_id in self.exclude_classes:
                            continue

                        conf = float(box.conf[0].item())
                        cls_name = names.get(cls_id, f"cls_{cls_id}")
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                        detections.append({
                            "id": cls_id,
                            "name": cls_name,
                            "conf": conf,
                            "bbox": (x1, y1, x2, y2)
                        })

                        # BBox 렌더링
                        color = self.colors[cls_id % len(self.colors)]
                        cv2.rectangle(out_img, (x1, y1), (x2, y2), color, 2)

                        # 라벨 포맷: [Class ID] Class Name | Conf%
                        label_text = f"[{cls_id}] {cls_name} {conf * 100:.1f}%"

                        # 텍스트 배경 박스 (가독성 향상)
                        (t_w, t_h), t_baseline = cv2.getTextSize(
                            label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1
                        )
                        label_y1 = max(t_h + 6, y1)
                        cv2.rectangle(
                            out_img,
                            (x1, label_y1 - t_h - 6),
                            (x1 + t_w + 6, label_y1 + t_baseline),
                            color,
                            -1
                        )
                        cv2.putText(
                            out_img,
                            label_text,
                            (x1 + 3, label_y1 - 2),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (0, 0, 0),
                            1,
                            cv2.LINE_AA
                        )

            except Exception as e:
                self.get_logger().error(f"추론 중 예외 발생: {e}")

        # FPS 계산
        elapsed = time.time() - start_time
        if elapsed > 0:
            current_fps = 1.0 / elapsed
            self.fps = 0.9 * self.fps + 0.1 * current_fps if self.fps > 0 else current_fps

        # 상단 상태 오버레이 바 (제외된 클래스 ID 표시 포함)
        exclude_str = ",".join(map(str, sorted(self.exclude_classes))) if self.exclude_classes else "None"
        overlay_text = f"FPS: {self.fps:.1f} | Detections: {len(detections)} | Conf: {self.conf_thres:.2f} | Excluded IDs: [{exclude_str}]"
        cv2.rectangle(out_img, (0, 0), (img_w, 35), (30, 30, 30), -1)
        cv2.putText(
            out_img,
            overlay_text,
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA
        )

        # 디버그 토픽 발행
        if self.publish_debug_topic:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(out_img, encoding="bgr8")
                self.debug_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f"디버그 토픽 발행 오류: {e}")

        # OpenCV GUI 창 렌더링
        if self.show_gui and os.environ.get("DISPLAY"):
            try:
                cv2.imshow("YOLO Object Detection Debug", out_img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info("'q' 키가 입력되어 종료합니다.")
                    rclpy.shutdown()
            except Exception as e:
                self.get_logger().warn(f"GUI 표시 오류: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    main()
