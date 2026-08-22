#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""차량·보행자 인식만 하는 노드. 주행 판단은 하지 않는다.

Lane_follow.py 에 차선 추종과 섞여 있던 YOLO + LiDAR 퓨전 부분만 떼어냈다.
계산식과 보정값은 원본 그대로다. 바꾼 것은 아래 셋뿐이다.
  - 차선 추종, 조향, 모터 발행을 전부 뺐다
  - 판정 결과를 JSON 한 토픽으로 모아 발행한다 (신호등 노드와 같은 방식)
  - 원본이 쓰던 /yolo/obstacle_dist, /yolo/obstacle_dir 도 그대로 발행한다

이 노드는 "무엇이 어디에 얼마나 가까이 있는가" 만 말한다.
그걸로 무엇을 할지는 차선 노드가 정한다.

토픽
  구독  /image_raw/compressed  또는 /image_raw
        /scan                  LaserScan
  발행  /vehicle/state         std_msgs/String   JSON. 아래 참고
        /yolo/obstacle_dist    std_msgs/Float32  원본 호환
        /yolo/obstacle_dir     std_msgs/String   원본 호환
        /vehicle/debug_img     sensor_msgs/Image 원본 + 박스
"""

import json
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import Float32, String

try:
    from ultralytics import YOLO
except ImportError:  # 노드 python 에는 없다. 런치가 PYTHONPATH 로 넣어준다.
    VENV = '/home/xytron/clrernet_eval/venv/lib/python3.10/site-packages'
    if os.path.isdir(VENV) and VENV not in sys.path:
        sys.path.insert(0, VENV)
    from ultralytics import YOLO

TOPIC_IMAGE_COMPRESSED = '/image_raw/compressed'
TOPIC_IMAGE_RAW = '/image_raw'
TOPIC_SCAN = '/scan'
TOPIC_STATE = '/vehicle/state'
TOPIC_DIST = '/yolo/obstacle_dist'
TOPIC_DIR = '/yolo/obstacle_dir'
TOPIC_DEBUG = '/vehicle/debug_img'

# ---- 아래 값은 전부 Lane_follow.py 원본 그대로다 ----
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
ROI_TOP_RATIO = 0.5

MODEL_PATH = '/home/xytron/2026_Xytron/models/yolov8n.pt'  # 모델은 전부 models/ 로 통일 (2026-08-22)
INFER_INTERVAL = 3        # N프레임당 1회 추론
CONF_THRES = 0.15  # 경계선 검출을 살린다
IMGSZ = 640        # 320 은 RC카가 47x40px 로 뭉개져 conf 0.14. 640 이면 0.34

CLASS_PERSON = 0          # COCO person
CLASS_CAR = 2             # COCO car
CLASS_BUS = 5             # COCO 'bus'
CLASS_TRUCK = 7           # COCO 'truck'
CLASS_SUITCASE = 28       # COCO 'suitcase'
# 트랙에 보행자가 없다. 사람 오검출로 비상정지가 걸리는 위험만 남아서 뺐다.
#
# RC카는 낮고 각져서 같은 물체가 프레임마다 car / truck / suitcase 로 흔들린다.
# 실측(같은 차, imgsz640): car 0.340 / suitcase 0.245 인 프레임이 있고
# suitcase 0.519 가 1등이라 car 가 밀리는 프레임도 있다.
# 클래스를 넓혀도 추론 시간은 그대로다. 모델은 어차피 80개를 전부 계산하고
# 필터는 결과 목록을 거르는 if 한 줄이다. 트랙에 여행가방도 트럭도 없다.
TARGET_CLASSES = [CLASS_CAR, CLASS_BUS, CLASS_TRUCK, CLASS_SUITCASE]
PERSON_STOP_DIST = 2.0    # 보행자가 이 거리 안이면 정지 신호

# bbox 하단이 이 y 위면 머리 위 구조물로 보고 버린다
OVERHEAD_REJECT_Y = int(IMAGE_HEIGHT * ROI_TOP_RATIO)

CAMERA_H_FOV_DEG = 68.0
LIDAR_VALID_MIN_DIST = 0.15
LIDAR_VALID_MAX_DIST = 4.0
LIDAR_ANGLE_OFFSET_RAD = -2.1217   # 라이다 장착각 보정
LIDAR_MAX_AGE_SEC = 0.5

CLASS_KO = {CLASS_PERSON: '사람', CLASS_CAR: '차량', CLASS_BUS: '차량',
            CLASS_TRUCK: '차량', CLASS_SUITCASE: '차량'}
CLASS_BGR = {CLASS_PERSON: (60, 60, 255), CLASS_CAR: (255, 200, 60),
             CLASS_BUS: (255, 200, 60), CLASS_TRUCK: (255, 200, 60),
             CLASS_SUITCASE: (255, 200, 60)}


class VehiclePerception(Node):
    def __init__(self):
        super().__init__('vehicle_perception_node')
        self.bridge = CvBridge()
        self.declare_parameter('debug_image', True)
        self.declare_parameter('infer_interval', INFER_INTERVAL)
        self.declare_parameter('conf', CONF_THRES)
        self.declare_parameter('model_path', MODEL_PATH)
        self.publish_debug = bool(self.get_parameter('debug_image').value)
        self.interval = max(1, int(self.get_parameter('infer_interval').value))
        self.conf = float(self.get_parameter('conf').value)

        self.scan = None
        self.scan_at = 0.0
        self.frames = 0
        self.infers = 0
        self.last = dict(dir='NONE', dist=float('inf'), person=False,
                         person_dist=float('inf'), boxes=[], ms=0.0)

        self.pub_state = self.create_publisher(String, TOPIC_STATE, 10)
        self.pub_dist = self.create_publisher(Float32, TOPIC_DIST, 10)
        self.pub_dir = self.create_publisher(String, TOPIC_DIR, 10)
        self.pub_debug = (self.create_publisher(Image, TOPIC_DEBUG, 1)
                          if self.publish_debug else None)

        self._paused = False
        # 압축 스트림 구독 제거 (2026-08-22): 원본과 중복 디코딩으로
        # 초당 ~50회 디코드가 CPU 를 포화시켜 주행 판단이 4Hz 로 떨어졌다.
        # 2026-08-22: 원본(900KB/장) 구독은 대역폭에 질식해 7.9Hz 까지
        # 떨어졌다. 압축(79KB, 29.9Hz)만 구독하고 JPEG 디코드는 감수한다.
        self.create_subscription(
            CompressedImage, TOPIC_IMAGE_COMPRESSED,
            self.on_compressed, qos_profile_sensor_data)
        self.create_subscription(LaserScan, TOPIC_SCAN,
                                 self.on_scan, qos_profile_sensor_data)

        self.model_path = str(self.get_parameter('model_path').value)
        self.model = YOLO(self.model_path)
        self.get_logger().info(
            '차량 인식 시작 · YOLOv8n imgsz%d conf%.2f · %d프레임당 1회 · %s'
            % (IMGSZ, self.conf, self.interval, self.model_path))

    # ---------------------------------------------------------------- 입력
    def on_scan(self, msg):
        self.scan = msg
        self.scan_at = time.time()

    def on_compressed(self, msg):
        # 신호 잠금(직진 30s / 좌회전 20s) 동안은 판정이 확정돼 있어
        # 인식이 무의미하다. CPU 를 조향에 넘긴다. (0822: 가드가
        # on_raw 에만 있어 압축 전환 후 무력화됐던 것을 바로잡음)
        if self._paused:
            return

        try:
            image = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                                 cv2.IMREAD_COLOR)
        except Exception as err:  # noqa: BLE001
            self.get_logger().warn('압축 영상 디코드 실패: %s' % err)
            return
        if image is not None:
            self.process(image)

        # 신호 잠금(직진 30s / 좌회전 20s) 동안은 판정이 확정돼 있어
        # 인식이 무의미하다. 그 사이 CPU 를 조향 파이프라인에 넘긴다.
        # (2026-08-22 실측: 주행 판단이 4.4Hz 까지 떨어져 커브 이탈)
        self.create_subscription(
            String, '/kobot_yolop/status', self._on_drive_status, 10)

    def _on_drive_status(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, AttributeError, TypeError):
            return
        # 주행 노드가 '지금 차량 인식이 필요한가' 를 직접 알려준다.
        # 좌회전 잠금·콘 구간이면 False, 콘 구간을 벗어나면 즉시 True,
        # 회피 유지 중이면 잠금 중이라도 True (2026-08-22).
        if 'vehicle_needed' in data:
            self._paused = not bool(data['vehicle_needed'])
            return
        lock = float(data.get('signal_lock_s', 0.0))
        self._paused = lock > 0.0

    def on_raw(self, msg):
        if self._paused:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as err:  # noqa: BLE001
            self.get_logger().warn('원본 영상 변환 실패: %s' % err)
            return
        self.process(image)

    # ------------------------------------------------------- 라이다 퓨전
    def distance_from_bbox(self, x1, x2, width):
        """bbox 가로 범위를 라이다 각도로 옮겨 최단 거리를 뽑는다.

        Lane_follow.py 원본 계산 그대로다. 화각 68도, 장착각 보정 -2.1217rad,
        노이즈를 피하려고 유효 거리의 10퍼센타일을 쓴다.
        """
        if self.scan is None:
            return float('inf')
        if (time.time() - self.scan_at) > LIDAR_MAX_AGE_SEC:
            return float('inf')

        scan = self.scan
        ranges = np.array(scan.ranges)
        ranges[np.isnan(ranges)] = 10.0
        ranges[np.isinf(ranges)] = 10.0

        fov = math.radians(CAMERA_H_FOV_DEG)
        a0 = fov * (0.5 - float(x2) / width) - LIDAR_ANGLE_OFFSET_RAD
        a1 = fov * (0.5 - float(x1) / width) - LIDAR_ANGLE_OFFSET_RAD
        a0 = math.atan2(math.sin(a0), math.cos(a0))
        a1 = math.atan2(math.sin(a1), math.cos(a1))

        i0 = int((a0 - scan.angle_min) / scan.angle_increment)
        i1 = int((a1 - scan.angle_min) / scan.angle_increment)
        n = len(ranges)
        i0 %= n
        i1 %= n
        window = (np.concatenate((ranges[i0:], ranges[:i1 + 1]))
                  if i0 > i1 else ranges[i0:i1 + 1])
        good = window[(window >= LIDAR_VALID_MIN_DIST)
                      & (window <= LIDAR_VALID_MAX_DIST)]
        if good.size == 0:
            return float('inf')
        return float(np.percentile(good, 10))

    # ---------------------------------------------------------------- 처리
    def process(self, image):
        self.frames += 1
        if image.shape[1] != IMAGE_WIDTH or image.shape[0] != IMAGE_HEIGHT:
            image = cv2.resize(image, (IMAGE_WIDTH, IMAGE_HEIGHT))

        if self.frames % self.interval == 0:
            self.detect(image)
        self.publish(image)

    def detect(self, image):
        started = time.perf_counter()
        boxes = []
        car_dist, car_dir = float('inf'), None
        person, person_dist = False, float('inf')
        try:
            result = self.model(image, imgsz=IMGSZ, conf=self.conf,
                                device='cuda:0', verbose=False)[0]
        except Exception as err:  # noqa: BLE001
            self.get_logger().error('YOLO 추론 실패: %s' % err)
            return

        for box in (result.boxes or []):
            cls = int(box.cls[0].item())
            if cls not in TARGET_CLASSES:
                continue
            conf = float(box.conf[0].item())
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            # 머리 위 구조물 배제
            if y2 <= OVERHEAD_REJECT_Y:
                continue
            dist = self.distance_from_bbox(x1, x2, IMAGE_WIDTH)
            cx = (x1 + x2) / 2.0
            side = 'LEFT' if cx < (IMAGE_WIDTH / 2.0) else 'RIGHT'
            boxes.append(dict(
                x=round(x1 / IMAGE_WIDTH, 4), y=round(y1 / IMAGE_HEIGHT, 4),
                w=round((x2 - x1) / IMAGE_WIDTH, 4),
                h=round((y2 - y1) / IMAGE_HEIGHT, 4),
                cls=cls, ko=CLASS_KO.get(cls, str(cls)), dir=side,
                dist=(None if math.isinf(dist) else round(dist, 3)),
                conf=round(conf, 3), px=[x1, y1, x2, y2]))
            if cls == CLASS_PERSON:
                if dist <= PERSON_STOP_DIST:
                    person = True
                    person_dist = min(person_dist, dist)
            else:   # car / bus / truck / suitcase 전부 차량으로 본다
                if dist < car_dist:
                    car_dist, car_dir = dist, side

        self.infers += 1
        self.last = dict(
            dir=(car_dir or 'NONE'), dist=car_dist, person=person,
            person_dist=person_dist, boxes=boxes,
            ms=(time.perf_counter() - started) * 1000.0)

    def publish(self, image):
        last = self.last
        payload = dict(
            car=dict(dir=last['dir'],
                     dist=(None if math.isinf(last['dist'])
                           else round(last['dist'], 3))),
            person=dict(stop=last['person'],
                        dist=(None if math.isinf(last['person_dist'])
                              else round(last['person_dist'], 3))),
            boxes=last['boxes'], n=len(last['boxes']),
            ms=round(last['ms'], 1), frames=self.frames, infers=self.infers,
            w=IMAGE_WIDTH, h=IMAGE_HEIGHT,
            lidar=(self.scan is not None
                   and (time.time() - self.scan_at) <= LIDAR_MAX_AGE_SEC),
            stamp=time.time())
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.pub_state.publish(message)

        # 원본 Lane_follow.py 가 쓰던 토픽도 그대로 낸다
        if not math.isinf(last['dist']):
            self.pub_dist.publish(Float32(data=float(last['dist'])))
        direction = String()
        direction.data = last['dir']
        self.pub_dir.publish(direction)

        if self.pub_debug is not None:
            self.pub_debug.publish(
                self.bridge.cv2_to_imgmsg(self.render(image), 'bgr8'))

    def render(self, image):
        """원본 위에 박스만 덧그린다."""
        canvas = image.copy()
        cv2.line(canvas, (IMAGE_WIDTH // 2, 0),
                 (IMAGE_WIDTH // 2, IMAGE_HEIGHT), (90, 90, 90), 1)
        cv2.line(canvas, (0, OVERHEAD_REJECT_Y),
                 (IMAGE_WIDTH, OVERHEAD_REJECT_Y), (90, 90, 90), 1)
        for box in self.last['boxes']:
            x1, y1, x2, y2 = box['px']
            color = CLASS_BGR.get(box['cls'], (200, 200, 200))
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
            tag = '%s %s %s' % (
                box['ko'], box['dir'],
                ('%.2fm' % box['dist']) if box['dist'] is not None else '거리없음')
            for col, th in (((0, 0, 0), 3), (color, 1)):
                cv2.putText(canvas, tag, (x1, max(y1 - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, th, cv2.LINE_AA)
        head = 'car %s %s  person %s  %.0fms' % (
            self.last['dir'],
            ('%.2fm' % self.last['dist']) if not math.isinf(self.last['dist'])
            else '-',
            'STOP' if self.last['person'] else '-', self.last['ms'])
        for col, th in (((0, 0, 0), 3), ((255, 255, 255), 1)):
            cv2.putText(canvas, head, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, col, th, cv2.LINE_AA)
        return canvas


def main(args=None):
    rclpy.init(args=args)
    node = VehiclePerception()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
