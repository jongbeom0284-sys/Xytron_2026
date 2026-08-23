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
# 0823: 0.15 → 0.12 (한계 거리 검출을 살리려고). 그러나 기록주행 3세션
# 1218프레임 재분석에서 그 임계가 **오검출 통로**였다: 검출 639개 중
# car 는 123개뿐이고 나머지는 대부분 suitcase 오검출이었다.
# 0823 저녁: 0.12 → 0.50 → **0.30**. 0.50 은 오검출을 잘 잘랐지만 진짜
# 앞차도 너무 놓쳤다 (실주행 체감). 같은 기록주행 1218프레임 기준:
#     전체클래스 0.12  검출 639개, 회피 성립 가능 구간 52개  (오검출 범람)
#     car만      0.50  검출  41개, 회피 성립 가능 구간  5개  (너무 적음)
#     car만      0.30  검출  73개, 회피 성립 가능 구간 11개
# 0823 밤: 0.30 → **0.13**. 실제 코스에 놓인 빨간 라바콘/정지차량을 찍어
# 돌려 보니 그 물체를 suitcase 0.177 / handbag 0.148 로만 잡는다 —
# 0.30 은 물론 0.20 으로도 못 넘는다 (같은 화면에서 배경 의자는 chair 0.44).
# 그래서 임계를 그 아래로 내린다. 대신 '좌·우 흰선 사이' 조건과
# '콘 탈출 후 vehicle_window_sec 동안만' 조건이 오검출 노출을 막는다.
# ⚠ 되돌리려면 이 값을 0.12 로. 주행 중에는 파라미터로 바꿀 수 있다:
#     ros2 param set /vehicle_perception_node conf 0.30
# 0824: 0.13 -> 0.20. 0.13 은 클래스를 넷으로 넓혔을 때 넓게 잡으려고
# 내린 값이다. 이제 car 하나만 보므로 임계를 올려 오검출을 줄인다.
CONF_THRES = 0.20
# 320 은 RC카가 47x40px 로 뭉개져 conf 0.14, 640 이면 0.34 였다 (0822).
# 0823: 960 을 시도했다가 되돌렸다. 혼자 재면 48ms 인데 실제 스택에서는
# 차선(YOLOPv2)·신호등과 GPU 를 다투며 89ms 가 됐고, 앞차 발행이
# 26.4Hz → 2.4Hz, 차선까지 20Hz → 4.3Hz 로 무너졌다 (부하평균 9.53/8코어).
# 멀리 보려다 파이프라인 전체를 망가뜨렸다 — 회피 확정도 0.13초 → 2초.
# 검출 거리는 해상도 대신 임계로 넓힌다 (아래 CONF_THRES).
IMGSZ = 640

CLASS_PERSON = 0          # COCO person
CLASS_CAR = 2             # COCO car
CLASS_BUS = 5             # COCO 'bus'
CLASS_TRUCK = 7           # COCO 'truck'
CLASS_SUITCASE = 28       # COCO 'suitcase'
CLASS_HANDBAG = 26        # COCO 'handbag'
CLASS_VASE = 75           # COCO 'vase'
# 트랙에 보행자가 없다. 사람 오검출로 비상정지가 걸리는 위험만 남아서 뺐다.
#
# RC카는 낮고 각져서 같은 물체가 프레임마다 car / truck / suitcase 로 흔들린다.
# 실측(같은 차, imgsz640): car 0.340 / suitcase 0.245 인 프레임이 있고
# suitcase 0.519 가 1등이라 car 가 밀리는 프레임도 있다.
# 클래스를 넓혀도 추론 시간은 그대로다. 모델은 어차피 80개를 전부 계산하고
# 필터는 결과 목록을 거르는 if 한 줄이다. 트랙에 여행가방도 트럭도 없다.
# 0823 저녁: car 만 본다. 클래스를 넓혔던 근거(같은 차가 프레임마다 car/
# truck/suitcase 로 흔들린다)는 conf 0.12 에서 관찰한 것인데, 그 낮은
# 임계에서는 트랙 190개 중 169개가 suitcase 로 잡히는 오검출이었다.
# conf 0.50 에서는 car 로도 충분히 잡힌다 (실측: car 검출 123개 중 43개가
# 0.5 이상, 최장 16프레임 연속). 되돌리려면 아래 줄을 주석 해제한다.
# TARGET_CLASSES = [CLASS_CAR, CLASS_BUS, CLASS_TRUCK, CLASS_SUITCASE]
# 0823 밤: RC카는 낮고 각져서 프레임마다 car / suitcase / handbag / vase 로
# 흔들린다. 클래스를 좁히면 그 흔들림만큼 놓친다. 그래서 **클래스를 넓히고,
# 대신 "좌·우 흰선 사이에 있는 물체" 만 앞차로 인정한다** (yolop_drive 가
# 판정한다 — 거기에만 차선 마스크가 있다). 차로 밖 물체는 애초에 피할 대상이
# 아니므로, 클래스를 넓혀 생기는 오검출을 이 기하 조건이 걸러낸다.
# 클래스를 넓혀도 추론 시간은 그대로다 (모델은 80개를 전부 계산한다).
# 0824: car 만 남긴다. suitcase/handbag/vase 는 라바콘·구조물을 물어
# 엉뚱한 회피를 만들었다 (01:33 의 11초짜리 좌측 오검출).
TARGET_CLASSES = [CLASS_CAR]
PERSON_STOP_DIST = 2.0    # 보행자가 이 거리 안이면 정지 신호

# bbox 하단이 이 y 위면 머리 위 구조물로 보고 버린다
OVERHEAD_REJECT_Y = int(IMAGE_HEIGHT * ROI_TOP_RATIO)

CAMERA_H_FOV_DEG = 68.0
LIDAR_VALID_MIN_DIST = 0.15
LIDAR_VALID_MAX_DIST = 4.0
LIDAR_ANGLE_OFFSET_RAD = -2.1217   # 라이다 장착각 보정
LIDAR_MAX_AGE_SEC = 0.5

CLASS_KO = {CLASS_PERSON: '사람', CLASS_CAR: '차량', CLASS_BUS: '차량',
            CLASS_TRUCK: '차량', CLASS_SUITCASE: '차량',
            CLASS_HANDBAG: '차량', CLASS_VASE: '차량'}
# 대시보드가 '무엇으로 잡았는지' 를 보여줄 수 있게 원래 이름도 싣는다.
# ko 는 전부 '차량' 이라 클래스가 흔들리는 것을 볼 수 없다 (0823 밤).
CLASS_NAME = {CLASS_PERSON: 'person', CLASS_CAR: 'car', CLASS_BUS: 'bus',
              CLASS_TRUCK: 'truck', CLASS_SUITCASE: 'suitcase',
              CLASS_HANDBAG: 'handbag', CLASS_VASE: 'vase'}

CLASS_BGR = {CLASS_PERSON: (60, 60, 255), CLASS_CAR: (255, 200, 60),
             CLASS_BUS: (255, 200, 60), CLASS_TRUCK: (255, 200, 60),
             CLASS_SUITCASE: (255, 200, 60), CLASS_HANDBAG: (255, 200, 60),
             CLASS_VASE: (255, 200, 60)}


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
        # 주행 노드가 '지금 차량 인식이 필요한가' 를 status 로 알려준다
        # (vehicle_needed). 콘 구간이면 쉬고, 벗어나면 즉시 다시 본다.
        # ★ 2026-08-23: 이 구독이 on_compressed() 본문 끝에 들여쓰기 실수로
        #   들어가 있어 **매 프레임(30Hz)마다 구독이 하나씩 늘었다**. 84분
        #   뒤 632개 — status 한 건에 콜백 632회, CPU 233%, 앞차 발행
        #   30Hz → 0.5Hz, 발행측(yolop_drive)도 구독자 634개를 상대하느라
        #   지연. "점점 느려지고 구독이 안 지워지는" 증상의 원인이었다.
        #   구독은 __init__ 에서 한 번만 만든다 (test_node_structure.py 가 지킨다).
        self.create_subscription(
            String, '/kobot_yolop/status', self._on_drive_status, 10)

        # 임계를 주행 중에 바꿀 수 있게 한다. 이 값은 트랙·조명마다 다시
        # 맞춰야 하는데, 그때마다 빌드하고 스택을 내리면 시험이 느려진다.
        #     ros2 param set /vehicle_perception_node conf 0.35
        self.add_on_set_parameters_callback(self._on_params)

        self.model_path = str(self.get_parameter('model_path').value)
        self.model = YOLO(self.model_path)
        self.get_logger().info(
            '차량 인식 시작 · YOLOv8n imgsz%d conf%.2f · %d프레임당 1회 · %s'
            % (IMGSZ, self.conf, self.interval, self.model_path))

    # ---------------------------------------------------------------- 입력
    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'conf':
                v = float(p.value)
                if not 0.0 < v <= 1.0:
                    return SetParametersResult(
                        successful=False, reason='conf 는 0~1 범위여야 한다')
                self.conf = v
                self.get_logger().warning('검출 임계 conf = %.2f 로 변경' % v)
            elif p.name == 'infer_interval':
                v = int(p.value)
                if v < 1:
                    return SetParametersResult(
                        successful=False, reason='infer_interval 은 1 이상')
                self.interval = v
                self.get_logger().warning('추론 주기 %d프레임당 1회로 변경' % v)
        return SetParametersResult(successful=True)

    def on_scan(self, msg):
        self.scan = msg
        self.scan_at = time.time()

    def on_compressed(self, msg):
        # 주행 노드가 '차량 인식 불필요'(콘 구간) 라고 하면 쉰다.
        # CPU 를 조향에 넘긴다. (0822: 가드가 on_raw 에만 있어 압축 전환 후
        # 무력화됐던 것을 바로잡음)
        if self._paused:
            return

        # 프레임 번호는 여기서 센다. 추론은 interval 프레임마다 한 번이고,
        # 디버그 영상을 안 만들 때는 나머지 프레임에서 JPEG 를 풀 이유가 없다
        # — 직전 결과를 그대로 다시 낸다 (발행 주기 30Hz 는 그대로 유지.
        # yolop_drive 의 CAR_CONFIRM 연속 장수 계산이 이 주기에 맞춰져 있다).
        # 2026-08-23: 30회/초 중 20회의 디코드(~3ms)를 없앤다.
        self.frames += 1
        need_detect = (self.frames % self.interval == 0)
        if not need_detect and self.pub_debug is None:
            self.publish(None)
            return

        try:
            image = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                                 cv2.IMREAD_COLOR)
        except Exception as err:  # noqa: BLE001
            self.get_logger().warn('압축 영상 디코드 실패: %s' % err)
            return
        if image is not None:
            self.process(image, need_detect)

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
        """원본 영상 경로. 지금은 구독하지 않는다 (압축만 쓴다) — 도구용으로 남긴다."""
        if self._paused:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as err:  # noqa: BLE001
            self.get_logger().warn('원본 영상 변환 실패: %s' % err)
            return
        self.frames += 1
        self.process(image, self.frames % self.interval == 0)

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
    def process(self, image, need_detect=True):
        if image.shape[1] != IMAGE_WIDTH or image.shape[0] != IMAGE_HEIGHT:
            image = cv2.resize(image, (IMAGE_WIDTH, IMAGE_HEIGHT))

        if need_detect:
            self.detect(image)
        self.publish(image)

    def detect(self, image):
        started = time.perf_counter()
        boxes = []
        car_dist, car_dir = float('inf'), None
        # 승격 순위: (거리를 쟀는가, 거리 또는 -넓이). 작을수록 먼저.
        # 거리를 못 재도 버리지 않는다 — 좌/우만 나오면 차로 인정한다.
        car_rank = None
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
                cls=cls, ko=CLASS_KO.get(cls, str(cls)),
                cn=CLASS_NAME.get(cls, str(cls)), dir=side,
                dist=(None if math.isinf(dist) else round(dist, 3)),
                conf=round(conf, 3), px=[x1, y1, x2, y2]))
            if cls == CLASS_PERSON:
                if dist <= PERSON_STOP_DIST:
                    person = True
                    person_dist = min(person_dist, dist)
            else:   # car / bus / truck / suitcase 전부 차량으로 본다
                area = (x2 - x1) * (y2 - y1)
                rank = ((0, dist) if math.isfinite(dist) else (1, -area))
                if car_rank is None or rank < car_rank:
                    car_rank, car_dist, car_dir = rank, dist, side

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

        if self.pub_debug is not None and image is not None:
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
            tag = '%s %.2f %s %s' % (
                box.get('cn', box['ko']), box.get('conf', 0.0), box['dir'],
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
