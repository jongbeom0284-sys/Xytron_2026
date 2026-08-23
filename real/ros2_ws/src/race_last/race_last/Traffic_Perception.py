#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""신호등 상태 인식 — 정규화 색도 + 원형성.

원리
  LED 는 과노출 블룸으로 흰빛이 섞여 정규화 청색 성분이 남는다 (nb 0.21~0.36).
  주황 의자 같은 확산 반사면은 청색이 거의 없다 (nb 0.137). nb 로 갈린다.
  천장 형광등은 무채색이라 색 편차가 거의 0 이다. spread 로 갈린다.
  램프는 원이다. 가로세로비와 채움률은 무차원이라 거리가 변해도 그대로다.

  켜진 램프의 색과 배치만 본다. 신호등 함체 박스를 찾을 필요가 없다.
  초록불과 좌회전은 색이 완전히 같다 (Hue 76 대 82). 화살표가 초록원
  왼쪽에 하나 더 켜지는지로만 갈린다.

  실측 60장에서 점등 4상태 48/48. 매 프레임 단독 판정이며 시간축을 쌓지 않는다.

토픽
  구독  /image_raw/compressed  또는  /image_raw
  발행  /traffic_light/state      std_msgs/String  (JSON)
        /traffic_light/debug_img  sensor_msgs/Image  (원본 + 박스)
"""

import json
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

TOPIC_IMAGE_COMPRESSED = '/image_raw/compressed'
TOPIC_IMAGE_RAW = '/image_raw'
TOPIC_STATE = '/traffic_light/state'
TOPIC_DEBUG = '/traffic_light/debug_img'

# ---- 점등 판정 (전부 실측 기반, 좌표는 쓰지 않는다) ----
MIN_VALUE = 150      # 켜진 램프의 밝기 하한
MIN_BLUE = 0.175     # 정규화 청색. 주황 확산면(0.137)을 배제한다
MIN_SPREAD = 0.045   # 색 편차. 무채색 형광등을 배제한다
MIN_AREA = 6         # 램프 최소 화소. 640x480 기준 약 4배 거리까지
MAX_ASPECT = 1.8     # 램프는 원이다. 무차원
MIN_FILL = 0.33      # 면적 / 외접사각형. 무차원
CYAN_MARGIN = 0.05   # 청록 판정. ng - nr 이 이보다 커야 한다. 무차원
ARROW_PITCH = 3.6    # 램프 간격 / 램프 반지름. 실측. 거리와 무관
ARROW_VALUE = 140    # 화살표 자리 밝기 하한. 자리를 알고 보므로 낮춰도 된다
ARROW_RATIO = 0.10   # 화살표로 인정할 화소 수 / 초록원 면적

# ---- 탐색 영역 (비율. 해상도가 바뀌어도 그대로 쓴다) ----
# 신호등은 머리 위에 달려 있어 화면에서 지평선 아래로 내려오지 않는다.
# 라바콘·의자는 바닥에 있어 항상 지평선 아래다. 그 경계로 자른다.
# 찍어둔 66장 실측: 램프가 x 0.470~0.583, y 0.325~0.404 안에만 나타났다.
# 아래 값은 그 범위에 여유를 준 것이다.
ROI_TOP = 0.373      # 179px. 위쪽 천장 형광등 배제
ROI_BOTTOM = 0.444   # 213px. 아래쪽 바닥(라바콘·의자) 배제
ROI_LEFT = 0.302     # 193px
ROI_RIGHT = 0.528    # 338px

MAX_HZ = 20.0        # 초당 최대 처리 횟수
CONFIRM = 3          # 안정화 상태로 바꾸기 위한 연속 동일 판정 수

STATE_KO = {'red': '빨간불', 'yellow': '노란불', 'green': '초록불',
            'left': '좌회전', 'none': '없음'}
DRAW_BGR = {'red': (60, 60, 255), 'amber': (0, 200, 255),
            'cyan': (200, 255, 60)}


def chroma_planes(image):
    """정규화 색도와 밝기. LED 블룸은 청색이 남고 확산 반사면은 안 남는다."""
    soft = cv2.GaussianBlur(image, (3, 3), 0).astype(np.float32)
    total = soft.sum(axis=2) + 1e-6
    nb, ng, nr = (soft[:, :, i] / total for i in range(3))
    return nr, ng, nb, soft.max(axis=2)


def classify(nr, ng, nb):
    """램프 하나의 색 계열. 실측 중앙값 기준."""
    if nb <= MIN_BLUE:
        return None
    # 청록(초록불·좌회전화살표)은 녹이 적을 확실히 넘는다. 거리가 멀어 화살표가
    # 흐려져도 이 차이는 남는다. 절대 문턱값을 쓰면 먼 화살표를 놓친다.
    if (ng - nr) > CYAN_MARGIN and ng > 0.36:
        return 'cyan'
    if nr > 0.385 and ng >= 0.355:
        return 'amber'
    if nr > 0.36 and ng < 0.355:
        return 'red'
    return None


_roi = {}

# 탐색 띠 둘레의 여유 [px]. 띠 밖으로 나가 보는 계산은 둘뿐이다:
#   · 3x3 가우시안 블러 — 이웃 1px
#   · arrow_beside — 초록원 왼쪽으로 (ARROW_PITCH + 1.6)·반지름 = 5.2·r,
#     위아래로 1.6·r. 램프 면적이 1000px(r≈18)이어도 가로 94px / 세로 29px.
# 그래서 이 여유 안에서는 전체 프레임으로 계산한 것과 결과가 같다.
# (기본 띠 0.28×0.06 + 여유 = 371×93px ≈ 전체 640×480 의 1/9)
CROP_PAD_X = 96
CROP_PAD_Y = 48


def roi_box(shape):
    height, width = shape[:2]
    return (int(ROI_LEFT * width), int(ROI_TOP * height),
            int(ROI_RIGHT * width), int(ROI_BOTTOM * height))


def crop_box(shape):
    """계산에 쓸 잘라낸 영역 (x0, y0, x1, y1). 띠 + 여유, 화면 안으로 자른다."""
    height, width = shape[:2]
    x0, y0, x1, y1 = roi_box(shape)
    return (max(0, x0 - CROP_PAD_X), max(0, y0 - CROP_PAD_Y),
            min(width, x1 + CROP_PAD_X), min(height, y1 + CROP_PAD_Y))


def roi_mask(shape):
    """잘라낸 영역(crop_box) 크기의 마스크. 탐색 띠 안만 True."""
    cx0, cy0, cx1, cy1 = crop_box(shape)
    key = (shape[0], shape[1], ROI_TOP, ROI_BOTTOM, ROI_LEFT, ROI_RIGHT)
    if key not in _roi:
        mask = np.zeros((cy1 - cy0, cx1 - cx0), dtype=bool)
        x0, y0, x1, y1 = roi_box(shape)
        mask[y0 - cy0:y1 - cy0, x0 - cx0:x1 - cx0] = True
        _roi.clear()
        _roi[key] = mask
    return _roi[key]


def arrow_beside(planes, main):
    """초록원 왼쪽 화살표 자리를 직접 본다.

    화살표는 원이 아니다. 원형성 조건을 걸면 거리가 멀어질수록 놓친다.
    대신 위치로 찾는다. 램프 간격은 반지름의 3.6배(실측)이므로 초록원의
    면적에서 역산한다. 좌표를 알려주는 것이 아니라 검출된 램프에서 잰다.

    planes 는 잘라낸 영역 좌표계이고 (ox, oy) 가 그 원점이다. 램프 좌표는
    원본 좌표계이므로 여기서 옮겨 보고, 돌려주는 상자는 다시 원본 좌표다.
    """
    nr, ng, nb, val, ox, oy = planes
    height, width = val.shape
    radius = max(1.0, (main['area'] / 3.14159) ** 0.5)
    cx = main['x'] - ox - ARROW_PITCH * radius
    cy = main['y'] - oy
    half = 1.6 * radius
    x0, x1 = int(round(cx - half)), int(round(cx + half)) + 1
    y0, y1 = int(round(cy - half)), int(round(cy + half)) + 1
    x0, y0 = max(x0, 0), max(y0, 0)
    x1, y1 = min(x1, width), min(y1, height)
    if x1 - x0 < 2 or y1 - y0 < 2:
        return 0, None
    sub = (slice(y0, y1), slice(x0, x1))
    hit = ((val[sub] > ARROW_VALUE)
           & ((ng[sub] - nr[sub]) > CYAN_MARGIN)
           & (nb[sub] > MIN_BLUE))
    return int(hit.sum()), (x0 + ox, y0 + oy, x1 - x0, y1 - y0)


def find_lamps(image):
    """켜진 램프 덩어리를 찾는다. 램프 좌표는 원본 프레임 기준이다.

    2026-08-23: 탐색 띠는 화면의 약 2% (0.28×0.06) 인데 정규화 색도를 전체
    640×480 에서 float32 로 계산하고 있었다 (노드 CPU 73%). 띠 + 여유만
    잘라 계산한다. 마스크는 여전히 띠 안만 켜므로 검출 결과는 같고, 외부
    도구(signal_grid/build_combined/overlay_frames)가 쓰는 반환 형식도
    그대로다: 램프는 원본 좌표, planes 는 decide() 에만 넘기는 불투명 값.
    """
    cx0, cy0, cx1, cy1 = crop_box(image.shape)
    crop = image[cy0:cy1, cx0:cx1]
    nr, ng, nb, val = chroma_planes(crop)
    spread = (np.maximum(np.maximum(nr, ng), nb)
              - np.minimum(np.minimum(nr, ng), nb))
    lit = (val > MIN_VALUE) & (nb > MIN_BLUE) & (spread > MIN_SPREAD)
    # 띠 밖은 마스크로 끈다 (여유 영역은 계산에만 쓰고 검출하지 않는다).
    lit &= roi_mask(image.shape)
    count, _, stats, cent = cv2.connectedComponentsWithStats(
        lit.astype(np.uint8), 8)
    lamps = []
    for i in range(1, count):
        x, y, w, h, area = stats[i]
        if area < MIN_AREA:
            continue
        aspect = max(w, h) / float(max(min(w, h), 1))
        fill = area / float(max(w * h, 1))
        if aspect > MAX_ASPECT or fill < MIN_FILL:
            continue
        sub = (slice(y, y + h), slice(x, x + w))
        mask = lit[sub]
        if not mask.any():
            continue
        vr = float(np.median(nr[sub][mask]))
        vg = float(np.median(ng[sub][mask]))
        vb = float(np.median(nb[sub][mask]))
        kind = classify(vr, vg, vb)
        if kind is None:
            continue
        lamps.append(dict(x=float(cent[i][0]) + cx0, y=float(cent[i][1]) + cy0,
                          rect=(int(x) + cx0, int(y) + cy0, int(w), int(h)),
                          area=int(area), c=kind, nb=vb, ng=vg, nr=vr,
                          aspect=aspect, fill=fill))
    return lamps, (nr, ng, nb, val, cx0, cy0)


def decide(lamps, planes=None):
    """켜진 램프의 색과 배치로 상태를 정한다.

    좌회전은 초록원 왼쪽에 화살표가 하나 더 켜진 경우다.
    화살표는 원이 아니라서 덩어리로 안 잡힐 때가 많다. 그래서 덩어리로
    못 찾으면 초록원에서 간격을 역산한 자리를 직접 본다.
    """
    cyan = sorted([b for b in lamps if b['c'] == 'cyan'], key=lambda b: -b['area'])
    if cyan:
        main = cyan[0]
        rad = max(1.0, (main['area'] / 3.14159) ** 0.5)
        mate = [b for b in cyan[1:]
                if abs(b['y'] - main['y']) < 2.0 * rad
                and 0 < (main['x'] - b['x']) < 8.0 * rad]
        if mate:
            return 'left', [main] + mate
        if planes is not None:
            hits, box = arrow_beside(planes, main)
            if box is not None and hits >= max(4, ARROW_RATIO * main['area']):
                spot = dict(x=box[0] + box[2] / 2.0, y=box[1] + box[3] / 2.0,
                            rect=box, area=hits, c='cyan', nb=0.0,
                            ng=0.0, nr=0.0, aspect=1.0, fill=1.0)
                return 'left', [main, spot]
        return 'green', [main]
    amber = [b for b in lamps if b['c'] == 'amber']
    if amber:
        return 'yellow', [max(amber, key=lambda b: b['area'])]
    red = [b for b in lamps if b['c'] == 'red']
    if red:
        return 'red', [max(red, key=lambda b: b['area'])]
    return 'none', []


class TrafficLightPerception(Node):
    def __init__(self):
        super().__init__('traffic_perception_node')
        self.bridge = CvBridge()
        self.declare_parameter('debug_image', True)
        self.declare_parameter('max_hz', MAX_HZ)
        # 탐색 띠(ROI)를 장소별로 즉시 조절할 수 있게 파라미터화 (2026-08-22).
        # 새 연습장에서 겐트리 높이가 달라 램프(y=0.35)가 띠(0.373~) 위로 벗어났다.
        # 띠는 주행 내내 고정이다 — 확장하면 비상구 유도등이 들어온다 (0823).
        self.declare_parameter('roi_top', ROI_TOP)
        self.declare_parameter('roi_bottom', ROI_BOTTOM)
        self.declare_parameter('roi_left', ROI_LEFT)
        self.declare_parameter('roi_right', ROI_RIGHT)
        self._apply_roi_params()
        self.add_on_set_parameters_callback(self._on_roi_params)
        self.publish_debug = bool(self.get_parameter('debug_image').value)
        self.max_hz = float(self.get_parameter('max_hz').value)

        self.pub_state = self.create_publisher(String, TOPIC_STATE, 10)
        self.pub_debug = (self.create_publisher(Image, TOPIC_DEBUG, 1)
                          if self.publish_debug else None)
        # 압축 스트림 구독 제거 (2026-08-22): 원본과 중복 디코딩으로
        # 초당 ~50회 디코드가 CPU 를 포화시켜 주행 판단이 4Hz 로 떨어졌다.
        # 2026-08-22: 원본(900KB/장) 구독은 대역폭에 질식해 7.9Hz 까지
        # 떨어졌다. 압축(79KB, 29.9Hz)만 구독하고 JPEG 디코드는 감수한다.
        self.create_subscription(
            CompressedImage, TOPIC_IMAGE_COMPRESSED,
            self.on_compressed, qos_profile_sensor_data)
        # ★ 신호 잠금 중 '인식 일시정지'(0822 perf 커밋)는 여기 없다 — 의도다.
        #   그 커밋의 status 구독은 구독되지 않는 on_raw() 본문에 들어가 있어
        #   한 번도 동작한 적이 없고, 0823 의 신호 로직(yolop_drive._on_traffic:
        #   확정한 신호가 **시야에서 사라진 시점**부터 조향억제·잠금을 센다)은
        #   확정 뒤에도 신호 판정이 계속 들어오는 것을 전제로 튜닝됐다. 지금
        #   멈추면 직진 억제가 '확정 +3초'에 끝나 삼거리 좌회전 차선으로 끌린다.
        #   CPU 는 대신 find_lamps 의 띠 크롭으로 줄였다 (73% → 한 자릿수).
        #   정말 쉬게 하려면 yolop_drive 가 status 에 signal_needed 를 실어
        #   '잠금 중 + 사라짐 확인 끝 + 직진억제 끝' 일 때만 False 를 주고
        #   여기서 그 값만 보면 된다 (Vehicle_Perception 의 vehicle_needed 와 같은 꼴).

        self.last_run = 0.0
        self.history = []
        self.stable = 'none'
        self.frames = 0
        self.get_logger().info(
            '신호등 인식 시작 · 정규화 색도 + 원형성 · 좌표 하드코딩 없음')

    # ---------------------------------------------------------- 입력
    def _apply_roi_params(self):
        global ROI_TOP, ROI_BOTTOM, ROI_LEFT, ROI_RIGHT
        ROI_TOP = float(self.get_parameter('roi_top').value)
        ROI_BOTTOM = float(self.get_parameter('roi_bottom').value)
        ROI_LEFT = float(self.get_parameter('roi_left').value)
        ROI_RIGHT = float(self.get_parameter('roi_right').value)
        _roi.clear()

    def _on_roi_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        global ROI_TOP, ROI_BOTTOM, ROI_LEFT, ROI_RIGHT
        for p in params:
            if p.name == 'roi_top':
                ROI_TOP = float(p.value)
            elif p.name == 'roi_bottom':
                ROI_BOTTOM = float(p.value)
            elif p.name == 'roi_left':
                ROI_LEFT = float(p.value)
            elif p.name == 'roi_right':
                ROI_RIGHT = float(p.value)
        _roi.clear()
        self.get_logger().info(
            'ROI 갱신: top %.3f bottom %.3f left %.3f right %.3f'
            % (ROI_TOP, ROI_BOTTOM, ROI_LEFT, ROI_RIGHT))
        return SetParametersResult(successful=True)

    def _due(self):
        """max_hz 제한. 디코드 **전에** 본다 — 버릴 프레임은 풀지 않는다.

        (2026-08-23: 30Hz 로 들어오는 JPEG 를 전부 풀고 나서 20Hz 로 버리고
        있었다. 초당 10장, ~3ms 씩 그냥 버린 셈이다.)
        """
        now = time.time()
        if now - self.last_run < 1.0 / max(self.max_hz, 0.1):
            return False
        self.last_run = now
        return True

    def on_compressed(self, msg):
        if not self._due():
            return
        try:
            buf = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        except Exception as err:  # noqa: BLE001
            self.get_logger().warn('압축 영상 디코드 실패: %s' % err)
            return
        if image is not None:
            self.process(image, msg.header)

    def on_raw(self, msg):
        """원본 영상 경로. 지금은 구독하지 않는다 (압축만 쓴다) — 도구용으로 남긴다."""
        if not self._due():
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as err:  # noqa: BLE001
            self.get_logger().warn('원본 영상 변환 실패: %s' % err)
            return
        self.process(image, msg.header)

    # ---------------------------------------------------------- 처리
    def process(self, image, header):
        now = time.time()
        started = time.perf_counter()

        lamps, planes = find_lamps(image)
        state, used = decide(lamps, planes)
        elapsed = (time.perf_counter() - started) * 1000.0
        self.frames += 1

        # 안정화 상태: 같은 판정이 연속 CONFIRM 회 나와야 바꾼다.
        # 원판정(state)과 따로 실어 보낸다. 주행 중에는 원판정을 봐야 한다.
        self.history.append(state)
        if len(self.history) > CONFIRM:
            self.history.pop(0)
        if len(self.history) == CONFIRM and len(set(self.history)) == 1:
            self.stable = state
            # 0823: 첫 직진 확정 뒤 띠를 넓히던 동작을 뺐다. 띠가 넓어지면
            # 비상구 표시등·창문 반사가 들어와 오검출이 늘었다. 띠는 고정.

        height, width = image.shape[:2]
        boxes = []
        for lamp in used:
            x, y, w, h = lamp['rect']
            boxes.append(dict(x=round(x / width, 4), y=round(y / height, 4),
                              w=round(w / width, 4), h=round(h / height, 4),
                              c=lamp['c'], area=lamp['area'],
                              nb=round(lamp['nb'], 3)))
        rx0, ry0, rx1, ry1 = roi_box(image.shape)
        payload = dict(state=state, stable=self.stable, ko=STATE_KO[state],
                       roi=dict(x=round(rx0 / width, 4), y=round(ry0 / height, 4),
                                w=round((rx1 - rx0) / width, 4),
                                h=round((ry1 - ry0) / height, 4)),
                       boxes=boxes, lamps=len(lamps), ms=round(elapsed, 1),
                       frames=self.frames, w=width, h=height,
                       stamp=now)
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.pub_state.publish(message)

        if self.pub_debug is not None:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(
                self.render(image, lamps, used, state, elapsed), 'bgr8'))

    def render(self, image, lamps, used, state, elapsed):
        """원본 위에 결과만 덧그린다. 크기·색 보정 없음."""
        canvas = image.copy()
        rx0, ry0, rx1, ry1 = roi_box(image.shape)
        cv2.rectangle(canvas, (rx0, ry0), (rx1, ry1), (90, 90, 90), 1)
        chosen = {id(b) for b in used}
        for lamp in lamps:
            x, y, w, h = lamp['rect']
            picked = id(lamp) in chosen
            color = DRAW_BGR.get(lamp['c'], (200, 200, 200))
            pad = 3 if picked else 1
            cv2.rectangle(canvas, (x - pad, y - pad), (x + w + pad, y + h + pad),
                          color, 2 if picked else 1)
        label = '%s  %d lamp  %.0fms' % (state.upper(), len(lamps), elapsed)
        cv2.putText(canvas, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 1, cv2.LINE_AA)
        return canvas


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightPerception()
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
