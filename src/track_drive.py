#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Signal-control-only ROS2 module for KMU assignment #1.

This node intentionally does not publish xycar_motor commands.  It only
publishes traffic-signal triggers that the driving stack can consume.
"""

from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String

try:
    from track_drive.traffic_light_detector import TrafficLightDetector
except ImportError:
    from traffic_light_detector import TrafficLightDetector


ACTION_STOP = "STOP"
ACTION_GO_STRAIGHT = "GO_STRAIGHT"
ACTION_GO_LEFT = "GO_LEFT"
ACTION_WAIT_LEFT = "WAIT_LEFT"
ACTION_WAIT_STRAIGHT = "WAIT_STRAIGHT"
ACTION_NO_SIGNAL = "NO_SIGNAL"

ZONE_NONE = "NO_SIGNAL_ZONE"
ZONE_START = "START"
ZONE_SIGNAL = "SIGNAL"
ZONE_LEFT_TURN = "LEFT_TURN"
ZONE_CAMERA_SIGNAL = "CAMERA_SIGNAL"
ZONE_IGNORED_ALWAYS_GREEN = "IGNORED_ALWAYS_GREEN"


@dataclass(frozen=True)
class MapPoint:
    name: str
    x: float
    z: float


@dataclass
class SignalSnapshot:
    start_signal: str = TrafficLightDetector.UNKNOWN
    four_signal: str = TrafficLightDetector.UNKNOWN
    left_arrow: bool = False
    left_presence: bool = False
    left_confidence: float = 0.0
    left_reason: str = "NO_IMAGE"
    signal_presence: bool = False
    candidate_source: str = "NO_IMAGE"
    confidence: float = 0.0
    raw_red_score: float = 0.0
    raw_yellow_score: float = 0.0
    raw_green_score: float = 0.0


@dataclass
class Decision:
    action: str
    zone: str
    go: bool
    turn: str
    reason: str
    desired_signal: str


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def fmt_float(value: Optional[float], ndigits: int = 2) -> str:
    if value is None:
        return "none"
    if not math.isfinite(value):
        return "none"
    return f"{value:.{ndigits}f}"


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """쿼터니언에서 yaw(헤딩 각도)를 추출한다."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class LapTracker:
    """누적 주행거리로 바퀴 수를 카운팅한다.

    odom은 루프 트랙을 돌아도 (0,0)으로 복귀하지 않으므로
    위치 기반 카운팅 대신 누적 이동 거리를 사용한다.

    count=0: 1바퀴 미완주  /  count>=1: 1바퀴 이상 완주 → 좌회전 허용
    lap_min_distance_m: 한 바퀴 거리(미터). 실제 트랙 길이에 맞게 튜닝.
    """

    def __init__(self, lap_min_distance_m: float = 200.0):
        self._lap_dist = lap_min_distance_m
        self.count: int = 0
        self.cumulative_dist: float = 0.0

    def update_distance(self, delta: float) -> None:
        if delta <= 0.0 or not math.isfinite(delta):
            return
        self.cumulative_dist += delta
        new_count = int(self.cumulative_dist / self._lap_dist)
        if new_count > self.count:
            self.count = new_count

    @property
    def left_turn_allowed(self) -> bool:
        """1바퀴 완주 이후(2바퀴째부터) 좌회전 신호 판단을 활성화한다."""
        return self.count >= 1


class SignalMap:
    """Small helper that loads final Unity-map trigger points."""

    def __init__(self, logger):
        self.logger = logger
        self.triggers: List[MapPoint] = []
        self.traffic_lights: List[MapPoint] = []
        self.police: List[MapPoint] = []
        self.map_dir = self._find_map_dir()
        self._load()

    def _candidate_dirs(self) -> Iterable[Path]:
        here = Path(__file__).resolve()
        yield here.parent.parent / "unity_map_extract"
        yield here.parent / "unity_map_extract"
        yield Path.cwd() / "unity_map_extract"
        yield Path("/mnt/c/Users/user/Desktop/Xycar_signal_control_review/unity_map_extract")
        yield Path("C:/Users/user/Desktop/Xycar_signal_control_review/unity_map_extract")

    def _find_map_dir(self) -> Optional[Path]:
        for candidate in self._candidate_dirs():
            if (candidate / "level_world_triggers.csv").exists():
                return candidate
        return None

    def _load_csv(self, filename: str) -> List[MapPoint]:
        if self.map_dir is None:
            return []
        path = self.map_dir / filename
        if not path.exists():
            return []
        points: List[MapPoint] = []
        with path.open("r", encoding="utf-8-sig", newline="") as file:
            for row in csv.DictReader(file):
                try:
                    points.append(
                        MapPoint(
                            name=str(row.get("name", "")),
                            x=float(row["world_x"]),
                            z=float(row["world_z"]),
                        )
                    )
                except (KeyError, TypeError, ValueError):
                    continue
        return points

    def _load(self) -> None:
        self.triggers = self._load_csv("level_world_triggers.csv")
        self.traffic_lights = self._load_csv("level_world_traffic_lights.csv")
        self.police = self._load_csv("level_world_police.csv")
        if self.map_dir is None:
            self.logger.warn("unity_map_extract not found; signal zones will use camera-only gating")
        else:
            self.logger.info(
                f"loaded signal map: {self.map_dir} "
                f"triggers={len(self.triggers)} lights={len(self.traffic_lights)} police={len(self.police)}"
            )

    @staticmethod
    def _dist(a: Tuple[float, float], b: MapPoint) -> float:
        return math.hypot(a[0] - b.x, a[1] - b.z)

    def nearest(self, odom_xy: Optional[Tuple[float, float]], points: Iterable[MapPoint]) -> Tuple[str, Optional[float]]:
        if odom_xy is None:
            return "none", None
        best_name = "none"
        best_dist: Optional[float] = None
        for point in points:
            dist = self._dist(odom_xy, point)
            if best_dist is None or dist < best_dist:
                best_name = point.name
                best_dist = dist
        return best_name, best_dist

    def start_points(self) -> List[MapPoint]:
        keys = ("start", "StartTrigger", "onlyGreen")
        points = [point for point in self.triggers if any(key.lower() in point.name.lower() for key in keys)]
        points.append(MapPoint("observed_start_origin", 0.0, 0.0))
        return points

    def left_turn_points(self) -> List[MapPoint]:
        keys = ("ShortcutTrigger", "TrafficSignCheck-2")
        points = [point for point in self.triggers if any(key.lower() in point.name.lower() for key in keys)]
        points.extend(
            [
                MapPoint("observed_left_turn_a", 18.78, 31.96),
                MapPoint("observed_left_turn_b", 112.73, -80.52),
                MapPoint("observed_left_turn_c", 1.18, -21.13),
                MapPoint("observed_left_turn_d", -15.99, 5.49),
            ]
        )
        return points

    def signal_points(self) -> List[MapPoint]:
        keys = ("TrafficSignCheck", "StartTrigger", "onlyGreen", "ShortcutTrigger", "start")
        points = [point for point in self.triggers if any(key.lower() in point.name.lower() for key in keys)]
        points.extend(self.traffic_lights)
        return points

    def lap_trigger_point(self) -> "Optional[MapPoint]":
        """랩 카운팅에 사용할 odom 기준 트리거 포인트를 반환한다.

        시뮬레이터 odom은 세션 시작 시 (0,0)에서 출발하므로 StartTrigger의
        Unity 세계좌표(37.97, -34.31) 대신 odom 원점(0,0)을 사용한다.
        """
        return MapPoint("StartTrigger_odom", 0.0, 0.0)

    def police_world_positions(self) -> "List[Tuple[float, float]]":
        """맵 CSV에서 실제 경찰차 오브젝트의 세계 좌표 (world_x, world_z)를 반환한다."""
        return [
            (p.x, p.z)
            for p in self.police
            if "policecar" in p.name.lower()
        ]


class SignalControlNode(Node):
    """Publishes signal triggers for the driving stack.

    Assignment policy:
    - red/yellow/orange or a non-driving signal in a signal zone -> STOP
    - circular green outside the left-turn decision case -> GO_STRAIGHT
    - left-turn zone + police blocked -> circular green means GO_STRAIGHT
    - left-turn zone + no police -> wait for green left arrow, then GO_LEFT
    - no visible/active signal zone -> NO_SIGNAL so the driver can use manual or normal driving
    """

    def __init__(self):
        super().__init__("signal_control")
        self.bridge = CvBridge()
        self.detector = TrafficLightDetector()
        self.signal_map = SignalMap(self.get_logger())

        self.image: Optional[np.ndarray] = None
        self.last_image_time = 0.0
        self.cluster_points: List[Tuple[float, float]] = []
        self.last_cluster_time = 0.0
        # /scan 직접 구독 포인트 (경찰차 감지용, 넓은 범위)
        self.scan_points: List[Tuple[float, float]] = []
        self.last_scan_time = 0.0
        self.odom_xy: Optional[Tuple[float, float]] = None
        self.odom_yaw: float = 0.0

        # 랩 카운터: 누적 주행거리 기반 (파라미터 lap_min_distance_m으로 튜닝)
        self.lap_tracker = LapTracker()

        # 경찰차 세계 좌표 (맵 CSV에서 로드)
        self._police_world_positions: List[Tuple[float, float]] = (
            self.signal_map.police_world_positions()
        )
        if self._police_world_positions:
            self.get_logger().info(
                f"police world positions loaded: {self._police_world_positions}"
            )

        self.stop_until = 0.0
        self.green_until = 0.0
        self.left_until = 0.0
        self.police_candidate_frames = 0
        self.police_clear_frames = 0
        self.police_blocked_state = False
        # 카메라 기반 경찰차 감지 상태
        self.police_cam_candidate_frames = 0
        self.police_cam_blocked_state = False

        self.last_snapshot = SignalSnapshot()
        self.last_decision = Decision(ACTION_NO_SIGNAL, ZONE_NONE, False, "none", "INIT", "none")
        self.last_debug: Dict[str, object] = {}

        self._declare_parameters()

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.trigger_pub = self.create_publisher(String, "/track_drive/signal_trigger", qos)
        self.debug_pub = self.create_publisher(String, "/track_drive/signal_debug", qos)

        self.create_subscription(Image, "/usb_cam/image_raw/front", self._on_image, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/lidar_cluster", self._on_lidar_cluster_cloud2, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.create_timer(1.0 / publish_hz, self._tick)
        self.get_logger().info(
            "signal_control ready: publishes /track_drive/signal_trigger and /track_drive/signal_debug only"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("image_stale_sec", 0.35)
        self.declare_parameter("start_zone_radius", 12.0)
        self.declare_parameter("signal_zone_radius", 13.0)
        self.declare_parameter("left_zone_radius", 16.0)
        self.declare_parameter("always_green_ignore_radius", 6.0)
        self.declare_parameter("stop_latch_sec", 0.30)
        self.declare_parameter("green_latch_sec", 0.25)
        self.declare_parameter("left_latch_sec", 0.35)
        self.declare_parameter("cluster_stale_sec", 0.50)
        self.declare_parameter("police_x_min", 0.35)
        self.declare_parameter("police_x_max", 7.80)
        self.declare_parameter("police_y_min", 0.25)
        self.declare_parameter("police_y_max", 3.30)
        self.declare_parameter("police_cluster_dist", 0.60)
        self.declare_parameter("police_cluster_min_points", 8)  # 5→8 (6점 오검출 제거)
        # false positive 기준: x_width=0.22m, y_width=0.46m, points=6
        # true positive 기준: x_width≈2.43m, y_width≈1.0m, points≈9+
        self.declare_parameter("police_car_min_x_width", 0.50)  # 0.18→0.50 (기둥/나무 제거)
        self.declare_parameter("police_car_max_x_width", 2.80)
        self.declare_parameter("police_car_min_y_width", 0.60)  # 0.18→0.60 (0.46m 오검출 제거)
        self.declare_parameter("police_car_max_y_width", 2.40)
        self.declare_parameter("police_confirm_frames", 3)
        self.declare_parameter("police_clear_frames", 2)
        self.declare_parameter("left_arrow_fallback_green_ratio", 0.55)  # 기존 0.18에서 상향 (오판 방지)
        self.declare_parameter("left_arrow_min_confidence", 0.30)
        # 경찰차 세계 좌표 탐색 반경 (미터). 이 범위 밖의 LiDAR 포인트는 무시한다.
        self.declare_parameter("police_world_search_radius", 8.0)
        # 오도메트리 yaw 보정값 (라디안). 시뮬레이터 yaw 규약이 다를 경우 튜닝.
        self.declare_parameter("yaw_offset", 0.0)
        # 랩 카운터: 한 바퀴 누적 주행거리 임계값
        # 튜닝 기준: 1바퀴 ShortcutTrigger cumul_dist < 이 값 < 2바퀴 ShortcutTrigger cumul_dist
        # 실측 참고: cumul=494.5에서 lap=2 관측 → 1바퀴≈330m → 기본값 300
        self.declare_parameter("lap_min_distance_m", 300.0)
        # 경찰차 감지용 /scan 직접 구독 파라미터 (실측 기준 튜닝됨)
        # 실제 관측: 경찰차 후보 x≈5.1~7.5m, y≈1.9~2.9m (y 양수 방향)
        self.declare_parameter("police_scan_range_max", 15.0)  # 광역 scan 최대 거리
        self.declare_parameter("police_scan_y_abs_max", 4.0)   # ROI y 상한 (2~3m + 마진)
        self.declare_parameter("police_scan_x_min", 3.0)       # ROI x 시작 (관측 5m에 마진)
        self.declare_parameter("police_scan_x_max", 9.0)       # ROI x 끝 (경찰차≈7.5m+마진, 나무 제외)
        # 경찰차 ROI y 하한 (자차 측면 노이즈 제거용)
        self.declare_parameter("police_left_lateral_min", 1.5)
        # WORLD_GATE: odom좌표계≠Unity세계좌표계이므로 기본 비활성화
        # odom 좌표 보정이 확인된 후 True로 변경 가능
        self.declare_parameter("police_world_gate_enable", False)
        # ── 카메라 기반 경찰차 감지 (라바콘 주황색 감지)
        # 경찰차 앞에 배치된 라바콘(주황색)을 카메라로 감지해 경찰차 유무 판단
        # 시뮬레이터 실측 (640x480, 프론트 카메라):
        #   ROI x=0~22%, y=50~75%  ← 배경 주황 나무 완벽히 제외됨
        #   경찰차 있음 → 주황 픽셀 ≈ 246px
        #   경찰차 없음 → ≈   0px  (이 ROI 안에 주황 배경 없음)
        self.declare_parameter("police_cam_enable", True)
        # 라바콘 ROI (배경 주황 나무는 x>60%에 위치 → x=0~22%로 완전 분리)
        self.declare_parameter("police_cam_roi_x2", 0.22)   # 왼쪽 22%까지
        self.declare_parameter("police_cam_roi_y1", 0.50)   # 위에서 50%
        self.declare_parameter("police_cam_roi_y2", 0.75)   # 위에서 75%
        # 주황색 HSV 기준 (라바콘: H=5~22, S>120, V>80)
        self.declare_parameter("police_cam_cone_h_lo",  5)
        self.declare_parameter("police_cam_cone_h_hi", 22)
        self.declare_parameter("police_cam_cone_s_min", 120)
        self.declare_parameter("police_cam_cone_v_min",  80)
        # 감지 임계값
        # 실측: 경찰차 없음(배경 트랙 라바콘) ≈ 120px / 경찰차 있음(근접) ≈ 162px+
        # 중간값 140px 사용 (여유 ~20px)
        self.declare_parameter("police_cam_cone_min_px", 140)
        # 연속 프레임 확인 수 (튀는 값 방지)
        self.declare_parameter("police_cam_confirm_frames", 2)

    def _on_image(self, msg: Image) -> None:
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.last_image_time = time.monotonic()
        except Exception as exc:
            self.get_logger().warn(f"image conversion failed: {exc}")

    def _on_lidar_cluster_cloud2(self, msg: PointCloud2) -> None:
        points: List[Tuple[float, float]] = []
        try:
            for point in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True):
                points.append((float(point[0]), float(point[1])))
        except Exception as exc:
            self.get_logger().warn(f"lidar_cluster PointCloud2 parse failed: {exc}")
            return
        self.cluster_points = points
        self.last_cluster_time = time.monotonic()

    def _on_scan(self, msg: LaserScan) -> None:
        """경찰차 감지 전용 /scan 구독.

        lidar_cluster_node는 range_max=8m, y_abs_max=4m으로 설정되어 있어
        왼쪽 약 11.45m 위치의 경찰차 포인트를 모두 차단한다.
        이 콜백은 더 넓은 범위(기본 15m, ±13m)로 직접 변환해 self.scan_points에 저장한다.
        lidar_cluster_node와 동일한 좌표 규약: x=전방(양수), y=우측(양수, y_sign=-1).
        """
        r_max = float(self.get_parameter("police_scan_range_max").value)
        y_abs_max = float(self.get_parameter("police_scan_y_abs_max").value)
        x_min = float(self.get_parameter("police_scan_x_min").value)
        x_max = float(self.get_parameter("police_scan_x_max").value)
        front_offset = math.pi / 2.0  # lidar_cluster_node 기본값과 동일
        pts: List[Tuple[float, float]] = []
        for idx, raw_r in enumerate(msg.ranges):
            r = float(raw_r)
            if not math.isfinite(r) or r < 0.05 or r > r_max:
                continue
            theta = normalize_angle(
                float(msg.angle_min) + idx * float(msg.angle_increment) - front_offset
            )
            x = r * math.cos(theta)
            y = r * math.sin(theta) * (-1.0)  # 실측 결과: 경찰차(왼쪽)=y 양수
            if x_min <= x <= x_max and abs(y) <= y_abs_max:
                pts.append((x, y))
        self.scan_points = pts
        self.last_scan_time = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        new_xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        if self.odom_xy is not None:
            dx = new_xy[0] - self.odom_xy[0]
            dy = new_xy[1] - self.odom_xy[1]
            delta = math.hypot(dx, dy)
            if delta < 2.0:  # spike filter: 2m 이상 점프는 초기화/리셋으로 간주
                self.lap_tracker.update_distance(delta)
        self.odom_xy = new_xy
        q = msg.pose.pose.orientation
        raw_yaw = _quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
        self.odom_yaw = normalize_angle(raw_yaw + float(self.get_parameter("yaw_offset").value))

    def _tick(self) -> None:
        now = time.monotonic()
        # LapTracker 파라미터 동적 반영 (런타임 튜닝 가능)
        self.lap_tracker._lap_dist = float(self.get_parameter("lap_min_distance_m").value)
        snapshot = self._detect_signal(now)
        zone, nearest_name, nearest_dist, map_gate = self._zone(snapshot)
        if zone == ZONE_IGNORED_ALWAYS_GREEN:
            police_blocked, police_count, police_debug = False, 0, {
                "police_x_minmax": "ignored",
                "police_y_minmax": "ignored",
                "police_median_x": "ignored",
                "police_reason": "IGNORED_ALWAYS_GREEN_ZONE",
            }
        elif zone not in (ZONE_LEFT_TURN, ZONE_CAMERA_SIGNAL) or not self.lap_tracker.left_turn_allowed:
            self._reset_police_state()
            police_blocked, police_count, police_debug = False, 0, {
                "police_x_minmax": "skipped",
                "police_y_minmax": "skipped",
                "police_median_x": "skipped",
                "police_cluster_count": 0,
                "police_candidates": "[]",
                "police_reason": "SKIPPED_NON_LEFT_TURN_ZONE",
            }
        else:
            # ── LEFT_TURN 또는 CAMERA_SIGNAL (lap 2/3): 라이다 + 카메라 경찰차 감지 병행
            lidar_blocked, police_count, police_debug = self._detect_police_from_clusters(now)
            cam_blocked, cam_reason = self._detect_police_from_camera()
            # 어느 하나라도 감지되면 경찰차 있음으로 판단
            police_blocked = lidar_blocked or cam_blocked
            police_debug["police_cam_blocked"] = str(cam_blocked).lower()
            police_debug["police_cam_reason"] = cam_reason  # CAM_DARK_RATIO=X.XXX
        decision = self._decide(snapshot, zone, police_blocked, now)

        self.last_snapshot = snapshot
        self.last_decision = decision
        self.last_debug = {"police_count": police_count, **police_debug}
        self._publish(decision, snapshot, police_blocked, police_count, nearest_name, nearest_dist, map_gate)

    def _detect_signal(self, now: float) -> SignalSnapshot:
        image_stale_sec = float(self.get_parameter("image_stale_sec").value)
        if self.image is None or now - self.last_image_time > image_stale_sec:
            return SignalSnapshot(left_reason="NO_IMAGE_OR_STALE")

        image = self.image
        start_signal = self.detector.detect_start_light(image)
        four_detail = self.detector.detect_four_light_state(image)
        left_detail = self.detector.detect_left_turn_detail(image, update_history=True)

        four_signal = str(four_detail.get("signal", TrafficLightDetector.UNKNOWN))
        left_conf = float(left_detail.get("confidence", 0.0))
        left_raw = bool(left_detail.get("raw_signal", False))
        left_stable = bool(left_detail.get("stable_signal", False))
        left_min_conf = float(self.get_parameter("left_arrow_min_confidence").value)
        left_arrow = left_stable or (left_raw and left_conf >= left_min_conf)

        scores = [
            float(four_detail.get("red_score", 0.0)),
            float(four_detail.get("yellow_score", 0.0)),
            float(four_detail.get("green_score", 0.0)),
            float(four_detail.get("raw_red_score", 0.0)),
            float(four_detail.get("raw_yellow_score", 0.0)),
            float(four_detail.get("raw_green_score", 0.0)),
        ]
        confidence = max(scores + [left_conf])
        raw_red_score = float(four_detail.get("raw_red_score", 0.0))
        raw_yellow_score = float(four_detail.get("raw_yellow_score", 0.0))
        raw_green_score = float(four_detail.get("raw_green_score", 0.0))
        green_ratio = raw_green_score / max(1.0, raw_red_score + raw_yellow_score + raw_green_score)
        arrow_fallback = (
            bool(left_detail.get("presence", False))
            and green_ratio >= float(self.get_parameter("left_arrow_fallback_green_ratio").value)
            and four_signal in (TrafficLightDetector.START_RED, TrafficLightDetector.UNKNOWN)
        )
        left_arrow = left_arrow or arrow_fallback

        # stop_until 래치: 4구 신호(four_signal)만 사용.
        # start_signal은 출발선 전용 단순 검출기라 좌회전/신호 구간에서 오검출(RED)이 빈번함.
        # ZONE_START 판단은 _decide()에서 start_signal을 직접 확인하므로 여기서는 제외.
        stop_detected = four_signal in (TrafficLightDetector.START_RED, TrafficLightDetector.START_YELLOW)
        green_detected = four_signal == TrafficLightDetector.START_GREEN or start_signal == TrafficLightDetector.START_GREEN

        if stop_detected:
            self.stop_until = now + float(self.get_parameter("stop_latch_sec").value)
        if green_detected:
            self.green_until = now + float(self.get_parameter("green_latch_sec").value)
        if left_arrow:
            self.left_until = now + float(self.get_parameter("left_latch_sec").value)

        if now < self.stop_until:
            if four_signal == TrafficLightDetector.UNKNOWN:
                four_signal = TrafficLightDetector.START_RED
        if now < self.green_until and four_signal == TrafficLightDetector.UNKNOWN:
            four_signal = TrafficLightDetector.START_GREEN
        if now < self.left_until:
            left_arrow = True

        return SignalSnapshot(
            start_signal=start_signal,
            four_signal=four_signal,
            left_arrow=left_arrow,
            left_presence=bool(left_detail.get("presence", False)),
            left_confidence=left_conf,
            left_reason=str(left_detail.get("reason", "NO_REASON")),
            signal_presence=bool(four_detail.get("presence", False) or left_detail.get("presence", False)),
            candidate_source=str(four_detail.get("candidate_source", "NONE")),
            confidence=confidence,
            raw_red_score=raw_red_score,
            raw_yellow_score=raw_yellow_score,
            raw_green_score=raw_green_score,
        )

    def _zone(self, snapshot: SignalSnapshot) -> Tuple[str, str, Optional[float], str]:
        start_name, start_dist = self.signal_map.nearest(self.odom_xy, self.signal_map.start_points())
        left_name, left_dist = self.signal_map.nearest(self.odom_xy, self.signal_map.left_turn_points())
        signal_name, signal_dist = self.signal_map.nearest(self.odom_xy, self.signal_map.signal_points())
        ignore_name, ignore_dist = self.signal_map.nearest(self.odom_xy, self._always_green_ignore_points())

        start_radius = float(self.get_parameter("start_zone_radius").value)
        left_radius = float(self.get_parameter("left_zone_radius").value)
        signal_radius = float(self.get_parameter("signal_zone_radius").value)
        ignore_radius = float(self.get_parameter("always_green_ignore_radius").value)

        # ── 우선순위 1: 좌회전 구간 (2바퀴 이상, 카메라 gate 불필요)
        # 경찰차로 인해 원형 녹색이 나올 때도 zone=LEFT_TURN이어야 경찰차 감지가 실행됨.
        # 신호 색(좌화살표 vs 원형녹색) 판단은 _decide()에서 수행한다.
        if left_dist is not None and left_dist <= left_radius and self.lap_tracker.left_turn_allowed:
            return ZONE_LEFT_TURN, left_name, left_dist, "map_left_turn"

        # ── 우선순위 2: 항상-녹색 통과 구간 (START보다 먼저 체크해야 오분류 방지)
        if ignore_dist is not None and ignore_dist <= ignore_radius:
            return ZONE_IGNORED_ALWAYS_GREEN, ignore_name, ignore_dist, "map_always_green_ignore"

        # ── 우선순위 3: 출발 구간
        # left_presence=True(4구 신호 감지)이면 출발 구간이 아님 → START 오분류 방지
        if start_dist is not None and start_dist <= start_radius and not snapshot.left_presence:
            return ZONE_START, start_name, start_dist, "map_start"

        # ── 우선순위 4: 일반 신호 구간
        if signal_dist is not None and signal_dist <= signal_radius:
            return ZONE_SIGNAL, signal_name, signal_dist, "map_signal"

        # ── 카메라 기반 폴백 (odom/맵 좌표 불일치 시 보완)
        if snapshot.left_presence and self.lap_tracker.left_turn_allowed:
            return ZONE_LEFT_TURN, signal_name, signal_dist, "camera_left_presence"
        if snapshot.signal_presence:
            return ZONE_CAMERA_SIGNAL, signal_name, signal_dist, "camera_signal_presence"
        return ZONE_NONE, signal_name, signal_dist, "none"

    def _always_green_ignore_points(self) -> List[MapPoint]:
        """TrafficLight-1 is a constant circular-green pass-through in this map.

        The signal controller should not emit STOP/WAIT/GO triggers for this
        location because the driving team does not need a signal decision here.
        """

        names = (
            "TrafficLight-1",
            "PREFAB_trafficLight-2B_1",
            "LIGHT_red-1",
            "LIGHT_yellow-1",
            "LIGHT_green-1",
        )
        points = [
            point
            for point in self.signal_map.traffic_lights
            if any(name.lower() in point.name.lower() for name in names)
        ]
        # Runtime odom coordinates observed at the same always-green signal can
        # differ from extracted Unity world coordinates depending on simulator
        # session/reset state.  Keep both observed centers out of signal gating.
        points.extend(
            [
                MapPoint("observed_always_green_trafficlight1_a", 17.53, -30.31),
                MapPoint("observed_always_green_trafficlight1_b", 110.86, -104.08),
                MapPoint("observed_ignore_traffic_sign_check_1", -2.76, -33.30),
            ]
        )
        return points

    def _reset_police_state(self) -> None:
        self.police_candidate_frames = 0
        self.police_clear_frames = 0
        self.police_blocked_state = False
        self.police_cam_candidate_frames = 0
        self.police_cam_blocked_state = False

    def _detect_police_from_camera(self) -> Tuple[bool, str]:
        """프론트 카메라에서 경찰차 앞 라바콘(주황색)을 감지해 경찰차 유무를 판단한다.

        실측(640x480, 프론트 카메라):
          ROI x=0~22%, y=50~75%  → 배경 주황 나무는 x>60%에 있어 완벽히 제외됨
          경찰차+라바콘 있음 → 주황 픽셀 ≈ 246 px
          경찰차 없음       → ≈   0 px
          임계값 50 px 으로 두 케이스 완벽히 분리.
        """
        if not bool(self.get_parameter("police_cam_enable").value):
            return False, "DISABLED"
        if self.image is None:
            return False, "NO_IMAGE"

        img = self.image
        h, w = img.shape[:2]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # ── 라바콘 ROI ────────────────────────────────────────────────
        x2 = int(w * float(self.get_parameter("police_cam_roi_x2").value))
        y1 = int(h * float(self.get_parameter("police_cam_roi_y1").value))
        y2 = int(h * float(self.get_parameter("police_cam_roi_y2").value))
        roi_hsv = hsv[y1:y2, 0:x2]
        if roi_hsv.size == 0:
            return False, "EMPTY_ROI"

        # ── 주황색 라바콘 마스크 ──────────────────────────────────────
        h_lo  = int(self.get_parameter("police_cam_cone_h_lo").value)
        h_hi  = int(self.get_parameter("police_cam_cone_h_hi").value)
        s_min = int(self.get_parameter("police_cam_cone_s_min").value)
        v_min = int(self.get_parameter("police_cam_cone_v_min").value)
        cone_mask = cv2.inRange(
            roi_hsv,
            np.array([h_lo, s_min, v_min], dtype=np.uint8),
            np.array([h_hi, 255,   255],   dtype=np.uint8),
        )
        cone_px = int(cv2.countNonZero(cone_mask))

        thr = int(self.get_parameter("police_cam_cone_min_px").value)
        confirm_n = int(self.get_parameter("police_cam_confirm_frames").value)

        if cone_px >= thr:
            self.police_cam_candidate_frames = min(
                self.police_cam_candidate_frames + 1, confirm_n + 1
            )
            if self.police_cam_candidate_frames >= confirm_n:
                self.police_cam_blocked_state = True
        else:
            self.police_cam_candidate_frames = 0
            self.police_cam_blocked_state = False

        reason = f"CAM_CONE_PX={cone_px}_THR={thr}"
        return self.police_cam_blocked_state, reason

    def _vehicle_to_world(self, lx: float, ly: float) -> Tuple[float, float]:
        """차량 좌표계 LiDAR 포인트를 세계 좌표계로 변환한다.

        lidar_cluster_node의 출력: x=전방(양수), y=우측(양수, y_sign=-1 적용 결과)
        ROS 표준: x=전방, y=좌측. y를 반전해서 ROS 표준으로 변환 후 회전 적용.
        """
        if self.odom_xy is None:
            return lx, ly
        ox, oy = self.odom_xy
        yaw = self.odom_yaw
        # LiDAR 출력의 y는 우측이 양수 → ROS 표준(좌측 양수)으로 반전
        ly_std = -ly
        wx = ox + lx * math.cos(yaw) - ly_std * math.sin(yaw)
        wy = oy + lx * math.sin(yaw) + ly_std * math.cos(yaw)
        return wx, wy

    def _world_gated_police_points(self, world_radius: float) -> Tuple[List[Tuple[float, float]], str]:
        """경찰차 후보 /scan 포인트를 반환한다.

        [왜 WORLD_GATE가 기본 비활성화인가]
        odom 좌표계(시뮬레이터 세션 시작 시 (0,0)에서 누적)와
        Unity 세계 좌표계(CSV: 경찰차 36.20,21.10)가 달라
        _vehicle_to_world()의 결과가 CSV 좌표와 수십 m 오차가 생긴다.
        police_world_gate_enable=True는 odom 보정이 확인된 후 사용한다.

        기본값: 차량 상대 왼쪽 ROI
        경찰차 위치: 왼쪽(음수y) 약 11.45m, x ∈ [-7, +12]
        """
        px_min = float(self.get_parameter("police_scan_x_min").value)
        px_max = float(self.get_parameter("police_scan_x_max").value)
        lat_min = float(self.get_parameter("police_left_lateral_min").value)   # 8.0
        lat_max = float(self.get_parameter("police_scan_y_abs_max").value)     # 13.0

        world_gate_enable = bool(self.get_parameter("police_world_gate_enable").value)
        if world_gate_enable and self.odom_xy is not None and self._police_world_positions:
            result: List[Tuple[float, float]] = []
            for lx, ly in self.scan_points:
                if not math.isfinite(lx) or not math.isfinite(ly):
                    continue
                wx, wy = self._vehicle_to_world(lx, ly)
                for pwx, pwz in self._police_world_positions:
                    if math.hypot(wx - pwx, wy - pwz) <= world_radius:
                        result.append((lx, ly))
                        break
            return result, "WORLD_GATE"

        # 차량 상대 ROI (실측: 경찰차 후보 x≈5~7m, y≈+2~3m)
        pts = [
            (x, y)
            for x, y in self.scan_points
            if math.isfinite(x) and math.isfinite(y)
            and px_min <= x <= px_max
            and lat_min <= y <= lat_max  # 실측 기준: y 양수 방향
        ]
        return pts, "VEHICLE_ROI_LEFT"

    def _detect_police_from_clusters(self, now: float) -> Tuple[bool, int, Dict[str, object]]:
        if now - self.last_scan_time > float(self.get_parameter("cluster_stale_sec").value):
            self.police_candidate_frames = 0
            self.police_clear_frames += 1
            if self.police_clear_frames >= int(self.get_parameter("police_clear_frames").value):
                self.police_blocked_state = False
            return self.police_blocked_state, 0, {
                "police_filter": "STALE",
                "police_roi_points": 0,
                "police_cluster_count": 0,
                "police_candidates": "[]",
                "police_candidate_frames": self.police_candidate_frames,
                "police_clear_frames": self.police_clear_frames,
                "police_reason": "NO_SCAN",
            }

        world_radius = float(self.get_parameter("police_world_search_radius").value)
        roi_points, filter_mode = self._world_gated_police_points(world_radius)

        clusters = self._cluster_2d(roi_points, float(self.get_parameter("police_cluster_dist").value))
        candidates = [c for c in (self._car_like_candidate(cl) for cl in clusters) if c is not None]

        if candidates:
            self.police_candidate_frames += 1
            self.police_clear_frames = 0
            if self.police_candidate_frames >= int(self.get_parameter("police_confirm_frames").value):
                self.police_blocked_state = True
        else:
            self.police_candidate_frames = 0
            self.police_clear_frames += 1
            if self.police_clear_frames >= int(self.get_parameter("police_clear_frames").value):
                self.police_blocked_state = False

        best = candidates[0] if candidates else None
        return self.police_blocked_state, len(roi_points), {
            "police_filter": filter_mode,
            "police_roi_points": len(roi_points),
            "police_cluster_count": len(clusters),
            "police_candidates": self._format_candidates(candidates),
            "police_candidate_bbox": best["bbox"] if best else "none",
            "police_candidate_frames": self.police_candidate_frames,
            "police_clear_frames": self.police_clear_frames,
            "police_reason": "CLUSTER_CAR_LIKE_CONFIRMED" if self.police_blocked_state else "CLEAR",
        }

    def _cluster_2d(self, points: List[Tuple[float, float]], threshold: float) -> List[List[Tuple[float, float]]]:
        if not points:
            return []
        unused = set(range(len(points)))
        clusters: List[List[Tuple[float, float]]] = []
        threshold_sq = threshold * threshold
        while unused:
            seed = unused.pop()
            queue = [seed]
            cluster_indices = [seed]
            while queue:
                current = queue.pop()
                cx, cy = points[current]
                neighbors = []
                for other in list(unused):
                    ox, oy = points[other]
                    if (cx - ox) * (cx - ox) + (cy - oy) * (cy - oy) <= threshold_sq:
                        neighbors.append(other)
                for other in neighbors:
                    unused.remove(other)
                    queue.append(other)
                    cluster_indices.append(other)
            clusters.append([points[index] for index in cluster_indices])
        return clusters

    def _car_like_candidate(self, cluster: List[Tuple[float, float]]) -> Optional[Dict[str, object]]:
        if len(cluster) < int(self.get_parameter("police_cluster_min_points").value):
            return None
        xs = [point[0] for point in cluster]
        ys = [point[1] for point in cluster]
        x_width = max(xs) - min(xs)
        y_width = max(ys) - min(ys)
        if not (
            float(self.get_parameter("police_car_min_x_width").value)
            <= x_width
            <= float(self.get_parameter("police_car_max_x_width").value)
        ):
            return None
        if not (
            float(self.get_parameter("police_car_min_y_width").value)
            <= y_width
            <= float(self.get_parameter("police_car_max_y_width").value)
        ):
            return None
        bbox = f"{min(xs):.2f}:{max(xs):.2f},{min(ys):.2f}:{max(ys):.2f}"
        return {
            "bbox": bbox,
            "points": len(cluster),
            "x_width": round(x_width, 2),
            "y_width": round(y_width, 2),
        }

    @staticmethod
    def _format_candidates(candidates: List[Dict[str, object]]) -> str:
        if not candidates:
            return "[]"
        return "[" + ";".join(str(candidate) for candidate in candidates[:3]) + "]"

    def _decide(self, s: SignalSnapshot, zone: str, police_blocked: bool, now: float) -> Decision:
        if zone == ZONE_NONE:
            return Decision(ACTION_NO_SIGNAL, zone, False, "none", "NO_ACTIVE_SIGNAL_ZONE", "none")
        if zone == ZONE_IGNORED_ALWAYS_GREEN:
            return Decision(
                ACTION_NO_SIGNAL,
                zone,
                False,
                "none",
                "ALWAYS_GREEN_LOCATION_IGNORED",
                "none",
            )

        if zone == ZONE_START:
            red_or_yellow = s.start_signal in (
                TrafficLightDetector.START_RED,
                TrafficLightDetector.START_YELLOW,
            ) or s.four_signal in (
                TrafficLightDetector.START_RED,
                TrafficLightDetector.START_YELLOW,
            )
            circular_green = (
                s.start_signal == TrafficLightDetector.START_GREEN
                or s.four_signal == TrafficLightDetector.START_GREEN
                or now < self.green_until
            )
        else:
            # Outside the start gate, the 3-lamp start detector can see bright
            # scenery or a side lamp and produce stale YELLOW/RED.  The active
            # driving signal is the four-light/left-turn detector here.
            red_or_yellow = s.four_signal in (
                TrafficLightDetector.START_RED,
                TrafficLightDetector.START_YELLOW,
            )
            circular_green = s.four_signal == TrafficLightDetector.START_GREEN or now < self.green_until

        if zone == ZONE_LEFT_TURN:
            # ── 판단 순서 ──────────────────────────────────────────────────────
            # [1단계] 경찰차(라바콘) 감지 먼저
            # [2단계] 경찰차 있음: 원형녹색만 직진 / 나머지(빨강·노랑·좌화살표) 정지
            # [3단계] 경찰차 없음: 좌화살표만 좌회전 / 나머지(빨강·노랑·원형녹색) 정지
            # ──────────────────────────────────────────────────────────────────
            if police_blocked:
                # 경찰차 있음 → 원형녹색에만 직진, 그 외 모두 정지
                if circular_green:
                    return Decision(
                        ACTION_GO_STRAIGHT, zone, True, "straight",
                        "POLICE_CIRCULAR_GREEN_GO_STRAIGHT", "circular_green"
                    )
                return Decision(
                    ACTION_STOP, zone, False, "none",
                    "POLICE_WAIT_CIRCULAR_GREEN", "circular_green"
                )
            else:
                # 경찰차 없음 → 좌화살표에만 좌회전, 그 외 모두 정지
                if s.left_arrow:
                    return Decision(
                        ACTION_GO_LEFT, zone, True, "left",
                        "NO_POLICE_LEFT_ARROW_GO_LEFT", "left_arrow"
                    )
                return Decision(
                    ACTION_STOP, zone, False, "none",
                    "NO_POLICE_WAIT_LEFT_ARROW", "left_arrow"
                )

        if red_or_yellow or now < self.stop_until:
            return Decision(ACTION_STOP, zone, False, "none", "RED_OR_YELLOW_STOP", "not_red_yellow")

        if zone == ZONE_START:
            if circular_green:
                return Decision(
                    ACTION_GO_STRAIGHT,
                    zone,
                    True,
                    "straight",
                    "START_CIRCULAR_GREEN_GO",
                    "circular_green",
                )
            return Decision(ACTION_STOP, zone, False, "straight", "START_WAIT_GREEN", "circular_green")

        if circular_green:
            return Decision(
                ACTION_GO_STRAIGHT,
                zone,
                True,
                "straight",
                "CIRCULAR_GREEN_GO_STRAIGHT",
                "circular_green",
            )

        return Decision(ACTION_STOP, zone, False, "none", "SIGNAL_ZONE_NO_GO_SIGNAL_STOP", "circular_green")

    def _publish(
        self,
        decision: Decision,
        snapshot: SignalSnapshot,
        police_blocked: bool,
        police_count: int,
        nearest_name: str,
        nearest_dist: Optional[float],
        map_gate: str,
    ) -> None:
        odom_x = self.odom_xy[0] if self.odom_xy else None
        odom_y = self.odom_xy[1] if self.odom_xy else None
        fields: Dict[str, object] = {
            "action": decision.action,
            "go": str(decision.go).lower(),
            "turn": decision.turn,
            "zone": decision.zone,
            "reason": decision.reason,
            "desired_signal": decision.desired_signal,
            "signal": snapshot.four_signal,
            "start_signal": snapshot.start_signal,
            "left_arrow": str(snapshot.left_arrow).lower(),
            "left_presence": str(snapshot.left_presence).lower(),
            "left_conf": fmt_float(snapshot.left_confidence, 2),
            "left_reason": snapshot.left_reason,
            "police_blocked": str(police_blocked).lower(),
            "police_points": police_count,
            "candidate_source": snapshot.candidate_source,
            "confidence": fmt_float(snapshot.confidence, 1),
            "raw_red": fmt_float(snapshot.raw_red_score, 1),
            "raw_yellow": fmt_float(snapshot.raw_yellow_score, 1),
            "raw_green": fmt_float(snapshot.raw_green_score, 1),
            "map_gate": map_gate,
            "nearest": nearest_name,
            "dist": fmt_float(nearest_dist, 2),
            "odom_x": fmt_float(odom_x, 2),
            "odom_y": fmt_float(odom_y, 2),
            "odom_yaw": fmt_float(self.odom_yaw, 3),
            "lap": self.lap_tracker.count,
            "cumul_dist": fmt_float(self.lap_tracker.cumulative_dist, 1),
            "left_turn_allowed": str(self.lap_tracker.left_turn_allowed).lower(),
            "stamp": f"{time.time():.3f}",
        }
        fields.update(self.last_debug)
        payload = " ".join(f"{key}={value}" for key, value in fields.items())

        msg = String()
        msg.data = payload
        self.trigger_pub.publish(msg)
        self.debug_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SignalControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
