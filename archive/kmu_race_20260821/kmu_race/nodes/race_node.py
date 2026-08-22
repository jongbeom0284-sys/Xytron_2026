"""메인 노드 — 이 패키지에서 rclpy 를 아는 유일한 실행 진입점.

구조 원칙 (doldolmeng2/2026-kookmin-contest 의 설계를 따랐다):
    노드는 얇게. 센서 → 데이터클래스 변환, core 호출, 모터 발행만 한다.
    판단 로직은 전부 core/ 에 있고 rclpy 를 모른다. 그래서 실차 없이
    `pytest` 로 FSM 과 정책을 통째로 검증할 수 있다.

동시성:
    카메라 콜백에서 무거운 CV 를 돌리고, 별도 타이머에서 제어를 발행한다.
    콜백별 MutuallyExclusive 그룹 + MultiThreadedExecutor 를 써서 CV 가 제어
    주기를 막지 않으면서도, 같은 콜백이 자기 자신과 병렬 실행되어 낡은
    프레임이 최신 관측을 덮어쓰는 일은 없게 한다. (`rclpy.spin_once` 를
    수동으로 여러 번 부르는 방식은 콜백 지연과 프레임 드랍을 만들어 쓰지 않는다.)

대회 당일 주의:
    규정 p.16 — 실행 명령은 `ros2 launch ...` 여야 하고 심판이 화면을 봐야 한다.
    `debug_gui:=false` 로 OpenCV 창을 전부 끄고 돌릴 것. 창 19개를 띄운 채로는
    프레임이 밀려 4분 예산(p.37)을 지키기 어렵다. 상태는 로그로 보여 준다.
"""

from __future__ import annotations

import math
import time
from dataclasses import replace
from typing import List, Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Empty

from ..core.heading import GyroHeading
from ..core.policy import DrivePolicy, PolicyConfig
from ..core.race_context import RaceContext
from ..core.race_fsm import Mode, RaceFSM
from ..core.safety import SafetyMonitor
from ..core.types import (
    ConeReading,
    LaneReading,
    LidarReading,
    Observation,
    SignalReading,
)
from ..io.motor import MotorPublisher
from ..perception.cone_detector import ConeDetector
from ..perception.lidar_cones import LidarGate, find_cone_gate
from ..perception.lane_detector import LaneDetector
from ..perception.lidar_tracker import LidarTracker
from ..io.camera_exposure import CameraExposure
from ..perception.signal_detector import SignalDetector
from ..perception.signal_watcher import SignalWatcher, find_lamp_blobs


class RaceNode(Node):
    def __init__(self) -> None:
        super().__init__("kmu_race")

        # ── 파라미터 ──────────────────────────────────────────────
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("debug_gui", False)
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("motor_topic", "xycar_motor")
        self.declare_parameter("use_xycar_msgs", False)
        # 명령속도[자이카 단위] → 실제 속도[m/s] 환산. 실차에서 반드시 실측할 것.
        # 정지/주행 장애물 판별의 정확도가 여기에 달려 있다.
        self.declare_parameter("speed_to_mps", 0.08)
        self.declare_parameter("input_timeout_s", 0.35)
        # 최종 명령속도에 곱하는 전역 스케일 — 시운전 단계 상승용
        # (0.5 저속 → 0.75 → 1.0). 조향은 건드리지 않는다: 저속에서도 같은
        # 곡률을 그리려면 조향각은 그대로여야 한다. 본선은 반드시 1.0.
        self.declare_parameter("speed_scale", 1.0)
        # 구간별 목표속도 — 트랙에서 바로 조절하기 위한 노브.
        # 0 이하면 코드 기본값(SpeedConfig)을 그대로 쓴다.
        self.declare_parameter("cone_speed", 0.0)
        self.declare_parameter("lane_speed", 0.0)
        # 시운전 스위치 (core/race_fsm.py 주석) — 본선은 둘 다 기본값!
        self.declare_parameter("manual_start", False)
        # IMU 를 아예 없는 것으로 치고 달린다. 이 차의 IMU 는 0820 실측에서
        # 자이로·가속도·쿼터니언이 전부 정확히 0 이었다(중력조차 안 보임) — 즉
        # 센서를 못 읽고 빈 메시지만 50Hz 로 나온다. 요 적산이 0 에 묶이므로
        # 요 보충 랩·지름길 완료 판정은 어차피 동작하지 않는다. 이 값을 False 로
        # 두면 그 사실을 알고 가는 것으로 보고 경고를 반복하지 않는다.
        self.declare_parameter("imu_enabled", True)
        # 겐트리 목격 임계 [px] — 트랙별 교정값. replay --sweep-gantry 로 실측한다.
        # 0820 실측(run_0820_0019): 이 실내 배경에서 겐트리가 99.6% 프레임에 잡혀
        # 코드 기본값 5 로는 1899 프레임이 목격으로 세어졌다 (한 바퀴가 여러 바퀴로).
        # 목격이 정확히 2회 나오는 구간 135~184 → 중앙 163.
        self.declare_parameter("min_gantry_span_px", 40)
        self.declare_parameter("maneuvers_enabled", True)
        self.declare_parameter("shortcut_enabled", True)
        # 규정 p.35 추론: 랩2 에서 겐트리를 또렷이 보고도 직좌가 아니었으면
        # 좌회전은 랩3 확정 → 랩3 에서 판독이 침묵해도 지름길로 들어간다.
        # 기본 봉인. 진짜 직좌 bag 으로 페어 검출을 검증한 뒤에만 켤 것.
        self.declare_parameter("lap3_infer_enabled", False)
        # 시운전용: 3 이 아니면 랩 목표를 바꾼다 (99 = 자진 FINISH 없음 —
        # 유령 랩이 남아 있어도 데이터 수집 주행이 안 끊긴다). 본선은 3!
        self.declare_parameter("total_laps", 3)
        # 신호등 대기 중 카메라 노출 (0=사용 안 함). 낮출수록 램프만 남는다.
        self.declare_parameter("signal_exposure", 60)
        # ★ 주행 중 고정 노출 (0 = 자동 노출 유지 = 실측 전 기본값).
        #   자동 노출은 세션 내내 화면 평균 V 를 109~214 로 흔든다(0818 실측).
        #   고정하면 그 요동이 사라져 차선 대비가 프레임마다 같아진다.
        #   **트랙에서 실측해서 넣을 것** — `ros2 run kmu_race exposure_scan`.
        self.declare_parameter("drive_exposure", 0)

        self.debug_gui = bool(self.get_parameter("debug_gui").value)
        self.speed_to_mps = float(self.get_parameter("speed_to_mps").value)
        self.input_timeout = float(self.get_parameter("input_timeout_s").value)
        self.speed_scale = float(self.get_parameter("speed_scale").value)
        self.cone_speed = float(self.get_parameter("cone_speed").value)
        self.lane_speed = float(self.get_parameter("lane_speed").value)
        self.manual_start = bool(self.get_parameter("manual_start").value)
        self.imu_enabled = bool(self.get_parameter("imu_enabled").value)
        self.maneuvers_enabled = bool(self.get_parameter("maneuvers_enabled").value)
        self.signal_exposure = int(self.get_parameter("signal_exposure").value)
        self.drive_exposure = int(self.get_parameter("drive_exposure").value)
        if not self.imu_enabled:
            self.get_logger().warn(
                "⚠️ imu_enabled=False — IMU 를 무시한다. 요 적산 0 고정, "
                "요 보충 랩·지름길 좌회전 완료 판정 없음 (겐트리 목격만으로 랩 계수)"
            )
        if self.manual_start:
            self.get_logger().warn(
                "⚠️ 시운전: manual_start=True — /kmu_race/go 수신 시 출발 (본선 금지!)"
            )
        if not self.maneuvers_enabled:
            self.get_logger().warn(
                "⚠️ 시운전: maneuvers_enabled=False — 회피/추월 트리거 잠금 (본선 금지!)"
            )
        if self.speed_scale != 1.0:
            self.get_logger().warn(
                f"⚠️ 시운전 모드: speed_scale={self.speed_scale:.2f} — 본선은 1.0!"
            )

        # ── 퍼셉션 ────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.signal_det = SignalDetector()
        self.lane_det = LaneDetector()
        self.cone_det = ConeDetector()
        self.lidar_trk = LidarTracker()
        # 워처 go 이벤트 유지창 — 위 _on_image 주석 참조. 0.30s 면 30fps 에서
        # 9프레임, 50Hz 제어로 15틱이라 _green(3틱) 이 확실히 본다.
        self.WATCHER_GO_HOLD_S = 0.30
        self._watcher_go_until = -1e9
        # 색을 쓰지 않는 위치 기반 신호 판정 (perception/signal_watcher.py)
        self.watcher = SignalWatcher()
        self.exposure = CameraExposure()
        self._exposure_mode = ""     # "" | "signal" | "drive"

        # ── core ─────────────────────────────────────────────────
        self.ctx = RaceContext()
        # 트랙 교정값을 랩 트래커에 주입한다 (코드 기본값은 시험용 중립값).
        self.ctx.lap.config.min_gantry_span_px = int(
            self.get_parameter("min_gantry_span_px").value
        )
        self.fsm = RaceFSM()
        self.fsm.maneuvers_enabled = self.maneuvers_enabled
        self.fsm.shortcut_enabled = bool(self.get_parameter("shortcut_enabled").value)
        total_laps = int(self.get_parameter("total_laps").value)
        if total_laps != 3:
            from kmu_race.core import race_context as _rc
            _rc.TOTAL_LAPS = total_laps
            self.get_logger().warn(
                f"⚠️ 시운전: total_laps={total_laps} — 자진 FINISH 없음, 본선은 3!"
            )
        if not self.fsm.shortcut_enabled:
            self.get_logger().warn("⚠️ 시운전: shortcut_enabled=False — 지름길 잠금 (본선 금지!)")
        self.fsm.config.lap3_infer_enabled = bool(
            self.get_parameter("lap3_infer_enabled").value
        )
        if self.fsm.config.lap3_infer_enabled:
            self.get_logger().warn(
                "⚠️ lap3_infer_enabled=True — 랩2 음성 관측이 서면 랩3 은 "
                "판독 없이도 지름길 진입 (규정 p.35 추론, 무신호 진입 리스크 감수)"
            )
        self.fsm.manual_start_mode = self.manual_start
        self.policy = DrivePolicy(PolicyConfig())
        # 구간 목표속도 주입 (0 이하면 코드 기본값 유지)
        _sp = self.policy.config.speed
        if self.cone_speed > 0.0:
            _sp.base["CONE_DRIVE"] = self.cone_speed
        if self.lane_speed > 0.0:
            _sp.base["LANE_DRIVE"] = self.lane_speed
            _sp.max_speed = max(_sp.max_speed, self.lane_speed)
        self.get_logger().info(
            f"속도: 차선 {_sp.base['LANE_DRIVE']:.0f}"
            f"({_sp.base['LANE_DRIVE']*self.speed_to_mps:.2f}m/s) / "
            f"라바콘 {_sp.base['CONE_DRIVE']:.0f}"
            f"({_sp.base['CONE_DRIVE']*self.speed_to_mps:.2f}m/s) / "
            f"상한 {_sp.max_speed:.0f}"
        )
        self.safety = SafetyMonitor()

        # ── 최신 관측 (콜백이 쓰고 타이머가 읽는다) ──────────────
        self._signal = SignalReading()
        self._lane = LaneReading()
        self._cone = ConeReading()
        self._lidar = LidarReading()
        self._lidar_gate = LidarGate()
        self._lidar_gate_t = 0.0
        self._yaw_deg = 0.0
        # 헤딩은 자이로 적분으로 얻는다 — 이 IMU 의 orientation 쿼터니언은
        # 회전을 전혀 담지 못한다는 것이 실차 bag 으로 확정됐다
        # (core/heading.py 모듈 주석의 0817 포렌식 참조).
        self._heading = GyroHeading()
        self._heading_bias_logged = False
        self._last_image_t = 0.0
        self._last_scan_t = 0.0
        # ── 갇힘 감지 (0821 신설) ──────────────────────────────────
        # cone_0821_0016 t54~66: 라이다 값(폭 115cm·깊이 58cm·횡편차 +40cm·
        # 콘 10개)이 12초 동안 **한 글자도 변하지 않았다** = 차가 물리적으로
        # 못 움직인 것이다. 그런데 조향 −26 이 계속 나갔다. 장면이 안 변하면
        # 명령을 아무리 보내도 소용없고, 운영자가 그걸 즉시 알아야 한다.
        self._prev_ranges: Optional[List[float]] = None
        self._scene_delta: float = 1.0        # 스캔 간 중앙 |Δrange| [m]
        self._still_since: Optional[float] = None
        self._stuck: bool = False
        self._stuck_warned_at: float = -1e9
        self._last_cmd_speed: float = 0.0

        # ── ROS I/O ──────────────────────────────────────────────
        # 콜백 그룹: 서로 다른 콜백은 병렬로, '같은' 콜백은 직렬로.
        # 전부 하나의 Reentrant 그룹에 넣으면 _on_image 가 프레임 주기를
        # 넘기는 순간 자기 자신과 병렬 실행되어(cv2 는 GIL 을 놓는다) 낡은
        # 프레임이 최신 관측을 덮어쓰고 yaw 적산 RMW 가 겹친다 (리뷰 T2).
        # 그룹을 콜백별 MutuallyExclusive 로 나누면 CV 가 제어 주기를 막지
        # 않는다는 원래 목적은 그대로 달성된다.
        img_group = MutuallyExclusiveCallbackGroup()
        scan_group = MutuallyExclusiveCallbackGroup()
        imu_group = MutuallyExclusiveCallbackGroup()
        ctrl_group = MutuallyExclusiveCallbackGroup()
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self._on_image,
            sensor_qos,
            callback_group=img_group,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self._on_scan,
            sensor_qos,
            callback_group=scan_group,
        )
        self.create_subscription(
            Imu,
            self.get_parameter("imu_topic").value,
            self._on_imu,
            sensor_qos,
            callback_group=imu_group,
        )

        if self.manual_start:
            self.create_subscription(
                Empty, "kmu_race/go", self._on_manual_go, 1, callback_group=ctrl_group
            )

        self.motor = MotorPublisher(
            self,
            topic=self.get_parameter("motor_topic").value,
            use_xycar_msgs=bool(self.get_parameter("use_xycar_msgs").value),
        )

        hz = float(self.get_parameter("control_hz").value)
        self.create_timer(1.0 / hz, self._control_step, callback_group=ctrl_group)
        # 노출 제어는 v4l2-ctl **subprocess**(read-back 2s 주기, timeout 1.5s)라
        # 제어 루프 안에서 부르면 USB 가 굼뜬 순간 50Hz 조향이 그만큼 멈춘다
        # (0818 저녁 감사). 별도 콜백 그룹의 느린 타이머로 뺀다 — 제어 스레드는
        # 절대 블로킹 호출을 하지 않는다.
        exp_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(0.25, self._apply_exposure, callback_group=exp_group)
        self._last_mode = Mode.INIT
        self._log_throttle = 0.0
        self._calib_warn_t = 0.0
        self._detect_err_t = 0.0
        # 요 교차검증 경고를 목격 횟수당 1번만 내기 위한 래치
        self._yaw_warned_at_sighting = -1
        self._watcher_logged = False
        # 워처 go 가 정면 물체로 보류됐을 때 붙들어 두는 래치 (위 주석 참조)
        self._pending_go = False
        # 노출 read-back 주기 검증용 (대기 중 자동 풀림 실측 — _apply_exposure)
        self._exposure_checked_at = 0.0

        self.get_logger().info(
            f"kmu_race up | debug_gui={self.debug_gui} | control={hz:.0f}Hz"
        )
        self.get_logger().info("규정 3바퀴 / 4분 / 지름길 1회 — 신호등 대기 중")

    # ── 콜백 ──────────────────────────────────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn(f"cv_bridge: {exc}")
            return

        self._signal = self.signal_det.detect(frame)
        # 위치 워처 — 대기 중(정지)에만 트랙을 유지한다
        stationary = self.fsm.state in (Mode.INIT, Mode.WAIT_GREEN)
        try:
            import cv2 as _cv2
            gray = _cv2.cvtColor(frame, _cv2.COLOR_BGR2GRAY)
            wr = self.watcher.update(
                find_lamp_blobs(frame), self._now(),
                stationary=stationary, gray=gray,
            )
            # 차 앞 2m 안에 물체(사람)가 있으면 장면이 동적이다 — 워처 판정 보류
            # (0818 실사고: 앞 0.44m 사람의 옷이 '점등 이동'으로 잡혀 오출발)
            # ★ 보류는 '폐기'가 아니라 '래치'다: 워처 발화는 1회성이라 여기서
            #   버리면 앞이 비어도 영영 출발 못 한다 (0818 2차 실측 — 초록인데
            #   정지). go 를 붙들고 있다가 정면이 비면 그때 통과시킨다.
            front_clear = (
                not math.isfinite(self._lidar.front_dist)
                or self._lidar.front_dist > 2.0
            )
            if wr.go and not front_clear:
                self._pending_go = True
                self.get_logger().warn(
                    f"신호 워처 go 래치 — 정면 {self._lidar.front_dist:.2f}m 물체가 비면 출발"
                )
                wr.go = False
            elif self._pending_go and front_clear:
                self._pending_go = False
                wr.go = True
                self.get_logger().warn("신호 워처 go 래치 해제 — 정면 비었음, 출발")
            # ★ 워처 go 는 런 전체에서 정확히 한 프레임만 True 다
            #   (signal_watcher 의 _fired 원샷). 그런데 FSM 의 _green 은
            #   **연속 3틱**을 요구한다 — 카메라 33.3ms / 제어 20ms 라 한
            #   프레임이 소비되는 틱은 1~2회, 절대 3회가 아니다. 즉 이 경로는
            #   구조적으로 발화할 수 없었다 (0820 감사).
            #   검출 게이트는 그대로 두고 **이벤트 유지시간만** 늘린다.
            if wr.go:
                self._watcher_go_until = self._now() + self.WATCHER_GO_HOLD_S
            go_live = self._now() < self._watcher_go_until
            if go_live or wr.green_left:
                self._signal = replace(
                    self._signal,
                    watcher_go=go_live,
                    watcher_green_left=wr.green_left,
                    lamp_spacing_px=wr.spacing_px,
                )
            if wr.go and not self._watcher_logged:
                self._watcher_logged = True
                self.get_logger().warning(f"신호 워처: {wr.reason}")
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn(f"signal watcher: {exc}")

        # 차선·콘 검출 예외는 노드를 죽이면 안 된다 — 콜백 예외는 executor 를
        # 통해 spin() 을 끝내고 그건 곧 정지(1분 룰)다. 한 프레임 건너뛰고
        # 직전 관측을 유지한다 (input_timeout 이 신선도를 따로 감시한다).
        try:
            self._lane = self.lane_det.detect(frame)
        except Exception as exc:                      # noqa: BLE001
            self._warn_throttled("lane detect", exc)
            return

        # 라바콘 검출은 차선 detect() 가 캐시한 BEV ROI 를 재사용한다 —
        # 여기서 warp 를 또 부르면 프레임당 워프가 2회가 된다 (리뷰 E1).
        # frame + 워프 행렬을 함께 넘겨 원거리 콘 베이스 투영을 켠다 (0818).
        bev_roi = self.lane_det.last_bev_roi
        try:
            self._cone = self.cone_det.detect(
                bev_roi, frame=frame, warp_m=self.lane_det.warp_matrix()
            )
        except Exception as exc:                      # noqa: BLE001
            self._warn_throttled("cone detect", exc)
            self._cone = ConeReading()
        # 라이다 게이트를 합류 (신선할 때만 — 스캔 10~15Hz vs 영상 30Hz).
        # 미터 단위 직접 관측이라 policy 가 이 쪽을 1순위로 쓴다.
        lg = self._lidar_gate
        if (self._now() - self._lidar_gate_t) < 0.4:
            # ★ 게이트가 안 서도 **차 옆 콘 거리는 싣는다** (0821 저녁).
            #   옛 코드는 lg.valid 일 때만 합류시켰다. 그런데 policy 의
            #   폴백 경로(게이트도 콘 픽셀도 없을 때)는 바로 그 near_left/right
            #   로 밀어내기를 한다 — 즉 **그 경로의 밀어내기가 통째로 죽어
            #   있었다**. 실측(cone_0821_0037) 라바콘 구간 스캔의 16.5% 가
            #   게이트 미성립인데, 그 동안 옆 콘이 1cm 로 붙어도 아무 반응이
            #   없었다는 뜻이다.
            self._cone = replace(
                self._cone,
                lidar_gate_valid=bool(lg.valid),
                lidar_gate_depth_m=lg.depth_m,
                lidar_gate_width_m=lg.width_m,
                lidar_gate_lateral_m=lg.lateral_m,
                lidar_cones=lg.cones,
                lidar_near_left_m=lg.near_left_m,
                lidar_near_right_m=lg.near_right_m,
                lidar_cone_pts=lg.points,
                lidar_gate_age_s=max(0.0, self._now() - self._lidar_gate_t),
            )

        self._last_image_t = self._now()

        if self.debug_gui:
            self._draw_debug(frame, bev_roi)

    def _warn_throttled(self, what: str, exc: Exception, period_s: float = 2.0) -> None:
        now = self._now()
        if now - self._detect_err_t >= period_s:
            self._detect_err_t = now
            self.get_logger().error(f"{what} 예외 (프레임 건너뜀): {exc!r}")

    def _on_scan(self, msg: LaserScan) -> None:
        self._lidar = self.lidar_trk.update(
            msg.ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            now=self._now(),
        )
        # 갇힘 감지 — 장면이 안 변하면 차가 못 가고 있는 것이다 (위 주석)
        try:
            self._update_stuck(msg.ranges)
        except Exception as exc:                      # noqa: BLE001
            self._warn_throttled("stuck", exc)

        # 라이다 콘 게이트 (0818 신설) — 카메라와 무관한 제2의 눈.
        # 같은 각도 보정을 그대로 쓴다 (전진축 = 라이다 +124°, 실측 3중 일치).
        # ★ 직전 스캔의 줄 소속을 물려준다 (LidarGate.near_row_side 주석).
        #   신선할 때만 — 오래된 값을 잇는 것보다 모른다고 하는 편이 낫다.
        prev_side = (
            self._lidar_gate.near_row_side
            if (self._now() - self._lidar_gate_t) < 0.4
            else 0
        )
        try:
            self._lidar_gate = find_cone_gate(
                msg.ranges,
                msg.angle_min,
                msg.angle_increment,
                angle_offset=self.lidar_trk.config.angle_offset,
                prev_row_side=prev_side,
            )
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn(f"lidar cone gate: {exc}")
            self._lidar_gate = LidarGate()
        self._lidar_gate_t = self._now()
        self._last_scan_t = self._now()

    # 갇힘 판정 상수 — 0820 실측: 정지 구간의 중앙 |Δrange| 는 0.007~0.010,
    # 주행 중은 0.019~0.111 이다. 0.012 면 둘을 가른다.
    STUCK_DELTA_M = 0.012
    STUCK_SECONDS = 2.5

    def _update_stuck(self, ranges) -> None:
        """연속 스캔의 변화량으로 '차가 못 가고 있다'를 판정한다."""
        cur = [r for r in ranges]
        prev, self._prev_ranges = self._prev_ranges, cur
        if prev is None or len(prev) != len(cur):
            return
        diffs = [
            abs(a - b)
            for a, b in zip(prev, cur)
            if math.isfinite(a) and math.isfinite(b) and 0.3 < a < 6.0 and 0.3 < b < 6.0
        ]
        if len(diffs) < 30:
            return
        diffs.sort()
        self._scene_delta = diffs[len(diffs) // 2]
        now = self._now()
        moving_cmd = self._last_cmd_speed > 0.5
        if self._scene_delta < self.STUCK_DELTA_M and moving_cmd:
            if self._still_since is None:
                self._still_since = now
            elif now - self._still_since >= self.STUCK_SECONDS:
                self._stuck = True
                if now - self._stuck_warned_at >= 2.0:
                    self._stuck_warned_at = now
                    self.get_logger().error(
                        f"⛔ 갇힘 의심 — 라이다 장면이 {now - self._still_since:.1f}초째 "
                        f"거의 안 변한다 (Δ={self._scene_delta*100:.1f}cm/scan) 인데 "
                        f"속도 명령 {self._last_cmd_speed:.0f} 이 나가고 있다. "
                        f"차가 콘·벽에 걸렸을 가능성"
                    )
        else:
            self._still_since = None
            self._stuck = False

    def _apply_exposure(self) -> None:
        """대기 중에는 신호등용 저노출, 주행 중에는 **주행용 고정 노출**.

        0818 이전에는 주행 시 자동 노출로 복귀시켰다. 그런데 자동 노출은
        세션 내내 화면 평균 V 를 109~214 로 흔들고(실측), 밝기 분포가 매
        프레임 바뀌면 차선 검출이 프레임마다 다른 문제를 푼다. drive_exposure
        를 실측해 넣으면 그 요동이 사라진다.

        drive_exposure=0 이면 종전대로 자동 복귀한다 — **실측 전에는 그게
        안전하다.** 지어낸 상수로 주행하느니 자동이 낫다.
        (값 찾기: `ros2 run kmu_race exposure_scan`)
        """
        if self.signal_exposure <= 0 and self.drive_exposure <= 0:
            return
        try:
            self._apply_exposure_impl()
        except Exception as exc:                      # noqa: BLE001
            self._warn_throttled("exposure", exc)

    def _apply_exposure_impl(self) -> None:
        want = (
            "signal"
            if self.fsm.state in (Mode.INIT, Mode.WAIT_GREEN)
            else "drive"
        )
        # ★ 0818 실측: 수동 노출이 스스로 풀린다 (run4 115s / run5 80s 후
        # 밝기 127→192 점프). '한 번 설정'을 믿지 말고 2s 주기로 read-back
        # 검증 → 어긋나면 재적용한다. 주행 중에도 마찬가지다 — 주행 중에
        # 풀리면 대비가 무너져 그 랩을 통째로 잃는다.
        if want == self._exposure_mode:
            now = self._now()
            if now - self._exposure_checked_at >= 2.0:
                self._exposure_checked_at = now
                target = (
                    self.signal_exposure if want == "signal" else self.drive_exposure
                )
                if target > 0 and self.exposure.enforce_manual(target):
                    self.get_logger().warn(
                        f"노출이 풀려 있어 재적용 (수동 {target}, {want})"
                    )
            return
        if want == "signal":
            ok = self.exposure.set_manual(self.signal_exposure)
            msg = f"노출 수동 {self.signal_exposure} (신호등 판독용)"
        elif self.drive_exposure > 0:
            ok = self.exposure.set_drive(self.drive_exposure)
            msg = f"노출 수동 {self.drive_exposure} (주행 고정)"
        else:
            ok = self.exposure.set_auto()
            msg = "노출 자동 복귀 (주행 — drive_exposure 미실측)"
        self._exposure_mode = want
        self._exposure_checked_at = self._now()
        if ok:
            self.get_logger().info(msg)
        else:
            self.get_logger().warn(
                f"노출 제어 실패({self.exposure.last_error}) — 기존 경로로 계속"
            )

    def _on_manual_go(self, _msg: Empty) -> None:
        self.get_logger().warn("MANUAL GO 수신 — 다음 틱에 출발")
        self.fsm.manual_go = True

    def _on_imu(self, msg: Imu) -> None:
        # 누적 헤딩 = 자이로 z 적분 (정지 바이어스 자동 보정 — core/heading.py).
        # 랩 카운팅 교차검증과 지름길 좌회전 완료 판정이 이 누적값을 쓴다.
        g = msg.angular_velocity
        self._yaw_deg = self._heading.update(g.x, g.y, g.z, self._now())
        if self._heading.calibrated and not self._heading_bias_logged:
            self._heading_bias_logged = True
            self.get_logger().info(
                f"자이로 바이어스 캘리브레이션 완료: "
                f"{math.degrees(self._heading.bias_rad_s):+.2f}°/s (축 "
                f"{self._heading.config.axis})"
            )

    # ── 제어 루프 ─────────────────────────────────────────────────
    def _control_step(self) -> None:
        now = self._now()
        camera_ok = (now - self._last_image_t) < self.input_timeout
        # 카메라가 죽으면 마지막 관측이 동결된 채 매 틱 재사용된다 — 특히
        # 동결된 GREEN_LEFT 가 지름길 래치를 계속 갱신하는 사고를 원천 차단
        # 하기 위해, 신선하지 않은 신호 관측은 빈 값으로 강등한다.
        obs = Observation(
            now=now,
            signal=self._signal if camera_ok else SignalReading(),
            lane=self._lane,
            lidar=self._lidar,
            cone=self._cone,
            yaw_deg=self._yaw_deg,
            ego_speed_mps=self.motor.last_speed * self.speed_to_mps,
            camera_ok=camera_ok,
            lidar_ok=(now - self._last_scan_t) < self.input_timeout,
        )

        racing = self.fsm.state not in (Mode.INIT, Mode.WAIT_GREEN)
        decision = self.safety.evaluate(obs, racing=racing)

        # policy 가 보고한 기동 종료를 FSM 에 넘겨준다.
        self.fsm.maneuver_done = self.policy.maneuver_done
        self.fsm.maneuver_aborted = self.policy.maneuver_aborted

        transition = self.fsm.step(obs, self.ctx, decision)
        if transition.changed:
            self.get_logger().warning(
                f"[{transition.source.value} → {transition.target.value}] "
                f"{transition.reason} | lap {self.ctx.lap.current_lap}/3 "
                f"| t={self.ctx.elapsed(now):.1f}s"
            )
            if transition.target is Mode.LANE_DRIVE:
                # 기동에서 빠져나올 때 PID 적분/스무더를 털어 준다.
                self.policy.pid.reset()

        # (노출 제어는 별도 타이머 — 제어 루프에서 subprocess 를 부르지 않는다)
        cmd = self.policy.update(self.fsm.state, obs, self.ctx, decision)
        # speed_scale 은 최종 명령에만 곱한다 — 조향은 그대로 (동일 곡률 유지).
        # last_speed(→ego_speed_mps 추정)는 스케일된 실제 명령을 반영한다.
        self._last_cmd_speed = float(cmd.speed) * self.speed_scale
        self.motor.publish(cmd.steer, cmd.speed * self.speed_scale)

        if self.fsm.state is Mode.FINISH and self._last_mode is not Mode.FINISH:
            # 즉시 정지하지 않는다 — 요 보충 랩의 FINISH 는 결승선 '앞'에서
            # 발화할 수 있어 policy 가 finish_coast_s 동안 선을 넘겨 세운다
            # (core/policy.py FINISH 분기). 정지는 coast 만료 후 policy 몫.
            self._report()
        self._last_mode = self.fsm.state

        # IMU 요 적산 교차검증 — 겐트리 목격을 놓쳤을 가능성을 알린다.
        # 자동 보정은 하지 않는다 (오검으로 랩을 잘못 올리면 실격,
        # race_context.py 설계 노트 참조). 목격 횟수당 1번만 경고.
        if (
            self.fsm.state not in (Mode.INIT, Mode.WAIT_GREEN)
            and self.ctx.lap.yaw_suggests_missed_lap(obs.yaw_deg)
            and self._yaw_warned_at_sighting != self.ctx.lap.sightings
        ):
            self._yaw_warned_at_sighting = self.ctx.lap.sightings
            self.get_logger().warning(
                f"⚠ 요 적산 랩 누락 의심 — 목격 {self.ctx.lap.sightings}회 상태로 "
                f"한 바퀴치(≥{self.ctx.lap.config.yaw_per_lap_deg:.0f}°) 이상 회전. "
                f"겐트리를 놓쳤다면 min_gantry_span_px 가 너무 높은 것이다"
            )

        # 자이로 바이어스 캘리브가 안 끝났으면 요 적산이 0 에 묶인다 → 요 보충
        # 랩·지름길 완료 판정이 전부 무력해진다. 출발 대기 중에 크게 알린다
        # (차를 들고 옮기는 중이면 조용한 3s 창이 안 나온다 — 내려놓고 기다릴 것).
        if (
            self.imu_enabled
            and self.fsm.state is Mode.WAIT_GREEN
            and not self._heading.calibrated
            and now - self._calib_warn_t >= 3.0
        ):
            self._calib_warn_t = now
            self.get_logger().warning(
                "⚠ 자이로 바이어스 캘리브 미완 — 차를 3초 이상 완전히 정지시킬 것 "
                f"(창 기각 {self._heading.calib_rejects}회). 이대로 출발하면 요 랩 보충·"
                "지름길 좌회전 완료 판정이 안 된다"
            )

        # 심판이 화면으로 진행 상황을 봐야 하므로(p.16) 상태를 주기적으로 찍는다.
        if now - self._log_throttle > 1.0:
            self._log_throttle = now
            ln = obs.lane
            lane_flags = (
                ("L" if ln.left_found else "-")
                + ("R" if ln.right_found else "-")
                + ("C" if ln.center_found else "-")
            )
            self.get_logger().info(
                f"{self.fsm.state.value:<12} lap{self.ctx.lap.current_lap}/3 "
                f"t={self.ctx.elapsed(now):5.1f}s "
                f"steer={cmd.steer:+6.1f} v={cmd.speed:5.1f} "
                f"lane={lane_flags} off={ln.offset_px:+5.0f} "
                f"sig={obs.signal.state.value:<10} "
                # ★ 랩 마커 진단 (0820 밤). 이번 주행에서 lap 이 1/3 에 고정
                #   됐는데, 상태줄에 state 만 있어서 '구조는 풀렸는데 점등
                #   검증에서 잘렸다' 와 '구조 자체가 안 풀렸다' 를 사후에
                #   구분할 수 없었다. 그 둘은 조치가 완전히 다르다.
                #   mk = 랩 마커(구조+점등), hs = 하우징 폭(구조만)
                f"mk={obs.signal.lap_marker_px:<3d} hs={obs.signal.housing_span_px:<3d} "
                f"{('rj=' + obs.signal.reject_reason[:22]) if obs.signal.reject_reason else '':<26}"
                f"front={obs.lidar.front_dist:4.2f}m "
                f"{'LG' if obs.cone.lidar_gate_valid else ''}"
                f"{' ⛔STUCK' if self._stuck else ''}"
                f"{'SHORTCUT-USED' if self.ctx.shortcut_used else ''}"
            )

    def _report(self) -> None:
        el = self.ctx.elapsed(self._now())
        self.get_logger().warning("=" * 52)
        self.get_logger().warning(f"  경주 종료  주행시간 {el:.2f}s / 240s 제한")
        self.get_logger().warning(
            f"  랩 출처 — 목격 {self.ctx.lap.sightings}회 "
            f"(요 보충 {self.ctx.lap.yaw_filled} / 콘 보충 {self.ctx.lap.cone_filled})"
        )
        self.get_logger().warning(
            f"  지름길 {'사용' if self.ctx.shortcut_used else '미사용'}"
            + (
                f" (lap {self.ctx.left_signal_seen_on_lap})"
                if self.ctx.shortcut_used
                else ""
            )
        )
        for line in self.ctx.events:
            self.get_logger().warning(f"  {line}")
        self.get_logger().warning("=" * 52)

    # ── 디버그 시각화 (튜닝 전용) ─────────────────────────────────
    def _draw_debug(self, frame, bev_roi) -> None:
        import cv2

        sig = self.signal_det.annotate()
        if sig is not None and sig.size:
            cv2.imshow("signal", sig)

        overlay = bev_roi.copy()
        h_roi, w_roi = overlay.shape[:2]
        cx = w_roi // 2
        cv2.line(overlay, (cx, 0), (cx, h_roi), (80, 80, 80), 1)
        px = int(cx + self._lane.offset_px)
        cv2.line(overlay, (px, 0), (px, h_roi), (0, 220, 0), 2)
        if self._cone.gate_valid:
            gx = int(cx + self._cone.gate_center_px)
            cv2.line(overlay, (gx, 0), (gx, h_roi), (0, 140, 255), 2)
        cv2.putText(
            overlay,
            f"{self.fsm.state.value} lap{self.ctx.lap.current_lap} "
            f"off={self._lane.offset_px:+.0f} cone={self._cone.orange_px}",
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )
        cv2.imshow("bev", overlay)
        cv2.waitKey(1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RaceNode()
    # 4 스레드: 영상·스캔·IMU·제어 + 노출(subprocess, 블로킹 가능) 그룹이 5개라
    # 3 이면 노출 타이머가 블로킹될 때 제어 스레드가 굶을 수 있다.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.motor.stop()
        except Exception:                              # noqa: BLE001
            pass
        # SIGTERM(timeout/kill_race.sh)은 rclpy 시그널 핸들러가 컨텍스트를 먼저
        # 닫는다 — 그 뒤 shutdown() 을 또 부르면 RuntimeError 트레이스백으로
        # 끝난다 (0818 저녁 스모크에서 목격). try_shutdown 은 멱등이다.
        try:
            node.destroy_node()
        except Exception:                              # noqa: BLE001
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
