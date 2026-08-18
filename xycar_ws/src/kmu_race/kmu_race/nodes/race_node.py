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
from typing import Optional

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
        # 시운전 스위치 (core/race_fsm.py 주석) — 본선은 둘 다 기본값!
        self.declare_parameter("manual_start", False)
        self.declare_parameter("maneuvers_enabled", True)
        self.declare_parameter("shortcut_enabled", True)
        # 시운전용: 3 이 아니면 랩 목표를 바꾼다 (99 = 자진 FINISH 없음 —
        # 유령 랩이 남아 있어도 데이터 수집 주행이 안 끊긴다). 본선은 3!
        self.declare_parameter("total_laps", 3)
        # 신호등 대기 중 카메라 노출 (0=사용 안 함). 낮출수록 램프만 남는다.
        # io/camera_exposure.py 주석 참조 — 주행 중에는 절대 적용되지 않는다.
        self.declare_parameter("signal_exposure", 60)

        self.debug_gui = bool(self.get_parameter("debug_gui").value)
        self.speed_to_mps = float(self.get_parameter("speed_to_mps").value)
        self.input_timeout = float(self.get_parameter("input_timeout_s").value)
        self.speed_scale = float(self.get_parameter("speed_scale").value)
        self.manual_start = bool(self.get_parameter("manual_start").value)
        self.maneuvers_enabled = bool(self.get_parameter("maneuvers_enabled").value)
        self.signal_exposure = int(self.get_parameter("signal_exposure").value)
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
        # 색을 쓰지 않는 위치 기반 신호 판정 (perception/signal_watcher.py)
        self.watcher = SignalWatcher()
        self.exposure = CameraExposure()
        self._exposure_mode = ""     # "" | "signal" | "drive"

        # ── core ─────────────────────────────────────────────────
        self.ctx = RaceContext()
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
        self.fsm.manual_start_mode = self.manual_start
        self.policy = DrivePolicy(PolicyConfig())
        self.safety = SafetyMonitor()

        # ── 최신 관측 (콜백이 쓰고 타이머가 읽는다) ──────────────
        self._signal = SignalReading()
        self._lane = LaneReading()
        self._cone = ConeReading()
        self._lidar = LidarReading()
        self._yaw_deg = 0.0
        # 헤딩은 자이로 적분으로 얻는다 — 이 IMU 의 orientation 쿼터니언은
        # 회전을 전혀 담지 못한다는 것이 실차 bag 으로 확정됐다
        # (core/heading.py 모듈 주석의 0817 포렌식 참조).
        self._heading = GyroHeading()
        self._heading_bias_logged = False
        self._last_image_t = 0.0
        self._last_scan_t = 0.0

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
        self._last_mode = Mode.INIT
        self._log_throttle = 0.0
        # 요 교차검증 경고를 목격 횟수당 1번만 내기 위한 래치
        self._yaw_warned_at_sighting = -1
        self._watcher_logged = False
        # 워처 go 가 정면 물체로 보류됐을 때 붙들어 두는 래치 (위 주석 참조)
        self._pending_go = False

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
            if wr.go or wr.green_left:
                self._signal = replace(
                    self._signal,
                    watcher_go=wr.go,
                    watcher_green_left=wr.green_left,
                    lamp_spacing_px=wr.spacing_px,
                )
            if wr.go and not self._watcher_logged:
                self._watcher_logged = True
                self.get_logger().warning(f"신호 워처: {wr.reason}")
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn(f"signal watcher: {exc}")

        self._lane = self.lane_det.detect(frame)

        # 라바콘 검출은 차선 detect() 가 캐시한 BEV ROI 를 재사용한다 —
        # 여기서 warp 를 또 부르면 프레임당 워프가 2회가 된다 (리뷰 E1).
        bev_roi = self.lane_det.last_bev_roi
        self._cone = self.cone_det.detect(bev_roi)

        self._last_image_t = self._now()

        if self.debug_gui:
            self._draw_debug(frame, bev_roi)

    def _on_scan(self, msg: LaserScan) -> None:
        self._lidar = self.lidar_trk.update(
            msg.ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            now=self._now(),
        )
        self._last_scan_t = self._now()

    def _apply_exposure(self) -> None:
        """대기 중에는 노출을 낮추고, 출발하면 즉시 되돌린다.

        주행 중 노출을 건드리면 차선 흰색 임계(V 243~255)가 무너지므로
        WAIT_GREEN 을 벗어나는 순간 자동으로 복귀시킨다.
        """
        if self.signal_exposure <= 0:
            return
        want = (
            "signal"
            if self.fsm.state in (Mode.INIT, Mode.WAIT_GREEN)
            else "drive"
        )
        if want == self._exposure_mode:
            return
        if want == "signal":
            ok = self.exposure.set_manual(self.signal_exposure)
            msg = f"노출 수동 {self.signal_exposure} (신호등 판독용)"
        else:
            ok = self.exposure.set_auto()
            msg = "노출 자동 복귀 (주행)"
        self._exposure_mode = want
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

        self._apply_exposure()
        cmd = self.policy.update(self.fsm.state, obs, self.ctx, decision)
        # speed_scale 은 최종 명령에만 곱한다 — 조향은 그대로 (동일 곡률 유지).
        # last_speed(→ego_speed_mps 추정)는 스케일된 실제 명령을 반영한다.
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

        # 심판이 화면으로 진행 상황을 봐야 하므로(p.16) 상태를 주기적으로 찍는다.
        if now - self._log_throttle > 1.0:
            self._log_throttle = now
            self.get_logger().info(
                f"{self.fsm.state.value:<12} lap{self.ctx.lap.current_lap}/3 "
                f"t={self.ctx.elapsed(now):5.1f}s "
                f"steer={cmd.steer:+6.1f} v={cmd.speed:5.1f} "
                f"sig={obs.signal.state.value:<10} "
                f"front={obs.lidar.front_dist:4.2f}m "
                f"{'SHORTCUT-USED' if self.ctx.shortcut_used else ''}"
            )

    def _report(self) -> None:
        el = self.ctx.elapsed(self._now())
        self.get_logger().warning("=" * 52)
        self.get_logger().warning(f"  경주 종료  주행시간 {el:.2f}s / 240s 제한")
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
    executor = MultiThreadedExecutor(num_threads=3)
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
