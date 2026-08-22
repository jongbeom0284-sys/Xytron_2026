"""프리플라이트 점검 — 차에 처음 붙였을 때 한 방에 끝내는 인터페이스 검증.

    ros2 run kmu_race preflight
    ros2 run kmu_race preflight --duration 20
    ros2 run kmu_race preflight --motor-test      # 조향 서보만 움직임 (speed=0)

무엇을 보는가:
  · 토픽이 실제로 있는가, 타입이 맞는가, 몇 Hz 로 오는가
  · **QoS 가 맞는가** — 드라이버가 BEST_EFFORT 인데 우리가 RELIABLE 로 구독하면
    토픽은 '있는데' 콜백이 한 번도 안 불린다. 원인 찾는 데 반나절 날리는 고전 함정.
  · 라이다 0rad 이 정말 정면인가 (`angle_offset` 실측)
  · 신호등 겐트리 span 이 얼마로 잡히는가 (`min_gantry_span_px` 실측)
  · 차선/콘 검출이 지금 조명에서 값을 내는가

이 스크립트는 **차를 움직이지 않는다.** --motor-test 를 줘야 조향 서보만 움직이고,
그 경우에도 speed=0 이라 구동은 안 걸린다.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

OK, WARN, BAD = "✅", "⚠️ ", "❌"


@dataclass
class TopicStat:
    name: str
    type_name: str
    seen: int = 0
    stamps: deque = field(default_factory=lambda: deque(maxlen=400))
    detail: Dict[str, object] = field(default_factory=dict)

    def tick(self, now: float) -> None:
        self.seen += 1
        self.stamps.append(now)

    @property
    def hz(self) -> float:
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return (len(self.stamps) - 1) / span if span > 1e-6 else 0.0


class Preflight(Node):
    def __init__(self, duration: float, motor_test: bool) -> None:
        super().__init__("kmu_race_preflight")
        self.duration = duration
        self.motor_test = motor_test
        self.t0 = time.time()

        self.image = TopicStat("/image_raw", "sensor_msgs/msg/Image")
        self.scan = TopicStat("/scan", "sensor_msgs/msg/LaserScan")
        self.imu = TopicStat("/imu", "sensor_msgs/msg/Imu")
        self.stats = [self.image, self.scan, self.imu]

        # 드라이버가 뭘로 쏘든 받도록 BEST_EFFORT + KEEP_LAST 로 구독한다.
        # (RELIABLE 퍼블리셔 → BEST_EFFORT 구독은 호환되지만 그 반대는 안 된다)
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        from sensor_msgs.msg import Image, Imu, LaserScan

        self.create_subscription(Image, "/image_raw", self._on_image, qos)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos)
        self.create_subscription(Imu, "/imu", self._on_imu, qos)

        self._bridge = None
        self._sig_spans: List[int] = []
        self._sig_states: Dict[str, int] = {}
        self._lane_offsets: List[float] = []
        self._cone_px: List[int] = []
        self._brightness: List[float] = []
        self._yaw_samples: List[float] = []
        from ..core.heading import GyroHeading
        self._heading = GyroHeading()
        self._motor_pub = None

        if motor_test:
            from std_msgs.msg import Float32MultiArray

            self._motor_msg = Float32MultiArray()
            self._motor_pub = self.create_publisher(
                Float32MultiArray, "xycar_motor", 1
            )
            self.create_timer(0.05, self._sweep_servo)
            self._sweep_t0 = time.time()

        self.create_timer(0.5, self._progress)
        self._last_print = 0.0

    # ── 콜백 ──────────────────────────────────────────────────────
    def _on_image(self, msg) -> None:
        self.image.tick(time.time())
        self.image.detail = {
            "width": msg.width,
            "height": msg.height,
            "encoding": msg.encoding,
        }
        # 프레임 몇 장만 실제로 퍼셉션을 돌려 본다 (전부 하면 느려진다)
        if self.image.seen % 5 != 0:
            return
        try:
            if self._bridge is None:
                from cv_bridge import CvBridge

                self._bridge = CvBridge()
            import numpy as np

            from ..perception.cone_detector import ConeDetector
            from ..perception.lane_detector import LaneDetector
            from ..perception.signal_detector import SignalDetector

            if not hasattr(self, "_det"):
                self._det = (SignalDetector(), LaneDetector(), ConeDetector())

            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            self._brightness.append(float(np.mean(frame)))

            sig_det, lane_det, cone_det = self._det
            sig = sig_det.detect(frame)
            if sig.gantry_span_px > 0:
                self._sig_spans.append(sig.gantry_span_px)
            key = sig.state.value if sig.state.value != "UNKNOWN" else (
                f"UNKNOWN({sig.reject_reason})"
            )
            self._sig_states[key] = self._sig_states.get(key, 0) + 1

            lane = lane_det.detect(frame)
            if lane.valid:
                self._lane_offsets.append(lane.offset_px)

            bev = lane_det.last_bev_roi
            self._cone_px.append(cone_det.detect(bev).orange_px)
        except Exception as exc:                       # noqa: BLE001
            self.image.detail["perception_error"] = str(exc)

    def _on_scan(self, msg) -> None:
        self.scan.tick(time.time())
        n = len(msg.ranges)
        finite = [
            (i, r) for i, r in enumerate(msg.ranges)
            if r is not None and math.isfinite(r) and msg.range_min < r < msg.range_max
        ]
        nearest_angle = None
        nearest_r = None
        if finite:
            i, r = min(finite, key=lambda p: p[1])
            nearest_angle = msg.angle_min + i * msg.angle_increment
            nearest_r = r
        # 0 rad 방향의 거리 (정면이라고 가정한 방향)
        idx0 = int(round((0.0 - msg.angle_min) / msg.angle_increment))
        front0 = (
            msg.ranges[idx0]
            if 0 <= idx0 < n and math.isfinite(msg.ranges[idx0])
            else float("inf")
        )
        self.scan.detail = {
            "points": n,
            "angle_min": round(msg.angle_min, 4),
            "angle_max": round(msg.angle_max, 4),
            "angle_increment": round(msg.angle_increment, 6),
            "range_min": round(msg.range_min, 3),
            "range_max": round(msg.range_max, 3),
            "valid_points": len(finite),
            "nearest_r": None if nearest_r is None else round(nearest_r, 3),
            "nearest_angle_deg": (
                None if nearest_angle is None else round(math.degrees(nearest_angle), 1)
            ),
            "range_at_0rad": (
                round(front0, 3) if math.isfinite(front0) else "inf"
            ),
        }

    def _on_imu(self, msg) -> None:
        self.imu.tick(time.time())
        q = msg.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        # 본선 헤딩 경로는 자이로 z 적분이다 (core/heading.py — 0817 포렌식:
        # 이 IMU 의 쿼터니언은 회전을 안 담는다). 여기서는 검사용으로
        # 정지 상태 자이로 평균(=바이어스)과 캘리브 후 잔여 드리프트를 잰다.
        g = msg.angular_velocity
        yaw_deg = self._heading.update(g.x, g.y, g.z, time.time())
        self._yaw_samples.append(yaw_deg)
        self.imu.detail = {
            "quat_norm": round(norm, 4),
            "gyro_z_dps": round(math.degrees(g.z), 2),
            "bias_dps": (
                round(math.degrees(self._heading.bias_rad_s), 2)
                if self._heading.calibrated
                else None
            ),
        }

    def _sweep_servo(self) -> None:
        """조향 서보만 좌우로 흔든다. speed=0 이라 차는 안 나간다."""
        t = time.time() - self._sweep_t0
        angle = 30.0 * math.sin(2.0 * math.pi * 0.5 * t)
        self._motor_msg.data = [float(angle), 0.0]
        self._motor_pub.publish(self._motor_msg)

    def _progress(self) -> None:
        el = time.time() - self.t0
        if el - self._last_print >= 2.0:
            self._last_print = el
            print(
                f"  ... {el:4.1f}/{self.duration:.0f}s  "
                f"img={self.image.seen} scan={self.scan.seen} imu={self.imu.seen}",
                flush=True,
            )

    @property
    def done(self) -> bool:
        return (time.time() - self.t0) >= self.duration

    # ── 리포트 ────────────────────────────────────────────────────
    def report(self) -> bool:
        print()
        print("=" * 68)
        print("  kmu_race 프리플라이트 리포트")
        print("=" * 68)

        problems: List[str] = []
        actions: List[str] = []

        # 퍼블리셔 QoS 확인 — 콜백이 안 불리는 대표 원인
        print("\n[ 토픽 ]")
        for st in self.stats:
            infos = self.get_publishers_info_by_topic(st.name)
            if not infos:
                print(f"  {BAD} {st.name:<14} 퍼블리셔 없음")
                problems.append(f"{st.name} 퍼블리셔 없음")
                continue
            rel = infos[0].qos_profile.reliability.name
            if st.seen == 0:
                print(
                    f"  {BAD} {st.name:<14} 퍼블리셔는 있는데 수신 0건 "
                    f"(pub QoS={rel})"
                )
                problems.append(f"{st.name} 수신 0건")
                actions.append(
                    f"{st.name}: QoS 불일치 의심. 퍼블리셔가 {rel} 이다. "
                    f"race_node 의 구독 QoS 를 맞출 것"
                )
                continue
            mark = OK if st.hz > 5.0 else WARN
            print(
                f"  {mark} {st.name:<14} {st.hz:5.1f} Hz  "
                f"{st.seen:5d}건  pub QoS={rel}"
            )
            if st.hz <= 5.0:
                problems.append(f"{st.name} 레이트 낮음 ({st.hz:.1f}Hz)")

        # ── 카메라 ──
        print("\n[ 카메라 ]")
        if self.image.seen:
            d = self.image.detail
            print(f"  해상도 {d.get('width')}x{d.get('height')}  "
                  f"인코딩 {d.get('encoding')}")
            if self._brightness:
                b = sum(self._brightness) / len(self._brightness)
                mark = OK if 40 < b < 200 else WARN
                print(f"  {mark} 평균 밝기 {b:.0f}/255")
                if b <= 40:
                    actions.append("카메라가 너무 어둡다 — 노출(exposure) 조정 필요")
                elif b >= 200:
                    actions.append("카메라 과노출 — 흰색 차선 임계가 안 먹는다")
            if "perception_error" in d:
                print(f"  {BAD} 퍼셉션 예외: {d['perception_error']}")
                problems.append("퍼셉션 예외")
        else:
            print(f"  {BAD} 프레임 없음")

        # ── 신호등 (gantry_span = 채도 초록 램프 블롭 폭 — 8/13 재설계) ──
        print("\n[ 신호등 — min_gantry_span_px 실측 ]")
        if self._sig_spans:
            sp = sorted(self._sig_spans)
            lo, mid, hi = sp[0], sp[len(sp) // 2], sp[-1]
            print(f"  {OK} span  최소 {lo}  중앙 {mid}  최대 {hi}  "
                  f"({len(sp)}프레임)")
            print(f"     → 지금 위치에서 잡히는 초록 블롭 폭이 약 {mid}px 이다.")
            print(f"     → 임계값은 여기서 정하지 말 것 — 3바퀴 bag 을 받아")
            print(f"        tune_laps.sh(--sweep-gantry) 로 재산출한다 (실측:")
            print(f"        진짜 통과 32px vs 최악 오검 14px — 낮게 잡으면 위험)")
        else:
            print(f"  {WARN} 초록 블롭 미검출 — 정상일 수 있다: 빨간불 대기 중이거나")
            print(f"        신호등(초록 점등)이 화면에 없으면 span 은 구조적으로 0 이다")
            actions.append(
                "신호등이 초록으로 점등된 상태를 화면에 넣고 다시 실행. "
                "그래도 0 이면 SignalConfig 의 green_h/chroma_min_s/blob_min_v 확인 "
                "(원 반지름 파라미터는 블롭 경로와 무관하다)"
            )
        if self._sig_states:
            print("  판정 분포:")
            for k, v in sorted(
                self._sig_states.items(), key=lambda kv: -kv[1]
            )[:5]:
                print(f"     {v:4d}회  {k}")

        # ── 차선 / 콘 ──
        print("\n[ 차선 · 라바콘 ]")
        if self._lane_offsets:
            avg = sum(self._lane_offsets) / len(self._lane_offsets)
            print(f"  {OK} 차선 검출 {len(self._lane_offsets)}프레임  "
                  f"평균 offset {avg:+.0f}px")
        else:
            print(f"  {WARN} 차선 미검출 — 트랙 위에 놓고 다시 볼 것")
        if self._cone_px:
            mx = max(self._cone_px)
            print(f"  주황 픽셀 최대 {mx}  "
                  f"(구간 진입 임계 min_section_px 기본 900)")
            if mx > 900:
                print(f"     {WARN} 지금 시야에 이미 900 을 넘는 주황이 있다 — "
                      f"콘이 없다면 임계를 올릴 것")

        # ── 라이다 ──
        print("\n[ 라이다 — angle_offset 실측 ]")
        if self.scan.seen:
            d = self.scan.detail
            print(f"  포인트 {d['points']}  유효 {d['valid_points']}  "
                  f"범위 {d['range_min']}~{d['range_max']}m")
            print(f"  angle_min {d['angle_min']} rad  "
                  f"increment {d['angle_increment']} rad")
            print(f"  0rad 방향 거리 = {d['range_at_0rad']} m")
            na = d["nearest_angle_deg"]
            if na is not None:
                print(f"  최근접 물체: {d['nearest_r']}m @ {na:+.1f}°")
                if abs(na) > 8.0:
                    print(f"     {WARN} 차 정면 1m 앞에 물체를 놓고 이 값이 0° 근처가")
                    print(f"        아니면 LidarConfig.angle_offset 에 "
                          f"{-math.radians(na):+.3f} 를 넣을 것")
                else:
                    print(f"     {OK} 정면 물체가 0° 부근 — angle_offset 보정 불필요")
        else:
            print(f"  {BAD} 스캔 없음")

        # ── IMU ──
        print("\n[ IMU ]")
        if self.imu.seen:
            d = self.imu.detail
            bias = d.get("bias_dps")
            gz = d.get("gyro_z_dps", 0.0)
            if bias is not None:
                print(f"  {OK} 자이로 z 살아 있음 (현재 {gz:+.2f}°/s, "
                      f"정지 바이어스 {bias:+.2f}°/s 자동 보정)")
                if len(self._yaw_samples) > 10:
                    drift = self._yaw_samples[-1] - self._yaw_samples[0]
                    el = self.duration
                    print(f"     캘리브 후 잔여 헤딩 드리프트 {drift:+.1f}° / {el:.0f}s "
                          f"({drift / max(1.0, el) * 60:+.1f}°/min)")
                    if abs(drift / max(1.0, el) * 60) > 10.0:
                        print(f"     {WARN} 잔여 드리프트가 크다 — 랩 교차검증·지름길 "
                              f"요 판정 여유 확보 (shortcut_timeout_s 가 안전망)")
                        problems.append("자이로 잔여 드리프트 과대")
            else:
                print(f"  {WARN} 바이어스 캘리브레이션 미완 (검사 시간 부족) — "
                      f"현재 자이로 z {gz:+.2f}°/s")
            # 참고 정보: 이 IMU 의 쿼터니언 융합은 죽어 있음이 실측됐다 (0817).
            # 본선 경로는 자이로 적분이므로 quat_norm 은 진단 표기만 한다.
            print(f"     (orientation quat_norm={d.get('quat_norm')} — "
                  f"융합 헤딩은 사용하지 않음, core/heading.py 참조)")
        else:
            print(f"  {WARN} /imu 없음")
            actions.append(
                "/imu 가 없다 → 랩 카운팅 요 교차검증 불가, 지름길 좌회전은 "
                "타임아웃으로만 종료. race_node 는 동작하지만 안전망이 하나 준다"
            )

        # ── 모터 ──
        print("\n[ 모터 ]")
        subs = self.count_subscribers("xycar_motor")
        if subs > 0:
            print(f"  {OK} xycar_motor 구독자 {subs}개 (모터 드라이버 살아 있음)")
        else:
            print(f"  {BAD} xycar_motor 구독자 없음 — 모터 드라이버가 안 떠 있다")
            problems.append("모터 드라이버 미기동")

        # 경쟁 퍼블리셔 검사 — kobot 대시보드(WASD 수동주행)가 같은 토픽에
        # 발행한다. 본선 중 브라우저 세션이 남아 있으면 포커스를 잃는 순간
        # [0,0] 을 쏴서 우리 명령과 충돌한다. 우리(race/preflight) 외의
        # 퍼블리셔가 보이면 경고한다.
        pubs = self.get_publishers_info_by_topic("xycar_motor")
        rivals = sorted(
            {p.node_name for p in pubs}
            - {self.get_name(), "kmu_race"}
        )
        if rivals:
            print(f"  {WARN} xycar_motor 에 다른 퍼블리셔 존재: {', '.join(rivals)}")
            actions.append(
                "xycar_motor 경쟁 퍼블리셔 감지 — 대시보드(WASD) 브라우저 탭을 "
                "전부 닫고, 본선에서는 우리 노드만 발행하는지 재확인할 것"
            )
        if self.motor_test:
            print(f"  {OK} 조향 서보 스윕 발행 완료 (speed=0). 바퀴가 좌우로 움직였는가?")
        else:
            print("     (--motor-test 를 주면 조향 서보를 흔들어 확인한다)")

        # ── 결론 ──
        print("\n" + "=" * 68)
        if problems:
            print(f"  {BAD} NO-GO — 문제 {len(problems)}건")
            for p in problems:
                print(f"     · {p}")
        else:
            print(f"  {OK} GO — 인터페이스 정상. 다음은 bag 녹화다:")
            print("       ros2 bag record /image_raw/compressed /scan /imu -o run1")
            print("       (raw 는 3바퀴에 4GB+ — compressed 로 찍는다. replay 가 둘 다 읽음)")
        if actions:
            print("\n  다음 조치:")
            for a in actions:
                print(f"     → {a}")
        print("=" * 68)
        return not problems


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="kmu_race 프리플라이트 점검")
    parser.add_argument("--duration", type=float, default=10.0, help="관측 시간(초)")
    parser.add_argument(
        "--motor-test",
        action="store_true",
        help="조향 서보를 흔들어 본다 (speed=0, 차는 움직이지 않음)",
    )
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Preflight(args.duration, args.motor_test)
    print(f"\n{args.duration:.0f}초 관측합니다. 차를 트랙 위, 신호등이 보이는 "
          f"출발 위치에 두세요.\n")
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
        ok = node.report()
    except KeyboardInterrupt:
        ok = False
    finally:
        if node._motor_pub is not None:
            from std_msgs.msg import Float32MultiArray

            stop = Float32MultiArray()
            stop.data = [0.0, 0.0]
            for _ in range(3):
                node._motor_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
