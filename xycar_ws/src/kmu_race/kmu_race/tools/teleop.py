"""터미널 WASD 텔레옵 — 웹 대시보드 대신 ssh 터미널에서 수동주행.

    ros2 run kmu_race teleop                          # 실차 (xycar_motor)
    ros2 run kmu_race teleop --topic /kmu_smoke_motor # 무해 벤치 테스트

키:
    w       전진 래치 (한 번 누르면 계속 전진 — 후진 중이면 먼저 정지)
    s       정지 (전진 중일 때) / 후진 래치 (정지 상태일 때)
    a / d   좌 / 우 조향 — **누르는 동안 각도가 서서히 깊어진다**
            (짧게 톡 = 살짝, 길게 = 최대. 떼면 서서히 중앙 복귀)
    A / D   조향 래치 (Shift) — 최대각으로 고정
    f       현재 각도 그대로 고정 (긴 코너에서 각 유지)
    x       중앙 복귀 (래치 해제)
    1 ~ 9   순항속도 = 숫자 × 2  (기본 8, 주행 중 변경 즉시 반영)
    , / .   최대 조향각 -5 / +5  (상한 없음, 기본 40)
    [ / ]   조향 속도 -20 / +20 °/s  (느리게 / 빠르게, 기본 90)
    space   즉시 정지
    q       정지 후 종료

조작 설계 (8/17 실주행 피드백 2차 반영):
  터미널의 OS 자동반복은 '마지막으로 누른 키'만 반복 전송한다 — 그래서
  W 홀드 + D 홀드 동시 조작이 원리적으로 불가능하다 (W 이벤트가 끊겨
  데드맨이 전진을 꺼버림). 해법: 전진/후진은 **래치**(토글)로, 조향만
  홀드로 둔다. 스트리밍되는 키가 조향 하나뿐이라 '달리면서 조향'이 된다.

  ★ 조향 레이트 리밋 (2차 피드백: "a/d 가 너무 확 꺾여 불안정"):
    키를 누르면 목표각으로 **점진 이동**한다 (기본 90°/s — 최대 40°까지
    0.44s). 각도 범위는 그대로다(제한 아님) — 끝까지 누르면 끝까지 간다.
    실차 조향 서보는 명령을 즉시 따라가므로, 계단 입력은 그대로 차체
    요 레이트의 계단이 되어 뒤가 털린다. 램프를 두면 같은 최대각이어도
    차가 물고 들어간다. 톡톡 치면 미세각, 지그시 누르면 깊은 각 — 아날로그
    스틱에 가까운 조작감이 된다.
    복귀는 조금 더 빠르게(1.6배) — 중앙 복귀는 안정화 방향이라 늦출 이유가
    없고, 슬라럼에서 좌↔우 전환이 굼떠지는 것을 막는다.

  조향 홀드는 2단계 데드맨 — 첫 입력은 자동반복 초기 지연(~0.5s)을 덮는
  0.6s 창, 반복 중(이벤트 간격 <0.2s)엔 0.4s 창. 손을 떼면 창이 닫히고
  중앙으로 램프 복귀한다. 키 이벤트는 다음 틱을 기다리지 않고 즉시 발행한다.

  래치의 안전망: **아무 키 입력이 IDLE_STOP_S(5s) 동안 없으면 자동 정지**
  (조향 중이면 이벤트가 계속 흐르므로 걸리지 않는다 — 긴 직선에서는 5초
  안에 아무 키나 한 번 눌러 주면 된다. w 재입력이 자연스럽다).
  ssh 가 끊기면 발행 자체가 멎고 VESC 명령 타임아웃이 모터를 세운다.
  종료 시(정상/Ctrl-C 모두) 정지 명령을 5회 발행한다.
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

DEADMAN_TAP_S = 0.60     # 조향: 첫 입력·단발 탭 창 (자동반복 초기 지연 커버)
DEADMAN_REPEAT_S = 0.40  # 조향: 자동반복 중 창 — ssh/WiFi 딸꾹(~0.3s)을 흡수
REPEAT_GAP_S = 0.20      # 이벤트 간격이 이보다 짧으면 '반복 중'으로 본다
IDLE_STOP_S = 5.0        # 래치 안전망: 이 시간 동안 아무 입력 없으면 정지
RATE_HZ = 25.0
REVERSE_FACTOR = 0.8
RETURN_FACTOR = 1.6      # 중앙 복귀는 진입보다 이 배수만큼 빠르다
RAMP_STEP = 20.0         # [ / ] 한 번에 바뀌는 조향 속도 [°/s]
RAMP_MIN = 20.0          # 이보다 느리면 조작이 불가능해진다


class _HoldKey:
    """자동반복 기반 '누르는 동안' 판정 (2단계 데드맨)."""

    def __init__(self) -> None:
        self.last_t = -1e9
        self.window = 0.0

    def press(self, now: float) -> None:
        gap = now - self.last_t
        self.window = DEADMAN_REPEAT_S if gap < REPEAT_GAP_S else DEADMAN_TAP_S
        self.last_t = now

    def active(self, now: float) -> bool:
        return (now - self.last_t) < self.window

    def release(self) -> None:
        self.last_t = -1e9


def _bar(angle: float, span: float, width: int = 21) -> str:
    """조향 위치 막대. 중앙이 직진, 좌우 끝이 ±span."""
    if span <= 0.0:
        span = 1.0
    half = width // 2
    pos = int(round(max(-1.0, min(1.0, angle / span)) * half))
    cells = ["-"] * width
    cells[half] = "|"
    cells[half + pos] = "#"
    return "".join(cells)


def main(argv=None) -> None:
    p = argparse.ArgumentParser(description="kmu_race 터미널 텔레옵")
    p.add_argument("--topic", default="xycar_motor")
    p.add_argument("--speed", type=float, default=8.0, help="초기 순항속도")
    p.add_argument("--steer", type=float, default=40.0, help="최대 조향각")
    p.add_argument(
        "--ramp", type=float, default=90.0, help="조향 변화 속도 [deg/s]"
    )
    args, _ = p.parse_known_args(argv)

    rclpy.init()
    node = Node("kmu_teleop")
    pub = node.create_publisher(Float32MultiArray, args.topic, 1)
    msg = Float32MultiArray()

    cruise = float(args.speed)
    steer_deg = float(args.steer)          # 최대 조향각 (상한 없음)
    ramp_deg_s = max(RAMP_MIN, float(args.ramp))
    left, right = _HoldKey(), _HoldKey()
    steer_latch: Optional[float] = None    # 고정 각도 [deg] 또는 None
    steer_actual = 0.0                     # 실제 발행 중인 각도 (램프 적용)
    drive_dir = 0                          # +1 전진 래치 / 0 정지 / -1 후진 래치
    last_key_t = time.monotonic()          # 래치 안전망용 — 마지막 '아무 키' 시각
    last_tick = time.monotonic()           # 램프 적분용
    idle_stopped_note = False

    fd = sys.stdin.fileno()
    is_tty = sys.stdin.isatty()
    old_attr = None
    if is_tty:
        old_attr = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        no_echo = termios.tcgetattr(fd)
        no_echo[3] &= ~termios.ECHO
        termios.tcsetattr(fd, termios.TCSADRAIN, no_echo)

    print(
        f"teleop → {args.topic}\n"
        f"  w 전진래치 · s 정지/후진래치 · a/d 조향(누른 만큼 깊어짐) · "
        f"A/D 최대각 래치 · f 현재각 고정 · x 중앙\n"
        f"  최대 ±{steer_deg:.0f}° (,/. 조절, 상한 없음) · "
        f"조향속도 {ramp_deg_s:.0f}°/s ([/] 조절) · "
        f"속도 {cruise:.0f} (1~9) · space 정지 · q 종료\n"
        f"  ★ a/d 는 톡 치면 살짝, 지그시 누르면 최대각까지 부드럽게 들어갑니다.\n"
        f"  단, {IDLE_STOP_S:.0f}초간 아무 키도 없으면 자동 정지 — 긴 직선에선 "
        f"아무 키나 한 번씩.",
        flush=True,
    )

    def target_angle(now: float) -> float:
        """키 상태가 요구하는 목표 조향각."""
        l_on = left.active(now)
        r_on = right.active(now)
        if l_on and not r_on:
            return -steer_deg
        if r_on and not l_on:
            return +steer_deg
        # 홀드 입력이 없을 때만 래치가 적용된다 (홀드가 우선)
        if steer_latch is not None:
            return steer_latch
        return 0.0

    def ramp(actual: float, target: float, dt: float) -> float:
        """레이트 리밋 — 목표까지 정해진 속도로만 움직인다."""
        if dt <= 0.0:
            return actual
        diff = target - actual
        if diff == 0.0:
            return actual
        # 중앙으로 가는 '구간'은 더 빠르게 — 안정화 방향이라 늦출 이유가 없다.
        # 목표가 아니라 '지금 이 스텝이 |각|을 줄이는가'로 판단한다: 좌 최대 →
        # 우 최대 전환(슬라럼)에서도 중앙까지는 빠르게 넘어온다.
        toward_center = (actual > 0.0 and diff < 0.0) or (actual < 0.0 and diff > 0.0)
        rate = ramp_deg_s * (RETURN_FACTOR if toward_center else 1.0)
        step = rate * dt
        if abs(diff) <= step:
            return target
        return actual + (step if diff > 0.0 else -step)

    def compute(now: float, dt: float) -> tuple:
        nonlocal steer_actual
        speed = 0.0
        if drive_dir > 0:
            speed = cruise
        elif drive_dir < 0:
            speed = -cruise * REVERSE_FACTOR
        steer_actual = ramp(steer_actual, target_angle(now), dt)
        return steer_actual, speed

    def publish(angle: float, speed: float) -> None:
        msg.data = [float(angle), float(speed)]
        pub.publish(msg)

    try:
        period = 1.0 / RATE_HZ
        last_status = 0.0
        while True:
            readable, _, _ = select.select([sys.stdin], [], [], period)
            now = time.monotonic()
            dt = max(0.0, now - last_tick)
            last_tick = now
            if readable:
                ch = sys.stdin.read(1)
                if ch == "" or ch == "q":          # EOF(파이프) 또는 종료
                    break
                last_key_t = now
                idle_stopped_note = False
                if ch == " ":
                    drive_dir = 0
                elif ch in ("w", "W"):
                    # 후진 중이면 안전하게 정지부터, 정지 상태면 전진 래치
                    drive_dir = 0 if drive_dir < 0 else 1
                elif ch in ("s", "S"):
                    # 전진 중이면 정지(브레이크), 정지 상태면 후진 래치
                    drive_dir = 0 if drive_dir > 0 else -1
                elif ch == "a":
                    left.press(now)
                    steer_latch = None            # 홀드를 쓰면 래치는 풀린다
                elif ch == "d":
                    right.press(now)
                    steer_latch = None
                elif ch == "A":                  # Shift+a = 좌 최대각 래치
                    steer_latch = -steer_deg
                elif ch == "D":                  # Shift+d = 우 최대각 래치
                    steer_latch = +steer_deg
                elif ch in ("f", "F"):           # 현재 각도 그대로 고정
                    steer_latch = steer_actual
                    left.release()
                    right.release()
                elif ch in ("x", "X"):           # 래치 해제(중앙 복귀)
                    steer_latch = None
                    left.release()
                    right.release()
                elif ch == ",":
                    steer_deg = max(5.0, steer_deg - 5.0)
                elif ch == ".":
                    steer_deg = steer_deg + 5.0    # 상한 없음 (사용자 지시)
                elif ch == "[":
                    ramp_deg_s = max(RAMP_MIN, ramp_deg_s - RAMP_STEP)
                elif ch == "]":
                    ramp_deg_s = ramp_deg_s + RAMP_STEP   # 상한 없음
                elif ch.isdigit() and ch != "0":
                    cruise = float(ch) * 2.0
                # 키 이벤트는 다음 틱을 기다리지 않고 즉시 반영한다 (반응성).
                # 조향은 램프가 적용되므로 '즉시'는 속도/방향 쪽에서 체감된다.
                angle, speed = compute(now, dt)
                publish(angle, speed)
                continue

            # 래치 안전망: 입력이 끊긴 채 방치되면 정지
            if drive_dir != 0 and (now - last_key_t) > IDLE_STOP_S:
                drive_dir = 0
                if not idle_stopped_note:
                    idle_stopped_note = True
                    print(
                        f"\n  ⏸ {IDLE_STOP_S:.0f}초 무입력 — 자동 정지 "
                        f"(w 로 재출발)",
                        flush=True,
                    )

            angle, speed = compute(now, dt)
            publish(angle, speed)

            if now - last_status > 0.2:
                last_status = now
                state = (
                    "FWD" if drive_dir > 0 else "REV" if drive_dir < 0 else "stop"
                )
                lock = "L" if steer_latch is not None else " "
                print(
                    f"\r  {_bar(angle, steer_deg)}{lock} "
                    f"angle={angle:+6.1f}/{steer_deg:.0f}  "
                    f"speed={speed:+5.1f}  ramp={ramp_deg_s:3.0f}°/s  "
                    f"[{state}]   ",
                    end="",
                    flush=True,
                )
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            publish(0.0, 0.0)
            time.sleep(0.03)
        if is_tty and old_attr is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
        print("\n정지 명령 5회 발행 — teleop 종료", flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
