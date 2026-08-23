"""정지차량(라이다) -> 이동차량(욜로) 인계 (0823 밤).

코스 한 바퀴:
    직진 -> 콘 -> 정지차량 추월 -> 코너 -> 이동차량 추월 -> 코너

계약:
  콘 탈출      라이다만 켠다. 욜로는 이 동안 끈다.
  라이다 확정  방향 한 번 정하고 **라이다 즉시 끈다**. 계산한 시간만큼 유지.
  유지 끝      정지차량 통과. 욜로를 켠다 (이동차량 구간).
  못 보면      obs_arm_sec 뒤 라이다 접고 욜로로 넘어간다.

왜 확정하는 순간 끄는가 — 223056 실측:
  확정 뒤에도 매 스캔 다시 판정해서, 지나치는 콘·벽이 번갈아 가장 가까운
  덩어리가 되며 좌/우가 뒤집혔다 (f285 R 0.73m -> f286 R 0.63m -> f287 L 0.42m).
  욜로도 같이 켜져 있어 둘이 서로 다른 방향을 냈고, 회피 오프셋이 20Hz 로
  -90/+90 을 왕복하다 차선을 놓쳤다 (drivelog 523~558행 basis=none).
"""
import json
import threading
import time

from race_last.yolop_drive_node import YolopDriveNode


class Log:
    def info(self, *a, **k): pass
    def warning(self, *a, **k): pass
    def error(self, *a, **k): pass


class Link:
    active = False
    approach = False


class Fake:
    """_on_obstacle / _check_cone_exit / _vehicle_needed 가 쓰는 것만."""

    def __init__(self, mps=1.6):
        self.lidar_avoid_enabled = True
        self.obs_confirm = 2
        self.obs_arm_sec = 10.0
        self.vehicle_window_sec = 30.0
        self.vehicle_off_after_pass = False
        self._vehicle_window_until = 0.0
        self._vehicle_passed = False
        self._vehicle_force_until = 0.0
        self._cone_link = Link()
        self._cone_was_active = False
        self._cone_exited = False
        self._go_locked_until = 0.0
        self._lock_kind = 'green'
        self.speed_units = mps / 0.08
        self.mps_per_unit = 0.08
        self._obs = None
        self._obs_recent = []
        self._obs_hold_until = 0.0
        self._obs_armed = False
        self._obs_arm_until = 0.0
        self._avoid_side = None
        self._avoid_until = 0.0
        self._avoid_rearm_until = 0.0
        self._lock = threading.Lock()
        self._log = Log()

    def get_logger(self):
        return self._log

    def feed(self, side, dist, n=1):
        msg = type('M', (), {})()
        msg.data = json.dumps(dict(side=side, dist=dist, y=0.0, n=n, groups=1))
        YolopDriveNode._on_obstacle(self, msg)

    def cone_exit(self):
        """콘 모드가 켜졌다가 꺼지는 순간을 재현한다."""
        self._cone_link.active = True
        YolopDriveNode._check_cone_exit(self, time.monotonic())
        self._cone_link.active = False
        YolopDriveNode._check_cone_exit(self, time.monotonic())

    def yolo_on(self):
        return YolopDriveNode._vehicle_needed(self, time.monotonic())


def test_lidar_off_before_cone_exit():
    """콘 탈출 전에는 라이다를 쓰지 않는다 (직진·콘 구간)."""
    f = Fake()
    f._vehicle_window_until = time.monotonic() + 30.0   # 창이 열려 있어도
    f.feed('R', 1.0)
    f.feed('R', 1.0)
    assert f._obs_armed is False
    assert f._avoid_side is None


def test_cone_exit_arms_lidar_and_mutes_yolo():
    """콘 탈출은 라이다를 켜고, 그 동안 욜로는 끈다."""
    f = Fake()
    f.cone_exit()
    assert f._obs_armed is True
    assert f.yolo_on() is False, '라이다가 보는 동안 욜로를 켜면 둘이 싸운다'


def test_confirm_latches_and_turns_lidar_off():
    """확정하는 순간 방향을 잠그고 라이다를 끈다."""
    f = Fake()
    f.cone_exit()
    f.feed('R', 1.19)
    f.feed('R', 1.02)
    assert f._avoid_side == 'left', '우측 물체 -> 좌측 흰선'
    assert f._obs_armed is False, '확정했으면 라이다는 끈다'
    assert f._obs_hold_until > time.monotonic()
    assert f.yolo_on() is False, '유지 시간 동안은 욜로도 끈다'


def test_later_scans_cannot_flip_direction():
    """확정 뒤 스캔은 방향을 못 바꾼다 — 이것이 223056 의 좌우 왕복 원인."""
    f = Fake()
    f.cone_exit()
    f.feed('R', 1.19)
    f.feed('R', 1.02)
    assert f._avoid_side == 'left'

    for _ in range(20):          # 지나치는 콘·벽이 좌우로 번갈아 잡힌다
        f.feed('L', 0.4)
        f.feed('R', 0.4)
    assert f._avoid_side == 'left', '한 번 정한 방향을 유지한다'


def test_yolo_comes_back_after_hold():
    """유지 시간이 끝나면 이동차량용으로 욜로를 켠다."""
    f = Fake()
    f.cone_exit()
    f.feed('R', 1.19)
    f.feed('R', 1.02)
    assert f.yolo_on() is False

    f._obs_hold_until = 0.0      # 유지 시간 경과
    assert f.yolo_on() is True


def test_no_obstacle_hands_over_after_timeout():
    """정지차량이 없던 바퀴 — obs_arm_sec 뒤 라이다를 접고 욜로로."""
    f = Fake()
    f.cone_exit()
    assert f.yolo_on() is False

    f._obs_arm_until = time.monotonic() - 0.1   # 시한 경과
    f.feed('', None)                            # 다음 스캔에서 접는다
    assert f._obs_armed is False
    assert f.yolo_on() is True


def test_next_lap_rearms():
    """다음 바퀴 콘 탈출이 정지차량 구간을 다시 연다."""
    f = Fake()
    f.cone_exit()
    f.feed('R', 1.19)
    f.feed('R', 1.02)
    f._obs_hold_until = 0.0
    assert f._obs_armed is False

    f.cone_exit()
    assert f._obs_armed is True
    f.feed('R', 1.19)
    f.feed('R', 1.02)
    assert f._avoid_side == 'left'
