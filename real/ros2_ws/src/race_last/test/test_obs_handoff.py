"""정지차량(라이다) -> 이동차량(욜로) 인계 (0823 밤).

코스 한 바퀴:
    직진 -> 콘 -> 정지차량 추월 -> 코너 -> 이동차량 추월 -> 코너
  4번 정지차량 : 라이다. 콘 탈출에 켜고, 한 번 지나가면 끈다.
  6번 이동차량 : 욜로. 4번이 끝나는 순간부터 계속 본다.

라이다를 6번까지 끌고 가면 안 된다 — 밴드가 차 정면 기준이라 코너에서
벽이 정면에 들어온다. 4번은 콘~코너 사이 직선이라 300~316 17프레임 연속
'없음' 이었다 (실측 214410).
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
    """_on_obstacle / _check_cone_exit 가 건드리는 것만 갖춘 최소 대역."""

    def __init__(self, mps=1.6):
        self.lidar_avoid_enabled = True
        self.obs_confirm = 2
        self.vehicle_window_sec = 30.0
        self.vehicle_off_after_pass = False
        self._vehicle_window_until = 0.0
        self._vehicle_passed = False
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
        self._avoid_side = None
        self._avoid_until = 0.0
        self._avoid_rearm_until = 0.0
        self._lock = threading.Lock()
        self._log = Log()

    def get_logger(self):
        return self._log

    def _side_by_lane(self, boxes, payload):
        """차가 안 보이는 상황만 재현하면 되므로 항상 '없음'."""
        return 'NONE'

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


def test_lidar_off_before_cone_exit():
    """콘 탈출 전에는 라이다 회피를 쓰지 않는다 (직진·콘 구간)."""
    f = Fake()
    f._vehicle_window_until = time.monotonic() + 30.0   # 창은 열려 있어도
    f.feed('R', 1.0)
    f.feed('R', 1.0)
    assert f._obs_armed is False
    assert f._avoid_side is None


def test_cone_exit_arms_lidar():
    """콘 탈출이 정지차량 구간을 연다."""
    f = Fake()
    f.cone_exit()
    assert f._obs_armed is True
    f.feed('R', 1.19)
    f.feed('R', 1.02)
    assert f._avoid_side == 'left', '정지차량 우측 -> 좌측 흰선'


def test_pass_disarms_lidar_but_keeps_yolo():
    """정지차량을 지나면 라이다는 끄고 앞차 인식(욜로)은 켜둔다."""
    f = Fake()
    f.cone_exit()
    f.feed('R', 1.19)
    f.feed('R', 1.02)
    assert f._obs_armed is True

    _release(f)

    assert f._obs_armed is False, '라이다 회피는 꺼야 한다 (코너 벽 오검출)'
    assert f._obs_hold_until == 0.0
    assert f._vehicle_passed is False, '이동차량을 욜로로 봐야 하므로 끄면 안 된다'


def test_lidar_stays_off_through_corner():
    """끈 뒤에는 코너에서 벽이 정면에 들어와도 회피가 안 걸린다."""
    f = Fake()
    f.cone_exit()
    f.feed('R', 1.19)
    f.feed('R', 1.02)
    _release(f)

    for _ in range(10):                      # 코너 벽이 계속 정면에 잡힌다
        f.feed('L', 0.9)
    assert f._avoid_side is None


def test_next_lap_rearms():
    """다음 바퀴 콘 탈출이 정지차량 구간을 다시 연다."""
    f = Fake()
    f.cone_exit()
    f.feed('R', 1.19)
    f.feed('R', 1.02)
    _release(f)
    assert f._obs_armed is False

    f.cone_exit()
    assert f._obs_armed is True
    f.feed('R', 1.19)
    f.feed('R', 1.02)
    assert f._avoid_side == 'left'


def _release(f):
    """차가 시야에서 사라져 회피가 해제되는 순간을 재현한다."""
    f._avoid_until = 0.0
    f._obs_hold_until = 0.0
    msg = type('M', (), {})()
    msg.data = json.dumps(dict(car={'dir': 'NONE', 'dist': None}, boxes=[]))
    f._cone_reading = None
    f._car_dist = None
    f._car_recent = []
    f.car_confirm = 1
    f._straight_hold_until = 0.0
    f._straight_hold_from = 0.0
    f.vehicle_inside_lane_only = True
    YolopDriveNode._on_vehicle(f, msg)
