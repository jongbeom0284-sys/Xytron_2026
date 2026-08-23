"""라이다 관측 -> 회피 방향·유지 시간 (0823 밤).

정지물체이므로 상대속도 = 자차속도다. 그래서 유지 시간을 계산할 수 있다.
    (거리 + 차체길이+여유) / 현재속도
고정 3초는 2.16 m/s 에서 4배 길고, 0.7 m/s 에서는 짧다.
"""
import json

from race_last.yolop_drive_node import (OBS_HOLD_MAX_SEC, OBS_HOLD_MIN_SEC,
                                        OBS_PASS_MARGIN_M, YolopDriveNode)


class Log:
    def info(self, *a, **k): pass
    def warning(self, *a, **k): pass
    def error(self, *a, **k): pass


class Link:
    active = False
    approach = False


class Fake:
    """_on_obstacle 이 실제로 건드리는 것만 갖춘 최소 대역."""

    def __init__(self, mps=1.6, window_open=True):
        self.lidar_avoid_enabled = True
        self.obs_confirm = 2
        self.vehicle_window_sec = 30.0
        self._vehicle_window_until = 1e9 if window_open else 0.0
        self._cone_link = Link()
        self.speed_units = mps / 0.08
        self.mps_per_unit = 0.08
        self._obs = None
        self._obs_recent = []
        self._obs_hold_until = 0.0
        self._obs_armed = True        # 정지차량 구간(콘 탈출 직후)을 전제
        self._obs_arm_until = 1e9     # 무장 시한은 이 테스트의 관심사가 아니다
        self.obs_arm_sec = 10.0
        self._avoid_side = None
        self._avoid_until = 0.0
        self._avoid_rearm_until = 0.0
        self._lock = __import__('threading').Lock()
        self._log = Log()

    def get_logger(self):
        return self._log

    def feed(self, side, dist, n=1):
        msg = type('M', (), {})()
        msg.data = json.dumps(dict(side=side, dist=dist, y=0.0, n=n, groups=1))
        YolopDriveNode._on_obstacle(self, msg)


def test_one_frame_is_not_enough():
    """obs_confirm=2 — 한 장으로는 차선을 바꾸지 않는다."""
    f = Fake()
    f.feed('R', 1.19)
    assert f._avoid_side is None


def test_two_frames_confirm_and_steer_opposite():
    """실측 f317/f318 두 장이면 확정. 우측 물체 -> 좌측 흰선."""
    f = Fake()
    f.feed('R', 1.19)
    f.feed('R', 1.02)
    assert f._avoid_side == 'left'
    assert f._obs_hold_until > 0.0


def test_left_object_steers_right():
    f = Fake()
    f.feed('L', 1.19)
    f.feed('L', 1.02)
    assert f._avoid_side == 'right'


def test_alternating_side_does_not_confirm():
    """좌/우가 번갈아 나오면 확정하지 않는다."""
    f = Fake()
    f.feed('R', 1.0)
    f.feed('L', 1.0)
    assert f._avoid_side is None


def test_hold_scales_with_speed():
    """빠를수록 짧게 잡는다 — 고정 3초가 틀린 이유."""
    import time
    holds = {}
    for mps in (0.7, 1.6, 2.16):
        f = Fake(mps=mps)
        f.feed('R', 1.0)
        f.feed('R', 1.0)
        holds[mps] = f._obs_hold_until - time.monotonic()
    assert holds[0.7] > holds[1.6] > holds[2.16]
    # 1.6 m/s, 1.0m 앞: (1.0 + 0.8) / 1.6 = 1.125초
    assert abs(holds[1.6] - (1.0 + OBS_PASS_MARGIN_M) / 1.6) < 0.05


def test_hold_is_clamped():
    """오검출이 오래 붙잡지 못하게, 너무 짧아 조향이 안 먹지도 않게."""
    import time
    slow = Fake(mps=0.3)
    slow.feed('R', 2.4)
    slow.feed('R', 2.4)
    assert (slow._obs_hold_until - time.monotonic()) <= OBS_HOLD_MAX_SEC + 0.05

    near = Fake(mps=2.16)
    near.feed('R', 0.25)
    near.feed('R', 0.25)
    assert (near._obs_hold_until - time.monotonic()) >= OBS_HOLD_MIN_SEC - 0.05


def test_disabled_flag_does_nothing():
    f = Fake()
    f.lidar_avoid_enabled = False
    f.feed('R', 1.0)
    f.feed('R', 1.0)
    assert f._avoid_side is None
    assert f._obs is not None, '끄더라도 상태 표시용 관측은 남긴다'


def test_cone_mode_ignores_lidar():
    """콘 모드에서는 콘 자체가 잡히므로 쓰지 않는다."""
    f = Fake()
    f._cone_link.active = True
    f.feed('R', 1.0)
    f.feed('R', 1.0)
    assert f._avoid_side is None


def test_outside_vehicle_window_ignored():
    """콘 탈출 후 vehicle_window_sec 밖에서는 안 쓴다."""
    f = Fake(window_open=False)
    f.feed('R', 1.0)
    f.feed('R', 1.0)
    assert f._avoid_side is None


def test_avoid_until_never_shortened():
    """카메라가 잡아둔 유지 시간을 라이다가 깎지 않는다."""
    import time
    f = Fake(mps=2.16)
    f._avoid_until = time.monotonic() + 3.0
    f.feed('R', 0.5)
    f.feed('R', 0.5)
    assert f._avoid_until - time.monotonic() > 2.9
