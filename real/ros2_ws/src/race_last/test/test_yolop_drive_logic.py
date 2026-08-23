"""yolop_drive 판단 로직 회귀 — 0823 저녁 변경 셋.

  1) 직진 조향 억제: 신호가 **사라진 뒤** delay(1s) 정상 조향 → hold(3s) 억제
  2) 콘 진입 트리거: 초록을 확정할 때마다 재무장 (2·3바퀴), 좌회전은 안 센다
  3) 앞차 인식: 추월을 마치면 다음 콘 탈출까지 끈다 (vehicle_needed=False)

rclpy 가 있는 곳(Jetson)에서만 돈다. TRT/토치는 스텁, 시계는 가짜라 결정적이다.
    cd ros2_ws/src/race_last && ROS_DOMAIN_ID=99 python3 -m pytest test/test_yolop_drive_logic.py -q
"""
import json
import os
import time as real_time

import pytest

rclpy = pytest.importorskip('rclpy')
os.environ.setdefault('ROS_DOMAIN_ID', '99')

from std_msgs.msg import String  # noqa: E402

import race_last.yolop_drive_node as ydn  # noqa: E402


class FakeTime:
    """time 모듈 대역. monotonic 만 손으로 돌리고 나머지는 진짜에 넘긴다."""

    def __init__(self):
        self.now = 1000.0

    def monotonic(self):
        return self.now

    def __getattr__(self, name):
        return getattr(real_time, name)


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros, monkeypatch):
    clock = FakeTime()
    monkeypatch.setattr(ydn, 'time', clock)
    monkeypatch.setattr(ydn.YolopDriveNode, '_load_model', lambda self: None)
    n = ydn.YolopDriveNode()
    n.clock = clock
    yield n
    n.shutdown()
    n.destroy_node()


def traffic(n, state, count=1):
    for _ in range(count):
        n._on_traffic(String(data=json.dumps({'state': state})))


def vehicle(n, side=None, dist=1.0):
    boxes = ([dict(x=0.1, y=0.5, w=0.2, h=0.2, dir=side, dist=dist)]
             if side else [])
    n._on_vehicle(String(data=json.dumps(
        {'car': {'dir': side or 'NONE', 'dist': dist if side else None},
         'boxes': boxes, 'w': 640, 'h': 480})))


# ------------------------------------------------------------ 1) 직진 억제
def test_hold_is_off_by_default(node):
    """0823 저녁: 직진 구간은 흰선 중앙을 따라가므로 조향 억제를 기본으로 끈다.

    (straight_hold_sec 기본 0.0 → 창 길이 0 → 억제 없음)
    """
    c = node.clock
    assert node.hold_sec == 0.0
    traffic(node, 'green', ydn.GO_CONFIRM)
    traffic(node, 'none', ydn.GONE_CONFIRM)
    for dt in (0.0, 0.5, 1.0, 1.5, 3.0, 5.0):
        assert not node._hold_active(c.now + dt), dt


def test_hold_window_returns_when_param_set(node):
    """되살리고 싶으면 straight_hold_sec 에 초를 넣는다.

    그때 창은 '신호가 사라진 시점 + delay' 부터 hold 초 동안이다.
    """
    from rclpy.parameter import Parameter
    node._on_parameters([Parameter('straight_hold_sec', value=3.0)])
    c = node.clock
    traffic(node, 'green', ydn.GO_CONFIRM)
    assert not node._hold_active(c.now)          # 확정 직후는 정상 조향
    c.now += 2.0
    traffic(node, 'green')                        # 아직 보인다
    assert not node._hold_active(c.now)
    t_gone = c.now
    traffic(node, 'none', ydn.GONE_CONFIRM)       # 사라짐
    assert node._straight_hold_from == pytest.approx(t_gone + node.hold_delay_sec)
    assert node._straight_hold_until == pytest.approx(
        t_gone + node.hold_delay_sec + 3.0)
    node._avoid_side, node._avoid_until = 'right', c.now + 100
    c.now = t_gone + 0.5                          # delay 중 = 정상 조향, 회피 허용
    assert not node._hold_active(c.now)
    assert node._avoid_offset(c.now) == ydn.AVOID_OFFSET_PX
    c.now = t_gone + 1.5                          # 억제 중 = 회피 금지
    assert node._hold_active(c.now)
    assert node._avoid_offset(c.now) == 0.0
    c.now = t_gone + 4.5
    assert not node._hold_active(c.now)


def test_hold_when_signal_never_disappears_uses_wait_gone_max(node):
    c = node.clock
    traffic(node, 'green', ydn.GO_CONFIRM)
    c.now += ydn.WAIT_GONE_MAX_SEC + 0.1
    t = c.now
    traffic(node, 'green')            # 안 사라짐 → 여기서 사라진 것으로 친다
    assert node._lock_kind is None
    # 억제를 껐어도 '사라진 시점' 기준은 그대로 잡힌다
    assert node._straight_hold_from == pytest.approx(t + node.hold_delay_sec)


def test_hold_params_live_update(node):
    from rclpy.parameter import Parameter
    res = node._on_parameters([Parameter('straight_hold_delay_sec', value=0.5),
                               Parameter('straight_hold_sec', value=2.0),
                               Parameter('straight_hold_gain', value=0.3)])
    assert res.successful
    assert (node.hold_delay_sec, node.hold_sec, node.hold_gain) == (0.5, 2.0, 0.3)
    bad = node._on_parameters([Parameter('straight_hold_gain', value=5.0)])
    assert not bad.successful


# ------------------------------------------------------------ 2) 콘 트리거 재무장
def test_cone_trigger_rearms_on_each_green_confirm(node):
    c = node.clock
    traffic(node, 'green', ydn.GO_CONFIRM)          # 1바퀴 확정
    assert node._cone_triggered is False
    node._cone_triggered = True                     # 1바퀴 콘에서 울렸다 치자
    node._cone_trig_run = 2
    # 2바퀴: 잠금이 풀린 뒤 신호등을 다시 지난다
    traffic(node, 'none', ydn.GONE_CONFIRM)
    c.now = node._go_locked_until + 1.0
    traffic(node, 'green', ydn.GO_CONFIRM)
    assert node._cone_triggered is False, '초록 재확정에 재무장돼야 한다'
    assert node._cone_trig_run == 0


def test_left_turn_does_not_count_cone_trigger(node):
    c = node.clock
    traffic(node, 'green', ydn.GO_CONFIRM)
    node._cone_triggered = True
    traffic(node, 'none', ydn.GONE_CONFIRM)
    c.now = node._go_locked_until + 1.0
    traffic(node, 'left', ydn.GO_CONFIRM)
    assert node._straight is False                  # 트리거는 _straight 에만 센다
    assert node._cone_triggered is True, '좌회전은 재무장하지 않는다'


# ------------------------------------------------------------ 3) 추월 후 앞차 인식 끔
def test_vehicle_perception_off_after_pass_until_cone_exit(node):
    c = node.clock
    assert node._vehicle_needed(c.now) is True
    for _ in range(ydn.CAR_CONFIRM):                # 앞차 확정
        vehicle(node, 'LEFT')
    assert node._avoid_side == 'right'
    assert node._vehicle_needed(c.now) is True      # 회피 중엔 계속 본다
    c.now += ydn.AVOID_HOLD_SEC + 0.1
    vehicle(node, None)                             # 사라진 채 유지시간 경과
    assert node._avoid_side is None
    assert node._vehicle_passed is True
    assert node._vehicle_needed(c.now) is False, '추월 완료 → 끈다'
    # 대시보드 강제는 이긴다
    node._vehicle_force_until = c.now + 5.0
    assert node._vehicle_needed(c.now) is True
    node._vehicle_force_until = 0.0
    # 다음 콘 구간 탈출 → 다시 켠다
    node._cone_was_active = True
    node._cone_link._active = False
    node._check_cone_exit(c.now)
    assert node._vehicle_passed is False
    assert node._vehicle_needed(c.now) is True


def test_vehicle_off_after_pass_can_be_disabled(node):
    from rclpy.parameter import Parameter
    node._on_parameters([Parameter('vehicle_off_after_pass', value=False)])
    c = node.clock
    for _ in range(ydn.CAR_CONFIRM):
        vehicle(node, 'RIGHT')
    c.now += ydn.AVOID_HOLD_SEC + 0.1
    vehicle(node, None)
    assert node._vehicle_passed is False
    assert node._vehicle_needed(c.now) is True


def test_status_json_builds_with_new_fields(node):
    node._last_status = -1e9
    node._publish_status('none', 0.0, 0.0, False)   # 예외 없이 JSON 이 만들어진다


def test_green_confirm_clears_vehicle_passed(node):
    node._vehicle_passed = True
    assert node._vehicle_needed(node.clock.now) is False
    traffic(node, 'green', ydn.GO_CONFIRM)
    assert node._vehicle_passed is False
    assert node._vehicle_needed(node.clock.now) is True


def test_car_confirm_is_tunable(node):
    """앞차 확정 메시지 수는 주행 중에 바꿀 수 있다 (기본 3 = 검출 1회)."""
    from rclpy.parameter import Parameter
    assert node.car_confirm == 3
    c = node.clock
    for _ in range(3):
        vehicle(node, 'LEFT')
    assert node._avoid_side == 'right', '3메시지면 확정돼야 한다'

    node._on_parameters([Parameter('car_confirm', value=9)])
    assert node.car_confirm == 9
    assert node._car_recent == [], '값을 바꾸면 카운터를 비운다'
    node._avoid_side, node._avoid_until, node._avoid_rearm_until = None, 0.0, 0.0
    for _ in range(8):
        vehicle(node, 'RIGHT')
    assert node._avoid_side is None, '8메시지로는 아직 아니다'
    vehicle(node, 'RIGHT')
    assert node._avoid_side == 'left'


def test_car_confirm_rejects_bad_value(node):
    from rclpy.parameter import Parameter
    assert not node._on_parameters([Parameter('car_confirm', value=0)]).successful
    assert not node._on_parameters([Parameter('car_confirm', value=99)]).successful
