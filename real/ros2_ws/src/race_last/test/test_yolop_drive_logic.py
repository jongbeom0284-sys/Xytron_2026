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


def open_vehicle_window(n, sec=30.0):
    """콘 구간을 빠져나온 직후 상태로 만든다 — 앞차 인식이 켜지는 구간.

    실제로는 _check_cone_exit 가 이 창을 연다. 회피 로직만 보는 테스트에서는
    '차로 안 물체인가' 판정도 끈다 — 그건 test_vehicle_in_lane 이 따로 본다.
    """
    n._vehicle_window_until = n.clock.now + sec
    n.vehicle_inside_lane_only = False
    # 라이다 구간이 끝난 뒤를 전제한다 (라이다가 켜져 있으면 욜로는 꺼진다).
    n._obs_armed = False
    n._obs_hold_until = 0.0


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
    # 기본은 False 다 (코너 뒤 이동차량을 욜로로 봐야 하므로). 이 테스트는
    # 켰을 때의 동작을 지킨다 — 대시보드에서 다시 켤 수 있는 선택지다.
    node.vehicle_off_after_pass = True
    c = node.clock
    open_vehicle_window(node)
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
    # 다음 콘 구간 탈출 → 추월 완료가 풀린다. 다만 그 직후는 라이다 구간이라
    # 욜로는 라이다가 접은 뒤에 켜진다.
    node._cone_was_active = True
    node._cone_link._active = False
    node._check_cone_exit(c.now)
    assert node._vehicle_passed is False
    assert node._vehicle_needed(c.now) is False, '라이다 구간'
    node._obs_armed = False
    assert node._vehicle_needed(c.now) is True


def test_vehicle_off_after_pass_can_be_disabled(node):
    from rclpy.parameter import Parameter
    node._on_parameters([Parameter('vehicle_off_after_pass', value=False)])
    c = node.clock
    open_vehicle_window(node)
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
    open_vehicle_window(node)
    node._vehicle_passed = True
    assert node._vehicle_needed(node.clock.now) is False
    traffic(node, 'green', ydn.GO_CONFIRM)
    assert node._vehicle_passed is False
    assert node._vehicle_needed(node.clock.now) is True


def test_car_confirm_is_tunable(node):
    """앞차 확정 메시지 수는 주행 중에 바꿀 수 있다.

    기본 1 — 콘 탈출 후 30초 창 안에서, 차로 안에 있는 물체만 올라오므로
    한 번만 봐도 피한다 (0823 밤 요구).
    """
    from rclpy.parameter import Parameter
    assert node.car_confirm == 1
    open_vehicle_window(node)
    c = node.clock
    vehicle(node, 'LEFT')
    assert node._avoid_side == 'right', '한 번 보면 바로 회피'

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


# ─────────────────── 앞차 인식 창 (콘 탈출 후 30초) ───────────────────
def test_cone_exit_opens_vehicle_window(node):
    """콘 구간을 빠져나오면 그때부터 vehicle_window_sec 동안만 앞차를 본다."""
    c = node.clock
    assert node.vehicle_window_sec == 30.0
    assert node._vehicle_needed(c.now) is False, '창이 열리기 전에는 안 본다'

    node._cone_was_active = True
    node._cone_link._active = False
    node._check_cone_exit(c.now)                 # 콘 탈출
    assert node._vehicle_window_until == pytest.approx(c.now + 30.0)
    # 콘 탈출 직후는 라이다(정지차량) 구간이다 — 욜로는 아직 안 켠다
    assert node._obs_armed is True
    assert node._vehicle_needed(c.now) is False, '라이다가 보는 동안은 욜로를 끈다'

    node._obs_armed = False                       # 라이다가 접었다
    assert node._vehicle_needed(c.now) is True

    c.now += 29.0
    assert node._vehicle_needed(c.now) is True
    c.now += 2.0                                  # 창을 넘김
    assert node._vehicle_needed(c.now) is False


def test_vehicle_window_can_be_disabled(node):
    """0 이면 제한 없음 — 콘 구간 밖에서 항상 본다."""
    from rclpy.parameter import Parameter
    node._on_parameters([Parameter('vehicle_window_sec', value=0.0)])
    assert node.vehicle_window_sec == 0.0
    assert node._vehicle_needed(node.clock.now) is True


def test_vehicle_window_rejects_bad_value(node):
    from rclpy.parameter import Parameter
    bad = node._on_parameters([Parameter('vehicle_window_sec', value=-1.0)])
    assert not bad.successful


def test_dashboard_force_beats_window(node):
    """대시보드 강제 켜기는 창과 무관하게 이긴다."""
    c = node.clock
    assert node._vehicle_needed(c.now) is False
    node._vehicle_force_until = c.now + 5.0
    assert node._vehicle_needed(c.now) is True
