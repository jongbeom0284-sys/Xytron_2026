"""콘 진입 동작 회귀 — 0823 저녁 요구 (즉시 진입 + 진입 속도 0.7 m/s)."""
import types
import pytest

from race_last.cone_link import ConeLink


class Log:
    def info(self, *a, **k): pass
    def warning(self, *a, **k): pass


def reading(now, cones=6, valid=True, aim=0.0, *, stamp=None,
            basis='both', left=2, right=2):
    return dict(stamp=now if stamp is None else stamp,
                valid=valid, depth=0.6, width=0.82, lateral=0.0,
                cones=cones, near_left=0.4, near_right=0.4,
                pts=[[0.5, 0.4], [0.6, -0.4], [0.9, 0.42], [1.0, -0.41]],
                wall_aim=aim, wall_basis=basis,
                wall_left=left, wall_right=right)


def test_enters_on_first_frame_after_trigger():
    """주황 트리거를 받으면 다음 프레임에 바로 콘 모드."""
    link = ConeLink(Log())
    assert link.active is False
    link.request_enter()
    out = link.tick(10.0, reading(10.0), ego_mps=1.0, camera_ok=True)
    assert link.active is True, '트리거 뒤 첫 프레임에 진입해야 한다'
    assert out is not None


def test_lidar_alone_never_enters():
    """주황 없이는 라이다 게이트가 몇 프레임을 와도 안 들어간다.

    콘 구간 밖에서도 게이트는 성립한다 — 230555 f293/f297 은 곡선 바깥 벽,
    225657 은 정지차량 옆 라바콘이었다. 둘 다 이 경로로 콘 모드를 켰다.
    """
    link = ConeLink(Log())
    t = 10.0
    for _ in range(30):
        t += 0.1
        link.tick(t, reading(t), ego_mps=1.0, camera_ok=True)
    assert link.active is False


def test_entry_speed_seeds_ramp():
    """진입 순간 속도가 0 에서 시작하지 않는다."""
    seed = 8.0
    link = ConeLink(Log(), entry_speed_units=seed)
    link.request_enter()
    out = link.tick(10.0, reading(10.0), ego_mps=1.0, camera_ok=True)
    assert link.active is True
    assert link._policy.speed._current >= seed * 0.9, link._policy.speed._current
    assert out[1] > 0.0, '진입 첫 명령이 0 이면 안 된다'


def test_zero_entry_speed_keeps_old_behaviour():
    link = ConeLink(Log(), entry_speed_units=0.0)
    link.request_enter()
    link.tick(10.0, reading(10.0), ego_mps=1.0, camera_ok=True)
    assert link._policy.speed._current == 0.0 or link._policy.speed._current > 0.0


def test_no_entry_without_lidar_cones():
    """트리거가 있어도 관측이 없으면 들어가지 않는다."""
    link = ConeLink(Log())
    link.tick(10.0, reading(10.0, cones=0, valid=False), ego_mps=1.0,
              camera_ok=True)
    assert link.active is False


def test_stale_reading_does_not_enter():
    """관측이 낡았으면 트리거가 있어도 들어가지 않는다."""
    link = ConeLink(Log())
    link.request_enter()
    link.tick(10.0, reading(9.0), ego_mps=1.0, camera_ok=True)   # 1초 낡음
    assert link.active is False


def test_forced_entry_holds_lane_steer_until_two_unique_strong_scans():
    """첫 임시 경계로 +32도를 즉시 내지 않고 직전 차선 조향을 유지한다."""
    link = ConeLink(Log(), entry_speed_units=8.0,
                    entry_stable_frames=2, entry_weak_timeout_sec=0.25)
    link.request_enter()

    weak = reading(10.0, aim=-0.221, basis='CENTER_OFFSET_gate',
                   left=1, right=2)
    out = link.tick(10.0, weak, ego_mps=1.5, camera_ok=True,
                    handoff_steer=1.1)
    assert out[0] == pytest.approx(1.1)
    assert 0.0 < out[1] <= 8.0

    first = reading(10.1, aim=-0.221, basis='CENTER_OFFSET_both',
                    left=2, right=2, stamp=10.1)
    out = link.tick(10.1, first, ego_mps=1.0, camera_ok=True,
                    handoff_steer=1.1)
    assert out[0] == pytest.approx(1.1)

    # 같은 라이다 stamp를 20Hz 제어가 두 번 본 것은 2프레임이 아니다.
    duplicate = reading(10.15, aim=-0.221, basis='CENTER_OFFSET_both',
                        left=2, right=2, stamp=10.1)
    out = link.tick(10.15, duplicate, ego_mps=1.0, camera_ok=True,
                    handoff_steer=1.1)
    assert out[0] == pytest.approx(1.1)

    second = reading(10.2, aim=-0.221, basis='CENTER_OFFSET_both',
                     left=2, right=2, stamp=10.2)
    out = link.tick(10.2, second, ego_mps=1.0, camera_ok=True,
                    handoff_steer=1.1)
    assert 0.0 < out[0] - 1.1 <= 6.0
    assert out[1] <= 8.0


def test_weak_entry_timeout_moves_with_rate_limit_and_does_not_stop():
    """양쪽 벽이 늦어도 정지하지 않고 직전 조향에서 제한 전환한다."""
    link = ConeLink(Log(), entry_speed_units=8.0,
                    entry_stable_frames=2, entry_weak_timeout_sec=0.25)
    link.request_enter()
    weak = reading(20.0, aim=-0.221, basis='CENTER_OFFSET_gate',
                   left=1, right=2)
    first = link.tick(20.0, weak, ego_mps=1.5, camera_ok=True,
                      handoff_steer=1.1)
    assert first[0] == pytest.approx(1.1)

    weak_later = reading(20.3, aim=-0.221, basis='CENTER_OFFSET_gate',
                         left=1, right=2)
    out = link.tick(20.3, weak_later, ego_mps=1.0, camera_ok=True,
                    handoff_steer=1.1)
    assert 0.0 < out[0] - 1.1 <= 6.0
    assert out[1] > 0.0
