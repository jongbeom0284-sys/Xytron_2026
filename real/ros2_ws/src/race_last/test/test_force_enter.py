"""주황 트리거 강제 진입 회귀 (0823 요구: 콘 개수·게이트 조건 없이 즉시)."""
from race_last.cone_link import ConeLink


class Log:
    def info(self, *a, **k): pass
    def warning(self, *a, **k): pass


def rd(now, cones=0, valid=False, aim=None):
    return dict(stamp=now, valid=valid, depth=0.0, width=0.0, lateral=0.0,
                cones=cones, near_left=9.9, near_right=9.9, pts=[],
                wall_aim=aim, wall_basis='', wall_left=0, wall_right=0)


def test_force_enter_ignores_gate_and_count():
    """콘 0개·게이트 무효여도 트리거가 울렸으면 바로 콘 모드."""
    link = ConeLink(Log())
    link.tick(10.0, rd(10.0), ego_mps=1.0, camera_ok=True)
    assert link.active is False, '트리거 전에는 안 들어간다'
    link.request_enter()
    out = link.tick(10.1, rd(10.1), ego_mps=1.0, camera_ok=True)
    assert link.active is True, '트리거 뒤 첫 관측에 바로 진입'
    assert out is not None


def test_force_enter_waits_for_fresh_reading():
    """관측이 낡았으면(라이다 아직 안 옴) 대기했다가 오는 즉시 들어간다."""
    link = ConeLink(Log())
    link.request_enter()
    link.tick(10.0, rd(8.0), ego_mps=1.0, camera_ok=True)   # 2초 낡음
    assert link.active is False
    link.tick(10.1, rd(10.1), ego_mps=1.0, camera_ok=True)  # 신선
    assert link.active is True


def test_force_enter_primes_speed():
    link = ConeLink(Log(), entry_speed_units=8.0)
    link.request_enter()
    out = link.tick(10.0, rd(10.0), ego_mps=1.0, camera_ok=True)
    assert link.active is True
    assert out[1] > 0.0, '진입 첫 명령이 0 이면 안 된다'


def test_force_flag_is_one_shot():
    link = ConeLink(Log())
    link.request_enter()
    link.tick(10.0, rd(10.0), ego_mps=1.0, camera_ok=True)
    assert link.active is True
    link._active = False                      # 이탈했다고 치자
    link.tick(10.2, rd(10.2), ego_mps=1.0, camera_ok=True)
    assert link.active is False, '플래그는 1회용 — 다시 트리거돼야 들어간다'


def test_normal_entry_still_needs_cones():
    """트리거 없이 라이다만으로 들어갈 때는 예전 조건 그대로."""
    link = ConeLink(Log())
    link.tick(10.0, rd(10.0, cones=1, valid=False), ego_mps=1.0, camera_ok=True)
    assert link.active is False
