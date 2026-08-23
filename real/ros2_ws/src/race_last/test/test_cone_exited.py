"""콘을 빠져나온 뒤 approach 가 되살아나지 않는지 (0823 밤).

실측 근거: 214410 세션에서 콘 모드가 f302 에 끝났는데 f319~324, f331~340 에
cone_approach 가 다시 떠 속도가 27 -> 9 units 로 떨어지고 앞차 인식이 꺼졌다.
"""
from race_last.cone_link import ConeLink


class Log:
    def info(self, *a, **k): pass
    def warning(self, *a, **k): pass


def gate(now, cones=5, depth=0.8):
    """통로가 코앞에 보이는 관측 (approach 조건을 만족한다)."""
    return dict(stamp=now, valid=True, depth=depth, width=0.82, lateral=0.0,
                cones=cones, near_left=0.4, near_right=0.4,
                pts=[[0.5, 0.4], [0.6, -0.4], [0.9, 0.42], [1.0, -0.41]],
                wall_aim=0.0, wall_basis='both', wall_left=2, wall_right=2)


def empty(now):
    """콘이 안 보이는 관측."""
    return dict(stamp=now, valid=False, depth=0.0, width=0.0, lateral=0.0,
                cones=0, near_left=9.9, near_right=9.9, pts=[],
                wall_aim=None, wall_basis='', wall_left=0, wall_right=0)


def test_approach_on_before_entering():
    """진입 전에는 접근 감속이 켜진다 (기존 동작)."""
    link = ConeLink(Log())
    link.tick(10.0, gate(10.0), ego_mps=1.0, camera_ok=True)
    assert link.approach is True


def test_approach_off_after_exit():
    """빠져나온 뒤에는 같은 관측이 와도 접근 감속이 다시 켜지지 않는다."""
    link = ConeLink(Log())
    link.request_enter()
    link.tick(10.0, gate(10.0), ego_mps=1.0, camera_ok=True)
    assert link.active is True

    # 콘이 사라져 이탈 (게이트 상실 타임아웃)
    t = 10.0
    for _ in range(60):
        t += 0.1
        link.tick(t, empty(t), ego_mps=1.0, camera_ok=True)
        if not link.active:
            break
    assert link.active is False, '이탈해야 한다'

    # 지나온 콘이 다시 짝을 이뤄 보인다 — 예전에는 여기서 approach 가 켜졌다
    t += 0.1
    link.tick(t, gate(t), ego_mps=1.0, camera_ok=True)
    assert link.approach is False, '이미 지난 구간이면 접근 감속을 켜지 않는다'


def test_new_trigger_reenables_approach():
    """다음 바퀴 주황 트리거가 오면 접근 감속을 다시 허용한다."""
    link = ConeLink(Log())
    link._exited = True
    link.tick(10.0, gate(10.0), ego_mps=1.0, camera_ok=True)
    assert link.approach is False
    link.request_enter()
    assert link._exited is False


def test_entering_clears_exited():
    """실제로 다시 진입하면 플래그가 풀린다."""
    link = ConeLink(Log())
    link._exited = True
    link.request_enter()
    link.tick(10.0, gate(10.0), ego_mps=1.0, camera_ok=True)
    assert link.active is True
    assert link._exited is False
