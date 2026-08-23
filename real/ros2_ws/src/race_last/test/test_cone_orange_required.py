"""콘 모드 진입은 언제나 주황 트리거가 있어야 한다 (0823 밤).

라이다 게이트만으로는 절대 안 들어간다. 콘 구간 밖에서도 게이트는 얼마든지
성립하기 때문이다 — 실측:
  · 230555 f293 '깊이0.83 폭0.82', f297 '깊이0.75 폭0.82' 는 곡선 바깥 벽의
    좌·우 반사다. 폭이 콘 통로 규격과 그대로 맞아 f299 에 콘 모드가 켜졌고,
    그때 신호등 램프는 0개, 신호잠금은 17.2 -> 14.8초로 줄기만 했다.
  · 225657 은 정지차량 옆 라바콘으로 같은 일이 나 콘 조향 -40 풀락, 트랙 이탈.

그리고 한 번 빠져나오면 request_enter() 말고는 진입 경로가 없다.
그 트리거는 초록불 확정으로만 재무장되므로 재진입 조건 = 초록불이다.
"""
from race_last.cone_link import ConeLink


class Log:
    def info(self, *a, **k): pass
    def warning(self, *a, **k): pass


def gate(now, cones=6, depth=0.83, width=0.82):
    """콘이 좌우로 짝을 이뤄 통로가 보이는 관측 (라이다가 아주 확실한 상태).

    곡선 벽 반사도 이 모양으로 온다 — 그래서 이것만으로 들어가면 안 된다.
    """
    return dict(stamp=now, valid=True, depth=depth, width=width, lateral=0.0,
                cones=cones, near_left=0.4, near_right=0.4,
                pts=[[0.5, 0.41], [0.6, -0.41], [0.9, 0.42], [1.0, -0.41]],
                wall_aim=0.0, wall_basis='both', wall_left=3, wall_right=3)


def empty(now):
    return dict(stamp=now, valid=False, depth=0.0, width=0.0, lateral=0.0,
                cones=0, near_left=9.9, near_right=9.9, pts=[],
                wall_aim=None, wall_basis='', wall_left=0, wall_right=0)


def run(link, t0, obs_fn, n=40, step=0.1):
    t = t0
    for _ in range(n):
        t += step
        link.tick(t, obs_fn(t), ego_mps=1.6, camera_ok=True)
    return t


def test_lidar_gate_alone_never_enters():
    """주황 없이 라이다 게이트만 계속 보여도 콘 모드로 안 들어간다."""
    link = ConeLink(Log())
    run(link, 10.0, gate, n=40)
    assert link.active is False, '곡선 벽·정지차량 옆 콘이 이걸로 들어왔다'


def test_orange_trigger_enters():
    """주황 트리거는 조건 없이 들어간다 (기존 동작)."""
    link = ConeLink(Log())
    link.request_enter()
    link.tick(10.0, gate(10.0), ego_mps=1.6, camera_ok=True)
    assert link.active is True


def test_no_reentry_after_exit_without_trigger():
    """빠져나온 뒤 게이트가 다시 보여도 재진입하지 않는다 (f299)."""
    link = ConeLink(Log())
    link.request_enter()
    link.tick(10.0, gate(10.0), ego_mps=1.6, camera_ok=True)
    assert link.active is True

    t = run(link, 10.0, empty, n=60)          # 콘이 사라져 이탈
    assert link.active is False, '이탈해야 한다'

    run(link, t, gate, n=60)                  # 곡선 벽이 통로로 보인다
    assert link.active is False, '주황 없이 다시 들어가면 안 된다'
    assert link.approach is False


def test_reentry_only_via_trigger():
    """다음 바퀴 주황 트리거(=초록불로 재무장)가 오면 다시 들어간다."""
    link = ConeLink(Log())
    link.request_enter()
    link.tick(10.0, gate(10.0), ego_mps=1.6, camera_ok=True)
    t = run(link, 10.0, empty, n=60)
    assert link.active is False

    link.request_enter()                      # 초록 -> 주황 트리거
    t += 0.1
    link.tick(t, gate(t), ego_mps=1.6, camera_ok=True)
    assert link.active is True
    assert link._exited is False
