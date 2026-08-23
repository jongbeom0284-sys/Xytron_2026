"""노드 구조 회귀 — ROS 엔티티(구독·발행·타이머)는 __init__ 에서만 만든다.

2026-08-23 실측 사고: Vehicle_Perception.on_compressed() 본문 끝에
create_subscription('/kobot_yolop/status') 가 들여쓰기 실수로 들어가 있었다.
매 프레임(30Hz)마다 구독이 하나씩 늘어 84분 뒤 632개가 됐고,
  · status 한 건마다 콜백이 632번 불려 CPU 233%
  · /vehicle/state 발행이 30Hz → 0.5Hz
  · status 발행자(yolop_drive)도 reliable 구독자 634개를 상대하느라 지연
즉 "점점 느려지고 구독이 안 지워지는" 증상의 원인이었다.
Traffic_Perception 에서는 같은 줄이 구독되지 않는 on_raw() 안에 들어가
가드가 아예 동작하지 않았다.

이 테스트는 AST 로 모든 노드 모듈을 훑어 같은 실수를 빌드 전에 잡는다.
rclpy·cv2 없이 돈다 (소스만 파싱한다).
"""
import ast
import os

PKG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'race_last')

CREATORS = {
    'create_subscription', 'create_publisher', 'create_timer',
    'create_client', 'create_service',
}

# 의도적으로 콜백에서 만드는 것만 허용한다. (모듈, 클래스, 메서드, 호출)
ALLOWED = {
    # armed 를 켜고 끌 때 모터 발행자를 만들고 없앤다 — 조건문으로 1개만 유지
    ('yolop_drive_node.py', 'YolopDriveNode', '_apply_armed', 'create_publisher'),
}


def _node_modules():
    for name in sorted(os.listdir(PKG_DIR)):
        if name.endswith('.py'):
            yield name, os.path.join(PKG_DIR, name)


def _creations_outside_init(path):
    tree = ast.parse(open(path, encoding='utf-8').read())
    found = []
    for cls in (n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)):
        for fn in (n for n in cls.body if isinstance(n, ast.FunctionDef)):
            if fn.name == '__init__':
                continue
            for node in ast.walk(fn):
                if (isinstance(node, ast.Call)
                        and isinstance(node.func, ast.Attribute)
                        and node.func.attr in CREATORS):
                    found.append((cls.name, fn.name, node.func.attr,
                                  node.lineno))
    return found


def test_ros_entities_created_only_in_init():
    bad = []
    for name, path in _node_modules():
        for cls, fn, call, line in _creations_outside_init(path):
            if (name, cls, fn, call) in ALLOWED:
                continue
            bad.append(f'{name}:{line} {cls}.{fn}() -> {call}')
    assert not bad, (
        '콜백/메서드 안에서 ROS 엔티티를 만들고 있다 (매 호출마다 누적된다):\n  '
        + '\n  '.join(bad))


def _subscribed_topics(path, only_init):
    tree = ast.parse(open(path, encoding='utf-8').read())
    fns = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
           and (n.name == '__init__' or not only_init)]
    topics = []
    for fn in fns:
        for node in ast.walk(fn):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == 'create_subscription'):
                for arg in node.args:
                    if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                        topics.append(arg.value)
    return topics


def test_vehicle_perception_subscribes_status_once_in_init():
    """vehicle_needed 가드를 먹이는 status 구독이 __init__ 에 정확히 한 번 있다."""
    path = os.path.join(PKG_DIR, 'Vehicle_Perception.py')
    in_init = _subscribed_topics(path, only_init=True)
    anywhere = _subscribed_topics(path, only_init=False)
    assert in_init.count('/kobot_yolop/status') == 1, in_init
    assert anywhere.count('/kobot_yolop/status') == 1, (
        '/kobot_yolop/status 구독이 __init__ 밖에도 있다 (누수)')


def test_traffic_perception_keeps_running_during_signal_lock():
    """신호등 노드는 잠금 중에도 **판정을** 멈추지 않는다.

    0823 신호 로직(확정한 신호가 시야에서 사라진 시점부터 억제·잠금을 센다)이
    확정 뒤에도 판정이 계속 들어오는 것을 전제로 튜닝됐다. 일시정지를 다시
    넣으려면 yolop_drive 가 status 에 signal_needed 를 실어 주는 설계가 먼저다.

    status 구독 자체는 이제 있다 — 첫 출발 뒤 탐색 띠를 넓히는 용도다
    (test_roi_started.py). 그 핸들러가 띠만 만지고, 프레임 처리 경로에는
    건너뛰기가 없다는 것을 지킨다."""
    path = os.path.join(PKG_DIR, 'Traffic_Perception.py')
    src = open(path, encoding='utf-8').read()

    handler = src[src.index('def _on_drive_status'):]
    handler = handler[:handler.index('\n    def ', 1)]
    assert '_apply_roi_params' in handler, '핸들러는 탐색 띠를 다시 잡는다'

    body = src[src.index('def on_compressed'):]
    body = body[:body.index('\n    def ', 1)]
    for token in ('signal_needed', 'go_locked', 'signal_lock', '_started'):
        assert token not in body, (
            '프레임 처리 경로에서 판정을 건너뛰면 안 된다: %s' % token)
