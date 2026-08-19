"""차선 주행 모듈이 공유하는 순수 계산 헬퍼.

ROS 와 OpenCV 에 의존하지 않는다. 어떤 노드에서도 import 할 수 있고
단위 시험에서 그대로 호출할 수 있어야 한다.
"""

from __future__ import annotations


def move_towards(current: float, target: float, maximum_step: float) -> float:
    """한 주기에 움직일 수 있는 양을 maximum_step 으로 제한해 current 를 target 쪽으로 옮긴다.

    조향과 횡방향 목표를 같은 방식으로 완만하게 만들기 위해 쓴다. 상한을 거는 것(cap)과
    변화율을 거는 것(rate limit)은 다르다. 여기서는 변화율만 건다.
    """
    if maximum_step <= 0.0:
        return current
    if target > current:
        return min(target, current + maximum_step)
    return max(target, current - maximum_step)


def clamp(value: float, minimum: float, maximum: float) -> float:
    """value 를 [minimum, maximum] 안으로 자른다."""
    return max(minimum, min(maximum, value))


def blend(current: float, target: float, alpha: float) -> float:
    """1차 저역통과. alpha 가 클수록 target 을 빨리 따라간다."""
    return current + clamp(alpha, 0.0, 1.0) * (target - current)


# 이전 이름과의 호환. 새 코드는 move_towards 를 쓴다.
_move_towards = move_towards
