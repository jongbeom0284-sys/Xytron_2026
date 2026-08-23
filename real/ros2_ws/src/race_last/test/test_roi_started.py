"""신호 탐색 띠를 첫 출발 뒤에만 넓힌다 (0823 밤).

출발선에서는 신호등이 가까워 좁은 띠로 충분하고, 넓으면 바닥의 라바콘·의자가
램프로 잡힌다. 2·3바퀴는 멀리서 접근하므로 넓혀야 놓치지 않는다.
'첫 출발' = yolop_drive 가 초록을 처음 확정한 순간 (status.started).
"""
import json

from std_msgs.msg import String

import race_last.Traffic_Perception as TP


class Log:
    def info(self, *a, **k): pass
    def warning(self, *a, **k): pass


class Params:
    """get_parameter 만 흉내낸다 — 노드를 띄우지 않고 띠 선택만 본다."""

    def __init__(self, **kw):
        self._v = kw

    def get_parameter(self, name):
        return type('P', (), {'value': self._v[name]})()


class Fake(Params):
    def __init__(self):
        Params.__init__(self, roi_top=0.28, roi_bottom=0.34,
                        roi_bottom_started=0.42, roi_left=0.36,
                        roi_right=0.64)
        self._started = False
        self._log = Log()

    def get_logger(self):
        return self._log

    def _apply_roi_params(self):
        TP.TrafficLightPerception._apply_roi_params(self)

    def status(self, started):
        TP.TrafficLightPerception._on_drive_status(
            self, String(data=json.dumps({'started': started})))


def bottom(f):
    TP.TrafficLightPerception._apply_roi_params(f)
    return TP.ROI_BOTTOM


def test_narrow_before_start():
    """출발 전에는 좁은 띠 — 바닥 라바콘이 램프로 잡히면 안 된다."""
    f = Fake()
    assert bottom(f) == 0.34


def test_wide_after_first_green():
    """첫 초록 확정 뒤에는 넓은 띠."""
    f = Fake()
    bottom(f)
    f.status(True)
    assert TP.ROI_BOTTOM == 0.42
    assert f._started is True


def test_stays_wide():
    """이미 넓힌 뒤 같은 값이 또 와도 그대로다."""
    f = Fake()
    f.status(True)
    f.status(True)
    assert TP.ROI_BOTTOM == 0.42


def test_reset_goes_back_to_narrow():
    """대시보드 reset 으로 출발선에 다시 서면 좁은 띠로 돌아간다."""
    f = Fake()
    f.status(True)
    assert TP.ROI_BOTTOM == 0.42
    f.status(False)
    assert TP.ROI_BOTTOM == 0.34
    assert f._started is False


def test_bad_payload_ignored():
    f = Fake()
    bottom(f)
    TP.TrafficLightPerception._on_drive_status(f, String(data='not json'))
    assert f._started is False
    assert TP.ROI_BOTTOM == 0.34


def test_roi_cache_cleared_on_switch():
    """띠가 바뀌면 마스크 캐시를 비운다 — 안 비우면 옛 띠로 계속 본다."""
    f = Fake()
    bottom(f)
    TP.roi_mask((480, 640, 3))
    assert TP._roi, '캐시가 채워져야 한다'
    f.status(True)
    assert not TP._roi, '띠가 바뀌었으면 캐시는 비어야 한다'
