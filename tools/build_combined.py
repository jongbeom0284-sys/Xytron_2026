# -*- coding: utf-8 -*-
"""신호등 + 차선을 한 장에 합친 재생 자료를 만든다.

프레임마다
  1. 신호등 노드를 그대로 돌려 이 장의 판정을 낸다
  2. 차선 노드의 확정·잠금 규칙을 그대로 재현해 그 시점 유지 상태를 구한다
     (직진 트리거 켬/끔, 정지 여부)
  3. 그 유지 상태로 차선을 돌려 오버레이를 그린다
     즉 그림이 실제 주행에서 일어났을 판단과 같다

  python3 build_combined.py <기록폴더이름>
"""
import csv
import importlib.util
import json
import sys
import types
from pathlib import Path

import cv2
import numpy as np
import torch

INSTALL = ('/home/xytron/2026_Xytron/real/ros2_ws/install/kobot_yolop_drive/'
           'lib/python3.10/site-packages')
sys.path.insert(0, INSTALL)
from kobot_yolop_drive.lane_core import LaneConfig, LaneTracker  # noqa: E402
from kobot_yolop_drive.trt_backend import TensorRTLane  # noqa: E402

REC = Path('/home/xytron/recordings/camera')
OUT = Path('/home/xytron/lane_eval/replay')
SHOT = OUT / 'combined'
TP = ('/home/xytron/2026_Xytron/real/ros2_ws/src/kobot_vehicle/'
      'kobot_vehicle/Traffic_Perception.py')
NODE = ('/home/xytron/2026_Xytron/real/ros2_ws/install/kobot_yolop_drive/'
        'lib/python3.10/site-packages/kobot_yolop_drive/yolop_drive_node.py')
ENGINE = '/home/xytron/yolotl_offline_eval/models/yolopv2_lane_fp16.engine'

LAMP_BGR = {'red': (60, 60, 255), 'amber': (0, 200, 255),
            'cyan': (200, 255, 60)}


def load_traffic():
    for name in ('rclpy', 'rclpy.executors', 'rclpy.node', 'rclpy.qos',
                 'cv_bridge', 'sensor_msgs', 'sensor_msgs.msg',
                 'std_msgs', 'std_msgs.msg'):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules['rclpy'].__path__ = []
    sys.modules['rclpy.executors'].ExternalShutdownException = Exception
    sys.modules['rclpy.node'].Node = object
    sys.modules['rclpy.qos'].qos_profile_sensor_data = None
    sys.modules['cv_bridge'].CvBridge = object
    sys.modules['sensor_msgs.msg'].CompressedImage = object
    sys.modules['sensor_msgs.msg'].Image = object
    sys.modules['std_msgs.msg'].String = object
    spec = importlib.util.spec_from_file_location('tp_node', TP)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def node_rules():
    """차선 노드 파일에서 확정·잠금 상수를 그대로 읽어온다."""
    text = Path(NODE).read_text(encoding='utf-8')
    out = {}
    for key in ('RED_CONFIRM', 'GO_CONFIRM', 'GO_LOCK_SEC',
                'STOP_RELEASE_SEC'):
        for line in text.splitlines():
            if line.startswith(key + ' '):
                out[key] = float(line.split('=')[1].split('#')[0].strip())
                break
    return out


class Latch:
    """차선 노드 _on_traffic + _on_control 의 상태 기계를 그대로 재현한다."""

    def __init__(self, rules):
        self.red_n = int(rules.get('RED_CONFIRM', 3))
        self.go_n = int(rules.get('GO_CONFIRM', 7))
        self.lock = rules.get('GO_LOCK_SEC', 10.0)
        self.release = rules.get('STOP_RELEASE_SEC', 1.0)
        self.recent = []
        self.straight = False
        self.stop = False
        self.last_red = -1e9
        self.locked_until = -1e9
        self.why = '초기값'

    def feed(self, state, now):
        # _on_control: red 를 오래 못 보면 정지를 스스로 푼다
        if self.stop and (now - self.last_red) > self.release:
            self.stop = False
            self.why = '빨간불 %.1f초 안 보임 — 정지 해제' % self.release

        self.recent.append(state)
        if len(self.recent) > self.go_n:
            self.recent.pop(0)

        def run(want, need):
            tail = self.recent[-need:]
            return len(tail) == need and all(v == want for v in tail)

        if state == 'red':
            if run('red', self.red_n):
                if not self.stop:
                    self.why = '빨간불 %d장 연속 — 정지' % self.red_n
                self.stop = True
                self.last_red = now
            return
        if state not in ('green', 'left'):
            return
        if now < self.locked_until:
            self.why = '잠금 중 — 남은 %.1f초' % (self.locked_until - now)
            return
        if not run(state, self.go_n):
            return
        self.stop = False
        self.locked_until = now + self.lock
        self.recent.clear()
        self.straight = (state == 'green')
        self.why = ('%s %d장 연속 — %s, %.0f초 잠금'
                    % ('초록불' if self.straight else '좌회전', self.go_n,
                       '직진 켬' if self.straight else '직진 끔', self.lock))


def infer(engine, bgr):
    tensor = torch.from_numpy(
        cv2.resize(bgr, (640, 384))[:, :, ::-1].copy()).cuda()
    tensor = tensor.permute(2, 0, 1).unsqueeze(0).float().div_(255.0)
    drive_out, lane_out = engine.infer(tensor.to(engine.input_dtype))
    height, width = bgr.shape[:2]
    lane = cv2.resize((lane_out[0, 0].float() > 0.5).cpu().numpy()
                      .astype(np.uint8), (width, height)) > 0
    road = cv2.resize((drive_out[0].float().argmax(0) == 1).cpu().numpy()
                      .astype(np.uint8), (width, height)) > 0
    return lane, road


def render(bgr, ym, wm, lamps, used):
    """ROI 는 굽지 않는다. 슬라이더로 바꿔야 하므로 페이지에서 그린다."""
    canvas = bgr.copy()
    tint = np.zeros_like(canvas)
    tint[wm] = (255, 150, 60)
    tint[ym] = (0, 210, 255)
    hit = wm | ym
    canvas[hit] = cv2.addWeighted(bgr, 0.42, tint, 0.58, 0)[hit]
    chosen = {id(b) for b in used}
    for lamp in lamps:
        x, y, w, h = lamp['rect']
        pick = id(lamp) in chosen
        color = LAMP_BGR.get(lamp['c'], (200, 200, 200))
        pad = 4 if pick else 2
        cv2.rectangle(canvas, (x - pad, y - pad), (x + w + pad, y + h + pad),
                      color, 2 if pick else 1)
    return canvas


def main():
    name = sys.argv[1]
    src = REC / name
    dst = SHOT / name
    dst.mkdir(parents=True, exist_ok=True)
    for old in dst.glob('*.jpg'):
        old.unlink()
    frames = sorted(p.name for p in src.glob('*.jpg'))

    stamps = {}
    ts = src / 'timestamps.csv'
    if ts.is_file():
        with open(ts, newline='', encoding='utf-8') as handle:
            for row in csv.DictReader(handle):
                stamps[row['filename']] = int(row['ros_time_ns']) / 1e9

    tp = load_traffic()
    rules = node_rules()
    latch = Latch(rules)
    engine = TensorRTLane(ENGINE, torch)
    tracker = LaneTracker(LaneConfig())

    base = stamps.get(frames[0], 0.0)
    roi_px = None
    out = []
    for i, fname in enumerate(frames):
        bgr = cv2.imread(str(src / fname))
        if bgr is None:
            continue
        now = stamps.get(fname, base + i * 0.15)
        lamps, planes = tp.find_lamps(bgr)
        state, used = tp.decide(lamps, planes)
        latch.feed(state, now)

        # ROI 를 잠시 전체로 열어 후보를 전부 모은다. 슬라이더로 ROI 를
        # 바꿨을 때 무엇이 들어오고 나가는지 페이지에서 보여주기 위함이다.
        keep = (tp.ROI_TOP, tp.ROI_BOTTOM, tp.ROI_LEFT, tp.ROI_RIGHT)
        tp.ROI_TOP, tp.ROI_BOTTOM, tp.ROI_LEFT, tp.ROI_RIGHT = 0.0, 1.0, 0.0, 1.0
        every, _ = tp.find_lamps(bgr)
        tp.ROI_TOP, tp.ROI_BOTTOM, tp.ROI_LEFT, tp.ROI_RIGHT = keep

        if roi_px is None:
            roi_px = tp.roi_box(bgr.shape)
        tracker.config.straight_only = latch.straight
        lane, road = infer(engine, bgr)
        yellow_px, _ = tracker._classify_pixels(bgr, lane, road)
        ym, wm, ny, nw, _ = tracker._split_blobs(lane, yellow_px)

        cv2.imwrite(str(dst / fname),
                    render(bgr, ym, wm, lamps, used),
                    [cv2.IMWRITE_JPEG_QUALITY, 88])
        height, width = bgr.shape[:2]
        cand = [dict(x=round(b['rect'][0] / width, 4),
                     y=round(b['rect'][1] / height, 4),
                     w=round(b['rect'][2] / width, 4),
                     h=round(b['rect'][3] / height, 4),
                     c=b['c'], a=b['area']) for b in every]
        out.append(dict(
            f=fname, t=round(now - base, 2), s=state, n=len(lamps),
            straight=latch.straight, stop=latch.stop, why=latch.why,
            lock=round(max(0.0, latch.locked_until - now), 1),
            ypx=int(ym.sum()), yblob=ny, wblob=nw, cand=cand))
        if (i + 1) % 100 == 0:
            print('  %d/%d' % (i + 1, len(frames)), flush=True)

    payload = dict(name=name, rules=rules, frames=out,
                   roi=dict(top=tp.ROI_TOP, bottom=tp.ROI_BOTTOM,
                            left=tp.ROI_LEFT, right=tp.ROI_RIGHT),
                   session=json.loads((src / 'session.json').read_text(
                       encoding='utf-8')) if (src / 'session.json').is_file()
                   else {})
    (OUT / 'combined.json').write_text(json.dumps(payload, ensure_ascii=False),
                                       encoding='utf-8')
    counts = {}
    for item in out:
        counts[item['s']] = counts.get(item['s'], 0) + 1
    on = sum(1 for x in out if x['straight'])
    stopped = sum(1 for x in out if x['stop'])
    print('%d장 · 판정 %s' % (len(out), counts))
    print('직진 유지 %d장 · 정지 유지 %d장' % (on, stopped))


if __name__ == '__main__':
    main()
