# -*- coding: utf-8 -*-
"""신호등 판정 방법 전수 비교표.

원칙
  1. 입력은 항상 원본 640x480 전체 프레임. 크롭·확대·전처리 금지.
     모델에는 파일 경로를 그대로 넘긴다. 내부 letterbox 만 허용.
  2. 출력 이미지는 무가공 원본 위에 검출 결과만 덧그린다. PNG 무손실.
  3. 실측 램프 좌표는 표시 프레임 선택과 PWM 진단에만 쓴다.
     어떤 판정 함수에도 넘기지 않는다.
"""
import html
import json
import time
from pathlib import Path

import cv2
import numpy as np

SRC = Path('/home/xytron/signal_shots')
OUT = Path('/home/xytron/lane_eval/replay')
SHOT = OUT / 'signal_grid'
PERF = Path('/home/xytron/lane_eval/perf')

STATES = [('red', '빨간불'), ('yellow', '노란불'), ('green', '초록불'),
          ('left', '좌회전'), ('left_far', '좌회전 · 먼 거리')]

# left_far 는 실제 주행 중 받은 프레임이다. 램프가 8px 로 작다.
# 정답은 left. 여기서 변경 전 코드가 무너졌다.
TRUTH = {'left_far': 'left'}

# 진단 전용. 판정 로직에 절대 전달하지 않는다.
GT_LIT = {'red': (308, 162), 'yellow': (327, 162),
          'green': (368, 162), 'left': (368, 162), 'off': None,
          'left_far': (339, 186)}

ON_SAT, ON_VAL, MIN_ON_PIX = 70, 140, 8
PITCHES = (12, 15, 18, 22, 27, 33, 40)

GREY = (130, 130, 130)
AMBER = (0, 210, 255)
LIME = (110, 230, 130)
SCARLET = (60, 60, 255)
CYAN = (255, 255, 0)
WHITE = (255, 255, 255)


# ---------------------------------------------------------------- 공통 판정
def lit_counts(image, x0, x1, ytop, ybot):
    """박스를 가로 4등분해 칸마다 켜진 픽셀을 센다."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    sat = hsv[:, :, 1].astype(np.int16)
    val = hsv[:, :, 2].astype(np.int16)
    width = (x1 - x0) / 4.0
    counts, cells = [], []
    for i in range(4):
        a = int(round(x0 + width * i))
        b = int(round(x0 + width * (i + 1)))
        c, d = int(ytop), int(ybot)
        cells.append((a, c, b, d))
        a2, c2, b2, d2 = (max(a, 0), max(c, 0),
                          min(b, image.shape[1]), min(d, image.shape[0]))
        if b2 <= a2 or d2 <= c2:
            counts.append(0)
            continue
        mask = (sat[c2:d2, a2:b2] > ON_SAT) & (val[c2:d2, a2:b2] > ON_VAL)
        counts.append(int(mask.sum()))
    return counts, cells


def decide(counts):
    on = [i for i, n in enumerate(counts) if n >= MIN_ON_PIX]
    if not on:
        return 'off'
    if 2 in on and 3 in on:
        return 'left'
    if 3 in on:
        return 'green'
    if 2 in on:
        return 'left'
    if 1 in on:
        return 'yellow'
    return 'red'


def from_box(image, box, layers, note):
    x1, y1, x2, y2 = box
    counts, cells = lit_counts(image, x1, x2, y1, y2)
    layers += [('cell', c, counts[i] >= MIN_ON_PIX, counts[i])
               for i, c in enumerate(cells)]
    return decide(counts), layers, '%s · 칸 %s' % (note, counts)


# ---------------------------------------------------------------- 색도 도구
def chroma_planes(image):
    """정규화 색도. LED 블룸은 청색이 남고 확산 반사면은 안 남는다."""
    soft = cv2.GaussianBlur(image, (3, 3), 0).astype(np.float32)
    total = soft.sum(axis=2) + 1e-6
    nb, ng, nr = (soft[:, :, i] / total for i in range(3))
    return nr, ng, nb, soft.max(axis=2)


def classify(nr, ng, nb):
    """램프 하나의 색 계열. 실측 중앙값 기준.

    LED 는 과노출 블룸으로 흰빛이 섞여 청색 성분이 남는다(nb 0.21~0.36).
    주황 의자 같은 확산 반사면은 청색이 거의 없다(nb 0.137). 램프가 아니다.
    """
    if nb <= 0.175:
        return None
    if ng > 0.385 and nr < 0.30:
        return 'cyan'
    if nr > 0.385 and ng >= 0.355:
        return 'amber'
    if nr > 0.36 and ng < 0.355:
        return 'red'
    return None


# 램프는 원형이다. 둘 다 무차원이라 거리·크기와 무관하다.
MAX_ASPECT, MIN_FILL = 1.8, 0.33


def blobs_from_mask(mask, nr, ng, nb, min_area=6, shape_gate=True):
    count, _, stats, cent = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), 8)
    found = []
    for i in range(1, count):
        x, y, w, h, area = stats[i]
        if area < min_area:
            continue
        aspect = max(w, h) / float(max(min(w, h), 1))
        fill = area / float(max(w * h, 1))
        if shape_gate and (aspect > MAX_ASPECT or fill < MIN_FILL):
            continue
        if not shape_gate and aspect > 6:
            continue
        sub = (slice(y, y + h), slice(x, x + w))
        m = mask[sub]
        if m.sum() == 0:
            continue
        vr = float(np.median(nr[sub][m]))
        vg = float(np.median(ng[sub][m]))
        vb = float(np.median(nb[sub][m]))
        kind = classify(vr, vg, vb)
        found.append(dict(x=float(cent[i][0]), y=float(cent[i][1]),
                          rect=(int(x), int(y), int(x + w), int(y + h)),
                          area=int(area), c=kind, nr=vr, ng=vg, nb=vb,
                          aspect=aspect, fill=fill))
    return found


def decide_from_blobs(blobs):
    """켜진 램프 덩어리의 색과 배치로 상태를 정한다. 박스가 필요 없다."""
    known = [b for b in blobs if b['c']]
    cyan = sorted([b for b in known if b['c'] == 'cyan'],
                  key=lambda b: -b['area'])
    if cyan:
        main = cyan[0]
        rad = max(1.0, (main['area'] / 3.14159) ** 0.5)
        mate = [b for b in cyan[1:]
                if abs(b['y'] - main['y']) < 2.0 * rad
                and 0 < (main['x'] - b['x']) < 8.0 * rad]
        if mate:
            return 'left', [main] + mate, '초록원+왼쪽 동색 덩어리 %d개' % len(mate)
        return 'green', [main], '초록원 단독'
    amber = [b for b in known if b['c'] == 'amber']
    if amber:
        best = max(amber, key=lambda b: b['area'])
        return 'yellow', [best], '주황 덩어리'
    red = [b for b in known if b['c'] == 'red']
    if red:
        best = max(red, key=lambda b: b['area'])
        return 'red', [best], '적색 덩어리'
    return 'off', [], '점등 덩어리 없음'


# ---------------------------------------------------------------- 콤 필터
def comb_best(vchan):
    """가로 등간격 4구 콤 정합. 램프자리 최대 - 램프사이자리 최대."""
    best = None
    for pitch in PITCHES:
        k = max(3, int(round(pitch / 1.8)) | 1)
        lvl = cv2.blur(vchan, (k, k))
        lamp = gap = None
        for off in (-1.5, -0.5, 0.5, 1.5):
            sh = np.roll(lvl, -int(round(pitch * off)), axis=1)
            lamp = sh if lamp is None else np.maximum(lamp, sh)
        for off in (-1.0, 0.0, 1.0):
            sh = np.roll(lvl, -int(round(pitch * off)), axis=1)
            gap = sh if gap is None else np.maximum(gap, sh)
        score = lamp - gap
        edge = int(round(pitch * 1.5)) + k
        score[:, :edge] = -1e4
        score[:, -edge:] = -1e4
        score[lamp < 140] = -1e4
        y, x = np.unravel_index(int(np.argmax(score)), score.shape)
        val = float(score[y, x])
        if best is None or val > best[0]:
            best = (val, int(x), int(y), pitch)
    return best


def comb_box(x, y, pitch):
    return (int(x - 2.65 * pitch), int(y - 0.83 * pitch),
            int(x + 2.65 * pitch), int(y + 0.83 * pitch))


def comb_score_at(vchan, cx, cy):
    """특정 지점에서 가능한 모든 피치의 콤 점수 최대값."""
    best = -1e4
    hgt, wid = vchan.shape
    for pitch in PITCHES:
        k = max(3, int(round(pitch / 1.8)) | 1)
        lvl = cv2.blur(vchan, (k, k))
        for anchor in (-1.5, -0.5, 0.5, 1.5):
            # 이 램프가 blob 이라고 보고 함체 중심을 역산
            ox = int(round(cx - anchor * pitch))
            if ox - 1.5 * pitch - k < 0 or ox + 1.5 * pitch + k >= wid:
                continue
            lamp = max(float(lvl[cy, int(round(ox + o * pitch))])
                       for o in (-1.5, -0.5, 0.5, 1.5))
            gap = max(float(lvl[cy, int(round(ox + o * pitch))])
                      for o in (-1.0, 0.0, 1.0))
            best = max(best, lamp - gap)
    return best


# ---------------------------------------------------------------- YOLO
_models = {}
CACHE = PERF / 'signal_det_cache.json'
_ycache = (json.loads(CACHE.read_text(encoding='utf-8'))
           if CACHE.is_file() else {})
_dirty = [False]


def model(name):
    if name not in _models:
        from ultralytics import YOLO
        _models[name] = YOLO(str(PERF / name))
    return _models[name]


def save_cache():
    if _dirty[0]:
        CACHE.write_text(json.dumps(_ycache), encoding='utf-8')
        _dirty[0] = False


def yolo_boxes(path, weights, size):
    """원본 경로를 그대로 넘긴다. 우리가 자르거나 키우지 않는다."""
    key = '%s|%s|%d' % (path, weights, size)
    if key not in _ycache:
        started = time.perf_counter()
        res = model(weights).predict(path, imgsz=size, classes=[9, 74],
                                     conf=0.005, verbose=False)[0]
        elapsed = (time.perf_counter() - started) * 1000
        got = [([int(v) for v in b.xyxy[0].tolist()], float(b.conf[0]))
               for b in res.boxes]
        _ycache[key] = [got, elapsed]
        _dirty[0] = True
    got, elapsed = _ycache[key]
    return [(tuple(r), c) for r, c in got], elapsed


def yolo_method(weights, size, judge='quarter'):
    def run(image, path, meta):
        started = time.perf_counter()
        got, elapsed = yolo_boxes(path, weights, size)
        layers = [('cand', r, False, c) for r, c in got]
        if not got:
            return 'off', layers, '검출 0개 · %.0fms' % elapsed
        rect, conf = max(got, key=lambda z: z[1])
        layers.append(('anchor', rect, False, conf))
        note = '검출 %d개 · 1등 conf %.3f · %.0fms' % (len(got), conf, elapsed)
        if judge == 'quarter':
            return from_box(image, rect, layers, note)
        return boxed_chroma(image, rect, layers, note, started)
    return run


def world_method(judge='quarter'):
    def run(image, path, meta):
        return method_world(image, path, meta, judge)
    return run


def method_world(image, path, meta, judge='quarter'):
    started = time.perf_counter()
    key = '%s|world|1280' % path
    if key not in _ycache:
        try:
            from ultralytics import YOLO
            net = YOLO(str(PERF / 'yolov8s-worldv2.pt'))
            net.set_classes(['traffic light'])
            started = time.perf_counter()
            res = net.predict(path, imgsz=1280, conf=0.005, verbose=False)[0]
            elapsed = (time.perf_counter() - started) * 1000
        except Exception as err:  # noqa: BLE001
            return 'off', [], '사용 불가: %s' % type(err).__name__
        _ycache[key] = [[([int(v) for v in b.xyxy[0].tolist()],
                          float(b.conf[0])) for b in res.boxes], elapsed]
        _dirty[0] = True
    raw, elapsed = _ycache[key]
    got = [(tuple(r), c) for r, c in raw]
    layers = [('cand', r, False, c) for r, c in got]
    if not got:
        return 'off', layers, '검출 0개 · %.0fms' % elapsed
    rect, conf = max(got, key=lambda z: z[1])
    layers.append(('anchor', rect, False, conf))
    note = '검출 %d개 · 1등 conf %.3f · %.0fms' % (len(got), conf, elapsed)
    if judge == 'quarter':
        return from_box(image, rect, layers, note)
    return boxed_chroma(image, rect, layers, note, started)


def ensemble_method(judge='quarter'):
    def run(image, path, meta):
        return method_ensemble(image, path, meta, judge)
    return run


def method_ensemble(image, path, meta, judge='quarter'):
    """네 모델의 박스를 모아 같은 자리에 동의한 곳을 고른다."""
    started = time.perf_counter()
    specs = [('yolov8n.pt', 1280), ('yolov8s.pt', 1280),
             ('yolov8m.pt', 1280), ('yolo11n.pt', 1280)]
    pool, total = [], 0.0
    for weights, size in specs:
        got, elapsed = yolo_boxes(path, weights, size)
        total += elapsed
        pool += [(r, c, weights) for r, c in got]
    groups = []
    for rect, conf, weights in sorted(pool, key=lambda z: -z[1]):
        cx, cy = (rect[0] + rect[2]) / 2.0, (rect[1] + rect[3]) / 2.0
        for grp in groups:
            if abs(cx - grp['cx']) < 30 and abs(cy - grp['cy']) < 20:
                grp['items'].append((rect, conf))
                grp['nets'].add(weights)
                break
        else:
            groups.append(dict(cx=cx, cy=cy, items=[(rect, conf)],
                               nets={weights}))
    layers = [('cand', r, False, c) for r, c, _ in pool]
    if not groups:
        return 'off', layers, '검출 0개 · %.0fms' % total
    groups.sort(key=lambda g: (len(g['nets']), sum(i[1] for i in g['items'])),
                reverse=True)
    top = groups[0]
    rect = tuple(int(np.mean([i[0][j] for i in top['items']])) for j in range(4))
    layers.append(('anchor', rect, False, len(top['nets'])))
    note = ('군집 %d개 · 최다합의 모델 %d/4 · conf합 %.2f · %.0fms'
            % (len(groups), len(top['nets']),
               sum(i[1] for i in top['items']), total))
    if judge == 'quarter':
        return from_box(image, rect, layers, note)
    return boxed_chroma(image, rect, layers, note, started)


# ---------------------------------------------------------------- 고전 CV
_tpl = None


def template_method(judge='quarter'):
    def run(image, path, meta):
        return method_template(image, path, meta, judge)
    return run


def method_template(image, path, meta, judge='quarter'):
    global _tpl
    if _tpl is None:
        base = cv2.imread(str(sorted((SRC / 'off').glob('*.jpg'))[0]))
        _tpl = cv2.cvtColor(base[150:176, 296:381], cv2.COLOR_BGR2GRAY)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    started = time.perf_counter()
    best = None
    for scale in (0.6, 0.8, 1.0, 1.25, 1.6):
        tpl = cv2.resize(_tpl, None, fx=scale, fy=scale)
        if tpl.shape[0] >= gray.shape[0] or tpl.shape[1] >= gray.shape[1]:
            continue
        resp = cv2.matchTemplate(gray, tpl, cv2.TM_CCOEFF_NORMED)
        _, val, _, loc = cv2.minMaxLoc(resp)
        if best is None or val > best[0]:
            best = (val, loc, tpl.shape[1], tpl.shape[0])
    elapsed = (time.perf_counter() - started) * 1000
    val, (x, y), w, h = best
    rect = (x, y, x + w, y + h)
    layers = [('anchor', rect, False, val)]
    note = '최고 상관 %.3f · %.0fms' % (val, elapsed)
    if judge == 'quarter':
        return from_box(image, rect, layers, note)
    return boxed_chroma(image, rect, layers, note, started)


def housing_method(judge='quarter'):
    def run(image, path, meta):
        return method_housing(image, path, meta, judge)
    return run


def method_housing(image, path, meta, judge='quarter'):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    started = time.perf_counter()
    dark = (gray < 90).astype(np.uint8)
    bar = cv2.morphologyEx(dark, cv2.MORPH_OPEN,
                           cv2.getStructuringElement(cv2.MORPH_RECT, (25, 3)))
    count, _, stats, _ = cv2.connectedComponentsWithStats(bar, 8)
    layers, best = [], None
    for i in range(1, count):
        x, y, w, h, area = stats[i]
        if w < 40 or h < 8 or h > 60 or w < 2.0 * h or area < 250:
            continue
        rect = (x, y, x + w, y + h)
        layers.append(('cand', rect, False, area))
        if best is None or area > best[0]:
            best = (area, rect)
    elapsed = (time.perf_counter() - started) * 1000
    if best is None:
        return 'off', layers, '가로 함체 후보 없음 · %.0fms' % elapsed
    layers.append(('anchor', best[1], False, best[0]))
    note = '함체 후보 %d개 · %.0fms' % (len(layers) - 1, elapsed)
    if judge == 'quarter':
        return from_box(image, best[1], layers, note)
    return boxed_chroma(image, best[1], layers, note, started)


def ring_method(judge='quarter'):
    def run(image, path, meta):
        return method_ring(image, path, meta, judge)
    return run


def method_ring(image, path, meta, judge='quarter'):
    """밝은 원 중 둘레가 어두운 것만 고른다. 형광등은 둘레도 밝아 탈락한다."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.int16)
    nr, ng, nb, _ = chroma_planes(image)
    started = time.perf_counter()
    mask = (hsv[:, :, 1] > ON_SAT) & (hsv[:, :, 2] > ON_VAL)
    count, _, stats, cent = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), 8)
    layers, kept = [], []
    for i in range(1, count):
        x, y, w, h, area = stats[i]
        if area < 12:
            continue
        aspect = max(w, h) / float(max(min(w, h), 1))
        fill = area / float(max(w * h, 1))
        rect = (x, y, x + w, y + h)
        layers.append(('cand', rect, False, area))
        if aspect > MAX_ASPECT or fill < MIN_FILL:
            continue
        radius = 0.5 * (w + h) / 2.0
        inner = float(gray[y:y + h, x:x + w].mean())
        pad = int(radius * 1.8) + 2
        ay, by = max(y - pad, 0), min(y + h + pad, gray.shape[0])
        ax, bx = max(x - pad, 0), min(x + w + pad, gray.shape[1])
        win = gray[ay:by, ax:bx].copy()
        win[y - ay:y - ay + h, x - ax:x - ax + w] = -1
        ring = win[win >= 0]
        if ring.size < 10:
            continue
        contrast = inner - float(ring.mean())
        if contrast <= 40:
            continue
        sub = (slice(y, y + h), slice(x, x + w))
        m = mask[sub]
        blob = dict(x=float(cent[i][0]), y=float(cent[i][1]), rect=rect,
                    area=int(area), radius=radius, contrast=contrast,
                    nr=float(np.median(nr[sub][m])),
                    ng=float(np.median(ng[sub][m])),
                    nb=float(np.median(nb[sub][m])),
                    aspect=aspect, fill=fill)
        blob['c'] = classify(blob['nr'], blob['ng'], blob['nb'])
        kept.append(blob)
    if not kept:
        elapsed = (time.perf_counter() - started) * 1000
        return 'off', layers, ('후보 %d개 · 링대비 통과 0개 · %.0fms'
                               % (len(layers), elapsed))
    head = '후보 %d개 중 링대비 통과 %d개 · 최고대비 %.0f' % (
        len(layers), len(kept), max(b['contrast'] for b in kept))
    if judge == 'quarter':
        # 반지름으로 램프 간격을 역산해 함체 박스를 그리고 4등분한다.
        best = max(kept, key=lambda b: b['contrast'])
        box = comb_box(best['x'], best['y'], 3.6 * best['radius'])
        layers.append(('anchor', box, False, best['contrast']))
        elapsed = (time.perf_counter() - started) * 1000
        return from_box(image, box, layers, '%s · %.0fms' % (head, elapsed))
    # 베젤이 잡혔으면 그 안의 색을 바로 읽는다. 위치를 추측하지 않는다.
    state, used, why = decide_from_blobs(kept)
    elapsed = (time.perf_counter() - started) * 1000
    layers += [('blob', b['rect'], True, b['area']) for b in used]
    return state, layers, '%s · %s' % (head, blob_note(kept, why, elapsed,
                                                       head='통과'))


# ---------------------------------------------------------------- 새 방법
def method_comb(image, path, meta):
    """M1. 램프 자리는 밝고 램프 사이는 어두워야 한다는 구조만 본다."""
    hsv = cv2.cvtColor(cv2.GaussianBlur(image, (3, 3), 0), cv2.COLOR_BGR2HSV)
    val = hsv[:, :, 2].astype(np.float32)
    started = time.perf_counter()
    best = comb_best(val)
    elapsed = (time.perf_counter() - started) * 1000
    if best is None or best[0] < 25:
        return 'off', [], '콤 응답 없음 · %.0fms' % elapsed
    score, x, y, pitch = best
    box = comb_box(x, y, pitch)
    return from_box(image, box, [('anchor', box, False, score)],
                    '콤점수 %.0f · 피치 %dpx · %.0fms' % (score, pitch, elapsed))


def method_stable(image, path, meta):
    """M2. 임계값을 낮춰도 안 커지는 밝은 점. 어두운 함체에 갇힌 LED 서명."""
    started = time.perf_counter()
    nr, ng, nb, val = chroma_planes(image)
    hi = (val > 200)
    lo = (val > 110).astype(np.uint8)
    nlo, llo, slo, _ = cv2.connectedComponentsWithStats(lo, 8)
    cand = blobs_from_mask(hi, nr, ng, nb, min_area=8)
    layers, keep = [], []
    for blob in cand:
        layers.append(('cand', blob['rect'], False, blob['area']))
        idx = int(llo[int(blob['y']), int(blob['x'])])
        grow = (slo[idx][4] / float(blob['area'])) if idx > 0 else 999.0
        blob['grow'] = grow
        if grow < 4.0:
            keep.append(blob)
    state, used, why = decide_from_blobs(keep)
    elapsed = (time.perf_counter() - started) * 1000
    layers += [('blob', b['rect'], True, b['area']) for b in used]
    grows = ' '.join('%.1f' % b['grow'] for b in cand[:6])
    return state, layers, ('밝은점 %d개 중 안정 %d개 · 성장비[%s] · %s · %.0fms'
                           % (len(cand), len(keep), grows, why, elapsed))


def chroma_blobs(image, gates):
    """점등 램프 덩어리를 찾는다. gates 로 조건을 한 단계씩 켠다.

    'v'      밝기만
    'vs'     밝기 + 채도       (기존 HSV 방식이 쓰던 조건)
    'vc'     밝기 + 정규화색도  (의자 배제)
    'vcr'    밝기 + 색도 + 원형성 (최종)
    """
    nr, ng, nb, val = chroma_planes(image)
    lit = val > 150
    if 's' in gates:
        hsv = cv2.cvtColor(cv2.GaussianBlur(image, (3, 3), 0), cv2.COLOR_BGR2HSV)
        lit &= hsv[:, :, 1] > ON_SAT
    if 'c' in gates:
        spread = (np.maximum(np.maximum(nr, ng), nb)
                  - np.minimum(np.minimum(nr, ng), nb))
        lit &= (nb > 0.175) & (spread > 0.045)
    return blobs_from_mask(lit, nr, ng, nb, 6, shape_gate=('r' in gates))


def blob_note(blobs, why, elapsed, head='점등후보'):
    detail = ' '.join('%s a%d nb%.2f 비%.1f 채%.2f'
                      % (b['c'] or '?', b['area'], b['nb'],
                         b['aspect'], b['fill'])
                      for b in sorted(blobs, key=lambda z: -z['area'])[:3])
    return '%s %d개 [%s] · %s · %.0fms' % (head, len(blobs), detail, why, elapsed)


def chroma_method(gates):
    def run(image, path, meta):
        started = time.perf_counter()
        blobs = chroma_blobs(image, gates)
        state, used, why = decide_from_blobs(blobs)
        elapsed = (time.perf_counter() - started) * 1000
        layers = [('cand', b['rect'], False, b['area']) for b in blobs]
        layers += [('blob', b['rect'], True, b['area']) for b in used]
        return state, layers, blob_note(blobs, why, elapsed)
    return run


method_chroma = chroma_method('vcr')


def in_box(blob, box, slack=0.3):
    """검출 박스 안(살짝 여유)에 중심이 들어오는 덩어리만 남긴다. 자르지 않는다."""
    x1, y1, x2, y2 = box
    px, py = slack * max(1, x2 - x1), slack * max(1, y2 - y1)
    return (x1 - px) <= blob['x'] <= (x2 + px) and (y1 - py) <= blob['y'] <= (y2 + py)


def boxed_chroma(image, box, layers, note, started):
    """검출 박스를 4등분하는 대신, 그 안의 램프 덩어리를 색·배치로 읽는다."""
    blobs = chroma_blobs(image, 'vcr')
    inside = [b for b in blobs if in_box(b, box)]
    state, used, why = decide_from_blobs(inside)
    elapsed = (time.perf_counter() - started) * 1000
    layers += [('cand', b['rect'], False, b['area']) for b in inside]
    layers += [('blob', b['rect'], True, b['area']) for b in used]
    return state, layers, '%s · %s' % (note, blob_note(inside, why, elapsed,
                                                       head='박스안 후보'))


def method_cross(image, path, meta):
    """M5. M3 색도 후보를 M1 콤 구조로 재검증. 둘 다 통과해야 신호등."""
    started = time.perf_counter()
    blobs = chroma_blobs(image, 'vcr')
    hsv = cv2.cvtColor(cv2.GaussianBlur(image, (3, 3), 0), cv2.COLOR_BGR2HSV)
    vch = hsv[:, :, 2].astype(np.float32)
    layers, keep = [], []
    for blob in blobs:
        layers.append(('cand', blob['rect'], False, blob['area']))
        blob['comb'] = comb_score_at(vch, int(blob['x']), int(blob['y']))
        if blob['comb'] > 30:
            keep.append(blob)
    state, used, why = decide_from_blobs(keep)
    elapsed = (time.perf_counter() - started) * 1000
    layers += [('blob', b['rect'], True, b['area']) for b in used]
    combs = ' '.join('%.0f' % b['comb'] for b in blobs[:5])
    return state, layers, ('색도후보 %d개 중 콤통과 %d개 · 콤점수[%s] · %s · %.0fms'
                           % (len(blobs), len(keep), combs, why, elapsed))


# --------------------------------------------------- 실제 노드 코드 그대로
NODE_PATH = ('/home/xytron/2026_Xytron/real/ros2_ws/src/kobot_vehicle/'
             'kobot_vehicle/Traffic_Perception.py')
_node = [None]


def node_module():
    """돌고 있는 ROS 노드 파일을 그대로 불러온다. 사본이 아니다."""
    if _node[0] is None:
        import importlib.util
        import sys
        import types
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
        spec = importlib.util.spec_from_file_location('tp_node', NODE_PATH)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _node[0] = mod
    return _node[0]


def method_node(image, path, meta):
    """지금 차에서 도는 노드가 이 사진에 어떻게 답하는가."""
    mod = node_module()
    started = time.perf_counter()
    lamps, planes = mod.find_lamps(image)
    state, used = mod.decide(lamps, planes)
    elapsed = (time.perf_counter() - started) * 1000
    if state == 'none':
        state = 'off'
    layers = [('cand', (b['rect'][0], b['rect'][1],
                        b['rect'][0] + b['rect'][2], b['rect'][3] + b['rect'][1]),
               False, b['area']) for b in lamps]
    layers += [('blob', (b['rect'][0], b['rect'][1],
                         b['rect'][0] + b['rect'][2], b['rect'][1] + b['rect'][3]),
                True, b['area']) for b in used]
    detail = ' '.join('%s a%d ng-nr%.3f 비%.2f'
                      % (b['c'], b['area'], b.get('ng', 0) - b.get('nr', 0),
                         b['aspect']) for b in used)
    return state, layers, ('점등램프 %d개 · 채택 %d개 [%s] · %.0fms'
                           % (len(lamps), len(used), detail, elapsed))


# ---------------------------------------------------------------- 목록
METHODS = [
    ('NODE', '지금 도는 노드', 'Traffic_Perception.py 를 그대로 불러 돌린다 (ng-nr 판정 + 화살표 자리 탐색)', method_node, 'cv'),
    ('M3', '변경 전 색도+원형성', 'ng>0.385 and nr<0.30 절대 문턱값 · 화살표도 원형성 요구', chroma_method('vcr'), 'cv'),
    ('N3', '링 대비', '밝은 원 중 둘레가 어두운 것 → 그 안의 색을 바로 읽음', ring_method('chroma'), 'cv'),
    ('M5', '색도 × 콤', '변경 전 색도 후보를 등간격 콤 구조로 재검증', method_cross, 'cv'),
]




# ---------------------------------------------------------------- 그리기
def draw(image, layers):
    """무가공 원본 복사본 위에 결과만 덧그린다. 크기·색 보정 없음."""
    canvas = image.copy()
    for kind, rect, ok, val in layers:
        x1, y1, x2, y2 = [int(v) for v in rect]
        if kind == 'cand':
            cv2.rectangle(canvas, (x1, y1), (x2, y2), GREY, 1)
        elif kind == 'anchor':
            cv2.rectangle(canvas, (x1, y1), (x2, y2), AMBER, 2)
        elif kind == 'cell':
            cv2.rectangle(canvas, (x1, y1), (x2, y2),
                          LIME if ok else (90, 90, 90), 1)
            if ok:
                cv2.putText(canvas, str(val), (x1 + 1, y2 + 11),
                            cv2.FONT_HERSHEY_PLAIN, 0.8, LIME, 1)
        elif kind == 'blob':
            cv2.rectangle(canvas, (x1 - 3, y1 - 3), (x2 + 3, y2 + 3), CYAN, 2)
    return canvas


def frame_score(image, spot):
    if spot is None:
        return 0.0
    x, y = spot
    return float(image[max(y - 4, 0):y + 5, max(x - 4, 0):x + 5].max())


# ---------------------------------------------------------------- 실행
def main():
    SHOT.mkdir(parents=True, exist_ok=True)
    for old in SHOT.glob('*.png'):
        old.unlink()

    bank = {}
    for key, _ in STATES:
        paths = sorted(str(p) for p in (SRC / key).iterdir()
                       if p.suffix.lower() in ('.jpg', '.png'))
        images = [cv2.imread(p) for p in paths]
        scores = [frame_score(im, GT_LIT[key]) for im in images]
        pick = 0 if GT_LIT[key] is None else int(np.argmax(scores))
        dark = sum(1 for s in scores if GT_LIT[key] and s < 150)
        bank[key] = dict(paths=paths, images=images, pick=pick, dark=dark,
                         n=len(paths))
        print('%-7s %d장 · 표시프레임 #%d · PWM으로 어두운 장 %d'
              % (key, len(paths), pick, dark), flush=True)

    table, totals = {}, {}
    for code, name, how, fn, group in METHODS:
        hits = shots = ohit = oshot = 0
        for key, _ in STATES:
            info = bank[key]
            verdicts = []
            for path, image in zip(info['paths'], info['images']):
                try:
                    state, _, note = fn(image, path, dict(state=key))
                except Exception as err:  # noqa: BLE001
                    state, note = '오류', '%s: %s' % (type(err).__name__, err)
                verdicts.append((state, note))
            want = TRUTH.get(key, key)
            good = sum(1 for st, _ in verdicts if st == want)
            bad = [i for i, (st, _) in enumerate(verdicts) if st != want]
            # 틀린 장이 있으면 그 장을 보여준다. 없으면 대표 프레임.
            show = bad[0] if bad else info['pick']
            image, path = info['images'][show], info['paths'][show]
            try:
                state, layers, note = fn(image, path, dict(state=key))
            except Exception as err:  # noqa: BLE001
                state, layers, note = '오류', [], str(err)
            out = SHOT / ('%s_%s.png' % (key, code.replace('★', 'S')))
            cv2.imwrite(str(out), draw(image, layers))
            cell = dict(state=state, note=note,
                        img='signal_grid/%s' % out.name,
                        good=good, total=info['n'], bad=len(bad),
                        shown=show, isbad=bool(bad),
                        frame=Path(path).name,
                        others=' '.join('#%d %s' % (i, st)
                                        for i, (st, _) in enumerate(verdicts)))
            table['%s|%s' % (code, key)] = cell
            if key == 'off':
                ohit, oshot = good, info['n']
            else:
                hits += good
                shots += info['n']
            print('  %-5s %-7s %d/%d  %s' % (code, key, good, info['n'],
                                             cell['state']), flush=True)
        totals[code] = (hits, shots, ohit, oshot)
        print('%-5s 점등 %d/%d · 소등 %d/%d' % (code, hits, shots, ohit, oshot),
              flush=True)

    save_cache()
    payload = dict(table=table, totals=totals,
                   bank={k: dict(pick=v['pick'], dark=v['dark'], n=v['n'])
                         for k, v in bank.items()})
    (OUT / 'signal_grid.json').write_text(
        json.dumps(payload, ensure_ascii=False), encoding='utf-8')
    write_page(table, totals, bank)
    print('완료')


# ---------------------------------------------------------------- 페이지
def write_page(table, totals, bank):
    rows = []
    order = sorted(METHODS, key=lambda m: (-totals[m[0]][0], m[0]))
    for code, name, how, _, group in order:
        hit, shot, ohit, oshot = totals[code]
        pct = 100.0 * hit / shot
        tone = 'good' if pct >= 90 else ('mid' if pct >= 50 else 'bad')
        cells = []
        for key, _ in STATES:
            cell = table['%s|%s' % (code, key)]
            ok = not cell['isbad']
            tag = ('<div class="badtag">오답 프레임 #%d (%s) · 12장 중 %d장 오답</div>'
                   % (cell['shown'], cell['frame'], cell['bad'])
                   if cell['isbad'] else
                   '<div class="oktag">12장 전부 정답 · 표시 #%d (%s)</div>'
                   % (cell['shown'], cell['frame']))
            cells.append(
                '<td class="%s">%s<a href="%s" target="_blank">'
                '<img src="%s" loading="lazy"></a>'
                '<div class="verdict">%s 이 장의 판정 <b>%s</b></div>'
                '<div class="acc">12장 중 %d장 정답</div>'
                '<div class="note">%s</div>'
                '<div class="note frames">장별 판정 %s</div></td>'
                % ('hit' if ok else 'miss', tag, cell['img'], cell['img'],
                   '✓' if ok else '✗', html.escape(cell['state']),
                   cell['good'], html.escape(cell['note']),
                   html.escape(cell['others'])))
        rows.append(
            '<tr class="g-%s"><th><div class="code">%s</div>'
            '<div class="name">%s</div><div class="how">%s</div>'
            '<div class="tot %s">%d/%d · %.0f%%</div></th>%s</tr>'
            % (group, code, html.escape(name), html.escape(how), tone,
               hit, shot, pct, ''.join(cells)))

    head = ''.join('<th><div class="st">%s</div><div class="sub">%s</div>'
                   '<div class="sub">PWM 어두운 장 %d/%d</div></th>'
                   % (ko, key, bank[key]['dark'], bank[key]['n'])
                   for key, ko in STATES)

    page = PAGE % dict(head=head, rows=''.join(rows))
    (OUT / 'signal_all.html').write_text(page, encoding='utf-8')
    (OUT / 'signal_grid.html').write_text(page, encoding='utf-8')


PAGE = '''<!doctype html><html lang="ko"><meta charset="utf-8">
<title>신호등 판정 방법 비교</title>
<style>
:root{color-scheme:dark}
body{background:#0d0f12;color:#e6e8ea;font:13px/1.5 system-ui,"Malgun Gothic",sans-serif;margin:0;padding:18px}
h1{font-size:19px;margin:0 0 4px}
.lead{color:#9aa3ad;max-width:1100px;margin:0 0 14px}
.lead b{color:#e6e8ea}
.rule{background:#14181d;border:1px solid #232a32;border-radius:6px;padding:10px 13px;margin:0 0 16px;max-width:1100px}
.rule li{margin:3px 0;color:#b8c0c8}
table{border-collapse:separate;border-spacing:0;width:max-content}
th,td{border:1px solid #232a32;vertical-align:top}
thead th{position:sticky;top:0;background:#171c22;z-index:3;padding:8px 10px;text-align:left}
tbody th{position:sticky;left:0;background:#171c22;z-index:2;padding:9px 11px;text-align:left;width:210px;min-width:210px}
thead th:first-child{z-index:4;left:0}
td{padding:7px;width:330px;background:#101317}
td.hit{background:#0f1a12}
td.miss{background:#1a1113}
img{width:320px;display:block;border-radius:3px}
.st{font-size:15px;font-weight:600}
.sub{color:#8b949e;font-size:11px;font-weight:400}
.code{font-size:15px;font-weight:700;color:#7fd1ff}
.name{font-size:13px;margin-top:1px}
.how{color:#8b949e;font-size:11px;margin-top:3px}
.tot{margin-top:6px;font-size:12px;font-weight:600;padding:2px 6px;border-radius:3px;display:inline-block}
.tot.good{background:#12351f;color:#78e09a}
.tot.mid{background:#3a3113;color:#e2c05a}
.tot.bad{background:#3a1519;color:#f08a92}
.off{color:#6f7780;font-size:11px;margin-top:3px}
tr.g-step tbody th,tbody th{border-left:3px solid #232a32}
tr.g-step th{border-left:3px solid #d08b3a!important}
tr.g-yolo th{border-left:3px solid #4a7fb5!important}
tr.g-cv th{border-left:3px solid #6a9c5a!important}

.verdict{margin-top:6px;font-size:13px}
td.hit .verdict{color:#78e09a}
td.miss .verdict{color:#f08a92}
.acc{color:#9aa3ad;font-size:11px}
.badtag{background:#4a161c;color:#ffb3ba;font-size:11px;font-weight:600;padding:3px 6px;border-radius:3px;margin-bottom:5px}
.oktag{background:#13301e;color:#8fe0a8;font-size:11px;padding:3px 6px;border-radius:3px;margin-bottom:5px}
.frames{color:#5f676f;font-size:10px}
.note{color:#7d868f;font-size:11px;margin-top:3px;word-break:break-all;line-height:1.45}
.legend span{display:inline-block;margin-right:14px;color:#9aa3ad;font-size:12px}
.sw{display:inline-block;width:11px;height:11px;border-radius:2px;vertical-align:-1px;margin-right:4px}
</style>
<h1>신호등 판정 방법 비교 &mdash; 행 = 방법, 열 = 신호 상태</h1>
<p class="lead">48장(상태당 12장) 전부에 대해 각 방법을 돌린 결과. 정확도는 <b>12장 전부</b>에 대한 값이고,
그림은 <b>틀린 장이 있으면 그 틀린 장</b>, 없으면 대표 프레임이다. 셀 맨 아래 &ldquo;장별 판정&rdquo;에 12장 각각의 답이 있다.</p>
<p class="lead"></p>
<div class="rule"><ul>
<li>입력은 <b>원본 640&times;480 전체 프레임</b>. 크롭·확대·색보정 없이 파일 경로를 그대로 모델에 넘긴다.</li>
<li>그림은 <b>무가공 원본 위에 검출 결과만 덧그린 것</b>. PNG 무손실. 클릭하면 원본 크기.</li>
<li>어떤 방법에도 램프 좌표를 알려주지 않는다. 실측 좌표는 <b>표시할 프레임 고르기와 PWM 진단</b>에만 썼다.</li>
<li>★ 행의 색도 판정도 <b>전체 프레임에서</b> 램프 덩어리를 찾은 뒤 검출 박스 안에 드는 것만 남긴다.
이미지를 잘라 다시 처리하지 않는다.</li>
<li>LED가 PWM으로 깜빡여 12장 중 일부는 실제로 꺼져 있다. 열 머리글의 &ldquo;PWM 어두운 장&rdquo;이 그 수다.</li>
</ul></div>
<p class="legend">
<span><i class="sw" style="background:#00d2ff"></i>노랑 굵은 테두리 = 1등 검출 박스</span>
<span><i class="sw" style="background:#828282"></i>회색 얇은 테두리 = 나머지 후보</span>
<span><i class="sw" style="background:#86e682"></i>연두 칸 = 4등분 중 켜졌다고 본 칸</span>
<span><i class="sw" style="background:#00ffff"></i>청록 테두리 = 판정 근거로 쓴 램프 덩어리</span>
</p>
<table><thead><tr><th>방법</th>%(head)s</tr></thead>
<tbody>%(rows)s</tbody></table>
</html>'''


if __name__ == '__main__':
    main()
