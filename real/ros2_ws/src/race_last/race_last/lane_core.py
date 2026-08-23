"""YOLOPv2 기반 차선 목표 생성 + 조향 계산 (순수 로직, ROS 의존 없음).

파이프라인
  1) YOLOPv2 -> lane / road 마스크
  2) road 를 무채색 기준으로 화이트밸런스 정규화
  3) 차선 픽셀 중 노란 것을 가려냄 (b* 축). 흰색은 따로 분류하지 않고
     4)에서 "노랑 씨앗이 없는 덩이"로 판정한다
  4) ROI 상단 절단 후 연결요소 분리.
     덩이 안에 노란 픽셀이 충분하면 그 덩이 전체가 노란 중앙선이다.
     (YOLOPv2 가 점선을 이어 그리므로 끊김을 신경 쓸 필요가 없다)
  5) 목표선 결정
       노란 덩이 있으면  -> 그 덩이의 행별 중심
       없으면            -> 흰 덩이 2개의 행별 중점
  6) 2차 다항식으로 피팅하고 먼 행/가까운 행에서 샘플
  7) 조향 = 먼 오차 위주 + 가까운 오차 소량 + 미분 감쇠
     -> 먼 곳을 보므로 직선에서 덜 흔들리고 커브를 미리 읽는다

조향 부호와 단위는 차량의 /xycar_motor legacy 단위(-40..40 권장)를 따른다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class LaneConfig:
    # --- 검출 ---
    roi_top_ratio: float = 0.58      # 이 위로는 버린다
    min_blob_area: int = 150
    min_yellow_px: int = 25          # 덩이를 노랑으로 볼 최소 노란 픽셀 수
    # 횡단보도 배제. 두 조건을 모두 만족해야 버린다.
    # 0 이하로 두면 이 검사를 끈다.
    # 가로로 이만큼 긴 연속구간은 차선이 아니라 횡단보도의 가로선으로 본다.
    # 잘라내고 나서 덩이를 나누므로 좌우 흰선이 노랑으로 이어지지 않는다.
    # 실측: 정상 차선 최대 런 25~82, 횡단보도 다리는 350 이상.
    # 0 이하로 두면 절단하지 않는다.
    max_row_run: int = 80
    crosswalk_row_width: float = 32.0   # 행별 가로폭 중앙값이 이보다 크고
    crosswalk_aspect: float = 0.30      # 세로/가로가 이보다 작으면 횡단보도
    yellow_b: int = 20               # b* 중립 대비 초과량
    # --- 라바콘 주황을 노란 씨앗에서 뺀다 (0823 저녁) ---
    # 라바콘은 주황이라 LAB b* 축에서도 양수다. 그래서 차선 마스크와 겹친
    # 콘 화소가 '노란 중앙선 씨앗'이 되어, 그 덩이에 붙은 흰선까지 노랑으로
    # 물들인다 (실측 195303/f130: 콘 화소 14797개 중 50개가 씨앗이 되어
    # 흰선 덩이 442화소를 노랑으로 만들었다).
    # 노란 중앙선과 라바콘은 **색상(Hue)이 다르다** — 노랑 H≈25~35,
    # 콘 주황 H 3~12 (OpenCV 0~179). 그 구간을 씨앗에서 제외한다.
    # 콘 진입 트리거(cone_trigger_px)는 별도 경로라 영향받지 않는다.
    exclude_cone_orange: bool = True
    cone_hue_lo: int = 3
    cone_hue_hi: int = 12
    cone_sat_min: int = 110          # 채도가 낮으면 주황이 아니다 (그림자·노면)
    # --- 직진 모드: 노랑을 세로로만 전파 ---
    # 삼거리에서 좌회전 도로의 노란 중앙선이 직진 도로의 좌측 흰선과 마스크상
    # 붙어 있다. 덩이 전체에 노랑을 퍼뜨리면 흰선까지 물든다. 그 오염 덕분에
    # 트리거가 없을 때는 차가 좌측으로 따라간다. 그래서 기본값은 꺼짐이다.
    # 초록불(직진)일 때만 켜서, 노랑이 세로로만 번지게 한다.
    straight_only: bool = False       # 노랑 오염 방지 (세로 전파 + 흰색 되덮기)
    # --- 직진 구간: 노란선을 아예 보지 않고 흰선 중앙만 따라간다 ---
    # 0823 저녁 요구. 삼거리에서 좌회전 도로의 노란 중앙선이 직진 도로 좌측
    # 흰선과 붙어 있어, 노란선을 목표로 삼는 한 어떤 방식이든 좌측으로 끌린다.
    # 그래서 직진 확정 구간에서는 노란선 자체를 목표에서 뺀다.
    # 이 구간은 초록불 확정 ~ 콘 진입 트리거 사이이고, 노드가 매 프레임 넣는다.
    # ★ 반드시 straight_only 와 **함께** 켠다. 오염 방지가 없으면 흰선이 노랑
    #   덩이에 먹혀 white_mask 에서 빠지고, 흰선이 2줄로 안 잡혀 이 모드가
    #   성립하지 않는다 (0823 프레임309 실측: 흰 덩이 2개 -> 1개).
    # 꺼지면(콘 트리거) 노란선 우선 추종으로 돌아간다 — 그쪽이 정상 동작이다.
    white_center_only: bool = False
    grow_height: int = 21             # 세로 커널 높이. 한 번에 10px 전진
    grow_width: int = 3               # 가로 커널 폭. 한 번에 1px 옆으로
    grow_iters: int = 40              # 최대 반복. 더 안 자라면 일찍 끝난다
    # 세로 전파 뒤 흰선을 한 번 더 퍼뜨려 오염된 노랑을 되덮는다.
    # 실제 노란 화소(씨앗)에서 막히므로 진짜 중앙선은 안 지워진다.
    white_reclaim: bool = True
    # --- 차량 회피: 한쪽 흰선에 붙어 달린다 ---
    # 0 이면 평소대로 (좌우 흰선 한가운데 또는 노란선 추종).
    # 양수면 우측 흰선에서 왼쪽으로 그만큼, 음수면 좌측 흰선에서 오른쪽으로 그만큼.
    # 노드가 회피 상태에 따라 매 프레임 넣는다.
    avoid_offset_px: float = 0.0
    white_dl: int = 10               # 노면 대비 밝기차
    white_tophat: int = 12
    chroma_max: int = 18

    # --- 목표선 샘플 위치 (화면 세로 비율) ---
    y_far_ratio: float = 0.64        # 먼 곳. 조향의 주 입력
    y_near_ratio: float = 0.88       # 가까운 곳. 정상상태 오차 보정

    # --- 조향 ---
    # 먼 오차 이득. 구간별로 나눈다.
    #   |e| <= knee : kp_far_lo 기울기 (직선. 낮게 잡아 사행을 막는다)
    #   |e| >  knee : kp_far_hi 기울기 (코너. 높게 잡아 확실히 꺾는다)
    # kp_far_hi <= 0 이면 kp_far 를 쓰는 예전 선형 동작으로 되돌아간다.
    kp_far: float = 26.0             # 선형 폴백용
    kp_far_lo: float = 12.0
    kp_far_hi: float = 90.0
    knee_norm: float = 0.20
    kp_near: float = 6.0             # 가까운 오차 이득 (작게)
    kd: float = 8.0                  # 오차 변화율 감쇠
    deadband_norm: float = 0.035     # 이 안쪽은 조향하지 않는다
    max_steer: float = 40.0          # legacy 단위 상한
    max_rate: float = 140.0           # 초당 legacy 단위 변화 상한
    filter_alpha: float = 0.45       # 저역통과 (낮을수록 부드럽고 느리다)
    steering_sign: float = -1.0      # 차량 방향에 맞춰 뒤집는다

    # --- 유지 ---
    hold_seconds: float = 0.8        # 목표를 놓쳤을 때 직전 값을 유지할 시간
    min_fit_rows: int = 6            # 피팅에 필요한 최소 행 수


def cut_horizontal_bridges(lane_roi: np.ndarray,
                           config: LaneConfig) -> np.ndarray:
    """가로로 아주 긴 연속구간을 차선 마스크에서 잘라낸다.

    횡단보도의 가로 노란선은 좌우 흰선을 잇는 다리가 되어
    노랑 전파가 흰선까지 번지게 만든다. 그 다리만 끊는다.
    가로 선분 구조요소로 opening 하면 그 길이 이상인 런만 남으므로,
    남은 것을 원본에서 빼면 된다. 곡선 차선은 런이 짧아 손대지 않는다.
    """
    if config.max_row_run <= 0:
        return lane_roi
    kernel = np.ones((1, config.max_row_run + 1), np.uint8)
    long_runs = cv2.morphologyEx(lane_roi.astype(np.uint8),
                                 cv2.MORPH_OPEN, kernel)
    return lane_roi & ~(long_runs > 0)


def grow_vertical(blob: np.ndarray, seed: np.ndarray,
                  config: LaneConfig) -> np.ndarray:
    """씨앗에서 세로 방향으로만 노랑을 넓힌다. 덩이 밖으로는 안 나간다.

    커널이 세로로 길고 가로로 좁다. 한 번에 세로 (h-1)/2, 가로 (w-1)/2 만큼
    번지므로 그 비가 허용 기울기다. 기본 21x3 이면 세로 10px 당 가로 1px,
    수직에서 약 6도까지만 따라간다.

    노란 중앙선은 세로로 이어져 있어 위아래로 끝까지 번지고, 옆으로 가로
    연결된 좌측 흰선으로는 넘어가지 않는다.
    """
    kh = max(3, config.grow_height) | 1
    kw = max(1, config.grow_width) | 1
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kw, kh))
    field = blob.astype(np.uint8)
    marker = (seed & blob).astype(np.uint8)
    for _ in range(max(1, config.grow_iters)):
        grown = cv2.dilate(marker, kernel) & field
        if np.array_equal(grown, marker):
            break
        marker = grown
    return marker.astype(bool)


def divider_x(result, y: int) -> float | None:
    """그 행에서 우리 차선을 가르는 기준선의 x 를 돌려준다.

    노란 중앙선이 있으면 그것을 쓴다. 장면과 함께 움직이므로 카메라 장착
    위치나 차가 치우친 정도에 영향받지 않는다.
    없으면 좌우 흰선의 중간을 쓰고, 그것도 없으면 None 이다.
    """
    if result is None:
        return None
    height = None
    for mask in (result.yellow_mask, result.white_mask):
        if mask is not None:
            height = mask.shape[0]
            break
    if height is None:
        return None
    y = int(max(0, min(height - 1, y)))

    yellow = result.yellow_mask
    if yellow is not None:
        runs = LaneTracker._row_runs(yellow, y, min_run=2)
        if runs:
            return float(max(runs, key=len).mean())

        # 그 행에 노란선이 없을 수 있다. 점선이거나 차 아래에서 끊긴 경우다.
        # 실측 프레임 118: 차 아래가 y=264 인데 노란선은 y=270 에서 끝난다.
        # 보이는 구간으로 직선을 맞춰 그 행의 위치를 계산한다.
        rows, xs = [], []
        for probe in range(0, yellow.shape[0], 4):
            got = LaneTracker._row_runs(yellow, probe, min_run=2)
            if got:
                rows.append(float(probe))
                xs.append(float(max(got, key=len).mean()))
        if len(rows) >= 3:
            slope, base = np.polyfit(np.array(rows), np.array(xs), 1)
            span = max(rows) - min(rows)
            # 보이는 길이의 1.5배 밖까지는 연장하지 않는다. 그 이상은 믿을 수 없다.
            if (abs(y - min(rows)) <= span * 1.5
                    and abs(y - max(rows)) <= span * 1.5):
                return float(slope * y + base)

    white = result.white_mask
    if white is not None:
        runs = LaneTracker._row_runs(white, y)
        if len(runs) >= 2:
            return float((runs[0].mean() + runs[-1].mean()) / 2.0)
    return None


def reclaim_white(blob: np.ndarray, yellow: np.ndarray,
                  seed: np.ndarray) -> np.ndarray:
    """흰선에서 한 번 더 퍼뜨려 노랑을 되덮는다.

    씨앗을 뺀 영역에서 연결요소를 나눈다. 흰선 화소가 하나라도 들어 있는
    덩어리는 통째로 흰선으로 본다. 씨앗을 지나야만 닿는 곳은 못 넘어오므로
    진짜 노란 중앙선은 그대로 남는다.

    좌회전 도로 노란선과 직진 도로 좌측 흰선이 붙어 있을 때, 세로 전파가
    미처 못 자른 접합부 근처가 이 단계에서 흰선으로 돌아온다.
    """
    field = (blob & ~seed)
    if not field.any():
        return yellow
    count, labels = cv2.connectedComponents(field.astype(np.uint8), 8)
    if count <= 1:
        return yellow
    white_now = field & ~yellow
    if not white_now.any():
        return yellow
    reachable = np.unique(labels[white_now])
    reachable = reachable[reachable > 0]
    if reachable.size == 0:
        return yellow
    return yellow & ~np.isin(labels, reachable)


def looks_like_crosswalk(blob: np.ndarray, config: LaneConfig) -> bool:
    """가로로 넓고 납작한 덩이면 횡단보도로 본다.

    차선은 가는 선이라 행마다 좁은 폭만 차지한다.
    횡단보도 지브라는 행마다 넓게 차지하면서 세로로는 짧다.
    """
    if config.crosswalk_row_width <= 0.0 or config.crosswalk_aspect <= 0.0:
        return False
    rows = np.nonzero(blob.any(axis=1))[0]
    if rows.size < 2:
        return False
    widths = blob[rows].sum(axis=1)
    if float(np.median(widths)) <= config.crosswalk_row_width:
        return False
    columns = np.nonzero(blob.any(axis=0))[0]
    height = float(rows[-1] - rows[0] + 1)
    width = float(columns[-1] - columns[0] + 1)
    return (height / max(width, 1.0)) < config.crosswalk_aspect


def far_gain(error: float, config: LaneConfig) -> float:
    """먼 오차에 구간별 이득을 적용한다. 무릎에서 연속이다."""
    if config.kp_far_hi <= 0.0:
        return config.kp_far * error
    knee = max(config.knee_norm, 0.0)
    magnitude = abs(error)
    if magnitude <= knee:
        return config.kp_far_lo * error
    beyond = magnitude - knee
    value = config.kp_far_lo * knee + config.kp_far_hi * beyond
    return math.copysign(value, error)


@dataclass
class LaneResult:
    target_far: float | None = None
    target_near: float | None = None
    error_far: float = 0.0
    error_near: float = 0.0
    steer: float = 0.0
    basis: str = 'none'              # yellow / white_center / hold / none
    yellow_blobs: int = 0
    white_blobs: int = 0
    fit_rows: int = 0
    crosswalk_blobs: int = 0
    curve: np.ndarray | None = None  # 피팅된 (x, y) 점들. 표시용
    yellow_mask: np.ndarray | None = None
    white_mask: np.ndarray | None = None


class LaneTracker:
    def __init__(self, config: LaneConfig | None = None) -> None:
        self.config = config or LaneConfig()
        self._steer = 0.0
        self._prev_error = None
        self._prev_time = None
        self._hold_until = None
        self._hold_result = None
        self._last_crosswalk = 0

    # ------------------------------------------------------------ 검출
    def _white_balance(self, bgr, road, patch=None):
        """도로를 무채색으로 보고 채널 이득을 맞춘다.

        이득은 언제나 도로 전체(bgr)에서 구한다. 적용 대상만 patch 로 좁힐 수
        있게 해서, 호출부가 ROI 만 넘기면 그만큼만 보정한다.

        cv2.mean 은 마스크 평균을 복사 없이 낸다. 이전 bgr[road] 는 도로 픽셀
        전부를 새 배열로 복사했다. 적용은 LUT 로 바꿨다. 입력이 정수 0~255 라
        ramp[v] * gain 과 v * gain 이 같은 값이고, clip 과 절단 방식도 같으므로
        결과가 비트 단위로 동일하다.
        """
        target = bgr if patch is None else patch
        road_u8 = road.astype(np.uint8)
        if cv2.countNonZero(road_u8) < 500:
            return target
        mean_b, mean_g, mean_r, _ = cv2.mean(bgr, mask=road_u8)
        means = np.array([mean_b, mean_g, mean_r], dtype=np.float32)
        gains = means.mean() / np.maximum(means, 1e-3)
        ramp = np.arange(256, dtype=np.float32)
        table = np.clip(ramp[None, :] * gains[:, None], 0, 255).astype(np.uint8)
        return cv2.merge([cv2.LUT(target[:, :, i], table[i]) for i in range(3)])

    def _classify_pixels(self, bgr, lane, road):
        """차선 픽셀 중 노란 것을 가려낸다.

        ROI 위쪽은 계산하지 않는다. _split_blobs 가 seed = yellow_px & roi_lane
        로 쓰는데 roi_lane 은 위쪽이 전부 False 라, 위쪽 yellow_px 는 어떤 값이든
        결과에 영향이 없다.

        두 번째 반환값은 과거 유물이다. 덩이 판정이 "노랑 씨앗이 없으면 흰선"
        으로 바뀐 뒤로 어디서도 쓰이지 않는다. 그것만 위해 chroma / tophat /
        노면 휘도 중앙값을 매 프레임 계산하고 있었다(실측 4.5ms). 호출 규약만
        지키려고 빈 배열을 돌려준다.
        """
        cfg = self.config
        y0 = int(lane.shape[0] * cfg.roi_top_ratio)
        fixed = self._white_balance(bgr, road, bgr[y0:])
        # 노랑은 b* 축(파랑-노랑)의 양수쪽이다. L 과 a* 는 쓰지 않는다.
        b_ch = cv2.cvtColor(fixed, cv2.COLOR_BGR2LAB)[:, :, 2].astype(np.int16)
        seed = (b_ch - 128) >= cfg.yellow_b
        if cfg.exclude_cone_orange:
            # 라바콘 주황은 노란 중앙선이 아니다 (설정 주석의 실측 근거).
            hsv = cv2.cvtColor(fixed, cv2.COLOR_BGR2HSV)
            hue, sat = hsv[:, :, 0], hsv[:, :, 1]
            seed &= ~((hue >= cfg.cone_hue_lo) & (hue <= cfg.cone_hue_hi)
                      & (sat >= cfg.cone_sat_min))
        yellow_px = np.zeros_like(lane)
        yellow_px[y0:] = lane[y0:] & seed
        return yellow_px, np.zeros_like(lane)

    def _split_blobs(self, lane, yellow_px):
        """ROI 안에서 연결요소를 나누고, 노란 픽셀을 품은 덩이를 노랑으로 본다."""
        cfg = self.config
        height, width = lane.shape
        y0 = int(height * cfg.roi_top_ratio)

        # ROI 위쪽을 0 으로 덮지 말고 아예 잘라낸다. 라벨링이 훑을 면적이 42% 로
        # 준다. 가로 커널 opening 도 8연결 라벨링도 행 단위 연산이라, 위쪽을
        # 떼어내도 아래쪽 결과는 그대로다.
        roi_lane = lane[y0:].copy()

        # 횡단보도가 좌우 흰선을 잇는 가로 다리를 먼저 끊는다.
        # 이렇게 하면 덩이 전체에 전파해도 흰선까지 번지지 않는다.
        roi_lane = cut_horizontal_bridges(roi_lane, cfg)
        seed = yellow_px[y0:] & roi_lane

        yellow_mask = np.zeros_like(lane)
        white_mask = np.zeros_like(lane)
        n_yellow = n_white = n_crosswalk = 0

        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            roi_lane.astype(np.uint8), 8)
        for i in range(1, count):
            if stats[i, cv2.CC_STAT_AREA] < cfg.min_blob_area:
                continue
            # 덩이는 얇고 길다. 전체 배열을 비교하지 말고 제 바운딩박스만 본다.
            x = stats[i, cv2.CC_STAT_LEFT]
            y = stats[i, cv2.CC_STAT_TOP]
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]
            blob = labels[y:y + h, x:x + w] == i
            if looks_like_crosswalk(blob, cfg):
                # 횡단보도 조각은 노랑도 흰선도 아닌 것으로 버린다.
                n_crosswalk += 1
                continue
            box = (slice(y0 + y, y0 + y + h), slice(x, x + w))
            blob_seed = blob & seed[y:y + h, x:x + w]
            if int(blob_seed.sum()) >= cfg.min_yellow_px:
                if cfg.straight_only:
                    # 직진일 때만. 씨앗에서 세로로만 번지고, 옆에 붙어 있던
                    # 흰선 부분은 노랑이 되지 않고 흰선으로 남는다.
                    grown = grow_vertical(blob, blob_seed, cfg)
                    if cfg.white_reclaim:
                        # 흰선에서 한 번 더 퍼뜨려 남은 오염을 되덮는다.
                        grown = reclaim_white(blob, grown, blob_seed)
                    yellow_mask[box] |= grown
                    rest = blob & ~grown
                    if rest.any():
                        white_mask[box] |= rest
                        n_white += 1
                else:
                    # 기존 동작. 덩이 전체를 노랑으로 본다.
                    yellow_mask[box] |= blob
                n_yellow += 1
            else:
                white_mask[box] |= blob
                n_white += 1

        self._last_crosswalk = n_crosswalk
        return yellow_mask, white_mask, n_yellow, n_white, y0

    # ------------------------------------------------------------ 목표선
    @staticmethod
    def _row_runs(mask, y, min_run=3, gap=4):
        xs = np.nonzero(mask[y])[0]
        if xs.size < min_run:
            return []
        parts = np.split(xs, np.where(np.diff(xs) > gap)[0] + 1)
        return [p for p in parts if p.size >= min_run]

    def _target_points(self, yellow_mask, white_mask, y0, height):
        """행마다 목표 x 를 구한다. 노란선 우선, 없으면 흰선 중앙.

        회피 중(avoid_offset_px != 0)에는 노란선을 보지 않고 지정한 쪽
        흰선에 붙는다. 앞차가 왼쪽이면 우측 흰선 안쪽으로 붙어 지나간다.
        """
        rows = list(range(y0, height - 2, 4))
        points, basis = [], 'none'

        # 직진 구간: 노란선을 건너뛰고 흰선 중앙만 쓴다. 흰선이 부족하면
        # 목표 없음('none')으로 두어 직전 조향을 유지하다 직진으로 푼다 —
        # 여기서 노란선으로 폴백하면 애초에 피하려던 그 끌림이 되살아난다.
        if self.config.white_center_only and float(self.config.avoid_offset_px) == 0.0:
            white_only = []
            for y in rows:
                runs = self._row_runs(white_mask, y)
                if len(runs) >= 2:
                    white_only.append(
                        (y, float((runs[0].mean() + runs[-1].mean()) / 2)))
            if len(white_only) >= self.config.min_fit_rows:
                return white_only, 'white_center'
            return points, basis

        offset = float(self.config.avoid_offset_px)
        if offset != 0.0:
            hug = []
            for y in rows:
                runs = self._row_runs(white_mask, y)
                if not runs:
                    continue
                edge = runs[-1] if offset > 0 else runs[0]
                hug.append((y, float(edge.mean()) - offset))
            if len(hug) >= self.config.min_fit_rows:
                return hug, ('avoid_right' if offset > 0 else 'avoid_left')
            # 흰선이 부족하면 아래 평소 경로로 떨어진다

        yellow_rows = []
        for y in rows:
            runs = self._row_runs(yellow_mask, y, min_run=2)
            if runs:
                widest = max(runs, key=len)
                yellow_rows.append((y, float(widest.mean())))
        if len(yellow_rows) >= self.config.min_fit_rows:
            return yellow_rows, 'yellow'

        white_rows = []
        for y in rows:
            runs = self._row_runs(white_mask, y)
            if len(runs) >= 2:
                white_rows.append(
                    (y, float((runs[0].mean() + runs[-1].mean()) / 2)))
        if len(white_rows) >= self.config.min_fit_rows:
            return white_rows, 'white_center'
        if yellow_rows:
            return yellow_rows, 'yellow'
        return points, basis

    # ------------------------------------------------------------ 조향
    def _steering(self, error_far, error_near, now):
        cfg = self.config
        dt = 0.05
        if self._prev_time is not None:
            dt = max(1e-3, min(0.5, now - self._prev_time))

        derivative = 0.0
        if self._prev_error is not None:
            derivative = (error_far - self._prev_error) / dt
        self._prev_error, self._prev_time = error_far, now

        far = 0.0 if abs(error_far) < cfg.deadband_norm else error_far
        near = 0.0 if abs(error_near) < cfg.deadband_norm else error_near
        raw = far_gain(far, cfg) + cfg.kp_near * near + cfg.kd * derivative
        raw = cfg.steering_sign * raw

        filtered = (cfg.filter_alpha * raw
                    + (1.0 - cfg.filter_alpha) * self._steer)
        step = cfg.max_rate * dt
        filtered = max(self._steer - step, min(self._steer + step, filtered))
        self._steer = max(-cfg.max_steer, min(cfg.max_steer, filtered))
        return self._steer

    # ------------------------------------------------------------ 진입점
    def process(self, bgr, lane, road, now: float) -> LaneResult:
        cfg = self.config
        height, width = lane.shape
        centre = width * 0.5

        yellow_px, white_px = self._classify_pixels(bgr, lane, road)
        yellow_mask, white_mask, n_yel, n_wht, y0 = self._split_blobs(
            lane, yellow_px)
        rows, basis = self._target_points(yellow_mask, white_mask, y0, height)

        result = LaneResult(basis=basis, yellow_blobs=n_yel,
                            white_blobs=n_wht, fit_rows=len(rows),
                            crosswalk_blobs=self._last_crosswalk,
                            yellow_mask=yellow_mask, white_mask=white_mask)

        if len(rows) < cfg.min_fit_rows:
            # 목표를 놓쳤다. 잠시 직전 조향을 유지하고, 그 뒤엔 직진으로 푼다.
            if self._hold_until is not None and now <= self._hold_until:
                result.basis = 'hold'
                result.steer = self._steer
                return result
            self._steer *= 0.85
            result.steer = self._steer
            result.basis = 'none'
            return result

        ys = np.array([r[0] for r in rows], dtype=np.float32)
        xs = np.array([r[1] for r in rows], dtype=np.float32)
        order = 2 if len(rows) >= 8 else 1
        coefficients = np.polyfit(ys, xs, order)

        y_far = float(height * cfg.y_far_ratio)
        y_near = float(height * cfg.y_near_ratio)
        y_far = max(y_far, ys.min())
        y_near = min(y_near, max(ys.max(), y_far + 1))
        target_far = float(np.polyval(coefficients, y_far))
        target_near = float(np.polyval(coefficients, y_near))

        # 2차 피팅이 화면 밖으로 외삽되면 목표가 무의미해진다.
        # 실측에서 e_near 가 -1.19 까지 나온 적이 있다. 화면 범위로 제한한다.
        margin = width * 0.25
        target_far = float(np.clip(target_far, -margin, width + margin))
        target_near = float(np.clip(target_near, -margin, width + margin))
        error_far = float(np.clip((target_far - centre) / centre, -1.0, 1.0))
        error_near = float(np.clip((target_near - centre) / centre, -1.0, 1.0))
        steer = self._steering(error_far, error_near, now)

        self._hold_until = now + cfg.hold_seconds
        result.target_far = target_far
        result.target_near = target_near
        result.error_far = error_far
        result.error_near = error_near
        result.steer = steer
        curve_y = np.arange(int(ys.min()), int(ys.max()) + 1, 2, dtype=np.float32)
        result.curve = np.stack(
            [np.polyval(coefficients, curve_y), curve_y], axis=1)
        return result


def draw_overlay(bgr, result: LaneResult, config: LaneConfig,
                 speed_units: float = 0.0, armed: bool = False):
    """디버그 화면. 실제 제어와 무관하게 보기용이다."""
    view = bgr.copy()
    height, width = view.shape[:2]
    if result.white_mask is not None:
        view[result.white_mask] = (255, 220, 60)
    if result.yellow_mask is not None:
        view[result.yellow_mask] = (0, 215, 255)

    y0 = int(height * config.roi_top_ratio)
    cv2.line(view, (0, y0), (width, y0), (0, 0, 255), 2)
    cv2.line(view, (width // 2, y0), (width // 2, height), (90, 90, 90), 1)

    if result.curve is not None:
        pts = np.rint(result.curve).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(view, [pts], False, (255, 0, 255), 3, cv2.LINE_AA)
    if result.target_far is not None:
        yf = int(height * config.y_far_ratio)
        cv2.circle(view, (int(result.target_far), yf), 9, (255, 255, 255), 2)
        cv2.circle(view, (int(result.target_far), yf), 3, (0, 0, 255), -1)
    if result.target_near is not None:
        yn = int(height * config.y_near_ratio)
        cv2.circle(view, (int(result.target_near), yn), 6, (200, 200, 200), 2)

    cv2.rectangle(view, (0, 0), (width, 46), (0, 0, 0), -1)
    tag = 'ARMED' if armed else 'SAFE'
    colour = (0, 0, 255) if armed else (0, 255, 0)
    cv2.putText(view, f'[{tag}] {result.basis}  steer {result.steer:+.1f}'
                      f'  speed {speed_units:.1f}',
                (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)
    cv2.putText(view, f'e_far {result.error_far:+.3f}  '
                      f'e_near {result.error_near:+.3f}  '
                      f'rows {result.fit_rows}  '
                      f'yel {result.yellow_blobs} wht {result.white_blobs}'
                      + (f'  XWALK {result.crosswalk_blobs}'
                         if result.crosswalk_blobs else ''),
                (6, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1,
                cv2.LINE_AA)
    return view
