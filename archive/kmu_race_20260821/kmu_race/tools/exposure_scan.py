"""주행용 고정 노출 실측 도구 — 트랙에서 한 번 돌려 drive_exposure 를 정한다.

왜 필요한가 (0818 실측):
    자동 노출은 세션 내내 화면 평균 V 를 109~214 로 흔든다. 차선 임계
    (V 243~255)는 그 요동 위에서 프레임마다 다른 문제를 풀게 된다.
    노출을 하나로 고정하면 문제 자체가 정지한다 — 단, 그 '하나'는 조명에
    따라 다르므로 **트랙 현장에서 실측**해야 한다.

사용 (차를 트랙 위 차선이 보이는 곳에 세워 두고):
    ros2 run kmu_race exposure_scan
    ros2 run kmu_race exposure_scan --candidates 40,60,80,120,180,260
    ros2 run kmu_race exposure_scan --frames 45      # 후보당 프레임 수

동작: 후보 노출을 하나씩 걸고 → 카메라 안정화 0.7s 대기 → N 프레임에서
차선 검출률·양쪽 관측률·흰 마스크 픽셀비를 재서 표로 보여 주고, 검출률이
가장 높은 값을 추천한다. 끝나면 자동 노출로 복귀한다.
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from ..io.camera_exposure import CameraExposure
from ..perception.lane_detector import LaneDetector


class _Grabber(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("exposure_scan")
        self.bridge = CvBridge()
        self.frame = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, topic, self._on_image, qos)

    def _on_image(self, msg: Image) -> None:
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:                              # noqa: BLE001
            pass

    def grab(self, timeout_s: float = 1.0):
        self.frame = None
        t0 = time.time()
        while self.frame is None and time.time() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.frame


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/image_raw")
    ap.add_argument("--candidates", default="30,45,60,90,130,180,260,360")
    ap.add_argument("--frames", type=int, default=30)
    args = ap.parse_args()

    cands = [int(x) for x in args.candidates.split(",") if x.strip()]
    rclpy.init()
    node = _Grabber(args.topic)
    exp = CameraExposure()
    if not exp.available:
        print("v4l2-ctl 없음 — 노출 제어 불가")
        return

    # ★ 선택 기준 (0818 저녁): '검출률'은 엣지 채널 덕에 어느 노출이든 ~97%
    #   로 포화해 변별력이 없다. 노란 중앙선이 **주역 목표선**이므로 노란선
    #   생존율(center)이 1순위다 — 노출을 너무 낮추면 노란 게이트(V≥110)가
    #   죽어 주역 경로 전체가 꺼진다. 다음이 검출률, 그다음 흰 선 관측.
    print("후보   노란선   검출률   흰선(한쪽+)   노랑픽셀비   평균V")
    results = []
    try:
        for cand in cands:
            exp.set_manual(cand)
            time.sleep(0.7)                       # 카메라 안정화
            det = LaneDetector()
            n = ok = center = white = 0
            yfrac = []
            vmean = []
            for _ in range(args.frames):
                frame = node.grab()
                if frame is None:
                    continue
                r = det.detect(frame)
                n += 1
                ok += r.valid
                center += r.center_found
                white += r.left_found or r.right_found
                vmean.append(frame.mean())
                roi = det.last_bev_roi
                if roi is not None:
                    import cv2
                    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    c = det.config
                    my = cv2.inRange(
                        hsv,
                        np.array([c.yellow_h[0], c.yellow_s_min, c.yellow_v_min]),
                        np.array([c.yellow_h[1], 255, 255]),
                    )
                    yfrac.append(np.count_nonzero(my) / my.size)
            if n == 0:
                print("%4d   (프레임 없음)" % cand)
                continue
            results.append((center / n, ok / n, white / n, cand))
            print("%4d   %5.0f%%   %5.0f%%   %8.0f%%   %8.2f%%   %5.0f"
                  % (cand, 100 * center / n, 100 * ok / n, 100 * white / n,
                     100 * float(np.mean(yfrac) if yfrac else 0.0),
                     float(np.mean(vmean) if vmean else 0.0)))
    finally:
        exp.set_auto()
        print("(자동 노출로 복귀했다)")

    if results:
        # 노란선 생존율 → 검출률 → 흰 선 순. 동률이면 **낮은** 노출(램프 판독·
        # 반사 억제에 유리)을 고른다.
        results.sort(key=lambda r: (round(r[0], 2), round(r[1], 2), round(r[2], 2), -r[3]),
                     reverse=True)
        best = results[0]
        print()
        print("추천: drive_exposure=%d  (노란선 %.0f%% / 검출 %.0f%% / 흰선 %.0f%%)"
              % (best[3], 100 * best[0], 100 * best[1], 100 * best[2]))
        if best[0] < 0.5:
            print("⚠ 이 자리에서 노란 중앙선이 잘 안 보인다 — 차를 노란 점선이 "
                  "BEV 안에 들어오는 곳(도로 중앙 근처)에 세우고 다시 돌릴 것")
        print("적용: ros2 run kmu_race race --ros-args -p drive_exposure:=%d ..."
              % best[3])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
