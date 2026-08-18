"""bag 리플레이 도구의 순수 로직 검증.

`--sweep-gantry` 가 랩 임계값을 제대로 골라 주는지 본다. 이 기능이 틀리면
잘못된 min_gantry_span_px 를 확신을 갖고 넣게 되고, 그게 대회 당일
3바퀴 미완주로 이어진다. bag 없이 합성 시퀀스로 검증한다.
"""

from kmu_race.tools.replay import Sample, count_sightings, find_good_thresholds


def synth_race(
    *, laps: int = 3, peak: int = 44, noise_peak: int = 12, lap_s: float = 45.0
) -> list:
    """3바퀴 주행을 합성한다.

    한 바퀴에 겐트리를 한 번 지나가고(span 이 0 → peak → 0 으로 부풀었다 꺼짐),
    반대편 직선에서는 멀리 작게 잡힌다(noise_peak). 출발 시점에는 겐트리 앞에
    한동안 서 있다.
    """
    samples: list = []
    t = 0.0
    step = 0.05

    def push(span: int, n: int) -> None:
        nonlocal t
        for _ in range(n):
            samples.append(Sample(t=t, gantry_span=span))
            t += step

    # 출발 대기 — 겐트리 앞에 정지
    push(peak, 60)

    for _ in range(laps):
        push(0, int(lap_s * 0.5 / step))          # 반 바퀴: 안 보임
        # 반대편 직선에서 멀리 작게 잡힘
        for span in (noise_peak, noise_peak, noise_peak):
            push(span, 6)
        push(0, int(lap_s * 0.4 / step))
        # 겐트리 접근 — 점점 커졌다가 지나가며 사라짐
        for span in (16, 26, 34, peak, peak, 34, 20):
            push(span, 5)
        push(0, 40)
    return samples


def test_correct_threshold_counts_four_sightings():
    """출발 1 + 랩마커 3 = 4회."""
    samples = synth_race()
    assert count_sightings(samples, 26) == 4


def test_threshold_too_high_misses_laps():
    samples = synth_race(peak=44)
    assert count_sightings(samples, 60) == 0, "임계가 너무 높으면 아무것도 못 센다"


def test_threshold_too_low_double_counts():
    """멀리서 잡히는 노이즈까지 세면 목격이 늘어난다."""
    samples = synth_race(noise_peak=12)
    low = count_sightings(samples, 5)
    assert low > 4, f"낮은 임계는 과다 카운트해야 하는데 {low}"


def test_sweep_finds_a_usable_band():
    samples = synth_race()
    good, lo, hi = find_good_thresholds(samples, expect=4)
    assert good, "정확히 4회가 나오는 임계 구간이 있어야 한다"
    assert lo <= good[0] <= hi
    # 권장값(구간 중앙)이 실제로 4회를 준다
    best = good[len(good) // 2]
    assert count_sightings(samples, best) == 4


def test_sweep_band_excludes_noise_level():
    """권장 구간은 반대편 직선 노이즈(12)보다 위에 있어야 한다."""
    samples = synth_race(noise_peak=12, peak=44)
    good, _, _ = find_good_thresholds(samples, expect=4)
    assert min(good) > 12


def test_sweep_handles_empty_detection():
    """겐트리를 한 번도 못 잡은 bag 이면 빈 결과를 돌려준다 (예외 아님)."""
    samples = [Sample(t=0.05 * i, gantry_span=0) for i in range(200)]
    good, lo, hi = find_good_thresholds(samples, expect=4)
    assert good == [] and lo == 0 and hi == 0


def test_two_lap_bag_reports_three_sightings():
    """bag 이 2바퀴면 3회가 정상 — --expect 로 맞춰 쓸 수 있어야 한다."""
    samples = synth_race(laps=2)
    good, _, _ = find_good_thresholds(samples, expect=3)
    assert good
    assert count_sightings(samples, good[len(good) // 2]) == 3
