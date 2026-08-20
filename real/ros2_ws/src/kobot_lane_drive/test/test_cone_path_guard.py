"""라바콘 경로 연속성 가드 단위 시험."""

import pytest

from kobot_lane_drive.cone_corridor import (
    CorridorSlice,
    ConeCorridorResult,
    EMPTY_CORRIDOR_RESULT,
)
from kobot_lane_drive.cone_path_guard import (
    ConePathContinuityGuard,
    ConePathGuardConfig,
)


def _result(
    target_x: float,
    speed: float = 0.70,
    steering: float | None = None,
    side: str = "BOTH",
    profile_x: float | None = None,
    boundary_mode: str = "NONE",
) -> ConeCorridorResult:
    slices = []
    if profile_x is not None:
        slices = [
            CorridorSlice(
                y_m=y_m,
                left_x_m=profile_x + 0.40,
                right_x_m=profile_x - 0.40,
                center_x_m=profile_x,
                is_interpolated=False,
            )
            for y_m in (0.35, 0.45, 0.55, 0.65)
        ]
    return ConeCorridorResult(
        found=True,
        target_x_m=target_x,
        target_y_m=0.72,
        steering_rad=target_x if steering is None else steering,
        speed_mps=speed,
        confidence=0.8,
        slices=slices,
        left_points_count=8,
        right_points_count=8,
        corridor_width_est_m=0.80,
        primary_side=side,
        left_cones_count=2,
        right_cones_count=2,
        boundary_mode=boundary_mode,
    )


@pytest.fixture
def guard() -> ConePathContinuityGuard:
    return ConePathContinuityGuard(
        ConePathGuardConfig(
            maximum_target_jump_m=0.18,
            maximum_steering_jump_rad=0.20,
            hold_seconds=0.35,
            hold_speed_mps=0.50,
            sharp_hold_steering_rad=0.349,
            sharp_hold_seconds=0.12,
            sharp_hold_speed_mps=0.30,
            transition_target_rate_mps=0.80,
            transition_steering_rate_rad_s=2.00,
            pending_target_step_m=0.05,
            pending_steering_step_rad=0.10,
            candidate_confirmation_frames=2,
            candidate_match_target_m=0.08,
            candidate_match_steering_rad=0.10,
            profile_minimum_overlap_slices=3,
            profile_median_tolerance_m=0.12,
            profile_maximum_tolerance_m=0.20,
        )
    )


def test_continuous_target_is_accepted(guard: ConePathContinuityGuard):
    assert guard.update(_result(0.00), 1.0).path_state == "TRACKING"

    decision = guard.update(_result(0.12), 1.1)

    assert decision.path_state == "TRACKING"
    assert decision.result.target_x_m == pytest.approx(0.12)
    assert decision.target_jump_m == pytest.approx(0.12)


def test_first_large_jump_gets_bounded_preview_before_candidate_is_confirmed(
    guard: ConePathContinuityGuard,
):
    guard.update(_result(0.00, steering=0.00), 2.0)

    decision = guard.update(_result(0.35, steering=0.35), 2.1)

    assert decision.path_state == "HOLD_REJECTED"
    assert decision.rejection_reason == "TARGET_JUMP"
    assert decision.result.target_x_m == pytest.approx(0.05)
    assert decision.result.steering_rad == pytest.approx(0.10)
    assert decision.result.speed_mps == pytest.approx(0.70)
    assert decision.transition_phase == "PENDING"


def test_consistent_jump_is_confirmed_while_every_frame_remains_bounded(
    guard: ConePathContinuityGuard,
):
    guard.update(_result(0.00, steering=0.00), 3.0)

    first = guard.update(_result(0.35, steering=0.35), 3.1)
    second = guard.update(_result(0.35, steering=0.35), 3.2)

    assert first.path_state == "HOLD_REJECTED"
    assert second.path_state == "TRACKING"
    assert first.result.target_x_m == pytest.approx(0.05)
    assert second.result.target_x_m == pytest.approx(0.13)
    assert first.result.steering_rad == pytest.approx(0.10)
    assert second.result.steering_rad == pytest.approx(0.30)
    assert second.transition_phase == "ACTIVE"


def test_consistently_advancing_curve_candidate_is_confirmed(
    guard: ConePathContinuityGuard,
):
    """매 프레임 움직이는 정상 S자 후보가 정적 match 오차 때문에 고정되지 않는다."""
    guard.update(_result(0.00, steering=0.00), 3.5)

    first = guard.update(_result(0.35, steering=0.35), 3.6)
    second = guard.update(_result(0.45, steering=0.46), 3.7)
    third = guard.update(_result(0.55, steering=0.57), 3.8)

    assert first.path_state == "HOLD_REJECTED"
    assert second.path_state == "TRACKING"
    assert second.transition_phase == "ACTIVE"
    assert third.path_state == "TRACKING"
    assert second.result.target_x_m > first.result.target_x_m
    assert third.result.target_x_m > second.result.target_x_m
    assert second.result.steering_rad > first.result.steering_rad
    assert third.result.steering_rad > second.result.steering_rad


def test_same_side_reversing_candidate_does_not_fake_progress_confirmation(
    guard: ConePathContinuityGuard,
):
    guard.update(_result(0.00, steering=0.00), 3.9)

    outward = guard.update(_result(0.45, steering=0.45), 4.0)
    reversing = guard.update(_result(0.35, steering=0.34), 4.1)

    assert outward.path_state == "HOLD_REJECTED"
    assert reversing.path_state == "HOLD_REJECTED"
    assert reversing.transition_phase == "PENDING"


def test_persistent_jump_no_longer_stops_the_car(guard: ConePathContinuityGuard):
    guard.update(_result(0.00, steering=0.00), 4.0)

    states = [
        guard.update(_result(0.35, steering=0.35), now).path_state
        for now in (4.1, 4.2, 4.3, 4.4)
    ]

    assert "STOPPED" not in states
    assert states[0] == "HOLD_REJECTED"
    assert states[-1] == "TRACKING"


def test_inconsistent_jump_resets_confirmation_and_limits_motion(
    guard: ConePathContinuityGuard,
):
    guard.update(_result(0.00, steering=0.00), 5.0)

    left = guard.update(_result(0.35, steering=0.35), 5.1)
    right = guard.update(_result(-0.35, steering=-0.35), 5.2)

    assert left.path_state == "HOLD_REJECTED"
    assert right.path_state == "HOLD_REJECTED"
    assert left.result.target_x_m == pytest.approx(0.05)
    assert right.result.target_x_m == pytest.approx(-0.03)
    assert left.result.steering_rad == pytest.approx(0.10)
    assert right.result.steering_rad == pytest.approx(-0.10)


def test_large_steering_jump_is_bounded_even_when_target_x_is_stable(
    guard: ConePathContinuityGuard,
):
    guard.update(_result(0.00, steering=0.00), 6.0)

    decision = guard.update(_result(0.02, steering=0.30), 6.1)

    assert decision.path_state == "HOLD_REJECTED"
    assert decision.rejection_reason == "STEERING_JUMP"
    assert decision.result.target_x_m == pytest.approx(0.02)
    assert decision.result.steering_rad == pytest.approx(0.10)


def test_missing_detection_holds_then_stops_and_reacquires(
    guard: ConePathContinuityGuard,
):
    guard.update(_result(-0.08), 7.0)

    held = guard.update(EMPTY_CORRIDOR_RESULT, 7.2)
    stopped = guard.update(EMPTY_CORRIDOR_RESULT, 7.4)

    assert held.path_state == "HOLD_LOST"
    assert held.result.found
    assert held.result.target_x_m == pytest.approx(-0.08)
    assert held.result.speed_mps == pytest.approx(0.50)
    assert stopped.path_state == "STOPPED"
    assert not stopped.result.found

    reacquired = guard.update(_result(0.30), 7.5)
    assert reacquired.path_state == "TRACKING"
    assert reacquired.target_jump_m is None


def test_smooth_curve_updates_do_not_trigger_transition(
    guard: ConePathContinuityGuard,
):
    states = [
        guard.update(_result(target), 8.0 + index * 0.1).path_state
        for index, target in enumerate((0.00, 0.10, 0.20, 0.30))
    ]

    assert states == ["TRACKING"] * 4


def test_continuous_curve_bypasses_guard_rate_limit(
    guard: ConePathContinuityGuard,
):
    guard.update(_result(0.00, steering=0.00), 9.0)

    decision = guard.update(_result(0.12, steering=0.18), 9.1)

    assert decision.path_state == "TRACKING"
    assert decision.result.target_x_m == pytest.approx(0.12)
    assert decision.result.steering_rad == pytest.approx(0.18)


def test_boundary_mode_flicker_does_not_pulse_speed(
    guard: ConePathContinuityGuard,
):
    guard.update(
        _result(0.00, speed=0.80, boundary_mode="PAIRED"),
        9.5,
    )

    flicker = guard.update(
        _result(
            0.01,
            speed=0.50,
            side="LEFT_ONLY",
            boundary_mode="LEFT_FALLBACK",
        ),
        9.6,
    )
    recovered = guard.update(
        _result(0.01, speed=0.80, boundary_mode="PAIRED"),
        9.7,
    )

    assert flicker.path_state == "HOLD_REJECTED"
    assert flicker.rejection_reason == "SIDE_SWITCH"
    assert flicker.result.speed_mps == pytest.approx(0.80)
    assert flicker.result.primary_side == "BOTH"
    assert recovered.path_state == "TRACKING"
    assert recovered.result.speed_mps == pytest.approx(0.80)


def test_persistent_boundary_mode_change_updates_speed_after_confirmation(
    guard: ConePathContinuityGuard,
):
    guard.update(
        _result(0.00, speed=0.80, boundary_mode="PAIRED"),
        9.8,
    )
    single = _result(
        0.01,
        speed=0.50,
        side="LEFT_ONLY",
        boundary_mode="LEFT_FALLBACK",
    )

    first = guard.update(single, 9.9)
    second = guard.update(single, 10.0)

    assert first.path_state == "HOLD_REJECTED"
    assert first.result.speed_mps == pytest.approx(0.80)
    assert second.path_state == "TRACKING"
    assert second.result.primary_side == "LEFT_ONLY"
    assert second.result.speed_mps == pytest.approx(0.50)


def test_alternating_lock_to_lock_candidates_never_stop_or_accumulate_drift(
    guard: ConePathContinuityGuard,
):
    guard.update(_result(0.00, steering=0.00), 10.0)

    decisions = [
        guard.update(
            _result(
                0.35 if index % 2 else -0.35,
                steering=0.45 if index % 2 else -0.45,
            ),
            10.0 + index * 0.1,
        )
        for index in range(1, 101)
    ]

    assert all(item.path_state == "HOLD_REJECTED" for item in decisions)
    assert all(item.result.found for item in decisions)
    assert all(abs(item.result.target_x_m) <= 0.05 + 1e-9 for item in decisions)
    assert all(abs(item.result.steering_rad) <= 0.10 + 1e-9 for item in decisions)
    assert decisions[0].result.steering_rad > 0.0
    assert decisions[1].result.steering_rad < 0.0


def test_fast_alternating_candidates_obey_rate_limits_without_drift(
    guard: ConePathContinuityGuard,
):
    initial = guard.update(_result(0.00, steering=0.00), 10.5).result
    outputs = [initial]
    for index in range(1, 101):
        outputs.append(
            guard.update(
                _result(
                    0.35 if index % 2 else -0.35,
                    steering=0.45 if index % 2 else -0.45,
                ),
                10.5 + index * 0.05,
            ).result
        )

    for previous, current in zip(outputs, outputs[1:]):
        assert abs(current.target_x_m - previous.target_x_m) <= 0.04 + 1e-9
        assert abs(current.steering_rad - previous.steering_rad) <= 0.10 + 1e-9
    assert all(abs(item.target_x_m) <= 0.05 + 1e-9 for item in outputs)
    assert all(abs(item.steering_rad) <= 0.10 + 1e-9 for item in outputs)


def test_stable_candidate_is_accepted_after_long_alternating_sequence(
    guard: ConePathContinuityGuard,
):
    guard.update(_result(0.00, steering=0.00), 11.0)
    for index in range(1, 101):
        guard.update(
            _result(
                0.35 if index % 2 else -0.35,
                steering=0.45 if index % 2 else -0.45,
            ),
            11.0 + index * 0.1,
        )

    first_same = guard.update(_result(0.35, steering=0.45), 21.1)
    second_same = guard.update(_result(0.35, steering=0.45), 21.2)

    assert first_same.path_state == "HOLD_REJECTED"
    assert second_same.path_state == "TRACKING"
    assert second_same.result.found
    assert second_same.result.steering_rad == pytest.approx(0.30)


def test_long_candidate_flapping_then_real_lost_still_gets_hold_window(
    guard: ConePathContinuityGuard,
):
    guard.update(_result(0.00, steering=0.00), 22.0)
    for index in range(1, 101):
        guard.update(
            _result(
                0.35 if index % 2 else -0.35,
                steering=0.45 if index % 2 else -0.45,
            ),
            22.0 + index * 0.1,
        )

    held = guard.update(EMPTY_CORRIDOR_RESULT, 32.1)
    stopped = guard.update(EMPTY_CORRIDOR_RESULT, 32.36)

    assert held.path_state == "HOLD_LOST"
    assert held.result.found
    assert held.result.speed_mps == pytest.approx(0.50)
    assert stopped.path_state == "STOPPED"


def test_sharp_lost_uses_short_slow_hold_then_stops(
    guard: ConePathContinuityGuard,
):
    guard.update(_result(0.15, steering=0.424, speed=0.70), 12.0)

    held = guard.update(EMPTY_CORRIDOR_RESULT, 12.10)
    stopped = guard.update(EMPTY_CORRIDOR_RESULT, 12.13)

    assert held.path_state == "HOLD_LOST"
    assert held.hold_profile == "SHARP_LOST"
    assert held.result.speed_mps == pytest.approx(0.30)
    assert stopped.path_state == "STOPPED"


def test_timed_lost_stop_can_be_disabled_without_deleting_the_policy():
    no_stop_guard = ConePathContinuityGuard(
        ConePathGuardConfig(
            hold_seconds=0.35,
            hold_speed_mps=0.50,
            sharp_hold_steering_rad=0.349,
            sharp_hold_seconds=0.0,
            sharp_hold_speed_mps=0.50,
            stop_on_lost=False,
        )
    )
    no_stop_guard.update(_result(0.15, steering=0.424, speed=1.00), 40.0)

    after_ten_seconds = no_stop_guard.update(
        EMPTY_CORRIDOR_RESULT,
        50.0,
        last_commanded_steering_rad=0.30,
    )

    assert after_ten_seconds.path_state == "HOLD_LOST"
    assert after_ten_seconds.hold_profile == "NO_STOP_LOST"
    assert after_ten_seconds.hold_limit_sec == pytest.approx(-1.0)
    assert after_ten_seconds.result.found
    assert after_ten_seconds.result.steering_rad == pytest.approx(0.30)
    assert after_ten_seconds.result.speed_mps == pytest.approx(0.50)


def test_no_stop_policy_does_not_blind_start_without_any_trusted_path():
    no_stop_guard = ConePathContinuityGuard(
        ConePathGuardConfig(stop_on_lost=False)
    )

    decision = no_stop_guard.update(EMPTY_CORRIDOR_RESULT, 1.0)

    assert decision.path_state == "STOPPED"
    assert not decision.result.found


def test_actual_last_commanded_steering_selects_sharp_lost_profile(
    guard: ConePathContinuityGuard,
):
    guard.update(_result(0.02, steering=0.10, speed=0.70), 12.5)

    held = guard.update(
        EMPTY_CORRIDOR_RESULT,
        12.60,
        last_commanded_steering_rad=0.424,
    )
    stopped = guard.update(
        EMPTY_CORRIDOR_RESULT,
        12.63,
        last_commanded_steering_rad=0.424,
    )

    assert held.path_state == "HOLD_LOST"
    assert held.hold_profile == "SHARP_LOST"
    assert held.result.speed_mps == pytest.approx(0.30)
    assert stopped.path_state == "STOPPED"


def test_lost_holds_actual_command_instead_of_chasing_stale_planner_steering(
    guard: ConePathContinuityGuard,
):
    guard.update(_result(0.30, steering=0.42, speed=0.70), 12.8)

    held = guard.update(
        EMPTY_CORRIDOR_RESULT,
        12.90,
        last_commanded_steering_rad=0.08,
    )

    assert held.path_state == "HOLD_LOST"
    assert held.result.steering_rad == pytest.approx(0.08)
    assert held.result.speed_mps == pytest.approx(0.30)

    stopped = guard.update(
        EMPTY_CORRIDOR_RESULT,
        12.93,
        last_commanded_steering_rad=0.08,
    )
    assert stopped.path_state == "STOPPED"


def test_continuous_reacquisition_is_left_to_node_steering_filter(
    guard: ConePathContinuityGuard,
):
    guard.update(_result(0.00, steering=0.00), 13.5)
    held = guard.update(
        EMPTY_CORRIDOR_RESULT,
        13.7,
        last_commanded_steering_rad=0.00,
    )
    reacquired = guard.update(_result(0.15, steering=0.15), 13.8)

    assert held.path_state == "HOLD_LOST"
    assert reacquired.path_state == "TRACKING"
    assert reacquired.result.target_x_m == pytest.approx(0.15)
    assert reacquired.result.steering_rad == pytest.approx(0.15)


def test_boundary_profile_jump_needs_confirmation_even_with_small_target_jump(
    guard: ConePathContinuityGuard,
):
    guard.update(_result(0.00, steering=0.00, profile_x=0.00), 13.0)

    first = guard.update(
        _result(0.05, steering=0.05, profile_x=0.25),
        13.1,
    )
    second = guard.update(
        _result(0.05, steering=0.05, profile_x=0.25),
        13.2,
    )

    assert first.path_state == "HOLD_REJECTED"
    assert first.rejection_reason == "BOUNDARY_JUMP"
    assert first.result.target_x_m == pytest.approx(0.05)
    assert second.path_state == "TRACKING"


def test_direct_single_side_flip_needs_confirmation(
    guard: ConePathContinuityGuard,
):
    guard.update(
        _result(0.05, steering=0.05, side="LEFT_ONLY", profile_x=0.05),
        14.0,
    )

    first = guard.update(
        _result(0.04, steering=0.04, side="RIGHT_ONLY", profile_x=0.04),
        14.1,
    )
    second = guard.update(
        _result(0.04, steering=0.04, side="RIGHT_ONLY", profile_x=0.04),
        14.2,
    )

    assert first.path_state == "HOLD_REJECTED"
    assert first.rejection_reason == "SIDE_SWITCH"
    assert first.result.primary_side == "LEFT_ONLY"
    assert second.path_state == "TRACKING"
    assert second.result.primary_side == "RIGHT_ONLY"


def test_normal_tracking_keeps_racing_speed_limits(
    guard: ConePathContinuityGuard,
):
    straight = guard.update(_result(0.00, speed=0.80), 15.0)
    corner = guard.update(_result(0.08, speed=0.70), 15.1)
    straight_again = guard.update(_result(0.00, speed=0.80), 15.2)

    assert straight.result.speed_mps == pytest.approx(0.80)
    assert corner.result.speed_mps == pytest.approx(0.70)
    assert straight_again.result.speed_mps == pytest.approx(0.80)


def test_transition_does_not_accelerate_until_new_path_is_close(
    guard: ConePathContinuityGuard,
):
    guard.update(_result(0.00, steering=0.00, speed=0.50), 16.0)

    pending = guard.update(
        _result(0.35, steering=0.35, speed=0.80),
        16.1,
    )
    confirmed = guard.update(
        _result(0.35, steering=0.35, speed=0.80),
        16.2,
    )
    approaching = guard.update(
        _result(0.35, steering=0.35, speed=0.80),
        16.3,
    )
    converged = guard.update(
        _result(0.35, steering=0.35, speed=0.80),
        16.4,
    )

    assert pending.result.speed_mps == pytest.approx(0.50)
    assert confirmed.transition_phase == "ACTIVE"
    assert confirmed.result.speed_mps == pytest.approx(0.50)
    assert approaching.result.speed_mps == pytest.approx(0.50)
    assert converged.result.speed_mps == pytest.approx(0.80)
