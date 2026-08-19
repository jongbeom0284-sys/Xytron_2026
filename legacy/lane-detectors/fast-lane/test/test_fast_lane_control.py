from kobot_lane_drive.fast_lane_control import (
    FastLaneControlConfig,
    FastLaneController,
    LaneObservation,
)


def _observation(
    state: str = "BOTH",
    confidence: float = 0.80,
    offset_px: float | None = 80.0,
    mask_rejected: bool = False,
) -> LaneObservation:
    return LaneObservation(state, confidence, offset_px, mask_rejected)


def test_both_lane_tracks_up_to_configured_speed_without_hard_confidence_gate():
    controller = FastLaneController()

    command = controller.step(_observation(confidence=0.65), 0.0)

    assert command.mode == "TRACKING"
    assert 0.28 <= command.speed_mps <= 0.70
    assert command.steering_rad < 0.0


def test_single_lane_keeps_tracking_at_reduced_speed():
    controller = FastLaneController()

    command = controller.step(
        _observation(state="RIGHT_ONLY", confidence=0.48),
        0.0,
    )

    assert command.mode == "TRACKING"
    assert 0.28 <= command.speed_mps <= 0.48


def test_short_lane_loss_holds_last_steering_and_slows_instead_of_stopping():
    controller = FastLaneController()
    tracking = controller.step(_observation(), 0.0)

    grace = controller.step(_observation(state="LOST", offset_px=None), 0.10)

    assert grace.mode == "GRACE"
    assert grace.steering_rad == tracking.steering_rad
    assert 0.0 < grace.speed_mps <= 0.30


def test_persistent_lane_loss_stops():
    controller = FastLaneController()
    controller.step(_observation(), 0.0)

    command = controller.step(_observation(state="LOST", offset_px=None), 0.60)

    assert command.mode == "STOPPED"
    assert command.speed_mps == 0.0


def test_mask_rejection_uses_temporal_grace_then_stops():
    controller = FastLaneController()
    controller.step(_observation(), 0.0)

    grace = controller.step(_observation(mask_rejected=True), 0.10)
    stopped = controller.step(_observation(mask_rejected=True), 0.60)

    assert grace.mode == "GRACE"
    assert grace.reason == "mask_rejected"
    assert stopped.mode == "STOPPED"
    assert stopped.speed_mps == 0.0


def test_steering_angle_and_change_rate_are_bounded():
    config = FastLaneControlConfig(max_steering_rate_rad_s=1.0)
    controller = FastLaneController(config)

    first = controller.step(_observation(offset_px=10000.0), 0.0)
    commands = [first]
    for index in range(1, 20):
        commands.append(
            controller.step(_observation(offset_px=10000.0), index * 0.05)
        )

    assert abs(first.steering_rad) <= 0.05 + 1e-9
    assert all(abs(command.steering_rad) <= 0.34 for command in commands)
    assert all(
        abs(current.steering_rad - previous.steering_rad) <= 0.05 + 1e-9
        for previous, current in zip(commands, commands[1:])
    )


def test_brief_both_to_single_transition_does_not_jump_to_reconstructed_center():
    controller = FastLaneController()
    both = controller.step(_observation(offset_px=-5.0), 0.0)

    transient_single = controller.step(
        _observation(state="LEFT_ONLY", offset_px=-95.0),
        0.05,
    )

    assert transient_single.mode == "TRACKING"
    assert abs(transient_single.steering_rad - both.steering_rad) < 0.01


def test_persistent_single_lane_moves_toward_reconstructed_center_gradually():
    controller = FastLaneController()
    controller.step(_observation(offset_px=0.0), 0.0)
    controller.step(_observation(state="LEFT_ONLY", offset_px=-100.0), 0.10)

    first_after_hold = controller.step(
        _observation(state="LEFT_ONLY", offset_px=-100.0),
        0.35,
    )
    later = controller.step(
        _observation(state="LEFT_ONLY", offset_px=-100.0),
        0.40,
    )

    assert first_after_hold.steering_rad > 0.0
    assert later.steering_rad >= first_after_hold.steering_rad
    assert later.steering_rad <= controller.config.max_steering_rad
