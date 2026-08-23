import pytest

from race_last.center_offset import calculate_center_offset_target


def corridor(center):
    points = []
    for x in (0.30, 0.70, 1.10, 1.50):
        y = center(x)
        points.extend(((x, y + 0.41), (x, y - 0.41)))
    return points


def test_integrated_default_uses_half_tuned_offset():
    result = calculate_center_offset_target(corridor(lambda _x: 0.0))
    assert result is not None
    assert result.aim_y_m == pytest.approx(0.0)
    assert result.target_x_from_lidar_m == pytest.approx(0.40)
    assert result.target_x_from_center_m == pytest.approx(0.55)


def test_linear_target_matches_validated_standalone_profile():
    result = calculate_center_offset_target(corridor(lambda x: 0.20 * x))
    assert result is not None
    assert result.aim_y_m == pytest.approx(0.08, abs=1e-6)


def test_body_width_and_clearance_limit_center_target():
    result = calculate_center_offset_target(corridor(lambda _x: 0.35))
    assert result is not None
    assert result.maximum_center_offset_m == pytest.approx(0.24)
    assert result.aim_y_m == pytest.approx(0.24)


def test_missing_gate_returns_none_for_original_fallback():
    assert calculate_center_offset_target([(0.5, 0.4), (0.9, 0.4)]) is None


@pytest.mark.parametrize('length,width', [(0.0, 0.3), (0.6, -0.1)])
def test_invalid_vehicle_geometry_is_rejected(length, width):
    with pytest.raises(ValueError):
        calculate_center_offset_target(
            corridor(lambda _x: 0.0), vehicle_length_m=length,
            vehicle_width_m=width)
