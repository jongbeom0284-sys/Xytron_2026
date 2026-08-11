from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable


@dataclass
class LaneVisionConfig:
    # Vehicle frame: X is forward in meters, Y is left in meters.
    vehicle_width_m: float = 0.42
    safety_margin_m: float = 0.08
    min_safe_corridor_m: float = 0.12

    initial_lane_width_m: float = 1.35
    lane_width_min_m: float = 0.80
    lane_width_max_m: float = 2.20
    lane_width_alpha: float = 0.04
    memory_timeout_s: float = 0.8

    roi_y0_ratio: float = 0.42
    source_top_y_ratio: float = 0.50
    source_bottom_y_ratio: float = 0.98
    source_top_half_width_ratio: float = 0.18
    source_bottom_half_width_ratio: float = 0.49
    source_center_x_ratio: float = 0.50

    ground_x_max_m: float = 8.0
    ground_y_left_m: float = 2.2
    ground_y_right_m: float = -2.2
    bev_width_px: int = 480
    bev_height_px: int = 640

    yellow_h_min: int = 15
    yellow_h_max: int = 42
    yellow_s_min: int = 70
    yellow_v_min: int = 80
    white_s_max: int = 85
    white_v_min: int = 165
    road_s_max: int = 110
    road_v_min: int = 35
    road_v_max: int = 230

    min_component_area_px: int = 18
    max_component_area_px: int = 12000
    min_component_height_px: int = 4
    min_component_width_px: int = 2
    horizontal_component_aspect_ratio: float = 4.0
    horizontal_component_min_area_px: int = 90
    morphology_kernel_px: int = 3
    boundary_row_step_px: int = 3

    fit_min_points: int = 18
    fit_min_x_m: float = 0.6
    fit_max_x_m: float = 7.5
    ransac_iterations: int = 80
    ransac_residual_m: float = 0.10
    ransac_min_inlier_ratio: float = 0.38
    max_fit_rmse_m: float = 0.16

    previous_fit_max_offset_m: float = 0.65
    max_center_jump_m: float = 0.55
    max_center_step_m: float = 0.28
    low_confidence_max_center_step_m: float = 0.12
    center_filter_alpha: float = 0.32
    low_confidence_center_filter_alpha: float = 0.14
    max_lane_width_jump_m: float = 0.45
    max_curvature_jump: float = 0.18
    intersection_horizontal_count: int = 4
    intersection_component_count: int = 28
    intersection_hold_s: float = 0.65

    # Sparse horizontal scanlines are linked into persistent lane chains.  The
    # controller fits these chains instead of deciding left/right from a single
    # image column.
    scanline_step_px: int = 8
    scanline_min_slices: int = 5
    scanline_max_missed_slices: int = 10
    scanline_link_max_gap_m: float = 0.32

    # When a real (non-predicted) lane has not been observed recently, use a
    # wider/farther homography for reacquisition.  Wide-search detections are
    # deliberately treated as lower confidence by the detector.
    wide_search_after_s: float = 0.25
    wide_source_top_y_ratio: float = 0.30
    wide_source_top_half_width_ratio: float = 0.32
    wide_confidence_scale: float = 0.65


def _param_value(get_param: Callable[[str, Any], Any], name: str, default: Any, *aliases: str) -> Any:
    value = get_param(name, default)
    if value is not None:
        return value
    for alias in aliases:
        alias_value = get_param(alias, None)
        if alias_value is not None:
            return alias_value
    return default


def lane_vision_config_from_getter(get_param: Callable[[str, Any], Any]) -> LaneVisionConfig:
    defaults = LaneVisionConfig()
    return LaneVisionConfig(
        vehicle_width_m=float(_param_value(get_param, "vehicle_width_m", defaults.vehicle_width_m)),
        safety_margin_m=float(_param_value(get_param, "safety_margin_m", defaults.safety_margin_m)),
        min_safe_corridor_m=float(_param_value(get_param, "min_safe_corridor_m", defaults.min_safe_corridor_m)),
        initial_lane_width_m=float(_param_value(get_param, "initial_lane_width_m", defaults.initial_lane_width_m)),
        lane_width_min_m=float(_param_value(get_param, "lane_width_min_m", defaults.lane_width_min_m)),
        lane_width_max_m=float(_param_value(get_param, "lane_width_max_m", defaults.lane_width_max_m)),
        lane_width_alpha=float(_param_value(get_param, "lane_width_alpha", defaults.lane_width_alpha)),
        memory_timeout_s=float(_param_value(get_param, "memory_timeout_s", defaults.memory_timeout_s)),
        roi_y0_ratio=float(_param_value(get_param, "roi_y0_ratio", defaults.roi_y0_ratio, "lane_roi_y0_ratio")),
        source_top_y_ratio=float(_param_value(get_param, "source_top_y_ratio", defaults.source_top_y_ratio)),
        source_bottom_y_ratio=float(_param_value(get_param, "source_bottom_y_ratio", defaults.source_bottom_y_ratio)),
        source_top_half_width_ratio=float(
            _param_value(get_param, "source_top_half_width_ratio", defaults.source_top_half_width_ratio)
        ),
        source_bottom_half_width_ratio=float(
            _param_value(get_param, "source_bottom_half_width_ratio", defaults.source_bottom_half_width_ratio)
        ),
        source_center_x_ratio=float(_param_value(get_param, "source_center_x_ratio", defaults.source_center_x_ratio)),
        ground_x_max_m=float(_param_value(get_param, "ground_x_max_m", defaults.ground_x_max_m)),
        ground_y_left_m=float(_param_value(get_param, "ground_y_left_m", defaults.ground_y_left_m)),
        ground_y_right_m=float(_param_value(get_param, "ground_y_right_m", defaults.ground_y_right_m)),
        bev_width_px=int(_param_value(get_param, "bev_width_px", defaults.bev_width_px)),
        bev_height_px=int(_param_value(get_param, "bev_height_px", defaults.bev_height_px)),
        yellow_h_min=int(_param_value(get_param, "yellow_h_min", defaults.yellow_h_min)),
        yellow_h_max=int(_param_value(get_param, "yellow_h_max", defaults.yellow_h_max)),
        yellow_s_min=int(_param_value(get_param, "yellow_s_min", defaults.yellow_s_min)),
        yellow_v_min=int(_param_value(get_param, "yellow_v_min", defaults.yellow_v_min)),
        white_s_max=int(_param_value(get_param, "white_s_max", defaults.white_s_max)),
        white_v_min=int(_param_value(get_param, "white_v_min", defaults.white_v_min)),
        road_s_max=int(_param_value(get_param, "road_s_max", defaults.road_s_max)),
        road_v_min=int(_param_value(get_param, "road_v_min", defaults.road_v_min)),
        road_v_max=int(_param_value(get_param, "road_v_max", defaults.road_v_max)),
        min_component_area_px=int(_param_value(get_param, "min_component_area_px", defaults.min_component_area_px)),
        max_component_area_px=int(_param_value(get_param, "max_component_area_px", defaults.max_component_area_px)),
        min_component_height_px=int(
            _param_value(get_param, "min_component_height_px", defaults.min_component_height_px)
        ),
        min_component_width_px=int(
            _param_value(get_param, "min_component_width_px", defaults.min_component_width_px)
        ),
        horizontal_component_aspect_ratio=float(
            _param_value(get_param, "horizontal_component_aspect_ratio", defaults.horizontal_component_aspect_ratio)
        ),
        horizontal_component_min_area_px=int(
            _param_value(get_param, "horizontal_component_min_area_px", defaults.horizontal_component_min_area_px)
        ),
        morphology_kernel_px=int(_param_value(get_param, "morphology_kernel_px", defaults.morphology_kernel_px)),
        boundary_row_step_px=int(_param_value(get_param, "boundary_row_step_px", defaults.boundary_row_step_px)),
        fit_min_points=int(_param_value(get_param, "fit_min_points", defaults.fit_min_points)),
        fit_min_x_m=float(_param_value(get_param, "fit_min_x_m", defaults.fit_min_x_m)),
        fit_max_x_m=float(_param_value(get_param, "fit_max_x_m", defaults.fit_max_x_m)),
        ransac_iterations=int(_param_value(get_param, "ransac_iterations", defaults.ransac_iterations)),
        ransac_residual_m=float(_param_value(get_param, "ransac_residual_m", defaults.ransac_residual_m)),
        ransac_min_inlier_ratio=float(
            _param_value(get_param, "ransac_min_inlier_ratio", defaults.ransac_min_inlier_ratio)
        ),
        max_fit_rmse_m=float(_param_value(get_param, "max_fit_rmse_m", defaults.max_fit_rmse_m)),
        previous_fit_max_offset_m=float(
            _param_value(get_param, "previous_fit_max_offset_m", defaults.previous_fit_max_offset_m)
        ),
        max_center_jump_m=float(_param_value(get_param, "max_center_jump_m", defaults.max_center_jump_m)),
        max_center_step_m=float(_param_value(get_param, "max_center_step_m", defaults.max_center_step_m)),
        low_confidence_max_center_step_m=float(
            _param_value(get_param, "low_confidence_max_center_step_m", defaults.low_confidence_max_center_step_m)
        ),
        center_filter_alpha=float(_param_value(get_param, "center_filter_alpha", defaults.center_filter_alpha)),
        low_confidence_center_filter_alpha=float(
            _param_value(get_param, "low_confidence_center_filter_alpha", defaults.low_confidence_center_filter_alpha)
        ),
        max_lane_width_jump_m=float(_param_value(get_param, "max_lane_width_jump_m", defaults.max_lane_width_jump_m)),
        max_curvature_jump=float(_param_value(get_param, "max_curvature_jump", defaults.max_curvature_jump)),
        intersection_horizontal_count=int(
            _param_value(get_param, "intersection_horizontal_count", defaults.intersection_horizontal_count)
        ),
        intersection_component_count=int(
            _param_value(get_param, "intersection_component_count", defaults.intersection_component_count)
        ),
        intersection_hold_s=float(_param_value(get_param, "intersection_hold_s", defaults.intersection_hold_s)),
        scanline_step_px=int(_param_value(get_param, "scanline_step_px", defaults.scanline_step_px)),
        scanline_min_slices=int(_param_value(get_param, "scanline_min_slices", defaults.scanline_min_slices)),
        scanline_max_missed_slices=int(
            _param_value(get_param, "scanline_max_missed_slices", defaults.scanline_max_missed_slices)
        ),
        scanline_link_max_gap_m=float(
            _param_value(get_param, "scanline_link_max_gap_m", defaults.scanline_link_max_gap_m)
        ),
        wide_search_after_s=float(_param_value(get_param, "wide_search_after_s", defaults.wide_search_after_s)),
        wide_source_top_y_ratio=float(
            _param_value(get_param, "wide_source_top_y_ratio", defaults.wide_source_top_y_ratio)
        ),
        wide_source_top_half_width_ratio=float(
            _param_value(get_param, "wide_source_top_half_width_ratio", defaults.wide_source_top_half_width_ratio)
        ),
        wide_confidence_scale=float(
            _param_value(get_param, "wide_confidence_scale", defaults.wide_confidence_scale)
        ),
    )
