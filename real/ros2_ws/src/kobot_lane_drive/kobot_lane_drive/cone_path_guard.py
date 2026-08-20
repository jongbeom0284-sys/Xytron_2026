"""라바콘 경로의 시간축 연속성과 고속 전환을 검증한다.

한 프레임짜리 좌/우 경계 뒤집힘에는 trusted 기준의 작은 반응만 허용하고,
같은 새 경로가 연속 확인됐을 때 변화율을 제한해 전환한다. 실제 미검출은 잠시 보간하되
급조향 중에는 유지 거리와 속도를 더 작게 제한한다.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from .cone_corridor import ConeCorridorResult, EMPTY_CORRIDOR_RESULT


@dataclass(frozen=True)
class ConePathGuardConfig:
    """현장에서 YAML로 튜닝하는 경로 연속성 기준."""

    maximum_target_jump_m: float = 0.18
    maximum_steering_jump_rad: float = 0.20
    hold_seconds: float = 0.35
    hold_speed_mps: float = 0.50
    sharp_hold_steering_rad: float = 0.349
    sharp_hold_seconds: float = 0.12
    sharp_hold_speed_mps: float = 0.30
    # False는 실험용 레이싱 정책이다. 한 번 잡은 경로가 있으면 LOST가
    # 계속되어도 timed STOP으로 넘어가지 않고 마지막 실제 조향을 유지한다.
    stop_on_lost: bool = True
    transition_target_rate_mps: float = 0.80
    transition_steering_rate_rad_s: float = 2.00
    pending_target_step_m: float = 0.05
    pending_steering_step_rad: float = 0.10
    candidate_confirmation_frames: int = 2
    candidate_match_target_m: float = 0.08
    candidate_match_steering_rad: float = 0.10
    profile_minimum_overlap_slices: int = 3
    profile_median_tolerance_m: float = 0.12
    profile_maximum_tolerance_m: float = 0.20


@dataclass(frozen=True)
class ConePathDecision:
    """원시 인식과 실제 제어에 쓸 결과를 함께 남긴다."""

    result: ConeCorridorResult
    raw_result: ConeCorridorResult
    path_state: str
    rejection_reason: str
    target_jump_m: float | None
    steering_jump_rad: float | None
    hold_age_sec: float
    transition_phase: str = "NONE"
    hold_profile: str = "NONE"
    hold_limit_sec: float = 0.0
    hold_speed_limit_mps: float = 0.0
    profile_error_m: float | None = None


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


class ConePathContinuityGuard:
    """가짜 순간점은 제한하고, 지속되는 실제 급커브에는 빠르게 수렴한다."""

    def __init__(self, config: ConePathGuardConfig | None = None) -> None:
        self._config = config or ConePathGuardConfig()
        self._trusted: ConeCorridorResult | None = None
        self._output: ConeCorridorResult | None = None
        self._last_output_at = 0.0
        self._last_found_at = 0.0
        self._pending: ConeCorridorResult | None = None
        self._pending_count = 0

    def reset(self) -> None:
        self._trusted = None
        self._output = None
        self._last_output_at = 0.0
        self._last_found_at = 0.0
        self._pending = None
        self._pending_count = 0

    def _clear_pending(self) -> None:
        self._pending = None
        self._pending_count = 0

    def _update_pending(self, raw_result: ConeCorridorResult) -> int:
        if self._pending is None:
            self._pending_count = 1
        else:
            target_delta = raw_result.target_x_m - self._pending.target_x_m
            steering_delta = raw_result.steering_rad - self._pending.steering_rad
            target_match = abs(target_delta) <= max(
                0.0,
                self._config.candidate_match_target_m,
            )
            steering_match = abs(steering_delta) <= max(
                0.0,
                self._config.candidate_match_steering_rad,
            )
            mode_match = self._path_mode(raw_result) == self._path_mode(self._pending)

            # 정상 S자에서는 새 경로가 정지해 있지 않고 프레임마다 같은
            # 방향으로 진행한다. 후보 간 차이가 정적 match 허용치를 조금
            # 넘더라도 trusted에서 첫 후보로 향한 방향과 이번 변화 방향이
            # 같고 한 프레임 변화가 정상 전환 한계 이내라면 동일하게 진행
            # 중인 후보로 본다. 좌/우 교대나 같은 쪽 왕복 후보는 진행 방향
            # 내적이 음수가 되므로 여기에 들어오지 못한다.
            if self._trusted is not None:
                pending_target_offset = (
                    self._pending.target_x_m - self._trusted.target_x_m
                )
                pending_steering_offset = (
                    self._pending.steering_rad - self._trusted.steering_rad
                )
                coherent_target_progress = (
                    pending_target_offset * target_delta > 0.0
                    and abs(target_delta)
                    <= max(0.0, self._config.maximum_target_jump_m)
                )
                coherent_steering_progress = (
                    pending_steering_offset * steering_delta > 0.0
                    and abs(steering_delta)
                    <= max(0.0, self._config.maximum_steering_jump_rad)
                )
                target_match = target_match or coherent_target_progress
                steering_match = steering_match or coherent_steering_progress
            self._pending_count = (
                self._pending_count + 1
                if target_match and steering_match and mode_match
                else 1
            )
        self._pending = raw_result
        return self._pending_count

    def _profile_continuity(
        self,
        raw_result: ConeCorridorResult,
    ) -> tuple[bool, float | None]:
        """같은 전방 y에서 이전/현재 중심선이 이어지는지 확인한다."""
        if self._trusted is None:
            return True, None

        trusted_valid = {
            round(item.y_m, 3): float(item.center_x_m)
            for item in self._trusted.slices
            if item.center_x_m is not None
        }
        raw_valid = {
            round(item.y_m, 3): float(item.center_x_m)
            for item in raw_result.slices
            if item.center_x_m is not None
        }

        # 슬라이스 정보가 없는 외부 호출은 목표점/조향 연속성만으로 판단한다.
        if not trusted_valid and not raw_valid:
            return True, None

        shared_y = trusted_valid.keys() & raw_valid.keys()
        minimum_overlap = max(1, self._config.profile_minimum_overlap_slices)
        if len(shared_y) < minimum_overlap:
            return False, None

        errors = sorted(abs(raw_valid[y] - trusted_valid[y]) for y in shared_y)
        middle = len(errors) // 2
        median_error = (
            errors[middle]
            if len(errors) % 2
            else 0.5 * (errors[middle - 1] + errors[middle])
        )
        return (
            median_error <= max(0.0, self._config.profile_median_tolerance_m)
            and errors[-1]
            <= max(0.0, self._config.profile_maximum_tolerance_m),
            median_error,
        )

    @staticmethod
    def _path_mode(result: ConeCorridorResult) -> str:
        """실제 경계 생성 모드를 우선하고, 구형 결과는 primary_side를 쓴다."""

        return (
            result.boundary_mode
            if result.boundary_mode and result.boundary_mode != "NONE"
            else result.primary_side
        )

    def _side_is_continuous(self, raw_result: ConeCorridorResult) -> bool:
        if self._trusted is None:
            return True
        # BOTH↔단측, PAIR↔SPARSE도 한 프레임 반짝임이면 속도 명령이
        # 0.80/0.70↔0.50m/s로 왕복한다. 경계 모드 변경은 동일 후보가
        # 두 프레임 확인된 뒤에만 승격한다.
        return self._path_mode(self._trusted) == self._path_mode(raw_result)

    def _bounded_transition(
        self,
        raw_result: ConeCorridorResult,
        now: float,
    ) -> ConeCorridorResult:
        base = self._output or self._trusted
        if base is None:
            return raw_result

        dt = 0.05 if self._last_output_at <= 0.0 else _clamp(
            now - self._last_output_at,
            0.01,
            0.20,
        )
        target_step = max(0.0, self._config.transition_target_rate_mps) * dt
        steering_step = max(
            0.0,
            self._config.transition_steering_rate_rad_s,
        ) * dt
        target_x = base.target_x_m + _clamp(
            raw_result.target_x_m - base.target_x_m,
            -target_step,
            target_step,
        )
        steering = base.steering_rad + _clamp(
            raw_result.steering_rad - base.steering_rad,
            -steering_step,
            steering_step,
        )
        transition_converged = (
            abs(raw_result.target_x_m - target_x)
            <= max(0.0, self._config.candidate_match_target_m)
            and abs(raw_result.steering_rad - steering)
            <= max(0.0, self._config.candidate_match_steering_rad)
        )
        return replace(
            raw_result,
            target_x_m=target_x,
            steering_rad=steering,
            # 새 인식의 고속값을 옛 조향 상태에 즉시 얹지 않는다. 감속은
            # 즉시 허용하되, 가속은 목표점과 조향이 새 경로에 충분히
            # 수렴한 뒤에만 승격한다.
            speed_mps=(
                raw_result.speed_mps
                if transition_converged
                else min(base.speed_mps, raw_result.speed_mps)
            ),
        )

    def _bounded_pending_preview(
        self,
        raw_result: ConeCorridorResult,
        now: float,
        preserve_trusted_speed: bool = False,
    ) -> ConeCorridorResult:
        """미확정 후보 쪽으로 작게 반응하되 trusted 경로에는 누적시키지 않는다."""

        trusted = self._trusted or self._output
        if trusted is None:
            return raw_result
        dt = 0.05 if self._last_output_at <= 0.0 else _clamp(
            now - self._last_output_at,
            0.01,
            0.20,
        )
        target_offset = max(0.0, self._config.pending_target_step_m)
        steering_offset = max(0.0, self._config.pending_steering_step_rad)
        target_slew = max(0.0, self._config.transition_target_rate_mps) * dt
        steering_slew = (
            max(0.0, self._config.transition_steering_rate_rad_s) * dt
        )
        desired_target_x = trusted.target_x_m + _clamp(
            raw_result.target_x_m - trusted.target_x_m,
            -target_offset,
            target_offset,
        )
        desired_steering = trusted.steering_rad + _clamp(
            raw_result.steering_rad - trusted.steering_rad,
            -steering_offset,
            steering_offset,
        )

        # 좌/우 후보가 매 프레임 뒤집히면 trusted 기준의 preview 자체는
        # 누적되지 않아도 +limit에서 -limit로 한 번에 건너뛸 수 있다. 이전
        # 실제 가드 출력에서 desired preview까지 다시 slew-limit하여 후보
        # 방향 전환도 정상 경로와 같은 초당 변화율을 지키게 한다.
        previous = self._output or trusted
        return replace(
            trusted,
            target_x_m=previous.target_x_m
            + _clamp(
                desired_target_x - previous.target_x_m,
                -target_slew,
                target_slew,
            ),
            steering_rad=previous.steering_rad
            + _clamp(
                desired_steering - previous.steering_rad,
                -steering_slew,
                steering_slew,
            ),
            # 경계 모드가 한 프레임만 BOTH↔단측으로 반짝이는 경우에는
            # 속도까지 즉시 왕복시키지 않는다. 동일 모드가 두 프레임
            # 이어지면 confirmed branch에서 새 속도로 전환된다. 기하가
            # 급변한 후보는 기존처럼 낮은 쪽 속도를 즉시 사용한다.
            speed_mps=(
                trusted.speed_mps
                if preserve_trusted_speed
                else min(trusted.speed_mps, raw_result.speed_mps)
            ),
        )

    def _lost_decision(
        self,
        raw_result: ConeCorridorResult,
        now: float,
        reason: str,
        last_commanded_steering_rad: float | None,
    ) -> ConePathDecision:
        base = self._output or self._trusted
        hold_age = (
            max(0.0, now - self._last_found_at)
            if base is not None
            else 0.0
        )
        guarded_steering = max(
            abs(base.steering_rad) if base is not None else 0.0,
            abs(self._trusted.steering_rad)
            if self._trusted is not None
            else 0.0,
        )
        commanded_steering = (
            abs(last_commanded_steering_rad)
            if last_commanded_steering_rad is not None
            else 0.0
        )
        sharp = max(guarded_steering, commanded_steering) >= max(
            0.0,
            self._config.sharp_hold_steering_rad,
        )
        hold_limit = (
            max(0.0, self._config.sharp_hold_seconds)
            if sharp
            else max(0.0, self._config.hold_seconds)
        )
        speed_limit = (
            max(0.0, self._config.sharp_hold_speed_mps)
            if sharp
            else max(0.0, self._config.hold_speed_mps)
        )
        hold_profile = "SHARP_LOST" if sharp else "NORMAL_LOST"

        # 실험용 no-stop 경로. 아래의 기존 시간제 HOLD -> STOP 로직은 삭제하지
        # 않고 그대로 보존하여 파라미터 하나로 즉시 복구할 수 있게 한다.
        # 최초부터 경로를 한 번도 잡지 못한 경우에는 blind start하지 않는다.
        if base is not None and not self._config.stop_on_lost:
            held_steering = (
                float(last_commanded_steering_rad)
                if last_commanded_steering_rad is not None
                else base.steering_rad
            )
            held_result = replace(
                base,
                steering_rad=held_steering,
                speed_mps=min(base.speed_mps, speed_limit),
            )
            self._output = held_result
            self._last_output_at = now
            return ConePathDecision(
                result=held_result,
                raw_result=raw_result,
                path_state="HOLD_LOST",
                rejection_reason=reason,
                target_jump_m=None,
                steering_jump_rad=None,
                hold_age_sec=hold_age,
                hold_profile="NO_STOP_LOST",
                hold_limit_sec=-1.0,
                hold_speed_limit_mps=speed_limit,
            )

        if base is not None and hold_age <= hold_limit:
            # planner 목표를 직전 출력이 아직 따라가는 중에 인식이 끊길 수
            # 있다. 이때 planner 조향을 hold하면 필터가 LOST 중에도 stale
            # 조향을 더 키운다. 노드가 전달한 실제 최종 조향을 목표로 삼아
            # 물리적으로 직전 조향을 그대로 유지한다.
            held_steering = (
                float(last_commanded_steering_rad)
                if last_commanded_steering_rad is not None
                else base.steering_rad
            )
            held_result = replace(
                base,
                steering_rad=held_steering,
                speed_mps=min(base.speed_mps, speed_limit),
            )
            self._output = held_result
            self._last_output_at = now
            return ConePathDecision(
                result=held_result,
                raw_result=raw_result,
                path_state="HOLD_LOST",
                rejection_reason=reason,
                target_jump_m=None,
                steering_jump_rad=None,
                hold_age_sec=hold_age,
                hold_profile=hold_profile,
                hold_limit_sec=hold_limit,
                hold_speed_limit_mps=speed_limit,
            )

        decision = ConePathDecision(
            result=EMPTY_CORRIDOR_RESULT,
            raw_result=raw_result,
            path_state="STOPPED",
            rejection_reason=reason,
            target_jump_m=None,
            steering_jump_rad=None,
            hold_age_sec=hold_age,
            hold_profile=hold_profile,
            hold_limit_sec=hold_limit,
            hold_speed_limit_mps=speed_limit,
        )
        self.reset()
        return decision

    def update(
        self,
        raw_result: ConeCorridorResult,
        now: float,
        last_commanded_steering_rad: float | None = None,
    ) -> ConePathDecision:
        target_jump: float | None = None
        steering_jump: float | None = None
        if raw_result.found and self._trusted is not None:
            target_jump = abs(raw_result.target_x_m - self._trusted.target_x_m)
            steering_jump = abs(
                raw_result.steering_rad - self._trusted.steering_rad
            )

        profile_is_continuous, profile_error = self._profile_continuity(raw_result)
        side_is_continuous = self._side_is_continuous(raw_result)
        candidate_is_continuous = (
            raw_result.found
            and (
                self._trusted is None
                or (
                    target_jump is not None
                    and target_jump <= self._config.maximum_target_jump_m
                    and steering_jump is not None
                    and steering_jump <= self._config.maximum_steering_jump_rad
                    and profile_is_continuous
                    and side_is_continuous
                )
            )
        )

        if candidate_is_continuous:
            # 정상적으로 이어지는 경로에는 앞단 2rad/s 제한을 중복 적용하지
            # 않는다. 노드 후단의 저역통과 필터와 5rad/s rate limit가 실제
            # 조향 명령을 제한한다. 급변/모드 전환 후보만 아래 confirmation과
            # bounded transition을 통과시킨다.
            self._trusted = raw_result
            self._output = raw_result
            self._last_output_at = now
            self._last_found_at = now
            self._clear_pending()
            return ConePathDecision(
                result=raw_result,
                raw_result=raw_result,
                path_state="TRACKING",
                rejection_reason="NONE",
                target_jump_m=target_jump,
                steering_jump_rad=steering_jump,
                hold_age_sec=0.0,
                profile_error_m=profile_error,
            )

        if raw_result.found:
            if (
                target_jump is not None
                and target_jump > self._config.maximum_target_jump_m
            ):
                rejection_reason = "TARGET_JUMP"
            elif (
                steering_jump is not None
                and steering_jump > self._config.maximum_steering_jump_rad
            ):
                rejection_reason = "STEERING_JUMP"
            elif not side_is_continuous:
                rejection_reason = "SIDE_SWITCH"
            else:
                rejection_reason = "BOUNDARY_JUMP"

            pending_count = self._update_pending(raw_result)
            confirmed = pending_count >= max(
                1,
                int(self._config.candidate_confirmation_frames),
            )
            if confirmed:
                transitioned = self._bounded_transition(raw_result, now)
                self._output = transitioned
                self._last_output_at = now
                self._trusted = transitioned
                self._last_found_at = now
                return ConePathDecision(
                    result=transitioned,
                    raw_result=raw_result,
                    path_state="TRACKING",
                    rejection_reason=rejection_reason,
                    target_jump_m=target_jump,
                    steering_jump_rad=steering_jump,
                    hold_age_sec=0.0,
                    transition_phase="ACTIVE",
                    profile_error_m=profile_error,
                )

            # 첫 급변 프레임도 stale 조향을 그대로 유지하지 않고 후보 방향으로
            # 작은 bounded preview를 낸다. 단, 기준은 언제나 trusted 경로라서
            # 좌우 후보가 반복되어도 출력이 한쪽으로 누적 드리프트하지 않는다.
            pending_result = self._bounded_pending_preview(
                raw_result,
                now,
                preserve_trusted_speed=not side_is_continuous,
            )
            self._output = pending_result
            self._last_output_at = now
            # raw 경로는 존재하므로 실제 LOST 타이머는 갱신한다. 장시간 후보가
            # 흔들린 직후 한 프레임 미검출됐다고 즉시 정지하지 않게 한다.
            self._last_found_at = now

            return ConePathDecision(
                result=pending_result,
                raw_result=raw_result,
                path_state="HOLD_REJECTED",
                rejection_reason=rejection_reason,
                target_jump_m=target_jump,
                steering_jump_rad=steering_jump,
                hold_age_sec=0.0,
                transition_phase="PENDING",
                profile_error_m=profile_error,
            )

        # 실제 미검출일 때만 직전 경로를 짧게 유지한다.
        self._clear_pending()
        return self._lost_decision(
            raw_result,
            now,
            "LOST",
            last_commanded_steering_rad,
        )
