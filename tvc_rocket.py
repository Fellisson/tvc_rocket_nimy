from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field, replace
from itertools import product
from math import atan2, cos, degrees, exp, pi, sin
from pathlib import Path


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(angle: float) -> float:
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle


@dataclass
class RocketParameters:
    dry_mass: float = 18.0
    propellant_mass: float = 12.0
    thrust: float = 850.0
    burn_time: float = 18.0
    area_ref: float = 0.018
    cd0: float = 0.22
    cd_alpha: float = 2.8
    cl_alpha: float = 1.4
    iyy: float = 7.5
    body_length: float = 2.2
    tvc_arm: float = 0.85
    pitch_damping: float = 18.0
    static_stability: float = 1.8
    max_gimbal_deg: float = 6.0
    max_gimbal_rate_deg_s: float = 18.0
    max_alpha_cmd_deg: float = 3.0
    dt: float = 0.02
    t_final: float = 120.0
    gamma_initial_deg: float = 90.0
    theta_initial_deg: float = 90.0
    altitude_target: float = 2500.0
    impact_target: float = 800.0
    gravity_turn_start: float = 4.0
    gravity_turn_end: float = 14.0
    gamma_final_deg: float = 72.0
    guidance_gain: float = 0.65
    guidance_altitude_gain: float = 0.00015
    wind_ref_m_s: float = 0.0
    wind_gradient_m_s_per_m: float = 0.0008
    wind_decay_altitude_m: float = 2600.0
    launch_rail_time: float = 1.4
    save_plots: bool = True
    show_plots: bool = True
    plot_display_seconds: float = 4.0
    tuning_mode: str = "accurate"

    @property
    def mass_flow(self) -> float:
        return self.propellant_mass / self.burn_time

    @property
    def max_gimbal_rad(self) -> float:
        return self.max_gimbal_deg * pi / 180.0

    @property
    def max_gimbal_rate_rad_s(self) -> float:
        return self.max_gimbal_rate_deg_s * pi / 180.0

    @property
    def max_alpha_cmd_rad(self) -> float:
        return self.max_alpha_cmd_deg * pi / 180.0


@dataclass
class PIDController:
    kp: float
    ki: float
    kd: float
    output_limit: float
    integral_limit: float
    derivative_filter: float = 0.85
    integral: float = 0.0
    previous_error: float = 0.0
    filtered_derivative: float = 0.0

    def update(self, error: float, dt: float, freeze_integrator: bool = False) -> float:
        if not freeze_integrator:
            self.integral += error * dt
            self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)

        raw_derivative = (error - self.previous_error) / dt if dt > 0.0 else 0.0
        self.filtered_derivative = (
            self.derivative_filter * self.filtered_derivative
            + (1.0 - self.derivative_filter) * raw_derivative
        )
        self.previous_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * self.filtered_derivative
        return clamp(output, -self.output_limit, self.output_limit)


@dataclass(frozen=True)
class ControllerGains:
    kp: float
    ki: float
    kd: float
    guidance_gain: float
    guidance_altitude_gain: float
    gamma_final_deg: float


@dataclass(frozen=True)
class TuningResult:
    gains: ControllerGains
    metrics: dict[str, float]
    score: float


@dataclass
class TVCActuator:
    max_angle: float
    max_rate: float
    angle: float = 0.0

    def update(self, command: float, dt: float) -> float:
        command = clamp(command, -self.max_angle, self.max_angle)
        max_step = self.max_rate * dt
        delta = clamp(command - self.angle, -max_step, max_step)
        self.angle = clamp(self.angle + delta, -self.max_angle, self.max_angle)
        return self.angle


@dataclass
class GuidanceState:
    last_gamma_ref: float = field(default=90.0 * pi / 180.0)


@dataclass
class RocketState:
    t: float
    x: float
    z: float
    vx: float
    vz: float
    theta: float
    q: float
    mass: float


def sample_state(state: RocketState, telemetry: dict[str, float]) -> dict[str, float]:
    return {
        "t": state.t,
        "x": state.x,
        "z": state.z,
        "vx": state.vx,
        "vz": state.vz,
        "theta": state.theta,
        "q": state.q,
        "mass": state.mass,
        **telemetry,
    }


def interpolate_ground_impact(previous: dict[str, float], current: dict[str, float]) -> dict[str, float]:
    z_prev = previous["z"]
    z_curr = current["z"]
    if z_prev <= 0.0 or z_prev == z_curr:
        impact = current.copy()
        impact["z"] = 0.0
        impact["altitude"] = 0.0
        return impact

    ratio = z_prev / (z_prev - z_curr)
    impact: dict[str, float] = {}
    for key, prev_value in previous.items():
        curr_value = current[key]
        if isinstance(prev_value, (int, float)) and isinstance(curr_value, (int, float)):
            impact[key] = prev_value + ratio * (curr_value - prev_value)
        else:
            impact[key] = curr_value
    impact["z"] = 0.0
    impact["altitude"] = 0.0
    return impact


def air_density(altitude: float) -> float:
    return 1.225 * exp(-max(altitude, 0.0) / 8500.0)


def wind_velocity(altitude: float, params: RocketParameters) -> float:
    gradient = 1.0 + params.wind_gradient_m_s_per_m * max(altitude, 0.0)
    decay = exp(-max(altitude, 0.0) / params.wind_decay_altitude_m)
    return params.wind_ref_m_s * gradient * decay


def gamma_reference(t: float, params: RocketParameters) -> float:
    start = params.gravity_turn_start
    end = params.gravity_turn_end
    gamma0 = params.gamma_initial_deg * pi / 180.0
    gammaf = params.gamma_final_deg * pi / 180.0

    if t <= start:
        return gamma0
    if t >= end:
        return gammaf

    ratio = (t - start) / (end - start)
    return gamma0 + ratio * (gammaf - gamma0)


def altitude_shaping_gamma(
    t: float,
    altitude: float,
    params: RocketParameters,
    guidance_state: GuidanceState,
) -> float:
    base_gamma = gamma_reference(t, params)
    altitude_error = params.altitude_target - altitude
    correction = clamp(
        altitude_error * params.guidance_altitude_gain,
        -4.0 * pi / 180.0,
        4.0 * pi / 180.0,
    )
    gamma_ref = clamp(base_gamma + correction, 45.0 * pi / 180.0, 90.0 * pi / 180.0)
    guidance_state.last_gamma_ref = gamma_ref
    return gamma_ref


def step_dynamics(
    state: RocketState,
    params: RocketParameters,
    pid: PIDController,
    actuator: TVCActuator,
    guidance_state: GuidanceState,
) -> tuple[RocketState, dict[str, float]]:
    g = 9.81
    dt = params.dt

    wind_x = wind_velocity(state.z, params)
    vax = state.vx - wind_x
    vaz = state.vz
    airspeed = max((vax**2 + vaz**2) ** 0.5, 1.0)
    speed = max((state.vx**2 + state.vz**2) ** 0.5, 1.0)
    gamma = atan2(state.vz, state.vx)
    gamma_air = atan2(vaz, vax)
    alpha = wrap_angle(state.theta - gamma)
    alpha_air = wrap_angle(state.theta - gamma_air) if airspeed > 25.0 else alpha

    is_burning = state.t < params.burn_time and state.mass > params.dry_mass

    if state.t < params.launch_rail_time:
        gamma_ref = params.gamma_initial_deg * pi / 180.0
        alpha_cmd = 0.0
        freeze_integrator = True
        gimbal_cmd = 0.0
    elif not is_burning:
        gamma_ref = gamma_air
        alpha_cmd = 0.0
        freeze_integrator = True
        gimbal_cmd = 0.0
    else:
        gamma_ref = altitude_shaping_gamma(state.t, state.z, params, guidance_state)
        alpha_cmd = clamp(
            params.guidance_gain * wrap_angle(gamma_ref - gamma_air),
            -params.max_alpha_cmd_rad,
            params.max_alpha_cmd_rad,
        )
        freeze_integrator = False
        gimbal_cmd = pid.update(alpha_cmd - alpha_air, dt, freeze_integrator=freeze_integrator)
    gimbal = actuator.update(gimbal_cmd, dt)

    thrust = params.thrust if is_burning else 0.0
    mdot = params.mass_flow if is_burning else 0.0

    rho = air_density(state.z)
    q_dyn = 0.5 * rho * airspeed * airspeed
    drag = q_dyn * params.area_ref * (params.cd0 + params.cd_alpha * alpha_air * alpha_air)
    lift = q_dyn * params.area_ref * params.cl_alpha * alpha_air

    thrust_angle = state.theta + gimbal
    ax = (thrust * cos(thrust_angle) - drag * cos(gamma_air) - lift * sin(gamma_air)) / state.mass
    az = (thrust * sin(thrust_angle) - drag * sin(gamma_air) + lift * cos(gamma_air)) / state.mass - g

    aero_restoring_moment = -q_dyn * params.area_ref * params.body_length * params.static_stability * alpha_air
    moment = thrust * params.tvc_arm * sin(gimbal) + aero_restoring_moment - params.pitch_damping * state.q
    q_dot = moment / params.iyy

    next_state = RocketState(
        t=state.t + dt,
        x=state.x + state.vx * dt,
        z=max(0.0, state.z + state.vz * dt),
        vx=state.vx + ax * dt,
        vz=state.vz + az * dt,
        theta=wrap_angle(state.theta + state.q * dt),
        q=state.q + q_dot * dt,
        mass=max(params.dry_mass, state.mass - mdot * dt),
    )

    telemetry = {
        "gamma": gamma,
        "gamma_ref": gamma_ref,
        "alpha": alpha,
        "alpha_air": alpha_air,
        "alpha_cmd": alpha_cmd,
        "gimbal_cmd": gimbal_cmd,
        "gimbal": gimbal,
        "speed": speed,
        "airspeed": airspeed,
        "thrust": thrust,
        "altitude": state.z,
        "x": state.x,
        "wind_x": wind_x,
    }
    return next_state, telemetry


def simulate_rocket(params: RocketParameters, pid: PIDController) -> list[dict[str, float]]:
    initial_speed = 25.0
    gamma0 = params.gamma_initial_deg * pi / 180.0
    theta0 = params.theta_initial_deg * pi / 180.0

    state = RocketState(
        t=0.0,
        x=0.0,
        z=0.0,
        vx=initial_speed * cos(gamma0),
        vz=initial_speed * sin(gamma0),
        theta=theta0,
        q=0.0,
        mass=params.dry_mass + params.propellant_mass,
    )
    actuator = TVCActuator(
        max_angle=params.max_gimbal_rad,
        max_rate=params.max_gimbal_rate_rad_s,
    )
    guidance_state = GuidanceState(last_gamma_ref=gamma0)

    history: list[dict[str, float]] = []
    while state.t <= params.t_final:
        previous_state = state
        state, telemetry = step_dynamics(state, params, pid, actuator, guidance_state)
        current_sample = sample_state(state, telemetry)
        history.append(current_sample)
        if state.t > params.burn_time and state.z <= 0.0:
            if history:
                previous_telemetry = history[-2] if len(history) >= 2 else sample_state(previous_state, telemetry)
                history[-1] = interpolate_ground_impact(previous_telemetry, current_sample)
            break
    return history


def build_pid(params: RocketParameters, gains: ControllerGains) -> PIDController:
    return PIDController(
        kp=gains.kp,
        ki=gains.ki,
        kd=gains.kd,
        output_limit=params.max_gimbal_rad,
        integral_limit=0.35,
        derivative_filter=0.85,
    )


def evaluate_history(history: list[dict[str, float]], params: RocketParameters | None = None) -> dict[str, float]:
    peak_altitude = max(sample["z"] for sample in history)
    final = history[-1]
    impact_target = params.impact_target if params is not None else 0.0
    landed = final["z"] <= 1e-6 and final["t"] > 0.0
    mean_alpha_deg = degrees(sum(abs(sample["alpha_air"]) for sample in history) / len(history))
    max_alpha_deg = degrees(max(abs(sample["alpha_air"]) for sample in history))
    max_gimbal_deg = degrees(max(abs(sample["gimbal"]) for sample in history))
    mean_gimbal_deg = degrees(sum(abs(sample["gimbal"]) for sample in history) / len(history))
    gamma_error_deg = degrees(
        sum(abs(sample["gamma_ref"] - sample["gamma"]) for sample in history) / len(history)
    )
    gimbal_limit_deg = degrees(max(abs(sample["gimbal_cmd"]) for sample in history))
    sat_threshold = max(0.1, gimbal_limit_deg * 0.98)
    saturation_ratio = sum(abs(degrees(sample["gimbal"])) >= sat_threshold for sample in history) / len(history)
    return {
        "peak_altitude": peak_altitude,
        "downrange": final["x"],
        "impact_error": abs(final["x"] - impact_target),
        "final_speed": final["speed"],
        "flight_time": final["t"],
        "landed": 1.0 if landed else 0.0,
        "mean_alpha_deg": mean_alpha_deg,
        "max_alpha_deg": max_alpha_deg,
        "max_gimbal_deg": max_gimbal_deg,
        "mean_gimbal_deg": mean_gimbal_deg,
        "saturation_ratio": saturation_ratio,
        "mean_gamma_error_deg": gamma_error_deg,
    }


def score_history(history: list[dict[str, float]], params: RocketParameters) -> float:
    metrics = evaluate_history(history, params)
    altitude_error = abs(metrics["peak_altitude"] - params.altitude_target)
    impact_error = abs(metrics["downrange"] - params.impact_target)
    downrange_penalty = impact_error * 4.0 + (0.0 if metrics["downrange"] >= 0.0 else abs(metrics["downrange"]) * 6.0)
    landing_penalty = 0.0 if metrics["landed"] >= 0.5 else 50000.0 + 300.0 * max(0.0, history[-1]["z"])
    gimbal_margin_penalty = max(0.0, metrics["max_gimbal_deg"] - 4.8) * 900.0
    saturation_penalty = metrics["saturation_ratio"] * 4500.0 + metrics["mean_gimbal_deg"] * 90.0
    alpha_penalty = metrics["mean_alpha_deg"] * 140.0 + metrics["max_alpha_deg"] * 18.0
    gamma_penalty = metrics["mean_gamma_error_deg"] * 80.0
    return (
        altitude_error
        + downrange_penalty
        + landing_penalty
        + gimbal_margin_penalty
        + saturation_penalty
        + alpha_penalty
        + gamma_penalty
    )


def candidate_gain_sets(
    impact_target: float,
    tuning_mode: str,
) -> list[tuple[float, float, float, float, float, float]]:
    if tuning_mode == "fast":
        if impact_target <= 1000.0:
            return list(
                product(
                    (3.2, 4.0, 4.8),
                    (0.35, 0.7),
                    (0.9, 1.3),
                    (0.32, 0.45, 0.58),
                    (0.00006, 0.0001),
                    (74.0, 78.0),
                )
            )

        return list(
            product(
                (3.2, 4.0, 4.8),
                (0.2, 0.35, 0.7),
                (0.7, 0.9, 1.3),
                (0.45, 0.58, 0.72),
                (0.00003, 0.00006, 0.0001),
                (68.0, 72.0, 74.0),
            )
        )

    if impact_target <= 1000.0:
        return list(
            product(
                (3.2, 4.0, 4.8),
                (0.35, 0.7, 1.0),
                (0.9, 1.3, 1.7),
                (0.32, 0.45, 0.58),
                (0.00006, 0.0001, 0.00015),
                (74.0, 78.0, 82.0),
            )
        )

    return list(
        product(
            (3.2, 4.0, 4.8, 5.6),
            (0.2, 0.35, 0.7, 1.0),
            (0.7, 0.9, 1.3, 1.7),
            (0.45, 0.58, 0.72, 0.9),
            (0.00003, 0.00006, 0.0001, 0.00015),
            (68.0, 72.0, 74.0, 78.0),
        )
    )


def refine_gain_sets(best_results: list[TuningResult], tuning_mode: str) -> list[ControllerGains]:
    refined: list[ControllerGains] = []
    top_count = 2 if tuning_mode == "fast" else 3
    for result in best_results[:top_count]:
        g = result.gains
        if tuning_mode == "fast":
            kp_deltas = (-0.3, 0.0, 0.3)
            ki_deltas = (-0.08, 0.0, 0.08)
            kd_deltas = (-0.15, 0.0, 0.15)
            guidance_deltas = (-0.06, 0.0, 0.06)
            altitude_deltas = (-0.00001, 0.0, 0.00001)
            gamma_deltas = (-1.0, 0.0, 1.0)
        else:
            kp_deltas = (-0.4, 0.0, 0.4)
            ki_deltas = (-0.1, 0.0, 0.1)
            kd_deltas = (-0.2, 0.0, 0.2)
            guidance_deltas = (-0.08, 0.0, 0.08)
            altitude_deltas = (-0.00002, 0.0, 0.00002)
            gamma_deltas = (-2.0, 0.0, 2.0)

        kp_values = sorted({max(1.0, round(g.kp + delta, 2)) for delta in kp_deltas})
        ki_values = sorted({max(0.0, round(g.ki + delta, 2)) for delta in ki_deltas})
        kd_values = sorted({max(0.0, round(g.kd + delta, 2)) for delta in kd_deltas})
        guidance_values = sorted({max(0.1, round(g.guidance_gain + delta, 2)) for delta in guidance_deltas})
        altitude_values = sorted(
            {
                max(0.00001, round(g.guidance_altitude_gain + delta, 5))
                for delta in altitude_deltas
            }
        )
        gamma_values = sorted({round(g.gamma_final_deg + delta, 1) for delta in gamma_deltas})

        for kp, ki, kd, guidance_gain, guidance_altitude_gain, gamma_final_deg in product(
            kp_values,
            ki_values,
            kd_values,
            guidance_values,
            altitude_values,
            gamma_values,
        ):
            refined.append(
                ControllerGains(
                    kp=kp,
                    ki=ki,
                    kd=kd,
                    guidance_gain=guidance_gain,
                    guidance_altitude_gain=guidance_altitude_gain,
                    gamma_final_deg=gamma_final_deg,
                )
            )

    unique: dict[tuple[float, float, float, float, float, float], ControllerGains] = {}
    for gains in refined:
        key = (
            gains.kp,
            gains.ki,
            gains.kd,
            gains.guidance_gain,
            gains.guidance_altitude_gain,
            gains.gamma_final_deg,
        )
        unique[key] = gains
    return list(unique.values())


def select_seed_results(results: list[TuningResult], tuning_mode: str) -> list[TuningResult]:
    ordered_by_score = sorted(results, key=lambda item: item.score)
    ordered_by_impact = sorted(results, key=lambda item: item.metrics["impact_error"])
    ordered_by_alpha = sorted(
        results,
        key=lambda item: (
            item.metrics["mean_alpha_deg"],
            item.metrics["saturation_ratio"],
            item.metrics["impact_error"],
        ),
    )

    target_count = 3 if tuning_mode == "fast" else 6
    seeds: list[TuningResult] = []
    seen: set[tuple[float, float, float, float, float, float]] = set()

    for pool in (ordered_by_score, ordered_by_impact, ordered_by_alpha):
        for result in pool:
            key = (
                result.gains.kp,
                result.gains.ki,
                result.gains.kd,
                result.gains.guidance_gain,
                result.gains.guidance_altitude_gain,
                result.gains.gamma_final_deg,
            )
            if key in seen:
                continue
            seen.add(key)
            seeds.append(result)
            if len(seeds) >= target_count:
                return seeds
    return seeds


def ultra_refine_gain_sets(best_results: list[TuningResult]) -> list[ControllerGains]:
    refined: list[ControllerGains] = []
    for result in best_results[:2]:
        g = result.gains
        for kp, ki, kd, guidance_gain, guidance_altitude_gain, gamma_final_deg in product(
            sorted({max(1.0, round(g.kp + delta, 2)) for delta in (-0.15, 0.0, 0.15)}),
            sorted({max(0.0, round(g.ki + delta, 2)) for delta in (-0.05, 0.0, 0.05)}),
            sorted({max(0.0, round(g.kd + delta, 2)) for delta in (-0.08, 0.0, 0.08)}),
            sorted({max(0.1, round(g.guidance_gain + delta, 2)) for delta in (-0.03, 0.0, 0.03)}),
            sorted(
                {
                    max(0.00001, round(g.guidance_altitude_gain + delta, 5))
                    for delta in (-0.00001, 0.0, 0.00001)
                }
            ),
            sorted({round(g.gamma_final_deg + delta, 1) for delta in (-0.8, 0.0, 0.8)}),
        ):
            refined.append(
                ControllerGains(
                    kp=kp,
                    ki=ki,
                    kd=kd,
                    guidance_gain=guidance_gain,
                    guidance_altitude_gain=guidance_altitude_gain,
                    gamma_final_deg=gamma_final_deg,
                )
            )

    unique: dict[tuple[float, float, float, float, float, float], ControllerGains] = {}
    for gains in refined:
        key = (
            gains.kp,
            gains.ki,
            gains.kd,
            gains.guidance_gain,
            gains.guidance_altitude_gain,
            gains.gamma_final_deg,
        )
        unique[key] = gains
    return list(unique.values())


def evaluate_candidate(
    base_params: RocketParameters,
    gains: ControllerGains,
) -> tuple[RocketParameters, TuningResult]:
    params = replace(
        base_params,
        guidance_gain=gains.guidance_gain,
        guidance_altitude_gain=gains.guidance_altitude_gain,
        gamma_final_deg=gains.gamma_final_deg,
    )
    history = simulate_rocket(params, build_pid(params, gains))
    score = score_history(history, params)
    metrics = evaluate_history(history, params)
    return params, TuningResult(gains=gains, metrics=metrics, score=score)


def auto_tune_controller(
    base_params: RocketParameters,
) -> tuple[RocketParameters, ControllerGains, dict[str, float], list[TuningResult]]:
    coarse_candidates = candidate_gain_sets(base_params.impact_target, base_params.tuning_mode)

    best_score = float("inf")
    best_params = base_params
    best_gains = ControllerGains(
        5.5,
        1.2,
        1.8,
        base_params.guidance_gain,
        base_params.guidance_altitude_gain,
        base_params.gamma_final_deg,
    )
    best_metrics: dict[str, float] = {}
    results: list[TuningResult] = []
    evaluated: set[tuple[float, float, float, float, float, float]] = set()

    def evaluate_and_store(gains: ControllerGains) -> None:
        nonlocal best_score, best_params, best_gains, best_metrics
        key = (
            gains.kp,
            gains.ki,
            gains.kd,
            gains.guidance_gain,
            gains.guidance_altitude_gain,
            gains.gamma_final_deg,
        )
        if key in evaluated:
            return
        evaluated.add(key)

        params, result = evaluate_candidate(base_params, gains)
        results.append(result)
        if result.score < best_score:
            best_score = result.score
            best_params = params
            best_gains = gains
            best_metrics = result.metrics

    for kp, ki, kd, guidance_gain, guidance_altitude_gain, gamma_final_deg in coarse_candidates:
        gains = ControllerGains(kp, ki, kd, guidance_gain, guidance_altitude_gain, gamma_final_deg)
        evaluate_and_store(gains)

    results.sort(key=lambda item: item.score)
    seed_results = select_seed_results(results, base_params.tuning_mode)
    refined_candidates = refine_gain_sets(seed_results, base_params.tuning_mode)

    for gains in refined_candidates:
        evaluate_and_store(gains)

    if base_params.tuning_mode == "accurate":
        results.sort(key=lambda item: item.score)
        ultra_seeds = select_seed_results(results, "accurate")
        for gains in ultra_refine_gain_sets(ultra_seeds):
            evaluate_and_store(gains)

    best_metrics["score"] = best_score
    results.sort(key=lambda item: item.score)
    return best_params, best_gains, best_metrics, results


def write_tuning_csv(results: list[TuningResult], output_path: Path) -> None:
    fieldnames = [
        "rank",
        "score",
        "kp",
        "ki",
        "kd",
        "guidance_gain",
        "guidance_altitude_gain",
        "gamma_final_deg",
        "peak_altitude",
        "downrange",
        "impact_error",
        "final_speed",
        "flight_time",
        "landed",
        "mean_alpha_deg",
        "max_alpha_deg",
        "mean_gimbal_deg",
        "max_gimbal_deg",
        "saturation_ratio",
        "mean_gamma_error_deg",
    ]
    with output_path.open("w", newline="", encoding="utf-8") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for rank, result in enumerate(results, start=1):
            writer.writerow(
                {
                    "rank": rank,
                    "score": f"{result.score:.4f}",
                    "kp": result.gains.kp,
                    "ki": result.gains.ki,
                    "kd": result.gains.kd,
                    "guidance_gain": result.gains.guidance_gain,
                    "guidance_altitude_gain": result.gains.guidance_altitude_gain,
                    "gamma_final_deg": result.gains.gamma_final_deg,
                    **{key: f"{value:.6f}" for key, value in result.metrics.items()},
                }
            )


def write_history_csv(history: list[dict[str, float]], output_path: Path) -> None:
    if not history:
        return
    fieldnames = list(history[0].keys())
    with output_path.open("w", newline="", encoding="utf-8") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(history)


def save_figure(fig, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=160, bbox_inches="tight")


def prepare_matplotlib(show_plots: bool):
    import matplotlib

    if show_plots:
        try:
            import PyQt6  # noqa: F401

            matplotlib.use("QtAgg")
        except Exception:
            try:
                import tkinter as tk

                root = tk.Tk()
                root.withdraw()
                root.destroy()
            except Exception:
                print("Backend graphique indisponible : bascule automatique en sauvegarde PNG seule.")
                show_plots = False

    if not show_plots:
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    return plt, show_plots


def present_figure(plt, fig, show_plots: bool, display_seconds: float) -> None:
    if show_plots:
        plt.show(block=False)
        plt.pause(max(display_seconds, 0.1))
        plt.close(fig)
    else:
        plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simulation de fusee TVC avec tuning automatique vers un point d'impact cible."
    )
    parser.add_argument(
        "--impact-target",
        type=float,
        default=None,
        help="Distance cible d'impact au sol en metres.",
    )
    parser.add_argument(
        "--altitude-target",
        type=float,
        default=None,
        help="Apogee cible en metres.",
    )
    parser.add_argument(
        "--display-seconds",
        type=float,
        default=None,
        help="Duree d'affichage automatique des graphes en secondes.",
    )
    parser.add_argument(
        "--save-only",
        action="store_true",
        help="Sauvegarde les PNG sans ouvrir les fenetres de graphes.",
    )
    parser.add_argument(
        "--tuning-mode",
        choices=("fast", "accurate"),
        default="accurate",
        help="Mode de tuning: rapide ou plus precis.",
    )
    return parser.parse_args()


def maybe_plot_tuning_comparison(
    base_params: RocketParameters,
    tuning_results: list[TuningResult],
    output_dir: Path,
    top_n: int = 5,
) -> None:
    try:
        plt, show_plots = prepare_matplotlib(base_params.show_plots)
    except ImportError:
        print("matplotlib non disponible : comparaison tuning ignoree.")
        return

    top_results = tuning_results[:top_n]
    if not top_results:
        return

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))

    for index, result in enumerate(top_results, start=1):
        params = replace(
            base_params,
            guidance_gain=result.gains.guidance_gain,
            guidance_altitude_gain=result.gains.guidance_altitude_gain,
            gamma_final_deg=result.gains.gamma_final_deg,
        )
        history = simulate_rocket(params, build_pid(params, result.gains))
        x = [sample["x"] for sample in history]
        z = [sample["z"] for sample in history]
        t = [sample["t"] for sample in history]
        alpha = [degrees(sample["alpha_air"]) for sample in history]
        label = (
            f"#{index} "
            f"S={result.score:.0f} "
            f"sat={result.metrics['saturation_ratio'] * 100.0:.1f}%"
        )

        axes[0].plot(x, z, linewidth=2.0, label=label)
        axes[1].plot(t, alpha, linewidth=2.0, label=label)

    axes[0].set_title("Top 5 trajectoires")
    axes[0].set_xlabel("Distance horizontale (m)")
    axes[0].set_ylabel("Altitude (m)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)

    axes[1].set_title("Top 5 angles d'attaque")
    axes[1].set_xlabel("Temps (s)")
    axes[1].set_ylabel("Alpha (deg)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(fontsize=8)

    fig.tight_layout()
    if base_params.save_plots:
        save_figure(fig, output_dir / "tvc_rocket_top5_comparison.png")
    present_figure(plt, fig, show_plots, base_params.plot_display_seconds)


def summarize(history: list[dict[str, float]], params: RocketParameters) -> None:
    metrics = evaluate_history(history, params)
    final = history[-1]
    max_wind = max(abs(sample["wind_x"]) for sample in history)

    print("Simulation TVC + PID terminee")
    print(f"Apogee               : {metrics['peak_altitude']:8.1f} m")
    print(f"Portee horizontale   : {metrics['downrange']:8.1f} m")
    print(f"Cible d'impact       : {params.impact_target:8.1f} m")
    print(f"Erreur d'impact      : {abs(metrics['downrange'] - params.impact_target):8.1f} m")
    print(f"Temps de vol         : {metrics['flight_time']:8.2f} s")
    print(f"Impact au sol        : {'oui' if metrics['landed'] >= 0.5 else 'non'}")
    print(f"Vitesse finale       : {final['speed']:8.1f} m/s")
    print(f"Angle d'attaque moyen: {metrics['mean_alpha_deg']:8.3f} deg")
    print(f"Alpha final          : {degrees(final['alpha_air']):8.3f} deg")
    print(f"Braquage final TVC   : {degrees(final['gimbal']):8.3f} deg")
    print(f"Braquage moyen TVC   : {metrics['mean_gimbal_deg']:8.3f} deg")
    print(f"Braquage max TVC     : {metrics['max_gimbal_deg']:8.3f} deg")
    print(f"Taux de saturation   : {metrics['saturation_ratio'] * 100.0:8.3f} %")
    print(f"Erreur gamma moyenne : {metrics['mean_gamma_error_deg']:8.3f} deg")
    print(f"Vent horizontal max  : {max_wind:8.3f} m/s")


def maybe_plot(history: list[dict[str, float]], params: RocketParameters, output_dir: Path) -> None:
    try:
        plt, show_plots = prepare_matplotlib(params.show_plots)
    except ImportError:
        print("matplotlib non disponible : graphiques ignores.")
        return

    t = [sample["t"] for sample in history]
    x = [sample["x"] for sample in history]
    z = [sample["z"] for sample in history]
    alpha = [degrees(sample["alpha_air"]) for sample in history]
    alpha_cmd = [degrees(sample["alpha_cmd"]) for sample in history]
    gamma = [degrees(sample["gamma"]) for sample in history]
    gamma_ref = [degrees(sample["gamma_ref"]) for sample in history]
    gimbal = [degrees(sample["gimbal"]) for sample in history]
    gimbal_cmd = [degrees(sample["gimbal_cmd"]) for sample in history]
    wind_x = [sample["wind_x"] for sample in history]

    fig, axes = plt.subplots(2, 2, figsize=(11, 8))

    axes[0, 0].plot(x, z, color="tab:blue", linewidth=2.0)
    axes[0, 0].set_title("Trajectoire")
    axes[0, 0].set_xlabel("Distance horizontale (m)")
    axes[0, 0].set_ylabel("Altitude (m)")
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(t, alpha, label="alpha reel", linewidth=2.0)
    axes[0, 1].plot(t, alpha_cmd, "--", label="alpha consigne", linewidth=2.0)
    axes[0, 1].set_title("Controle de l'angle d'attaque")
    axes[0, 1].set_xlabel("Temps (s)")
    axes[0, 1].set_ylabel("Angle (deg)")
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].plot(t, gamma, label="gamma reel", linewidth=2.0)
    axes[1, 0].plot(t, gamma_ref, "--", label="gamma reference", linewidth=2.0)
    axes[1, 0].set_title("Loi de guidage")
    axes[1, 0].set_xlabel("Temps (s)")
    axes[1, 0].set_ylabel("Angle de trajectoire (deg)")
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].plot(t, gimbal_cmd, "--", label="commande PID", linewidth=1.8)
    axes[1, 1].plot(t, gimbal, color="tab:red", label="actuateur TVC", linewidth=2.0)
    axes[1, 1].set_title("Commande TVC")
    axes[1, 1].set_xlabel("Temps (s)")
    axes[1, 1].set_ylabel("Braquage tuyere (deg)")
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    fig.tight_layout()
    if params.save_plots:
        save_figure(fig, output_dir / "tvc_rocket_main_plots.png")
    present_figure(plt, fig, show_plots, params.plot_display_seconds)

    fig_wind = plt.figure(figsize=(8, 3.5))
    plt.plot(t, wind_x, color="tab:green", linewidth=2.0)
    plt.title("Profil de vent horizontal")
    plt.xlabel("Temps (s)")
    plt.ylabel("Vent (m/s)")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    if params.save_plots:
        save_figure(fig_wind, output_dir / "tvc_rocket_wind_profile.png")
    present_figure(plt, fig_wind, show_plots, params.plot_display_seconds)


def main() -> None:
    args = parse_args()
    base_params = RocketParameters(tuning_mode=args.tuning_mode)
    if args.impact_target is not None:
        base_params = replace(base_params, impact_target=args.impact_target)
    if args.altitude_target is not None:
        base_params = replace(base_params, altitude_target=args.altitude_target)
    if args.display_seconds is not None:
        base_params = replace(base_params, plot_display_seconds=max(args.display_seconds, 0.1))
    if args.save_only:
        base_params = replace(base_params, show_plots=False, save_plots=True)

    params, gains, tuning_metrics, tuning_results = auto_tune_controller(base_params)
    pid = build_pid(params, gains)
    output_dir = Path(__file__).resolve().parent
    results_dir = output_dir / "results"
    plots_dir = output_dir / "plots"
    results_dir.mkdir(parents=True, exist_ok=True)
    tuning_csv = results_dir / "tvc_rocket_tuning_results.csv"
    history_csv = results_dir / "tvc_rocket_best_history.csv"

    print("Meilleure configuration trouvee")
    print(f"Mode de tuning       : {params.tuning_mode}")
    print(
        "PID="
        f"({gains.kp:.2f}, {gains.ki:.2f}, {gains.kd:.2f}) "
        f"guidage={gains.guidance_gain:.2f} "
        f"alt_gain={gains.guidance_altitude_gain:.5f} "
        f"gamma_fin={gains.gamma_final_deg:.1f} deg"
    )
    print(
        f"Score={tuning_metrics['score']:.1f} "
        f"apogee={tuning_metrics['peak_altitude']:.1f} m "
        f"portee={tuning_metrics['downrange']:.1f} m "
        f"err_impact={tuning_metrics['impact_error']:.1f} m "
        f"t_vol={tuning_metrics['flight_time']:.1f} s "
        f"alpha_moy={tuning_metrics['mean_alpha_deg']:.2f} deg "
        f"sat={tuning_metrics['saturation_ratio'] * 100.0:.1f} %"
    )

    history = simulate_rocket(params, pid)
    write_tuning_csv(tuning_results, tuning_csv)
    write_history_csv(history, history_csv)
    print(f"CSV tuning           : {tuning_csv}")
    print(f"CSV trajectoire      : {history_csv}")
    if params.save_plots:
        print(f"Dossier PNG          : {plots_dir}")
    summarize(history, params)
    maybe_plot(history, params, plots_dir)
    maybe_plot_tuning_comparison(params, tuning_results, plots_dir)


if __name__ == "__main__":
    main()
