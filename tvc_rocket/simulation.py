from __future__ import annotations

from math import atan2, cos, degrees, exp, pi, sin

from .models import (
    ControllerGains,
    GuidanceState,
    PIDController,
    RocketParameters,
    RocketState,
    TVCActuator,
    clamp,
    wrap_angle,
)


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


def scheduled_alpha_limit(q_dyn: float, params: RocketParameters) -> float:
    q_low = params.alpha_schedule_q_low_pa
    q_high = max(q_low + 1.0, params.alpha_schedule_q_high_pa)
    if q_dyn <= q_low:
        return params.max_alpha_cmd_rad
    if q_dyn >= q_high:
        return params.max_alpha_cmd_high_q_rad

    ratio = (q_dyn - q_low) / (q_high - q_low)
    return params.max_alpha_cmd_rad + ratio * (
        params.max_alpha_cmd_high_q_rad - params.max_alpha_cmd_rad
    )


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
    alpha_limit = scheduled_alpha_limit(q_dyn, params)
    if params.use_alpha_schedule and state.t >= params.launch_rail_time and is_burning:
        alpha_cmd = clamp(alpha_cmd, -alpha_limit, alpha_limit)
        gimbal_cmd = pid.update(alpha_cmd - alpha_air, dt, freeze_integrator=freeze_integrator)
        gimbal = actuator.update(gimbal_cmd, dt)
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
        "alpha_limit": alpha_limit,
        "gimbal_cmd": gimbal_cmd,
        "gimbal": gimbal,
        "speed": speed,
        "airspeed": airspeed,
        "thrust": thrust,
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

    history: list[dict[str, float]] = [
        sample_state(
            state,
            {
                "gamma": gamma0,
                "gamma_ref": gamma0,
                "alpha": wrap_angle(state.theta - gamma0),
                "alpha_air": wrap_angle(state.theta - gamma0),
                "alpha_cmd": 0.0,
                "alpha_limit": params.max_alpha_cmd_rad,
                "gimbal_cmd": 0.0,
                "gimbal": 0.0,
                "speed": initial_speed,
                "airspeed": initial_speed,
                "thrust": params.thrust,
                "wind_x": wind_velocity(state.z, params),
            },
        )
    ]
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
