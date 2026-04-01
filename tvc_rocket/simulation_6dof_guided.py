from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, degrees, pi, sin, sqrt

from .models import PIDController, RocketParameters, clamp, wrap_angle
from .simulation import atmosphere_state, effective_drag_coefficient, gamma_reference, thrust_profile_factor


@dataclass
class RocketState6DOFGuided:
    t: float
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    roll: float
    pitch: float
    yaw: float
    p: float
    q: float
    r: float
    mass: float


def _safe(value: float, limit: float) -> float:
    return clamp(value, -limit, limit)


def _crosswind_y(altitude: float, params: RocketParameters) -> float:
    altitude = max(altitude, 0.0)
    return params.crosswind_ref_m_s * (1.0 - 0.25 * min(altitude / 10000.0, 1.0))


def build_short_range_guided_params(params: RocketParameters) -> RocketParameters:
    return RocketParameters(
        **{
            **params.__dict__,
            "thrust": min(params.thrust, 350.0),
            "burn_time": min(params.burn_time, 6.5),
            "propellant_mass": min(params.propellant_mass, 3.2),
            "altitude_target": min(params.altitude_target, 180.0),
            "theta_initial_deg": min(params.theta_initial_deg, 52.0),
            "gamma_final_deg": min(params.gamma_final_deg, 20.0),
            "crosswind_ref_m_s": min(params.crosswind_ref_m_s, 2.0),
            "guidance_altitude_gain": min(max(params.guidance_altitude_gain, 0.00010), 0.00014),
            "guidance_crossrange_gain": max(params.guidance_crossrange_gain, 2.8),
        }
    )


def _build_pids(params: RocketParameters) -> tuple[PIDController, PIDController, PIDController]:
    roll_pid = PIDController(
        kp=1.4,
        ki=0.02,
        kd=0.30,
        output_limit=6.0 * pi / 180.0,
        integral_limit=0.08,
        derivative_filter=0.88,
    )
    pitch_pid = PIDController(
        kp=3.0,
        ki=0.18,
        kd=0.75,
        output_limit=params.max_gimbal_rad,
        integral_limit=0.16,
        derivative_filter=0.88,
    )
    yaw_pid = PIDController(
        kp=2.4,
        ki=0.10,
        kd=0.35,
        output_limit=0.55 * params.max_gimbal_rad,
        integral_limit=0.10,
        derivative_filter=0.88,
    )
    return roll_pid, pitch_pid, yaw_pid


def _guidance_targets(
    state: RocketState6DOFGuided,
    params: RocketParameters,
    phase_mode: str,
) -> tuple[str, float, float, float]:
    # Convention d'axes/signes figee pour ce module de reglage :
    # +X = downrange, +Y = droite/crossrange, +Z = haut
    # yaw > 0 pointe le nez vers +Y ; pitch > 0 pointe le nez vers +Z ; roll > 0 roule a droite
    burn_progress = clamp(
        (state.t - params.launch_rail_time) / max(params.burn_time - params.launch_rail_time, 1e-6),
        0.0,
        1.0,
    )
    dx = params.impact_target - state.x
    dy = params.impact_target_y - state.y
    range_progress = clamp(state.x / max(params.impact_target, 1.0), 0.0, 2.0)
    desired_altitude_profile = params.target_final_altitude + (params.altitude_target - params.target_final_altitude) * max(0.0, 1.0 - range_progress) ** 1.35
    flight_path_pitch = atan2(state.vz, max(sqrt(state.vx * state.vx + state.vy * state.vy), 1e-6))
    flight_path_yaw = atan2(state.vy, max(abs(state.vx), 1e-6))

    lateral_priority = abs(dy) > max(25.0, 0.08 * max(abs(params.impact_target_y), 1.0))
    if phase_mode == "longitudinal":
        phase = "longitudinal"
    elif phase_mode == "lateral":
        phase = "lateral"
    elif phase_mode == "target3d":
        phase = "target_3d"
    elif burn_progress < 0.28 or state.x < 0.18 * max(params.impact_target, 1.0):
        phase = "longitudinal"
    elif lateral_priority and burn_progress < 0.82:
        phase = "lateral"
    else:
        phase = "target_3d"

    if phase == "longitudinal":
        desired_pitch = clamp(
            0.75 * gamma_reference(state.t, params)
            + 0.25 * flight_path_pitch
            + params.guidance_altitude_gain * (desired_altitude_profile - state.z),
            5.0 * pi / 180.0,
            40.0 * pi / 180.0,
        )
        desired_yaw = 0.0
    elif phase == "lateral":
        desired_pitch = clamp(
            0.55 * flight_path_pitch + 0.45 * atan2(desired_altitude_profile - state.z, max(abs(dx), 80.0)),
            -8.0 * pi / 180.0,
            20.0 * pi / 180.0,
        )
        desired_yaw = clamp(
            0.15 * flight_path_yaw + 0.85 * atan2(params.guidance_crossrange_gain * dy, max(abs(dx), 40.0)),
            -24.0 * pi / 180.0,
            24.0 * pi / 180.0,
        )
    else:
        desired_pitch = clamp(
            0.35 * flight_path_pitch
            + 0.65 * atan2(
                params.target_final_altitude - state.z,
                dx if abs(dx) > 25.0 else (25.0 if dx >= 0.0 else -25.0),
            ),
            -35.0 * pi / 180.0,
            18.0 * pi / 180.0,
        )
        desired_yaw = clamp(
            0.10 * flight_path_yaw
            + 0.90
            * atan2(
                params.guidance_crossrange_gain * dy,
                dx if abs(dx) > 1.0 else (1.0 if dx >= 0.0 else -1.0),
            ),
            -24.0 * pi / 180.0,
            24.0 * pi / 180.0,
        )

    desired_roll = clamp(-0.12 * desired_yaw, -6.0 * pi / 180.0, 6.0 * pi / 180.0)
    return phase, desired_roll, desired_pitch, desired_yaw


def _sample(state: RocketState6DOFGuided, telemetry: dict[str, float]) -> dict[str, float]:
    return {
        "t": state.t,
        "x": state.x,
        "y": state.y,
        "z": state.z,
        "vx": state.vx,
        "vy": state.vy,
        "vz": state.vz,
        "roll": state.roll,
        "pitch": state.pitch,
        "yaw": state.yaw,
        "p": state.p,
        "q": state.q,
        "r": state.r,
        "mass": state.mass,
        **telemetry,
    }


def _interpolate_ground(previous: dict[str, float], current: dict[str, float]) -> dict[str, float]:
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


def simulate_rocket_6dof_guided(params: RocketParameters, phase_mode: str = "auto") -> list[dict[str, float]]:
    roll_pid, pitch_pid, yaw_pid = _build_pids(params)
    pitch0 = params.theta_initial_deg * pi / 180.0

    state = RocketState6DOFGuided(
        t=0.0,
        x=0.0,
        y=0.0,
        z=0.0,
        vx=25.0 * cos(pitch0),
        vy=0.0,
        vz=25.0 * sin(pitch0),
        roll=0.0,
        pitch=pitch0,
        yaw=0.0,
        p=0.0,
        q=0.0,
        r=0.0,
        mass=params.dry_mass + params.propellant_mass,
    )

    atm0 = atmosphere_state(0.0)
    history: list[dict[str, float]] = [
        _sample(
            state,
            {
                "phase": "rail",
                "speed": 25.0,
                "airspeed": 25.0,
                "mach": 25.0 / max(atm0["sound_speed"], 1.0),
                "q_dyn": 0.5 * atm0["density"] * 25.0 * 25.0,
                "rho": atm0["density"],
                "cd_eff": params.cd0,
                "alpha_deg": 0.0,
                "beta_deg": 0.0,
                "roll_ref_deg": 0.0,
                "pitch_ref_deg": degrees(pitch0),
                "yaw_ref_deg": 0.0,
                "roll_error_deg": 0.0,
                "pitch_error_deg": 0.0,
                "yaw_error_deg": 0.0,
                "gimbal_pitch_deg": 0.0,
                "gimbal_yaw_deg": 0.0,
            },
        )
    ]

    while state.t <= params.t_final:
        dt = params.dt
        wind_x = params.wind_ref_m_s
        wind_y = _crosswind_y(state.z, params)
        air_vx = state.vx - wind_x
        air_vy = state.vy - wind_y
        air_vz = state.vz
        speed = max(sqrt(state.vx * state.vx + state.vy * state.vy + state.vz * state.vz), 1.0)
        airspeed = max(sqrt(air_vx * air_vx + air_vy * air_vy + air_vz * air_vz), 1.0)

        gamma_pitch = atan2(air_vz, max(sqrt(air_vx * air_vx + air_vy * air_vy), 1e-6))
        gamma_yaw = atan2(air_vy, max(abs(air_vx), 1e-6))
        alpha = wrap_angle(state.pitch - gamma_pitch)
        beta = clamp(atan2(air_vy, max(sqrt(air_vx * air_vx + air_vz * air_vz), 1e-6)), -35.0 * pi / 180.0, 35.0 * pi / 180.0)

        is_burning = state.t < params.burn_time and state.mass > params.dry_mass
        atm = atmosphere_state(state.z)
        q_dyn = 0.5 * atm["density"] * airspeed * airspeed
        mach = airspeed / max(atm["sound_speed"], 1.0)

        phase, roll_ref, pitch_ref, yaw_ref = _guidance_targets(state, params, phase_mode)
        if state.t < params.launch_rail_time or not is_burning:
            gimbal_pitch = 0.0
            gimbal_yaw = 0.0
            roll_cmd = 0.0
        else:
            roll_error = wrap_angle(roll_ref - state.roll)
            pitch_error = wrap_angle(pitch_ref - state.pitch)
            yaw_error = wrap_angle(yaw_ref - state.yaw)
            roll_cmd = roll_pid.update(roll_error, dt, freeze_integrator=False)
            gimbal_pitch = pitch_pid.update(pitch_error - 0.35 * alpha, dt, freeze_integrator=False)
            gimbal_yaw = yaw_pid.update(yaw_error - 0.25 * beta, dt, freeze_integrator=False)
            gimbal_pitch = clamp(gimbal_pitch, -params.max_gimbal_rad, params.max_gimbal_rad)
            yaw_gimbal_limit = 0.0
            if phase == "lateral":
                yaw_gimbal_limit = 0.80 * params.max_gimbal_rad
            elif phase == "target_3d":
                yaw_gimbal_limit = 0.70 * params.max_gimbal_rad
            gimbal_yaw = clamp(gimbal_yaw, -yaw_gimbal_limit, yaw_gimbal_limit)

            if phase == "longitudinal":
                roll_cmd = 0.0
                gimbal_yaw = 0.0

        thrust = params.thrust * thrust_profile_factor(state.t, params) if is_burning else 0.0
        mdot = params.mass_flow if is_burning else 0.0

        thrust_x = thrust * cos(state.pitch + gimbal_pitch) * cos(state.yaw + gimbal_yaw)
        thrust_y = thrust * sin(state.yaw + gimbal_yaw)
        thrust_z = thrust * sin(state.pitch + gimbal_pitch)

        cd_eff = effective_drag_coefficient((alpha * alpha + beta * beta) ** 0.5, mach, params)
        drag = q_dyn * params.area_ref * cd_eff
        if airspeed > 1e-9:
            drag_x = drag * air_vx / airspeed
            drag_y = drag * air_vy / airspeed
            drag_z = drag * air_vz / airspeed
        else:
            drag_x = drag_y = drag_z = 0.0

        lateral_force_scale = 0.05 if phase == "longitudinal" else (0.12 if phase == "lateral" else 0.18)
        lateral_moment_scale = 0.05 if phase == "longitudinal" else (0.10 if phase == "lateral" else 0.15)
        side_y = -lateral_force_scale * q_dyn * params.area_ref * params.cy_beta * beta
        lift_z = -0.55 * q_dyn * params.area_ref * params.cl_alpha * alpha

        ax = (thrust_x - drag_x) / state.mass
        ay = (thrust_y + side_y - drag_y) / state.mass
        az = (thrust_z + lift_z - drag_z) / state.mass - 9.81

        roll_moment = -params.roll_damping * state.p + (0.0 if phase == "longitudinal" else 0.6 * roll_cmd)
        pitch_moment = thrust * params.tvc_arm * sin(gimbal_pitch) - 0.35 * q_dyn * params.area_ref * params.body_length * alpha - params.pitch_damping * state.q
        yaw_moment = thrust * params.tvc_arm * sin(gimbal_yaw) - lateral_moment_scale * q_dyn * params.area_ref * params.body_length * beta - params.yaw_damping * state.r

        p_dot = _safe(roll_moment / max(params.ixx, 1e-6), 3.0)
        q_dot = _safe(pitch_moment / max(params.iyy, 1e-6), 5.0)
        r_dot = _safe(yaw_moment / max(params.izz, 1e-6), 3.0)

        next_state = RocketState6DOFGuided(
            t=state.t + dt,
            x=state.x + state.vx * dt,
            y=state.y + state.vy * dt,
            z=max(0.0, state.z + state.vz * dt),
            vx=_safe(state.vx + ax * dt, 600.0),
            vy=_safe(state.vy + ay * dt, 250.0),
            vz=_safe(state.vz + az * dt, 600.0),
            roll=clamp(wrap_angle(state.roll + state.p * dt), -35.0 * pi / 180.0, 35.0 * pi / 180.0),
            pitch=clamp(wrap_angle(state.pitch + state.q * dt), -60.0 * pi / 180.0, 85.0 * pi / 180.0),
            yaw=clamp(wrap_angle(state.yaw + state.r * dt), -20.0 * pi / 180.0, 20.0 * pi / 180.0),
            p=_safe(state.p + p_dot * dt, 2.0),
            q=_safe(state.q + q_dot * dt, 3.0),
            r=_safe(state.r + r_dot * dt, 2.0),
            mass=max(params.dry_mass, state.mass - mdot * dt),
        )

        sample = _sample(
            next_state,
            {
                "phase": phase,
                "speed": speed,
                "airspeed": airspeed,
                "mach": mach,
                "q_dyn": q_dyn,
                "rho": atm["density"],
                "cd_eff": cd_eff,
                "alpha_deg": degrees(alpha),
                "beta_deg": degrees(beta),
                "roll_ref_deg": degrees(roll_ref),
                "pitch_ref_deg": degrees(pitch_ref),
                "yaw_ref_deg": degrees(yaw_ref),
                "roll_error_deg": degrees(wrap_angle(roll_ref - state.roll)),
                "pitch_error_deg": degrees(wrap_angle(pitch_ref - state.pitch)),
                "yaw_error_deg": degrees(wrap_angle(yaw_ref - state.yaw)),
                "gimbal_pitch_deg": degrees(gimbal_pitch),
                "gimbal_yaw_deg": degrees(gimbal_yaw),
            },
        )
        history.append(sample)
        state = next_state
        if state.t > params.burn_time and state.z <= 0.0:
            history[-1] = _interpolate_ground(history[-2], sample) if len(history) >= 2 else sample
            break

    return history


def summarize_6dof_guided_metrics(history: list[dict[str, float]], params: RocketParameters) -> dict[str, float]:
    final = history[-1]
    return {
        "peak_altitude": max(sample["z"] for sample in history),
        "downrange": final["x"],
        "crossrange": final["y"],
        "impact_error_3d": sqrt((final["x"] - params.impact_target) ** 2 + (final["y"] - params.impact_target_y) ** 2),
        "flight_time": final["t"],
        "max_abs_yaw_error_deg": max(abs(sample["yaw_error_deg"]) for sample in history),
        "max_abs_pitch_error_deg": max(abs(sample["pitch_error_deg"]) for sample in history),
        "max_abs_beta_deg": max(abs(sample["beta_deg"]) for sample in history),
        "max_abs_alpha_deg": max(abs(sample["alpha_deg"]) for sample in history),
    }
