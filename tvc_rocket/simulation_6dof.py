from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, degrees, pi, sin, sqrt, tan

from .models import PIDController, RocketParameters, clamp, wrap_angle
from .simulation import (
    atmosphere_state,
    effective_drag_coefficient,
    gamma_reference,
    thrust_profile_factor,
)


@dataclass
class RocketState6DOF:
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


def lateral_wind_velocity(altitude: float, params: RocketParameters) -> float:
    altitude = max(altitude, 0.0)
    return params.crosswind_ref_m_s * (1.0 - 0.35 * min(altitude / 12000.0, 1.0))


def build_attitude_pid(params: RocketParameters) -> tuple[PIDController, PIDController]:
    pitch_pid = PIDController(
        kp=5.0,
        ki=0.55,
        kd=1.5,
        output_limit=params.max_gimbal_rad,
        integral_limit=0.3,
        derivative_filter=0.82,
    )
    yaw_pid = PIDController(
        kp=1.6,
        ki=0.08,
        kd=0.45,
        output_limit=0.4 * params.max_gimbal_rad,
        integral_limit=0.08,
        derivative_filter=0.88,
    )
    return pitch_pid, yaw_pid


def _rotation_body_to_inertial(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    cr = cos(roll)
    sr = sin(roll)
    cp = cos(pitch)
    sp = sin(pitch)
    cy = cos(yaw)
    sy = sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _mat_vec_mul(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    ]


def _transpose(matrix: list[list[float]]) -> list[list[float]]:
    return [
        [matrix[0][0], matrix[1][0], matrix[2][0]],
        [matrix[0][1], matrix[1][1], matrix[2][1]],
        [matrix[0][2], matrix[1][2], matrix[2][2]],
    ]


def _guidance_reference_3d(state: RocketState6DOF, params: RocketParameters) -> tuple[float, float, float]:
    target_x = params.impact_target
    target_y = params.impact_target_y
    target_z = params.target_final_altitude

    dx = target_x - state.x
    dy = target_y - state.y
    burn_progress = clamp(
        (state.t - params.launch_rail_time) / max(0.75 * params.burn_time, 1e-6),
        0.0,
        1.0,
    )
    desired_altitude = target_z + (params.altitude_target - target_z) * (1.0 - burn_progress) ** 1.5
    dz = desired_altitude - state.z
    horizontal_range = max(sqrt(dx * dx + dy * dy), 1.0)

    schedule_pitch = gamma_reference(state.t, params)
    los_pitch = atan2(dz, horizontal_range)
    pitch_ref = clamp(
        (1.0 - burn_progress * params.guidance_los_blend) * schedule_pitch
        + (burn_progress * params.guidance_los_blend) * los_pitch,
        -10.0 * pi / 180.0,
        90.0 * pi / 180.0,
    )
    pitch_ref = clamp(
        pitch_ref + params.guidance_altitude_gain * (desired_altitude - state.z),
        -10.0 * pi / 180.0,
        90.0 * pi / 180.0,
    )

    dx_ref = dx if abs(dx) >= 1.0 else (1.0 if dx >= 0.0 else -1.0)
    desired_yaw = atan2(dy, dx_ref)
    yaw_ref = clamp(
        wrap_angle(params.guidance_crossrange_gain * desired_yaw),
        -20.0 * pi / 180.0,
        20.0 * pi / 180.0,
    )

    roll_ref = 0.0
    return roll_ref, pitch_ref, yaw_ref


def _safe_value(value: float, limit: float) -> float:
    return clamp(value, -limit, limit)


def _sample_6dof(state: RocketState6DOF, telemetry: dict[str, float]) -> dict[str, float]:
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


def _interpolate_ground_impact(previous: dict[str, float], current: dict[str, float]) -> dict[str, float]:
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


def _step_6dof(
    state: RocketState6DOF,
    params: RocketParameters,
    pitch_pid: PIDController,
    yaw_pid: PIDController,
) -> tuple[RocketState6DOF, dict[str, float]]:
    dt = params.dt
    g = 9.81

    wind_x = params.wind_ref_m_s
    wind_y = lateral_wind_velocity(state.z, params)
    air_velocity_inertial = [state.vx - wind_x, state.vy - wind_y, state.vz]
    speed = max(sqrt(state.vx**2 + state.vy**2 + state.vz**2), 1.0)
    airspeed = max(sqrt(air_velocity_inertial[0] ** 2 + air_velocity_inertial[1] ** 2 + air_velocity_inertial[2] ** 2), 1.0)

    rotation_bi = _rotation_body_to_inertial(state.roll, state.pitch, state.yaw)
    rotation_ib = _transpose(rotation_bi)
    air_velocity_body = _mat_vec_mul(rotation_ib, air_velocity_inertial)
    vbx, vby, vbz = air_velocity_body

    alpha_pitch = atan2(vbz, max(vbx, 1e-9))
    beta_yaw = atan2(vby, max(vbx, 1e-9))

    is_burning = state.t < params.burn_time and state.mass > params.dry_mass
    atm = atmosphere_state(state.z)
    mach = airspeed / max(atm["sound_speed"], 1.0)
    q_dyn = 0.5 * atm["density"] * airspeed * airspeed

    roll_ref, pitch_ref, yaw_ref = _guidance_reference_3d(state, params)
    if state.t < params.launch_rail_time or not is_burning:
        gimbal_pitch_cmd = 0.0
        gimbal_yaw_cmd = 0.0
        freeze_integrator = True
    else:
        freeze_integrator = False
        pitch_error = wrap_angle(pitch_ref - state.pitch)
        yaw_error = wrap_angle(yaw_ref - state.yaw)
        gimbal_pitch_cmd = pitch_pid.update(pitch_error - 0.55 * alpha_pitch, dt, freeze_integrator)
        gimbal_yaw_cmd = yaw_pid.update(yaw_error - 0.65 * beta_yaw, dt, freeze_integrator)

    gimbal_pitch = clamp(gimbal_pitch_cmd, -params.max_gimbal_rad, params.max_gimbal_rad)
    gimbal_yaw = clamp(gimbal_yaw_cmd, -0.4 * params.max_gimbal_rad, 0.4 * params.max_gimbal_rad)

    thrust = params.thrust * thrust_profile_factor(state.t, params) if is_burning else 0.0
    mdot = params.mass_flow if is_burning else 0.0

    thrust_body = [
        thrust * cos(gimbal_pitch) * cos(gimbal_yaw),
        thrust * sin(gimbal_yaw),
        thrust * sin(gimbal_pitch),
    ]

    alpha_total = sqrt(alpha_pitch * alpha_pitch + beta_yaw * beta_yaw)
    cd_eff = effective_drag_coefficient(alpha_total, mach, params)
    drag_body = [-q_dyn * params.area_ref * cd_eff, 0.0, 0.0]
    side_force = -0.35 * q_dyn * params.area_ref * params.cy_beta * beta_yaw
    lift_force = -q_dyn * params.area_ref * params.cl_alpha * alpha_pitch
    aero_force_body = [drag_body[0], side_force, lift_force]
    total_force_body = [
        thrust_body[0] + aero_force_body[0],
        thrust_body[1] + aero_force_body[1],
        thrust_body[2] + aero_force_body[2],
    ]
    total_force_inertial = _mat_vec_mul(rotation_bi, total_force_body)

    ax = total_force_inertial[0] / state.mass
    ay = total_force_inertial[1] / state.mass
    az = total_force_inertial[2] / state.mass - g

    aero_roll_moment = -q_dyn * params.area_ref * params.body_length * (
        params.roll_stability * (wrap_angle(state.roll - roll_ref)) + 0.25 * beta_yaw
    )
    aero_pitch_moment = -q_dyn * params.area_ref * params.body_length * params.static_stability * alpha_pitch
    aero_yaw_moment = -0.45 * q_dyn * params.area_ref * params.body_length * params.static_stability * beta_yaw

    roll_acc = _safe_value((aero_roll_moment - params.roll_damping * state.p) / params.ixx, 8.0)
    pitch_acc = _safe_value(
        (thrust * params.tvc_arm * sin(gimbal_pitch) + aero_pitch_moment - params.pitch_damping * state.q) / params.iyy,
        12.0,
    )
    yaw_acc = _safe_value(
        (thrust * params.tvc_arm * sin(gimbal_yaw) + aero_yaw_moment - params.yaw_damping * state.r) / params.izz,
        12.0,
    )

    cos_pitch = max(cos(state.pitch), 1e-4)
    roll_dot = _safe_value(state.p + tan(state.pitch) * (state.q * sin(state.roll) + state.r * cos(state.roll)), 6.0)
    pitch_dot = _safe_value(state.q * cos(state.roll) - state.r * sin(state.roll), 6.0)
    yaw_dot = _safe_value((state.q * sin(state.roll) + state.r * cos(state.roll)) / cos_pitch, 6.0)

    next_state = RocketState6DOF(
        t=state.t + dt,
        x=state.x + state.vx * dt,
        y=state.y + state.vy * dt,
        z=max(0.0, state.z + state.vz * dt),
        vx=_safe_value(state.vx + ax * dt, 1500.0),
        vy=_safe_value(state.vy + ay * dt, 1500.0),
        vz=_safe_value(state.vz + az * dt, 1500.0),
        roll=wrap_angle(state.roll + roll_dot * dt),
        pitch=clamp(wrap_angle(state.pitch + pitch_dot * dt), -89.0 * pi / 180.0, 89.0 * pi / 180.0),
        yaw=wrap_angle(state.yaw + yaw_dot * dt),
        p=_safe_value(state.p + roll_acc * dt, 8.0),
        q=_safe_value(state.q + pitch_acc * dt, 8.0),
        r=_safe_value(state.r + yaw_acc * dt, 8.0),
        mass=max(params.dry_mass, state.mass - mdot * dt),
    )

    telemetry = {
        "speed": speed,
        "airspeed": airspeed,
        "mach": mach,
        "q_dyn": q_dyn,
        "rho": atm["density"],
        "cd_eff": cd_eff,
        "thrust": thrust,
        "wind_x": wind_x,
        "wind_y": wind_y,
        "roll_ref": roll_ref,
        "pitch_ref": pitch_ref,
        "yaw_ref": yaw_ref,
        "alpha_pitch_deg": degrees(alpha_pitch),
        "beta_yaw_deg": degrees(beta_yaw),
        "gimbal_pitch_deg": degrees(gimbal_pitch),
        "gimbal_yaw_deg": degrees(gimbal_yaw),
    }
    return next_state, telemetry


def simulate_rocket_6dof(params: RocketParameters) -> list[dict[str, float]]:
    pitch0 = params.theta_initial_deg * pi / 180.0
    yaw0 = 0.0
    roll0 = 0.0
    initial_speed = 25.0
    state = RocketState6DOF(
        t=0.0,
        x=0.0,
        y=0.0,
        z=0.0,
        vx=initial_speed * cos(pitch0),
        vy=0.0,
        vz=initial_speed * sin(pitch0),
        roll=roll0,
        pitch=pitch0,
        yaw=yaw0,
        p=0.0,
        q=0.0,
        r=0.0,
        mass=params.dry_mass + params.propellant_mass,
    )
    pitch_pid, yaw_pid = build_attitude_pid(params)

    atm0 = atmosphere_state(0.0)
    history: list[dict[str, float]] = [
        _sample_6dof(
            state,
            {
                "speed": initial_speed,
                "airspeed": initial_speed,
                "mach": initial_speed / max(atm0["sound_speed"], 1.0),
                "q_dyn": 0.5 * atm0["density"] * initial_speed * initial_speed,
                "rho": atm0["density"],
                "cd_eff": params.cd0,
                "thrust": params.thrust * thrust_profile_factor(0.0, params),
                "wind_x": params.wind_ref_m_s,
                "wind_y": lateral_wind_velocity(0.0, params),
                "roll_ref": 0.0,
                "pitch_ref": pitch0,
                "yaw_ref": yaw0,
                "alpha_pitch_deg": 0.0,
                "beta_yaw_deg": 0.0,
                "gimbal_pitch_deg": 0.0,
                "gimbal_yaw_deg": 0.0,
            },
        )
    ]

    while state.t <= params.t_final:
        state, telemetry = _step_6dof(state, params, pitch_pid, yaw_pid)
        current_sample = _sample_6dof(state, telemetry)
        history.append(current_sample)
        if state.t > params.burn_time and state.z <= 0.0:
            history[-1] = _interpolate_ground_impact(history[-2], current_sample) if len(history) >= 2 else current_sample
            break

    return history


def summarize_6dof_metrics(history: list[dict[str, float]], params: RocketParameters) -> dict[str, float]:
    final = history[-1]
    peak_altitude = max(sample["z"] for sample in history)
    max_lateral = max(abs(sample["y"]) for sample in history)
    return {
        "peak_altitude": peak_altitude,
        "downrange": final["x"],
        "crossrange": final["y"],
        "impact_error": sqrt((final["x"] - params.impact_target) ** 2 + (final["y"] - params.impact_target_y) ** 2),
        "flight_time": final["t"],
        "final_speed": final["speed"],
        "max_lateral": max_lateral,
    }
