from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, degrees, pi, sin, sqrt

from .geodesic_guidance import GeodesicGuidancePlan, build_geodesic_guidance_plan
from .models import RocketParameters, clamp
from .simulation import atmosphere_state, effective_drag_coefficient, thrust_profile_factor
from .targeting import MissionTargetProfile


@dataclass
class EarthGuidedState:
    t: float
    east_m: float
    north_m: float
    up_m: float
    ve_m_s: float
    vn_m_s: float
    vu_m_s: float
    mass_kg: float
    pitch_rad: float


def build_regional_earth_guided_params(
    params: RocketParameters,
    mission: MissionTargetProfile,
) -> RocketParameters:
    target_distance_m = mission.surface_distance_m
    if target_distance_m <= 30_000.0:
        thrust = max(params.thrust, 1600.0)
        burn_time = max(params.burn_time, 38.0)
        propellant_mass = max(params.propellant_mass, 24.0)
        theta_initial_deg = min(params.theta_initial_deg, 48.0)
        altitude_target = min(max(params.altitude_target, 750.0), 950.0)
    elif target_distance_m <= 80_000.0:
        thrust = max(params.thrust, 1800.0)
        burn_time = max(params.burn_time, 55.0)
        propellant_mass = max(params.propellant_mass, 28.0)
        theta_initial_deg = min(params.theta_initial_deg, 45.0)
        altitude_target = min(max(params.altitude_target, 900.0), 1800.0)
    else:
        thrust = max(params.thrust, 2200.0)
        burn_time = max(params.burn_time, 70.0)
        propellant_mass = max(params.propellant_mass, 34.0)
        theta_initial_deg = min(params.theta_initial_deg, 40.0)
        altitude_target = min(max(params.altitude_target, 1500.0), 3000.0)

    return RocketParameters(
        **{
            **params.__dict__,
            "thrust": thrust,
            "burn_time": burn_time,
            "propellant_mass": propellant_mass,
            "theta_initial_deg": theta_initial_deg,
            "altitude_target": altitude_target,
            "gamma_final_deg": min(params.gamma_final_deg, 20.0),
            "t_final": max(params.t_final, burn_time + 20.0),
        }
    )


def _sample(
    state: EarthGuidedState,
    plan: GeodesicGuidancePlan,
    mission: MissionTargetProfile,
    telemetry: dict[str, float | str],
) -> dict[str, float | str]:
    horizontal_range = sqrt(state.east_m * state.east_m + state.north_m * state.north_m)
    speed = sqrt(state.ve_m_s * state.ve_m_s + state.vn_m_s * state.vn_m_s + state.vu_m_s * state.vu_m_s)
    return {
        "t": state.t,
        "east_m": state.east_m,
        "north_m": state.north_m,
        "up_m": state.up_m,
        "ve_m_s": state.ve_m_s,
        "vn_m_s": state.vn_m_s,
        "vu_m_s": state.vu_m_s,
        "mass_kg": state.mass_kg,
        "pitch_deg": degrees(state.pitch_rad),
        "ground_distance_m": horizontal_range,
        "speed_m_s": speed,
        "target_east_m": plan.enu_east_m,
        "target_north_m": plan.enu_north_m,
        "target_up_m": plan.enu_up_m,
        "target_surface_distance_m": mission.surface_distance_m,
        **telemetry,
    }


def simulate_earth_guided(
    params: RocketParameters,
    mission: MissionTargetProfile,
    plan: GeodesicGuidancePlan | None = None,
) -> list[dict[str, float | str]]:
    if plan is None:
        plan = build_geodesic_guidance_plan(mission, params)

    azimuth_rad = plan.launch_azimuth_deg * pi / 180.0
    speed0 = 25.0
    pitch0 = min(params.theta_initial_deg, 88.0) * pi / 180.0
    state = EarthGuidedState(
        t=0.0,
        east_m=0.0,
        north_m=0.0,
        up_m=0.0,
        ve_m_s=speed0 * sin(azimuth_rad) * cos(pitch0),
        vn_m_s=speed0 * cos(azimuth_rad) * cos(pitch0),
        vu_m_s=speed0 * sin(pitch0),
        mass_kg=params.dry_mass + params.propellant_mass,
        pitch_rad=pitch0,
    )

    history: list[dict[str, float | str]] = []
    target_ground_m = max(plan.horizontal_range_m, 1.0)
    while state.t <= params.t_final:
        dt = params.dt
        ground_distance = sqrt(state.east_m * state.east_m + state.north_m * state.north_m)
        remaining_ground_m = max(target_ground_m - ground_distance, 0.0)
        is_burning = state.t < params.burn_time and state.mass_kg > params.dry_mass

        if remaining_ground_m > 0.0:
            dir_e = (plan.enu_east_m - state.east_m) / max(remaining_ground_m, 1.0)
            dir_n = (plan.enu_north_m - state.north_m) / max(remaining_ground_m, 1.0)
        else:
            dir_e = sin(azimuth_rad)
            dir_n = cos(azimuth_rad)

        range_fraction = clamp(ground_distance / target_ground_m, 0.0, 1.0)
        desired_up_m = mission.altitude_delta_m + (params.altitude_target - mission.altitude_delta_m) * max(0.0, 1.0 - range_fraction) ** 1.2
        desired_pitch_rad = clamp(
            atan2(desired_up_m - state.up_m, max(remaining_ground_m, 50.0)),
            -25.0 * pi / 180.0,
            85.0 * pi / 180.0,
        )
        if state.t < params.launch_rail_time:
            desired_pitch_rad = max(desired_pitch_rad, state.pitch_rad)

        max_pitch_step = params.max_gimbal_rate_rad_s * dt
        state.pitch_rad = clamp(
            state.pitch_rad + clamp(desired_pitch_rad - state.pitch_rad, -max_pitch_step, max_pitch_step),
            -30.0 * pi / 180.0,
            88.0 * pi / 180.0,
        )

        atm = atmosphere_state(max(state.up_m, 0.0))
        speed = max(sqrt(state.ve_m_s * state.ve_m_s + state.vn_m_s * state.vn_m_s + state.vu_m_s * state.vu_m_s), 1.0)
        q_dyn = 0.5 * atm["density"] * speed * speed
        mach = speed / max(atm["sound_speed"], 1.0)
        thrust = params.thrust * thrust_profile_factor(state.t, params) if is_burning else 0.0
        mdot = params.mass_flow if is_burning else 0.0
        cd_eff = effective_drag_coefficient(0.0, mach, params)
        drag = q_dyn * params.area_ref * cd_eff

        if speed > 1e-9:
            drag_e = drag * state.ve_m_s / speed
            drag_n = drag * state.vn_m_s / speed
            drag_u = drag * state.vu_m_s / speed
        else:
            drag_e = drag_n = drag_u = 0.0

        thrust_horizontal = thrust * cos(state.pitch_rad)
        thrust_up = thrust * sin(state.pitch_rad)
        acc_e = (thrust_horizontal * dir_e - drag_e) / max(state.mass_kg, 1e-9)
        acc_n = (thrust_horizontal * dir_n - drag_n) / max(state.mass_kg, 1e-9)
        acc_u = (thrust_up - drag_u) / max(state.mass_kg, 1e-9) - 9.81

        sample = _sample(
            state,
            plan,
            mission,
            {
                "phase": "burn" if is_burning else "coast",
                "desired_pitch_deg": degrees(desired_pitch_rad),
                "remaining_ground_m": remaining_ground_m,
                "q_dyn": q_dyn,
                "mach": mach,
                "cd_eff": cd_eff,
                "azimuth_deg": plan.launch_azimuth_deg,
                "line_of_sight_elevation_deg": plan.line_of_sight_elevation_deg,
            },
        )
        history.append(sample)

        next_state = EarthGuidedState(
            t=state.t + dt,
            east_m=state.east_m + state.ve_m_s * dt,
            north_m=state.north_m + state.vn_m_s * dt,
            up_m=state.up_m + state.vu_m_s * dt,
            ve_m_s=state.ve_m_s + acc_e * dt,
            vn_m_s=state.vn_m_s + acc_n * dt,
            vu_m_s=state.vu_m_s + acc_u * dt,
            mass_kg=max(params.dry_mass, state.mass_kg - mdot * dt),
            pitch_rad=state.pitch_rad,
        )
        state = next_state
        if state.t > params.burn_time and state.up_m <= mission.altitude_delta_m:
            state.up_m = mission.altitude_delta_m
            break

    final_sample = _sample(
        state,
        plan,
        mission,
        {
            "phase": "impact",
            "desired_pitch_deg": degrees(state.pitch_rad),
            "remaining_ground_m": max(target_ground_m - sqrt(state.east_m * state.east_m + state.north_m * state.north_m), 0.0),
            "q_dyn": 0.0,
            "mach": 0.0,
            "cd_eff": params.cd0,
            "azimuth_deg": plan.launch_azimuth_deg,
            "line_of_sight_elevation_deg": plan.line_of_sight_elevation_deg,
        },
    )
    history.append(final_sample)
    return history


def summarize_earth_guided_metrics(
    history: list[dict[str, float | str]],
    plan: GeodesicGuidancePlan,
    mission: MissionTargetProfile,
) -> dict[str, float]:
    final = history[-1]
    east_error = float(final["target_east_m"]) - float(final["east_m"])
    north_error = float(final["target_north_m"]) - float(final["north_m"])
    up_error = float(final["target_up_m"]) - float(final["up_m"])
    horizontal_error = sqrt(east_error * east_error + north_error * north_error)
    return {
        "flight_time": float(final["t"]),
        "peak_altitude_m": max(float(sample["up_m"]) for sample in history),
        "ground_distance_m": float(final["ground_distance_m"]),
        "horizontal_error_m": horizontal_error,
        "slant_error_m": sqrt(horizontal_error * horizontal_error + up_error * up_error),
        "target_surface_distance_m": mission.surface_distance_m,
        "azimuth_deg": plan.launch_azimuth_deg,
        "final_speed_m_s": float(final["speed_m_s"]),
    }
