from __future__ import annotations

import math
from dataclasses import dataclass

from .geodesic_guidance import GeodesicGuidancePlan
from .models import RocketParameters
from .targeting import MissionTargetProfile

STANDARD_GRAVITY_M_S2 = 9.80665


@dataclass(frozen=True)
class MissionDesignRecommendation:
    mission_class: str
    recommended_burn_time_s: float
    recommended_propellant_mass_kg: float
    recommended_thrust_n: float
    recommended_initial_pitch_deg: float
    recommended_altitude_target_m: float
    recommended_t_final_s: float
    required_average_speed_m_s: float
    required_delta_v_m_s: float
    estimated_mass_ratio: float
    required_exhaust_velocity_m_s: float
    comment: str


def build_mission_design_recommendation(
    params: RocketParameters,
    mission: MissionTargetProfile,
    plan: GeodesicGuidancePlan,
) -> MissionDesignRecommendation:
    surface_distance_m = mission.surface_distance_m
    altitude_scale_m = max(abs(mission.altitude_delta_m), 50.0)
    required_average_speed_m_s = surface_distance_m / max(params.t_final, 1.0)

    if surface_distance_m <= 30_000.0:
        mission_class = "regional_demo"
        recommended_burn_time_s = 38.0
        recommended_propellant_mass_kg = 24.0
        recommended_thrust_n = 1600.0
        recommended_initial_pitch_deg = 48.0
        recommended_altitude_target_m = 750.0
        comment = "configuration adaptee aux cibles regionales proches autour de Kinshasa"
    elif surface_distance_m <= 150_000.0:
        mission_class = "theater_scale"
        recommended_burn_time_s = 75.0
        recommended_propellant_mass_kg = 55.0
        recommended_thrust_n = 4200.0
        recommended_initial_pitch_deg = 42.0
        recommended_altitude_target_m = 6_000.0
        comment = "necessite un vehicule plus energetique et un guidage boost-midcourse-terminal"
    elif surface_distance_m <= 1_000_000.0:
        mission_class = "long_range"
        recommended_burn_time_s = 140.0
        recommended_propellant_mass_kg = 180.0
        recommended_thrust_n = 18_000.0
        recommended_initial_pitch_deg = 36.0
        recommended_altitude_target_m = 45_000.0
        comment = "demande deja une architecture inertielle complete et une montee en altitude importante"
    else:
        mission_class = "global_study"
        recommended_burn_time_s = 220.0
        recommended_propellant_mass_kg = 500.0
        recommended_thrust_n = 55_000.0
        recommended_initial_pitch_deg = 30.0
        recommended_altitude_target_m = 140_000.0
        comment = "hors domaine du demonstrateur ; utile pour etudes systeme, navigation Terre et budget energetique"

    recommended_t_final_s = recommended_burn_time_s + max(surface_distance_m / 700.0, 30.0)
    required_delta_v_m_s = max(
        1.15 * required_average_speed_m_s + 2.2 * math.sqrt(STANDARD_GRAVITY_M_S2 * altitude_scale_m),
        100.0,
    )
    required_exhaust_velocity_m_s = max(2_100.0, params.thrust / max(params.mass_flow, 1e-9))
    estimated_mass_ratio = math.exp(required_delta_v_m_s / required_exhaust_velocity_m_s)

    return MissionDesignRecommendation(
        mission_class=mission_class,
        recommended_burn_time_s=recommended_burn_time_s,
        recommended_propellant_mass_kg=recommended_propellant_mass_kg,
        recommended_thrust_n=recommended_thrust_n,
        recommended_initial_pitch_deg=recommended_initial_pitch_deg,
        recommended_altitude_target_m=recommended_altitude_target_m,
        recommended_t_final_s=recommended_t_final_s,
        required_average_speed_m_s=required_average_speed_m_s,
        required_delta_v_m_s=required_delta_v_m_s,
        estimated_mass_ratio=estimated_mass_ratio,
        required_exhaust_velocity_m_s=required_exhaust_velocity_m_s,
        comment=comment,
    )


def apply_mission_design_recommendation(
    params: RocketParameters,
    recommendation: MissionDesignRecommendation,
) -> RocketParameters:
    return RocketParameters(
        **{
            **params.__dict__,
            "burn_time": max(params.burn_time, recommendation.recommended_burn_time_s),
            "propellant_mass": max(params.propellant_mass, recommendation.recommended_propellant_mass_kg),
            "thrust": max(params.thrust, recommendation.recommended_thrust_n),
            "theta_initial_deg": min(params.theta_initial_deg, recommendation.recommended_initial_pitch_deg),
            "altitude_target": max(params.altitude_target, recommendation.recommended_altitude_target_m),
            "t_final": max(params.t_final, recommendation.recommended_t_final_s),
        }
    )


def format_mission_design_report(recommendation: MissionDesignRecommendation) -> str:
    return "\n".join(
        [
            "Design mission",
            f"Classe mission       : {recommendation.mission_class}",
            f"Vitesse moy. requise : {recommendation.required_average_speed_m_s:.1f} m/s",
            f"Delta-v requis est.  : {recommendation.required_delta_v_m_s:.1f} m/s",
            f"Vitesse ejection ref.: {recommendation.required_exhaust_velocity_m_s:.1f} m/s",
            f"Ratio de masse est.  : {recommendation.estimated_mass_ratio:.2f}",
            f"Poussee recommandee  : {recommendation.recommended_thrust_n:.1f} N",
            f"Burn time recommande : {recommendation.recommended_burn_time_s:.1f} s",
            f"Propellant recommande: {recommendation.recommended_propellant_mass_kg:.1f} kg",
            f"Pitch initial rec.   : {recommendation.recommended_initial_pitch_deg:.1f} deg",
            f"Altitude cible rec.  : {recommendation.recommended_altitude_target_m:.1f} m",
            f"Tfinal recommande    : {recommendation.recommended_t_final_s:.1f} s",
            f"Lecture design       : {recommendation.comment}",
        ]
    )
