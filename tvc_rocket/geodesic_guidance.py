from __future__ import annotations

import math
from dataclasses import dataclass

from .models import RocketParameters
from .targeting import EARTH_RADIUS_M, MissionTargetProfile

EARTH_ROTATION_RATE_RAD_S = 7.2921159e-5


@dataclass(frozen=True)
class ECEFPoint:
    x_m: float
    y_m: float
    z_m: float


@dataclass(frozen=True)
class GeodesicGuidancePlan:
    launch_ecef: ECEFPoint
    target_ecef: ECEFPoint
    enu_east_m: float
    enu_north_m: float
    enu_up_m: float
    horizontal_range_m: float
    slant_range_m: float
    launch_azimuth_deg: float
    line_of_sight_elevation_deg: float
    earth_rotation_east_speed_m_s: float
    estimated_required_average_speed_m_s: float
    estimated_ballistic_time_s: float
    local_frame_note: str
    guidance_note: str


def geodetic_to_ecef(latitude_deg: float, longitude_deg: float, altitude_m: float) -> ECEFPoint:
    lat = math.radians(latitude_deg)
    lon = math.radians(longitude_deg)
    radius = EARTH_RADIUS_M + altitude_m
    cos_lat = math.cos(lat)
    return ECEFPoint(
        x_m=radius * cos_lat * math.cos(lon),
        y_m=radius * cos_lat * math.sin(lon),
        z_m=radius * math.sin(lat),
    )


def ecef_to_enu(
    point: ECEFPoint,
    reference: ECEFPoint,
    reference_latitude_deg: float,
    reference_longitude_deg: float,
) -> tuple[float, float, float]:
    lat = math.radians(reference_latitude_deg)
    lon = math.radians(reference_longitude_deg)
    dx = point.x_m - reference.x_m
    dy = point.y_m - reference.y_m
    dz = point.z_m - reference.z_m

    east_m = -math.sin(lon) * dx + math.cos(lon) * dy
    north_m = (
        -math.sin(lat) * math.cos(lon) * dx
        - math.sin(lat) * math.sin(lon) * dy
        + math.cos(lat) * dz
    )
    up_m = math.cos(lat) * math.cos(lon) * dx + math.cos(lat) * math.sin(lon) * dy + math.sin(lat) * dz
    return east_m, north_m, up_m


def build_geodesic_guidance_plan(
    mission_profile: MissionTargetProfile,
    params: RocketParameters,
) -> GeodesicGuidancePlan:
    launch_ecef = geodetic_to_ecef(
        mission_profile.launch_site.latitude_deg,
        mission_profile.launch_site.longitude_deg,
        mission_profile.launch_site.altitude_m,
    )
    target_ecef = geodetic_to_ecef(
        mission_profile.target.latitude_deg,
        mission_profile.target.longitude_deg,
        mission_profile.target.altitude_m,
    )
    east_m, north_m, up_m = ecef_to_enu(
        target_ecef,
        launch_ecef,
        mission_profile.launch_site.latitude_deg,
        mission_profile.launch_site.longitude_deg,
    )
    horizontal_range_m = math.hypot(east_m, north_m)
    slant_range_m = math.sqrt(horizontal_range_m * horizontal_range_m + up_m * up_m)
    launch_azimuth_deg = (math.degrees(math.atan2(east_m, north_m)) + 360.0) % 360.0
    line_of_sight_elevation_deg = math.degrees(math.atan2(up_m, max(horizontal_range_m, 1.0)))

    launch_lat_rad = math.radians(mission_profile.launch_site.latitude_deg)
    earth_rotation_east_speed_m_s = EARTH_ROTATION_RATE_RAD_S * (EARTH_RADIUS_M + mission_profile.launch_site.altitude_m) * math.cos(launch_lat_rad)
    estimated_required_average_speed_m_s = mission_profile.surface_distance_m / max(params.t_final, 1.0)
    estimated_ballistic_time_s = mission_profile.surface_distance_m / max(params.thrust / max(params.dry_mass, 1e-9), 1.0)

    if mission_profile.surface_distance_m <= 50_000.0:
        local_frame_note = "cible compatible avec une approximation ENU locale autour de Kinshasa"
    elif mission_profile.surface_distance_m <= 500_000.0:
        local_frame_note = "cible regionale ; un repere ENU local reste utile mais la courbure Terre devient importante"
    else:
        local_frame_note = "cible globale ; il faut propager la navigation dans un repere Terre plutot qu'en plan local seul"

    if mission_profile.scale_ratio_vs_current_impact_target <= 10.0:
        guidance_note = "le guidage actuel peut servir de base apres rotation vers l'azimuth de lancement"
    elif mission_profile.scale_ratio_vs_current_impact_target <= 100.0:
        guidance_note = "prevoir un guidage en plusieurs phases : boost, croisiere inertielle, acquisition terminale"
    else:
        guidance_note = "viser une architecture complete de navigation Terre, estimation d'etat et guidance energetique longue portee"

    return GeodesicGuidancePlan(
        launch_ecef=launch_ecef,
        target_ecef=target_ecef,
        enu_east_m=east_m,
        enu_north_m=north_m,
        enu_up_m=up_m,
        horizontal_range_m=horizontal_range_m,
        slant_range_m=slant_range_m,
        launch_azimuth_deg=launch_azimuth_deg,
        line_of_sight_elevation_deg=line_of_sight_elevation_deg,
        earth_rotation_east_speed_m_s=earth_rotation_east_speed_m_s,
        estimated_required_average_speed_m_s=estimated_required_average_speed_m_s,
        estimated_ballistic_time_s=estimated_ballistic_time_s,
        local_frame_note=local_frame_note,
        guidance_note=guidance_note,
    )


def format_geodesic_guidance_report(plan: GeodesicGuidancePlan) -> str:
    return "\n".join(
        [
            "Plan de guidage geodesique",
            (
                "Repere ENU cible     : "
                f"east={plan.enu_east_m / 1000.0:.1f} km "
                f"north={plan.enu_north_m / 1000.0:.1f} km "
                f"up={plan.enu_up_m / 1000.0:.1f} km"
            ),
            f"Portee horizontale   : {plan.horizontal_range_m / 1000.0:.1f} km",
            f"Distance visee 3D    : {plan.slant_range_m / 1000.0:.1f} km",
            f"Azimuth lancement    : {plan.launch_azimuth_deg:.2f} deg",
            f"Elevation LOS        : {plan.line_of_sight_elevation_deg:.2f} deg",
            f"Rotation Terre est   : {plan.earth_rotation_east_speed_m_s:.1f} m/s a Kinshasa",
            f"Vitesse moy. requise : {plan.estimated_required_average_speed_m_s:.1f} m/s sur t_final actuel",
            f"Temps balistique est.: {plan.estimated_ballistic_time_s:.1f} s",
            f"Lecture repere       : {plan.local_frame_note}",
            f"Lecture guidage      : {plan.guidance_note}",
            (
                "Integration future   : "
                f"orienter le solveur sur azimuth={plan.launch_azimuth_deg:.2f} deg, "
                "exprimer les erreurs de poursuite en ENU puis convertir vers le repere corps"
            ),
        ]
    )
