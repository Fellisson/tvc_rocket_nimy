from __future__ import annotations

import json
import math
import urllib.parse
import urllib.request
from dataclasses import dataclass

from .models import RocketParameters

EARTH_RADIUS_M = 6_371_000.0
STANDARD_GRAVITY_M_S2 = 9.80665


@dataclass(frozen=True)
class LaunchSite:
    name: str
    latitude_deg: float
    longitude_deg: float
    altitude_m: float


@dataclass(frozen=True)
class GeoTarget:
    city: str
    country: str
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    source: str


@dataclass(frozen=True)
class MissionTargetProfile:
    launch_site: LaunchSite
    target: GeoTarget
    surface_distance_m: float
    chord_distance_m: float
    initial_bearing_deg: float
    north_offset_m: float
    east_offset_m: float
    altitude_delta_m: float
    initial_mass_kg: float
    dry_mass_kg: float
    effective_exhaust_velocity_m_s: float
    ideal_delta_v_m_s: float
    initial_thrust_to_weight: float
    estimated_burnout_speed_m_s: float
    minimum_time_at_ideal_delta_v_s: float
    scale_ratio_vs_current_impact_target: float
    feasibility_note: str


KINSHASA_LAUNCH_SITE = LaunchSite(
    name="Kinshasa Launch Site",
    latitude_deg=-4.4419,
    longitude_deg=15.2663,
    altitude_m=280.0,
)

_OFFLINE_CITY_CATALOG: dict[str, GeoTarget] = {
    "kinshasa": GeoTarget("Kinshasa", "DR Congo", -4.4419, 15.2663, 280.0, "offline_catalog"),
    "brazzaville": GeoTarget("Brazzaville", "Republic of the Congo", -4.2634, 15.2429, 317.0, "offline_catalog"),
    "lubumbashi": GeoTarget("Lubumbashi", "DR Congo", -11.6647, 27.4794, 1225.0, "offline_catalog"),
    "paris": GeoTarget("Paris", "France", 48.8566, 2.3522, 35.0, "offline_catalog"),
    "london": GeoTarget("London", "United Kingdom", 51.5074, -0.1278, 11.0, "offline_catalog"),
    "brussels": GeoTarget("Brussels", "Belgium", 50.8503, 4.3517, 13.0, "offline_catalog"),
    "new york": GeoTarget("New York", "United States", 40.7128, -74.0060, 10.0, "offline_catalog"),
    "los angeles": GeoTarget("Los Angeles", "United States", 34.0522, -118.2437, 71.0, "offline_catalog"),
    "mexico city": GeoTarget("Mexico City", "Mexico", 19.4326, -99.1332, 2240.0, "offline_catalog"),
    "sao paulo": GeoTarget("Sao Paulo", "Brazil", -23.5558, -46.6396, 760.0, "offline_catalog"),
    "lagos": GeoTarget("Lagos", "Nigeria", 6.5244, 3.3792, 41.0, "offline_catalog"),
    "cairo": GeoTarget("Cairo", "Egypt", 30.0444, 31.2357, 23.0, "offline_catalog"),
    "nairobi": GeoTarget("Nairobi", "Kenya", -1.2921, 36.8219, 1795.0, "offline_catalog"),
    "johannesburg": GeoTarget("Johannesburg", "South Africa", -26.2041, 28.0473, 1753.0, "offline_catalog"),
    "cape town": GeoTarget("Cape Town", "South Africa", -33.9249, 18.4241, 25.0, "offline_catalog"),
    "dubai": GeoTarget("Dubai", "United Arab Emirates", 25.2048, 55.2708, 16.0, "offline_catalog"),
    "mumbai": GeoTarget("Mumbai", "India", 19.0760, 72.8777, 14.0, "offline_catalog"),
    "singapore": GeoTarget("Singapore", "Singapore", 1.3521, 103.8198, 15.0, "offline_catalog"),
    "beijing": GeoTarget("Beijing", "China", 39.9042, 116.4074, 44.0, "offline_catalog"),
    "tokyo": GeoTarget("Tokyo", "Japan", 35.6762, 139.6503, 40.0, "offline_catalog"),
    "sydney": GeoTarget("Sydney", "Australia", -33.8688, 151.2093, 58.0, "offline_catalog"),
    "moscow": GeoTarget("Moscow", "Russia", 55.7558, 37.6173, 156.0, "offline_catalog"),
}


def _normalize_city_name(name: str) -> str:
    return " ".join(name.strip().lower().split())


def _bearing_and_distance(
    latitude_1_deg: float,
    longitude_1_deg: float,
    latitude_2_deg: float,
    longitude_2_deg: float,
) -> tuple[float, float, float, float, float]:
    lat1 = math.radians(latitude_1_deg)
    lon1 = math.radians(longitude_1_deg)
    lat2 = math.radians(latitude_2_deg)
    lon2 = math.radians(longitude_2_deg)
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    hav = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    central_angle = 2.0 * math.atan2(math.sqrt(hav), math.sqrt(max(1.0 - hav, 0.0)))
    surface_distance_m = EARTH_RADIUS_M * central_angle
    chord_distance_m = 2.0 * EARTH_RADIUS_M * math.sin(central_angle / 2.0)

    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    initial_bearing_deg = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
    north_offset_m = surface_distance_m * math.cos(math.radians(initial_bearing_deg))
    east_offset_m = surface_distance_m * math.sin(math.radians(initial_bearing_deg))
    return surface_distance_m, chord_distance_m, initial_bearing_deg, north_offset_m, east_offset_m


def _fetch_json(url: str) -> list[dict[str, object]] | dict[str, object]:
    request = urllib.request.Request(
        url,
        headers={"User-Agent": "tvc-rocket-nimy/1.0 (targeting module)"},
    )
    with urllib.request.urlopen(request, timeout=5.0) as response:
        return json.loads(response.read().decode("utf-8"))


def _lookup_city_online(city_name: str, country: str | None = None) -> GeoTarget | None:
    query = city_name if not country else f"{city_name}, {country}"
    params = urllib.parse.urlencode({"q": query, "format": "jsonv2", "limit": 1})
    payload = _fetch_json(f"https://nominatim.openstreetmap.org/search?{params}")
    if not isinstance(payload, list) or not payload:
        return None

    entry = payload[0]
    latitude_deg = float(entry["lat"])
    longitude_deg = float(entry["lon"])
    display_name = str(entry.get("display_name", query))
    city = display_name.split(",")[0].strip() or city_name
    country_name = country or display_name.split(",")[-1].strip()
    altitude_m = _lookup_elevation_online(latitude_deg, longitude_deg)
    return GeoTarget(city, country_name, latitude_deg, longitude_deg, altitude_m, "nominatim")


def _lookup_elevation_online(latitude_deg: float, longitude_deg: float) -> float:
    params = urllib.parse.urlencode({"locations": f"{latitude_deg:.6f},{longitude_deg:.6f}"})
    payload = _fetch_json(f"https://api.open-elevation.com/api/v1/lookup?{params}")
    if not isinstance(payload, dict):
        return 0.0
    results = payload.get("results", [])
    if not isinstance(results, list) or not results:
        return 0.0
    elevation = results[0].get("elevation", 0.0)
    return float(elevation) if isinstance(elevation, (int, float)) else 0.0


def resolve_city_target(
    city_name: str,
    country: str | None = None,
    allow_online: bool = True,
) -> GeoTarget:
    normalized_name = _normalize_city_name(city_name)
    if normalized_name in _OFFLINE_CITY_CATALOG:
        target = _OFFLINE_CITY_CATALOG[normalized_name]
        if country is None or country.strip().lower() in target.country.lower():
            return target

    if allow_online:
        try:
            online_target = _lookup_city_online(city_name, country)
        except Exception:
            online_target = None
        if online_target is not None:
            return online_target

    available = ", ".join(sorted(_OFFLINE_CITY_CATALOG)[:8])
    raise ValueError(
        "Ville introuvable en mode hors ligne. Essayez avec Internet actif, ou une ville du catalogue local comme "
        f"{available}."
    )


def build_mission_target_profile(
    city_name: str,
    params: RocketParameters,
    country: str | None = None,
    allow_online: bool = True,
    launch_site: LaunchSite = KINSHASA_LAUNCH_SITE,
) -> MissionTargetProfile:
    target = resolve_city_target(city_name, country=country, allow_online=allow_online)
    surface_distance_m, chord_distance_m, initial_bearing_deg, north_offset_m, east_offset_m = _bearing_and_distance(
        launch_site.latitude_deg,
        launch_site.longitude_deg,
        target.latitude_deg,
        target.longitude_deg,
    )

    initial_mass_kg = params.dry_mass + params.propellant_mass
    effective_exhaust_velocity_m_s = params.thrust / max(params.mass_flow, 1e-9)
    ideal_delta_v_m_s = effective_exhaust_velocity_m_s * math.log(initial_mass_kg / max(params.dry_mass, 1e-9))
    average_mass_kg = 0.5 * (initial_mass_kg + params.dry_mass)
    estimated_burnout_speed_m_s = max(
        (params.thrust / max(average_mass_kg, 1e-9) - STANDARD_GRAVITY_M_S2) * params.burn_time,
        0.0,
    )
    initial_thrust_to_weight = params.thrust / max(initial_mass_kg * STANDARD_GRAVITY_M_S2, 1e-9)
    minimum_time_at_ideal_delta_v_s = surface_distance_m / max(ideal_delta_v_m_s, 1.0)
    scale_ratio = surface_distance_m / max(params.impact_target, 1.0)
    if scale_ratio <= 10.0:
        feasibility_note = "cible du meme ordre de grandeur que le demonstrateur actuel"
    elif scale_ratio <= 100.0:
        feasibility_note = "cible exigeant une forte montee en echelle du vehicule et du guidage"
    else:
        feasibility_note = "cible mondiale tres au-dela du demonstrateur actuel ; utile pour etudes de guidage seulement"

    return MissionTargetProfile(
        launch_site=launch_site,
        target=target,
        surface_distance_m=surface_distance_m,
        chord_distance_m=chord_distance_m,
        initial_bearing_deg=initial_bearing_deg,
        north_offset_m=north_offset_m,
        east_offset_m=east_offset_m,
        altitude_delta_m=target.altitude_m - launch_site.altitude_m,
        initial_mass_kg=initial_mass_kg,
        dry_mass_kg=params.dry_mass,
        effective_exhaust_velocity_m_s=effective_exhaust_velocity_m_s,
        ideal_delta_v_m_s=ideal_delta_v_m_s,
        initial_thrust_to_weight=initial_thrust_to_weight,
        estimated_burnout_speed_m_s=estimated_burnout_speed_m_s,
        minimum_time_at_ideal_delta_v_s=minimum_time_at_ideal_delta_v_s,
        scale_ratio_vs_current_impact_target=scale_ratio,
        feasibility_note=feasibility_note,
    )


def format_mission_target_report(profile: MissionTargetProfile) -> str:
    return "\n".join(
        [
            "Cible geographique",
            f"Ville cible          : {profile.target.city}, {profile.target.country}",
            f"Source geocodage     : {profile.target.source}",
            (
                "Coordonnees cible    : "
                f"lat={profile.target.latitude_deg:.6f} deg "
                f"lon={profile.target.longitude_deg:.6f} deg "
                f"alt={profile.target.altitude_m:.1f} m"
            ),
            (
                "Site de lancement    : "
                f"{profile.launch_site.name} "
                f"(lat={profile.launch_site.latitude_deg:.6f} deg "
                f"lon={profile.launch_site.longitude_deg:.6f} deg "
                f"alt={profile.launch_site.altitude_m:.1f} m)"
            ),
            f"Distance surface     : {profile.surface_distance_m / 1000.0:.1f} km",
            f"Distance corde       : {profile.chord_distance_m / 1000.0:.1f} km",
            f"Azimuth initial      : {profile.initial_bearing_deg:.2f} deg depuis le nord",
            (
                "Projection locale    : "
                f"north={profile.north_offset_m / 1000.0:.1f} km "
                f"east={profile.east_offset_m / 1000.0:.1f} km"
            ),
            f"Delta altitude       : {profile.altitude_delta_m:.1f} m par rapport a Kinshasa",
            f"Masse initiale       : {profile.initial_mass_kg:.2f} kg",
            f"Vitesse d'ejection   : {profile.effective_exhaust_velocity_m_s:.1f} m/s",
            f"Delta-v ideal        : {profile.ideal_delta_v_m_s:.1f} m/s",
            f"Thrust-to-weight     : {profile.initial_thrust_to_weight:.2f}",
            f"Vitesse burnout est. : {profile.estimated_burnout_speed_m_s:.1f} m/s",
            f"Temps mini theorique : {profile.minimum_time_at_ideal_delta_v_s:.1f} s",
            f"Ratio vs cible demo  : x{profile.scale_ratio_vs_current_impact_target:.1f}",
            f"Lecture systeme      : {profile.feasibility_note}",
            (
                "Guidage futur        : "
                f"utiliser azimuth={profile.initial_bearing_deg:.2f} deg, "
                f"distance={profile.surface_distance_m:.1f} m, "
                f"altitude_cible={profile.target.altitude_m:.1f} m"
            ),
        ]
    )
