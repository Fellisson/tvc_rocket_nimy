from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .geodesic_guidance import GeodesicGuidancePlan
from .mission_design import MissionDesignRecommendation
from .models import RocketParameters
from .targeting import MissionTargetProfile


def presets_dir(project_root: Path) -> Path:
    path = project_root / "presets"
    path.mkdir(parents=True, exist_ok=True)
    return path


def exports_dir(project_root: Path) -> Path:
    path = project_root / "exports"
    path.mkdir(parents=True, exist_ok=True)
    return path


def sanitize_name(name: str) -> str:
    cleaned = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in name.strip())
    return cleaned.strip("_") or "preset"


def build_gui_preset_payload(
    *,
    city: str,
    country: str,
    offline_only: bool,
    short_range: bool,
    regional_earth: bool,
    apply_design: bool,
    favorite: bool,
    tags: list[str],
    params: RocketParameters,
) -> dict[str, object]:
    return {
        "city": city,
        "country": country,
        "offline_only": offline_only,
        "short_range": short_range,
        "regional_earth": regional_earth,
        "apply_design": apply_design,
        "favorite": favorite,
        "tags": tags,
        "params": {
            "thrust": params.thrust,
            "propellant_mass": params.propellant_mass,
            "burn_time": params.burn_time,
            "theta_initial_deg": params.theta_initial_deg,
            "altitude_target": params.altitude_target,
            "t_final": params.t_final,
        },
    }


def save_gui_preset(project_root: Path, name: str, payload: dict[str, object]) -> Path:
    path = presets_dir(project_root) / f"{sanitize_name(name)}.json"
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=True), encoding="utf-8")
    return path


def load_gui_preset(project_root: Path, name: str) -> dict[str, object]:
    path = presets_dir(project_root) / f"{sanitize_name(name)}.json"
    return json.loads(path.read_text(encoding="utf-8"))


def list_gui_presets(project_root: Path) -> list[str]:
    return sorted(path.stem for path in presets_dir(project_root).glob("*.json"))


def delete_gui_preset(project_root: Path, name: str) -> Path:
    path = presets_dir(project_root) / f"{sanitize_name(name)}.json"
    path.unlink()
    return path


def build_mission_export_payload(
    *,
    city: str,
    country: str,
    params: RocketParameters,
    mission: MissionTargetProfile,
    guidance: GeodesicGuidancePlan,
    design: MissionDesignRecommendation,
) -> dict[str, object]:
    return {
        "city": city,
        "country": country,
        "vehicle": {
            "thrust_n": params.thrust,
            "propellant_mass_kg": params.propellant_mass,
            "burn_time_s": params.burn_time,
            "theta_initial_deg": params.theta_initial_deg,
            "altitude_target_m": params.altitude_target,
            "t_final_s": params.t_final,
        },
        "mission_target": {
            "city": mission.target.city,
            "country": mission.target.country,
            "latitude_deg": mission.target.latitude_deg,
            "longitude_deg": mission.target.longitude_deg,
            "altitude_m": mission.target.altitude_m,
            "surface_distance_m": mission.surface_distance_m,
            "initial_bearing_deg": mission.initial_bearing_deg,
            "north_offset_m": mission.north_offset_m,
            "east_offset_m": mission.east_offset_m,
            "altitude_delta_m": mission.altitude_delta_m,
        },
        "guidance_plan": {
            "launch_azimuth_deg": guidance.launch_azimuth_deg,
            "line_of_sight_elevation_deg": guidance.line_of_sight_elevation_deg,
            "enu_east_m": guidance.enu_east_m,
            "enu_north_m": guidance.enu_north_m,
            "enu_up_m": guidance.enu_up_m,
            "horizontal_range_m": guidance.horizontal_range_m,
            "slant_range_m": guidance.slant_range_m,
            "earth_rotation_east_speed_m_s": guidance.earth_rotation_east_speed_m_s,
        },
        "mission_design": {
            "mission_class": design.mission_class,
            "recommended_burn_time_s": design.recommended_burn_time_s,
            "recommended_propellant_mass_kg": design.recommended_propellant_mass_kg,
            "recommended_thrust_n": design.recommended_thrust_n,
            "recommended_initial_pitch_deg": design.recommended_initial_pitch_deg,
            "recommended_altitude_target_m": design.recommended_altitude_target_m,
            "recommended_t_final_s": design.recommended_t_final_s,
            "required_average_speed_m_s": design.required_average_speed_m_s,
            "required_delta_v_m_s": design.required_delta_v_m_s,
            "estimated_mass_ratio": design.estimated_mass_ratio,
            "required_exhaust_velocity_m_s": design.required_exhaust_velocity_m_s,
            "comment": design.comment,
        },
    }


def export_mission_json(project_root: Path, name: str, payload: dict[str, object]) -> Path:
    path = exports_dir(project_root) / f"{sanitize_name(name)}_mission.json"
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=True), encoding="utf-8")
    return path


def load_exported_mission(project_root: Path, name: str) -> dict[str, Any]:
    path = exports_dir(project_root) / f"{sanitize_name(name)}_mission.json"
    return json.loads(path.read_text(encoding="utf-8"))


def list_exported_missions(project_root: Path) -> list[str]:
    suffix = "_mission.json"
    return sorted(
        path.name[: -len(suffix)]
        for path in exports_dir(project_root).glob(f"*{suffix}")
    )


def delete_exported_mission(project_root: Path, name: str) -> Path:
    path = exports_dir(project_root) / f"{sanitize_name(name)}_mission.json"
    path.unlink()
    return path


def history_path(project_root: Path) -> Path:
    return exports_dir(project_root) / "mission_history.jsonl"


def append_mission_history(project_root: Path, entry: dict[str, object]) -> Path:
    path = history_path(project_root)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(entry, ensure_ascii=True) + "\n")
    return path


def read_mission_history(project_root: Path, limit: int = 20) -> list[dict[str, Any]]:
    path = history_path(project_root)
    if not path.exists():
        return []
    lines = path.read_text(encoding="utf-8").splitlines()
    records = [json.loads(line) for line in lines if line.strip()]
    return records[-limit:]


def clear_mission_history(project_root: Path) -> Path:
    path = history_path(project_root)
    if path.exists():
        path.unlink()
    return path


def build_preset_comparison_rows(project_root: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for name in list_gui_presets(project_root):
        payload = load_gui_preset(project_root, name)
        params = payload.get("params", {})
        rows.append(
            {
                "name": name,
                "city": payload.get("city", ""),
                "country": payload.get("country", ""),
                "thrust_n": float(params.get("thrust", 0.0)),
                "propellant_mass_kg": float(params.get("propellant_mass", 0.0)),
                "burn_time_s": float(params.get("burn_time", 0.0)),
                "theta_initial_deg": float(params.get("theta_initial_deg", 0.0)),
                "altitude_target_m": float(params.get("altitude_target", 0.0)),
                "regional_earth": bool(payload.get("regional_earth", False)),
                "apply_design": bool(payload.get("apply_design", False)),
                "favorite": bool(payload.get("favorite", False)),
                "tags": ", ".join(str(tag) for tag in payload.get("tags", [])),
            }
        )
    return rows


def build_export_comparison_rows(project_root: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for name in list_exported_missions(project_root):
        payload = load_exported_mission(project_root, name)
        vehicle = payload.get("vehicle", {})
        mission_target = payload.get("mission_target", {})
        mission_design = payload.get("mission_design", {})
        rows.append(
            {
                "name": name,
                "city": payload.get("city", ""),
                "country": payload.get("country", ""),
                "distance_km": float(mission_target.get("surface_distance_m", 0.0)) / 1000.0,
                "thrust_n": float(vehicle.get("thrust_n", 0.0)),
                "burn_time_s": float(vehicle.get("burn_time_s", 0.0)),
                "mission_class": mission_design.get("mission_class", ""),
                "delta_v_m_s": float(mission_design.get("required_delta_v_m_s", 0.0)),
            }
        )
    return rows
