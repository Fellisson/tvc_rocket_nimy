from __future__ import annotations

from dataclasses import dataclass
from math import atan2, degrees

from .models import RocketParameters
from .simulation import atmosphere_state


@dataclass
class RocketState3D:
    t: float
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float


def lateral_wind_velocity(altitude: float, params: RocketParameters) -> float:
    state = atmosphere_state(altitude)
    density_ratio = state["density"] / max(atmosphere_state(0.0)["density"], 1e-6)
    return params.crosswind_ref_m_s * density_ratio**0.35


def simulate_rocket_3d_preview(
    history_2d: list[dict[str, float]],
    params: RocketParameters,
) -> list[dict[str, float]]:
    if not history_2d:
        return []

    preview_history: list[dict[str, float]] = []
    y = 0.0
    vy = 0.0

    for index, sample in enumerate(history_2d):
        if index == 0:
            dt = 0.0
        else:
            dt = max(sample["t"] - history_2d[index - 1]["t"], 0.0)

        crosswind = lateral_wind_velocity(sample["z"], params)
        # Very lightweight 3D preview: lateral acceleration is driven by
        # crosswind tracking with atmospheric damping, while keeping the
        # validated 2D longitudinal solution intact.
        lateral_acc = 0.22 * (crosswind - vy)
        vy += lateral_acc * dt
        y += vy * dt

        heading_deg = degrees(0.0 if abs(sample["vx"]) < 1e-9 and abs(vy) < 1e-9 else atan2(vy, sample["vx"]))
        preview_history.append(
            {
                "t": sample["t"],
                "x": sample["x"],
                "y": y,
                "z": sample["z"],
                "vx": sample["vx"],
                "vy": vy,
                "vz": sample["vz"],
                "crosswind_y": crosswind,
                "heading_deg": heading_deg,
                "speed_3d": (sample["vx"] ** 2 + vy**2 + sample["vz"] ** 2) ** 0.5,
            }
        )

    return preview_history
