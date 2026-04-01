from __future__ import annotations

from pathlib import Path

from .geodesic_guidance import GeodesicGuidancePlan
from .models import RocketParameters
from .output import prepare_matplotlib, present_figure, save_figure
from .simulation_earth_guided import summarize_earth_guided_metrics
from .targeting import MissionTargetProfile


def summarize_earth_guided(
    history: list[dict[str, float | str]],
    mission: MissionTargetProfile,
    plan: GeodesicGuidancePlan,
) -> None:
    if not history:
        return
    metrics = summarize_earth_guided_metrics(history, plan, mission)
    print("Simulation Earth Guided")
    print(f"Distance cible sol    : {metrics['target_surface_distance_m'] / 1000.0:8.1f} km")
    print(f"Distance parcourue    : {metrics['ground_distance_m'] / 1000.0:8.1f} km")
    print(f"Erreur horizontale    : {metrics['horizontal_error_m'] / 1000.0:8.1f} km")
    print(f"Erreur 3D             : {metrics['slant_error_m'] / 1000.0:8.1f} km")
    print(f"Altitude max          : {metrics['peak_altitude_m'] / 1000.0:8.2f} km")
    print(f"Azimuth lancement     : {metrics['azimuth_deg']:8.2f} deg")
    print(f"Temps de vol          : {metrics['flight_time']:8.2f} s")
    print(f"Vitesse finale        : {metrics['final_speed_m_s']:8.1f} m/s")


def maybe_plot_earth_guided(
    history: list[dict[str, float | str]],
    params: RocketParameters,
    mission: MissionTargetProfile,
    plan: GeodesicGuidancePlan,
    output_dir: Path,
) -> None:
    if not history:
        return
    try:
        plt, show_plots = prepare_matplotlib(params.show_plots)
    except ImportError:
        print("matplotlib non disponible : graphes earth-guided ignores.")
        return

    east = [float(sample["east_m"]) / 1000.0 for sample in history]
    north = [float(sample["north_m"]) / 1000.0 for sample in history]
    up = [float(sample["up_m"]) / 1000.0 for sample in history]
    t = [float(sample["t"]) for sample in history]
    speed = [float(sample["speed_m_s"]) for sample in history]
    desired_pitch = [float(sample["desired_pitch_deg"]) for sample in history]
    pitch = [float(sample["pitch_deg"]) for sample in history]

    fig = plt.figure(figsize=(12, 8))
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(east, north, linewidth=2.0, label="trajectoire")
    ax1.scatter([plan.enu_east_m / 1000.0], [plan.enu_north_m / 1000.0], color="tab:red", label="cible")
    ax1.set_title("Trace sol ENU")
    ax1.set_xlabel("East (km)")
    ax1.set_ylabel("North (km)")
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(t, up, linewidth=2.0)
    ax2.axhline(mission.altitude_delta_m / 1000.0, color="tab:red", linestyle="--", linewidth=1.5)
    ax2.set_title("Altitude locale")
    ax2.set_xlabel("Temps (s)")
    ax2.set_ylabel("Up (km)")
    ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(t, speed, linewidth=2.0)
    ax3.set_title("Vitesse")
    ax3.set_xlabel("Temps (s)")
    ax3.set_ylabel("m/s")
    ax3.grid(True, alpha=0.3)

    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(t, pitch, linewidth=2.0, label="pitch")
    ax4.plot(t, desired_pitch, "--", linewidth=1.8, label="pitch cible")
    ax4.set_title("Profil de tangage")
    ax4.set_xlabel("Temps (s)")
    ax4.set_ylabel("deg")
    ax4.grid(True, alpha=0.3)
    ax4.legend()

    fig.tight_layout()
    if params.save_plots:
        save_figure(fig, output_dir / "tvc_rocket_earth_guided_plots.png")
    present_figure(plt, fig, show_plots, params.plot_display_seconds)
