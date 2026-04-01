from __future__ import annotations

from dataclasses import replace
from math import degrees
from pathlib import Path

from .models import RocketParameters, TuningResult
from .output import prepare_matplotlib, present_figure, save_figure
from .simulation import build_pid, evaluate_history, simulate_rocket


def maybe_plot_tuning_comparison(
    base_params: RocketParameters,
    tuning_results: list[TuningResult],
    output_dir: Path,
    top_n: int = 5,
) -> None:
    try:
        plt, show_plots = prepare_matplotlib(base_params.show_plots)
    except ImportError:
        print("matplotlib non disponible : comparaison tuning ignoree.")
        return

    top_results = tuning_results[:top_n]
    if not top_results:
        return

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))

    for index, result in enumerate(top_results, start=1):
        params = replace(
            base_params,
            guidance_gain=result.gains.guidance_gain,
            guidance_altitude_gain=result.gains.guidance_altitude_gain,
            gamma_final_deg=result.gains.gamma_final_deg,
        )
        history = simulate_rocket(params, build_pid(params, result.gains))
        x = [sample["x"] for sample in history]
        z = [sample["z"] for sample in history]
        t = [sample["t"] for sample in history]
        alpha = [degrees(sample["alpha_air"]) for sample in history]
        label = (
            f"#{index} "
            f"S={result.score:.0f} "
            f"sat={result.metrics['saturation_ratio'] * 100.0:.1f}%"
        )

        axes[0].plot(x, z, linewidth=2.0, label=label)
        axes[1].plot(t, alpha, linewidth=2.0, label=label)

    axes[0].set_title("Top 5 trajectoires")
    axes[0].set_xlabel("Distance horizontale (m)")
    axes[0].set_ylabel("Altitude (m)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)

    axes[1].set_title("Top 5 angles d'attaque")
    axes[1].set_xlabel("Temps (s)")
    axes[1].set_ylabel("Alpha (deg)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(fontsize=8)

    fig.tight_layout()
    if base_params.save_plots:
        save_figure(fig, output_dir / "tvc_rocket_top5_comparison.png")
    present_figure(plt, fig, show_plots, base_params.plot_display_seconds)


def summarize(history: list[dict[str, float]], params: RocketParameters) -> None:
    metrics = evaluate_history(history, params)
    final = history[-1]
    max_wind = max(abs(sample["wind_x"]) for sample in history)

    print("Simulation TVC + PID terminee")
    print(f"Apogee               : {metrics['peak_altitude']:8.1f} m")
    print(f"Portee horizontale   : {metrics['downrange']:8.1f} m")
    print(f"Cible d'impact       : {params.impact_target:8.1f} m")
    print(f"Erreur d'impact      : {abs(metrics['downrange'] - params.impact_target):8.1f} m")
    print(f"Temps de vol         : {metrics['flight_time']:8.2f} s")
    print(f"Impact au sol        : {'oui' if metrics['landed'] >= 0.5 else 'non'}")
    print(f"Vitesse finale       : {final['speed']:8.1f} m/s")
    print(f"Angle d'attaque moyen: {metrics['mean_alpha_deg']:8.3f} deg")
    print(f"Alpha final          : {degrees(final['alpha_air']):8.3f} deg")
    print(f"Braquage final TVC   : {degrees(final['gimbal']):8.3f} deg")
    print(f"Braquage moyen TVC   : {metrics['mean_gimbal_deg']:8.3f} deg")
    print(f"Braquage max TVC     : {metrics['max_gimbal_deg']:8.3f} deg")
    print(f"Taux de saturation   : {metrics['saturation_ratio'] * 100.0:8.3f} %")
    print(f"Erreur gamma moyenne : {metrics['mean_gamma_error_deg']:8.3f} deg")
    print(f"Vent horizontal max  : {max_wind:8.3f} m/s")


def maybe_plot(history: list[dict[str, float]], params: RocketParameters, output_dir: Path) -> None:
    try:
        plt, show_plots = prepare_matplotlib(params.show_plots)
    except ImportError:
        print("matplotlib non disponible : graphiques ignores.")
        return

    t = [sample["t"] for sample in history]
    x = [sample["x"] for sample in history]
    z = [sample["z"] for sample in history]
    alpha = [degrees(sample["alpha_air"]) for sample in history]
    alpha_cmd = [degrees(sample["alpha_cmd"]) for sample in history]
    alpha_limit = [degrees(sample["alpha_limit"]) for sample in history]
    gamma = [degrees(sample["gamma"]) for sample in history]
    gamma_ref = [degrees(sample["gamma_ref"]) for sample in history]
    gimbal = [degrees(sample["gimbal"]) for sample in history]
    gimbal_cmd = [degrees(sample["gimbal_cmd"]) for sample in history]
    wind_x = [sample["wind_x"] for sample in history]

    fig, axes = plt.subplots(2, 2, figsize=(11, 8))

    axes[0, 0].plot(x, z, color="tab:blue", linewidth=2.0)
    axes[0, 0].set_title("Trajectoire")
    axes[0, 0].set_xlabel("Distance horizontale (m)")
    axes[0, 0].set_ylabel("Altitude (m)")
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(t, alpha, label="alpha reel", linewidth=2.0)
    axes[0, 1].plot(t, alpha_cmd, "--", label="alpha consigne", linewidth=2.0)
    axes[0, 1].plot(t, alpha_limit, ":", label="alpha max planifie", linewidth=1.6)
    axes[0, 1].set_title("Controle de l'angle d'attaque")
    axes[0, 1].set_xlabel("Temps (s)")
    axes[0, 1].set_ylabel("Angle (deg)")
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].plot(t, gamma, label="gamma reel", linewidth=2.0)
    axes[1, 0].plot(t, gamma_ref, "--", label="gamma reference", linewidth=2.0)
    axes[1, 0].set_title("Loi de guidage")
    axes[1, 0].set_xlabel("Temps (s)")
    axes[1, 0].set_ylabel("Angle de trajectoire (deg)")
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].plot(t, gimbal_cmd, "--", label="commande PID", linewidth=1.8)
    axes[1, 1].plot(t, gimbal, color="tab:red", label="actuateur TVC", linewidth=2.0)
    axes[1, 1].set_title("Commande TVC")
    axes[1, 1].set_xlabel("Temps (s)")
    axes[1, 1].set_ylabel("Braquage tuyere (deg)")
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    fig.tight_layout()
    if params.save_plots:
        save_figure(fig, output_dir / "tvc_rocket_main_plots.png")
    present_figure(plt, fig, show_plots, params.plot_display_seconds)

    fig_wind = plt.figure(figsize=(8, 3.5))
    plt.plot(t, wind_x, color="tab:green", linewidth=2.0)
    plt.title("Profil de vent horizontal")
    plt.xlabel("Temps (s)")
    plt.ylabel("Vent (m/s)")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    if params.save_plots:
        save_figure(fig_wind, output_dir / "tvc_rocket_wind_profile.png")
    present_figure(plt, fig_wind, show_plots, params.plot_display_seconds)


def summarize_3d_preview(history_3d: list[dict[str, float]]) -> None:
    if not history_3d:
        return
    final = history_3d[-1]
    max_lateral = max(abs(sample["y"]) for sample in history_3d)
    max_crosswind = max(abs(sample["crosswind_y"]) for sample in history_3d)
    print("Preview 3D")
    print(f"Deviation laterale fin : {final['y']:8.2f} m")
    print(f"Deviation laterale max : {max_lateral:8.2f} m")
    print(f"Vent lateral max      : {max_crosswind:8.2f} m/s")
    print(f"Cap final             : {final['heading_deg']:8.2f} deg")


def maybe_plot_3d_preview(
    history_3d: list[dict[str, float]],
    params: RocketParameters,
    output_dir: Path,
) -> None:
    if not history_3d:
        return

    try:
        plt, show_plots = prepare_matplotlib(params.show_plots)
    except ImportError:
        print("matplotlib non disponible : preview 3D ignoree.")
        return

    x = [sample["x"] for sample in history_3d]
    y = [sample["y"] for sample in history_3d]
    z = [sample["z"] for sample in history_3d]
    t = [sample["t"] for sample in history_3d]
    crosswind = [sample["crosswind_y"] for sample in history_3d]

    fig = plt.figure(figsize=(12, 5))
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax3d.plot(x, y, z, color="tab:orange", linewidth=2.0)
    ax3d.set_title("Trajectoire 3D preview")
    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(t, y, label="deviation laterale", linewidth=2.0)
    ax2.plot(t, crosswind, "--", label="vent lateral", linewidth=1.8)
    ax2.set_title("Effet lateral")
    ax2.set_xlabel("Temps (s)")
    ax2.set_ylabel("Metres / m/s")
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    fig.tight_layout()
    if params.save_plots:
        save_figure(fig, output_dir / "tvc_rocket_3d_preview.png")
    present_figure(plt, fig, show_plots, params.plot_display_seconds)


def summarize_6dof(history: list[dict[str, float]]) -> None:
    if not history:
        return
    final = history[-1]
    max_lateral = max(abs(sample["y"]) for sample in history)
    print("Simulation 6DOF")
    print(f"Portee horizontale   : {final['x']:8.1f} m")
    print(f"Deviation laterale   : {final['y']:8.1f} m")
    print(f"Deviation max        : {max_lateral:8.1f} m")
    print(f"Altitude max         : {max(sample['z'] for sample in history):8.1f} m")
    print(f"Vitesse finale       : {final['speed']:8.1f} m/s")
    print(f"Roll final           : {degrees(final['roll']):8.2f} deg")
    print(f"Pitch final          : {degrees(final['pitch']):8.2f} deg")
    print(f"Yaw final            : {degrees(final['yaw']):8.2f} deg")


def maybe_plot_6dof(history: list[dict[str, float]], params: RocketParameters, output_dir: Path) -> None:
    if not history:
        return
    try:
        plt, show_plots = prepare_matplotlib(params.show_plots)
    except ImportError:
        print("matplotlib non disponible : graphes 6DOF ignores.")
        return

    t = [sample["t"] for sample in history]
    x = [sample["x"] for sample in history]
    y = [sample["y"] for sample in history]
    z = [sample["z"] for sample in history]
    roll = [degrees(sample["roll"]) for sample in history]
    pitch = [degrees(sample["pitch"]) for sample in history]
    yaw = [degrees(sample["yaw"]) for sample in history]
    gimbal_pitch = [sample["gimbal_pitch_deg"] for sample in history]
    gimbal_yaw = [sample["gimbal_yaw_deg"] for sample in history]

    fig = plt.figure(figsize=(12, 8))
    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    ax3d.plot(x, y, z, color="tab:purple", linewidth=2.0)
    ax3d.set_title("Trajectoire 6DOF")
    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(t, roll, label="roll", linewidth=1.8)
    ax2.plot(t, pitch, label="pitch", linewidth=2.0)
    ax2.plot(t, yaw, label="yaw", linewidth=2.0)
    ax2.set_title("Attitude")
    ax2.set_xlabel("Temps (s)")
    ax2.set_ylabel("Angle (deg)")
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(t, y, label="crossrange", linewidth=2.0)
    ax3.set_title("Deviation laterale")
    ax3.set_xlabel("Temps (s)")
    ax3.set_ylabel("Y (m)")
    ax3.grid(True, alpha=0.3)

    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(t, gimbal_pitch, label="gimbal pitch", linewidth=1.8)
    ax4.plot(t, gimbal_yaw, label="gimbal yaw", linewidth=1.8)
    ax4.set_title("Commande TVC 2 axes")
    ax4.set_xlabel("Temps (s)")
    ax4.set_ylabel("Angle (deg)")
    ax4.grid(True, alpha=0.3)
    ax4.legend()

    fig.tight_layout()
    if params.save_plots:
        save_figure(fig, output_dir / "tvc_rocket_6dof_plots.png")
    present_figure(plt, fig, show_plots, params.plot_display_seconds)
