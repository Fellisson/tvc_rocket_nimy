from __future__ import annotations

from pathlib import Path

from .models import RocketParameters
from .output import prepare_matplotlib, present_figure, save_figure


def summarize_6dof_guided(history: list[dict[str, float]]) -> None:
    if not history:
        return
    final = history[-1]
    print("Simulation 6DOF Guided")
    print(f"Portee horizontale   : {final['x']:8.1f} m")
    print(f"Deviation laterale   : {final['y']:8.1f} m")
    print(f"Altitude max         : {max(sample['z'] for sample in history):8.1f} m")
    print(f"Erreur yaw max       : {max(abs(s['yaw_error_deg']) for s in history):8.2f} deg")
    print(f"Erreur pitch max     : {max(abs(s['pitch_error_deg']) for s in history):8.2f} deg")
    print(f"Beta max             : {max(abs(s['beta_deg']) for s in history):8.2f} deg")
    print(f"Alpha max            : {max(abs(s['alpha_deg']) for s in history):8.2f} deg")


def maybe_plot_6dof_guided(history: list[dict[str, float]], params: RocketParameters, output_dir: Path) -> None:
    if not history:
        return
    try:
        plt, show_plots = prepare_matplotlib(params.show_plots)
    except ImportError:
        print("matplotlib non disponible : graphes 6DOF guided ignores.")
        return

    t = [s["t"] for s in history]
    x = [s["x"] for s in history]
    y = [s["y"] for s in history]
    z = [s["z"] for s in history]
    yaw_err = [s["yaw_error_deg"] for s in history]
    pitch_err = [s["pitch_error_deg"] for s in history]
    beta = [s["beta_deg"] for s in history]
    alpha = [s["alpha_deg"] for s in history]
    gimbal_pitch = [s["gimbal_pitch_deg"] for s in history]
    gimbal_yaw = [s["gimbal_yaw_deg"] for s in history]

    fig = plt.figure(figsize=(13, 9))
    ax1 = fig.add_subplot(2, 2, 1, projection="3d")
    ax1.plot(x, y, z, color="tab:cyan", linewidth=2.0)
    ax1.set_title("Trajectoire 6DOF guided")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(t, yaw_err, label="yaw error", linewidth=1.9)
    ax2.plot(t, pitch_err, label="pitch error", linewidth=1.9)
    ax2.set_title("Erreurs de guidage")
    ax2.set_xlabel("Temps (s)")
    ax2.set_ylabel("Erreur (deg)")
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(t, beta, label="beta", linewidth=1.9)
    ax3.plot(t, alpha, label="alpha", linewidth=1.9)
    ax3.set_title("Erreurs aerodynamiques")
    ax3.set_xlabel("Temps (s)")
    ax3.set_ylabel("Angle (deg)")
    ax3.grid(True, alpha=0.3)
    ax3.legend()

    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(t, gimbal_pitch, label="gimbal pitch", linewidth=1.8)
    ax4.plot(t, gimbal_yaw, label="gimbal yaw", linewidth=1.8)
    ax4.set_title("Reglage separe longitudinal/lateral")
    ax4.set_xlabel("Temps (s)")
    ax4.set_ylabel("Commande (deg)")
    ax4.grid(True, alpha=0.3)
    ax4.legend()

    fig.tight_layout()
    if params.save_plots:
        save_figure(fig, output_dir / "tvc_rocket_6dof_guided_plots.png")
    present_figure(plt, fig, show_plots, params.plot_display_seconds)
