from __future__ import annotations

import csv
from pathlib import Path

from .models import TuningResult


def write_tuning_csv(results: list[TuningResult], output_path: Path) -> None:
    fieldnames = [
        "rank",
        "score",
        "kp",
        "ki",
        "kd",
        "guidance_gain",
        "guidance_altitude_gain",
        "gamma_final_deg",
        "peak_altitude",
        "downrange",
        "impact_error",
        "final_speed",
        "flight_time",
        "landed",
        "mean_alpha_deg",
        "max_alpha_deg",
        "mean_gimbal_deg",
        "max_gimbal_deg",
        "saturation_ratio",
        "mean_gamma_error_deg",
    ]
    with output_path.open("w", newline="", encoding="utf-8") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for rank, result in enumerate(results, start=1):
            writer.writerow(
                {
                    "rank": rank,
                    "score": f"{result.score:.4f}",
                    "kp": result.gains.kp,
                    "ki": result.gains.ki,
                    "kd": result.gains.kd,
                    "guidance_gain": result.gains.guidance_gain,
                    "guidance_altitude_gain": result.gains.guidance_altitude_gain,
                    "gamma_final_deg": result.gains.gamma_final_deg,
                    **{key: f"{value:.6f}" for key, value in result.metrics.items()},
                }
            )


def write_history_csv(history: list[dict[str, float]], output_path: Path) -> None:
    if not history:
        return
    fieldnames = list(history[0].keys())
    with output_path.open("w", newline="", encoding="utf-8") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(history)


def save_figure(fig, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=160, bbox_inches="tight")


def prepare_matplotlib(show_plots: bool):
    import matplotlib

    if show_plots:
        try:
            import PyQt6  # noqa: F401

            matplotlib.use("QtAgg")
        except Exception:
            try:
                import tkinter as tk

                root = tk.Tk()
                root.withdraw()
                root.destroy()
            except Exception:
                print("Backend graphique indisponible : bascule automatique en sauvegarde PNG seule.")
                show_plots = False

    if not show_plots:
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    return plt, show_plots


def present_figure(plt, fig, show_plots: bool, display_seconds: float) -> None:
    if show_plots:
        plt.show(block=False)
        plt.pause(max(display_seconds, 0.1))
        plt.close(fig)
    else:
        plt.close(fig)
