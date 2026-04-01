from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path

from .models import ControllerGains, RocketParameters, TuningResult, default_controller_gains
from .output import write_history_csv, write_tuning_csv
from .plotting import maybe_plot, maybe_plot_3d_preview, maybe_plot_tuning_comparison, summarize, summarize_3d_preview
from .simulation import build_pid, evaluate_history, score_history, simulate_rocket
from .simulation_3d import simulate_rocket_3d_preview
from .tuning import auto_tune_controller


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simulation de fusee TVC avec tuning automatique vers un point d'impact cible."
    )
    parser.add_argument("--impact-target", type=float, default=None, help="Distance cible d'impact au sol en metres.")
    parser.add_argument("--altitude-target", type=float, default=None, help="Apogee cible en metres.")
    parser.add_argument(
        "--display-seconds",
        type=float,
        default=None,
        help="Duree d'affichage automatique des graphes en secondes.",
    )
    parser.add_argument(
        "--save-only",
        action="store_true",
        help="Sauvegarde les PNG sans ouvrir les fenetres de graphes.",
    )
    parser.add_argument(
        "--tuning-mode",
        choices=("fast", "accurate"),
        default="accurate",
        help="Mode de tuning: rapide ou plus precis.",
    )
    parser.add_argument(
        "--no-tune",
        action="store_true",
        help="Desactive l'autotuning et lance directement une simulation avec les gains fournis.",
    )
    parser.add_argument("--kp", type=float, default=None, help="Gain proportionnel du PID.")
    parser.add_argument("--ki", type=float, default=None, help="Gain integral du PID.")
    parser.add_argument("--kd", type=float, default=None, help="Gain derive du PID.")
    parser.add_argument("--guidance-gain", type=float, default=None, help="Gain principal de la loi de guidage.")
    parser.add_argument(
        "--crosswind-ref",
        type=float,
        default=None,
        help="Vent lateral de reference en m/s pour la preview 3D.",
    )
    parser.add_argument(
        "--guidance-altitude-gain",
        type=float,
        default=None,
        help="Gain de correction d'altitude de la loi de guidage.",
    )
    parser.add_argument(
        "--gamma-final-deg",
        type=float,
        default=None,
        help="Angle de trajectoire vise en fin de gravity turn, en degres.",
    )
    parser.add_argument(
        "--adaptive-alpha-schedule",
        action="store_true",
        help="Active une limitation experimentale de la consigne alpha en fonction de la pression dynamique.",
    )
    parser.add_argument(
        "--preview-3d",
        action="store_true",
        help="Genere une preview 3D simplifiee a partir de la trajectoire 2D et d'un vent lateral.",
    )
    return parser.parse_args(argv)


def main() -> None:
    args = parse_args()
    base_params = RocketParameters(tuning_mode=args.tuning_mode)
    if args.impact_target is not None:
        base_params = replace(base_params, impact_target=args.impact_target)
    if args.altitude_target is not None:
        base_params = replace(base_params, altitude_target=args.altitude_target)
    if args.crosswind_ref is not None:
        base_params = replace(base_params, crosswind_ref_m_s=args.crosswind_ref)
    if args.display_seconds is not None:
        base_params = replace(base_params, plot_display_seconds=max(args.display_seconds, 0.1))
    if args.save_only:
        base_params = replace(base_params, show_plots=False, save_plots=True)
    if args.adaptive_alpha_schedule:
        base_params = replace(base_params, use_alpha_schedule=True)

    output_dir = Path(__file__).resolve().parent.parent
    results_dir = output_dir / "results"
    plots_dir = output_dir / "plots"
    results_dir.mkdir(parents=True, exist_ok=True)
    tuning_csv = results_dir / "tvc_rocket_tuning_results.csv"
    history_csv = results_dir / "tvc_rocket_best_history.csv"
    history_3d_csv = results_dir / "tvc_rocket_3d_preview.csv"

    if args.no_tune:
        gains = default_controller_gains(base_params)
        gains = ControllerGains(
            kp=args.kp if args.kp is not None else gains.kp,
            ki=args.ki if args.ki is not None else gains.ki,
            kd=args.kd if args.kd is not None else gains.kd,
            guidance_gain=args.guidance_gain if args.guidance_gain is not None else gains.guidance_gain,
            guidance_altitude_gain=(
                args.guidance_altitude_gain
                if args.guidance_altitude_gain is not None
                else gains.guidance_altitude_gain
            ),
            gamma_final_deg=(
                args.gamma_final_deg if args.gamma_final_deg is not None else gains.gamma_final_deg
            ),
        )
        params = replace(
            base_params,
            guidance_gain=gains.guidance_gain,
            guidance_altitude_gain=gains.guidance_altitude_gain,
            gamma_final_deg=gains.gamma_final_deg,
        )
        pid = build_pid(params, gains)
        history = simulate_rocket(params, pid)
        tuning_metrics = evaluate_history(history, params)
        tuning_metrics["score"] = score_history(history, params)
        tuning_results: list[TuningResult] = []
        print("Autotuning           : desactive")
    else:
        params, gains, tuning_metrics, tuning_results = auto_tune_controller(base_params)
        pid = build_pid(params, gains)
        history = simulate_rocket(params, pid)

    print("Meilleure configuration trouvee")
    print(f"Mode de tuning       : {params.tuning_mode}")
    print(
        "PID="
        f"({gains.kp:.2f}, {gains.ki:.2f}, {gains.kd:.2f}) "
        f"guidage={gains.guidance_gain:.2f} "
        f"alt_gain={gains.guidance_altitude_gain:.5f} "
        f"gamma_fin={gains.gamma_final_deg:.1f} deg"
    )
    print(
        f"Score={tuning_metrics['score']:.1f} "
        f"apogee={tuning_metrics['peak_altitude']:.1f} m "
        f"portee={tuning_metrics['downrange']:.1f} m "
        f"err_impact={tuning_metrics['impact_error']:.1f} m "
        f"t_vol={tuning_metrics['flight_time']:.1f} s "
        f"alpha_moy={tuning_metrics['mean_alpha_deg']:.2f} deg "
        f"sat={tuning_metrics['saturation_ratio'] * 100.0:.1f} %"
    )

    if tuning_results:
        write_tuning_csv(tuning_results, tuning_csv)
    write_history_csv(history, history_csv)
    if tuning_results:
        print(f"CSV tuning           : {tuning_csv}")
    print(f"CSV trajectoire      : {history_csv}")
    if params.save_plots:
        print(f"Dossier PNG          : {plots_dir}")
    summarize(history, params)
    maybe_plot(history, params, plots_dir)
    if args.preview_3d:
        history_3d = simulate_rocket_3d_preview(history, params)
        write_history_csv(history_3d, history_3d_csv)
        print(f"CSV preview 3D       : {history_3d_csv}")
        summarize_3d_preview(history_3d)
        maybe_plot_3d_preview(history_3d, params, plots_dir)
    if tuning_results:
        maybe_plot_tuning_comparison(params, tuning_results, plots_dir)
