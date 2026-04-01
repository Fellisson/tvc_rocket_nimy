from __future__ import annotations

from dataclasses import replace
from itertools import product

from .models import ControllerGains, RocketParameters, TuningResult, default_controller_gains, gains_key
from .simulation import build_pid, evaluate_history, score_history, simulate_rocket


def candidate_gain_sets(
    impact_target: float,
    tuning_mode: str,
) -> list[tuple[float, float, float, float, float, float]]:
    if tuning_mode == "fast":
        if impact_target <= 1000.0:
            return list(
                product(
                    (3.2, 4.0, 4.8),
                    (0.35, 0.7),
                    (0.9, 1.3),
                    (0.32, 0.45, 0.58),
                    (0.00006, 0.0001),
                    (74.0, 78.0),
                )
            )

        return list(
            product(
                (3.2, 4.0, 4.8),
                (0.2, 0.35, 0.7),
                (0.7, 0.9, 1.3),
                (0.45, 0.58, 0.72),
                (0.00003, 0.00006, 0.0001),
                (68.0, 72.0, 74.0),
            )
        )

    if impact_target <= 1000.0:
        return list(
            product(
                (3.2, 4.0, 4.8),
                (0.35, 0.7, 1.0),
                (0.9, 1.3, 1.7),
                (0.32, 0.45, 0.58),
                (0.00006, 0.0001, 0.00015),
                (74.0, 78.0, 82.0),
            )
        )

    return list(
        product(
            (3.2, 4.0, 4.8, 5.6),
            (0.2, 0.35, 0.7, 1.0),
            (0.7, 0.9, 1.3, 1.7),
            (0.45, 0.58, 0.72, 0.9),
            (0.00003, 0.00006, 0.0001, 0.00015),
            (68.0, 72.0, 74.0, 78.0),
        )
    )


def refine_gain_sets(best_results: list[TuningResult], tuning_mode: str) -> list[ControllerGains]:
    refined: list[ControllerGains] = []
    top_count = 2 if tuning_mode == "fast" else 3
    for result in best_results[:top_count]:
        g = result.gains
        if tuning_mode == "fast":
            kp_deltas = (-0.3, 0.0, 0.3)
            ki_deltas = (-0.08, 0.0, 0.08)
            kd_deltas = (-0.15, 0.0, 0.15)
            guidance_deltas = (-0.06, 0.0, 0.06)
            altitude_deltas = (-0.00001, 0.0, 0.00001)
            gamma_deltas = (-1.0, 0.0, 1.0)
        else:
            kp_deltas = (-0.4, 0.0, 0.4)
            ki_deltas = (-0.1, 0.0, 0.1)
            kd_deltas = (-0.2, 0.0, 0.2)
            guidance_deltas = (-0.08, 0.0, 0.08)
            altitude_deltas = (-0.00002, 0.0, 0.00002)
            gamma_deltas = (-2.0, 0.0, 2.0)

        kp_values = sorted({max(1.0, round(g.kp + delta, 2)) for delta in kp_deltas})
        ki_values = sorted({max(0.0, round(g.ki + delta, 2)) for delta in ki_deltas})
        kd_values = sorted({max(0.0, round(g.kd + delta, 2)) for delta in kd_deltas})
        guidance_values = sorted({max(0.1, round(g.guidance_gain + delta, 2)) for delta in guidance_deltas})
        altitude_values = sorted(
            {
                max(0.00001, round(g.guidance_altitude_gain + delta, 5))
                for delta in altitude_deltas
            }
        )
        gamma_values = sorted({round(g.gamma_final_deg + delta, 1) for delta in gamma_deltas})

        for kp, ki, kd, guidance_gain, guidance_altitude_gain, gamma_final_deg in product(
            kp_values,
            ki_values,
            kd_values,
            guidance_values,
            altitude_values,
            gamma_values,
        ):
            refined.append(
                ControllerGains(
                    kp=kp,
                    ki=ki,
                    kd=kd,
                    guidance_gain=guidance_gain,
                    guidance_altitude_gain=guidance_altitude_gain,
                    gamma_final_deg=gamma_final_deg,
                )
            )

    unique: dict[tuple[float, float, float, float, float, float], ControllerGains] = {}
    for gains in refined:
        unique[gains_key(gains)] = gains
    return list(unique.values())


def select_seed_results(results: list[TuningResult], tuning_mode: str) -> list[TuningResult]:
    ordered_by_score = sorted(results, key=lambda item: item.score)
    ordered_by_impact = sorted(results, key=lambda item: item.metrics["impact_error"])
    ordered_by_alpha = sorted(
        results,
        key=lambda item: (
            item.metrics["mean_alpha_deg"],
            item.metrics["saturation_ratio"],
            item.metrics["impact_error"],
        ),
    )

    target_count = 3 if tuning_mode == "fast" else 6
    seeds: list[TuningResult] = []
    seen: set[tuple[float, float, float, float, float, float]] = set()

    for pool in (ordered_by_score, ordered_by_impact, ordered_by_alpha):
        for result in pool:
            key = gains_key(result.gains)
            if key in seen:
                continue
            seen.add(key)
            seeds.append(result)
            if len(seeds) >= target_count:
                return seeds
    return seeds


def ultra_refine_gain_sets(best_results: list[TuningResult]) -> list[ControllerGains]:
    refined: list[ControllerGains] = []
    for result in best_results[:2]:
        g = result.gains
        for kp, ki, kd, guidance_gain, guidance_altitude_gain, gamma_final_deg in product(
            sorted({max(1.0, round(g.kp + delta, 2)) for delta in (-0.15, 0.0, 0.15)}),
            sorted({max(0.0, round(g.ki + delta, 2)) for delta in (-0.05, 0.0, 0.05)}),
            sorted({max(0.0, round(g.kd + delta, 2)) for delta in (-0.08, 0.0, 0.08)}),
            sorted({max(0.1, round(g.guidance_gain + delta, 2)) for delta in (-0.03, 0.0, 0.03)}),
            sorted(
                {
                    max(0.00001, round(g.guidance_altitude_gain + delta, 5))
                    for delta in (-0.00001, 0.0, 0.00001)
                }
            ),
            sorted({round(g.gamma_final_deg + delta, 1) for delta in (-0.8, 0.0, 0.8)}),
        ):
            refined.append(
                ControllerGains(
                    kp=kp,
                    ki=ki,
                    kd=kd,
                    guidance_gain=guidance_gain,
                    guidance_altitude_gain=guidance_altitude_gain,
                    gamma_final_deg=gamma_final_deg,
                )
            )

    unique: dict[tuple[float, float, float, float, float, float], ControllerGains] = {}
    for gains in refined:
        unique[gains_key(gains)] = gains
    return list(unique.values())


def evaluate_candidate(
    base_params: RocketParameters,
    gains: ControllerGains,
) -> tuple[RocketParameters, TuningResult]:
    params = replace(
        base_params,
        guidance_gain=gains.guidance_gain,
        guidance_altitude_gain=gains.guidance_altitude_gain,
        gamma_final_deg=gains.gamma_final_deg,
    )
    history = simulate_rocket(params, build_pid(params, gains))
    score = score_history(history, params)
    metrics = evaluate_history(history, params)
    return params, TuningResult(gains=gains, metrics=metrics, score=score)


def auto_tune_controller(
    base_params: RocketParameters,
) -> tuple[RocketParameters, ControllerGains, dict[str, float], list[TuningResult]]:
    coarse_candidates = candidate_gain_sets(base_params.impact_target, base_params.tuning_mode)

    best_score = float("inf")
    best_params = base_params
    best_gains = default_controller_gains(base_params)
    best_metrics: dict[str, float] = {}
    results: list[TuningResult] = []
    evaluated: set[tuple[float, float, float, float, float, float]] = set()

    def evaluate_and_store(gains: ControllerGains) -> None:
        nonlocal best_score, best_params, best_gains, best_metrics
        key = gains_key(gains)
        if key in evaluated:
            return
        evaluated.add(key)

        params, result = evaluate_candidate(base_params, gains)
        results.append(result)
        if result.score < best_score:
            best_score = result.score
            best_params = params
            best_gains = gains
            best_metrics = result.metrics

    for kp, ki, kd, guidance_gain, guidance_altitude_gain, gamma_final_deg in coarse_candidates:
        gains = ControllerGains(kp, ki, kd, guidance_gain, guidance_altitude_gain, gamma_final_deg)
        evaluate_and_store(gains)

    results.sort(key=lambda item: item.score)
    seed_results = select_seed_results(results, base_params.tuning_mode)
    refined_candidates = refine_gain_sets(seed_results, base_params.tuning_mode)

    for gains in refined_candidates:
        evaluate_and_store(gains)

    if base_params.tuning_mode == "accurate":
        results.sort(key=lambda item: item.score)
        ultra_seeds = select_seed_results(results, "accurate")
        for gains in ultra_refine_gain_sets(ultra_seeds):
            evaluate_and_store(gains)

    best_metrics["score"] = best_score
    results.sort(key=lambda item: item.score)
    return best_params, best_gains, best_metrics, results
