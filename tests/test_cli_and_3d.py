import unittest

from tvc_rocket.cli import parse_args
from tvc_rocket.models import RocketParameters, default_controller_gains
from tvc_rocket.simulation import build_pid, simulate_rocket
from tvc_rocket.simulation_3d import simulate_rocket_3d_preview
from tvc_rocket.simulation_6dof import simulate_rocket_6dof
from tvc_rocket.simulation_6dof_guided import (
    build_short_range_guided_params,
    simulate_rocket_6dof_guided,
    summarize_6dof_guided_metrics,
)


class CliAnd3DTestCase(unittest.TestCase):
    def test_parse_args_supports_preview_3d(self):
        args = parse_args(
            [
                "--no-tune",
                "--preview-3d",
                "--impact-target",
                "900",
                "--impact-target-y",
                "120",
                "--crosswind-ref",
                "6",
                "--guided-phase",
                "lateral",
            ]
        )
        self.assertTrue(args.no_tune)
        self.assertTrue(args.preview_3d)
        self.assertEqual(args.impact_target, 900.0)
        self.assertEqual(args.impact_target_y, 120.0)
        self.assertEqual(args.crosswind_ref, 6.0)
        self.assertEqual(args.guided_phase, "lateral")

    def test_preview_3d_generates_lateral_history(self):
        params = RocketParameters(
            t_final=30.0,
            crosswind_ref_m_s=8.0,
            show_plots=False,
            save_plots=False,
        )
        gains = default_controller_gains(params)
        history_2d = simulate_rocket(params, build_pid(params, gains))
        history_3d = simulate_rocket_3d_preview(history_2d, params)
        self.assertEqual(len(history_3d), len(history_2d))
        self.assertGreater(max(abs(sample["y"]) for sample in history_3d), 0.0)

    def test_simulation_6dof_runs(self):
        params = RocketParameters(
            t_final=120.0,
            crosswind_ref_m_s=6.0,
            impact_target_y=150.0,
            show_plots=False,
            save_plots=False,
        )
        history = simulate_rocket_6dof(params)
        self.assertGreater(len(history), 10)
        self.assertAlmostEqual(history[-1]["z"], 0.0, places=4)
        self.assertIn("gimbal_pitch_deg", history[-1])
        self.assertIn("roll", history[-1])

    def test_simulation_6dof_guided_tracks_error_signals(self):
        params = RocketParameters(
            t_final=120.0,
            crosswind_ref_m_s=4.0,
            impact_target=800.0,
            impact_target_y=100.0,
            show_plots=False,
            save_plots=False,
        )
        history = simulate_rocket_6dof_guided(params)
        self.assertGreater(len(history), 10)
        self.assertAlmostEqual(history[-1]["z"], 0.0, places=4)
        self.assertIn("yaw_error_deg", history[-1])
        self.assertIn("pitch_error_deg", history[-1])
        self.assertIn("alpha_deg", history[-1])
        self.assertIn("beta_deg", history[-1])

    def test_short_range_guided_preset_reduces_energy(self):
        params = RocketParameters(impact_target=800.0, show_plots=False, save_plots=False)
        tuned = build_short_range_guided_params(params)
        self.assertLess(tuned.thrust, params.thrust)
        self.assertLess(tuned.burn_time, params.burn_time)
        history = simulate_rocket_6dof_guided(tuned, phase_mode="auto")
        self.assertGreater(len(history), 10)

    def test_short_range_guided_preset_improves_short_target_tracking(self):
        params = RocketParameters(
            impact_target=800.0,
            impact_target_y=120.0,
            show_plots=False,
            save_plots=False,
        )
        tuned = build_short_range_guided_params(params)
        history = simulate_rocket_6dof_guided(tuned, phase_mode="auto")
        metrics = summarize_6dof_guided_metrics(history, tuned)
        self.assertLess(metrics["impact_error_3d"], 80.0)
        self.assertGreater(metrics["crossrange"], 50.0)


if __name__ == "__main__":
    unittest.main()
