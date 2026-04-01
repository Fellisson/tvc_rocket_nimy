import unittest

from tvc_rocket.cli import parse_args
from tvc_rocket.models import RocketParameters, default_controller_gains
from tvc_rocket.simulation import build_pid, simulate_rocket
from tvc_rocket.simulation_3d import simulate_rocket_3d_preview


class CliAnd3DTestCase(unittest.TestCase):
    def test_parse_args_supports_preview_3d(self):
        args = parse_args(["--no-tune", "--preview-3d", "--impact-target", "900", "--crosswind-ref", "6"])
        self.assertTrue(args.no_tune)
        self.assertTrue(args.preview_3d)
        self.assertEqual(args.impact_target, 900.0)
        self.assertEqual(args.crosswind_ref, 6.0)

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


if __name__ == "__main__":
    unittest.main()
