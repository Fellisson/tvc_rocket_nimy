import unittest

from tvc_rocket.models import RocketParameters, default_controller_gains
from tvc_rocket.simulation import (
    atmosphere_state,
    build_pid,
    effective_drag_coefficient,
    evaluate_history,
    simulate_rocket,
    thrust_profile_factor,
)


class SimulationTestCase(unittest.TestCase):
    def test_atmosphere_density_decreases_with_altitude(self):
        sea_level = atmosphere_state(0.0)
        high_alt = atmosphere_state(5000.0)
        self.assertGreater(sea_level["density"], high_alt["density"])
        self.assertGreater(sea_level["pressure"], high_alt["pressure"])

    def test_drag_rises_near_transonic_regime(self):
        params = RocketParameters()
        cd_low = effective_drag_coefficient(0.0, 0.3, params)
        cd_transonic = effective_drag_coefficient(0.0, 1.0, params)
        self.assertGreater(cd_transonic, cd_low)

    def test_thrust_profile_has_ramp_and_tailoff(self):
        params = RocketParameters()
        self.assertLess(thrust_profile_factor(0.0, params), 1.0)
        self.assertEqual(thrust_profile_factor(params.burn_time + 1.0, params), 0.0)
        self.assertLess(thrust_profile_factor(params.burn_time - 0.1, params), 1.0)

    def test_simulation_runs_and_lands(self):
        params = RocketParameters(t_final=80.0, show_plots=False, save_plots=False)
        gains = default_controller_gains(params)
        history = simulate_rocket(params, build_pid(params, gains))
        metrics = evaluate_history(history, params)
        self.assertGreater(len(history), 10)
        self.assertAlmostEqual(history[-1]["z"], 0.0, places=4)
        self.assertGreater(metrics["flight_time"], 0.0)


if __name__ == "__main__":
    unittest.main()
