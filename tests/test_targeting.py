import unittest

from tvc_rocket.cli import parse_args
from tvc_rocket.geodesic_guidance import (
    build_geodesic_guidance_plan,
    format_geodesic_guidance_report,
    geodetic_to_ecef,
)
from tvc_rocket.models import RocketParameters
from tvc_rocket.simulation_earth_guided import (
    build_regional_earth_guided_params,
    simulate_earth_guided,
    summarize_earth_guided_metrics,
)
from tvc_rocket.targeting import (
    KINSHASA_LAUNCH_SITE,
    build_mission_target_profile,
    format_mission_target_report,
    resolve_city_target,
)


class TargetingTestCase(unittest.TestCase):
    def test_parse_args_supports_city_targeting(self):
        args = parse_args(
            [
                "--target-city",
                "Paris",
                "--target-country",
                "France",
                "--target-offline-only",
                "--target-report-only",
                "--apply-city-target",
                "--guidance-report-only",
                "--mode",
                "earth-guided",
                "--gui",
            ]
        )
        self.assertEqual(args.target_city, "Paris")
        self.assertEqual(args.target_country, "France")
        self.assertTrue(args.target_offline_only)
        self.assertTrue(args.target_report_only)
        self.assertTrue(args.apply_city_target)
        self.assertTrue(args.guidance_report_only)
        self.assertEqual(args.mode, "earth-guided")
        self.assertTrue(args.gui)

    def test_resolve_city_target_offline_catalog(self):
        target = resolve_city_target("Paris", allow_online=False)
        self.assertEqual(target.city, "Paris")
        self.assertEqual(target.country, "France")
        self.assertAlmostEqual(target.latitude_deg, 48.8566, places=4)

    def test_build_mission_target_profile_from_kinshasa(self):
        params = RocketParameters()
        profile = build_mission_target_profile("Paris", params, allow_online=False)
        self.assertEqual(profile.launch_site, KINSHASA_LAUNCH_SITE)
        self.assertGreater(profile.surface_distance_m, 1_000_000.0)
        self.assertGreaterEqual(profile.initial_bearing_deg, 0.0)
        self.assertLess(profile.initial_bearing_deg, 360.0)
        self.assertGreater(profile.ideal_delta_v_m_s, 0.0)
        self.assertGreater(profile.initial_thrust_to_weight, 0.0)

    def test_format_report_mentions_guidance_and_city(self):
        params = RocketParameters()
        profile = build_mission_target_profile("Tokyo", params, allow_online=False)
        report = format_mission_target_report(profile)
        self.assertIn("Tokyo", report)
        self.assertIn("Guidage futur", report)
        self.assertIn("Distance surface", report)

    def test_geodetic_to_ecef_has_expected_radius(self):
        point = geodetic_to_ecef(-4.4419, 15.2663, 280.0)
        radius = (point.x_m * point.x_m + point.y_m * point.y_m + point.z_m * point.z_m) ** 0.5
        self.assertAlmostEqual(radius, 6_371_280.0, delta=2.0)

    def test_build_geodesic_guidance_plan_for_paris(self):
        params = RocketParameters()
        profile = build_mission_target_profile("Paris", params, allow_online=False)
        plan = build_geodesic_guidance_plan(profile, params)
        self.assertGreater(plan.horizontal_range_m, 1_000_000.0)
        self.assertGreater(plan.slant_range_m, 1_000_000.0)
        self.assertGreaterEqual(plan.launch_azimuth_deg, 0.0)
        self.assertLess(plan.launch_azimuth_deg, 360.0)
        self.assertGreater(plan.earth_rotation_east_speed_m_s, 0.0)

    def test_format_geodesic_guidance_report_mentions_azimuth(self):
        params = RocketParameters()
        profile = build_mission_target_profile("Tokyo", params, allow_online=False)
        plan = build_geodesic_guidance_plan(profile, params)
        report = format_geodesic_guidance_report(plan)
        self.assertIn("Azimuth lancement", report)
        self.assertIn("Repere ENU cible", report)
        self.assertIn("Integration future", report)

    def test_earth_guided_simulation_runs(self):
        params = RocketParameters(
            t_final=40.0,
            show_plots=False,
            save_plots=False,
        )
        profile = build_mission_target_profile("Brazzaville", params, allow_online=False)
        plan = build_geodesic_guidance_plan(profile, params)
        history = simulate_earth_guided(params, profile, plan)
        metrics = summarize_earth_guided_metrics(history, plan, profile)
        self.assertGreater(len(history), 10)
        self.assertGreater(metrics["flight_time"], 0.0)
        self.assertGreaterEqual(metrics["azimuth_deg"], 0.0)

    def test_regional_earth_guided_preset_improves_brazzaville_tracking(self):
        base = RocketParameters(show_plots=False, save_plots=False)
        profile = build_mission_target_profile("Brazzaville", base, allow_online=False)
        base_plan = build_geodesic_guidance_plan(profile, base)
        base_history = simulate_earth_guided(base, profile, base_plan)
        base_metrics = summarize_earth_guided_metrics(base_history, base_plan, profile)

        tuned = build_regional_earth_guided_params(base, profile)
        tuned_plan = build_geodesic_guidance_plan(profile, tuned)
        tuned_history = simulate_earth_guided(tuned, profile, tuned_plan)
        tuned_metrics = summarize_earth_guided_metrics(tuned_history, tuned_plan, profile)

        self.assertLess(tuned_metrics["horizontal_error_m"], base_metrics["horizontal_error_m"])
        self.assertLess(tuned_metrics["horizontal_error_m"], 50.0)


if __name__ == "__main__":
    unittest.main()
