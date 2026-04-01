import shutil
from pathlib import Path
import unittest

from tvc_rocket.cli import parse_args
from tvc_rocket.geodesic_guidance import (
    build_geodesic_guidance_plan,
    format_geodesic_guidance_report,
    geodetic_to_ecef,
)
from tvc_rocket.gui import (
    apply_export_payload_to_form,
    apply_gui_parameter_overrides,
    build_canvas_polyline_points,
    filter_preset_rows,
    format_json_payload,
)
from tvc_rocket.gui_storage import (
    append_mission_history,
    build_export_comparison_rows,
    build_gui_preset_payload,
    build_preset_comparison_rows,
    build_mission_export_payload,
    clear_mission_history,
    delete_exported_mission,
    delete_gui_preset,
    export_mission_json,
    list_gui_presets,
    load_gui_preset,
    read_mission_history,
    save_gui_preset,
)
from tvc_rocket.mission_design import (
    apply_mission_design_recommendation,
    build_mission_design_recommendation,
    format_mission_design_report,
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
                "--mission-design-only",
                "--apply-mission-design",
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
        self.assertTrue(args.mission_design_only)
        self.assertTrue(args.apply_mission_design)

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

    def test_mission_design_recommendation_has_positive_values(self):
        params = RocketParameters(show_plots=False, save_plots=False)
        profile = build_mission_target_profile("Brazzaville", params, allow_online=False)
        plan = build_geodesic_guidance_plan(profile, params)
        design = build_mission_design_recommendation(params, profile, plan)
        self.assertGreater(design.recommended_thrust_n, 0.0)
        self.assertGreater(design.required_delta_v_m_s, 0.0)
        self.assertGreater(design.estimated_mass_ratio, 1.0)
        report = format_mission_design_report(design)
        self.assertIn("Design mission", report)

    def test_apply_mission_design_recommendation_increases_vehicle_energy(self):
        params = RocketParameters(show_plots=False, save_plots=False)
        profile = build_mission_target_profile("Brazzaville", params, allow_online=False)
        plan = build_geodesic_guidance_plan(profile, params)
        design = build_mission_design_recommendation(params, profile, plan)
        tuned = apply_mission_design_recommendation(params, design)
        self.assertGreaterEqual(tuned.thrust, params.thrust)
        self.assertGreaterEqual(tuned.burn_time, params.burn_time)
        self.assertGreaterEqual(tuned.propellant_mass, params.propellant_mass)

    def test_apply_gui_parameter_overrides_updates_numeric_fields(self):
        params = RocketParameters()
        tuned = apply_gui_parameter_overrides(
            params,
            thrust_text="1234",
            propellant_mass_text="15.5",
            burn_time_text="22",
            theta_initial_deg_text="47",
            altitude_target_text="900",
            t_final_text="140",
        )
        self.assertEqual(tuned.thrust, 1234.0)
        self.assertEqual(tuned.propellant_mass, 15.5)
        self.assertEqual(tuned.burn_time, 22.0)
        self.assertEqual(tuned.theta_initial_deg, 47.0)
        self.assertEqual(tuned.altitude_target, 900.0)
        self.assertEqual(tuned.t_final, 140.0)

    def test_build_canvas_polyline_points_returns_pairs(self):
        points = build_canvas_polyline_points([0.0, 1.0, 2.0], [10.0, 20.0, 15.0], 300, 200)
        self.assertEqual(len(points), 6)
        self.assertGreaterEqual(min(points[0::2]), 0.0)
        self.assertGreaterEqual(min(points[1::2]), 0.0)

    def test_format_json_payload_contains_keys(self):
        payload = {"city": "Brazzaville", "thrust_n": 1600.0}
        text = format_json_payload(payload)
        self.assertIn('"city"', text)
        self.assertIn('"thrust_n"', text)

    def test_filter_preset_rows_can_filter_by_search_favorite_and_tag(self):
        rows = [
            {"name": "demo_a", "city": "Brazzaville", "country": "Republic of the Congo", "tags": "regional, demo", "favorite": True},
            {"name": "demo_b", "city": "Paris", "country": "France", "tags": "global", "favorite": False},
        ]
        filtered = filter_preset_rows(rows, search_text="brazz", favorites_only=True, tag_filter="regional")
        self.assertEqual(len(filtered), 1)
        self.assertEqual(filtered[0]["name"], "demo_a")

    def test_apply_export_payload_to_form_restores_values(self):
        payload = {
            "city": "Brazzaville",
            "country": "Republic of the Congo",
            "vehicle": {
                "thrust_n": 1600.0,
                "propellant_mass_kg": 24.0,
                "burn_time_s": 38.0,
                "theta_initial_deg": 48.0,
                "altitude_target_m": 750.0,
                "t_final_s": 68.0,
            },
        }
        restored = {"city": "", "country": "", "params": {}}
        apply_export_payload_to_form(
            payload,
            set_city=lambda value: restored.__setitem__("city", value),
            set_country=lambda value: restored.__setitem__("country", value),
            set_params=lambda value: restored.__setitem__("params", value),
        )
        self.assertEqual(restored["city"], "Brazzaville")
        self.assertEqual(restored["country"], "Republic of the Congo")
        self.assertEqual(restored["params"]["thrust"], 1600.0)

    def test_gui_storage_can_save_and_load_preset(self):
        params = RocketParameters()
        payload = build_gui_preset_payload(
            city="Brazzaville",
            country="Republic of the Congo",
            offline_only=True,
            short_range=False,
            regional_earth=True,
            apply_design=True,
            favorite=True,
            tags=["regional", "demo"],
            params=params,
        )
        root = Path(__file__).resolve().parent / "_tmp_gui_storage"
        if root.exists():
            shutil.rmtree(root)
        root.mkdir(parents=True, exist_ok=True)
        try:
            path = save_gui_preset(root, "brazzaville_demo", payload)
            self.assertTrue(path.exists())
            self.assertIn("brazzaville_demo", list_gui_presets(root))
            loaded = load_gui_preset(root, "brazzaville_demo")
            self.assertEqual(loaded["city"], "Brazzaville")
            self.assertTrue(loaded["regional_earth"])
            self.assertTrue(loaded["favorite"])
            self.assertEqual(loaded["tags"], ["regional", "demo"])
        finally:
            if root.exists():
                shutil.rmtree(root)

    def test_gui_storage_can_export_mission_json(self):
        params = RocketParameters(show_plots=False, save_plots=False)
        profile = build_mission_target_profile("Brazzaville", params, allow_online=False)
        plan = build_geodesic_guidance_plan(profile, params)
        design = build_mission_design_recommendation(params, profile, plan)
        payload = build_mission_export_payload(
            city="Brazzaville",
            country="Republic of the Congo",
            params=params,
            mission=profile,
            guidance=plan,
            design=design,
        )
        root = Path(__file__).resolve().parent / "_tmp_gui_export"
        if root.exists():
            shutil.rmtree(root)
        root.mkdir(parents=True, exist_ok=True)
        try:
            path = export_mission_json(root, "brazzaville_demo", payload)
            self.assertTrue(path.exists())
            content = path.read_text(encoding="utf-8")
            self.assertIn("mission_target", content)
            self.assertIn("guidance_plan", content)
        finally:
            if root.exists():
                shutil.rmtree(root)

    def test_gui_storage_can_append_and_read_history(self):
        root = Path(__file__).resolve().parent / "_tmp_gui_history"
        if root.exists():
            shutil.rmtree(root)
        root.mkdir(parents=True, exist_ok=True)
        try:
            append_mission_history(root, {"type": "simulation", "mode": "earth_guided", "city": "Brazzaville"})
            append_mission_history(root, {"type": "mission_export", "preset_name": "demo"})
            history = read_mission_history(root, limit=10)
            self.assertEqual(len(history), 2)
            self.assertEqual(history[-1]["type"], "mission_export")
        finally:
            if root.exists():
                shutil.rmtree(root)

    def test_gui_storage_can_build_comparison_rows(self):
        root = Path(__file__).resolve().parent / "_tmp_gui_compare"
        if root.exists():
            shutil.rmtree(root)
        root.mkdir(parents=True, exist_ok=True)
        try:
            params = RocketParameters(show_plots=False, save_plots=False)
            preset_payload = build_gui_preset_payload(
                city="Brazzaville",
                country="Republic of the Congo",
                offline_only=True,
                short_range=False,
                regional_earth=True,
                apply_design=False,
                favorite=True,
                tags=["favorite", "regional"],
                params=params,
            )
            save_gui_preset(root, "demo", preset_payload)
            profile = build_mission_target_profile("Brazzaville", params, allow_online=False)
            plan = build_geodesic_guidance_plan(profile, params)
            design = build_mission_design_recommendation(params, profile, plan)
            export_payload = build_mission_export_payload(
                city="Brazzaville",
                country="Republic of the Congo",
                params=params,
                mission=profile,
                guidance=plan,
                design=design,
            )
            export_mission_json(root, "demo", export_payload)
            preset_rows = build_preset_comparison_rows(root)
            export_rows = build_export_comparison_rows(root)
            self.assertEqual(len(preset_rows), 1)
            self.assertEqual(len(export_rows), 1)
            self.assertEqual(preset_rows[0]["name"], "demo")
            self.assertTrue(preset_rows[0]["favorite"])
            self.assertIn("favorite", preset_rows[0]["tags"])
            self.assertEqual(export_rows[0]["mission_class"], design.mission_class)
        finally:
            if root.exists():
                shutil.rmtree(root)

    def test_gui_storage_can_delete_preset_and_export(self):
        root = Path(__file__).resolve().parent / "_tmp_gui_delete"
        if root.exists():
            shutil.rmtree(root)
        root.mkdir(parents=True, exist_ok=True)
        try:
            params = RocketParameters()
            preset_payload = build_gui_preset_payload(
                city="Brazzaville",
                country="Republic of the Congo",
                offline_only=True,
                short_range=False,
                regional_earth=True,
                apply_design=False,
                favorite=False,
                tags=[],
                params=params,
            )
            preset_path = save_gui_preset(root, "demo", preset_payload)
            self.assertTrue(preset_path.exists())
            delete_gui_preset(root, "demo")
            self.assertFalse(preset_path.exists())

            profile = build_mission_target_profile("Brazzaville", params, allow_online=False)
            plan = build_geodesic_guidance_plan(profile, params)
            design = build_mission_design_recommendation(params, profile, plan)
            export_payload = build_mission_export_payload(
                city="Brazzaville",
                country="Republic of the Congo",
                params=params,
                mission=profile,
                guidance=plan,
                design=design,
            )
            export_path = export_mission_json(root, "demo", export_payload)
            self.assertTrue(export_path.exists())
            delete_exported_mission(root, "demo")
            self.assertFalse(export_path.exists())
        finally:
            if root.exists():
                shutil.rmtree(root)

    def test_gui_storage_can_clear_history(self):
        root = Path(__file__).resolve().parent / "_tmp_gui_history_clear"
        if root.exists():
            shutil.rmtree(root)
        root.mkdir(parents=True, exist_ok=True)
        try:
            append_mission_history(root, {"type": "simulation", "mode": "earth_guided"})
            self.assertEqual(len(read_mission_history(root, limit=10)), 1)
            clear_mission_history(root)
            self.assertEqual(read_mission_history(root, limit=10), [])
        finally:
            if root.exists():
                shutil.rmtree(root)


if __name__ == "__main__":
    unittest.main()
