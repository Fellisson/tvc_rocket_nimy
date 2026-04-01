from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path
import tkinter as tk
from tkinter import messagebox
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

from .geodesic_guidance import build_geodesic_guidance_plan, format_geodesic_guidance_report
from .gui_storage import (
    append_mission_history,
    build_export_comparison_rows,
    build_gui_preset_payload,
    build_preset_comparison_rows,
    build_mission_export_payload,
    clear_mission_history,
    delete_exported_mission,
    delete_gui_preset,
    export_mission_json,
    list_exported_missions,
    list_gui_presets,
    load_exported_mission,
    load_gui_preset,
    read_mission_history,
    save_gui_preset,
)
from .mission_design import (
    apply_mission_design_recommendation,
    build_mission_design_recommendation,
    format_mission_design_report,
)
from .plotting_earth_guided import summarize_earth_guided
from .plotting_6dof_guided import summarize_6dof_guided
from .simulation_6dof_guided import (
    build_short_range_guided_params,
    simulate_rocket_6dof_guided,
    summarize_6dof_guided_metrics,
)
from .simulation_earth_guided import build_regional_earth_guided_params, simulate_earth_guided
from .targeting import build_mission_target_profile, format_mission_target_report


def build_canvas_polyline_points(
    xs: list[float],
    ys: list[float],
    width: int,
    height: int,
    padding: int = 24,
) -> list[float]:
    if not xs or not ys or len(xs) != len(ys):
        return []
    if len(xs) == 1:
        return [padding, height - padding]

    x_min = min(xs)
    x_max = max(xs)
    y_min = min(ys)
    y_max = max(ys)
    x_span = max(x_max - x_min, 1e-9)
    y_span = max(y_max - y_min, 1e-9)
    inner_width = max(width - 2 * padding, 1)
    inner_height = max(height - 2 * padding, 1)

    points: list[float] = []
    for x_value, y_value in zip(xs, ys):
        canvas_x = padding + (x_value - x_min) / x_span * inner_width
        canvas_y = height - padding - (y_value - y_min) / y_span * inner_height
        points.extend([canvas_x, canvas_y])
    return points


def format_json_payload(payload: dict) -> str:
    return json.dumps(payload, indent=2, ensure_ascii=True)


def filter_preset_rows(
    rows: list[dict],
    *,
    search_text: str,
    favorites_only: bool,
    tag_filter: str,
) -> list[dict]:
    normalized_search = search_text.strip().lower()
    normalized_tag = tag_filter.strip().lower()
    filtered: list[dict] = []
    for row in rows:
        if favorites_only and not row.get("favorite", False):
            continue
        haystack = " ".join(
            [
                str(row.get("name", "")),
                str(row.get("city", "")),
                str(row.get("country", "")),
                str(row.get("tags", "")),
            ]
        ).lower()
        if normalized_search and normalized_search not in haystack:
            continue
        if normalized_tag:
            tags = [token.strip().lower() for token in str(row.get("tags", "")).split(",") if token.strip()]
            if normalized_tag not in tags:
                continue
        filtered.append(row)
    return filtered


def apply_export_payload_to_form(
    payload: dict,
    *,
    set_city,
    set_country,
    set_params,
) -> None:
    set_city(str(payload.get("city", "")))
    set_country(str(payload.get("country", "")))
    vehicle = payload.get("vehicle", {})
    params = {
        "thrust": float(vehicle.get("thrust_n", 0.0)),
        "propellant_mass": float(vehicle.get("propellant_mass_kg", 0.0)),
        "burn_time": float(vehicle.get("burn_time_s", 0.0)),
        "theta_initial_deg": float(vehicle.get("theta_initial_deg", 0.0)),
        "altitude_target": float(vehicle.get("altitude_target_m", 0.0)),
        "t_final": float(vehicle.get("t_final_s", 0.0)),
    }
    set_params(params)


def apply_gui_parameter_overrides(
    base_params,
    *,
    thrust_text: str,
    propellant_mass_text: str,
    burn_time_text: str,
    theta_initial_deg_text: str,
    altitude_target_text: str,
    t_final_text: str,
):
    def parse_or_default(text: str, default: float) -> float:
        cleaned = text.strip()
        if not cleaned:
            return default
        return float(cleaned)

    return replace(
        base_params,
        thrust=parse_or_default(thrust_text, base_params.thrust),
        propellant_mass=parse_or_default(propellant_mass_text, base_params.propellant_mass),
        burn_time=parse_or_default(burn_time_text, base_params.burn_time),
        theta_initial_deg=parse_or_default(theta_initial_deg_text, base_params.theta_initial_deg),
        altitude_target=parse_or_default(altitude_target_text, base_params.altitude_target),
        t_final=parse_or_default(t_final_text, base_params.t_final),
    )


def launch_gui(base_params) -> None:
    root = tk.Tk()
    root.title("TVC Rocket Nimy - Ciblage geographique")
    root.geometry("1120x820")

    city_var = tk.StringVar(value="Paris")
    country_var = tk.StringVar(value="France")
    offline_var = tk.BooleanVar(value=True)
    short_range_var = tk.BooleanVar(value=False)
    regional_earth_var = tk.BooleanVar(value=True)
    apply_design_var = tk.BooleanVar(value=False)
    favorite_var = tk.BooleanVar(value=False)
    preset_favorites_only_var = tk.BooleanVar(value=False)
    thrust_var = tk.StringVar(value=f"{base_params.thrust:.1f}")
    propellant_mass_var = tk.StringVar(value=f"{base_params.propellant_mass:.1f}")
    burn_time_var = tk.StringVar(value=f"{base_params.burn_time:.1f}")
    theta_var = tk.StringVar(value=f"{base_params.theta_initial_deg:.1f}")
    altitude_var = tk.StringVar(value=f"{base_params.altitude_target:.1f}")
    t_final_var = tk.StringVar(value=f"{base_params.t_final:.1f}")
    preset_name_var = tk.StringVar(value="mission_kinshasa")
    tags_var = tk.StringVar(value="")
    preset_search_var = tk.StringVar(value="")
    preset_filter_tag_var = tk.StringVar(value="")
    project_root = Path(__file__).resolve().parent.parent

    frame = ttk.Frame(root, padding=12)
    frame.pack(fill="both", expand=True)

    controls = ttk.Frame(frame)
    controls.pack(fill="x")

    ttk.Label(controls, text="Ville").grid(row=0, column=0, sticky="w", padx=(0, 6), pady=4)
    ttk.Entry(controls, textvariable=city_var, width=30).grid(row=0, column=1, sticky="ew", pady=4)
    ttk.Label(controls, text="Pays").grid(row=0, column=2, sticky="w", padx=(12, 6), pady=4)
    ttk.Entry(controls, textvariable=country_var, width=30).grid(row=0, column=3, sticky="ew", pady=4)

    ttk.Checkbutton(controls, text="Catalogue local uniquement", variable=offline_var).grid(
        row=1, column=0, columnspan=2, sticky="w", pady=4
    )
    ttk.Checkbutton(controls, text="Preset courte portee", variable=short_range_var).grid(
        row=1, column=2, columnspan=2, sticky="w", pady=4
    )
    ttk.Checkbutton(controls, text="Preset earth-guided regional", variable=regional_earth_var).grid(
        row=2, column=0, columnspan=2, sticky="w", pady=4
    )
    ttk.Checkbutton(controls, text="Appliquer design mission", variable=apply_design_var).grid(
        row=2, column=2, columnspan=2, sticky="w", pady=4
    )

    controls.columnconfigure(1, weight=1)
    controls.columnconfigure(3, weight=1)

    params_frame = ttk.LabelFrame(frame, text="Parametres vehicule", padding=10)
    params_frame.pack(fill="x", pady=(12, 0))

    ttk.Label(params_frame, text="Thrust (N)").grid(row=0, column=0, sticky="w", padx=(0, 6), pady=4)
    ttk.Entry(params_frame, textvariable=thrust_var, width=14).grid(row=0, column=1, sticky="w", pady=4)
    ttk.Label(params_frame, text="Propellant (kg)").grid(row=0, column=2, sticky="w", padx=(12, 6), pady=4)
    ttk.Entry(params_frame, textvariable=propellant_mass_var, width=14).grid(row=0, column=3, sticky="w", pady=4)
    ttk.Label(params_frame, text="Burn time (s)").grid(row=0, column=4, sticky="w", padx=(12, 6), pady=4)
    ttk.Entry(params_frame, textvariable=burn_time_var, width=14).grid(row=0, column=5, sticky="w", pady=4)

    ttk.Label(params_frame, text="Pitch initial (deg)").grid(row=1, column=0, sticky="w", padx=(0, 6), pady=4)
    ttk.Entry(params_frame, textvariable=theta_var, width=14).grid(row=1, column=1, sticky="w", pady=4)
    ttk.Label(params_frame, text="Altitude cible (m)").grid(row=1, column=2, sticky="w", padx=(12, 6), pady=4)
    ttk.Entry(params_frame, textvariable=altitude_var, width=14).grid(row=1, column=3, sticky="w", pady=4)
    ttk.Label(params_frame, text="Tfinal (s)").grid(row=1, column=4, sticky="w", padx=(12, 6), pady=4)
    ttk.Entry(params_frame, textvariable=t_final_var, width=14).grid(row=1, column=5, sticky="w", pady=4)

    storage_frame = ttk.LabelFrame(frame, text="Presets et exports", padding=10)
    storage_frame.pack(fill="x", pady=(12, 0))
    ttk.Label(storage_frame, text="Nom").grid(row=0, column=0, sticky="w", padx=(0, 6), pady=4)
    ttk.Entry(storage_frame, textvariable=preset_name_var, width=28).grid(row=0, column=1, sticky="w", pady=4)
    ttk.Checkbutton(storage_frame, text="Favori", variable=favorite_var).grid(row=0, column=2, sticky="w", padx=(12, 6), pady=4)
    ttk.Label(storage_frame, text="Tags").grid(row=0, column=3, sticky="w", padx=(12, 6), pady=4)
    ttk.Entry(storage_frame, textvariable=tags_var, width=32).grid(row=0, column=4, sticky="w", pady=4)

    notebook = ttk.Notebook(frame)
    notebook.pack(fill="both", expand=True, pady=(12, 0))

    report_tab = ttk.Frame(notebook, padding=6)
    graphics_tab = ttk.Frame(notebook, padding=6)
    presets_tab = ttk.Frame(notebook, padding=6)
    exports_tab = ttk.Frame(notebook, padding=6)
    history_tab = ttk.Frame(notebook, padding=6)
    notebook.add(report_tab, text="Rapports")
    notebook.add(graphics_tab, text="Graphiques")
    notebook.add(presets_tab, text="Presets")
    notebook.add(exports_tab, text="Exports")
    notebook.add(history_tab, text="Historique")

    output = ScrolledText(report_tab, wrap="word", font=("Consolas", 10))
    output.pack(fill="both", expand=True)

    preset_columns = ("name", "favorite", "tags", "city", "country", "thrust", "propellant", "burn", "theta", "altitude", "regional", "design")
    preset_tree = ttk.Treeview(presets_tab, columns=preset_columns, show="headings", height=12)
    preset_headers = {
        "name": "Nom",
        "favorite": "Fav",
        "tags": "Tags",
        "city": "Ville",
        "country": "Pays",
        "thrust": "Thrust N",
        "propellant": "Prop kg",
        "burn": "Burn s",
        "theta": "Theta deg",
        "altitude": "Alt m",
        "regional": "Regional",
        "design": "Design",
    }
    for key in preset_columns:
        preset_tree.heading(key, text=preset_headers[key])
        preset_tree.column(key, width=90 if key not in ("country", "city", "name", "tags") else 120, anchor="center")
    preset_filter_frame = ttk.Frame(presets_tab)
    preset_filter_frame.pack(fill="x", pady=(0, 6))
    ttk.Label(preset_filter_frame, text="Recherche").pack(side="left")
    ttk.Entry(preset_filter_frame, textvariable=preset_search_var, width=18).pack(side="left", padx=6)
    ttk.Label(preset_filter_frame, text="Tag").pack(side="left", padx=(8, 0))
    ttk.Entry(preset_filter_frame, textvariable=preset_filter_tag_var, width=14).pack(side="left", padx=6)
    ttk.Checkbutton(preset_filter_frame, text="Favoris seulement", variable=preset_favorites_only_var).pack(side="left", padx=8)
    preset_tree.pack(fill="both", expand=True)

    export_columns = ("name", "city", "country", "distance", "thrust", "burn", "mission_class", "delta_v")
    export_tree = ttk.Treeview(exports_tab, columns=export_columns, show="headings", height=12)
    export_headers = {
        "name": "Nom",
        "city": "Ville",
        "country": "Pays",
        "distance": "Distance km",
        "thrust": "Thrust N",
        "burn": "Burn s",
        "mission_class": "Classe",
        "delta_v": "Delta-v",
    }
    for key in export_columns:
        export_tree.heading(key, text=export_headers[key])
        export_tree.column(key, width=100 if key not in ("country", "city", "name", "mission_class") else 130, anchor="center")
    export_tree.pack(fill="both", expand=True)

    history_columns = ("type", "mode", "preset_name", "city", "country", "samples", "impact_error_3d_m", "mission_class")
    history_tree = ttk.Treeview(history_tab, columns=history_columns, show="headings", height=12)
    history_headers = {
        "type": "Type",
        "mode": "Mode",
        "preset_name": "Preset",
        "city": "Ville",
        "country": "Pays",
        "samples": "Samples",
        "impact_error_3d_m": "Err 3D m",
        "mission_class": "Classe",
    }
    for key in history_columns:
        history_tree.heading(key, text=history_headers[key])
        history_tree.column(key, width=100 if key not in ("country", "city", "preset_name") else 120, anchor="center")
    history_tree.pack(fill="both", expand=True)

    graphics_grid = ttk.Frame(graphics_tab)
    graphics_grid.pack(fill="both", expand=True)
    graphics_grid.columnconfigure(0, weight=1)
    graphics_grid.columnconfigure(1, weight=1)
    graphics_grid.rowconfigure(0, weight=1)
    graphics_grid.rowconfigure(1, weight=1)

    canvas_ground = tk.Canvas(graphics_grid, bg="white", highlightthickness=1, highlightbackground="#cccccc")
    canvas_altitude = tk.Canvas(graphics_grid, bg="white", highlightthickness=1, highlightbackground="#cccccc")
    canvas_speed = tk.Canvas(graphics_grid, bg="white", highlightthickness=1, highlightbackground="#cccccc")
    canvas_pitch = tk.Canvas(graphics_grid, bg="white", highlightthickness=1, highlightbackground="#cccccc")
    canvas_ground.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
    canvas_altitude.grid(row=0, column=1, sticky="nsew", padx=6, pady=6)
    canvas_speed.grid(row=1, column=0, sticky="nsew", padx=6, pady=6)
    canvas_pitch.grid(row=1, column=1, sticky="nsew", padx=6, pady=6)

    def render_text(text: str) -> None:
        output.configure(state="normal")
        output.delete("1.0", "end")
        output.insert("1.0", text)
        output.configure(state="disabled")
        notebook.select(report_tab)

    def draw_series_chart(
        canvas: tk.Canvas,
        title: str,
        xs: list[float],
        ys: list[float],
        color: str,
        y_label: str,
    ) -> None:
        canvas.update_idletasks()
        width = max(canvas.winfo_width(), 320)
        height = max(canvas.winfo_height(), 220)
        canvas.delete("all")
        canvas.create_text(12, 12, anchor="nw", text=title, fill="#222222", font=("Segoe UI", 10, "bold"))
        canvas.create_text(12, height - 12, anchor="sw", text=y_label, fill="#555555", font=("Segoe UI", 9))
        canvas.create_rectangle(24, 24, width - 24, height - 24, outline="#dddddd")
        points = build_canvas_polyline_points(xs, ys, width, height)
        if len(points) >= 4:
            canvas.create_line(*points, fill=color, width=2.0, smooth=False)

    def show_graphics_for_earth_guided(history) -> None:
        t = [float(sample["t"]) for sample in history]
        east = [float(sample["east_m"]) / 1000.0 for sample in history]
        north = [float(sample["north_m"]) / 1000.0 for sample in history]
        ground = [float(sample["ground_distance_m"]) / 1000.0 for sample in history]
        altitude = [float(sample["up_m"]) / 1000.0 for sample in history]
        speed = [float(sample["speed_m_s"]) for sample in history]
        pitch = [float(sample["pitch_deg"]) for sample in history]
        draw_series_chart(canvas_ground, "Trace sol est-ouest", north, east, "#0f766e", "east (km)")
        draw_series_chart(canvas_altitude, "Altitude", t, altitude, "#2563eb", "up (km)")
        draw_series_chart(canvas_speed, "Vitesse", t, speed, "#dc2626", "m/s")
        draw_series_chart(canvas_pitch, "Pitch", t, pitch, "#7c3aed", "deg")
        notebook.select(graphics_tab)

    def show_graphics_for_6dof_guided(history) -> None:
        t = [float(sample["t"]) for sample in history]
        x = [float(sample["x"]) / 1000.0 for sample in history]
        y = [float(sample["y"]) for sample in history]
        z = [float(sample["z"]) / 1000.0 for sample in history]
        speed = [float(sample["speed"]) for sample in history]
        pitch_err = [float(sample["pitch_error_deg"]) for sample in history]
        draw_series_chart(canvas_ground, "Trace sol downrange/crossrange", x, y, "#0f766e", "crossrange (m)")
        draw_series_chart(canvas_altitude, "Altitude", t, z, "#2563eb", "z (km)")
        draw_series_chart(canvas_speed, "Vitesse", t, speed, "#dc2626", "m/s")
        draw_series_chart(canvas_pitch, "Erreur pitch", t, pitch_err, "#7c3aed", "deg")
        notebook.select(graphics_tab)

    def _fill_tree(tree: ttk.Treeview, rows: list[tuple[str, ...]]) -> None:
        for item in tree.get_children():
            tree.delete(item)
        for row in rows:
            tree.insert("", "end", values=row)

    def refresh_preset_table() -> None:
        rows = filter_preset_rows(
            build_preset_comparison_rows(project_root),
            search_text=preset_search_var.get(),
            favorites_only=preset_favorites_only_var.get(),
            tag_filter=preset_filter_tag_var.get(),
        )
        formatted = [
            (
                row["name"],
                "oui" if row["favorite"] else "non",
                row["tags"],
                row["city"],
                row["country"],
                f"{row['thrust_n']:.1f}",
                f"{row['propellant_mass_kg']:.1f}",
                f"{row['burn_time_s']:.1f}",
                f"{row['theta_initial_deg']:.1f}",
                f"{row['altitude_target_m']:.1f}",
                "oui" if row["regional_earth"] else "non",
                "oui" if row["apply_design"] else "non",
            )
            for row in rows
        ]
        _fill_tree(preset_tree, formatted)

    def refresh_export_table() -> None:
        rows = build_export_comparison_rows(project_root)
        formatted = [
            (
                row["name"],
                row["city"],
                row["country"],
                f"{row['distance_km']:.1f}",
                f"{row['thrust_n']:.1f}",
                f"{row['burn_time_s']:.1f}",
                row["mission_class"],
                f"{row['delta_v_m_s']:.1f}",
            )
            for row in rows
        ]
        _fill_tree(export_tree, formatted)

    def refresh_history_table() -> None:
        records = read_mission_history(project_root, limit=50)
        formatted = [
            (
                str(record.get("type", "")),
                str(record.get("mode", "")),
                str(record.get("preset_name", "")),
                str(record.get("city", "")),
                str(record.get("country", "")),
                str(record.get("samples", "")),
                str(record.get("impact_error_3d_m", "")),
                str(record.get("mission_class", "")),
            )
            for record in reversed(records)
        ]
        _fill_tree(history_tree, formatted)

    def build_params():
        params = apply_gui_parameter_overrides(
            base_params,
            thrust_text=thrust_var.get(),
            propellant_mass_text=propellant_mass_var.get(),
            burn_time_text=burn_time_var.get(),
            theta_initial_deg_text=theta_var.get(),
            altitude_target_text=altitude_var.get(),
            t_final_text=t_final_var.get(),
        )
        if short_range_var.get():
            params = build_short_range_guided_params(params)
        return params

    def set_params_in_form(params) -> None:
        thrust_var.set(f"{params.thrust:.1f}")
        propellant_mass_var.set(f"{params.propellant_mass:.1f}")
        burn_time_var.set(f"{params.burn_time:.1f}")
        theta_var.set(f"{params.theta_initial_deg:.1f}")
        altitude_var.set(f"{params.altitude_target:.1f}")
        t_final_var.set(f"{params.t_final:.1f}")

    def set_params_dict_in_form(params_dict: dict) -> None:
        thrust_var.set(f"{float(params_dict.get('thrust', 0.0)):.1f}")
        propellant_mass_var.set(f"{float(params_dict.get('propellant_mass', 0.0)):.1f}")
        burn_time_var.set(f"{float(params_dict.get('burn_time', 0.0)):.1f}")
        theta_var.set(f"{float(params_dict.get('theta_initial_deg', 0.0)):.1f}")
        altitude_var.set(f"{float(params_dict.get('altitude_target', 0.0)):.1f}")
        t_final_var.set(f"{float(params_dict.get('t_final', 0.0)):.1f}")

    def make_profile():
        params = build_params()
        profile = build_mission_target_profile(
            city_var.get(),
            params,
            country=country_var.get() or None,
            allow_online=not offline_var.get(),
        )
        if regional_earth_var.get():
            params = build_regional_earth_guided_params(params, profile)
        guidance = build_geodesic_guidance_plan(profile, params)
        design = build_mission_design_recommendation(params, profile, guidance)
        if apply_design_var.get():
            params = apply_mission_design_recommendation(params, design)
            guidance = build_geodesic_guidance_plan(profile, params)
            design = build_mission_design_recommendation(params, profile, guidance)
        return params, profile, guidance, design

    def format_applied_params(params) -> str:
        return "\n".join(
            [
                "Parametres appliques",
                f"Thrust              : {params.thrust:.1f} N",
                f"Propellant          : {params.propellant_mass:.1f} kg",
                f"Burn time           : {params.burn_time:.1f} s",
                f"Pitch initial       : {params.theta_initial_deg:.1f} deg",
                f"Altitude cible      : {params.altitude_target:.1f} m",
                f"Tfinal              : {params.t_final:.1f} s",
            ]
        )

    def show_report():
        try:
            params, profile, guidance, design = make_profile()
            render_text(
                format_applied_params(params)
                + "\n\n"
                + format_mission_target_report(profile)
                + "\n\n"
                + format_geodesic_guidance_report(guidance)
                + "\n\n"
                + format_mission_design_report(design)
            )
        except Exception as exc:
            render_text(f"Erreur\n{exc}")

    def show_design():
        try:
            params, profile, guidance, design = make_profile()
            render_text(
                format_applied_params(params)
                + "\n\n"
                + format_mission_target_report(profile)
                + "\n\n"
                + format_geodesic_guidance_report(guidance)
                + "\n\n"
                + format_mission_design_report(design)
            )
        except Exception as exc:
            render_text(f"Erreur\n{exc}")

    def run_earth_guided():
        try:
            params, profile, guidance, design = make_profile()
            params = replace(params, show_plots=False, save_plots=False)
            history = simulate_earth_guided(params, profile, guidance)
            lines = [
                format_applied_params(params),
                "",
                format_mission_target_report(profile),
                "",
                format_geodesic_guidance_report(guidance),
                "",
                format_mission_design_report(design),
                "",
                "Resume de simulation",
            ]
            from io import StringIO
            import contextlib

            buffer = StringIO()
            with contextlib.redirect_stdout(buffer):
                summarize_earth_guided(history, profile, guidance)
            lines.append(buffer.getvalue().strip())
            lines.append("")
            lines.append(f"Echantillons          : {len(history)}")
            lines.append(f"Dossier projet        : {Path(__file__).resolve().parent.parent}")
            append_mission_history(
                project_root,
                {
                    "type": "simulation",
                    "mode": "earth_guided",
                    "preset_name": preset_name_var.get(),
                    "city": profile.target.city,
                    "country": profile.target.country,
                    "samples": len(history),
                },
            )
            show_graphics_for_earth_guided(history)
            refresh_history_table()
            render_text("\n".join(lines))
        except Exception as exc:
            render_text(f"Erreur\n{exc}")

    def run_6dof_guided():
        try:
            params, profile, guidance, design = make_profile()
            params = replace(
                params,
                impact_target=min(profile.surface_distance_m, 2000.0),
                impact_target_y=0.0,
                target_final_altitude=profile.altitude_delta_m,
                show_plots=False,
                save_plots=False,
            )
            params = build_short_range_guided_params(params)
            history = simulate_rocket_6dof_guided(params, phase_mode="auto")
            metrics = summarize_6dof_guided_metrics(history, params)
            lines = [
                format_applied_params(params),
                "",
                format_mission_target_report(profile),
                "",
                format_geodesic_guidance_report(guidance),
                "",
                format_mission_design_report(design),
                "",
                "Resume de simulation 6DOF Guided",
            ]
            from io import StringIO
            import contextlib

            buffer = StringIO()
            with contextlib.redirect_stdout(buffer):
                summarize_6dof_guided(history)
            lines.append(buffer.getvalue().strip())
            lines.append("")
            lines.append(f"Erreur 3D            : {metrics['impact_error_3d']:.1f} m")
            lines.append(f"Echantillons         : {len(history)}")
            lines.append(
                "Note                 : le mode 6DOF Guided reste local/courte portee ; "
                "la cible geographique est rabattue sur un scenario de demonstration."
            )
            append_mission_history(
                project_root,
                {
                    "type": "simulation",
                    "mode": "6dof_guided",
                    "preset_name": preset_name_var.get(),
                    "city": profile.target.city,
                    "country": profile.target.country,
                    "samples": len(history),
                    "impact_error_3d_m": round(metrics["impact_error_3d"], 3),
                },
            )
            show_graphics_for_6dof_guided(history)
            refresh_history_table()
            render_text("\n".join(lines))
        except Exception as exc:
            render_text(f"Erreur\n{exc}")

    def save_preset_action():
        try:
            params = build_params()
            payload = build_gui_preset_payload(
                city=city_var.get(),
                country=country_var.get(),
                offline_only=offline_var.get(),
                short_range=short_range_var.get(),
                regional_earth=regional_earth_var.get(),
                apply_design=apply_design_var.get(),
                favorite=favorite_var.get(),
                tags=[tag.strip() for tag in tags_var.get().split(",") if tag.strip()],
                params=params,
            )
            path = save_gui_preset(project_root, preset_name_var.get(), payload)
            append_mission_history(
                project_root,
                {
                    "type": "preset_save",
                    "preset_name": preset_name_var.get(),
                    "city": city_var.get(),
                    "country": country_var.get(),
                },
            )
            refresh_preset_table()
            refresh_history_table()
            messagebox.showinfo("Preset", f"Preset enregistre :\n{path}")
        except Exception as exc:
            messagebox.showerror("Erreur preset", str(exc))

    def load_preset_action():
        try:
            available = list_gui_presets(project_root)
            if not available:
                messagebox.showinfo("Preset", "Aucun preset enregistre pour le moment.")
                return
            payload = load_gui_preset(project_root, preset_name_var.get())
            city_var.set(str(payload.get("city", city_var.get())))
            country_var.set(str(payload.get("country", country_var.get())))
            offline_var.set(bool(payload.get("offline_only", offline_var.get())))
            short_range_var.set(bool(payload.get("short_range", short_range_var.get())))
            regional_earth_var.set(bool(payload.get("regional_earth", regional_earth_var.get())))
            apply_design_var.set(bool(payload.get("apply_design", apply_design_var.get())))
            favorite_var.set(bool(payload.get("favorite", favorite_var.get())))
            tags_var.set(", ".join(str(tag) for tag in payload.get("tags", [])))
            params_payload = payload.get("params", {})
            params = replace(
                base_params,
                thrust=float(params_payload.get("thrust", base_params.thrust)),
                propellant_mass=float(params_payload.get("propellant_mass", base_params.propellant_mass)),
                burn_time=float(params_payload.get("burn_time", base_params.burn_time)),
                theta_initial_deg=float(params_payload.get("theta_initial_deg", base_params.theta_initial_deg)),
                altitude_target=float(params_payload.get("altitude_target", base_params.altitude_target)),
                t_final=float(params_payload.get("t_final", base_params.t_final)),
            )
            set_params_in_form(params)
            show_report()
        except FileNotFoundError:
            messagebox.showerror("Erreur preset", "Preset introuvable. Verifiez le nom saisi.")
        except Exception as exc:
            messagebox.showerror("Erreur preset", str(exc))

    def delete_preset_action():
        try:
            name = preset_name_var.get().strip()
            if not name:
                messagebox.showinfo("Preset", "Saisissez d'abord un nom de preset.")
                return
            if not messagebox.askyesno("Supprimer preset", f"Supprimer le preset '{name}' ?"):
                return
            path = delete_gui_preset(project_root, name)
            append_mission_history(
                project_root,
                {
                    "type": "preset_delete",
                    "preset_name": name,
                },
            )
            refresh_preset_table()
            refresh_history_table()
            messagebox.showinfo("Preset", f"Preset supprime :\n{path}")
        except FileNotFoundError:
            messagebox.showerror("Erreur preset", "Preset introuvable. Verifiez le nom saisi.")
        except Exception as exc:
            messagebox.showerror("Erreur preset", str(exc))

    def export_mission_action():
        try:
            params, profile, guidance, design = make_profile()
            payload = build_mission_export_payload(
                city=city_var.get(),
                country=country_var.get(),
                params=params,
                mission=profile,
                guidance=guidance,
                design=design,
            )
            path = export_mission_json(project_root, preset_name_var.get(), payload)
            append_mission_history(
                project_root,
                {
                    "type": "mission_export",
                    "preset_name": preset_name_var.get(),
                    "city": profile.target.city,
                    "country": profile.target.country,
                    "mission_class": design.mission_class,
                },
            )
            refresh_export_table()
            refresh_history_table()
            messagebox.showinfo("Export mission", f"Mission exportee :\n{path}")
        except Exception as exc:
            messagebox.showerror("Erreur export", str(exc))

    def delete_export_action():
        try:
            name = preset_name_var.get().strip()
            if not name:
                messagebox.showinfo("Export", "Saisissez d'abord un nom d'export.")
                return
            if not messagebox.askyesno("Supprimer export", f"Supprimer l'export '{name}' ?"):
                return
            path = delete_exported_mission(project_root, name)
            append_mission_history(
                project_root,
                {
                    "type": "mission_export_delete",
                    "preset_name": name,
                },
            )
            refresh_export_table()
            refresh_history_table()
            messagebox.showinfo("Export", f"Export supprime :\n{path}")
        except FileNotFoundError:
            messagebox.showerror("Erreur export", "Export introuvable. Verifiez le nom saisi.")
        except Exception as exc:
            messagebox.showerror("Erreur export", str(exc))

    def open_export_action():
        try:
            available = list_exported_missions(project_root)
            if not available:
                messagebox.showinfo("Exports", "Aucun export JSON disponible pour le moment.")
                return
            payload = load_exported_mission(project_root, preset_name_var.get())
            render_text(json.dumps(payload, indent=2, ensure_ascii=True))
        except FileNotFoundError:
            available = ", ".join(list_exported_missions(project_root))
            details = f"\nDisponibles : {available}" if available else ""
            messagebox.showerror("Erreur export", "Export introuvable. Verifiez le nom saisi." + details)
        except Exception as exc:
            messagebox.showerror("Erreur export", str(exc))

    def show_history_action():
        refresh_history_table()
        notebook.select(history_tab)

    def clear_history_action():
        if not messagebox.askyesno("Historique", "Vider l'historique local des missions ?"):
            return
        try:
            clear_mission_history(project_root)
            refresh_history_table()
            render_text("Historique missions\nHistorique local vide.")
        except Exception as exc:
            messagebox.showerror("Erreur historique", str(exc))

    def compare_presets_action():
        refresh_preset_table()
        notebook.select(presets_tab)

    def compare_exports_action():
        refresh_export_table()
        notebook.select(exports_tab)

    def on_preset_selected(_event=None):
        selection = preset_tree.selection()
        if not selection:
            return
        values = preset_tree.item(selection[0], "values")
        if not values:
            return
        preset_name_var.set(str(values[0]))
        try:
            payload = load_gui_preset(project_root, str(values[0]))
            city_var.set(str(payload.get("city", city_var.get())))
            country_var.set(str(payload.get("country", country_var.get())))
            offline_var.set(bool(payload.get("offline_only", offline_var.get())))
            short_range_var.set(bool(payload.get("short_range", short_range_var.get())))
            regional_earth_var.set(bool(payload.get("regional_earth", regional_earth_var.get())))
            apply_design_var.set(bool(payload.get("apply_design", apply_design_var.get())))
            favorite_var.set(bool(payload.get("favorite", favorite_var.get())))
            tags_var.set(", ".join(str(tag) for tag in payload.get("tags", [])))
            params_payload = payload.get("params", {})
            params = replace(
                base_params,
                thrust=float(params_payload.get("thrust", base_params.thrust)),
                propellant_mass=float(params_payload.get("propellant_mass", base_params.propellant_mass)),
                burn_time=float(params_payload.get("burn_time", base_params.burn_time)),
                theta_initial_deg=float(params_payload.get("theta_initial_deg", base_params.theta_initial_deg)),
                altitude_target=float(params_payload.get("altitude_target", base_params.altitude_target)),
                t_final=float(params_payload.get("t_final", base_params.t_final)),
            )
            set_params_in_form(params)
            render_text("Preset charge\n\n" + format_json_payload(payload))
        except Exception as exc:
            render_text(f"Erreur\n{exc}")

    def on_export_selected(_event=None):
        selection = export_tree.selection()
        if not selection:
            return
        values = export_tree.item(selection[0], "values")
        if not values:
            return
        preset_name_var.set(str(values[0]))
        try:
            payload = load_exported_mission(project_root, str(values[0]))
            apply_export_payload_to_form(
                payload,
                set_city=city_var.set,
                set_country=country_var.set,
                set_params=set_params_dict_in_form,
            )
            show_report()
            render_text("Mission exportee restauree\n\n" + format_json_payload(payload))
        except Exception as exc:
            render_text(f"Erreur\n{exc}")

    def on_history_selected(_event=None):
        selection = history_tree.selection()
        if not selection:
            return
        values = history_tree.item(selection[0], "values")
        if not values:
            return
        record = {
            "type": values[0],
            "mode": values[1],
            "preset_name": values[2],
            "city": values[3],
            "country": values[4],
            "samples": values[5],
            "impact_error_3d_m": values[6],
            "mission_class": values[7],
        }
        if record["preset_name"]:
            preset_name_var.set(str(record["preset_name"]))
        restored = False
        if record["preset_name"]:
            try:
                payload = load_gui_preset(project_root, str(record["preset_name"]))
                city_var.set(str(payload.get("city", city_var.get())))
                country_var.set(str(payload.get("country", country_var.get())))
                offline_var.set(bool(payload.get("offline_only", offline_var.get())))
                short_range_var.set(bool(payload.get("short_range", short_range_var.get())))
                regional_earth_var.set(bool(payload.get("regional_earth", regional_earth_var.get())))
                apply_design_var.set(bool(payload.get("apply_design", apply_design_var.get())))
                params_payload = payload.get("params", {})
                params = replace(
                    base_params,
                    thrust=float(params_payload.get("thrust", base_params.thrust)),
                    propellant_mass=float(params_payload.get("propellant_mass", base_params.propellant_mass)),
                    burn_time=float(params_payload.get("burn_time", base_params.burn_time)),
                    theta_initial_deg=float(params_payload.get("theta_initial_deg", base_params.theta_initial_deg)),
                    altitude_target=float(params_payload.get("altitude_target", base_params.altitude_target)),
                    t_final=float(params_payload.get("t_final", base_params.t_final)),
                )
                set_params_in_form(params)
                restored = True
            except Exception:
                restored = False
        if not restored and record["preset_name"]:
            try:
                payload = load_exported_mission(project_root, str(record["preset_name"]))
                apply_export_payload_to_form(
                    payload,
                    set_city=city_var.set,
                    set_country=country_var.set,
                    set_params=set_params_dict_in_form,
                )
                restored = True
            except Exception:
                restored = False
        lines = ["Detail historique"]
        if restored:
            lines.append("Contexte restaure     : oui")
        lines.extend(f"{key}: {value}" for key, value in record.items() if value not in ("", None))
        render_text("\n".join(lines))

    buttons = ttk.Frame(frame)
    buttons.pack(fill="x", pady=(12, 0))
    ttk.Button(buttons, text="Rapport geographique", command=show_report).pack(side="left")
    ttk.Button(buttons, text="Design mission", command=show_design).pack(side="left", padx=8)
    ttk.Button(buttons, text="Simulation Earth Guided", command=run_earth_guided).pack(side="left", padx=8)
    ttk.Button(buttons, text="Simulation 6DOF Guided", command=run_6dof_guided).pack(side="left", padx=8)
    ttk.Button(buttons, text="Quitter", command=root.destroy).pack(side="right")

    storage_buttons = ttk.Frame(frame)
    storage_buttons.pack(fill="x", pady=(8, 0))
    ttk.Button(storage_buttons, text="Sauver preset", command=save_preset_action).pack(side="left")
    ttk.Button(storage_buttons, text="Charger preset", command=load_preset_action).pack(side="left", padx=8)
    ttk.Button(storage_buttons, text="Supprimer preset", command=delete_preset_action).pack(side="left", padx=8)
    ttk.Button(storage_buttons, text="Exporter mission JSON", command=export_mission_action).pack(side="left", padx=8)
    ttk.Button(storage_buttons, text="Ouvrir export JSON", command=open_export_action).pack(side="left", padx=8)
    ttk.Button(storage_buttons, text="Supprimer export", command=delete_export_action).pack(side="left", padx=8)
    ttk.Button(storage_buttons, text="Historique missions", command=show_history_action).pack(side="left", padx=8)
    ttk.Button(storage_buttons, text="Vider historique", command=clear_history_action).pack(side="left", padx=8)
    ttk.Button(storage_buttons, text="Comparer presets", command=compare_presets_action).pack(side="left", padx=8)
    ttk.Button(storage_buttons, text="Comparer exports", command=compare_exports_action).pack(side="left", padx=8)

    preset_tree.bind("<<TreeviewSelect>>", on_preset_selected)
    export_tree.bind("<<TreeviewSelect>>", on_export_selected)
    history_tree.bind("<<TreeviewSelect>>", on_history_selected)
    preset_search_var.trace_add("write", lambda *_: refresh_preset_table())
    preset_filter_tag_var.trace_add("write", lambda *_: refresh_preset_table())
    preset_favorites_only_var.trace_add("write", lambda *_: refresh_preset_table())

    refresh_preset_table()
    refresh_export_table()
    refresh_history_table()
    show_report()
    root.mainloop()
