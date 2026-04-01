from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

from .geodesic_guidance import build_geodesic_guidance_plan, format_geodesic_guidance_report
from .plotting_earth_guided import summarize_earth_guided
from .simulation_earth_guided import build_regional_earth_guided_params, simulate_earth_guided
from .targeting import build_mission_target_profile, format_mission_target_report


def launch_gui(base_params) -> None:
    root = tk.Tk()
    root.title("TVC Rocket Nimy - Ciblage geographique")
    root.geometry("980x720")

    city_var = tk.StringVar(value="Paris")
    country_var = tk.StringVar(value="France")
    offline_var = tk.BooleanVar(value=True)
    short_range_var = tk.BooleanVar(value=False)
    regional_earth_var = tk.BooleanVar(value=True)

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

    controls.columnconfigure(1, weight=1)
    controls.columnconfigure(3, weight=1)

    output = ScrolledText(frame, wrap="word", font=("Consolas", 10))
    output.pack(fill="both", expand=True, pady=(12, 0))

    def render_text(text: str) -> None:
        output.configure(state="normal")
        output.delete("1.0", "end")
        output.insert("1.0", text)
        output.configure(state="disabled")

    def build_params():
        params = base_params
        if short_range_var.get():
            params = replace(
                params,
                thrust=min(params.thrust, 350.0),
                burn_time=min(params.burn_time, 6.5),
                propellant_mass=min(params.propellant_mass, 3.2),
                theta_initial_deg=min(params.theta_initial_deg, 52.0),
                altitude_target=min(params.altitude_target, 180.0),
            )
        return params

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
        return params, profile, guidance

    def show_report():
        try:
            _, profile, guidance = make_profile()
            render_text(format_mission_target_report(profile) + "\n\n" + format_geodesic_guidance_report(guidance))
        except Exception as exc:
            render_text(f"Erreur\n{exc}")

    def run_earth_guided():
        try:
            params, profile, guidance = make_profile()
            params = replace(params, show_plots=False, save_plots=False)
            history = simulate_earth_guided(params, profile, guidance)
            lines = [
                format_mission_target_report(profile),
                "",
                format_geodesic_guidance_report(guidance),
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
            render_text("\n".join(lines))
        except Exception as exc:
            render_text(f"Erreur\n{exc}")

    buttons = ttk.Frame(frame)
    buttons.pack(fill="x", pady=(12, 0))
    ttk.Button(buttons, text="Rapport geographique", command=show_report).pack(side="left")
    ttk.Button(buttons, text="Simulation Earth Guided", command=run_earth_guided).pack(side="left", padx=8)
    ttk.Button(buttons, text="Quitter", command=root.destroy).pack(side="right")

    show_report()
    root.mainloop()
