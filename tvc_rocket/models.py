from __future__ import annotations

from dataclasses import dataclass, field
from math import pi


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(angle: float) -> float:
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle


@dataclass
class RocketParameters:
    dry_mass: float = 18.0
    propellant_mass: float = 12.0
    thrust: float = 850.0
    burn_time: float = 18.0
    thrust_ramp_up_time: float = 0.6
    thrust_tailoff_time: float = 1.2
    area_ref: float = 0.018
    cd0: float = 0.22
    cd_alpha: float = 2.8
    cd_mach_rise: float = 0.16
    transonic_mach_center: float = 1.0
    transonic_mach_width: float = 0.22
    cl_alpha: float = 1.4
    iyy: float = 7.5
    body_length: float = 2.2
    tvc_arm: float = 0.85
    pitch_damping: float = 18.0
    static_stability: float = 1.8
    max_gimbal_deg: float = 6.0
    max_gimbal_rate_deg_s: float = 18.0
    max_alpha_cmd_deg: float = 3.0
    max_alpha_cmd_high_q_deg: float = 1.2
    alpha_schedule_q_low_pa: float = 3000.0
    alpha_schedule_q_high_pa: float = 18000.0
    use_alpha_schedule: bool = False
    dt: float = 0.02
    t_final: float = 120.0
    gamma_initial_deg: float = 90.0
    theta_initial_deg: float = 90.0
    altitude_target: float = 2500.0
    impact_target: float = 800.0
    gravity_turn_start: float = 4.0
    gravity_turn_end: float = 14.0
    gamma_final_deg: float = 72.0
    guidance_gain: float = 0.65
    guidance_altitude_gain: float = 0.00015
    wind_ref_m_s: float = 0.0
    wind_gradient_m_s_per_m: float = 0.0008
    wind_decay_altitude_m: float = 2600.0
    crosswind_ref_m_s: float = 0.0
    launch_rail_time: float = 1.4
    save_plots: bool = True
    show_plots: bool = True
    plot_display_seconds: float = 4.0
    tuning_mode: str = "accurate"

    @property
    def mass_flow(self) -> float:
        return self.propellant_mass / self.burn_time

    @property
    def max_gimbal_rad(self) -> float:
        return self.max_gimbal_deg * pi / 180.0

    @property
    def max_gimbal_rate_rad_s(self) -> float:
        return self.max_gimbal_rate_deg_s * pi / 180.0

    @property
    def max_alpha_cmd_rad(self) -> float:
        return self.max_alpha_cmd_deg * pi / 180.0

    @property
    def max_alpha_cmd_high_q_rad(self) -> float:
        return self.max_alpha_cmd_high_q_deg * pi / 180.0


@dataclass
class PIDController:
    kp: float
    ki: float
    kd: float
    output_limit: float
    integral_limit: float
    derivative_filter: float = 0.85
    integral: float = 0.0
    previous_error: float = 0.0
    filtered_derivative: float = 0.0

    def update(self, error: float, dt: float, freeze_integrator: bool = False) -> float:
        if not freeze_integrator:
            self.integral += error * dt
            self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)

        raw_derivative = (error - self.previous_error) / dt if dt > 0.0 else 0.0
        self.filtered_derivative = (
            self.derivative_filter * self.filtered_derivative
            + (1.0 - self.derivative_filter) * raw_derivative
        )
        self.previous_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * self.filtered_derivative
        return clamp(output, -self.output_limit, self.output_limit)


@dataclass(frozen=True)
class ControllerGains:
    kp: float
    ki: float
    kd: float
    guidance_gain: float
    guidance_altitude_gain: float
    gamma_final_deg: float


@dataclass(frozen=True)
class TuningResult:
    gains: ControllerGains
    metrics: dict[str, float]
    score: float


@dataclass
class TVCActuator:
    max_angle: float
    max_rate: float
    angle: float = 0.0

    def update(self, command: float, dt: float) -> float:
        command = clamp(command, -self.max_angle, self.max_angle)
        max_step = self.max_rate * dt
        delta = clamp(command - self.angle, -max_step, max_step)
        self.angle = clamp(self.angle + delta, -self.max_angle, self.max_angle)
        return self.angle


@dataclass
class GuidanceState:
    last_gamma_ref: float = field(default=90.0 * pi / 180.0)


@dataclass
class RocketState:
    t: float
    x: float
    z: float
    vx: float
    vz: float
    theta: float
    q: float
    mass: float


def gains_key(gains: ControllerGains) -> tuple[float, float, float, float, float, float]:
    return (
        gains.kp,
        gains.ki,
        gains.kd,
        gains.guidance_gain,
        gains.guidance_altitude_gain,
        gains.gamma_final_deg,
    )


def default_controller_gains(params: RocketParameters) -> ControllerGains:
    return ControllerGains(
        kp=5.5,
        ki=1.2,
        kd=1.8,
        guidance_gain=params.guidance_gain,
        guidance_altitude_gain=params.guidance_altitude_gain,
        gamma_final_deg=params.gamma_final_deg,
    )
