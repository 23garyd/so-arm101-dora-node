"""Three-wheel omni base on the same Feetech bus as the arms (Nano / LeKiwi wiring).

Three STS servos in wheel mode at 120° (ids 13 left-front 60°, 14 back 180°,
15 right-front 300° on the Nano). Kinematics and register use follow the
feetech-omni-car plugin and DoRobot Studio's native base driver:

    left  = -0.866·vx + 0.5·vy + ω
    back  =          -  vy     + ω
    right =  0.866·vx + 0.5·vy + ω      (all in wheel speed ticks)

The node speaks SI at its boundary. Linear speed uses the omni-car bench
measurement (1 tick ≈ 0.000077 m/s on 10 cm wheels); turning uses the wheel
distance from the centre (``radius_m``), which still needs measuring on a real
Nano. There is no wheel odometry: commanded speeds are not measured motion.

If one wheel would exceed ``max_ticks`` the three speeds are scaled down together,
so the direction of travel is kept and only the speed drops.
"""

from __future__ import annotations

from dataclasses import dataclass

from so_arm101_node import feetech as ft

M_PER_S_PER_TICK = 0.000077
SPEED_LIMIT_TICKS = 1500  # register limit used by omni-car


class BaseError(RuntimeError):
    pass


@dataclass(frozen=True)
class BaseConfig:
    ids: tuple[int, int, int] = (13, 14, 15)
    max_ticks: int = 600
    radius_m: float = 0.12
    m_per_s_per_tick: float = M_PER_S_PER_TICK

    def __post_init__(self) -> None:
        if len(self.ids) != 3 or len(set(self.ids)) != 3:
            raise ValueError("the omni base needs three distinct wheel servo ids")
        if not 0 < self.max_ticks <= SPEED_LIMIT_TICKS:
            raise ValueError(f"max_ticks must be in (0, {SPEED_LIMIT_TICKS}]")
        if self.radius_m <= 0:
            raise ValueError("radius_m must be positive")

    @property
    def max_linear_m_s(self) -> float:
        return self.max_ticks * self.m_per_s_per_tick


def wheel_ticks(vx: float, vy: float, omega: float, config: BaseConfig) -> tuple[int, int, int]:
    """Body velocity (m/s, m/s, rad/s) → wheel speed ticks, scaled to keep direction."""
    k = config.m_per_s_per_tick
    tx, ty, tw = vx / k, vy / k, omega * config.radius_m / k
    wheels = (-0.866 * tx + 0.5 * ty + tw, -ty + tw, 0.866 * tx + 0.5 * ty + tw)
    peak = max(abs(w) for w in wheels)
    scale = config.max_ticks / peak if peak > config.max_ticks else 1.0
    return tuple(round(w * scale) for w in wheels)  # type: ignore[return-value]


class OmniBase:
    def __init__(self, bus: ft.FeetechBus, config: BaseConfig | None = None) -> None:
        self.bus = bus
        self.config = config or BaseConfig()
        self.enabled = False
        self.last_ticks: tuple[int, int, int] = (0, 0, 0)

    def preflight(self) -> None:
        for servo_id in self.config.ids:
            try:
                model = self.bus.read(servo_id, ft.MODEL_NUMBER)
            except ft.FeetechError as e:
                raise BaseError(f"base wheel servo {servo_id} did not answer: {e}") from e
            if model != ft.STS3215_MODEL:
                raise BaseError(f"base wheel servo {servo_id} is model {model}, not an STS3215")

    def configure(self) -> None:
        """Stop, then switch every wheel to wheel (constant speed) mode."""
        self.stop()
        for servo_id in self.config.ids:
            self.bus.write(servo_id, ft.LOCK, 0)
            self.bus.write(servo_id, ft.OPERATING_MODE, 1)
            self.bus.write(servo_id, ft.LOCK, 1)

    def drive(self, ticks: tuple[int, int, int]) -> None:
        if not self.enabled:
            for servo_id in self.config.ids:  # zero speed before torque, so enabling never lurches
                self.bus.write(servo_id, ft.GOAL_SPEED, 0)
            for servo_id in self.config.ids:
                self.bus.write(servo_id, ft.TORQUE_ENABLE, 1)
            self.enabled = True
        for servo_id, value in zip(self.config.ids, ticks):
            self.bus.write(servo_id, ft.GOAL_SPEED, ft.encode_sign_magnitude(value, 15))
        self.last_ticks = ticks

    def stop(self) -> None:
        """Zero every wheel, then drop wheel torque (a stopped base does not need to hold)."""
        errors = []
        for register in (ft.GOAL_SPEED, ft.TORQUE_ENABLE):
            for servo_id in self.config.ids:
                try:
                    self.bus.write(servo_id, register, 0)
                except ft.FeetechError as e:
                    errors.append(str(e))
        if errors:  # do not claim the wheels stopped: last_ticks keeps the last command
            raise BaseError(f"base stop failed: {'; '.join(errors)}")
        self.enabled = False
        self.last_ticks = (0, 0, 0)
