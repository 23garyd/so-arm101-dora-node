"""Motion limits and trajectories for one arm, in user units (degrees, gripper percent).

Defaults follow the SO-101 safety rules shipped with Google DeepMind's robotics
Antigravity plugin (Apache-2.0, rule R5): 30 °/s cosine-eased moves with a 90 °/s
ceiling, and a 5° margin inside the calibrated range that never blocks a move
back to the rest pose.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from so_arm101_node.calibration import BODY_JOINTS, GRIPPER, JOINTS

# Rest pose from the SO-101 bring-up reference (degrees; gripper percent).
DEFAULT_REST = {
    "shoulder_pan": 0.0, "shoulder_lift": -88.5, "elbow_flex": 92.0,
    "wrist_flex": 64.5, "wrist_roll": 0.0, "gripper": 12.0,
}
DEFAULT_HOME = {j: 0.0 for j in BODY_JOINTS} | {GRIPPER: 50.0}

HARD_MAX_SPEED_DEG_S = 90.0


@dataclass(frozen=True)
class Limits:
    max_speed_deg_s: float = 30.0
    gripper_speed_pct_s: float = 60.0
    joint_margin_deg: float = 5.0
    max_tracking_error_deg: float = 20.0
    settle_tolerance_deg: float = 2.0
    settle_timeout_s: float = 1.5
    min_duration_s: float = 0.2

    def __post_init__(self) -> None:
        if not 0 < self.max_speed_deg_s <= HARD_MAX_SPEED_DEG_S:
            raise ValueError(f"max_speed_deg_s must be in (0, {HARD_MAX_SPEED_DEG_S}]")


class LimitError(ValueError):
    pass


def check_target(
    target: dict[str, float],
    bounds: dict[str, tuple[float, float]],
    limits: Limits,
    rest: dict[str, float],
) -> None:
    """Reject targets outside the calibrated range minus the margin.

    A joint target equal to the rest pose is allowed anywhere inside the raw range,
    so the arm can always be sent home even if rest sits inside the margin.
    """
    for joint, value in target.items():
        if joint not in JOINTS:
            raise LimitError(f"unknown joint {joint!r}; expected one of {list(JOINTS)}")
        if not math.isfinite(value):
            raise LimitError(f"{joint}: target must be a finite number")
        lo, hi = bounds[joint]
        if not lo <= value <= hi:
            raise LimitError(f"{joint}: {value:.1f} is outside the calibrated range [{lo:.1f}, {hi:.1f}]")
        if joint == GRIPPER:
            continue
        margin = limits.joint_margin_deg
        if lo + margin <= value <= hi - margin:
            continue
        if joint in rest and abs(value - rest[joint]) < 1e-6:
            continue
        raise LimitError(
            f"{joint}: {value:.1f} is within {margin:.0f}° of the calibrated limit "
            f"[{lo:.1f}, {hi:.1f}]"
        )


@dataclass
class Trajectory:
    """Cosine-eased interpolation from ``start`` to ``goal`` over ``duration_s``."""

    start: dict[str, float]
    goal: dict[str, float]
    duration_s: float
    started_at: float
    finished_at: float | None = field(default=None)

    @classmethod
    def plan(
        cls,
        start: dict[str, float],
        goal: dict[str, float],
        limits: Limits,
        now: float,
        speed_deg_s: float | None = None,
    ) -> Trajectory:
        speed = limits.max_speed_deg_s if speed_deg_s is None else min(speed_deg_s, limits.max_speed_deg_s)
        if speed <= 0:
            raise LimitError("speed must be positive")
        full_goal = dict(start) | goal
        # A cosine ease peaks at pi/2 times the mean speed; size the duration on the peak.
        duration = limits.min_duration_s
        for joint, value in goal.items():
            delta = abs(value - start[joint])
            joint_speed = limits.gripper_speed_pct_s if joint == GRIPPER else speed
            duration = max(duration, delta * math.pi / 2 / joint_speed)
        return cls(start=dict(start), goal=full_goal, duration_s=duration, started_at=now)

    def sample(self, now: float) -> dict[str, float]:
        t = min(1.0, max(0.0, (now - self.started_at) / self.duration_s))
        s = (1 - math.cos(math.pi * t)) / 2
        return {j: self.start[j] + (self.goal[j] - self.start[j]) * s for j in self.goal}

    def done_commanding(self, now: float) -> bool:
        return now - self.started_at >= self.duration_s
