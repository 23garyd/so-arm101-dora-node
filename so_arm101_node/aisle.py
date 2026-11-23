"""AISLE backend: run this node against AISLE's physics instead of Feetech servos.

The arm talks to an AISLE SO-101 scene over its dora topics (SPEC 010, TC-5), so
AISLE can evaluate the same vendor node that ships to the robot:

    joint_state   in   Float32[6] radians, wire order shoulder_pan … wrist_roll, gripper
    joint_cmd     out  Float32[6] radians, same order (gripper = jaw joint angle)
    gripper_cmd   out  Float32[1] in [0, 1]; 0 open, 1 closed
    reset_done    in   an episode reset finished; any move in flight is cancelled

The model is the official SO-101 new-calibration URDF (AISLE ADR-27): joint zero is
LeRobot's calibrated middle, so node degrees convert directly to radians. The jaw
runs from -0.174533 rad (closed, 0 %) to 1.74533 rad (open, 100 %). Commands go to
AISLE's budget-guard, which clamps position, per-step velocity and workspace.

There is no torque in simulation: ``disable_torque`` just stops commanding, and the
scene keeps its last target.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable

from so_arm101_node.arm import ArmConfig, ArmError
from so_arm101_node.calibration import GRIPPER, JOINTS

# Official new-calibration follower URDF limits (AISLE env/limits.toml, so101), radians.
URDF_LIMITS_RAD = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
}
JAW_CLOSED_RAD = -0.174533
JAW_OPEN_RAD = 1.74533
# AISLE's scene home and guard fallback: arm zero, jaw open.
AISLE_HOME = {j: 0.0 for j in JOINTS if j != GRIPPER} | {GRIPPER: 100.0}


def jaw_rad_to_percent(rad: float) -> float:
    return min(100.0, max(0.0, (rad - JAW_CLOSED_RAD) / (JAW_OPEN_RAD - JAW_CLOSED_RAD) * 100))


def percent_to_jaw_rad(percent: float) -> float:
    p = min(100.0, max(0.0, percent))
    return JAW_CLOSED_RAD + p / 100 * (JAW_OPEN_RAD - JAW_CLOSED_RAD)


def to_wire(joints: dict[str, float]) -> tuple[list[float], float]:
    """Node units → (joint_cmd radians, gripper_cmd 0 open..1 closed)."""
    cmd = [math.radians(joints[j]) for j in JOINTS if j != GRIPPER]
    cmd.append(percent_to_jaw_rad(joints[GRIPPER]))
    return cmd, 1.0 - min(100.0, max(0.0, joints[GRIPPER])) / 100


def from_wire(state: list[float]) -> dict[str, float]:
    """joint_state radians → node units."""
    out = {j: math.degrees(state[i]) for i, j in enumerate(JOINTS) if j != GRIPPER}
    out[GRIPPER] = jaw_rad_to_percent(state[JOINTS.index(GRIPPER)])
    return out


class AisleLink:
    """Latest joint_state in, joint_cmd / gripper_cmd out. Shared by the arm and the pump."""

    def __init__(self, *, stale_s: float = 1.0, clock: Callable[[], float] = time.monotonic) -> None:
        self.stale_s = stale_s
        self._clock = clock
        self._state: list[float] | None = None
        self._stamp = 0.0
        self.outbox: list[tuple[str, list[float]]] = []

    def update(self, values: list[float]) -> None:
        if len(values) != len(JOINTS):
            raise ValueError(f"joint_state must be Float32[{len(JOINTS)}], got {len(values)} values")
        self._state = [float(v) for v in values]
        self._stamp = self._clock()

    @property
    def has_state(self) -> bool:
        return self._state is not None

    def state(self) -> list[float]:
        if self._state is None:
            raise ArmError("no joint_state from AISLE yet")
        age = self._clock() - self._stamp
        if age > self.stale_s:
            raise ArmError(f"AISLE joint_state is {age:.2f} s old (limit {self.stale_s} s)")
        return self._state

    def send(self, joints: dict[str, float]) -> None:
        cmd, grip = to_wire(joints)
        self.outbox.append(("joint_cmd", cmd))
        self.outbox.append(("gripper_cmd", [grip]))

    def take_outbox(self) -> list[tuple[str, list[float]]]:
        out, self.outbox = self.outbox, []
        return out


class AisleArm:
    """The ``Arm`` interface on top of an ``AisleLink`` (no servos, no calibration)."""

    def __init__(self, link: AisleLink, config: ArmConfig) -> None:
        self.link = link
        self.config = config
        self._enabled = False
        self._last_goal: dict[str, float] | None = None

    @property
    def name(self) -> str:
        return self.config.name

    def bounds(self) -> dict[str, tuple[float, float]]:
        out = {j: (math.degrees(lo), math.degrees(hi)) for j, (lo, hi) in URDF_LIMITS_RAD.items()}
        out[GRIPPER] = (0.0, 100.0)
        return out

    def preflight(self) -> dict[str, bool]:
        self.link.state()  # raises until a fresh joint_state has arrived
        return dict.fromkeys(JOINTS, self._enabled)

    def verify_calibration(self) -> None:
        """The scene model is the calibration; nothing to check."""

    def write_calibration(self) -> None:
        """Nothing to write in simulation."""

    def configure(self) -> None:
        """Gains and limits live in the AISLE scene and budget-guard."""

    def read_joints(self) -> dict[str, float]:
        return from_wire(self.link.state())

    def torque_enabled(self) -> bool:
        return self._enabled

    def enable_torque(self) -> dict[str, float]:
        joints = self.read_joints()
        self.write_goal(joints)  # hold where the scene is
        self._enabled = True
        return joints

    def disable_torque(self) -> None:
        self._enabled = False

    def on_reset(self) -> None:
        """The scene teleported home: forget the previous goal."""
        self._last_goal = None

    def write_goal(self, values: dict[str, float]) -> None:
        goal = dict(self._last_goal or self.read_joints()) | values
        self._last_goal = goal
        self.link.send(goal)
