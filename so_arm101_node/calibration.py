"""LeRobot-compatible SO-ARM101 calibration and unit conversion.

A calibration file maps joint names to ``{id, drive_mode, homing_offset, range_min,
range_max}``, exactly as LeRobot writes it under
``~/.cache/huggingface/lerobot/calibration/robots/so_follower/<id>.json`` (``so101_follower/`` before
LeRobot 0.4). Files
from ``lerobot-calibrate``, DoRobot Studio and this package are interchangeable.
LeKiwi-style prefixed names (``arm_shoulder_pan``) are accepted too.

Units at the node boundary: body joints in degrees around the calibrated middle
(LeRobot ``MotorNormMode.DEGREES``), the gripper in percent open (``RANGE_0_100``).
As in LeRobot, ``drive_mode`` inverts only the percent-mode gripper.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

from so_arm101_node.feetech import RESOLUTION

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
BODY_JOINTS = JOINTS[:5]
GRIPPER = "gripper"
_MAX_RES = RESOLUTION - 1


class CalibrationError(ValueError):
    pass


@dataclass(frozen=True)
class MotorCalibration:
    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int

    def validate(self, joint: str, expected_id: int | None = None) -> None:
        if expected_id is not None and self.id != expected_id:
            raise CalibrationError(f"{joint}: calibration is for servo {self.id}, expected {expected_id}")
        if self.drive_mode not in (0, 1):
            raise CalibrationError(f"{joint}: drive_mode must be 0 or 1")
        if abs(self.homing_offset) > 2047:
            raise CalibrationError(f"{joint}: homing_offset outside ±2047")
        if not 0 <= self.range_min < self.range_max <= _MAX_RES:
            raise CalibrationError(f"{joint}: range must satisfy 0 <= min < max <= {_MAX_RES}")

    @property
    def mid(self) -> float:
        return (self.range_min + self.range_max) / 2


def to_user(joint: str, cal: MotorCalibration, raw: int) -> float:
    """Raw present position → degrees (body) or percent open (gripper)."""
    if joint == GRIPPER:
        bounded = min(cal.range_max, max(cal.range_min, raw))
        pct = (bounded - cal.range_min) / (cal.range_max - cal.range_min) * 100
        return 100 - pct if cal.drive_mode else pct
    return (raw - cal.mid) * 360 / _MAX_RES


def to_raw(joint: str, cal: MotorCalibration, value: float) -> int:
    """Degrees (body) or percent (gripper) → raw goal position, clamped to the calibrated range."""
    if joint == GRIPPER:
        pct = 100 - value if cal.drive_mode else value
        pct = min(100.0, max(0.0, pct))
        raw = int(pct / 100 * (cal.range_max - cal.range_min) + cal.range_min)
    else:
        raw = int(value * _MAX_RES / 360 + cal.mid)
    return min(cal.range_max, max(cal.range_min, raw))


def user_bounds(joint: str, cal: MotorCalibration) -> tuple[float, float]:
    """The calibrated range in user units (low, high)."""
    if joint == GRIPPER:
        return 0.0, 100.0
    return to_user(joint, cal, cal.range_min), to_user(joint, cal, cal.range_max)


def _match_joint(key: str) -> str | None:
    for joint in JOINTS:
        if key == joint or key.endswith("_" + joint):
            return joint
    return None


def load(path: Path, ids: tuple[int, ...] | None = None) -> dict[str, MotorCalibration]:
    """Load one arm's calibration; ``ids`` (in joint order) are checked when given."""
    try:
        raw = json.loads(Path(path).read_text())
    except FileNotFoundError as e:
        raise CalibrationError(f"calibration file not found: {path}") from e
    except json.JSONDecodeError as e:
        raise CalibrationError(f"calibration file is not JSON: {path}: {e}") from e
    cals: dict[str, MotorCalibration] = {}
    for key, value in raw.items():
        joint = _match_joint(key)
        if joint is None:
            continue
        if joint in cals:
            raise CalibrationError(f"{path}: joint {joint} appears twice")
        try:
            cals[joint] = MotorCalibration(**{k: int(value[k]) for k in (
                "id", "drive_mode", "homing_offset", "range_min", "range_max")})
        except (KeyError, TypeError, ValueError) as e:
            raise CalibrationError(f"{path}: bad entry for {key}: {e}") from e
    missing = [j for j in JOINTS if j not in cals]
    if missing:
        raise CalibrationError(f"{path}: missing joints {missing}")
    for index, joint in enumerate(JOINTS):
        cals[joint].validate(joint, ids[index] if ids else None)
    return cals


def save(path: Path, cals: dict[str, MotorCalibration]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps({j: asdict(cals[j]) for j in JOINTS}, indent=4) + "\n")
    tmp.replace(path)


def default_sim(ids: tuple[int, ...]) -> dict[str, MotorCalibration]:
    """A plausible calibration for simulated servos: centred, ±150° body, full-turn wrist roll."""
    cals = {}
    for joint, servo_id in zip(JOINTS, ids):
        if joint == "wrist_roll":
            lo, hi = 0, _MAX_RES
        elif joint == GRIPPER:
            lo, hi = 2000, 3400
        else:
            lo, hi = 2048 - 1700, 2048 + 1700
        cals[joint] = MotorCalibration(servo_id, 0, 0, lo, hi)
    return cals
