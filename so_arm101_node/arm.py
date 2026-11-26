"""One SO-ARM101 (five STS3215 joints + gripper) on a shared Feetech bus.

A Nano carries two arms and a three-wheel base on one serial line, the same way a
LeKiwi does. Each ``SoArm101`` therefore only addresses its own six servo ids and
never owns the bus; whoever builds the robot passes the same ``FeetechBus`` to
every arm (and later to the base driver).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from so_arm101_node import calibration as calib
from so_arm101_node import feetech as ft
from so_arm101_node.calibration import GRIPPER, JOINTS, MotorCalibration


class ArmError(RuntimeError):
    pass


@dataclass(frozen=True)
class ArmConfig:
    name: str
    ids: tuple[int, ...]  # servo ids in JOINTS order
    sim: bool = False  # simulated servos even when the node runs on a real bus

    def __post_init__(self) -> None:
        if len(self.ids) != len(JOINTS) or len(set(self.ids)) != len(JOINTS):
            raise ValueError(f"arm {self.name}: needs {len(JOINTS)} distinct servo ids")

    @classmethod
    def from_first_id(cls, name: str, first_id: int, sim: bool = False) -> ArmConfig:
        return cls(name, tuple(range(first_id, first_id + len(JOINTS))), sim)


class Arm(Protocol):
    """What the controller needs from an arm: real servos (``SoArm101``) or AISLE (``AisleArm``)."""

    config: ArmConfig

    @property
    def name(self) -> str: ...
    def bounds(self) -> dict[str, tuple[float, float]]: ...
    def preflight(self) -> dict[str, bool]: ...
    def verify_calibration(self) -> None: ...
    def write_calibration(self) -> None: ...
    def configure(self) -> None: ...
    def read_joints(self) -> dict[str, float]: ...
    def read_load(self) -> dict[str, dict[str, float]] | None: ...
    def torque_enabled(self) -> bool: ...
    def enable_torque(self) -> dict[str, float]: ...
    def disable_torque(self) -> None: ...
    def write_goal(self, values: dict[str, float]) -> None: ...


class SoArm101:
    def __init__(self, bus: ft.FeetechBus, config: ArmConfig, cal: dict[str, MotorCalibration]) -> None:
        for index, joint in enumerate(JOINTS):
            cal[joint].validate(joint, config.ids[index])
        self.bus = bus
        self.config = config
        self.cal = cal
        self._id = dict(zip(JOINTS, config.ids))

    @property
    def name(self) -> str:
        return self.config.name

    def bounds(self) -> dict[str, tuple[float, float]]:
        return {j: calib.user_bounds(j, self.cal[j]) for j in JOINTS}

    # -- bring-up ----------------------------------------------------------

    def preflight(self) -> dict[str, bool]:
        """Check every servo answers as an STS3215; return each joint's torque state."""
        torque = {}
        for joint, servo_id in self._id.items():
            try:
                model = self.bus.read(servo_id, ft.MODEL_NUMBER)
            except ft.FeetechError as e:
                raise ArmError(f"arm {self.name}: {joint} (servo {servo_id}) did not answer: {e}") from e
            if model != ft.STS3215_MODEL:
                raise ArmError(f"arm {self.name}: servo {servo_id} is model {model}, not an STS3215")
            torque[joint] = bool(self.bus.read(servo_id, ft.TORQUE_ENABLE))
        return torque

    def verify_calibration(self) -> None:
        """The servo EEPROM must hold the same offsets and limits as the calibration file."""
        for joint, servo_id in self._id.items():
            c = self.cal[joint]
            actual = (
                self.bus.read_homing_offset(servo_id),
                self.bus.read(servo_id, ft.MIN_POSITION_LIMIT),
                self.bus.read(servo_id, ft.MAX_POSITION_LIMIT),
            )
            if actual != (c.homing_offset, c.range_min, c.range_max):
                raise ArmError(
                    f"arm {self.name}: {joint} EEPROM {actual} does not match the calibration "
                    f"{(c.homing_offset, c.range_min, c.range_max)}; recalibrate this arm"
                )

    def write_calibration(self) -> None:
        """Store offsets and limits in EEPROM (torque must be off)."""
        self.disable_torque()
        for joint, servo_id in self._id.items():
            c = self.cal[joint]
            self.bus.write_homing_offset(servo_id, c.homing_offset)
            self.bus.write(servo_id, ft.MIN_POSITION_LIMIT, c.range_min)
            self.bus.write(servo_id, ft.MAX_POSITION_LIMIT, c.range_max)

    def configure(self) -> None:
        """Position mode and LeRobot SO-101 follower gains; gripper torque and current capped at 50%."""
        self.disable_torque()
        for joint, servo_id in self._id.items():
            self.bus.write(servo_id, ft.RETURN_DELAY_TIME, 0)
            self.bus.write(servo_id, ft.ACCELERATION, 254)
            self.bus.write(servo_id, ft.OPERATING_MODE, 0)
            self.bus.write(servo_id, ft.P_COEFFICIENT, 16)
            self.bus.write(servo_id, ft.I_COEFFICIENT, 0)
            self.bus.write(servo_id, ft.D_COEFFICIENT, 32)
            if joint == GRIPPER:
                self.bus.write(servo_id, ft.MAX_TORQUE_LIMIT, 500)
                self.bus.write(servo_id, ft.PROTECTION_CURRENT, 250)
                self.bus.write(servo_id, ft.OVERLOAD_TORQUE, 25)

    # -- runtime -----------------------------------------------------------

    def read_raw(self) -> dict[str, int]:
        return {j: self.bus.read_position(i) for j, i in self._id.items()}

    def read_joints(self) -> dict[str, float]:
        return {j: calib.to_user(j, self.cal[j], raw) for j, raw in self.read_raw().items()}

    def read_load(self) -> dict[str, dict[str, float]]:
        """Per joint: ``load`` in signed percent of max torque and ``current_ma``."""
        load, current = {}, {}
        for j, i in self._id.items():
            load[j] = round(self.bus.read_load(i) * ft.LOAD_PERCENT_PER_UNIT, 1)
            current[j] = round(self.bus.read_current(i) * ft.CURRENT_MA_PER_UNIT, 1)
        return {"load": load, "current_ma": current}

    def torque_enabled(self) -> bool:
        return all(self.bus.read(i, ft.TORQUE_ENABLE) for i in self._id.values())

    def enable_torque(self) -> dict[str, float]:
        """Hold the current pose: goal := present position first, so enabling never jumps."""
        raw = self.read_raw()
        self.bus.sync_write(ft.GOAL_POSITION, {self._id[j]: max(0, v) for j, v in raw.items()})
        for servo_id in self._id.values():
            self.bus.write(servo_id, ft.TORQUE_ENABLE, 1)
            self.bus.write(servo_id, ft.LOCK, 1)
        return {j: calib.to_user(j, self.cal[j], v) for j, v in raw.items()}

    def disable_torque(self) -> None:
        errors = []
        for servo_id in self._id.values():
            for register, value in ((ft.TORQUE_ENABLE, 0), (ft.LOCK, 0)):
                try:
                    self.bus.write(servo_id, register, value)
                except ft.FeetechError as e:
                    errors.append(str(e))
        if errors:
            raise ArmError(f"arm {self.name}: torque disable failed: {'; '.join(errors)}")

    def write_goal(self, values: dict[str, float]) -> None:
        self.bus.sync_write(
            ft.GOAL_POSITION,
            {self._id[j]: calib.to_raw(j, self.cal[j], v) for j, v in values.items()},
        )
