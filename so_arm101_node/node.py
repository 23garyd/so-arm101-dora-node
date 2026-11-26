"""SPEC-VENDOR-NODE-V1 controller for one or more SO-ARM101 arms on a shared bus.

The controller is free of dora: ``handle()`` answers a cmd_request now (a dict) or
later (``None``; the answer comes out of a later ``tick()``), and ``tick()``
advances motions, enforces the tracking guard and the heartbeat watchdog, and
returns finished responses plus safety events. ``runtime.py`` wires it to dora.

Safety behaviour:
- Motion needs torque enabled first (``enable_torque`` holds the current pose).
- ``robot.estop`` and the watchdog cancel motion and hold the current pose
  (``ESTOP_ACTION=torque_off`` drops torque instead). Motion stays refused until
  ``vendor.lerobot.so101.clear_estop``.
- ``disable_torque`` refuses unless the arm is near its rest pose (``force`` overrides),
  because a limp arm falls under gravity.
- A body joint lagging its commanded position by more than the tracking limit
  (a collision or stall) aborts the move and holds.
- A move that does not settle on its goal holds where it stopped (``NOT_REACHED``)
  instead of pushing on toward the goal.
- A move whose gripper TCP would dip below the floor (``TCP_Z_FLOOR_M``, forward
  kinematics on the official URDF) is refused before anything moves.
- ``plan_joint_state`` / ``plan_named`` run the same checks as ``move_to_joint_state`` /
  ``move_to_named`` and return the plan; they never queue a move or write to the servos.
"""

from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable, Collection, Mapping
from dataclasses import dataclass, field
from typing import Any

from so_arm101_node import feetech as ft
from so_arm101_node._envelope import CmdRequest, cmd_response
from so_arm101_node.arm import Arm, ArmError
from so_arm101_node.base import BaseError, OmniBase, wheel_ticks
from so_arm101_node.calibration import BODY_JOINTS, GRIPPER, JOINTS
from so_arm101_node.kinematics import tcp_position
from so_arm101_node.motion import (
    DEFAULT_HOME,
    DEFAULT_REST,
    DEFAULT_TCP_FLOOR_M,
    LimitError,
    Limits,
    Trajectory,
    check_target,
    check_tcp_floor,
    lowest_tcp,
)

logger = logging.getLogger(__name__)

VENDOR = "lerobot"
MODEL = "so101"
NODE_VERSION = "0.1.0"
NS = "vendor.lerobot.so101"

VERB_TIERS: dict[str, str] = {
    "robot.heartbeat": "observe",
    "robot.get_capabilities": "observe",
    "robot.estop": "emergency_override",
    f"{NS}.stop": "emergency_override",
    f"{NS}.get_joint_state": "observe",
    f"{NS}.clear_estop": "safe_motion",
    f"{NS}.enable_torque": "safe_motion",
    f"{NS}.disable_torque": "safe_motion",
    f"{NS}.move_to_joint_state": "safe_motion",
    f"{NS}.move_to_named": "safe_motion",
    f"{NS}.plan_joint_state": "observe",
    f"{NS}.plan_named": "observe",
    f"{NS}.gripper.set": "safe_motion",
    f"{NS}.gripper.open": "safe_motion",
    f"{NS}.gripper.close": "safe_motion",
}
BASE_NS = "vendor.dora_nav.base"
BASE_VERB_TIERS: dict[str, str] = {
    f"{BASE_NS}.set_velocity": "safe_motion",
    f"{BASE_NS}.stop": "emergency_override",
}
MAX_BASE_DURATION_S = 10.0

MOTION_VERBS = frozenset({
    f"{NS}.move_to_joint_state", f"{NS}.move_to_named",
    f"{NS}.gripper.set", f"{NS}.gripper.open", f"{NS}.gripper.close",
})

GRIPPER_OPEN = 100.0
GRIPPER_CLOSED = 0.0
GRIPPER_TOLERANCE = 3.0
REST_TOLERANCE_DEG = 10.0


@dataclass
class _Motion:
    request: CmdRequest
    trajectory: Trajectory
    last_command: dict[str, float]
    settling_since: float | None = None


@dataclass
class _BaseCommand:
    velocity: tuple[float, float, float]
    ticks: tuple[int, int, int]
    expires_at: float
    request: CmdRequest | None  # set when the caller asked for a fixed duration


@dataclass
class TickResult:
    responses: list[dict[str, Any]] = field(default_factory=list)
    safety_events: list[dict[str, Any]] = field(default_factory=list)


class _Refusal(Exception):
    def __init__(self, code: str, msg: str) -> None:
        super().__init__(msg)
        self.code = code
        self.msg = msg


class So101Controller:
    def __init__(
        self,
        arms: Mapping[str, Arm],
        *,
        robot_id: str = "so101",
        limits: Limits | None = None,
        rest: dict[str, float] | None = None,
        home: dict[str, float] | None = None,
        heartbeat_timeout_ms: int = 1000,
        estop_action: str = "hold",
        base: OmniBase | None = None,
        base_command_timeout_s: float = 0.5,
        tcp_floor_m: float | None = DEFAULT_TCP_FLOOR_M,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not arms:
            raise ValueError("at least one arm is required")
        if estop_action not in ("hold", "torque_off"):
            raise ValueError("estop_action must be 'hold' or 'torque_off'")
        self.arms: dict[str, Arm] = dict(arms)
        self.base = base
        self.base_command_timeout_s = base_command_timeout_s
        self.tcp_floor_m = tcp_floor_m
        self._base_cmd: _BaseCommand | None = None
        self.robot_id = robot_id
        self.limits = limits or Limits()
        self.named_poses = {"rest": dict(rest or DEFAULT_REST), "home": dict(home or DEFAULT_HOME)}
        self.heartbeat_timeout_ms = heartbeat_timeout_ms
        self.estop_action = estop_action
        self._clock = clock
        self._motions: dict[str, _Motion] = {}
        self._cancelled: list[dict[str, Any]] = []
        self._request_events: list[dict[str, Any]] = []
        self._torque: dict[str, bool] = {name: False for name in arms}
        self._last_joints: dict[str, dict[str, float]] = {}
        self._last_beat = clock()
        self._watchdog_fired = False
        self.estopped = False
        self.estop_reason: str | None = None
        self._handlers: dict[str, Callable[[CmdRequest], dict[str, Any] | None]] = {
            "robot.heartbeat": self._heartbeat,
            "robot.get_capabilities": lambda r: self._ok(r, self.capabilities_advert()),
            "robot.estop": self._estop,
            f"{NS}.stop": self._stop,
            f"{NS}.get_joint_state": self._get_joint_state,
            f"{NS}.clear_estop": self._clear_estop,
            f"{NS}.enable_torque": self._enable_torque,
            f"{NS}.disable_torque": self._disable_torque,
            f"{NS}.move_to_joint_state": self._move_to_joint_state,
            f"{NS}.move_to_named": self._move_to_named,
            f"{NS}.plan_joint_state": self._plan_joint_state,
            f"{NS}.plan_named": self._plan_named,
            f"{NS}.gripper.set": self._gripper_set,
            f"{NS}.gripper.open": lambda r: self._gripper_to(r, GRIPPER_OPEN),
            f"{NS}.gripper.close": lambda r: self._gripper_to(r, GRIPPER_CLOSED),
        }
        self.verb_tiers = dict(VERB_TIERS)
        if base is not None:
            self._handlers[f"{BASE_NS}.set_velocity"] = self._base_set_velocity
            self._handlers[f"{BASE_NS}.stop"] = self._base_stop
            self.verb_tiers |= BASE_VERB_TIERS
        assert set(self._handlers) == set(self.verb_tiers)

    # =================================================================
    # Startup
    # =================================================================

    def bring_up(self, *, write_calibration: bool | Collection[str] = False) -> list[str]:
        """Preflight every arm (and the base); configure the ones that are limp. Returns warnings.

        ``write_calibration``: True for every arm, or the names of the arms whose
        calibration is written to EEPROM (simulated arms); the others are verified.
        """
        warnings = []
        writes = set(self.arms) if write_calibration is True else set(write_calibration or ())
        for name, arm in self.arms.items():
            torque = arm.preflight()
            if name in writes:
                if any(torque.values()):
                    raise ArmError(f"arm {name}: refuse to write calibration with torque on")
                arm.write_calibration()
            else:
                arm.verify_calibration()
            if any(torque.values()):
                self._torque[name] = True
                warnings.append(f"arm {name}: torque already on; holding and skipping configure")
            else:
                arm.configure()
            self._last_joints[name] = arm.read_joints()
        if self.base is not None:
            self.base.preflight()
            self.base.configure()
        return warnings

    # =================================================================
    # Requests
    # =================================================================

    def handle(self, request: CmdRequest) -> dict[str, Any] | None:
        self._last_beat = self._clock()
        self._watchdog_fired = False
        handler = self._handlers.get(request.verb)
        if handler is None:
            return cmd_response(request, ok=False, code="INVALID_PARAMS", msg=f"unknown verb: {request.verb}")
        try:
            return handler(request)
        except _Refusal as e:
            return cmd_response(request, ok=False, code=e.code, msg=e.msg)
        except LimitError as e:
            return cmd_response(request, ok=False, code="LIMIT_VIOLATION", msg=str(e))
        except (ArmError, BaseError, ft.FeetechError) as e:
            logger.exception("hardware error on %s", request.verb)
            return cmd_response(request, ok=False, code="HARDWARE_ERROR", msg=str(e))

    def _ok(self, request: CmdRequest, data: dict[str, Any] | None = None) -> dict[str, Any]:
        return cmd_response(request, ok=True, code="0", data=data)

    def _arm_names(self, request: CmdRequest, *, required: bool) -> list[str]:
        name = request.params.get("arm")
        if name is None:
            if required and len(self.arms) > 1:
                raise _Refusal("INVALID_PARAMS", f"choose an arm: {sorted(self.arms)}")
            return list(self.arms) if not required else [next(iter(self.arms))]
        if name not in self.arms:
            raise _Refusal("INVALID_PARAMS", f"unknown arm {name!r}; available: {sorted(self.arms)}")
        return [name]

    def _heartbeat(self, request: CmdRequest) -> dict[str, Any]:
        return self._ok(request, {"deadline_ms": self.heartbeat_timeout_ms})

    def _get_joint_state(self, request: CmdRequest) -> dict[str, Any]:
        data = {}
        for name in self._arm_names(request, required=False):
            joints = self.arms[name].read_joints()
            self._last_joints[name] = joints
            data[name] = {"joints": joints, "torque": self._torque[name], "moving": name in self._motions}
        return self._ok(request, {"arms": data, "estopped": self.estopped})

    def _estop(self, request: CmdRequest) -> dict[str, Any]:
        reason = str(request.params.get("reason", "estop"))
        events = self._engage_estop(reason, kind="estop_engaged")
        self._request_events += events
        return self._ok(request, {"action": self.estop_action, "events": len(events)})

    def _clear_estop(self, request: CmdRequest) -> dict[str, Any]:
        self.estopped = False
        self.estop_reason = None
        return self._ok(request)

    def _stop(self, request: CmdRequest) -> dict[str, Any]:
        stopped = []
        for name in self._arm_names(request, required=False):
            if self._cancel_motion(name, "PREEMPTED", "stopped by vendor.lerobot.so101.stop"):
                stopped.append(name)
        return self._ok(request, {"stopped": stopped})

    def _enable_torque(self, request: CmdRequest) -> dict[str, Any]:
        self._refuse_if_estopped()
        data = {}
        for name in self._arm_names(request, required=False):
            data[name] = self.arms[name].enable_torque()
            self._torque[name] = True
            self._last_joints[name] = data[name]
        return self._ok(request, {"holding": data})

    def _disable_torque(self, request: CmdRequest) -> dict[str, Any]:
        force = bool(request.params.get("force", False))
        names = self._arm_names(request, required=False)
        for name in names:
            if name in self._motions:
                raise _Refusal("CONTROLLER_BUSY", f"arm {name} is moving; stop it first")
            if not force:
                joints = self.arms[name].read_joints()
                rest = self.named_poses["rest"]
                off = {j: round(joints[j] - rest[j], 1) for j in BODY_JOINTS
                       if abs(joints[j] - rest[j]) > REST_TOLERANCE_DEG}
                if off:
                    raise _Refusal(
                        "NOT_AT_REST",
                        f"arm {name} is not at rest (offsets {off}); move_to_named rest first, "
                        "or support the arm and pass force=true",
                    )
        for name in names:
            self.arms[name].disable_torque()
            self._torque[name] = False
        return self._ok(request, {"limp": names})

    def _base_set_velocity(self, request: CmdRequest) -> dict[str, Any] | None:
        """Body velocity in m/s, m/s, rad/s. Without ``duration_s`` it lapses after the
        command timeout unless repeated (a deadman); with it, the reply comes when done."""
        self._refuse_if_estopped()
        assert self.base is not None
        try:
            vx, vy, omega = (float(request.params.get(k, 0.0)) for k in ("vx", "vy", "omega"))
            duration = request.params.get("duration_s")
            duration = None if duration is None else float(duration)
        except (TypeError, ValueError) as e:
            raise _Refusal("INVALID_PARAMS", f"vx, vy, omega and duration_s must be numbers: {e}") from e
        if not all(map(math.isfinite, (vx, vy, omega))):
            raise _Refusal("INVALID_PARAMS", "vx, vy and omega must be finite")
        if duration is not None and not 0 < duration <= MAX_BASE_DURATION_S:
            raise _Refusal("INVALID_PARAMS", f"duration_s must be in (0, {MAX_BASE_DURATION_S}]")
        self._finish_base_request("PREEMPTED", "replaced by a newer set_velocity")
        if vx == vy == omega == 0.0:
            self.base.stop()
            self._base_cmd = None
            return self._ok(request, {"ticks": [0, 0, 0]})
        ticks = wheel_ticks(vx, vy, omega, self.base.config)
        self.base.drive(ticks)
        now = self._clock()
        expires = now + (duration if duration is not None else self.base_command_timeout_s)
        self._base_cmd = _BaseCommand((vx, vy, omega), ticks, expires, request if duration else None)
        if duration is not None:
            return None
        return self._ok(request, {"ticks": list(ticks), "expires_in_s": self.base_command_timeout_s})

    def _base_stop(self, request: CmdRequest) -> dict[str, Any]:
        assert self.base is not None
        self._finish_base_request("PREEMPTED", f"stopped by {BASE_NS}.stop")
        self.base.stop()
        self._base_cmd = None
        return self._ok(request)

    def _finish_base_request(self, code: str, msg: str) -> None:
        """Answer a pending fixed-duration base request that is being cut short."""
        if self._base_cmd is not None and self._base_cmd.request is not None:
            self._cancelled.append(cmd_response(self._base_cmd.request, ok=False, code=code, msg=msg))
            self._base_cmd.request = None

    def _refuse_if_estopped(self) -> None:
        if self.estopped:
            raise _Refusal("ESTOPPED", f"estopped ({self.estop_reason}); call {NS}.clear_estop first")

    def _check_and_plan(self, name: str, target: dict[str, float], speed: float | None,
                        *, planning: bool) -> tuple[dict[str, float], Trajectory, list[str]]:
        """The validation and planning shared by the move verbs and the plan verbs.

        For a move, an estopped or limp arm is refused. For a plan, those become
        ``blockers`` and planning goes on, so a move can be checked before torque is
        enabled. Both are refused while the arm is moving (the start pose is not settled)
        and both run the same limit and floor checks, so they fail with the same errors.
        """
        blockers = []
        if self.estopped:
            if not planning:
                self._refuse_if_estopped()
            blockers.append("ESTOPPED")
        if name in self._motions:
            raise _Refusal("CONTROLLER_BUSY", f"arm {name} is already moving")
        if not self._torque[name]:
            if not planning:
                raise _Refusal("NOT_ENABLED", f"arm {name}: call {NS}.enable_torque first")
            blockers.append("NOT_ENABLED")
        arm = self.arms[name]
        check_target(target, arm.bounds(), self.limits, self.named_poses["rest"])
        start = arm.read_joints()
        if self.tcp_floor_m is not None:
            check_tcp_floor(start, target, self.tcp_floor_m, self.named_poses["rest"])
        trajectory = Trajectory.plan(start, target, self.limits, self._clock(), speed)
        return start, trajectory, blockers

    def _start_motion(self, request: CmdRequest, name: str, target: dict[str, float],
                      speed: float | None = None) -> None:
        start, trajectory, _ = self._check_and_plan(name, target, speed, planning=False)
        self._motions[name] = _Motion(request, trajectory, last_command=dict(start))

    def _plan(self, request: CmdRequest, name: str, target: dict[str, float],
              speed: float | None = None) -> dict[str, Any]:
        """Answer with the plan the matching move would execute; nothing is queued or written."""
        start, trajectory, blockers = self._check_and_plan(name, target, speed, planning=True)
        lowest, at = lowest_tcp(start, target)
        return self._ok(request, {
            "arm": name,
            "start": start,
            "goal": trajectory.goal,
            "duration_s": round(trajectory.duration_s, 3),
            "tcp_z_m": {
                "start": round(tcp_position(start)[2], 4),
                "goal": round(tcp_position(trajectory.goal)[2], 4),
                "lowest": round(lowest, 4),
                "lowest_at": round(at, 3),
            },
            "tcp_floor_m": self.tcp_floor_m,
            "blockers": blockers,
        })

    @staticmethod
    def _refuse_dry_run(request: CmdRequest) -> None:
        # Never let a planning request turn into a real move by being ignored.
        if "dry_run" in request.params:
            raise _Refusal("INVALID_PARAMS",
                           f"move verbs take no dry_run; use {NS}.plan_joint_state or {NS}.plan_named")

    def _joint_target(self, request: CmdRequest) -> tuple[str, dict[str, float], float | None]:
        (name,) = self._arm_names(request, required=True)
        joints = request.params.get("joints")
        if isinstance(joints, list):
            if len(joints) not in (len(BODY_JOINTS), len(JOINTS)):
                raise _Refusal("INVALID_PARAMS", f"joints list needs {len(BODY_JOINTS)} or {len(JOINTS)} values")
            joints = dict(zip(JOINTS, joints))
        if not isinstance(joints, dict) or not joints:
            raise _Refusal("INVALID_PARAMS", f"joints must map names {list(JOINTS)} to values")
        try:
            target = {str(k): float(v) for k, v in joints.items()}
        except (TypeError, ValueError) as e:
            raise _Refusal("INVALID_PARAMS", f"joint values must be numbers: {e}") from e
        speed = request.params.get("speed_deg_s")
        try:
            return name, target, None if speed is None else float(speed)
        except (TypeError, ValueError) as e:
            raise _Refusal("INVALID_PARAMS", f"speed_deg_s must be a number: {e}") from e

    def _named_target(self, request: CmdRequest) -> tuple[str, dict[str, float]]:
        (name,) = self._arm_names(request, required=True)
        pose = str(request.params.get("name", ""))
        if pose not in self.named_poses:
            raise _Refusal("INVALID_PARAMS", f"unknown pose {pose!r}; available: {sorted(self.named_poses)}")
        return name, dict(self.named_poses[pose])

    def _move_to_joint_state(self, request: CmdRequest) -> None:
        self._refuse_dry_run(request)
        name, target, speed = self._joint_target(request)
        self._start_motion(request, name, target, speed)
        return None

    def _move_to_named(self, request: CmdRequest) -> None:
        self._refuse_dry_run(request)
        name, target = self._named_target(request)
        self._start_motion(request, name, target)
        return None

    def _plan_joint_state(self, request: CmdRequest) -> dict[str, Any]:
        name, target, speed = self._joint_target(request)
        return self._plan(request, name, target, speed)

    def _plan_named(self, request: CmdRequest) -> dict[str, Any]:
        name, target = self._named_target(request)
        return self._plan(request, name, target)

    def _gripper_set(self, request: CmdRequest) -> None:
        try:
            percent = float(request.params["percent"])
        except (KeyError, TypeError, ValueError) as e:
            raise _Refusal("INVALID_PARAMS", "percent (0 = closed, 100 = open) is required") from e
        return self._gripper_to(request, percent)

    def _gripper_to(self, request: CmdRequest, percent: float) -> None:
        (name,) = self._arm_names(request, required=True)
        self._start_motion(request, name, {GRIPPER: percent})
        return None

    # =================================================================
    # Periodic work
    # =================================================================

    def tick(self) -> TickResult:
        result = TickResult()
        now = self._clock()
        if self._watchdog_due(now):
            self._watchdog_fired = True
            result.safety_events += self._engage_estop("heartbeat lapsed during motion", kind="watchdog_timeout")
        if self._base_cmd is not None and now >= self._base_cmd.expires_at:
            command, self._base_cmd = self._base_cmd, None
            try:
                assert self.base is not None
                self.base.stop()
            except (BaseError, ft.FeetechError) as e:
                logger.exception("base: stop after the command ended failed")
                result.safety_events.append({"kind": "hardware_error", "part": "base", "detail": str(e)})
            if command.request is not None:
                result.responses.append(cmd_response(command.request, ok=True, code="0",
                                                     data={"velocity": list(command.velocity),
                                                           "ticks": list(command.ticks)}))
        for name in list(self._motions):
            try:
                self._advance(name, now, result)
            except (ArmError, ft.FeetechError) as e:
                logger.exception("arm %s: bus error during motion", name)
                motion = self._motions.pop(name)
                result.responses.append(cmd_response(motion.request, ok=False, code="HARDWARE_ERROR", msg=str(e)))
                result.safety_events.append({"kind": "hardware_error", "arm": name, "detail": str(e)})
        result.responses += self._drain_cancelled()
        return result

    def _watchdog_due(self, now: float) -> bool:
        moving = bool(self._motions) or self._base_cmd is not None
        if self.heartbeat_timeout_ms <= 0 or self._watchdog_fired or not moving:
            return False
        return (now - self._last_beat) * 1000 > self.heartbeat_timeout_ms

    def _advance(self, name: str, now: float, result: TickResult) -> None:
        motion = self._motions[name]
        arm = self.arms[name]
        present = arm.read_joints()
        self._last_joints[name] = present
        lag = {j: abs(motion.last_command[j] - present[j]) for j in BODY_JOINTS}
        worst = max(lag, key=lambda j: lag[j])
        if lag[worst] > self.limits.max_tracking_error_deg:
            arm.write_goal(present)
            del self._motions[name]
            msg = (f"arm {name}: {worst} lags its command by {lag[worst]:.1f}° "
                   f"(limit {self.limits.max_tracking_error_deg:.0f}°); holding")
            result.responses.append(cmd_response(motion.request, ok=False, code="TRACKING_ERROR",
                                                 msg=msg, data={"joints": present}))
            result.safety_events.append({"kind": "tracking_error", "arm": name, "joint": worst,
                                         "lag_deg": round(lag[worst], 2)})
            return
        if motion.settling_since is None:
            command = motion.trajectory.sample(now)
            arm.write_goal(command)
            motion.last_command = command
            if motion.trajectory.done_commanding(now):
                motion.settling_since = now
            return
        goal = motion.trajectory.goal
        body_ok = all(abs(present[j] - goal[j]) <= self.limits.settle_tolerance_deg for j in BODY_JOINTS)
        gripper_ok = abs(present[GRIPPER] - goal[GRIPPER]) <= GRIPPER_TOLERANCE
        timed_out = now - motion.settling_since >= self.limits.settle_timeout_s
        if body_ok and (gripper_ok or timed_out):
            del self._motions[name]
            data = {"joints": present, "goal": goal, "gripper_blocked": not gripper_ok}
            result.responses.append(cmd_response(motion.request, ok=True, code="0", data=data))
        elif timed_out:
            # Hold where the arm stopped: leaving the goal in place keeps a blocked joint
            # pushing against whatever stopped it.
            arm.write_goal(present)
            del self._motions[name]
            off = {j: round(present[j] - goal[j], 1) for j in BODY_JOINTS
                   if abs(present[j] - goal[j]) > self.limits.settle_tolerance_deg}
            result.responses.append(cmd_response(
                motion.request, ok=False, code="NOT_REACHED",
                msg=f"arm {name}: joints did not settle on the goal: {off}; holding where it stopped",
                data={"joints": present, "goal": goal}))
            result.safety_events.append({"kind": "not_reached", "arm": name,
                                         "offsets_deg": off})

    # =================================================================
    # Estop and cancellation
    # =================================================================

    def _cancel_motion(self, name: str, code: str, msg: str) -> bool:
        motion = self._motions.pop(name, None)
        arm = self.arms[name]
        if self._torque[name]:
            try:
                arm.write_goal(arm.read_joints())
            except (ArmError, ft.FeetechError):
                logger.exception("arm %s: hold after cancel failed", name)
        if motion is None:
            return False
        self._cancelled.append(cmd_response(motion.request, ok=False, code=code, msg=msg))
        return True

    def _drain_cancelled(self) -> list[dict[str, Any]]:
        out, self._cancelled = self._cancelled, []
        return out

    def _engage_estop(self, reason: str, *, kind: str) -> list[dict[str, Any]]:
        self.estopped = True
        self.estop_reason = reason
        events = []
        for name in self.arms:
            self._cancel_motion(name, "ESTOPPED", f"estop: {reason}")
            if self.estop_action == "torque_off":
                try:
                    self.arms[name].disable_torque()
                    self._torque[name] = False
                except ArmError:
                    logger.exception("arm %s: torque off during estop failed", name)
            events.append({"kind": kind, "arm": name, "reason": reason, "action": self.estop_action})
        if self.base is not None:
            self._finish_base_request("ESTOPPED", f"estop: {reason}")
            self._base_cmd = None
            try:
                self.base.stop()
            except BaseError:
                logger.exception("base: stop during estop failed")
            events.append({"kind": kind, "part": "base", "reason": reason, "action": "stop"})
        return events

    def on_reset(self, reason: str = "episode reset") -> None:
        """An external reset moved the robot (e.g. AISLE teleported home): end moves in flight."""
        for name in list(self._motions):
            motion = self._motions.pop(name)
            self._cancelled.append(cmd_response(motion.request, ok=False, code="PREEMPTED", msg=reason))
        for name, arm in self.arms.items():
            hook = getattr(arm, "on_reset", None)
            if callable(hook):
                hook()
            try:
                self._last_joints[name] = arm.read_joints()
            except (ArmError, ft.FeetechError):
                logger.warning("arm %s: no fresh state after reset", name)

    def take_request_events(self) -> list[dict[str, Any]]:
        """Safety events raised while handling a request (e.g. robot.estop)."""
        out, self._request_events = self._request_events, []
        return out

    def take_cancelled_responses(self) -> list[dict[str, Any]]:
        """Responses for motions cancelled by a request (estop/stop); send them after its reply."""
        return self._drain_cancelled()

    # =================================================================
    # Streams
    # =================================================================

    def state_payload(self, *, refresh: bool = True) -> dict[str, Any]:
        arms = {}
        for name, arm in self.arms.items():
            if refresh and name not in self._motions:
                try:
                    self._last_joints[name] = arm.read_joints()
                except (ArmError, ft.FeetechError) as e:
                    arms[name] = {"error": str(e), "torque": self._torque[name]}
                    continue
            arms[name] = {
                "joints": self._last_joints.get(name, {}),
                "torque": self._torque[name],
                "moving": name in self._motions,
            }
        state: dict[str, Any] = {"arms": arms, "estopped": self.estopped, "estop_reason": self.estop_reason}
        if self.base is not None:
            cmd = self._base_cmd
            state["base"] = {"moving": cmd is not None,
                             "velocity": list(cmd.velocity) if cmd else [0.0, 0.0, 0.0],
                             "ticks": list(self.base.last_ticks), "odometry": None}
        return state

    def capabilities_advert(self) -> dict[str, Any]:
        return {
            "envelope_version": "1.0",
            "spec_version": "1.0.0",
            "robot_id": self.robot_id,
            "vendor": VENDOR,
            "model": MODEL,
            "node_version": NODE_VERSION,
            "arms": {name: {"servo_ids": list(arm.config.ids), "bounds": arm.bounds()}
                     for name, arm in self.arms.items()},
            "joints": list(JOINTS),
            "units": {"body": "deg", "gripper": "percent_open"},
            "named_poses": self.named_poses,
            "limits": {
                "max_speed_deg_s": self.limits.max_speed_deg_s,
                "joint_margin_deg": self.limits.joint_margin_deg,
                "max_tracking_error_deg": self.limits.max_tracking_error_deg,
                "tcp_floor_m": self.tcp_floor_m,
            },
            "safety": {
                "heartbeat_timeout_ms": self.heartbeat_timeout_ms,
                "estop_action": self.estop_action,
                "watchdog_action": self.estop_action,
            },
            "base": None if self.base is None else {
                "wheel_ids": list(self.base.config.ids),
                "max_linear_m_s": round(self.base.config.max_linear_m_s, 4),
                "radius_m": self.base.config.radius_m,
                "command_timeout_s": self.base_command_timeout_s,
                "max_duration_s": MAX_BASE_DURATION_S,
                "odometry": False,
            },
            "commands": [{"verb": v, "safety_tier": t} for v, t in self.verb_tiers.items()],
            "streams": [
                {"name": "robot.state", "schema": "vendor.lerobot.so101.state.v1"},
                {"name": "robot.safety_event", "schema": "robot.safety_event.v1", "rate_hz": "on-event"},
            ],
        }
