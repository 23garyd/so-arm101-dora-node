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
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from so_arm101_node import feetech as ft
from so_arm101_node._envelope import CmdRequest, cmd_response
from so_arm101_node.arm import ArmError, SoArm101
from so_arm101_node.calibration import BODY_JOINTS, GRIPPER, JOINTS
from so_arm101_node.motion import DEFAULT_HOME, DEFAULT_REST, LimitError, Limits, Trajectory, check_target

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
    f"{NS}.gripper.set": "safe_motion",
    f"{NS}.gripper.open": "safe_motion",
    f"{NS}.gripper.close": "safe_motion",
}
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
        arms: dict[str, SoArm101],
        *,
        robot_id: str = "so101",
        limits: Limits | None = None,
        rest: dict[str, float] | None = None,
        home: dict[str, float] | None = None,
        heartbeat_timeout_ms: int = 1000,
        estop_action: str = "hold",
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not arms:
            raise ValueError("at least one arm is required")
        if estop_action not in ("hold", "torque_off"):
            raise ValueError("estop_action must be 'hold' or 'torque_off'")
        self.arms = arms
        self.robot_id = robot_id
        self.limits = limits or Limits()
        self.named_poses = {"rest": dict(rest or DEFAULT_REST), "home": dict(home or DEFAULT_HOME)}
        self.heartbeat_timeout_ms = heartbeat_timeout_ms
        self.estop_action = estop_action
        self._clock = clock
        self._motions: dict[str, _Motion] = {}
        self._cancelled: list[dict[str, Any]] = []
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
            f"{NS}.gripper.set": self._gripper_set,
            f"{NS}.gripper.open": lambda r: self._gripper_to(r, GRIPPER_OPEN),
            f"{NS}.gripper.close": lambda r: self._gripper_to(r, GRIPPER_CLOSED),
        }
        assert set(self._handlers) == set(VERB_TIERS)

    # =================================================================
    # Startup
    # =================================================================

    def bring_up(self, *, write_calibration: bool = False) -> list[str]:
        """Preflight every arm; configure the ones that are limp. Returns warnings."""
        warnings = []
        for name, arm in self.arms.items():
            torque = arm.preflight()
            if write_calibration:
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
        except (ArmError, ft.FeetechError) as e:
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

    def _refuse_if_estopped(self) -> None:
        if self.estopped:
            raise _Refusal("ESTOPPED", f"estopped ({self.estop_reason}); call {NS}.clear_estop first")

    def _start_motion(self, request: CmdRequest, name: str, target: dict[str, float],
                      speed: float | None = None) -> None:
        self._refuse_if_estopped()
        if name in self._motions:
            raise _Refusal("CONTROLLER_BUSY", f"arm {name} is already moving")
        if not self._torque[name]:
            raise _Refusal("NOT_ENABLED", f"arm {name}: call {NS}.enable_torque first")
        arm = self.arms[name]
        check_target(target, arm.bounds(), self.limits, self.named_poses["rest"])
        start = arm.read_joints()
        now = self._clock()
        trajectory = Trajectory.plan(start, target, self.limits, now, speed)
        self._motions[name] = _Motion(request, trajectory, last_command=dict(start))

    def _move_to_joint_state(self, request: CmdRequest) -> None:
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
        self._start_motion(request, name, target, None if speed is None else float(speed))
        return None

    def _move_to_named(self, request: CmdRequest) -> None:
        (name,) = self._arm_names(request, required=True)
        pose = str(request.params.get("name", ""))
        if pose not in self.named_poses:
            raise _Refusal("INVALID_PARAMS", f"unknown pose {pose!r}; available: {sorted(self.named_poses)}")
        self._start_motion(request, name, dict(self.named_poses[pose]))
        return None

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
        if self.heartbeat_timeout_ms <= 0 or self._watchdog_fired or not self._motions:
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
            del self._motions[name]
            off = {j: round(present[j] - goal[j], 1) for j in BODY_JOINTS
                   if abs(present[j] - goal[j]) > self.limits.settle_tolerance_deg}
            result.responses.append(cmd_response(
                motion.request, ok=False, code="NOT_REACHED",
                msg=f"arm {name}: joints did not settle on the goal: {off}",
                data={"joints": present, "goal": goal}))

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
        return events

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
        return {"arms": arms, "estopped": self.estopped, "estop_reason": self.estop_reason}

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
            },
            "safety": {
                "heartbeat_timeout_ms": self.heartbeat_timeout_ms,
                "estop_action": self.estop_action,
                "watchdog_action": self.estop_action,
            },
            "commands": [{"verb": v, "safety_tier": t} for v, t in VERB_TIERS.items()],
            "streams": [
                {"name": "robot.state", "schema": "vendor.lerobot.so101.state.v1"},
                {"name": "robot.safety_event", "schema": "robot.safety_event.v1", "rate_hz": "on-event"},
            ],
        }
