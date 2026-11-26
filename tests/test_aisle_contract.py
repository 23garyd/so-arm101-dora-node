"""Contract between the ``aisle`` backend and AISLE's SO-101 dora topics (SPEC 010, TC-5).

``FakeAisleScene`` checks every command exactly as AISLE's sim node does
(``joint_cmd`` Float32[6], ``gripper_cmd`` Float32[1] in [0, 1], otherwise it raises)
and tracks the commanded joints. When AISLE is importable, every command also goes
through AISLE's real budget-guard, and the tests require zero violations: the node
must stay inside the joint limits, the per-step velocity limit and the workspace.
"""

from __future__ import annotations

import json
import math

import numpy as np
import pyarrow as pa
import pytest

from so_arm101_node.aisle import (
    AISLE_HOME,
    JAW_CLOSED_RAD,
    JAW_OPEN_RAD,
    AisleArm,
    AisleLink,
    from_wire,
    to_wire,
)
from so_arm101_node.arm import ArmConfig, ArmError
from so_arm101_node.calibration import JOINTS
from so_arm101_node.config import from_env
from so_arm101_node.node import NS, So101Controller
from so_arm101_node.runtime import Pump

try:
    from aisle.nodes import budget_guard as aisle_guard
except ImportError:  # AISLE not installed: run the contract without its guard
    aisle_guard = None


class FakeAisleScene:
    """AISLE's SO-101 scene as seen from the topics, with first-order tracking."""

    def __init__(self, q0=None, alpha: float = 0.5) -> None:
        self.q = np.asarray(q0 if q0 is not None else [0, 0, 0, 0, 0, JAW_OPEN_RAD], np.float32)
        self.alpha = alpha
        self.violations: list[dict] = []
        self.commands = 0
        self.limits = aisle_guard.load_limits("so101") if aisle_guard else None
        self.last_safe = self.q.copy()
        self.last_grip = 0.0

    def receive(self, output: str, value: pa.Array) -> None:
        payload = np.asarray(value.to_numpy(zero_copy_only=False))
        assert payload.dtype == np.float32, f"{output} must be Float32, got {payload.dtype}"
        if output == "joint_cmd":
            if payload.shape != (6,):
                raise ValueError(f"joint_cmd must be Float32[6], got length {payload.shape[0]} (TC-5)")
            cmd = payload
            if self.limits is not None:
                cmd, v = aisle_guard.clamp_joint_cmd(payload, self.last_safe, self.limits, False)
                self.violations += v
                self.last_safe = cmd
            self.commands += 1
            self.q = self.q + self.alpha * (np.asarray(cmd, np.float32) - self.q)
        elif output == "gripper_cmd":
            if payload.shape != (1,) or not 0.0 <= float(payload[0]) <= 1.0:
                raise ValueError(f"gripper_cmd must be Float32[1] in [0, 1], got {payload!r}")
            if self.limits is not None:
                self.last_grip, v = aisle_guard.clamp_gripper_cmd(float(payload[0]), self.last_grip,
                                                                  self.limits, False)
                self.violations += v

    def joint_state(self) -> pa.Array:
        return pa.array(self.q, type=pa.float32())


class Rig:
    def __init__(self, clock, q0=None) -> None:
        self.clock = clock
        self.scene = FakeAisleScene(q0)
        self.link = AisleLink(clock=clock)
        arm = AisleArm(self.link, ArmConfig.from_first_id("arm", 1))
        self.controller = So101Controller({"arm": arm}, rest=AISLE_HOME, clock=clock)
        self.out: list[tuple[str, dict]] = []
        self.pump = Pump(self.controller, self._send, state_hz=10, clock=clock, aisle=self.link)
        self._n = 0

    def _send(self, output, value) -> None:
        if output in ("joint_cmd", "gripper_cmd"):
            self.scene.receive(output, value)
        else:
            self.out.append((output, json.loads(value.to_pylist()[0])))

    def step(self) -> None:
        self.clock.advance(0.02)
        self.pump.on_joint_state(self.scene.joint_state())
        self.pump.on_tick()

    def call(self, verb: str, max_s: float = 15.0, **params) -> dict:
        self._n += 1
        rid = f"r{self._n}"
        self.pump.on_request({"request_id": rid, "verb": verb, "params": params})
        for i in range(int(max_s / 0.02)):
            done = [v for o, v in self.out if o == "cmd_response" and v["request_id"] == rid]
            if done:
                return done[0]
            if i % 10 == 0:  # the bridge's heartbeat
                self.pump.on_request({"request_id": f"hb{rid}{i}", "verb": "robot.heartbeat"})
            self.step()
        raise AssertionError(f"{verb} did not answer")


@pytest.fixture
def rig(clock):
    return Rig(clock)


def test_wire_units_match_the_official_urdf():
    joints = {"shoulder_pan": 90.0, "shoulder_lift": -45.0, "elbow_flex": 0.0,
              "wrist_flex": 10.0, "wrist_roll": -90.0, "gripper": 100.0}
    cmd, grip = to_wire(joints)
    assert len(cmd) == 6
    assert cmd[0] == pytest.approx(math.pi / 2) and cmd[4] == pytest.approx(-math.pi / 2)
    assert cmd[5] == pytest.approx(JAW_OPEN_RAD) and grip == 0.0  # 100 % open = AISLE gripper 0
    closed, grip_closed = to_wire(joints | {"gripper": 0.0})
    assert closed[5] == pytest.approx(JAW_CLOSED_RAD) and grip_closed == 1.0
    back = from_wire(cmd)
    assert all(back[j] == pytest.approx(joints[j], abs=1e-6) for j in JOINTS)


def test_not_ready_until_the_first_joint_state(clock):
    rig = Rig(clock)
    rig.pump.on_request({"request_id": "a", "verb": f"{NS}.enable_torque"})
    assert rig.out[-1][1]["code"] == "NOT_READY"
    rig.pump.on_request({"request_id": "b", "verb": "robot.get_capabilities"})
    assert rig.out[-1][1]["ok"]
    rig.step()
    assert rig.pump.ready
    assert rig.call(f"{NS}.enable_torque")["ok"]


def test_stale_joint_state_is_a_hardware_error(clock):
    link = AisleLink(stale_s=0.5, clock=clock)
    with pytest.raises(ArmError, match="no joint_state"):
        link.state()
    link.update([0.0] * 6)
    clock.advance(0.6)
    with pytest.raises(ArmError, match="old"):
        link.state()
    with pytest.raises(ValueError):
        link.update([0.0] * 5)


def test_advert_uses_the_urdf_limits(rig):
    rig.step()
    bounds = rig.controller.capabilities_advert()["arms"]["arm"]["bounds"]
    assert bounds["shoulder_pan"][1] == pytest.approx(math.degrees(1.91986))
    assert bounds["elbow_flex"][0] == pytest.approx(-math.degrees(1.69))


def test_verbs_drive_the_scene_through_the_contract(rig):
    rig.step()
    assert rig.call(f"{NS}.enable_torque")["ok"]
    r = rig.call(f"{NS}.move_to_joint_state", joints={"shoulder_pan": 30.0, "wrist_flex": -20.0})
    assert r["ok"], r
    assert math.degrees(rig.scene.q[0]) == pytest.approx(30.0, abs=2.0)
    assert math.degrees(rig.scene.q[3]) == pytest.approx(-20.0, abs=2.0)
    assert rig.call(f"{NS}.gripper.set", percent=40)["ok"]
    assert from_wire(list(rig.scene.q))["gripper"] == pytest.approx(40.0, abs=3.0)
    assert rig.call(f"{NS}.move_to_named", name="rest")["ok"]  # rest = AISLE home
    assert np.allclose(rig.scene.q[:5], 0.0, atol=math.radians(2))
    assert rig.scene.commands > 50
    assert rig.scene.violations == []


def test_joint_state_has_no_load_in_aisle(rig):
    rig.step()
    state = rig.call(f"{NS}.get_joint_state")["data"]["arms"]
    assert all(arm["load"] is None and arm["current_ma"] is None for arm in state.values())


def test_limit_violations_are_refused_before_reaching_aisle(rig):
    rig.step()
    rig.call(f"{NS}.enable_torque")
    sent = rig.scene.commands
    r = rig.call(f"{NS}.move_to_joint_state", joints={"elbow_flex": 120.0})  # URDF limit 96.8°
    assert r["code"] == "LIMIT_VIOLATION"
    assert rig.scene.commands == sent


def test_reset_done_cancels_a_move_in_flight(rig):
    rig.step()
    rig.call(f"{NS}.enable_torque")
    rig.pump.on_request({"request_id": "m", "verb": f"{NS}.move_to_joint_state",
                         "params": {"joints": {"shoulder_pan": 60.0}}})
    for _ in range(10):
        rig.step()
    rig.scene.q[:] = [0, 0, 0, 0, 0, JAW_OPEN_RAD]  # AISLE teleports home
    rig.pump.on_joint_state(rig.scene.joint_state())
    rig.pump.on_event({"type": "INPUT", "id": "reset_done", "value": pa.array([1], type=pa.uint32())})
    cancelled = [v for o, v in rig.out if o == "cmd_response" and v["request_id"] == "m"]
    assert cancelled and cancelled[0]["code"] == "PREEMPTED"
    assert rig.call(f"{NS}.move_to_joint_state", joints={"shoulder_pan": 10.0})["ok"]
    assert rig.scene.violations == []


@pytest.mark.skipif(aisle_guard is None, reason="AISLE not installed")
def test_every_command_passes_aisles_budget_guard_unclamped(rig):
    """Sweep the reachable workspace at full speed; the real guard must never clamp.

    Targets keep the TCP above the 5 mm floor along the whole joint-space path
    (checked with AISLE's forward kinematics).
    """
    rig.step()
    rig.call(f"{NS}.enable_torque")
    targets = [{"shoulder_pan": 90.0}, {"shoulder_pan": -90.0}, {"shoulder_pan": 0.0},
               {"elbow_flex": 45.0, "wrist_flex": -60.0}, {"wrist_roll": 150.0}, {"wrist_roll": -150.0},
               {"shoulder_lift": -60.0, "elbow_flex": 60.0, "wrist_flex": 0.0, "wrist_roll": 0.0},
               {"shoulder_lift": 30.0, "elbow_flex": -30.0, "wrist_flex": 40.0},
               {"shoulder_lift": 0.0, "elbow_flex": 0.0, "wrist_flex": 0.0}]
    for joints in targets:
        r = rig.call(f"{NS}.move_to_joint_state", joints=joints, speed_deg_s=30, max_s=30)
        assert r["ok"], (joints, r)
    assert rig.call(f"{NS}.gripper.close")["ok"] and rig.call(f"{NS}.gripper.open")["ok"]
    assert rig.scene.violations == []


def test_a_move_into_the_table_is_refused_before_reaching_aisle(rig):
    """The node's forward-kinematics floor stops it before AISLE's guard has to."""
    rig.step()
    rig.call(f"{NS}.enable_torque")
    sent = rig.scene.commands
    r = rig.call(f"{NS}.move_to_joint_state", joints={"shoulder_lift": 60.0, "elbow_flex": 30.0})
    assert r["code"] == "LIMIT_VIOLATION" and "floor" in r["msg"]
    assert rig.scene.commands == sent
    assert rig.scene.violations == []


def test_config_accepts_the_aisle_backend():
    cfg = from_env({"SO101_BACKEND": "aisle"})
    assert cfg.backend == "aisle"
    with pytest.raises(ValueError, match="one arm"):
        from_env({"SO101_BACKEND": "aisle", "SO101_ARMS": "left:1,right:7"})
    with pytest.raises(ValueError, match="no base"):
        from_env({"SO101_BACKEND": "aisle", "SO101_BASE": "sim"})


