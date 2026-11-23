from __future__ import annotations

import json
from pathlib import Path

import pyarrow as pa
import pytest

from so_arm101_node.config import from_env
from so_arm101_node.node import NS
from so_arm101_node.runtime import Pump, build_controller, decode

from .conftest import make_rig


class Outputs:
    def __init__(self) -> None:
        self.sent: list[tuple[str, dict]] = []

    def __call__(self, output: str, value: pa.Array) -> None:
        self.sent.append((output, json.loads(value.to_pylist()[0])))

    def of(self, output: str) -> list[dict]:
        return [v for o, v in self.sent if o == output]


def _event(input_id: str, payload: dict | None = None) -> dict:
    value = pa.array([json.dumps(payload)]) if payload is not None else pa.array([0])
    return {"type": "INPUT", "id": input_id, "value": value}


def test_pump_answers_requests_and_streams_state(clock):
    controller, _ = make_rig(clock)
    out = Outputs()
    pump = Pump(controller, out, state_hz=10, clock=clock)
    assert pump.on_event(_event("tick"))
    assert out.of("capabilities")[0]["robot_id"] == "so101"
    state = out.of("state")[0]
    assert state["stream"] == "robot.state" and state["seq"] == 1
    assert "left" in state["payload"]["arms"]

    pump.on_event(_event("cmd_request", {"request_id": "a", "verb": f"{NS}.enable_torque", "params": {}}))
    assert out.of("cmd_response")[-1]["request_id"] == "a"

    pump.on_event(_event("cmd_request", {"request_id": "b", "verb": f"{NS}.gripper.open", "params": {}}))
    assert out.of("cmd_response")[-1]["request_id"] == "a"  # deferred
    for i in range(200):
        clock.advance(0.02)
        if i % 10 == 0:  # the bridge's heartbeat
            pump.on_event(_event("cmd_request", {"request_id": f"hb{i}", "verb": "robot.heartbeat"}))
        pump.on_event(_event("tick"))
        if out.of("cmd_response")[-1]["request_id"] == "b":
            break
    assert out.of("cmd_response")[-1]["ok"]


def test_pump_rejects_bad_envelopes_and_ignores_other_targets(clock):
    controller, _ = make_rig(clock)
    out = Outputs()
    pump = Pump(controller, out, state_hz=10, clock=clock)
    pump.on_event(_event("cmd_request", {"request_id": "x", "params": {}}))
    assert out.of("cmd_response")[-1]["code"] == "INVALID_PARAMS"
    pump.on_event(_event("cmd_request", {"request_id": "y", "verb": "robot.heartbeat", "target": "other"}))
    assert len(out.of("cmd_response")) == 1
    assert pump.on_event({"type": "STOP"}) is False


def test_estop_request_emits_safety_events(clock):
    controller, _ = make_rig(clock, arm_specs=(("left", 1), ("right", 7)))
    out = Outputs()
    pump = Pump(controller, out, state_hz=10, clock=clock)
    pump.on_event(_event("cmd_request", {"request_id": "e", "verb": "robot.estop", "params": {"reason": "t"}}))
    events = out.of("safety_event")
    assert {e["payload"]["arm"] for e in events} == {"left", "right"}
    assert events[0]["stream"] == "robot.safety_event"


def test_decode_handles_garbage():
    assert decode(pa.array(["{\"a\": 1}"])) == {"a": 1}
    assert decode(pa.array([])) is None
    assert decode(pa.array(["not json"])) is None


def test_config_parses_nano_arms_and_calibration_paths(tmp_path):
    env = {"ROBOT_ID": "nano", "SO101_ARMS": "left:1,right:7", "HF_LEROBOT_CALIBRATION": str(tmp_path),
           "SO101_CALIBRATION": "right=/cal/r.json"}
    cfg = from_env(env)
    assert [a.ids for a in cfg.arms] == [tuple(range(1, 7)), tuple(range(7, 13))]
    assert cfg.calibration["left"] == tmp_path / "robots/so_follower/nano_left.json"
    assert cfg.calibration["right"] == Path("/cal/r.json")
    single = from_env({"ROBOT_ID": "bench", "HF_LEROBOT_CALIBRATION": str(tmp_path)})
    assert single.calibration["arm"] == tmp_path / "robots/so_follower/bench.json"


@pytest.mark.parametrize("bad", ["left:1,right:4", "left", "left:x", "a:1,b:1"])
def test_config_rejects_bad_arm_specs(bad):
    with pytest.raises(ValueError):
        from_env({"SO101_ARMS": bad})


def test_sim_backend_builds_from_env():
    cfg = from_env({"SO101_BACKEND": "sim", "SO101_ARMS": "left:1,right:7"})
    controller, bus = build_controller(cfg)
    assert sorted(controller.arms) == ["left", "right"]
    assert controller.state_payload()["arms"]["right"]["torque"] is False
    bus.close()


def test_rest_pose_from_env():
    cfg = from_env({"SO101_REST_POSE": json.dumps({"shoulder_pan": 1, "shoulder_lift": -90, "elbow_flex": 90,
                                                   "wrist_flex": 60, "wrist_roll": 0, "gripper": 10})})
    assert cfg.rest_pose["shoulder_lift"] == -90.0
    with pytest.raises(ValueError, match="missing joints"):
        from_env({"SO101_REST_POSE": json.dumps({"shoulder_pan": 1})})


def test_calibration_falls_back_to_the_older_lerobot_layout(tmp_path):
    old = tmp_path / "robots/so101_follower/bench.json"
    old.parent.mkdir(parents=True)
    old.write_text("{}")
    cfg = from_env({"ROBOT_ID": "bench", "HF_LEROBOT_CALIBRATION": str(tmp_path)})
    assert cfg.calibration["arm"] == old
    new = tmp_path / "robots/so_follower/bench.json"
    new.parent.mkdir(parents=True)
    new.write_text("{}")
    assert from_env({"ROBOT_ID": "bench", "HF_LEROBOT_CALIBRATION": str(tmp_path)}).calibration["arm"] == new


def test_hello_dry_run_writes_nothing(monkeypatch, capsys):
    from so_arm101_node import cli
    from so_arm101_node.sim import SimulatedServos

    sims = []
    real_init = SimulatedServos.__init__

    def track(self, *a, **kw):
        real_init(self, *a, **kw)
        sims.append(self)

    monkeypatch.setattr(SimulatedServos, "__init__", track)
    assert cli.main(["--sim", "hello", "--arm", "arm"]) == 0
    # The sim arm is calibrated by writing EEPROM first; everything after that must be reads.
    calibration_writes = {(i, a) for i in range(1, 7) for a in (9, 11, 31, 40, 55)}
    extra = [w for w in sims[0].writes if (w[0], w[1]) not in calibration_writes]
    assert extra == []
    assert "dry run: read only" in capsys.readouterr().out
