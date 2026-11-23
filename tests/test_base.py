from __future__ import annotations

import json

import pyarrow as pa
import pytest

from so_arm101_node import calibration as calib
from so_arm101_node.arm import ArmConfig, SoArm101
from so_arm101_node.base import BaseConfig, OmniBase, wheel_ticks
from so_arm101_node.config import from_env
from so_arm101_node.feetech import FeetechBus
from so_arm101_node.node import BASE_NS, NS, So101Controller
from so_arm101_node.runtime import Pump, build_controller
from so_arm101_node.sim import SimulatedServos

from .test_controller import req, run

WHEELS = (13, 14, 15)


def make_nano(clock, **kw):
    """Left arm 1-6, right arm 7-12, omni base 13-15 on one simulated bus."""
    sim = SimulatedServos(list(range(1, 16)), clock=clock, slew_ticks_per_s=4000)
    bus = FeetechBus(sim)
    arms = {}
    for name, first in (("left", 1), ("right", 7)):
        config = ArmConfig.from_first_id(name, first)
        arms[name] = SoArm101(bus, config, calib.default_sim(config.ids))
    controller = So101Controller(arms, base=OmniBase(bus), clock=clock, **kw)
    controller.bring_up(write_calibration=True)
    return controller, sim


def speeds(sim):
    return tuple(sim.wheel_speed(i) for i in WHEELS)


def test_wheel_mixing_matches_omni_car_and_keeps_direction():
    cfg = BaseConfig(max_ticks=600)
    left, back, right = wheel_ticks(0.02, 0.0, 0.0, cfg)  # forward
    assert back == 0 and left == -right and right > 0
    assert len(set(wheel_ticks(0.0, 0.0, 0.2, cfg))) == 1  # pure rotation: all wheels equal
    fast = wheel_ticks(1.0, 0.5, 0.0, cfg)  # far beyond the limit
    slow = wheel_ticks(0.01, 0.005, 0.0, cfg)
    assert max(map(abs, fast)) == 600
    assert fast[0] / fast[2] == pytest.approx(slow[0] / slow[2], rel=0.02)
    with pytest.raises(ValueError):
        BaseConfig(ids=(13, 13, 15))


def test_base_verbs_only_exist_with_a_base(clock, rig):
    controller, _ = rig
    assert f"{BASE_NS}.set_velocity" not in controller.verb_tiers
    nano, _ = make_nano(clock)
    assert nano.verb_tiers[f"{BASE_NS}.set_velocity"] == "safe_motion"
    assert nano.verb_tiers[f"{BASE_NS}.stop"] == "emergency_override"
    assert nano.capabilities_advert()["base"]["odometry"] is False


def test_velocity_without_duration_is_a_deadman(clock):
    controller, sim = make_nano(clock, base_command_timeout_s=0.5)
    r = controller.handle(req(f"{BASE_NS}.set_velocity", vx=0.02))
    assert r["ok"] and r["data"]["expires_in_s"] == 0.5
    assert speeds(sim)[2] > 0 and sim.torque(13)
    clock.advance(0.3)
    controller.tick()
    assert speeds(sim) != (0, 0, 0)
    clock.advance(0.3)
    controller.tick()
    assert speeds(sim) == (0, 0, 0) and not sim.torque(13)


def test_velocity_with_duration_answers_when_done(clock):
    controller, sim = make_nano(clock)
    r = run(controller, clock, f"{BASE_NS}.set_velocity", vy=0.01, duration_s=1.0)
    assert r["ok"] and r["data"]["velocity"] == [0.0, 0.01, 0.0]
    assert speeds(sim) == (0, 0, 0)


def test_new_velocity_preempts_and_stop_halts(clock):
    controller, sim = make_nano(clock)
    first = req(f"{BASE_NS}.set_velocity", vx=0.02, duration_s=3.0)
    assert controller.handle(first) is None
    assert controller.handle(req(f"{BASE_NS}.set_velocity", omega=0.1))["ok"]
    cancelled = controller.take_cancelled_responses()
    assert [(c["request_id"], c["code"]) for c in cancelled] == [(first.request_id, "PREEMPTED")]
    assert controller.handle(req(f"{BASE_NS}.stop"))["ok"]
    assert speeds(sim) == (0, 0, 0)
    assert controller.handle(req(f"{BASE_NS}.set_velocity", vx=0, vy=0, omega=0))["data"]["ticks"] == [0, 0, 0]


def test_estop_stops_the_base_and_answers_pending(clock):
    controller, sim = make_nano(clock)
    pending = req(f"{BASE_NS}.set_velocity", vx=0.02, duration_s=3.0)
    controller.handle(pending)
    controller.handle(req("robot.estop", reason="test"))
    assert speeds(sim) == (0, 0, 0)
    codes = {c["request_id"]: c["code"] for c in controller.take_cancelled_responses()}
    assert codes[pending.request_id] == "ESTOPPED"
    events = controller.take_request_events()
    assert {e.get("part") or e.get("arm") for e in events} == {"left", "right", "base"}
    r = controller.handle(req(f"{BASE_NS}.set_velocity", vx=0.01))
    assert r["code"] == "ESTOPPED"


def test_watchdog_covers_the_base(clock):
    controller, sim = make_nano(clock, heartbeat_timeout_ms=200)
    controller.handle(req(f"{BASE_NS}.set_velocity", vx=0.02, duration_s=5.0))
    events = []
    for _ in range(20):
        clock.advance(0.02)
        events += controller.tick().safety_events
    assert any(e["kind"] == "watchdog_timeout" and e.get("part") == "base" for e in events)
    assert speeds(sim) == (0, 0, 0)


def test_arm_and_base_move_together(clock):
    controller, sim = make_nano(clock)
    run(controller, clock, f"{NS}.enable_torque")
    controller.handle(req(f"{BASE_NS}.set_velocity", vx=0.02, duration_s=4.0))
    r = run(controller, clock, f"{NS}.move_to_joint_state", arm="left", joints={"shoulder_pan": 20.0})
    assert r["ok"]
    assert speeds(sim) != (0, 0, 0)  # base still driving while the arm moved


def test_config_parses_sim_arms_and_base():
    cfg = from_env({"SO101_ARMS": "left:1,right:7@sim", "SO101_BASE": "sim", "BASE_RADIUS_M": "0.1"})
    assert [(a.name, a.sim) for a in cfg.arms] == [("left", False), ("right", True)]
    assert cfg.base == "sim" and cfg.base_config.radius_m == 0.1
    with pytest.raises(ValueError, match="overlap"):
        from_env({"SO101_ARMS": "left:10", "SO101_BASE": "sim"})
    with pytest.raises(ValueError):
        from_env({"SO101_ARMS": "left:1@real"})


def test_simulated_parts_never_open_the_serial_port():
    cfg = from_env({"SO101_BACKEND": "serial", "SO101_PORT": "/dev/does-not-exist",
                    "SO101_ARMS": "left:1@sim,right:7@sim", "SO101_BASE": "sim"})
    controller, buses = build_controller(cfg)
    assert buses.serial is None and buses.sim is not None
    assert controller.base is not None
    buses.close()


def test_pump_streams_base_estop_events(clock):
    controller, _ = make_nano(clock)
    sent = []
    pump = Pump(controller, lambda o, v: sent.append((o, json.loads(v.to_pylist()[0]))), state_hz=10, clock=clock)
    pump.on_event({"type": "INPUT", "id": "cmd_request",
                   "value": pa.array([json.dumps({"request_id": "e", "verb": "robot.estop"})])})
    parts = {v["payload"].get("part") or v["payload"].get("arm") for o, v in sent if o == "safety_event"}
    assert parts == {"left", "right", "base"}
    pump.on_tick()
    state = [v for o, v in sent if o == "state"][-1]
    assert state["payload"]["base"]["moving"] is False
