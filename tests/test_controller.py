from __future__ import annotations

import pytest

from so_arm101_node._envelope import CmdRequest
from so_arm101_node.arm import ArmError
from so_arm101_node.node import NS, VERB_TIERS

from .conftest import make_rig

_seq = iter(range(1, 10_000))


def req(verb: str, **params) -> CmdRequest:
    return CmdRequest(f"r{next(_seq)}", verb, params, None, "1.0.0", None)


def run(controller, clock, verb, *, max_s=10.0, events=None, **params):
    """Send one request and tick the fake clock until its response arrives.

    Heartbeats go out every 0.2 s, as the octos bridge sends them for a robot whose
    advert declares a heartbeat timeout.
    """
    request = req(verb, **params)
    response = controller.handle(request)
    waited = 0.0
    while response is None:
        assert waited < max_s, f"{verb} did not finish"
        clock.advance(0.02)
        waited += 0.02
        if round(waited / 0.02) % 10 == 0:
            controller.handle(req("robot.heartbeat"))
        result = controller.tick()
        if events is not None:
            events += result.safety_events
        mine = [r for r in result.responses if r["request_id"] == request.request_id]
        response = mine[0] if mine else None
    return response


def test_motion_requires_torque(rig, clock):
    controller, _ = rig
    r = run(controller, clock, f"{NS}.move_to_named", name="home")
    assert (r["ok"], r["code"]) == (False, "NOT_ENABLED")


def test_move_completes_and_reaches_goal(rig, clock):
    controller, sim = rig
    assert run(controller, clock, f"{NS}.enable_torque")["ok"]
    assert sim.torque(1) and sim.torque(6)
    r = run(controller, clock, f"{NS}.move_to_joint_state",
            joints={"shoulder_pan": 30.0, "elbow_flex": -20.0})
    assert r["ok"], r
    assert r["data"]["joints"]["shoulder_pan"] == pytest.approx(30.0, abs=2.0)
    assert r["data"]["joints"]["elbow_flex"] == pytest.approx(-20.0, abs=2.0)


def test_list_form_and_named_pose(rig, clock):
    controller, _ = rig
    run(controller, clock, f"{NS}.enable_torque")
    r = run(controller, clock, f"{NS}.move_to_joint_state", joints=[10, 0, 0, 0, 0])
    assert r["ok"], r
    r = run(controller, clock, f"{NS}.move_to_named", name="home")
    assert r["ok"] and r["data"]["joints"]["shoulder_pan"] == pytest.approx(0.0, abs=2.0)
    r = run(controller, clock, f"{NS}.move_to_named", name="dance")
    assert r["code"] == "INVALID_PARAMS"


def test_second_motion_while_moving_is_busy(rig, clock):
    controller, _ = rig
    run(controller, clock, f"{NS}.enable_torque")
    assert controller.handle(req(f"{NS}.move_to_joint_state", joints={"shoulder_pan": 40.0})) is None
    r = controller.handle(req(f"{NS}.gripper.open"))
    assert r["code"] == "CONTROLLER_BUSY"


def test_limit_violation(rig, clock):
    controller, _ = rig
    run(controller, clock, f"{NS}.enable_torque")
    r = run(controller, clock, f"{NS}.move_to_joint_state", joints={"elbow_flex": 179.0})
    assert r["code"] == "LIMIT_VIOLATION"


def test_two_arms_need_an_arm_name(clock):
    controller, sim = make_rig(clock, arm_specs=(("left", 1), ("right", 7)))
    r = run(controller, clock, f"{NS}.move_to_named", name="home")
    assert r["code"] == "INVALID_PARAMS" and "left" in r["msg"]
    run(controller, clock, f"{NS}.enable_torque")  # no arm → all arms
    assert sim.torque(1) and sim.torque(12)
    r = run(controller, clock, f"{NS}.move_to_joint_state", arm="right", joints={"shoulder_pan": 20.0})
    assert r["ok"]
    state = controller.state_payload()
    assert state["arms"]["right"]["joints"]["shoulder_pan"] == pytest.approx(20.0, abs=2.0)
    assert state["arms"]["left"]["joints"]["shoulder_pan"] == pytest.approx(0.0, abs=1.0)


def test_stalled_joint_trips_tracking_guard_and_holds(rig, clock):
    controller, sim = rig
    run(controller, clock, f"{NS}.enable_torque")
    sim.block(2)  # shoulder_lift hits something
    events = []
    r = run(controller, clock, f"{NS}.move_to_joint_state", joints={"shoulder_lift": 60.0}, events=events)
    assert r["code"] == "TRACKING_ERROR"
    assert events and events[0]["kind"] == "tracking_error" and events[0]["joint"] == "shoulder_lift"
    held = r["data"]["joints"]["shoulder_lift"]
    assert sim.goal(2) == pytest.approx(2048 + held * 4095 / 360, abs=2)


def test_gripper_blocked_by_object_still_succeeds(rig, clock):
    controller, sim = rig
    run(controller, clock, f"{NS}.enable_torque")
    run(controller, clock, f"{NS}.gripper.open")
    sim.block(6)  # an object between the jaws
    r = run(controller, clock, f"{NS}.gripper.close")
    assert r["ok"] and r["data"]["gripper_blocked"] is True


def test_estop_cancels_motion_and_latches(rig, clock):
    controller, sim = rig
    run(controller, clock, f"{NS}.enable_torque")
    pending = req(f"{NS}.move_to_joint_state", joints={"shoulder_pan": 80.0})
    assert controller.handle(pending) is None
    clock.advance(0.2)
    controller.tick()
    r = controller.handle(req("robot.estop", reason="test"))
    assert r["ok"]
    cancelled = controller.take_cancelled_responses()
    assert [c["code"] for c in cancelled] == ["ESTOPPED"]
    assert cancelled[0]["request_id"] == pending.request_id
    assert sim.torque(1)  # hold, not drop
    r = run(controller, clock, f"{NS}.move_to_named", name="home")
    assert r["code"] == "ESTOPPED"
    assert run(controller, clock, f"{NS}.clear_estop")["ok"]
    assert run(controller, clock, f"{NS}.move_to_named", name="home")["ok"]


def test_estop_torque_off_mode(clock):
    controller, sim = make_rig(clock, estop_action="torque_off")
    run(controller, clock, f"{NS}.enable_torque")
    controller.handle(req("robot.estop"))
    assert not sim.torque(1)


def test_watchdog_holds_when_heartbeats_stop(clock):
    controller, _ = make_rig(clock, heartbeat_timeout_ms=200)
    run(controller, clock, f"{NS}.enable_torque")
    assert controller.handle(req(f"{NS}.move_to_joint_state", joints={"shoulder_pan": 80.0})) is None
    events, responses = [], []
    for _ in range(30):
        clock.advance(0.02)
        result = controller.tick()
        events += result.safety_events
        responses += result.responses
    assert events[0]["kind"] == "watchdog_timeout"
    assert responses[0]["code"] == "ESTOPPED"
    assert controller.estopped


def test_heartbeats_keep_motion_alive(clock):
    controller, _ = make_rig(clock, heartbeat_timeout_ms=200)
    run(controller, clock, f"{NS}.enable_torque")
    request = req(f"{NS}.move_to_joint_state", joints={"shoulder_pan": 50.0})
    assert controller.handle(request) is None
    done = None
    for i in range(500):
        clock.advance(0.02)
        if i % 5 == 0:
            controller.handle(req("robot.heartbeat"))
        result = controller.tick()
        assert not result.safety_events
        if result.responses:
            done = result.responses[0]
            break
    assert done and done["ok"]


def test_disable_torque_requires_rest_or_force(rig, clock):
    controller, sim = rig
    run(controller, clock, f"{NS}.enable_torque")
    r = run(controller, clock, f"{NS}.disable_torque")
    assert r["code"] == "NOT_AT_REST"
    assert sim.torque(1)
    assert run(controller, clock, f"{NS}.move_to_named", name="rest")["ok"]
    assert run(controller, clock, f"{NS}.disable_torque")["ok"]
    assert not sim.torque(1)
    run(controller, clock, f"{NS}.enable_torque")
    assert run(controller, clock, f"{NS}.disable_torque", force=True)["ok"]


def test_bring_up_rejects_missing_servo(clock):
    from so_arm101_node import calibration as calib
    from so_arm101_node.arm import ArmConfig, SoArm101
    from so_arm101_node.feetech import FeetechBus
    from so_arm101_node.node import So101Controller
    from so_arm101_node.sim import SimulatedServos

    sim = SimulatedServos(range(1, 7), clock=clock)
    sim.remove(4)
    config = ArmConfig.from_first_id("arm", 1)
    controller = So101Controller({"arm": SoArm101(FeetechBus(sim), config, calib.default_sim(config.ids))},
                                 clock=clock)
    with pytest.raises(ArmError, match="wrist_flex"):
        controller.bring_up(write_calibration=True)


def test_bring_up_detects_calibration_mismatch(clock):
    controller, sim = make_rig(clock)
    sim.servos[3].set((11, 2), 1234)  # someone rewrote elbow max limit
    with pytest.raises(ArmError, match="recalibrate"):
        controller.bring_up(write_calibration=False)


def test_advert_lists_every_verb_with_a_tier(rig):
    controller, _ = rig
    advert = controller.capabilities_advert()
    verbs = {c["verb"]: c["safety_tier"] for c in advert["commands"]}
    assert verbs == VERB_TIERS
    assert advert["safety"]["heartbeat_timeout_ms"] == 1000
    assert set(verbs.values()) <= {"observe", "safe_motion", "full_actuation", "emergency_override"}
    r = controller.handle(req("vendor.nope"))
    assert r["code"] == "INVALID_PARAMS"


def test_not_reached_holds_where_the_joint_stopped(rig, clock):
    """A joint blocked close to its goal (lag below the tracking limit) must stop pushing."""
    controller, sim = rig
    run(controller, clock, f"{NS}.enable_torque")
    start_encoder = sim.servos[4].encoder
    sim.block(4, start_encoder + 4 * 4095 / 360)  # stops 4° along a 10° move
    events = []
    r = run(controller, clock, f"{NS}.move_to_joint_state", joints={"wrist_flex": 10.0}, events=events)
    assert r["code"] == "NOT_REACHED" and "holding" in r["msg"]
    assert any(e["kind"] == "not_reached" for e in events)
    present_raw = sim.servos[4].present()
    assert abs(sim.goal(4) - present_raw) <= 2  # goal pulled back to where it stopped
