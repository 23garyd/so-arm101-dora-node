"""plan_joint_state / plan_named: the plan a move would execute, without moving.

The plan verbs share the move verbs' parsing, limit and floor checks and trajectory
planner, so they must return the same plan and fail with the same errors, and they
must never write to a servo or queue a move.
"""

from __future__ import annotations

import pytest

from so_arm101_node.kinematics import tcp_position
from so_arm101_node.motion import lowest_tcp
from so_arm101_node.node import NS, VERB_TIERS

from .conftest import make_rig
from .test_controller import req, run

MOVE, PLAN = f"{NS}.move_to_joint_state", f"{NS}.plan_joint_state"
MOVE_NAMED, PLAN_NAMED = f"{NS}.move_to_named", f"{NS}.plan_named"
INTO_TABLE = {"shoulder_lift": 60.0, "elbow_flex": 30.0}


def servo_writes(sim) -> list:
    """Every write to any servo register (goal, torque, speed, EEPROM, ...)."""
    return list(sim.writes)


def test_plan_verbs_are_observe_tier_and_advertised(rig):
    controller, _ = rig
    assert VERB_TIERS[PLAN] == VERB_TIERS[PLAN_NAMED] == "observe"
    advertised = {c["verb"]: c["safety_tier"] for c in controller.capabilities_advert()["commands"]}
    assert advertised[PLAN] == advertised[PLAN_NAMED] == "observe"


@pytest.mark.parametrize("torque", [False, True])
def test_plan_writes_nothing_and_queues_nothing(rig, clock, torque):
    controller, sim = rig
    if torque:
        run(controller, clock, f"{NS}.enable_torque")
    before = servo_writes(sim)
    start = controller.arms["left"].read_joints()
    for verb, params in ((PLAN, {"joints": {"shoulder_pan": 30.0, "wrist_flex": 20.0}}),
                         (PLAN_NAMED, {"name": "home"}),
                         (PLAN, {"joints": INTO_TABLE})):  # a refused plan writes nothing either
        r = controller.handle(req(verb, **params))
        assert r is not None  # answered at once
    assert servo_writes(sim) == before
    assert controller._motions == {}
    clock.advance(1.0)
    controller.tick()
    assert controller.arms["left"].read_joints() == pytest.approx(start, abs=0.2)


@pytest.mark.parametrize("verbs, params", [
    ((PLAN, MOVE), {"joints": {"shoulder_pan": 30.0, "wrist_flex": 20.0}}),
    ((PLAN, MOVE), {"joints": {"elbow_flex": 40.0}, "speed_deg_s": 10}),
    ((PLAN_NAMED, MOVE_NAMED), {"name": "home"}),
])
def test_plan_matches_the_move(rig, clock, verbs, params):
    controller, _ = rig
    run(controller, clock, f"{NS}.enable_torque")
    plan = controller.handle(req(verbs[0], **params))
    assert plan["ok"], plan
    data = plan["data"]
    assert data["blockers"] == []
    assert controller.handle(req(verbs[1], **params)) is None  # the move starts
    trajectory = controller._motions["left"].trajectory
    assert data["goal"] == trajectory.goal
    assert data["start"] == trajectory.start
    assert data["duration_s"] == pytest.approx(trajectory.duration_s, abs=1e-3)
    z = data["tcp_z_m"]
    assert z["start"] == pytest.approx(tcp_position(trajectory.start)[2], abs=1e-4)
    assert z["goal"] == pytest.approx(tcp_position(trajectory.goal)[2], abs=1e-4)
    assert z["lowest"] <= min(z["start"], z["goal"]) + 1e-9


@pytest.mark.parametrize("verbs, params", [
    ((PLAN, MOVE), {"joints": INTO_TABLE}),  # below the floor
    ((PLAN, MOVE), {"joints": {"shoulder_pan": 500.0}}),  # outside the calibrated range
    ((PLAN, MOVE), {"joints": {"nose": 1.0}}),  # unknown joint
    ((PLAN, MOVE), {"joints": [1, 2]}),  # wrong list length
    ((PLAN, MOVE), {"joints": {"shoulder_pan": "left"}}),  # not a number
    ((PLAN, MOVE), {"joints": {"shoulder_pan": 10.0}, "speed_deg_s": "fast"}),
    ((PLAN, MOVE), {"joints": {"shoulder_pan": 10.0}, "arm": "nope"}),
    ((PLAN_NAMED, MOVE_NAMED), {"name": "dance"}),
])
def test_plan_fails_with_the_same_error_as_the_move(rig, clock, verbs, params):
    controller, sim = rig
    run(controller, clock, f"{NS}.enable_torque")
    before = servo_writes(sim)
    plan = controller.handle(req(verbs[0], **params))
    move = controller.handle(req(verbs[1], **params))
    assert plan is not None and move is not None
    assert not plan["ok"] and not move["ok"]
    assert (plan["code"], plan["msg"]) == (move["code"], move["msg"])
    assert servo_writes(sim) == before


def test_two_arms_need_an_arm_name_for_plans_too(clock):
    controller, _ = make_rig(clock, arm_specs=(("left", 1), ("right", 7)))
    plan = controller.handle(req(PLAN_NAMED, name="home"))
    move = controller.handle(req(MOVE_NAMED, name="home"))
    assert plan["code"] == move["code"] == "INVALID_PARAMS" and plan["msg"] == move["msg"]
    assert controller.handle(req(PLAN_NAMED, name="home", arm="right"))["data"]["arm"] == "right"


def test_plan_before_torque_and_while_estopped_lists_blockers(rig, clock):
    controller, _ = rig
    r = controller.handle(req(PLAN, joints={"shoulder_pan": 20.0}))
    assert r["ok"] and r["data"]["blockers"] == ["NOT_ENABLED"]
    assert controller.handle(req(MOVE, joints={"shoulder_pan": 20.0}))["code"] == "NOT_ENABLED"

    run(controller, clock, f"{NS}.enable_torque")
    controller.handle(req("robot.estop", reason="test"))
    r = controller.handle(req(PLAN_NAMED, name="rest"))
    assert r["ok"] and r["data"]["blockers"] == ["ESTOPPED"]
    assert controller.handle(req(MOVE_NAMED, name="rest"))["code"] == "ESTOPPED"


def test_plan_is_refused_while_moving(rig, clock):
    controller, _ = rig
    run(controller, clock, f"{NS}.enable_torque")
    assert controller.handle(req(MOVE, joints={"shoulder_pan": 40.0})) is None
    assert controller.handle(req(PLAN, joints={"shoulder_pan": 10.0}))["code"] == "CONTROLLER_BUSY"


def test_move_verbs_refuse_a_dry_run_parameter(rig, clock):
    """A planning request sent the old way must not become a real move."""
    controller, sim = rig
    run(controller, clock, f"{NS}.enable_torque")
    before = servo_writes(sim)
    for verb, params in ((MOVE, {"joints": {"shoulder_pan": 30.0}}), (MOVE_NAMED, {"name": "home"})):
        for flag in (True, False):
            r = controller.handle(req(verb, dry_run=flag, **params))
            assert r is not None and r["code"] == "INVALID_PARAMS" and "plan_" in r["msg"]
    assert servo_writes(sim) == before and controller._motions == {}


def test_plan_with_the_floor_disabled_still_reports_heights(clock):
    controller, _ = make_rig(clock, tcp_floor_m=None)
    r = controller.handle(req(PLAN, joints=INTO_TABLE))
    assert r["ok"] and r["data"]["tcp_floor_m"] is None
    assert r["data"]["tcp_z_m"]["lowest"] < 0  # below the mounting surface


def test_lowest_tcp_matches_fine_sampling():
    start = {"shoulder_pan": 0.0, "shoulder_lift": -40.0, "elbow_flex": 60.0, "wrist_flex": 70.0, "wrist_roll": 0.0}
    goal = {"shoulder_lift": 40.0, "elbow_flex": -20.0, "wrist_flex": -40.0}
    lowest, at = lowest_tcp(start, goal)
    full = start | goal
    fine = min(
        tcp_position({j: start[j] + (full[j] - start[j]) * i / 1000 for j in start})[2] for i in range(1001)
    )
    assert lowest == pytest.approx(fine, abs=0.003)
    assert 0.0 <= at <= 1.0

