"""``dry_run: true`` on move_to_joint_state / move_to_named: validate and plan, never move."""

from __future__ import annotations

import pytest

from so_arm101_node import feetech as ft
from so_arm101_node.kinematics import tcp_position
from so_arm101_node.motion import lowest_tcp
from so_arm101_node.node import NS

from .conftest import make_rig
from .test_controller import req, run

MOVE = f"{NS}.move_to_joint_state"
NAMED = f"{NS}.move_to_named"
INTO_TABLE = {"shoulder_lift": 60.0, "elbow_flex": 30.0}


def motion_writes(sim) -> list:
    """Every write that could move a servo: goal positions and torque/lock changes."""
    moving = {ft.GOAL_POSITION[0], ft.TORQUE_ENABLE[0], ft.GOAL_SPEED[0]}
    return [w for w in sim.writes if w[1] in moving]


def test_dry_run_plans_without_moving(rig, clock):
    controller, sim = rig
    run(controller, clock, f"{NS}.enable_torque")
    before = list(motion_writes(sim))
    start = controller.arms["left"].read_joints()

    r = controller.handle(req(MOVE, joints={"shoulder_pan": 30.0, "wrist_flex": 20.0}, dry_run=True))

    assert r is not None and r["ok"], r  # answered at once, nothing to wait for
    data = r["data"]
    assert data["dry_run"] is True and data["blockers"] == []
    assert data["goal"]["shoulder_pan"] == 30.0 and data["goal"]["elbow_flex"] == start["elbow_flex"]
    # 30° at 60 °/s peak-sized cosine ease: 30 * pi/2 / 60
    assert data["duration_s"] == pytest.approx(30 * 3.14159265 / 2 / 60, abs=1e-3)
    z = data["tcp_z_m"]
    assert z["lowest"] <= min(z["start"], z["goal"]) + 1e-9
    assert z["start"] == pytest.approx(tcp_position(start)[2], abs=1e-4)
    assert data["tcp_floor_m"] == 0.005

    assert motion_writes(sim) == before  # no goal, torque or speed writes
    assert controller._motions == {}
    clock.advance(1.0)
    controller.tick()
    assert controller.arms["left"].read_joints() == pytest.approx(start, abs=0.2)


def test_dry_run_duration_matches_the_real_move(rig, clock):
    controller, _ = rig
    run(controller, clock, f"{NS}.enable_torque")
    dry = controller.handle(req(NAMED, name="home", dry_run=True))["data"]
    assert controller.handle(req(NAMED, name="home")) is None  # the real move starts
    assert controller._motions["left"].trajectory.duration_s == pytest.approx(dry["duration_s"], abs=1e-3)


def test_dry_run_works_before_torque_and_lists_blockers(rig, clock):
    controller, sim = rig
    r = controller.handle(req(MOVE, joints={"shoulder_pan": 20.0}, dry_run=True))
    assert r["ok"] and r["data"]["blockers"] == ["NOT_ENABLED"]
    assert not any(sim.torque(i) for i in range(1, 7))
    assert controller.handle(req(MOVE, joints={"shoulder_pan": 20.0}))["code"] == "NOT_ENABLED"

    run(controller, clock, f"{NS}.enable_torque")
    controller.handle(req("robot.estop", reason="test"))
    r = controller.handle(req(MOVE, joints={"shoulder_pan": 20.0}, dry_run=True))
    assert r["ok"] and r["data"]["blockers"] == ["ESTOPPED"]
    assert controller.handle(req(MOVE, joints={"shoulder_pan": 20.0}))["code"] == "ESTOPPED"


def test_dry_run_reports_the_same_refusals_as_the_real_move(rig, clock):
    controller, sim = rig
    run(controller, clock, f"{NS}.enable_torque")
    before = list(motion_writes(sim))
    for joints in (INTO_TABLE, {"shoulder_pan": 500.0}):
        dry = controller.handle(req(MOVE, joints=joints, dry_run=True))
        real = controller.handle(req(MOVE, joints=joints))
        assert dry["code"] == real["code"] == "LIMIT_VIOLATION"
        assert dry["msg"] == real["msg"]
    assert motion_writes(sim) == before


def test_dry_run_is_refused_while_moving_and_must_be_a_bool(rig, clock):
    controller, _ = rig
    run(controller, clock, f"{NS}.enable_torque")
    assert controller.handle(req(MOVE, joints={"shoulder_pan": 40.0})) is None
    r = controller.handle(req(MOVE, joints={"shoulder_pan": 10.0}, dry_run=True))
    assert r["code"] == "CONTROLLER_BUSY"  # the start pose is still changing
    r = controller.handle(req(NAMED, name="rest", dry_run="yes"))
    assert r["code"] == "INVALID_PARAMS"


def test_dry_run_with_the_floor_disabled_still_reports_heights(clock):
    controller, _ = make_rig(clock, tcp_floor_m=None)
    r = controller.handle(req(MOVE, joints=INTO_TABLE, dry_run=True))
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
    assert lowest == pytest.approx(fine, abs=0.003)  # 24 samples are within a few mm
    assert 0.0 <= at <= 1.0
