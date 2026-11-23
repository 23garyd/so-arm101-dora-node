from __future__ import annotations

import math
import random

import pytest

from so_arm101_node.kinematics import tcp_position
from so_arm101_node.motion import LimitError, check_tcp_floor
from so_arm101_node.node import NS

from .conftest import make_rig
from .test_controller import run

BODY = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
HOME = dict.fromkeys(BODY, 0.0)
# Folded rest measured on a real SO-101 follower; its TCP sits below the base plane.
FOLDED = {"shoulder_pan": 11.2, "shoulder_lift": -100.4, "elbow_flex": 96.2, "wrist_flex": 82.5, "wrist_roll": -0.6}


def test_tcp_matches_reference_points():
    # Reference values from AISLE's URDF-chain forward kinematics.
    assert tcp_position(HOME) == pytest.approx((0.391, 0.0, 0.226), abs=1e-3)
    assert tcp_position(HOME | {"shoulder_pan": 90.0}) == pytest.approx((0.039, -0.353, 0.226), abs=1e-3)
    assert tcp_position(HOME | {"shoulder_lift": 60.0, "elbow_flex": 30.0})[2] == pytest.approx(-0.145, abs=1e-3)


def test_tcp_matches_aisle_forward_kinematics():
    guard = pytest.importorskip("aisle.nodes.budget_guard")
    np = pytest.importorskip("numpy")
    limits = guard.load_limits("so101")
    rng = random.Random(0)
    for _ in range(500):
        joints = {j: rng.uniform(-110, 110) for j in BODY}
        ref = guard.fk_ee_pos(np.radians([joints[j] for j in BODY]).astype(np.float32), limits)
        assert tcp_position(joints) == pytest.approx(tuple(ref), abs=1e-6)


def test_a_target_into_the_table_is_refused():
    with pytest.raises(LimitError, match="below the 5 mm floor"):
        check_tcp_floor(HOME, {"shoulder_lift": 60.0, "elbow_flex": 30.0}, 0.005)
    check_tcp_floor(HOME, {"shoulder_pan": 90.0, "wrist_flex": -30.0}, 0.005)


def test_from_below_the_floor_rising_is_allowed_and_sinking_is_not():
    assert tcp_position(FOLDED)[2] < 0
    check_tcp_floor(FOLDED, {"wrist_flex": 72.9}, 0.005)  # the nod that rises
    with pytest.raises(LimitError):
        check_tcp_floor(FOLDED, {"shoulder_lift": -80.0}, 0.005)  # sinks to -25 mm from -19 mm


def test_rest_is_always_reachable():
    start = HOME | {"shoulder_lift": -40.0, "elbow_flex": 40.0}
    with pytest.raises(LimitError):
        check_tcp_floor(start, FOLDED, 0.005)
    check_tcp_floor(start, FOLDED, 0.005, rest=FOLDED | {"gripper": 2.0})


def test_controller_refuses_before_moving(clock):
    controller, sim = make_rig(clock)
    run(controller, clock, f"{NS}.enable_torque")
    goals = {i: sim.goal(i) for i in range(1, 7)}
    r = run(controller, clock, f"{NS}.move_to_joint_state", joints={"shoulder_lift": 60.0, "elbow_flex": 30.0})
    assert r["code"] == "LIMIT_VIOLATION" and "floor" in r["msg"]
    assert {i: sim.goal(i) for i in range(1, 7)} == goals
    assert controller.capabilities_advert()["limits"]["tcp_floor_m"] == 0.005
    off, _ = make_rig(clock, tcp_floor_m=None)
    run(off, clock, f"{NS}.enable_torque")
    assert run(off, clock, f"{NS}.move_to_joint_state", joints={"shoulder_lift": 30.0})["ok"]
    assert math.isfinite(tcp_position(HOME)[2])
