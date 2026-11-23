from __future__ import annotations

import json

import pytest

from so_arm101_node import calibration as calib
from so_arm101_node.calibration import MotorCalibration
from so_arm101_node.motion import DEFAULT_REST, LimitError, Limits, Trajectory, check_target

BODY = MotorCalibration(id=2, drive_mode=0, homing_offset=0, range_min=1000, range_max=3000)
GRIP = MotorCalibration(id=6, drive_mode=0, homing_offset=0, range_min=2000, range_max=3000)


def test_degrees_match_lerobot_formula():
    # LeRobot DEGREES: (raw - mid) * 360 / (resolution - 1)
    assert calib.to_user("shoulder_lift", BODY, 2000) == 0.0
    assert calib.to_user("shoulder_lift", BODY, 3000) == pytest.approx(1000 * 360 / 4095)
    for deg in (-80.0, -1.5, 0.0, 33.3, 87.0):
        raw = calib.to_raw("shoulder_lift", BODY, deg)
        assert calib.to_user("shoulder_lift", BODY, raw) == pytest.approx(deg, abs=0.1)


def test_gripper_percent_and_drive_mode():
    assert calib.to_user("gripper", GRIP, 2500) == 50.0
    inverted = MotorCalibration(6, 1, 0, 2000, 3000)
    assert calib.to_user("gripper", inverted, 2250) == 75.0
    assert calib.to_raw("gripper", inverted, 75.0) == 2250
    assert calib.to_raw("gripper", GRIP, 150.0) == 3000  # clamped


def test_to_raw_never_leaves_calibrated_range():
    assert calib.to_raw("elbow_flex", BODY, 400.0) == 3000
    assert calib.to_raw("elbow_flex", BODY, -400.0) == 1000


def test_load_accepts_lekiwi_prefixed_names_and_checks_ids(tmp_path):
    entries = {f"arm_{j}": {"id": 7 + i, "drive_mode": 0, "homing_offset": 0,
                            "range_min": 500, "range_max": 3500}
               for i, j in enumerate(calib.JOINTS)}
    path = tmp_path / "right.json"
    path.write_text(json.dumps(entries))
    cals = calib.load(path, tuple(range(7, 13)))
    assert cals["gripper"].id == 12
    with pytest.raises(calib.CalibrationError, match="expected 1"):
        calib.load(path, tuple(range(1, 7)))


def test_save_load_round_trip_and_missing_joint(tmp_path):
    cals = calib.default_sim(tuple(range(1, 7)))
    path = tmp_path / "c.json"
    calib.save(path, cals)
    assert calib.load(path, tuple(range(1, 7))) == cals
    data = json.loads(path.read_text())
    del data["wrist_roll"]
    path.write_text(json.dumps(data))
    with pytest.raises(calib.CalibrationError, match="missing joints"):
        calib.load(path)


BOUNDS = {j: (-100.0, 100.0) for j in calib.BODY_JOINTS} | {"gripper": (0.0, 100.0)}


def test_margin_rejects_targets_near_limits_but_allows_rest():
    limits = Limits(joint_margin_deg=5.0)
    check_target({"elbow_flex": 94.0}, BOUNDS, limits, DEFAULT_REST)
    with pytest.raises(LimitError, match="within 5°"):
        check_target({"elbow_flex": 97.0}, BOUNDS, limits, DEFAULT_REST)
    with pytest.raises(LimitError, match="outside"):
        check_target({"elbow_flex": 120.0}, BOUNDS, limits, DEFAULT_REST)
    rest = {"elbow_flex": 98.0}
    check_target({"elbow_flex": 98.0}, BOUNDS, limits, rest)  # rest inside margin is allowed
    with pytest.raises(LimitError, match="unknown joint"):
        check_target({"knee": 0.0}, BOUNDS, limits, rest)


def test_speed_ceiling_and_trajectory_shape():
    with pytest.raises(ValueError):
        Limits(max_speed_deg_s=120)
    limits = Limits(max_speed_deg_s=30)
    start = dict.fromkeys(calib.JOINTS, 0.0)
    traj = Trajectory.plan(start, {"shoulder_pan": 60.0}, limits, now=0.0)
    assert traj.duration_s == pytest.approx(60 * 3.14159265 / 2 / 30, rel=1e-3)
    assert traj.sample(0.0)["shoulder_pan"] == 0.0
    assert traj.sample(traj.duration_s / 2)["shoulder_pan"] == pytest.approx(30.0)
    assert traj.sample(traj.duration_s + 1)["shoulder_pan"] == 60.0
    # peak speed of the cosine ease stays at the limit
    dt = 1e-3
    mid = traj.duration_s / 2
    peak = (traj.sample(mid + dt)["shoulder_pan"] - traj.sample(mid - dt)["shoulder_pan"]) / (2 * dt)
    assert peak == pytest.approx(30.0, rel=1e-3)
    # a request for a faster speed is capped
    fast = Trajectory.plan(start, {"shoulder_pan": 60.0}, limits, now=0.0, speed_deg_s=80)
    assert fast.duration_s == traj.duration_s
