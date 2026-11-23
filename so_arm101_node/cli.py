"""``so-arm101`` bring-up tool: scan the bus, read joints, calibrate, wave, go limp.

Runs without dora. Follows the staged SO-101 bring-up from Google DeepMind's
robotics Antigravity plugin (Apache-2.0): read-only checks first, a dry run
before any motion, and live motion only with an explicit ``--go``.

Arms are given as ``name:first_servo_id`` (``left:1,right:7`` on a Nano). Every
command accepts ``--sim`` to run against simulated servos instead of a port.
"""

from __future__ import annotations

import argparse
import json
import select
import sys
import time
from pathlib import Path

from so_arm101_node import calibration as calib
from so_arm101_node import feetech as ft
from so_arm101_node._envelope import CmdRequest
from so_arm101_node.arm import ArmConfig, SoArm101
from so_arm101_node.calibration import JOINTS, MotorCalibration
from so_arm101_node.config import _parse_arms, default_calibration_path
from so_arm101_node.motion import Limits, check_target
from so_arm101_node.node import NS, So101Controller
from so_arm101_node.sim import SimulatedServos


def _bus(args: argparse.Namespace, ids: list[int]) -> ft.FeetechBus:
    if args.sim:
        return ft.FeetechBus(SimulatedServos(ids))
    return ft.FeetechBus.open_serial(args.port, args.baud)


def _calibration(args: argparse.Namespace, arm: ArmConfig, single: bool) -> dict[str, MotorCalibration]:
    if args.sim:
        return calib.default_sim(arm.ids)
    path = Path(args.calibration_dir) / f"{args.robot_id if single else f'{args.robot_id}_{arm.name}'}.json" \
        if args.calibration_dir else default_calibration_path(args.robot_id, arm.name, single, {})
    return calib.load(path, arm.ids)


def cmd_scan(args: argparse.Namespace) -> int:
    first, _, last = args.ids.partition("-")
    ids = list(range(int(first), int(last or first) + 1))
    bus = _bus(args, ids)
    found = 0
    for servo_id in ids:
        try:
            model = bus.read(servo_id, ft.MODEL_NUMBER)
        except ft.FeetechError:
            continue
        found += 1
        position = bus.read_position(servo_id)
        torque = bus.read(servo_id, ft.TORQUE_ENABLE)
        volts = bus.read(servo_id, ft.PRESENT_VOLTAGE) / 10
        print(f"id {servo_id:3d}  model {model}{'' if model == ft.STS3215_MODEL else ' (not STS3215)'}"
              f"  position {position:5d}  torque {torque}  {volts:.1f} V")
    print(f"{found} servo(s) answered")
    return 0 if found else 1


def _arms(args: argparse.Namespace) -> dict[str, SoArm101]:
    configs = _parse_arms(args.arms)
    bus = _bus(args, [i for a in configs for i in a.ids])
    arms = {}
    for config in configs:
        cal = _calibration(args, config, len(configs) == 1)
        arm = SoArm101(bus, config, cal)
        if args.sim:
            arm.write_calibration()
        arms[config.name] = arm
    return arms


def cmd_read(args: argparse.Namespace) -> int:
    for name, arm in _arms(args).items():
        torque = arm.preflight()
        joints = arm.read_joints()
        print(json.dumps({"arm": name, "torque": torque, "joints": {j: round(v, 2) for j, v in joints.items()}}))
    return 0


def cmd_torque_off(args: argparse.Namespace) -> int:
    for name, arm in _arms(args).items():
        arm.disable_torque()
        print(f"arm {name}: torque off — support the arm, it is now limp")
    return 0


def _wait_enter(prompt: str) -> None:
    try:
        input(f"{prompt}\nPress Enter to continue (Ctrl-C to abort). ")
    except EOFError:
        raise SystemExit("calibration needs an interactive terminal to confirm each step") from None


def _enter_pressed() -> bool:
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if ready:
        sys.stdin.readline()
        return True
    return False


def cmd_calibrate(args: argparse.Namespace) -> int:
    """LeRobot-style calibration: middle pose sets the offsets, a full sweep sets the limits."""
    config = ArmConfig.from_first_id(args.arm, args.first_id)
    bus = _bus(args, list(config.ids))
    for servo_id in config.ids:
        bus.write(servo_id, ft.TORQUE_ENABLE, 0)
        bus.write(servo_id, ft.LOCK, 0)
    _wait_enter(f"Arm {args.arm}: torque is off. Move it by hand to the middle of every joint's range.")
    offsets = {}
    for joint, servo_id in zip(JOINTS, config.ids):
        bus.write(servo_id, ft.OPERATING_MODE, 0)
        bus.write_homing_offset(servo_id, 0)
        bus.write(servo_id, ft.MIN_POSITION_LIMIT, 0)
        bus.write(servo_id, ft.MAX_POSITION_LIMIT, 4095)
        offset = bus.read_position(servo_id) - 2047
        if abs(offset) > 2047:
            raise SystemExit(f"{joint}: position outside the calibration range; restart from the middle")
        bus.write_homing_offset(servo_id, offset)
        offsets[joint] = offset
    print("Now move every joint slowly through its full range, including the gripper.")
    print("Wrist roll keeps its full turn. Press Enter when done.")
    low = dict.fromkeys(JOINTS, 4095)
    high = dict.fromkeys(JOINTS, 0)
    while True:
        for joint, servo_id in zip(JOINTS, config.ids):
            p = bus.read_position(servo_id)
            if not 0 <= p <= 4095:
                raise SystemExit("encoder wrapped during the sweep; restart from the middle pose")
            low[joint], high[joint] = min(low[joint], p), max(high[joint], p)
        if args.sim or _enter_pressed():
            break
        time.sleep(0.02)
    low["wrist_roll"], high["wrist_roll"] = 0, 4095
    cals = {}
    for joint, servo_id in zip(JOINTS, config.ids):
        if high[joint] - low[joint] < 32 and not args.sim:
            raise SystemExit(f"{joint} did not move enough; repeat the sweep")
        if args.sim and high[joint] - low[joint] < 32:
            low[joint], high[joint] = 2048 - 1000, 2048 + 1000
        cals[joint] = MotorCalibration(servo_id, 0, offsets[joint], low[joint], high[joint])
        cals[joint].validate(joint, servo_id)
        bus.write(servo_id, ft.MIN_POSITION_LIMIT, low[joint])
        bus.write(servo_id, ft.MAX_POSITION_LIMIT, high[joint])
    out = Path(args.out) if args.out else default_calibration_path(args.robot_id, args.arm, False, {})
    calib.save(out, cals)
    print(f"saved {out}")
    return 0


FROM_TOLERANCE_DEG = 3.0


def cmd_hello(args: argparse.Namespace) -> int:
    """Nod wrist_flex by --degrees toward the middle of its range and back. Dry run unless --go.

    The dry run prints the exact ``--go --from`` command. ``--go`` refuses to move when
    the wrist is no longer where the dry run saw it, so the move that runs is the one
    the operator approved.
    """
    arms = _arms(args)
    if args.arm not in arms:
        raise SystemExit(f"unknown arm {args.arm}; have {sorted(arms)}")
    arm = arms[args.arm]
    controller = So101Controller(arms, limits=Limits(max_speed_deg_s=args.speed), heartbeat_timeout_ms=0)
    # Dry run reads only: preflight, calibration check and the limit check write nothing.
    torque = arm.preflight()
    if not args.sim:
        arm.verify_calibration()
    start = arm.read_joints()["wrist_flex"]
    step = abs(args.degrees)
    target = start - step if start >= 0 else start + step  # toward 0°, away from the nearer limit
    check_target({"wrist_flex": target}, arm.bounds(), controller.limits, controller.named_poses["rest"])
    print(f"plan: arm {args.arm} wrist_flex {start:.1f}° → {target:.1f}° → back, "
          f"≤ {args.speed:.0f}°/s; target inside limits; abort with Ctrl-C (holds pose)")
    if not args.go:
        state = "on" if any(torque.values()) else "off"
        print(f"dry run: read only, nothing written or moved (torque {state}).")
        print(f"Clear the workspace, then run the same command with: --go --from {start:.1f}")
        return 0
    if args.start_deg is None:
        raise SystemExit("--go needs --from <wrist_flex from the dry run>; run the dry run first")
    if abs(start - args.start_deg) > FROM_TOLERANCE_DEG:
        raise SystemExit(
            f"refusing to move: wrist_flex is {start:.1f}°, the dry run saw {args.start_deg:.1f}°. "
            "The arm moved since the dry run; run the dry run again."
        )
    for warning in controller.bring_up(write_calibration=args.sim):
        print("warning:", warning)

    def run(verb: str, params: dict) -> dict:
        response = controller.handle(CmdRequest(verb, verb, {"arm": args.arm, **params}, None, "1.0.0", None))
        while response is None:
            time.sleep(0.02)
            done = controller.tick().responses
            response = done[0] if done else None
        if not response["ok"]:
            print(f"{verb}: {response['code']} {response['msg']}")
            print("stopped: torque is on and holds where the arm stopped. "
                  "Check the arm, then use torque-off with the arm supported or at rest.")
            raise SystemExit(1)
        return response

    try:
        run(f"{NS}.enable_torque", {})
        run(f"{NS}.move_to_joint_state", {"joints": {"wrist_flex": target}})
        run(f"{NS}.move_to_joint_state", {"joints": {"wrist_flex": start}})
    except KeyboardInterrupt:
        controller.handle(CmdRequest("abort", "robot.estop", {"reason": "operator abort"}, None, "1.0.0", None))
        print("aborted: holding the current pose")
        return 1
    print("done: torque stays on and holds the pose. Use torque-off with the arm supported.")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="so-arm101", description=__doc__.split("\n")[0])
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--sim", action="store_true", help="use simulated servos")
    parser.add_argument("--robot-id", default="so101")
    parser.add_argument("--calibration-dir", help="directory of <robot-id>[_<arm>].json files")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("scan", help="list servos that answer (read-only)")
    p.add_argument("--ids", default="1-15", help="id range, e.g. 1-15")
    p.set_defaults(func=cmd_scan)

    for name, func, text in (("read", cmd_read, "print calibrated joint positions (read-only)"),
                             ("torque-off", cmd_torque_off, "make the arms limp")):
        p = sub.add_parser(name, help=text)
        p.add_argument("--arms", default="arm:1")
        p.set_defaults(func=func)

    p = sub.add_parser("calibrate", help="record offsets and joint limits for one arm")
    p.add_argument("--arm", required=True)
    p.add_argument("--first-id", type=int, required=True)
    p.add_argument("--out", help="calibration JSON path (default: LeRobot cache)")
    p.set_defaults(func=cmd_calibrate)

    p = sub.add_parser("hello", help="small wrist nod toward mid-range (dry run unless --go --from)")
    p.add_argument("--arms", default="arm:1")
    p.add_argument("--arm", default="arm")
    p.add_argument("--degrees", type=float, default=10.0)
    p.add_argument("--speed", type=float, default=20.0)
    p.add_argument("--go", action="store_true", help="really move the arm")
    p.add_argument("--from", dest="start_deg", type=float,
                   help="wrist_flex reported by the dry run; --go refuses if the arm has moved since")
    p.set_defaults(func=cmd_hello)

    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except (ft.FeetechError, calib.CalibrationError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
