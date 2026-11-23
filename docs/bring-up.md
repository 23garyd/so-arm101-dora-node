# SO-ARM101 bring-up

A staged procedure adapted from the SO-101 bring-up skill in Google DeepMind's
robotics Antigravity plugin (Apache-2.0). Read-only stages and dry runs advance
on their own; anything that energises a servo needs an explicit `--go` from the
operator. Keep the workspace clear and a hand near the power switch.

Examples use a Nano: `--robot-id nano`, arms `left:1,right:7`, base wheels on
ids 13-15. For a single arm, drop `--robot-id`/`--arms` (defaults `so101`, `arm:1`).

## Stage 0 — Install

Follow the README install. Make sure no other process holds the serial port
(stop the `feetech-omni-car` plugin, dorobot2 teleop and any LeRobot script).

Done when: `so-arm101 --help` runs from the venv.

## Stage 1 — Read-only bus check (torque off)

```bash
so-arm101 --port /dev/ttyACM0 scan --ids 1-15
```

Done when: every expected id answers as model 777 (STS3215) with torque 0, and
the voltage is in the 6-12 V range. A missing id means power, cabling or a wrong
servo id; fix it before going on.

## Stage 2 — Calibration (torque off)

Reuse an existing LeRobot calibration if it belongs to this arm (copy it to the
path in the README configuration table). Otherwise, for each arm:

```bash
so-arm101 --robot-id nano calibrate --arm left --first-id 1
```

1. Torque goes off. Move every joint by hand to the middle of its range, press Enter.
2. Sweep every joint slowly through its full range, including the gripper. Press Enter.

The tool writes offsets and limits to the servo EEPROM and the JSON file. Then:

```bash
so-arm101 --robot-id nano read --arms left:1,right:7
```

Done when: joint readings change smoothly in the expected direction as you move
each joint by hand, and read about 0° at the middle pose.

Put the arm in its rest pose and note the `read` output. Pass it to the node as
`SO101_REST_POSE` if it differs from the reference rest pose.

The node refuses moves that take the gripper TCP below `TCP_Z_FLOOR_M` (default
5 mm above the bottom of the base). If the arm is mounted above or below the work
surface, set the floor to the table's height in the base frame; Google's Stage 5
table probe measures it. A folded rest pose can sit below the floor: moves that
rise from it and moves back to rest are always allowed.

## Stage 3 — First motion

```bash
so-arm101 --robot-id nano hello --arms left:1,right:7 --arm left                       # dry run, prints the plan
so-arm101 --robot-id nano hello --arms left:1,right:7 --arm left --go --from <deg>      # nods the wrist 10° and back
```

The wrist nods toward the middle of its range, away from the nearer limit. The dry
run prints the exact `--go --from` value; `--go` refuses to move if the wrist is
more than 3° away from it, so the move that runs is the one you approved. Do not
reposition the arm between the dry run and `--go`.

If a joint is blocked before it settles, the move ends with `NOT_REACHED` and the
arm holds where it stopped instead of pushing on.

Ctrl-C during the move estops and holds the pose. Torque stays on afterwards;
support the arm before `so-arm101 torque-off`.

Done when: the wrist moves smoothly to the planned angle and back, with no
servo error.

## Stage 4 — Node and bridge

```bash
export SO101_VENV=$PWD/.venv
dora run examples/dataflow-real.yml
```

From another terminal, run `enable_torque`, `move_to_named home`, `gripper.open`,
`gripper.close`, `move_to_named rest` and `disable_torque` through the bridge
(see the README curl examples). Try `robot.estop` mid-move: the arm must hold.

Done when: every call returns `ok`, `get_state` shows live joints, and the estop
holds the arm and refuses motion until `clear_estop`.

## Not covered yet

Camera checks, hand-eye calibration (Google's `so101-calibration` Stage 5),
object localisation and visual servoing are separate skills and come next.
