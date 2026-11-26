# so-arm101-dora-node

A [dora-rs](https://github.com/dora-rs/dora) node that exposes one or more
**SO-ARM101** arms (5 joints + gripper, Feetech STS3215 servos) as a
SPEC-VENDOR-NODE-V1 robot. Put the octos SPEC bridge from
[octos-robot-skills](https://github.com/dorarobotics/octos-robot-skills) in front of
it and the arms become HTTP tools and MCP tools for octos, Claude Code, Cursor or
an AISLE harness.

It is written for the Nano: two SO-ARM101 arms and a three-wheel base on **one
serial bus**, wired like a LeKiwi. This package drives only the arms; the bus code
and arm classes are shared libraries so a Nano node can add the base on the same
line later.

## What it provides

| Verb | Tier | What it does |
|---|---|---|
| `robot.heartbeat` | observe | Feeds the watchdog (the bridge sends it automatically) |
| `robot.get_capabilities` | observe | The capabilities advert |
| `robot.estop` | emergency_override | Cancel motion and hold the pose (`ESTOP_ACTION=torque_off` drops torque instead); latches |
| `vendor.lerobot.so101.stop` | emergency_override | Cancel motion and hold, without latching |
| `vendor.lerobot.so101.get_joint_state` | observe | Joint positions, torque and motion state |
| `vendor.lerobot.so101.clear_estop` | safe_motion | Allow motion again after an estop |
| `vendor.lerobot.so101.enable_torque` | safe_motion | Energise and hold the current pose (never jumps) |
| `vendor.lerobot.so101.disable_torque` | safe_motion | Go limp; refused unless near the rest pose (`force=true` overrides) |
| `vendor.lerobot.so101.move_to_joint_state` | safe_motion | Move to joint targets: `{"arm": "left", "joints": {"shoulder_pan": 20}}`; `dry_run` (below) |
| `vendor.lerobot.so101.move_to_named` | safe_motion | `rest` or `home`; `dry_run` (below) |
| `vendor.lerobot.so101.gripper.set` / `.open` / `.close` | safe_motion | Gripper percent (0 closed, 100 open) |

Units: body joints in **degrees** around the calibrated middle, the gripper in
**percent open** (LeRobot `DEGREES` and `RANGE_0_100`). With several arms, motion
verbs need `"arm"`; observation and stop verbs default to every arm.

Motion verbs answer when the move has finished and settled. A gripper that closes
on an object reports `ok` with `gripper_blocked: true`.

`move_to_joint_state` and `move_to_named` take `"dry_run": true` to check a move
without making it. The node runs the same limit and floor checks, plans the
trajectory and answers at once; nothing is written to the servos. The answer
has the full `goal`, `duration_s`, the gripper TCP height (`tcp_z_m`: `start`,
`goal`, and the `lowest` point on the path with `lowest_at` as a 0–1 fraction)
and `blockers`. A dry run also works while the arm is limp or estopped, and
lists `NOT_ENABLED` / `ESTOPPED` in `blockers` because the real move would be
refused. A dry run that fails a check returns the same error code and message
as the real move would. It is refused while the arm is moving
(`CONTROLLER_BUSY`), since the start pose is not settled.

### Safety behaviour

Defaults follow the SO-101 safety rules of Google DeepMind's robotics Antigravity
plugin (rule R5):

- Cosine-eased moves at **30 °/s** (`MAX_SPEED_DEG_S`, ceiling 90 °/s).
- Targets must stay **5° inside** the calibrated range; the rest pose is always reachable.
- A body joint lagging its command by more than **20°** (collision or stall)
  aborts the move with `TRACKING_ERROR`, holds, and emits a `tracking_error` safety event.
- A **1 s heartbeat watchdog** (`HEARTBEAT_TIMEOUT_MS`) estops a moving arm if the
  bridge goes silent. The bridge reads the timeout from the advert and sends heartbeats itself.
- A move whose gripper TCP would dip below **5 mm** above the base's mounting
  surface (`TCP_Z_FLOOR_M`) anywhere along its path is refused before anything
  moves. Forward kinematics uses the official SO-101 new-calibration URDF.
  Staying level or rising is never refused, even from below the floor, and the
  rest pose is always reachable. This checks the TCP only: the jaws and wrist can
  still touch the table or the arm's own links, which the tracking guard catches.
- Motion requires `enable_torque` first. Calibration in the servo EEPROM must match
  the calibration file, or the node refuses to start.

## Install

```bash
python3 -m venv .venv && source .venv/bin/activate     # 3.10+
# dora 1.0.1 or 1.1.0; the CLI and dora-rs must be the same release, and match the bridge's venv.
# 1.1.0 needs Python 3.11+; for 1.0.1 on 3.10 use the vendored wheels in octos-robot-skills/vendor/wheels.
pip install "dora-rs==1.1.0" "dora-rs-cli==1.1.0"
pip install -e ".[dev]" -e ../octos-robot-skills/bridge
```

## Bring up the hardware

See [docs/bring-up.md](docs/bring-up.md). Short version, read-only steps first:

```bash
so-arm101 --port /dev/ttyACM0 scan --ids 1-15                 # Nano: 1-12 arms, 13-15 wheels
so-arm101 --robot-id nano calibrate --arm left --first-id 1   # once per arm, LeRobot format
so-arm101 --robot-id nano calibrate --arm right --first-id 7
so-arm101 --robot-id nano read --arms left:1,right:7
so-arm101 --robot-id nano hello --arms left:1,right:7 --arm left          # dry run
so-arm101 --robot-id nano hello --arms left:1,right:7 --arm left --go --from <deg>   # moves
```

Every command takes `--sim` to run against simulated servos.

## Run in a dataflow

```bash
export SO101_VENV=$PWD/.venv
dora run examples/dataflow-sim.yml            # simulated servos, no hardware
# or examples/dataflow-real.yml on the robot
curl -s http://127.0.0.1:8773/tools | jq '.tools[].name'
curl -s -X POST http://127.0.0.1:8773/tools/vendor.lerobot.so101.enable_torque \
     -H 'Content-Type: application/json' -d '{"args":{}}'
curl -s -X POST http://127.0.0.1:8773/tools/vendor.lerobot.so101.move_to_named \
     -H 'Content-Type: application/json' -d '{"args":{"arm":"left","name":"home"}}'
```

MCP clients connect to `http://127.0.0.1:8773/mcp` (see
`octos-robot-skills/docs/mcp-clients.md`).

### Configuration

| Variable | Default | Meaning |
|---|---|---|
| `ROBOT_ID` | `so101` | Must equal the bridge's `ROBOT_ID` |
| `SO101_BACKEND` | `serial` | `serial`, `sim`, or `aisle` (AISLE's SO-101 scene, see below) |
| `SO101_PORT`, `SO101_BAUD` | `/dev/ttyACM0`, `1000000` | Serial bus |
| `SO101_ARMS` | `arm:1` | `name:first_servo_id` list; Nano `left:1,right:7` |
| `SO101_CALIBRATION` | LeRobot cache | `left=/path.json,right=/path.json`; default `~/.cache/huggingface/lerobot/calibration/robots/so_follower/<ROBOT_ID>[_<arm>].json`, or the older `so101_follower/` directory |
| `SO101_REST_POSE` | reference pose | JSON of all six joints, measured from your arm at rest |
| `HEARTBEAT_TIMEOUT_MS` | `1000` | Watchdog; `0` disables |
| `ESTOP_ACTION` | `hold` | `hold` or `torque_off` |
| `MAX_SPEED_DEG_S` | `30` | Up to 90 |
| `STATE_HZ` | `10` | `robot.state` rate |
| `TCP_Z_FLOOR_M` | `0.005` | Lowest gripper TCP height in metres above the base's mounting surface; `none` disables. Measure the table height per rig |
| `SO101_BASE` | `none` | Three-wheel omni base: `none`, `sim`, or `serial` (same bus as the arms) |
| `BASE_IDS` | `13,14,15` | Wheel servo ids (left-front, back, right-front) |
| `BASE_MAX_TICKS` | `600` | Wheel speed cap in ticks (≈ 0.046 m/s); the three wheels scale together |
| `BASE_RADIUS_M` | `0.12` | Wheel distance from the centre, for `omega`; measure it on the robot |
| `BASE_COMMAND_TIMEOUT_S` | `0.5` | A `set_velocity` without `duration_s` stops after this unless repeated |

## Nano: two arms and the omni base on one bus

The Feetech line is half-duplex, so exactly one process may open it. On a Nano
this node owns the whole bus: `SO101_ARMS=left:1,right:7 SO101_BASE=serial`.
Do not run the `feetech-omni-car` plugin at the same time.

With the base enabled the node adds two verbs:

| Verb | Tier | What it does |
|---|---|---|
| `vendor.dora_nav.base.set_velocity` | safe_motion | `{"vx": m/s, "vy": m/s, "omega": rad/s}`. Without `duration_s` it is a deadman that stops after `BASE_COMMAND_TIMEOUT_S` unless repeated; with `duration_s` (≤ 10 s) the reply comes when the base has stopped |
| `vendor.dora_nav.base.stop` | emergency_override | Zero the wheels and drop wheel torque |

`robot.estop` and the heartbeat watchdog stop the base too. There is no wheel
odometry: speeds are commands, not measured motion.

### Developing the Nano with partial hardware

Any arm can run on simulated servos with `@sim`, and the base with
`SO101_BASE=sim`, while the rest uses the real bus. With one physical SO-101
follower on `/dev/ttyACM0`:

```bash
SO101_PORT=/dev/ttyACM0 SO101_ARMS=left:1,right:7@sim SO101_BASE=sim \
SO101_CALIBRATION=left=~/.cache/huggingface/lerobot/calibration/robots/so_follower/so101_follower.json \
dora run dataflows/so-arm101-bridge.yaml   # run from octos-robot-skills; it takes these settings from the environment
```

The real arm is `left`; `right` and the base answer from simulated servos. The
serial port is only opened when a real part needs it.

## AISLE backend: evaluate this node in AISLE's physics

`SO101_BACKEND=aisle` replaces the servos with AISLE's SO-101 dora topics, so AISLE
evaluates the same vendor node that ships to the robot:

| Topic | Direction | Contract |
|---|---|---|
| `joint_state` | in | Float32[6] radians: `shoulder_pan … wrist_roll, gripper` |
| `joint_cmd` | out | Float32[6] radians, same order; the gripper entry is the jaw angle |
| `gripper_cmd` | out | Float32[1] in [0, 1]; 0 open, 1 closed |
| `reset_done` | in | Optional; cancels a move in flight after an episode reset |

AISLE uses the official SO-101 new-calibration URDF, so node degrees are radians at
the calibrated middle; bounds come from the URDF limits, and `rest` defaults to
AISLE's home (arm zero, jaw open). Route `joint_cmd`/`gripper_cmd` through AISLE's
budget-guard. The node answers `NOT_READY` until the first `joint_state` arrives,
and treats a `joint_state` older than `AISLE_STATE_STALE_S` (1 s) as a hardware error.
One arm, no base.

`examples/dataflow-aisle-stub.yml` runs the node against a stand-in scene that checks
the topic contract; `tests/test_aisle_contract.py` also runs every command through
AISLE's real budget-guard when AISLE is installed (`pip install --no-deps -e <aisle>`,
Python 3.11+) and requires zero clamps across a workspace sweep.

The node's TCP floor uses the same URDF chain as AISLE's budget-guard (identical to
within 1 µm), so a target into the table is refused by the node before AISLE's guard
has to clamp it.

## Development

```bash
pytest            # protocol, calibration, limits, controller, dora pump, bridge HTTP contract
ruff check so_arm101_node tests && mypy
```

The bridge test runs when `octos-spec-bridge` is installed and checks the real
HTTP contract, including the bridge's own heartbeat thread.

## License

Apache-2.0. See [NOTICE](NOTICE) for adapted third-party work.
