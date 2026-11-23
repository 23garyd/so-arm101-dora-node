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
| `vendor.lerobot.so101.move_to_joint_state` | safe_motion | Move to joint targets: `{"arm": "left", "joints": {"shoulder_pan": 20}}` |
| `vendor.lerobot.so101.move_to_named` | safe_motion | `rest` or `home` |
| `vendor.lerobot.so101.gripper.set` / `.open` / `.close` | safe_motion | Gripper percent (0 closed, 100 open) |

Units: body joints in **degrees** around the calibrated middle, the gripper in
**percent open** (LeRobot `DEGREES` and `RANGE_0_100`). With several arms, motion
verbs need `"arm"`; observation and stop verbs default to every arm.

Motion verbs answer when the move has finished and settled. A gripper that closes
on an object reports `ok` with `gripper_blocked: true`.

### Safety behaviour

Defaults follow the SO-101 safety rules of Google DeepMind's robotics Antigravity
plugin (rule R5):

- Cosine-eased moves at **30 °/s** (`MAX_SPEED_DEG_S`, ceiling 90 °/s).
- Targets must stay **5° inside** the calibrated range; the rest pose is always reachable.
- A body joint lagging its command by more than **20°** (collision or stall)
  aborts the move with `TRACKING_ERROR`, holds, and emits a `tracking_error` safety event.
- A **1 s heartbeat watchdog** (`HEARTBEAT_TIMEOUT_MS`) estops a moving arm if the
  bridge goes silent. The bridge reads the timeout from the advert and sends heartbeats itself.
- Motion requires `enable_torque` first. Calibration in the servo EEPROM must match
  the calibration file, or the node refuses to start.

## Install

```bash
python3 -m venv .venv && source .venv/bin/activate     # 3.10+
# dora-rs 1.0.1: from PyPI on 3.11+, or the vendored cp310 wheel in octos-robot-skills/vendor/wheels
pip install "dora-rs==1.0.1" "dora-rs-cli==1.0.1"
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
so-arm101 --robot-id nano hello --arms left:1,right:7 --arm left --go     # moves
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
| `SO101_BACKEND` | `serial` | `serial` or `sim` |
| `SO101_PORT`, `SO101_BAUD` | `/dev/ttyACM0`, `1000000` | Serial bus |
| `SO101_ARMS` | `arm:1` | `name:first_servo_id` list; Nano `left:1,right:7` |
| `SO101_CALIBRATION` | LeRobot cache | `left=/path.json,right=/path.json`; default `~/.cache/huggingface/lerobot/calibration/robots/so_follower/<ROBOT_ID>[_<arm>].json`, or the older `so101_follower/` directory |
| `SO101_REST_POSE` | reference pose | JSON of all six joints, measured from your arm at rest |
| `HEARTBEAT_TIMEOUT_MS` | `1000` | Watchdog; `0` disables |
| `ESTOP_ACTION` | `hold` | `hold` or `torque_off` |
| `MAX_SPEED_DEG_S` | `30` | Up to 90 |
| `STATE_HZ` | `10` | `robot.state` rate |

## One bus, one owner

The Feetech line is half-duplex, so exactly one process may open it. On a Nano,
do not run this node and the `feetech-omni-car` plugin at the same time: both
open `/dev/ttyACM0`. The base belongs in the same process as the arms; that is
the next step for the Nano node.

## Development

```bash
pytest            # protocol, calibration, limits, controller, dora pump, bridge HTTP contract
ruff check so_arm101_node tests && mypy
```

The bridge test runs when `octos-spec-bridge` is installed and checks the real
HTTP contract, including the bridge's own heartbeat thread.

## License

Apache-2.0. See [NOTICE](NOTICE) for adapted third-party work.
