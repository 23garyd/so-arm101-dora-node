"""Node configuration from environment variables (set in the dora dataflow)."""

from __future__ import annotations

import json
import os
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from so_arm101_node.arm import ArmConfig
from so_arm101_node.calibration import JOINTS


@dataclass(frozen=True)
class NodeConfig:
    robot_id: str
    backend: str  # "serial" or "sim"
    port: str
    baudrate: int
    arms: tuple[ArmConfig, ...]
    calibration: dict[str, Path]
    heartbeat_timeout_ms: int
    estop_action: str
    max_speed_deg_s: float
    state_hz: float
    write_calibration: bool
    rest_pose: dict[str, float] | None = None


def _parse_arms(spec: str) -> tuple[ArmConfig, ...]:
    """``"left:1,right:7"`` → two arms with servo ids 1-6 and 7-12."""
    arms = []
    for item in spec.split(","):
        name, _, first = item.strip().partition(":")
        if not name or not first.isdigit():
            raise ValueError(f"SO101_ARMS entry {item!r} must look like name:first_servo_id")
        arms.append(ArmConfig.from_first_id(name, int(first)))
    ids = [i for arm in arms for i in arm.ids]
    if len(ids) != len(set(ids)):
        raise ValueError(f"SO101_ARMS {spec!r} assigns a servo id to two arms")
    return tuple(arms)


def _parse_rest(raw: str | None) -> dict[str, float] | None:
    """``SO101_REST_POSE``: JSON of all six joints, e.g. measured from the arm at rest."""
    if not raw:
        return None
    pose = json.loads(raw)
    missing = set(JOINTS) - set(pose)
    if missing:
        raise ValueError(f"SO101_REST_POSE is missing joints {sorted(missing)}")
    return {j: float(pose[j]) for j in JOINTS}


def _calibration_root(env: Mapping[str, str]) -> Path:
    base = env.get("HF_LEROBOT_CALIBRATION") or str(Path.home() / ".cache/huggingface/lerobot/calibration")
    return Path(base) / "robots" / "so101_follower"


def default_calibration_path(robot_id: str, arm: str, single: bool, env: Mapping[str, str]) -> Path:
    stem = robot_id if single else f"{robot_id}_{arm}"
    return _calibration_root(env) / f"{stem}.json"


def from_env(env: Mapping[str, str] | None = None) -> NodeConfig:
    env = os.environ if env is None else env
    robot_id = env.get("ROBOT_ID", "so101")
    backend = env.get("SO101_BACKEND", "serial")
    if backend not in ("serial", "sim"):
        raise ValueError("SO101_BACKEND must be 'serial' or 'sim'")
    arms = _parse_arms(env.get("SO101_ARMS", "arm:1"))
    overrides = {}
    for item in filter(None, (s.strip() for s in env.get("SO101_CALIBRATION", "").split(","))):
        name, _, path = item.partition("=")
        overrides[name] = Path(path).expanduser()
    unknown = set(overrides) - {a.name for a in arms}
    if unknown:
        raise ValueError(f"SO101_CALIBRATION names unknown arms {sorted(unknown)}")
    calibration = {
        a.name: overrides.get(a.name) or default_calibration_path(robot_id, a.name, len(arms) == 1, env)
        for a in arms
    }
    return NodeConfig(
        robot_id=robot_id,
        backend=backend,
        port=env.get("SO101_PORT", "/dev/ttyACM0"),
        baudrate=int(env.get("SO101_BAUD", "1000000")),
        arms=arms,
        calibration=calibration,
        heartbeat_timeout_ms=int(env.get("HEARTBEAT_TIMEOUT_MS", "1000")),
        estop_action=env.get("ESTOP_ACTION", "hold"),
        max_speed_deg_s=float(env.get("MAX_SPEED_DEG_S", "30")),
        state_hz=float(env.get("STATE_HZ", "10")),
        write_calibration=env.get("SO101_WRITE_CALIBRATION", "0") == "1",
        rest_pose=_parse_rest(env.get("SO101_REST_POSE")),
    )
