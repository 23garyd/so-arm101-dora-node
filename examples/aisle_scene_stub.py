"""A stand-in for AISLE's SO-101 sim node, for wiring checks without a physics engine.

Speaks AISLE's TC-5 topics: consumes joint_cmd (Float32[6], radians) and gripper_cmd
(Float32[1] in [0, 1]) and raises on anything else, exactly as AISLE's sim node does;
publishes joint_state (Float32[6]) every tick, tracking the last command.
"""

from __future__ import annotations

import numpy as np
import pyarrow as pa
from dora import Node

JAW_OPEN = 1.74533


def main() -> None:
    q = np.array([0, 0, 0, 0, 0, JAW_OPEN], np.float32)
    target = q.copy()
    node = Node()
    for event in node:
        if event["type"] == "STOP":
            break
        if event["type"] != "INPUT":
            continue
        value = np.asarray(event["value"].to_numpy(zero_copy_only=False))
        if event["id"] == "joint_cmd":
            if value.dtype != np.float32 or value.shape != (6,):
                raise ValueError(f"joint_cmd must be Float32[6], got {value.dtype}{value.shape} (TC-5)")
            target = value.copy()
        elif event["id"] == "gripper_cmd":
            if value.dtype != np.float32 or value.shape != (1,) or not 0.0 <= float(value[0]) <= 1.0:
                raise ValueError(f"gripper_cmd must be Float32[1] in [0, 1], got {value!r}")
        elif event["id"] == "tick":
            q = q + 0.5 * (target - q)
            node.send_output("joint_state", pa.array(q, type=pa.float32()))


if __name__ == "__main__":
    main()
