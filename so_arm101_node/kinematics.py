"""Forward kinematics of the SO-101 arm to its gripper TCP (pure Python).

Joint origins come from the official new-calibration URDF,
TheRobotStudio/SO-ARM100 ``Simulation/SO101/so101_new_calib.urdf`` at commit
7629d2ad (Apache-2.0), the model AISLE pins (ADR-27). Joint zero is LeRobot's
calibrated middle, so the node's degrees feed in directly. Every revolute joint
turns about its local z axis; URDF ``rpy`` is applied as Rz(yaw)·Ry(pitch)·Rx(roll).

The result is the ``gripper_frame_link`` origin in ``base_link`` coordinates
(metres; z up from the bottom of the base, i.e. the mounting surface).
"""

from __future__ import annotations

import math

from so_arm101_node.calibration import BODY_JOINTS

Matrix = list[list[float]]

# (joint, xyz, rpy) parent → child, base_link … gripper_frame_link.
CHAIN: tuple[tuple[str, tuple[float, float, float], tuple[float, float, float]], ...] = (
    ("shoulder_pan", (0.0388353, -8.97657e-09, 0.0624), (3.14159, 4.18253e-17, -3.14159)),
    ("shoulder_lift", (-0.0303992, -0.0182778, -0.0542), (-1.5708, -1.5708, 0.0)),
    ("elbow_flex", (-0.11257, -0.028, 1.73763e-16), (-3.63608e-16, 8.74301e-16, 1.5708)),
    ("wrist_flex", (-0.1349, 0.0052, 3.62355e-17), (4.02456e-15, 8.67362e-16, -1.5708)),
    ("wrist_roll", (5.55112e-17, -0.0611, 0.0181), (1.5708, 0.0486795, 3.14159)),
    ("gripper_frame_joint", (-0.0079, -0.000218121, -0.0981274), (0.0, 3.14159, 0.0)),
)


def _matmul(a: Matrix, b: Matrix) -> Matrix:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _origin(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> Matrix:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rot_z(q: float) -> Matrix:
    c, s = math.cos(q), math.sin(q)
    return [[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


_ORIGINS = {name: _origin(xyz, rpy) for name, xyz, rpy in CHAIN}


def tcp_position(joints_deg: dict[str, float]) -> tuple[float, float, float]:
    """Gripper TCP (x, y, z) in metres for body joint angles in degrees."""
    t: Matrix = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    for name, _, _ in CHAIN:
        t = _matmul(t, _ORIGINS[name])
        if name in BODY_JOINTS:
            t = _matmul(t, _rot_z(math.radians(joints_deg[name])))
    return t[0][3], t[1][3], t[2][3]
