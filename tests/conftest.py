from __future__ import annotations

import pytest

from so_arm101_node import calibration as calib
from so_arm101_node.arm import ArmConfig, SoArm101
from so_arm101_node.feetech import FeetechBus
from so_arm101_node.motion import Limits
from so_arm101_node.node import So101Controller
from so_arm101_node.sim import SimulatedServos


class FakeClock:
    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


def make_rig(clock: FakeClock, arm_specs=(("left", 1),), **controller_kw):
    configs = [ArmConfig.from_first_id(name, first) for name, first in arm_specs]
    sim = SimulatedServos([i for c in configs for i in c.ids], clock=clock, slew_ticks_per_s=4000)
    bus = FeetechBus(sim)
    arms = {c.name: SoArm101(bus, c, calib.default_sim(c.ids)) for c in configs}
    controller_kw.setdefault("limits", Limits(max_speed_deg_s=60))
    controller = So101Controller(arms, clock=clock, **controller_kw)
    controller.bring_up(write_calibration=True)
    return controller, sim


@pytest.fixture
def rig(clock):
    return make_rig(clock)
