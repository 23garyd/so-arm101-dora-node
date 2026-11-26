"""Stop paths must not report success when the hardware command failed.

A fault-injecting port fails chosen writes the way a dropped USB-serial link does
(pyserial SerialException), and can recover later in the test.
"""

from __future__ import annotations

import serial

from so_arm101_node import feetech as ft
from so_arm101_node.node import BASE_NS, NS

from .conftest import make_rig
from .test_base import WHEELS, make_nano, speeds
from .test_controller import req, run


class FaultyPort:
    """Wrap the simulated bus; ``fail(...)`` makes matching writes raise until ``recover()``."""

    def __init__(self, sim):
        self.sim = sim
        self.rules: list[tuple[set[int] | None, int, int]] = []  # (servo ids, instruction, address)
        self.failed = 0

    def fail(self, address: int, *, ids=None, instruction=ft.INSTR_WRITE):
        self.rules.append((None if ids is None else set(ids), instruction, address))

    def fail_link(self, ids):
        """Every write to these servos fails (speed and torque alike)."""
        for address in (ft.GOAL_SPEED[0], ft.TORQUE_ENABLE[0]):
            self.fail(address, ids=ids)

    def recover(self):
        self.rules.clear()

    def write(self, data):
        if len(data) > 6:
            servo_id, instruction, address = data[2], data[4], data[5]
            for ids, instr, addr in self.rules:
                if instr == instruction and addr == address and (ids is None or servo_id in ids):
                    self.failed += 1
                    raise serial.SerialException("write failed: [Errno 5] Input/output error")
        return self.sim.write(data)

    def read(self, size):
        return self.sim.read(size)

    def reset_input_buffer(self):
        self.sim.reset_input_buffer()


def faulty(controller, sim) -> FaultyPort:
    port = FaultyPort(sim)
    bus = next(iter(controller.arms.values())).bus
    bus._io = port  # arms and base share one bus
    return port


def _tick(controller, clock, seconds):
    events, responses = [], []
    for _ in range(round(seconds / 0.02)):
        clock.advance(0.02)
        result = controller.tick()
        events += result.safety_events
        responses += result.responses
    return events, responses


# ---------------------------------------------------------------- arm


def test_stop_reports_a_failed_hold(clock):
    controller, sim = make_rig(clock)
    run(controller, clock, f"{NS}.enable_torque")
    assert controller.handle(req(f"{NS}.move_to_joint_state", joints={"shoulder_pan": 60.0})) is None
    _tick(controller, clock, 0.2)
    port = faulty(controller, sim)
    port.fail(ft.GOAL_POSITION[0], instruction=ft.INSTR_SYNC_WRITE)  # the hold write
    r = controller.handle(req(f"{NS}.stop"))
    assert port.failed and r["ok"] is False and r["code"] == "HARDWARE_ERROR"
    assert "hold" in r["msg"] and r["data"]["failed"]["left"]
    events = controller.take_request_events()
    assert any(e["kind"] == "hardware_error" and e["arm"] == "left" and e["action"] == "hold" for e in events)


def test_estop_hold_failure_is_reported_and_the_latch_stays(clock):
    controller, sim = make_rig(clock)
    run(controller, clock, f"{NS}.enable_torque")
    controller.handle(req(f"{NS}.move_to_joint_state", joints={"shoulder_pan": 60.0}))
    _tick(controller, clock, 0.2)
    port = faulty(controller, sim)
    port.fail(ft.GOAL_POSITION[0], instruction=ft.INSTR_SYNC_WRITE)
    r = controller.handle(req("robot.estop", reason="test"))
    assert r["ok"] is False and r["code"] == "HARDWARE_ERROR"
    assert r["data"]["estopped"] is True and r["data"]["failed"]["left"]
    events = controller.take_request_events()
    left = next(e for e in events if e.get("arm") == "left")
    assert left["kind"] == "estop_engaged" and left["ok"] is False and "hold" in left["failed"]
    port.recover()
    r = controller.handle(req(f"{NS}.move_to_joint_state", joints={"shoulder_pan": 0.0}))
    assert r["code"] == "ESTOPPED"  # latched despite the failed hardware action


def test_estop_torque_off_failure_is_reported_and_torque_stays_on(clock):
    controller, sim = make_rig(clock, estop_action="torque_off")
    run(controller, clock, f"{NS}.enable_torque")
    port = faulty(controller, sim)
    port.fail(ft.TORQUE_ENABLE[0], ids={3})  # one servo will not drop torque
    r = controller.handle(req("robot.estop", reason="test"))
    assert r["ok"] is False and r["code"] == "HARDWARE_ERROR" and r["data"]["estopped"] is True
    assert sim.torque(3)  # really still powered ...
    state = run(controller, clock, f"{NS}.get_joint_state")["data"]["arms"]["left"]
    assert state["torque"] is True  # ... and reported as powered, not as torque_off
    left = next(e for e in controller.take_request_events() if e.get("arm") == "left")
    assert left["ok"] is False and "torque_off" in left["failed"]


def test_estop_success_is_still_ok(clock):
    controller, _ = make_rig(clock, estop_action="torque_off")
    run(controller, clock, f"{NS}.enable_torque")
    r = controller.handle(req("robot.estop", reason="test"))
    assert r["ok"] is True and r["data"]["failed"] == {}
    assert all(e["ok"] for e in controller.take_request_events())


# ---------------------------------------------------------------- base


def test_timed_base_move_with_a_failed_stop_is_an_error_and_the_stop_is_retried(clock):
    controller, sim = make_nano(clock)
    request = req(f"{BASE_NS}.set_velocity", vx=0.02, duration_s=0.2)
    assert controller.handle(request) is None
    port = faulty(controller, sim)
    port.fail_link(WHEELS)
    events, responses = _tick(controller, clock, 0.3)
    mine = [r for r in responses if r["request_id"] == request.request_id]
    assert mine and mine[0]["ok"] is False and mine[0]["code"] == "HARDWARE_ERROR"
    assert any(e["kind"] == "hardware_error" and e.get("part") == "base" for e in events)
    assert speeds(sim) != (0, 0, 0)  # the wheels really still turn ...
    state = controller.state_payload()["base"]
    assert state["moving"] is True and state["ticks"] != [0, 0, 0]  # ... and are reported so
    port.recover()
    _tick(controller, clock, 0.1)  # the stop is retried once the bus works again
    assert speeds(sim) == (0, 0, 0)
    assert controller.state_payload()["base"]["moving"] is False


def test_base_stop_verb_failure_is_retried(clock):
    controller, sim = make_nano(clock)
    controller.handle(req(f"{BASE_NS}.set_velocity", vx=0.02, duration_s=5.0))
    port = faulty(controller, sim)
    port.fail_link(WHEELS)
    r = controller.handle(req(f"{BASE_NS}.stop"))
    assert r["ok"] is False and r["code"] == "HARDWARE_ERROR"
    port.recover()
    _tick(controller, clock, 0.06)  # retried promptly, not after the 5 s duration
    assert speeds(sim) == (0, 0, 0)


def test_failed_base_start_leaves_no_wheel_without_a_deadman(clock):
    controller, sim = make_nano(clock, base_command_timeout_s=0.5)
    controller.handle(req(f"{BASE_NS}.set_velocity", vx=0.02))  # wheels enabled and turning
    controller.handle(req(f"{BASE_NS}.stop"))
    controller.handle(req(f"{BASE_NS}.set_velocity", omega=0.1))
    port = faulty(controller, sim)
    port.fail_link([WHEELS[1]])  # the link to the 2nd wheel fails mid-start
    r = controller.handle(req(f"{BASE_NS}.set_velocity", vx=0.03))
    assert r["ok"] is False and r["code"] == "HARDWARE_ERROR"
    port.recover()
    _tick(controller, clock, 0.06)  # the base is stopped, not left on the 1st wheel's new speed
    assert speeds(sim) == (0, 0, 0)


def test_estop_base_stop_failure_is_reported_and_retried(clock):
    controller, sim = make_nano(clock)
    controller.handle(req(f"{BASE_NS}.set_velocity", vx=0.02, duration_s=5.0))
    port = faulty(controller, sim)
    port.fail_link(WHEELS)
    r = controller.handle(req("robot.estop", reason="test"))
    assert r["ok"] is False and r["data"]["failed"]["base"]
    base = next(e for e in controller.take_request_events() if e.get("part") == "base")
    assert base["ok"] is False
    port.recover()
    _tick(controller, clock, 0.06)
    assert speeds(sim) == (0, 0, 0)
