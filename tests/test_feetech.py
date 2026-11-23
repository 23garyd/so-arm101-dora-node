from __future__ import annotations

import pytest

from so_arm101_node import feetech as ft
from so_arm101_node.sim import SimulatedServos


def test_ping_packet_matches_feetech_reference():
    # FF FF 01 02 01 FB is the documented ping for servo 1.
    assert ft.build_packet(1, ft.INSTR_PING) == bytes.fromhex("ffff010201fb")


def test_sign_magnitude_round_trip():
    for value in (-2047, -1, 0, 1, 2047):
        assert ft.decode_sign_magnitude(ft.encode_sign_magnitude(value, 11), 11) == value
    assert ft.encode_sign_magnitude(-5, 15) == 0x8005
    with pytest.raises(ValueError):
        ft.encode_sign_magnitude(2048, 11)


def test_read_write_through_simulated_servo():
    bus = ft.FeetechBus(SimulatedServos([3]))
    assert bus.read(3, ft.MODEL_NUMBER) == ft.STS3215_MODEL
    bus.write(3, ft.GOAL_POSITION, 1234)
    assert bus.read(3, ft.GOAL_POSITION) == 1234
    bus.write_homing_offset(3, -100)
    assert bus.read_homing_offset(3) == -100
    # present = encoder - offset
    assert bus.read_position(3) == 2048 + 100


def test_missing_servo_raises_and_ping_is_false():
    bus = ft.FeetechBus(SimulatedServos([1]))
    assert bus.ping(1) is True
    assert bus.ping(9) is False
    with pytest.raises(ft.FeetechError, match="no reply"):
        bus.read(9, ft.MODEL_NUMBER)


def test_servo_fault_is_reported_not_retried():
    class Faulty(SimulatedServos):
        def _reply(self, servo_id, payload):
            body = bytes([servo_id, len(payload) + 2, 0x20]) + payload
            self._out += b"\xff\xff" + body + bytes([ft.checksum(body)])

    bus = ft.FeetechBus(Faulty([1]))
    with pytest.raises(ft.FeetechError, match="overload"):
        bus.read(1, ft.MODEL_NUMBER)


def test_corrupt_reply_is_rejected():
    class Corrupt(SimulatedServos):
        def read(self, size):
            data = bytearray(super().read(size))
            if data:
                data[-1] ^= 0xFF
            return bytes(data)

    bus = ft.FeetechBus(Corrupt([1]), retries=0)
    with pytest.raises(ft.FeetechError, match="checksum"):
        bus.read(1, ft.MODEL_NUMBER)


def test_sync_write_reaches_every_listed_servo():
    sim = SimulatedServos([1, 2, 7])
    bus = ft.FeetechBus(sim)
    bus.sync_write(ft.GOAL_POSITION, {1: 100, 7: 300})
    assert sim.goal(1) == 100
    assert sim.goal(2) == 0
    assert sim.goal(7) == 300
