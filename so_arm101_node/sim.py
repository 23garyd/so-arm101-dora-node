"""In-memory STS3215 servos behind the same byte protocol as the real bus.

``SimulatedServos`` implements the ``Transport`` protocol, so ``FeetechBus`` runs
its real packet code against it. The node's ``sim`` backend and the tests use it;
it is a protocol-level stand-in, not a physics simulator.

Behaviour that matters to callers:
- present position = encoder - homing offset (sign-magnitude, like the STS3215);
- with torque on, the encoder slews toward the goal at ``slew_ticks_per_s``;
- a servo can be told to stall (``block``) to emulate a collision or a held object;
- a servo id can be removed to emulate a missing or unpowered motor;
- present load (drive duty cycle) and current are plausible, not measured: zero with
  torque off, a small holding load, a moderate load while slewing, and the torque
  limit while stalled against a block (what a gripper holding an object reports);
  both carry the drive direction as their sign.
"""

from __future__ import annotations

import time
from collections.abc import Callable, Iterable

from so_arm101_node import feetech as ft

# Simulated effort, in Present_Load units (0.1 % drive duty cycle).
HOLD_LOAD = 20
SLEW_LOAD = 150
CURRENT_PER_LOAD = 0.4  # raw current units per load unit: 100 % load ~ 400 x 6.5 mA = 2.6 A
IDLE_CURRENT = 2  # raw units with torque on and no load


class _Servo:
    def __init__(self, servo_id: int, encoder: int) -> None:
        self.regs = bytearray(256)
        self.encoder = float(encoder)
        self.blocked_at: float | None = None
        self.regs[ft.MODEL_NUMBER[0]:ft.MODEL_NUMBER[0] + 2] = ft.STS3215_MODEL.to_bytes(2, "little")
        self.regs[5] = servo_id
        self.set(ft.MAX_POSITION_LIMIT, 4095)
        self.set(ft.MAX_TORQUE_LIMIT, 1000)
        self.set(ft.LOCK, 1)
        self.set(ft.PRESENT_VOLTAGE, 120)
        self.set(ft.PRESENT_TEMPERATURE, 30)

    def get(self, register: tuple[int, int]) -> int:
        address, size = register
        return int.from_bytes(self.regs[address:address + size], "little")

    def set(self, register: tuple[int, int], value: int) -> None:
        address, size = register
        self.regs[address:address + size] = value.to_bytes(size, "little")

    def homing_offset(self) -> int:
        return ft.decode_sign_magnitude(self.get(ft.HOMING_OFFSET), 11)

    def present(self) -> int:
        return round(self.encoder) - self.homing_offset()


class SimulatedServos:
    """A bus full of simulated STS3215 servos (default: centred, torque off)."""

    def __init__(
        self,
        ids: Iterable[int],
        *,
        slew_ticks_per_s: float = 2000.0,
        clock: Callable[[], float] = time.monotonic,
        start_encoder: int = 2048,
    ) -> None:
        self.servos = {i: _Servo(i, start_encoder) for i in ids}
        self.slew = slew_ticks_per_s
        self._clock = clock
        self._last = clock()
        self._out = bytearray()
        self.writes: list[tuple[int, int, int]] = []  # (id, address, value) for assertions

    # -- test helpers ------------------------------------------------------

    def set_encoder(self, servo_id: int, encoder: int) -> None:
        self.servos[servo_id].encoder = float(encoder)

    def block(self, servo_id: int, encoder: float | None = None) -> None:
        servo = self.servos[servo_id]
        servo.blocked_at = servo.encoder if encoder is None else float(encoder)

    def unblock(self, servo_id: int) -> None:
        self.servos[servo_id].blocked_at = None

    def remove(self, servo_id: int) -> None:
        del self.servos[servo_id]

    def torque(self, servo_id: int) -> bool:
        return bool(self.servos[servo_id].get(ft.TORQUE_ENABLE))

    def goal(self, servo_id: int) -> int:
        return self.servos[servo_id].get(ft.GOAL_POSITION)

    def wheel_speed(self, servo_id: int) -> int:
        """Signed commanded wheel speed in ticks (0 unless in wheel mode with torque on)."""
        servo = self.servos[servo_id]
        if servo.get(ft.OPERATING_MODE) != 1 or not servo.get(ft.TORQUE_ENABLE):
            return 0
        return ft.decode_sign_magnitude(servo.get(ft.GOAL_SPEED), 15)

    def advance(self) -> None:
        now = self._clock()
        dt = max(0.0, now - self._last)
        self._last = now
        step = self.slew * dt
        for servo in self.servos.values():
            if not servo.get(ft.TORQUE_ENABLE):
                continue
            if servo.get(ft.OPERATING_MODE) == 1:  # wheel mode: spin at the goal speed
                servo.encoder = (servo.encoder + ft.decode_sign_magnitude(servo.get(ft.GOAL_SPEED), 15) * dt) % 4096
                continue
            target_encoder: float = servo.get(ft.GOAL_POSITION) + servo.homing_offset()
            if servo.blocked_at is not None:
                target_encoder = servo.blocked_at
            delta = target_encoder - servo.encoder
            servo.encoder += max(-step, min(step, delta))

    # -- Transport ---------------------------------------------------------

    def reset_input_buffer(self) -> None:
        self._out.clear()

    def read(self, size: int) -> bytes:
        data = bytes(self._out[:size])
        del self._out[:size]
        return data

    def write(self, data: bytes) -> int:
        self.advance()
        if len(data) < 6 or data[0:2] != b"\xff\xff":
            return len(data)
        servo_id, length, instruction = data[2], data[3], data[4]
        params = data[5:5 + length - 2]
        if ft.checksum(data[2:-1]) != data[-1]:
            return len(data)
        if instruction == ft.INSTR_SYNC_WRITE:
            self._sync_write(params)
            return len(data)
        servo = self.servos.get(servo_id)
        if servo is None:
            return len(data)  # silent: no such servo on the bus
        if instruction == ft.INSTR_PING:
            self._reply(servo_id, b"")
        elif instruction == ft.INSTR_READ:
            address, size = params[0], params[1]
            self._refresh_present(servo)
            self._reply(servo_id, bytes(servo.regs[address:address + size]))
        elif instruction == ft.INSTR_WRITE:
            self._apply(servo, params[0], bytes(params[1:]))
            self._reply(servo_id, b"")
        return len(data)

    def _sync_write(self, params: bytes) -> None:
        address, size = params[0], params[1]
        body = params[2:]
        for i in range(0, len(body), size + 1):
            servo = self.servos.get(body[i])
            if servo is not None:
                self._apply(servo, address, bytes(body[i + 1:i + 1 + size]))

    def _apply(self, servo: _Servo, address: int, value: bytes) -> None:
        servo.regs[address:address + len(value)] = value
        self.writes.append((servo.regs[5], address, int.from_bytes(value, "little")))

    def _refresh_present(self, servo: _Servo) -> None:
        servo.set(ft.PRESENT_POSITION, ft.encode_sign_magnitude(servo.present(), 15))
        goal = servo.get(ft.GOAL_POSITION) + servo.homing_offset()
        moving = servo.get(ft.TORQUE_ENABLE) and abs(goal - servo.encoder) > 1
        servo.set(ft.MOVING, 1 if moving else 0)
        load = self._load(servo, goal)
        servo.set(ft.PRESENT_LOAD, ft.encode_sign_magnitude(load, ft.LOAD_SIGN_BIT))
        current = 0
        if servo.get(ft.TORQUE_ENABLE):
            current = round(abs(load) * CURRENT_PER_LOAD) + IDLE_CURRENT
            current = -current if load < 0 else current  # current flows with the drive direction
        servo.set(ft.PRESENT_CURRENT, ft.encode_sign_magnitude(current, ft.CURRENT_SIGN_BIT))

    @staticmethod
    def _load(servo: _Servo, goal_encoder: float) -> int:
        if not servo.get(ft.TORQUE_ENABLE):
            return 0
        limit = min(servo.get(ft.MAX_TORQUE_LIMIT), 1000)
        if servo.get(ft.OPERATING_MODE) == 1:  # wheel mode
            speed = ft.decode_sign_magnitude(servo.get(ft.GOAL_SPEED), 15)
            return 0 if speed == 0 else (SLEW_LOAD if speed > 0 else -SLEW_LOAD)
        error = goal_encoder - servo.encoder
        if abs(error) <= 1:
            return min(HOLD_LOAD, limit)
        sign = 1 if error > 0 else -1
        if servo.blocked_at is not None:
            return sign * limit  # pushing against the block at the torque limit
        return sign * min(SLEW_LOAD, limit)

    def _reply(self, servo_id: int, payload: bytes) -> None:
        body = bytes([servo_id, len(payload) + 2, 0]) + payload
        self._out += b"\xff\xff" + body + bytes([ft.checksum(body)])
