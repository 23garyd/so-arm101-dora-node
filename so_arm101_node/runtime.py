"""Dora wiring: builds the arms from config and pumps events through the controller.

Inputs:  ``cmd_request`` (from the octos SPEC bridge), ``tick`` (a dora timer, e.g. 20 ms).
Outputs: ``cmd_response``, ``capabilities``, ``state``, ``safety_event``.
"""

from __future__ import annotations

import json
import logging
import time
from typing import Any

from so_arm101_node import calibration as calib
from so_arm101_node._envelope import (
    CmdRequest,
    InvalidEnvelope,
    cmd_response,
    parse_cmd_request,
    stream_envelope,
)
from so_arm101_node.aisle import AISLE_HOME, AisleArm, AisleLink
from so_arm101_node.arm import Arm, SoArm101
from so_arm101_node.base import OmniBase
from so_arm101_node.config import NodeConfig
from so_arm101_node.feetech import FeetechBus
from so_arm101_node.motion import Limits
from so_arm101_node.node import So101Controller
from so_arm101_node.sim import SimulatedServos

logger = logging.getLogger(__name__)

# The bridge may start after this node, so repeat the advert: often at first, then rarely.
ADVERT_STARTUP_PERIOD_S = 2.0
ADVERT_STARTUP_WINDOW_S = 10.0
ADVERT_PERIOD_S = 30.0


class _Buses:
    """The real serial bus (opened only if a part needs it) and one simulated bus."""

    def __init__(self, cfg: NodeConfig) -> None:
        self.cfg = cfg
        self.serial: FeetechBus | None = None
        self.sim: FeetechBus | None = None
        self.aisle: AisleLink | None = None

    def get(self, simulated: bool, ids: list[int]) -> FeetechBus:
        if simulated:
            if self.sim is None:
                arm_ids = [i for a in self.cfg.arms for i in a.ids]
                base_ids = list(self.cfg.base_config.ids) if self.cfg.base != "none" else []
                self.sim = FeetechBus(SimulatedServos(arm_ids + base_ids))
            return self.sim
        if self.serial is None:
            self.serial = FeetechBus.open_serial(self.cfg.port, self.cfg.baudrate)
        return self.serial

    def close(self) -> None:
        for bus in (self.serial, self.sim):
            if bus is not None:
                bus.close()


def build_controller(cfg: NodeConfig) -> tuple[So101Controller, _Buses]:
    """Build the arms (and base) for the configured backend.

    Serial and sim controllers are brought up here. An AISLE controller is not: the
    scene's first joint_state has to arrive first, so ``Pump`` brings it up then.
    """
    buses = _Buses(cfg)
    if cfg.backend == "aisle":
        buses.aisle = AisleLink(stale_s=cfg.aisle_stale_s)
        aisle_arms: dict[str, Arm] = {a.name: AisleArm(buses.aisle, a) for a in cfg.arms}
        controller = So101Controller(
            aisle_arms,
            robot_id=cfg.robot_id,
            limits=Limits(max_speed_deg_s=cfg.max_speed_deg_s),
            rest=cfg.rest_pose or AISLE_HOME,
            heartbeat_timeout_ms=cfg.heartbeat_timeout_ms,
            estop_action="hold",
            tcp_floor_m=cfg.tcp_floor_m,
        )
        return controller, buses
    all_sim = cfg.backend == "sim"
    arms: dict[str, Arm] = {}
    writes = set()
    for arm in cfg.arms:
        simulated = all_sim or arm.sim
        if simulated:
            cal = calib.default_sim(arm.ids)
            writes.add(arm.name)
        else:
            cal = calib.load(cfg.calibration[arm.name], arm.ids)
            if cfg.write_calibration:
                writes.add(arm.name)
        arms[arm.name] = SoArm101(buses.get(simulated, list(arm.ids)), arm, cal)
    base = None
    if cfg.base != "none":
        base = OmniBase(buses.get(all_sim or cfg.base == "sim", list(cfg.base_config.ids)), cfg.base_config)
    controller = So101Controller(
        arms,
        base=base,
        base_command_timeout_s=cfg.base_command_timeout_s,
        tcp_floor_m=cfg.tcp_floor_m,
        robot_id=cfg.robot_id,
        limits=Limits(max_speed_deg_s=cfg.max_speed_deg_s),
        heartbeat_timeout_ms=cfg.heartbeat_timeout_ms,
        rest=cfg.rest_pose,
        estop_action=cfg.estop_action,
    )
    for warning in controller.bring_up(write_calibration=writes):
        logger.warning(warning)
    return controller, buses


def decode(value: Any) -> dict[str, Any] | None:
    try:
        items = value.to_pylist() if hasattr(value, "to_pylist") else list(value)
        first = items[0]
        return json.loads(first) if isinstance(first, str) else dict(first)
    except (IndexError, TypeError, ValueError):
        return None


class Pump:
    """Turns dora events into controller calls and controller results into dora outputs."""

    def __init__(self, controller: So101Controller, send: Any, *, state_hz: float,
                 clock: Any = time.monotonic, aisle: AisleLink | None = None) -> None:
        self.c = controller
        self.aisle = aisle
        self.ready = aisle is None  # an AISLE controller comes up on the first joint_state
        self._send_raw = send
        self._clock = clock
        self._state_period = 1.0 / max(state_hz, 0.1)
        self._next_state = 0.0
        self._next_advert = 0.0
        self._started = clock()
        self._seq = {"state": 0, "safety_event": 0}

    def _send(self, output: str, envelope: dict[str, Any]) -> None:
        import pyarrow as pa  # lazy: the controller and CLI work without pyarrow

        self._send_raw(output, pa.array([json.dumps(envelope)]))

    def _stream(self, output: str, stream: str, schema: str, payload: dict[str, Any]) -> None:
        self._seq[output] += 1
        self._send(output, stream_envelope(source=self.c.robot_id, stream=stream, schema=schema,
                                           seq=self._seq[output], payload=payload))

    def on_request(self, raw: dict[str, Any] | None) -> None:
        try:
            request = parse_cmd_request(raw or {})
        except InvalidEnvelope as e:
            bad = CmdRequest(str((raw or {}).get("request_id", "")), "", {}, None, "1.0.0", None)
            self._send("cmd_response", cmd_response(bad, ok=False, code="INVALID_PARAMS", msg=str(e)))
            return
        if request.target not in (None, self.c.robot_id):
            return  # addressed to another robot on the same dataflow
        if not self.ready and request.verb not in ("robot.heartbeat", "robot.get_capabilities"):
            self._send("cmd_response", cmd_response(request, ok=False, code="NOT_READY",
                                                    msg="waiting for the first joint_state from AISLE"))
            return
        response = self.c.handle(request)
        if response is not None:
            self._send("cmd_response", response)
        for cancelled in self.c.take_cancelled_responses():
            self._send("cmd_response", cancelled)
        for event in self.c.take_request_events():
            self._stream("safety_event", "robot.safety_event", "robot.safety_event.v1", event)
        self._flush_aisle()

    def _flush_aisle(self) -> None:
        if self.aisle is None:
            return
        import pyarrow as pa  # lazy, as in _send

        for output, values in self.aisle.take_outbox():
            self._send_raw(output, pa.array(values, type=pa.float32()))

    def on_joint_state(self, value: Any) -> None:
        if self.aisle is None:
            return
        values = value.to_pylist() if hasattr(value, "to_pylist") else list(value)
        self.aisle.update(values)
        if not self.ready:
            for warning in self.c.bring_up():
                logger.warning(warning)
            self.ready = True

    def on_reset_done(self) -> None:
        self.c.on_reset("AISLE episode reset")
        for cancelled in self.c.take_cancelled_responses():
            self._send("cmd_response", cancelled)

    def on_tick(self) -> None:
        if not self.ready:
            if self._clock() >= self._next_advert:  # still advertise while waiting for AISLE
                self._next_advert = self._clock() + ADVERT_STARTUP_PERIOD_S
                self._send("capabilities", self.c.capabilities_advert())
            return
        result = self.c.tick()
        for response in result.responses:
            self._send("cmd_response", response)
        for event in result.safety_events:
            self._stream("safety_event", "robot.safety_event", "robot.safety_event.v1", event)
        now = self._clock()
        if now >= self._next_state:
            self._next_state = now + self._state_period
            self._stream("state", "robot.state", "vendor.lerobot.so101.state.v1", self.c.state_payload())
        if now >= self._next_advert:
            early = now - self._started < ADVERT_STARTUP_WINDOW_S
            self._next_advert = now + (ADVERT_STARTUP_PERIOD_S if early else ADVERT_PERIOD_S)
            self._send("capabilities", self.c.capabilities_advert())
        self._flush_aisle()

    def on_event(self, event: Any) -> bool:
        """Handle one dora event; return False when the dataflow is stopping."""
        etype = event["type"]
        if etype == "STOP":
            return False
        if etype != "INPUT":
            return True
        if event["id"] == "cmd_request":
            self.on_request(decode(event["value"]))
        elif event["id"] == "tick":
            self.on_tick()
        elif event["id"] == "joint_state":
            self.on_joint_state(event["value"])
        elif event["id"] == "reset_done":
            self.on_reset_done()
        return True
