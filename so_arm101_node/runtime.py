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
from so_arm101_node.arm import SoArm101
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


def build_controller(cfg: NodeConfig) -> tuple[So101Controller, FeetechBus]:
    if cfg.backend == "sim":
        ids = [i for arm in cfg.arms for i in arm.ids]
        bus = FeetechBus(SimulatedServos(ids))
        cals = {arm.name: calib.default_sim(arm.ids) for arm in cfg.arms}
        write_cal = True
    else:
        bus = FeetechBus.open_serial(cfg.port, cfg.baudrate)
        cals = {arm.name: calib.load(cfg.calibration[arm.name], arm.ids) for arm in cfg.arms}
        write_cal = cfg.write_calibration
    arms = {arm.name: SoArm101(bus, arm, cals[arm.name]) for arm in cfg.arms}
    controller = So101Controller(
        arms,
        robot_id=cfg.robot_id,
        limits=Limits(max_speed_deg_s=cfg.max_speed_deg_s),
        heartbeat_timeout_ms=cfg.heartbeat_timeout_ms,
        rest=cfg.rest_pose,
        estop_action=cfg.estop_action,
    )
    for warning in controller.bring_up(write_calibration=write_cal):
        logger.warning(warning)
    return controller, bus


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
                 clock: Any = time.monotonic) -> None:
        self.c = controller
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
        response = self.c.handle(request)
        if response is not None:
            self._send("cmd_response", response)
        for cancelled in self.c.take_cancelled_responses():
            self._send("cmd_response", cancelled)
        if request.verb == "robot.estop":
            for name in self.c.arms:
                self._stream("safety_event", "robot.safety_event", "robot.safety_event.v1",
                             {"kind": "estop_engaged", "arm": name, "reason": self.c.estop_reason,
                              "action": self.c.estop_action})

    def on_tick(self) -> None:
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
        return True
