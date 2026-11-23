"""Dora entry point: ``python -m so_arm101_node`` inside a dataflow."""

from __future__ import annotations

import logging
import os

from so_arm101_node.config import from_env
from so_arm101_node.runtime import Pump, build_controller


def main() -> None:
    logging.basicConfig(level=os.environ.get("LOG_LEVEL", "INFO"))
    cfg = from_env()
    controller, bus = build_controller(cfg)
    from dora import Node  # lazy: tests and the CLI run without dora

    dora_node = Node()
    pump = Pump(controller, dora_node.send_output, state_hz=cfg.state_hz)
    pump.on_tick()  # publish the capabilities advert and a first state right away
    try:
        for event in dora_node:
            if not pump.on_event(event):
                break
    finally:
        bus.close()


if __name__ == "__main__":
    main()
