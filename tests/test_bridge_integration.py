"""End-to-end against the real octos SPEC bridge (HTTP + MCP), without dora.

Skipped unless ``octos-spec-bridge`` (from octos-robot-skills/bridge) is installed.
A small adapter stands in for the bridge's DoraLoop: it hands each published
cmd_request to this node's Pump and resolves the bridge's pending futures from the
node's cmd_response outputs, while a thread ticks the node at 50 Hz. The bridge's own
HeartbeatRunner reads the advert's heartbeat timeout and keeps the node's watchdog fed.
"""

from __future__ import annotations

import json
import threading
import time
from concurrent.futures import Future

import pytest

pytest.importorskip("octos_spec_bridge")
from fastapi.testclient import TestClient
from octos_spec_bridge.heartbeat import HeartbeatRunner
from octos_spec_bridge.http_api import create_app
from octos_spec_bridge.state_cache import StateCache

from so_arm101_node.config import from_env
from so_arm101_node.runtime import Pump, build_controller


class LoopAdapter:
    def __init__(self) -> None:
        controller, self.bus = build_controller(from_env({"SO101_BACKEND": "sim", "SO101_ARMS": "left:1,right:7"}))
        self.cache = StateCache()
        self._advert = None
        self._pending: dict[str, Future] = {}
        self._lock = threading.Lock()
        self.pump = Pump(controller, self._on_output, state_hz=10)
        self._stop = threading.Event()
        self.pump.on_tick()
        self._thread = threading.Thread(target=self._ticker, daemon=True)
        self._thread.start()

    def _on_output(self, output, value) -> None:
        env = json.loads(value.to_pylist()[0])
        if output == "capabilities":
            self._advert = env
        elif output == "state":
            self.cache.set_state(env)
        elif output == "safety_event":
            self.cache.append_safety_event(env)
        elif output == "cmd_response":
            fut = self._pending.pop(env["request_id"], None)
            if fut is not None:
                fut.set_result(env)

    def _ticker(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                self.pump.on_tick()
            time.sleep(0.02)

    # DoraLoopLike
    def advert(self):
        return self._advert

    def register_pending(self, request_id: str) -> Future:
        fut: Future = Future()
        self._pending[request_id] = fut
        return fut

    def cancel_pending(self, request_id: str) -> None:
        self._pending.pop(request_id, None)

    def publish_cmd_request(self, envelope) -> None:
        with self._lock:
            self.pump.on_request(envelope)

    def close(self) -> None:
        self._stop.set()
        self._thread.join()
        self.bus.close()


@pytest.fixture
def client():
    loop = LoopAdapter()
    heartbeat = HeartbeatRunner(dora_loop=loop, robot_id="so101")  # same wiring as the bridge's __main__
    heartbeat.start()
    app = create_app(dora_loop=loop, state_cache=loop.cache, robot_id="so101", cmd_timeout_s=20.0,
                     max_allowed_tier="safe_motion", mcp_enabled=False)
    try:
        yield TestClient(app)
    finally:
        heartbeat.stop()
        loop.close()


def call(client, tool: str, **args) -> dict:
    resp = client.post(f"/tools/{tool}", json={"args": args})
    assert resp.status_code == 200, resp.text
    return resp.json()


def test_tool_catalog_exposes_every_verb_under_safe_motion(client):
    tools = {t["name"]: t for t in client.get("/tools").json()["tools"]}
    assert "vendor.lerobot.so101.move_to_joint_state" in tools
    assert tools["robot.estop"]["safety_tier"] == "emergency_override"
    assert tools["vendor.lerobot.so101.get_joint_state"]["safety_tier"] == "observe"


def test_pick_like_sequence_over_http(client):
    ns = "vendor.lerobot.so101"
    assert call(client, f"{ns}.enable_torque")["ok"]
    r = call(client, f"{ns}.move_to_joint_state", arm="left", joints={"shoulder_pan": 25.0, "wrist_flex": 15.0})
    assert r["ok"], r
    assert r["data"]["joints"]["shoulder_pan"] == pytest.approx(25.0, abs=2.0)
    assert call(client, f"{ns}.gripper.close", arm="left")["ok"]
    r = call(client, f"{ns}.get_joint_state", arm="right")
    assert r["data"]["arms"]["right"]["torque"] is True
    state = call(client, "get_state")
    assert state["data"]["stream"]["payload"]["arms"]["left"]["torque"] is True
    assert call(client, "robot.estop", reason="test")["ok"]
    events = call(client, "get_recent_safety_events")["data"]["events"]
    assert {e["payload"]["arm"] for e in events} == {"left", "right"}
    r = call(client, f"{ns}.move_to_named", arm="left", name="home")
    assert r["code"] == "ESTOPPED"
