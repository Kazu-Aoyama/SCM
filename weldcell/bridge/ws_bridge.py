#!/usr/bin/env python3
"""ws_bridge.py — WebSocket bridge server for the WeldCell platform (protocol 'weldcell-bridge/1').

The browser UI (stage ⑤ Sim⇄Real → 接続先: WebSocket) connects here. The server
relays the contract between the UI / policies and a BACKEND:

  * MockBackend  — virtual real machine (Python twin of sim2real.js MockRealBackend):
                   hidden "true" plant calibration, actuator lag, sensor noise.
  * Ros2Backend  — skeleton for a real cell (ROS 2 / rclpy). Maps the protocol to
                   topics; fill in the TODOs for your robot / welder / vision.

Usage:
    pip install websockets
    python3 bridge/ws_bridge.py --backend mock --port 8765 --rate 20
Messages (JSON):  {v:1, protocol, type, seq, t, data}   — see docs/PROTOCOL.md
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import random
import time
from typing import Any, Dict, Optional

import weldpool as W

PROTOCOL = "weldcell-bridge/1"
DEG = math.pi / 180


def msg(type_: str, data: Dict[str, Any], seq: int = 0, ack: Optional[int] = None) -> str:
    m = {"v": 1, "protocol": PROTOCOL, "type": type_, "seq": seq, "t": time.time(), "data": data}
    if ack is not None:
        m["ack"] = ack
    return json.dumps(m)


# ----------------------------------------------------------------- backends
class MockBackend:
    """Virtual real machine. Hidden plant differs from the nominal sim (Real2Sim target)."""

    name = "MockBackend(py)"

    def __init__(self, seed: int = 4242, truth: Optional[Dict[str, float]] = None):
        self.rng = random.Random(seed)
        self.truth = truth or {"effScale": 0.90, "kScale": 1.15, "sigScale": 1.10}
        self.process = {"I": 175.0, "V": 24.0, "eta": 0.80, "material": "steel", "T0": 20.0, "thickness": 0.009}
        self.sensor_noise, self.act_tau, self.pool_tau = 0.04, 0.25, 0.25
        self.reset()

    def reset(self):
        self.sp = {"travelSpeed": 0.007, "workRef": 45 * DEG, "workAngle": 45 * DEG, "travelAngle": -15 * DEG,
                   "standoff": 0.002, "lateralOffset": 0.0, "wristRoll": 0.0}
        self.cmd: Dict[str, float] = {}
        self.s, self.noise, self.nv, self.trk, self.t = 0.0, 0.0, 0.0, 0.0, 0.0
        self.obs = None
        self.arc, self.estopped = True, False

    def weld_input(self) -> Dict[str, Any]:
        sp, p = self.sp, dict(self.process)
        p["Q"] = p["eta"] * p["V"] * p["I"]
        return {"travelSpeed": sp["travelSpeed"], "workAngle": sp["workAngle"], "workDev": sp["workAngle"] - sp["workRef"],
                "travelAngle": sp["travelAngle"], "standoff": sp["standoff"], "weldPosition": "F", "process": p,
                "seamParam": self.s, "contact": self.arc, "weldPoint": [0, 0, self.s * 0.18], "travelDir": [0, 0, 1],
                "acrossDir": [1, 0, 0], "surfaceNormal": [0, 1, 0], "gravityDir": [0, -1, 0]}

    def control(self, d: Dict[str, float]):
        if not self.estopped:
            self.cmd.update({k: float(v) for k, v in d.items() if isinstance(v, (int, float))})

    def tick(self, dt: float):
        self.t += dt
        if self.cmd and not self.estopped:
            k = min(1.0, dt / self.act_tau)
            for key, v in self.cmd.items():
                if key in self.sp:
                    self.sp[key] += (v - self.sp[key]) * k
        if not self.estopped:
            self.nv += (-self.nv * 1.2 + (self.rng.random() - 0.5) * 0.04) * dt
            self.noise = max(-0.0016, min(0.0016, self.noise + self.nv * dt))
            seam_y = 0.008 * math.sin(math.pi * self.s) + self.noise
            self.sp["lateralOffset"] += 0.45 * (seam_y - self.sp["lateralOffset"]) * dt
            self.trk = seam_y - self.sp["lateralOffset"]
            self.s = min(1.0, self.s + self.sp["travelSpeed"] * dt / 0.18)
        r = W.evaluate(W.params_from_weld_input(self.weld_input(), self.truth))
        tgt = {"L": r["lenMM"] / 1000, "W": r["widMM"] / 1000, "area": r["areaMM"] / 1e6, "pen": r["penMM"] / 1000, "status": r["status"],
               "cx": r["cxMM"] / 1000, "cy": r["cyMM"] / 1000, "peakC": r["peakC"], "t85": r["t85"]}
        if self.obs is None or self.obs["status"] != "ok" or tgt["status"] != "ok":
            self.obs = tgt
        else:
            k = min(1.0, dt / self.pool_tau)
            for key in ("L", "W", "area", "pen", "cx", "cy"):
                self.obs[key] += (tgt[key] - self.obs[key]) * k
            self.obs.update(status=tgt["status"], peakC=tgt["peakC"], t85=tgt["t85"])

    def state(self) -> Dict[str, Any]:
        sp = self.sp
        pose = [0.0, sp["lateralOffset"], sp["standoff"], sp["wristRoll"], sp["travelAngle"], sp["workAngle"] - sp["workRef"]]
        return {"pose": pose, "vel": [0.0] * 6, "deviation": [0.0, self.trk, 0.0], "seamParam": self.s, "trackError": self.trk,
                "lateralOffset": sp["lateralOffset"], "wristRoll": sp["wristRoll"], "phase": "done" if self.s >= 1 else "weld",
                "weldInput": self.weld_input(), "arc": self.arc, "estopped": self.estopped, "t": self.t}

    def metrics(self) -> Dict[str, Any]:
        o = self.obs or {"status": "no_pool"}
        ok = self.arc and not self.estopped and o.get("status") == "ok"
        g = lambda: 1 + self.sensor_noise * self.rng.gauss(0, 1)
        pool = {"status": o.get("status", "no_pool") if (self.arc and not self.estopped) else "no_pool"}
        if ok:
            pool.update(L_mm=o["L"] * 1000 * g(), W_mm=o["W"] * 1000 * g(), area_mm2=o["area"] * 1e6 * g(),
                        cx_mm=o["cx"] * 1000, cy_mm=o["cy"] * 1000, peakC=o["peakC"], t85=o["t85"])   # penetration not observable by vision
        return {"arc_length_mm": self.sp["standoff"] * 1000, "deviation_mm": self.trk * 1000,
                "velocity_cm_min": self.sp["travelSpeed"] * 6000, "eval_score": None, "pool": pool,
                "process": {"I": self.process["I"], "V": self.process["V"]}, "t": self.t}

    def calibration(self) -> Dict[str, Any]:
        return {"frame": "seam", "units": {"length": "mm", "angle": "rad"}, "origin": [0, 0, 0], "scale": 1.0,
                "note": "mock backend (python): seam frame == sim frame"}


class Ros2Backend(MockBackend):
    """Skeleton: connect the protocol to a real welding cell through ROS 2.

    Suggested topics (adapt to your stack):
      /weldcell/cmd/setpoints   (std_msgs/Float64MultiArray)  <- control (travelSpeed, workAngle, travelAngle, standoff, lateralOffset)
      /weldcell/cmd/torch_pose  (geometry_msgs/PoseStamped)   <- set_torch_pose (seam frame)
      /weldcell/cmd/arc         (std_msgs/Bool)               <- set_trigger
      /weldcell/state/tcp       (geometry_msgs/PoseStamped)   -> state.pose
      /weldcell/sense/pool      (std_msgs/Float64MultiArray)  -> metrics.pool  [L_mm, W_mm, area_mm2, cx_mm, cy_mm]
      /weldcell/sense/process   (std_msgs/Float64MultiArray)  -> metrics.process [I, V]
    Until those callbacks are filled in, it behaves like the mock (so the UI keeps working).
    """

    name = "Ros2Backend"

    def __init__(self, **kw):
        super().__init__(**kw)
        try:
            import rclpy  # noqa: F401  (optional dependency)
            self.ros_ok = True
            # TODO: rclpy.init(); create node, publishers/subscribers listed above; spin in a thread.
        except Exception:  # pragma: no cover - rclpy not installed
            self.ros_ok = False
            print("[Ros2Backend] rclpy not available — running as mock. Install ROS 2 and fill in the TODOs.")

    def control(self, d):
        super().control(d)
        # TODO: publish d to /weldcell/cmd/setpoints (respect the robot's own safety layer!)


# ------------------------------------------------------------------- server
async def serve(backend: MockBackend, host: str, port: int, rate: float):
    import websockets  # pip install websockets

    clients = set()

    async def handler(ws, path=None):
        clients.add(ws)
        try:
            async for raw in ws:
                try:
                    m = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                t, d, seq = m.get("type"), m.get("data") or {}, m.get("seq", 0)
                if t == "hello":
                    await ws.send(msg("hello_ack", {"backend": backend.name, "protocol": PROTOCOL, "rateHz": rate,
                                                    "capabilities": ["control", "metrics", "state", "set_process", "set_trigger",
                                                                     "reset_episode", "calibration", "estop"]}, ack=seq))
                elif t == "ping":
                    await ws.send(msg("pong", {}, ack=seq))
                elif t == "control":
                    backend.control(d)
                    await ws.send(msg("ack", {"seq": seq}, ack=seq))
                elif t == "set_process":
                    backend.process.update({k: v for k, v in d.items() if k in backend.process})
                    await ws.send(msg("ack", {}, ack=seq))
                elif t == "set_trigger":
                    backend.arc = bool(d.get("on"))
                    await ws.send(msg("ack", {}, ack=seq))
                elif t == "set_torch_pose":
                    pos, rot = d.get("position", [0, 0, 2]), d.get("rotation", [0, 0, 0])
                    backend.control({"lateralOffset": pos[1] / 1000, "standoff": pos[2] / 1000,
                                     "travelAngle": rot[1], "workAngle": backend.sp["workRef"] + rot[2]})
                    await ws.send(msg("ack", {}, ack=seq))
                elif t == "reset_episode":
                    backend.reset()
                    await ws.send(msg("ack", {}, ack=seq))
                elif t == "get_calibration_info":
                    await ws.send(msg("calibration", backend.calibration(), ack=seq))
                elif t == "get_metrics":
                    await ws.send(msg("metrics", backend.metrics(), ack=seq))
                elif t == "estop":
                    backend.estopped, backend.arc, backend.cmd = True, False, {}
                    await ws.send(msg("ack", {"estop": True}, ack=seq))
                else:
                    await ws.send(msg("error", {"error": f"unknown type {t}"}, ack=seq))
        finally:
            clients.discard(ws)

    async def ticker():
        dt = 1.0 / rate
        while True:
            backend.tick(dt)
            if clients:
                st, me = msg("state", backend.state()), msg("metrics", backend.metrics())
                for c in list(clients):
                    try:
                        await c.send(st)
                        await c.send(me)
                    except Exception:
                        clients.discard(c)
            await asyncio.sleep(dt)

    async with websockets.serve(handler, host, port):
        print(f"WeldCell bridge [{backend.name}] on ws://{host}:{port}  ({rate:g} Hz, protocol {PROTOCOL})")
        await ticker()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=["mock", "ros2"], default="mock")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--rate", type=float, default=20.0, help="state/metrics streaming rate [Hz]")
    ap.add_argument("--seed", type=int, default=4242)
    a = ap.parse_args()
    backend = Ros2Backend(seed=a.seed) if a.backend == "ros2" else MockBackend(seed=a.seed)
    asyncio.run(serve(backend, a.host, a.port, a.rate))


if __name__ == "__main__":
    main()
