#!/usr/bin/env python3
"""policy_runtime.py — run a WeldCell policy JSON (exported from stage ④) outside the browser.

Reproduces core.js Codec + learning.js LinearPolicy exactly, so the SAME artifact
that was trained/evaluated in the simulator can drive a real robot controller
(e.g. inside a ROS 2 node at 20 Hz), behind your own safety layer.

    from policy_runtime import Policy
    pol = Policy.load("weldcell_policy_compact3_delta3.json")
    action = pol(robot_state, pool_obs)    # -> ControlAction dict (SI units; d* = per-tick deltas)

Only the 'compact3' / 'pool11' / 'req19' observation presets are implemented here
(the ones the UI exports); add features in OBS_FEATURES for custom specs.
"""
from __future__ import annotations

import json
import math
from typing import Any, Callable, Dict, List


def _g(arr, i):
    return arr[i] if arr is not None and len(arr) > i and arr[i] is not None else 0.0


def _rel(o, ctx):
    if o.get("status") == "ok" and ctx.get("target"):
        return (o["area"] - ctx["target"]) / ctx["target"]
    return 0.0 if o.get("status") == "ok" else -1.0


wi = lambda rs: rs.get("weldInput") or {}
OBS_FEATURES: Dict[str, Callable[[Dict, Dict, Dict], float]] = {
    # compact3
    "areaErr": lambda rs, o, c: max(-3.0, min(3.0, _rel(o, c))),
    "trackErr": lambda rs, o, c: (rs.get("trackError") or 0.0) / 0.003,
    "progress": lambda rs, o, c: rs.get("seamParam") or 0.0,
    # pool11
    "L": lambda rs, o, c: o.get("L", 0.0), "W": lambda rs, o, c: o.get("W", 0.0), "area": lambda rs, o, c: o.get("area", 0.0),
    "centroidAlong": lambda rs, o, c: o.get("centroidAlong", 0.0), "centroidAcross": lambda rs, o, c: o.get("centroidAcross", 0.0),
    "travelSpeed": lambda rs, o, c: wi(rs).get("travelSpeed", 0.0), "workDev": lambda rs, o, c: wi(rs).get("workDev", 0.0),
    "travelAngle": lambda rs, o, c: wi(rs).get("travelAngle", 0.0), "standoff": lambda rs, o, c: wi(rs).get("standoff", 0.0),
    "seamParam": lambda rs, o, c: rs.get("seamParam", 0.0), "trackError": lambda rs, o, c: rs.get("trackError", 0.0),
    # req19
    **{f"pose.{k}": (lambda i: lambda rs, o, c: _g(rs.get("pose"), i))(i) for i, k in enumerate(["x", "y", "z", "roll", "pitch", "yaw"])},
    **{f"dev.{k}": (lambda i: lambda rs, o, c: _g(rs.get("deviation"), i))(i) for i, k in enumerate(["along", "lateral", "height"])},
    **{f"vel.{k}": (lambda i: lambda rs, o, c: _g(rs.get("vel"), i))(i) for i, k in enumerate(["x", "y", "z", "wx", "wy", "wz"])},
    "arcLength": lambda rs, o, c: wi(rs).get("standoff", 0.0), "speed": lambda rs, o, c: wi(rs).get("travelSpeed", 0.0),
    "angle": lambda rs, o, c: wi(rs).get("workDev", 0.0),
}


class Policy:
    def __init__(self, j: Dict[str, Any]):
        if j.get("kind") != "LinearPolicy":
            raise ValueError("unsupported policy kind %r" % j.get("kind"))
        self.j, self.c = j, j["codec"]
        self.obs_dim, self.act_dim, self.theta = j["obsDim"], j["actDim"], j["theta"]
        self.ctx = {"target": j.get("targetArea")}
        missing = [n for n in self.c["obsNames"] if n not in OBS_FEATURES]
        if missing:
            raise ValueError("observation features not implemented: %s" % missing)

    @classmethod
    def load(cls, path: str) -> "Policy":
        with open(path, "r", encoding="utf-8") as f:
            return cls(json.load(f))

    def encode(self, rs: Dict, obs: Dict) -> List[float]:
        out = []
        for name, lo, hi, raw in zip(self.c["obsNames"], self.c["obsLo"], self.c["obsHi"], self.c["obsRaw"]):
            v = OBS_FEATURES[name](rs, obs, self.ctx)
            v = 0.0 if v is None or not math.isfinite(v) else float(v)
            out.append(max(-5.0, min(5.0, v)) if raw else max(-1.0, min(1.0, (v - lo) / (hi - lo) * 2 - 1)))
        return out

    def act_vec(self, o: List[float]) -> List[float]:
        n = self.obs_dim + 1
        return [math.tanh(self.theta[i * n + self.obs_dim] + sum(self.theta[i * n + k] * o[k] for k in range(self.obs_dim)))
                for i in range(self.act_dim)]

    def decode(self, a: List[float]) -> Dict[str, float]:
        return {name: (max(-1.0, min(1.0, x)) + 1) / 2 * (hi - lo) + lo
                for name, lo, hi, x in zip(self.c["actNames"], self.c["actLo"], self.c["actHi"], a)}

    def __call__(self, robot_state: Dict, pool_obs: Dict) -> Dict[str, float]:
        return self.decode(self.act_vec(self.encode(robot_state, pool_obs)))


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(0)
    pol = Policy.load(sys.argv[1])
    rs = {"trackError": 0.001, "seamParam": 0.3, "weldInput": {"travelSpeed": 0.007, "workDev": 0.0, "standoff": 0.002, "travelAngle": -0.26}}
    ob = {"status": "ok", "area": (pol.ctx["target"] or 5e-5) * 1.2, "L": 0.01, "W": 0.007}
    print("obs ->", pol.encode(rs, ob))
    print("action ->", pol(rs, ob))
