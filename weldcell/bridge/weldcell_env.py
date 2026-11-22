"""weldcell_env.py — Python (Gymnasium-compatible) twin of the browser training cell.

Same physics (weldpool.py == js/pool_model.js), same 'compact3' observation,
'delta3' action, reward and dt as stage ④, so you can train PPO/SAC with
stable-baselines3 / CleanRL and run the result through the same contract.

    from weldcell_env import WeldCellEnv
    env = WeldCellEnv(seed=0)                 # gymnasium.Env if gymnasium is installed
    obs, info = env.reset()
    obs, r, terminated, truncated, info = env.step(env.action_space.sample())

Domain randomisation of the plant (effScale/kScale/sigScale) and initial setpoints
is on by default (dr=0.12), mirroring the browser defaults.
"""
from __future__ import annotations

import math
import random
from typing import Any, Dict, Optional

import weldpool as W

DEG = math.pi / 180
try:  # optional dependency
    import gymnasium as gym
    import numpy as np
    _Base = gym.Env
except Exception:  # pragma: no cover
    gym, np, _Base = None, None, object

ACT_LO = [-0.0008, -0.05, -0.0010]          # dTravelSpeed [m/s], dWorkAngle [rad], dLateralOffset [m] per tick
ACT_HI = [0.0008, 0.05, 0.0010]


class WeldCellEnv(_Base):
    metadata = {"render_modes": []}

    def __init__(self, process: Optional[Dict[str, Any]] = None, dt: float = 0.05, max_steps: int = 300, dr: float = 0.12,
                 obs_noise: float = 0.03, seed: Optional[int] = None, w_area=0.7, w_track=0.5, w_effort=0.05, nominal_speed=0.007,
                 nominal_standoff=0.002):
        self.process = dict(I=175.0, V=24.0, eta=0.80, material="steel", T0=20.0, thickness=0.009)
        if process:
            self.process.update(process)
        self.dt, self.max_steps, self.dr, self.obs_noise = dt, max_steps, dr, obs_noise
        self.w = (w_area, w_track, w_effort)
        self.v0, self.h0 = nominal_speed, nominal_standoff
        self.rng = random.Random(seed)
        nom = W.evaluate(W.params_from_weld_input(self._wi(nominal_speed, 0.0, nominal_standoff)))
        self.target = nom["areaMM"] / 1e6
        if gym is not None:
            self.observation_space = gym.spaces.Box(-5.0, 5.0, shape=(3,), dtype=np.float32)
            self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)

    def _wi(self, v, work_dev, standoff):
        p = dict(self.process)
        p["Q"] = p["eta"] * p["V"] * p["I"]
        return {"travelSpeed": v, "workDev": work_dev, "travelAngle": -15 * DEG, "standoff": standoff, "weldPosition": "F", "process": p}

    def reset(self, seed: Optional[int] = None, options=None):
        if seed is not None:
            self.rng.seed(seed)
        u = lambda: 1 + self.dr * (2 * self.rng.random() - 1)
        self.calib = {"effScale": u(), "kScale": u(), "sigScale": u()}
        self.v = self.v0 * (0.7 + 0.6 * self.rng.random())
        self.h = min(0.012, max(0.0005, self.h0 * (0.7 + 0.6 * self.rng.random())))
        self.wd, self.lat, self.s, self.noise, self.nv, self.n = 0.0, 0.0, 0.0, 0.0, 0.0, 0
        self.area = None
        self._advance(0.0)
        return self._obs(), {}

    def _advance(self, dt):
        self.nv += (-self.nv * 1.2 + (self.rng.random() - 0.5) * 0.04) * dt
        self.noise = max(-0.0016, min(0.0016, self.noise + self.nv * dt))
        seam = 0.010 * math.sin(math.pi * self.s) + self.noise
        self.lat += 0.6 * (seam - self.lat) * dt
        self.trk = seam - self.lat
        self.s = min(1.0, self.s + self.v * dt / 0.18)
        r = W.evaluate(W.params_from_weld_input(self._wi(self.v, self.wd, self.h), self.calib))
        self.status = r["status"]
        a = r["areaMM"] / 1e6
        tau = 0.15
        self.area = a if (self.area is None or self.status != "ok" or dt == 0) else self.area + (a - self.area) * min(1.0, dt / tau)

    def _obs(self):
        a = self.area * (1 + self.obs_noise * self.rng.gauss(0, 1)) if self.status == "ok" else 0.0
        o = [max(-3.0, min(3.0, (a - self.target) / self.target)) if self.status == "ok" else -1.0, self.trk / 0.003, self.s]
        return np.asarray(o, dtype=np.float32) if np is not None else o

    def step(self, action):
        a = [max(-1.0, min(1.0, float(x))) for x in action]
        d = [(x + 1) / 2 * (hi - lo) + lo for x, lo, hi in zip(a, ACT_LO, ACT_HI)]
        self.v = min(0.020, max(0.002, self.v + d[0]))
        self.wd = min(0.40, max(-0.40, self.wd + d[1]))
        self.lat = min(0.015, max(-0.015, self.lat + d[2]))
        self._advance(self.dt)
        self.n += 1
        if self.status == "ok":
            eA = abs(self.area - self.target) / self.target
            r = max(-1.0, min(1.0, 1 - self.w[0] * eA - self.w[1] * abs(self.trk) / 0.003 - self.w[2] * abs(d[1])))
        else:
            r = -1.0
        terminated = self.s >= 1.0 or self.status == "burn_through"
        truncated = self.n >= self.max_steps
        return self._obs(), r, terminated, truncated, {"area": self.area, "status": self.status, "calib": self.calib}


if __name__ == "__main__":
    env = WeldCellEnv(seed=0)
    obs, _ = env.reset()
    tot, done = 0.0, False
    while not done:
        obs, r, term, trunc, info = env.step([0.0, 0.0, 0.0])
        tot += r
        done = term or trunc
    print(f"zero-action episode: steps={env.n} mean reward={tot / env.n:.3f} target area={env.target * 1e6:.1f} mm2 calib={info['calib']}")
