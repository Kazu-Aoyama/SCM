"""weldpool.py — Python port of js/pool_model.js (WeldPoolModel v2.0).

Superposition of a Gaussian-weighted 3x3 cluster of Rosenthal 3-D moving point
sources on a semi-infinite solid. Numerically identical (to ~1e-9) to the
browser model so that policies / calibrations move freely between the web
simulator, Python RL code (PPO/SAC) and the real-robot bridge.

    evaluate(params) -> dict(lenMM, widMM, cxMM, cyMM, penMM, peakC, areaMM,
                             status, t85, HImm, Q)
Units: mm, mm/s, deg, A, V, degC.  No third-party dependencies.
"""
from __future__ import annotations

import math
from typing import Any, Dict, Optional

VERSION = "2.0"
MATS = {
    "steel": dict(k=50.0, rho=7870.0, c=490.0, Tm=1500.0, hazT=727.0),
    "sus": dict(k=16.0, rho=8000.0, c=500.0, Tm=1450.0, hazT=850.0),
    "al": dict(k=170.0, rho=2700.0, c=900.0, Tm=660.0, hazT=250.0),
}
GRAV = {
    "flat": (0.0, 0.0, 0.0), "horiz": (0.0, -3.2, 0.0), "vertup": (-3.2, 0.0, 0.6),
    "vertdn": (3.2, 0.0, -1.0), "over": (0.0, 0.0, -2.4),
}
DEFAULTS: Dict[str, Any] = dict(I=175.0, V=24.0, eta=0.80, T0=20.0, material="steel", v_mmps=7.0, travelDeg=-15.0,
                                workDeg=0.0, arcLenMM=2.0, position="flat", grav=0.6, thicknessMM=9.0, calib=None)
M = 1e-3
S_ELONG = 3.0
Z_DIG = 3.0


def _sig0(L: float) -> float:
    return 0.6 + 0.35 * L


def _params(p: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    q = dict(DEFAULTS)
    if p:
        q.update({k: v for k, v in p.items() if v is not None})
    return q


def heat_input(p: Dict[str, Any]) -> float:
    c = p.get("calib") or {}
    return p["eta"] * c.get("effScale", 1.0) * p["V"] * p["I"]


def _mat(p):
    base = MATS.get(p["material"], MATS["steel"])
    c = p.get("calib") or {}
    m = dict(base)
    m["k"] = base["k"] * c.get("kScale", 1.0)
    return m


def _build_source(p, mat):
    a = math.radians(p["travelDeg"])
    b = math.radians(p["workDeg"])
    La = max(p["arcLenMM"], 0.2)
    lx, ly = math.tan(a), math.tan(b)
    lean = math.hypot(lx, ly)
    Cx, Cy = La * math.sin(a), La * math.sin(b)
    zoff = -Z_DIG * math.sin(a)
    gdx, gdy, gdz = GRAV.get(p["position"], GRAV["flat"])
    g = 0.6 if p.get("grav") is None else p["grav"]
    Cx += gdx * g
    Cy += gdy * g
    zoff += gdz * g
    zoff = min(3.0, max(-2.5, zoff))
    sigS = (p.get("calib") or {}).get("sigScale", 1.0)
    sig0 = _sig0(La) * sigS
    sigA, sigP = sig0 + S_ELONG * lean, sig0
    if lean > 1e-6:
        ux, uy = lx / lean, ly / lean
    else:
        ux, uy = 1.0, 0.0
    pxn, pyn = -uy, ux
    Q = heat_input(p)
    base = Q / (2 * math.pi * mat["k"])
    tmp = [(i, j, math.exp(-(i * i + j * j) / 1.4)) for i in (-1, 0, 1) for j in (-1, 0, 1)]
    wsum = sum(t[2] for t in tmp)
    subs = [(Cx + ux * i * sigA + pxn * j * sigP, Cy + uy * i * sigA + pyn * j * sigP, base * w / wsum) for i, j, w in tmp]
    return subs, zoff, Cx, Cy


def _temp(x, y, z, subs, zoff, ec, T0):
    T = T0
    for sx, sy, coef in subs:
        dx, dy, dz = (x - sx) * M, (y - sy) * M, (z - zoff) * M
        r = math.sqrt(dx * dx + dy * dy + dz * dz) + 1e-6
        T += coef / r * math.exp(-ec * (r + dx))
    return 4000.0 if T > 4000.0 else T


def _prep(p):
    mat = _mat(p)
    alpha = mat["k"] / (mat["rho"] * mat["c"])
    ec = (max(p["v_mmps"], 0.05) * M) / (2 * alpha)
    T0 = 20.0 if p.get("T0") is None else p["T0"]
    subs, zoff, Cx, Cy = _build_source(p, mat)
    return mat, ec, T0, subs, zoff, Cx, Cy


def solve(p_in: Optional[Dict[str, Any]] = None):
    p = _params(p_in)
    mat, ec, T0, subs, zoff, Cx, Cy = _prep(p)
    Tm, yC = mat["Tm"], Cy
    x0, x1, N = -45.0, 22.0, 120
    xb = xf = None
    xPeak, Tpeak, pa, px, pt = 0.0, -1.0, False, x0, 0.0
    for i in range(N + 1):
        x = x0 + (x1 - x0) * i / N
        T = _temp(x, yC, 0.0, subs, zoff, ec, T0)
        if T > Tpeak:
            Tpeak, xPeak = T, x
        ab = T >= Tm
        if i > 0:
            if ab and not pa and xb is None:
                xb = px + (x - px) * (Tm - pt) / (T - pt)
            if (not ab) and pa:
                xf = px + (x - px) * (Tm - pt) / (T - pt)
        pa, px, pt = ab, x, T
    if xb is None or xf is None or Tpeak < Tm:
        return None
    lenMM, cxMM = xf - xb, (xb + xf) / 2

    def half(d):
        lo, hi = yC, yC + d * 45
        if _temp(xPeak, hi, 0.0, subs, zoff, ec, T0) > Tm:
            hi = yC + d * 110
        for _ in range(26):
            m = (lo + hi) / 2
            if _temp(xPeak, m, 0.0, subs, zoff, ec, T0) >= Tm:
                lo = m
            else:
                hi = m
        return (lo + hi) / 2

    yhi, ylo = half(1), half(-1)
    pen, z = 0.0, 0.1
    while z <= 14 + 1e-9:
        if _temp(xPeak, yC, z, subs, zoff, ec, T0) >= Tm:
            pen = z
        else:
            break
        z += 0.1
    wid = yhi - ylo
    return dict(lenMM=lenMM, widMM=wid, cxMM=cxMM, cyMM=(yhi + ylo) / 2, penMM=pen, peakC=Tpeak,
                areaMM=math.pi * (lenMM / 2) * (wid / 2))


def t85(p_in=None) -> float:
    p = _params(p_in)
    mat = _mat(p)
    T0 = p["T0"]
    if T0 >= 500:
        return float("nan")
    E = heat_input(p) / max(p["v_mmps"] * M, 1e-5)
    return E / (2 * math.pi * mat["k"]) * (1 / (500 - T0) - 1 / (800 - T0))


def evaluate(p_in: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    p = _params(p_in)
    r = solve(p)
    Q = heat_input(p)
    HImm = Q / max(p["v_mmps"], 0.05)
    if r is None:
        return dict(lenMM=0.0, widMM=0.0, cxMM=0.0, cyMM=0.0, penMM=0.0, peakC=float("nan"), areaMM=0.0,
                    status="no_pool", t85=float("nan"), HImm=HImm, Q=Q)
    th = p.get("thicknessMM")
    r["status"] = "burn_through" if (th and r["penMM"] >= 0.95 * th) else "ok"
    r.update(t85=t85(p), HImm=HImm, Q=Q)
    return r


POS = {"F": "flat", "H": "horiz", "VU": "vertup", "VD": "vertdn", "OH": "over"}


def params_from_weld_input(wi: Dict[str, Any], calib: Optional[Dict[str, float]] = None) -> Dict[str, Any]:
    """Contract WeldInput (SI) -> model params (same mapping as core.js ModelPool.paramsFrom)."""
    pr = wi.get("process") or {}
    return dict(I=pr.get("I", 175), V=pr.get("V", 24), eta=pr.get("eta", 0.8), T0=pr.get("T0", 20), material=pr.get("material", "steel"),
                grav=0.6, thicknessMM=pr.get("thickness", 0.009) * 1000, v_mmps=max(wi.get("travelSpeed", 0.007) * 1000, 0.3),
                travelDeg=math.degrees(wi.get("travelAngle", 0.0)), workDeg=math.degrees(wi.get("workDev", 0.0)),
                arcLenMM=max(wi.get("standoff", 0.002) * 1000, 0.3), position=POS.get(wi.get("weldPosition", "F"), "flat"), calib=calib)
