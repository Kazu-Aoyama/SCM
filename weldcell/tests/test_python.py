#!/usr/bin/env python3
"""Cross-language parity tests (Python bridge <-> JS core). Requires node.
    python3 tests/test_python.py
"""
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "bridge"))
import weldpool as W  # noqa: E402
from policy_runtime import Policy  # noqa: E402
from weldcell_env import WeldCellEnv  # noqa: E402


def node(js):
    return json.loads(subprocess.check_output(["node", "-e", js], cwd=ROOT))


def test_pool_model_parity():
    ref = node("""const M=require('./js/pool_model.js');const out=[];
      for(const v of [2.5,6,11,18])for(const w of [-12,0,9])for(const a of [0.8,2.2,5.5])for(const pos of ['flat','horiz','vertdn'])
      {const c={v_mmps:v,workDeg:w,arcLenMM:a,position:pos,travelDeg:-10,I:190,calib:{effScale:0.93,kScale:1.08,sigScale:0.97}};out.push([c,M.evaluate(c)]);}
      console.log(JSON.stringify(out));""")
    worst = 0.0
    for c, r in ref:
        p = W.evaluate(c)
        assert p["status"] == r["status"], (c, p["status"], r["status"])
        for k in ("lenMM", "widMM", "penMM", "areaMM"):
            worst = max(worst, abs(p[k] - r[k]))
    assert worst < 1e-9, worst
    return f"{len(ref)} cases, max |diff| {worst:.2e}"


def test_policy_runtime_parity():
    out = node("""const C=require('./js/core.js'),L=require('./js/learning.js');const r=C.makeRng(3);const res=[];
      for(const [o,a] of [['compact3','delta3'],['pool11','delta3'],['req19','req7']]){
        const codec=new C.Codec(o,a,{targetArea:5e-5});const pol=new L.LinearPolicy(codec.obsDim,codec.actDim);
        for(let i=0;i<pol.theta.length;i++)pol.theta[i]=r.gauss();
        const rob=new C.SurrogateRobotCell({rng:r});rob.step(0.05);rob.step(0.05);const rs=rob.getState();
        const ob={status:'ok',area:6e-5,L:0.011,W:0.007,centroidAlong:-0.002,centroidAcross:0.0003};
        const enc=Array.from(codec.encodeObs(rs,ob));const act=codec.decodeAction(pol.act(enc));
        res.push({json:pol.toJSON(codec,{targetArea:5e-5}),rs,ob,enc,act});}
      console.log(JSON.stringify(res));""")
    for case in out:
        pol = Policy(case["json"])
        enc = pol.encode(case["rs"], case["ob"])
        assert max(abs(a - b) for a, b in zip(enc, case["enc"])) < 1e-6, (enc, case["enc"])
        act = pol(case["rs"], case["ob"])
        for k, v in case["act"].items():
            assert abs(act[k] - v) < 1e-6 * max(1, abs(v)), (k, act[k], v)
    return f"{len(out)} codec presets match"


def test_env_runs():
    env = WeldCellEnv(seed=1)
    env.reset()
    tot, n, done = 0.0, 0, False
    while not done:
        _, r, term, trunc, _ = env.step([0.0, 0.0, 0.0])
        tot += r
        n += 1
        done = term or trunc
    assert n > 10 and -1 <= tot / n <= 1
    return f"{n} steps, mean reward {tot / n:.3f}"


if __name__ == "__main__":
    fail = 0
    for name, fn in [(k, v) for k, v in dict(globals()).items() if k.startswith("test_")]:
        try:
            print(f"  ✓ {name}: {fn()}")
        except Exception as e:  # noqa: BLE001
            fail += 1
            print(f"  ✗ {name}: {e!r}")
    sys.exit(1 if fail else 0)
