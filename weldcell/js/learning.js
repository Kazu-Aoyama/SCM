/* =============================================================================
 * learning.js — WeldCell learning toolkit v2.0
 * -----------------------------------------------------------------------------
 *  - SurrogateModel : lookup-table emulator of WeldPoolModel (trilinear over
 *                     speed × work-angle deviation × arc length). Built FROM the
 *                     physics, so training and the live sim share one physics
 *                     (fixes the v21 surrogate/physics mismatch). Exportable.
 *  - LinearPolicy   : a = tanh(W·[o;1])  — tiny, interpretable, portable
 *                     (bridge/policy_runtime.py runs the same JSON on a robot PC).
 *  - CEMTrainer     : cross-entropy method, chunked (UI stays responsive).
 *  - fitBC          : behaviour cloning (ridge regression) from demonstrations
 *                     (e.g. AUTO controller or skilled-welder logs) -> warm start.
 *  - makeTrainingCell / evaluatePolicy : headless episodes on the contract.
 * Requires core.js (WeldCell) and, for SurrogateModel, a model with evaluate().
 * ========================================================================== */
(function (root, factory) {
  const core = (typeof module !== 'undefined' && module.exports) ? require('./core.js') : root.WeldCell;
  const api = factory(core);
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  if (typeof window !== 'undefined') window.WeldCell = Object.assign(window.WeldCell || {}, api);
})(typeof self !== 'undefined' ? self : this, function (C) {
  'use strict';
  const clamp = C.clamp;

  /* ---------------------------------------------------------------- Surrogate */
  const GRID = {
    v:   [1.5, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24],   // mm/s
    w:   [-23, -15, -10, -5, 0, 5, 10, 15, 23],                // deg (work-angle deviation)
    a:   [0.3, 1, 2, 3, 4, 5, 6.5, 8, 10, 12],                 // mm  (arc length / standoff)
  };
  const FIELDS = ['lenMM', 'widMM', 'areaMM', 'penMM', 'cxMM', 'cyMM', 'peakC', 't85', 'ok'];
  class SurrogateModel {
    /** @param model WeldPoolModel-like {evaluate}; base = params held fixed (process, position, travelDeg, calib) */
    constructor(model, base, grid = GRID) {
      this.base = Object.assign({}, base); this.grid = grid; this.meta = { name: 'SurrogateTable', kind: 'surrogate' };
      if (model) this._build(model);
    }
    _build(model) {
      const G = this.grid, n = G.v.length * G.w.length * G.a.length;
      this.data = {}; for (const f of FIELDS) this.data[f] = new Float32Array(n);
      let k = 0;
      for (let i = 0; i < G.v.length; i++) for (let j = 0; j < G.w.length; j++) for (let l = 0; l < G.a.length; l++, k++) {
        const r = model.evaluate(Object.assign({}, this.base, { v_mmps: G.v[i], workDeg: G.w[j], arcLenMM: G.a[l], thicknessMM: 0 }));
        const ok = r.status !== 'no_pool';
        for (const f of FIELDS) this.data[f][k] = f === 'ok' ? (ok ? 1 : 0) : (ok && Number.isFinite(r[f]) ? r[f] : 0);
      }
      this.builtSolves = n;
    }
    _idx(arr, x) { // returns [i, frac]
      if (x <= arr[0]) return [0, 0];
      if (x >= arr[arr.length - 1]) return [arr.length - 2, 1];
      let i = 0; while (x > arr[i + 1]) i++;
      return [i, (x - arr[i]) / (arr[i + 1] - arr[i])];
    }
    _interp(f, iv, iw, ia) {
      const G = this.grid, D = this.data[f], nw = G.w.length, na = G.a.length;
      let s = 0;
      for (let a = 0; a < 2; a++) for (let b = 0; b < 2; b++) for (let c = 0; c < 2; c++) {
        const w = (a ? iv[1] : 1 - iv[1]) * (b ? iw[1] : 1 - iw[1]) * (c ? ia[1] : 1 - ia[1]);
        if (w) s += w * D[((iv[0] + a) * nw + (iw[0] + b)) * na + (ia[0] + c)];
      }
      return s;
    }
    evaluate(p) {
      const G = this.grid, iv = this._idx(G.v, p.v_mmps), iw = this._idx(G.w, p.workDeg || 0), ia = this._idx(G.a, p.arcLenMM);
      const ok = this._interp('ok', iv, iw, ia);
      const HImm = (this.base.eta * this.base.V * this.base.I) / Math.max(p.v_mmps, 0.05);
      if (ok < 0.5) return { lenMM: 0, widMM: 0, areaMM: 0, penMM: 0, cxMM: 0, cyMM: 0, peakC: NaN, t85: NaN, status: 'no_pool', HImm };
      const r = {}; for (const f of FIELDS) if (f !== 'ok') r[f] = this._interp(f, iv, iw, ia) / ok;   // renormalise over valid corners
      const th = p.thicknessMM ?? this.base.thicknessMM;
      r.status = (th && r.penMM >= 0.95 * th) ? 'burn_through' : 'ok'; r.HImm = HImm;
      return r;
    }
    toJSON() { const d = {}; for (const f of FIELDS) d[f] = Array.from(this.data[f], x => Math.round(x * 1e4) / 1e4);
      return { kind: 'SurrogateModel', base: this.base, grid: this.grid, data: d }; }
    static fromJSON(j) { const s = new SurrogateModel(null, j.base, j.grid); s.data = {}; for (const f of FIELDS) s.data[f] = Float32Array.from(j.data[f]); return s; }
  }

  /* ------------------------------------------------------------- LinearPolicy */
  class LinearPolicy {
    constructor(obsDim, actDim, theta = null) {
      this.obsDim = obsDim; this.actDim = actDim;
      this.theta = theta ? Float64Array.from(theta) : new Float64Array(actDim * (obsDim + 1));
    }
    get dim() { return this.theta.length; }
    act(o) {
      const a = new Float32Array(this.actDim), n = this.obsDim + 1;
      for (let i = 0; i < this.actDim; i++) { let s = this.theta[i * n + this.obsDim];
        for (let j = 0; j < this.obsDim; j++) s += this.theta[i * n + j] * o[j];
        a[i] = Math.tanh(s); }
      return a;
    }
    toJSON(codec, extra = {}) { return Object.assign({ kind: 'LinearPolicy', format: 'a = tanh(W [o;1]); W row-major actDim x (obsDim+1)',
      obsDim: this.obsDim, actDim: this.actDim, theta: Array.from(this.theta), codec: codec ? codec.toJSON() : null }, extra); }
    static fromJSON(j) { return new LinearPolicy(j.obsDim, j.actDim, j.theta); }
  }

  /* --------------------------------------------------------------- CEMTrainer */
  class CEMTrainer {
    constructor({ dim, evaluate, pop = 24, elite = 6, sigma0 = 0.6, sigmaMin = 0.04, mean0 = null, rng = C.rng, episodes = 2 }) {
      this.dim = dim; this.evaluate = evaluate; this.pop = pop; this.elite = elite; this.sigmaMin = sigmaMin;
      this.rng = rng; this.episodes = episodes;
      this.mean = mean0 ? Float64Array.from(mean0) : new Float64Array(dim);
      this.std = new Float64Array(dim).fill(sigma0);
      this.iter = 0; this.history = []; this.best = { theta: Float64Array.from(this.mean), score: -Infinity };
    }
    step() {
      const S = [];
      for (let p = 0; p < this.pop; p++) {
        const th = new Float64Array(this.dim);
        for (let i = 0; i < this.dim; i++) th[i] = this.mean[i] + this.std[i] * this.rng.gauss();
        let r = 0; for (let e = 0; e < this.episodes; e++) r += this.evaluate(th);
        S.push({ th, r: r / this.episodes });
      }
      S.sort((a, b) => b.r - a.r); const E = S.slice(0, this.elite);
      for (let i = 0; i < this.dim; i++) {
        let mu = 0; for (const z of E) mu += z.th[i]; mu /= E.length;
        let v = 0; for (const z of E) v += (z.th[i] - mu) ** 2; v /= E.length;
        this.mean[i] = mu; this.std[i] = Math.max(this.sigmaMin, Math.sqrt(v));
      }
      if (S[0].r > this.best.score) this.best = { theta: Float64Array.from(S[0].th), score: S[0].r };
      const eliteMean = E.reduce((a, z) => a + z.r, 0) / E.length;
      this.iter++; const rec = { iter: this.iter, best: S[0].r, eliteMean }; this.history.push(rec); return rec;
    }
  }

  /* ------------------------------------------------------------- BC (ridge) */
  function _solve(A, b) { const n = b.length, M = A.map((r, i) => r.concat([b[i]]));
    for (let c = 0; c < n; c++) { let piv = c; for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
      [M[c], M[piv]] = [M[piv], M[c]]; const d = M[c][c] || 1e-12; for (let j = c; j <= n; j++) M[c][j] /= d;
      for (let r = 0; r < n; r++) if (r !== c) { const f = M[r][c]; if (f) for (let j = c; j <= n; j++) M[r][j] -= f * M[c][j]; } }
    return M.map(r => r[n]); }
  /**
   * Behaviour cloning of a LinearPolicy from (obs, action_vec) pairs, action_vec in [-1,1].
   * Fits W by ridge regression on atanh(clip(a)). Returns {policy, mse}.
   */
  function fitBC(pairs, obsDim, actDim, lambda = 1e-2) {
    const n = obsDim + 1, pol = new LinearPolicy(obsDim, actDim);
    if (!pairs.length) return { policy: pol, mse: NaN, n: 0 };
    const XtX = Array.from({ length: n }, () => new Array(n).fill(0));
    const XtY = Array.from({ length: actDim }, () => new Array(n).fill(0));
    for (const { obs, act } of pairs) { const x = Array.from(obs).concat([1]);
      for (let i = 0; i < n; i++) { for (let j = 0; j < n; j++) XtX[i][j] += x[i] * x[j];
        for (let k = 0; k < actDim; k++) XtY[k][i] += x[i] * Math.atanh(clamp(act[k], -0.995, 0.995)); } }
    for (let i = 0; i < n; i++) XtX[i][i] += lambda * pairs.length;
    for (let k = 0; k < actDim; k++) { const w = _solve(XtX, XtY[k]); for (let i = 0; i < n; i++) pol.theta[k * n + i] = w[i]; }
    let se = 0; for (const { obs, act } of pairs) { const a = pol.act(obs); for (let k = 0; k < actDim; k++) se += (a[k] - act[k]) ** 2; }
    return { policy: pol, mse: se / (pairs.length * actDim), n: pairs.length };
  }
  /** Collect (obs, action_vec) demonstrations by running a controller through the codec. */
  function collectDemos(makeCo, controller, codec, { episodes = 4, steps = 400, dt = 0.05 } = {}) {
    const pairs = [];
    for (let e = 0; e < episodes; e++) {
      const co = makeCo(); co.setController(controller); co.reset(); codec.reset();
      let prevObs = null, prevF = null;
      for (let i = 0; i < steps; i++) { const f = co.step(dt);
        if (prevObs && prevF) pairs.push({ obs: prevObs, act: Array.from(codec.encodeAction(toDeltas(f.action || {}, prevF))) });
        prevObs = codec.encodeObs(f.robotState, f.poolObs); prevF = f;
        if (f.robotState.seamParam >= 1) break; }
    }
    return pairs;
  }
  /** Express an action (absolute setpoints and/or deltas) as deltas w.r.t. the previous frame's setpoints. */
  function toDeltas(a, prevF) {
    const wi = prevF.weldInput || {}, rs = prevF.robotState || {}, d = Object.assign({}, a);
    const pairs = [['travelSpeed', wi.travelSpeed], ['workAngle', wi.workAngle], ['travelAngle', wi.travelAngle],
      ['standoff', wi.standoff], ['lateralOffset', rs.lateralOffset], ['wristRoll', rs.wristRoll]];
    for (const [k, prev] of pairs) { const dk = 'd' + k[0].toUpperCase() + k.slice(1);
      if (a[k] != null && prev != null && a[dk] == null) d[dk] = a[k] - prev; }
    if (d.travelSpeed == null && wi.travelSpeed != null) d.travelSpeed = wi.travelSpeed + (d.dTravelSpeed || 0);
    return d;
  }

  /* -------------------------------------------------- training cell factory */
  /**
   * Builds a headless Coordinator for training/evaluation.
   *   o.process, o.position, o.travelAngle (rad), o.surrogate (SurrogateModel) | o.model,
   *   o.randomizer (sim2real DomainRandomizer, optional), o.rng
   */
  function makeTrainingCell(o) {
    const r = o.rng || C.rng;
    const robot = new C.SurrogateRobotCell({ process: o.process, position: o.position || 'F', travelAngle: o.travelAngle ?? -15 * C.DEG,
      workRef: o.workRef ?? 45 * C.DEG, standoff: o.standoff ?? 0.002, travelSpeed: o.travelSpeed ?? 0.007, bow: o.bow ?? 0.010, rng: r,
      trackGain: o.trackGain ?? 0.6 });
    let pool = new C.ModelPool(o.surrogate || o.model, { tau: o.tau ?? 0.15, name: o.surrogate ? 'SurrogateTable' : 'WeldPoolModel' });
    if (o.randomizer) pool = o.randomizer.wrapPool(pool, r);
    return new C.Coordinator({ robot, pool, reward: new C.PoolHoldReward(o.reward || {}) });
  }
  function evaluatePolicy(makeCo, policy, codec, { steps = 300, dt = 0.05, episodes = 1 } = {}) {
    let tot = 0;
    for (let e = 0; e < episodes; e++) { const co = makeCo(); co.setController(new C.PolicyController(policy, codec));
      tot += C.rollout(co, steps, dt).meanReward; }
    return tot / episodes;
  }

  return { SurrogateModel, SURROGATE_GRID: GRID, LinearPolicy, CEMTrainer, fitBC, collectDemos, toDeltas, makeTrainingCell, evaluatePolicy };
});
