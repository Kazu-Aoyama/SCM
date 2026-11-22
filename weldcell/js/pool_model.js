/* =============================================================================
 * pool_model.js — WeldPoolModel v2.0  (Stage 1.8 analytical weld-pool physics)
 * -----------------------------------------------------------------------------
 * Superposition of a Gaussian-weighted 3×3 cluster of Rosenthal 3-D moving point
 * sources on a semi-infinite solid. Pure: no DOM / no Three.js.
 *
 *   params : { I, V, eta, T0, material, v_mmps, travelDeg, workDeg, arcLenMM,
 *              position, grav,
 *              // --- v2 additions (all optional) ---
 *              thicknessMM,          // plate thickness for burn-through check
 *              calib:{ kScale, sigScale, effScale } }   // Real2Sim calibration knobs
 *   solve(p)    -> { lenMM, widMM, cxMM, cyMM, penMM, peakC, areaMM } | null (no pool)
 *   evaluate(p) -> always an object: solve() fields + { status, t85, HImm, Q }
 *   makeField(p)-> { T(x,y,z), Tm, hazT, T0, cx, cy, xPeak, peakC }
 *
 * Local frame: x = travel (+ ahead of the arc), y = lateral, z = depth (mm).
 * The calibration knobs let Real2Sim fit the model to measured pools without
 * touching the physics code (see sim2real.js: Calibrator).
 * ========================================================================== */
(function (root, factory) {
  const api = factory();
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  if (typeof window !== 'undefined') window.WeldPoolModel = api;
})(this, function () {
  'use strict';
  const VERSION = '2.0';
  const MATS = {
    steel: { name: '軟鋼',       k: 50,  rho: 7870, c: 490, Tm: 1500, hazT: 727 },
    sus:   { name: 'ステンレス', k: 16,  rho: 8000, c: 500, Tm: 1450, hazT: 850 },
    al:    { name: 'アルミ',     k: 170, rho: 2700, c: 900, Tm: 660,  hazT: 250 },
  };
  const M = 1e-3, S_ELONG = 3.0, Z_DIG = 3.0;
  const SIG0 = (L) => 0.6 + 0.35 * L;               // base footprint radius (mm) vs arc length
  // empirical gravity bias per welding position (centroid shift dx travel / dy lateral / dz depth, mm)
  const GRAV = {
    flat:   { dx: 0.0,  dy: 0.0,  dz: 0.0 },
    horiz:  { dx: 0.0,  dy: -3.2, dz: 0.0 },
    vertup: { dx: -3.2, dy: 0.0,  dz: 0.6 },
    vertdn: { dx: 3.2,  dy: 0.0,  dz: -1.0 },
    over:   { dx: 0.0,  dy: 0.0,  dz: -2.4 },
  };
  const DEFAULTS = { I: 175, V: 24, eta: 0.80, T0: 20, material: 'steel', v_mmps: 7, travelDeg: -15,
    workDeg: 0, arcLenMM: 2, position: 'flat', grav: 0.6, thicknessMM: 9 };

  function _mat(p) {
    const base = MATS[p.material] || MATS.steel, c = p.calib || {};
    const kS = c.kScale == null ? 1 : c.kScale;
    return { k: base.k * kS, rho: base.rho, c: base.c, Tm: base.Tm, hazT: base.hazT, name: base.name };
  }
  function heatInput(p) { const c = p.calib || {}; return (p.eta * (c.effScale == null ? 1 : c.effScale)) * p.V * p.I; }

  function buildSource(p, mat) {
    const a = p.travelDeg * Math.PI / 180, b = p.workDeg * Math.PI / 180, La = Math.max(p.arcLenMM, 0.2);
    const lx = Math.tan(a), ly = Math.tan(b), lean = Math.hypot(lx, ly);
    let Cx = La * Math.sin(a), Cy = La * Math.sin(b);       // lever-arm: wire-tip projection -> heat-input point
    let zoff = -Z_DIG * Math.sin(a);                         // forehand shallow / backhand deep
    const G = GRAV[p.position] || GRAV.flat, g = (p.grav == null ? 0.6 : p.grav);
    Cx += G.dx * g; Cy += G.dy * g; zoff += G.dz * g;
    if (zoff > 3.0) zoff = 3.0; if (zoff < -2.5) zoff = -2.5;
    const sigS = (p.calib && p.calib.sigScale != null) ? p.calib.sigScale : 1;
    const sig0 = SIG0(La) * sigS, sigA = sig0 + S_ELONG * lean, sigP = sig0;
    const ux = lean > 1e-6 ? lx / lean : 1, uy = lean > 1e-6 ? ly / lean : 0, pxn = -uy, pyn = ux;
    const Q = heatInput(p), baseCoef = Q / (2 * Math.PI * mat.k);
    const subs = []; let wsum = 0; const tmp = [];
    for (const i of [-1, 0, 1]) for (const j of [-1, 0, 1]) { const w = Math.exp(-(i * i + j * j) / 1.4); tmp.push({ i, j, w }); wsum += w; }
    for (const t of tmp) { const da = t.i * sigA, dp = t.j * sigP;
      subs.push({ x: Cx + ux * da + pxn * dp, y: Cy + uy * da + pyn * dp, coef: baseCoef * t.w / wsum }); }
    return { subs, zoff, Cx, Cy, Q };
  }
  function tempAt(xmm, ymm, zmm, S, ec, T0) {
    let T = T0; const subs = S.subs, zo = S.zoff;
    for (let j = 0; j < subs.length; j++) { const s = subs[j];
      const dx = (xmm - s.x) * M, dy = (ymm - s.y) * M, dz = (zmm - zo) * M;
      const r = Math.sqrt(dx * dx + dy * dy + dz * dz) + 1e-6;
      T += s.coef / r * Math.exp(-ec * (r + dx));            // Rosenthal 3-D moving point source
    }
    return T > 4000 ? 4000 : T;
  }
  function _prep(p) {
    p = Object.assign({}, DEFAULTS, p);
    const mat = _mat(p);
    const alpha = mat.k / (mat.rho * mat.c), ec = (Math.max(p.v_mmps, 0.05) * M) / (2 * alpha), T0 = (p.T0 == null ? 20 : p.T0);
    return { p, mat, alpha, ec, T0, S: buildSource(p, mat) };
  }
  function solve(pIn) {
    const { mat, ec, T0, S } = _prep(pIn), Tm = mat.Tm, yC = S.Cy;
    // centerline (y=Cy, z=0): molten extent + peak
    const x0 = -45, x1 = 22, N = 120; let xb = null, xf = null, xPeak = 0, Tpeak = -1, pa = false, px = x0, pt = 0;
    for (let i = 0; i <= N; i++) { const x = x0 + (x1 - x0) * i / N, T = tempAt(x, yC, 0, S, ec, T0);
      if (T > Tpeak) { Tpeak = T; xPeak = x; }
      const ab = T >= Tm;
      // first entry = rear end, LAST exit = front end (v21 kept the last entry, which collapsed
      // the length when the centreline profile had several humps, e.g. long arcs)
      if (i > 0) { if (ab && !pa && xb === null) xb = px + (x - px) * (Tm - pt) / (T - pt); if (!ab && pa) xf = px + (x - px) * (Tm - pt) / (T - pt); }
      pa = ab; px = x; pt = T;
    }
    if (xb === null || xf === null || Tpeak < Tm) return null;
    const lenMM = xf - xb, cxMM = (xb + xf) / 2;
    const half = (dir) => { let lo = yC, hi = yC + dir * 45; if (tempAt(xPeak, hi, 0, S, ec, T0) > Tm) hi = yC + dir * 110;
      for (let it = 0; it < 26; it++) { const m = (lo + hi) / 2; if (tempAt(xPeak, m, 0, S, ec, T0) >= Tm) lo = m; else hi = m; } return (lo + hi) / 2; };
    const yhi = half(1), ylo = half(-1), widMM = yhi - ylo, cyMM = (yhi + ylo) / 2;
    let pen = 0; for (let z = 0.1; z <= 14; z += 0.1) { if (tempAt(xPeak, yC, z, S, ec, T0) >= Tm) pen = z; else break; }
    return { lenMM, widMM, cxMM, cyMM, penMM: pen, peakC: Tpeak, areaMM: Math.PI * (lenMM / 2) * (widMM / 2) };
  }
  /** Rosenthal thick-plate cooling time 800→500 °C [s] (line energy per unit length). */
  function t85(pIn) {
    const { p, mat, T0 } = _prep(pIn);
    const E = heatInput(p) / Math.max(p.v_mmps * M, 1e-5);         // J/m
    if (T0 >= 500) return NaN;
    return E / (2 * Math.PI * mat.k) * (1 / (500 - T0) - 1 / (800 - T0));
  }
  /** Full observation incl. status: 'ok' | 'no_pool' | 'burn_through'. */
  function evaluate(pIn) {
    const p = Object.assign({}, DEFAULTS, pIn);
    const r = solve(p), Q = heatInput(p), HImm = Q / Math.max(p.v_mmps, 0.05);   // J/mm
    if (!r) return { lenMM: 0, widMM: 0, cxMM: 0, cyMM: 0, penMM: 0, peakC: NaN, areaMM: 0, status: 'no_pool', t85: NaN, HImm, Q };
    const th = p.thicknessMM;
    const status = (th && r.penMM >= 0.95 * th) ? 'burn_through' : 'ok';
    return Object.assign(r, { status, t85: t85(p), HImm, Q });
  }
  function makeField(pIn) {
    const { mat, ec, T0, S } = _prep(pIn);
    let xPeak = S.Cx, Tpk = -1;
    for (let i = 0; i <= 90; i++) { const x = -45 + 67 * i / 90, T = tempAt(x, S.Cy, 0, S, ec, T0); if (T > Tpk) { Tpk = T; xPeak = x; } }
    return { T: (x, y, z) => tempAt(x, y, z, S, ec, T0), Tm: mat.Tm, hazT: mat.hazT, T0, cx: S.Cx, cy: S.Cy, xPeak, peakC: Tpk };
  }
  return { VERSION, materials: MATS, DEFAULTS, solve, evaluate, makeField, t85, heatInput };
});
