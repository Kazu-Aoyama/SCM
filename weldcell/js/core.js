/* =============================================================================
 * core.js — weldcell_core v2.0 : Weld-Cell Integration Core
 * -----------------------------------------------------------------------------
 * 溶接ロボット（アーム）と溶融池モデルを「契約（contract）」で疎結合に束ねる中核層。
 * The robot cell, the weld-pool model and the controller are bound through a
 * narrow, stable CONTRACT so that each side can be swapped at runtime:
 *
 *     robot  : surrogate | live 3-D sim | REAL robot via bridge   (IRobotCell)
 *     pool   : analytic  | surrogate table | world model | REAL sensor (IWeldPool)
 *     control: classic AUTO | learned policy | none (agent/env)    (IController)
 *
 * The SAME policy / reward / codec therefore runs in simulation, in the live
 * 3-D cell and on real hardware (Sim2Real), and real logs can be replayed or
 * used to calibrate the simulator (Real2Sim) — see sim2real.js.
 *
 * Framework-agnostic: NO DOM, NO Three.js. Attaches to window.WeldCell and
 * module.exports (node tests).   UNITS: SI — m, s, rad, °C, A, V, W, m/s.
 *
 * FRAMES: each tick the robot supplies a local weld frame
 *   u = travel direction, v = across-seam, n = surface normal  (+ world weld point).
 * The pool model works purely in that frame and never sees robot kinematics.
 * ========================================================================== */
(function (root, factory) {
  const api = factory();
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  if (typeof window !== 'undefined') window.WeldCell = Object.assign(window.WeldCell || {}, api);
})(this, function () {
  'use strict';
  const VERSION = '2.0';
  const SCHEMA = 'weldcell-traj/2';

  /* ===========================================================================
   * CONTRACT (typedefs) — the only things the components must agree on.
   * ===========================================================================
   * @typedef {Object} WeldInput            robot -> pool (heat-source command)
   *   t, weldPoint[3] (m), travelDir[3], acrossDir[3], surfaceNormal[3], gravityDir[3],
   *   travelSpeed (m/s), workAngle (rad, absolute), workDev (rad, deviation from joint
   *   reference), travelAngle (rad, +push/−drag), standoff (m), weldPosition 'F'|'H'|'VU'|'VD',
   *   process {I, V, eta, Q, material, T0, thickness}, seamParam [0,1], contact (bool)
   *
   * @typedef {Object} PoolObservation       pool -> controller/agent
   *   t, L, W (m), penetration (m), area (m²), centroidAlong, centroidAcross (m),
   *   peakT (°C), t85 (s), status 'ok'|'no_pool'|'burn_through'|'stale', [source]
   *
   * @typedef {Object} RobotState
   *   q[], tcp[3] (m), pose[6] = [x,y,z,roll,pitch,yaw] of the torch in the seam frame
   *   (m, rad), vel[6] (m/s, rad/s), deviation[3] = [along, lateral, height] error (m),
   *   ikError (m), phase, seamParam, trackError (m, lateral), wristRoll (rad),
   *   weldInput (WeldInput)
   *
   * @typedef {Object} ControlAction          controller/agent -> robot
   *   ABSOLUTE setpoints : travelSpeed, workAngle, travelAngle, standoff, lateralOffset, wristRoll
   *   DELTAS             : dTravelSpeed, dWorkAngle, dTravelAngle, dStandoff, dLateralOffset,
   *                        dWristRoll, dAlong
   *   (legacy: {absolute:true, travelSpeed,...} is still accepted)
   *
   * INTERFACES (duck-typed):
   *   IRobotCell : reset(); step(dt); getState():RobotState; getWeldInput():WeldInput;
   *                applyControl(ControlAction); meta:{name,kind}
   *   IWeldPool  : reset(init?); setInput(WeldInput); step(dt):PoolObservation;
   *                observe():PoolObservation; getState?(); setState?(s); meta
   *   IController: reset(); update(RobotState, PoolObservation, dt):ControlAction|null; meta
   * ========================================================================= */

  // ---- helpers ---------------------------------------------------------------
  const clamp = (x, a, b) => x < a ? a : (x > b ? b : x);
  const DEG = Math.PI / 180;

  /** Seeded PRNG (mulberry32) — reproducible experiments. */
  function makeRng(seed) {
    let a = (seed >>> 0) || 0x9e3779b9;
    const rng = function () {
      a |= 0; a = (a + 0x6D2B79F5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
    rng.seed = seed >>> 0;
    rng.gauss = function () { let u = 0, v = 0; while (u === 0) u = rng(); while (v === 0) v = rng();
      return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); };
    rng.uniform = (lo, hi) => lo + (hi - lo) * rng();
    return rng;
  }
  let _globalRng = makeRng(12345);
  function setSeed(seed) { _globalRng = makeRng(seed); return _globalRng; }
  function rng() { return _globalRng(); }
  rng.gauss = () => _globalRng.gauss();

  function _need(obj, methods, who) {
    if (!obj) throw new Error(`[WeldCell] ${who} is missing.`);
    for (const m of methods)
      if (typeof obj[m] !== 'function')
        throw new Error(`[WeldCell] ${who} is missing required method "${m}()". It must implement the ${who} contract.`);
  }
  function assertPool(p)  { _need(p, ['reset', 'setInput', 'step', 'observe'], 'IWeldPool'); return p; }
  function assertRobot(r) { _need(r, ['reset', 'step', 'getState', 'getWeldInput', 'applyControl'], 'IRobotCell'); return r; }
  function assertCtl(c)   { if (c) _need(c, ['update'], 'IController'); return c; }

  function emptyObs() {
    return { t: 0, L: 0, W: 0, penetration: NaN, area: 0, centroidAlong: 0, centroidAcross: 0,
             peakT: NaN, t85: NaN, status: 'no_pool' };
  }

  /* ===========================================================================
   * Setpoints & control semantics — ONE implementation shared by every robot
   * (surrogate, live 3-D cell, real robot bridge) so actions mean the same thing
   * everywhere. LIMITS are the physical/safety envelope.
   * ========================================================================= */
  const DEFAULT_LIMITS = {
    travelSpeed:   [0.002, 0.020],          // m/s
    workDev:       [-0.40, 0.40],           // rad, deviation from the joint's reference work angle
    travelAngle:   [-0.45, 0.45],           // rad
    standoff:      [0.0, 0.012],            // m
    lateralOffset: [-0.015, 0.015],         // m
    wristRoll:     [-1.57, 1.57],           // rad
  };
  /** sp = {travelSpeed, workAngle, workRef, travelAngle, standoff, lateralOffset, wristRoll, along} (mutated) */
  function applyAction(sp, a, limits) {
    if (!a) return sp;
    const L = limits || DEFAULT_LIMITS, ref = sp.workRef || 0;
    const setWork = (w) => { sp.workAngle = ref + clamp(w - ref, L.workDev[0], L.workDev[1]); };
    if (a.travelSpeed != null)   sp.travelSpeed = clamp(a.travelSpeed, L.travelSpeed[0], L.travelSpeed[1]);
    if (a.workAngle != null)     setWork(a.workAngle);
    if (a.travelAngle != null)   sp.travelAngle = clamp(a.travelAngle, L.travelAngle[0], L.travelAngle[1]);
    if (a.standoff != null)      sp.standoff = clamp(a.standoff, L.standoff[0], L.standoff[1]);
    if (a.lateralOffset != null) sp.lateralOffset = clamp(a.lateralOffset, L.lateralOffset[0], L.lateralOffset[1]);
    if (a.wristRoll != null)     sp.wristRoll = clamp(a.wristRoll, L.wristRoll[0], L.wristRoll[1]);
    if (a.dTravelSpeed)   sp.travelSpeed = clamp(sp.travelSpeed + a.dTravelSpeed, L.travelSpeed[0], L.travelSpeed[1]);
    if (a.dWorkAngle)     setWork(sp.workAngle + a.dWorkAngle);
    if (a.dTravelAngle)   sp.travelAngle = clamp(sp.travelAngle + a.dTravelAngle, L.travelAngle[0], L.travelAngle[1]);
    if (a.dStandoff)      sp.standoff = clamp(sp.standoff + a.dStandoff, L.standoff[0], L.standoff[1]);
    if (a.dLateralOffset) sp.lateralOffset = clamp(sp.lateralOffset + a.dLateralOffset, L.lateralOffset[0], L.lateralOffset[1]);
    if (a.dWristRoll)     sp.wristRoll = clamp((sp.wristRoll || 0) + a.dWristRoll, L.wristRoll[0], L.wristRoll[1]);
    if (a.dAlong)         sp.along = clamp((sp.along || 0) + a.dAlong, -0.01, 0.01);
    return sp;
  }

  /* ===========================================================================
   * Reward — "hold the pool & track the seam" (single definition, used by the
   * env, the recorder, rollouts and the UI so numbers are comparable).
   *   r = clamp(1 − wA·|A−A*|/A* − wT·|e_track|/s_T − wE·effort, −1, 1);  r=−1 if no pool
   * A* is either given or captured from the first valid pool (autoTarget).
   * ========================================================================= */
  class PoolHoldReward {
    constructor(o = {}) {
      this.wArea = o.wArea ?? 0.7; this.wTrack = o.wTrack ?? 0.5; this.wEffort = o.wEffort ?? 0.05;
      this.trackScale = o.trackScale ?? 0.003; this.burnPenalty = o.burnPenalty ?? -1;
      this.fixedTarget = o.targetArea ?? null; this.target = this.fixedTarget;
    }
    reset() { this.target = this.fixedTarget; }
    /** control effort in rad (angle increments per tick) — v1 used |Δφw| only */
    effort(a) {
      if (!a) return 0;
      return Math.abs(a.dWorkAngle || 0) + Math.abs(a.dTravelAngle || 0) + 0.5 * Math.abs(a.dWristRoll || 0);
    }
    compute(rs, obs, action) {
      if (!obs || obs.status === 'burn_through') return this.burnPenalty;
      if (obs.status !== 'ok' || !(obs.area > 0)) return -1;
      if (this.target == null) this.target = obs.area;
      const eA = Math.abs(obs.area - this.target) / this.target;
      const eT = Math.abs((rs && rs.trackError) || 0) / this.trackScale;
      return clamp(1 - this.wArea * eA - this.wTrack * eT - this.wEffort * this.effort(action), -1, 1);
    }
    toJSON() { return { kind: 'PoolHoldReward', wArea: this.wArea, wTrack: this.wTrack, wEffort: this.wEffort,
      trackScale: this.trackScale, targetArea: this.fixedTarget }; }
  }

  /* ===========================================================================
   * EventBus — telemetry / decoupled observers (UI, loggers, monitors).
   * ========================================================================= */
  class EventBus {
    constructor() { this._h = {}; }
    on(ev, fn)  { (this._h[ev] || (this._h[ev] = [])).push(fn); return () => this.off(ev, fn); }
    off(ev, fn) { const a = this._h[ev]; if (a) { const i = a.indexOf(fn); if (i >= 0) a.splice(i, 1); } }
    emit(ev, x) { const a = this._h[ev]; if (a) for (const fn of a.slice()) fn(x); }
  }

  /* ===========================================================================
   * Coordinator — THE CORE. Owns the clock and the coupling tick:
   *   [external action] -> robot.step -> WeldInput -> pool.setInput/step
   *   -> PoolObservation -> controller.update -> robot.applyControl
   *   -> reward -> emit / record frame.
   * ========================================================================= */
  class Coordinator {
    constructor({ robot, pool, controller = null, recorder = null, bus = null, reward = null } = {}) {
      this.robot = assertRobot(robot);
      this.pool = assertPool(pool);
      this.controller = assertCtl(controller);
      this.recorder = recorder;
      this.reward = reward || new PoolHoldReward();
      this.bus = bus || new EventBus();
      this.t = 0; this.lastFrame = null; this.stepCount = 0;
    }
    setPool(pool)    { this.pool = assertPool(pool); this.bus.emit('pool:swap', pool.meta || {}); }
    setRobot(robot)  { this.robot = assertRobot(robot); this.bus.emit('robot:swap', robot.meta || {}); }
    setController(c) { this.controller = assertCtl(c); if (c && c.reset) c.reset(); }
    setRecorder(r)   { this.recorder = r; }
    setReward(r)     { this.reward = r; }

    reset() {
      this.t = 0; this.stepCount = 0; this.lastFrame = null; this._pending = null;
      this.robot.reset(); this.pool.reset();
      if (this.controller && this.controller.reset) this.controller.reset();
      if (this.reward && this.reward.reset) this.reward.reset();
      this.bus.emit('reset', {});
    }
    /**
     * One coupling tick. The action applied at the START of this tick is either the
     * external one (agent/env) or the controller's decision from the previous tick,
     * so frame.action is always "a" of the transition (s_prev, a, r, s_this).
     */
    step(dt, extAction = null) {
      this.t += dt; this.stepCount++;
      const action = extAction || this._pending || null; this._pending = null;
      if (action) this.robot.applyControl(action);
      this.robot.step(dt);
      const wi = this.robot.getWeldInput(); wi.t = this.t;
      this.pool.setInput(wi);
      const obs = this.pool.step(dt); obs.t = this.t;
      const rs = this.robot.getState();
      if (!extAction && this.controller) this._pending = this.controller.update(rs, obs, dt) || null;
      const reward = this.reward ? this.reward.compute(rs, obs, action) : 0;
      const frame = { t: this.t, dt, step: this.stepCount, weldInput: wi, poolObs: obs, robotState: rs, action, reward };
      this.lastFrame = frame;
      this.bus.emit('tick', frame);
      if (this.recorder) this.recorder.push(frame);
      return frame;
    }
    run(dt, n) { let f = null; for (let i = 0; i < n; i++) f = this.step(dt); return f; }
  }

  /* ===========================================================================
   * Controllers
   * ========================================================================= */
  // Baseline: hold pool AREA via travel speed, track the seam via lateral offset.
  class AutoHoldController {
    constructor(opts = {}) {
      this.meta = { name: 'AutoHold', kind: 'classic' };
      this.kArea = opts.kArea ?? 8.0; this.kTrack = opts.kTrack ?? 3.0;
      this.vMin = opts.vMin ?? 0.002; this.vMax = opts.vMax ?? 0.020;
      this.fixedTarget = opts.targetArea ?? null;          // desired pool area [m²] (else captured from the first pool)
      this.target = null; this._v = null; this._lat = null;
    }
    reset() { this.target = this.fixedTarget ? { area: this.fixedTarget } : null; this._v = null; this._lat = null; }
    update(rs, obs, dt) {
      if (obs.status !== 'ok') return null;
      const wi = rs.weldInput || {};
      if (this._v == null) this._v = wi.travelSpeed || 0.007;        // sync with the robot's actual setpoint
      if (this._lat == null) this._lat = rs.lateralOffset || 0;
      if (!this.target) { this.target = { area: obs.area }; return null; }
      const eA = (obs.area - this.target.area) / Math.max(this.target.area, 1e-12);
      this._v = clamp(this._v * (1 + this.kArea * eA * dt), this.vMin, this.vMax);
      this._lat += this.kTrack * (rs.trackError || 0) * dt;           // stable negative feedback toward the seam
      return { travelSpeed: this._v, lateralOffset: this._lat };
    }
  }
  /** Wraps a learned policy (object with act(vec) or a function vec->vec) through a Codec. */
  class PolicyController {
    constructor(policy, codec) {
      this.meta = { name: 'Policy', kind: 'learned' };
      this.policy = policy; this.codec = codec; this.lastVec = null;
    }
    reset() { if (this.codec && this.codec.reset) this.codec.reset(); }
    update(rs, obs) {
      const o = this.codec.encodeObs(rs, obs);
      const a = typeof this.policy === 'function' ? this.policy(o) : this.policy.act(o);
      this.lastVec = a;
      return this.codec.decodeAction(a);
    }
  }

  /* ===========================================================================
   * CODEC — observation / action spaces as declarative feature lists.
   * Each feature: {name, lo, hi, get(rs, obs, ctx)}; ctx = {target area, ...}.
   * Presets:
   *   obs 'compact3' : [areaErr, trackErr/3mm, progress]           (fast linear policies)
   *   obs 'pool11'   : v1 default (L, W, area, centroids, setpoints, progress, track)
   *   obs 'req19'    : 要件定義書 MDM — torch pose 6 + deviation 3 + velocity 6 + weld state 4
   *   act 'delta3'   : [dTravelSpeed, dWorkAngle, dLateralOffset]
   *   act 'req7'     : 要件定義書 — [Δx(along), Δy(lateral), Δz(standoff), Δroll, Δpitch(travel), Δyaw(work), v]
   * ========================================================================= */
  const rel = (o, c) => (o.status === 'ok' && c.target) ? (o.area - c.target) / c.target : (o.status === 'ok' ? 0 : -1);
  const g = (arr, i) => (arr && arr[i] != null) ? arr[i] : 0;
  const OBS_SPECS = {
    compact3: [
      { name: 'areaErr',  lo: -1, hi: 1, raw: true, get: (rs, o, c) => clamp(rel(o, c), -3, 3) },
      { name: 'trackErr', lo: -1, hi: 1, raw: true, get: (rs) => (rs.trackError || 0) / 0.003 },
      { name: 'progress', lo: -1, hi: 1, raw: true, get: (rs) => rs.seamParam || 0 },
    ],
    pool11: [
      { name: 'L', lo: 0, hi: 0.03, get: (rs, o) => o.L },
      { name: 'W', lo: 0, hi: 0.03, get: (rs, o) => o.W },
      { name: 'area', lo: 0, hi: 7e-4, get: (rs, o) => o.area },
      { name: 'centroidAlong', lo: -0.02, hi: 0.02, get: (rs, o) => o.centroidAlong },
      { name: 'centroidAcross', lo: -0.02, hi: 0.02, get: (rs, o) => o.centroidAcross },
      { name: 'travelSpeed', lo: 0, hi: 0.03, get: (rs) => (rs.weldInput || {}).travelSpeed },
      { name: 'workDev', lo: -0.6, hi: 0.6, get: (rs) => (rs.weldInput || {}).workDev },
      { name: 'travelAngle', lo: -0.8, hi: 0.8, get: (rs) => (rs.weldInput || {}).travelAngle },
      { name: 'standoff', lo: 0, hi: 0.02, get: (rs) => (rs.weldInput || {}).standoff },
      { name: 'seamParam', lo: 0, hi: 1, get: (rs) => rs.seamParam },
      { name: 'trackError', lo: -0.02, hi: 0.02, get: (rs) => rs.trackError },
    ],
    req19: [
      // torch pose 6D (seam frame: x along, y lateral, z normal)
      { name: 'pose.x', lo: -0.02, hi: 0.02, get: (rs) => g(rs.pose, 0) },
      { name: 'pose.y', lo: -0.02, hi: 0.02, get: (rs) => g(rs.pose, 1) },
      { name: 'pose.z', lo: 0, hi: 0.03, get: (rs) => g(rs.pose, 2) },
      { name: 'pose.roll', lo: -1.6, hi: 1.6, get: (rs) => g(rs.pose, 3) },
      { name: 'pose.pitch', lo: -0.8, hi: 0.8, get: (rs) => g(rs.pose, 4) },
      { name: 'pose.yaw', lo: -0.8, hi: 0.8, get: (rs) => g(rs.pose, 5) },
      // seam deviation 3D
      { name: 'dev.along', lo: -0.01, hi: 0.01, get: (rs) => g(rs.deviation, 0) },
      { name: 'dev.lateral', lo: -0.01, hi: 0.01, get: (rs) => g(rs.deviation, 1) },
      { name: 'dev.height', lo: -0.01, hi: 0.01, get: (rs) => g(rs.deviation, 2) },
      // velocity 6D
      { name: 'vel.x', lo: -0.03, hi: 0.03, get: (rs) => g(rs.vel, 0) },
      { name: 'vel.y', lo: -0.03, hi: 0.03, get: (rs) => g(rs.vel, 1) },
      { name: 'vel.z', lo: -0.03, hi: 0.03, get: (rs) => g(rs.vel, 2) },
      { name: 'vel.wx', lo: -1, hi: 1, get: (rs) => g(rs.vel, 3) },
      { name: 'vel.wy', lo: -1, hi: 1, get: (rs) => g(rs.vel, 4) },
      { name: 'vel.wz', lo: -1, hi: 1, get: (rs) => g(rs.vel, 5) },
      // weld state 4D: arc length, speed, angle, progress (+ pool size enters via area error)
      { name: 'arcLength', lo: 0, hi: 0.012, get: (rs) => (rs.weldInput || {}).standoff },
      { name: 'speed', lo: 0, hi: 0.02, get: (rs) => (rs.weldInput || {}).travelSpeed },
      { name: 'angle', lo: -0.6, hi: 0.6, get: (rs) => (rs.weldInput || {}).workDev },
      { name: 'progress', lo: 0, hi: 1, get: (rs) => rs.seamParam },
    ],
  };
  const ACT_SPECS = {
    delta3: [
      { name: 'dTravelSpeed', lo: -0.0008, hi: 0.0008 },
      { name: 'dWorkAngle', lo: -0.05, hi: 0.05 },
      { name: 'dLateralOffset', lo: -0.0010, hi: 0.0010 },
    ],
    req7: [
      { name: 'dAlong', lo: -0.0005, hi: 0.0005 },
      { name: 'dLateralOffset', lo: -0.0010, hi: 0.0010 },
      { name: 'dStandoff', lo: -0.0004, hi: 0.0004 },
      { name: 'dWristRoll', lo: -0.05, hi: 0.05 },
      { name: 'dTravelAngle', lo: -0.03, hi: 0.03 },
      { name: 'dWorkAngle', lo: -0.05, hi: 0.05 },
      { name: 'travelSpeed', lo: 0.003, hi: 0.015, absolute: true },
    ],
  };
  // legacy v1 spec objects (names/low/high arrays) — kept for compatibility
  const DEFAULT_OBS_SPEC = { names: OBS_SPECS.pool11.map(f => f.name), low: OBS_SPECS.pool11.map(f => f.lo), high: OBS_SPECS.pool11.map(f => f.hi) };
  const DEFAULT_ACT_SPEC = { names: ACT_SPECS.delta3.map(f => f.name), low: ACT_SPECS.delta3.map(f => f.lo), high: ACT_SPECS.delta3.map(f => f.hi) };

  class Codec {
    /** new Codec('compact3','delta3') | new Codec(featureArray, actArray) */
    constructor(obs = 'pool11', act = 'delta3', opts = {}) {
      this.obsName = typeof obs === 'string' ? obs : 'custom';
      this.actName = typeof act === 'string' ? act : 'custom';
      this.obsF = typeof obs === 'string' ? OBS_SPECS[obs] : obs;
      this.actF = typeof act === 'string' ? ACT_SPECS[act] : act;
      if (!this.obsF || !this.actF) throw new Error('[WeldCell] unknown codec spec ' + obs + '/' + act);
      this.normalize = opts.normalize !== false;
      this.ctx = { target: opts.targetArea ?? null };
      this.fixedTarget = opts.targetArea ?? null;
    }
    reset() { this.ctx.target = this.fixedTarget; }
    get obsDim() { return this.obsF.length; }
    get actDim() { return this.actF.length; }
    get obsNames() { return this.obsF.map(f => f.name); }
    get actNames() { return this.actF.map(f => f.name); }
    encodeObs(rs, obs) {
      if (obs && obs.status === 'ok' && this.ctx.target == null && obs.area > 0) this.ctx.target = obs.area;
      const o = new Float32Array(this.obsF.length);
      for (let i = 0; i < this.obsF.length; i++) {
        const f = this.obsF[i]; let v = f.get(rs || {}, obs || emptyObs(), this.ctx);
        if (!Number.isFinite(v)) v = 0;
        o[i] = (f.raw || !this.normalize) ? clamp(v, -5, 5) : clamp((v - f.lo) / (f.hi - f.lo) * 2 - 1, -1, 1);
      }
      return o;
    }
    /** vec in [-1,1]^actDim -> ControlAction */
    decodeAction(vec) {
      const a = {};
      for (let i = 0; i < this.actF.length; i++) {
        const f = this.actF[i], x = clamp(Number(vec[i]) || 0, -1, 1);
        a[f.name] = this.normalize ? (x + 1) / 2 * (f.hi - f.lo) + f.lo : vec[i];
      }
      return a;
    }
    /** ControlAction (deltas/absolutes) -> normalised vec (for imitation datasets) */
    encodeAction(a) {
      const v = new Float32Array(this.actF.length);
      for (let i = 0; i < this.actF.length; i++) { const f = this.actF[i], x = (a && a[f.name] != null) ? a[f.name] : (f.absolute ? (f.lo + f.hi) / 2 : 0);
        v[i] = clamp((x - f.lo) / (f.hi - f.lo) * 2 - 1, -1, 1); }
      return v;
    }
    randomAction(r = rng) { const v = new Float32Array(this.actDim); for (let i = 0; i < v.length; i++) v[i] = r() * 2 - 1; return v; }
    toJSON() { return { obs: this.obsName, act: this.actName, obsNames: this.obsNames, actNames: this.actNames,
      obsLo: this.obsF.map(f => f.lo), obsHi: this.obsF.map(f => f.hi), obsRaw: this.obsF.map(f => !!f.raw),
      actLo: this.actF.map(f => f.lo), actHi: this.actF.map(f => f.hi), actAbsolute: this.actF.map(f => !!f.absolute) }; }
  }

  /* ===========================================================================
   * WeldCellEnv — Gym-like environment ON the Coordinator. The AGENT is the
   * controller: env.step(actionVec) -> {obs, reward, done, info}.
   * ========================================================================= */
  class WeldCellEnv {
    constructor({ coordinator, codec = new Codec('compact3', 'delta3'), reward = null, dt = 1 / 20,
                  maxSteps = 600, targetArea = null } = {}) {
      this.co = coordinator; this.co.setController(null);
      this.codec = codec; this.dt = dt; this.maxSteps = maxSteps;
      this.reward = reward || new PoolHoldReward({ targetArea });
      this.co.setReward(this.reward);
      this._n = 0;
      this.observationSpace = { shape: [codec.obsDim], low: -1, high: 1, names: codec.obsNames };
      this.actionSpace = { shape: [codec.actDim], low: -1, high: 1, names: codec.actNames };
    }
    reset() {
      this.co.reset(); this.codec.reset(); this._n = 0;
      const f = this.co.step(this.dt);
      return this.codec.encodeObs(f.robotState, f.poolObs);
    }
    step(actionVec) {
      const act = this.codec.decodeAction(actionVec);
      const f = this.co.step(this.dt, act);
      this._n++;
      const obs = this.codec.encodeObs(f.robotState, f.poolObs);
      const done = (f.robotState.seamParam >= 1) || (f.poolObs.status === 'burn_through') || (this._n >= this.maxSteps);
      return { obs, reward: f.reward, done, info: { t: f.t, status: f.poolObs.status, area: f.poolObs.area,
        trackError: f.robotState.trackError, action: act } };
    }
  }

  /* ===========================================================================
   * TrajectoryRecorder — frames -> ML-ready (s, a, r, s', done) JSONL with a
   * metadata header line. The SAME schema is written for simulated, live and
   * REAL sessions, so real logs feed Real2Sim calibration / replay directly.
   * ========================================================================= */
  class TrajectoryRecorder {
    constructor(codec = null, meta = {}) { this.codec = codec; this.meta = meta; this.frames = []; this.recording = false; }
    start() { this.recording = true; }
    stop()  { this.recording = false; }
    clear() { this.frames = []; }
    push(frame) { if (this.recording) this.frames.push(_slimFrame(frame)); }
    transitions() {
      const out = [], F = this.frames;
      if (this.codec) this.codec.reset();
      const enc = this.codec ? F.map(f => Array.from(this.codec.encodeObs(f.robotState, f.poolObs))) : null;
      for (let i = 0; i < F.length - 1; i++) {
        const f = F[i], g2 = F[i + 1];
        out.push({ t: f.t, obs: enc ? enc[i] : null, action: g2.action || null,
          action_vec: (this.codec && g2.action) ? Array.from(this.codec.encodeAction(g2.action)) : null,
          reward: g2.reward, next_obs: enc ? enc[i + 1] : null, done: i === F.length - 2,
          weldInput: f.weldInput, poolObs: f.poolObs, robotState: f.robotState });
      }
      return out;
    }
    header(extra = {}) {
      return Object.assign({ _type: 'header', schema: SCHEMA, coreVersion: VERSION, created: new Date().toISOString(),
        frames: this.frames.length, codec: this.codec ? this.codec.toJSON() : null }, this.meta, extra);
    }
    toJSONL(extra = {}) {
      return [JSON.stringify(this.header(extra))].concat(this.transitions().map(x => JSON.stringify(x))).join('\n');
    }
    summary() { return { frames: this.frames.length, recording: this.recording }; }
  }
  function _round(x) { return (typeof x === 'number' && Number.isFinite(x)) ? Math.round(x * 1e7) / 1e7 : x; }
  function _slim(o) { if (!o || typeof o !== 'object') return _round(o);
    if (Array.isArray(o) || ArrayBuffer.isView(o)) return Array.from(o, _round);
    const r = {}; for (const k in o) { const v = o[k]; if (typeof v === 'function' || k === 'extra') continue;
      r[k] = (v && typeof v === 'object') ? _slim(v) : _round(v); } return r; }
  function _slimFrame(f) { return { t: _round(f.t), step: f.step, weldInput: _slim(f.weldInput), poolObs: _slim(f.poolObs),
    robotState: _slim(Object.assign({}, f.robotState, { weldInput: undefined })), action: _slim(f.action), reward: _round(f.reward) }; }
  /** Parse a JSONL produced by TrajectoryRecorder (header + transitions). */
  function parseTrajectoryJSONL(text) {
    const lines = text.split(/\r?\n/).filter(l => l.trim()); let header = null; const rows = [];
    for (const l of lines) { const o = JSON.parse(l); if (o._type === 'header') header = o; else rows.push(o); }
    return { header, rows };
  }

  /* ===========================================================================
   * WORLD-MODEL SEAM — a learned dynamics implementing the SAME IWeldPool
   * contract can REPLACE the physics pool (model-based RL / planning / dreaming).
   *   predictFn(latent, weldInput, dt) -> { latent, obs }
   * ========================================================================= */
  class WorldModelPool {
    constructor(predictFn, opts = {}) {
      this.meta = { name: opts.name || 'WorldModel', kind: 'learned' };
      this.predict = predictFn;
      this.latent = opts.latent0 ? Float32Array.from(opts.latent0) : new Float32Array(opts.latentDim || 8);
      this._wi = null; this._obs = emptyObs(); this._latent0 = Float32Array.from(this.latent);
    }
    reset(initState) { this.latent = initState ? Float32Array.from(initState) : Float32Array.from(this._latent0); this._obs = emptyObs(); }
    setInput(wi) { this._wi = wi; }
    step(dt) { const r = this.predict(this.latent, this._wi, dt); this.latent = r.latent || this.latent; this._obs = r.obs || this._obs; return this._obs; }
    observe() { return this._obs; }
    getState() { return Float32Array.from(this.latent); }
    setState(s) { this.latent = Float32Array.from(s); }
  }
  function makeIdentityWorldModel(physicsRef) {
    return new WorldModelPool(() => ({ obs: physicsRef ? physicsRef.observe() : emptyObs() }), { name: 'IdentityWM' });
  }

  /* ===========================================================================
   * Reference implementations of the contract (headless, testable).
   * ========================================================================= */
  /** Process parameters shared by all components (single source of truth). */
  function makeProcess(o = {}) {
    const p = { I: o.I ?? 175, V: o.V ?? 24, eta: o.eta ?? 0.80, material: o.material ?? 'steel',
      T0: o.T0 ?? 20, thickness: o.thickness ?? 0.009 };
    p.Q = p.eta * p.V * p.I; return p;
  }

  /** Kinematic surrogate of the cell: seam with a lateral bow + wandering disturbance. */
  class SurrogateRobotCell {
    constructor(o = {}) {
      this.meta = { name: 'CellSurrogate', kind: 'analytic' };
      this.o = Object.assign({ seamLen: 0.18, bow: 0.010, noiseAmp: 0.0016, workRef: 45 * DEG, travelAngle: -15 * DEG,
        standoff: 0.002, travelSpeed: 0.007, position: 'F', trackGain: 0.6, limits: DEFAULT_LIMITS }, o);
      this.process = o.process || makeProcess();
      this.rng = o.rng || rng;
      this.reset();
    }
    reset() {
      const o = this.o;
      this.sp = { travelSpeed: o.travelSpeed, workRef: o.workRef, workAngle: o.workRef, travelAngle: o.travelAngle,
        standoff: o.standoff, lateralOffset: 0, wristRoll: 0, along: 0 };
      this.st = { s: 0, noise: 0, nv: 0, seamY: 0, trk: 0, prevPose: null, vel: [0, 0, 0, 0, 0, 0] };
    }
    step(dt) {
      const st = this.st, sp = this.sp, o = this.o, r = this.rng;
      st.nv += (-st.nv * 1.2 + (r() - 0.5) * 0.04) * dt;
      st.noise = clamp(st.noise + st.nv * dt, -o.noiseAmp, o.noiseAmp);
      st.seamY = o.bow * Math.sin(Math.PI * st.s) + st.noise;
      // built-in low-level tracking (the "seam sensor" loop of the robot controller)
      sp.lateralOffset = clamp(sp.lateralOffset + o.trackGain * (st.seamY - sp.lateralOffset) * dt, o.limits.lateralOffset[0], o.limits.lateralOffset[1]);
      st.trk = st.seamY - sp.lateralOffset;
      st.s = clamp(st.s + sp.travelSpeed * dt / o.seamLen, 0, 1);
      const pose = this._pose();
      if (st.prevPose && dt > 0) st.vel = pose.map((v, i) => (v - st.prevPose[i]) / dt);
      st.prevPose = pose;
    }
    _pose() { const sp = this.sp; return [sp.along || 0, sp.lateralOffset, sp.standoff, sp.wristRoll || 0, sp.travelAngle, sp.workAngle - sp.workRef]; }
    getWeldInput() {
      const sp = this.sp, p = this.process;
      return { weldPoint: [0, 0, this.st.s * this.o.seamLen], travelDir: [0, 0, 1], acrossDir: [1, 0, 0], surfaceNormal: [0, 1, 0],
        gravityDir: [0, -1, 0], travelSpeed: sp.travelSpeed, workAngle: sp.workAngle, workDev: sp.workAngle - sp.workRef,
        travelAngle: sp.travelAngle, standoff: sp.standoff, weldPosition: this.o.position,
        process: Object.assign({}, p, { Q: p.eta * p.V * p.I }), seamParam: this.st.s, contact: true };
    }
    getState() {
      const st = this.st, sp = this.sp;
      return { q: [], tcp: [0, 0, 0], pose: this._pose(), vel: st.vel.slice(), deviation: [0, st.trk, sp.standoff - this.o.standoff],
        ikError: 0, phase: st.s >= 1 ? 'done' : 'weld', seamParam: st.s, trackError: st.trk, lateralOffset: sp.lateralOffset,
        weldInput: this.getWeldInput() };
    }
    applyControl(a) { applyAction(this.sp, a, this.o.limits); }
  }

  /** Physics pool: wraps any model with evaluate(params) (WeldPoolModel) + first-order lag. */
  class ModelPool {
    constructor(model, o = {}) {
      this.meta = { name: o.name || 'WeldPoolModel', kind: 'analytic' };
      this.model = model; this.tau = o.tau ?? 0.15; this.calib = o.calib || null; this.every = o.every ?? 1;
      this.reset();
    }
    reset() { this._wi = null; this._obs = emptyObs(); this._tgt = null; this._k = 0; }
    setInput(wi) { this._wi = wi; }
    static paramsFrom(wi, calib) {
      const p = wi.process || {}, POS = { F: 'flat', H: 'horiz', VU: 'vertup', VD: 'vertdn', OH: 'over' };
      return { I: p.I, V: p.V, eta: p.eta, T0: p.T0 ?? 20, material: p.material || 'steel', grav: 0.6,
        thicknessMM: (p.thickness ?? 0.009) * 1000, v_mmps: Math.max((wi.travelSpeed || 0) * 1000, 0.3),
        travelDeg: (wi.travelAngle || 0) / DEG, workDeg: (wi.workDev || 0) / DEG,
        arcLenMM: Math.max((wi.standoff || 0) * 1000, 0.3), position: POS[wi.weldPosition] || 'flat', calib: calib || undefined };
    }
    step(dt) {
      if (!this._wi) return this._obs;
      if (this._tgt == null || (this._k++ % this.every) === 0) {
        const r = this.model.evaluate(ModelPool.paramsFrom(this._wi, this.calib));
        this._tgt = { L: r.lenMM / 1000, W: r.widMM / 1000, penetration: r.penMM / 1000, area: r.areaMM / 1e6,
          centroidAlong: r.cxMM / 1000, centroidAcross: r.cyMM / 1000, peakT: r.peakC, t85: r.t85, status: r.status };
      }
      const T = this._tgt, o = this._obs;
      if (T.status === 'no_pool') { this._obs = Object.assign(emptyObs(), { status: 'no_pool' }); return this._obs; }
      const k = (o.status === 'ok' || o.status === 'burn_through') && this.tau > 0 ? clamp(dt / this.tau, 0, 1) : 1;
      const lerp = (a, b) => (a == null || !Number.isFinite(a)) ? b : a + (b - a) * k;
      this._obs = { t: 0, L: lerp(o.L, T.L), W: lerp(o.W, T.W), penetration: lerp(o.penetration, T.penetration), area: lerp(o.area, T.area),
        centroidAlong: lerp(o.centroidAlong, T.centroidAlong), centroidAcross: lerp(o.centroidAcross, T.centroidAcross),
        peakT: T.peakT, t85: T.t85, status: T.status };
      return this._obs;
    }
    observe() { return this._obs; }
    getState() { return { obs: Object.assign({}, this._obs), tgt: this._tgt && Object.assign({}, this._tgt) }; }
    setState(s) { this._obs = Object.assign({}, s.obs); this._tgt = s.tgt && Object.assign({}, s.tgt); }
  }

  /** Run n ticks and summarise (mean reward, area CV, tracking RMS). */
  function rollout(co, n, dt) {
    co.reset(); let R = 0, k = 0; const areas = []; let tr2 = 0, eA = 0, nA = 0;
    for (let i = 0; i < n; i++) { const f = co.step(dt); R += f.reward; k++;
      if (f.poolObs.status === 'ok') { areas.push(f.poolObs.area); const T = co.reward && co.reward.target; if (T) { eA += Math.abs(f.poolObs.area - T) / T; nA++; } }
      tr2 += (f.robotState.trackError || 0) ** 2;
      if (f.robotState.seamParam >= 1) break; }
    const mA = areas.reduce((a, b) => a + b, 0) / Math.max(areas.length, 1);
    const sA = Math.sqrt(areas.reduce((a, b) => a + (b - mA) ** 2, 0) / Math.max(areas.length, 1));
    return { steps: k, meanReward: R / Math.max(k, 1), areaMean: mA, areaCV: mA > 0 ? sA / mA : NaN,
      areaErr: nA ? eA / nA : NaN, trackRMS: Math.sqrt(tr2 / Math.max(k, 1)) };
  }

  return {
    VERSION, SCHEMA, clamp, DEG, makeRng, setSeed, rng,
    EventBus, Coordinator, PoolHoldReward,
    AutoHoldController, PolicyController,
    Codec, OBS_SPECS, ACT_SPECS, DEFAULT_OBS_SPEC, DEFAULT_ACT_SPEC,
    WeldCellEnv, TrajectoryRecorder, parseTrajectoryJSONL,
    WorldModelPool, makeIdentityWorldModel,
    DEFAULT_LIMITS, applyAction, makeProcess, SurrogateRobotCell, ModelPool, rollout,
    assertPool, assertRobot, assertCtl, emptyObs,
  };
});
