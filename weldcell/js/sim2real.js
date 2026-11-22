/* =============================================================================
 * sim2real.js — WeldCell Sim2Real / Real2Sim toolkit v2.0
 * -----------------------------------------------------------------------------
 *  SIM → REAL
 *   - Bridge protocol 'weldcell-bridge/1' (JSON over WebSocket), aligned with
 *     the API spec (Set_Torch_Pose / Set_Camera_Pose / Set_Trigger / Get_Metrics /
 *     Reset_Episode / Get_Calibration_Info) + streaming state/metrics + E-stop.
 *   - BridgeClient + transports (WebSocket, in-process Loopback with latency).
 *   - RealRobotCell / RealPoolSensor : IRobotCell / IWeldPool adapters over the
 *     bridge -> the SAME Coordinator, controllers and policies drive hardware.
 *   - SafetyLimiter : bounds, rate limits, watchdog, E-stop (always between a
 *     policy and the real robot).
 *   - DomainRandomizer : physics variants (efficiency, conductivity, footprint)
 *     + sensor noise + latency, for robust training.
 *   - MockRealBackend : a "virtual real machine" with hidden true parameters,
 *     actuator lag, sensor noise and latency (develop/test without hardware;
 *     mirrored in bridge/ws_bridge.py).
 *  REAL → SIM
 *   - parseMeasurements : CSV / JSONL (recorder or bridge logs) -> records
 *   - Calibrator : Nelder–Mead fit of the pool model's calibration knobs
 *     (effScale, kScale, sigScale) to measured pool width/length/penetration.
 *   - ReplayPool / ReplayRobot : replay real logs through the contract.
 *   - ResidualMonitor : shadow-mode (digital twin) sim-vs-real residuals.
 * ========================================================================== */
(function (root, factory) {
  const isNode = typeof module !== 'undefined' && module.exports;
  const core = isNode ? require('./core.js') : root.WeldCell;
  const api = factory(core);
  if (isNode) module.exports = api;
  if (typeof window !== 'undefined') window.WeldCell = Object.assign(window.WeldCell || {}, api);
})(typeof self !== 'undefined' ? self : this, function (C) {
  'use strict';
  const clamp = C.clamp, DEG = C.DEG;
  const PROTOCOL = 'weldcell-bridge/1';
  const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());

  /* =========================================================== protocol */
  // client -> server
  const MSG = {
    HELLO: 'hello', SET_PROCESS: 'set_process', SET_TORCH_POSE: 'set_torch_pose', SET_CAMERA_POSE: 'set_camera_pose',
    SET_TRIGGER: 'set_trigger', CONTROL: 'control', RESET_EPISODE: 'reset_episode', GET_CALIBRATION: 'get_calibration_info',
    GET_METRICS: 'get_metrics', ESTOP: 'estop', PING: 'ping',
    // server -> client
    HELLO_ACK: 'hello_ack', STATE: 'state', METRICS: 'metrics', CALIBRATION: 'calibration', ACK: 'ack', ERROR: 'error', PONG: 'pong',
  };
  function encode(type, data, seq) { return JSON.stringify({ v: 1, protocol: PROTOCOL, type, seq: seq ?? 0, t: now() / 1000, data: data || {} }); }
  function decode(text) { const m = typeof text === 'string' ? JSON.parse(text) : text; if (!m || !m.type) throw new Error('bad message'); return m; }

  /** Metrics message (API-spec JSON + pool block) -> PoolObservation (SI). */
  function metricsToObs(m) {
    const p = (m && m.pool) || {};
    const ok = p.status ? p.status : ((p.W_mm > 0) ? 'ok' : 'no_pool');
    return { t: m.t || 0, L: (p.L_mm || 0) / 1000, W: (p.W_mm || 0) / 1000, penetration: p.pen_mm != null ? p.pen_mm / 1000 : NaN,
      area: p.area_mm2 != null ? p.area_mm2 / 1e6 : Math.PI * (p.L_mm || 0) * (p.W_mm || 0) / 4 / 1e6,
      centroidAlong: (p.cx_mm || 0) / 1000, centroidAcross: (p.cy_mm || 0) / 1000, peakT: p.peakC ?? NaN, t85: p.t85 ?? NaN,
      status: ok, source: 'real', evalScore: m.eval_score ?? null, arcLength: m.arc_length_mm != null ? m.arc_length_mm / 1000 : null,
      deviation: m.deviation_mm != null ? m.deviation_mm / 1000 : null, velocity: m.velocity_cm_min != null ? m.velocity_cm_min / 6000 : null };
  }
  /** PoolObservation (+ robot state) -> metrics message (used by mock backends). */
  function obsToMetrics(obs, rs, extra = {}) {
    const wi = (rs && rs.weldInput) || {};
    return Object.assign({ arc_length_mm: (wi.standoff || 0) * 1000, deviation_mm: ((rs && rs.trackError) || 0) * 1000,
      velocity_cm_min: (wi.travelSpeed || 0) * 6000, eval_score: null,
      pool: { L_mm: obs.L * 1000, W_mm: obs.W * 1000, pen_mm: obs.penetration * 1000, area_mm2: obs.area * 1e6,
        cx_mm: obs.centroidAlong * 1000, cy_mm: obs.centroidAcross * 1000, peakC: obs.peakT, t85: obs.t85, status: obs.status } }, extra);
  }

  /* ========================================================= transports */
  class Emitter { constructor() { this._h = {}; }
    on(e, f) { (this._h[e] || (this._h[e] = [])).push(f); return () => { const a = this._h[e]; const i = a.indexOf(f); if (i >= 0) a.splice(i, 1); }; }
    emit(e, x) { (this._h[e] || []).slice().forEach(f => f(x)); } }
  /** In-process transport pair with one-way latency (ms). */
  class LoopbackTransport extends Emitter {
    constructor(latencyMs = 0) { super(); this.peer = null; this.latencyMs = latencyMs; this.open = true; }
    static pair(latencyMs = 0) { const a = new LoopbackTransport(latencyMs), b = new LoopbackTransport(latencyMs); a.peer = b; b.peer = a;
      setTimeout(() => { a.emit('open'); b.emit('open'); }, 0); return [a, b]; }
    send(text) { if (!this.open) return; const p = this.peer; const d = () => p.open && p.emit('message', text);
      if (this.latencyMs > 0) setTimeout(d, this.latencyMs); else Promise.resolve().then(d); }
    close() { this.open = false; this.emit('close'); if (this.peer && this.peer.open) { this.peer.open = false; this.peer.emit('close'); } }
  }
  /** Browser/WHATWG WebSocket transport. */
  class WebSocketTransport extends Emitter {
    constructor(url, WS) { super(); const W = WS || (typeof WebSocket !== 'undefined' ? WebSocket : null);
      if (!W) throw new Error('WebSocket not available'); this.ws = new W(url); this.open = false;
      this.ws.onopen = () => { this.open = true; this.emit('open'); };
      this.ws.onmessage = (e) => this.emit('message', e.data);
      this.ws.onclose = () => { this.open = false; this.emit('close'); };
      this.ws.onerror = (e) => this.emit('error', e); }
    send(text) { if (this.open) this.ws.send(text); }
    close() { try { this.ws.close(); } catch (e) { /* ignore */ } }
  }

  /* ========================================================= BridgeClient */
  class BridgeClient extends Emitter {
    constructor(transport) {
      super(); this.tr = transport; this.seq = 0; this.pending = new Map();
      this.latest = { state: null, metrics: null, calibration: null, hello: null };
      this.stats = { rx: 0, tx: 0, rttMs: NaN, lastRx: 0, connected: false };
      transport.on('open', () => { this.stats.connected = true; this.emit('open'); this.send(MSG.HELLO, { client: 'weldcell-ui', protocol: PROTOCOL }); });
      transport.on('close', () => { this.stats.connected = false; this.emit('close'); });
      transport.on('error', (e) => this.emit('error', e));
      transport.on('message', (txt) => this._rx(txt));
    }
    _rx(txt) {
      let m; try { m = decode(txt); } catch (e) { this.emit('error', e); return; }
      this.stats.rx++; this.stats.lastRx = now();
      if (m.type === MSG.STATE) this.latest.state = m.data;
      else if (m.type === MSG.METRICS) this.latest.metrics = m.data;
      else if (m.type === MSG.CALIBRATION) this.latest.calibration = m.data;
      else if (m.type === MSG.HELLO_ACK) this.latest.hello = m.data;
      if (m.ack != null || m.type === MSG.ACK || m.type === MSG.PONG || m.type === MSG.CALIBRATION || m.type === MSG.ERROR) {
        const id = m.ack != null ? m.ack : m.data && m.data.seq; const p = this.pending.get(id);
        if (p) { this.pending.delete(id); p.t1 = now(); this.stats.rttMs = p.t1 - p.t0; (m.type === MSG.ERROR ? p.reject : p.resolve)(m); }
      }
      this.emit(m.type, m); this.emit('message', m);
    }
    send(type, data) { const s = ++this.seq; this.tr.send(encode(type, data, s)); this.stats.tx++; return s; }
    request(type, data, timeoutMs = 2000) {
      return new Promise((resolve, reject) => { const s = this.send(type, data); const p = { resolve, reject, t0: now() };
        this.pending.set(s, p); setTimeout(() => { if (this.pending.has(s)) { this.pending.delete(s); reject(new Error('timeout ' + type)); } }, timeoutMs); });
    }
    ping() { return this.request(MSG.PING, {}); }
    age() { return this.stats.lastRx ? now() - this.stats.lastRx : Infinity; }
    close() { this.tr.close(); }
  }

  /* ========================================================= SafetyLimiter */
  class SafetyLimiter {
    constructor(o = {}) {
      this.limits = Object.assign({}, C.DEFAULT_LIMITS, o.limits || {});
      // max change per second for each setpoint (slew-rate limits)
      this.rate = Object.assign({ travelSpeed: 0.004, workAngle: 0.35, travelAngle: 0.35, standoff: 0.004, lateralOffset: 0.004, wristRoll: 0.5 }, o.rate || {});
      this.watchdogMs = o.watchdogMs ?? 500; this.estopped = false; this.armed = false; this.violations = 0;
    }
    arm(on = true) { this.armed = on; if (on) this.estopped = false; }
    estop() { this.estopped = true; this.armed = false; }
    /** Returns an ABSOLUTE, bounded, slew-limited setpoint action (or null if not allowed). */
    filter(action, sp, dt, ageMs = 0) {
      if (!action || this.estopped || !this.armed) return null;
      if (ageMs > this.watchdogMs) { this.violations++; return null; }
      const tgt = C.applyAction(Object.assign({}, sp), action, this.limits), out = {};
      for (const k of ['travelSpeed', 'workAngle', 'travelAngle', 'standoff', 'lateralOffset', 'wristRoll']) {
        if (tgt[k] == null || sp[k] == null) continue;
        const maxD = (this.rate[k] || Infinity) * Math.max(dt, 1e-3), d = tgt[k] - sp[k];
        if (Math.abs(d) > maxD) this.violations++;
        out[k] = sp[k] + clamp(d, -maxD, maxD);
      }
      return out;
    }
  }

  /* ====================================================== real adapters */
  /** IRobotCell backed by the bridge. The real controller owns the clock; step() is a no-op watchdog. */
  class RealRobotCell {
    constructor(client, o = {}) {
      this.meta = { name: 'RealRobot', kind: 'real' }; this.client = client;
      this.limiter = o.limiter || new SafetyLimiter(); this.workRef = o.workRef ?? 45 * DEG; this._lastDt = 0.05; this.sent = 0;
    }
    reset() { this.client.send(MSG.RESET_EPISODE, { scenario_id: 'default' }); }
    step(dt) { this._lastDt = dt; }
    _sp() { const s = this.client.latest.state || {}, wi = s.weldInput || {};
      return { travelSpeed: wi.travelSpeed ?? 0.007, workRef: this.workRef, workAngle: wi.workAngle ?? this.workRef, travelAngle: wi.travelAngle ?? -15 * DEG,
        standoff: wi.standoff ?? 0.002, lateralOffset: s.lateralOffset ?? 0, wristRoll: s.wristRoll ?? 0 }; }
    getWeldInput() { const s = this.client.latest.state; return Object.assign({ travelSpeed: 0.007, workDev: 0, process: {} }, (s && s.weldInput) || {}); }
    getState() { const s = this.client.latest.state || {}; return Object.assign({ seamParam: 0, trackError: 0, phase: 'idle' }, s, { weldInput: this.getWeldInput() }); }
    applyControl(a) {
      const cmd = this.limiter.filter(a, this._sp(), this._lastDt, this.client.age());
      if (cmd) { this.client.send(MSG.CONTROL, cmd); this.sent++; }
      return cmd;
    }
  }
  /** IWeldPool backed by the bridge's metrics stream (the real pool sensor, e.g. vision features). */
  class RealPoolSensor {
    constructor(client, o = {}) { this.meta = { name: 'RealPoolSensor', kind: 'real' }; this.client = client; this.staleMs = o.staleMs ?? 500; }
    reset() {} setInput() {}
    step() { return this.observe(); }
    observe() { const m = this.client.latest.metrics; if (!m) return C.emptyObs();
      const o = metricsToObs(m); if (this.client.age() > this.staleMs) o.status = 'stale'; return o; }
  }

  /* ===================================================== domain randomization */
  class DomainRandomizer {
    constructor(o = {}) {
      this.ranges = Object.assign({ effScale: [0.85, 1.10], kScale: [0.85, 1.20], sigScale: [0.85, 1.20] }, o.ranges || {});
      this.obsNoise = o.obsNoise ?? 0.03;          // relative noise on L/W/area
      this.trackNoise = o.trackNoise ?? 0.0003;    // m (not applied to the pool: see wrapRobot)
      this.delay = o.delay ?? [0, 3];              // observation delay in ticks
      this.variants = [];
    }
    /** Pre-build K physics variants (surrogate tables) so per-episode randomization is cheap. */
    prepare(model, base, SurrogateModel, K = 6, r = C.rng) {
      this.variants = []; this.samples = [];
      for (let k = 0; k < K; k++) {
        const calib = {}; for (const key in this.ranges) calib[key] = r.uniform ? r.uniform(...this.ranges[key]) : this.ranges[key][0] + r() * (this.ranges[key][1] - this.ranges[key][0]);
        this.samples.push(calib);
        this.variants.push(SurrogateModel ? new SurrogateModel(model, Object.assign({}, base, { calib })) : null);
      }
      return this;
    }
    wrapPool(inner, r = C.rng) { return new RandomizedPool(inner, this, r); }
    toJSON() { return { ranges: this.ranges, obsNoise: this.obsNoise, delay: this.delay, variants: this.samples || [] }; }
  }
  class RandomizedPool {
    constructor(inner, dr, r) { this.inner = inner; this.dr = dr; this.r = r; this.meta = { name: 'Randomized(' + (inner.meta || {}).name + ')', kind: 'randomized' }; this.reset(); }
    reset(s) {
      const v = this.dr.variants; if (v.length && v[0]) { this.variant = Math.floor(this.r() * v.length); this.inner.model = v[this.variant]; }
      const [d0, d1] = this.dr.delay; this.delaySteps = d0 + Math.floor(this.r() * (d1 - d0 + 1)); this.buf = [];
      this.inner.reset(s); this._obs = C.emptyObs();
    }
    setInput(wi) { this.inner.setInput(wi); }
    step(dt) {
      const o = Object.assign({}, this.inner.step(dt)), n = this.dr.obsNoise;
      if (o.status === 'ok' && n > 0) { const g = () => 1 + n * this.r.gauss(); o.L *= g(); o.W *= g(); o.area *= g(); }
      this.buf.push(o); while (this.buf.length > this.delaySteps + 1) this.buf.shift();
      this._obs = this.buf[0]; return this._obs;
    }
    observe() { return this._obs; }
  }

  /* ==================================================== virtual real machine */
  /**
   * MockRealBackend: server side of the bridge. Hidden "true" plant differs from
   * the nominal sim (calibration truth, actuator lag, sensor noise, latency).
   * Use: const [cli, srv] = LoopbackTransport.pair(15); new MockRealBackend(srv, {model}).start(20)
   */
  class MockRealBackend {
    constructor(transport, o = {}) {
      this.tr = transport; this.model = o.model; this.rng = o.rng || C.makeRng(o.seed ?? 4242);
      this.truth = Object.assign({ effScale: 0.90, kScale: 1.15, sigScale: 1.10 }, o.truth || {});
      this.sensorNoise = o.sensorNoise ?? 0.04; this.actTau = o.actTau ?? 0.25;   // actuator first-order lag (s)
      this.process = C.makeProcess(o.process || {}); this.position = o.position || 'F';
      this.robot = new C.SurrogateRobotCell({ process: this.process, position: this.position, rng: this.rng, trackGain: 0.45, bow: o.bow ?? 0.008 });
      this.pool = new C.ModelPool(this.model, { tau: 0.25, calib: this.truth, name: 'TruePlant' });
      this.cmd = null; this.arc = true; this.estopped = false; this.t = 0; this.timer = null; this.rateHz = 20;
      transport.on('message', (txt) => this._rx(txt));
    }
    _send(type, data, ack) { const m = JSON.parse(encode(type, data, 0)); if (ack != null) m.ack = ack; this.tr.send(JSON.stringify(m)); }
    _rx(txt) {
      let m; try { m = decode(txt); } catch (e) { return; }
      const d = m.data || {};
      switch (m.type) {
        case MSG.HELLO: this._send(MSG.HELLO_ACK, { backend: 'MockRealBackend', protocol: PROTOCOL, rateHz: this.rateHz,
          capabilities: ['control', 'metrics', 'state', 'set_process', 'set_trigger', 'reset_episode', 'calibration', 'estop'] }, m.seq); break;
        case MSG.PING: this._send(MSG.PONG, {}, m.seq); break;
        case MSG.CONTROL: if (!this.estopped) this.cmd = Object.assign(this.cmd || {}, d); this._send(MSG.ACK, { seq: m.seq }, m.seq); break;
        case MSG.SET_PROCESS: Object.assign(this.process, d); this.process.Q = this.process.eta * this.process.V * this.process.I;
          this.robot.process = this.process; this._send(MSG.ACK, {}, m.seq); break;
        case MSG.SET_TRIGGER: this.arc = !!d.on; this._send(MSG.ACK, {}, m.seq); break;
        case MSG.SET_TORCH_POSE: // pose expressed in the seam frame (mm, rad): y = lateral, z = standoff
          this.cmd = Object.assign(this.cmd || {}, { lateralOffset: (d.position || [0, 0, 0])[1] / 1000, standoff: (d.position || [0, 0, 2])[2] / 1000,
            workAngle: this.robot.sp.workRef + (d.rotation || [0, 0, 0])[2], travelAngle: (d.rotation || [0, 0, 0])[1] }); this._send(MSG.ACK, {}, m.seq); break;
        case MSG.RESET_EPISODE: this.robot.reset(); this.pool.reset(); this.cmd = null; this.t = 0; this.estopped = false; this._send(MSG.ACK, {}, m.seq); break;
        case MSG.GET_CALIBRATION: this._send(MSG.CALIBRATION, { frame: 'seam', units: { length: 'mm', angle: 'rad' }, origin: [0, 0, 0], scale: 1.0,
          note: 'mock backend: seam frame == sim frame' }, m.seq); break;
        case MSG.GET_METRICS: this._send(MSG.METRICS, this._metrics(), m.seq); break;
        case MSG.ESTOP: this.estopped = true; this.cmd = null; this.arc = false; this._send(MSG.ACK, { estop: true }, m.seq); break;
        default: this._send(MSG.ERROR, { error: 'unknown type ' + m.type }, m.seq);
      }
    }
    tick(dt) {
      this.t += dt;
      if (this.cmd && !this.estopped) {   // actuator lag toward commanded setpoints
        const k = clamp(dt / this.actTau, 0, 1), sp = this.robot.sp, a = {};
        for (const key of ['travelSpeed', 'workAngle', 'travelAngle', 'standoff', 'lateralOffset', 'wristRoll'])
          if (this.cmd[key] != null) a[key] = sp[key] + (this.cmd[key] - sp[key]) * k;
        this.robot.applyControl(a);
      }
      if (!this.estopped) this.robot.step(dt);
      const wi = this.robot.getWeldInput();
      this.pool.setInput(wi); let o = this.pool.step(dt);
      if (!this.arc || this.estopped) o = C.emptyObs();
      this._last = { wi, o };
      const rs = this.robot.getState(); delete rs.weldInput;
      this._send(MSG.STATE, Object.assign(rs, { weldInput: wi, arc: this.arc, estopped: this.estopped, t: this.t }));
      this._send(MSG.METRICS, this._metrics());
    }
    _metrics() {
      if (!this._last) return obsToMetrics(C.emptyObs(), null);
      const o = Object.assign({}, this._last.o), n = this.sensorNoise, g = () => 1 + n * this.rng.gauss();
      if (o.status === 'ok') { o.L *= g(); o.W *= g(); o.area *= g(); o.penetration = NaN; }   // vision cannot see penetration
      const rs = this.robot.getState();
      return obsToMetrics(o, rs, { process: { I: this.process.I, V: this.process.V }, t: this.t });
    }
    start(rateHz = 20) { this.rateHz = rateHz; this.stop(); this.timer = setInterval(() => this.tick(1 / rateHz), 1000 / rateHz); return this; }
    stop() { if (this.timer) clearInterval(this.timer); this.timer = null; }
  }

  /* ================================================================ Real2Sim */
  const POS = { F: 'flat', H: 'horiz', VU: 'vertup', VD: 'vertdn', OH: 'over', flat: 'flat', horiz: 'horiz', vertup: 'vertup', vertdn: 'vertdn', over: 'over' };
  function _num(x) { const v = parseFloat(x); return Number.isFinite(v) ? v : null; }
  /**
   * Parse measurements from CSV (header row) or JSONL (TrajectoryRecorder / bridge log).
   * Record: { I, V, eta, v_mmps, arcLenMM, workDeg, travelDeg, position, material, T0, meas:{W, L, pen} (mm) }
   * CSV column aliases: I|current_A, V|voltage_V, eta, v_mmps|speed_mm_s|velocity_cm_min, arc_length_mm|standoff_mm,
   *   work_dev_deg, travel_deg, position, material, T0, width_mm|W_mm, length_mm|L_mm, pen_mm|penetration_mm
   */
  function parseMeasurements(text, defaults = {}) {
    const d = Object.assign({ I: 175, V: 24, eta: 0.8, arcLenMM: 2, workDeg: 0, travelDeg: -15, position: 'flat', material: 'steel', T0: 20 }, defaults);
    const t = text.trim(); const recs = [];
    if (t.startsWith('{')) {
      for (const line of t.split(/\r?\n/)) { if (!line.trim()) continue; let o; try { o = JSON.parse(line); } catch (e) { continue; }
        if (o._type === 'header') continue;
        const wi = o.weldInput || {}, po = o.poolObs || (o.pool ? metricsToObs(o) : null);
        if (!po || po.status !== 'ok' || !(po.W > 0)) continue;
        const pr = wi.process || o.process || {};
        recs.push({ I: pr.I ?? d.I, V: pr.V ?? d.V, eta: pr.eta ?? d.eta, material: pr.material || d.material, T0: pr.T0 ?? d.T0,
          v_mmps: (wi.travelSpeed ?? po.velocity ?? 0.007) * 1000, arcLenMM: (wi.standoff ?? po.arcLength ?? d.arcLenMM / 1000) * 1000,
          workDeg: (wi.workDev ?? 0) / DEG, travelDeg: wi.travelAngle != null ? wi.travelAngle / DEG : d.travelDeg,
          position: POS[wi.weldPosition] || d.position,
          meas: { W: po.W * 1000, L: po.L > 0 ? po.L * 1000 : null, pen: Number.isFinite(po.penetration) && po.penetration > 0 ? po.penetration * 1000 : null } });
      }
    } else {
      const lines = t.split(/\r?\n/).filter(l => l.trim() && !l.trim().startsWith('#'));
      const hdr = lines[0].split(/[,\t;]/).map(s => s.trim().toLowerCase());
      const col = (...names) => { for (const n of names) { const i = hdr.indexOf(n.toLowerCase()); if (i >= 0) return i; } return -1; };
      const ci = { I: col('I', 'current_A', 'current'), V: col('V', 'voltage_V', 'voltage'), eta: col('eta', 'efficiency'),
        v: col('v_mmps', 'speed_mm_s', 'speed'), vcm: col('velocity_cm_min', 'speed_cm_min'), a: col('arc_length_mm', 'standoff_mm', 'arclen_mm'),
        w: col('work_dev_deg', 'workdeg'), tr: col('travel_deg', 'travelangle_deg'), pos: col('position'), mat: col('material'), T0: col('T0', 'preheat'),
        W: col('width_mm', 'W_mm', 'w'), L: col('length_mm', 'L_mm', 'l'), pen: col('pen_mm', 'penetration_mm', 'pen') };
      for (const line of lines.slice(1)) { const c = line.split(/[,\t;]/).map(s => s.trim()); const get = (i) => i >= 0 ? c[i] : undefined;
        const W = _num(get(ci.W)); if (W == null) continue;
        let v = _num(get(ci.v)); if (v == null && ci.vcm >= 0) v = _num(get(ci.vcm)) * 10 / 60;
        recs.push({ I: _num(get(ci.I)) ?? d.I, V: _num(get(ci.V)) ?? d.V, eta: _num(get(ci.eta)) ?? d.eta, v_mmps: v ?? 7,
          arcLenMM: _num(get(ci.a)) ?? d.arcLenMM, workDeg: _num(get(ci.w)) ?? d.workDeg, travelDeg: _num(get(ci.tr)) ?? d.travelDeg,
          position: POS[get(ci.pos)] || d.position, material: get(ci.mat) || d.material, T0: _num(get(ci.T0)) ?? d.T0,
          meas: { W, L: _num(get(ci.L)), pen: _num(get(ci.pen)) } });
      }
    }
    return recs;
  }
  /** Thin a long time series (e.g. 20 Hz logs) into representative records (average per bucket). */
  function thinRecords(recs, maxN = 200) {
    if (recs.length <= maxN) return recs;
    const step = recs.length / maxN, out = [];
    for (let k = 0; k < maxN; k++) { const seg = recs.slice(Math.floor(k * step), Math.floor((k + 1) * step)); if (!seg.length) continue;
      const r = Object.assign({}, seg[Math.floor(seg.length / 2)]); const avg = (f) => { const v = seg.map(f).filter(x => x != null && Number.isFinite(x)); return v.length ? v.reduce((a, b) => a + b, 0) / v.length : null; };
      r.v_mmps = avg(x => x.v_mmps); r.arcLenMM = avg(x => x.arcLenMM); r.workDeg = avg(x => x.workDeg);
      r.meas = { W: avg(x => x.meas.W), L: avg(x => x.meas.L), pen: avg(x => x.meas.pen) }; out.push(r); }
    return out;
  }
  function nelderMead(f, x0, { step = 0.1, iters = 200, tol = 1e-8 } = {}) {
    const n = x0.length; let S = [x0.slice()];
    for (let i = 0; i < n; i++) { const x = x0.slice(); x[i] += step; S.push(x); }
    let F = S.map(f);
    for (let it = 0; it < iters; it++) {
      const idx = F.map((v, i) => i).sort((a, b) => F[a] - F[b]); S = idx.map(i => S[i]); F = idx.map(i => F[i]);
      if (Math.abs(F[n] - F[0]) < tol) break;
      const c = new Array(n).fill(0); for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) c[j] += S[i][j] / n;
      const pt = (a, b, t) => a.map((v, j) => v + t * (b[j] - v));
      const xr = pt(c, S[n], -1), fr = f(xr);
      if (fr < F[0]) { const xe = pt(c, S[n], -2), fe = f(xe); if (fe < fr) { S[n] = xe; F[n] = fe; } else { S[n] = xr; F[n] = fr; } }
      else if (fr < F[n - 1]) { S[n] = xr; F[n] = fr; }
      else { const xc = pt(c, S[n], 0.5), fc = f(xc);
        if (fc < F[n]) { S[n] = xc; F[n] = fc; }
        else { for (let i = 1; i <= n; i++) { S[i] = pt(S[0], S[i], 0.5); F[i] = f(S[i]); } } }
    }
    const b = F.indexOf(Math.min(...F)); return { x: S[b], fx: F[b] };
  }
  /** Real2Sim calibration of the pool model's knobs to measured pools. */
  class Calibrator {
    constructor(model, o = {}) {
      this.model = model; this.params = o.params || ['effScale', 'kScale', 'sigScale'];
      this.weights = Object.assign({ W: 1.0, L: 0.5, pen: 0.7 }, o.weights || {});
      this.bounds = Object.assign({ effScale: [0.5, 1.5], kScale: [0.5, 2.0], sigScale: [0.4, 2.5] }, o.bounds || {});
    }
    predict(rec, calib) {
      const r = this.model.evaluate({ I: rec.I, V: rec.V, eta: rec.eta, T0: rec.T0, material: rec.material, v_mmps: rec.v_mmps,
        travelDeg: rec.travelDeg, workDeg: rec.workDeg, arcLenMM: rec.arcLenMM, position: rec.position, grav: 0.6, thicknessMM: 0, calib });
      return { W: r.widMM, L: r.lenMM, pen: r.penMM, ok: r.status !== 'no_pool' };
    }
    errors(recs, calib) {
      const acc = { W: [], L: [], pen: [] }; let loss = 0, n = 0;
      for (const rec of recs) { const p = this.predict(rec, calib);
        for (const ch of ['W', 'L', 'pen']) { const m = rec.meas[ch]; if (m == null || !(m > 0)) continue;
          const e = p.ok ? (p[ch] - m) : -m; acc[ch].push(e); loss += this.weights[ch] * (e / m) ** 2; n++; } }
      const rmse = {}; for (const ch in acc) rmse[ch] = acc[ch].length ? Math.sqrt(acc[ch].reduce((a, b) => a + b * b, 0) / acc[ch].length) : null;
      return { loss: n ? loss / n : Infinity, rmse, n };
    }
    fit(recs, { iters = 160, init = null } = {}) {
      const P = this.params, B = this.bounds;
      const toCal = (x) => { const c = {}; P.forEach((k, i) => { c[k] = clamp(Math.exp(x[i]), B[k][0], B[k][1]); }); return c; };
      const x0 = P.map(k => Math.log((init && init[k]) || 1));
      const before = this.errors(recs, toCal(P.map(() => 0)));
      const res = nelderMead((x) => this.errors(recs, toCal(x)).loss, x0, { step: 0.15, iters });
      const calib = toCal(res.x), after = this.errors(recs, calib);
      const preds = recs.map(r => ({ meas: r.meas, before: this.predict(r, {}), after: this.predict(r, calib) }));
      return { calib, before, after, preds, n: recs.length };
    }
  }
  /** Synthetic "measured" dataset from a hidden truth (demo / tests). */
  function makeSyntheticDataset(model, truth, n = 40, r = C.makeRng(7), noise = 0.03) {
    const recs = [];
    for (let i = 0; i < n; i++) {
      const rec = { I: 150 + 60 * r(), V: 22 + 5 * r(), eta: 0.8, T0: 20, material: 'steel', v_mmps: 3 + 12 * r(), arcLenMM: 1 + 4 * r(),
        workDeg: -10 + 20 * r(), travelDeg: -15, position: 'flat' };
      const t = model.evaluate(Object.assign({}, rec, { calib: truth, thicknessMM: 0 })); if (t.status === 'no_pool') continue;
      const g = () => 1 + noise * r.gauss();
      rec.meas = { W: t.widMM * g(), L: t.lenMM * g(), pen: t.penMM * g() }; recs.push(rec);
    }
    return recs;
  }

  /* =================================================== replay & residuals */
  class ReplayPool {
    constructor(rows) { this.meta = { name: 'ReplayPool', kind: 'replay' }; this.rows = rows; this.reset(); }
    reset() { this.i = 0; this._obs = C.emptyObs(); }
    setInput() {}
    step() { const r = this.rows[Math.min(this.i++, this.rows.length - 1)]; this._obs = r ? Object.assign({}, r.poolObs || metricsToObs(r)) : C.emptyObs(); return this._obs; }
    observe() { return this._obs; }
  }
  class ReplayRobot {
    constructor(rows) { this.meta = { name: 'ReplayRobot', kind: 'replay' }; this.rows = rows; this.reset(); }
    reset() { this.i = -1; }
    step() { this.i = Math.min(this.i + 1, this.rows.length - 1); }
    _r() { return this.rows[Math.max(this.i, 0)] || {}; }
    getWeldInput() { return Object.assign({}, this._r().weldInput || {}); }
    getState() { const r = this._r(); return Object.assign({ seamParam: 0, trackError: 0 }, r.robotState || {}, { weldInput: this.getWeldInput() }); }
    applyControl() {}   // replay is open-loop (counterfactual actions are ignored)
  }
  class ResidualMonitor {
    constructor(maxN = 300) { this.maxN = maxN; this.reset(); }
    reset() { this.series = []; this.sum = { W: 0, L: 0, area: 0 }; this.n = 0; }
    push(t, sim, real) {
      if (!sim || !real || sim.status !== 'ok' || real.status !== 'ok') return null;
      const e = { t, W: (real.W - sim.W) * 1000, L: (real.L - sim.L) * 1000, area: (real.area - sim.area) * 1e6,
        simW: sim.W * 1000, realW: real.W * 1000, simA: sim.area * 1e6, realA: real.area * 1e6 };
      this.series.push(e); if (this.series.length > this.maxN) this.series.shift();
      this.sum.W += e.W * e.W; this.sum.L += e.L * e.L; this.sum.area += e.area * e.area; this.n++; return e;
    }
    rmse() { const n = Math.max(this.n, 1); return { W: Math.sqrt(this.sum.W / n), L: Math.sqrt(this.sum.L / n), area: Math.sqrt(this.sum.area / n), n: this.n }; }
  }

  return { PROTOCOL, MSG, encode, decode, metricsToObs, obsToMetrics, LoopbackTransport, WebSocketTransport, BridgeClient,
    SafetyLimiter, RealRobotCell, RealPoolSensor, DomainRandomizer, RandomizedPool, MockRealBackend,
    parseMeasurements, thinRecords, nelderMead, Calibrator, makeSyntheticDataset, ReplayPool, ReplayRobot, ResidualMonitor };
});
