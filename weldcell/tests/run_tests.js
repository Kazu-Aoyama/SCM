#!/usr/bin/env node
/* Headless unit tests for the WeldCell core library (no dependencies).
 *   node tests/run_tests.js            */
'use strict';
const path = require('path');
const J = (f) => require(path.join(__dirname, '..', 'js', f));
const M = J('pool_model.js'), C = J('core.js'), L = J('learning.js'), S = J('sim2real.js');

const results = []; const tests = [];
const test = (name, fn) => tests.push({ name, fn });
const assert = (c, msg) => { if (!c) throw new Error(msg || 'assertion failed'); };
const near = (a, b, tol, msg) => assert(Math.abs(a - b) <= tol, `${msg || ''} expected ${b}±${tol}, got ${a}`);
const BASE = { I: 175, V: 24, eta: 0.8, T0: 20, material: 'steel', travelDeg: -15, position: 'flat', grav: 0.6, thicknessMM: 9 };

/* ---------------------------------------------------------------- physics */
test('pool model: pool shrinks with travel speed, grows with current', () => {
  const a = M.evaluate({ ...BASE, v_mmps: 4, workDeg: 0, arcLenMM: 2 });
  const b = M.evaluate({ ...BASE, v_mmps: 12, workDeg: 0, arcLenMM: 2 });
  const c = M.evaluate({ ...BASE, I: 250, v_mmps: 4, workDeg: 0, arcLenMM: 2 });
  assert(a.status === 'ok' && b.status === 'ok');
  assert(a.areaMM > b.areaMM, 'area decreases with speed');
  assert(c.areaMM > a.areaMM, 'area increases with current');
  assert(a.t85 > b.t85, 't8/5 decreases with speed');
});
test('pool model: v1 compatibility (solve == evaluate fields) and calibration knobs act', () => {
  const p = { ...BASE, v_mmps: 7, workDeg: 0, arcLenMM: 2 };
  const s = M.solve(p), e = M.evaluate(p);
  near(s.widMM, e.widMM, 1e-9);
  const hot = M.evaluate({ ...p, calib: { effScale: 1.2 } }), soft = M.evaluate({ ...p, calib: { kScale: 1.5 } });
  assert(hot.areaMM > e.areaMM, 'effScale>1 enlarges pool'); assert(soft.areaMM < e.areaMM, 'higher k shrinks pool');
});
test('pool model: burn-through flagged on thin plate', () => {
  const r = M.evaluate({ ...BASE, v_mmps: 3, workDeg: 0, arcLenMM: 2, thicknessMM: 3 });
  assert(r.status === 'burn_through', 'status ' + r.status);
});
test('surrogate table matches physics within 6% (area)', () => {
  const sm = new L.SurrogateModel(M, BASE);
  for (const v of [2.5, 5, 9, 15]) for (const w of [-10, 0, 12]) for (const a of [1.5, 3, 6]) {
    const t = M.evaluate({ ...BASE, v_mmps: v, workDeg: w, arcLenMM: a, thicknessMM: 0 }), s = sm.evaluate({ v_mmps: v, workDeg: w, arcLenMM: a });
    if (t.status !== 'ok') continue;
    assert(Math.abs(s.areaMM - t.areaMM) / t.areaMM < 0.06, `v=${v} w=${w} a=${a}: ${s.areaMM} vs ${t.areaMM}`);
  }
  const j = L.SurrogateModel.fromJSON(JSON.parse(JSON.stringify(sm.toJSON())));
  near(j.evaluate({ v_mmps: 7, workDeg: 0, arcLenMM: 2 }).areaMM, sm.evaluate({ v_mmps: 7, workDeg: 0, arcLenMM: 2 }).areaMM, 0.05, 'json roundtrip');
});

/* ------------------------------------------------------------------- core */
test('seeded RNG is reproducible', () => {
  const a = C.makeRng(99), b = C.makeRng(99); for (let i = 0; i < 10; i++) assert(a() === b());
});
test('applyAction: absolute + delta + limits', () => {
  const sp = { travelSpeed: 0.007, workRef: 0.785, workAngle: 0.785, travelAngle: 0, standoff: 0.002, lateralOffset: 0, wristRoll: 0 };
  C.applyAction(sp, { travelSpeed: 1 }); near(sp.travelSpeed, C.DEFAULT_LIMITS.travelSpeed[1], 1e-12);
  C.applyAction(sp, { dWorkAngle: 10 }); near(sp.workAngle - sp.workRef, 0.40, 1e-12);
  C.applyAction(sp, { dStandoff: 0.001, lateralOffset: 0.002 }); near(sp.standoff, 0.003, 1e-12); near(sp.lateralOffset, 0.002, 1e-12);
});
test('Coordinator + AutoHold: reward recorded, transitions carry (s,a,r,s′)', () => {
  const rng = C.makeRng(1);
  const robot = new C.SurrogateRobotCell({ rng }), pool = new C.ModelPool(M);
  const codec = new C.Codec('pool11', 'delta3'), rec = new C.TrajectoryRecorder(codec, { source: 'test' });
  const co = new C.Coordinator({ robot, pool, controller: new C.AutoHoldController(), recorder: rec });
  rec.start(); co.reset(); co.run(1 / 20, 120);
  const tr = rec.transitions();
  assert(tr.length === 119, 'transitions ' + tr.length);
  assert(tr.every(x => typeof x.reward === 'number'), 'reward on every transition');
  assert(tr.some(x => x.action && x.action.travelSpeed != null), 'controller actions recorded');
  assert(tr[5].obs.length === 11 && tr[5].next_obs.length === 11);
  const parsed = C.parseTrajectoryJSONL(rec.toJSONL());
  assert(parsed.header.schema === C.SCHEMA && parsed.rows.length === 119, 'jsonl roundtrip');
});
test('WeldCellEnv: effort penalty is applied (v21 bug fixed)', () => {
  const mk = () => new C.Coordinator({ robot: new C.SurrogateRobotCell({ rng: C.makeRng(3) }), pool: new C.ModelPool(M) });
  const e1 = new C.WeldCellEnv({ coordinator: mk(), codec: new C.Codec('compact3', 'delta3') });
  const e2 = new C.WeldCellEnv({ coordinator: mk(), codec: new C.Codec('compact3', 'delta3') });
  e1.reset(); e2.reset(); let r1 = 0, r2 = 0;
  for (let i = 0; i < 5; i++) { r1 += e1.step([0, 0.0, 0]).reward; r2 += e2.step([0, 1.0, 0]).reward; }   // same except work-angle effort
  assert(r2 < r1, `effort should reduce reward (${r2} < ${r1})`);
});
test('Codec presets: dims match the requirements (19 obs / 7 act)', () => {
  const c = new C.Codec('req19', 'req7'); assert(c.obsDim === 19 && c.actDim === 7);
  const rs = new C.SurrogateRobotCell({ rng: C.makeRng(2) }); rs.step(0.05);
  const o = c.encodeObs(rs.getState(), C.emptyObs()); assert(o.length === 19 && Array.from(o).every(Number.isFinite));
  const a = c.decodeAction([0, 0, 0, 0, 0, 0, 1]); near(a.travelSpeed, 0.015, 1e-12); near(a.dLateralOffset, 0, 1e-12);
  const back = c.encodeAction(a); near(back[6], 1, 1e-6);
});

/* --------------------------------------------------------------- learning */
test('CEM improves a linear policy over the zero policy (surrogate, seeded)', () => {
  const rng = C.makeRng(11); const sm = new L.SurrogateModel(M, BASE);
  const codec = new C.Codec('compact3', 'delta3');
  const mk = () => L.makeTrainingCell({ process: C.makeProcess(), surrogate: sm, rng, travelSpeed: 0.004 });   // start too slow: pool too big
  const evalTheta = (th) => L.evaluatePolicy(mk, new L.LinearPolicy(3, 3, th), codec, { steps: 200 });
  const zero = evalTheta(new Float64Array(12));
  const cem = new L.CEMTrainer({ dim: 12, evaluate: evalTheta, pop: 16, elite: 4, rng, episodes: 1 });
  for (let i = 0; i < 8; i++) cem.step();
  const best = evalTheta(cem.best.theta);
  assert(best >= zero - 1e-9, `CEM best ${best} >= zero ${zero}`);
  assert(cem.history.length === 8);
});
test('BC fits demonstrations of the AUTO controller', () => {
  const rng = C.makeRng(5); const codec = new C.Codec('compact3', 'delta3');
  const mk = () => L.makeTrainingCell({ process: C.makeProcess(), model: M, rng, travelSpeed: 0.005 });
  const pairs = L.collectDemos(mk, new C.AutoHoldController(), codec, { episodes: 2, steps: 200 });
  assert(pairs.length > 300, 'pairs ' + pairs.length);
  const { policy, mse } = L.fitBC(pairs, 3, 3);
  assert(Number.isFinite(mse) && mse < 0.5, 'bc mse ' + mse);
  assert(policy.act([0.2, 0, 0.5]).length === 3);
});

/* ---------------------------------------------------------------- sim2real */
test('Real2Sim: calibrator recovers hidden plant parameters', () => {
  const truth = { effScale: 0.9, kScale: 1.15, sigScale: 1.1 };
  const data = S.makeSyntheticDataset(M, truth, 40, C.makeRng(8), 0.01);
  const cal = new S.Calibrator(M).fit(data, { iters: 220 });
  assert(cal.after.loss < cal.before.loss * 0.2, `loss ${cal.after.loss} vs ${cal.before.loss}`);
  assert(cal.after.rmse.W < 0.25, 'width rmse mm ' + cal.after.rmse.W);
});
test('parseMeasurements: CSV aliases and recorder JSONL', () => {
  const csv = 'current_A,voltage_V,speed_mm_s,arc_length_mm,width_mm,length_mm\n180,25,6,2,7.1,11.2\n200,26,8,2.5,6.8,\n';
  const r = S.parseMeasurements(csv); assert(r.length === 2 && r[0].I === 180 && r[1].meas.L === null && r[0].v_mmps === 6);
  const rec = new C.TrajectoryRecorder(new C.Codec());
  const co = new C.Coordinator({ robot: new C.SurrogateRobotCell({ rng: C.makeRng(4) }), pool: new C.ModelPool(M, { tau: 0 }), recorder: rec });
  rec.start(); co.reset(); co.run(0.05, 30);
  const j = S.parseMeasurements(rec.toJSONL()); assert(j.length > 20 && j[0].meas.W > 0, 'jsonl records ' + j.length);
});
test('SafetyLimiter: disarmed/estop blocks, slew-rate limits, watchdog', () => {
  const lim = new S.SafetyLimiter({ watchdogMs: 100 });
  const sp = { travelSpeed: 0.007, workRef: 0.785, workAngle: 0.785, travelAngle: 0, standoff: 0.002, lateralOffset: 0, wristRoll: 0 };
  assert(lim.filter({ travelSpeed: 0.01 }, sp, 0.05) === null, 'disarmed');
  lim.arm(); const out = lim.filter({ travelSpeed: 0.02 }, sp, 0.05);
  near(out.travelSpeed, 0.007 + 0.004 * 0.05, 1e-12, 'slew limited');
  assert(lim.filter({ travelSpeed: 0.01 }, sp, 0.05, 500) === null, 'watchdog');
  lim.estop(); assert(lim.filter({ travelSpeed: 0.01 }, sp, 0.05) === null, 'estop');
});
test('Bridge loopback: policy-free closed loop on the virtual real machine', async () => {
  const [cli, srv] = S.LoopbackTransport.pair(2);
  const backend = new S.MockRealBackend(srv, { model: M });
  const client = new S.BridgeClient(cli);
  await new Promise(r => client.on('open', r));
  const pong = await client.ping(); assert(pong.type === 'pong');
  const cal = await client.request(S.MSG.GET_CALIBRATION, {}); assert(cal.data.frame === 'seam');
  for (let i = 0; i < 10; i++) backend.tick(0.05);
  await new Promise(r => setTimeout(r, 30));
  const robot = new S.RealRobotCell(client), sensor = new S.RealPoolSensor(client, { staleMs: 5000 });
  const obs = sensor.observe(); assert(obs.status === 'ok' && obs.W > 0, 'real obs ' + JSON.stringify(obs).slice(0, 80));
  robot.limiter.arm(); robot.step(0.05);
  const sent = robot.applyControl({ dTravelSpeed: 0.001 }); assert(sent && sent.travelSpeed > 0.007, 'command sent');
  await new Promise(r => setTimeout(r, 20)); for (let i = 0; i < 20; i++) backend.tick(0.05);
  assert(backend.robot.sp.travelSpeed > 0.007, 'plant followed command');
  await client.request(S.MSG.ESTOP, {}); assert(backend.estopped);
  backend.stop(); client.close();
});
test('Coordinator drives REAL adapters unchanged (same contract)', async () => {
  const [cli, srv] = S.LoopbackTransport.pair(0);
  const backend = new S.MockRealBackend(srv, { model: M }); const client = new S.BridgeClient(cli);
  await new Promise(r => client.on('open', r));
  const robot = new S.RealRobotCell(client); robot.limiter.arm();
  const co = new C.Coordinator({ robot, pool: new S.RealPoolSensor(client, { staleMs: 1e9 }), controller: new C.AutoHoldController() });
  co.reset(); await new Promise(r => setTimeout(r, 5));
  for (let i = 0; i < 40; i++) { backend.tick(0.05); await new Promise(r => setTimeout(r, 0)); co.step(0.05); }
  assert(robot.sent > 5, 'commands sent ' + robot.sent);
  client.close();
});
test('DomainRandomizer + ReplayPool + ResidualMonitor', () => {
  const rng = C.makeRng(21); const dr = new S.DomainRandomizer().prepare(M, BASE, L.SurrogateModel, 3, rng);
  assert(dr.variants.length === 3);
  const co = L.makeTrainingCell({ process: C.makeProcess(), surrogate: new L.SurrogateModel(M, BASE), randomizer: dr, rng });
  const st = C.rollout(co, 100, 0.05); assert(Number.isFinite(st.meanReward));
  const rows = [{ poolObs: { status: 'ok', W: 0.007, L: 0.01, area: 5e-5 } }];
  const rp = new S.ReplayPool(rows); assert(rp.step().W === 0.007);
  const mon = new S.ResidualMonitor(); mon.push(0, { status: 'ok', W: 0.007, L: 0.01, area: 5e-5 }, { status: 'ok', W: 0.008, L: 0.01, area: 6e-5 });
  near(mon.rmse().W, 1, 1e-9);
});

(async () => {
  let fail = 0;
  for (const t of tests) {
    const t0 = Date.now();
    try { await t.fn(); console.log(`  ✓ ${t.name}  (${Date.now() - t0} ms)`); }
    catch (e) { fail++; console.log(`  ✗ ${t.name}\n      ${e && e.stack ? e.stack.split('\n').slice(0, 3).join('\n      ') : e}`); }
  }
  console.log(`\n${tests.length - fail}/${tests.length} passed`);
  process.exit(fail ? 1 : 0);
})();
