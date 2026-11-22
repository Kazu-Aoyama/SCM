/* =============================================================================
 * panels.js — Learn (BC→CEM, evaluation, policy I/O, integration core),
 * Sim⇄Real (bridge, SHADOW/CONTROL, safety, excitation, Real2Sim calibration),
 * config I/O, stage/tab navigation, keyboard shortcuts, app init.
 * ========================================================================== */
'use strict';
const WC = WeldCell;

/* ================================================================ LEARN */
const RL = {
  policy: null, meta: null, targetArea: null, surrogate: null, busy: false, stop: false, history: [],
  makeCodec() { const c = new WC.Codec(this.meta ? this.meta.obs : CH.obsSpec, this.meta ? this.meta.act : CH.actSpec, { targetArea: this.targetArea });
    return c; },
};
function invalidateSurrogate() { RL.surrogate = null; }
function trainBase() {
  return { I: P.I, V: P.V, eta: P.eta, T0: P.T0, material: CH.material, travelDeg: P.travel, position: POSMAP[weldPos] || 'flat', grav: 0.6,
    thicknessMM: P.thick, calib: Object.assign({}, CALIB) };
}
/** desired pool area A* [m²] = model pool at the nominal (slider) conditions — fixed target for training & deployment */
function nominalArea() {
  const r = WeldPoolModel.evaluate(Object.assign(trainBase(), { v_mmps: P.speed, workDeg: P.work - WORKREF / D2R, arcLenMM: P.height, thicknessMM: 0 }));
  return r.status === 'no_pool' ? null : r.areaMM / 1e6;
}
function specNote() {
  const c = new WC.Codec(CH.obsSpec, CH.actSpec), el = $('specNote'); if (!el) return;
  el.innerHTML = `観測 ${c.obsDim} 次元: <span class="mono">${c.obsNames.join(', ')}</span><br>行動 ${c.actDim} 次元: <span class="mono">${c.actNames.join(', ')}</span><br>線形方策 a=tanh(W[o;1]) — パラメータ ${c.actDim * (c.obsDim + 1)} 個．`;
}
function makeTrainFactory(rng, A) {
  const base = trainBase(), useSur = CH.trainEnv === 'surrogate';
  if (useSur && !RL.surrogate) RL.surrogate = new WC.SurrogateModel(WeldPoolModel, base);
  let dr = null;
  if (CH.dr === 'on') {
    const s = P.drStr / 100, c = CALIB;
    dr = new WC.DomainRandomizer({ ranges: { effScale: [c.effScale * (1 - s), c.effScale * (1 + s)], kScale: [c.kScale * (1 - s), c.kScale * (1 + s)], sigScale: [c.sigScale * (1 - s), c.sigScale * (1 + s)] },
      obsNoise: P.drNoise / 100, delay: [0, P.drDelay] });
    if (useSur) dr.prepare(WeldPoolModel, base, WC.SurrogateModel, 6, rng);
    else { dr.prepare(WeldPoolModel, base, null, 6, rng); dr.variants = dr.samples.map(cal => ({ evaluate: p => WeldPoolModel.evaluate(Object.assign({}, p, { calib: cal })) })); }
  }
  const v0 = P.speed / 1000, h0 = P.height / 1000;
  return () => WC.makeTrainingCell({ process: processNow(), position: weldPos, travelAngle, workRef: WORKREF, rng, randomizer: dr,
    surrogate: useSur ? RL.surrogate : null, model: useSur ? null : { evaluate: p => WeldPoolModel.evaluate(Object.assign({}, p, { calib: CALIB })) },
    travelSpeed: v0 * (0.7 + 0.6 * rng()), standoff: clamp(h0 * (0.7 + 0.6 * rng()), 0.0005, 0.012), bow: 0.010,
    reward: { targetArea: A, wArea: P.wArea, wTrack: P.wTrack, wEffort: P.wEffort } });
}
function drawCurve(cv, series, opts = {}) {
  if (!cv) return; const dpr = devicePixelRatio || 1, w = cv.clientWidth || 300, h = cv.clientHeight || 90;
  cv.width = w * dpr; cv.height = h * dpr; const x = cv.getContext('2d'); x.setTransform(dpr, 0, 0, dpr, 0, 0); x.clearRect(0, 0, w, h);
  const lo = opts.lo ?? -1, hi = opts.hi ?? 1, Y = v => h - 14 - (clamp(v, lo, hi) - lo) / (hi - lo) * (h - 28);
  x.strokeStyle = '#1e2a36'; x.lineWidth = 1; x.font = '9px IBM Plex Mono'; x.fillStyle = '#566372';
  for (const g of [lo, (lo + hi) / 2, hi]) { x.beginPath(); x.moveTo(26, Y(g)); x.lineTo(w, Y(g)); x.stroke(); x.fillText(g.toFixed(1), 2, Y(g) + 3); }
  series.forEach(s => { if (!s.data.length) return; x.strokeStyle = s.color; x.lineWidth = s.w || 2; x.setLineDash(s.dash || []); x.beginPath();
    s.data.forEach((v, i) => { const xx = 26 + (s.data.length > 1 ? i / (s.data.length - 1) : 0) * (w - 30); i ? x.lineTo(xx, Y(v)) : x.moveTo(xx, Y(v)); }); x.stroke(); x.setLineDash([]); });
  x.fillStyle = '#7e8c9a'; x.font = '10px IBM Plex Mono'; x.fillText(opts.label || '', 30, 11);
  let lx = w - 8; x.textAlign = 'right'; series.slice().reverse().forEach(s => { x.fillStyle = s.color; x.fillText(s.name || '', lx, 11); lx -= x.measureText(s.name || '').width + 12; }); x.textAlign = 'left';
}
function drawRL() {
  const H = RL.history, a = { data: H.map(h => h.best), color: '#46d39a', name: 'best' }, b = { data: H.map(h => h.eliteMean), color: '#ffb13b', name: 'elite mean', w: 1.4 };
  const ref = RL.autoRef != null ? { data: H.map(() => RL.autoRef), color: '#6fb8ff', name: 'AUTO', dash: [4, 3], w: 1.2 } : null;
  const ser = [a, b].concat(ref ? [ref] : []);
  drawCurve($('rlCurve'), ser, { label: 'reward / iter' }); drawCurve($('rlCurveBig'), ser, { label: 'CEM reward / iteration' });
}
async function train() {
  if (RL.busy) return; RL.busy = true; RL.stop = false; RL.history = [];
  const out = $('rlOut'); out.className = 'out'; const t0 = performance.now();
  try {
    const rng = WC.makeRng(P.seed), codec = new WC.Codec(CH.obsSpec, CH.actSpec), A = nominalArea();
    if (!A) throw new Error('公称条件で溶融池が形成されません（電流・速度を見直してください）');
    codec.fixedTarget = A; codec.reset();
    const mk = makeTrainFactory(rng, A), dim = codec.actDim * (codec.obsDim + 1), ep = { steps: P.epSteps, dt: 0.05 };
    const evalTheta = th => WC.evaluatePolicy(mk, new WC.LinearPolicy(codec.obsDim, codec.actDim, th), codec, ep);
    out.textContent = `準備: A*=${(A * 1e6).toFixed(1)} mm² · ${CH.trainEnv === 'surrogate' ? 'サロゲート表 ' + (RL.surrogate ? RL.surrogate.builtSolves + ' 点' : '') : '物理モデル直'} · DR ${CH.dr}`;
    // reference: classic AUTO through the contract
    const autoEval = () => { const co = mk(); co.setController(new WC.AutoHoldController({ targetArea: A })); return WC.rollout(co, P.epSteps, 0.05).meanReward; };
    RL.autoRef = (autoEval() + autoEval()) / 2;
    let mean0 = null;
    if (CH.bcWarm === 'on') {
      await tick();
      const pairs = WC.collectDemos(mk, new WC.AutoHoldController({ targetArea: A }), codec, { episodes: 4, steps: P.epSteps });
      const bc = WC.fitBC(pairs, codec.obsDim, codec.actDim); mean0 = bc.policy.theta;
      RL.bcPolicy = bc.policy; log(`BC: ${bc.n} サンプルから初期方策 (mse ${bc.mse.toFixed(3)})`);
    } else RL.bcPolicy = null;
    const cem = new WC.CEMTrainer({ dim, evaluate: evalTheta, pop: P.pop, elite: Math.min(P.elite, P.pop - 1), rng, mean0,
      sigma0: (mean0 ? 0.3 : 0.6) * Math.min(1, Math.sqrt(12 / dim)), sigmaMin: dim > 30 ? 0.01 : 0.04, episodes: 2 });
    for (let i = 0; i < P.iters && !RL.stop; i++) {
      const r = cem.step(); RL.history.push(r); drawRL();
      out.textContent = `iter ${r.iter}/${P.iters} · best ${r.best.toFixed(3)} · elite ${r.eliteMean.toFixed(3)} · AUTO ${RL.autoRef.toFixed(3)} · ${((performance.now() - t0) / 1000).toFixed(1)} s`;
      await tick();
    }
    // model selection on fresh validation episodes: the single best CEM sample can be a lucky draw
    // (esp. 140-param req19/req7 policies), so compare it with the CEM mean and the BC start.
    const vRng = WC.makeRng(P.seed + 500), vMk = makeTrainFactory(vRng, A), cands = [['CEM best', cem.best.theta], ['CEM mean', cem.mean]];
    if (mean0) cands.push(['BC init', mean0]);
    let pick = null;
    for (const [name, th] of cands) { const sc = WC.evaluatePolicy(vMk, new WC.LinearPolicy(codec.obsDim, codec.actDim, th), codec, { steps: P.epSteps, dt: 0.05, episodes: 4 });
      if (!pick || sc > pick.sc) pick = { name, th, sc }; }
    log(`方策選択（検証 4 ep）: ${pick.name} (${pick.sc.toFixed(3)})`);
    RL.policy = new WC.LinearPolicy(codec.obsDim, codec.actDim, pick.th);
    RL.meta = { obs: CH.obsSpec, act: CH.actSpec, iters: cem.iter, seed: P.seed, trainEnv: CH.trainEnv, dr: CH.dr === 'on' ? { str: P.drStr, noise: P.drNoise, delay: P.drDelay } : null,
      bc: CH.bcWarm === 'on', time: new Date().toISOString(), bestTrain: cem.best.score, selected: pick.name, validation: pick.sc };
    RL.targetArea = A;
    out.className = 'out ok'; out.textContent = `学習完了 ${cem.iter} iter · 採用 ${pick.name}（検証 ${pick.sc.toFixed(3)}）· best ${cem.best.score.toFixed(3)}（AUTO ${RL.autoRef.toFixed(3)}）· ${((performance.now() - t0) / 1000).toFixed(1)} s\n→ 評価結果は ダッシュボード「学習」タブ · Deploy で配備`;
    log('学習完了: best ' + cem.best.score.toFixed(3), 'ok');
    await tick(); evaluatePolicies(mk, codec, A);
  } catch (e) { out.className = 'out err'; out.textContent = '学習エラー: ' + e.message; log('学習エラー: ' + e.message, 'err'); console.error(e); }
  RL.busy = false;
}
const tick = () => new Promise(r => setTimeout(r, 0));
function evaluatePolicies(mkTrain, codec, A) {
  const rows = [], N = 6, run = (name, ctlFn) => {
    const rng = WC.makeRng(P.seed + 1000), mk = makeTrainFactory(rng, A); const acc = { meanReward: 0, areaErr: 0, areaCV: 0, trackRMS: 0 };
    for (let e = 0; e < N; e++) { const co = mk(); const c = ctlFn(); co.setController(c); const r = WC.rollout(co, P.epSteps, 0.05); for (const k in acc) acc[k] += (r[k] || 0) / N; }
    rows.push(Object.assign({ name }, acc));
  };
  run('制御なし', () => null);
  run('AUTO（契約版）', () => new WC.AutoHoldController({ targetArea: A }));
  if (RL.bcPolicy) run('BC（模倣のみ）', () => new WC.PolicyController(RL.bcPolicy, new WC.Codec(codec.obsName, codec.actName, { targetArea: A })));
  run('RL 方策（BC→CEM）', () => new WC.PolicyController(RL.policy, new WC.Codec(codec.obsName, codec.actName, { targetArea: A })));
  const best = k => (k === 'meanReward' ? Math.max : Math.min)(...rows.map(r => r[k]));
  const td = (r, k, f) => `<td class="${Math.abs(r[k] - best(k)) < 1e-12 ? 'best' : ''}">${f(r[k])}</td>`;
  $('evalBox').innerHTML = `<table class="res"><tr><th>コントローラ</th><th>平均報酬 ↑</th><th>面積誤差 ↓</th><th>面積変動 CV ↓</th><th>倣い RMS ↓</th></tr>` +
    rows.map(r => `<tr><td>${r.name}</td>${td(r, 'meanReward', v => v.toFixed(3))}${td(r, 'areaErr', v => (v * 100).toFixed(1) + '%')}${td(r, 'areaCV', v => (v * 100).toFixed(1) + '%')}${td(r, 'trackRMS', v => (v * 1000).toFixed(2) + ' mm')}</tr>`).join('') +
    `</table><div class="note" style="margin-top:8px">評価: 学習とは別シード（${P.seed + 1000}）の ${N} エピソード · ${CH.dr === 'on' ? 'ドメインランダム化 ON（物理ばらつき ±' + P.drStr + '%）' : 'DR OFF'} · 目標面積 A*=${(A * 1e6).toFixed(1)} mm²</div>`;
  RL.evalRows = rows;
}
function deployToggle() {
  if (!RL.policy) { toast('先に Train してください', 'warn'); return; }
  setChoice('ctlMode', CH.ctlMode === 'policy' ? 'auto3' : 'policy');
}
function exportPolicy() {
  if (!RL.policy) { toast('方策がありません', 'warn'); return; }
  const codec = RL.makeCodec();
  const j = RL.policy.toJSON(codec, { app: 'WeldCell', version: APP_VERSION, targetArea: RL.targetArea, reward: { wArea: P.wArea, wTrack: P.wTrack, wEffort: P.wEffort, trackScale: 0.003 },
    process: processNow(), calib: CALIB, training: Object.assign({}, RL.meta, { history: RL.history }), control_dt: 0.05,
    limits: WC.DEFAULT_LIMITS, note: 'obs: encode with codec (normalised to [-1,1] unless obsRaw); act: decode [-1,1] -> actLo..actHi; d* = delta per control tick' });
  downloadText(`weldcell_policy_${CH.obsSpec}_${CH.actSpec}.json`, JSON.stringify(j, null, 1)); log('方策を書き出しました', 'ok');
}
async function importPolicy() {
  try { const f = await readFile($('polFile')); const j = JSON.parse(f.text);
    RL.policy = WC.LinearPolicy.fromJSON(j); RL.targetArea = j.targetArea || nominalArea();
    RL.meta = Object.assign({ obs: j.codec.obs, act: j.codec.act }, j.training || {});
    setChoice('obsSpec', j.codec.obs, { silent: true }); setChoice('actSpec', j.codec.act, { silent: true }); specNote();
    $('rlOut').className = 'out ok'; $('rlOut').textContent = `方策を読込: ${f.name} (${j.codec.obs}/${j.codec.act}, ${RL.policy.dim} params)`; log('方策を読込: ' + f.name, 'ok');
  } catch (e) { toast('方策の読込に失敗: ' + e.message, 'err'); }
}

/* ------------------------------------------------- integration core panel */
let coreRec = null;
function coreRollout() {
  const A = nominalArea(), rng = WC.makeRng(P.seed + 7), codec = new WC.Codec(CH.obsSpec, CH.actSpec, { targetArea: A });
  const co = makeTrainFactory(rng, A)();
  coreRec = new WC.TrajectoryRecorder(codec, { source: 'core-rollout', controller: CH.coreCtl, seed: P.seed + 7 }); co.setRecorder(coreRec);
  const ctl = CH.coreCtl === 'policy' && RL.policy ? new WC.PolicyController(RL.policy, RL.makeCodec()) : new WC.AutoHoldController({ targetArea: A });
  if (CH.coreCtl === 'policy' && !RL.policy) toast('RL 方策が未学習のため AUTO で実行', 'warn');
  co.setController(ctl); coreRec.start();
  const rs = []; co.reset(); for (let i = 0; i < P.coreSteps; i++) { const f = co.step(0.05); rs.push(f.reward); if (f.robotState.seamParam >= 1) break; }
  coreRec.stop();
  drawCurve($('coreCurve'), [{ data: rs, color: '#c792ff', name: ctl.meta.name, w: 1.4 }], { label: 'reward / step' });
  const mean = rs.reduce((a, b) => a + b, 0) / Math.max(rs.length, 1);
  $('coreOut').textContent = `core v${WC.VERSION} · Coordinator ${rs.length} steps · ${ctl.meta.name} · 平均報酬 ${mean.toFixed(3)} · 遷移 ${coreRec.transitions().length}`;
}
function coreEnvEval() {
  const A = nominalArea(), rng = WC.makeRng(P.seed + 9), co = makeTrainFactory(rng, A)();
  const codec = RL.policy ? RL.makeCodec() : new WC.Codec(CH.obsSpec, CH.actSpec, { targetArea: A });
  const env = new WC.WeldCellEnv({ coordinator: co, codec, dt: 0.05, maxSteps: P.coreSteps, targetArea: A });
  const pol = CH.coreCtl === 'policy' && RL.policy ? RL.policy : null;
  let o = env.reset(), done = false, tot = 0, k = 0; const curve = [];
  while (!done) { const a = pol ? pol.act(o) : codec.randomAction(rng); const r = env.step(a); o = r.obs; done = r.done; tot += r.reward; curve.push(r.reward); k++; }
  drawCurve($('coreCurve'), [{ data: curve, color: '#6fb8ff', name: pol ? 'policy' : 'random', w: 1.2 }], { label: 'WeldCellEnv reward / step' });
  $('coreOut').textContent = `WeldCellEnv · obs ${codec.obsDim} / act ${codec.actDim} · ${k} steps · ${pol ? 'RL 方策' : 'random'} · 累積 ${tot.toFixed(1)} · 平均 ${(tot / Math.max(k, 1)).toFixed(3)}`;
}

/* ============================================================ SIM ⇄ REAL */
const S2R = { client: null, backend: null, limiter: null, co: null, timer: null, connected: false, armed: false,
  realRec: new WC.TrajectoryRecorder(null, { source: 'real' }), residual: new WC.ResidualMonitor(), excite: null, lastSent: null, lastCal: null };
function syncSafety() {
  if (!S2R.limiter) return; const L = S2R.limiter;
  L.rate.travelSpeed = P.rateV / 1000; L.rate.workAngle = L.rate.travelAngle = P.rateAng * D2R; L.watchdogMs = P.watchdog;
}
function s2rStatus() {
  const c = S2R.client, el = $('connOut'), b = $('connBadge'), dot = $('s2rDot');
  if (!c || !S2R.connected) { el.textContent = '未接続'; el.className = 'out'; if (b) b.textContent = 'offline'; dot.classList.add('hide'); return; }
  const h = c.latest.hello || {}; el.className = 'out ok';
  el.textContent = `接続中: ${h.backend || '?'} · ${h.protocol || WC.PROTOCOL}\nrx ${c.stats.rx} · tx ${c.stats.tx} · rtt ${Number.isFinite(c.stats.rttMs) ? c.stats.rttMs.toFixed(1) + ' ms' : '—'} · 最終受信 ${c.age().toFixed(0)} ms 前\n実機ログ ${S2R.realRec.frames.length} frames`;
  if (b) b.textContent = 'online'; dot.classList.remove('hide');
}
function setMode(m) {
  if (m !== 'sim' && !S2R.connected) { toast('先に接続してください', 'warn'); setChoice('opMode', 'sim', { silent: true }); m = 'sim'; }
  CH.opMode = m; const b = $('modeBadge'); b.textContent = m.toUpperCase(); b.className = 'modebadge ' + m;
  if (m !== 'control') disarm();
  if (S2R.co) S2R.co.setController(m === 'control' ? s2rController() : null);
  $('opNote2').innerHTML = { sim: '<b>SIM</b>: 実機は参照しません．', shadow: '<b>SHADOW</b>: 実機の設定値をライブ・シミュレータに写し（デジタルツイン），実機計測とシミュレーション予測の残差を監視します．実機へは指令しません．',
    control: '<b>CONTROL</b>: 選択中の溶融池制御（AUTO 契約版 / RL 方策）が<b>実機の観測</b>で動作し，SafetyLimiter 経由で実機へ指令します．ARM で送信開始．' }[m];
  log('運用モード: ' + m.toUpperCase(), m === 'control' ? 'real' : '');
}
function s2rController() {
  if (CH.ctlMode === 'policy' && RL.policy) return new WC.PolicyController(RL.policy, RL.makeCodec());
  return new WC.AutoHoldController({ targetArea: RL.targetArea || nominalArea() });
}
function sendProcess() { if (S2R.client) S2R.client.send(WC.MSG.SET_PROCESS, { I: P.I, V: P.V, eta: P.eta, material: CH.material, T0: P.T0, thickness: P.thick / 1000 }); }
async function connect() {
  disconnect();
  try {
    let tr;
    if (CH.backend === 'mock') {
      const [cli, srv] = WC.LoopbackTransport.pair(P.latency);
      S2R.backend = new WC.MockRealBackend(srv, { model: WeldPoolModel, seed: P.seed + 77, process: { I: P.I, V: P.V, eta: P.eta, material: CH.material, T0: P.T0, thickness: P.thick / 1000 } });
      tr = cli;
    } else tr = new WC.WebSocketTransport($('wsUrl').value.trim());
    const c = new WC.BridgeClient(tr); S2R.client = c;
    c.on('close', () => { S2R.connected = false; s2rStatus(); setChoice('opMode', 'sim'); log('ブリッジ切断', 'warn'); });
    c.on('error', e => log('ブリッジエラー: ' + (e && e.message || e), 'err'));
    c.on('metrics', m => onRealMetrics(m.data || {}));
    await new Promise((res, rej) => { c.on('open', res); setTimeout(() => rej(new Error('接続タイムアウト（ブリッジ起動・URL を確認）')), 3000); });
    S2R.connected = true;
    if (S2R.backend) S2R.backend.start(P.rateHz);
    sendProcess(); c.send(WC.MSG.RESET_EPISODE, { scenario_id: 'default' });
    S2R.limiter = new WC.SafetyLimiter(); syncSafety();
    S2R.co = new WC.Coordinator({ robot: new WC.RealRobotCell(c, { limiter: S2R.limiter, workRef: 45 * D2R }), pool: new WC.RealPoolSensor(c, { staleMs: P.watchdog }),
      reward: new WC.PoolHoldReward({ targetArea: RL.targetArea || nominalArea(), wArea: P.wArea, wTrack: P.wTrack, wEffort: P.wEffort }) });
    S2R.timer = setInterval(s2rTick, 1000 / P.rateHz);
    S2R.residual.reset(); S2R.realRec.clear(); S2R.realRec.start();
    toast('接続: ' + (CH.backend === 'mock' ? '仮想実機（ループバック, 遅延 ' + P.latency + ' ms）' : $('wsUrl').value), 'real');
    if (CH.opMode === 'sim') setChoice('opMode', 'shadow');
  } catch (e) { toast('接続失敗: ' + e.message, 'err'); disconnect(); }
  s2rStatus();
}
function disconnect() {
  if (S2R.timer) clearInterval(S2R.timer); S2R.timer = null;
  if (S2R.backend) S2R.backend.stop(); S2R.backend = null;
  if (S2R.client) { try { S2R.client.close(); } catch (e) { /* ignore */ } } S2R.client = null; S2R.connected = false; S2R.co = null; S2R.excite = null;
  disarm(); s2rStatus();
}
function arm() { if (CH.opMode !== 'control') { toast('CONTROL モードで ARM できます', 'warn'); return; } if (!S2R.limiter) return;
  S2R.limiter.arm(true); S2R.armed = true; if (S2R.co) { S2R.co.setController(s2rController()); } $('twSafe').textContent = 'ARMED'; $('twSafe').className = 'v bad'; log('ARM: 実機への指令を許可', 'real'); }
function disarm() { if (S2R.limiter) S2R.limiter.arm(false); S2R.armed = false; const el = $('twSafe'); if (el) { el.textContent = 'disarmed'; el.className = 'v ok'; } }
function estop() {
  if (S2R.limiter) S2R.limiter.estop(); S2R.armed = false; S2R.excite = null;
  if (S2R.client && S2R.connected) S2R.client.send(WC.MSG.ESTOP, {});
  opStop(); const el = $('twSafe'); if (el) { el.textContent = 'E-STOP'; el.className = 'v bad'; } toast('E-STOP', 'err');
}
/** control tick at rateHz: CONTROL → the same Coordinator/controller as the sim, with REAL adapters */
function s2rTick() {
  if (!S2R.connected || !S2R.co) return; const dt = 1 / P.rateHz;
  if (CH.opMode === 'control' && S2R.armed) {
    if (S2R.excite) exciteStep(dt);
    else { const f = S2R.co.step(dt); S2R.lastSent = f.action; }
  }
}
function onRealMetrics(m) {
  const c = S2R.client, st = c.latest.state; if (!st) return;
  const obs = WC.metricsToObs(m), wi = Object.assign({}, st.weldInput || {});
  // digital twin prediction at the REAL setpoints with the current calibration
  const pr = WeldPoolModel.evaluate(WC.ModelPool.paramsFrom(wi, CALIB));
  const sim = { status: pr.status, W: pr.widMM / 1000, L: pr.lenMM / 1000, area: pr.areaMM / 1e6 };
  const e = S2R.residual.push(m.t || 0, sim, obs); S2R.lastPair = { sim, obs, e };
  const r = S2R.co && S2R.co.reward ? S2R.co.reward.compute(st, obs, S2R.lastSent) : 0;
  S2R.realRec.push({ t: m.t || 0, step: S2R.realRec.frames.length, weldInput: wi, poolObs: obs, robotState: st, action: S2R.lastSent, reward: r });
}
/** SHADOW/CONTROL: the live 3-D cell mirrors the real setpoints (digital twin view) */
function s2rBeforeFrame() {
  if (!S2R.connected || CH.opMode === 'sim' || !S2R.client) return;
  const st = S2R.client.latest.state; if (!st || !st.weldInput) return; const wi = st.weldInput;
  applyLiveAction({ travelSpeed: wi.travelSpeed, workAngle: WORKREF + (wi.workDev || 0), travelAngle: wi.travelAngle, standoff: wi.standoff }, { auto: true });
}
function s2rControlsLive() { return S2R.connected && CH.opMode !== 'sim'; }
let s2rUiAcc = 0;
function s2rAfterFrame(dt) {
  s2rUiAcc += dt; if (s2rUiAcc < 0.25) return; s2rUiAcc = 0;
  if (!S2R.connected) return; s2rStatus();
  const p = S2R.lastPair, rm = S2R.residual.rmse();
  if (p && p.obs.status === 'ok') $('twW').textContent = `${(p.obs.W * 1000).toFixed(2)} / ${(p.sim.W * 1000).toFixed(2)} mm`;
  $('twR').textContent = rm.n ? `${rm.W.toFixed(2)} mm (n=${rm.n})` : '—';
  $('twRtt').textContent = `${Number.isFinite(S2R.client.stats.rttMs) ? S2R.client.stats.rttMs.toFixed(0) : '—'} ms / ${S2R.client.stats.rx}`;
  if (S2R.limiter && S2R.limiter.estopped) { $('twSafe').textContent = 'E-STOP'; $('twSafe').className = 'v bad'; }
  const cv = $('resChart'); if (cv && cv.offsetParent) { const ser = S2R.residual.series;
    drawCurve(cv, [{ data: ser.map(x => x.realW), color: '#c792ff', name: '実機 W' }, { data: ser.map(x => x.simW), color: '#46d39a', name: 'シミュ予測 W', w: 1.5 }],
      { lo: Math.min(...ser.map(x => Math.min(x.realW, x.simW)), 99) - 0.5, hi: Math.max(...ser.map(x => Math.max(x.realW, x.simW)), 0) + 0.5, label: 'W [mm]' });
    $('resLeg').textContent = `RMSE  W ${rm.W.toFixed(2)} mm · L ${rm.L.toFixed(2)} mm · 面積 ${rm.area.toFixed(1)} mm² · n=${rm.n} · 校正 effScale ${CALIB.effScale.toFixed(3)} / kScale ${CALIB.kScale.toFixed(3)} / sigScale ${CALIB.sigScale.toFixed(3)}`; }
}
/* -- Real2Sim: excitation (system identification), calibration */
function startExcitation() {
  if (!S2R.connected || CH.opMode !== 'control' || !S2R.armed) { toast('励起: 接続 → CONTROL → ARM が必要です', 'warn'); return; }
  const rng = WC.makeRng(P.seed + 31), pts = [];
  for (const v of [0.004, 0.007, 0.011]) for (const h of [0.0015, 0.003, 0.0045]) for (const w of [-8, 0, 8]) pts.push({ travelSpeed: v, standoff: h, workAngle: 45 * D2R + w * D2R });
  for (let i = pts.length - 1; i > 0; i--) { const j = Math.floor(rng() * (i + 1)); [pts[i], pts[j]] = [pts[j], pts[i]]; }
  S2R.excite = { pts, i: 0, t: 0, hold: P.exciteT / pts.length };
  S2R.client.send(WC.MSG.RESET_EPISODE, { scenario_id: 'excitation' }); S2R.realRec.clear(); S2R.realRec.start();
  log(`励起シーケンス開始: ${pts.length} 点 × ${S2R.excite.hold.toFixed(1)} s`, 'real');
}
function exciteStep(dt) {
  const E = S2R.excite; E.t += dt;
  if (E.t >= E.hold) { E.t = 0; E.i++; if (E.i >= E.pts.length) { S2R.excite = null; toast(`励起完了: ${S2R.realRec.frames.length} frames → 「実機ログで校正」`, 'ok'); return; } }
  const cmd = S2R.co.robot.applyControl(E.pts[E.i]); S2R.lastSent = cmd;
}
function runCalibration(recs, label) {
  if (!recs.length) { toast('校正用データがありません（status ok の計測が必要）', 'warn'); return; }
  const thin = WC.thinRecords(recs, 160), t0 = performance.now();
  const res = new WC.Calibrator(WeldPoolModel).fit(thin, { iters: 200 });
  S2R.lastCal = res; const c = res.calib;
  const f = (e) => `W ${e.rmse.W != null ? e.rmse.W.toFixed(2) : '—'} · L ${e.rmse.L != null ? e.rmse.L.toFixed(2) : '—'} · pen ${e.rmse.pen != null ? e.rmse.pen.toFixed(2) : '—'} mm`;
  $('calOut').className = 'out ok';
  $('calOut').textContent = `${label}: ${res.n} 点 (${((performance.now() - t0) / 1000).toFixed(1)} s)\n推定: effScale ${c.effScale.toFixed(3)} · kScale ${c.kScale.toFixed(3)} · sigScale ${c.sigScale.toFixed(3)}\nRMSE 校正前 ${f(res.before)}\nRMSE 校正後 ${f(res.after)}\n→「校正値をシミュレータへ適用」`;
  $('calBadge').textContent = 'fit ready'; drawParity(res); log(`Real2Sim 校正 (${label}): effScale ${c.effScale.toFixed(3)} kScale ${c.kScale.toFixed(3)} sigScale ${c.sigScale.toFixed(3)}`, 'ok');
  activateTab('s2r');
  return res;
}
function drawParity(res) {
  const cv = $('parityChart'); if (!cv) return; const c = _fit(cv); if (!c) return; const { ctx, w, h } = c; ctx.clearRect(0, 0, w, h);
  const pts = res.preds.filter(p => p.meas.W > 0), vals = pts.flatMap(p => [p.meas.W, p.before.W, p.after.W]).filter(Number.isFinite);
  const lo = Math.min(...vals) * 0.95, hi = Math.max(...vals) * 1.05, pad = 30, X = v => pad + (v - lo) / (hi - lo) * (w - pad - 10), Y = v => h - pad + 8 - (v - lo) / (hi - lo) * (h - pad - 6);
  ctx.strokeStyle = 'rgba(255,255,255,.25)'; ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(X(lo), Y(lo)); ctx.lineTo(X(hi), Y(hi)); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = '#566372'; ctx.font = '9px IBM Plex Mono'; ctx.fillText(lo.toFixed(1), X(lo), h - 8); ctx.fillText(hi.toFixed(1), X(hi) - 20, h - 8); ctx.fillText('計測 W [mm] →', w / 2 - 30, h - 8);
  ctx.save(); ctx.translate(10, h / 2 + 30); ctx.rotate(-Math.PI / 2); ctx.fillText('予測 W [mm] →', 0, 0); ctx.restore();
  for (const p of pts) { ctx.fillStyle = 'rgba(255,111,99,.55)'; ctx.beginPath(); ctx.arc(X(p.meas.W), Y(p.before.W), 2.6, 0, 7); ctx.fill();
    ctx.fillStyle = 'rgba(70,211,154,.9)'; ctx.beginPath(); ctx.arc(X(p.meas.W), Y(p.after.W), 2.6, 0, 7); ctx.fill(); }
  $('parityLeg').innerHTML = `<span style="color:#ff6f63">● 校正前</span> RMSE ${res.before.rmse.W.toFixed(2)} mm · <span style="color:#46d39a">● 校正後</span> RMSE ${res.after.rmse.W.toFixed(2)} mm · n=${res.n}`;
}
function applyCalibration() {
  if (!S2R.lastCal) { toast('先に校正を実行してください', 'warn'); return; }
  CALIB = Object.assign({}, S2R.lastCal.calib); invalidateSurrogate(); poolEval = null; S2R.residual.reset();
  $('calBadge').textContent = 'applied'; toast(`校正値を適用: effScale ${CALIB.effScale.toFixed(3)} · kScale ${CALIB.kScale.toFixed(3)} · sigScale ${CALIB.sigScale.toFixed(3)}（学習も校正済みモデルで行われます）`, 'ok');
}

/* ============================================================== nav / init */
function setStage(n) {
  n = clamp(+n || 1, 1, 5); document.body.dataset.stage = n;
  document.querySelectorAll('.step').forEach(b => b.classList.toggle('on', +b.dataset.s === n));
  $('inspN').textContent = n; $('inspTitle').textContent = STAGES[n].title; $('inspDesc').textContent = STAGES[n].desc;
  $('inspN').style.background = n === 5 ? 'var(--real)' : 'var(--acc)';
  refreshVisibility(); safeStore.set('weldcell.stage', String(n));
  if (n === 2 && !weldEval) runWeldEval();
  if (n === 4) { activateTab('learn'); specNote(); }
  if (n === 5) activateTab('s2r');
  if (n === 3) activateTab('tele');
  if (location.hash !== '#stage=' + n) history.replaceState(null, '', '#stage=' + n);
}
function activateTab(p) {
  document.querySelectorAll('.tab').forEach(t => t.classList.toggle('on', t.dataset.p === p));
  document.querySelectorAll('.pane').forEach(x => x.classList.toggle('on', x.id === 'pane-' + p)); safeStore.set('weldcell.tab', p);
}
function applyHashStage() { const m = (location.hash || '').match(/stage=(\d)/); if (m) setStage(+m[1]); }
function on(id, ev, fn) { const el = $(id); if (el) el.addEventListener(ev, fn); }
function wire() {
  document.querySelectorAll('.step').forEach(b => b.addEventListener('click', () => setStage(+b.dataset.s)));
  document.querySelectorAll('.tab').forEach(t => t.addEventListener('click', () => activateTab(t.dataset.p)));
  window.addEventListener('hashchange', applyHashStage);
  on('homeBtn', 'click', opHome); on('approachBtn', 'click', opApproach); on('dryBtn', 'click', opDry); on('weldBtn', 'click', opWeld); on('stopBtn', 'click', opStop);
  on('recBtn', 'click', () => toggleRec());
  on('recExport', 'click', () => { if (!liveRec.frames.length) { toast('記録がありません（REC → 溶接）', 'warn'); return; }
    downloadText('weldcell_live.jsonl', liveRec.toJSONL({ config: exportConfig(), calib: CALIB }), 'application/x-ndjson'); });
  on('recClear', 'click', () => { liveRec.clear(); log('記録をクリア'); });
  on('clearBead', 'click', clearBeads);
  on('reseedBtn', 'click', () => { R = WC.makeRng(P.seed); WC.setSeed(P.seed); toast('乱数シードを再適用: ' + P.seed); });
  on('homeSetBtn', 'click', () => { Q_HOME = q.slice(); log('現在姿勢を Home に設定'); });
  on('homeDefBtn', 'click', () => { Q_HOME = Q_HOME_DEF.slice(); log('Home を既定に戻しました'); });
  on('evalBtn', 'click', runWeldEval); on('autoPlaceBtn', 'click', autoPlaceBase);
  on('trainBtn', 'click', train); on('trainStop', 'click', () => { RL.stop = true; }); on('deployBtn', 'click', deployToggle);
  on('polExport', 'click', exportPolicy); on('polImport', 'click', () => $('polFile').click()); on('polFile', 'change', importPolicy);
  on('coreRunBtn', 'click', coreRollout); on('coreEnvBtn', 'click', coreEnvEval);
  on('coreExportBtn', 'click', () => { if (!coreRec || !coreRec.frames.length) { toast('先に Coordinator ロールアウトを実行', 'warn'); return; }
    downloadText('weldcell_trajectory.jsonl', coreRec.toJSONL({ config: exportConfig() }), 'application/x-ndjson'); });
  on('tableExportBtn', 'click', () => { const sm = RL.surrogate || new WC.SurrogateModel(WeldPoolModel, trainBase()); RL.surrogate = sm;
    downloadText('weldcell_surrogate_table.json', JSON.stringify(sm.toJSON())); });
  on('connBtn', 'click', connect); on('discBtn', 'click', () => { disconnect(); setChoice('opMode', 'sim'); log('切断'); });
  on('pingBtn', 'click', async () => { if (!S2R.client) return toast('未接続', 'warn');
    try { const p = await S2R.client.ping(); const c = await S2R.client.request(WC.MSG.GET_CALIBRATION, {});
      toast(`pong rtt ${S2R.client.stats.rttMs.toFixed(1)} ms · 校正情報 frame=${c.data.frame}`, 'real'); } catch (e) { toast(e.message, 'err'); } });
  on('armBtn', 'click', () => S2R.armed ? disarm() : arm()); on('estopBtn', 'click', estop);
  on('exciteBtn', 'click', startExcitation);
  on('calFromLog', 'click', () => runCalibration(WC.parseMeasurements(S2R.realRec.toJSONL()), '実機ログ'));
  on('calFileBtn', 'click', () => $('calFile').click());
  on('calFile', 'change', async () => { try { const f = await readFile($('calFile')); runCalibration(WC.parseMeasurements(f.text, { I: P.I, V: P.V, eta: P.eta, material: CH.material, position: POSMAP[weldPos] }), f.name); }
    catch (e) { toast('読込失敗: ' + e.message, 'err'); } });
  on('calSynth', 'click', () => { const truth = { effScale: 0.90, kScale: 1.15, sigScale: 1.10 };
    const res = runCalibration(WC.makeSyntheticDataset(WeldPoolModel, truth, 50, WC.makeRng(P.seed), 0.03), '合成データ（真値 0.90/1.15/1.10）'); });
  on('calApply', 'click', applyCalibration);
  on('calReset', 'click', () => { CALIB = { effScale: 1, kScale: 1, sigScale: 1 }; invalidateSurrogate(); poolEval = null; $('calBadge').textContent = ''; S2R.residual.reset();
    $('calOut').textContent = 'calib: effScale 1.000 · kScale 1.000 · sigScale 1.000'; log('校正をリセット'); });
  on('logClear', 'click', () => { $('logBox').innerHTML = ''; });
  on('cfgSave', 'click', () => downloadText(`weldcell_config_${new Date().toISOString().slice(0, 10)}.json`, JSON.stringify(exportConfig({ calib: CALIB, home: Q_HOME }), null, 1)));
  on('cfgLoad', 'click', () => $('cfgFile').click());
  on('cfgFile', 'change', async () => { try { const f = await readFile($('cfgFile')); const cfg = JSON.parse(f.text); importConfig(cfg);
    if (cfg.calib) CALIB = Object.assign({ effScale: 1, kScale: 1, sigScale: 1 }, cfg.calib); if (cfg.home) Q_HOME = cfg.home.slice();
    R = WC.makeRng(P.seed); invalidateSurrogate(); toast('設定を読み込みました: ' + f.name, 'ok'); } catch (e) { toast('設定の読込に失敗: ' + e.message, 'err'); } });
  on('cfgReset', 'click', () => { for (const k in CHOICES) setChoice(k, CHOICES[k].def); for (const k in PARAMS) setParam(k, PARAMS[k].def); toast('全パラメータを既定値に戻しました'); });
  document.addEventListener('keydown', e => {
    if (e.target && (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT' || e.target.tagName === 'TEXTAREA')) return;
    if (e.code === 'Space') { e.preventDefault(); phase === 'weld' ? opStop() : opWeld(); }
    else if (e.key === 'Escape') { if (S2R.connected && CH.opMode === 'control') estop(); else opStop(); }
    else if (/^[1-5]$/.test(e.key) && !e.ctrlKey && !e.metaKey && !e.altKey) setStage(+e.key);
  });
  $('wsCanvas').addEventListener('dblclick', () => WSorbit.reset());
  window.addEventListener('beforeunload', () => disconnect());
}
function syncChoice(cid) { onChoice(cid, CH[cid]); }

/* stable handle for external agents / notebooks / automation (Playwright etc.) */
window.CobotAPI = {
  version: APP_VERSION,
  getState: () => liveRobotState(0), getPoolObs: livePoolObs, getWeldInput: liveWeldInput,
  applyControl: (a) => applyLiveAction(a, { auto: true }), startWeld: opWeld, stop: opStop, reset: opHome, approach: opApproach,
  setParam: (k, v) => setParam(k, v), setChoice: (k, v) => setChoice(k, v), getConfig: () => exportConfig({ calib: CALIB }), setStage,
  quality: () => qualityMetrics(),
};
window.WeldCellApp = { RL, S2R, train, connect, disconnect, runCalibration, applyCalibration, evaluatePolicies, get CALIB() { return CALIB; } };

(function init() {
  buildInspector(); wire();
  buildJbars(); initCharts(); applyCamMode();
  setSysMode(CH.sysMode); applyJointPos(); procBadge(); setCtlMode(CH.ctlMode); syncReward(); specNote();
  for (const id of ['speed', 'work', 'travel', 'height', 'wrRoll', 'wrTilt', 'leg', 'curve', 'noise', 'look', 'camD', 'camE', 'camA', 'lam', 'dwell']) onParamChange(id, P[id]);
  setMode('sim'); setPhase('idle');
  const hm = (location.hash || '').match(/stage=(\d)/);   // deep link (#stage=N from the manual) wins over the remembered stage
  setStage(hm ? +hm[1] : +(safeStore.get('weldcell.stage') || 1));
  const tab = safeStore.get('weldcell.tab'); if (tab && $('pane-' + tab) && !location.hash) activateTab(tab);
  log(`WeldCell v${APP_VERSION} · core v${WC.VERSION} · pool model v${WeldPoolModel.VERSION} · seed ${P.seed}`, 'ok');
  loop();
})();
