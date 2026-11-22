/* =============================================================================
 * loop.js — state machine, render loop, parameter/choice handlers.
 * ========================================================================== */
'use strict';
function setPhase(p) {
  if (phase !== p) log('phase: ' + phase + ' → ' + p);
  phase = p; const el = $('mPhase'); el.textContent = p; el.className = 'v' + (p === 'weld' ? ' run' : (p === 'done' ? ' ok' : ''));
}
function resize() {
  for (const X of [WS, CAM]) { const c = X.renderer.domElement, w = c.clientWidth, h = c.clientHeight;
    if (w && h && (c.width !== Math.floor(w * X.renderer.getPixelRatio()) || c.height !== Math.floor(h * X.renderer.getPixelRatio()))) { X.renderer.setSize(w, h, false); X.cam.aspect = w / h; X.cam.updateProjectionMatrix(); } }
  if (ov.width !== ov.clientWidth || ov.height !== ov.clientHeight) { ov.width = ov.clientWidth; ov.height = ov.clientHeight; }
}

/* ------------------------------------------------ joint × position × path */
function applyJointPos() {
  if (weldPos === 'F') { PBX.set(1, 0, 0); PBY.set(0, 1, 0); PBZ.set(0, 0, 1); SEAMO.set(0.40, 0.00, 0); SEAMLEN = 0.18; }
  else if (weldPos === 'H') { PBX.set(0, 1, 0); PBY.set(-1, 0, 0); PBZ.set(0, 0, 1); SEAMO.set(0.40, 0.12, 0); SEAMLEN = 0.18; }
  else if (weldPos === 'VU') { PBX.set(0, 0, -1); PBY.set(-1, 0, 0); PBZ.set(0, 1, 0); SEAMO.set(0.40, 0.175, 0); SEAMLEN = 0.24; }
  else { PBX.set(0, 0, 1); PBY.set(-1, 0, 0); PBZ.set(0, -1, 0); SEAMO.set(0.40, 0.175, 0); SEAMLEN = 0.24; }
  SDIR.copy(PBZ); BOWDIR.copy(PBX);
  const toWorld = v => PBX.clone().multiplyScalar(v.x).addScaledVector(PBY, v.y).addScaledVector(PBZ, v.z);
  let lN, lA;
  if (jointType === 'fillet') { lN = V3(-1, 1, 0); lA = V3(1, -1, 0); WORKREF = 45 * D2R; setParamRange('work', 30, 60, 45); }
  else { lN = V3(0, 1, 0); lA = V3(0, -1, 0); WORKREF = 90 * D2R; setParamRange('work', 70, 110, 90); }
  workAngle = P.work * D2R;
  CORNER_OPEN.copy(toWorld(lN)).normalize(); CORNER_INTO.copy(toWorld(lA)).normalize();
  BASE_N.copy(CORNER_OPEN); BASE_INTO.copy(CORNER_INTO);
  if (boxing) SEAMLEN = BOX.total;
  weldEval = null;
  buildWorkpiece(); clearBeads(); poolGroup.visible = false; rebuildSeamLine(); resetCharts(); setPhase('idle'); s = 0;
  const jn = { fillet: '隅肉', bop: 'ビードオンプレート', butt: '突き合わせ' }[jointType];
  const pn = { F: '下向 (F)', H: '横向 (H)', VU: '立向上進 (V↑)', VD: '立向下進 (V↓)' }[weldPos];
  $('wsSub').textContent = `${jn} · ${pn}${boxing ? ' · 角回し' : ''}${multipass ? '＋別部材' : ''}`;
  drawFillet(); reachBadge();
}
function setPath(v) {
  boxing = v !== 'line'; multipass = v === 'multi'; pass2Active = false; passIdx = 0;
  if (boxing && (jointType !== 'fillet' || weldPos !== 'F')) { jointType = 'fillet'; weldPos = 'F'; setChoice('joint', 'fillet', { silent: true }); setChoice('pos', 'F', { silent: true });
    log('角回しは隅肉・下向で実行します'); }
  applyJointPos();
}
function setSysMode(m) {
  sysMode = m; setCamArmVisible(m === 'dual');
  if (m === 'single') { if (WORK_Y > 0) { setParam('bench', 0, { silent: true }); setWorkY(0); } if (CH.baseTarget === 'camera') setChoice('baseTarget', 'torch'); }
  if (m === 'dual' && robotForm === 'humanoid') setWorkY(P.bench / 1000);
  applyFormVisibility(); recomputeBases(); updateCamTags();
  $('sysNote').textContent = m === 'dual' ? '双腕: 2台目アームがトーチ・ワイヤ・溶融池を遮蔽なく撮影し，画像からトーチ姿勢を推定．干渉回避で視点を探索します．'
    : '単腕: トーチ一体カメラ（眼手）で溶接線と溶融池を観測します．';
}
function setRobotForm(f) {
  robotForm = f;
  if (f !== 'humanoid' && camFeed === 'head') setChoice('camFeed', 'arm');
  if (f === 'humanoid') { if (baseTarget === 'torch' && P.baseY < 200) setParam('baseY', 300, { silent: true }); if (P.bench < 100) setParam('bench', 380, { silent: true }); setWorkY(P.bench / 1000); }
  else if (WORK_Y > 0) { setParam('bench', 0, { silent: true }); setWorkY(0); }
  applyFormVisibility(); recomputeBases();
  log('ロボット形態: ' + ({ humanoid: '立位ヒューマノイド', dual: '双腕統合', separate: '独立2台' }[f]));
}
function updateCamTags() {
  const t = sysMode === 'dual' ? ({ arm: 'カメラアーム', head: 'ヘッドカメラ', torch: 'トーチカメラ' }[camFeed]) : '眼手カメラ';
  $('camSub').textContent = t; $('poolCamTag').textContent = t + ' + 池輪郭';
}
/** robot base positions from sliders (+ humanoid waist joints) */
function recomputeBases() {
  const bx = P.baseX / 1000, by = P.baseY / 1000, bz = P.baseZ / 1000;
  if (baseTarget === 'camera') { CAM_OFFSET.set(bx, by, bz); camViewSel = null; } else BASE_RAW.set(bx, by, bz);
  const humA = (robotForm === 'humanoid' && sysMode === 'dual'), hp = humA ? torsoPitch : 0, wy = humA ? waistYaw : 0, ox = Math.sin(hp) * HTORSO;
  ROBOT_BASE.set(BASE_RAW.x + ox * Math.cos(wy), BASE_RAW.y + (Math.cos(hp) - 1) * HTORSO, BASE_RAW.z - ox * Math.sin(wy));
  CAM_BASE.copy(ROBOT_BASE).add(rotOffY(CAM_OFFSET, wy));
  pedestal.position.set(ROBOT_BASE.x, ROBOT_BASE.y + 0.03, ROBOT_BASE.z);
  const bk = [ROBOT_BASE.x, ROBOT_BASE.y, ROBOT_BASE.z, WORK_Y].map(v => v.toFixed(4)).join(',');
  if (bk !== _lastBaseKey) { _lastBaseKey = bk; if (weldEval) { weldEval = null; rebuildSeamLine(); reachBadge(); } }
}
function reachBadge() {
  const b = $('reachBadge'); if (!b) return;
  if (!weldEval) { b.textContent = '未評価'; b.style.color = ''; return; }
  b.textContent = (weldEval.frac * 100).toFixed(0) + '% 到達'; b.style.color = weldEval.frac >= 0.999 ? 'var(--acc)' : 'var(--bad)';
}
function runWeldEval() { const ev = evaluateWeldLine(); const msg = reportWeldEval(ev); $('reachOut').textContent = msg + `\n最大 IK 誤差 ${(ev.worstErr * 1000).toFixed(2)} mm`;
  $('reachOut').className = 'out ' + (ev.frac >= 0.999 ? 'ok' : 'warn'); reachBadge(); log(msg, ev.frac >= 0.999 ? 'ok' : 'warn'); return ev; }
/** grid-search the torch base X/Z that maximises weld-line reachability */
function autoPlaceBase() {
  if (baseTarget !== 'torch') setChoice('baseTarget', 'torch');
  const x0 = P.baseX, z0 = P.baseZ; let best = { frac: -1, x: x0, z: z0, err: 1 };
  for (const dx of [-120, -60, 0, 60, 120]) for (const dz of [-160, -80, 0, 80, 160]) {
    setParam('baseX', x0 + dx, { silent: true }); setParam('baseZ', z0 + dz, { silent: true }); recomputeBases();
    const ev = evaluateWeldLine(); if (ev.frac > best.frac || (ev.frac === best.frac && ev.worstErr < best.err)) best = { frac: ev.frac, x: P.baseX, z: P.baseZ, err: ev.worstErr };
  }
  setParam('baseX', best.x, { silent: true }); setParam('baseZ', best.z, { silent: true }); recomputeBases(); runWeldEval();
  toast(`基台を自動配置: X=${best.x} mm, Z=${best.z} mm（到達 ${(best.frac * 100).toFixed(0)}%）`, best.frac >= 0.999 ? 'ok' : 'warn');
}

/* -------------------------------------------------------- param handlers */
function onParamChange(id, v) {
  switch (id) {
    case 'speed': weldSpeed = v / 1000; break;
    case 'work': workAngle = v * D2R; drawFillet(); break;
    case 'travel': travelAngle = v * D2R; break;
    case 'height': standoff = v / 1000; break;
    case 'wrRoll': wristRoll = v * D2R; break;
    case 'wrTilt': wristTilt = v * D2R; break;
    case 'leg': legLen = v / 1000; break;
    case 'curve': seamCurve = v / 1000; rebuildSeamLine(); weldEval = null; reachBadge(); break;
    case 'noise': seamNoise = v / 1000; break;
    case 'look': lookAhead = v / 1000; break;
    case 'camD': camDist = v / 1000; break;
    case 'camE': camElev = v * D2R; break;
    case 'camA': camAzim = v * D2R; break;
    case 'baseX': case 'baseY': case 'baseZ': recomputeBases(); break;
    case 'lam': ikLambda = v; break;
    case 'dwell': cornerDwell = v; break;
    case 'pitch': torsoPitch = v * D2R; recomputeBases(); break;
    case 'waist': waistYaw = v * D2R; recomputeBases(); break;
    case 'bench': setWorkY(v / 1000); break;
    case 'headY': headYaw = v * D2R; if (headAuto) setChoice('headAuto', 'manual'); break;
    case 'headP': headPit = v * D2R; if (headAuto) setChoice('headAuto', 'manual'); break;
    case 'thick': buildWorkpiece(); poolEval = null; break;
    case 'I': case 'V': case 'eta': case 'T0': poolEval = null; procBadge(); invalidateSurrogate(); break;
    case 'seed': break;
    case 'wArea': case 'wTrack': case 'wEffort': syncReward(); break;
    case 'rateV': case 'rateAng': case 'watchdog': if (typeof syncSafety === 'function') syncSafety(); break;
  }
  if (PARAM_AUTO.has(id) && _rows[id]) _rows[id].row.classList.remove('auto');
}
function procBadge() { const b = $('procBadge'); if (b) b.textContent = `${P.I} A · ${P.V} V · ${(P.eta * P.V * P.I / 1000).toFixed(2)} kW`; }
function syncReward() { for (const r of [liveReward]) { r.wArea = P.wArea; r.wTrack = P.wTrack; r.wEffort = P.wEffort; } }
function onChoice(cid, v) {
  switch (cid) {
    case 'joint': jointType = v; if (boxing && v !== 'fillet') { setChoice('path', 'line', { silent: true }); boxing = multipass = false; } applyJointPos(); break;
    case 'pos': weldPos = v; if (boxing && v !== 'F') { setChoice('path', 'line', { silent: true }); boxing = multipass = false; } applyJointPos(); break;
    case 'path': setPath(v); break;
    case 'material': poolEval = null; buildWorkpiece(); invalidateSurrogate(); break;
    case 'sysMode': setSysMode(v); break;
    case 'form': setRobotForm(v); break;
    case 'baseTarget': baseTarget = v; { const src = v === 'camera' ? CAM_OFFSET : BASE_RAW;
      setParam('baseX', Math.round(src.x * 1000), { silent: true }); setParam('baseY', Math.round(src.y * 1000), { silent: true }); setParam('baseZ', Math.round(src.z * 1000), { silent: true }); } break;
    case 'camFeed': if (v === 'head' && robotForm !== 'humanoid') { setChoice('camFeed', camFeed, { silent: true }); toast('ヘッドカメラはヒューマノイド形態のみ'); return; }
      camFeed = v; updateCamTags(); break;
    case 'headAuto': headAuto = v === 'auto'; break;
    case 'camMode': camMode = v; applyCamMode(); break;
    case 'poolShape': poolShapeMode = v; lastPoolField = null; break;
    case 'section': showSection = v === 'on'; if (!showSection) poolSection.visible = false; break;
    case 'ctlMode': setCtlMode(v); break;
    case 'obsSpec': case 'actSpec': specNote(); break;
    case 'trainEnv': case 'dr': invalidateSurrogate(); break;
    case 'opMode': setMode(v); break;
    case 'backend': $('wsRow').classList.toggle('hide', v !== 'ws'); break;
  }
}
function setCtlMode(v) {
  if (v === 'policy' && !RL.policy) { toast('RL 方策が未学習です（④ 学習で Train）', 'warn'); setChoice('ctlMode', 'auto3', { silent: true }); v = 'auto3'; }
  CH.ctlMode = v; target.set = false;
  liveController = v === 'autoCore' ? new WeldCell.AutoHoldController() : v === 'policy' ? new WeldCell.PolicyController(RL.policy, RL.makeCodec()) : null;
  if (liveController) liveController.reset();
  $('mCtl').textContent = controllerLabel(); $('mCtl').className = 'v' + (v === 'policy' ? ' real' : v === 'off' ? '' : ' ok');
  const db = $('deployBtn'); if (db) { db.textContent = 'Deploy: ' + (v === 'policy' ? 'ON' : 'OFF'); db.classList.toggle('primary', v === 'policy'); }
  for (const id of PARAM_AUTO) if (_rows[id]) _rows[id].row.classList.remove('auto');
  $('ctlNote').innerHTML = {
    off: '制御なし．スライダの値がそのまま溶接条件になります．',
    auto3: '<b>AUTO 3軸</b>（v21 古典）：溶融池の面積→速度，幅→アーク長，縦横比→作業角 を比例制御．',
    autoCore: '<b>AUTO 契約版</b>：連携中核の AutoHoldController を契約（RobotState / PoolObservation → ControlAction）経由で適用．実機でも同一コード．',
    policy: '<b>RL 方策</b>：④ で学習した方策を PolicyController（Codec 経由）で配備．サロゲート・ライブ・実機で同一の観測／行動定義．',
  }[v];
  log('溶融池制御: ' + controllerLabel());
}

/* -------------------------------------------------------- operation */
function opHome() { setPhase('home'); s = 0; poolGroup.visible = false; resetCharts(); restoreNominal(); }
function opApproach() { s = 0; poolGroup.visible = false; resetCharts(); restoreNominal(); const ev = evaluateWeldLine(); reachBadge(); setPhase('approach');
  if (ev.frac < 0.999) toast(reportWeldEval(ev), 'warn'); }
function opWeld() {
  if (phase === 'weld') return;
  dryRun = false; if (phase === 'idle' || phase === 'home') setPhase('approach'); if (s >= 1) { s = 0; resetCharts(); }
  if (s === 0) { restoreNominal(); target.set = false; liveReward.reset(); if (liveController) liveController.reset(); }
  setPhase('weld'); log('溶接開始 — 制御: ' + controllerLabel(), 'ok');
}
function opDry() { const ev = evaluateWeldLine(); reachBadge(); dryRun = true; if (s >= 1) s = 0; passIdx = 0; pass2Active = false; setPhase('weld'); log('動作シミュレーション（溶接なし）· ' + reportWeldEval(ev)); }
function opStop() { if (phase === 'weld') { dryRun = false; setPhase('ready'); arcLight.intensity = 0; arcDot.visible = false; log('停止'); } else if (phase === 'approach' || phase === 'home') setPhase('idle'); }

/* -------------------------------------------------------------- recording */
const liveRec = new WeldCell.TrajectoryRecorder(null, { source: 'live-sim' });
let recOn = false, recAcc = 0;
function toggleRec(on) {
  recOn = on == null ? !recOn : on; if (recOn) liveRec.start(); else liveRec.stop();
  $('recBtn').classList.toggle('go', recOn); $('recBtn').textContent = recOn ? '● REC ON' : '● REC'; log(recOn ? '記録開始' : '記録停止 · ' + liveRec.frames.length + ' frames', recOn ? 'ok' : '');
}

/* ------------------------------------------------------------- main loop */
let rewAcc = 0, uiAcc = 0, ctlAcc = 0;
const CTL_DT = 0.05;   // control period [s] shared with training (dt) and the real bridge (20 Hz)
function loop() {
  requestAnimationFrame(loop); resize();
  const dt = Math.min(0.05, clock.getDelta()), now = performance.now(), ns = seamNoise;
  disturb.lat = disturb.lat * 0.97 + (R() - 0.5) * ns * 0.5;
  disturb.work = disturb.work * 0.85 + (R() - 0.5) * (ns * 1000 * 0.05) * D2R;
  disturb.travel = disturb.travel * 0.85 + (R() - 0.5) * (ns * 1000 * 0.05) * D2R;
  if (typeof s2rBeforeFrame === 'function') s2rBeforeFrame(dt);       // SHADOW/CONTROL: mirror real setpoints

  if (phase === 'approach') {
    { const fr = frameAt(0); CORNER_OPEN.copy(fr.n); CORNER_INTO.copy(fr.into); }
    const tt = torchTarget(0); let err = 0; for (let k = 0; k < 10; k++) err = ikStep(tt.p, tt.Rd, ikLambda);
    lastErr = err;
    if (err < 0.0015) { if (autoWeldNext) { autoWeldNext = false; s = 0; setPhase('weld'); } else { setPhase('ready'); } }
  } else if (phase === 'weld') {
    const dCur = clamp(s, 0, 1) * BOX.total;
    const inCornerNow = boxing && !pass2Active && ((dCur >= BOX.b1 - 0.004 && dCur <= BOX.b2 + 0.004) || (dCur >= BOX.b3 - 0.004 && dCur <= BOX.b4 + 0.004));
    if (inCornerNow && !prevInCorner && cornerDwell > 0) { dwellRemain = cornerDwell; log('角回し: コーナー一旦停止 ' + cornerDwell.toFixed(1) + ' s'); }
    prevInCorner = inCornerNow;
    if (dwellRemain > 0) dwellRemain -= dt; else s += weldSpeed * dt / SEAMLEN;
    if (s >= 1) { s = 1;
      if (multipass && passIdx < PASSES.length - 1) advancePass();
      else { setPhase('done'); log(dryRun ? '動作シミュレーション完了（溶接なし）' : '溶接完了', 'ok'); arcLight.intensity = 0; arcDot.visible = false; } }
    { const fr = frameAt(clamp(s, 0, 1)); CORNER_OPEN.copy(fr.n); CORNER_INTO.copy(fr.into); }
    cornerHeat += ((inCornerNow ? 230 : 0) - cornerHeat) * Math.min(1, dt / 0.6);
    const tt = torchTarget(clamp(s, 0, 1)); let err = 0; for (let k = 0; k < 12; k++) err = ikStep(tt.p, tt.Rd, ikLambda); lastErr = err;
    if (phase === 'weld' && !dryRun) {
      const B = teBasis(fk(q).Te), appr = B.y.clone().normalize(), wp = weldOnSurface(B.p.clone(), appr, s);
      if (CH.track === 'on') seamTracking(wp);
      if (!beads.length || beads[beads.length - 1].mesh.position.distanceTo(wp) > legLen * 0.7) addBead(wp);
      emitSparks(wp);
      arcLight.position.copy(wp).add(CORNER_OPEN.clone().multiplyScalar(0.02)); arcLight.intensity = 1.6 + Math.random() * 1.2; arcLight.color.setHex(0xbfe3ff);
      arcDot.visible = true; arcDot.position.copy(wp); arcDot.material.opacity = 0.7 + Math.random() * 0.3;
      updatePool(wp, s, dt); poolGroup.visible = true;
    } else if (phase === 'weld' && dryRun) { poolGroup.visible = false; arcLight.intensity = 0; arcDot.visible = false; }
  } else if (phase === 'home') {
    let d = 0; for (let i = 0; i < JD.length; i++) { q[i] += (Q_HOME[i] - q[i]) * Math.min(1, 8 * dt); d += Math.abs(Q_HOME[i] - q[i]); }   // all 7 joints (v21: J7 never homed)
    if (d < 0.01) setPhase('idle');
  } else { arcLight.intensity = 0; arcDot.visible = false; }

  // ---- closed-loop pool control (one dispatcher, contract semantics) ----
  let rs = null, ob = null, act = null;
  const welding = phase === 'weld' && !dryRun && poolFeat && poolGroup.visible;
  if (welding) {
    rs = liveRobotState(dt); ob = livePoolObs();
    if (!(typeof s2rControlsLive === 'function' && s2rControlsLive())) {
      if (CH.ctlMode === 'auto3') classicAuto();
      else if (liveController) {            // contract controllers run at the SAME period as in training/real (20 Hz)
        ctlAcc += dt;
        if (ctlAcc >= CTL_DT) { ctlAcc -= CTL_DT; if (ctlAcc > CTL_DT) ctlAcc = 0; act = liveController.update(rs, ob, CTL_DT); if (act) applyLiveAction(act); }
      }
    }
    const r = liveReward.compute(rs, ob, act); lastReward = r;
    rewAcc += dt; if (rewAcc >= 0.08) { rewAcc = 0; qualWin.push({ area: ob.area, track: rs.trackError || 0 }); if (qualWin.length > 60) qualWin.shift(); }
    recAcc += dt; if (recOn && recAcc >= 0.05) { recAcc = 0; liveRec.push({ t: now / 1000, step: liveRec.frames.length, weldInput: rs.weldInput, poolObs: ob, robotState: rs, action: act, reward: r }); }
  }

  updateArm(); if (humanoidBody.visible) updateHumanoid(); placeCamera(); if (dualBase.visible) updateDualBase(); coolBeads(now); updateSparks(dt);
  if (sysMode === 'dual') { visionAcc += dt; if (visionAcc >= 0.2) { visionAcc = 0; visionState = visionEstimate(); } } else visionState = null;

  if (phase === 'weld') {
    const wpNow = seamPoint(clamp(s, 0, 1));
    if (lastWP) { const inst = wpNow.distanceTo(lastWP) / Math.max(dt, 1e-3) * 1000; poolSpeed = poolSpeed * 0.8 + inst * 0.2; }
    lastWP = wpNow.clone(); chartAcc += dt;
    if (chartAcc >= 0.08) { chartAcc = 0; const m = poseMetrics();
      poseChart.push([m.distMM, m.travelA, m.workA]);
      poolChart.push([poolSpeed, poolFeat ? poolFeat.areaMM : 0, poolFeat ? poolFeat.widMM : 0, poolFeat ? poolFeat.lenMM : 0]);
      if (welding) { const qm = qualityMetrics(); qualChart.push([Number.isFinite(qm.cv) ? qm.cv : 0, Number.isFinite(qm.trms) ? qm.trms : 0, lastReward]); } }
  }
  if (target.set) { poolChart.series[1].ref = target.A; poolChart.series[2].ref = target.W; poolChart.series[3].ref = target.L; }
  else if (liveReward.target) { poolChart.series[1].ref = liveReward.target * 1e6; }
  if ($('poseChart').offsetParent) { poseChart.draw(); poolChart.draw(); }
  if ($('qualChart').offsetParent) qualChart.draw();
  drawPoolShapes(dt); drawModelField(dt); updateQualityPane(dt);
  if (!poolGroup.visible) poolSection.visible = false;
  if (typeof s2rAfterFrame === 'function') s2rAfterFrame(dt);

  uiAcc += dt;
  if (uiAcc >= 0.1) { uiAcc = 0;
    const pr = Math.round(clamp(s, 0, 1) * 100); $('mProg').textContent = pr + '%' + (multipass ? ` · pass ${passIdx + 1}/${PASSES.length}` : ''); $('mProgBar').style.width = pr + '%';
    $('mErr').textContent = (lastErr * 1000).toFixed(2) + ' mm'; $('mErr').className = 'v' + (lastErr > 0.004 ? ' bad' : '');
    const live = poolFeat && poolGroup.visible;
    $('mPool').textContent = live ? `${poolFeat.lenMM.toFixed(1)}×${poolFeat.widMM.toFixed(1)}` : '—';
    $('mArea').textContent = live ? poolFeat.areaMM.toFixed(1) + ' mm²' : '—';
    $('mArea').className = 'v' + (live && poolEval && poolEval.status === 'burn_through' ? ' bad' : '');
    $('mTrack').textContent = rs ? ((rs.trackError || 0) * 1000).toFixed(2) + ' mm' : '—';
    $('mRew').textContent = Number.isFinite(lastReward) && welding ? lastReward.toFixed(2) : '—';
    if ($('recOut')) $('recOut').textContent = `${liveRec.frames.length} frames${recOn ? ' · recording' : ''}`;
    updateJbars();
    if (sysMode === 'dual' && visionState) { const v = visionState;
      $('visStat').textContent = v.visible ? '良好' : '不可'; $('visStat').className = 'v ' + (v.visible ? 'ok' : 'bad');
      $('visStand').textContent = `${v.standEst.toFixed(1)} (真 ${v.standTrue.toFixed(1)})`; $('visAng').textContent = `${v.workEst.toFixed(1)}° / ${v.travelEst.toFixed(1)}°`;
      $('visIntf').textContent = (camInterf.hit ? 'あり' : 'なし') + ` · ${(camInterf.minClear * 1000).toFixed(0)} mm`; $('visIntf').className = 'v ' + (camInterf.hit ? 'bad' : 'ok'); }
  }
  $('camHud').innerHTML = hudText();
  WS.renderer.render(WS.scene, WS.cam);
  const dim = (camMode === 'nir' || camMode === 'pool'), sArc = arcLight.intensity, sDot = arcDot.visible;
  if (dim) { arcLight.intensity *= 0.18; arcDot.visible = false; }
  CAM.renderer.render(CAM.scene, CAM.cam);
  if (dim) { arcLight.intensity = sArc; arcDot.visible = sDot; }
  drawOverlay(); drawPoolCamImage();
}
