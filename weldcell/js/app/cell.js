/* =============================================================================
 * cell.js — live 3-D welding cell (7-DOF cobot, dual-arm / humanoid, vision,
 * weld pool, bead) driven at display rate. Exposes the cell through the core
 * contract (liveRobotState / livePoolObs / applyLiveAction) so classic AUTO,
 * core controllers, learned policies and the real-robot bridge all use the
 * same semantics as the headless training cell.
 *
 * Kinematics: human-arm-like 7-DOF chain (J3 upper-arm roll; J3=0 = former 6R),
 * FANUC CRX-class reach ≈ 0.69 m, damped-least-squares IK.
 * ========================================================================== */
'use strict';
const V3 = (x, y, z) => new THREE.Vector3(x, y, z);
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const D2R = Math.PI / 180;
let R = WeldCell.makeRng(P.seed);          // seeded RNG for disturbances & sensing (reproducible)
let CALIB = { effScale: 1, kScale: 1, sigScale: 1 };   // Real2Sim calibration applied to the live pool model

// ---- robot kinematics (meters) ----
const Lbase = 0.20, Lupper = 0.25, Lfore = 0.24, Lw = 0.07;        // link lengths -> ~0.69 m reach
// compact air-cooled gooseneck torch (~177 mm body, 31deg neck)
const TB = 0.120, BEND = 31 * Math.PI / 180, TN = 0.100;
const JD = [
  { axis: [0, 1, 0], t: [0, 0, 0] },            // J1 shoulder yaw
  { axis: [0, 0, 1], t: [0, Lbase, 0] },        // J2 shoulder pitch
  { axis: [0, 1, 0], t: [0, Lupper * 0.5, 0] }, // J3 upper-arm roll
  { axis: [0, 0, 1], t: [0, Lupper * 0.5, 0] }, // J4 elbow pitch
  { axis: [0, 1, 0], t: [0, Lfore, 0] },        // J5 forearm roll
  { axis: [0, 0, 1], t: [0, Lw, 0] },           // J6 wrist pitch
  { axis: [0, 1, 0], t: [0, Lw, 0] },           // J7 tool roll
];
const LIM = [[-180, 180], [-135, 135], [-150, 150], [-170, 170], [-190, 190], [-180, 180], [-200, 200]].map(p => [p[0] * D2R, p[1] * D2R]);
let q = [0, 0.6, 0, -1.15, 0, 1.0, 0];
const Q_HOME_DEF = [0, 0.6, 0, -1.15, 0, 1.0, 0];
let Q_HOME = Q_HOME_DEF.slice();
let ROBOT_BASE = new THREE.Vector3(0, 0, 0);
let BASE_RAW = new THREE.Vector3(0, 0, 0);
let torsoPitch = 18 * D2R;
const HTORSO = 0.26;
let WORK_Y = 0;
let ikLambda = 0.05;
let cornerDwell = 0.4;
let waistYaw = 0;
let headAuto = true, headYaw = 0, headPit = -25 * D2R;
let wristRoll = 0, wristTilt = 0;
function rotOffY(v, a) { const c = Math.cos(a), s = Math.sin(a); return V3(v.x * c + v.z * s, v.y, -v.x * s + v.z * c); }
const _m = new THREE.Matrix4(), _t = new THREE.Matrix4();
function fk(qq, base, withTorch) {
  if (!base) base = ROBOT_BASE; if (withTorch === undefined) withTorch = true;
  const M = new THREE.Matrix4().makeTranslation(base.x, base.y, base.z); const joints = [];
  for (let i = 0; i < JD.length; i++) {
    M.multiply(_t.makeTranslation(JD[i].t[0], JD[i].t[1], JD[i].t[2]));
    const p = new THREE.Vector3().setFromMatrixPosition(M);
    const z = new THREE.Vector3(JD[i].axis[0], JD[i].axis[1], JD[i].axis[2]).transformDirection(M.clone());
    joints.push({ p, z });
    M.multiply(_m.makeRotationAxis(V3(JD[i].axis[0], JD[i].axis[1], JD[i].axis[2]), qq[i]));
  }
  const Tf = M.clone();
  if (withTorch) { M.multiply(_t.makeTranslation(0, TB, 0)); M.multiply(_m.makeRotationX(BEND)); M.multiply(_t.makeTranslation(0, TN, 0)); }
  return { joints, Te: M.clone(), Tf };
}
function teBasis(Te) {
  return { x: new THREE.Vector3().setFromMatrixColumn(Te, 0).normalize(), y: new THREE.Vector3().setFromMatrixColumn(Te, 1).normalize(),
    z: new THREE.Vector3().setFromMatrixColumn(Te, 2).normalize(), p: new THREE.Vector3().setFromMatrixPosition(Te) };
}
function rotErr(Rd, Re) {
  const a = Rd.elements, b = Re.elements;
  const rd = [[a[0], a[4], a[8]], [a[1], a[5], a[9]], [a[2], a[6], a[10]]];
  const re = [[b[0], b[4], b[8]], [b[1], b[5], b[9]], [b[2], b[6], b[10]]];
  const m = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) { let s = 0; for (let k = 0; k < 3; k++) s += rd[i][k] * re[j][k]; m[i][j] = s; }
  const angle = Math.acos(clamp((m[0][0] + m[1][1] + m[2][2] - 1) / 2, -1, 1));
  if (angle < 1e-6) return [0, 0, 0];
  const d = 2 * Math.sin(angle);
  return [(m[2][1] - m[1][2]) / d * angle, (m[0][2] - m[2][0]) / d * angle, (m[1][0] - m[0][1]) / d * angle];
}
function solve6(A, b) {
  const n = 6, M = A.map((r, i) => r.concat([b[i]]));
  for (let c = 0; c < n; c++) {
    let piv = c; for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
    [M[c], M[piv]] = [M[piv], M[c]];
    const d = M[c][c] || 1e-9; for (let j = c; j <= n; j++) M[c][j] /= d;
    for (let r = 0; r < n; r++) if (r !== c) { const f = M[r][c]; for (let j = c; j <= n; j++) M[r][j] -= f * M[c][j]; }
  }
  return M.map(r => r[n]);
}
function ikSolve(state, base, withTorch, targetP, Rd, lambda) {
  const f = fk(state, base, withTorch); const B = teBasis(f.Te); const pe = B.p;
  const Re = new THREE.Matrix4().extractRotation(f.Te);
  const ep = [targetP.x - pe.x, targetP.y - pe.y, targetP.z - pe.z];
  const er = rotErr(Rd, Re);
  const e = [ep[0], ep[1], ep[2], er[0], er[1], er[2]];
  const N = f.joints.length, J = [[], [], [], [], [], []];
  for (let i = 0; i < N; i++) {
    const z = f.joints[i].z, pi = f.joints[i].p;
    const jvc = z.clone().cross(new THREE.Vector3().subVectors(pe, pi));
    J[0][i] = jvc.x; J[1][i] = jvc.y; J[2][i] = jvc.z; J[3][i] = z.x; J[4][i] = z.y; J[5][i] = z.z;
  }
  const JJt = [[], [], [], [], [], []];
  for (let r = 0; r < 6; r++) for (let c = 0; c < 6; c++) { let s = 0; for (let k = 0; k < N; k++) s += J[r][k] * J[c][k]; JJt[r][c] = s + (r === c ? lambda * lambda : 0); }
  const y = solve6(JJt, e);
  const dq = new Array(N).fill(0);
  for (let i = 0; i < N; i++) { let s = 0; for (let r = 0; r < 6; r++) s += J[r][i] * y[r]; dq[i] = s; }
  let mx = 0; for (const v of dq) mx = Math.max(mx, Math.abs(v));
  const cap = 0.18, sc = mx > cap ? cap / mx : 1;
  for (let i = 0; i < N; i++) state[i] = clamp(state[i] + dq[i] * sc, LIM[i][0], LIM[i][1]);
  return Math.hypot(...ep);
}
function ikStep(targetP, Rd, lambda) { return ikSolve(q, ROBOT_BASE, true, targetP, Rd, lambda); }

// ---- workpiece & seam ----
let SEAMO = V3(0.40, 0, 0), SDIR = V3(0, 0, 1), BOWDIR = V3(1, 0, 0), SEAMLEN = 0.18;
let seamCurve = 0;
let boxing = false;
const BOX = { X0: 0.40, Th: 0.012, Z0: -0.09, Z1: 0.09, rc: 0.003, Z3: -0.085 };
(function () { const Xn = BOX.X0, Xf = BOX.X0 + BOX.Th, Zc = BOX.Z1, rc = BOX.rc;
  BOX.L1 = (Zc - rc) - BOX.Z0; BOX.a1 = (Math.PI / 2) * rc; BOX.L2 = (Xf - rc) - (Xn + rc); BOX.a2 = (Math.PI / 2) * rc; BOX.L3 = (Zc - rc) - BOX.Z3;
  BOX.b1 = BOX.L1; BOX.b2 = BOX.b1 + BOX.a1; BOX.b3 = BOX.b2 + BOX.L2; BOX.b4 = BOX.b3 + BOX.a2; BOX.total = BOX.b4 + BOX.L3;
})();
function boxingPoint(u) { const d = u * BOX.total, rc = BOX.rc, Xn = BOX.X0, Xf = BOX.X0 + BOX.Th, Zc = BOX.Z1;
  if (d <= BOX.b1) return V3(Xn, 0, BOX.Z0 + d);
  if (d <= BOX.b2) { const t = (d - BOX.b1) / BOX.a1 * (Math.PI / 2); return V3(Xn + rc - rc * Math.cos(t), 0, (Zc - rc) + rc * Math.sin(t)); }
  if (d <= BOX.b3) { const dd = d - BOX.b2; return V3(Xn + rc + dd, 0, Zc); }
  if (d <= BOX.b4) { const t = (d - BOX.b3) / BOX.a2 * (Math.PI / 2); return V3((Xf - rc) + rc * Math.sin(t), 0, (Zc - rc) + rc * Math.cos(t)); }
  const dd = d - BOX.b4; return V3(Xf, 0, (Zc - rc) - dd);
}
let multipass = false, pass2Active = false, autoWeldNext = false;
const PASSES = ['box', 'member2'];
const PASS2 = { X: 0.56, Z0: -0.09, Z1: 0.09, LEN: 0.18 };
function pass2Point(s) { return V3(PASS2.X, 0, PASS2.Z0 + (PASS2.Z1 - PASS2.Z0) * clamp(s, 0, 1)); }
function advancePass() {
  passIdx++; pass2Active = true; SEAMLEN = PASS2.LEN;
  prevInCorner = false; dwellRemain = 0; cornerHeat = 0; target.set = false; trackOffset = 0;
  s = 0; poolGroup.visible = false; rebuildSeamLine(); autoWeldNext = true;
  setPhase('approach'); log('別部材の溶接線へ移動（マルチパス ' + (passIdx + 1) + '/' + PASSES.length + '）');
}
function seamPoint(s) {
  let Pp;
  if (boxing && pass2Active) Pp = pass2Point(s);
  else if (boxing) Pp = boxingPoint(clamp(s, 0, 1));
  else { const bow = seamCurve * Math.sin(Math.PI * s); Pp = SEAMO.clone().addScaledVector(SDIR, (s - 0.5) * SEAMLEN).addScaledVector(BOWDIR, bow); }
  Pp.y += WORK_Y; return Pp;
}
function seamTangent(s) { const a = seamPoint(Math.max(0, s - 0.01)), b = seamPoint(Math.min(1, s + 0.01)); return b.sub(a).normalize(); }
let BASE_N = V3(-1, 1, 0).normalize(), BASE_INTO = V3(1, -1, 0).normalize();
let CORNER_OPEN = V3(-1, 1, 0).normalize();
let CORNER_INTO = V3(1, -1, 0).normalize();
let WORKREF = 45 * D2R;
function frameAt(s) {
  if (boxing && pass2Active) return { n: V3(-1, 1, 0).normalize(), into: V3(1, -1, 0).normalize() };
  if (boxing) { const tan = seamTangent(clamp(s, 0, 1)); const perp = V3(-tan.z, 0, tan.x).normalize();
    const n = perp.add(V3(0, 1, 0)).normalize(); return { n, into: n.clone().multiplyScalar(-1) }; }
  return { n: BASE_N, into: BASE_INTO };
}
let jointType = 'fillet';
let weldPos = 'F';
let PBX = V3(1, 0, 0), PBY = V3(0, 1, 0), PBZ = V3(0, 0, 1);

let workAngle = 45 * D2R, travelAngle = -15 * D2R;
function torchTarget(s) {
  const p0 = seamPoint(s), tan = seamTangent(s);
  const wA = workAngle + disturb.work, tA = travelAngle + disturb.travel;
  let approach = CORNER_INTO.clone().applyAxisAngle(tan, (wA - WORKREF));
  approach = approach.normalize().multiplyScalar(Math.cos(tA)).add(tan.clone().multiplyScalar(Math.sin(tA))).normalize();
  if (wristTilt) { const lat = new THREE.Vector3().crossVectors(tan, approach).normalize(); approach.applyAxisAngle(lat, wristTilt).normalize(); }
  const yv = approach.clone();
  let xv = tan.clone().sub(yv.clone().multiplyScalar(tan.dot(yv))); if (xv.lengthSq() < 1e-6) xv = V3(0, 0, 1); xv.normalize();
  if (wristRoll) { xv.applyAxisAngle(yv, wristRoll).sub(yv.clone().multiplyScalar(xv.dot(yv))).normalize(); }
  const zv = new THREE.Vector3().crossVectors(xv, yv).normalize();
  const Rd = new THREE.Matrix4().makeBasis(xv, yv, zv);
  const vlat = new THREE.Vector3().crossVectors(CORNER_OPEN, tan).normalize();
  const p = p0.clone().addScaledVector(vlat, disturb.lat + trackOffset).addScaledVector(tan, alongOffset).addScaledVector(yv, -standoff);
  return { p, Rd, approach: yv, tan, p0, vlat };
}
function weldOnSurface(tip, appr, ss) {
  const n = CORNER_OPEN, P0 = seamPoint(clamp(ss, 0, 1)), d = appr.dot(n);
  if (Math.abs(d) > 1e-3) { const t = clamp(P0.clone().sub(tip).dot(n) / d, -0.06, 0.06); return tip.clone().addScaledVector(appr, t); }
  return tip.clone().addScaledVector(appr, standoff);
}

// ============================ THREE scene ============================
function newScene(canvas, clear) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); renderer.setClearColor(clear === undefined ? 0x000000 : clear, clear === undefined ? 0 : 1);
  const scene = new THREE.Scene();
  const cam = new THREE.PerspectiveCamera(42, 1, 0.004, 50);
  scene.add(new THREE.AmbientLight(0x90a0b0, 0.75));
  const d1 = new THREE.DirectionalLight(0xffffff, 0.85); d1.position.set(0.6, 1.0, 0.5); scene.add(d1);
  const d2 = new THREE.DirectionalLight(0x2a4a66, 0.4); d2.position.set(-0.6, 0.4, -0.5); scene.add(d2);
  return { renderer, scene, cam };
}
const WS = newScene($('wsCanvas'));
const CAM = newScene($('camCanvas'), 0x05080b);
const world = WS.scene; CAM.scene = world;
(function () {
  const table = new THREE.Mesh(new THREE.BoxGeometry(1.2, 0.04, 1.0), new THREE.MeshStandardMaterial({ color: 0x12303a, metalness: .2, roughness: .85 }));
  table.position.set(0.25, -0.024, 0); world.add(table);
  const grid = new THREE.GridHelper(1.2, 24, 0x274050, 0x18242e); grid.position.y = 0.0005; grid.position.x = 0.25; world.add(grid);
})();
const workGroup = new THREE.Group(); world.add(workGroup);
function buildWorkpiece() {
  while (workGroup.children.length) { const c = workGroup.children[0]; workGroup.remove(c); if (c.geometry) c.geometry.dispose(); }
  const steel = new THREE.MeshStandardMaterial({ color: CH.material === 'al' ? 0xc4ccd2 : CH.material === 'sus' ? 0xa5b0b8 : 0x8a97a3, metalness: .7, roughness: .35 });
  const th = P.thick / 1000;
  const add = (gx, gy, gz, lx, ly, lz) => { const m = new THREE.Mesh(new THREE.BoxGeometry(gx, gy, gz), steel); m.position.set(lx, ly, lz); workGroup.add(m); };
  if (boxing) {
    add(0.40, th, 0.40, 0.04, -th / 2, 0.02);
    add(0.012, 0.11, 0.20, 0.006, 0.055, -0.01);
    if (multipass) add(0.012, 0.11, 0.20, 0.166, 0.055, -0.01);
  } else if (jointType === 'fillet') {
    add(0.40, th, 0.36, 0.04, -th / 2, 0); add(0.012, 0.12, 0.30, 0.006, 0.06, 0);
  } else if (jointType === 'bop') { add(0.34, th, 0.34, 0, -th / 2, 0); }
  else { const gap = 0.004; add(0.17, th, 0.34, -0.085 - gap / 2, -th / 2, 0); add(0.17, th, 0.34, 0.085 + gap / 2, -th / 2, 0); }
  workGroup.position.copy(SEAMO); workGroup.position.y += WORK_Y;
  workGroup.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(PBX, PBY, PBZ));
  updateBench();
}
const benchMat = new THREE.MeshStandardMaterial({ color: 0x4b4136, metalness: .15, roughness: .85 });
const bench = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), benchMat); bench.visible = false; world.add(bench);
const benchLegs = [0, 1, 2, 3].map(() => { const m = new THREE.Mesh(new THREE.BoxGeometry(0.03, 1, 0.03), benchMat); m.visible = false; world.add(m); return m; });
function updateBench() {
  const top = WORK_Y - 0.012, th = 0.025, show = top > 0.04;
  bench.visible = show; benchLegs.forEach(l => l.visible = show); if (!show) return;
  bench.scale.set(0.54, th, 0.48); bench.position.set(0.46, top - th / 2, 0.01);
  const lx = [0.46 - 0.24, 0.46 + 0.24], lzz = [0.01 - 0.20, 0.01 + 0.20]; let k = 0;
  for (const x of lx) for (const z of lzz) { const l = benchLegs[k++]; l.scale.y = top - th; l.position.set(x, (top - th) / 2, z); }
}
const pedestal = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.07, 0.06, 28), new THREE.MeshStandardMaterial({ color: 0x20262c, metalness: .5, roughness: .5 }));
pedestal.position.set(0, 0.03, 0); world.add(pedestal);
const armMat = new THREE.MeshStandardMaterial({ color: 0xe9edf1, metalness: .25, roughness: .5 });
const jointMat = new THREE.MeshStandardMaterial({ color: 0x223038, metalness: .6, roughness: .4 });
const armCyl = [], armJoint = [];
for (let i = 0; i < 8; i++) { const c = new THREE.Mesh(new THREE.CylinderGeometry(0.026, 0.026, 1, 20), armMat); world.add(c); armCyl.push(c); }
for (let i = 0; i < 7; i++) { const j = new THREE.Mesh(new THREE.SphereGeometry(0.034, 18, 14), jointMat); world.add(j); armJoint.push(j); }
const _up = V3(0, 1, 0);
function orient(mesh, a, b, r) {
  const dir = new THREE.Vector3().subVectors(b, a), len = dir.length();
  if (len < 1e-5) { mesh.visible = false; return; } mesh.visible = true;
  mesh.scale.set(r / 0.026, len, r / 0.026); mesh.position.copy(a).add(b).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(_up, dir.normalize());
}
const torch = new THREE.Group(); world.add(torch);
(function buildTorch() {
  const mMount = new THREE.MeshStandardMaterial({ color: 0x2b3038, metalness: .6, roughness: .42 });
  const mBarrel = new THREE.MeshStandardMaterial({ color: 0x515a64, metalness: .72, roughness: .32 });
  const mInsul = new THREE.MeshStandardMaterial({ color: 0x14181d, metalness: .2, roughness: .7 });
  const mNoz = new THREE.MeshStandardMaterial({ color: 0xb9742e, metalness: .85, roughness: .28 });
  const mTip = new THREE.MeshStandardMaterial({ color: 0xcaa23a, metalness: .9, roughness: .22 });
  const mWire = new THREE.MeshStandardMaterial({ color: 0xdfe8ef, metalness: .9, roughness: .2, emissive: 0x161616 });
  const mCable = new THREE.MeshStandardMaterial({ color: 0x0d0f12, metalness: .1, roughness: .85 });
  function cyl(parent, r1, r2, y0, y1, mat, open) { const m = new THREE.Mesh(new THREE.CylinderGeometry(r1, r2, (y1 - y0), 22, 1, !!open), mat); m.position.y = (y0 + y1) / 2; parent.add(m); return m; }
  cyl(torch, 0.025, 0.023, 0.000, 0.020, mMount); cyl(torch, 0.015, 0.013, 0.020, 0.030, mInsul); cyl(torch, 0.016, 0.015, 0.030, TB, mBarrel);
  const elbow = new THREE.Mesh(new THREE.SphereGeometry(0.016, 16, 12), mBarrel); elbow.position.set(0, TB, 0); torch.add(elbow);
  const neck = new THREE.Group(); neck.position.set(0, TB, 0); neck.rotation.x = BEND; torch.add(neck);
  cyl(neck, 0.012, 0.012, 0.000, 0.030, mInsul); cyl(neck, 0.012, 0.014, 0.030, TN - 0.004, mNoz, true);
  cyl(neck, 0.0035, 0.003, TN - 0.022, TN - 0.004, mTip); cyl(neck, 0.0011, 0.0011, TN - 0.012, TN + 0.002, mWire);
  const cv = new THREE.CatmullRomCurve3([V3(0, 0.012, 0), V3(-0.018, -0.01, 0.02), V3(-0.024, -0.06, 0.045), V3(-0.008, -0.13, 0.05)]);
  torch.add(new THREE.Mesh(new THREE.TubeGeometry(cv, 26, 0.0072, 10, false), mCable));
  const cb = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.024, 0.04), mInsul); cb.position.set(0, TB * 0.72, 0.036); torch.add(cb);
  const cl = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.012, 14), new THREE.MeshStandardMaterial({ color: 0x0b1116, emissive: 0x123a2a, emissiveIntensity: .7 }));
  cl.position.set(0, TB * 0.66, 0.056); cl.rotation.x = 1.0; torch.add(cl);
})();

// ---- CAMERA ARM (second cobot) ----
let sysMode = 'single', baseTarget = 'torch', camFeed = 'arm';
let qc = [0, 0.7, 0, -1.2, 0, -0.9, 0];
const CAM_OFFSET = new THREE.Vector3(0, 0, 0.42);
let CAM_BASE = new THREE.Vector3();
const camArmMat = new THREE.MeshStandardMaterial({ color: 0xc9d4dd, metalness: .3, roughness: .5 });
const camJointMat = new THREE.MeshStandardMaterial({ color: 0x1c3038, metalness: .6, roughness: .4 });
const camGroup = new THREE.Group(); world.add(camGroup);
const camPedestal = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.07, 0.06, 28), new THREE.MeshStandardMaterial({ color: 0x202a30, metalness: .5, roughness: .5 })); camGroup.add(camPedestal);
const camArmCyl = [], camArmJoint = [];
for (let i = 0; i < 8; i++) { const c = new THREE.Mesh(new THREE.CylinderGeometry(0.026, 0.026, 1, 20), camArmMat); camGroup.add(c); camArmCyl.push(c); }
for (let i = 0; i < 7; i++) { const j = new THREE.Mesh(new THREE.SphereGeometry(0.034, 18, 14), camJointMat); camGroup.add(j); camArmJoint.push(j); }
const camHead = new THREE.Group(); camGroup.add(camHead);
(function buildCamHead() {
  const mBody = new THREE.MeshStandardMaterial({ color: 0x12181d, metalness: .5, roughness: .5 });
  const mRing = new THREE.MeshStandardMaterial({ color: 0x3a4650, metalness: .7, roughness: .3 });
  const mLens = new THREE.MeshStandardMaterial({ color: 0x081016, emissive: 0x1aa06a, emissiveIntensity: .8, metalness: .4, roughness: .2 });
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.05, 0.06), mBody); body.position.y = 0.025; camHead.add(body);
  const ring = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.02, 20), mRing); ring.position.y = 0.052; camHead.add(ring);
  const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.017, 0.016, 20), mLens); lens.position.y = 0.064; camHead.add(lens);
  const mount = new THREE.Mesh(new THREE.CylinderGeometry(0.024, 0.024, 0.012, 18), mRing); mount.position.y = 0.006; camHead.add(mount);
})();
function setCamArmVisible(v) { camGroup.visible = v; }
setCamArmVisible(false);
function updateCamArm() {
  CAM_BASE.copy(ROBOT_BASE).add(rotOffY(CAM_OFFSET, (robotForm === 'humanoid' && sysMode === 'dual') ? waistYaw : 0));
  camPedestal.position.set(CAM_BASE.x, CAM_BASE.y + 0.03, CAM_BASE.z);
  const f = fk(qc, CAM_BASE, false); const J = f.joints;
  const flange = new THREE.Vector3().setFromMatrixPosition(f.Tf);
  const pts = [CAM_BASE.clone().add(V3(0, 0.06, 0))];
  for (let i = 1; i < J.length; i++) pts.push(J[i].p);
  pts.push(flange);
  for (let i = 0; i < pts.length - 1; i++) orient(camArmCyl[i], pts[i], pts[i + 1], i < 2 ? 0.03 : 0.024);
  for (let i = pts.length - 1; i < camArmCyl.length; i++) camArmCyl[i].visible = false;
  for (let i = 0; i < J.length; i++) camArmJoint[i].position.copy(J[i].p);
  camHead.position.copy(flange); camHead.quaternion.setFromRotationMatrix(f.Tf);
  return f;
}
// ---- unified dual-arm body ----
let robotForm = 'separate';
const torsoMat = new THREE.MeshStandardMaterial({ color: 0x2a3138, metalness: .55, roughness: .5 });
const dualBase = new THREE.Group(); world.add(dualBase); dualBase.visible = false;
const dbPlate = new THREE.Mesh(new THREE.BoxGeometry(0.20, 0.03, 0.20), torsoMat); dualBase.add(dbPlate);
const dbColumn = new THREE.Mesh(new THREE.CylinderGeometry(0.026, 0.026, 1, 22), torsoMat); dualBase.add(dbColumn);
const dbBeam = new THREE.Mesh(new THREE.CylinderGeometry(0.026, 0.026, 1, 18), torsoMat); dualBase.add(dbBeam);
const dbShoT = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.07, 0.06, 28), torsoMat); dualBase.add(dbShoT);
const dbShoC = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.07, 0.06, 28), torsoMat); dualBase.add(dbShoC);
function updateDualBase() {
  const shT = V3(ROBOT_BASE.x, ROBOT_BASE.y + 0.03, ROBOT_BASE.z), shC = V3(CAM_BASE.x, CAM_BASE.y + 0.03, CAM_BASE.z);
  dbShoT.position.copy(shT); dbShoC.position.copy(shC); orient(dbBeam, shT, shC, 0.045);
  const mid = shT.clone().add(shC).multiplyScalar(0.5);
  orient(dbColumn, V3(mid.x, 0.015, mid.z), V3(mid.x, Math.max(0.05, mid.y), mid.z), 0.05);
  dbPlate.position.set(mid.x, 0.015, mid.z);
}
// ---- humanoid upper body ----
const armorMat = new THREE.MeshStandardMaterial({ color: 0xeef1f4, metalness: .35, roughness: .42 });
const frameMat = new THREE.MeshStandardMaterial({ color: 0x23282e, metalness: .6, roughness: .45 });
const glowMat = new THREE.MeshBasicMaterial({ color: 0x6fe6ff });
const visorMat = new THREE.MeshStandardMaterial({ color: 0x0a1216, metalness: .7, roughness: .18 });
const HEAD_EYE = new THREE.Vector3(), HEAD_AIM = new THREE.Vector3(1, 0, 0);
const humanoidBody = new THREE.Group(); world.add(humanoidBody); humanoidBody.visible = false;
const hm = (geo, mat) => { const m = new THREE.Mesh(geo, mat); humanoidBody.add(m); return m; };
const hPelvis = hm(new THREE.BoxGeometry(0.15, 0.08, 0.17), armorMat), hWaist = hm(new THREE.CylinderGeometry(0.045, 0.05, 0.085, 18), frameMat);
const hChest = hm(new THREE.BoxGeometry(0.16, 0.19, 0.24), armorMat), hCore = hm(new THREE.SphereGeometry(0.016, 14, 12), glowMat);
const hNeck = hm(new THREE.CylinderGeometry(0.024, 0.028, 1, 16), frameMat), hHead = hm(new THREE.SphereGeometry(0.072, 24, 18), armorMat);
const hVisor = hm(new THREE.BoxGeometry(0.024, 0.046, 0.108), visorMat), hEye = hm(new THREE.SphereGeometry(0.012, 14, 12), glowMat);
const hBeam = hm(new THREE.CylinderGeometry(0.026, 0.026, 1, 18), frameMat);
const hShoT = hm(new THREE.SphereGeometry(0.062, 20, 16), armorMat), hShoC = hm(new THREE.SphereGeometry(0.062, 20, 16), armorMat);
const hLegL = hm(new THREE.CylinderGeometry(0.03, 0.026, 1, 14), armorMat), hLegR = hm(new THREE.CylinderGeometry(0.03, 0.026, 1, 14), armorMat);
const hFootL = hm(new THREE.BoxGeometry(0.12, 0.03, 0.07), frameMat), hFootR = hm(new THREE.BoxGeometry(0.12, 0.03, 0.07), frameMat);
function updateHumanoid() {
  const shT = V3(ROBOT_BASE.x, ROBOT_BASE.y + 0.03, ROBOT_BASE.z), shC = V3(CAM_BASE.x, CAM_BASE.y + 0.03, CAM_BASE.z);
  hShoT.position.copy(shT); hShoC.position.copy(shC); orient(hBeam, shT, shC, 0.04);
  const mid = shT.clone().add(shC).multiplyScalar(0.5), lz = 0.06;
  const hum = robotForm === 'humanoid' && sysMode === 'dual', p = hum ? torsoPitch : 0, wy = hum ? waistYaw : 0;
  const up = rotOffY(V3(Math.sin(p), Math.cos(p), 0), wy), fwd = rotOffY(V3(Math.cos(p), -Math.sin(p), 0), wy);
  const pel = mid.clone().addScaledVector(up, -HTORSO); if (pel.y < 0.10) pel.y = 0.10;
  const tq = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(fwd, up, new THREE.Vector3().crossVectors(fwd, up).normalize()));
  const cmid = pel.clone().addScaledVector(up, HTORSO * 0.62);
  hChest.position.copy(cmid); hChest.quaternion.copy(tq); hChest.scale.y = Math.max(0.55, pel.distanceTo(mid) / 0.20 * 0.86);
  hWaist.position.copy(pel.clone().addScaledVector(up, 0.085)); hWaist.quaternion.copy(tq);
  hCore.position.copy(cmid).addScaledVector(fwd, 0.082).addScaledVector(up, 0.045);
  hPelvis.position.copy(pel); hPelvis.rotation.set(0, wy, 0);
  orient(hLegL, V3(pel.x, pel.y, pel.z + lz), V3(pel.x, 0.03, pel.z + lz), 0.03);
  orient(hLegR, V3(pel.x, pel.y, pel.z - lz), V3(pel.x, 0.03, pel.z - lz), 0.03);
  hFootL.position.set(pel.x + 0.02, 0.015, pel.z + lz); hFootL.rotation.set(0, wy * 0.4, 0);
  hFootR.position.set(pel.x + 0.02, 0.015, pel.z - lz); hFootR.rotation.set(0, wy * 0.4, 0);
  const headC = mid.clone().addScaledVector(up, 0.175);
  orient(hNeck, mid.clone().addScaledVector(up, 0.035), mid.clone().addScaledVector(up, 0.115), 0.022);
  let f;
  if (headAuto) { f = weldPointNow().sub(headC).normalize(); }
  else { const yT = wy + headYaw, pT = -p + headPit; f = V3(Math.cos(pT) * Math.cos(yT), Math.sin(pT), -Math.cos(pT) * Math.sin(yT)); }
  let u = up.clone().sub(f.clone().multiplyScalar(up.dot(f))); if (u.lengthSq() < 1e-6) u = V3(0, 1, 0); u.normalize();
  const sdv = new THREE.Vector3().crossVectors(f, u).normalize();
  const hq = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(f, u, sdv));
  hHead.position.copy(headC); hHead.quaternion.copy(hq);
  hVisor.position.copy(headC).addScaledVector(f, 0.058).addScaledVector(u, 0.008); hVisor.quaternion.copy(hq);
  hEye.position.copy(headC).addScaledVector(f, 0.071).addScaledVector(u, 0.008);
  HEAD_EYE.copy(headC).addScaledVector(f, 0.076).addScaledVector(u, 0.008); HEAD_AIM.copy(f);
}
function applyFormVisibility() {
  const dual = (sysMode === 'dual'), dualF = (robotForm === 'dual' && dual), humF = (robotForm === 'humanoid' && dual), sepF = (robotForm === 'separate');
  dualBase.visible = dualF; humanoidBody.visible = humF; pedestal.visible = (!dual) || sepF; camPedestal.visible = dual && sepF;
}
function setWorkY(y) { WORK_Y = y; if (typeof WSorbit !== 'undefined') WSorbit.retarget(V3(0.30, 0.10 + y, 0)); weldEval = null; buildWorkpiece(); clearBeads(); poolGroup.visible = false; rebuildSeamLine(); setPhase('idle'); s = 0; }

const arcLight = new THREE.PointLight(0xbfe3ff, 0, 0.5, 2); world.add(arcLight);
const arcDot = new THREE.Mesh(new THREE.SphereGeometry(0.01, 12, 10), new THREE.MeshBasicMaterial({ color: 0xeaf6ff })); arcDot.visible = false; world.add(arcDot);

// ---- molten weld pool ----
const poolGroup = new THREE.Group(); poolGroup.visible = false; world.add(poolGroup);
let poolShapeMode = 'simple', lastPoolField = null;
const POOL_N = 64;
function _fan(color) { const g = new THREE.BufferGeometry();
  const pos = new Float32Array((POOL_N + 2) * 3); const attr = new THREE.BufferAttribute(pos, 3); attr.setUsage(THREE.DynamicDrawUsage); g.setAttribute('position', attr);
  const idx = []; for (let i = 1; i <= POOL_N; i++) idx.push(0, i, i + 1); g.setIndex(idx);
  const m = new THREE.Mesh(g, new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.95, side: THREE.DoubleSide }));
  m.userData.pos = pos; poolGroup.add(m); return m; }
const poolOuter = _fan(0xff6a16), poolMid = _fan(0xffae45), poolCore = _fan(0xfff0c6);
poolOuter.renderOrder = 1; poolMid.renderOrder = 2; poolCore.renderOrder = 3; poolMid.position.z = 0.0004; poolCore.position.z = 0.0008;
function _updFan(mesh, base, scale) { const pos = mesh.userData.pos, pts = base.pts, cen = base.cen;
  pos[0] = cen.x; pos[1] = cen.y; pos[2] = 0;
  for (let i = 0; i <= POOL_N; i++) { const p = pts[i % pts.length], k = (i + 1) * 3; pos[k] = cen.x + (p.x - cen.x) * scale; pos[k + 1] = cen.y + (p.y - cen.y) * scale; pos[k + 2] = 0; }
  mesh.geometry.attributes.position.needsUpdate = true; }
function buildPoolOutline(a, b) { const N = POOL_N, pts = new Array(N);
  if (poolShapeMode === 'detailed' && lastPoolField) {
    const F = lastPoolField, mx = F.xPeak, my = F.cy, Rmax = 20, hot = F.T(mx, my, 0) >= F.Tm;
    for (let k = 0; k < N; k++) { const th = k / N * 2 * Math.PI, dx = Math.cos(th), dy = Math.sin(th); let r = 0;
      if (hot) { let lo = 0, hi = Rmax; for (let it = 0; it < 18; it++) { const m = (lo + hi) / 2; if (F.T(mx + dx * m, my + dy * m, 0) >= F.Tm) lo = m; else hi = m; } r = lo; }
      pts[k] = { x: (mx + dx * r) * 1e-3 - poolDisp.cx, y: (my + dy * r) * 1e-3 - poolDisp.cy }; }
  } else { for (let k = 0; k < N; k++) { const t = k / N * 2 * Math.PI; pts[k] = { x: a * Math.cos(t), y: b * Math.sin(t) }; } }
  let sx = 0, sy = 0; for (const p of pts) { sx += p.x; sy += p.y; } return { pts, cen: { x: sx / N, y: sy / N } }; }
let poolFeat = null, poolEval = null;
let cornerHeat = 0, dwellRemain = 0, prevInCorner = false, passIdx = 0;

// ===== weld-pool adapter (loosely coupled to window.WeldPoolModel) =====
const POSMAP = { F: 'flat', H: 'horiz', VU: 'vertup', VD: 'vertdn' };
function processNow() { return WeldCell.makeProcess({ I: P.I, V: P.V, eta: P.eta, material: CH.material, T0: P.T0, thickness: P.thick / 1000 }); }
function poolParams() {
  return { I: P.I, V: P.V, eta: P.eta, T0: P.T0 + cornerHeat, material: CH.material, grav: 0.6, thicknessMM: P.thick,
    v_mmps: Math.max(weldSpeed * 1000, 0.3), travelDeg: travelAngle / D2R, workDeg: (workAngle - WORKREF) / D2R,
    arcLenMM: Math.max(standoff * 1000, 0.3), position: POSMAP[weldPos] || 'flat', calib: CALIB };
}
let poolSolveAcc = 0;
let poolTarget = { L: 0.010, W: 0.008, cx: -0.003, cy: 0, pen: 2 };
let poolDisp = { L: 0.010, W: 0.008, cx: -0.003, cy: 0, pen: 2 };
function updatePool(Pw, ss, dt) {
  const tan = seamTangent(clamp(ss, 0, 1)), nn = CORNER_OPEN.clone();
  const uu = tan.clone().sub(nn.clone().multiplyScalar(tan.dot(nn))).normalize();
  const vv = new THREE.Vector3().crossVectors(nn, uu).normalize();
  poolSolveAcc += dt;
  if (poolSolveAcc >= 0.10 || !poolEval) { poolSolveAcc = 0; const r = WeldPoolModel.evaluate(poolParams()); poolEval = r;
    if (r.status !== 'no_pool') { poolTarget.L = r.lenMM / 1000; poolTarget.W = r.widMM / 1000; poolTarget.cx = r.cxMM / 1000; poolTarget.cy = r.cyMM / 1000; poolTarget.pen = r.penMM; }
    if (poolShapeMode === 'detailed') lastPoolField = WeldPoolModel.makeField(poolParams()); }
  const kk = Math.min(1, dt * 6);
  poolDisp.L += (poolTarget.L - poolDisp.L) * kk; poolDisp.W += (poolTarget.W - poolDisp.W) * kk;
  poolDisp.cx += (poolTarget.cx - poolDisp.cx) * kk; poolDisp.cy += (poolTarget.cy - poolDisp.cy) * kk; poolDisp.pen += (poolTarget.pen - poolDisp.pen) * kk;
  const a = Math.max(0.001, poolDisp.L / 2), b = Math.max(0.0008, poolDisp.W / 2);
  const fl = 1 + 0.05 * Math.sin(performance.now() * 0.018) + 0.03 * Math.sin(performance.now() * 0.045);
  const Cc = Pw.clone().addScaledVector(uu, poolDisp.cx).addScaledVector(vv, poolDisp.cy).addScaledVector(CORNER_INTO, 0.0015);
  poolGroup.position.copy(Cc); poolGroup.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(uu, vv, nn));
  const base = buildPoolOutline(a, b);
  _updFan(poolOuter, base, fl); _updFan(poolMid, base, 0.8 * fl); _updFan(poolCore, base, 0.5 * fl);
  // drawing uses the flickering outline; MEASURED features use the steady outline + seeded sensor noise
  // (v21 measured the visual flicker itself, which injected ~5 % fake oscillation into AUTO & quality metrics)
  const cen = base.cen, O = base.pts.map(p => ({ x: cen.x + (p.x - cen.x) * fl, y: cen.y + (p.y - cen.y) * fl }));
  const mn = 1 + 0.01 * R.gauss();
  let xmn = 1e9, xmx = -1e9, ymn = 1e9, ymx = -1e9, area2 = 0;
  for (let i = 0; i < base.pts.length; i++) { const p = base.pts[i], q2 = base.pts[(i + 1) % base.pts.length];
    xmn = Math.min(xmn, p.x); xmx = Math.max(xmx, p.x); ymn = Math.min(ymn, p.y); ymx = Math.max(ymx, p.y); area2 += p.x * q2.y - q2.x * p.y; }
  xmn *= mn; xmx *= mn; ymn *= mn; ymx *= mn; area2 *= mn * mn;
  poolFeat = { C: Cc.clone(), u: uu.clone(), v: vv.clone(), a: (xmx - xmn) / 2, b: (ymx - ymn) / 2, outlineLocal: O, shapeMode: poolShapeMode,
    arc: Pw.clone(), lead: Cc.clone().addScaledVector(uu, xmx), toeW: Cc.clone().addScaledVector(vv, ymx), toeB: Cc.clone().addScaledVector(vv, ymn), leadY: xmx * 0.75,
    lenMM: (xmx - xmn) * 1000, widMM: (ymx - ymn) * 1000, areaMM: Math.abs(area2) / 2 * 1e6, penMM: poolDisp.pen };
  updatePoolSection(Pw, uu, vv, nn, dt);
}
// ===== 3D transverse cross-section slice =====
const _secTexCv = document.createElement('canvas'); _secTexCv.width = 88; _secTexCv.height = 60;
const poolSectionTex = new THREE.CanvasTexture(_secTexCv); poolSectionTex.minFilter = THREE.LinearFilter;
const poolSection = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial({ map: poolSectionTex, transparent: true, depthTest: false, depthWrite: false, side: THREE.DoubleSide }));
poolSection.renderOrder = 6; poolSection.visible = false; world.add(poolSection);
let showSection = true, secAcc = 0;
const SEC_YH = 10, SEC_ZMAX = 13;
function updatePoolSection(Pw, uu, vv, nn, dt) {
  if (!showSection) { poolSection.visible = false; return; }
  poolSection.visible = true;
  poolSection.position.copy(Pw).addScaledVector(nn, -(SEC_ZMAX / 2) / 1000);
  poolSection.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(vv, nn, uu));
  poolSection.scale.set((2 * SEC_YH) / 1000, (SEC_ZMAX) / 1000, 1);
  secAcc += dt; if (secAcc < 0.1) return; secAcc = 0;
  const F = WeldPoolModel.makeField(poolParams());
  const gw = _secTexCv.width, gh = _secTexCv.height, o = _secTexCv.getContext('2d');
  const img = o.createImageData(gw, gh), d = img.data, col = [0, 0, 0], Tg = new Float32Array(gw * gh);
  for (let j = 0; j < gh; j++) for (let i = 0; i < gw; i++) {
    const y = -SEC_YH + 2 * SEC_YH * (i + 0.5) / gw, z = SEC_ZMAX * (j + 0.5) / gh;
    const T = F.T(F.xPeak, y, z); Tg[j * gw + i] = T; const k = (j * gw + i) * 4;
    let a = 0; if (T >= F.Tm) a = 235; else if (T >= F.hazT) a = 150; else if (T >= 600) a = 70 * (T - 600) / (F.hazT - 600);
    mpfHot(Math.max(T, 360), col); d[k] = col[0]; d[k + 1] = col[1]; d[k + 2] = col[2]; d[k + 3] = a;
  }
  for (let j = 0; j < gh; j++) for (let i = 0; i < gw; i++) { const T = Tg[j * gw + i], k = (j * gw + i) * 4;
    const Tr = i < gw - 1 ? Tg[j * gw + i + 1] : T, Td = j < gh - 1 ? Tg[(j + 1) * gw + i] : T;
    if ((T >= F.Tm) !== (Tr >= F.Tm) || (T >= F.Tm) !== (Td >= F.Tm)) { d[k] = 255; d[k + 1] = 255; d[k + 2] = 255; d[k + 3] = 255; }
    else if ((T >= F.hazT) !== (Tr >= F.hazT) || (T >= F.hazT) !== (Td >= F.hazT)) { d[k] = 88; d[k + 1] = 216; d[k + 2] = 228; d[k + 3] = 225; } }
  o.putImageData(img, 0, 0); poolSectionTex.needsUpdate = true;
}

// seam highlight + reachability
let seamLine = null, weldEval = null, _lastBaseKey = '';
function rebuildSeamLine() {
  if (seamLine) { world.remove(seamLine); seamLine.geometry.dispose(); }
  const N = 60, pts = [], cols = [];
  for (let i = 0; i <= N; i++) { const u = i / N; pts.push(seamPoint(u).add(frameAt(u).into.clone().multiplyScalar(-0.001)));
    let ok = true; if (weldEval && weldEval.reach) { const r = weldEval.reach; ok = r[Math.min(r.length - 1, Math.round(u * (r.length - 1)))] > 0.5; }
    if (ok) cols.push(0.27, 0.83, 0.60); else cols.push(1.0, 0.32, 0.32); }
  const g = new THREE.BufferGeometry().setFromPoints(pts); g.setAttribute('color', new THREE.Float32BufferAttribute(cols, 3));
  seamLine = new THREE.Line(g, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: .9 })); world.add(seamLine);
}
let dryRun = false;
/** IK feasibility + joint limits along the weld line (ALL 7 joints — v21 checked only 6). */
function evaluateWeldLine() {
  const M = 41, reach = new Float32Array(M), q0 = q.slice();
  const co = CORNER_OPEN.clone(), ci = CORNER_INTO.clone();
  const dl = disturb.lat, dw = disturb.work, dtr = disturb.travel, to = trackOffset, ao = alongOffset;
  disturb.lat = 0; disturb.work = 0; disturb.travel = 0; trackOffset = 0; alongOffset = 0;
  let okN = 0, worstErr = 0;
  for (let i = 0; i < M; i++) { const u = i / (M - 1);
    const fr = frameAt(u); CORNER_OPEN.copy(fr.n); CORNER_INTO.copy(fr.into);
    const tt = torchTarget(u); let err = 1; for (let k = 0, n = (i === 0 ? 150 : 22); k < n; k++) { err = ikStep(tt.p, tt.Rd, ikLambda); if (err < 2e-4 && k > 4) break; }
    let inLim = true; for (let j = 0; j < JD.length; j++) { if (q[j] < LIM[j][0] + 1e-3 || q[j] > LIM[j][1] - 1e-3) { inLim = false; break; } }
    const ok = err < 0.004 && inLim; reach[i] = ok ? 1 : 0; if (ok) okN++; worstErr = Math.max(worstErr, err);
  }
  q = q0.slice(); CORNER_OPEN.copy(co); CORNER_INTO.copy(ci);
  disturb.lat = dl; disturb.work = dw; disturb.travel = dtr; trackOffset = to; alongOffset = ao;
  const ranges = []; let st = -1;
  for (let i = 0; i < M; i++) { if (reach[i] < 0.5) { if (st < 0) st = i; } else if (st >= 0) { ranges.push([st / (M - 1), (i - 1) / (M - 1)]); st = -1; } }
  if (st >= 0) ranges.push([st / (M - 1), 1]);
  weldEval = { reach, frac: okN / M, ranges, worstErr }; rebuildSeamLine();
  return weldEval;
}
function reportWeldEval(ev) {
  if (ev.frac >= 0.999) return '溶接線評価: 全域 到達可（基台↔ワーク配置 OK）';
  const rs = ev.ranges.map(r => `s=${r[0].toFixed(2)}–${r[1].toFixed(2)}`).join(' , ');
  return `溶接線評価: 到達 ${(ev.frac * 100).toFixed(0)}% ／ 到達不可 区間 ${rs}（基台位置を調整してください）`;
}

// weld bead
const beadGroup = new THREE.Group(); world.add(beadGroup);
const beads = [];
let legLen = 0.005;
function addBead(p) {
  const m = new THREE.Mesh(new THREE.SphereGeometry(legLen * 0.6, 10, 8), new THREE.MeshStandardMaterial({ color: 0xffe2a0, emissive: 0xffb84d, emissiveIntensity: 1.4, metalness: .3, roughness: .5 }));
  m.position.copy(p).add(CORNER_OPEN.clone().multiplyScalar(legLen * 0.25)); beadGroup.add(m); beads.push({ mesh: m, t0: performance.now() });
  if (beads.length > 1800) { const b = beads.shift(); beadGroup.remove(b.mesh); b.mesh.geometry.dispose(); b.mesh.material.dispose(); }
}
function clearBeads() { beads.forEach(b => { beadGroup.remove(b.mesh); b.mesh.geometry.dispose(); b.mesh.material.dispose(); }); beads.length = 0; }
function coolBeads(now) {
  for (const b of beads) { const age = (now - b.t0) / 1000; if (age > 3) continue; const k = clamp(1 - age / 2.2, 0, 1), mat = b.mesh.material;
    mat.emissiveIntensity = 0.05 + 1.5 * k; mat.emissive.setRGB(0.35 + 0.65 * k, 0.18 + 0.5 * k, 0.05 + 0.15 * k); mat.color.setRGB(0.45 + 0.3 * k, 0.42 + 0.25 * k, 0.42 + 0.18 * k); }
}
const sparkN = 70, sparkGeo = new THREE.BufferGeometry(), sparkPos = new Float32Array(sparkN * 3);
sparkGeo.setAttribute('position', new THREE.BufferAttribute(sparkPos, 3));
world.add(new THREE.Points(sparkGeo, new THREE.PointsMaterial({ color: 0xffd27a, size: 0.006, transparent: true, opacity: 0.9 })));
const spk = []; for (let i = 0; i < sparkN; i++) spk.push({ p: V3(0, -1, 0), v: V3(0, 0, 0), life: 0 });
function emitSparks(p) { for (let i = 0; i < 6; i++) { const sp = spk[Math.floor(Math.random() * sparkN)]; sp.p.copy(p); sp.v.set((Math.random() - .5) * 0.6, Math.random() * 0.7, (Math.random() - .5) * 0.6); sp.life = 0.18 + Math.random() * 0.18; } }
function updateSparks(dt) { for (let i = 0; i < sparkN; i++) { const sp = spk[i]; if (sp.life > 0) { sp.life -= dt; sp.v.y -= 2.8 * dt; sp.p.addScaledVector(sp.v, dt); } else sp.p.set(0, -1, 0);
  sparkPos[i * 3] = sp.p.x; sparkPos[i * 3 + 1] = sp.p.y; sparkPos[i * 3 + 2] = sp.p.z; } sparkGeo.attributes.position.needsUpdate = true; }

// ---- orbit camera ----
function orbit(cam, el, target, r, theta, phi) {
  const st = { r, theta, phi }; let mode = 0, px = 0, py = 0;
  function upd() { cam.position.set(target.x + st.r * Math.sin(st.phi) * Math.cos(st.theta), target.y + st.r * Math.cos(st.phi), target.z + st.r * Math.sin(st.phi) * Math.sin(st.theta)); cam.lookAt(target.x, target.y, target.z); }
  function pan(dx, dy) { const fwd = new THREE.Vector3().subVectors(target, cam.position).normalize(), right = new THREE.Vector3().crossVectors(fwd, V3(0, 1, 0)).normalize(),
    up = new THREE.Vector3().crossVectors(right, fwd).normalize(), k = st.r * 0.0016; target.addScaledVector(right, -dx * k).addScaledVector(up, dy * k); }
  el.addEventListener('contextmenu', e => e.preventDefault());
  el.addEventListener('pointerdown', e => { mode = (e.button === 2 || e.button === 1 || e.shiftKey) ? 2 : 1; px = e.clientX; py = e.clientY; el.setPointerCapture(e.pointerId); });
  el.addEventListener('pointermove', e => { if (!mode) return; const dx = e.clientX - px, dy = e.clientY - py; px = e.clientX; py = e.clientY;
    if (mode === 1) { st.theta -= dx * 0.006; st.phi = clamp(st.phi - dy * 0.006, 0.08, Math.PI - 0.08); } else pan(dx, dy); upd(); });
  el.addEventListener('pointerup', () => mode = 0); el.addEventListener('pointercancel', () => mode = 0);
  el.addEventListener('wheel', e => { e.preventDefault(); st.r = clamp(st.r * (1 + e.deltaY * 0.0012), 0.3, 4); upd(); }, { passive: false });
  let home = target.clone();
  upd(); return { upd, reset: () => { st.r = r; st.theta = theta; st.phi = phi; target.copy(home); upd(); },
    retarget: (t) => { home = t.clone(); target.copy(t); upd(); } };
}
const WSorbit = orbit(WS.cam, $('wsCanvas'), V3(0.30, 0.10, 0), 0.95, 0.7, 1.05);

// ============================ control / state ============================
let phase = 'idle', s = 0;
let weldSpeed = 0.007, lookAhead = 0.018, seamNoise = 0.003;
let camDist = 0.095, camElev = 50 * D2R, camAzim = 22 * D2R;
let lastErr = 0, trackOffset = 0, alongOffset = 0;
let camMode = 'nir', standoff = 0.002;
let disturb = { lat: 0, work: 0, travel: 0 };
let target = { set: false, A: 0, L: 0, W: 0 };
const clock = new THREE.Clock();

function updateArm() {
  const f = fk(q), J = f.joints, B = teBasis(f.Te), flange = new THREE.Vector3().setFromMatrixPosition(f.Tf);
  const pts = [ROBOT_BASE.clone().add(V3(0, 0.06, 0))];
  for (let i = 1; i < J.length; i++) pts.push(J[i].p);
  pts.push(flange);
  for (let i = 0; i < pts.length - 1; i++) orient(armCyl[i], pts[i], pts[i + 1], i < 2 ? 0.03 : 0.024);
  for (let i = pts.length - 1; i < armCyl.length; i++) armCyl[i].visible = false;
  for (let i = 0; i < J.length; i++) armJoint[i].position.copy(J[i].p);
  torch.position.copy(flange); torch.quaternion.setFromRotationMatrix(f.Tf);
  return B;
}
const lastTip = new THREE.Vector3();
function placeTorchCam() {
  const ss = clamp(s, 0, 1), B = teBasis(fk(q).Te), appr = B.y.clone().normalize(), tip = B.p.clone(); lastTip.copy(tip);
  const weld = weldOnSurface(tip, appr, ss), N = CORNER_OPEN, F = seamTangent(ss), Sd = new THREE.Vector3().crossVectors(N, F).normalize();
  const horiz = F.clone().multiplyScalar(-Math.cos(camAzim)).addScaledVector(Sd, Math.sin(camAzim));
  const dir = N.clone().multiplyScalar(Math.sin(camElev)).addScaledVector(horiz, Math.cos(camElev)).normalize();
  const camPos = weld.clone().addScaledVector(dir, camDist), aim = weld.clone().addScaledVector(F, lookAhead * 0.30);
  const viewDir = aim.clone().sub(camPos).normalize();
  let up = F.clone().addScaledVector(viewDir, -F.dot(viewDir)); if (up.lengthSq() < 1e-6) up = N.clone(); up.normalize();
  CAM.cam.position.copy(camPos); CAM.cam.up.copy(up); CAM.cam.lookAt(aim.x, aim.y, aim.z);
  const poolA = poolFeat ? poolFeat.a : 0.006, back = weld.clone().addScaledVector(F, -(poolA * 2.4)), ahead = weld.clone().addScaledVector(F, lookAhead);
  let rad = 0.012; [tip, weld, back, ahead].forEach(p => rad = Math.max(rad, p.distanceTo(aim)));
  const dist = Math.max(0.02, camPos.distanceTo(aim));
  CAM.cam.fov = clamp(2 * Math.atan((rad * 1.35) / dist) * 180 / Math.PI * (2 / P.zoom), 12, 82); CAM.cam.updateProjectionMatrix();
}
let camArmErr = 0, visionState = null, visionAcc = 0, camInterf = { hit: false, minClear: 1 }, camViewSel = null;
function weldPointNow() {
  const ss = clamp(s, 0, 1); if (poolFeat && poolGroup.visible) return poolFeat.arc.clone();
  const B = teBasis(fk(q).Te); return weldOnSurface(B.p.clone(), B.y.clone().normalize(), ss);
}
function segDist(p1, q1, p2, q2) {
  const d1 = q1.clone().sub(p1), d2 = q2.clone().sub(p2), r = p1.clone().sub(p2);
  const a = d1.dot(d1), e = d2.dot(d2), f = d2.dot(r); let s2, t2; const EPS = 1e-9;
  if (a < EPS && e < EPS) return r.length();
  if (a < EPS) { s2 = 0; t2 = clamp(f / e, 0, 1); }
  else { const c = d1.dot(r);
    if (e < EPS) { t2 = 0; s2 = clamp(-c / a, 0, 1); }
    else { const b = d1.dot(d2), den = a * e - b * b; s2 = den > EPS ? clamp((b * f - c * e) / den, 0, 1) : 0; t2 = (b * s2 + f) / e;
      if (t2 < 0) { t2 = 0; s2 = clamp(-c / a, 0, 1); } else if (t2 > 1) { t2 = 1; s2 = clamp((b - c) / a, 0, 1); } } }
  return p1.clone().addScaledVector(d1, s2).distanceTo(p2.clone().addScaledVector(d2, t2));
}
function armSegments(f, base, withTip, tipPose) {
  const J = f.joints, flange = new THREE.Vector3().setFromMatrixPosition(f.Tf), pts = [base.clone().add(V3(0, 0.06, 0))];
  for (let i = 1; i < J.length; i++) pts.push(J[i].p); pts.push(flange);
  const segs = []; for (let i = 0; i < pts.length - 1; i++) segs.push({ a: pts[i], b: pts[i + 1], r: i < 2 ? 0.03 : 0.024 });
  if (withTip) segs.push({ a: flange, b: new THREE.Vector3().setFromMatrixPosition(tipPose || f.Te), r: 0.02 });
  return segs;
}
function armsInterfere(margin) {
  const ft = fk(q, ROBOT_BASE, true), fc = fk(qc, CAM_BASE, false);
  const segT = armSegments(ft, ROBOT_BASE, true, ft.Te), flC = new THREE.Vector3().setFromMatrixPosition(fc.Tf);
  const camTip = flC.clone().addScaledVector(new THREE.Vector3().setFromMatrixColumn(fc.Tf, 1).normalize(), 0.07);
  const segC = armSegments(fc, CAM_BASE, false).concat([{ a: flC, b: camTip, r: 0.03 }]);
  let minClear = 1e9, hit = false;
  for (let i = 2; i < segT.length; i++) for (let j = 2; j < segC.length; j++) {
    const d = segDist(segT[i].a, segT[i].b, segC[j].a, segC[j].b) - (segT[i].r + segC[j].r);
    if (d < minClear) minClear = d; if (d < margin) hit = true; }
  return { hit, minClear };
}
function viewpointPose(Wp, az, el, dist) {
  const ss = clamp(s, 0, 1), N = CORNER_OPEN, F = seamTangent(ss), Sd = new THREE.Vector3().crossVectors(N, F).normalize();
  const horiz = F.clone().multiplyScalar(-Math.cos(az)).addScaledVector(Sd, Math.sin(az));
  const dir = N.clone().multiplyScalar(Math.sin(el)).addScaledVector(horiz, Math.cos(el)).normalize();
  const camPos = Wp.clone().addScaledVector(dir, dist), yv = Wp.clone().sub(camPos).normalize();
  let xv = F.clone(); xv.sub(yv.clone().multiplyScalar(xv.dot(yv))); if (xv.lengthSq() < 1e-6) xv = V3(1, 0, 0); xv.normalize();
  return { camPos, Rd: new THREE.Matrix4().makeBasis(xv, yv, new THREE.Vector3().crossVectors(xv, yv).normalize()) };
}
function tryViewpoint(Wp, az, el, dist, seed) {
  qc = seed.slice(); const vp = viewpointPose(Wp, az, el, dist);
  let err = 1; for (let k = 0; k < 10; k++) err = ikSolve(qc, CAM_BASE, false, vp.camPos, vp.Rd, ikLambda);
  const intf = armsInterfere(0.012), Bc = teBasis(fk(qc, CAM_BASE, false).Te);
  const graze = Math.abs(Wp.clone().sub(Bc.p).normalize().dot(CORNER_OPEN)), reach = err < 0.012;
  return { qc: qc.slice(), err, intf, graze, reach, ok: reach && !intf.hit, az, el, dist };
}
function parkCamArm() {
  CAM_BASE.copy(ROBOT_BASE).add(rotOffY(CAM_OFFSET, (robotForm === 'humanoid' && sysMode === 'dual') ? waistYaw : 0));
  const sgz = (CAM_OFFSET.z >= 0 ? 1 : -1), tgt = CAM_BASE.clone().add(V3(-0.06, 0.34, sgz * 0.30));
  const yv = V3(0, -1, 0.0001).normalize(), xv = new THREE.Vector3().crossVectors(yv, V3(1, 0, 0)).normalize();
  const Rd = new THREE.Matrix4().makeBasis(xv, yv, new THREE.Vector3().crossVectors(xv, yv));
  const seed = qc.slice(); let err = 1, best = null;
  for (let trial = 0; trial < 3; trial++) {
    qc = (trial === 0 ? seed : [0, 0.5, 0, -1.4, 0, -1.1, 0]).slice();
    for (let k = 0; k < 12; k++) err = ikSolve(qc, CAM_BASE, false, tgt.clone().addScaledVector(V3(0, 1, 0), trial * 0.05), Rd, ikLambda);
    const intf = armsInterfere(0.02), cand = { qc: qc.slice(), intf, err };
    if (!intf.hit) { best = cand; break; } if (!best || cand.intf.minClear > best.intf.minClear) best = cand;
  }
  qc = best.qc; camInterf = best.intf; camArmErr = best.err; camViewSel = null;
  camArmMat.color.setHex(best.intf.hit ? 0xff5a5a : 0x8a96a0); return updateCamArm();
}
function driveCamArm() {
  CAM_BASE.copy(ROBOT_BASE).add(rotOffY(CAM_OFFSET, (robotForm === 'humanoid' && sysMode === 'dual') ? waistYaw : 0));
  const Wp = weldPointNow(), seed = qc.slice(), azB = 0.55 * (CAM_OFFSET.z >= 0 ? 1 : -1), sg = azB >= 0 ? 1 : -1;
  const bAz = camAzim + azB, bEl = camElev, bD = camDist * 1.7, cands = [];
  if (camViewSel) cands.push([camViewSel.az, camViewSel.el, camViewSel.dist]);
  cands.push([bAz, bEl, bD], [bAz + sg * 0.35, bEl + 0.18, bD], [bAz + sg * 0.7, bEl + 0.32, bD * 1.1], [bAz + sg * 0.7, bEl + 0.5, bD * 1.18], [bAz - sg * 0.3, bEl + 0.3, bD * 1.12],
    [bAz + sg * 1.05, bEl + 0.55, bD * 1.25], [bAz + sg * 0.35, bEl + 0.7, bD * 1.3], [bAz + sg * 1.3, bEl + 0.4, bD * 1.3], [bAz - sg * 0.6, bEl + 0.55, bD * 1.22],
    [bAz + sg * 0.9, bEl + 0.85, bD * 1.4], [bAz + sg * 1.5, bEl + 0.7, bD * 1.45]);
  let best = null;
  for (const [az, el, dist] of cands) { const r = tryViewpoint(Wp, az, el, dist, seed); if (r.ok) { best = r; break; }
    const sc = v => v.graze - (v.intf.hit ? 1 : 0) - (v.reach ? 0 : 0.5) + v.intf.minClear; if (!best || sc(r) > sc(best)) best = r; }
  qc = best.qc; camArmErr = best.err; camInterf = best.intf; camViewSel = best.ok ? { az: best.az, el: best.el, dist: best.dist } : null;
  camArmMat.color.setHex(best.intf.hit ? 0xff5a5a : 0xc9d4dd); return updateCamArm();
}
function placeCamera() {
  if (sysMode === 'dual') {
    setCamArmVisible(true);
    const f = (camFeed === 'arm') ? driveCamArm() : parkCamArm();
    if (camFeed === 'arm') {
      const Bc = teBasis(f.Te), eye = Bc.p.clone().addScaledVector(Bc.y, 0.075); lastTip.copy(teBasis(fk(q).Te).p);
      CAM.cam.position.copy(eye); CAM.cam.up.copy(Bc.x); const aim = eye.clone().addScaledVector(Bc.y, 1.0); CAM.cam.lookAt(aim.x, aim.y, aim.z);
      const Wp = weldPointNow(), poolA = poolFeat ? poolFeat.a : 0.006, rad = Math.max(0.018, poolA * 3 + 0.012), dist = Math.max(0.03, eye.distanceTo(Wp));
      CAM.cam.fov = clamp(2 * Math.atan((rad * 1.4) / dist) * 180 / Math.PI * (2 / P.zoom), 12, 82); CAM.cam.updateProjectionMatrix();
    } else if (camFeed === 'head') {
      CAM.cam.position.copy(HEAD_EYE); CAM.cam.up.set(0, 1, 0); const aim = HEAD_EYE.clone().add(HEAD_AIM); CAM.cam.lookAt(aim.x, aim.y, aim.z);
      lastTip.copy(teBasis(fk(q).Te).p);
      const Wp = weldPointNow(), poolA = poolFeat ? poolFeat.a : 0.006, rad = Math.max(0.02, poolA * 3 + 0.012), dist = Math.max(0.05, HEAD_EYE.distanceTo(Wp));
      CAM.cam.fov = clamp(2 * Math.atan((rad * 1.4) / dist) * 180 / Math.PI * (2 / P.zoom), 12, 82); CAM.cam.updateProjectionMatrix();
    } else placeTorchCam();
  } else { setCamArmVisible(false); placeTorchCam(); }
}
function visionEstimate() {
  const B = teBasis(fk(q).Te), tip = B.p.clone(), Wp = weldPointNow(), N = CORNER_OPEN;
  const standTrue = tip.distanceTo(Wp) * 1000, workTrue = workAngle / D2R, travelTrue = travelAngle / D2R;
  const graze = Math.abs(Wp.clone().sub(CAM.cam.position).normalize().dot(N));
  const tipPx = project(tip, CAM.cam, 100, 100), poolPx = project(Wp, CAM.cam, 100, 100);
  const inFov = p => p && p.x >= -10 && p.x <= 110 && p.y >= -10 && p.y <= 110, visible = inFov(tipPx) && inFov(poolPx);
  const ns = visible ? (0.25 + 0.85 * (1 - graze)) : 5.0, g = () => (R() * 2 - 1);
  return { visible, graze, ns, tipTrue: tip, dTip: ns * 0.6, standTrue, standEst: standTrue + g() * ns * 0.5,
    workTrue, workEst: workTrue + g() * ns * 0.45, travelTrue, travelEst: travelTrue + g() * ns * 0.45 };
}
const _v = new THREE.Vector3();
function project(pt, cam, w, h) {
  _v.copy(pt); const cs = _v.clone().applyMatrix4(cam.matrixWorldInverse); if (cs.z > -0.002) return null;
  _v.project(cam); return { x: (_v.x * 0.5 + 0.5) * w, y: (-_v.y * 0.5 + 0.5) * h };
}
const ov = $('camOverlay'), octx = ov.getContext('2d');
const FILT = { normal: 'none', nir: 'grayscale(1) contrast(1.55) brightness(0.92) sepia(0.28) hue-rotate(-12deg)',
  pool: 'grayscale(1) contrast(1.75) brightness(0.85) sepia(0.32) hue-rotate(-12deg)', seam: 'saturate(1.08)' };
function applyCamMode() { $('camCanvas').style.filter = FILT[camMode] || 'none'; }
function hudText() {
  const lbl = { normal: 'RAW', nir: 'NIR 950nm', pool: 'POOL EDGES', seam: 'SEAM TRACK' }[camMode] || camMode;
  let t = `<span class="lock">● ${lbl}</span>`;
  if (sysMode === 'dual') t += ` <span class="lock" style="color:#7fd0ff">● ${camFeed === 'arm' ? 'CAM-ARM' : camFeed === 'head' ? 'HEAD-CAM' : 'TORCH-CAM'}</span>`;
  t += '<br>';
  if (camMode === 'seam') t += `work ${(workAngle / D2R).toFixed(0)}° · travel ${(travelAngle / D2R).toFixed(0)}°<br>speed ${(weldSpeed * 1000).toFixed(1)} mm/s · prog ${Math.round(clamp(s, 0, 1) * 100)}%`;
  else if ((camMode === 'pool' || camMode === 'nir') && poolFeat && poolGroup.visible) t += `pool ${poolFeat.lenMM.toFixed(1)}×${poolFeat.widMM.toFixed(1)} mm<br>LeadY ${(poolFeat.leadY * 1000).toFixed(1)} mm · ND+950nm`;
  else t += `speed ${(weldSpeed * 1000).toFixed(1)} mm/s · prog ${Math.round(clamp(s, 0, 1) * 100)}%`;
  if (sysMode === 'dual' && visionState) { const v = visionState;
    t += `<br><span style="color:${v.visible ? '#7fffb0' : '#ff7a7a'}">画像推定: ${v.visible ? '視認 良好' : '視認 不可'}</span> standoff ${v.standEst.toFixed(1)} mm (真 ${v.standTrue.toFixed(1)})`;
    t += `<br><span style="color:${camInterf.hit ? '#ff5a5a' : '#7fd0ff'}">アーム干渉: ${camInterf.hit ? 'あり (回避中)' : 'なし'} · 余裕 ${(camInterf.minClear * 1000).toFixed(0)} mm</span>`; }
  return t;
}
function drawSeamLine(w, h, alpha) {
  const poly = []; for (let i = 0; i <= 60; i++) { const pr = project(seamPoint(i / 60), CAM.cam, w, h); if (pr) poly.push(pr); }
  if (poly.length > 1) { octx.lineWidth = 2; octx.strokeStyle = 'rgba(70,211,154,' + alpha + ')'; octx.beginPath(); poly.forEach((p, i) => i ? octx.lineTo(p.x, p.y) : octx.moveTo(p.x, p.y)); octx.stroke(); }
}
function drawSeamOverlay(w, h) {
  drawSeamLine(w, h, 0.35);
  const sLook = clamp(s + lookAhead / SEAMLEN, 0, 1), noisy = seamPoint(sLook).add(CORNER_INTO.clone().multiplyScalar((R() - .5) * seamNoise));
  const tp = project(noisy, CAM.cam, w, h);
  if (tp) { octx.strokeStyle = 'rgba(255,177,59,0.95)'; octx.lineWidth = 1.5; octx.beginPath(); octx.arc(tp.x, tp.y, 11, 0, 7); octx.stroke();
    octx.beginPath(); octx.moveTo(tp.x - 16, tp.y); octx.lineTo(tp.x + 16, tp.y); octx.moveTo(tp.x, tp.y - 16); octx.lineTo(tp.x, tp.y + 16); octx.stroke();
    octx.fillStyle = 'rgba(255,177,59,0.95)'; octx.font = '10px IBM Plex Mono'; octx.fillText('SEAM', tp.x + 14, tp.y - 8); }
  const cp = project((poolFeat && poolGroup.visible) ? poolFeat.arc : seamPoint(clamp(s, 0, 1)), CAM.cam, w, h);
  if (cp && phase === 'weld') { octx.fillStyle = 'rgba(191,227,255,0.9)'; octx.beginPath(); octx.arc(cp.x, cp.y, 5, 0, 7); octx.fill(); }
}
function drawPoolOverlay(w, h, full) {
  if (!poolFeat || !poolGroup.visible) return;
  const F = poolFeat, pts = [], O = F.outlineLocal || [];
  for (let i = 0; i <= O.length; i++) { const p = O[i % O.length]; const pr = project(F.C.clone().addScaledVector(F.u, p.x).addScaledVector(F.v, p.y), CAM.cam, w, h); if (pr) pts.push(pr); }
  if (pts.length > 2) { octx.lineWidth = 2; octx.strokeStyle = 'rgba(70,211,154,0.95)'; octx.beginPath(); pts.forEach((p, i) => i ? octx.lineTo(p.x, p.y) : octx.moveTo(p.x, p.y)); octx.closePath(); octx.stroke();
    octx.fillStyle = 'rgba(70,211,154,0.10)'; octx.fill(); }
  function mark(wp, c, lbl) { const pr = project(wp, CAM.cam, w, h); if (!pr) return null; octx.fillStyle = c; octx.beginPath(); octx.arc(pr.x, pr.y, 4, 0, 7); octx.fill();
    if (lbl) { octx.font = '10px IBM Plex Mono'; octx.fillText(lbl, pr.x + 6, pr.y - 5); } return pr; }
  const aPr = mark(F.arc, 'rgba(191,227,255,0.95)', full ? 'arc' : null), lPr = mark(F.lead, 'rgba(255,177,59,0.95)', full ? 'lead edge' : null);
  if (full) { mark(F.toeW, 'rgba(255,120,90,0.95)', 'toe'); mark(F.toeB, 'rgba(255,120,90,0.95)', 'toe');
    if (aPr && lPr) { octx.strokeStyle = 'rgba(255,177,59,0.85)'; octx.setLineDash([4, 3]); octx.beginPath(); octx.moveTo(aPr.x, aPr.y); octx.lineTo(lPr.x, lPr.y); octx.stroke(); octx.setLineDash([]); } }
}
function drawTip(w, h) {
  const tp = project(lastTip, CAM.cam, w, h), ap = (poolFeat && poolGroup.visible) ? project(poolFeat.arc, CAM.cam, w, h) : null;
  if (tp && ap) { octx.strokeStyle = 'rgba(220,235,255,0.85)'; octx.lineWidth = 1.5; octx.beginPath(); octx.moveTo(tp.x, tp.y); octx.lineTo(ap.x, ap.y); octx.stroke(); }
  if (tp) { octx.fillStyle = 'rgba(190,225,255,0.95)'; octx.fillRect(tp.x - 3, tp.y - 3, 6, 6); octx.font = '10px IBM Plex Mono'; octx.textAlign = 'left'; octx.fillText('wire', tp.x + 6, tp.y - 4); }
}
function drawOverlay() {
  const w = ov.width, h = ov.height; octx.clearRect(0, 0, w, h);
  if (camMode === 'seam') drawSeamOverlay(w, h);
  else if (camMode === 'pool') { drawSeamLine(w, h, 0.30); drawPoolOverlay(w, h, true); drawTip(w, h); }
  else if (camMode === 'nir') { drawPoolOverlay(w, h, false); drawSeamLine(w, h, 0.30); drawTip(w, h); }
}
function drawFillet() {
  const svg = $('filletSvg'); if (!svg) return; const wa = Math.round(workAngle / D2R);
  if (jointType === 'fillet') {
    const cx = 120, cy = 110, leg = 46, torchLen = 70, tx = cx - torchLen * Math.sin(wa * D2R), ty = cy - torchLen * Math.cos(wa * D2R);
    svg.innerHTML = `<rect x="40" y="110" width="160" height="14" fill="#8a97a3"/><rect x="106" y="40" width="14" height="70" fill="#7a8893"/>
      <path d="M120 110 L120 ${110 - leg} L${120 - leg} 110 Z" fill="#ffb13b" opacity="0.85"/>
      <line x1="${cx}" y1="${cy}" x2="${tx}" y2="${ty}" stroke="#46d39a" stroke-width="5" stroke-linecap="round"/><circle cx="${cx}" cy="${cy}" r="3" fill="#bfe3ff"/>
      <text x="150" y="100" fill="#7e8c9a" font-family="IBM Plex Mono" font-size="10">web</text><text x="150" y="134" fill="#7e8c9a" font-family="IBM Plex Mono" font-size="10">base</text>
      <text x="46" y="58" fill="#46d39a" font-family="IBM Plex Mono" font-size="11">隅肉 φw=${wa}&#176;</text>`;
  } else {
    const cx = 120, cy = 104, L = 70, dev = (wa - 90) * D2R, tx = cx + L * Math.sin(dev), ty = cy - L * Math.cos(dev);
    const plates = jointType === 'bop' ? `<rect x="44" y="104" width="152" height="18" fill="#8a97a3"/>`
      : `<rect x="44" y="104" width="70" height="16" fill="#8a97a3"/><rect x="126" y="104" width="70" height="16" fill="#8a97a3"/><path d="M114 104 L120 112 L126 104 Z" fill="#5c6670"/>`;
    svg.innerHTML = `${plates}<line x1="${cx}" y1="${cy}" x2="${tx}" y2="${ty}" stroke="#46d39a" stroke-width="5" stroke-linecap="round"/>
      <line x1="${cx}" y1="${cy}" x2="${cx}" y2="${cy - L}" stroke="#3a4a57" stroke-width="1" stroke-dasharray="3 3"/><circle cx="${cx}" cy="${cy}" r="3" fill="#bfe3ff"/>
      <text x="44" y="60" fill="#46d39a" font-family="IBM Plex Mono" font-size="11">${jointType === 'bop' ? 'ビードオンプレート' : '突き合わせ'} φw=${wa}&#176;</text>`;
  }
}
function buildJbars() { const c = $('jbars'); if (!c) return; c.innerHTML = '';
  for (let i = 0; i < JD.length; i++) c.insertAdjacentHTML('beforeend', `<span>J${i + 1}</span><div class="jbar"><i id="jb${i}"></i></div><span id="jt${i}">0&#176;</span>`); }
function updateJbars() { for (let i = 0; i < JD.length; i++) { const lo = LIM[i][0], hi = LIM[i][1], f = (q[i] - lo) / (hi - lo), el = $('jb' + i); if (!el) return;
  el.style.left = '2%'; el.style.width = Math.max(3, f * 96) + '%'; el.classList.toggle('lim', f < 0.04 || f > 0.96); $('jt' + i).textContent = Math.round(q[i] / D2R) + '°'; } }
function poseMetrics() {
  const f = fk(q), B = teBasis(f.Te), ss = clamp(s, 0, 1), Pw = seamPoint(ss), tan = seamTangent(ss);
  const a = B.y.clone().normalize(), vlat = new THREE.Vector3().crossVectors(CORNER_OPEN, tan).normalize();
  const tipWork = B.p.clone().addScaledVector(a, standoff), d = tipWork.sub(Pw);
  const distMM = d.dot(vlat) * 1000, travelA = Math.asin(clamp(a.dot(tan), -1, 1)) / D2R;
  const aPerp = a.clone().sub(tan.clone().multiplyScalar(a.dot(tan))); aPerp.normalize();
  const dev = Math.acos(clamp(aPerp.dot(CORNER_INTO), -1, 1)) / D2R;
  const sgn = Math.sign(new THREE.Vector3().crossVectors(CORNER_INTO, aPerp).dot(tan)) || 1;
  return { distMM, travelA, workA: (WORKREF / D2R) + sgn * dev };
}

/* ======================= CONTRACT ADAPTERS (live cell) =======================
 * The live cell speaks the same contract as SurrogateRobotCell / RealRobotCell.
 * ========================================================================== */
function liveLimits() {
  return Object.assign({}, WeldCell.DEFAULT_LIMITS, { travelSpeed: [PARAMS.speed.min / 1000, PARAMS.speed.max / 1000],
    workDev: [PARAMS.work.min * D2R - WORKREF, PARAMS.work.max * D2R - WORKREF], travelAngle: [PARAMS.travel.min * D2R, PARAMS.travel.max * D2R],
    standoff: [PARAMS.height.min / 1000, PARAMS.height.max / 1000], wristRoll: [PARAMS.wrRoll.min * D2R, PARAMS.wrRoll.max * D2R], lateralOffset: [-0.03, 0.03] });
}
function liveWeldInput() {
  const ss = clamp(s, 0, 1), tan = seamTangent(ss), n = CORNER_OPEN.clone(), vl = new THREE.Vector3().crossVectors(n, tan).normalize();
  const wp = poolFeat && poolGroup.visible ? poolFeat.arc : seamPoint(ss);
  return { weldPoint: [wp.x, wp.y, wp.z], travelDir: [tan.x, tan.y, tan.z], acrossDir: [vl.x, vl.y, vl.z], surfaceNormal: [n.x, n.y, n.z], gravityDir: [0, -1, 0],
    travelSpeed: weldSpeed, workAngle, workDev: workAngle - WORKREF, travelAngle, standoff, weldPosition: weldPos,
    process: processNow(), seamParam: ss, contact: phase === 'weld' && !dryRun };
}
let _prevPose = null;
function liveRobotState(dt) {
  const ss = clamp(s, 0, 1), B = teBasis(fk(q).Te), tip = B.p.clone(), appr = B.y.clone().normalize();
  const Pw = seamPoint(ss), tan = seamTangent(ss), n = CORNER_OPEN.clone(), vl = new THREE.Vector3().crossVectors(n, tan).normalize();
  const wp = weldOnSurface(tip.clone(), appr, ss), d = tip.clone().sub(Pw), m = poseMetrics();
  const pose = [d.dot(tan), d.dot(vl), d.dot(n), wristRoll, m.travelA * D2R, m.workA * D2R - WORKREF];
  const vel = (_prevPose && dt > 0) ? pose.map((v, i) => (v - _prevPose[i]) / dt) : [0, 0, 0, 0, 0, 0]; _prevPose = pose;
  const lat = wp.clone().sub(Pw).dot(vl), along = wp.clone().sub(Pw).dot(tan);
  return { q: q.slice(), tcp: [tip.x, tip.y, tip.z], pose, vel, deviation: [along, -lat, d.dot(n) - standoff], ikError: lastErr, phase, seamParam: ss,
    trackError: -lat, lateralOffset: trackOffset, wristRoll, weldInput: liveWeldInput() };
}
function livePoolObs() {
  if (!poolFeat || !poolGroup.visible || !poolEval) return WeldCell.emptyObs();
  return { t: 0, L: poolFeat.lenMM / 1000, W: poolFeat.widMM / 1000, penetration: poolDisp.pen / 1000, area: poolFeat.areaMM / 1e6,
    centroidAlong: poolDisp.cx, centroidAcross: poolDisp.cy, peakT: poolEval.peakC, t85: poolEval.t85, status: poolEval.status };
}
/** Apply a ControlAction (absolute and/or deltas) to the live cell through the shared semantics. */
function applyLiveAction(a, opts = {}) {
  if (!a) return;
  const sp = { travelSpeed: weldSpeed, workRef: WORKREF, workAngle, travelAngle, standoff, lateralOffset: trackOffset, wristRoll, along: alongOffset };
  WeldCell.applyAction(sp, a, liveLimits());
  weldSpeed = sp.travelSpeed; workAngle = sp.workAngle; travelAngle = sp.travelAngle; standoff = sp.standoff; trackOffset = sp.lateralOffset;
  wristRoll = sp.wristRoll; alongOffset = sp.along || 0;
  reflectSetpoints(opts.auto !== false);
}
/** Show the ACTUAL (controller-driven) setpoints in the rows without touching the nominal conditions P. */
function reflectSetpoints(auto) {
  showActual('speed', weldSpeed * 1000, auto); showActual('work', workAngle / D2R, auto); showActual('travel', travelAngle / D2R, auto);
  showActual('height', standoff * 1000, auto); showActual('wrRoll', wristRoll / D2R, auto); drawFillet();
}
/** Return the live setpoints to the nominal conditions P (after a weld / on reset). */
function restoreNominal() {
  weldSpeed = P.speed / 1000; workAngle = P.work * D2R; travelAngle = P.travel * D2R; standoff = P.height / 1000; wristRoll = P.wrRoll * D2R;
  trackOffset = 0; alongOffset = 0; for (const id of PARAM_AUTO) showActual(id, P[id], false); drawFillet();
}

/* ---------------------------------------------------------- telemetry charts */
class TS {
  constructor(cv, series) { this.cv = cv; this.ctx = cv.getContext('2d'); this.series = series; this.N = 300; this.data = series.map(() => []); }
  reset() { this.data = this.series.map(() => []); }
  push(v) { for (let i = 0; i < this.series.length; i++) { const a = this.data[i]; a.push(v[i]); if (a.length > this.N) a.shift(); } }
  draw() {
    const cv = this.cv, ctx = this.ctx, w = cv.clientWidth, h = cv.clientHeight; if (!w || !h) return;
    if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
    ctx.clearRect(0, 0, w, h);
    const n = this.series.length, LM = 46, bandH = h / n;
    for (let i = 0; i < n; i++) {
      const S = this.series[i], a = this.data[i], top = i * bandH, hdr = top + 11, pTop = top + 17, pBot = top + bandH - 6, unit = (S.unit || '').trim();
      if (i > 0) { ctx.strokeStyle = 'rgba(255,255,255,0.08)'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(0, top); ctx.lineTo(w, top); ctx.stroke(); }
      let mn, mx; if (a.length) { mn = Math.min(...a); mx = Math.max(...a); } else { mn = 0; mx = 1; }
      if (S.ref != null) { mn = Math.min(mn, S.ref); mx = Math.max(mx, S.ref); }
      let r = mx - mn; if (r < 1e-6) { const e = Math.max(1, Math.abs(mx) * 0.05); mn -= e; mx += e; r = mx - mn; } else { mn -= r * 0.12; mx += r * 0.12; r = mx - mn; }
      const Y = v => pBot - ((v - mn) / r) * (pBot - pTop);
      ctx.font = '9px "IBM Plex Mono"'; ctx.textAlign = 'right';
      for (const tv of [mx, (mn + mx) / 2, mn]) { const y = Y(tv); ctx.strokeStyle = 'rgba(255,255,255,0.05)'; ctx.beginPath(); ctx.moveTo(LM, y); ctx.lineTo(w, y); ctx.stroke();
        ctx.fillStyle = '#566372'; ctx.fillText(tv.toFixed(S.dp), LM - 4, y + 3); }
      if (mn < 0 && mx > 0) { const y0 = Y(0); ctx.strokeStyle = 'rgba(255,255,255,0.20)'; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(LM, y0); ctx.lineTo(w, y0); ctx.stroke(); ctx.setLineDash([]); }
      if (S.ref != null) { const yr = Y(S.ref); ctx.strokeStyle = 'rgba(110,184,255,0.55)'; ctx.setLineDash([5, 3]); ctx.beginPath(); ctx.moveTo(LM, yr); ctx.lineTo(w, yr); ctx.stroke(); ctx.setLineDash([]); }
      ctx.textAlign = 'left'; ctx.fillStyle = '#c6d2dc'; ctx.font = '10px "IBM Plex Mono"'; ctx.fillText(`${S.label} [${unit}]`, LM + 2, hdr);
      ctx.textAlign = 'right'; ctx.fillStyle = S.color; ctx.fillText(a.length ? a[a.length - 1].toFixed(S.dp) : '—', w - 3, hdr);
      if (a.length > 1) { ctx.strokeStyle = S.color; ctx.lineWidth = 1.6; ctx.beginPath();
        for (let k = 0; k < a.length; k++) { const x = LM + (k / (this.N - 1)) * (w - LM), y = Y(a[k]); k ? ctx.lineTo(x, y) : ctx.moveTo(x, y); } ctx.stroke(); }
    }
  }
}
let poseChart, poolChart, qualChart, chartAcc = 0, lastWP = null, poolSpeed = 0;
function initCharts() {
  poseChart = new TS($('poseChart'), [{ label: '横ずれ', color: '#46d39a', unit: 'mm', dp: 2 }, { label: '進行角', color: '#ffb13b', unit: '°', dp: 1 }, { label: '作業角', color: '#6fb8ff', unit: '°', dp: 1 }]);
  poolChart = new TS($('poolChart'), [{ label: '速度', color: '#46d39a', unit: 'mm/s', dp: 1 }, { label: '面積', color: '#ff6f63', unit: 'mm²', dp: 1 }, { label: '幅', color: '#ffb13b', unit: 'mm', dp: 2 }, { label: '長さ', color: '#6fb8ff', unit: 'mm', dp: 2 }]);
  qualChart = new TS($('qualChart'), [{ label: '面積変動 CV', color: '#ff6f63', unit: '%', dp: 1 }, { label: '倣い誤差 RMS', color: '#46d39a', unit: 'mm', dp: 2 }, { label: '報酬', color: '#c792ff', unit: '', dp: 2 }]);
}
function resetCharts() {
  if (poseChart) poseChart.reset(); if (poolChart) poolChart.reset(); if (qualChart) qualChart.reset();
  lastWP = null; poolSpeed = 0; chartAcc = 0; poolSolveAcc = 0; cornerHeat = 0; dwellRemain = 0; prevInCorner = false; passIdx = 0; pass2Active = false; autoWeldNext = false;
  target.set = false; trackOffset = 0; alongOffset = 0; disturb.lat = 0; disturb.work = 0; disturb.travel = 0; poolEval = null; _prevPose = null;
  qualWin.length = 0; liveReward.reset(); if (liveController && liveController.reset) liveController.reset();
  if (poolChart) poolChart.series[1].ref = poolChart.series[2].ref = poolChart.series[3].ref = null;
}
function _fitTransform(pts, w, h, pad) { let xmn = 1e9, xmx = -1e9, ymn = 1e9, ymx = -1e9, any = false;
  for (const Pp of pts) { if (!Pp) continue; any = true; xmn = Math.min(xmn, Pp.x); xmx = Math.max(xmx, Pp.x); ymn = Math.min(ymn, Pp.y); ymx = Math.max(ymx, Pp.y); }
  if (!any) return Pp => Pp; const ex = (xmx - xmn) || 1, ey = (ymx - ymn) || 1, cx = (xmn + xmx) / 2, cy = (ymn + ymx) / 2;
  const sc = Math.min((w - 2 * pad) / ex, (h - 2 * pad) / ey); return Pp => ({ x: w / 2 + (Pp.x - cx) * sc, y: h / 2 + (Pp.y - cy) * sc, s: sc }); }
function _poly(ctx, pts, close) { let started = false; ctx.beginPath();
  for (const p of pts) { if (!p) { started = false; continue; } if (!started) { ctx.moveTo(p.x, p.y); started = true; } else ctx.lineTo(p.x, p.y); } if (close) ctx.closePath(); }
function _grid(ctx, w, h) { ctx.strokeStyle = 'rgba(255,255,255,0.045)'; ctx.lineWidth = 1;
  for (let g = 0; g <= w; g += Math.max(20, w / 8)) { ctx.beginPath(); ctx.moveTo(g, 0); ctx.lineTo(g, h); ctx.stroke(); }
  for (let g = 0; g <= h; g += Math.max(20, h / 5)) { ctx.beginPath(); ctx.moveTo(0, g); ctx.lineTo(w, g); ctx.stroke(); } }
function _fit(cv) { const w = cv.clientWidth, h = cv.clientHeight; if (!w || !h) return null; if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; } return { ctx: cv.getContext('2d'), w, h }; }
let poolShapeAcc = 0;
function drawPoolShapes(dt) {
  const top = $('poolShape'); if (!top || !top.offsetParent) return;
  poolShapeAcc += dt; if (poolShapeAcc < 0.05) return; poolShapeAcc = 0;
  const c = _fit(top); if (!c) return; const { ctx, w, h } = c; ctx.clearRect(0, 0, w, h); _grid(ctx, w, h);
  if (!(poolFeat && poolGroup.visible)) { ctx.fillStyle = '#566372'; ctx.font = '11px "IBM Plex Mono"'; ctx.textAlign = 'center'; ctx.fillText('待機中 / no pool', w / 2, h / 2); return; }
  const F = poolFeat, loc = (F.outlineLocal || []).map(p => ({ x: p.x * 1000, y: p.y * 1000 })), fit = _fitTransform(loc, w, h, 26), Pp = loc.map(fit);
  let sx = 0, sy = 0; loc.forEach(p => { sx += p.x; sy += p.y; }); const cc = fit({ x: sx / loc.length, y: sy / loc.length });
  _poly(ctx, Pp, true); const g = ctx.createRadialGradient(cc.x, cc.y, 2, cc.x, cc.y, Math.max(w, h) * 0.4);
  g.addColorStop(0, 'rgba(255,250,228,0.96)'); g.addColorStop(0.42, 'rgba(255,192,92,0.92)'); g.addColorStop(0.78, 'rgba(255,112,42,0.82)'); g.addColorStop(1, 'rgba(120,32,8,0.42)');
  ctx.fillStyle = g; ctx.fill(); ctx.strokeStyle = 'rgba(255,210,150,0.96)'; ctx.lineWidth = 2; ctx.stroke();
  const sc = fit({ x: 0, y: 0 }).s || 1;
  ctx.strokeStyle = 'rgba(255,255,255,0.5)'; ctx.lineWidth = 1.2; ctx.beginPath(); ctx.moveTo(8, h - 9); ctx.lineTo(8 + 5 * sc, h - 9); ctx.stroke();
  ctx.fillStyle = '#9aa6b4'; ctx.font = '9px "IBM Plex Mono"'; ctx.textAlign = 'left'; ctx.fillText('5 mm', 8, h - 13);
  ctx.font = '10px "IBM Plex Mono"'; ctx.fillStyle = '#dbe6f0'; ctx.fillText(`WELD POOL  ${F.lenMM.toFixed(1)}×${F.widMM.toFixed(1)} mm · pen ${poolDisp.pen.toFixed(1)} mm`, 6, 14);
  ctx.fillStyle = 'rgba(255,200,120,0.95)'; ctx.fillText(F.shapeMode === 'detailed' ? '詳細計算 (Tm 等温線)' : '簡易 (楕円近似)', 6, 28);
  ctx.textAlign = 'right'; ctx.fillStyle = 'rgba(198,210,220,0.8)'; ctx.fillText('travel →', w - 8, 14);
}
function drawPoolCamImage() {
  const cv = $('poolCamImg'); if (!cv || !cv.offsetParent) return; const c = _fit(cv); if (!c) return; const { ctx, w, h } = c;
  ctx.fillStyle = '#05080b'; ctx.fillRect(0, 0, w, h);
  const src = $('camCanvas');
  if (src && src.width > 1) {
    const ca = src.width / src.height, pa = w / h; let dw = w, dh = h, dx = 0, dy = 0;
    if (ca > pa) { dh = w / ca; dy = (h - dh) / 2; } else { dw = h * ca; dx = (w - dw) / 2; }
    try { ctx.filter = FILT[camMode] || 'none'; ctx.drawImage(src, dx, dy, dw, dh); ctx.filter = 'none'; } catch (e) { /* ignore */ }
    if (poolFeat && poolGroup.visible) { const F = poolFeat, O = F.outlineLocal || [];
      const pts = O.map(p => { const pr = project(F.C.clone().addScaledVector(F.u, p.x).addScaledVector(F.v, p.y), CAM.cam, src.width, src.height);
        return pr ? { x: dx + pr.x * (dw / src.width), y: dy + pr.y * (dh / src.height) } : null; });
      _poly(ctx, pts, true); ctx.strokeStyle = 'rgba(70,255,160,0.95)'; ctx.lineWidth = 2; ctx.stroke(); }
  }
}
const MPF_STOPS = [[360, 26, 28, 34], [500, 90, 10, 0], [760, 180, 20, 0], [1040, 255, 90, 0], [1320, 255, 174, 51], [1620, 255, 227, 154], [2100, 255, 255, 255]];
function mpfHot(T, o) { if (T < MPF_STOPS[0][0]) return false;
  for (let i = 0; i < MPF_STOPS.length - 1; i++) { const a = MPF_STOPS[i], b = MPF_STOPS[i + 1];
    if (T <= b[0]) { const f = (T - a[0]) / (b[0] - a[0]); o[0] = a[1] + (b[1] - a[1]) * f; o[1] = a[2] + (b[2] - a[2]) * f; o[2] = a[3] + (b[3] - a[3]) * f; return true; } }
  o[0] = o[1] = o[2] = 255; return true; }
const _mpfOff = document.createElement('canvas');
function mpfFill(ctx, W, H, F, gw, gh, coord) {
  _mpfOff.width = gw; _mpfOff.height = gh; const o = _mpfOff.getContext('2d'), img = o.createImageData(gw, gh), d = img.data, col = [0, 0, 0];
  for (let j = 0; j < gh; j++) for (let i = 0; i < gw; i++) { const c = coord(i, j), T = F.T(c[0], c[1], c[2]), k = (j * gw + i) * 4;
    if (mpfHot(T, col)) { d[k] = col[0]; d[k + 1] = col[1]; d[k + 2] = col[2]; } else { d[k] = 11; d[k + 1] = 13; d[k + 2] = 18; } d[k + 3] = 255; }
  o.putImageData(img, 0, 0); ctx.imageSmoothingEnabled = true; ctx.drawImage(_mpfOff, 0, 0, W, H);
}
function isoRadii(F, thr, cx, cy, N, Rmax) { const r = new Array(N), hotC = F.T(cx, cy, 0) >= thr;
  for (let k = 0; k < N; k++) { if (!hotC) { r[k] = null; continue; } const th = k / N * 2 * Math.PI, dx = Math.cos(th), dy = Math.sin(th);
    let lo = 0, hi = Rmax; for (let it = 0; it < 20; it++) { const mid = (lo + hi) / 2; if (F.T(cx + dx * mid, cy + dy * mid, 0) >= thr) lo = mid; else hi = mid; } r[k] = lo; }
  return r; }
function isoBox(r, cx, cy, N) { let xmn = 1e9, xmx = -1e9, ymn = 1e9, ymx = -1e9, any = false;
  for (let k = 0; k < N; k++) { if (r[k] == null) continue; any = true; const th = k / N * 2 * Math.PI, x = cx + r[k] * Math.cos(th), y = cy + r[k] * Math.sin(th);
    xmn = Math.min(xmn, x); xmx = Math.max(xmx, x); ymn = Math.min(ymn, y); ymx = Math.max(ymx, y); }
  return any ? { L: xmx - xmn, W: ymx - ymn } : { L: 0, W: 0 }; }
function drawIso(ctx, r, cx, cy, N, col, lw, sx, sy) { let started = false; ctx.strokeStyle = col; ctx.lineWidth = lw; ctx.beginPath();
  for (let k = 0; k <= N; k++) { const kk = k % N; if (r[kk] == null) { started = false; continue; } const th = kk / N * 2 * Math.PI, X = sx(cx + r[kk] * Math.cos(th)), Y = sy(cy + r[kk] * Math.sin(th));
    if (!started) { ctx.moveTo(X, Y); started = true; } else ctx.lineTo(X, Y); } ctx.stroke(); }
let mpfAcc = 0;
function drawModelField(dt) {
  const top = $('mpfTop'), sec = $('mpfSec'), cyc = $('mpfCycle'); if (!top || !top.offsetParent) return;
  mpfAcc += dt; if (mpfAcc < 0.08) return; mpfAcc = 0;
  const F = WeldPoolModel.makeField(poolParams()), cx = F.xPeak, cy = F.cy;
  let c = _fit(top);
  if (c) { const { ctx, w, h } = c, x0 = cx - 19, x1 = cx + 9, yH = 11, sx = x => (x - x0) / (x1 - x0) * w, sy = y => (y + yH) / (2 * yH) * h;
    ctx.clearRect(0, 0, w, h); mpfFill(ctx, w, h, F, 120, 84, (i, j) => [x0 + (x1 - x0) * (i + 0.5) / 120, -yH + 2 * yH * (j + 0.5) / 84, 0]);
    const N = 120, Rm = 26, rTm = isoRadii(F, F.Tm, cx, cy, N, Rm), r11 = isoRadii(F, 1100, cx, cy, N, Rm), r8 = isoRadii(F, 800, cx, cy, N, Rm), r5 = isoRadii(F, 500, cx, cy, N, Rm);
    drawIso(ctx, r5, cx, cy, N, 'rgba(110,184,255,0.95)', 1.4, sx, sy); drawIso(ctx, r8, cx, cy, N, 'rgba(255,138,42,0.95)', 1.4, sx, sy);
    drawIso(ctx, r11, cx, cy, N, 'rgba(255,196,90,0.9)', 1.2, sx, sy); drawIso(ctx, rTm, cx, cy, N, 'rgba(255,255,255,0.98)', 1.6, sx, sy);
    const pb = isoBox(rTm, cx, cy, N), hb = isoBox(r8, cx, cy, N);
    ctx.font = '10px "IBM Plex Mono"'; ctx.fillStyle = '#dbe6f0'; ctx.textAlign = 'left'; ctx.fillText(`pool ${pb.L.toFixed(1)}×${pb.W.toFixed(1)} mm`, 6, h - 8);
    ctx.fillStyle = 'rgba(255,138,42,0.95)'; ctx.textAlign = 'right'; ctx.fillText(`HAZ(800℃) ${hb.W.toFixed(1)} mm`, w - 6, h - 8);
    ctx.fillStyle = 'rgba(198,210,220,0.9)'; ctx.textAlign = 'left'; ctx.fillText('travel →', 8, 14); }
  c = _fit(sec);
  if (c) { const { ctx, w, h } = c, yH = 10, zMax = 13; ctx.clearRect(0, 0, w, h);
    mpfFill(ctx, w, h, F, 90, 60, (i, j) => [cx, -yH + 2 * yH * (i + 0.5) / 90, zMax * (j + 0.5) / 60]);
    const PY = y => (y + yH) / (2 * yH) * w, PZ = z => z / zMax * h;
    const drawYZ = (thr, col, lw) => { const hot = F.T(cx, cy, 0) >= thr; ctx.strokeStyle = col; ctx.lineWidth = lw; ctx.beginPath();
      for (let a = 0; a <= 64; a++) { const th = Math.PI * a / 64, dy = Math.cos(th), dz = Math.sin(th); let r = 0;
        if (hot) { let lo = 0, hi = 20; for (let it = 0; it < 18; it++) { const m = (lo + hi) / 2; if (F.T(cx, cy + dy * m, dz * m) >= thr) lo = m; else hi = m; } r = lo; }
        const X = PY(cy + dy * r), Y = PZ(dz * r); a ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y); } ctx.stroke(); };
    drawYZ(500, 'rgba(110,184,255,0.95)', 1.3); drawYZ(800, 'rgba(255,138,42,0.95)', 1.3); drawYZ(1100, 'rgba(255,196,90,0.9)', 1.1); drawYZ(F.Tm, 'rgba(255,255,255,0.98)', 1.6);
    const thPx = PZ(P.thick); if (thPx < h) { ctx.strokeStyle = 'rgba(255,111,99,0.7)'; ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(0, thPx); ctx.lineTo(w, thPx); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = 'rgba(255,111,99,0.9)'; ctx.textAlign = 'right'; ctx.font = '9px "IBM Plex Mono"'; ctx.fillText('板厚 ' + P.thick + ' mm', w - 4, thPx - 3); }
    let pen = 0; for (let z = 0.1; z <= zMax; z += 0.1) { if (F.T(cx, cy, z) >= F.Tm) pen = z; else break; }
    ctx.fillStyle = '#dbe6f0'; ctx.font = '10px "IBM Plex Mono"'; ctx.textAlign = 'left'; ctx.fillText(`溶込み ${pen.toFixed(1)} mm`, 6, 14); }
  c = _fit(cyc);
  if (c) { const { ctx, w, h } = c; ctx.clearRect(0, 0, w, h);
    const x0 = cx - 26, x1 = cx + 9, Tmax = 2800, padL = 26, padR = 6, padT = 18, padB = 16;
    const PX = x => padL + (x - x0) / (x1 - x0) * (w - padL - padR), PY = T => h - padB - (Math.min(T, Tmax) / Tmax) * (h - padT - padB);
    ctx.fillStyle = 'rgba(255,150,60,0.12)';
    for (let i = 0; i < 200; i++) { const xa = x0 + (x1 - x0) * i / 200, xb = x0 + (x1 - x0) * (i + 1) / 200; if (F.T((xa + xb) / 2, cy, 0) >= F.Tm) ctx.fillRect(PX(xa), padT, PX(xb) - PX(xa), h - padT - padB); }
    ctx.lineWidth = 1; ctx.font = '8px "IBM Plex Mono"';
    for (const [T, lbl, col] of [[F.Tm, 'Tm', 'rgba(255,255,255,0.55)'], [1100, '1100', 'rgba(255,196,90,0.5)'], [800, '800', 'rgba(255,138,42,0.5)'], [500, '500', 'rgba(110,184,255,0.5)']]) {
      const y = PY(T); ctx.setLineDash([3, 3]); ctx.strokeStyle = col; ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = col; ctx.textAlign = 'right'; ctx.fillText(lbl, padL - 2, y + 3); }
    ctx.strokeStyle = 'rgba(124,255,176,0.96)'; ctx.lineWidth = 1.8; ctx.beginPath();
    for (let i = 0; i <= 200; i++) { const x = x0 + (x1 - x0) * i / 200, X = PX(x), Y = PY(F.T(x, cy, 0)); i ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y); } ctx.stroke();
    ctx.fillStyle = '#9aa6b4'; ctx.font = '9px "IBM Plex Mono"'; ctx.textAlign = 'left'; ctx.fillText(`peak ${F.peakC.toFixed(0)}℃ · t8/5 ${(poolEval && Number.isFinite(poolEval.t85)) ? poolEval.t85.toFixed(2) + ' s' : '—'}`, padL, 12);
    ctx.textAlign = 'right'; ctx.fillText('travel ξ →', w - padR, h - 4); }
}

/* --------------------------------------------------------- quality metrics */
const qualWin = [];   // recent {area, track}
function qualityMetrics() {
  const Q = WeldPoolModel.heatInput(poolParams()), H = Q / Math.max(weldSpeed * 1000, 0.05) / 1000;   // kJ/mm
  const ev = poolEval || WeldPoolModel.evaluate(poolParams());
  const areas = qualWin.map(x => x.area).filter(x => x > 0), mA = areas.reduce((a, b) => a + b, 0) / Math.max(areas.length, 1);
  const cv = areas.length > 3 ? Math.sqrt(areas.reduce((a, b) => a + (b - mA) ** 2, 0) / areas.length) / mA * 100 : NaN;
  const trms = qualWin.length ? Math.sqrt(qualWin.reduce((a, b) => a + b.track * b.track, 0) / qualWin.length) * 1000 : NaN;
  const penR = ev.penMM / P.thick;
  const flags = [];
  if (ev.status === 'no_pool') flags.push(['bad', '溶融池が形成されない（入熱不足）→ 融合不良']);
  if (ev.status === 'burn_through') flags.push(['bad', '溶込みが板厚に到達 → 溶け落ち（burn-through）']);
  else if (penR > 0.8) flags.push(['warn', `溶込み/板厚 ${(penR * 100).toFixed(0)}% → 溶け落ちリスク`]);
  if (ev.status === 'ok' && ev.penMM < 1.0) flags.push(['warn', '溶込み < 1 mm → 融合不良リスク']);
  if (jointType === 'fillet' && ev.status === 'ok' && ev.widMM < legLen * 1000 * 1.2) flags.push(['warn', `池幅 ${ev.widMM.toFixed(1)} mm < 脚長×1.2 → 脚長不足・アンダーカット傾向`]);
  if (Math.abs(trms) > 1.0) flags.push(['warn', `倣い誤差 RMS ${trms.toFixed(2)} mm → ビード偏り`]);
  if (cv > 10) flags.push(['warn', `池面積の変動 ${cv.toFixed(1)}% → ビード不揃い`]);
  if (Number.isFinite(ev.t85) && ev.t85 < 1.0 && CH.material === 'steel') flags.push(['warn', `t8/5=${ev.t85.toFixed(2)} s（急冷）→ HAZ 硬化傾向（参考）`]);
  return { Q, H, ev, cv, trms, penR, flags };
}
let qualAcc = 0;
function updateQualityPane(dt) {
  qualAcc += dt; if (qualAcc < 0.3) return; qualAcc = 0;
  const m = qualityMetrics();
  $('qQ').textContent = (m.Q / 1000).toFixed(2) + ' kW'; $('qH').textContent = m.H.toFixed(3) + ' kJ/mm'; $('mHI').textContent = m.H.toFixed(2) + ' kJ/mm';
  const box = $('qualityBox'); if (!box || !box.offsetParent) return;
  const row = (k, v, cls = '') => `<div class="e ${cls}"><time style="min-width:150px">${k}</time><span>${v}</span></div>`;
  const ev = m.ev;
  box.innerHTML = row('入熱 Q / H', `${(m.Q / 1000).toFixed(2)} kW · ${m.H.toFixed(3)} kJ/mm`) +
    row('溶融池 L × W', ev.status === 'no_pool' ? '—' : `${ev.lenMM.toFixed(1)} × ${ev.widMM.toFixed(1)} mm · 面積 ${ev.areaMM.toFixed(1)} mm²`) +
    row('溶込み / 板厚', `${ev.penMM.toFixed(1)} / ${P.thick} mm (${(m.penR * 100).toFixed(0)}%)`, m.penR > 0.8 ? 'warn' : '') +
    row('冷却時間 t8/5', Number.isFinite(ev.t85) ? ev.t85.toFixed(2) + ' s（厚板 Rosenthal 近似）' : '—') +
    row('ピーク温度', Number.isFinite(ev.peakC) ? ev.peakC.toFixed(0) + ' ℃（表示上限 4000）' : '—') +
    row('池面積の変動 CV', Number.isFinite(m.cv) ? m.cv.toFixed(1) + ' %（直近）' : '—（溶接中に計測）') +
    row('倣い誤差 RMS', Number.isFinite(m.trms) ? m.trms.toFixed(2) + ' mm' : '—') +
    row('Real2Sim 校正', `effScale ${CALIB.effScale.toFixed(3)} · kScale ${CALIB.kScale.toFixed(3)} · sigScale ${CALIB.sigScale.toFixed(3)}`) +
    (m.flags.length ? m.flags.map(([k, t]) => row(k === 'bad' ? '✗ 判定' : '⚠ 注意', t, k === 'bad' ? 'err' : 'warn')).join('') : row('✓ 判定', '目立った欠陥傾向なし（教育用近似）', 'ok'));
}

/* ============================ controllers (live) ============================ */
let liveController = null;                      // IController for 'autoCore' / 'policy'
const liveReward = new WeldCell.PoolHoldReward();
let lastReward = NaN;
function controllerLabel() { return { off: 'OFF', auto3: 'AUTO 3軸', autoCore: 'AUTO 契約', policy: 'RL 方策' }[CH.ctlMode] || CH.ctlMode; }
/** v21 classic 3-axis AUTO: hold pool area / width / aspect via speed / standoff / work angle. */
function classicAuto() {
  if (!poolFeat) return;
  const A = poolFeat.areaMM, L = poolFeat.lenMM, W = poolFeat.widMM, k = P.autoGain;
  if (!target.set) { target.A = A; target.L = L; target.W = W; target.set = true; }
  const eA = (A - target.A) / target.A, eW = (W - target.W) / target.W, eAsp = (L / W) - (target.L / target.W);
  applyLiveAction({ travelSpeed: weldSpeed * (1 + 0.22 * k * eA), standoff: standoff - 0.0011 * k * eW, workAngle: workAngle + 0.9 * k * eAsp * D2R });
}
/** Low-level seam tracking (robot controller's seam-sensor loop) — same role as SurrogateRobotCell.trackGain. */
function seamTracking(wp) {
  const ss = clamp(s, 0, 1), vl = new THREE.Vector3().crossVectors(CORNER_OPEN, seamTangent(ss)).normalize();
  const measErr = wp.clone().sub(seamPoint(ss)).dot(vl) + (R() - 0.5) * seamNoise * 0.2;
  trackOffset = clamp(trackOffset - 0.5 * measErr, -0.03, 0.03);
}
