/* =============================================================================
 * params.js — single source of truth for every user-facing parameter.
 * The schema drives: inspector rows (slider + numeric + unit + reset), choice
 * chips, per-stage layout, conditional visibility, config save/load (JSON).
 * Values in P are in DISPLAY units (mm, deg, mm/s …); the cell converts to SI.
 * ========================================================================== */
'use strict';
const $ = (id) => document.getElementById(id);
const APP_VERSION = '22.0';

/* ----------------------------------------------------------------- params */
// id: {label, sym, unit, min, max, step, def, desc, dp}
const PARAMS = {
  // ① process
  I:      { label: '溶接電流', sym: 'I', unit: 'A', min: 80, max: 350, step: 5, def: 175, desc: '溶接電流．入熱 Q=ηVI を通じて溶融池寸法・溶込みを支配．' },
  V:      { label: 'アーク電圧', sym: 'V', unit: 'V', min: 14, max: 34, step: 0.5, def: 24, desc: 'アーク電圧．入熱 Q=ηVI に比例．', dp: 1 },
  eta:    { label: 'アーク効率', sym: 'η', unit: '', min: 0.5, max: 0.95, step: 0.01, def: 0.80, desc: 'アーク熱効率（GMAW 代表値 0.8）．Real2Sim 校正では effScale として補正．', dp: 2 },
  thick:  { label: '板厚', sym: 't', unit: 'mm', min: 2, max: 25, step: 0.5, def: 9, desc: '母材板厚．溶込みが板厚の 95% 以上で溶け落ち（burn-through）判定．', dp: 1 },
  T0:     { label: '予熱温度', sym: 'T₀', unit: '℃', min: 20, max: 300, step: 5, def: 20, desc: '予熱・パス間温度．冷却時間 t8/5 に影響．' },
  // ① torch / conditions
  speed:  { label: '溶接速度', sym: 'v', unit: 'mm/s', min: 2, max: 20, step: 0.5, def: 7, desc: '走行速度．溶融池の長さ・面積・溶込みを支配（AUTO/方策が自動調整）．', dp: 1 },
  work:   { label: '作業角', sym: 'φw', unit: '°', min: 30, max: 60, step: 1, def: 45, desc: '両板に対するトーチの傾き（隅肉は 45° 基準，平板は 90° 基準）．' },
  travel: { label: '進行角', sym: 'φt', unit: '°', min: -25, max: 25, step: 1, def: -15, desc: '前進角（−）／後退角（+）．溶込み傾向を変える．' },
  height: { label: 'アーク長', sym: 'ℓa', unit: 'mm', min: 0, max: 12, step: 0.1, def: 2, desc: 'トーチ高さ（標準距離）．熱源の広がり σ0=0.6+0.35ℓa に影響．', dp: 1 },
  wrRoll: { label: '手首ロール', sym: 'ψ', unit: '°', min: -90, max: 90, step: 1, def: 0, desc: 'ツール軸まわりの手首回転（7 軸冗長自由度の活用）．' },
  wrTilt: { label: '手首チルト', sym: 'θw', unit: '°', min: -30, max: 30, step: 1, def: 0, desc: 'ツール軸の追加傾斜．' },
  leg:    { label: '脚長', sym: 'z', unit: 'mm', min: 3, max: 9, step: 0.5, def: 5, desc: '隅肉ビードの脚長（ビード表示）．', dp: 1 },
  curve:  { label: '溶接線の曲がり', sym: 'b', unit: 'mm', min: -30, max: 30, step: 1, def: 0, desc: '溶接線の横方向の弓なり量．倣い制御の評価に使用．' },
  noise:  { label: '外乱（倣いノイズ）', sym: 'σs', unit: 'mm', min: 0, max: 20, step: 0.5, def: 3, desc: '溶接線位置・姿勢の確率的外乱（シード固定で再現可能）．', dp: 1 },
  // ② placement
  baseX:  { label: 'base X（前後）', sym: 'x', unit: 'mm', min: -300, max: 500, step: 5, def: 0, desc: '選択アームの基準位置（カメラアーム選択時はトーチ基台からの相対オフセット）．' },
  baseY:  { label: 'base Y（高さ）', sym: 'y', unit: 'mm', min: -100, max: 450, step: 5, def: 0, desc: '基準位置の高さ．' },
  baseZ:  { label: 'base Z（左右）', sym: 'z', unit: 'mm', min: -500, max: 500, step: 5, def: 0, desc: '基準位置の左右．' },
  lam:    { label: 'IK 減衰', sym: 'λ', unit: '', min: 0.005, max: 0.2, step: 0.005, def: 0.05, desc: '減衰最小二乗 IK の λ（小=機敏／大=安定）．', dp: 3 },
  dwell:  { label: '角部停止', sym: 'td', unit: 's', min: 0, max: 2, step: 0.1, def: 0.4, desc: '角回し時のコーナー一旦停止（クレータ処理）．', dp: 1 },
  pitch:  { label: '胴体ピッチ', sym: 'θp', unit: '°', min: 0, max: 45, step: 1, def: 18, desc: 'ヒューマノイドの前傾．' },
  waist:  { label: '腰ヨー', sym: 'θy', unit: '°', min: -60, max: 60, step: 1, def: 0, desc: 'ヒューマノイドの腰回転．' },
  bench:  { label: '作業台高さ', sym: 'h', unit: 'mm', min: 0, max: 450, step: 5, def: 380, desc: 'ヒューマノイド立位時のワーク高さ．' },
  headY:  { label: '頭部 pan', sym: '', unit: '°', min: -80, max: 80, step: 1, def: 0, desc: '頭部の左右（手動時）．' },
  headP:  { label: '頭部 tilt', sym: '', unit: '°', min: -60, max: 40, step: 1, def: -25, desc: '頭部の上下（手動時）．' },
  // ③ vision
  look:   { label: '先読み距離', sym: '', unit: 'mm', min: 5, max: 40, step: 1, def: 18, desc: 'カメラが溶接線を先読みする距離．' },
  zoom:   { label: 'ズーム', sym: '', unit: '×', min: 1, max: 6, step: 0.1, def: 2, desc: 'ビジョンカメラの画角倍率．', dp: 1 },
  camD:   { label: 'カメラ距離', sym: '', unit: 'mm', min: 40, max: 170, step: 1, def: 95, desc: '溶接点からカメラまでの距離．' },
  camE:   { label: 'カメラ仰角', sym: '', unit: '°', min: 12, max: 82, step: 1, def: 50, desc: '俯角．' },
  camA:   { label: 'カメラ方位', sym: '', unit: '°', min: -120, max: 120, step: 1, def: 22, desc: '方位角（0=後方から前方を見る）．' },
  autoGain: { label: 'AUTO ゲイン', sym: 'k', unit: '×', min: 0.2, max: 3, step: 0.1, def: 1, desc: '古典 AUTO（溶融池一定）制御のゲイン倍率．', dp: 1 },
  // ④ learning
  iters:  { label: 'CEM 反復', sym: 'N', unit: '', min: 3, max: 60, step: 1, def: 15, desc: '交差エントロピー法の反復回数．' },
  pop:    { label: '母集団', sym: 'n', unit: '', min: 8, max: 64, step: 2, def: 24, desc: '1 反復あたりのサンプル方策数．' },
  elite:  { label: 'エリート', sym: 'k', unit: '', min: 2, max: 16, step: 1, def: 6, desc: '平均・分散の再推定に使う上位数．' },
  epSteps:{ label: 'エピソード長', sym: '', unit: 'step', min: 100, max: 800, step: 20, def: 300, desc: '学習 1 エピソードのステップ数（dt=0.05 s）．' },
  wArea:  { label: '報酬: 池一定', sym: 'wA', unit: '', min: 0, max: 2, step: 0.05, def: 0.7, desc: '溶融池面積の偏差 |A−A*|/A* の重み．', dp: 2 },
  wTrack: { label: '報酬: 追従', sym: 'wT', unit: '', min: 0, max: 2, step: 0.05, def: 0.5, desc: '倣い誤差 |e|/3mm の重み．', dp: 2 },
  wEffort:{ label: '報酬: 操作量', sym: 'wE', unit: '', min: 0, max: 1, step: 0.01, def: 0.05, desc: '制御操作量（角度増分）の重み．', dp: 2 },
  drStr:  { label: 'DR 物理ばらつき', sym: '±', unit: '%', min: 0, max: 30, step: 1, def: 12, desc: 'ドメインランダム化：効率・熱伝導率・熱源径の ± 範囲．' },
  drNoise:{ label: 'DR 計測ノイズ', sym: 'σ', unit: '%', min: 0, max: 10, step: 0.5, def: 3, desc: '溶融池計測の相対ノイズ．', dp: 1 },
  drDelay:{ label: 'DR 遅延', sym: '', unit: 'step', min: 0, max: 6, step: 1, def: 2, desc: '観測遅延（0〜指定ステップ）．' },
  coreSteps: { label: 'ロールアウト長', sym: '', unit: 'step', min: 100, max: 1500, step: 50, def: 600, desc: 'Coordinator ロールアウトのステップ数．' },
  // ⑤ sim2real
  latency:{ label: '仮想実機 遅延', sym: '', unit: 'ms', min: 0, max: 200, step: 5, def: 15, desc: '仮想実機（ループバック）の片道通信遅延．' },
  rateHz: { label: '制御周期', sym: 'f', unit: 'Hz', min: 5, max: 100, step: 5, def: 20, desc: '実機ブリッジの制御・計測周期（API 仕様: 50–100 Hz 目標）．' },
  rateV:  { label: '速度 変化率上限', sym: '', unit: 'mm/s²', min: 0.5, max: 20, step: 0.5, def: 4, desc: 'セーフティ：走行速度指令の変化率上限．', dp: 1 },
  rateAng:{ label: '角度 変化率上限', sym: '', unit: '°/s', min: 2, max: 60, step: 1, def: 20, desc: 'セーフティ：作業角・進行角指令の変化率上限．' },
  watchdog:{ label: 'ウォッチドッグ', sym: '', unit: 'ms', min: 100, max: 3000, step: 50, def: 500, desc: '計測が途絶えたら指令を停止する時間．' },
  exciteT:{ label: '励起時間', sym: '', unit: 's', min: 5, max: 120, step: 5, def: 30, desc: 'Real2Sim 用システム同定の励起シーケンス長．' },
  seed:   { label: '乱数シード', sym: '', unit: '', min: 0, max: 99999, step: 1, def: 12345, desc: '外乱・学習・ドメインランダム化の乱数シード（再現性）．' },
};
/* ----------------------------------------------------------------- choices */
const CHOICES = {
  joint:    { label: '継手', def: 'fillet', opts: [['fillet', '隅肉'], ['bop', 'ビードオンプレート'], ['butt', '突き合わせ']] },
  pos:      { label: '溶接姿勢', def: 'F', opts: [['F', '下向 F'], ['H', '横向 H'], ['VU', '立向 V↑'], ['VD', '立向 V↓']] },
  path:     { label: '経路', def: 'line', opts: [['line', '直線'], ['box', '角回し'], ['multi', '角回し＋別部材']] },
  material: { label: '材料', def: 'steel', opts: [['steel', '軟鋼'], ['sus', 'ステンレス'], ['al', 'アルミ']] },
  sysMode:  { label: 'システム構成', def: 'single', opts: [['single', '単腕（眼手カメラ）'], ['dual', '双腕（カメラアーム）']] },
  form:     { label: 'ロボット形態', def: 'separate', opts: [['separate', '独立2台'], ['dual', '双腕統合'], ['humanoid', 'ヒューマノイド']] },
  baseTarget: { label: '操作対象', def: 'torch', opts: [['torch', 'トーチアーム'], ['camera', 'カメラアーム']] },
  camFeed:  { label: 'VISION 映像', def: 'arm', opts: [['torch', 'トーチ'], ['arm', 'カメラアーム'], ['head', 'ヘッド']] },
  headAuto: { label: '頭部', def: 'auto', opts: [['auto', '自動追尾'], ['manual', '手動']] },
  camMode:  { label: 'カメラモード', def: 'nir', opts: [['normal', '実画像'], ['nir', 'NIR'], ['pool', '池輪郭'], ['seam', 'シーム']] },
  poolShape:{ label: '池形状', def: 'simple', opts: [['simple', '簡易（楕円）'], ['detailed', '詳細（等温線）']] },
  section:  { label: '3D 断面', def: 'on', opts: [['on', '表示'], ['off', '非表示']] },
  ctlMode:  { label: '溶融池制御', def: 'auto3', opts: [['off', 'OFF（手動）'], ['auto3', 'AUTO 3軸'], ['autoCore', 'AUTO 契約版'], ['policy', 'RL 方策']] },
  track:    { label: '倣い制御', def: 'on', opts: [['on', 'ON（シームセンサ）'], ['off', 'OFF']] },
  obsSpec:  { label: '観測', def: 'compact3', opts: [['compact3', 'compact 3'], ['pool11', 'pool 11'], ['req19', '要件 19']] },
  actSpec:  { label: '行動', def: 'delta3', opts: [['delta3', 'Δ 3'], ['req7', '要件 7']] },
  trainEnv: { label: '学習環境', def: 'surrogate', opts: [['surrogate', 'サロゲート表（高速）'], ['physics', '物理モデル直']] },
  bcWarm:   { label: '模倣学習', def: 'on', opts: [['on', 'AUTO から BC 初期化'], ['off', 'ゼロ初期化']] },
  dr:       { label: 'ドメインランダム化', def: 'on', opts: [['on', 'ON'], ['off', 'OFF']] },
  coreCtl:  { label: 'コントローラ', def: 'auto', opts: [['auto', 'AUTO'], ['policy', 'RL 方策']] },
  backend:  { label: '接続先', def: 'mock', opts: [['mock', '仮想実機（ブラウザ内）'], ['ws', 'WebSocket ブリッジ']] },
  opMode:   { label: '運用モード', def: 'sim', opts: [['sim', 'SIM'], ['shadow', 'SHADOW 監視'], ['control', 'CONTROL 制御']] },
};
const P = {}; const CH = {};
for (const k in PARAMS) P[k] = PARAMS[k].def;
for (const k in CHOICES) CH[k] = CHOICES[k].def;
const PARAM_AUTO = new Set(['speed', 'work', 'height', 'travel', 'wrRoll']);   // can be driven by controllers

/* ------------------------------------------------------------------ stages */
const STAGES = {
  1: { title: '条件設定 — Plan', desc: '継手・姿勢・経路とプロセス条件（電流・電圧・速度・角度）を決めます．値は数値入力もでき，↺ で既定値に戻せます．' },
  2: { title: '配置・到達性 — Place', desc: 'システム構成（単腕／双腕／ヒューマノイド）とロボット基台を決め，溶接線の到達性を評価します（緑=到達可）．' },
  3: { title: '溶接・観測 — Weld', desc: '閉ループ制御を選んで溶接します．ビジョン・溶融池・熱場・品質はダッシュボードで観測します．' },
  4: { title: '学習 — Learn', desc: '契約（Coordinator）上で方策を学習します：AUTO の模倣（BC）→ CEM 強化学習（ドメインランダム化）→ 評価 → 配備／書き出し．' },
  5: { title: 'Sim⇄Real — Bridge & Calibrate', desc: '実機（または仮想実機）へ接続し，SHADOW（監視・残差）／CONTROL（方策で制御）を行います．実機ログから溶融池モデルを校正（Real2Sim）します．' },
};

/* ------------------------------------------------------------------ layout */
// item: 'paramId' | {c:'choiceId'} | {sub:'text'} | {html:'...'} ; section: {stage,title,items,open,when,cls,badge}
const LAYOUT = [
  { stage: 1, title: '継手・姿勢・経路', open: true, items: [{ c: 'joint' }, { c: 'pos' }, { c: 'path' },
    { html: '<div class="diagram"><svg id="filletSvg" viewBox="0 0 240 150" width="100%" height="110"></svg></div>' }] },
  { stage: 1, title: 'プロセス条件', open: true, badge: 'procBadge', items: [{ c: 'material' }, 'I', 'V', 'eta', 'thick', 'T0',
    { html: '<div class="grid2" style="margin-top:6px"><div class="mini"><div class="k">入熱 Q=ηVI</div><div class="v" id="qQ">—</div></div><div class="mini"><div class="k">単位長入熱 H=Q/v</div><div class="v" id="qH">—</div></div></div>' }] },
  { stage: 1, title: 'トーチ姿勢・速度', open: true, items: ['speed', 'work', 'travel', 'height', 'wrRoll', 'wrTilt', 'leg',
    { html: '<div class="note">作業角＝両板に対するトーチの傾き（隅肉 45°基準）．進行角＝前進角（−）／後退角（+）．<b>橙色の値</b>は閉ループ制御が自動調整中．</div>' }] },
  { stage: 1, title: 'ワーク・外乱', open: false, items: ['curve', 'noise', 'seed', { html: '<div class="btnrow"><button class="btn" id="clearBead">ビード消去</button><button class="btn" id="reseedBtn">シード再適用</button></div>' }] },

  { stage: 2, title: 'システム構成', open: true, items: [{ c: 'sysMode' },
    { c: 'form', when: () => CH.sysMode === 'dual' }, { c: 'camFeed', when: () => CH.sysMode === 'dual' },
    { html: '<div class="note" id="sysNote"></div>' }] },
  { stage: 2, title: 'ヒューマノイド', open: true, when: () => CH.sysMode === 'dual' && CH.form === 'humanoid',
    items: ['pitch', 'waist', 'bench', { c: 'headAuto' }, 'headY', 'headP'] },
  { stage: 2, title: 'ロボット基台・IK', open: true, items: [{ c: 'baseTarget', when: () => CH.sysMode === 'dual' }, 'baseX', 'baseY', 'baseZ', 'lam', 'dwell',
    { html: '<div class="btnrow"><button class="btn" id="homeSetBtn">Home＝現在姿勢</button><button class="btn" id="homeDefBtn">Home 既定</button></div>' }] },
  { stage: 2, title: '到達性評価', open: true, badge: 'reachBadge', items: [
    { html: '<div class="btnrow"><button class="btn primary" id="evalBtn">溶接線を評価</button><button class="btn" id="autoPlaceBtn" title="基台 X/Z を格子探索して到達率が最大の位置を探す">基台を自動探索</button></div><div class="out" id="reachOut">未評価</div>' }] },
  { stage: 2, title: '関節 J1–J7', open: false, items: [{ html: '<div class="jbars" id="jbars"></div>' }] },
  { stage: 2, title: 'カメラアーム状態', open: true, when: () => CH.sysMode === 'dual', items: [
    { html: '<div class="grid2"><div class="mini"><div class="k">視認</div><div class="v" id="visStat">—</div></div><div class="mini"><div class="k">アーム干渉</div><div class="v" id="visIntf">—</div></div><div class="mini"><div class="k">standoff 推定</div><div class="v" id="visStand">—</div></div><div class="mini"><div class="k">work/travel 推定</div><div class="v" id="visAng">—</div></div></div>' }] },

  { stage: 3, title: '閉ループ制御', open: true, items: [{ c: 'ctlMode' }, { c: 'track' }, 'autoGain',
    { html: '<div class="note" id="ctlNote"></div>' }] },
  { stage: 3, title: '溶融池の表示', open: true, items: [{ c: 'poolShape' }, { c: 'section' }] },
  { stage: 3, title: 'ビジョン', open: true, items: [{ c: 'camMode' }, 'look', 'zoom', 'camD', 'camE', 'camA'] },
  { stage: 3, title: 'ライブ記録', open: false, items: [{ html: '<div class="note">REC（ランバー）で溶接中のフレームを契約スキーマ（WeldInput / PoolObservation / RobotState / action / reward）で記録し，JSONL で書き出します．実機ログと同一形式なので Real2Sim・オフライン学習にそのまま使えます．</div><div class="btnrow"><button class="btn" id="recExport">記録を JSONL 書出し</button><button class="btn" id="recClear">記録クリア</button></div><div class="out" id="recOut">0 frames</div>' }] },

  { stage: 4, title: 'タスク定義（契約）', open: true, items: [{ c: 'obsSpec' }, { c: 'actSpec' }, 'wArea', 'wTrack', 'wEffort',
    { html: '<div class="note" id="specNote"></div>' }] },
  { stage: 4, title: '学習環境・ドメインランダム化', open: true, items: [{ c: 'trainEnv' }, { c: 'dr' }, 'drStr', 'drNoise', 'drDelay'] },
  { stage: 4, title: 'アルゴリズム（BC → CEM）', open: true, items: [{ c: 'bcWarm' }, 'iters', 'pop', 'elite', 'epSteps',
    { html: '<div class="btnrow"><button class="btn primary" id="trainBtn">▶ Train</button><button class="btn" id="trainStop">■ 中断</button><button class="btn" id="deployBtn">Deploy: OFF</button></div><canvas class="curve" id="rlCurve"></canvas><div class="out" id="rlOut">未学習</div><div class="btnrow"><button class="btn" id="polExport">方策 JSON 書出し</button><button class="btn" id="polImport">方策 JSON 読込</button><input type="file" id="polFile" accept=".json" class="hide"></div>' }] },
  { stage: 4, title: '連携中核 weldcell_core', open: false, items: [{ c: 'coreCtl' }, 'coreSteps',
    { html: '<div class="btnrow"><button class="btn" id="coreRunBtn">▶ Coordinator ロールアウト</button><button class="btn" id="coreEnvBtn">WeldCellEnv 評価</button></div><div class="btnrow"><button class="btn" id="coreExportBtn">軌跡 JSONL</button><button class="btn" id="tableExportBtn">サロゲート表 JSON</button></div><canvas class="curve" id="coreCurve"></canvas><div class="out" id="coreOut">core idle</div>' }] },

  { stage: 5, title: '接続', open: true, cls: 'real', badge: 'connBadge', items: [{ c: 'backend' },
    { html: '<div class="field" id="wsRow"><label>WebSocket URL</label><input type="text" id="wsUrl" value="ws://localhost:8765"></div>' },
    'latency', 'rateHz',
    { html: '<div class="btnrow"><button class="btn real" id="connBtn">接続</button><button class="btn" id="discBtn">切断</button><button class="btn" id="pingBtn">ping / 校正情報</button></div><div class="out" id="connOut">未接続</div>' }] },
  { stage: 5, title: '運用モード', open: true, cls: 'real', items: [{ c: 'opMode' },
    { html: '<div class="note" id="opNote2"></div><div class="btnrow"><button class="btn real" id="armBtn">ARM（指令許可）</button><button class="btn danger" id="estopBtn">E-STOP</button></div><div class="grid2" style="margin-top:6px"><div class="mini"><div class="k">実機 W / 予測 W</div><div class="v" id="twW">—</div></div><div class="mini"><div class="k">残差 RMSE W</div><div class="v" id="twR">—</div></div><div class="mini"><div class="k">rtt / rx</div><div class="v" id="twRtt">—</div></div><div class="mini"><div class="k">安全</div><div class="v" id="twSafe">disarmed</div></div></div>' }] },
  { stage: 5, title: 'セーフティ', open: false, cls: 'real', items: ['rateV', 'rateAng', 'watchdog',
    { html: '<div class="note">方策と実機の間には必ず SafetyLimiter（範囲・変化率・ウォッチドッグ・E-STOP）が入ります．ARM するまで指令は送信されません．</div>' }] },
  { stage: 5, title: 'Real2Sim 校正', open: true, cls: 'real', badge: 'calBadge', items: [
    { html: '<div class="note">実機（または仮想実機）の計測から溶融池モデルの補正係数 <b>effScale</b>（効率）・<b>kScale</b>（熱伝導率）・<b>sigScale</b>（熱源径）を推定します．データ：①接続中の実機ログ ②CSV/JSONL ファイル ③合成データ（検証用）．</div>' },
    'exciteT',
    { html: '<div class="btnrow"><button class="btn real" id="exciteBtn" title="CONTROL+ARM 時に速度・アーク長・作業角を段階的に変化させて同定用データを取得">励起シーケンス実行</button><button class="btn" id="calFromLog">実機ログで校正</button></div><div class="btnrow"><button class="btn" id="calFileBtn">ファイルで校正</button><button class="btn" id="calSynth">合成データで検証</button><input type="file" id="calFile" accept=".csv,.jsonl,.txt,.json" class="hide"></div><div class="btnrow"><button class="btn primary" id="calApply">校正値をシミュレータへ適用</button><button class="btn" id="calReset">校正リセット</button></div><div class="out" id="calOut">calib: effScale 1.000 · kScale 1.000 · sigScale 1.000</div>' }] },
];

/* -------------------------------------------------------------- UI builder */
const _rows = {};   // id -> {row, range, num}
function fmt(id, v) { const d = PARAMS[id].dp ?? (PARAMS[id].step < 1 ? 2 : 0); return (+v).toFixed(d); }
function paramRow(id) {
  const s = PARAMS[id], row = document.createElement('div'); row.className = 'prow'; row.dataset.param = id;
  row.innerHTML = `<label title="${s.desc || ''}">${s.label}${s.sym ? `<i>${s.sym}</i>` : ''}</label>
    <input type="range" min="${s.min}" max="${s.max}" step="${s.step}" value="${P[id]}">
    <input class="num" type="number" min="${s.min}" max="${s.max}" step="${s.step}" value="${fmt(id, P[id])}">
    <span class="unit">${s.unit}</span><button class="rst" title="既定値 ${s.def} ${s.unit} に戻す">↺</button>`;
  const range = row.children[1], num = row.children[2], rst = row.children[4];
  range.addEventListener('input', () => setParam(id, +range.value, { from: 'ui' }));
  num.addEventListener('change', () => { const v = parseFloat(num.value); if (Number.isFinite(v)) setParam(id, v, { from: 'ui' }); else num.value = fmt(id, P[id]); });
  rst.addEventListener('click', () => setParam(id, s.def, { from: 'ui' }));
  _rows[id] = { row, range, num };
  return row;
}
function choiceRow(cid) {
  const c = CHOICES[cid], row = document.createElement('div'); row.className = 'chiprow'; row.dataset.choice = cid;
  row.innerHTML = `<label>${c.label}</label><div class="chips">${c.opts.map(([v, l]) => `<span class="chip${CH[cid] === v ? ' on' : ''}" data-v="${v}">${l}</span>`).join('')}</div>`;
  row.querySelectorAll('.chip').forEach(ch => ch.addEventListener('click', () => setChoice(cid, ch.dataset.v, { from: 'ui' })));
  return row;
}
const _whens = [];
function buildInspector() {
  const host = $('inspector'); host.innerHTML = '';
  for (const sec of LAYOUT) {
    const d = document.createElement('details'); d.className = 'sec' + (sec.cls ? ' ' + sec.cls : ''); d.dataset.stage = sec.stage; if (sec.open) d.open = true;
    d.innerHTML = `<summary>${sec.title}${sec.badge ? `<span class="badge" id="${sec.badge}"></span>` : ''}</summary><div class="body"></div>`;
    const body = d.querySelector('.body');
    for (const it of sec.items) {
      let el;
      if (typeof it === 'string') el = paramRow(it);
      else if (it.c) el = choiceRow(it.c);
      else if (it.sub) { el = document.createElement('div'); el.className = 'sub'; el.textContent = it.sub; }
      else if (it.html) { el = document.createElement('div'); el.innerHTML = it.html; }
      if (it.when) _whens.push([el, it.when]);
      body.appendChild(el);
    }
    if (sec.when) _whens.push([d, sec.when, true]);
    host.appendChild(d);
  }
  // camera mode chips duplicated as an overlay on the vision view
  const cm = $('camModes'); cm.innerHTML = CHOICES.camMode.opts.map(([v, l]) => `<span class="chip${CH.camMode === v ? ' on' : ''}" data-v="${v}">${l}</span>`).join('');
  cm.querySelectorAll('.chip').forEach(ch => ch.addEventListener('click', () => setChoice('camMode', ch.dataset.v, { from: 'ui' })));
}
function refreshVisibility() {
  const stage = +document.body.dataset.stage || 1;
  document.querySelectorAll('#inspector > .sec').forEach(d => d.classList.toggle('hide', +d.dataset.stage !== stage));
  for (const [el, fn, isSec] of _whens) { const show = !!fn(); if (isSec) { if (+el.dataset.stage === stage) el.classList.toggle('hide', !show); } else el.classList.toggle('hide', !show); }
}
function _reflect(id) {
  const r = _rows[id]; if (!r) return;
  r.range.value = P[id]; if (document.activeElement !== r.num) r.num.value = fmt(id, P[id]);
  r.row.classList.toggle('changed', Math.abs(P[id] - PARAMS[id].def) > 1e-9);
}
/** Display an actual (controller-driven) value in a row without changing the nominal P[id]. */
function showActual(id, v, auto = true) {
  const r = _rows[id]; if (!r) return; r.range.value = v; if (document.activeElement !== r.num) r.num.value = fmt(id, v);
  r.row.classList.toggle('auto', !!auto); r.num.title = auto ? `実際値（制御中）· 設定値 ${fmt(id, P[id])} ${PARAMS[id].unit}` : '';
}
/** Set a parameter (display units). opts.silent: don't call onParamChange; opts.auto: value driven by a controller. */
function setParam(id, v, opts = {}) {
  const s = PARAMS[id]; if (!s) return;
  v = Math.min(s.max, Math.max(s.min, v)); P[id] = v; _reflect(id);
  if (_rows[id]) _rows[id].row.classList.toggle('auto', !!opts.auto);
  if (!opts.silent && typeof onParamChange === 'function') onParamChange(id, v, opts);
}
function setParamRange(id, min, max, val) {
  const s = PARAMS[id]; s.min = min; s.max = max; if (val != null) s.def = val;
  const r = _rows[id]; if (r) { r.range.min = r.num.min = min; r.range.max = r.num.max = max; }
  setParam(id, val != null ? val : P[id], { silent: true });
}
function setChoice(cid, v, opts = {}) {
  if (!CHOICES[cid]) return; const prev = CH[cid]; CH[cid] = v;
  document.querySelectorAll(`[data-choice="${cid}"] .chip`).forEach(ch => ch.classList.toggle('on', ch.dataset.v === v));
  if (cid === 'camMode') document.querySelectorAll('#camModes .chip').forEach(ch => ch.classList.toggle('on', ch.dataset.v === v));
  if (!opts.silent && typeof onChoice === 'function') onChoice(cid, v, prev, opts);
  refreshVisibility();
}

/* ---------------------------------------------------------- config (JSON) */
function exportConfig(extra = {}) {
  return Object.assign({ app: 'WeldCell', version: APP_VERSION, saved: new Date().toISOString(), params: Object.assign({}, P), choices: Object.assign({}, CH) }, extra);
}
function importConfig(cfg) {
  if (!cfg || !cfg.params) throw new Error('not a WeldCell config');
  for (const [k, v] of Object.entries(cfg.choices || {})) if (CHOICES[k]) setChoice(k, v);
  for (const [k, v] of Object.entries(cfg.params)) if (PARAMS[k]) setParam(k, v);
}
function downloadText(name, text, type = 'application/json') {
  const blob = new Blob([text], { type }), url = URL.createObjectURL(blob), a = document.createElement('a');
  a.href = url; a.download = name; document.body.appendChild(a); a.click(); a.remove(); setTimeout(() => URL.revokeObjectURL(url), 1500);
}
function readFile(input) { return new Promise((res, rej) => { const f = input.files && input.files[0]; if (!f) return rej(new Error('no file'));
  const r = new FileReader(); r.onload = () => { input.value = ''; res({ name: f.name, text: r.result }); }; r.onerror = rej; r.readAsText(f); }); }

/* ------------------------------------------------------------ log & toast */
function log(msg, kind = '') {
  const box = $('logBox'); if (!box) return; const e = document.createElement('div'); e.className = 'e ' + kind;
  const t = new Date(); e.innerHTML = `<time>${t.toTimeString().slice(0, 8)}</time><span></span>`; e.lastChild.textContent = msg;
  box.prepend(e); while (box.children.length > 400) box.lastChild.remove();
}
let _toastT = 0;
function toast(msg, kind = '') { const el = $('toast'); if (!el) return; el.textContent = msg; el.classList.add('show'); clearTimeout(_toastT);
  _toastT = setTimeout(() => el.classList.remove('show'), 2600); log(msg, kind); }
const safeStore = { get(k) { try { return localStorage.getItem(k); } catch (e) { return null; } }, set(k, v) { try { localStorage.setItem(k, v); } catch (e) { /* ignore */ } } };
