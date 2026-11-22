# WeldCell v22 — 溶接フィジカル AI 研究開発基盤

`welding_cell_dualarm_v21.html` をベースに，**シミュレータ・学習・実機接続（Sim2Real）・実機からの校正（Real2Sim）** を
一つの「契約（contract）」でつないだ研究開発基盤です．ブラウザだけで動き（ビルド不要・オフライン可），
Python 側にも同じ物理・同じ方策ランタイム・Gym 互換環境・WebSocket ブリッジを用意しています．

```
index.html           ← アプリ本体（file:// で開くだけ）
dist/weldcell_v22.html  ← 単一ファイル版（npm run build）
docs/manual.html     ← ユーザ＆理論マニュアル（アプリと相互リンク）
```

## クイックスタート

| やりたいこと | 手順 |
|---|---|
| シミュレータを使う | `index.html` をブラウザで開く（Chrome / Edge 推奨）．`Space` で溶接開始／停止，`1`〜`5` でステージ切替 |
| 単一ファイル版を作る | `node tools/build_single.js` → `dist/weldcell_v22.html` |
| テスト | `node tests/run_tests.js`（コア 17 件）・`python3 tests/test_python.py`（JS⇔Python 一致）・`node tests/smoke_browser.js`（ブラウザ E2E） |
| 実機ブリッジ（仮想実機） | `pip install websockets && python3 bridge/ws_bridge.py` → UI ⑤ で「WebSocket ブリッジ」を選び接続 |
| Python で PPO/SAC | `bridge/weldcell_env.py`（Gymnasium 互換．`pip install gymnasium stable-baselines3` で SB3 にそのまま渡せる） |
| 学習済み方策を実機 PC で動かす | ④ で「方策 JSON 書出し」→ `bridge/policy_runtime.py` で読み込み（JS と数値一致をテスト済み） |

## 画面構成（v21 からの改良点）

v21 では右パネルに約 40 個の入力が一列に並び，ステージ切替も一部しか連動していなかったため，以下のように再設計しました．

- **ヘッダ**: 5 ステージのステッパ（① 条件設定 → ② 配置・到達性 → ③ 溶接・観測 → ④ 学習 → ⑤ Sim⇄Real），運用モード表示（SIM / SHADOW / CONTROL），設定の保存・読込・既定化，マニュアル．
- **ランバー（常時表示）**: Home / Approach / 動作シミュ / Start Weld / Stop / REC と主要状態（phase・進捗・制御器・池寸法・面積・入熱・倣い誤差・IK 誤差・報酬）．どのステージにいても運転できます．
- **インスペクタ（右）**: ステージごとに折りたたみ可能なセクション．**全パラメータがスキーマ駆動**（`js/app/params.js`）で，
  ラベル・記号・単位・スライダ・**数値入力**・既定値リセット（↺）・説明（ホバー）が統一されています．既定値から変えた値は黄色，制御器が自動調整している値は橙色（設定値＝公称値は保持）．
- **ダッシュボード（下）**: タブ化 — テレメトリ／溶融池／熱場（計算）／**品質**／学習／Sim⇄Real／ログ．
- **品質タブ（新）**: 入熱 Q・H，L×W，溶込み/板厚，t8/5，面積変動 CV，倣い RMS，欠陥傾向（融合不良・溶け落ち・脚長不足・急冷）の判定（教育用近似）．
- **設定 JSON**: 全パラメータ・選択肢・乱数シード・校正値・Home 姿勢を保存／読込 → 実験の再現性．

## アーキテクチャ

```
                ┌──────────────── 契約 (core.js) ─────────────────┐
  IRobotCell    │ WeldInput  : robot → pool  (局所フレーム u/v/n, 速度, 角度, 標準距離, プロセス) │
  IWeldPool     │ PoolObservation : pool → controller (L, W, 面積, 溶込み, 重心, t8/5, status)    │
  IController   │ ControlAction   : controller → robot (絶対値 / 増分 d*) ─ applyAction() で統一解釈 │
                └─────────────────────────────────────────────────┘
      Robot 実装                  Pool 実装                      Controller 実装
  ─ SurrogateRobotCell      ─ ModelPool(WeldPoolModel)       ─ AutoHoldController（古典）
  ─ ライブ 3D セル (cell.js)  ─ ModelPool(SurrogateModel 表)  ─ PolicyController(LinearPolicy, Codec)
  ─ RealRobotCell (bridge)  ─ RandomizedPool (DR)           ─ （任意の update(rs,obs,dt)）
  ─ ReplayRobot (ログ)       ─ RealPoolSensor (bridge)
                            ─ WorldModelPool（学習モデル差替え口）／ReplayPool
```

**同じ Coordinator・同じ報酬（PoolHoldReward）・同じ Codec・同じ制御周期（0.05 s）** が，
サロゲート学習・ライブ 3D シミュレータ・実機ブリッジのすべてで使われます．これが Sim2Real の拡張性の核です．

| ファイル | 役割 |
|---|---|
| `js/pool_model.js` | 溶融池物理（Rosenthal 3D 移動熱源の 3×3 重ね合わせ）．v2: 校正ノブ（effScale / kScale / sigScale），t8/5，溶け落ち判定 |
| `js/core.js` | weldcell_core v2: 契約・Coordinator・報酬・Codec（compact3 / pool11 / **要件 19 次元** 観測，Δ3 / **要件 7 次元** 行動）・Env・TrajectoryRecorder（JSONL）・WorldModelPool・シード付き乱数 |
| `js/learning.js` | サロゲート表（物理から生成，JSON 出力可）・線形方策・CEM・BC（模倣学習）・学習セル |
| `js/sim2real.js` | ブリッジプロトコル・BridgeClient・Real 系アダプタ・SafetyLimiter・DomainRandomizer・仮想実機・Real2Sim Calibrator・Replay・残差モニタ |
| `js/app/*.js` | UI（params: スキーマ, cell: 3D セル, loop: 状態機械・制御ディスパッチ, panels: 学習・Sim⇄Real・初期化） |
| `bridge/` | Python: `weldpool.py`（物理の完全移植）, `ws_bridge.py`（サーバ・仮想実機・ROS 2 雛形）, `policy_runtime.py`, `weldcell_env.py`（Gym 互換） |
| `docs/` | `manual.html`（ユーザ＆理論マニュアル）, `PROTOCOL.md`（ブリッジ仕様） |

## Sim2Real / Real2Sim ワークフロー

1. **学習（④）**: AUTO の実演から **BC で初期化 → CEM**．目標面積 A* は公称条件の物理モデルから決定．
   **ドメインランダム化**（効率・熱伝導率・熱源径 ±%，計測ノイズ，遅延，初期条件）で頑健化．
   評価は別シードで「制御なし / AUTO / BC / RL」を比較表示．方策は JSON で書き出し（codec・報酬・校正値・学習履歴つき）．
2. **接続（⑤）**: 仮想実機（ブラウザ内，隠れ真値つき）または WebSocket ブリッジ（Python, ROS 2 雛形）．
3. **SHADOW（デジタルツイン）**: 実機の設定値をライブ 3D に写し，実機計測とシミュレーション予測の残差を常時監視．
4. **Real2Sim**: 励起シーケンス（速度×アーク長×作業角の 27 点）でデータ取得 → **Nelder–Mead で effScale / kScale / sigScale を推定** → 適用．
   仮想実機（真値 0.90 / 1.15 / 1.10）では 0.898 / 1.159 / 1.115 を推定し，幅 RMSE 1.14 → 0.21 mm（E2E テストで確認）．
   CSV / JSONL（記録・施工レポート）からも校正可能．
5. **再学習**: 校正済みモデルでサロゲート表を再生成して学習 → **CONTROL + ARM** で同じ方策を実機へ（SafetyLimiter 経由）．
6. **記録**: REC（ライブ）・実機ログ・Coordinator ロールアウトはすべて同一 JSONL スキーマ（`weldcell-traj/2`, (s, a, r, s′, done) + 生データ）．オフライン RL・世界モデル学習・システム同定にそのまま使えます．

## v21 からの修正（バグ・不整合）

| # | 内容 | 対応 |
|---|---|---|
| 1 | **溶融池モデル**: 中心線の温度分布が複数の山を持つと池長さが ~1/10 に潰れる（後端を「最後の入点」で上書き） | 最初の入点〜最後の出点に修正（長アーク時に顕在化） |
| 2 | 学習サロゲート（258 A, 26 V, η0.85）と物理（175 A, 24 V, 0.80）が不一致 | サロゲート表を同じ物理・同じプロセス条件から生成（誤差 <6%，テストで保証） |
| 3 | RL の横補正 dL を学習するが配備時に未使用 | 契約 `applyAction` 経由で全行動を適用 |
| 4 | 軌跡 JSONL に報酬 r が無い（マニュアルと不一致） | (s, a, r, s′, done) を出力 |
| 5 | WeldCellEnv の操作量ペナルティが常に 0 | 外部行動を Coordinator に渡して報酬計算 |
| 6 | Env 既定の目標面積 0.35 mm²（実際の数十 mm² と桁違い） | 公称条件から自動決定 |
| 7 | Home で J7 が戻らない／到達性評価で J7 の関節限界を未確認 | 7 軸すべてに修正 |
| 8 | 溶融池の「計測値」に表示用フリッカー（±5%）が混入し AUTO・品質を乱す | 計測は定常輪郭＋シード付きセンサノイズ |
| 9 | AUTO がスライダ（条件値）を上書きし，公称条件が失われる | 公称値（設定）と実際値（橙表示）を分離 |
| 10 | 制御周期が描画フレームレート依存 | 契約制御器は 0.05 s 固定周期（学習・実機と同一） |
| 11 | 電流・電圧・効率・材料・板厚が固定 | すべて UI で設定可能（入熱表示つき） |
| 12 | 乱数が非再現 | シード付き PRNG（設定 JSON に保存） |
| 13 | ヘッダコメント「CR-4iA 6-axis」等の機種表記の不一致 | 7 自由度・CRX クラスに統一 |
| 14 | 到達性評価の始点が IK 未収束で「到達不可」になる | 始点のみ反復を増やして収束 |

## 前提と限界（教育・研究用近似）

- 溶融池は線形熱伝導（Rosenthal）．対流・マランゴニ・潜熱・自由表面・溶滴移行は含みません（`weld_pool_fdm_cfd.html` 系の数値モデルは `IWeldPool` として将来差し込み可能）．
- 姿勢の重力補正・品質判定は経験的な目安です．定量的な施工判断には使わないでください．
- 仮想実機は「校正の正しさを検証するための既知の差」を持つ模擬プラントです．実機の非線形・外乱はより複雑です．
- 画像（Get_Images）系の API・視覚ベース学習は未実装（プロトコル拡張の余地を残しています）．
