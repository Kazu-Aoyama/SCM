# WeldCell ブリッジ・プロトコル `weldcell-bridge/1`

ブラウザ（WeldCell UI／方策）と実機セル（または仮想実機）を結ぶ JSON メッセージ仕様です。
要件定義書・API 仕様書（Set_Torch_Pose / Set_Camera_Pose / Set_Trigger / Get_Images / Get_Metrics /
Reset_Episode / Get_Calibration_Info）に対応づけ、さらにストリーミング（state / metrics）と E-STOP を加えています。

- 実装: クライアント `js/sim2real.js`（`BridgeClient`, `RealRobotCell`, `RealPoolSensor`）
- サーバ: `bridge/ws_bridge.py`（`MockBackend` / `Ros2Backend` 雛形）、ブラウザ内 `MockRealBackend`（ループバック）
- トランスポート: WebSocket（既定 `ws://localhost:8765`）。周期は既定 20 Hz（API 仕様の目標 50–100 Hz にも対応可）

## エンベロープ

```json
{ "v": 1, "protocol": "weldcell-bridge/1", "type": "control", "seq": 42, "t": 1732270000.12, "data": { } }
```

| フィールド | 意味 |
|---|---|
| `type` | メッセージ種別（下表） |
| `seq` | 送信側の連番。応答は `ack` に同じ値を入れる |
| `t` | 送信時刻 [s] |
| `data` | 本体。**単位は契約どおり SI（m, s, rad, A, V, °C）**。ただし `metrics` は API 仕様互換で mm・cm/min |

## クライアント → サーバ

| type | data | 対応する API 仕様 | 応答 |
|---|---|---|---|
| `hello` | `{client, protocol}` | – | `hello_ack {backend, protocol, rateHz, capabilities[]}` |
| `set_process` | `{I, V, eta, material, T0, thickness}` | – | `ack` |
| `control` | ControlAction の**絶対値**（SafetyLimiter 通過後）: `{travelSpeed, workAngle, travelAngle, standoff, lateralOffset, wristRoll}` | （Set_Torch_Pose の高水準版） | `ack` |
| `set_torch_pose` | `{position:[x,y,z] mm, rotation:[roll,pitch,yaw] rad, frame:"seam"}` — seam フレーム: y=横, z=標準距離, pitch=進行角, yaw=作業角偏差 | Set_Torch_Pose | `ack` |
| `set_camera_pose` | `{position, rotation}`（同上） | Set_Camera_Pose | `ack` |
| `set_trigger` | `{on: bool}` | Set_Trigger | `ack` |
| `reset_episode` | `{scenario_id}` | Reset_Episode | `ack` |
| `get_calibration_info` | `{}` | Get_Calibration_Info | `calibration {frame, units, origin, scale}` |
| `get_metrics` | `{}`（ポーリング用。通常はストリーム） | Get_Metrics | `metrics` |
| `estop` | `{}` | – | `ack {estop:true}`。以降の `control` は無視 |
| `ping` | `{}` | – | `pong`（RTT 計測） |

## サーバ → クライアント（ストリーム）

### `state`（RobotState, SI）
```json
{ "pose":[x,y,z,roll,pitch,yaw], "vel":[6], "deviation":[along,lateral,height], "seamParam":0.42,
  "trackError":0.0003, "lateralOffset":0.001, "wristRoll":0, "phase":"weld",
  "weldInput":{ "travelSpeed":0.007, "workAngle":0.785, "workDev":0.0, "travelAngle":-0.26, "standoff":0.002,
                "weldPosition":"F", "process":{"I":175,"V":24,"eta":0.8,"Q":3360}, "seamParam":0.42, "contact":true },
  "arc":true, "estopped":false, "t":12.3 }
```

### `metrics`（API 仕様の Get_Metrics 互換 + 溶融池ブロック）
```json
{ "arc_length_mm": 3.2, "deviation_mm": 0.1, "velocity_cm_min": 45.0, "eval_score": 0.98,
  "pool": { "L_mm": 10.4, "W_mm": 6.9, "area_mm2": 56.1, "cx_mm": -2.7, "cy_mm": 0.0,
            "pen_mm": null, "peakC": 2400, "t85": 1.2, "status": "ok" },
  "process": { "I": 175, "V": 24 }, "t": 12.3 }
```
- `pool.status`: `ok` / `no_pool` / `burn_through`。ビジョンで溶込みが観測できない場合 `pen_mm` は省略可。
- クライアントは `metricsToObs()` で PoolObservation（SI）へ変換し、**シミュレーションと同じ契約**として扱います。
- 画像（Get_Images）は本バージョンでは扱いません（将来: `image` メッセージ＋バイナリフレームを予定）。

## 安全設計

1. 方策・コントローラの出力は必ず `SafetyLimiter` を通過（範囲・変化率・ウォッチドッグ・E-STOP）。
2. UI で **CONTROL モード + ARM** されるまで `control` は送信されません。
3. 計測が `watchdog` ms 以上途絶えると指令停止。`Esc` キー / E-STOP ボタンで `estop` 送信。
4. 実機側コントローラ自身のインターロック（速度・領域・アーク監視）を**必ず併用**してください。本ブリッジは研究用です。

## 実機への接続手順（ROS 2 の例）

1. `bridge/ws_bridge.py` の `Ros2Backend` にトピック（docstring 参照）を実装。
2. `python3 bridge/ws_bridge.py --backend ros2 --rate 50`
3. UI ⑤ → 接続先「WebSocket ブリッジ」→ URL → 接続 → SHADOW で残差を確認 → Real2Sim 校正 → CONTROL / ARM。
