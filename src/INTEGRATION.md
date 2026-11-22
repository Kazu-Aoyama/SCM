# ツール間連携の取り決め（v1）

対象：`tools/` の5つのツール（索引・ナビ・ワークベンチ・ラボ・データブラウザ）。
どのツールも単体の HTML のまま動き，データは端末の外へ送らない（外部サイトへのリンクにデータを載せない）。

## 共通モジュール `SCMIO`（`src/scm-io.js`）

`python3 src/build.py` が各 HTML の `<!-- SCMIO:BEGIN -->〜<!-- SCMIO:END -->` に埋め込む。
**ツール側でこの区間を直接書き換えない。** 共通の処理は `src/scm-io.js` を直して build する。

| 関数 | 用途 |
|---|---|
| `SCMIO.readFile(file)` | .xlsx / .csv / .tsv を `[{name, rows:[[...]]}]` に。日付は `YYYY-MM-DD`（時刻つきは `YYYY-MM-DD hh:mm:ss`） |
| `SCMIO.toObjects(rows, headerRow?)` | 見出し行を自動で見つけ `{columns, rows:[{列名:値}]}` に |
| `SCMIO.guessMapping(columns, fields)` | `fields=[{key,label,aliases}]` から列の対応を推定 |
| `SCMIO.toDate(v)` / `SCMIO.toNum(v)` | 和文の日付・「1,234円」「▲5」などを正規化 |
| `SCMIO.writeXlsx([{name, rows, noteRows?}])` | xlsx の Blob（1行目は見出し・固定） |
| `SCMIO.toCsv(rows)` | BOM 付き・引用符つきの CSV 文字列 |
| `SCMIO.download(blob or text, filename, type?)` | 保存 |
| `SCMIO.envelope(kind, from, payload, title)` | 受け渡しの封筒を作る |
| `SCMIO.sendTo(tool, env, {done})` | 相手のツールを開いて渡す（届かなければ `.scm.json` を保存） |
| `SCMIO.receive(tool, cb)` | 起動時に1回呼ぶ。届いた封筒を `cb(env)` に渡す |
| `SCMIO.envelopeFromText(text)` | 「開く」で読んだファイルが封筒（.scm.json）なら返す |
| `SCMIO.openTool(tool, hash?)` | 相手のツールを開くだけ |

`tool` の名前：`index` `navi` `workbench` `lab` `data`

## 封筒（envelope）

```json
{ "scmExchange": 1, "kind": "cld", "from": "workbench", "to": "lab", "id": "…",
  "title": "ブルーマウンテン社", "created": "2026-11-22T10:00:00Z", "payload": { … } }
```

各ツールは，`SCMIO.receive` で届いたときと，「開く」で `.scm.json` を選んだときの両方で同じ処理をする。
受け取ったら，**いまの作業を上書きする前に確認する**（または「自分のCLD」のように別枠に追加する）。

### kind ごとの payload

| kind | 送り手 → 受け手 | payload |
|---|---|---|
| `problem` | ナビ ⇄ ワークベンチ，ナビ → ラボ | `{ src:{title, svn, docs, same, bridge, note}, pos:{svn:{名前:[x,y]}, cld:{名前:[x,y]}} }`（ナビの文の書式そのまま） |
| `cld` | ワークベンチ・ナビ → ラボ，ラボ → ワークベンチ | `{ title, arrows:[{c, e, p:'+'/'-'/'?', d:bool, who?:'立場', src?:'資料名'}], pos:{名前:[x,y]}, sdKey? }` |
| `arrows` | ワークベンチ → データブラウザ（検証の依頼）／データブラウザ → ワークベンチ（検証の結果） | `{ title, arrows:[{c, e, p, d?, verdict?:'支持'…, r?, lag?}] }`。受け手のワークベンチは **いまの CLD に追加・照合**する（置き換えない） |
| `series` | データブラウザ → ラボ | `{ title, period:'month'|'week'|'day', t:['2023-01',…], series:{ 名前:{ unit, v:[…] } }, source }` |
| `dataset-meta` | データブラウザ → ワークベンチ | `{ name, variables:[{name, unit, aliases:[…]}] }`（取り込んだ業務データにある変数の一覧。データそのものは含めない） |

`p` は `'+' | '-' | '?'`。`d` は遅れ（‖）があるとき true。

## 変数名の対応（データブラウザ ⇄ ワークベンチ）

データブラウザが持つ変数の一覧（名前・単位・別名）は，`dataset-meta` でワークベンチに渡す。
ワークベンチは，自前の別名表 `DBV` に加えてこの一覧で「データで確かめる」の可否を判定する。
サンプル（ブルーマウンテン社）のときは，従来の20変数をそのまま使う。

## ファイル名

書き出すファイル名は `YYYYMMDD_scm-recurrent-program_<内容>.<拡張子>`（日付は書き出した日）。
固定の `_v1` は付けない。

## 業務データの取り込み（Excel）

- Excel（.xlsx）と CSV（UTF-8 / Shift_JIS）を受け付ける。.xls は「.xlsx で保存し直す」と案内。
- 列は見出しの名前から自動で対応づけ，画面で直せる。対応づけ（データは含まない）は JSON で保存・再利用できる。
- 各ツールは「テンプレート（.xlsx）を保存」ボタンで，必要な列と記入例・説明を入れた xlsx を出せる。
