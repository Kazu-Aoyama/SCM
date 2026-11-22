#!/usr/bin/env python3
"""共通モジュール（src/*.js）を各ツールの HTML に埋め込む。

各ツールの HTML には次の目印がある：
    <!-- SCMIO:BEGIN --> ... <!-- SCMIO:END -->
その間を src/scm-io.js の中身（<script> で包む）に置き換える。目印がないツールは </head> の直前に目印ごと入れる。

使い方:  python3 src/build.py            # tools/*.html をすべて更新
         python3 src/build.py --check    # 埋め込みが最新かどうかだけ調べる（CI 用）
"""
import pathlib, re, sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
MODULES = [("SCMIO", ROOT / "src" / "scm-io.js")]


TOOL_KEYS = {"index": "演習ファイル索引", "navi": "システム分析ナビ", "workbench": "システム分析ワークベンチ",
             "lab": "システム分析ラボ", "data": "データブラウザ"}


def find_tool(kw):
    """索引は配布フォルダの一番上（リポジトリの直下），ほかのツールは tools/ にある"""
    c = sorted(list((ROOT / "tools").glob("*.html")) + list(ROOT.glob("*.html")))
    c = [f for f in c if kw in f.name]
    if len(c) != 1:
        raise SystemExit(f"「{kw}」のファイルがちょうど1つ必要です: {[f.name for f in c]}")
    return c[0]


def tool_files():
    """tools/ の中から見た相対パス（索引だけは1つ上）"""
    import json
    m = {}
    for k, kw in TOOL_KEYS.items():
        f = find_tool(kw)
        m[k] = ("../" if f.parent == ROOT else "") + f.name
    return json.dumps(m, ensure_ascii=False)


def update_index(index_path):
    """索引の ITEMS のうちツール（c:"tool"）の項目のファイル名・大きさを，実際のファイルに合わせる"""
    import json
    html = index_path.read_text(encoding="utf-8")
    m = re.search(r"const ITEMS=(\[.*?\]);const CATS", html, re.S)
    if not m:
        return False
    items = json.loads(m.group(1))
    for it in items:
        if it.get("c") != "tool":
            continue
        for k, kw in TOOL_KEYS.items():
            if kw in it["f"]:
                f = find_tool(kw)
                it["f"] = f.name
                it["dir"] = "" if f.parent == ROOT else "tools"
                it["size"] = f.stat().st_size
    new = html[:m.start(1)] + json.dumps(items, ensure_ascii=False) + html[m.end(1):]
    if new != html:
        index_path.write_text(new, encoding="utf-8")
        return True
    return False


def block(name, path):
    code = path.read_text(encoding="utf-8")
    code = re.sub(r"/\*TOOL_FILES\*/.*?/\*END\*/", lambda _: "/*TOOL_FILES*/" + tool_files() + "/*END*/", code)
    if "</script" in code.lower() or f"{name}:BEGIN" in code or f"{name}:END" in code:
        raise SystemExit(f"{path}: '</script' や目印の文字列を含めないでください")
    return f"<!-- {name}:BEGIN -->\n<script>\n{code}</script>\n<!-- {name}:END -->"


def apply(html, name, blk):
    # 目印は「<!-- X:BEGIN -->\n<script>」で始まり「</script>\n<!-- X:END -->」で終わる
    pat = re.compile(rf"<!-- {name}:BEGIN -->\n<script>\n.*?</script>\n<!-- {name}:END -->", re.S)
    if pat.search(html):
        return pat.sub(lambda m: blk, html, count=1)
    i = html.lower().find("</head>")
    if i < 0:
        raise SystemExit("</head> が見つかりません")
    return html[:i] + blk + "\n" + html[i:]


def main():
    check = "--check" in sys.argv
    stale = []
    index = find_tool(TOOL_KEYS["index"])
    for f in sorted((ROOT / "tools").glob("*.html")) + [index]:
        html = f.read_text(encoding="utf-8")
        new = html
        for name, path in MODULES:
            new = apply(new, name, block(name, path))
        if new != html:
            stale.append(f.name)
            if not check:
                f.write_text(new, encoding="utf-8")
    if not check:
        for _ in range(2):  # 索引自身の大きさが変わるので2回
            update_index(index)
    if check and stale:
        print("埋め込みが古いファイル:", *stale, sep="\n  ")
        sys.exit(1)
    print(("更新: " if not check else "最新: ") + (", ".join(stale) if stale else "（変更なし）"))


if __name__ == "__main__":
    main()
