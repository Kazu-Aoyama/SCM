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


def tool_files():
    import json
    m = {}
    for k, kw in TOOL_KEYS.items():
        c = sorted(f.name for f in (ROOT / "tools").glob("*.html") if kw in f.name)
        if len(c) != 1:
            raise SystemExit(f"tools/ に「{kw}」のファイルがちょうど1つ必要です: {c}")
        m[k] = c[0]
    return json.dumps(m, ensure_ascii=False)


def block(name, path):
    code = path.read_text(encoding="utf-8")
    code = re.sub(r"/\*TOOL_FILES\*/.*?/\*END\*/", lambda _: "/*TOOL_FILES*/" + tool_files() + "/*END*/", code)
    if "</script" in code.lower():
        raise SystemExit(f"{path}: '</script' を含めないでください")
    return f"<!-- {name}:BEGIN -->\n<script>\n{code}</script>\n<!-- {name}:END -->"


def apply(html, name, blk):
    pat = re.compile(rf"<!-- {name}:BEGIN -->.*?<!-- {name}:END -->", re.S)
    if pat.search(html):
        return pat.sub(lambda m: blk, html, count=1)
    i = html.lower().find("</head>")
    if i < 0:
        raise SystemExit("</head> が見つかりません")
    return html[:i] + blk + "\n" + html[i:]


def main():
    check = "--check" in sys.argv
    stale = []
    for f in sorted((ROOT / "tools").glob("*.html")):
        html = f.read_text(encoding="utf-8")
        new = html
        for name, path in MODULES:
            new = apply(new, name, block(name, path))
        if new != html:
            stale.append(f.name)
            if not check:
                f.write_text(new, encoding="utf-8")
    if check and stale:
        print("埋め込みが古いファイル:", *stale, sep="\n  ")
        sys.exit(1)
    print(("更新: " if not check else "最新: ") + (", ".join(stale) if stale else "（変更なし）"))


if __name__ == "__main__":
    main()
