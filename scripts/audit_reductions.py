#!/usr/bin/env python3
"""コード・設計文書中に自己申告された「縮約」(簡略化・近似・後回しの明記)の集計。

**なぜスクリプトにしたか**: 統合エディタ実装計画
(docs/reviews/2026-08-14-editor-implementation-plan.md「物差し」節)が指摘する
とおり、docs/22-roadmap/02-feature-checklist.md のチェックボックスは100%に
達しているが、それは「設計どおり実装した」であって「実装したが縮約した」箇所が
無いことを意味しない——**この2つは別の軸**であり、チェックボックスは後者を
一切表現できない。「実装したが縮約した」「設計にあるが作らなかった」箇所は
本文中に`縮約`という語で正直に記録される運用になっているが、これまで
その総数を機械的に数えたことが無く、実態が「チェックボックスの向こう側」に
隠れていた。`scripts/audit_domain_section7.py`と同じ手口(ソースをテキストとして
走査し、判定は`grep`相当の単純一致に留める)で、この総和を可視化する。

**判定方法(と、その限界)**: `crates/**/*.rs`・`docs/**/*.md`の各行に「縮約」と
いう文字列が現れるかどうかだけで判定する。厳密な意味解析ではなく、単なる
文字列一致——「縮約」という語を含まない簡略化(見落とし)は数えられないし、
逆に「縮約」という語が"この実装は縮約ではない"のような否定文脈で使われていても
1件と数える(実測ではほぼ発生しない用法だが、可能性として記録しておく)。
あくまで「どれだけの箇所が自己申告されているか」を数える篩であり、
各箇所の重要度・残タスクとしての優先度は判定しない(それは人間が読んで
判断する仕事——このスクリプトの役目は「読むべき一覧を漏れなく出す」ことまで)。
"""
import glob
import sys


def find_reductions(path: str) -> list[tuple[int, str]]:
    with open(path, encoding="utf-8") as f:
        lines = f.readlines()
    return [
        (lineno, line.strip())
        for lineno, line in enumerate(lines, start=1)
        if "縮約" in line
    ]


def main() -> int:
    targets = sorted(glob.glob("crates/**/*.rs", recursive=True)) + sorted(
        glob.glob("docs/**/*.md", recursive=True)
    )

    per_file: dict[str, list[tuple[int, str]]] = {}
    for path in targets:
        hits = find_reductions(path)
        if hits:
            per_file[path] = hits

    for path, hits in sorted(per_file.items()):
        print(f"=== {path} ({len(hits)}件) ===")
        for lineno, text in hits:
            print(f"  {lineno:5d}: {text}")

    total = sum(len(hits) for hits in per_file.values())
    crates_total = sum(
        len(hits) for path, hits in per_file.items() if path.startswith("crates/")
    )
    docs_total = sum(
        len(hits) for path, hits in per_file.items() if path.startswith("docs/")
    )

    print("\n=== サマリ ===")
    print(f"合計: {total}件({len(per_file)}ファイル)")
    print(f"  crates/: {crates_total}件")
    print(f"  docs/:   {docs_total}件")
    print("\nファイル別件数(多い順、上位20件):")
    for path, hits in sorted(per_file.items(), key=lambda kv: -len(kv[1]))[:20]:
        print(f"  {len(hits):4d}  {path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
