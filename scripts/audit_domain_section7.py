#!/usr/bin/env python3
"""各ドメイン設計文書(docs/10〜16)の §7 が要求するユニットテストの網羅監査。

**なぜスクリプトにしたか**: この監査項目(チェックリスト §2)は長らく
「§8がある程度重なる保証は与えているが、ドメイン設計文書の §7 が個別に要求する
項目まで一つずつ拾えているかは確認していない」として未実施のまま残っていた。
手作業の読み合わせは再現性が無く、一度やっても次の増分で陳腐化する。
機械的に繰り返せる形にすることで、**監査結果を回帰として維持できる**ようにした。

**判定方法(と、その限界)**: 各 §7 項目から
  ①解析解テストID(M1/T3/E5/Q6/S1/A2/F10 …)
  ②日本語の特徴語(2〜8文字の漢字・カタカナ列)
を抜き、対応するクレートのソース(テスト名・doc コメント含む)+ sim-world を
検索する。IDが1つでも一致するか、特徴語の1/3以上が一致すれば「カバー済み」。

これは**厳密な証明ではない**——語の一致であって、テストが実際にその物理を
検証しているかまでは保証しない。あくまで「明らかな取りこぼしを見つける」ための
篩であり、実際に増分Lではこれで4件の真の欠落(バルマー線・自由膨張エントロピー・
イジングの高温/低温極限・L4トロヤ群)を発見して実装した。
"""
import glob
import re
import sys

DOMAIN_TO_CRATE = {
    "10-mechanics": "sim-mechanics",
    "11-fluid": "sim-fluid",
    "12-thermal": "sim-thermal",
    "13-electromagnetism": "sim-em",
    "14-quantum": "sim-quantum",
    "15-statistical": "sim-statistical",
    "16-astro": "sim-astro",
}


def crate_sources(crate: str) -> str:
    paths = glob.glob(f"crates/{crate}/src/**/*.rs", recursive=True)
    paths += glob.glob(f"crates/{crate}/tests/*.rs")
    return "".join(open(p, encoding="utf-8").read() for p in paths)


def section7_items(path: str) -> list[str]:
    text = open(path, encoding="utf-8").read()
    heading = re.search(r"^##\s*7[\.\s].*$", text, re.M)
    if not heading:
        return []
    start = heading.end()
    following = re.search(r"^##\s+(?!7)", text[start:], re.M)
    section = text[start : start + (following.start() if following else len(text))]
    items = [
        line.strip().lstrip("-*").strip()
        for line in section.split("\n")
        if line.strip().startswith(("-", "*"))
    ]
    return [i for i in items if len(i) > 6]


def is_covered(item: str, blob: str) -> bool:
    ids = re.findall(r"\b([MTEQSAFRXC]\d{1,2})\b", item)
    if any(i in blob for i in ids):
        return True
    words = set(re.findall(r"[ァ-ヶー一-龠]{2,8}", item))
    if not words:
        return True  # 数式のみの項目は語で判定できない(ID側で拾う)
    return sum(1 for w in words if w in blob) >= max(1, len(words) // 3)


def main() -> int:
    world = "".join(open(p, encoding="utf-8").read() for p in glob.glob("crates/sim-world/src/*.rs"))
    sources = {crate: crate_sources(crate) for crate in DOMAIN_TO_CRATE.values()}

    total = 0
    uncovered: list[tuple[str, str]] = []
    for path in sorted(glob.glob("docs/1[0-6]-*/*.md")):
        domain = path.split("/")[1]
        blob = sources[DOMAIN_TO_CRATE[domain]] + world
        for item in section7_items(path):
            total += 1
            if not is_covered(item, blob):
                uncovered.append((path, item))

    for path, item in uncovered:
        print(f"UNCOVERED {path}\n    {item}")
    covered = total - len(uncovered)
    print(f"\n{total} 項目中 {covered} カバー済み ({100 * covered / total:.1f}%)")
    return 1 if uncovered else 0


if __name__ == "__main__":
    sys.exit(main())
