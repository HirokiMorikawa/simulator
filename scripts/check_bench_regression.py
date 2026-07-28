#!/usr/bin/env python3
"""性能ベンチ回帰ゲート(設計 docs/00-foundation/05-rust-wasm-platform.md §5、
横断DoD④「性能予算監視+ベンチ回帰なし」)。

## なぜこのスクリプトが要るのか

criterion 自体は回帰を **表示するだけで終了コードは 0 のまま** である
(実測: 同一ベンチを `--baseline` 付きで再実行すると
"Performance has regressed." と出力しながら exit 0)。したがって CI を落とすには
criterion が書き出す `target/criterion/<bench>/change/estimates.json` の
`mean.point_estimate`(基準比の相対変化、0.05 なら +5% 遅い)を読んで判定する
層が別に必要になる。それが本スクリプトである。

## なぜ「同一ランナー上での A/B 比較」なのか

共有 CI ランナーの絶対性能は実行ごとに大きくばらつくため、「前回の実行で採った
絶対時間」と比較する素朴な方式は偽陽性を量産する。そこで **同じジョブ・同じ
ランナーの中で** merge-base(既定 `origin/main`)を `git worktree` で取り出して
ベースラインを採り、続けて作業ツリー側を測って比較する。ビルドは2回走るが、
マシン差・ノイズの支配的な要因が消える。

## 閾値の根拠(実測)

同一ランナー・同一コードで背中合わせに2回測っても差は消えない。本スクリプト
実装時の実測では `mechanics_step_stack_of_20_boxes` が **+3.68%(95%信頼区間の
上端 +5.79%)** の「回帰」を示した——コードは1バイトも変えていない。
既定閾値 25% はこの実測ノイズ(約6%)の4倍に相当する余裕を取った値であり、
「本物の回帰(アルゴリズムの計算量が変わる、無駄なコピーが入る等)は数十%〜
数倍で現れる」という前提に立っている。ノイズで落ちない方を優先した設定である。
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

# criterion の統計サンプリングは既定(100サンプル・3秒ウォームアップ)だと
# CI には重すぎるため、A/B 両側で同じ設定に揃えた上で縮める。
# 両側が同一設定である限り比較の公平性は保たれる。
CRITERION_ARGS = [
    "--sample-size",
    "20",
    "--warm-up-time",
    "1",
    "--measurement-time",
    "3",
]


def run(cmd: list[str], cwd: Path | None = None, env: dict[str, str] | None = None) -> None:
    print(f"$ {' '.join(cmd)}", flush=True)
    subprocess.run(cmd, cwd=cwd, env=env, check=True)


def bench_targets(manifest_dir: Path) -> list[tuple[str, str]]:
    """`cargo metadata` から (パッケージ名, ベンチターゲット名) を列挙する。

    `cargo bench --workspace --benches` ではダメな理由: ベンチを1つも持たない
    クレートでも lib ターゲットが libtest の bench ハーネスを持つため、
    criterion 固有の `--save-baseline` が「Unrecognized option」で弾かれる
    (実測で sim-astro の lib が最初に踏んだ)。ベンチターゲットを明示的に
    列挙すればこれを避けられ、ベンチが増えても自動で追随する。
    """
    out = subprocess.run(
        ["cargo", "metadata", "--no-deps", "--format-version", "1"],
        cwd=manifest_dir,
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    meta = json.loads(out)
    targets: list[tuple[str, str]] = []
    for package in meta["packages"]:
        for target in package["targets"]:
            if "bench" in target["kind"]:
                targets.append((package["name"], target["name"]))
    return sorted(targets)


def run_benches(
    manifest_dir: Path,
    target_dir: Path,
    extra: list[str],
    targets: list[tuple[str, str]],
) -> None:
    # **`--target-dir`(cargoのフラグ)だけでは criterion には伝わらない**。
    # criterion は結果の書き出し先を独立に `CARGO_TARGET_DIR` 環境変数から
    # 決めるため、環境変数を立てないとベースライン側は
    # `<worktree>/target/criterion` へ書かれ、ワークツリー削除と共に消える
    # (実装中に実際に踏んだ: 比較段で "Baseline 'base' must exist" で落ちた)。
    env = {**os.environ, "CARGO_TARGET_DIR": str(target_dir)}
    for package, bench in targets:
        run(
            ["cargo", "bench", "-p", package, "--bench", bench, "--", *extra, *CRITERION_ARGS],
            cwd=manifest_dir,
            env=env,
        )


def check_changes(target_dir: Path, threshold: float) -> int:
    """`change/estimates.json` を全て読み、閾値超過があれば非ゼロを返す。"""
    files = sorted((target_dir / "criterion").glob("**/change/estimates.json"))
    if not files:
        print(
            "ERROR: change/estimates.json が1つも無い。ベースラインとの比較が"
            "実際には行われていない(--baseline の指定漏れ、あるいはベースライン側の"
            "ベンチが失敗した可能性がある)。",
            file=sys.stderr,
        )
        return 2

    print(f"\n=== 性能ベンチ回帰判定(閾値 +{threshold * 100:.0f}%) ===")
    regressions: list[tuple[str, float]] = []
    for path in files:
        # target/criterion/<bench 名>/change/estimates.json
        name = path.parent.parent.name
        data = json.loads(path.read_text())
        change = data["mean"]["point_estimate"]
        ci = data["mean"]["confidence_interval"]
        flag = "REGRESSION" if change > threshold else "ok"
        print(
            f"  {flag:<11} {name:<40} {change * 100:+7.2f}% "
            f"(95%CI {ci['lower_bound'] * 100:+.2f}% .. {ci['upper_bound'] * 100:+.2f}%)"
        )
        if change > threshold:
            regressions.append((name, change))

    if regressions:
        print(
            f"\n{len(regressions)}件のベンチが閾値 +{threshold * 100:.0f}% を超えて"
            "遅くなった:",
            file=sys.stderr,
        )
        for name, change in regressions:
            print(f"  - {name}: {change * 100:+.2f}%", file=sys.stderr)
        return 1

    print(f"\n全{len(files)}件が閾値以内。")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--baseline-ref",
        default="origin/main",
        help="ベースラインとして測る git ref(既定: origin/main)。"
        "実際には HEAD との merge-base を取り出す。",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.25,
        help="許容する相対劣化(0.25 = +25%%)。既定値の根拠はモジュールdoc参照。",
    )
    parser.add_argument(
        "--skip-baseline",
        action="store_true",
        help="ベースライン採取を省き、既存の base ベースラインと比較だけ行う"
        "(ローカルでの再判定用)。",
    )
    args = parser.parse_args()

    repo = Path(__file__).resolve().parent.parent
    # A/B を同じ criterion ディレクトリで突き合わせるため target を共有する。
    target_dir = repo / "target"

    head_targets = bench_targets(repo)
    if not head_targets:
        sys.exit("作業ツリーにベンチターゲットが1つも無い(cargo metadata の結果が空)")

    comparable = head_targets
    if not args.skip_baseline:
        merge_base = subprocess.run(
            ["git", "merge-base", "HEAD", args.baseline_ref],
            cwd=repo,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        print(f"ベースライン: {args.baseline_ref} との merge-base = {merge_base}")

        worktree = Path(tempfile.mkdtemp(prefix="bench-base-"))
        shutil.rmtree(worktree)  # git worktree add は空でないディレクトリを嫌う
        try:
            run(
                ["git", "worktree", "add", "--detach", "-q", str(worktree), merge_base],
                cwd=repo,
            )
            # **ベースライン側に存在するベンチだけを比較対象にする**。ブランチで
            # 新設したベンチは merge-base には無いため(実測: このリポジトリの
            # merge-base はベンチ導入前のコミットで、ベンチが1本も無かった)、
            # 積集合を取らないとベースライン採取が落ちる。新規ベンチは比較対象が
            # 存在しないので「回帰なし」として扱うのが唯一正しい——初回だけは
            # 必ず素通りするが、次回以降は基準ができて効くようになる。
            base_targets = bench_targets(worktree)
            comparable = [t for t in head_targets if t in base_targets]
            new_only = [t for t in head_targets if t not in base_targets]
            for package, bench in new_only:
                print(f"  [新規] {package}/{bench} はベースラインに存在しないため比較対象外")
            if not comparable:
                print(
                    "\nベースラインに比較可能なベンチが1本も無いため、回帰判定を"
                    "スキップする(ベンチ自体が新規に追加された初回の状態)。"
                )
                return 0
            run_benches(worktree, target_dir, ["--save-baseline", "base"], comparable)
        finally:
            subprocess.run(
                ["git", "worktree", "remove", "--force", str(worktree)],
                cwd=repo,
                check=False,
            )

    run_benches(repo, target_dir, ["--baseline", "base"], comparable)
    return check_changes(target_dir, args.threshold)


if __name__ == "__main__":
    sys.exit(main())
