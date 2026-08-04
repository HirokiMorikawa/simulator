# simulator

地球の物理法則を Rust で実装し、ブラウザ上で検証しながら遊べる物理エンジン。

力学・流体・熱・電磁気・量子・統計・天体の 7 ドメインを、それぞれ独立した `Solver` として実装する。
スケールごとに最良の理論を選び、理論間の受け渡しを `Coupling` として明示的に設計する
——「スケールの階梯と有効理論」([docs/00-foundation/02-scale-ladder.md](docs/00-foundation/02-scale-ladder.md))が本エンジンの根本思想である。
物理コアは WebAssembly にコンパイルされ、Three.js 製の統合エディタから操作できる。

## 特徴

- **7 ドメインを 1 つの `World` に統合** — `sim_world::World` が有効化されたドメインを固定順(力学 → 熱 → 電磁 → 天体 → 回路 → SPH → 格子流体)で進め、Lie–Trotter 分割で前後に結合処理を挟む。結合は 15 モジュール実装済み([crates/sim-coupling/src/](crates/sim-coupling/src/): `lorentz_force`・`joule_heat`・`phase_change_morph`・`sph_rigid`・`piston_gas` ほか)。
- **検証可能性** — `#[test]` 644 件が、解析解テスト表([docs/21-verification/01-analytic-tests.md](docs/21-verification/01-analytic-tests.md) の M/F/T/E/Q/S/A/R 系 ID)と保存則([docs/21-verification/02-conservation-laws.md](docs/21-verification/02-conservation-laws.md))に紐づく。`EnergyLedger` が実行中の数値誤差を常時可視化する。
- **決定論** — 固定タイムステップ・シード付き PRNG・状態ハッシュにより、同条件ならビット単位で同じ結果になる。リプレイ・共有・回帰テストの基盤。
- **近似の自己申告** — 各 `Solver` は使用中の近似を `sim_core::Approximation { name, reason, doc, can_disable }` として申告し、エディタがバッジで表示する。何を諦めているかが実行時に見える。
- **依存が実質ゼロ** — 線形代数・FFT・PRNG・PNG 書き出しに至るまで自前実装。外部クレートは物理コアの `serde`/`serde_json`、WASM 境界の `wasm-bindgen`/`js-sys`、ベンチの `criterion` のみ。
- **ブラウザ統合エディタ** — Edit/Play モード、W/E/R/Q ツールと Gizmo、Hierarchy / Inspector / Timeline(スナップショットリング + ブックマーク)/ Console(種別タブ)/ Probe グラフ(CSV エクスポート)/ Project ドロワー([docs/23-frontend/01-editor.md](docs/23-frontend/01-editor.md))。
- **物理正確なパストレーシング** — 分光レンダリング、GGX マイクロファセット、binned-SAH BVH、MIS、コースティクス、参加媒質、CIE 等色関数 + ACES トーンマップ([crates/sim-render](crates/sim-render/))。

## クイックスタート

### 必要環境

| 用途 | 要件 |
|---|---|
| 物理コア | Rust stable(edition 2021) |
| WASM ビルド | `wasm32-unknown-unknown` ターゲット + [wasm-pack](https://rustwasm.github.io/wasm-pack/) |
| デモ | Node.js 22 |

### ビルドと起動

```bash
# 1. 物理コアのテスト(全ドメインの解析解テスト・決定論テスト)
cargo test --workspace

# 2. WASM をビルドして demo/pkg に出力する
#    demo/pkg は .gitignore 済みのため、デモを動かす前に必ず実行する
rustup target add wasm32-unknown-unknown
wasm-pack build crates/sim-wasm --target web --out-dir ../../demo/pkg

# 3. デモ(統合エディタ)を起動する
cd demo
npm ci
npm run dev
```

Vite が表示する URL をブラウザで開くとエディタが立ち上がる。

## 使い方

### デモシーン

[scenes/](scenes/) に検証用シーンが 43 本ある。[scenes/index.json](scenes/index.json) がタイトル・説明・関連ドメインのマニフェストで、エディタのシーン選択はこれを読む。

| ID | シーン | 主なドメイン |
|---|---|---|
| D1 | 落下時計 — 落下時間が解析解 $t=\sqrt{2h/g}$ と一致するか | 力学 |
| D14 | 煙と渦 — 渦度強化つき格子流体 | 流体 |
| D19 | 電気工作台 — MNA 回路をその場で編集する | 電磁気・熱 |
| D24 | 車の実験場 — WheelJoint と簡易 Pacejka タイヤ | 力学 |
| D27 | 二重スリット(電子) — 2D 時間依存シュレディンガー方程式 | 量子 |
| D36 | スイングバイ — N 体重力と軌道 | 天体 |

各シーンの合否基準は [docs/21-verification/03-demo-scenarios.md](docs/21-verification/03-demo-scenarios.md) にある。D40–D43 はオフラインのパストレーサ用で、`cargo run --release -p sim-render --example render_demos -- <出力先>` が画像を生成する。

### エディタの操作

- **Edit / Play** — Edit はシミュレーション停止状態での直接編集、Play は `Command` 経由の操作のみを許す実行モード。
- **ツール** — `W` 移動 / `E` 回転 / `R` 拡縮 / `Q` 選択、`X` で Gizmo の World / Local 切替。
- **時間** — 再生・一時停止・指定 step 数のステップ実行、×1/8〜×128 の時間倍率(実効レート表示つき)。
- **レイアウト** — Default / 力学重点 / 回路重点 / 天体 の 4 プリセット。
- **Console** — All / Errors / Warnings / Info / Contacts / Events のタブ。イベント行をクリックすると発生源のボディを選択し、Timeline がその時刻へ飛ぶ。

### シーン JSON

シーンは `sim_world::Scenario`([crates/sim-world/src/scenario.rs](crates/sim-world/src/scenario.rs))が読む JSON で記述する。最小の例([scenes/d1-free-fall.json](scenes/d1-free-fall.json)):

```json
{
  "name": "d1-free-fall",
  "world": { "gravity": 9.80665, "dt": 0.008333333 },
  "bodies": [
    {
      "shape": { "sphere": { "radius": 0.3 } },
      "material": "鋼(炭素鋼)",
      "position": [0.0, 20.0, 0.0],
      "name": "clock"
    }
  ],
  "probes": [{ "body_pos_y": "clock" }]
}
```

セクションは `world` / `materials` / `bodies` / `fluids` / `thermal` / `joints` / `couplings` / `circuit` / `probes`。読み込み時に材料名・剛体名の参照整合と、二重計上を招く排他結合の同時有効化を検査する。

### Rust から使う

```rust
use sim_world::{run_headless_scenario, World, WorldOptions};

// シーン JSON をヘッドレスで回してプローブ履歴と状態ハッシュを得る
let json = std::fs::read_to_string("scenes/d1-free-fall.json")?;
let result = run_headless_scenario(&json, 600)?;
assert_eq!(result.probe_histories.len(), 1);

// あるいは World を直接組み立てる
let mut world = World::new(WorldOptions::default()); // gravity 9.80665, dt 1/120, seed 0
world.step();
let hash = world.state_hash(); // 同条件なら常に同じ値
```

各ドメインは `enable_thermal` / `enable_sph` / `enable_quantum_2d` などで個別に有効化する。すべてのソルバは共通の `Solver` トレイト([crates/sim-core/src/solver.rs](crates/sim-core/src/solver.rs))を実装する。

```rust
pub trait Solver {
    fn max_stable_dt(&self) -> f64;
    fn step(&mut self, dt: f64, ctx: &mut SolverContext);
    fn state_hash(&self, hasher: &mut StateHasher);
    fn total_energy(&self) -> EnergyBreakdown;
    fn approximations(&self) -> Vec<Approximation> { Vec::new() }
}
```

## アーキテクチャ

```
crates/    Rust ワークスペース(13 crate、約 58,000 行)
demo/      Vite + TypeScript + Three.js の統合エディタ
scenes/    検証デモシーン(43 本の JSON + マニフェスト)
docs/      設計書(日本語 59 文書)
scripts/   CI/開発用の Python ヘルパ 2 本
```

crate は下から順に積み上がる。

| crate | 役割 |
|---|---|
| [sim-math](crates/sim-math/) | `Vec3`/`Quat`/`Mat3`、場の補間、数値積分、決定論的 PRNG。依存ゼロ |
| [sim-core](crates/sim-core/) | `Solver` トレイト、`EnergyLedger`、`Approximation`、`MaterialDb`、状態ハッシュ、イベントキュー |
| [sim-mechanics](crates/sim-mechanics/) | 剛体、BVH broadphase、SAT/GJK/EPA、sequential impulses、摩擦、ジョイント、XPBD ソフトボディ、CCD、車両 |
| [sim-fluid](crates/sim-fluid/) | MAC 格子 Eulerian 2D/3D(PCG 圧力解法・3D はマルチグリッド前処理・渦度強化)、WCSPH、浮力、空力・水力 |
| [sim-thermal](crates/sim-thermal/) | 熱伝導・対流・放射、相変化(エンタルピー法)、気体コンパートメント |
| [sim-em](crates/sim-em/) | 静電磁場、MNA 回路(非線形素子つき)、FDTD(PML 吸収境界)、幾何光学、モーター |
| [sim-quantum](crates/sim-quantum/) | 1D/2D 時間依存シュレディンガー(split-step Fourier)、虚時間発展による固有状態探索 |
| [sim-statistical](crates/sim-statistical/) | 剛体球気体 MD、Langevin/ブラウン運動(BAOAB)、2D Ising(Metropolis / Wolff) |
| [sim-astro](crates/sim-astro/) | N 体重力、軌道遷移、J2 摂動、大気再突入、スイングバイ、1PN 相対論補正(オプトイン) |
| [sim-render](crates/sim-render/) | 分光パストレーサ、BSDF 各種、binned-SAH BVH、MIS、コースティクス、物理カメラ、PNG 書き出し |
| [sim-coupling](crates/sim-coupling/) | ドメイン間結合 15 種([docs/20-integration/01-coupling-matrix.md](docs/20-integration/01-coupling-matrix.md)) |
| [sim-world](crates/sim-world/) | `World` ファサード、ステップパイプライン、シーン JSON、プローブ、フレーム木、ヘッドレスランナー |
| [sim-wasm](crates/sim-wasm/) | `WasmWorld` — ブラウザ向けの `wasm-bindgen` 境界 |

## 開発

### チェック

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace

cd demo
npm run build      # tsc --noEmit + vite build
npm run test:e2e   # Playwright(chromium)による配線スモークテスト
```

Playwright のスモークテストが守るのは「起動し、wasm が初期化され、主要な操作でクラッシュしない」という配線の健全性だけである。物理の正しさは Rust 側の解析解テストが担保する。

### ベンチマーク

criterion ベンチが 4 本ある(`sim-mechanics`: `contact_solver`、`sim-fluid`: `grid_fluid_pcg` / `grid_fluid3d_mgpcg` / `sph_neighbor_search`)。3D の解像度スケーリングは criterion では重すぎるので `cargo run --release -p sim-fluid --example grid_fluid3d_bench` が別に測る(32³/64³/128³)。

```bash
cargo bench --workspace
python3 scripts/check_bench_regression.py --threshold 0.25
```

回帰ゲートは merge-base を `git worktree` で取り出し、同一マシン上で作業ツリーと A/B 比較する。criterion 自体は回帰を表示するだけで exit 0 のままなので、`change/estimates.json` を読んで閾値超過で落とすのがこのスクリプトの役目。既定閾値 25% は実測のノイズ床(同一コードで約 6%)に基づく。

設計文書 §7 が要求するユニットテストが実装側に存在するかは `python3 scripts/audit_domain_section7.py` が機械的に監査する。

### CI

[.github/workflows/ci.yml](.github/workflows/ci.yml) が `main` への push と全 PR で走る。

| ジョブ | 内容 |
|---|---|
| `native` | fmt / clippy / `cargo test --workspace` / ベンチ回帰ゲート |
| `wasm` | `wasm-pack build` して `demo/pkg` を成果物としてアップロード |
| `demo` | その成果物を使って `npm run build` + Playwright スモーク |

## ドキュメント

設計書は [docs/README.md](docs/README.md) が入口(全体目次と読み順ガイド)。

| セクション | 内容 |
|---|---|
| [00-foundation](docs/00-foundation/) | ビジョン、スケールの階梯と有効理論、単位・規約、アーキテクチャ、Rust+WASM、性能戦略 |
| [01-math](docs/01-math/) | 線形代数、場(格子・粒子)、数値積分、決定論的乱数 |
| [10-mechanics](docs/10-mechanics/) | 剛体、衝突検出、接触ソルバ、摩擦、ジョイント、ソフトボディ |
| [11-fluid](docs/11-fluid/) | Navier–Stokes、格子法、SPH、浮力、空力・水力 |
| [12-thermal](docs/12-thermal/) | 熱力学法則、熱伝達、相変化、材料物性 DB |
| [13-electromagnetism](docs/13-electromagnetism/) | 静電磁場、回路(MNA)、FDTD、光学、モーター結合 |
| [14-quantum](docs/14-quantum/) | 量子力学の役割と限界、シュレディンガーソルバ、有効モデル |
| [15-statistical](docs/15-statistical/) | ミクロ⇔マクロ、気体分子運動論、ブラウン運動、モンテカルロ |
| [16-astro](docs/16-astro/) | N 体重力・軌道・再突入、相対論オプトイン |
| [17-rendering](docs/17-rendering/) | 物理正確フルパストレーシング |
| [20-integration](docs/20-integration/) | ドメイン間結合行列、決定論・リプレイ、エンティティ層、World API |
| [21-verification](docs/21-verification/) | 解析解テスト表、保存則、デモシナリオ集 |
| [22-roadmap](docs/22-roadmap/) | 実装フェーズ計画(TDD: Phase A/B/C/D)と機能チェックリスト |
| [23-frontend](docs/23-frontend/) | 統合エディタの設計 |
| [reviews](docs/reviews/) | 設計レビュー・品質チェックの記録と対応 |

文書は日本語で、数式は GitHub Markdown の `$...$` 記法を使う。数値パラメータには必ず出典を、近似には必ず物理的な正当化を添える([docs/README.md](docs/README.md) の「文書規約」)。

## プロジェクトの状態

Phase 0(骨格)/ A(テスト先行)/ B(ドメイン実装)/ C(結合)/ D(レンダリング)はすべて完了し、機能チェックリストは 262 項目すべて消化済み。進行状況の唯一の記録は [docs/22-roadmap/02-feature-checklist.md](docs/22-roadmap/02-feature-checklist.md)。

既知の制約:

- **3D 格子流体の性能** — 64³ で 206.8 ms/step(設計予算 4 ms に対し約 50 倍)。マルチグリッド前処理 PCG([crates/sim-fluid/src/pressure_multigrid.rs](crates/sim-fluid/src/pressure_multigrid.rs))の導入で 795.7 ms/step から約 3.5 倍縮み、圧力投影の反復数は解像度に依存しなくなった(32³/64³/128³ のいずれも 7〜8 反復)が、SIMD・並列化・GPU が未着手なため予算には届かない([crates/sim-fluid/examples/grid_fluid3d_bench.rs](crates/sim-fluid/examples/grid_fluid3d_bench.rs))。
- **GPU 未対応** — 現状は CPU 実装のみ。WebGPU は設計上の選択肢として残してある([docs/00-foundation/06-performance-strategy.md](docs/00-foundation/06-performance-strategy.md))。

## コントリビュート

Issue / Pull Request を歓迎する。

- PR は CI(fmt / clippy / test / ベンチ回帰ゲート / wasm ビルド / Playwright スモーク)を通すこと。
- 物理の変更には、対応する解析解テストか保存則テストを添えること。近似を導入する場合は `Solver::approximations()` で申告すること。
- 設計文書を書き足す場合は [docs/README.md](docs/README.md) の「文書規約」に従うこと。
