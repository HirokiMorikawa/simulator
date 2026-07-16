# 04. アーキテクチャ — Solver / Coupling / Orchestrator

[02-scale-ladder.md](02-scale-ladder.md) の思想(スケールごとの有効理論 + 明示的結合)をソフトウェア構造に落とす。

## 1. 三層構造

```
┌────────────────────────────────────────────────────────────┐
│ World (facade)                                             │
│  シーン管理 / 公開API / 観測・記録 / シリアライズ              │
├────────────────────────────────────────────────────────────┤
│ Orchestrator                                               │
│  ステップパイプライン / operator splitting / sub-stepping    │
├───────────────┬──────────────────────────────┬─────────────┤
│ Solvers       │ Coupling                     │ Materials   │
│  mechanics    │  流体⇔剛体 (力・境界)          │  物性DB      │
│  fluid        │  力学→熱 (摩擦発熱)            │  (全ソルバ   │
│  thermal      │  熱→物性 (温度依存)            │   が参照)    │
│  em           │  EM⇔力学 (ローレンツ力)        │             │
│  quantum      │  EM⇔熱 (ジュール熱)           │             │
│  statistical  │  統計→力学 (ゆらぎ) ...        │             │
├───────────────┴──────────────────────────────┴─────────────┤
│ 基盤: sim-math (線形代数・場・積分器・PRNG) / sim-core (ID・イベント・時間) │
└────────────────────────────────────────────────────────────┘
```

### 1.1 Solver(ドメインソルバ)

各物理ドメイン(力学・流体・熱・電磁気・量子・統計)は独立したソルバモジュールである。

- 自分の**状態**(剛体集合、速度場、温度場、回路変数、波動関数…)を所有する。
- 自分の**支配方程式**を自分の**タイムステップ**で積分する。
- 他ソルバの内部状態に直接触れない。相互作用はすべて Coupling 経由。
- 共通トレイト:

```rust
/// すべてのドメインソルバが実装する。
pub trait Solver {
    /// このソルバが安定に積分できる最大タイムステップ(状態依存でよい)。
    /// Orchestrator が sub-step 数の決定に使う(CFL条件などから算出)。
    fn max_stable_dt(&self) -> f64;

    /// dt だけ状態を進める。dt <= max_stable_dt() が保証されて呼ばれる。
    fn step(&mut self, dt: f64, ctx: &mut SolverContext);

    /// 決定論検証・リプレイ照合用の状態ハッシュ。
    fn state_hash(&self, hasher: &mut StateHasher);

    /// このソルバが保持する全エネルギー(保存則の全体検算に使う)。
    fn total_energy(&self) -> EnergyBreakdown; // kinetic / potential / thermal / em / ...
}

/// step に渡される共有コンテキスト。
pub struct SolverContext<'a> {
    pub materials: &'a MaterialDb,
    pub rng: &'a mut SimRng,          // 決定論的PRNG (ソルバごとに独立ストリーム)
    pub events: &'a mut EventQueue,   // 接触・相変化などの通知
}
```

- ソルバは**シーンに応じて有効化**される。空のソルバはステップコストゼロ(剛体だけのシーンで
  流体・EM ソルバは動かない)。量子・統計ソルバは主に専用シナリオで使う([02-scale-ladder.md](02-scale-ladder.md) §2 の実装形態)。

### 1.2 Coupling(結合)

ドメイン間の相互作用を表す第一級のオブジェクト。ソルバ内部に他ドメインへの参照を埋め込まず、
「どのドメインからどのドメインへ、何を、いつ渡すか」を Coupling 層に集約する。
全結合の一覧と各結合の式は [20-integration/01-coupling-matrix.md](../20-integration/01-coupling-matrix.md) が正。

```rust
/// ドメイン間結合。2つ(以上)のソルバの状態を読み、互いに作用を書き込む。
pub trait Coupling {
    /// 依存するソルバ(実行順序の決定に使う)。
    fn domains(&self) -> (DomainId, DomainId);

    /// 結合の適用。例: 流体→剛体の力の集計、摩擦仕事→熱源項。
    fn apply(&mut self, world: &mut DomainStates, dt: f64);
}
```

設計原則:

- **保存量の橋は必ず対で書く**: 摩擦でエネルギーを散逸させたら、同じ量を熱源として計上する。
  結合は「取り出した量」と「入れた量」が一致することをデバッグビルドで検算する。
- 結合の粒度は「弱結合(operator splitting: 各ステップで力・源項を交換)」を基本とし、
  強い相互作用(流体中の軽い剛体など)で不安定になる場合のみ反復(sub-iteration)を導入する(判断基準は結合行列文書)。

### 1.3 Orchestrator(オーケストレータ)

タイムスケールの異なるソルバ群を単一のシミュレーション時刻に沿って進める。

- **World の基本ステップ** $\Delta t_{world} = 1/120$ s(固定)。
- 各ソルバは $\Delta t_{world}$ を自分の安定条件に合わせて**等分割 sub-stepping** する:
  $n_i = \lceil \Delta t_{world} / \Delta t_i^{max} \rceil$、$\Delta t_i = \Delta t_{world}/n_i$。
- ステップパイプライン(1 world step、固定順序 — 順序自体が決定論の一部):

```
1. 入力適用           ユーザー操作・エンティティ制御 → 力・目標値
2. Coupling (pre)     ソルバ間で力・源項を交換 (流体→剛体の力、摩擦発熱→熱源、EM力 など)
3. Solver group A     mechanics.step()   (衝突検出→接触ソルバ→積分 を内包)
4. Solver group B     fluid.step()  thermal.step()  (A と独立、将来並列化可)
5. Solver group C     em.step()  quantum.step()  statistical.step()
6. Coupling (post)    境界条件の更新 (剛体の新位置 → 流体の障害物、温度 → 物性)
7. イベント確定        接触・相変化・破壊などを EventQueue から購読者へ
8. 観測・記録          エネルギー集計、状態ハッシュ、リプレイチェックポイント
```

  ※ グループ分けは「同一ステップ内で互いの新状態を必要としない」ことが基準。
  pre/post の 2 相に分けることで、結合はすべて「前ステップの確定状態」を読む — 順序依存の隠れた結合を防ぐ。

- **時間の主権は World にある**: ソルバは自分から時間を進めない。描画層(デモ)はアキュムレータで
  world step を駆動し、表示は 2 ステップ間の補間で行う(実装フェーズの demo 設計)。

## 2. レイヤ依存規則

```
sim-math  ←  sim-core  ←  各ドメイン solver  ←  coupling  ←  world  ←  (wasm bindings / demo / tests)
```

- 依存は左から右への一方向のみ。ドメインソルバ同士は互いに依存しない(coupling のみが複数ドメインを知る)。
- `MaterialDb` は sim-core に置き、全ドメインが読む(書くのは World の初期化とごく限られた結合のみ。
  例: 温度依存物性は「読み出し時に温度を引数に取る」形にし、DB自体は不変に保つ)。
- 描画・UI・OS 依存コードはコアに一切入れない。コアの単体テストはネイティブ `cargo test` で完結する。

## 3. データ設計の方針

- **Structure of Arrays (SoA)**: 剛体・粒子・格子は属性ごとの `Vec<f64>` / `Vec<Vec3>` で持つ
  (キャッシュ効率と WASM への一括転送のため)。ID は世代付きインデックス
  (`BodyId = { index: u32, generation: u32 }`)で削除・再利用を安全にする。
- **場 (field)** は `sim-math::Grid3<T>`(セル中心)と `MacGrid`(スタガード)で統一
  ([01-math/02-fields.md](../01-math/02-fields.md))。流体・熱・EM が同じ格子基盤を共有する。
- **イベント**は push 型(EventQueue に積み、ステップ末尾でまとめて配送)。コールバック中の状態変更は禁止
  (変更要求はコマンドキューに積み、次ステップ先頭で適用 — 再入とステップ途中の不整合を防ぐ)。
- **スナップショット**: World 全状態は決定的順序でシリアライズ可能(リプレイ・保存・undo の基盤)。

## 4. 拡張点(将来のドメイン・機能)

- 新ドメイン追加 = `Solver` 実装 + 結合行列への行追加。既存ソルバの変更を要求しない。
- 同一ドメイン内の解法差し替え(例: 流体の格子法⇔SPH)は、ドメイン内トレイト(`FluidSolver` など)で行い、
  Coupling から見えるインターフェース(力の照会・境界条件の設定)を不変に保つ。
- エンティティ層([20-integration/03-entity-layer.md](../20-integration/03-entity-layer.md))は World の公開 API
  ([20-integration/04-world-api.md](../20-integration/04-world-api.md))のみを使う。エンジン内部への特権的アクセスを持たない —
  エンティティが要求する機能が API に無ければ、それは API 設計の欠陥として扱う。

## 5. エラー・数値破綻の方針

- ソルバは発散を検知したら(速度・エネルギーの閾値超過、NaN)、パニックせず `SolverDiagnostics` に記録して
  World に伝える。World は「直前スナップショットへの巻き戻し + 警告」をデフォルト動作とする。
  検証して遊ぶツールとして、**壊れたまま進む**ことと**黙って落ちる**ことの両方を避ける。
- すべての安定条件(CFL 等)は実行時に検査され、違反時は自動 sub-step 増加(上限あり)→ 上限超過で診断イベント。
