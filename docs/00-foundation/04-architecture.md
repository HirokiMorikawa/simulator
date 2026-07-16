# 04. アーキテクチャ — Solver / Coupling / Orchestrator / World

[02-scale-ladder.md](02-scale-ladder.md) の思想(スケールごとの有効理論 + 明示的結合)をソフトウェア構造に落とす。
本エンジンの三層は単なる「配管」ではなく、それぞれが**独立した理論的内容**を担う:

- **Solver** は各ドメインの**支配方程式(物理法則)**を担い、環境を「表現する / 制御する」唯一の道具である。
- **Coupling** は**保存量の橋**(結合前後のエネルギー・運動量の恒等)を担う。
- **Orchestrator** は**多物理系の時間発展の理論**(operator splitting・タイムスケール分離)を担う。
- **World** は**シミュレートされた環境そのものの定義**(状態の唯一の真理・不変条件の束)を担う。

## 1. 四層構造

```
┌────────────────────────────────────────────────────────────┐
│ World (§1.1) : シミュレートされた環境そのものの定義          │
│   状態オーナーシップの一意性 / エネルギー台帳の恒等式         │
│   決定論的同値関係 / 世界時刻の一意性 / 因果順序の全順序化    │
├────────────────────────────────────────────────────────────┤
│ Orchestrator (§1.4) : 多物理系の時間発展理論                 │
│   operator splitting / タイムスケール分離                    │
│   pre/post 二相の順序独立性 / イベント全順序                 │
├───────────────┬──────────────────────────────┬─────────────┤
│ Solver (§1.2) │ Coupling (§1.3)              │ Materials    │
│  各ドメインの │  保存量の橋                    │  物性DB     │
│  支配方程式   │  (取り出した量=注入した量)      │  (全ソルバ  │
│  mechanics    │  流体⇔剛体 (力・境界)          │   が読む)   │
│  fluid        │  力学→熱 (摩擦発熱)            │             │
│  thermal      │  熱→物性 (温度依存)            │             │
│  em           │  EM⇔力学 (ローレンツ力)        │             │
│  quantum      │  EM⇔熱 (ジュール熱)           │             │
│  statistical  │  統計→力学 (ゆらぎ) ...        │             │
│  astro        │                              │             │
├───────────────┴──────────────────────────────┴─────────────┤
│ 基盤: sim-math (線形代数・場・積分器・PRNG) / sim-core (ID・イベント・時間) │
└────────────────────────────────────────────────────────────┘
```

四層は独立した内容を持ち、以下の対比で整理される:

| 層 | 何を担うか(理論・内容) | 唯一の関心事 | 参照する外部理論 |
|---|---|---|---|
| **Solver**(§1.2) | 各ドメインの**支配方程式**(法則) | 「この物理現象はどう時間発展するか」 | Navier-Stokes・マクスウェル方程式・シュレディンガー方程式など |
| **Coupling**(§1.3) | ドメイン間の**保存量の橋** | 「取り出したものを行き先に等量入れる」 | 熱力学第 1 法則(結合前後のエネルギー恒等) |
| **Orchestrator**(§1.4) | **多物理系の時間発展理論** | 「異なる時間スケールの Solver 群を単一の世界時間に沿って進める」 | operator splitting、タイムスケール分離、決定論的スケジューリング |
| **World**(§1.1) | **シミュレートされた環境そのものの定義** | 「この世界とは何か、状態の唯一の真理」 | 状態同値関係、因果順序、エネルギー台帳の恒等式 |

Solver が環境を「表現する」道具であるのに対し、World は環境そのものが何であるかを規定し、
Orchestrator は時間がどう流れるかを規定する。

## 1.1 World — シミュレートされた環境そのものの定義

### 1.1.1 World とは何か

World は本エンジンにおいて「**シミュレートされた環境の全状態を単一の意味論的単位として
凝集したオブジェクト**」である。時間、場所、そこにある物・場・エネルギーの帳簿、因果の履歴 —
これらすべての正典(canonical)な所在である。

Solver が「その中で法則を実行する主体」であるのに対し、World は「法則が実行される場そのもの」。
World は物理法則を持たない代わりに、**環境が満たすべき不変条件**を持つ。

### 1.1.2 World が担う 5 つの不変条件

**(1) 状態オーナーシップの一意性**

任意の物理量(剛体位置・場の値・回路電圧など)について、その**正典の保管場所は World 内でただ 1 箇所**
であり、他の場所からの参照はすべて借用として明示される。Solver は自ドメイン状態を World から借りて
読み書きするのであって、Solver が「所有」しているのではない。

この不変条件により、スナップショット・リプレイ・並列化・undo が well-defined になる
(状態のコピーが複数あれば「どれが正か」の曖昧さで壊れる)。

**(2) エネルギー台帳の恒等式**

$$\sum_{d \in \text{Domain}} E_d(t) \;=\; E_{\text{total}}(0) \;+\; W_{\text{injected}}(t) \;-\; R_{\text{numerical}}(t)$$

- $E_d$: 各ドメインの全形態エネルギー(運動・ポテンシャル・弾性・熱・電磁場・化学…)
- $W_{\text{injected}}$: モーター・ユーザー操作等の外部注入(符号つき)
- $R_{\text{numerical}}$: 既知の数値散逸(Baumgarte 偽仕事・移流散逸)の**明示的な計上**

この式が破れることは「不明な源からのエネルギー漏れ・湧き」を意味し、バグとして CI で監視する
([21-verification/02-conservation-laws.md](../21-verification/02-conservation-laws.md))。
World の第一級の役割は、この恒等式が全ドメイン・全結合を横断して常に成立するように
帳簿(`EnergyLedger`)を提供・強制することである。

**(3) 決定論的同値関係**

初期シナリオ $W_0$・マスタシード $s$・実行中コマンド列 $C_{0:n}$ の三つ組から生成される
状態列 $(S_1, \ldots, S_n)$ について、次の同値関係が成立する:

$$(W_0, s, C_{0:n}) \equiv (W_0', s', C_{0:n}') \;\Longrightarrow\; (S_1, \ldots, S_n) = (S_1', \ldots, S_n')$$

(同一入力ならビット単位で同一)。この同値関係が、リプレイ・シーン共有・回帰テスト・
バグ再現の**数学的基盤**である([20-integration/02-determinism-replay.md](../20-integration/02-determinism-replay.md))。
World は同値関係を破りうる情報源(壁時計・OS 乱数・非決定的並列化)をコアに入れないという
規律の**執行者**でもある。

**(4) 世界時刻の一意性と単調性**

全 Solver・全 Coupling が参照する時刻は World の `SimClock` 一つのみ。実時計・壁時計から独立し、
$\Delta t_{\text{world}}$ ずつ厳密に単調増加する。ソルバの独立時間軸(FDTD・量子・天体)は
World 時刻の **入れ子時間** として定義され、World 時刻を追い越さない。

これは相対論的な同時刻を主張するものではなく、シミュレーション上の**唯一の時間座標**を World が
規定するという設計上の宣言である。

**(5) 因果順序の全順序化**

ステップ内に発生した全イベント(接触・相変化・スイッチ操作・破断など)は $(step, source\_id, kind)$ の
辞書式**全順序**でソートされ、この順序で購読者に配送される。並列実行下でも順序は変わらない。
これにより「A が起きた後に B が起きた」という因果的言明が確定的に成立する
([20-integration/02](../20-integration/02-determinism-replay.md))。

### 1.1.3 World が管理する状態空間の分解

任意の時刻 $t$ における世界のスナップショットは以下の 3 部分に分解される:

$$\Sigma_t \;=\; \bigl(\, S_{\text{phys}},\; S_{\text{ledger}},\; S_{\text{causal}} \,\bigr)_t$$

- **物理状態 $S_{\text{phys}}$**: 全ドメインの正典状態(剛体・場・粒子・回路変数・波動関数…)
- **会計状態 $S_{\text{ledger}}$**: エネルギー内訳・保存量集計・診断情報
- **因果状態 $S_{\text{causal}}$**: イベントキュー・コマンドキュー・時刻・PRNG マスタ状態

スナップショットは決定論的順序でシリアライズ可能。$\Sigma_t \to \Sigma_{t+\Delta t}$ の遷移は
Orchestrator が Solver 群と Coupling 群を呼び出して実行する(§1.4)が、遷移の**正しさ**を判定する
基準は上記 5 不変条件の維持である。

### 1.1.4 公開 API との関係

本節は「World は何か」の**意味論**を定める。呼び出し可能な操作の一覧(`create_body` / `step` /
`snapshot` / `push_command` 等)は [20-integration/04-world-api.md](../20-integration/04-world-api.md)。
API のシグネチャは本節の不変条件から演繹的に決まる:

- 「読み取りは自由、書き込みは規律」 ← §1.1.2 (1) 状態オーナーシップ
- 「コマンドキュー経由の変更」 ← §1.1.2 (3) 決定論的同値関係
- 「スナップショット / リプレイ」 ← §1.1.2 (3) と §1.1.3
- 「保存量パネル・observables」 ← §1.1.2 (2) エネルギー台帳の恒等式

### 1.1.5 Rust 型スケッチ(§1.1.2 の各不変条件を担うフィールドの明示)

```rust
pub struct World {
    /// §1.1.2 (4) 世界時刻の一意性: World が唯一の時刻を持つ。fixed dt、単調増加。
    clock: SimClock,

    /// §1.1.2 (1) 状態オーナーシップ: 全 Solver の正典状態はここに凝集される。
    solvers: SolverRegistry,          // Vec<Box<dyn Solver>> + DomainId 索引

    /// §1.3 保存量の橋: Coupling は正典状態の借用のみを介して作用する。
    couplings: CouplingRegistry,

    /// §1.4 時間発展の理論の実装。World は状態オーナー、Orchestrator が時間駆動を担当する分業。
    orchestrator: Orchestrator,

    /// 全ドメインが読む物性データベース。不変(構築時のみ書換)。
    materials: MaterialDb,

    /// §1.1.2 (3) 決定論の要: 唯一の乱数源からドメイン別ストリームを派生。
    rng: SimRngMaster,

    /// §1.1.2 (5) 因果順序: 全イベントの一時保管、全順序化 → 配送は Orchestrator が担う。
    events: EventQueue,

    /// §1.1.2 (3) 決定論: 実行中の状態変更はここに積まれ、step 先頭で決定的順序で適用される。
    commands: CommandQueue,

    /// §1.1.2 (2) エネルギー台帳の恒等式: 全ドメイン・全結合を横断した会計。
    ledger: EnergyLedger,

    /// §1.1.3 スナップショット系列: シナリオ + コマンド列と併せてリプレイの完全性を保証。
    recorder: Option<ReplayRecorder>,
}
```

## 1.2 Solver — 各ドメインの支配方程式(法則)

各物理ドメイン(力学・流体・熱・電磁気・量子・統計・天体)は独立したソルバモジュールである。
**Solver は環境の中の物理法則そのもの**を担い、環境を表現・制御する唯一の道具である
(World は状態の器、Coupling は保存量の橋、Orchestrator は時間の駆動 — 物理法則は Solver に凝集する)。

- 自分の**状態**(剛体集合、速度場、温度場、回路変数、波動関数…)の**中身**を規定する(器は World)。
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

## 1.3 Coupling — 保存量の橋

ドメイン間の相互作用を表す第一級のオブジェクト。ソルバ内部に他ドメインへの参照を埋め込まず、
「どのドメインからどのドメインへ、何を、いつ渡すか」を Coupling 層に集約する。
全結合の一覧と各結合の式は [20-integration/01-coupling-matrix.md](../20-integration/01-coupling-matrix.md) が正。

**Coupling が担う中身は「保存量の対応則」である**: 熱力学第 1 法則の要請として、結合前後で
エネルギー(あるいは運動量・電荷)は移動するだけであり、消失も湧出もしない。この対応則を
明示的なコードとして具現化するのが Coupling である。

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
  結合は「取り出した量」と「入れた量」が一致することをデバッグビルドで検算する
  (§1.1.2 (2) エネルギー台帳の恒等式の Coupling 側からの担保)。
- 結合の粒度は「弱結合(operator splitting: 各ステップで力・源項を交換)」を基本とし、
  強い相互作用(流体中の軽い剛体など)で不安定になる場合のみ反復(sub-iteration)を導入する(判断基準は結合行列文書)。

## 1.4 Orchestrator — 多物理系の時間発展理論

Orchestrator は「異なる支配方程式・異なる自然時間スケールを持つ Solver 群を、単一の世界時刻に沿って
時間発展させる」という**独立した数学的問題**を担う。単一ドメインの ODE ソルバ(積分器)とは別物であり、
Orchestrator 固有の理論的内容がある。

### 1.4.1 Operator Splitting の理論(実装の原理)

世界の全時間発展作用素 $\Phi_{\Delta t}$ を、ドメインごとの時間発展作用素 $\Phi_i^{\Delta t}$ の**合成**で近似する:

$$\Phi_{\Delta t} \;\approx\; \Phi_1^{\Delta t} \circ \Phi_2^{\Delta t} \circ \cdots \circ \Phi_N^{\Delta t}$$

これを **operator splitting** と呼ぶ。局所打ち切り誤差:

- **1 次分割(Lie-Trotter)** $\prod_i \Phi_i^{\Delta t}$: 誤差 $O(\Delta t^2)$
- **2 次分割(Strang 対称化)** $\prod_i \Phi_i^{\Delta t/2} \cdot \prod_i^{\text{逆順}} \Phi_i^{\Delta t/2}$: 誤差 $O(\Delta t^3)$

本エンジンの既定は 1 次分割(実装単純・十分)。精度モードで Strang 対称分割を選べる設計とする。
分割誤差は $R_{\text{numerical}}$ の一部として台帳に計上され、§1.1.2 (2) の恒等式との整合を保つ。

**この分割が可能である理由**は「各 Solver の作用素が自分のドメイン変数にしか作用しない」ことに由来する。
これを担保するのが §1.2 の「他ソルバの内部状態に触れない」規約であり、§1.1.2 (1) の状態オーナーシップの
一意性である。三つの層は互いに独立でなく、Orchestrator の理論的可能性を Solver / World が支えている。

### 1.4.2 タイムスケール分離(sub-step 数の決定的算出)

各 Solver $i$ が自身の安定条件から申告する最大安定刻み $\Delta t_i^{\max}$(状態依存でよい、例: CFL 条件・
シンプレクティック安定範囲)から、sub-step 数を次式で**一意に**決める:

$$n_i \;=\; \left\lceil \Delta t_{\text{world}} \,/\, \Delta t_i^{\max} \right\rceil, \qquad
\Delta t_i \;=\; \Delta t_{\text{world}} \,/\, n_i$$

- $\lceil \cdot \rceil$ 演算は状態から**決定的**に算出される。壁時計・浮動小数の非決定的比較を用いない —
  これが適応刻みの非決定性を排除し、§1.1.2 (3) の同値関係を守る鍵。
- **独立時間軸ドメイン**(天体・FDTD・量子)は上記公式を World 時刻に埋め込まず、自ドメインの
  大 / 小刻みで進む。近接イベント(再突入・パルス到達など)で状態から**決定論的レジーム切替**を行う
  ([00-foundation/02-scale-ladder.md](02-scale-ladder.md) §2.3)。

### 1.4.3 pre/post 二相の順序独立性

パイプラインを次の 3 段に整理する(1 world step 内):

1. **入力適用**: ユーザー操作・エンティティ制御 → 力・目標値
2. **Coupling (pre)**: ソルバ間で力・源項を交換(流体→剛体の力、摩擦発熱→熱源、EM 力 など)
3. **Solver 群 A**: `mechanics.step()`(衝突検出→接触ソルバ→積分 を内包)
4. **Solver 群 B**: `fluid.step()`  `thermal.step()`(A と独立、将来並列化可)
5. **Solver 群 C**: `em.step()`  `quantum.step()`  `statistical.step()`  `astro.step()`
6. **Coupling (post)**: 境界条件の更新(剛体の新位置 → 流体の障害物、温度 → 物性)
7. **イベント確定**: 接触・相変化・破壊などを EventQueue から購読者へ全順序で配送(§1.4.4)
8. **観測・記録**: エネルギー集計、状態ハッシュ、リプレイチェックポイント

**順序独立性の主張**: pre 段の Coupling はすべて「前ステップ確定状態のみ」を読み書きする。したがって
pre 内での実行順序を入れ替えても結果は不変である。同様に Solver 群内の A/B/C 分けは「同一ステップ内で
互いの新状態を必要としない」ことが基準であり、群内順序が結果を変えない。**これが並列化余地と決定論の
両立**を可能にする(順序変更が安全であることは、任意の決定的順序で実行しても同じ結果になることを含意)。

### 1.4.4 保存量の橋の対応則(Coupling との契約)

Orchestrator は Coupling ごとに「出所ドメイン → 行き先ドメイン、量 $Q$」のペアを台帳へ**2 度記帳**する
($Q$ を出所から引き、行き先に足す)。この対応則が破れる Coupling は Coupling 実装のバグとして
デバッグビルドで診断される — これは §1.1.2 (2) のエネルギー台帳恒等式の Orchestrator 側からの担保。

### 1.4.5 イベント順序の全順序化(決定論契約)

ステップ内に発生した全イベントを $(step, source\_id, kind)$ で辞書式**全順序**にソートし、この順序で
購読者に配送する。並列実行下でも順序は変わらない — この契約が §1.1.2 (5) の因果順序を実装する。

### 1.4.6 Rust 型スケッチ(§1.4.1〜§1.4.5 の各原理を担うフィールドの明示)

```rust
pub struct Orchestrator {
    /// §1.1.2 (4) 世界時刻: World 基本ステップ (fixed)。既定 1/120 s。
    pub dt_world: f64,

    /// §1.4.3 順序独立性: ソルバ実行のグループ分け。群内独立性が並列化を可能にする。
    groups: Vec<SolverGroup>,          // Vec<Vec<DomainId>>

    /// §1.4.3 pre/post 二相: Coupling の実行順序 (決定的順序で確定)。
    coupling_order: CouplingOrder,     // {pre: Vec<CouplingId>, post: Vec<CouplingId>}

    /// §1.4.2 タイムスケール分離: 前ステップの sub-step 数のキャッシュ (診断表示・回帰検知)。
    last_substeps: BTreeMap<DomainId, u32>,

    /// §1.4.1 分割精度: 既定 Trotter (1次)、精度モードで Strang (2次)。
    splitting: SplittingScheme,
}

impl Orchestrator {
    /// §1.4.1 分割の実行 + §1.4.3 二相 + §1.4.4 対応則の記帳 + §1.4.5 順序化。
    /// 副作用は引数の &mut への書き込みのみ。step 順序はここで完全に確定する — 決定論の要。
    pub fn step(
        &mut self,
        solvers: &mut SolverRegistry,
        couplings: &mut CouplingRegistry,
        materials: &MaterialDb,
        rng: &mut SimRngMaster,
        events: &mut EventQueue,
        ledger: &mut EnergyLedger,
    );

    /// §1.4.2 sub-step 公式の実装。状態から決定的に算出。適応刻みの非決定性を排除。
    fn compute_substeps(&self, solvers: &SolverRegistry) -> BTreeMap<DomainId, u32>;
}
```

## 2. レイヤ依存規則

```
sim-math  ←  sim-core  ←  各ドメイン solver  ←  coupling  ←  world (Orchestrator を含む)  ←  (wasm bindings / demo / tests)
```

- 依存は左から右への一方向のみ。ドメインソルバ同士は互いに依存しない(coupling のみが複数ドメインを知る)。
- `Orchestrator` は `sim-world` crate 内(World と同居)。新 crate は作らない — 時間発展理論は
  World の状態オーナーシップと不可分な位置関係にあるため。
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
- **スナップショット**: World 全状態は決定的順序でシリアライズ可能(リプレイ・保存・undo の基盤、§1.1.3)。

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
