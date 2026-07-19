# 03. 数値積分カタログ — 選定・安定性・使い分け

常微分方程式 $\dot{y} = f(t, y)$ の時間積分法。どのドメインがどの積分器を使うかをここで一元管理する。
crate: `sim-math`。

## 1. 使い分けの結論(一覧)

| 用途 | 積分器 | 次数 | 理由 |
|---|---|---|---|
| 剛体(接触あり) | semi-implicit Euler | 1 | インパルスソルバと整合、エネルギー的に安定 |
| 剛体の姿勢 | 一次quat積分 + 正規化 | 1 | §4 |
| 弾道検証(無衝突) | RK4 | 4 | 解析解比較の基準値生成 |
| ばね質点・XPBD | XPBD 内蔵(位置ベース) | – | [10-mechanics/06](../10-mechanics/06-soft-body-particles.md) |
| 分子動力学(気体デモ) | velocity Verlet | 2 | シンプレクティック、長時間エネルギー保存 |
| ランジュバン(ブラウン) | Euler–Maruyama / BAOAB | 1/2 | 確率微分方程式用 [15-statistical/03](../15-statistical/03-diffusion-brownian.md) |
| 熱伝導(硬い拡散項) | 陰的 Euler(PCG) | 1 | 無条件安定 [12-thermal/02](../12-thermal/02-heat-transfer.md) |
| 流体移流 | semi-Lagrangian | 1 | 無条件安定 [11-fluid/02](../11-fluid/02-eulerian-grid.md) |
| 回路(硬い系) | 後退 Euler / 台形則 | 1/2 | [13-electromagnetism/02](../13-electromagnetism/02-circuits.md) |
| 電磁場中の荷電粒子・点質量 | Boris pusher | 2 | 磁場回転を厳密ノルム保存(E2 の担当)[13-electromagnetism/05](../13-electromagnetism/05-em-mechanics-coupling.md) |
| FDTD | leapfrog(Yee) | 2 | [13-electromagnetism/03](../13-electromagnetism/03-maxwell-fdtd.md) |
| シュレディンガー | split-step Fourier | 2 | ノルム厳密保存 [14-quantum/02](../14-quantum/02-schrodinger-solver.md) |

方針: **積分器は用途ごとに最適なものを選び、共通トレイトで無理に統一しない**。
統一するのは「固定 $\Delta t$」「決定論」「安定条件の実行時検査」の 3 点のみ。
剛体用のみ差し替えトレイト(§5)を用意する(検証遊びとして積分器比較をデモ化するため)。

## 2. 主要積分器の定義と性質

状態 $(\mathbf{x}, \mathbf{v})$、加速度 $\mathbf{a}(\mathbf{x}, \mathbf{v})$ とする。

### 2.1 explicit Euler(参考・比較用)

$\mathbf{v}_{n+1} = \mathbf{v}_n + \mathbf{a}_n \Delta t$、$\mathbf{x}_{n+1} = \mathbf{x}_n + \mathbf{v}_n \Delta t$。
一次精度。**振動系でエネルギーが単調増加**(不安定)。教材デモとしてのみ実装する
(ユーザーが「なぜダメか」を見られるようにする)。

### 2.2 semi-implicit (symplectic) Euler — 剛体の既定

$$\mathbf{v}_{n+1} = \mathbf{v}_n + \mathbf{a}_n \Delta t, \qquad \mathbf{x}_{n+1} = \mathbf{x}_n + \mathbf{v}_{n+1} \Delta t$$

一次精度だがシンプレクティック: 調和振動子で位相誤差はあってもエネルギーは有界に留まる。
安定条件(ばね定数 $k$、質量 $m$): $\Delta t < 2/\omega$, $\omega=\sqrt{k/m}$。
接触インパルス(速度を直接書き換える)との相性が良く、Box2D/Bullet と同じ選択。

### 2.3 velocity Verlet — 分子動力学の既定

$$\mathbf{x}_{n+1} = \mathbf{x}_n + \mathbf{v}_n\Delta t + \tfrac{1}{2}\mathbf{a}_n\Delta t^2,\qquad
\mathbf{v}_{n+1} = \mathbf{v}_n + \tfrac{1}{2}(\mathbf{a}_n + \mathbf{a}_{n+1})\Delta t$$

二次精度・シンプレクティック・時間反転対称。$\mathbf{a}$ が $\mathbf{x}$ のみに依存する保存系
(分子間ポテンシャル)で長時間のエネルギードリフトがない。マクスウェル分布への緩和デモの土台。

### 2.4 RK4 — 高精度基準

古典的 4 段 Runge-Kutta。四次精度、非シンプレクティック。
衝突のない滑らかな系(弾道・軌道)の**基準解生成**と収束次数テストに使う。
接触・拘束と混ぜない(段の途中で速度が不連続になると次数が崩れるため)。

### 2.5 陰的 Euler(拡散・硬い系)

$y_{n+1} = y_n + f(t_{n+1}, y_{n+1})\Delta t$。線形問題では $(I - \Delta t A)y_{n+1} = y_n$ を解く。
無条件安定(A安定)。拡散方程式では PCG([02-fields.md](02-fields.md) §5)で解く。
数値散逸があるため、精度が要る検証モードでは Crank-Nicolson(台形則、二次)に切り替え可能にする。

### 2.6 Boris pusher — 電磁場中の荷電粒子の既定

ローレンツ力 $\mathbf{F} = q(\mathbf{E} + \mathbf{v}\times\mathbf{B})$ を受ける荷電粒子・帯電点質量の
標準積分器(プラズマ PIC 法の標準手法)。電場キックと磁場回転を分離する:

$$\mathbf{v}^- = \mathbf{v}_n + \frac{q\mathbf{E}}{m}\frac{\Delta t}{2}$$
$$\mathbf{t} = \frac{q\mathbf{B}}{m}\frac{\Delta t}{2},\quad \mathbf{s} = \frac{2\mathbf{t}}{1+|\mathbf{t}|^2},\quad
\mathbf{v}' = \mathbf{v}^- + \mathbf{v}^-\times\mathbf{t},\quad \mathbf{v}^+ = \mathbf{v}^- + \mathbf{v}'\times\mathbf{s}$$
$$\mathbf{v}_{n+1} = \mathbf{v}^+ + \frac{q\mathbf{E}}{m}\frac{\Delta t}{2},\qquad
\mathbf{x}_{n+1} = \mathbf{x}_n + \mathbf{v}_{n+1}\Delta t$$

- 磁場回転部($\mathbf{v}^-\to\mathbf{v}^+$)は**厳密な回転**であり $|\mathbf{v}|$ をビットレベルの
  丸めを除き保存する — 「磁場は仕事をしない」を離散レベルで再現し、E2 サイクロトロンの
  「速さ一定 abs 1e-9」を構造的に満たす(semi-implicit Euler は磁場回転で速さが系統的に
  増大するため不適)。
- 二次精度・長時間安定(位相誤差はあるがエネルギードリフトしない)。
- **適用範囲**: 電磁場中の荷電粒子・帯電点質量([13-electromagnetism/01](../13-electromagnetism/01-electrostatics-magnetostatics.md) §4・
  [05](../13-electromagnetism/05-em-mechanics-coupling.md) §2.1)。剛体接触とは混ぜない
  (接触インパルスを持つ帯電剛体はローレンツ力を force generator として semi-implicit Euler 側で扱う)。

## 3. 安定性の実行時検査

各ソルバは自分の安定条件から `max_stable_dt()` を計算して Orchestrator に申告する
([00-foundation/04-architecture.md](../00-foundation/04-architecture.md) §1.3)。代表的な条件:

| 系 | 条件 | 意味 |
|---|---|---|
| 陽的な振動(ばね) | $\Delta t < 2/\omega_{max}$ | 最硬のばねが上限を決める |
| 陽的拡散 | $\Delta t < h^2/(6\alpha)$ | 3D 陽解法。これが厳しいので陰解法を既定にする |
| 移流(CFL) | $\Delta t < h / |\mathbf{u}|_{max}$ | semi-Lagrangian は免除(ただし精度のため $CFL \lesssim 5$ 推奨) |
| FDTD | $\Delta t < h/(c\sqrt{3})$ | Courant 条件 (3D) |
| SPH | $\Delta t < 0.25\, h_{sph}/c_s$ | 人工音速 $c_s$ 基準(Monaghan) |

## 4. 回転の積分

角速度 $\boldsymbol{\omega}$ による姿勢の更新は一次:
$$q_{n+1} = \mathrm{normalize}\!\left(q_n + \frac{\Delta t}{2}\,\tilde{\omega}\otimes q_n\right),\quad \tilde\omega=(\omega_x,\omega_y,\omega_z,0)$$

自由回転体のオイラー方程式 $\mathbf{I}\dot{\boldsymbol\omega} + \boldsymbol\omega\times(\mathbf{I}\boldsymbol\omega) = \boldsymbol\tau$ の
ジャイロ項は、既定では**陽的に評価**する(小さい $\Delta t$ では十分)。
テニスラケット定理(中間軸の不安定回転)のデモを正しく出すため、検証モードでは
ジャイロ項の陰的解法(Catto の一次陰的ジャイロ積分: ボディ座標で 3×3 ニュートン 1 回)を選択可能にする。

## 5. 剛体積分器トレイト(差し替え可能)

```rust
/// 剛体力学用。速度積分と位置積分が分離されているのは
/// 「速度積分 → 接触ソルバ(速度修正) → 位置積分」の順で呼ぶため。
pub trait RigidIntegrator {
    fn integrate_velocities(&self, bodies: &mut RigidBodySet, dt: f64);
    fn integrate_positions(&self, bodies: &mut RigidBodySet, dt: f64);
}
pub struct SemiImplicitEuler;   // 既定
pub struct ExplicitEuler;       // 教材 (エネルギー増加を見せる)
```

RK4 は分離不能なのでこのトレイトに載せず、無衝突専用の `BallisticIntegrator` として別に提供する。

Boris pusher(§2.6)も剛体接触と混ぜないため `RigidIntegrator` には載せず、
荷電粒子・帯電点質量専用の独立型として提供する:

```rust
/// 電磁場中の荷電粒子・点質量専用 (§2.6)。剛体接触とは併用しない。
pub struct BorisPusher;
impl BorisPusher {
    /// E, B は粒子位置で評価済みの場。速度回転部は厳密ノルム保存。
    pub fn step(&self, x: &mut Vec3, v: &mut Vec3, q_over_m: f64, e: Vec3, b: Vec3, dt: f64);
}
```

## 6. 検証

- 調和振動子($\ddot x = -\omega^2 x$)解析解との比較: explicit Euler は発散、semi-implicit は
  エネルギー有界、Verlet は二次収束 — の 3 点を同一テストで確認(教材デモの数値的裏付け)。
- 収束次数: $\Delta t$ を半減させ誤差比が $2^p$($p$=公称次数)に載ることを各積分器で確認。
- ケプラー軌道(逆二乗力)1000 周: Verlet のエネルギードリフト $<10^{-6}$ 相対、RK4 の軌道誤差測定。
