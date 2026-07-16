# 天体 01. 万有引力と N 体問題 — Barnes-Hut・シンプレクティック積分

crate: `sim-astro`。太陽系スケール($10^6$–$10^{12}$ m)の重力多体系。統一 9 節フォーマット。

## 1. 担う現実の現象

惑星の公転、月の運動、人工衛星の軌道、彗星、潮汐(月・太陽)、ラグランジュ点。
遊び方の例: 太陽系儀を回して惑星の会合周期を見る、衛星を投入して安定軌道の速度を探す、
三体問題のカオス、地球-月-太陽の潮汐。

## 2. 支配方程式

万有引力: 質点 $i$ に働く力は全他質点からの重ね合わせ

$$\mathbf{F}_i = -G m_i \sum_{j \ne i} m_j \frac{\mathbf{r}_i - \mathbf{r}_j}{|\mathbf{r}_i - \mathbf{r}_j|^3}, \qquad G = 6.674\times10^{-11}\ \mathrm{N\,m^2/kg^2}$$

日常の一様重力 $g = 9.80665$ m/s² は、地球質量 $M_\oplus$・半径 $R_\oplus$ での
$g = GM_\oplus/R_\oplus^2$ の局所近似([10-mechanics/01](../10-mechanics/01-rigid-body.md) の重力生成器は
本ドメインの一様場極限)。ソフトニング $\varepsilon$(近接特異点の緩和): $|\mathbf{r}|^3 \to (|\mathbf{r}|^2 + \varepsilon^2)^{3/2}$
を衝突しない多体系で使用(実天体では半径接触を剛体/再突入に委ねるため既定 $\varepsilon=0$)。

## 3. 状態表現・Rust 型定義

```rust
pub struct GravBody {
    pub position: Vec3,        // ワールド絶対座標 [m] (原点 = 太陽 or 系重心)
    pub velocity: Vec3,
    pub mass: f64,             // [kg]。GM を直接持つ選択肢も (§9)
    pub radius: f64,           // 接触・潮汐・再突入判定用
    pub kind: AstroKind,       // Star / Planet / Moon / Spacecraft / Asteroid
}
pub struct NBodySystem {
    pub bodies: Vec<GravBody>, // SoA 実体は position:Vec<Vec3> ... ([00-foundation/04] §3)
    pub softening: f64,
    pub tree: BarnesHutTree,   // 毎ステップ再構築
    pub integrator: SymplecticKind,
}
```

- 座標系: f64 絶対座標。分解能は Neptune 距離($4.5\times10^{12}$ m)で ~1 mm
  ([00-foundation/02](../00-foundation/02-scale-ladder.md) §座標階梯)。表面スケール物理を回す天体では
  **浮動原点フレーム**に切替(同文書)。
- 大質量差(太陽 vs 探査機)に備え、質量は f64 で $10^{30}$〜$10^3$ kg を扱う(桁は問題なし)。

## 4. 数値解法

### 4.1 Barnes-Hut(力の計算、$O(N\log N)$)

八分木で空間を再帰分割し、遠方のノードは重心 1 点にまとめる。開き角基準
$s/d < \theta$($s$: ノード幅、$d$: 距離、$\theta \approx 0.5$)を満たせば近似。

- **決定論**: ツリー構築のセル分割順・走査順を空間インデックス昇順に固定。
  力の総和は分割固定の逐次結合([00-foundation/06](../00-foundation/06-performance-strategy.md) §2.2)。
- 少数体(< 数百: 太陽系の主要天体)は総当たり $O(N^2)$ で十分・より正確 — 体数で自動選択。

### 4.2 シンプレクティック積分(長時間安定)

軌道は何百万周も回るためエネルギー・角運動量のドリフトが致命的 → シンプレクティック法必須:

- **leapfrog(kick-drift-kick)**: 2 次、可変体系の汎用。
  $\mathbf{v}_{1/2} = \mathbf{v}_0 + \frac{\Delta t}{2}\mathbf{a}_0$、
  $\mathbf{x}_1 = \mathbf{x}_0 + \Delta t\,\mathbf{v}_{1/2}$、$\mathbf{v}_1 = \mathbf{v}_{1/2} + \frac{\Delta t}{2}\mathbf{a}_1$。
- **WHFast(Wisdom-Holman)**: ケプラー運動(主星まわり)+ 摂動に分割。惑星系で leapfrog より
  桁違いに高精度・大刻み可。太陽系デモの既定([02-orbital-mechanics.md](02-orbital-mechanics.md) と共用)。
- 時間刻み: 天体は独立時間軸(軌道周期基準、例 1 日〜1000 s)。剛体(1/120 s)とは
  co-step せず、**再突入・バーン等の近接イベントで決定論的に微細刻みへレジーム切替**
  ([00-foundation/02](../00-foundation/02-scale-ladder.md) §時間分離)。

## 5. 適用スケールと限界

- 対象: 太陽系〜惑星間 ~10¹² m。恒星間・銀河スケールは対象外。
- 相対論効果は既定オフ。近日点移動・GPS 精度が要るシーンでオプトイン
  ([03-relativistic-corrections.md](03-relativistic-corrections.md))。
- 剛体多体の衝突([10-mechanics](../10-mechanics/))とは別レイヤ: 天体は点質量。表面の物理は
  浮動原点で剛体/流体ドメインに委譲。
- 潮汐変形・自転の歳差は剛体拡張(潮汐力は §6、自転は剛体の慣性で)。

## 6. 他ドメインとの結合

| 相手 | 内容 |
|---|---|
| 力学 | 一様重力の一般化(局所場)、潮汐力 $\mathbf{F}_{tidal} \approx \frac{2GMm r}{d^3}$、天体表面の剛体(浮動原点) |
| 流体 | 大気(高度依存密度)による軌道減衰、再突入([02](02-orbital-mechanics.md)) |
| 熱 | 太陽輻射フラックス($S = L_\odot/(4\pi d^2)$)、再突入空力加熱、天体の平衡温度 |
| レンダリング | 天体位置・スケール([17-rendering](../17-rendering/)) |

結合の全体像は [20-integration/01](../20-integration/01-coupling-matrix.md)。

## 7. 検証

- ケプラー第3法則: $T^2 \propto a^3$、太陽系 8 惑星で相対誤差 < 0.1%。
- 二体保存: エネルギー・角運動量の長時間ドリフト(10⁶ 周)< 10⁻⁶ 相対(シンプレクティック)。
- 円軌道速度: $v = \sqrt{GM/r}$、± 0.1%。
- 地球-月系の潮汐周期(半日周潮の駆動)の定性。
- ラグランジュ点 L4/L5 のトロヤ群の安定滞在。
- 決定論: 三体カオスで同一初期条件→同一軌道(丸め感度の最強テスト)。

## 8. 実装フェーズ対応

Phase B の依存順で最後(math → … → 統計 → **天体**)。Barnes-Hut + WHFast + 太陽系デモ。
相対論・再突入は本ドメイン内の後続。

## 9. パラメータ表・擬似コード

天体定数(出典: JPL, IAU 2015 公称値):

| 天体 | GM [m³/s²] | 質量 [kg] | 平均距離 [m] |
|---|---|---|---|
| 太陽 | 1.327×10²⁰ | 1.989×10³⁰ | — |
| 地球 | 3.986×10¹⁴ | 5.972×10²⁴ | 1.496×10¹¹ (1 AU) |
| 月 | 4.903×10¹² | 7.342×10²² | 3.844×10⁸ (対地球) |
| 木星 | 1.267×10¹⁷ | 1.898×10²⁷ | 7.785×10¹¹ |

| パラメータ | 既定値 | 根拠 |
|---|---|---|
| Barnes-Hut $\theta$ | 0.5 | 精度と速度の標準妥協 |
| 総当たり切替 | N < 256 | 少数体は $O(N^2)$ が高精度・十分速い |
| 天体刻み(内惑星) | ~0.1 日 | WHFast の精度実績 |

```text
nbody_step(dt):                          # 天体独立時間軸
  if N < 256: a = direct_sum_forces()    # O(N²)
  else:       tree.rebuild(); a = tree.forces(θ)   # O(N log N), 走査順固定
  symplectic_integrate(x, v, a, dt)      # WHFast or leapfrog
  detect_close_events()                  # 再突入・接近 → レジーム切替
```

## 10. 性能プロファイル

- ホットスポット: 力計算(全ペア or ツリー走査)。
- 目標アルゴリズムとオーダー: Barnes-Hut $O(N\log N)$(少数体は直和)。大規模は FMM $O(N)$(将来)。
- SoA レイアウト: position/velocity/mass を別配列。ツリーノードは配列プール。
- 並列化単位: 力計算を粒子(または枝)で rayon 分割。リダクション順固定。
- SIMD 対象カーネル: 直和の内側ループ(距離・逆三乗)を simd128 バッチ化。
- GPU 適性: 中〜高(直和は GPU 向き。ツリー走査は分岐が多く中)。大規模粒子系で有効化。
- ベンチ: 太陽系(N≈30)と N=10⁴ ランダム系で criterion。
