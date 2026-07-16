# 流体 03. SPH — 粒子法による自由表面流

crate: `sim-fluid`。水しぶき・注水・波など自由表面が主役の流れを担当(格子法との使い分けは
[02-eulerian-grid.md](02-eulerian-grid.md) §5)。

## 1. 担う現実の現象

コップに注ぐ水、水槽に落ちる物体の飛沫、ダム崩壊(dam break)、波。
遊び方の例: 水位と放出量の関係(トリチェリの定理)、物体落下の王冠スプラッシュ。

## 2. 支配方程式(SPH 離散化の導出)

SPH (Smoothed Particle Hydrodynamics) は場をカーネル $W$ による粒子和で近似する:

$$f(\mathbf{x}) \approx \sum_j \frac{m_j}{\rho_j} f_j\, W(\mathbf{x}-\mathbf{x}_j, h)$$

### 2.1 カーネル

**cubic spline カーネル**(Monaghan 1992)を採用。$q = |\mathbf{r}|/h$:

$$W(q) = \frac{8}{\pi h^3}\begin{cases} 1 - 6q^2 + 6q^3 & 0\le q\le \tfrac12 \\ 2(1-q)^3 & \tfrac12 < q \le 1 \\ 0 & q > 1\end{cases}$$

(正規化 3D)。サポート半径 $h$、有効近傍 ~30–40 粒子。
勾配が必要な圧力項には spiky 勾配(Müller 2003)をオプション比較(クラスタリング耐性)。

### 2.2 密度と圧力

$$\rho_i = \sum_j m_j W_{ij}, \qquad p_i = \max\!\Big(k_{eos}\Big[\big(\tfrac{\rho_i}{\rho_0}\big)^{7} - 1\Big],\ 0\Big)$$

状態方程式は Tait 方程式(**弱圧縮 SPH, WCSPH**)。$k_{eos} = \rho_0 c_s^2/7$、人工音速
$c_s \ge 10\, u_{max}$ とすると密度変動 < 1% に収まる(Monaghan 1994)。
負圧クランプ($\max(\cdot,0)$)で表面の粒子凝集を防ぐ。
※ 真の水の音速(1481 m/s)を使うと $\Delta t$ が過小になるため、人工音速は**意図的な近似**
(密度変動 1% を許して時間刻みを稼ぐ)であることを明記。

### 2.3 運動方程式(対称形)

運動量保存を厳密にする対称化圧力項(Monaghan):

$$\frac{d\mathbf{v}_i}{dt} = -\sum_j m_j\left(\frac{p_i}{\rho_i^2} + \frac{p_j}{\rho_j^2}\right)\nabla_i W_{ij}
+ \sum_j m_j\,\Pi_{ij}\,\nabla_i W_{ij} + \mathbf{g}$$

$\Pi_{ij}$ は人工粘性(衝撃・振動抑制、Monaghan 標準形。実流体粘性は必要なら層流粘性項を追加)。
作用反作用が粒子対で厳密に対称 → 全運動量が機械精度で保存する。

## 3. 状態表現

```rust
pub struct SphFluid {
    pub particles: ParticleSet,       // position, velocity, mass
    pub density: Vec<f64>,
    pub pressure: Vec<f64>,
    pub h: f64,                       // カーネル半径
    pub rho0: f64, pub c_s: f64,      // 静止密度・人工音速
    pub hash: SpatialHash,            // [01-math/02] §6.1
}
```

粒子質量は初期格子配置から $m = \rho_0 \Delta x^3$($\Delta x = h/2$ 配置間隔)。

## 4. 数値解法

```text
sph_step(dt):                          # dt = min(0.25 h/c_s, 0.25 √(h/|g|)) で sub-step
  hash.rebuild(positions)
  for i: ρ_i = Σ_j m_j W_ij            # 近傍和 (27セル走査、インデックス昇順=決定論)
  for i: p_i = tait(ρ_i)
  for i: a_i = pressure_term + viscosity_term + g
  境界処理 (§4.1)
  velocity Verlet で積分                # [01-math/03] §2.3
```

### 4.1 境界条件

- **静的境界(壁・床)**: 境界粒子法(壁面に固定粒子を 2–3 層敷き、密度和に参加させる)。
  漏れがなく実装が単純。Akinci et al. 2012 の境界体積補正を採用。
- **動的剛体との結合**: 剛体表面をサンプリングした境界粒子が剛体と一緒に動く。
  流体→剛体: 境界粒子が受ける圧力・粘性力を剛体の力・トルクに集計。
  剛体→流体: 境界粒子の速度が壁面速度として効く。双方向で運動量が保存(Akinci 方式)。

### 4.2 高度化(Phase 5 オプション)

- PCISPH / DFSPH(非圧縮を反復で強制): タイムステップを 5–10 倍広げられる。
  WCSPH で性能不足になったら移行。インターフェースは `SphSolver` トレイトで差し替え。

## 5. 適用スケールと限界

- 粒子解像度以下の飛沫・薄膜は表現不可($h = 2$ cm なら 2 cm 未満の構造は消える)。
- 弱圧縮近似: 密度変動 ~1% を許容(音波は正しく伝わらない)。
- 表面張力は Phase 4 の追加モデル([04](04-free-surface-buoyancy.md) §4)— mm 以下の滴の物理は精度外。
- 粒子数の実用上限(WASM シングルスレッド): 2×10⁴ @60fps。それ以上はスロー再生か並列化後。

## 6. 他ドメインとの結合

- 剛体: §4.1 双方向。浮力は SPH では圧力項から**自然に創発する**(アルキメデスを仮定しない)—
  これ自体を検証デモにする([04](04-free-surface-buoyancy.md) §7 と相互検証)。
- 熱: 粒子に温度を持たせ、SPH 熱伝導項(Cleary-Monaghan 形式)で拡散(Phase 5)。
  温度依存粘性(蜂蜜)も同フェーズ。

## 7. 検証

- 静水圧平衡: 水柱静止後の密度プロファイル(深さ方向に ~0.5% 勾配)と圧力 $p=\rho g h \pm 3\%$。
- ダム崩壊: 先端位置 $x(t)$ を実験データ(Martin & Moyce 1952)と比較(±10%)。
- 全運動量: 外力なしで機械精度保存。エネルギー: 単調減少(人工粘性の散逸)。
- 浮力の創発: 密度 0.5ρ₀ の箱が半分沈んで平衡(アルキメデス ±5%)。
- 決定論: 近傍リスト順固定の下で同一シード→同一結果。

## 8. 実装フェーズ対応

Phase 4: WCSPH + 境界粒子 + 剛体双方向 + ダム崩壊/注水/浮力デモ。Phase 5: DFSPH・熱・表面張力。

## 9. パラメータ表

| パラメータ | 既定値 | 根拠 |
|---|---|---|
| カーネル半径 h | 0.04 m(2 cm 粒子間隔) | 2×10⁴ 粒子で浴槽スケール |
| 人工音速 $c_s$ | $10\,u_{max}$(シーン推定) | 密度変動 <1%(Monaghan 1994) |
| 人工粘性 α | 0.08 | Monaghan 推奨域 0.01–0.1 |
| CFL 係数 | 0.25 | SPH 慣例(Monaghan) |
| 境界粒子層数 | 3 | カーネル半径を覆う最小 |
