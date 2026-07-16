# 力学 06. ソフトボディ・粒子系 — ばね質点・XPBD・布・ロープ

crate: `sim-mechanics`(粒子コンテナは [01-math/02-fields.md](../01-math/02-fields.md) §6 を共用)。

## 1. 担う現実の現象

旗のはためき、ロープ・チェーンの垂れ、ゼリーの震え、クッションの変形、(将来)生物の軟組織。
遊び方の例: ロープの垂れがカテナリー曲線 $y = a\cosh(x/a)$ に載ることを確認、
布を風に当てる(流体結合)、ゼリーの固有振動。

## 2. 支配方程式

### 2.1 連続体としての正当化

弾性体の運動方程式(Cauchy): $\rho\ddot{\mathbf{u}} = \nabla\cdot\boldsymbol\sigma + \mathbf{f}$、
構成則(線形弾性): $\boldsymbol\sigma = \mathbf{C}:\boldsymbol\varepsilon$。
本エンジンは FEM ではなく**離散要素(質点+距離/曲げ拘束)**で近似する。
理由: (1) 剛体ソルバと同じ拘束機構に載る、(2) リアルタイム安定性、(3) 布・ロープ(1D/2D 構造)には
離散表現が自然。体積弾性(3D ゼリー)は四面体の体積拘束で近似する。
FEM 相当の精度が要る解析(応力分布)は非目標と明記する(§5)。

### 2.2 XPBD(Extended Position-Based Dynamics)を採用

拘束 $C_j(\mathbf{x})=0$ とコンプライアンス $\alpha_j = 1/k_j$(剛性の逆数)に対し、
各サブステップで位置を直接射影する:

$$\Delta\lambda_j = \frac{-C_j - \tilde\alpha_j \lambda_j}{\nabla C_j\, M^{-1} \nabla C_j^T + \tilde\alpha_j},
\qquad \tilde\alpha_j = \frac{\alpha_j}{\Delta t^2}, \qquad
\Delta\mathbf{x}_i = m_i^{-1}\nabla_{x_i} C_j\, \Delta\lambda_j$$

(Macklin et al., *XPBD*, 2016)。従来 PBD と違い**剛性が反復回数・$\Delta t$ に依存しない**
(コンプライアンスが物理単位を持つ)ため、「ヤング率からばね定数を決める」ことに意味がある。

主な拘束型:

- **距離拘束**: $C = |\mathbf{x}_i - \mathbf{x}_j| - L_0$(ロープ・布の構造ばね)
- **曲げ拘束**: 隣接 3 質点の角度、または布では隣接三角形の二面角
- **体積拘束**: 四面体体積 $C = 6(V - V_0)$(ゼリー)、閉曲面の全体積(風船)

### 2.3 物性からのコンプライアンス決定

断面積 $A$、自然長 $L_0$、ヤング率 $E$ のロープ要素: ばね定数 $k = EA/L_0$、$\alpha = 1/k$。
布は単位幅あたりの伸び剛性から同様に。これにより MaterialDb の実測物性(§9)と接続する。

## 3. 状態表現・Rust 型定義

```rust
pub struct SoftBody {
    pub particles: ParticleSet,            // position, velocity, mass (SoA)
    pub prev_position: Vec<Vec3>,          // XPBD 用
    pub inv_mass: Vec<f64>,                // 0 = 固定点 (ピン留め)
    pub constraints: Vec<SoftConstraint>,
    pub material: MaterialId,
}
pub enum SoftConstraint {
    Distance { i: u32, j: u32, rest: f64, compliance: f64, lambda: f64 },
    Bend     { i: u32, j: u32, k: u32, l: u32, rest_angle: f64, compliance: f64, lambda: f64 },
    Volume   { tet: [u32; 4], rest_vol: f64, compliance: f64, lambda: f64 },
}
```

生成ヘルパ: `rope(from, to, segments, material, radius)`、`cloth(corner, u, v, nu, nv, material)`、
`jelly_box(...)`(四面体分割)。

## 4. 数値解法

XPBD 標準ループ(サブステップ推奨: 反復より分割が精度に効く — Macklin et al. 2019):

```text
soft_step(dt):                     # dt = world_dt / n_sub (n_sub = 4 既定)
  for each particle: v += g*dt;  x_prev = x;  x += v*dt
  for iter in 0..n_iter:           # n_iter = 2 既定
    for each constraint (固定順): XPBD射影 (式は §2.2)
  剛体・地形との衝突: 粒子を表面外へ射影 (摩擦: 接線変位を μ 比例で戻す)
  for each particle: v = (x − x_prev)/dt
  速度減衰: v *= exp(−c_damp*dt)   # 数値・材料減衰 (小さく)
```

- 剛体との双方向結合: 粒子→剛体は接触点に等価インパルス
  $\mathbf{j} = m_i \Delta\mathbf{x}_i^{coll}/\Delta t$ を適用(片方向近似から開始、Phase 3 で双方向)。
- 自己衝突(布): 空間ハッシュで粒子間最小距離拘束(Phase 3 オプション、コスト高)。

## 5. 適用スケールと限界

- 離散化解像度以下の変形・皺は表現できない。応力の定量解析は非目標
  (カテナリー・固有振動数など**大域量**の検証に留める)。
- 破断は距離拘束の $\lambda$ 閾値で表現可能(Phase 5)。塑性は rest 長の更新で近似可能(同)。
- 高ヤング率(鋼線)は XPBD でも実質剛体 — 剛体+ジョイントで表す方が適切、と使い分けを文書化。

## 6. 他ドメインとの結合

- 流体: 布・ロープへの風力(パネル法: 三角形ごとに $\mathbf{F} = \frac{1}{2}\rho C_d A (\mathbf{u}_{rel}\cdot\hat{n})^2 \hat{n}$、
  [11-fluid/05](../11-fluid/05-aero-hydrodynamics.md))。
- 熱: 材料の温度依存剛性(将来)。摩擦発熱は剛体接触と同じ経路。

## 7. 検証

- ロープの垂れ: 静止形状がカテナリーと一致(端点間 1 m、20 分割で最大偏差 < 2%)。
- 伸び: 錘 $W$ を吊るしたロープの伸び $= WL_0/(EA) \pm 5\%$(XPBD の剛性正当性)。
- 布のピン留め落下: エネルギー単調減少、静止形状の対称性。
- 体積拘束: 風船の内外圧差と体積の関係(理想気体結合、Phase 4 デモ)。
- 収束: サブステップ倍増で誤差減少(XPBD は一次)。

## 8. 実装フェーズ対応

Phase 3: 距離・曲げ拘束、ロープ・布、剛体片方向結合。Phase 4: 体積拘束・風力結合・双方向。
Phase 5: 自己衝突・破断・塑性。

## 9. パラメータ表

| 材料 | ヤング率 E | 密度 | 出典 |
|---|---|---|---|
| ナイロンロープ | 2–4 GPa | 1150 kg/m³ | CRC Handbook |
| 綿布 | ~5 GPa(繊維) | 面密度 0.15 kg/m² | 繊維工学便覧(代表値) |
| 天然ゴム | 0.01–0.1 GPa | 920 kg/m³ | CRC Handbook |
| ゼラチンゲル(デモ用) | 10⁴–10⁵ Pa | 1050 kg/m³ | 食品物性の代表値 |

| パラメータ | 既定値 | 根拠 |
|---|---|---|
| サブステップ $n_{sub}$ | 4 | Macklin 2019(small steps)推奨方向 |
| 反復 $n_{iter}$ | 2 | 同上 |
| 減衰 $c_{damp}$ | 0.1 s⁻¹ | 目視安定の最小値(材料減衰は別途) |

## 10. 性能プロファイル

- ホットスポット: XPBD の拘束射影ループ、粒子↔剛体衝突。
- 目標アルゴリズムとオーダー: XPBD $O(\text{拘束数} \times \text{反復})$。サブステップ優先(反復より分割)。
- SoA レイアウト: position/prev_position/velocity/inv_mass 別配列。拘束は種別配列。
- 並列化単位: グラフ彩色で独立拘束を並列射影(色順固定で決定論)。
- SIMD 対象カーネル: 距離拘束の射影、位置更新。
- GPU 適性: 高(布・大規模粒子は GPU 向き。CPU 参照実装は残す)。
- ベンチ: 布(64×64)・ロープ・ゼリーで計測。
