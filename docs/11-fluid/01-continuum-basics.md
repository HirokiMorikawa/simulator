# 流体 01. 連続体基礎 — Navier-Stokes の導出と無次元数

crate: `sim-fluid`。流体ドメインの理論的土台。ソルバ実装は [02-eulerian-grid.md](02-eulerian-grid.md)(格子)と
[03-sph.md](03-sph.md)(粒子)。

## 1. 担う現実の現象

水の流れ・波・渦、空気の流れ・風・煙、注ぐ・こぼれる・混ざる。
遊び方の例: 蛇口から注ぐ水、水槽の波、煙の渦、管の中の流れの速度分布。

## 2. 支配方程式(導出)

### 2.1 連続の式(質量保存)

検査体積への質量流入出の収支から:

$$\frac{\partial\rho}{\partial t} + \nabla\cdot(\rho\mathbf{u}) = 0$$

非圧縮($\rho$ 一定、§5 で適用条件)なら $\nabla\cdot\mathbf{u} = 0$。

### 2.2 運動量保存(Cauchy → Navier-Stokes)

流体粒子(物質微分 $D/Dt = \partial_t + \mathbf{u}\cdot\nabla$)にニュートンの第 2 法則:

$$\rho\frac{D\mathbf{u}}{Dt} = \nabla\cdot\boldsymbol\sigma + \rho\mathbf{g}$$

ニュートン流体の構成則(応力 = 等方圧力 + 速度勾配に線形な粘性応力):

$$\boldsymbol\sigma = -p\mathbf{1} + \mu\left(\nabla\mathbf{u} + \nabla\mathbf{u}^T\right) + \lambda_v(\nabla\cdot\mathbf{u})\mathbf{1}$$

を代入し、非圧縮を仮定すると **Navier-Stokes 方程式**:

$$\boxed{\;\frac{\partial\mathbf{u}}{\partial t} + (\mathbf{u}\cdot\nabla)\mathbf{u}
= -\frac{1}{\rho}\nabla p + \nu\nabla^2\mathbf{u} + \mathbf{g}\;},\qquad \nabla\cdot\mathbf{u}=0$$

($\nu = \mu/\rho$: 動粘性係数)。各項の意味 — 移流(自分自身で運ばれる・非線形性と渦の源)、
圧力勾配(非圧縮を守る束縛力)、粘性(運動量の拡散)、外力。

### 2.3 圧力の役割

非圧縮流では $p$ は状態方程式でなく**拘束力**: $\nabla\cdot\mathbf{u}=0$ を保つよう瞬時に決まる。
速度の仮更新後に Poisson 方程式 $\nabla^2 p = \frac{\rho}{\Delta t}\nabla\cdot\mathbf{u}^*$ を解いて
速度を射影する(Chorin の射影法)— これが格子ソルバの構造を決める([02](02-eulerian-grid.md))。

## 3. 状態表現

- 格子法: MAC 格子の速度場 + セル中心圧力・マーカー([01-math/02-fields.md](../01-math/02-fields.md))。
- 粒子法: 粒子の位置・速度・密度([03-sph.md](03-sph.md))。
- 静的媒質(計算しない空気・水域): `Medium { density, viscosity, temperature }` の領域定義のみ
  ([05-aero-hydrodynamics.md](05-aero-hydrodynamics.md) が剛体への力に使う)。

## 4. 無次元数 — 解法選択の物差し

| 無次元数 | 定義 | 意味 | 本エンジンでの使い方 |
|---|---|---|---|
| レイノルズ数 | $Re = UL/\nu$ | 慣性/粘性 | $Re$ 高 → 乱流(格子解像度で捉えられない渦は数値散逸 or 渦強化で扱う)。$Re < 1$ → ストークス域(ブラウン粒子の抵抗則) |
| マッハ数 | $Ma = U/c_s$ | 圧縮性 | $Ma < 0.3$ で非圧縮近似(密度変化 < 5%)。日常の水・風はすべて該当 |
| フルード数 | $Fr = U/\sqrt{gL}$ | 慣性/重力 | 自由表面の波のスケーリング検証 |
| クヌーセン数 | $Kn = \lambda/L$ | 連続体の成立 | $Kn > 0.01$ は連続体不成立 → 統計ドメイン([00-foundation/02](../00-foundation/02-scale-ladder.md) §3) |
| CFL 数 | $u\Delta t/h$ | 数値安定 | [01-math/03](../01-math/03-integrators.md) §3 |

代表値: 歩く人のまわりの空気 $Re \approx 10^5$、蛇口の水流 $Re \approx 10^3$–$10^4$、
コップの中の対流 $Re \approx 10^2$。日常現象はほぼ乱流〜遷移域であることを UI の「近似表示」で正直に示す。

## 5. 適用スケールと限界

- **非圧縮のみ**(Phase 内)。音波・衝撃波・爆発は対象外(圧縮性ソルバは将来検討)。
  音の伝播が要るデモは波動方程式の専用ソルバで別途扱う(ロードマップ検討事項)。
- **乱流の直接解像はしない**: 実用解像度(64³)で解像できるのは大きな渦のみ。
  サブグリッドの渦は数値粘性に吸収される。これは LES(Large Eddy Simulation)の
  暗黙版に相当し、定量精度は大域量(流量・力の時間平均)に限る、と明記する。
  渦の見た目の維持には vorticity confinement(渦度強化、[02](02-eulerian-grid.md) §4.5)をオプション提供。
- 表面張力が支配的な微小スケール(ウェーバー数 $We = \rho U^2 L/\sigma_s < 1$、mm 以下の滴)は
  Phase 4 の表面張力モデルの精度限界を明記([04](04-free-surface-buoyancy.md))。

## 6. 他ドメインとの結合

- 剛体⇔流体: [05-aero-hydrodynamics.md](05-aero-hydrodynamics.md)(集中定数)と
  [02](02-eulerian-grid.md) §6(解像結合)の 2 レベル。
- 熱: 温度による密度差 → 浮力(Boussinesq 近似: $\mathbf{f}_b = -\beta(T-T_0)\mathbf{g}$ を運動量に加える)。
  煙・対流デモの駆動源([12-thermal/02](../12-thermal/02-heat-transfer.md))。
- 統計: 粘性係数・拡散係数の分子論的由来([15-statistical/01](../15-statistical/01-micro-macro-bridge.md))。

## 7. 検証

理論そのものの検証は解法文書に委ねるが、共通ベンチマーク:

- ポアズイユ流: 管内(平行平板間)定常流の放物型速度分布 $u(y) = \frac{G}{2\mu}y(H-y)$、誤差 < 2%。
- 減衰渦(Taylor-Green): 解析減衰率 $e^{-2\nu k^2 t}$ との比較(粘性項の検証)。
- 静水圧: $p = \rho g h$(圧力ソルバの検証)。

## 8. 実装フェーズ対応

Phase 1: 静的媒質 + 集中定数の力([05](05-aero-hydrodynamics.md))。Phase 2: 格子スカラー移流(煙)。
Phase 3: 完全な格子非圧縮ソルバ + Boussinesq。Phase 4: SPH 自由表面・表面張力。

## 9. パラメータ表

| 流体(20 °C, 1 atm) | 密度 ρ [kg/m³] | 粘性 μ [Pa·s] | 動粘性 ν [m²/s] | 出典 |
|---|---|---|---|---|
| 水 | 998.2 | 1.002×10⁻³ | 1.004×10⁻⁶ | CRC Handbook |
| 空気 | 1.204 | 1.825×10⁻⁵ | 1.516×10⁻⁵ | CRC Handbook |
| 海水(3.5%塩分) | 1025 | 1.08×10⁻³ | 1.05×10⁻⁶ | UNESCO 標準 |
| オリーブ油 | 911 | 8.4×10⁻² | 9.2×10⁻⁵ | CRC Handbook |
| 蜂蜜(代表) | 1420 | ~10 | ~7×10⁻³ | 食品物性代表値(温度依存大) |
| 水の表面張力 | — | $\sigma_s$ = 72.8×10⁻³ N/m | — | CRC Handbook |

## 10. 性能プロファイル

- ホットスポット: 本文書は理論。実計算は [02](02-eulerian-grid.md)/[03](03-sph.md) が担う。
- 目標アルゴリズムとオーダー: 各解法文書の §10 を参照。
- SoA レイアウト: 場は Grid3/MacGrid、粒子は ParticleSet([01-math/02](../01-math/02-fields.md))。
- 並列化単位: 解法文書に委譲。
- SIMD 対象カーネル: 解法文書に委譲。
- GPU 適性: 解法文書に委譲。
- ベンチ: ポアズイユ流・Taylor-Green(解法文書で計測)。
