# 流体 05. 空力・水力(集中定数) — 抗力・揚力・終端速度・風

crate: `sim-fluid`。流れを解像せずに剛体へ働く流体力を与える**集中定数モデル**。
Phase 1 の主力であり、解像ソルバ(格子/SPH)導入後も「小さい・速い物体」で使い続ける。

## 1. 担う現実の現象

落下物の空気抵抗と終端速度、紙と石の落ち方の違い、向かい風、投げたボールの減速・カーブ、
翼の揚力、パラシュート。遊び方の例: 空気抵抗ON/OFFでの弾道比較、雨滴の終端速度、
回転するボールの曲がり(マグヌス効果)。

## 2. 支配方程式

### 2.1 抗力方程式

高レイノルズ数($Re > \sim10^3$)の標準形:

$$\mathbf{F}_d = -\tfrac{1}{2}\,\rho_f\, C_d\, A\, |\mathbf{v}_{rel}|\,\mathbf{v}_{rel}, \qquad \mathbf{v}_{rel} = \mathbf{v} - \mathbf{u}_{wind}$$

$A$: 進行方向投影面積、$C_d$: 抗力係数(§9)。低 $Re$($<1$、霧・微粒子)では
ストークス抵抗 $\mathbf{F}_d = -6\pi\mu r\,\mathbf{v}_{rel}$ に切り替える(遷移域は
Schiller-Naumann 補正 $C_d = \frac{24}{Re}(1 + 0.15 Re^{0.687})$ で連続接続、$Re < 800$ で有効)。

**終端速度**(検証の要): $mg = \frac{1}{2}\rho C_d A v_t^2$ より
$v_t = \sqrt{2mg/(\rho C_d A)}$。例: 雨滴 $d=2$ mm → $v_t \approx 6.5$ m/s(実測 ≈ 6.5 m/s、Gunn & Kinzer 1949)。

### 2.2 揚力とマグヌス効果

$$\mathbf{F}_L = \tfrac{1}{2}\rho C_L A\, |\mathbf{v}_{rel}|^2\, \hat{\mathbf{L}}$$

- 翼: $C_L(\alpha) \approx 2\pi\alpha$(薄翼理論、失速前 $|\alpha| < 12°$)、失速はクランプ+線形減衰で近似。
  $\hat{\mathbf{L}}$ は $\mathbf{v}_{rel}$ と翼スパンに直交。Phase 4(乗り物: 飛行機)で使用。
- 回転球(マグヌス): $\mathbf{F}_M = \frac{1}{2}\rho C_M A |\mathbf{v}_{rel}|^2 \,
  (\hat{\boldsymbol\omega}\times\hat{\mathbf{v}}_{rel})$、$C_M \approx 0.2\,S$
  (スピン比 $S = \omega r/|\mathbf{v}_{rel}|$、$S<1$ の経験式)。カーブボールのデモ。

### 2.3 回転抗力

$\boldsymbol\tau_d = -c_{\omega}\,\rho\, r^5\, \boldsymbol\omega|\boldsymbol\omega|$(高 $Re$)。
球の解析値は低 $Re$ で $-8\pi\mu r^3\boldsymbol\omega$。

## 3. 状態表現

```rust
pub enum DragModel {
    None,
    /// 球近似: A = πr², Cd(Re) 自動 (Schiller-Naumann → 0.47)
    Sphere { radius: f64 },
    /// 直方体: 3軸の投影面積から姿勢依存の A, Cd を補間
    Box3 { half_extents: Vec3, cd: f64 },
    /// パネル: 三角形群で法線流を積分 (布・翼・パラシュート)
    Panels { areas: Vec<f64>, normals: Vec<Vec3>, cd: f64, cl_slope: f64 },
}

pub struct Atmosphere {
    pub density: f64,            // 1.225 (15°C, 海面) — 温度・高度依存は式で
    pub wind: WindField,         // Uniform(Vec3) / Turbulent{...} (§4.2)
}
```

## 4. 数値解法

- 力の適用は force generator として速度積分前(mechanics パイプライン)。
- **陰的減衰の注意**: 抗力は $\Delta t$ 内で速度を反転させ得る(軽い紙)。
  安定化: ステップ内で $|\Delta \mathbf{v}_{drag}| \le |\mathbf{v}_{rel}|$ にクランプ
  (解析的には指数減衰 $v e^{-kt}$ に相当する上限)。終端速度テストで精度確認。

### 4.2 風の場(Phase 3)

- Uniform: 定数ベクトル。
- Gust/Turbulent: 時間相関ノイズ(Ornstein-Uhlenbeck 過程
  $du = -\theta u\,dt + \sigma dW$、[15-statistical/03](../15-statistical/03-diffusion-brownian.md) と同じ数学)
  を 3 成分独立に。決定論的 PRNG のストリームを使用。
- 格子流体が有効な領域では格子速度場をサンプル(集中定数はフォールバック)。

## 5. 適用スケールと限界

- $C_d$ 一定近似は $10^3 < Re < 2\times10^5$(drag crisis 以前)。臨界域(ゴルフボールのディンプル効果)は
  $C_d(Re)$ テーブルで近似可能だが Phase 5。
- 物体まわりの流れ場は存在しない: 後流・乱流・物体間の空力干渉(スリップストリーム)は
  格子ソルバ結合でのみ表現される。集中定数モデルの精度は ±10〜30% (形状・姿勢依存)と明記。
- 圧縮性($Ma > 0.3$、音速近く)は対象外。

## 6. 他ドメインとの結合

- 剛体へ force/torque。静的水域([04](04-free-surface-buoyancy.md))では $\rho_f$ を水に切替。
- 格子流体と排他([02](02-eulerian-grid.md) §6)。
- 熱: 対流熱伝達係数は同じ相対速度から推定([12-thermal/02](../12-thermal/02-heat-transfer.md) §4 の
  強制対流相関式と $\mathbf{v}_{rel}$ を共有)。

## 7. 検証

- 終端速度: 球($d$=1cm, 鋼)で $v_t$ 解析値 ± 1%。雨滴 2mm → 6.5 m/s ± 5%(Gunn-Kinzer 実測)。
- 弾道: 空気抵抗つき斜方投射を RK4 基準解と比較(semi-implicit の一次収束確認)。
- ストークス域: 微小球の沈降速度 $v = \frac{2r^2(\rho_p-\rho_f)g}{9\mu}$ ± 2%。
- マグヌス: バックスピン球の落下遅れ(定性)+ 経験式との比較(±20%)。

## 8. 実装フェーズ対応

Phase 1: Sphere/Box3 抗力 + 一様風 + 終端速度デモ。Phase 3: 風の乱流場、Panels。
Phase 4: 揚力(翼・マグヌス)、車両空力。

## 9. パラメータ表 — 抗力係数(出典: Hoerner, *Fluid-Dynamic Drag*; White, *Fluid Mechanics*)

| 形状 | $C_d$ |
|---|---|
| 球(亜臨界) | 0.47 |
| 半球(凹面が流れに正対、パラシュート) | 1.4 |
| 立方体(面正対) | 1.05 |
| 立方体(角正対) | 0.8 |
| 円柱(横流) | 1.2 |
| 流線形(翼型胴) | 0.04 |
| 人間(立位) | 1.0–1.3 |
| 乗用車 | 0.25–0.35 |

| 環境パラメータ | 値 |
|---|---|
| 空気密度(15°C 海面, ISA) | 1.225 kg/m³ |
| 温度補正 | $\rho = p_0 M/(RT)$(理想気体) |

## 10. 性能プロファイル

- ホットスポット: 抗力・揚力・マグヌス力の評価(剛体ごと)。
- 目標アルゴリズムとオーダー: 集中定数は $O(N)$。パネル法は $O(\text{パネル数})$。
- SoA レイアウト: 剛体配列 + DragModel 別配列。
- 並列化単位: 剛体/パネルを rayon 分割。
- SIMD 対象カーネル: 相対速度・抗力式のバッチ、パネル法線和。
- GPU 適性: 低〜中(布パネルが多い場合中)。
- ベンチ: 終端速度(F1)・多パネル布の風シーン。
