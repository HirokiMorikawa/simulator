# 天体 02. 軌道力学 — 軌道要素・摂動・大気圏再突入

crate: `sim-astro`。N体重力([01-gravitation-nbody.md](01-gravitation-nbody.md))の上に、
軌道の記述・操作・大気との相互作用(再突入)を載せる。

## 1. 担う現実の現象

人工衛星の軌道、ホーマン遷移、スイングバイ、静止軌道、軌道の減衰と再突入(流れ星・カプセル帰還)、
ロケット打ち上げ。遊び方の例: 衛星を投入して周期・近地点を測る、スイングバイで加速する、
再突入角度を変えて生存/焼失を試す(熱結合)。

## 2. 支配方程式

### 2.1 二体問題と軌道要素

主星まわりの運動は円錐曲線。6 つの軌道要素 $(a, e, i, \Omega, \omega, \nu)$
(半長径・離心率・軌道傾斜・昇交点黄経・近点引数・真近点角)で状態を記述。
状態ベクトル $(\mathbf{r}, \mathbf{v})$ ⇔ 軌道要素の相互変換を提供。
ビス・ビバ方程式(速度): $v^2 = GM\left(\frac{2}{r} - \frac{1}{a}\right)$。
周期: $T = 2\pi\sqrt{a^3/(GM)}$。

### 2.2 摂動

理想二体からのずれ。本エンジンは摂動を**別の力として N体積分に直接加える**
(軌道要素の解析摂動論は使わず数値積分、実装が統一される):

- 他天体の重力(N体で自然に入る)
- 大気抗力(低軌道、§4)
- 扁平率 $J_2$(地球の赤道膨らみ → 昇交点の歳差)。$J_2 = 1.083\times10^{-3}$。
- 太陽輻射圧(軽い衛星、[12-thermal](../12-thermal/) の放射と共有)

### 2.3 大気圏再突入

高度依存の大気密度(指数モデル $\rho(h) = \rho_0 e^{-h/H}$、スケールハイト $H \approx 8.5$ km、
または層別 US Standard Atmosphere)中で:

- 抗力([11-fluid/05](../11-fluid/05-aero-hydrodynamics.md)): $\mathbf{F}_d = -\frac12\rho v^2 C_d A\,\hat{v}$、
  超音速では $C_d$ が変化(マッハ依存、簡易テーブル)。
- 空力加熱: よどみ点熱流束 $\dot q \approx C\sqrt{\rho/R_n}\,v^3$(Sutton-Graves 関係、$R_n$: 先端半径)。
  熱ドメイン([12-thermal/02](../12-thermal/02-heat-transfer.md))へ熱源として渡し、
  アブレーション(相変化 [12-thermal/03](../12-thermal/03-phase-change.md))で焼失を判定。

## 3. 状態表現

```rust
pub struct Orbit {                    // 表示・解析用 (状態ベクトルから導出)
    pub a: f64, pub e: f64, pub i: f64,
    pub raan: f64, pub arg_pe: f64, pub true_anomaly: f64,
    pub epoch: f64,
}
pub struct Spacecraft {
    pub body: GravBodyId,             // [01] の点質量
    pub dry_mass: f64, pub propellant: f64,
    pub thrust: f64, pub isp: f64,    // 推力・比推力 (推進)
    pub drag: DragModel, pub nose_radius: f64, pub heat_shield: MaterialId,
}
pub struct Atmosphere1D {             // 天体の大気 (高度プロファイル)
    pub surface_density: f64, pub scale_height: f64,
    pub layers: Option<Vec<AtmoLayer>>,   // US Standard 等
}
```

## 4. 数値解法

- 軌道伝播は [01](01-gravitation-nbody.md) のシンプレクティック積分 + 摂動力。
- **推進(ロケット方程式)**: 推力中は $\dot m = -F/(I_{sp} g_0)$、$\Delta v = I_{sp} g_0 \ln(m_0/m_1)$。
  バーン中は質量変化が速いのでレジーム切替で微細刻み([01](01-gravitation-nbody.md) §4.2)。
- **再突入**: 大気圏に入ると空気抗力・加熱が急増 → 自動で微細刻み(高度 or 動圧トリガ)。
  抗力・熱流束を毎ステップ評価し力学・熱に渡す。焼失(熱シールド質量がアブレーションで枯渇)を
  イベント化。
- パッチドコニック(近似遷移)はプランニング用の補助(実軌道は数値積分が正)。

## 5. 適用スケールと限界

- 大気は 1 次元(高度依存)モデル。3 次元の大気循環・風は当面対象外(格子流体は地表ローカルのみ)。
- アブレーションは簡易(潜熱ベースの質量除去)。詳細な熱防護材の化学は非対象。
- 相対論補正は [03](03-relativistic-corrections.md)。
- 恒星の内部・進化、輻射流体力学は対象外。

## 6. 他ドメインとの結合

- 天体重力([01](01-gravitation-nbody.md)): 軌道の土台。
- 流体([11-fluid/05](../11-fluid/05-aero-hydrodynamics.md)): 大気抗力(高度依存 $\rho$)。
- 熱([12-thermal](../12-thermal/)): 空力加熱・アブレーション・太陽輻射平衡温度。
- エンティティ([20-integration/03](../20-integration/03-entity-layer.md)): 宇宙機 = 剛体 + 推進 + 軌道。
- 相対論([03](03-relativistic-corrections.md)): GPS 衛星の時刻。

## 7. 検証

- 円/楕円軌道の周期・近点距離が軌道要素の解析値と一致(< 0.1%)。
- ホーマン遷移の $\Delta v$ が解析値 ± 0.5%。
- $J_2$ 歳差率(昇交点)の解析式との一致(± 2%)。
- 再突入: 弾道係数を変えたときの最大加熱率・減速 g の傾向が既知(アポロ/はやぶさ級のオーダー)。
- スイングバイ: 双曲線通過前後の速度ベクトル変化がパッチドコニック解析と一致(± 1%)。
- 状態ベクトル ⇔ 軌道要素の往復変換が機械精度。

## 8. 実装フェーズ対応

Phase B 天体ドメイン([01](01-gravitation-nbody.md) の後): 軌道要素・摂動($J_2$)・推進 →
大気モデル・再突入(流体・熱結合)。デモは [21-verification/03](../21-verification/03-demo-scenarios.md)。

## 9. パラメータ表

| パラメータ | 値 | 出典 |
|---|---|---|
| 地球大気スケールハイト $H$ | 8.5 km | US Standard Atmosphere |
| 海面大気密度 | 1.225 kg/m³ | ISA |
| 地球 $J_2$ | 1.08263×10⁻³ | IERS |
| Sutton-Graves 定数 $C$ | 1.83×10⁻⁴(SI, 地球空気) | Sutton & Graves 1971 |
| 静止軌道半径 | 4.216×10⁷ m | 導出値 |
| 太陽定数(1 AU) | 1361 W/m² | 観測値 |

## 10. 性能プロファイル

- ホットスポット: 摂動力評価(N体と共有)、再突入時の大気・加熱評価。
- 目標アルゴリズムとオーダー: [01](01-gravitation-nbody.md) の Barnes-Hut に準拠。軌道要素変換は $O(N)$。
- SoA レイアウト: [01](01-gravitation-nbody.md) と共有(GravBody 配列)。宇宙機属性は別配列。
- 並列化単位: 少数(衛星群)なので並列効果は限定的。多数衛星(メガコンステレーション)で粒子並列。
- SIMD 対象カーネル: 大気密度・抗力のバッチ評価。
- GPU 適性: 低〜中(体数が少ない用途が主)。
- ベンチ: 地球周回衛星 + 再突入シーン。
