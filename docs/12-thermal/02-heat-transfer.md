# 熱 02. 熱伝達 — 伝導・対流・放射の離散化

crate: `sim-thermal`。熱ノード間・ノード⇔環境・温度場の熱の移動則。

## 1. 担う現実の現象

コーヒーが冷める、金属スプーンの柄が熱くなる、焚き火の輻射熱、フライパンの加熱、
断熱材の効果。遊び方の例: 材質の違うスプーンの熱の伝わり比べ、冷却曲線の測定、
魔法瓶(伝導・対流・放射を各個撃破)の設計。

## 2. 支配方程式

### 2.1 伝導(フーリエの法則)

熱流束 $\mathbf{q} = -k\nabla T$ [W/m²]。エネルギー保存と合わせて熱伝導方程式:

$$\rho c_p \frac{\partial T}{\partial t} = \nabla\cdot(k\nabla T) + \dot{q}_v$$

### 2.2 対流(ニュートンの冷却則)

表面と流体間: $\dot{Q} = h A (T_{fluid} - T_{surf})$。$h$ [W/(m²K)] は流れに依存(§4.2)。

### 2.3 放射(シュテファン=ボルツマン)

$$\dot{Q}_{rad} = \varepsilon \sigma A (T_{env}^4 - T^4)$$

(灰色体・環境が大きい場合の形。$\sigma = 5.670\times10^{-8}$)。
面同士の放射交換(視係数)は Phase 5(焚き火→物体は点源近似で先行提供 §4.3)。

## 3. 状態表現

- **ノードネットワーク**(既定): ThermalNode + 熱コンダクタンスのグラフ。

```rust
pub struct ThermalLink {
    pub a: NodeId, pub b: NodeId,
    pub conductance: f64,        // G [W/K]:  Q̇ = G (T_a − T_b)
    pub kind: LinkKind,          // Contact / Bolted / Custom
}
```

- **温度場**(格子、流体・大物体内部用): `Grid3<f64>`([01-math/02](../01-math/02-fields.md))。

## 4. 数値解法

### 4.1 接触熱伝導のコンダクタンス

剛体接触から自動生成: 接触面積 $A_c$(マニフォールドの点数・形状から推定)、
2 材料の熱伝導率 $k_1, k_2$、実効ギャップ長 $L_{gap}$(表面粗さの実効値)で

$$G = \frac{A_c}{L_1/k_1 + R_c A_c + L_2/k_2}$$

$R_c$: 接触熱抵抗(粗さ・圧力依存、§9 に代表値。押し付け力が強いほど下がる —
法線インパルスから経験式 $R_c \propto p_c^{-0.95}$ で補正、Phase 5)。
接触の生成・消滅イベント([10-mechanics/02](../10-mechanics/02-collision-detection.md) §6)に同期して
リンクを張り替える。

### 4.2 対流係数 $h$ の相関式(集中定数)

| 状況 | 相関式(平均 Nusselt 数 $\overline{Nu} = hL/k_f$) | 出典 |
|---|---|---|
| 自然対流(垂直面) | $\overline{Nu} = 0.59\,Ra^{1/4}$($10^4<Ra<10^9$) | Churchill-Chu 簡略形 |
| 自然対流(球) | $\overline{Nu} = 2 + 0.43\,Ra^{1/4}$ | Yuge |
| 強制対流(球) | $\overline{Nu} = 2 + 0.6\,Re^{1/2}Pr^{1/3}$ | Ranz-Marshall |
| 強制対流(平板) | $\overline{Nu} = 0.664\,Re^{1/2}Pr^{1/3}$(層流) | Blasius 解 |

$Ra$: レイリー数、$Pr$: プラントル数(空気 0.71、水 7.0)。剛体の相対速度
([11-fluid/05](../11-fluid/05-aero-hydrodynamics.md) と共有)から $Re$ を計算。
目安値: 静止空気中 $h \approx 5{-}10$、風あり $10{-}100$、水中 $100{-}1000$ W/(m²K)。

### 4.3 ノードネットワークの積分

$$C_i \frac{dT_i}{dt} = \sum_j G_{ij}(T_j - T_i) + hA(T_{amb}-T_i) + \varepsilon\sigma A(T_{env}^4 - T_i^4) + \dot q_{src}$$

- 線形項(伝導・対流)は**陰的 Euler**: $(\mathbf{C}/\Delta t + \mathbf{L})\mathbf{T}^{n+1} = (\mathbf{C}/\Delta t)\mathbf{T}^n + \mathbf{b}$。
  $\mathbf{L}$ はグラフラプラシアン(SPD)→ PCG。ノード数は少ない(~10³)ので常に安価。
- 放射項($T^4$)は線形化 $h_{rad} = 4\varepsilon\sigma \bar{T}^3$ で陰行列に入れ、
  ステップごとに $\bar T$ を更新(Picard 1 回で十分、温度変化/step が小さいため)。
- 温度場(格子)は同じ陰的拡散を 7 点ステンシルで(流束形式・調和平均、
  [01-math/02](../01-math/02-fields.md) §4)。

### 4.4 散逸熱源の分配

力学からの摩擦・衝突熱 $\Delta Q$([10-mechanics/03](../10-mechanics/03-contact-solver.md) §6,
[04](../10-mechanics/04-friction.md) §6)は、両接触体に**熱浸透率**
$e_t = \sqrt{k\rho c_p}$ の比で分配する(半無限体の接触理論):
$Q_A/Q_B = e_{t,A}/e_{t,B}$。ジュール熱([13-electromagnetism/02](../13-electromagnetism/02-circuits.md))は
素子のノードへ全量。

## 5. 適用スケールと限界

- ノードモデルは $Bi < 0.1$([01](01-thermodynamics-laws.md) §5)。
- 相関式 $h$ の精度は ±20%(実験相関の常識的範囲)— UI で誤差帯を表示。
- 放射の視係数・多重反射は Phase 5。参加媒質(霧の放射吸収)は対象外。
- 超高速加熱(レーザー・非フーリエ効果)は対象外。

## 6. 他ドメインとの結合

[01-thermodynamics-laws.md](01-thermodynamics-laws.md) §6 の表に同じ。本文書は「熱の入る・出る」全経路の実装点。

## 7. 検証

- ニュートン冷却: $T(t) = T_{amb} + (T_0-T_{amb})e^{-t/\tau}$, $\tau = C/(hA)$ — 指数フィットで $\tau$ 誤差 < 1%(陰解法は無条件安定なので大 $\Delta t$ でも減衰率一次誤差、検証は $\Delta t$ 収束で)。
- 2 ノード伝導: 解析解 $\Delta T(t) = \Delta T_0 e^{-t(G/C_1+G/C_2)}$ と一致。
- 1D 棒の温度分布(格子): 定常線形分布、過渡はフーリエ級数解と比較(< 2%)。
- 放射平衡: 太陽定数モデル入力での平衡温度 $T = (q/(\varepsilon\sigma))^{1/4}$(球の昼夜平均で ± 2%)。
- 統合: 摩擦ブレーキデモ — 運動エネルギー減少 = 熱上昇(台帳 residual < 10⁻³)。

## 8. 実装フェーズ対応

Phase 1: ニュートン冷却 + 放射(線形化)+ 散逸熱の記帳。Phase 3: 接触伝導ネットワーク・格子温度場・
流体対流結合。Phase 5: 視係数放射・接触抵抗の圧力依存。

## 9. パラメータ表

| パラメータ | 値 | 出典 |
|---|---|---|
| 接触熱抵抗 $R_c$(金属-金属、乾燥) | 0.5–5 ×10⁻⁴ m²K/W | Incropera, *Fundamentals of Heat and Mass Transfer* |
| 放射率 ε: 酸化金属 / 磨いた金属 / 木・紙 / 水・氷 / 人肌 | 0.6–0.9 / 0.02–0.1 / 0.9 / 0.96 / 0.98 | Incropera 付録 |
| 空気 $k_f$ | 0.026 W/(m·K) (20°C) | CRC |
| 水 $k_f$ | 0.60 W/(m·K) | CRC |

材料の $k, c_p, \rho$ は [04-material-thermal-props.md](04-material-thermal-props.md) の一元表。
