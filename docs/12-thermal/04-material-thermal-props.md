# 熱 04. 材料物性データベース — MaterialDb の設計と物性表

crate: `sim-core`(全ドメイン参照のため。熱の列が最多なので本章で定義する)。

## 1. 役割

材料 = 全ドメイン横断の物性の束。**単一の DB に一元化**し、値の重複定義・不整合を防ぐ
([00-foundation/02-scale-ladder.md](../00-foundation/02-scale-ladder.md) §4)。
量子・統計ドメインは「これらの値がなぜその値か」の説明を担う
([14-quantum/03](../14-quantum/03-effective-models.md), [15-statistical/01](../15-statistical/01-micro-macro-bridge.md))。

## 2. Rust 型定義

```rust
pub struct Material {
    pub name: &'static str,
    // 力学
    pub density: f64,              // ρ [kg/m³]
    pub friction: f64,             // μ (単一材料値。ペア表が優先 [10-mechanics/04])
    pub restitution: f64,          // e (対剛壁の代表値)
    pub youngs_modulus: Option<f64>,
    // 熱
    pub specific_heat: f64,        // c_p [J/(kg·K)]
    pub conductivity: f64,         // k [W/(m·K)]
    pub emissivity: f64,           // ε
    pub melting: Option<PhaseChangeProps>,   // T_m, L_f, T_boil, L_v
    // 電磁気
    pub resistivity: Option<f64>,  // ρ_e [Ω·m] (None = 絶縁体扱い)
    pub relative_permittivity: f64,// ε_r
    pub refractive_index: Option<f64>, // n (透明材のみ)
    // メタ
    pub source: &'static str,      // 出典
    pub uncertainty: f64,          // 代表値の相対不確かさ (UI表示用)
}

pub struct MaterialDb {
    materials: Vec<Material>,                      // MaterialId = インデックス
    friction_pairs: BTreeMap<(MaterialId, MaterialId), PairOverride>,
}
```

- 温度依存物性は Phase 5 で `fn conductivity_at(&self, T: f64) -> f64`(区分線形テーブル)に拡張。
  シグネチャを最初からこの形にしておく(DB 自体は不変、[00-foundation/04](../00-foundation/04-architecture.md) §2)。
- シーン JSON からのカスタム材料追加を許す(遊びの中心機能: 「密度だけ変えた氷」等)。

## 3. 標準物性表

出典: CRC Handbook of Chemistry and Physics (103rd ed.)、Incropera *Fundamentals of Heat and Mass Transfer* 付録、
反発係数は代表実測(対コンクリート/鋼、不確かさ大)。20 °C・1 atm の代表値。

| 材料 | ρ [kg/m³] | c_p [J/kgK] | k [W/mK] | ε | μ | e | E [GPa] |
|---|---|---|---|---|---|---|---|
| 鋼(炭素鋼) | 7850 | 490 | 50 | 0.6(酸化) | 0.6 | 0.6 | 200 |
| アルミニウム | 2700 | 900 | 237 | 0.1 | 0.5 | 0.5 | 69 |
| 銅 | 8960 | 385 | 401 | 0.05 | 0.5 | 0.5 | 117 |
| ガラス | 2500 | 840 | 1.0 | 0.92 | 0.5 | 0.7 | 70 |
| コンクリート | 2400 | 880 | 1.4 | 0.9 | 0.7 | 0.2 | 30 |
| 木材(松) | 500 | 1700 | 0.12 | 0.9 | 0.45 | 0.4 | 9 |
| ゴム(天然) | 920 | 1900 | 0.16 | 0.94 | 0.9 | 0.8 | 0.05 |
| 氷(0°C) | 916.7 | 2100 | 2.2 | 0.96 | 0.05 | 0.1 | 9 |
| 水 | 998.2 | 4182 | 0.60 | 0.96 | — | — | — |
| 空気 | 1.204 | 1005 | 0.026 | — | — | — | — |
| 発泡スチロール | 30 | 1300 | 0.033 | 0.9 | 0.4 | 0.6 | 0.005 |
| 人体(平均) | 1010 | 3500 | 0.5 | 0.98 | — | — | — |
| PTFE(テフロン) | 2200 | 1000 | 0.25 | 0.9 | 0.04 | 0.4 | 0.5 |

電磁気列(該当材料のみ、[13-electromagnetism](../13-electromagnetism/) 用):

| 材料 | ρ_e [Ω·m] | ε_r | n |
|---|---|---|---|
| 銅 | 1.68×10⁻⁸ | — | — |
| 鋼 | 1.4×10⁻⁷ | — | — |
| ニクロム | 1.1×10⁻⁶ | — | — |
| ガラス | ~10¹² | 5–10 | 1.52 |
| 水(純) | ~2×10⁵ | 80.1 | 1.333 |
| 空気 | — | 1.0006 | 1.000293 |
| ダイヤモンド | — | 5.7 | 2.417 |

## 4. 値の信頼性の扱い

- `uncertainty`: 反発係数 0.3(±30%)、摩擦 0.3、熱物性 0.05、密度 0.01 を既定。
  UI は測定値と比較するとき誤差帯として表示する(「実測とのズレが物理の間違いか
  物性のばらつきか」を区別できるようにする — 検証遊びの誠実さ)。
- すべての行に `source` を持たせ、UI から出典を確認できる。
- 反発係数は本来ペア量+速度依存。単一値は「対硬い床、低速(< 5 m/s)」の代表と明記。

## 5. テスト

- 表の値の単位・桁の妥当性チェック(音速 $\sqrt{E/\rho}$ が既知値と桁一致、など派生量での相互検証):
  鋼の音速 ≈ 5000 m/s、熱拡散率 $\alpha = k/(\rho c_p)$ が文献値と一致(±5%)。
- DB ロードの決定論(順序固定)、ペア表の対称性 $(A,B)=(B,A)$。
