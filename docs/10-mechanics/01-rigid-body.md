# 力学 01. 剛体 — 状態・慣性テンソル・力/トルク API

crate: `sim-mechanics`。統一 9 節フォーマット。

## 1. 担う現実の現象

机から落ちる本、投げたボール、転がる樽、積み木、ドミノ — 変形が無視できる日常物体の並進と回転。
遊び方の例: 高さと落下時間の関係を測る、こまの歳差運動を見る、テニスラケット定理(中間軸まわりの
回転の不安定性)を宙返りするスマホで確かめる。

## 2. 支配方程式(導出込み)

ニュートン=オイラー方程式。質点系の全運動量 $\mathbf{P}=\sum m_i \mathbf{v}_i$ に
ニュートンの第 2 法則を適用し、内力が対で打ち消し合う(第 3 法則)ことから:

$$\frac{d\mathbf{P}}{dt} = \mathbf{F}_{ext} \quad\Rightarrow\quad m\,\dot{\mathbf{v}}_{cm} = \mathbf{F}_{ext}$$

角運動量 $\mathbf{L} = \sum \mathbf{r}_i \times m_i\mathbf{v}_i$(重心まわり)について同様に:

$$\frac{d\mathbf{L}}{dt} = \boldsymbol{\tau}_{ext}, \qquad \mathbf{L} = \mathbf{I}\,\boldsymbol{\omega}$$

慣性テンソル $\mathbf{I}$ は剛体の質量分布から
$I_{jk} = \int \rho(\mathbf{r})\,(r^2\delta_{jk} - r_j r_k)\,dV$。
ワールド座標では姿勢に依存し $\mathbf{I}_w = R\,\mathbf{I}_{local}\,R^T$($R$ は回転行列)。
これを $\dot{\mathbf{L}} = \boldsymbol\tau$ に代入するとオイラー方程式:

$$\mathbf{I}_w\,\dot{\boldsymbol\omega} + \boldsymbol\omega \times (\mathbf{I}_w\,\boldsymbol\omega) = \boldsymbol\tau_{ext}$$

第 2 項(ジャイロ項)が歳差・章動・テニスラケット定理の源。

## 3. 状態表現・Rust 型定義

**速度ベース**(運動量でなく $\mathbf{v}, \boldsymbol\omega$ を状態に持つ)を採用。
接触ソルバ(sequential impulses)が速度を直接修正する方式のため。SoA:

```rust
pub struct RigidBodySet {
    // 状態 (毎ステップ更新)
    pub position: Vec<Vec3>,          // 重心の所属フレームローカル座標
    pub frame: Vec<FrameId>,          // 所属フレーム。単一フレームシーンでは全て ROOT
    pub rotation: Vec<Quat>,
    pub linear_velocity: Vec<Vec3>,
    pub angular_velocity: Vec<Vec3>,  // ワールド座標系
    // ステップ内アキュムレータ
    pub force_accum: Vec<Vec3>,
    pub torque_accum: Vec<Vec3>,
    // 定数 (生成時に確定)
    pub inv_mass: Vec<f64>,           // static/kinematic は 0
    pub inv_inertia_local: Vec<Mat3>, // 対角化済みローカル慣性の逆
    pub inv_inertia_world: Vec<Mat3>, // 毎ステップ R I⁻¹ Rᵀ で更新 (キャッシュ)
    pub body_type: Vec<BodyType>,     // Dynamic / Static / Kinematic
    pub shape: Vec<ShapeHandle>,
    pub material: Vec<MaterialId>,
    // 熱結合用 (12-thermal と共有する状態)
    pub temperature: Vec<f64>,        // 集中熱容量モデル [K]
}

pub enum BodyType {
    Dynamic,    // 全法則に従う
    Static,     // 不動 (地面・壁)。inv_mass = 0
    Kinematic,  // スクリプト駆動 (速度は外部指定、力を受けない)。エンティティ制御用
}
```

生成記述子(公開 API、[20-integration/04-world-api.md](../../docs/20-integration/04-world-api.md)):

```rust
pub struct RigidBodyDesc {
    pub body_type: BodyType,
    pub shape: Shape,
    pub material: MaterialId,          // 密度→質量、摩擦・反発・熱物性の参照元
    pub transform: Transform,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub mass_override: Option<f64>,    // None なら shape.volume() * material.density
    pub initial_temperature: f64,      // 既定 293.15 K
    pub drag: DragModel,               // None / Sphere{cd} / Box{cd} ([11-fluid/05] 参照)
}
```

## 4. 数値解法

- 速度積分・位置積分は semi-implicit Euler([01-math/03-integrators.md](../01-math/03-integrators.md) §2.2)。
- 姿勢は一次 quat 積分 + 正規化(同 §4)。ジャイロ項は既定で陽的、検証モードで陰的。
- 1 ステップの mechanics 内パイプライン:

```
apply_forces(重力・抗力・結合からの力)     // force_accum に加算
integrate_velocities(dt)                  // v += (F/m)dt, ω += I_w⁻¹(τ − ω×I_wω)dt
collision detection                        // [02-collision-detection.md]
contact & joint solve                      // [03][05]: 速度を直接修正
integrate_positions(dt)                   // x += v dt, q = integrate(q, ω, dt)
update inv_inertia_world; clear accum
```

- **スリープ(Phase 2)**: 速度が閾値($|\mathbf{v}|<0.01$ m/s かつ $|\boldsymbol\omega|<0.02$ rad/s が 0.5 s 継続)
  未満の接触島単位で積分を停止。起床は新規接触・力適用時。決定論に影響しないよう閾値判定も固定順で行う。

### 4.1 慣性テンソル(基本形状の解析式)

| 形状 | ローカル主慣性モーメント |
|---|---|
| 球(半径 $r$) | $I = \frac{2}{5}mr^2$(全軸) |
| 中空球殻 | $I = \frac{2}{3}mr^2$ |
| 直方体($2a\times2b\times2c$) | $I_x = \frac{m}{3}(b^2{+}c^2)$ 等 |
| 円柱(半径 $r$、高さ $h$、軸=y) | $I_y=\frac{1}{2}mr^2,\; I_{x,z}=\frac{m}{12}(3r^2{+}h^2)$ |
| カプセル | 円柱 + 半球 2 個の平行軸定理合成(実装時に導出をコメントで残す) |

複合形状は平行軸定理 $\mathbf{I} = \mathbf{I}_{cm} + m(d^2\mathbf{1} - \mathbf{d}\mathbf{d}^T)$ で合成し、
重心と主軸を再計算する(固有値分解は対称 3×3 なので Jacobi 法で十分)。

## 5. 適用スケールと限界

- 剛体近似の条件: 変形量 ≪ 物体サイズ。目安として弾性変形 $\delta \sim FL/(EA)$ が
  サイズの 0.1% を超える柔らかい物体(ゴムシート・布・生体組織)は
  [06-soft-body-particles.md](06-soft-body-particles.md) の対象。
- 微小スケール($<10^{-6}$ m)ではブラウン運動が無視できない → 統計ドメインとの結合
  ([15-statistical/03](../15-statistical/03-diffusion-brownian.md))。
- 速度は $v \ll c$、音速との比較でも $Ma < 0.3$ 想定(超音速の衝撃波は対象外)。
- 破壊・塑性変形は当面対象外(ロードマップ Phase 5+ の検討事項)。

## 6. 他ドメインとの結合

| 相手 | 方向 | 内容 |
|---|---|---|
| 流体 | 受 | 浮力・抗力・揚力(force_accum へ)。返: 障害物境界・排除体積 |
| 熱 | 送 | 摩擦・衝突の散逸エネルギー → 熱源。受: 温度(将来: 熱膨張・物性変化) |
| 電磁気 | 受 | ローレンツ力・モータートルク。送: 導体の運動 → 誘導起電力 |
| 統計 | 受 | 微小粒子へのランダム力(ブラウン) |

すべて force/torque accum と境界条件を介する([20-integration/01-coupling-matrix.md](../20-integration/01-coupling-matrix.md))。

## 7. 検証

- 自由落下: $y(t) = y_0 - \frac{1}{2}g t^2$、着地時刻相対誤差 < 0.5%($\Delta t = 1/120$)。
- 斜方投射(真空): 到達距離 $R = v_0^2\sin 2\theta/g$、相対誤差 < 0.5%。
- 回転: トルクフリーの対称こまの歳差角速度 $\dot\phi = L/I_1$ が解析値と一致(< 1%)。
  中間軸回転の不安定性(摂動の指数成長率)を確認。
- 保存則: 外力なしで $\mathbf{P}, \mathbf{L}$ が機械精度で保存(ジャイロ項の陽的積分では
  $\mathbf{L}$ にドリフトが出る — 許容ドリフト率を測定し文書化、陰的モードで消えることを確認)。

## 8. 実装フェーズ対応

- **Phase 1**: Dynamic/Static、球・箱・平面、重力、semi-implicit Euler、慣性(球・箱)。
- **Phase 2**: Kinematic、円柱・カプセル・複合形状、スリープ、陰的ジャイロ。
- Phase 3+: 温度状態の活用(熱結合)、エンティティ層からの制御。

## 9. パラメータ表・擬似コード

主要既定値(出典: 単位・定数は [00-foundation/03-units-conventions.md](../00-foundation/03-units-conventions.md)):

| パラメータ | 既定値 | 根拠 |
|---|---|---|
| $\Delta t_{world}$ | 1/120 s | 接触安定性と 60fps 描画の整数比 |
| スリープ速度閾値 | 0.01 m/s / 0.02 rad/s | Box2D 準拠の経験値(調整可) |
| 最大速度クランプ | 1000 m/s | 発散検知(音速の3倍、日常シーンの妥当上限) |

```text
integrate_velocities(dt):
  for each dynamic body i:
    v[i] += inv_mass[i] * force_accum[i] * dt
    Iw_inv = R(q[i]) * inv_inertia_local[i] * R(q[i])^T   # キャッシュ更新
    gyro   = ω[i] × (Iw(q[i]) * ω[i])                     # 陽的ジャイロ
    ω[i] += Iw_inv * (torque_accum[i] − gyro) * dt

integrate_positions(dt):
  for each dynamic/kinematic body i:
    x[i] += v[i] * dt
    q[i] = normalize(q[i] + 0.5*dt * quat(ω[i],0) ⊗ q[i])
```

## 10. 性能プロファイル

- ホットスポット: 速度・位置積分、ワールド逆慣性の更新($R I^{-1} R^T$)。
- 目標アルゴリズムとオーダー: 積分は $O(N)$。スリープ島管理で稼働体数を削減。
- SoA レイアウト: position/rotation/linear_velocity/angular_velocity を別配列。ホット(状態)/コールド(材質ID・形状)分離。
- 並列化単位: 体を rayon チャンク分割(積分は体間独立)。
- SIMD 対象カーネル: ベクトル/クォータニオン演算のバッチ、逆慣性の相似変換。
- GPU 適性: 低〜中(接触ソルバがボトルネックで積分単体は軽い)。
- ベンチ: 500 体スタック・散乱シーンで criterion(予算 3 ms)。
