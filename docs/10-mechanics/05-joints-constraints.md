# 力学 05. ジョイントと拘束 — 抽象・ヤコビアン導出・各種ジョイント

crate: `sim-mechanics`。人・生物(関節骨格)・乗り物(車軸・サスペンション)・機構(ドア・振り子)の土台。

## 1. 担う現実の現象

ドアの蝶番、振り子、チェーン、人形の関節、車輪と車軸、モーター駆動の回転。
遊び方の例: 二重振り子のカオス、チェーンの垂れ(カテナリー)、モーターでの倒立振子制御。

## 2. 支配方程式 — 拘束力学の一般形

拘束は状態の関数 $C(\mathbf{x}_A, q_A, \mathbf{x}_B, q_B) = 0$(等式)または $C \ge 0$(不等式)。
速度レベルでは連鎖律により

$$\dot{C} = J\,\mathbf{V} = 0, \qquad \mathbf{V} = (\mathbf{v}_A, \boldsymbol\omega_A, \mathbf{v}_B, \boldsymbol\omega_B)^T \in \mathbb{R}^{12}$$

$J \in \mathbb{R}^{m\times 12}$ が**ヤコビアン**。拘束力は仮想仕事の原理から $\mathbf{F}_c = J^T\boldsymbol\lambda$
(拘束は仕事をしない)。インパルス定式化での 1 拘束行の解:

$$\Delta\lambda = -\frac{J\mathbf{V} + b}{J\,M^{-1}J^T}, \qquad M^{-1} = \mathrm{diag}(m_A^{-1}\mathbf{1}, \mathbf{I}_A^{-1}, m_B^{-1}\mathbf{1}, \mathbf{I}_B^{-1})$$

$J M^{-1} J^T$ が有効質量の逆数(接触ソルバの $1/m_{eff}$ の一般形)。バイアス $b$ は
位置ドリフト補正(Baumgarte)・モーター目標速度・ソフト拘束項を運ぶ。
接触・摩擦・ジョイントはすべてこの形の行の集合であり、**同一の PGS ループで解ける**。

### 2.1 例: ボールジョイントのヤコビアン導出

拘束: 両体のアンカー点一致 $C = (\mathbf{x}_B + \mathbf{r}_B) - (\mathbf{x}_A + \mathbf{r}_A) = \mathbf{0} \in \mathbb{R}^3$。
時間微分($\dot{\mathbf{r}} = \boldsymbol\omega\times\mathbf{r}$ を使う):

$$\dot{C} = \mathbf{v}_B + \boldsymbol\omega_B\times\mathbf{r}_B - \mathbf{v}_A - \boldsymbol\omega_A\times\mathbf{r}_A
= \underbrace{[-\mathbf{1} \;\; [\mathbf{r}_A]_\times \;\; \mathbf{1} \;\; -[\mathbf{r}_B]_\times]}_{J}\,\mathbf{V}$$

($[\cdot]_\times$ は歪対称行列、[01-math/01](../01-math/01-linear-algebra.md) §4)。
3 行拘束なので有効質量は 3×3 行列 $K = J M^{-1} J^T$ となり、ブロックで解く(§4.2)。

## 3. 状態表現・Rust 型定義

```rust
/// 接触も含む全拘束の共通トレイト。solver は列を区別しない。
pub trait Constraint {
    fn bodies(&self) -> (BodyIdx, Option<BodyIdx>);   // None = 対ワールド
    /// 有効質量・バイアス・warm start 適用
    fn prepare(&mut self, bodies: &RigidBodySet, dt: f64);
    fn solve_velocity(&mut self, bodies: &mut RigidBodySet);
    fn solve_position(&mut self, bodies: &mut RigidBodySet) {}  // NGS用 (任意)
    /// 現在の拘束力 (観測・破断判定用)
    fn impulse(&self) -> ConstraintImpulse;
}

pub enum Joint {
    Ball(BallJoint),          // 3自由度回転 (肩・股関節)
    Hinge(HingeJoint),        // 1自由度回転 + 角度制限 + モーター (肘・膝・ドア・車軸)
    Slider(SliderJoint),      // 1自由度並進 (ピストン・サスペンション)
    Fixed(FixedJoint),        // 全固定 (溶接。複合体の分割構築用)
    Distance(DistanceJoint),  // 2点間距離 (ロープ端点・スプリング)
    Wheel(WheelJoint),        // Phase 4: サス+駆動+操舵の複合
}

pub struct JointCommon {
    pub anchor_a: Vec3, pub anchor_b: Vec3,   // 各ボディローカル
    pub breaking_impulse: Option<f64>,        // 超過で破断イベント
    pub soft: Option<SoftParams>,             // §4.3 ばね化 (周波数・減衰比)
}
```

## 4. 数値解法

### 4.1 統一 PGS ループ

接触と同じ solver に投入(処理順: ジョイント → 接触、各カテゴリ内は生成順で固定)。
反復数も共有($N_v = 10$)。長いチェーン(10 リンク超)は収束が遅い —
反復増、または島ごとの反復数指定で対応(Phase 3 で評価)。

### 4.2 ブロックソルバ

3 行拘束(ボール等)は 3×3 の $K$ を直接逆行列で解く(行ごとの PGS より収束・剛性が良い)。
$K$ は prepare で一度計算し、コレスキー分解をキャッシュ。

### 4.3 ソフト拘束(ばね化)

Baumgarte 係数を物理パラメータで指定する標準変換(Catto, *Soft Constraints*, GDC 2011):
固有振動数 $f$ [Hz] と減衰比 $\zeta$ から

$$k = m_{eff}(2\pi f)^2,\quad c = 2 m_{eff}\zeta(2\pi f),\quad
\gamma = \frac{1}{\Delta t(c + \Delta t k)},\quad \beta_{soft} = \frac{\Delta t k}{c + \Delta t k}$$

を計算し、バイアスと対角正則化($K + \gamma\mathbf{1}$)に入れる。
サスペンション・筋のような「硬いばね」を安定に扱う正攻法。陽的ばね(force generator)は
$\Delta t < 2/\omega$ の制限で硬くできないため、硬い接続はすべてソフト拘束で表す。

### 4.4 主要ジョイントの拘束行(要約)

| ジョイント | 行数 | 内容 |
|---|---|---|
| Ball | 3 | アンカー一致(§2.1) |
| Hinge | 5 | アンカー一致 3 + 軸直交 2($\hat{a}_A\cdot\hat{b}_B=0$ 型 2 本) |
| + limit | +1 | 角度 $\theta\in[\theta_{min},\theta_{max}]$、不等式(接触と同じクランプ $\lambda \ge 0$) |
| + motor | +1 | $\dot\theta = \omega_{target}$、$|\lambda| \le \tau_{max}\Delta t$ にクランプ |
| Slider | 5 | 軸直交並進 2 + 相対回転固定 3 |
| Fixed | 6 | 並進 3 + 回転 3 |
| Distance | 1 | $|\mathbf{p}_B-\mathbf{p}_A| = L$、$J = (\hat{\mathbf{d}}, \mathbf{r}_A\times\hat{\mathbf{d}}, \ldots)$ |

角度 limit・モーターの角度測定はヒンジ軸まわりの相対回転角(atan2 で連続化、±π 跨ぎを追跡)。

### 4.5 モーター(アクチュエータ)

- 速度モーター: 目標角速度 $\omega_{target}$、トルク上限 $\tau_{max}$。
  エンティティ制御(歩行・車輪駆動)の基本 API。
- 位置サーボ: 目標角 $\theta_{target}$ を PD 制御($\omega_{target} = k_p(\theta_{target}-\theta) - k_d\dot\theta$
  をクランプ)でモーター行へ渡す。制御ループはエンティティ層、物理はモーター行のみ
  (責務分離、[20-integration/03-entity-layer.md](../20-integration/03-entity-layer.md))。
- **モーター仕事の計上**: 各ステップの $\tau\cdot\dot\theta\,\Delta t$ をエネルギー台帳に記録
  (エネルギー検算で「注入源」を明示するため。[21-verification/02](../21-verification/02-conservation-laws.md))。

## 5. 適用スケールと限界

- 関節のガタ・弾性・バックラッシュは既定で捨象(ソフト拘束で近似可)。
- PGS の収束限界: 質量比 1:100 超の接続(重い車体と軽いアンテナ)は硬くなる。
  対策: 質量比の警告診断、ブロックソルバ、Phase 5 で直接法(小さな島の LDL^T)検討。

## 6. 他ドメインとの結合

- モーター ⇔ 電磁気: DC モーターモデル([13-electromagnetism/05](../13-electromagnetism/05-em-mechanics-coupling.md))が
  トルク上限・目標速度を電気側から供給し、逆起電力で電気側へ返す。
- 拘束力の観測: `impulse()` をエンティティ層(足裏荷重・積載監視)と破断判定が使う。

## 7. 検証

- 単振り子: 小振幅周期 $T = 2\pi\sqrt{L/g} \pm 1\%$、大振幅は楕円積分の解析値と比較。
- 二重振り子: エネルギー保存(散逸なし設定で相対ドリフト $< 10^{-3}$/1000 step)。
  カオス感度は決定論テスト(同一初期条件→同一軌道)の題材にする。
- ヒンジ limit: 制限角で停止し、超過トルクに $\lambda \ge 0$ で抗する。
- モーター: 無負荷で目標速度に到達、負荷時に $\tau_{max}$ で飽和。仕事の台帳が運動エネルギー変化と一致。
- 位置ドリフト: 10 リンクチェーンを 60 s 揺らしてアンカー誤差 < 5 mm(NGS 有効時 < 1 mm)。

## 8. 実装フェーズ対応

Phase 3: Ball / Hinge(limit・motor)/ Distance / Fixed + ソフト拘束 + ブロックソルバ(ラグドールに必要な一式)。
Phase 4: Slider / Wheel。Phase 5: 直接法の検討。

## 9. パラメータ表・擬似コード

| パラメータ | 既定値 | 根拠 |
|---|---|---|
| ジョイント Baumgarte $\beta$ | 0.2 | 接触と同じ |
| ソフト拘束の推奨範囲 | $f \le \frac{1}{4\Delta t}$ Hz | サンプリング定理的上限の 1/2(安定余裕) |
| モーター PD 既定 | $k_p=20\,\mathrm{s^{-1}}, k_d=2$ | 減衰比 ≈ 0.7 目安(質量依存、エンティティ側で調整) |

```text
solve_velocity(hinge):
  # 並進 3 行 (ブロック)
  v_err = (v_B + ω_B×r_B) − (v_A + ω_A×r_A)
  Δλ3 = K3_chol.solve(−(v_err + bias3))
  apply J^T Δλ3
  # 軸直交 2 行、limit 1 行 (λ≥0 クランプ)、motor 1 行 (|λ|≤τmax·dt クランプ)
  ... 各行スカラー solve (接触と同形)
```
