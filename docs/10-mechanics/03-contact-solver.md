# 力学 03. 接触ソルバ — Sequential Impulses 完全導出

crate: `sim-mechanics`。接触制約を速度レベルで解くインパルスベースソルバ。摩擦は
[04-friction.md](04-friction.md)、一般拘束は [05-joints-constraints.md](05-joints-constraints.md)。

## 1. 担う現実の現象

衝突の跳ね返り(反発)、積み重なった物体が互いを支える力(垂直抗力)、めり込みの解消。
遊び方の例: 反発係数の異なるボールのバウンド比較、4 段積みの箱が静止し続けること。

## 2. 支配方程式(導出)

### 2.1 接触制約

接触点における相対速度の法線成分(接近速度):

$$v_n = \hat{\mathbf{n}} \cdot \big[(\mathbf{v}_B + \boldsymbol\omega_B \times \mathbf{r}_B) - (\mathbf{v}_A + \boldsymbol\omega_A \times \mathbf{r}_A)\big]$$

($\mathbf{r}_{A,B}$ は各重心から接触点へのベクトル)。非貫入条件は $v_n \ge 0$(離れる方向は自由)。
接触力は押すのみ: 法線インパルス $j_n \ge 0$。相補性条件 $v_n \cdot j_n = 0$(離れつつ押さない)。

### 2.2 インパルスと速度変化

接触点に撃力 $\mathbf{j} = j_n \hat{\mathbf{n}}$ を加えたときの速度変化:

$$\Delta\mathbf{v}_A = -\frac{\mathbf{j}}{m_A},\quad \Delta\boldsymbol\omega_A = -\mathbf{I}_A^{-1}(\mathbf{r}_A\times\mathbf{j}),\quad
\Delta\mathbf{v}_B = +\frac{\mathbf{j}}{m_B},\quad \Delta\boldsymbol\omega_B = +\mathbf{I}_B^{-1}(\mathbf{r}_B\times\mathbf{j})$$

これを $v_n$ の式に代入すると、単位法線インパルスあたりの $v_n$ 変化(**有効質量の逆数**)が得られる:

$$\frac{1}{m_{eff}} = \frac{1}{m_A} + \frac{1}{m_B}
+ \hat{\mathbf{n}}\cdot\big[(\mathbf{I}_A^{-1}(\mathbf{r}_A\times\hat{\mathbf{n}}))\times\mathbf{r}_A\big]
+ \hat{\mathbf{n}}\cdot\big[(\mathbf{I}_B^{-1}(\mathbf{r}_B\times\hat{\mathbf{n}}))\times\mathbf{r}_B\big]$$

### 2.3 反発(ニュートンの衝突則)

目標: 衝突後の法線速度 $v_n^+ = -e\, v_n^-$($e$: 反発係数、$v_n^-$: 衝突前接近速度)。
1 接触点・2 体では、必要なインパルスが閉形式で出る:

$$j_n = -\frac{(1+e)\,v_n^-}{1/m_{eff}}$$

多点・多体では互いに影響し合うため反復解法(§4)が要る。これは混合線形相補性問題
(MLCP: $A\boldsymbol\lambda + \mathbf{b} \ge 0,\ \boldsymbol\lambda \ge 0,\ \boldsymbol\lambda^T(A\boldsymbol\lambda+\mathbf{b})=0$)であり、
sequential impulses は射影ガウス=ザイデル(PGS)による反復解に相当する(Catto, GDC 2005/2009)。

## 3. 状態表現・Rust 型定義

```rust
pub struct ContactConstraint {
    pub body_a: BodyIdx, pub body_b: BodyIdx,
    pub normal: Vec3,
    pub tangent: (Vec3, Vec3),          // 接線基底 (摩擦用)
    pub points: ArrayVec<ContactPointConstraint, 4>,
    pub friction: f64,                  // 結合則 [04-friction.md]
    pub restitution: f64,
}
pub struct ContactPointConstraint {
    pub r_a: Vec3, pub r_b: Vec3,        // 重心→接触点
    pub normal_mass: f64,                // m_eff (prepare で計算)
    pub tangent_mass: (f64, f64),
    pub velocity_bias: f64,              // 反発 + Baumgarte
    pub normal_impulse: f64,             // 累積 (warm start で持ち越し)
    pub tangent_impulse: (f64, f64),
    pub feature_id: u32,
}
```

## 4. 数値解法 — sequential impulses

### 4.1 全体フロー(1 mechanics ステップ内)

```
prepare:  各接触点の m_eff・接線基底・velocity_bias を計算
warm start: 前ステップの累積インパルスを適用 (§4.4)
velocity iterations (N_v 回):
    for each constraint (固定順): 法線 solve → 摩擦 solve
position correction (§4.5)
```

### 4.2 法線方向の 1 反復(具体式)

```text
solve_normal(point p):
  v_n = n · [(v_B + ω_B×r_B) − (v_A + ω_A×r_A)]
  Δj  = −(v_n − p.velocity_bias) * p.normal_mass
  # 累積インパルスをクランプ (これが正しい非負条件の入れ方)
  j_old = p.normal_impulse
  p.normal_impulse = max(j_old + Δj, 0)
  Δj = p.normal_impulse − j_old
  apply impulse Δj·n to A(−), B(+)
```

**個々の $\Delta j$ でなく累積値をクランプする**のが要点: 反復中に一時的に負方向の修正
(押しすぎの取り消し)が可能になり、PGS が正しい MLCP 解に収束する。

### 4.3 velocity_bias(反発と位置補正)

$$b = \underbrace{-e \cdot \max(-v_n^{pre} - v_{thresh},\, 0)}_{反発} \;+\; \underbrace{\frac{\beta}{\Delta t}\max(\delta - \delta_{slop},\, 0)}_{Baumgarte}$$

- $v_n^{pre}$: ソルバ開始前(重力適用後)の法線速度。反発閾値 $v_{thresh} = 0.5$ m/s 未満の
  微小衝突では反発させない(静止接触のジッタ・微小バウンド防止)。
- Baumgarte 項: 貫入 $\delta$ を 1 ステップあたり比率 $\beta$ で押し戻す速度を注入する。
  副作用として偽のエネルギー注入がある → §4.5 の split impulse で置き換え可能にする。

### 4.4 Warm starting

前ステップの累積インパルス(feature_id で対応づけ)をソルバ開始時にそのまま適用する。
スタック(積み荷)では毎ステップほぼ同じ力分布になるため、収束が劇的に速くなる。
これが 4 段積みが 10 反復で安定する鍵。

### 4.5 位置補正の 2 方式

- **Baumgarte(Phase 1)**: §4.3 の bias 項。実装が簡単だが跳ねの副作用。
- **Split impulse / NGS(Phase 2)**: 速度とは別に擬似速度(または位置直接修正)で貫入を解消。
  velocity solve 後に position iterations($N_p$ 回)で
  $\Delta\lambda = \beta_{pos}\max(\delta - \delta_{slop},0)\, m_{eff}$ を位置・姿勢に直接適用。
  エネルギーを汚さないため反発テストが厳密になる。

### 4.6 反復回数と収束

- 既定 $N_v = 10$, $N_p = 4$(Box2D 準拠)。UI から変更可能にし、「反復を減らすと積みが崩れる」
  こと自体を観察対象にする(検証して遊ぶ)。
- 拘束の処理順は決定的(manifold 生成順)。順序が解に影響する(PGS の性質)ことは
  文書化し、順序を固定することで再現性を保証する。

## 5. 適用スケールと限界

- 速度レベル解法なので、静止摩擦下の微小ドリフト・長いスタックのわずかな沈み込みは残る
  (許容: 10 s で 1 mm 未満/box 4 段)。より高精度が要る場合は反復増 or 直接法(将来)。
- 反発係数モデル(ニュートン則)は速度非依存の近似。実物の $e$ は衝突速度に依存する
  (高速で小さくなる)— Phase 4 で速度依存 $e(v)$ をマテリアルに追加可能な設計とする。
- 同時多体衝突(ニュートンのゆりかご)は PGS では伝播が反復回数依存。
  ゆりかごデモでは反復を増やす(または衝突順逐次処理モード)ことを明記。

## 6. 他ドメインとの結合

- **散逸エネルギー → 熱**: 各接触の非弾性散逸
  $\Delta E = \frac{1}{2}m_{eff}\,(1-e^2)\,(v_n^{pre})^2$ と摩擦仕事([04-friction.md](04-friction.md) §6)を
  接触イベントに載せ、熱ドメインが両体の熱容量比で分配する([20-integration/01-coupling-matrix.md](../20-integration/01-coupling-matrix.md))。

## 7. 検証

- 1D 正面衝突(等質量・$e=1$): 速度交換が機械精度で成立。運動量保存 $<10^{-9}$ 相対。
- $e=0.5$ 落下バウンド: 跳ね返り高さ比 $= e^2 \pm 1\%$(split impulse モード)。
- 4 段スタック: 10 s 後の全速度 $< 10^{-3}$ m/s、貫入 $< \delta_{slop}$。
- エネルギー: 反発 $e<1$ で単調非増加(Baumgarte 起因の増加が閾値内であること、
  split impulse で消えること)。

## 8. 実装フェーズ対応

Phase 1: 法線 + 反発 + Baumgarte + warm start + 摩擦。Phase 2: split impulse、スリープ統合。

## 9. パラメータ表

| パラメータ | 既定値 | 根拠 |
|---|---|---|
| velocity iterations $N_v$ | 10 | Box2D 準拠、スタック 4〜8 段で十分 |
| position iterations $N_p$ | 4 | 同上 |
| Baumgarte $\beta$ | 0.2 | 標準値 (0.1–0.3)。大きいと跳ねる |
| slop $\delta_{slop}$ | 5 mm | 接触を保つ許容貫入。見た目と安定の妥協点 |
| 反発閾値 $v_{thresh}$ | 0.5 m/s | ジッタ防止 (≈ 1.3 cm からの落下速度) |
