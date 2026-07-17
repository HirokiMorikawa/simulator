# 01. 線形代数基盤 — Vec3 / Quat / Mat3 / Transform

全ドメインが使う幾何・線形代数の型と演算。crate: `sim-math`。外部数学ライブラリ(glam / nalgebra)は使わず**自作**する。
理由: (1) 検証して遊ぶ・学ぶプロジェクトとして実装の透明性に価値がある、(2) f64 固定・決定論の完全な制御、
(3) 必要な演算は限定的で自作コストが小さい。実装は既知値テストと恒等式テスト(§6)で保証する。

## 1. 型定義

```rust
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct Vec3 { pub x: f64, pub y: f64, pub z: f64 }

/// 単位クォータニオン (回転)。w が実部。恒等回転は (0,0,0,1)。
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Quat { pub x: f64, pub y: f64, pub z: f64, pub w: f64 }

/// 3x3 行列。行優先 (m[row][col])。慣性テンソル・回転行列に使う。
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Mat3 { pub m: [[f64; 3]; 3] }

/// 剛体変換 (回転→平行移動の順で適用)。スケールは持たない。
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Transform { pub position: Vec3, pub rotation: Quat }
```

- すべて `Copy`(16〜72 byte、値渡しで良い)。演算子オーバーロード(`Add/Sub/Mul/Neg`)を実装。
- SoA バッファ(`Vec<f64>` × 成分)との相互変換ヘルパを持つ(WASM 転送・SIMD 化の下地)。

## 2. Vec3 演算(仕様)

標準演算(add/sub/scale/dot/cross/length/normalize)に加え、物理でよく使う:

```rust
impl Vec3 {
    pub fn length_sq(self) -> f64;                    // 比較には length でなくこれを使う
    pub fn normalize_or_zero(self) -> Vec3;           // |v| < EPS_LEN なら 0 (EPS_LEN = 1e-12)
    pub fn cross(self, rhs: Vec3) -> Vec3;            // 右手系
    pub fn addcarry_scaled(self, v: Vec3, s: f64) -> Vec3;  // self + v*s (積分で頻出)
    /// v に直交する任意の単位ベクトル対 (接線基底)。摩擦・接触で使用。
    /// 決定的アルゴリズム: |x|が最小の軸との外積 → 正規化 → もう1本は外積。
    pub fn orthonormal_basis(self) -> (Vec3, Vec3);
}
```

- `orthonormal_basis` は分岐条件を成分の大小比較で固定し、決定論を保つ(プラットフォーム依存の
  三角関数を避ける)。

## 3. Quat 演算(仕様と式)

```rust
impl Quat {
    pub fn from_axis_angle(axis: Vec3, angle_rad: f64) -> Quat;  // (a sin(θ/2), cos(θ/2))
    pub fn mul(self, rhs: Quat) -> Quat;         // 回転の合成: self が後 (self∘rhs)
    pub fn conjugate(self) -> Quat;              // 単位quatでは逆回転
    pub fn rotate(self, v: Vec3) -> Vec3;        // v' = q v q*
    pub fn normalize(self) -> Quat;
    pub fn to_mat3(self) -> Mat3;
    pub fn slerp(self, to: Quat, t: f64) -> Quat; // 描画補間用 (コア物理では未使用)
    /// 角速度 ω による回転の積分 (一次):
    ///   q(t+dt) = normalize( q + dt/2 * ω_quat ⊗ q ),  ω_quat = (ωx, ωy, ωz, 0)
    pub fn integrate_angular_velocity(self, omega: Vec3, dt: f64) -> Quat;
}
```

- `rotate` はクォータニオン積 2 回でなく展開式(Rodrigues 相当、乗算 15 回)で実装する:
  $\mathbf{v}' = \mathbf{v} + 2\mathbf{q}_v \times (\mathbf{q}_v \times \mathbf{v} + w\,\mathbf{v})$。
- 積分後の正規化は毎回行う(ドリフト防止。正規化コストは無視できる)。
- 二重被覆($q$ と $-q$ が同じ回転)は、補間時のみ内積の符号で吸収する。

## 4. Mat3 演算(仕様)

慣性テンソルの座標変換が主用途。

```rust
impl Mat3 {
    pub fn identity() -> Mat3;
    pub fn from_diagonal(d: Vec3) -> Mat3;
    pub fn mul_vec(self, v: Vec3) -> Vec3;
    pub fn mul(self, rhs: Mat3) -> Mat3;
    pub fn transpose(self) -> Mat3;
    pub fn inverse(self) -> Option<Mat3>;        // 余因子法。det < EPS_DET で None
    /// 相似変換 R * self * R^T。ローカル慣性テンソル→ワールドで毎ステップ使用。
    pub fn similarity(self, r: Mat3) -> Mat3;
    /// 歪対称行列 [v]× (v.cross(x) = skew(v) * x)。ヤコビアン組み立てで使用。
    pub fn skew(v: Vec3) -> Mat3;
}
```

## 5. Transform

```rust
impl Transform {
    pub fn apply_point(self, p: Vec3) -> Vec3;      // R p + t
    pub fn apply_dir(self, d: Vec3) -> Vec3;        // R d (平行移動なし)
    pub fn inverse(self) -> Transform;              // (R^-1, -R^-1 t)
    pub fn compose(self, inner: Transform) -> Transform; // self ∘ inner
}
```

- 形状は常にローカル座標で定義し、衝突検出は Transform を介して評価する
  ([10-mechanics/02-collision-detection.md](../10-mechanics/02-collision-detection.md))。

## 6. テスト(実装フェーズの受け入れ基準)

- 既知値: 基本演算の手計算ケース(例: $\hat{x}\times\hat{y}=\hat{z}$、90° 回転)。
- 恒等式(ランダム入力 × 決定シードで 10⁴ ケース、$\epsilon_{abs}=10^{-12}$):
  - $|q\,\mathbf{v}\,q^*| = |\mathbf{v}|$(回転は長さを保つ)
  - `to_mat3(q).mul_vec(v) == q.rotate(v)`
  - `m.mul(m.inverse()) == I`(条件数の良い行列で)
  - `skew(a).mul_vec(b) == a.cross(b)`
  - `integrate_angular_velocity` を n 分割して合成 → 解析回転 `from_axis_angle(ω̂, |ω| t)` に一次収束
- 決定論: 同一バイナリ内では全演算が同一入力→ビット同一出力。ターゲット間(wasm⇔ネイティブ)の
  最適化差(FMA・libm 実装差)はビット一致を要求せず、許容誤差で照合する
  ([20-integration/02](../20-integration/02-determinism-replay.md) §5 階層 2)。

## 7. パフォーマンス指針

- ホットループ(接触ソルバ、SPH)ではメソッドチェーンより明示的な成分演算を許可(インライン化は
  `#[inline]` 指定 + ベンチで確認)。
- SIMD(wasm simd128 / std::simd)は Phase 3 以降にホットスポット限定で導入。導入時も
  スカラー実装を参照実装として残し、結果一致テストを課す(決定論)。
