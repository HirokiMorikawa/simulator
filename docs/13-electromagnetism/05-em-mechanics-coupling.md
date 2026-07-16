# 電磁気 05. 電磁気⇔力学の結合 — ローレンツ力・誘導・モーター

crate: `sim-em` + `sim-coupling`。電磁気と力学の間のエネルギー・力の橋。
モーターは「電磁⇔力学⇔熱」の三重結合の代表例としてここで完結に設計する。

## 1. 担う現実の現象

モーターが回る・負荷で電流が増える、発電機(手回しライト)、電磁誘導ブレーキ、
レール上を滑る導体棒、荷電粒子の螺旋運動。
遊び方の例: 手回し発電の重さと点灯の関係、磁石を落とすと銅管の中でゆっくり落ちる(渦電流)、
モーターの効率測定。

## 2. 支配方程式

### 2.1 ローレンツ力

$$\mathbf{F} = q(\mathbf{E} + \mathbf{v}\times\mathbf{B})$$

帯電剛体・荷電粒子デモに直接適用(force generator)。

### 2.2 電磁誘導(ファラデー則)

$$\mathcal{E} = -\frac{d\Phi_B}{dt}, \qquad \Phi_B = \int\mathbf{B}\cdot d\mathbf{A}$$

導体棒(長さ $\ell$、速度 $v$、磁場 $B$ 直交): $\mathcal{E} = B\ell v$。
回路の電圧源として注入し、流れた電流が受ける力 $\mathbf{F} = I\boldsymbol\ell\times\mathbf{B}$ が
運動を減速する(レンツ則が自動的に成立 — エネルギー保存の帰結)。

### 2.3 DC モーター(集中定数モデル)

電気側(回路素子として、[02-circuits.md](02-circuits.md)):
$$v = R_a i + L_a \frac{di}{dt} + k_e\,\omega$$
機械側(ヒンジモーター行として、[10-mechanics/05](../10-mechanics/05-joints-constraints.md) §4.5):
$$\tau = k_t\, i - \tau_{friction}(\omega)$$

SI では $k_e = k_t$(同一定数 $k$ [V·s/rad = N·m/A])— **これがエネルギー保存の証明**:
電気入力 $k_e\omega i$ = 機械出力 $k_t i\omega$。発電機は同じ式の逆向き(外部トルクで
$\omega$ を与えると $k\omega$ が起電力になる)。モデルは可逆で、モーター/発電機の区別は不要。

### 2.4 渦電流ブレーキ(現象論)

導体近傍で動く磁石の減速力: $\mathbf{F} = -c_{eddy}\,\mathbf{v}_\perp$
($c_{eddy} \propto \sigma_e t_{板} B^2 A$、幾何依存の係数はデモごとに校正)。
完全な渦電流場は解かない(§5)。散逸は導体のジュール熱として記帳。

## 3. 状態表現

```rust
/// 回路のモーター素子と力学のヒンジを対にする結合
pub struct MotorCoupling {
    pub circuit_element: ElementId,   // モーター素子 (R_a, L_a, k)
    pub hinge: JointId,               // 回転子のヒンジ
    pub rotor_inertia: f64,           // 回転子慣性 (ヒンジ側ボディに含める)
}
pub struct InductionRod { pub body: BodyId, pub circuit_node: (NodeId, NodeId), pub length: Vec3 }
pub struct EddyBrake { pub magnet: BodyId, pub conductor: BodyId, pub coeff: f64 }
```

## 4. 数値解法(結合の時間進行)

モーターは回路 sub-step(0.26 ms)と力学 step(8.3 ms)の 2 時間スケール:

```text
mechanics step 開始時:
  ω = hinge.angular_velocity          # 力学の確定値 (前ステップ)
circuit sub-steps (32 回):
  逆起電力 k·ω を電圧源として MNA を解く (ω はステップ内一定と近似)
  i を積算 → 平均電流 ī
mechanics solver:
  ヒンジのモーター行に τ = k·ī を目標トルクとして与える (上限 τ_max = k·i_max)
熱:
  ジュール熱 Σ i²R_a dt → モーターの ThermalNode
```

- $\omega$ をステップ内一定とする近似は、機械時定数 ≫ 電気時定数(通常成立)で妥当。
  成立しない極端ケース(無慣性ロータ)は診断警告。
- エネルギー台帳: 電池 → (ジュール熱) + (磁場蓄積 $\frac12L i^2$) + (機械仕事 $k\bar i\omega\Delta t$) の
  収支を毎ステップ検算(residual < 10⁻⁶)。

## 5. 適用スケールと限界

- モーターはブラシ付き DC の平均値モデル: トルクリップル・整流・突極性は捨象。
  ブラシレスの詳細な電気角制御はエンティティ層(コントローラ)の将来課題。
- 渦電流は現象論係数(±50%)。分布渦電流の直接計算(準静的 FEM)は対象外と明記。
- 磁気飽和・鉄損は捨象(効率がやや楽観的になる、と表示)。

## 6. 他ドメインとの結合

本文書自体が結合の設計。加えて:

- 熱 → 電気: 巻線抵抗の温度依存 $R_a(T)$ — モーターが熱くなると弱くなる(Phase 5)。
- エンティティ層: 乗り物の駆動(スロットル → 電圧指令)、ロボットの関節
  ([20-integration/03](../20-integration/03-entity-layer.md))。

## 7. 検証

- 無負荷回転数: $\omega_{nl} = (V - R_a i_{nl})/k \approx V/k$ ± 1%。
- ストールトルク: $\tau_{stall} = kV/R_a$ ± 1%。トルク-速度直線の傾き $-k^2/R_a$。
- 最大効率点の存在と理論値(モーター定数から)± 2%。
- 発電: 外部駆動 → $V_{oc} = k\omega$、負荷接続で制動トルク増(レンツ則)。
- サイクロトロン: 一様 B 中の荷電粒子の半径・周期(解析値 ± 0.5%、
  エネルギー保存: 磁場は仕事をしない → 速さ一定 < 10⁻⁹)。
- 磁石落下(銅管): 終端速度の存在、散逸 = ジュール熱(台帳)。

## 8. 実装フェーズ対応

Phase 4: ローレンツ力・モーター/発電機・導体棒・渦電流ブレーキ(全デモ含む)。
Phase 5: 温度依存・ブラシレス検討。

## 9. パラメータ表(小型 DC モーター代表値: マブチ FA-130 相当)

| パラメータ | 値 |
|---|---|
| $R_a$ | 1.0 Ω |
| $L_a$ | 0.3 mH |
| $k$ ($=k_e=k_t$) | 1.7×10⁻³ V·s/rad |
| 無負荷回転数 @3V | ~12000 rpm |
| 回転子慣性 | ~1×10⁻⁷ kg·m² |
| 摩擦トルク | ~5×10⁻⁵ N·m(クーロン+粘性) |
