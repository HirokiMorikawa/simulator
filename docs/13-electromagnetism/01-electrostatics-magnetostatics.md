# 電磁気 01. 静電場・静磁場 — クーロン力・帯電・磁石

crate: `sim-em`。時間変化の遅い($L \ll \lambda$、[00-foundation/02](../00-foundation/02-scale-ladder.md) §3)
電場・磁場。回路は [02-circuits.md](02-circuits.md)、波動は [03-maxwell-fdtd.md](03-maxwell-fdtd.md)。

## 1. 担う現実の現象

こすった風船が壁に貼りつく、静電気のパチッ、髪の毛が逆立つ、磁石の吸引・反発、方位磁針。
遊び方の例: 帯電量と吸引力の逆二乗則の測定、電場の可視化(電気力線)、磁石での砂鉄模様。

## 2. 支配方程式

静電場(マクスウェル方程式の静的極限):

$$\nabla\cdot\mathbf{E} = \rho_q/\varepsilon_0,\qquad \nabla\times\mathbf{E} = 0 \;\Rightarrow\; \mathbf{E} = -\nabla\phi,\qquad \nabla^2\phi = -\rho_q/\varepsilon_0$$

点電荷解(クーロンの法則): $\mathbf{E} = \frac{q}{4\pi\varepsilon_0 r^2}\hat{r}$、力 $\mathbf{F} = q\mathbf{E}$。

静磁場: $\nabla\cdot\mathbf{B}=0$, $\nabla\times\mathbf{B} = \mu_0\mathbf{J}$。
磁気双極子 $\mathbf{m}$ の場:

$$\mathbf{B}(\mathbf{r}) = \frac{\mu_0}{4\pi}\left[\frac{3(\mathbf{m}\cdot\hat r)\hat r - \mathbf{m}}{r^3}\right]$$

双極子間の力・トルク: $\mathbf{F} = \nabla(\mathbf{m}\cdot\mathbf{B})$, $\boldsymbol\tau = \mathbf{m}\times\mathbf{B}$。

## 3. 状態表現

**離散源モデル**(場を格子で持たず、源の重ね合わせで評価)を既定とする:

```rust
pub struct ChargedBody { pub body: BodyId, pub charge: f64 }          // 点電荷近似 (重心)
pub struct MagneticDipole { pub body: BodyId, pub moment: Vec3 }      // 永久磁石 (ボディローカル)
pub struct UniformField { pub e: Vec3, pub b: Vec3 }                  // 外部一様場 (地磁気など)
```

- 剛体への付加コンポーネント。帯電はエンティティ操作(こする=摩擦帯電イベント、接触で電荷分配)
  またはシーン指定。
- 導体の誘導電荷・誘電体の分極は**鏡像力の近似式**(平板近傍の点電荷
  $F = -q^2/(16\pi\varepsilon_0 d^2)$)のみ提供(風船が壁に貼りつくデモ用)。
  一般形状の誘導は境界要素法が必要 — Phase 5+ 検討、当面非対応と明記。

## 4. 数値解法

- $N$ 個の源の直接和 $O(N^2)$。対象は数十源(帯電風船・磁石)なので十分。
  多数(>10³)必要になれば Barnes-Hut ツリー(将来)。
- 力は速度に依存しない(静場)ので force generator として mechanics に注入。
  磁気双極子は近距離で $r^{-4}$ の強い力 — 発散防止に最近接距離クランプ
  $r \ge r_{min}$(物体半径和)と、接触後は接触ソルバに委ねる。
- ポテンシャルエネルギーの計上: $U = \sum_{i<j} \frac{q_iq_j}{4\pi\varepsilon_0 r_{ij}}$ 等を
  台帳の em_field に記帳(エネルギー検算)。
- 摩擦帯電: 材料ペアの摩擦帯電系列(トライボエレクトリック系列)から符号を、
  移動電荷量は経験パラメータ(接触面積×係数、§9)で。定量精度を主張しない
  (現実の摩擦帯電は湿度等に敏感)ことを UI に明示。

## 5. 適用スケールと限界

- 準静的近似: 変化の時間スケール $\tau \gg L/c$。日常の帯電・磁石は完全に該当。
- 点電荷・点双極子近似: 物体サイズ ≪ 距離で正確。近接時は誤差増大(表示)。
- 誘導・分極の一般解、強磁性体の非線形磁化(ヒステリシス)、超伝導は対象外。
- 放電(火花)は「電場強度 > 絶縁破壊 3 MV/m(空気)」の閾値イベント + 電荷中和 + 発光/熱として
  現象論的に扱う(Phase 4)。プラズマ物理はやらない。

## 6. 他ドメインとの結合

- 力学: クーロン力・磁気力・トルク(force generator)。
- 熱: 放電エネルギー $\frac{1}{2}CV^2$ 相当を熱・光へ。
- 回路: コンデンサの電場エネルギーは回路側で管理(二重計上禁止)。
- 光学: 放電の発光イベント → 描画層。

## 7. 検証

- 逆二乗: 2 点電荷の力 $F = kq_1q_2/r^2$ 機械精度(直接和なので厳密)。
- 軌道: 電場中の荷電粒子の放物軌道、一様磁場中のサイクロトロン半径 $r = mv/(qB) \pm 0.5\%$
  ([05-em-mechanics-coupling.md](05-em-mechanics-coupling.md) のローレンツ力と統合テスト)。
- 双極子: 2 磁石の吸引力の距離依存 $r^{-4}$(整列時)の冪フィット。
- エネルギー: 静電系の運動+ポテンシャルの保存(< 10⁻⁶ 相対/1000 step)。
- 方位磁針: 一様地磁気(25–65 μT)中の双極子の整列振動周期 $T = 2\pi\sqrt{I/(mB)}$。

## 8. 実装フェーズ対応

Phase 4: 点電荷・双極子・一様場・鏡像力・摩擦帯電イベント・放電イベント。Phase 5: ツリー法・誘導の一般化。

## 9. パラメータ表

| パラメータ | 値 | 出典 |
|---|---|---|
| 空気の絶縁破壊電場 | 3×10⁶ V/m | CRC |
| 地磁気(日本付近、全強度) | ~46 μT | IGRF 代表値 |
| ネオジム磁石の残留磁束密度 | 1.2–1.4 T(N42: $m \approx B_r V/\mu_0$) | メーカー標準値 |
| 摩擦帯電の代表電荷密度 | 10⁻⁶–10⁻⁵ C/m²(上限: 絶縁破壊) | 実験代表値(±1桁) |
| 風船デモの帯電量 | ~10⁻⁷ C | 上記から換算 |
