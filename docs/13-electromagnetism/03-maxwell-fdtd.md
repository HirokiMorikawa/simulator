# 電磁気 03. マクスウェル方程式と FDTD — 電磁波の直接シミュレーション

crate: `sim-em`。電磁波そのものを見る**専用シナリオ向け**ソルバ(常時稼働の環境物理ではない)。

## 1. 担う現実の現象

電波の伝播・反射・回折、アンテナからの放射、電子レンジの定在波、導波管。
遊び方の例: ダイポールアンテナの放射パターン、スリットでの回折、
誘電体での屈折(光学ドメインの幾何光学と同じ現象の波動版 — スケールの階梯を見せる)。

## 2. 支配方程式

真空・線形媒質中のマクスウェル方程式(回転系):

$$\frac{\partial\mathbf{B}}{\partial t} = -\nabla\times\mathbf{E},\qquad
\varepsilon\frac{\partial\mathbf{E}}{\partial t} = \frac{1}{\mu}\nabla\times\mathbf{B} - \mathbf{J}$$

(発散系 2 本は Yee 格子が自動保存する)。媒質は $\varepsilon(\mathbf{x}), \mu(\mathbf{x}), \sigma_e(\mathbf{x})$
(導電率 → 吸収)。

## 3. 数値解法 — Yee 格子 FDTD

### 3.1 空間配置

E 成分をセル辺、H(B)成分をセル面に置く(Yee 1966)。回転が自然な中心差分になり、
$\nabla\cdot\mathbf{B}=0$ が丸め誤差レベルで恒久保存。MAC 格子([01-math/02](../01-math/02-fields.md) §2)と
同じスタガード思想 — 実装は `Grid3` を成分ごとに再利用。

### 3.2 時間更新(leapfrog、2D TM モードの例)

$$H_x^{n+1/2} = H_x^{n-1/2} - \frac{\Delta t}{\mu h}\big(E_z[i,j{+}1]-E_z[i,j]\big)$$
$$E_z^{n+1} = \frac{1-\frac{\sigma_e\Delta t}{2\varepsilon}}{1+\frac{\sigma_e\Delta t}{2\varepsilon}} E_z^{n}
+ \frac{\Delta t/(\varepsilon h)}{1+\frac{\sigma_e\Delta t}{2\varepsilon}}\big[(H_y[i,j]-H_y[i{-}1,j]) - (H_x[i,j]-H_x[i,j{-}1])\big]$$

二次精度・陽的。**Courant 条件**: $\Delta t \le \frac{h}{c\sqrt{d}}$($d$: 次元)。

### 3.3 境界・源

- 吸収境界: PML(perfectly matched layer、8–16 層、多項式グレーディング)。
  簡易モードで Mur 1 次。
- 源: ハード/ソフト源(点・線)、ガウシアンパルス・連続正弦波。
  Total-Field/Scattered-Field(平面波入射)は Phase 5。

### 3.4 規模と実行モード

3D 128³ × 数千ステップは対話予算を超える → **既定は 2D**(TM/TE、128²–512²)。
3D は小域(64³)+ オフライン実行。「電磁波の時間スケール($10^{-9}$ s)を
スロー再生する専用シーン」として提供し、力学時間とは**独立の時間軸**で走らせる
(結合しない。理由は §5)。

```rust
pub struct FdtdSim {
    pub ez: Grid3<f64>, pub hx: Grid3<f64>, pub hy: Grid3<f64>,   // 2D TM
    pub eps_r: Grid3<f64>, pub sigma: Grid3<f64>,
    pub pml: PmlLayers,
    pub sources: Vec<FdtdSource>,
}
```

## 5. 適用スケールと限界(§4 と統合)

- **力学との時間結合はしない**: 光速の波と剛体($10^{10}$ 倍の時間スケール差)の同時進行は
  無意味(剛体は 1 波動ステップで動かない)。FDTD は「顕微鏡モード」の独立シーンとし、
  幾何形状(反射板・スリット)だけを共有する。日常シーンの電磁気は
  回路([02](02-circuits.md))・静場([01](01-electrostatics-magnetostatics.md))・光線([04](04-light-optics.md))が担う
  — この分業自体が有効理論の階梯の実演である。
- 格子分散: 波長あたり $\ge 20$ セルで位相速度誤差 < 0.2%。それ以下は分散が見える(教材化)。
- 非線形・分散媒質(Drude モデル等)は Phase 5+。

## 6. 他ドメインとの結合

- 幾何共有のみ(§5)。検証として幾何光学(スネル則)と同じ設定で屈折角が一致することを見せる。
- 吸収电力 $\sigma_e|\mathbf{E}|^2$ → 熱(電子レンジデモ、オフライン、Phase 5)。

## 7. 検証

- 平面波伝播速度: $c \pm 0.5\%$(20 セル/波長)。
- 誘電体界面: 反射・透過係数がフレネル式と一致(± 1%、垂直入射)。
- 共振器: 矩形空洞の共振周波数 $f_{mn} = \frac{c}{2}\sqrt{(m/a)^2+(n/b)^2}$ ± 1%。
- PML: 反射率 < −40 dB。
- エネルギー: 無損失域で $\int(\frac{\varepsilon E^2}{2} + \frac{B^2}{2\mu})dV$ 保存(PML 吸収分を除き < 0.1%)。
- 二重スリット回折パターン: フラウンホーファー近似 $\sin\theta = m\lambda/d$ と一致 —
  量子ドメインの電子二重スリット([14-quantum/02](../14-quantum/02-schrodinger-solver.md))と並べ、
  「波の干渉」の普遍性を見せる統合デモ。

## 8. 実装フェーズ対応

Phase 5: 2D TM/TE + PML + 基本源 + 上記デモ。3D 小域はその後。

## 9. パラメータ表

| パラメータ | 既定値 | 根拠 |
|---|---|---|
| Courant 数 | 0.5(2D 上限 0.707) | 安定余裕 |
| セル/波長 | 20 | 分散誤差 < 0.2% |
| PML 層数 / 次数 | 10 / 3 | Taflove & Hagness, *Computational Electrodynamics* 推奨域 |
| 2D 既定解像度 | 256² | 予算 5 ms([00-foundation/05](../00-foundation/05-rust-wasm-platform.md) §5) |
