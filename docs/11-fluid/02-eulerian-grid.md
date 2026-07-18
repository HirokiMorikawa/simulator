# 流体 02. 格子(Eulerian)ソルバ — MAC 格子・移流・投影法

crate: `sim-fluid`。煙・気流・閉領域の水など、固定領域の非圧縮流の主力ソルバ。
自由表面(水しぶき等)は SPH([03-sph.md](03-sph.md))と使い分ける(§5)。

## 1. 担う現実の現象

部屋の中の煙の流れ、対流(暖房・ろうそくの上昇気流)、風洞、水槽内の循環流。
遊び方の例: 障害物を置いて煙の渦(カルマン渦)を見る、ろうそくの熱で対流を起こす。

## 2. 支配方程式

Navier-Stokes(非圧縮、[01-continuum-basics.md](01-continuum-basics.md) §2)を operator splitting で分割:

$$\text{移流} \to \text{外力} \to \text{粘性} \to \text{投影(圧力)}$$

各段が 1 つの物理項を担当する(Stam, *Stable Fluids*, 1999 系列の標準構成)。

## 3. 状態表現

```rust
pub struct GridFluid {
    pub vel: MacGrid,                  // u,v,w (スタガード)
    pub pressure: Grid3<f64>,          // セル中心
    pub cell_type: Grid3<CellType>,    // Fluid / Solid / Empty
    pub smoke_density: Grid3<f64>,     // 受動スカラー (煙・染料)
    pub temperature: Grid3<f64>,       // 熱結合 (Boussinesq)
    pub solid_vel: MacGrid,            // 固体境界の速度 (動く剛体)
}
```

## 4. 数値解法(各段の具体式)

### 4.1 移流 — semi-Lagrangian

各速度サンプル点 $\mathbf{x}$ について、速度場を逆向きに辿った出発点の値を持ってくる:

$$q^{n+1}(\mathbf{x}) = q^n\big(\mathbf{x} - \Delta t\,\mathbf{u}^n(\mathbf{x})\big)$$

バックトレースは RK2(中点法): $\mathbf{x}_{mid} = \mathbf{x} - \frac{\Delta t}{2}\mathbf{u}(\mathbf{x})$、
$\mathbf{x}_{src} = \mathbf{x} - \Delta t\,\mathbf{u}(\mathbf{x}_{mid})$。補間はトライリニア(高品質モードで
クランプ付き Catmull-Rom、[01-math/02](../01-math/02-fields.md) §3)。無条件安定だが数値拡散が大きい —
渦の減衰は §4.5 で補償。スカラー(煙・温度)も同じ移流を使う。

### 4.2 外力

重力(密度差)、Boussinesq 浮力 $f_y = \alpha_{smoke}\, s - \beta (T - T_{amb})$ 系
(煙は重く $s$、熱は軽く $T$)、ユーザー入力(かき混ぜ)、渦度強化(§4.5)を速度に加算。

### 4.3 粘性

日常スケールの水・空気は $\nu\Delta t/h^2 \ll 1$ のため、既定では**陽的**
$\mathbf{u} \mathrel{+}= \nu\Delta t\,\nabla^2\mathbf{u}$、
高粘性流体(蜂蜜)では陰的(PCG、無条件安定)に切り替える。切替閾値: $\nu\Delta t/h^2 > 0.25$。

### 4.4 投影(圧力ソルバ)

仮速度 $\mathbf{u}^*$ から非圧縮成分を取り出す。圧力 Poisson 方程式:

$$\nabla^2 p = \frac{\rho}{\Delta t}\,\nabla\cdot\mathbf{u}^*$$

を PCG([01-math/02](../01-math/02-fields.md) §5)で解き、$\mathbf{u}^{n+1} = \mathbf{u}^* - \frac{\Delta t}{\rho}\nabla p$。
離散化は MAC 格子の標準 7 点ステンシル。境界条件:

- **Solid セル面**: $\mathbf{u}\cdot\hat{n} = \mathbf{u}_{solid}\cdot\hat{n}$(法線速度一致。
  接線は free-slip 既定、no-slip はオプション)。Poisson には Neumann($\partial p/\partial n$ 指定)。
- **Empty(自由表面/開放境界)**: $p = 0$(Dirichlet)。

### 4.5 渦度強化(vorticity confinement、オプション)

数値拡散で失われる小渦を補償: 渦度 $\boldsymbol\omega_v = \nabla\times\mathbf{u}$、
$\mathbf{N} = \nabla|\boldsymbol\omega_v| / |\nabla|\boldsymbol\omega_v||$ として
$\mathbf{f}_{conf} = \varepsilon_{conf}\, h\, (\mathbf{N}\times\boldsymbol\omega_v)$ を加える(Fedkiw et al. 2001)。
**非物理的な補償項**であることを UI の近似表示で明示し、検証モードでは無効化する。

ただし **F11(カルマン渦)は例外の可能性がある**:
実装時にまず 64³・渦度強化オフで渦離脱が自発的に立ち上がるかを数値実験で確認し、
立ち上がらない場合は検証モードでも渦度強化オンを許容(強化係数を合格条件に記録)するか、
解像度・レイノルズ数指定を変更して合格条件を確定する
([21-verification/01-analytic-tests.md](../21-verification/01-analytic-tests.md) F11 注記)。

### 4.6 ステップまとめ

```text
grid_fluid_step(dt):                        # dt: CFL≦5 なら world_dt そのまま
  advect(vel), advect(smoke), advect(temperature)      # semi-Lagrangian RK2
  add_forces(buoyancy, user, confinement)
  diffuse(vel) if ν有効                                  # 陽的 or PCG
  set_boundary_conditions(cell_type, solid_vel)
  solve_pressure(PCG, tol=1e-4 (対話) / 1e-6 (検証))
  project(vel)
```

計算量: 移流 $O(N)$、PCG $O(N^{4/3})$ 程度(反復×$O(N)$)。64³ で予算 4 ms([00-foundation/05](../00-foundation/05-rust-wasm-platform.md) §5)。

## 5. 適用スケールと限界

- 領域は固定の直方体格子。解像度以下の構造(細い隙間・薄い障害物)は表現不可(セル単位に丸める)。
- semi-Lagrangian の数値拡散: 渦の寿命が実際より短い。定量検証は大域量に限定([01](01-continuum-basics.md) §5)。
- 自由表面の追跡(level set / FLIP)は格子法では Phase 5 課題とし、
  **水しぶき・注水は SPH に割り当てる**。使い分け: 閉領域・気体 → 格子、自由表面・飛沫 → SPH。
- 質量保存: semi-Lagrangian のスカラー移流は保存誤差を持つ(煙の総量ドリフト ~%/s)。
  保存が重要な量(将来: 塩分・化学種)は保存形スキームを追加検討。

## 6. 他ドメインとの結合

- **剛体→流体**: 剛体を Solid セルにボクセル化(`Shape::contains`)、`solid_vel` に剛体表面速度を書く。
- **流体→剛体**: 剛体表面セルの圧力を面積分 $\mathbf{F} = -\oint p\,\hat{n}\,dA$(+粘性せん断は省略、
  誤差要因として明記)し、力・トルクとして返す。集中定数モデル([05](05-aero-hydrodynamics.md))とは
  **排他利用**(シーン設定でどちらかを選ぶ。二重計上禁止を結合行列に明記)。
- **熱**: 温度場は本ソルバの移流を使い、拡散・発熱は熱ドメインの式([12-thermal/02](../12-thermal/02-heat-transfer.md))。
  Boussinesq 浮力で運動量へ。

## 7. 検証

- 静水圧: 重力下の閉水槽で $p = \rho g h \pm 1\%$、速度が 0 に留まる。
- ポアズイユ流(2D 設定): 放物型プロファイル誤差 < 2%(粘性・境界条件の検証)。
- Taylor-Green 渦: 減衰率 $e^{-2\nu k^2 t}$、$\nu$ を数点変えて検証(< 5%、数値拡散分を除くため低解像度では緩和)。
- 発散: 投影後 $|\nabla\cdot\mathbf{u}| < 10^{-6}$(PCG 収束基準に整合)。
- カルマン渦列: 円柱後流のストローハル数 $St = fD/U \approx 0.2$($Re \sim 100$–1000、±20% —
  数値拡散を考慮した緩い基準)。

## 8. 実装フェーズ対応

Phase 2: スカラー移流+固定風場(煙のみ、投影なし)。Phase 3: 完全ソルバ(投影・浮力・剛体結合)。
Phase 5: FLIP/level set 検討。

## 9. パラメータ表

| パラメータ | 既定値 | 根拠 |
|---|---|---|
| 解像度 | 64³(対話)/ 128³(オフライン) | 性能予算 |
| PCG 許容残差 | 10⁻⁴(対話)/ 10⁻⁶(検証) | 視覚 vs 定量 |
| 渦度強化 $\varepsilon_{conf}$ | 0(検証)/ 0.5(見た目) | 非物理項ゆえ既定オフ |
| Boussinesq $\beta$(空気) | $1/T_{amb} \approx 3.4\times10^{-3}$ K⁻¹ | 理想気体の体膨張係数 |
| smoke 浮力 $\alpha_{smoke}$ | シーン指定 | 煙の相対密度による |

## 10. 性能プロファイル

- ホットスポット: 圧力 Poisson の PCG 反復、semi-Lagrangian 移流の補間。
- 目標アルゴリズムとオーダー: PCG → **マルチグリッド前処理**で反復数を大幅削減(~$O(n)$)。
- SoA レイアウト: MacGrid の u/v/w・圧力・cell_type を別配列。**キャッシュブロッキング/タイリング**(8³ ブロック)。
- 並列化単位: 格子スライス/タイルを rayon 分割。PCG の内積は順序固定リダクション。
- SIMD 対象カーネル: 7 点ステンシル(ラプラシアン・発散・勾配)、移流の補間。
- GPU 適性: **高**(格子ソルバは GPU の代表用途。WebGPU compute、CPU 参照実装維持)。
- ベンチ: 64³ 煙・カルマン渦で criterion(予算 4 ms)。
