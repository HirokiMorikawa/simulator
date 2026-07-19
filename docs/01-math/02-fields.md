# 02. 場の表現 — 格子・粒子・補間

流体([11-fluid/](../11-fluid/))・熱([12-thermal/](../12-thermal/))・電磁気([13-electromagnetism/](../13-electromagnetism/))が
共有する「場 (field)」のデータ構造。crate: `sim-math`。

## 1. セル中心格子 `Grid3<T>`

スカラー場(温度・圧力・密度・煙濃度)の基本表現。

```rust
/// 一様間隔の3次元格子。値はセル中心に置く。
pub struct Grid3<T> {
    pub nx: usize, pub ny: usize, pub nz: usize,
    pub h: f64,            // セル幅 [m] (等方)
    pub origin: Vec3,      // セル(0,0,0) の最小コーナーのワールド座標
    data: Vec<T>,          // 長さ nx*ny*nz、インデックス i + nx*(j + ny*k) (x が最内)
}

impl<T: Copy> Grid3<T> {
    pub fn at(&self, i: usize, j: usize, k: usize) -> T;
    pub fn set(&mut self, i: usize, j: usize, k: usize, v: T);
    /// セル(i,j,k) の中心のワールド座標: origin + h*(i+0.5, j+0.5, k+0.5)
    pub fn cell_center(&self, i: usize, j: usize, k: usize) -> Vec3;
    /// ワールド座標 → 含まれるセル (範囲外は境界条件に従う)
    pub fn world_to_cell(&self, p: Vec3) -> (i64, i64, i64);
}
```

- メモリレイアウトは x 最内の一次元 `Vec`(キャッシュ効率、WASM ビュー転送)。
- 2 次元問題(FDTD デモ等)は `nz = 1` で表す(専用 2D 型は作らない)。

### 1.1 境界条件

境界の扱いは場の意味に依存するため、格子ではなく**サンプラ**が持つ:

```rust
pub enum BoundaryRule<T> {
    Clamp,          // 最近傍セル値 (温度など)
    Constant(T),    // 固定値 (無限遠の環境温度など)
    ZeroGradient,   // ∂/∂n = 0 (断熱壁)
    // 周期境界は統計力学デモ用
    Periodic,
}
```

## 2. スタガード格子 `MacGrid`(流体速度用)

非圧縮流体の速度場は **MAC 格子 (Marker-and-Cell)** で持つ。速度成分をセル面の中心に置く:
$u$ は x 面($nx{+}1 \times ny \times nz$)、$v$ は y 面、$w$ は z 面。

```rust
pub struct MacGrid {
    pub u: Grid3<f64>,   // x面: サンプル位置 origin + h*(i, j+0.5, k+0.5)
    pub v: Grid3<f64>,   // y面
    pub w: Grid3<f64>,   // z面
}
```

採用理由: 圧力勾配と発散を**半セルずれた中心差分**で書けるため、圧力と速度の
チェッカーボード分離(奇偶デカップリング)が起きない。発散の離散式:

$$(\nabla\cdot\mathbf{u})_{ijk} = \frac{u_{i+1,j,k}-u_{i,j,k}}{h} + \frac{v_{i,j+1,k}-v_{i,j,k}}{h} + \frac{w_{i,j,k+1}-w_{i,j,k}}{h}$$

任意点の速度サンプリングは各成分を独立にトライリニア補間して合成する。

## 3. 補間

### 3.1 トライリニア補間(既定)

$$f(\mathbf{p}) = \sum_{c\in\{0,1\}^3} f_{i+c_x,\,j+c_y,\,k+c_z}\; \prod_d \big(c_d\, t_d + (1{-}c_d)(1{-}t_d)\big)$$

($t$ はセル内正規化座標)。C0 連続、単調(オーバーシュートしない)、8 セル参照。
移流のサンプリング・粒子↔格子の転送の既定とする。

### 3.2 三次補間(Catmull-Rom、オプション)

semi-Lagrangian 移流の数値拡散を抑えたい高品質モード用。64 セル参照。
オーバーシュートは近傍 8 セルの min/max へのクランプで抑制する(Fedkiw らの標準手法)。

## 4. 微分演算子(セル中心・二次精度中心差分)

$$\nabla f \approx \frac{1}{2h}(f_{i+1}-f_{i-1},\; f_{j+1}-f_{j-1},\; f_{k+1}-f_{k-1})$$
$$\nabla^2 f \approx \frac{1}{h^2}\Big(\sum_{\text{6近傍}} f_{nb} - 6 f_{ijk}\Big)$$

ラプラシアンは熱伝導([12-thermal/02-heat-transfer.md](../12-thermal/02-heat-transfer.md))・圧力 Poisson
([11-fluid/02-eulerian-grid.md](../11-fluid/02-eulerian-grid.md))で共用。
係数が空間変化する場合(熱伝導率の不均一)は流束形式
$\nabla\cdot(k\nabla T)$ を面中心の調和平均 $k_{i+1/2} = 2k_ik_{i+1}/(k_i{+}k_{i+1})$ で離散化する
(界面での流束連続性を保つため。出典: Patankar, *Numerical Heat Transfer*)。

## 5. 線形ソルバ(Poisson 方程式用)

対象: 圧力 Poisson・陰的熱伝導。行列は対称正定(SPD)・7 点ステンシル・疎。

- **既定: 前処理付き共役勾配法 (PCG)**。前処理は不完全コレスキー(IC(0))または対角(Jacobi)。
  収束判定: 相対残差 $\|r\|/\|b\| < 10^{-6}$(流体は $10^{-4}$ で十分、検証モードで厳しく)。
- 行列は組み立てず、**matrix-free**(ステンシル直接適用)で実装する(メモリと速度)。
- 反復回数の上限と残差を `SolverDiagnostics` に記録(発散検知は [00-foundation/04-architecture.md](../00-foundation/04-architecture.md) §5)。
- 決定論: PCG は逐次実行では決定的。並列化時はリダクション順を固定([00-foundation/05-rust-wasm-platform.md](../00-foundation/05-rust-wasm-platform.md) §4)。

```rust
/// A x = b を解く。apply_a はステンシル適用 (matrix-free)。
pub fn pcg(
    apply_a: impl Fn(&[f64], &mut [f64]),
    b: &[f64], x: &mut [f64],
    precond: &Preconditioner, tol_rel: f64, max_iter: usize,
) -> PcgResult;
```

## 6. 粒子集合 `ParticleSet`

SPH([11-fluid/03-sph.md](../11-fluid/03-sph.md))・気体分子([15-statistical/02-kinetic-gas.md](../15-statistical/02-kinetic-gas.md))・
ブラウン粒子が共有する SoA コンテナ。

```rust
pub struct ParticleSet {
    pub position: Vec<Vec3>,
    pub velocity: Vec<Vec3>,
    pub mass: Vec<f64>,
    // ドメイン固有の属性は列を追加 (密度、温度、電荷 …)
    pub extra_f64: BTreeMap<&'static str, Vec<f64>>,
}
```

### 6.1 近傍探索(空間ハッシュ)

- セル幅 = 相互作用半径(SPH のカーネル半径 $2h_{sph}$、分子の衝突判定半径)。
- 各粒子をセルにビニングし、27 近傍セルを走査。構築 $O(N)$、クエリ $O(N \cdot \bar{n})$。
- 決定論: セル内の粒子順序は粒子インデックス昇順に固定。ハッシュは座標整数化
  `(i,j,k) -> (i*73856093 ^ j*19349663 ^ k*83492791) mod table_size`(Teschner らの標準ハッシュ)だが、
  衝突時のバケット内順序もインデックス順にソートする。

```rust
pub struct SpatialHash {
    pub cell: f64,
    /// 再構築 (毎ステップ)。粒子インデックス順に安定。
    pub fn rebuild(&mut self, positions: &[Vec3]);
    /// p から半径 r 以内の粒子インデックスを昇順で返す
    pub fn query(&self, p: Vec3, r: f64, out: &mut Vec<u32>);
}
```

## 7. テスト

- 補間: 定数場・線形場の再現(トライリニアは線形場を厳密再現、$\epsilon_{abs}=10^{-12}$)。
- 微分: 多項式場($f=x^2$ 等)で二次収束($h$ 半減で誤差 1/4)。
- PCG: 既知解の Poisson 問題(製造解法 manufactured solution)で収束、SPD 性の乱数テスト。
- 空間ハッシュ: 総当たり結果と完全一致(乱数配置 10³ 粒子 × 決定シード)。
