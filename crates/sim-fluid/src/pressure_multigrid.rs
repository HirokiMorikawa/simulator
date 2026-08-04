//! 圧力ポアソンのマルチグリッド前処理(MGPCG)。
//! 設計: [docs/11-fluid/02-eulerian-grid.md] §4.4「難所: マルチグリッド前処理と不規則境界」・§10。
//!
//! 設計§4.4 は「マルチグリッド前処理は**性能ベンチ(64³ 4 ms 予算)が要求したときに**
//! 導入する(機能でなく性能の最適化)」と実装順序を定めていた。
//! `examples/grid_fluid3d_bench.rs` の実測(64³ で 795.7 ms/step)がその要求で、
//! **このモジュールがその応答**である。
//!
//! # 何を解いているか
//!
//! 対称正定値な離散ポアソン作用素
//!
//! $$ (L p)_c = \frac{1}{h^2} \sum_{f \in \partial c} a_f\,(p_c - p_{n(f)}) $$
//!
//! を、V サイクルで近似的に反転して PCG の前処理 $M^{-1}\approx L^{-1}$ に使う。
//! `grid_fluid3d.rs` が組む作用素は符号が逆($\nabla^2$、負定値)だが、
//! **CG は前処理の全体符号に依存しない**($M^{-1}\to cM^{-1}$ で反復列は不変)ので、
//! ここでは扱いやすい正定値の $L=-\nabla^2$ で一貫させる。
//!
//! 面係数 $a_f$ は:
//!
//! - 隣が `Fluid` / `Dirichlet`: $a_f=\tfrac12(w_c+w_n)$($w$ は流体体積率)
//! - 隣が `Solid`、または非周期の領域外: 面ごと落とす(Neumann)
//!
//! レベル0では $w\equiv1$ なので $a_f=1$ となり、`grid_fluid3d.rs` が従来使っていた
//! 7点ステンシルと**係数まで完全に一致する**(切り替えで物理が動かないことの担保)。
//!
//! # 粗格子化(設計§4.4 の規則をそのまま実装する)
//!
//! - **Dirichlet の支配性**: 8子に1つでも `Dirichlet` があれば粗セルも `Dirichlet`。
//! - **係数平均法**: 混在セルの粗格子係数は子セルの流体体積率で平均する
//!   ($w_C=\frac18\sum_{child} w_c$、`Solid` の子は $w=0$)。
//! - **分岐条件**: 8子が solid/fluid 混在の粗セルの割合が **30%** を超えたら、
//!   そのレベルの粗格子化を採用しない(トポロジからの静的判定なので決定論的)。
//! - **フォールバック**: 分岐条件を超えたレベル以下を打ち切り、直前のレベルを最粗として
//!   V サイクルを短縮する。**最悪時(1段も粗格子化できない)は、V サイクルが
//!   減衰 Jacobi の数回掃引に退化する** = Jacobi 前処理 PCG への退行。
//!   収束は遅くなるが、正しさと決定論は保たれる。
//!
//! 分母は「そのレベルで未知数を持ちうるセル」(= 全子が `Solid` ではない粗セル)を採る。
//! 全域が固体の広い領域を持つシーンで、混在率が薄まって規則が効かなくなるのを避けるため。
//!
//! # 縮約(正直な記録)
//!
//! - 平滑化は**減衰 Jacobi**($\omega=6/7$)。3D 7点ラプラシアンの $D^{-1}L$ の固有値は
//!   $(0,2]$ に入るので $\omega=6/7$ が高周波側をよく潰す(Bridson,
//!   *Fluid Simulation for Computer Graphics*, 2nd ed., §5.4)。Gauss–Seidel の方が
//!   1掃引あたりは強いが、Jacobi は掃引順序に依存しないぶん**対称性が自明**で、
//!   PCG が要求する $M=M^\top$ を構造的に保証できる。
//! - 制限は延長の転置 $R=\frac18 P^\top$(実装も転置として書いてある)。これも
//!   $M=M^\top$ のため。延長は三線形補間で、非流体の細セルへは書かない
//!   (McAdams et al. 2010 と同じ扱い)。粗セル側は判定しない——非流体の粗セルは
//!   $x=0$ のままなので寄与が無く、判定を省くと制限と延長で**全く同じマスク**に
//!   なるぶん転置関係が厳密に保たれる。
//! - 粗格子作用素は Galerkin 積($R L P$)ではなく**再離散化**。不規則境界で
//!   Galerkin 積の疎構造が 7点に収まらず、matrix-free の利点を失うため。
//! - 全 Neumann/周期では $L$ が特異(定数が零空間)なので、V サイクルの前後と
//!   各レベルの粗格子右辺・粗格子解から流体セル平均を引く。この射影 $\Pi$ は対称なので
//!   $\Pi V \Pi$ も対称に保たれる。

use std::cell::RefCell;

/// 圧力ポアソンにおけるセルの役割。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PressureCell {
    /// 未知数を持つセル。
    Fluid,
    /// 固体。未知数から外し、接する面は Neumann にする。
    Solid,
    /// 圧力 Dirichlet $p=0$(流路の流出層)。未知数から外す。
    Dirichlet,
}

/// 隣接添字テーブルの「領域外」。
const OUTSIDE: usize = usize::MAX;
/// 減衰 Jacobi の緩和係数(モジュール doc 参照)。
const OMEGA: f64 = 6.0 / 7.0;
/// V サイクルの前平滑化・後平滑化の掃引数(前後で同数 = 対称)。
const SMOOTHING_SWEEPS: usize = 2;
/// 設計§4.4 の分岐条件。
const MIXED_FRACTION_LIMIT: f64 = 0.30;
/// 最粗レベルでの掃引数。小さいレベルでは多めに回す(直接解法の代用)。
const BOTTOM_SWEEPS_SMALL: usize = 24;
const BOTTOM_SWEEPS_LARGE: usize = 4;
/// 「小さい」の境目(セル数)。
const SMALL_LEVEL_CELLS: usize = 4096;

/// 1本の軸について、細レベル添字 → 粗レベルの三線形補間(2点)。
#[derive(Clone, Copy)]
struct Interp {
    a: usize,
    wa: f64,
    b: usize,
    wb: f64,
}

/// 階層の1段。作用素の構造だけを持ち、作業配列は `Work` 側に分ける
/// (前処理適用は `&self` で呼べる必要があるため)。
struct Level {
    nx: usize,
    ny: usize,
    nz: usize,
    inv_h2: f64,
    kind: Vec<PressureCell>,
    /// 流体体積率(設計§4.4「係数平均法」)。レベル0では `Fluid`/`Dirichlet` が 1.0。
    weight: Vec<f64>,
    diag: Vec<f64>,
    inv_diag: Vec<f64>,
    xm: Vec<usize>,
    xp: Vec<usize>,
    ym: Vec<usize>,
    yp: Vec<usize>,
    zm: Vec<usize>,
    zp: Vec<usize>,
    /// 1段細かいレベルの添字で引く補間テーブル(最細レベルでは空)。
    fine_x: Vec<Interp>,
    fine_y: Vec<Interp>,
    fine_z: Vec<Interp>,
    fluid_count: usize,
    /// 全セルが `Fluid` かつ周期境界 = 定数係数の7点ステンシル。内点の分岐を落とす。
    uniform: bool,
}

/// レベルごとの作業配列。
struct Work {
    x: Vec<f64>,
    b: Vec<f64>,
    r: Vec<f64>,
}

impl Work {
    fn new(n: usize) -> Work {
        Work {
            x: vec![0.0; n],
            b: vec![0.0; n],
            r: vec![0.0; n],
        }
    }
}

fn axis_tables(n: usize, periodic: bool) -> (Vec<usize>, Vec<usize>) {
    let mut minus = vec![OUTSIDE; n];
    let mut plus = vec![OUTSIDE; n];
    for i in 0..n {
        minus[i] = if i > 0 {
            i - 1
        } else if periodic {
            n - 1
        } else {
            OUTSIDE
        };
        plus[i] = if i + 1 < n {
            i + 1
        } else if periodic {
            0
        } else {
            OUTSIDE
        };
    }
    (minus, plus)
}

/// 細レベルの添字 → 粗レベルの三線形補間の重み。セル中心配置の 2:1 粗格子化では
/// 「自分の親に 3/4、隣の親に 1/4」になる。非周期で隣の親が領域外なら、その重みを
/// 親へ畳む(定数を再現できるようにするため)。
fn interp_table(fine_n: usize, coarse_n: usize, periodic: bool) -> Vec<Interp> {
    (0..fine_n)
        .map(|i| {
            let a = i / 2;
            let other = if i % 2 == 0 {
                a as i64 - 1
            } else {
                a as i64 + 1
            };
            let b = if other < 0 || other >= coarse_n as i64 {
                if periodic {
                    other.rem_euclid(coarse_n as i64) as usize
                } else {
                    a
                }
            } else {
                other as usize
            };
            if b == a {
                Interp {
                    a,
                    wa: 1.0,
                    b: a,
                    wb: 0.0,
                }
            } else {
                Interp {
                    a,
                    wa: 0.75,
                    b,
                    wb: 0.25,
                }
            }
        })
        .collect()
}

impl Level {
    fn new(
        nx: usize,
        ny: usize,
        nz: usize,
        inv_h2: f64,
        kind: Vec<PressureCell>,
        weight: Vec<f64>,
        periodic: bool,
    ) -> Level {
        let n = nx * ny * nz;
        debug_assert_eq!(kind.len(), n);
        let (xm, xp) = axis_tables(nx, periodic);
        let (ym, yp) = axis_tables(ny, periodic);
        let (zm, zp) = axis_tables(nz, periodic);
        let fluid_count = kind.iter().filter(|k| **k == PressureCell::Fluid).count();
        // 全セルが流体・全重み 1・周期なら、対角は 6/h² で一定になり内点の分岐が要らない。
        // (粗格子化は流体だけの領域で重み 1 を保つので、この条件は粗レベルでも立つ。)
        let uniform = periodic && fluid_count == n && weight.iter().all(|w| *w == 1.0);
        let mut level = Level {
            nx,
            ny,
            nz,
            inv_h2,
            kind,
            weight,
            diag: vec![0.0; n],
            inv_diag: vec![0.0; n],
            xm,
            xp,
            ym,
            yp,
            zm,
            zp,
            fine_x: Vec::new(),
            fine_y: Vec::new(),
            fine_z: Vec::new(),
            fluid_count,
            uniform,
        };
        level.build_diagonal();
        level
    }

    fn cells(&self) -> usize {
        self.nx * self.ny * self.nz
    }

    /// 面 $c\!-\!m$ の係数。領域外・固体は 0(Neumann = 面を落とす)。
    #[inline(always)]
    fn face_coefficient(&self, wc: f64, m: usize) -> f64 {
        if m == OUTSIDE || self.kind[m] == PressureCell::Solid {
            0.0
        } else {
            self.inv_h2 * 0.5 * (wc + self.weight[m])
        }
    }

    /// 非対角項の寄与($x_m$ が既知の 0 である `Dirichlet` は寄与しない)。
    #[inline(always)]
    fn off_diagonal(&self, wc: f64, m: usize, x: &[f64]) -> f64 {
        if m == OUTSIDE || self.kind[m] != PressureCell::Fluid {
            0.0
        } else {
            self.inv_h2 * 0.5 * (wc + self.weight[m]) * x[m]
        }
    }

    fn build_diagonal(&mut self) {
        let plane = self.nx * self.ny;
        for k in 0..self.nz {
            let kbase = k * plane;
            for j in 0..self.ny {
                let row = kbase + j * self.nx;
                for i in 0..self.nx {
                    let c = row + i;
                    if self.kind[c] != PressureCell::Fluid {
                        continue;
                    }
                    let wc = self.weight[c];
                    let mut d = 0.0;
                    for m in self.neighbours(i, j, k, row, kbase) {
                        d += self.face_coefficient(wc, m);
                    }
                    self.diag[c] = d;
                    // 完全に固体で囲まれた孤立セルは対角が 0 になる。未知数から外す
                    // (右辺もそこでは 0 なので、解は 0 のままで整合する)。
                    self.inv_diag[c] = if d > 0.0 { 1.0 / d } else { 0.0 };
                }
            }
        }
    }

    #[inline(always)]
    fn neighbours(&self, i: usize, j: usize, k: usize, row: usize, kbase: usize) -> [usize; 6] {
        let plane = self.nx * self.ny;
        let ji = j * self.nx + i;
        [
            if self.xm[i] == OUTSIDE {
                OUTSIDE
            } else {
                row + self.xm[i]
            },
            if self.xp[i] == OUTSIDE {
                OUTSIDE
            } else {
                row + self.xp[i]
            },
            if self.ym[j] == OUTSIDE {
                OUTSIDE
            } else {
                kbase + self.ym[j] * self.nx + i
            },
            if self.yp[j] == OUTSIDE {
                OUTSIDE
            } else {
                kbase + self.yp[j] * self.nx + i
            },
            if self.zm[k] == OUTSIDE {
                OUTSIDE
            } else {
                self.zm[k] * plane + ji
            },
            if self.zp[k] == OUTSIDE {
                OUTSIDE
            } else {
                self.zp[k] * plane + ji
            },
        ]
    }

    #[inline(always)]
    fn cell_value(&self, i: usize, j: usize, k: usize, row: usize, kbase: usize, x: &[f64]) -> f64 {
        let c = row + i;
        let wc = self.weight[c];
        let mut acc = self.diag[c] * x[c];
        for m in self.neighbours(i, j, k, row, kbase) {
            acc -= self.off_diagonal(wc, m, x);
        }
        acc
    }

    /// $out = L x$。非流体セルには 0 を書く(未知数から外しているため)。
    fn apply(&self, x: &[f64], out: &mut [f64]) {
        let (nx, ny, nz) = (self.nx, self.ny, self.nz);
        let plane = nx * ny;
        // 定数係数の内点だけを分岐なしで回す(64³ 周期の圧力ソルバはここが全て)。
        let fast_interior = self.uniform && nx >= 3 && ny >= 3 && nz >= 3;
        let inv = self.inv_h2;
        let diag_uniform = 6.0 * inv;
        for k in 0..nz {
            let kbase = k * plane;
            let shell_k = k == 0 || k + 1 == nz;
            for j in 0..ny {
                let row = kbase + j * nx;
                let shell_jk = shell_k || j == 0 || j + 1 == ny;
                if !fast_interior || shell_jk {
                    for i in 0..nx {
                        out[row + i] = if self.kind[row + i] == PressureCell::Fluid {
                            self.cell_value(i, j, k, row, kbase, x)
                        } else {
                            0.0
                        };
                    }
                    continue;
                }
                out[row] = self.cell_value(0, j, k, row, kbase, x);
                for i in 1..nx - 1 {
                    let c = row + i;
                    out[c] = diag_uniform * x[c]
                        - inv
                            * (x[c - 1]
                                + x[c + 1]
                                + x[c - nx]
                                + x[c + nx]
                                + x[c - plane]
                                + x[c + plane]);
                }
                out[row + nx - 1] = self.cell_value(nx - 1, j, k, row, kbase, x);
            }
        }
    }

    /// 流体セルの平均を引く(特異系の可解性条件)。
    fn remove_mean(&self, v: &mut [f64]) {
        if self.fluid_count == 0 {
            return;
        }
        let mut sum = 0.0;
        for (c, value) in v.iter().enumerate() {
            if self.kind[c] == PressureCell::Fluid {
                sum += *value;
            }
        }
        let mean = sum / self.fluid_count as f64;
        for (c, value) in v.iter_mut().enumerate() {
            if self.kind[c] == PressureCell::Fluid {
                *value -= mean;
            }
        }
    }
}

/// 粗格子化の結果。`None` は設計§4.4 の分岐条件による打ち切り。
struct Coarsened {
    level: Level,
    mixed_fraction: f64,
}

/// `fine` を 2:1 で粗格子化する。分岐条件(混在30%超)に触れたら `Err(mixed_fraction)`。
fn coarsen(fine: &Level, periodic: bool) -> Result<Coarsened, f64> {
    let (nx, ny, nz) = (fine.nx / 2, fine.ny / 2, fine.nz / 2);
    let n = nx * ny * nz;
    let mut kind = vec![PressureCell::Solid; n];
    let mut weight = vec![0.0; n];
    let mut mixed = 0usize;
    let mut active = 0usize;

    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                let c = i + nx * (j + ny * k);
                let mut any_dirichlet = false;
                let mut fluid_children = 0usize;
                let mut solid_children = 0usize;
                let mut weight_sum = 0.0;
                for dk in 0..2 {
                    for dj in 0..2 {
                        for di in 0..2 {
                            let f =
                                (2 * i + di) + fine.nx * ((2 * j + dj) + fine.ny * (2 * k + dk));
                            match fine.kind[f] {
                                PressureCell::Dirichlet => any_dirichlet = true,
                                PressureCell::Solid => solid_children += 1,
                                PressureCell::Fluid => {
                                    fluid_children += 1;
                                    weight_sum += fine.weight[f];
                                }
                            }
                        }
                    }
                }
                if solid_children > 0 && fluid_children > 0 {
                    mixed += 1;
                }
                if any_dirichlet {
                    // Dirichlet の支配性: 子に1つでもあれば粗セルも Dirichlet。
                    kind[c] = PressureCell::Dirichlet;
                    weight[c] = 1.0;
                    active += 1;
                } else if fluid_children > 0 {
                    // 係数平均法: 子セルの流体体積率の平均。
                    kind[c] = PressureCell::Fluid;
                    weight[c] = weight_sum / 8.0;
                    active += 1;
                }
            }
        }
    }

    let mixed_fraction = if active == 0 {
        1.0
    } else {
        mixed as f64 / active as f64
    };
    if mixed_fraction > MIXED_FRACTION_LIMIT {
        return Err(mixed_fraction);
    }

    let mut level = Level::new(nx, ny, nz, fine.inv_h2 / 4.0, kind, weight, periodic);
    level.fine_x = interp_table(fine.nx, nx, periodic);
    level.fine_y = interp_table(fine.ny, ny, periodic);
    level.fine_z = interp_table(fine.nz, nz, periodic);
    Ok(Coarsened {
        level,
        mixed_fraction,
    })
}

/// 直近の粗格子化がどこで止まったかの記録(近似バッジ・診断用)。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HierarchyReport {
    /// 最細を含むレベル数。1 なら V サイクルは減衰 Jacobi 掃引に退化している。
    pub levels: usize,
    /// 設計§4.4 の分岐条件(混在30%超)で打ち切ったか。
    pub truncated_by_mixed_cells: bool,
    /// 打ち切りの直前に測った混在率(打ち切っていなければ最後に成功したレベルの値)。
    pub mixed_fraction: f64,
}

/// 圧力ポアソンのマルチグリッド V サイクル前処理。
pub struct MultigridPoisson {
    levels: Vec<Level>,
    work: RefCell<Vec<Work>>,
    /// Dirichlet セルが1つも無い = 作用素が特異(定数が零空間)。
    singular: bool,
    report: HierarchyReport,
}

impl MultigridPoisson {
    /// レベル0のセル種別から階層を組む。`h` は最細の格子幅、`periodic` は全軸周期か。
    pub fn build(
        nx: usize,
        ny: usize,
        nz: usize,
        h: f64,
        periodic: bool,
        kind: Vec<PressureCell>,
    ) -> MultigridPoisson {
        assert_eq!(kind.len(), nx * ny * nz, "セル種別の長さが格子と合わない");
        let weight = kind
            .iter()
            .map(|k| if *k == PressureCell::Solid { 0.0 } else { 1.0 })
            .collect();
        let singular = !kind.contains(&PressureCell::Dirichlet);
        let finest = Level::new(nx, ny, nz, 1.0 / (h * h), kind, weight, periodic);

        let mut levels = vec![finest];
        let mut truncated = false;
        let mut mixed_fraction = 0.0;
        loop {
            let fine = levels.last().expect("最細レベルは必ずある");
            let coarsenable = fine.nx.is_multiple_of(2)
                && fine.ny.is_multiple_of(2)
                && fine.nz.is_multiple_of(2)
                && fine.nx >= 4
                && fine.ny >= 4
                && fine.nz >= 4;
            if !coarsenable {
                break;
            }
            match coarsen(fine, periodic) {
                Ok(next) => {
                    mixed_fraction = next.mixed_fraction;
                    levels.push(next.level);
                }
                Err(fraction) => {
                    // 設計§4.4 のフォールバック: ここから下を捨て、直前を最粗にする。
                    mixed_fraction = fraction;
                    truncated = true;
                    break;
                }
            }
        }

        let work = levels.iter().map(|l| Work::new(l.cells())).collect();
        let report = HierarchyReport {
            levels: levels.len(),
            truncated_by_mixed_cells: truncated,
            mixed_fraction,
        };
        MultigridPoisson {
            levels,
            work: RefCell::new(work),
            singular,
            report,
        }
    }

    /// 階層の状態(段数・打ち切り)。
    pub fn report(&self) -> HierarchyReport {
        self.report
    }

    /// レベル0のセル種別。呼び出し側がキャッシュの有効性を判定するために使う。
    pub fn cell_kinds(&self) -> &[PressureCell] {
        &self.levels[0].kind
    }

    /// 作用素 $L=-\nabla^2$ をレベル0に適用する(PCG の matrix-free ステンシル)。
    pub fn apply_operator(&self, x: &[f64], out: &mut [f64]) {
        self.levels[0].apply(x, out);
    }

    /// レベル0の対角(Jacobi 前処理へ退行させたい場合に使える)。
    pub fn diagonal(&self) -> &[f64] {
        &self.levels[0].diag
    }

    /// 特異系(全 Neumann/周期)か。
    pub fn is_singular(&self) -> bool {
        self.singular
    }

    /// 前処理 $z = M^{-1} r$。1回の V サイクル。
    pub fn precondition(&self, r: &[f64], z: &mut [f64]) {
        let mut work = self.work.borrow_mut();
        work[0].b.copy_from_slice(r);
        work[0].x.fill(0.0);
        if self.singular {
            self.levels[0].remove_mean(&mut work[0].b);
        }
        self.vcycle(0, &mut work);
        z.copy_from_slice(&work[0].x);
        if self.singular {
            self.levels[0].remove_mean(z);
        }
    }

    /// `work[0]` が レベル `l` に対応する。`b` は設定済み、`x` は 0 で入ること。
    fn vcycle(&self, l: usize, work: &mut [Work]) {
        let level = &self.levels[l];
        if l + 1 == self.levels.len() {
            let sweeps = if level.cells() <= SMALL_LEVEL_CELLS {
                BOTTOM_SWEEPS_SMALL
            } else {
                BOTTOM_SWEEPS_LARGE
            };
            self.smooth(l, &mut work[0], sweeps, true);
            return;
        }

        self.smooth(l, &mut work[0], SMOOTHING_SWEEPS, true);
        level.apply(&work[0].x, &mut work[0].r);
        for c in 0..level.cells() {
            work[0].r[c] = work[0].b[c] - work[0].r[c];
        }

        let (head, tail) = work.split_at_mut(1);
        self.restrict(l + 1, &head[0].r, &mut tail[0].b);
        tail[0].x.fill(0.0);
        if self.singular {
            self.levels[l + 1].remove_mean(&mut tail[0].b);
        }
        self.vcycle(l + 1, tail);
        if self.singular {
            self.levels[l + 1].remove_mean(&mut tail[0].x);
        }
        self.prolong_add(l + 1, &tail[0].x, &mut head[0].x);

        self.smooth(l, &mut work[0], SMOOTHING_SWEEPS, false);
    }

    /// 減衰 Jacobi。`x_is_zero` なら初回掃引の作用素適用を省く。
    fn smooth(&self, l: usize, w: &mut Work, sweeps: usize, x_is_zero: bool) {
        let level = &self.levels[l];
        let n = level.cells();
        let mut zero = x_is_zero;
        for _ in 0..sweeps {
            if zero {
                for c in 0..n {
                    w.x[c] = OMEGA * level.inv_diag[c] * w.b[c];
                }
                zero = false;
            } else {
                level.apply(&w.x, &mut w.r);
                for c in 0..n {
                    w.x[c] += OMEGA * level.inv_diag[c] * (w.b[c] - w.r[c]);
                }
            }
        }
    }

    /// 細レベルの (k, j) 行に対応する、粗レベルの4本の行(先頭添字と重み)。
    /// 行ごとに1度だけ作れば、内側の i ループは x 方向の2点だけを見ればよくなる。
    #[inline]
    fn coarse_rows(&self, coarse_level: usize, k: usize, j: usize) -> [(usize, f64); 4] {
        let coarse = &self.levels[coarse_level];
        let iz = coarse.fine_z[k];
        let iy = coarse.fine_y[j];
        let plane = coarse.nx * coarse.ny;
        [
            (iz.a * plane + iy.a * coarse.nx, iz.wa * iy.wa),
            (iz.a * plane + iy.b * coarse.nx, iz.wa * iy.wb),
            (iz.b * plane + iy.a * coarse.nx, iz.wb * iy.wa),
            (iz.b * plane + iy.b * coarse.nx, iz.wb * iy.wb),
        ]
    }

    /// $b_{coarse} = \frac18 P^\top r_{fine}$。**延長の転置として書いてある**
    /// (対称性を構造で保証するため。重みの並びを2箇所に書かない)。
    ///
    /// 粗セル側の `Fluid` 判定は省いてある。非流体の粗セルは `inv_diag = 0` なので
    /// 平滑化が触らず、解 `x` は 0 のまま——右辺に値が入っても解には効かないし、
    /// 省くことで $R=\frac18P^\top$ という関係は**むしろ厳密に保たれる**
    /// (延長側でも同じ判定を省いているため)。
    /// **x方向を先に粗レベル1行ぶんの小さな作業配列へ畳んでから**、y/z の4行へ配る。
    /// 素朴に細セルごと8点を粗配列へ散らすと、離れた4行への read-modify-write が
    /// 8回走って転送段が V サイクルの支配項になる(実測 3 ms/回 @64³)。
    /// 畳んでおけば粗レベル側は連続アクセスになり、演算数も 8n から 4n へ落ちる。
    fn restrict(&self, coarse_level: usize, fine_r: &[f64], coarse_b: &mut [f64]) {
        let coarse = &self.levels[coarse_level];
        let fine = &self.levels[coarse_level - 1];
        coarse_b.fill(0.0);
        let mut row_acc = vec![0.0; coarse.nx];
        for k in 0..fine.nz {
            for j in 0..fine.ny {
                let row = k * fine.nx * fine.ny + j * fine.nx;
                row_acc.fill(0.0);
                let mut any = false;
                for i in 0..fine.nx {
                    let f = row + i;
                    if fine.kind[f] != PressureCell::Fluid {
                        continue;
                    }
                    let value = 0.125 * fine_r[f];
                    let ix = coarse.fine_x[i];
                    row_acc[ix.a] += value * ix.wa;
                    row_acc[ix.b] += value * ix.wb;
                    any = true;
                }
                if !any {
                    continue;
                }
                for (base, wyz) in self.coarse_rows(coarse_level, k, j) {
                    for (ic, acc) in row_acc.iter().enumerate() {
                        coarse_b[base + ic] += wyz * acc;
                    }
                }
            }
        }
    }

    /// $x_{fine} \mathrel{+}= P\,x_{coarse}$(三線形補間)。[`MultigridPoisson::restrict`]
    /// と鏡像の構成——y/z の4行を先に1行へ畳んでから x 方向を補間する。
    fn prolong_add(&self, coarse_level: usize, coarse_x: &[f64], fine_x: &mut [f64]) {
        let coarse = &self.levels[coarse_level];
        let fine = &self.levels[coarse_level - 1];
        let mut row_val = vec![0.0; coarse.nx];
        for k in 0..fine.nz {
            for j in 0..fine.ny {
                row_val.fill(0.0);
                for (base, wyz) in self.coarse_rows(coarse_level, k, j) {
                    for (ic, value) in row_val.iter_mut().enumerate() {
                        *value += wyz * coarse_x[base + ic];
                    }
                }
                let row = k * fine.nx * fine.ny + j * fine.nx;
                for i in 0..fine.nx {
                    let f = row + i;
                    if fine.kind[f] != PressureCell::Fluid {
                        continue;
                    }
                    let ix = coarse.fine_x[i];
                    fine_x[f] += ix.wa * row_val[ix.a] + ix.wb * row_val[ix.b];
                }
            }
        }
    }
}
