//! 格子(Eulerian)流体ソルバ、**3D**。設計: docs/11-fluid/02-eulerian-grid.md。
//! **群9で追加**——設計 §3 の `GridFluid` は最初から3Dだが、実装は長らく2D
//! (`grid_fluid.rs`)だけだった。
//!
//! 構成は2D版で確立したものをそのまま3次元へ持ち上げたもの:
//! MAC(staggered)格子・semi-Lagrangian RK2 移流(§4.1)・陽的粘性拡散(§4.3)・
//! matrix-free PCG による圧力投影(§4.4)・セル種別による固体境界(§3/§4.4)・
//! 渦度強化(§4.5)。加えて**受動スカラー `smoke_density`**(§3)を持つ
//! ——3Dにする実用上の意味は「煙が見えること」なので、そこは省略しない。
//!
//! **2Dとの本質的な違いは渦度**: 2Dの $\omega_z$ はスカラー(退化形)だったが、
//! 3Dでは $\boldsymbol\omega=\nabla\times\mathbf u$ が本来のベクトルになり、
//! 設計§4.5 の $\mathbf f_{conf}=\varepsilon h(\mathbf N\times\boldsymbol\omega)$ を
//! そのままの形で書ける。
//!
//! **縮約(正直な記録)**:
//!
//! - `sim_math::Grid3<T>` は使わず、2D版と同じ**平坦な `Vec<f64>` + 周期ラップ添字**で
//!   持つ。`Grid3` は符号なし添字のみで周期ラップを表現できず、既に検証済みの2D版の
//!   離散化パターンから離れる方が危険と判断した(2Dとの交差検証テストが効くのは
//!   両者の離散化が同型だからである)。
//! - `CellType` に `Empty`(自由表面)は無い(2D版と同じ理由。自由表面は SPH 側)。
//! - **前処理なしのPCG**。設計§4.4 が「マルチグリッド前処理は性能ベンチが要求したときに
//!   導入する(機能でなく性能の最適化)」と定めており、実測は§10の 64³/4ms 予算に
//!   全く届かない(64³ で 795.7 ms/step、約200倍の超過。
//!   `examples/grid_fluid3d_bench.rs` に実測を記録)。近似バッジ「前処理なしPCG」で
//!   常時申告し、黙って予算未達にしない。
//! - 壁は自由すべり(no-slip 境界層は作れない)。2D版と同じ。

use crate::grid_fluid::CellType;
use sim_core::{Approximation, EnergyBreakdown, Solver, SolverContext, StateHasher};
use sim_math::Vec3;

/// 境界条件(2D版の `GridBoundary` と同じ意味を3Dへ持ち上げたもの)。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GridBoundary3D {
    /// 全方向周期。Taylor-Green 等の検証はこれ。
    Periodic,
    /// 流路: x−から流入・x+へ流出、y/z方向は自由すべり壁。
    Channel { inflow_speed: f64 },
}

/// 3D格子流体。`u`/`v`/`w` は共に長さ `nx*ny*nz`(staggered配置、モジュールdoc参照)。
/// `u[i,j,k]` は x面 $(ih,(j+\frac12)h,(k+\frac12)h)$ に、`v` は y面、`w` は z面に置く。
#[derive(Clone)]
pub struct GridFluid3D {
    pub nx: usize,
    pub ny: usize,
    pub nz: usize,
    pub h: f64,
    pub u: Vec<f64>,
    pub v: Vec<f64>,
    pub w: Vec<f64>,
    /// 受動スカラー(煙・染料、設計§3 `smoke_density`)。移流のみを受ける。
    pub smoke_density: Vec<f64>,
    pub density: f64,
    /// 0.0 なら陽的粘性拡散の段をスキップする(2D版と同じ既定分岐)。
    pub kinematic_viscosity: f64,
    /// 渦度強化 $\varepsilon_{conf}$(設計§4.5)。**既定 0.0 = 検証モード**。
    pub vorticity_confinement_epsilon: f64,
    boundary: GridBoundary3D,
    cell_type: Vec<CellType>,
    solid_velocity: Vec<Vec3>,
    /// 直近の`step`が投影した圧力場(派生キャッシュ、`state_hash`には含めない)。
    pub last_pressure: Vec<f64>,
}

fn wrap(i: i64, n: usize) -> usize {
    i.rem_euclid(n as i64) as usize
}

impl GridFluid3D {
    pub fn new(nx: usize, ny: usize, nz: usize, h: f64) -> GridFluid3D {
        let n = nx * ny * nz;
        GridFluid3D {
            nx,
            ny,
            nz,
            h,
            u: vec![0.0; n],
            v: vec![0.0; n],
            w: vec![0.0; n],
            smoke_density: vec![0.0; n],
            density: 1.0,
            kinematic_viscosity: 0.0,
            vorticity_confinement_epsilon: 0.0,
            boundary: GridBoundary3D::Periodic,
            cell_type: vec![CellType::Fluid; n],
            solid_velocity: vec![Vec3::ZERO; n],
            last_pressure: vec![0.0; n],
        }
    }

    /// 境界条件を差し替える(2D版の `with_boundary` と同じ。`Channel` にすると
    /// 流入速度で場を初期化する——静止流体が流入してくる余計な過渡を避けるため)。
    pub fn with_boundary(mut self, boundary: GridBoundary3D) -> GridFluid3D {
        self.boundary = boundary;
        if let GridBoundary3D::Channel { inflow_speed } = boundary {
            self.u.iter_mut().for_each(|x| *x = inflow_speed);
            self.v.iter_mut().for_each(|x| *x = 0.0);
            self.w.iter_mut().for_each(|x| *x = 0.0);
        }
        self
    }

    pub fn boundary(&self) -> GridBoundary3D {
        self.boundary
    }

    /// 任意形状の固体境界(2D版の `set_solid_cells` と同じ。判定はセル中心)。
    pub fn set_solid_cells(&mut self, f: impl Fn(f64, f64, f64) -> Option<Vec3>) {
        for k in 0..self.nz {
            for j in 0..self.ny {
                for i in 0..self.nx {
                    let x = (i as f64 + 0.5) * self.h;
                    let y = (j as f64 + 0.5) * self.h;
                    let z = (k as f64 + 0.5) * self.h;
                    let idx = self.flat(i, j, k);
                    match f(x, y, z) {
                        Some(velocity) => {
                            self.cell_type[idx] = CellType::Solid;
                            self.solid_velocity[idx] = velocity;
                        }
                        None => {
                            self.cell_type[idx] = CellType::Fluid;
                            self.solid_velocity[idx] = Vec3::ZERO;
                        }
                    }
                }
            }
        }
    }

    fn flat(&self, i: usize, j: usize, k: usize) -> usize {
        i + self.nx * (j + self.ny * k)
    }

    pub fn idx(&self, i: i64, j: i64, k: i64) -> usize {
        wrap(i, self.nx) + self.nx * (wrap(j, self.ny) + self.ny * wrap(k, self.nz))
    }

    pub fn cell_type_at(&self, i: i64, j: i64, k: i64) -> CellType {
        self.cell_type[self.idx(i, j, k)]
    }

    fn is_solid(&self, i: i64, j: i64, k: i64) -> bool {
        self.cell_type_at(i, j, k) == CellType::Solid
    }

    fn has_solid(&self) -> bool {
        self.cell_type.contains(&CellType::Solid)
    }

    pub fn u_at(&self, i: i64, j: i64, k: i64) -> f64 {
        self.u[self.idx(i, j, k)]
    }
    pub fn v_at(&self, i: i64, j: i64, k: i64) -> f64 {
        self.v[self.idx(i, j, k)]
    }
    pub fn w_at(&self, i: i64, j: i64, k: i64) -> f64 {
        self.w[self.idx(i, j, k)]
    }

    /// x面の $u$(境界条件を考慮)。`i` は面index。`Channel` では `i<=0` が流入面
    /// (固定値)、`i>=nx` は流出(勾配ゼロ = 最終層の複製)。
    fn u_face(&self, i: i64, j: i64, k: i64) -> f64 {
        match self.boundary {
            GridBoundary3D::Periodic => self.u_at(i, j, k),
            GridBoundary3D::Channel { inflow_speed } => {
                let jj = j.clamp(0, self.ny as i64 - 1);
                let kk = k.clamp(0, self.nz as i64 - 1);
                if i <= 0 {
                    inflow_speed
                } else if i >= self.nx as i64 {
                    self.u[self.idx(self.nx as i64 - 1, jj, kk)]
                } else {
                    self.u[self.idx(i, jj, kk)]
                }
            }
        }
    }

    fn v_face(&self, i: i64, j: i64, k: i64) -> f64 {
        match self.boundary {
            GridBoundary3D::Periodic => self.v_at(i, j, k),
            GridBoundary3D::Channel { .. } => {
                if j <= 0 || j >= self.ny as i64 {
                    0.0 // y壁(自由すべり)
                } else {
                    let ii = i.clamp(0, self.nx as i64 - 1);
                    let kk = k.clamp(0, self.nz as i64 - 1);
                    self.v[self.idx(ii, j, kk)]
                }
            }
        }
    }

    fn w_face(&self, i: i64, j: i64, k: i64) -> f64 {
        match self.boundary {
            GridBoundary3D::Periodic => self.w_at(i, j, k),
            GridBoundary3D::Channel { .. } => {
                if k <= 0 || k >= self.nz as i64 {
                    0.0 // z壁(自由すべり)
                } else {
                    let ii = i.clamp(0, self.nx as i64 - 1);
                    let jj = j.clamp(0, self.ny as i64 - 1);
                    self.w[self.idx(ii, jj, k)]
                }
            }
        }
    }

    /// セル(i,j,k)の発散(MAC格子の標準式、設計§4.4)。
    pub fn divergence(&self, i: i64, j: i64, k: i64) -> f64 {
        (self.u_face(i + 1, j, k) - self.u_face(i, j, k)) / self.h
            + (self.v_face(i, j + 1, k) - self.v_face(i, j, k)) / self.h
            + (self.w_face(i, j, k + 1) - self.w_face(i, j, k)) / self.h
    }

    /// 三線形補間。`get` はサンプル点の格子添字から値を引く(境界条件を知っている
    /// 面アクセサ、または周期ラップ)。
    fn trilinear(&self, offset: Vec3, pos: Vec3, get: impl Fn(i64, i64, i64) -> f64) -> f64 {
        let local = Vec3::new(
            (pos.x - offset.x) / self.h,
            (pos.y - offset.y) / self.h,
            (pos.z - offset.z) / self.h,
        );
        let (i0f, j0f, k0f) = (local.x.floor(), local.y.floor(), local.z.floor());
        let (fx, fy, fz) = (local.x - i0f, local.y - j0f, local.z - k0f);
        let (i0, j0, k0) = (i0f as i64, j0f as i64, k0f as i64);
        let mut acc = 0.0;
        for (dk, wk) in [(0, 1.0 - fz), (1, fz)] {
            for (dj, wj) in [(0, 1.0 - fy), (1, fy)] {
                for (di, wi) in [(0, 1.0 - fx), (1, fx)] {
                    acc += wi * wj * wk * get(i0 + di, j0 + dj, k0 + dk);
                }
            }
        }
        acc
    }

    fn sample_u(&self, pos: Vec3) -> f64 {
        let offset = Vec3::new(0.0, 0.5 * self.h, 0.5 * self.h);
        self.trilinear(offset, pos, |i, j, k| self.u_face(i, j, k))
    }
    fn sample_v(&self, pos: Vec3) -> f64 {
        let offset = Vec3::new(0.5 * self.h, 0.0, 0.5 * self.h);
        self.trilinear(offset, pos, |i, j, k| self.v_face(i, j, k))
    }
    fn sample_w(&self, pos: Vec3) -> f64 {
        let offset = Vec3::new(0.5 * self.h, 0.5 * self.h, 0.0);
        self.trilinear(offset, pos, |i, j, k| self.w_face(i, j, k))
    }
    fn sample_smoke(&self, pos: Vec3) -> f64 {
        let offset = Vec3::new(0.5 * self.h, 0.5 * self.h, 0.5 * self.h);
        self.trilinear(offset, pos, |i, j, k| self.smoke_density[self.idx(i, j, k)])
    }

    fn velocity_at(&self, pos: Vec3) -> Vec3 {
        Vec3::new(self.sample_u(pos), self.sample_v(pos), self.sample_w(pos))
    }

    /// 出発点をRK2(中点法)で逆追跡する(設計§4.1)。
    fn backtrace(&self, pos: Vec3, dt: f64) -> Vec3 {
        let vel = self.velocity_at(pos);
        let mid = pos - vel.scale(0.5 * dt);
        let vel_mid = self.velocity_at(mid);
        pos - vel_mid.scale(dt)
    }

    /// semi-Lagrangian移流(速度、設計§4.1)。
    pub fn advect_velocity(&mut self, dt: f64) {
        let old = GridFluid3D {
            u: self.u.clone(),
            v: self.v.clone(),
            w: self.w.clone(),
            ..self.clone()
        };
        let h = self.h;
        for k in 0..self.nz {
            for j in 0..self.ny {
                for i in 0..self.nx {
                    let idx = self.flat(i, j, k);
                    let pu = Vec3::new(i as f64 * h, (j as f64 + 0.5) * h, (k as f64 + 0.5) * h);
                    self.u[idx] = old.sample_u(old.backtrace(pu, dt));
                    let pv = Vec3::new((i as f64 + 0.5) * h, j as f64 * h, (k as f64 + 0.5) * h);
                    self.v[idx] = old.sample_v(old.backtrace(pv, dt));
                    let pw = Vec3::new((i as f64 + 0.5) * h, (j as f64 + 0.5) * h, k as f64 * h);
                    self.w[idx] = old.sample_w(old.backtrace(pw, dt));
                }
            }
        }
    }

    /// semi-Lagrangian移流(受動スカラー = 煙、設計§4.1「スカラーも同じ移流を使う」)。
    /// 速度場は移流**前**のものを使う(operator splitting の同一段内なので、
    /// 速度と煙は同じ $\mathbf u^n$ で運ばれる)。
    pub fn advect_smoke(&mut self, dt: f64) {
        let old_smoke = self.smoke_density.clone();
        let sampler = GridFluid3D {
            smoke_density: old_smoke,
            ..self.clone()
        };
        let h = self.h;
        for k in 0..self.nz {
            for j in 0..self.ny {
                for i in 0..self.nx {
                    let idx = self.flat(i, j, k);
                    let p = Vec3::new(
                        (i as f64 + 0.5) * h,
                        (j as f64 + 0.5) * h,
                        (k as f64 + 0.5) * h,
                    );
                    self.smoke_density[idx] = sampler.sample_smoke(sampler.backtrace(p, dt));
                }
            }
        }
    }

    /// 陽的粘性拡散(7点ラプラシアン、設計§4.3)。
    pub fn diffuse_explicit(&mut self, dt: f64, kinematic_viscosity: f64) {
        let coeff = kinematic_viscosity * dt / (self.h * self.h);
        let (old_u, old_v, old_w) = (self.u.clone(), self.v.clone(), self.w.clone());
        for k in 0..self.nz as i64 {
            for j in 0..self.ny as i64 {
                for i in 0..self.nx as i64 {
                    let idx = self.idx(i, j, k);
                    // 隣接indexを先に取り出してから閉包を作る(`self.idx` を閉包内で
                    // 呼ぶと `self.u` への可変借用と衝突する)。
                    let neighbours = [
                        self.idx(i + 1, j, k),
                        self.idx(i - 1, j, k),
                        self.idx(i, j + 1, k),
                        self.idx(i, j - 1, k),
                        self.idx(i, j, k + 1),
                        self.idx(i, j, k - 1),
                    ];
                    let lap = |field: &[f64]| -> f64 {
                        neighbours.iter().map(|&m| field[m]).sum::<f64>() - 6.0 * field[idx]
                    };
                    self.u[idx] += coeff * lap(&old_u);
                    self.v[idx] += coeff * lap(&old_v);
                    self.w[idx] += coeff * lap(&old_w);
                }
            }
        }
    }

    /// セル中心の渦度ベクトル $\boldsymbol\omega=\nabla\times\mathbf u$(設計§4.5)。
    /// **2Dのスカラー $\omega_z$ の一般形**——3Dにして初めて本来の形になる。
    pub fn vorticity_at(&self, i: i64, j: i64, k: i64) -> Vec3 {
        let uc = |a: i64, b: i64, c: i64| 0.5 * (self.u_face(a, b, c) + self.u_face(a + 1, b, c));
        let vc = |a: i64, b: i64, c: i64| 0.5 * (self.v_face(a, b, c) + self.v_face(a, b + 1, c));
        let wc = |a: i64, b: i64, c: i64| 0.5 * (self.w_face(a, b, c) + self.w_face(a, b, c + 1));
        let d = 2.0 * self.h;
        let dwdy = (wc(i, j + 1, k) - wc(i, j - 1, k)) / d;
        let dvdz = (vc(i, j, k + 1) - vc(i, j, k - 1)) / d;
        let dudz = (uc(i, j, k + 1) - uc(i, j, k - 1)) / d;
        let dwdx = (wc(i + 1, j, k) - wc(i - 1, j, k)) / d;
        let dvdx = (vc(i + 1, j, k) - vc(i - 1, j, k)) / d;
        let dudy = (uc(i, j + 1, k) - uc(i, j - 1, k)) / d;
        Vec3::new(dwdy - dvdz, dudz - dwdx, dvdx - dudy)
    }

    /// 渦度強化(設計§4.5、Fedkiw et al. 2001)。
    /// $\mathbf N=\nabla|\boldsymbol\omega|/|\nabla|\boldsymbol\omega||$、
    /// $\mathbf f_{conf}=\varepsilon h(\mathbf N\times\boldsymbol\omega)$。
    /// **非物理的な補償項**なので `epsilon > 0` のとき近似バッジを申告する。
    pub fn apply_vorticity_confinement(&mut self, dt: f64, epsilon: f64) {
        if epsilon == 0.0 {
            return;
        }
        let n = self.nx * self.ny * self.nz;
        let mut abs_omega = vec![0.0; n];
        for k in 0..self.nz as i64 {
            for j in 0..self.ny as i64 {
                for i in 0..self.nx as i64 {
                    abs_omega[self.idx(i, j, k)] = self.vorticity_at(i, j, k).length();
                }
            }
        }
        let (mut du, mut dv, mut dw) = (vec![0.0; n], vec![0.0; n], vec![0.0; n]);
        for k in 0..self.nz as i64 {
            for j in 0..self.ny as i64 {
                for i in 0..self.nx as i64 {
                    if self.is_solid(i, j, k) {
                        continue;
                    }
                    let d = 2.0 * self.h;
                    let gradient = Vec3::new(
                        (abs_omega[self.idx(i + 1, j, k)] - abs_omega[self.idx(i - 1, j, k)]) / d,
                        (abs_omega[self.idx(i, j + 1, k)] - abs_omega[self.idx(i, j - 1, k)]) / d,
                        (abs_omega[self.idx(i, j, k + 1)] - abs_omega[self.idx(i, j, k - 1)]) / d,
                    );
                    let magnitude = gradient.length();
                    if magnitude < 1e-12 {
                        continue; // 勾配ゼロでは方向が定まらない
                    }
                    let normal = gradient.scale(1.0 / magnitude);
                    let force = normal
                        .cross(self.vorticity_at(i, j, k))
                        .scale(epsilon * self.h);
                    // セル中心の力を、そのセルを挟む2面へ半分ずつ配る。
                    du[self.idx(i, j, k)] += 0.5 * force.x * dt;
                    du[self.idx(i + 1, j, k)] += 0.5 * force.x * dt;
                    dv[self.idx(i, j, k)] += 0.5 * force.y * dt;
                    dv[self.idx(i, j + 1, k)] += 0.5 * force.y * dt;
                    dw[self.idx(i, j, k)] += 0.5 * force.z * dt;
                    dw[self.idx(i, j, k + 1)] += 0.5 * force.z * dt;
                }
            }
        }
        for idx in 0..n {
            self.u[idx] += du[idx];
            self.v[idx] += dv[idx];
            self.w[idx] += dw[idx];
        }
    }

    /// Solid セルに触れる面の法線速度を固体の速度に一致させる(設計§4.4)。
    fn enforce_solid_faces(&mut self) {
        for k in 0..self.nz as i64 {
            for j in 0..self.ny as i64 {
                for i in 0..self.nx as i64 {
                    let idx = self.idx(i, j, k);
                    let here = self.is_solid(i, j, k);
                    if here || self.is_solid(i - 1, j, k) {
                        let src = if here { idx } else { self.idx(i - 1, j, k) };
                        self.u[idx] = self.solid_velocity[src].x;
                    }
                    if here || self.is_solid(i, j - 1, k) {
                        let src = if here { idx } else { self.idx(i, j - 1, k) };
                        self.v[idx] = self.solid_velocity[src].y;
                    }
                    if here || self.is_solid(i, j, k - 1) {
                        let src = if here { idx } else { self.idx(i, j, k - 1) };
                        self.w[idx] = self.solid_velocity[src].z;
                    }
                }
            }
        }
    }

    /// 固体表面の圧力積分による流体力(設計§6)。固体セルが無ければ `None`。
    /// $\mathbf F=-\sum_{\text{Solid–Fluid 面}} p\,\hat n\,h^2$($\hat n$ は固体から外向き)。
    pub fn pressure_force_on_solid(&self) -> Option<Vec3> {
        if !self.has_solid() {
            return None;
        }
        let area = self.h * self.h;
        let mut force = Vec3::ZERO;
        for k in 0..self.nz as i64 {
            for j in 0..self.ny as i64 {
                for i in 0..self.nx as i64 {
                    let here = self.is_solid(i, j, k);
                    let mut face = |lower_solid: bool, lower: usize, axis: usize| {
                        if here == lower_solid {
                            return; // 固体表面ではない(両側とも固体 or 両側とも流体)
                        }
                        // 固体側が「自分」なら外向き法線は負方向、そうでなければ正方向。
                        let (p, sign) = if here {
                            (self.last_pressure[lower], 1.0)
                        } else {
                            (self.last_pressure[self.idx(i, j, k)], -1.0)
                        };
                        let contribution = sign * p * area;
                        match axis {
                            0 => force.x += contribution,
                            1 => force.y += contribution,
                            _ => force.z += contribution,
                        }
                    };
                    face(self.is_solid(i - 1, j, k), self.idx(i - 1, j, k), 0);
                    face(self.is_solid(i, j - 1, k), self.idx(i, j - 1, k), 1);
                    face(self.is_solid(i, j, k - 1), self.idx(i, j, k - 1), 2);
                }
            }
        }
        Some(force)
    }

    /// 圧力投影(設計§4.4)。2D版と同一の構成: Solidセルは恒等行で未知数から外し、
    /// Solid面・壁はNeumann、`Channel` の流出層は圧力Dirichlet $p=0$。
    /// 周期境界では特異なので右辺の(流体セルのみの)平均を引く。
    pub fn project(&mut self, dt: f64, density: f64) -> Vec<f64> {
        let (nx, ny, nz) = (self.nx, self.ny, self.nz);
        let n = nx * ny * nz;
        let solid_cell: Vec<bool> = (0..n)
            .map(|m| self.cell_type[m] == CellType::Solid)
            .collect();

        let mut rhs = vec![0.0; n];
        for k in 0..nz as i64 {
            for j in 0..ny as i64 {
                for i in 0..nx as i64 {
                    let idx = self.idx(i, j, k);
                    if solid_cell[idx] {
                        continue; // 恒等行
                    }
                    rhs[idx] = density / dt * self.divergence(i, j, k);
                }
            }
        }
        if let GridBoundary3D::Channel { .. } = self.boundary {
            for k in 0..nz as i64 {
                for j in 0..ny as i64 {
                    rhs[self.idx(nx as i64 - 1, j, k)] = 0.0;
                }
            }
        }
        if self.boundary == GridBoundary3D::Periodic {
            // 全Neumann/周期では特異なので可解性条件を満たす(平均は流体セルのみで取る)。
            let fluid_count = solid_cell.iter().filter(|s| !**s).count();
            if fluid_count > 0 {
                let mean: f64 = rhs
                    .iter()
                    .zip(solid_cell.iter())
                    .filter(|(_, s)| !**s)
                    .map(|(r, _)| *r)
                    .sum::<f64>()
                    / fluid_count as f64;
                for (r, s) in rhs.iter_mut().zip(solid_cell.iter()) {
                    if !*s {
                        *r -= mean;
                    }
                }
            }
        }

        let h2 = self.h * self.h;
        let boundary = self.boundary;
        let flat = |i: i64, j: i64, k: i64| -> usize {
            wrap(i, nx) + nx * (wrap(j, ny) + ny * wrap(k, nz))
        };
        let apply_a = |x: &[f64], out: &mut [f64]| {
            for k in 0..nz as i64 {
                for j in 0..ny as i64 {
                    for i in 0..nx as i64 {
                        let idx = flat(i, j, k);
                        if solid_cell[idx] {
                            out[idx] = x[idx];
                            continue;
                        }
                        if let GridBoundary3D::Channel { .. } = boundary {
                            if i == nx as i64 - 1 {
                                out[idx] = x[idx]; // 流出層は圧力Dirichlet p=0
                                continue;
                            }
                        }
                        let mut sum = 0.0;
                        let mut diag = 0.0;
                        let mut neighbour = |a: i64, b: i64, c: i64| {
                            if boundary != GridBoundary3D::Periodic
                                && (a < 0
                                    || a >= nx as i64
                                    || b < 0
                                    || b >= ny as i64
                                    || c < 0
                                    || c >= nz as i64)
                            {
                                return; // 壁・流入面はNeumann(鏡像で相殺)
                            }
                            let m = flat(a, b, c);
                            if solid_cell[m] {
                                return; // Solid面もNeumann
                            }
                            sum += x[m];
                            diag += 1.0;
                        };
                        neighbour(i + 1, j, k);
                        neighbour(i - 1, j, k);
                        neighbour(i, j + 1, k);
                        neighbour(i, j - 1, k);
                        neighbour(i, j, k + 1);
                        neighbour(i, j, k - 1);
                        out[idx] = (sum - diag * x[idx]) / h2;
                    }
                }
            }
        };

        let mut pressure = vec![0.0; n];
        let result = sim_math::pcg(
            apply_a,
            &rhs,
            &mut pressure,
            &sim_math::Preconditioner::None,
            1e-8,
            2000,
        );
        debug_assert!(
            result.converged,
            "3D pressure projection PCG did not converge: {result:?}"
        );

        let scale = dt / density;
        // Solid に接する面は補正しない(そこは圧力勾配ではなく固体速度が決める)。
        let free = |lower: usize, upper: usize| !solid_cell[lower] && !solid_cell[upper];
        let is_channel = matches!(boundary, GridBoundary3D::Channel { .. });
        for k in 0..nz as i64 {
            for j in 0..ny as i64 {
                for i in 0..nx as i64 {
                    let idx = flat(i, j, k);
                    if (!is_channel || i > 0) && free(flat(i - 1, j, k), idx) {
                        let dpdx = (pressure[idx] - pressure[flat(i - 1, j, k)]) / self.h;
                        self.u[idx] -= scale * dpdx;
                    }
                    if (!is_channel || j > 0) && free(flat(i, j - 1, k), idx) {
                        let dpdy = (pressure[idx] - pressure[flat(i, j - 1, k)]) / self.h;
                        self.v[idx] -= scale * dpdy;
                    }
                    if (!is_channel || k > 0) && free(flat(i, j, k - 1), idx) {
                        let dpdz = (pressure[idx] - pressure[flat(i, j, k - 1)]) / self.h;
                        self.w[idx] -= scale * dpdz;
                    }
                }
            }
        }
        // 流入面と壁を境界値へ戻す(投影が触っていないので実質的な再確認、2D版と同じ)。
        if let GridBoundary3D::Channel { inflow_speed } = self.boundary {
            for k in 0..nz as i64 {
                for j in 0..ny as i64 {
                    self.u[flat(0, j, k)] = inflow_speed;
                }
            }
            for k in 0..nz as i64 {
                for i in 0..nx as i64 {
                    self.v[flat(i, 0, k)] = 0.0;
                }
            }
            for j in 0..ny as i64 {
                for i in 0..nx as i64 {
                    self.w[flat(i, j, 0)] = 0.0;
                }
            }
        }

        pressure
    }

    fn max_speed(&self) -> f64 {
        let mut max_sq: f64 = 0.0;
        for idx in 0..self.u.len() {
            let s =
                self.u[idx] * self.u[idx] + self.v[idx] * self.v[idx] + self.w[idx] * self.w[idx];
            max_sq = max_sq.max(s);
        }
        max_sq.sqrt()
    }

    /// 1ステップ。設計§4.6のステップ順
    /// **移流(速度・煙)→ 外力(渦度強化)→ 粘性 → 境界条件 → 投影**。
    pub fn step(&mut self, dt: f64) {
        self.advect_smoke(dt); // 速度を書き換える前の u^n で運ぶ
        self.advect_velocity(dt);
        self.apply_vorticity_confinement(dt, self.vorticity_confinement_epsilon);
        if self.kinematic_viscosity > 0.0 {
            self.diffuse_explicit(dt, self.kinematic_viscosity);
        }
        self.enforce_solid_faces();
        self.last_pressure = self.project(dt, self.density);
        self.enforce_solid_faces();
    }
}

impl Solver for GridFluid3D {
    /// 2D版と同じ規約: 陽的粘性の安定限界(3Dは $\nu\Delta t/h^2\le 1/6$)と
    /// 移流のCFL規約(≦5)の厳しい方。
    fn max_stable_dt(&self) -> f64 {
        const ADVECTION_CFL: f64 = 5.0;
        let speed = self.max_speed();
        let dt_adv = if speed > 0.0 {
            ADVECTION_CFL * self.h / speed
        } else {
            f64::INFINITY
        };
        let dt_visc = if self.kinematic_viscosity > 0.0 {
            self.h * self.h / (6.0 * self.kinematic_viscosity)
        } else {
            f64::INFINITY
        };
        dt_adv.min(dt_visc)
    }

    fn step(&mut self, dt: f64, _ctx: &mut SolverContext) {
        // inherent メソッドが優先されるので無限再帰しない(2D版と同じパターン)。
        self.step(dt);
    }

    /// 運動エネルギーのみ(2D版と同じ理由: 非圧縮流は圧力ポテンシャルを持たず、
    /// 外力由来のポテンシャルはこの縮約実装が外力自体を扱わないため対象外)。
    fn total_energy(&self) -> EnergyBreakdown {
        let cell_mass = self.density * self.h * self.h * self.h;
        let mut kinetic = 0.0;
        for idx in 0..self.u.len() {
            kinetic += 0.5
                * cell_mass
                * (self.u[idx] * self.u[idx]
                    + self.v[idx] * self.v[idx]
                    + self.w[idx] * self.w[idx]);
        }
        EnergyBreakdown {
            kinetic,
            ..Default::default()
        }
    }

    fn state_hash(&self, hasher: &mut StateHasher) {
        hasher.write_u64(self.u.len() as u64);
        for idx in 0..self.u.len() {
            hasher.write_f64(self.u[idx]);
            hasher.write_f64(self.v[idx]);
            hasher.write_f64(self.w[idx]);
            hasher.write_f64(self.smoke_density[idx]);
            hasher.write_u64(if self.cell_type[idx] == CellType::Solid {
                1
            } else {
                0
            });
        }
        hasher.write_f64(self.vorticity_confinement_epsilon);
    }

    fn approximations(&self) -> Vec<Approximation> {
        let mut out = vec![Approximation {
            name: "semi-Lagrangian移流",
            reason: "無条件安定だが数値拡散が大きい(低粘性域では真の粘性より拡散的)。",
            doc: "docs/11-fluid/02-eulerian-grid.md",
            can_disable: false,
        }];
        out.push(match self.boundary {
            GridBoundary3D::Periodic => Approximation {
                name: "3D・周期境界",
                reason: "全方向が周期境界のため、下流の後流が上流へ回り込む。",
                doc: "docs/11-fluid/02-eulerian-grid.md",
                can_disable: true,
            },
            GridBoundary3D::Channel { .. } => Approximation {
                name: "3D・開境界(流路)",
                reason: "x−から流入・x+へ流出、y/z壁は自由すべり。粘着境界層は作れない。",
                doc: "docs/11-fluid/02-eulerian-grid.md",
                can_disable: true,
            },
        });
        out.push(Approximation {
            name: "前処理なしPCG",
            reason: "圧力ポアソンを前処理なしのPCGで解いており、設計§10の 64³/4ms 予算に\
                     全く届かない(実測 795.7 ms/step、約200倍)。マルチグリッド前処理は未実装。",
            doc: "docs/11-fluid/02-eulerian-grid.md",
            can_disable: false,
        });
        if self.has_solid() {
            out.push(Approximation {
                name: "セル単位の固体境界",
                reason: "固体はセル中心の内外判定でラスタライズする(cut-cell法ではない\
                         ため、表面が階段状になる)。",
                doc: "docs/11-fluid/02-eulerian-grid.md",
                can_disable: false,
            });
        }
        if self.vorticity_confinement_epsilon > 0.0 {
            out.push(Approximation {
                name: "渦度強化(非物理)",
                reason: "数値拡散で失われた小渦を補償する非物理的な外力を加えている\
                         (Fedkiw et al. 2001)。検証時は epsilon=0 にすること。",
                doc: "docs/11-fluid/02-eulerian-grid.md",
                can_disable: true,
            });
        }
        if self.kinematic_viscosity == 0.0 {
            out.push(Approximation {
                name: "粘性拡散をスキップ",
                reason: "kinematic_viscosity=0 のため陽的粘性拡散の段を実行していない。",
                doc: "docs/11-fluid/02-eulerian-grid.md",
                can_disable: true,
            });
        }
        out
    }
}
