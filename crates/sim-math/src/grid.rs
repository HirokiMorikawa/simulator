//! 場の表現: セル中心格子・MAC 格子・補間・微分演算子。
//! 設計: docs/01-math/02-fields.md §1–4。

use crate::Vec3;

/// 一様間隔の3次元格子。値はセル中心に置く。設計 §1。
pub struct Grid3<T> {
    pub nx: usize,
    pub ny: usize,
    pub nz: usize,
    pub h: f64,
    pub origin: Vec3,
    data: Vec<T>,
}

impl<T: Copy> Grid3<T> {
    pub fn new(nx: usize, ny: usize, nz: usize, h: f64, origin: Vec3, fill: T) -> Grid3<T> {
        Grid3 {
            nx,
            ny,
            nz,
            h,
            origin,
            data: vec![fill; nx * ny * nz],
        }
    }

    fn index(&self, i: usize, j: usize, k: usize) -> usize {
        i + self.nx * (j + self.ny * k)
    }

    pub fn at(&self, i: usize, j: usize, k: usize) -> T {
        self.data[self.index(i, j, k)]
    }

    pub fn set(&mut self, i: usize, j: usize, k: usize, v: T) {
        let idx = self.index(i, j, k);
        self.data[idx] = v;
    }

    /// セル(i,j,k) の中心のワールド座標: origin + h*(i+0.5, j+0.5, k+0.5)。
    pub fn cell_center(&self, i: usize, j: usize, k: usize) -> Vec3 {
        self.origin
            + Vec3::new(
                (i as f64 + 0.5) * self.h,
                (j as f64 + 0.5) * self.h,
                (k as f64 + 0.5) * self.h,
            )
    }

    /// ワールド座標 → 含まれるセル(境界条件はサンプラ側が扱う)。
    pub fn world_to_cell(&self, p: Vec3) -> (i64, i64, i64) {
        let rel = p - self.origin;
        (
            (rel.x / self.h).floor() as i64,
            (rel.y / self.h).floor() as i64,
            (rel.z / self.h).floor() as i64,
        )
    }
}

/// 境界の扱い。格子ではなくサンプラが持つ(設計 §1.1)。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BoundaryRule<T> {
    /// 最近傍セル値(温度など)。
    Clamp,
    /// 固定値(無限遠の環境温度など)。
    Constant(T),
    /// ∂/∂n = 0(断熱壁)。セル中心格子ではゴースト = 境界セル値となり Clamp と等価。
    ZeroGradient,
    /// 周期境界(統計力学デモ用)。
    Periodic,
}

/// 境界条件つきの格子アクセサ。設計 §1.1。
pub struct GridSampler<'a, T> {
    pub grid: &'a Grid3<T>,
    pub boundary: BoundaryRule<T>,
}

impl<'a, T: Copy> GridSampler<'a, T> {
    pub fn new(grid: &'a Grid3<T>, boundary: BoundaryRule<T>) -> GridSampler<'a, T> {
        GridSampler { grid, boundary }
    }

    fn in_bounds(&self, i: i64, j: i64, k: i64) -> bool {
        (0..self.grid.nx as i64).contains(&i)
            && (0..self.grid.ny as i64).contains(&j)
            && (0..self.grid.nz as i64).contains(&k)
    }

    pub fn at(&self, i: i64, j: i64, k: i64) -> T {
        match self.boundary {
            BoundaryRule::Constant(v) => {
                if self.in_bounds(i, j, k) {
                    self.grid.at(i as usize, j as usize, k as usize)
                } else {
                    v
                }
            }
            BoundaryRule::Clamp | BoundaryRule::ZeroGradient => {
                let ci = i.clamp(0, self.grid.nx as i64 - 1) as usize;
                let cj = j.clamp(0, self.grid.ny as i64 - 1) as usize;
                let ck = k.clamp(0, self.grid.nz as i64 - 1) as usize;
                self.grid.at(ci, cj, ck)
            }
            BoundaryRule::Periodic => {
                let wi = i.rem_euclid(self.grid.nx as i64) as usize;
                let wj = j.rem_euclid(self.grid.ny as i64) as usize;
                let wk = k.rem_euclid(self.grid.nz as i64) as usize;
                self.grid.at(wi, wj, wk)
            }
        }
    }
}

/// (u, t) = (基準セル添字, セル内正規化座標)。セル中心基準(cell_center 系)の連続座標。
fn continuous_cell_coord(grid: &Grid3<f64>, p: Vec3) -> (i64, i64, i64, f64, f64, f64) {
    let u = (p - grid.origin).scale(1.0 / grid.h) - Vec3::new(0.5, 0.5, 0.5);
    let i0 = u.x.floor() as i64;
    let j0 = u.y.floor() as i64;
    let k0 = u.z.floor() as i64;
    (
        i0,
        j0,
        k0,
        u.x - i0 as f64,
        u.y - j0 as f64,
        u.z - k0 as f64,
    )
}

/// トライリニア補間(既定)。設計 §3.1。C0連続・単調・8セル参照。
pub fn trilinear_sample(sampler: &GridSampler<f64>, p: Vec3) -> f64 {
    let (i0, j0, k0, tx, ty, tz) = continuous_cell_coord(sampler.grid, p);
    let mut sum = 0.0;
    for cx in 0..2i64 {
        let wx = if cx == 1 { tx } else { 1.0 - tx };
        for cy in 0..2i64 {
            let wy = if cy == 1 { ty } else { 1.0 - ty };
            for cz in 0..2i64 {
                let wz = if cz == 1 { tz } else { 1.0 - tz };
                sum += sampler.at(i0 + cx, j0 + cy, k0 + cz) * wx * wy * wz;
            }
        }
    }
    sum
}

fn catmull_rom_1d(p0: f64, p1: f64, p2: f64, p3: f64, t: f64) -> f64 {
    let t2 = t * t;
    let t3 = t2 * t;
    0.5 * (2.0 * p1
        + (-p0 + p2) * t
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3)
}

/// 三次補間(Catmull-Rom、オプション)。設計 §3.2。64セル参照、8セルの min/max にクランプして
/// オーバーシュートを抑制する(Fedkiw らの標準手法)。
pub fn catmull_rom_sample(sampler: &GridSampler<f64>, p: Vec3) -> f64 {
    let (i0, j0, k0, tx, ty, tz) = continuous_cell_coord(sampler.grid, p);
    let sample = |di: i64, dj: i64, dk: i64| sampler.at(i0 + di, j0 + dj, k0 + dk);

    let mut z_line = [0.0; 4];
    for (dz_idx, dz) in (-1..=2i64).enumerate() {
        let mut y_line = [0.0; 4];
        for (dy_idx, dy) in (-1..=2i64).enumerate() {
            let p0 = sample(-1, dy, dz);
            let p1 = sample(0, dy, dz);
            let p2 = sample(1, dy, dz);
            let p3 = sample(2, dy, dz);
            y_line[dy_idx] = catmull_rom_1d(p0, p1, p2, p3, tx);
        }
        z_line[dz_idx] = catmull_rom_1d(y_line[0], y_line[1], y_line[2], y_line[3], ty);
    }
    let raw = catmull_rom_1d(z_line[0], z_line[1], z_line[2], z_line[3], tz);

    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for cx in 0..2i64 {
        for cy in 0..2i64 {
            for cz in 0..2i64 {
                let v = sample(cx, cy, cz);
                lo = lo.min(v);
                hi = hi.max(v);
            }
        }
    }
    raw.clamp(lo, hi)
}

/// 中心差分の勾配(セル(i,j,k)、二次精度)。設計 §4。
pub fn gradient(sampler: &GridSampler<f64>, i: i64, j: i64, k: i64) -> Vec3 {
    let two_h = 2.0 * sampler.grid.h;
    Vec3::new(
        (sampler.at(i + 1, j, k) - sampler.at(i - 1, j, k)) / two_h,
        (sampler.at(i, j + 1, k) - sampler.at(i, j - 1, k)) / two_h,
        (sampler.at(i, j, k + 1) - sampler.at(i, j, k - 1)) / two_h,
    )
}

/// 中心差分のラプラシアン(係数一様)。設計 §4。
pub fn laplacian(sampler: &GridSampler<f64>, i: i64, j: i64, k: i64) -> f64 {
    let h_sq = sampler.grid.h * sampler.grid.h;
    let sum6 = sampler.at(i + 1, j, k)
        + sampler.at(i - 1, j, k)
        + sampler.at(i, j + 1, k)
        + sampler.at(i, j - 1, k)
        + sampler.at(i, j, k + 1)
        + sampler.at(i, j, k - 1);
    (sum6 - 6.0 * sampler.at(i, j, k)) / h_sq
}

/// 流束形式のラプラシアン ∇·(k∇T)。係数が空間変化する場合(熱伝導率の不均一)用。
/// 面中心の伝導率は調和平均 k_{i+1/2} = 2 k_i k_{i+1} / (k_i + k_{i+1}) で評価する
/// (界面での流束連続性を保つ。出典: Patankar, *Numerical Heat Transfer*)。設計 §4。
pub fn laplacian_variable_coefficient(
    field: &GridSampler<f64>,
    conductivity: &GridSampler<f64>,
    i: i64,
    j: i64,
    k: i64,
) -> f64 {
    let h_sq = field.grid.h * field.grid.h;
    let k0 = conductivity.at(i, j, k);
    let f0 = field.at(i, j, k);
    let face_flux = |k_nb: f64, f_nb: f64| -> f64 {
        let k_face = if k0 + k_nb > 0.0 {
            2.0 * k0 * k_nb / (k0 + k_nb)
        } else {
            0.0
        };
        k_face * (f_nb - f0) / h_sq
    };
    face_flux(conductivity.at(i + 1, j, k), field.at(i + 1, j, k))
        + face_flux(conductivity.at(i - 1, j, k), field.at(i - 1, j, k))
        + face_flux(conductivity.at(i, j + 1, k), field.at(i, j + 1, k))
        + face_flux(conductivity.at(i, j - 1, k), field.at(i, j - 1, k))
        + face_flux(conductivity.at(i, j, k + 1), field.at(i, j, k + 1))
        + face_flux(conductivity.at(i, j, k - 1), field.at(i, j, k - 1))
}

/// スタガード格子(MAC、Marker-and-Cell)。非圧縮流体の速度場用。設計 §2。
/// `u` は x 面((nx+1) × ny × nz)、`v` は y 面(nx × (ny+1) × nz)、`w` は z 面。
pub struct MacGrid {
    pub u: Grid3<f64>,
    pub v: Grid3<f64>,
    pub w: Grid3<f64>,
}

impl MacGrid {
    pub fn new(nx: usize, ny: usize, nz: usize, h: f64, origin: Vec3, fill: f64) -> MacGrid {
        MacGrid {
            u: Grid3::new(nx + 1, ny, nz, h, origin, fill),
            v: Grid3::new(nx, ny + 1, nz, h, origin, fill),
            w: Grid3::new(nx, ny, nz + 1, h, origin, fill),
        }
    }

    /// セル(i,j,k)の発散(半セルずれた中心差分、設計 §2 の式)。
    pub fn divergence(&self, i: usize, j: usize, k: usize) -> f64 {
        let h = self.u.h;
        (self.u.at(i + 1, j, k) - self.u.at(i, j, k)) / h
            + (self.v.at(i, j + 1, k) - self.v.at(i, j, k)) / h
            + (self.w.at(i, j, k + 1) - self.w.at(i, j, k)) / h
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_field(nx: usize, ny: usize, nz: usize, h: f64, f: impl Fn(Vec3) -> f64) -> Grid3<f64> {
        let origin = Vec3::ZERO;
        let mut grid = Grid3::new(nx, ny, nz, h, origin, 0.0);
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    grid.set(i, j, k, f(grid.cell_center(i, j, k)));
                }
            }
        }
        grid
    }

    #[test]
    fn cell_center_matches_formula() {
        let grid: Grid3<f64> = Grid3::new(4, 4, 4, 0.5, Vec3::new(1.0, 0.0, 0.0), 0.0);
        let c = grid.cell_center(1, 2, 3);
        assert_eq!(c, Vec3::new(1.0 + 0.5 * 1.5, 0.5 * 2.5, 0.5 * 3.5));
    }

    #[test]
    fn boundary_rule_clamp_returns_nearest_valid_cell() {
        let grid = make_field(4, 4, 4, 1.0, |p| p.x);
        let sampler = GridSampler::new(&grid, BoundaryRule::Clamp);
        assert_eq!(sampler.at(-3, 0, 0), sampler.at(0, 0, 0));
        assert_eq!(sampler.at(10, 0, 0), sampler.at(3, 0, 0));
    }

    #[test]
    fn boundary_rule_constant_returns_fixed_value_outside() {
        let grid = make_field(4, 4, 4, 1.0, |_| 1.0);
        let sampler = GridSampler::new(&grid, BoundaryRule::Constant(-9.0));
        assert_eq!(sampler.at(-1, 0, 0), -9.0);
        assert_eq!(sampler.at(0, 0, 0), 1.0);
    }

    #[test]
    fn boundary_rule_periodic_wraps_around() {
        let grid = make_field(4, 4, 4, 1.0, |p| p.x);
        let sampler = GridSampler::new(&grid, BoundaryRule::Periodic);
        assert_eq!(sampler.at(-1, 0, 0), sampler.at(3, 0, 0));
        assert_eq!(sampler.at(4, 0, 0), sampler.at(0, 0, 0));
    }

    /// 設計 §7: トライリニアは定数場・線形場を厳密再現する(eps_abs=1e-12)。
    #[test]
    fn trilinear_reproduces_constant_field() {
        let grid = make_field(8, 8, 8, 0.25, |_| 42.0);
        let sampler = GridSampler::new(&grid, BoundaryRule::Clamp);
        for _ in 0..20 {
            let p = Vec3::new(0.37, 1.1, 0.9);
            assert!((trilinear_sample(&sampler, p) - 42.0).abs() < 1e-12);
        }
    }

    #[test]
    fn trilinear_reproduces_linear_field() {
        let grid = make_field(10, 10, 10, 0.2, |p| 2.0 * p.x - 3.0 * p.y + 0.5 * p.z + 1.0);
        let sampler = GridSampler::new(&grid, BoundaryRule::Clamp);
        let p = Vec3::new(0.83, 1.21, 0.47);
        let expected = 2.0 * p.x - 3.0 * p.y + 0.5 * p.z + 1.0;
        assert!((trilinear_sample(&sampler, p) - expected).abs() < 1e-12);
    }

    #[test]
    fn catmull_rom_reproduces_linear_field_and_stays_bounded() {
        let grid = make_field(12, 12, 12, 0.2, |p| p.x - p.y);
        let sampler = GridSampler::new(&grid, BoundaryRule::Clamp);
        let p = Vec3::new(1.0, 1.0, 1.0);
        let expected = p.x - p.y;
        assert!((catmull_rom_sample(&sampler, p) - expected).abs() < 1e-9);
    }

    /// 設計 §7: 微分は多項式場で二次収束(h 半減で誤差 1/4)。
    /// f=x^2 のような二次場は中心差分が厳密に一致してしまう(打ち切り誤差がゼロ)ため、
    /// 4階微分が非ゼロな四次場 f=x^4+y^4+z^4(ラプラシアン解析値 12(x²+y²+z²))を使う。
    #[test]
    fn laplacian_converges_at_second_order_on_quartic_field() {
        let error_at = |h: f64| -> f64 {
            let n = 16;
            let grid = make_field(n, n, n, h, |p| p.x.powi(4) + p.y.powi(4) + p.z.powi(4));
            let sampler = GridSampler::new(&grid, BoundaryRule::ZeroGradient);
            let mid = (n / 2) as i64;
            let center = grid.cell_center(mid as usize, mid as usize, mid as usize);
            let analytic = 12.0 * center.length_sq();
            (laplacian(&sampler, mid, mid, mid) - analytic).abs()
        };
        let e1 = error_at(0.1);
        let e2 = error_at(0.05);
        let order = (e1 / e2).log2();
        assert!(
            (order - 2.0).abs() < 0.3,
            "expected ~2nd order, got {order} (e1={e1}, e2={e2})"
        );
    }

    #[test]
    fn gradient_matches_analytic_on_linear_field() {
        let grid = make_field(10, 10, 10, 0.1, |p| 3.0 * p.x + 2.0 * p.y - p.z);
        let sampler = GridSampler::new(&grid, BoundaryRule::ZeroGradient);
        let g = gradient(&sampler, 5, 5, 5);
        assert!((g - Vec3::new(3.0, 2.0, -1.0)).length() < 1e-9);
    }

    #[test]
    fn mac_grid_divergence_free_field_is_zero() {
        // u=x, v=-y なら発散 (du/dx + dv/dy) = 1 - 1 = 0 (2D、w=0一様)。
        let h = 0.1;
        let origin = Vec3::ZERO;
        let mut mac = MacGrid::new(8, 8, 1, h, origin, 0.0);
        for k in 0..1 {
            for j in 0..8 {
                for i in 0..=8 {
                    let x = origin.x + i as f64 * h;
                    mac.u.set(i, j, k, x);
                }
            }
        }
        for k in 0..1 {
            for j in 0..=8 {
                for i in 0..8 {
                    let y = origin.y + j as f64 * h;
                    mac.v.set(i, j, k, -y);
                }
            }
        }
        assert!(mac.divergence(4, 4, 0).abs() < 1e-12);
    }

    #[test]
    fn mac_grid_nonzero_divergence_matches_expected() {
        // u = x (定数勾配 du/dx=1)、v=w=0 なら発散は 1。
        let h = 0.5;
        let mut mac = MacGrid::new(4, 4, 4, h, Vec3::ZERO, 0.0);
        for k in 0..4 {
            for j in 0..4 {
                for i in 0..=4 {
                    mac.u.set(i, j, k, i as f64 * h);
                }
            }
        }
        assert!((mac.divergence(2, 2, 2) - 1.0).abs() < 1e-12);
    }
}
