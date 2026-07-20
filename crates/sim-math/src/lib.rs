//! 線形代数基盤。設計: docs/01-math/01-linear-algebra.md
//!
//! `Vec3`/`Quat`/`Mat3`/`Transform`(docs/01-math/01-linear-algebra.md)、
//! `SimRng`(docs/01-math/04-random.md)、積分器カタログの汎用部分
//! (docs/01-math/03-integrators.md)、場・PCG・粒子集合(docs/01-math/02-fields.md)
//! を実装する(math ウェーブ、docs/22-roadmap/01-phases.md)。

mod complex;
mod fft;
mod grid;
mod integrators;
mod particles;
mod pcg;
mod random;
pub use complex::Complex64;
pub use fft::{fft, ifft};
pub use grid::{
    catmull_rom_sample, gradient, laplacian, laplacian_variable_coefficient, trilinear_sample,
    BoundaryRule, Grid3, GridSampler, MacGrid,
};
pub use integrators::{
    explicit_euler_step, rk4_step, semi_implicit_euler_step, velocity_verlet_step,
    BallisticIntegrator, BorisPusher,
};
pub use particles::{ParticleSet, SpatialHash};
pub use pcg::{pcg, PcgResult, Preconditioner};
pub use random::SimRng;

use std::ops::{Add, Mul, Neg, Sub};

/// 長さ比較に使う下限(このスカラーより短いベクトルは実質ゼロ扱い)。
/// docs/01-math/01-linear-algebra.md §2 `normalize_or_zero` の規約値。
pub const EPS_LEN: f64 = 1e-12;

#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub fn new(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    pub fn dot(self, rhs: Vec3) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    /// 右手系の外積。
    pub fn cross(self, rhs: Vec3) -> Vec3 {
        Vec3::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }

    pub fn scale(self, s: f64) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }

    /// 比較には `length` でなくこちらを使う(sqrt を避ける)。
    pub fn length_sq(self) -> f64 {
        self.dot(self)
    }

    pub fn length(self) -> f64 {
        self.length_sq().sqrt()
    }

    /// self + v*s。積分の内部ループで頻出する形。
    pub fn addcarry_scaled(self, v: Vec3, s: f64) -> Vec3 {
        Vec3::new(self.x + v.x * s, self.y + v.y * s, self.z + v.z * s)
    }

    /// |v| < EPS_LEN ならゼロベクトルを返す(ゼロ除算回避)。
    pub fn normalize_or_zero(self) -> Vec3 {
        let len_sq = self.length_sq();
        if len_sq < EPS_LEN * EPS_LEN {
            Vec3::ZERO
        } else {
            self.scale(1.0 / len_sq.sqrt())
        }
    }

    /// self に直交する単位ベクトル対(接線基底)。
    /// 決定的アルゴリズム: |成分|が最小の軸との外積 → 正規化 → もう1本は外積。
    pub fn orthonormal_basis(self) -> (Vec3, Vec3) {
        let ax = self.x.abs();
        let ay = self.y.abs();
        let az = self.z.abs();
        let helper = if ax <= ay && ax <= az {
            Vec3::new(1.0, 0.0, 0.0)
        } else if ay <= az {
            Vec3::new(0.0, 1.0, 0.0)
        } else {
            Vec3::new(0.0, 0.0, 1.0)
        };
        let t1 = self.cross(helper).normalize_or_zero();
        let t2 = self.cross(t1);
        (t1, t2)
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    fn add(self, rhs: Vec3) -> Vec3 {
        Vec3::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, rhs: Vec3) -> Vec3 {
        Vec3::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}

impl Mul<f64> for Vec3 {
    type Output = Vec3;
    fn mul(self, rhs: f64) -> Vec3 {
        self.scale(rhs)
    }
}

/// 行列式の下限(この値未満なら特異とみなし `inverse()` は `None`)。
/// 設計 (docs/01-math/01-linear-algebra.md §4) は名前のみ規定し値は実装判断。
/// `EPS_LEN` と同オーダーの `1e-12` を採用する。
pub const EPS_DET: f64 = 1e-12;

/// 単位クォータニオン(回転)。w が実部。恒等回転は (0,0,0,1)。
/// 設計: docs/01-math/01-linear-algebra.md §3。
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Quat {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Quat {
    pub const IDENTITY: Quat = Quat {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };

    pub fn from_axis_angle(axis: Vec3, angle_rad: f64) -> Quat {
        let half = angle_rad * 0.5;
        let s = half.sin();
        let a = axis.normalize_or_zero();
        Quat {
            x: a.x * s,
            y: a.y * s,
            z: a.z * s,
            w: half.cos(),
        }
    }

    fn vector_part(self) -> Vec3 {
        Vec3::new(self.x, self.y, self.z)
    }

    /// 回転の合成: self が後(self ∘ rhs、つまり rhs を先に適用してから self)。
    // 設計 (docs/01-math/01-linear-algebra.md §3) がメソッド名 `mul` を明示するため、
    // `std::ops::Mul` との命名衝突の指摘は意図的に抑制する。
    #[allow(clippy::should_implement_trait)]
    pub fn mul(self, rhs: Quat) -> Quat {
        Quat {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }

    /// 単位quatでは逆回転。
    pub fn conjugate(self) -> Quat {
        Quat {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: self.w,
        }
    }

    pub fn length_sq(self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
    }

    pub fn normalize(self) -> Quat {
        let len = self.length_sq().sqrt();
        Quat {
            x: self.x / len,
            y: self.y / len,
            z: self.z / len,
            w: self.w / len,
        }
    }

    /// v' = v + 2*qv × (qv × v + w*v)(展開式、クォータニオン積2回より高速)。
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let qv = self.vector_part();
        let t = qv.cross(v).addcarry_scaled(v, self.w).scale(2.0);
        v + qv.cross(t)
    }

    pub fn to_mat3(self) -> Mat3 {
        let (x, y, z, w) = (self.x, self.y, self.z, self.w);
        Mat3 {
            m: [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w),
                ],
                [
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w),
                ],
                [
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y),
                ],
            ],
        }
    }

    /// 描画補間用(コア物理では未使用)。二重被覆は内積の符号で吸収する。
    pub fn slerp(self, to: Quat, t: f64) -> Quat {
        let mut b = to;
        let mut cos_theta = self.x * b.x + self.y * b.y + self.z * b.z + self.w * b.w;
        if cos_theta < 0.0 {
            b = Quat {
                x: -b.x,
                y: -b.y,
                z: -b.z,
                w: -b.w,
            };
            cos_theta = -cos_theta;
        }
        if cos_theta > 1.0 - 1e-9 {
            return Quat {
                x: self.x + (b.x - self.x) * t,
                y: self.y + (b.y - self.y) * t,
                z: self.z + (b.z - self.z) * t,
                w: self.w + (b.w - self.w) * t,
            }
            .normalize();
        }
        let theta = cos_theta.acos();
        let sin_theta = theta.sin();
        let wa = ((1.0 - t) * theta).sin() / sin_theta;
        let wb = (t * theta).sin() / sin_theta;
        Quat {
            x: wa * self.x + wb * b.x,
            y: wa * self.y + wb * b.y,
            z: wa * self.z + wb * b.z,
            w: wa * self.w + wb * b.w,
        }
    }

    /// 角速度 ω による回転の一次積分:
    /// q(t+dt) = normalize(q + dt/2 * ω_quat ⊗ q)、ω_quat = (ωx, ωy, ωz, 0)。
    pub fn integrate_angular_velocity(self, omega: Vec3, dt: f64) -> Quat {
        let omega_quat = Quat {
            x: omega.x,
            y: omega.y,
            z: omega.z,
            w: 0.0,
        };
        let derivative = omega_quat.mul(self);
        Quat {
            x: self.x + derivative.x * 0.5 * dt,
            y: self.y + derivative.y * 0.5 * dt,
            z: self.z + derivative.z * 0.5 * dt,
            w: self.w + derivative.w * 0.5 * dt,
        }
        .normalize()
    }
}

/// 3x3 行列。行優先(m[row][col])。慣性テンソル・回転行列に使う。
/// 設計: docs/01-math/01-linear-algebra.md §4。
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Mat3 {
    pub m: [[f64; 3]; 3],
}

impl Mat3 {
    pub fn identity() -> Mat3 {
        Mat3::from_diagonal(Vec3::new(1.0, 1.0, 1.0))
    }

    pub fn from_diagonal(d: Vec3) -> Mat3 {
        Mat3 {
            m: [[d.x, 0.0, 0.0], [0.0, d.y, 0.0], [0.0, 0.0, d.z]],
        }
    }

    pub fn mul_vec(self, v: Vec3) -> Vec3 {
        Vec3::new(
            self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z,
            self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z,
            self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z,
        )
    }

    // 設計 (docs/01-math/01-linear-algebra.md §4) がメソッド名 `mul` を明示するため、
    // `std::ops::Mul` との命名衝突の指摘は意図的に抑制する。
    #[allow(clippy::should_implement_trait)]
    pub fn mul(self, rhs: Mat3) -> Mat3 {
        let mut m = [[0.0; 3]; 3];
        for (i, row) in m.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = self.m[i][0] * rhs.m[0][j]
                    + self.m[i][1] * rhs.m[1][j]
                    + self.m[i][2] * rhs.m[2][j];
            }
        }
        Mat3 { m }
    }

    pub fn transpose(self) -> Mat3 {
        let mut m = [[0.0; 3]; 3];
        for (i, row) in m.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = self.m[j][i];
            }
        }
        Mat3 { m }
    }

    fn determinant(self) -> f64 {
        let m = self.m;
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    /// 余因子法(随伴行列 adj(A) = 余因子行列の転置、A^-1 = adj(A)/det)。
    /// det < EPS_DET で None。
    pub fn inverse(self) -> Option<Mat3> {
        let det = self.determinant();
        if det.abs() < EPS_DET {
            return None;
        }
        let m = self.m;
        let inv_det = 1.0 / det;
        let out = [
            [
                (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det,
                (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det,
                (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det,
            ],
            [
                (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det,
                (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det,
                (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det,
            ],
            [
                (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det,
                (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det,
                (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det,
            ],
        ];
        Some(Mat3 { m: out })
    }

    /// 相似変換 R * self * R^T。ローカル慣性テンソル→ワールドで毎ステップ使用。
    pub fn similarity(self, r: Mat3) -> Mat3 {
        r.mul(self).mul(r.transpose())
    }

    /// 歪対称行列 [v]×(v.cross(x) = skew(v) * x)。ヤコビアン組み立てで使用。
    pub fn skew(v: Vec3) -> Mat3 {
        Mat3 {
            m: [[0.0, -v.z, v.y], [v.z, 0.0, -v.x], [-v.y, v.x, 0.0]],
        }
    }
}

/// 剛体変換(回転→平行移動の順で適用)。スケールは持たない。
/// 設計: docs/01-math/01-linear-algebra.md §5。
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Transform {
    pub position: Vec3,
    pub rotation: Quat,
}

impl Transform {
    pub fn apply_point(self, p: Vec3) -> Vec3 {
        self.rotation.rotate(p) + self.position
    }

    pub fn apply_dir(self, d: Vec3) -> Vec3 {
        self.rotation.rotate(d)
    }

    pub fn inverse(self) -> Transform {
        let inv_rotation = self.rotation.conjugate();
        Transform {
            position: -inv_rotation.rotate(self.position),
            rotation: inv_rotation,
        }
    }

    /// self ∘ inner。
    pub fn compose(self, inner: Transform) -> Transform {
        Transform {
            position: self.apply_point(inner.position),
            rotation: self.rotation.mul(inner.rotation),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// テスト専用の決定論的 PRNG(xorshift64*)。正式な `SimRng`
    /// (docs/01-math/04-random.md、math ウェーブで実装)が入るまでの暫定。
    struct TestRng(u64);
    impl TestRng {
        fn next_u64(&mut self) -> u64 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            self.0 = x;
            x.wrapping_mul(0x2545F4914F6CDD1D)
        }
        fn next_f64(&mut self) -> f64 {
            (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
        }
        fn range(&mut self, lo: f64, hi: f64) -> f64 {
            lo + self.next_f64() * (hi - lo)
        }
        fn vec3(&mut self, lo: f64, hi: f64) -> Vec3 {
            Vec3::new(self.range(lo, hi), self.range(lo, hi), self.range(lo, hi))
        }
        fn quat(&mut self) -> Quat {
            let axis = self.vec3(-1.0, 1.0).normalize_or_zero();
            let angle = self.range(-std::f64::consts::PI, std::f64::consts::PI);
            Quat::from_axis_angle(axis, angle)
        }
    }

    #[test]
    fn cross_right_handed_basis() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let z = Vec3::new(0.0, 0.0, 1.0);
        assert_eq!(x.cross(y), z);
    }

    #[test]
    fn dot_orthogonal_is_zero() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        assert_eq!(x.dot(y), 0.0);
    }

    #[test]
    fn normalize_or_zero_below_eps_is_zero() {
        let tiny = Vec3::new(1e-13, 0.0, 0.0);
        assert_eq!(tiny.normalize_or_zero(), Vec3::ZERO);
    }

    #[test]
    fn normalize_or_zero_unit_length() {
        let v = Vec3::new(3.0, 4.0, 0.0);
        let n = v.normalize_or_zero();
        assert!((n.length() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn orthonormal_basis_is_orthogonal_to_self_and_each_other() {
        let v = Vec3::new(0.3, -0.7, 2.1).normalize_or_zero();
        let (t1, t2) = v.orthonormal_basis();
        assert!(v.dot(t1).abs() < 1e-12);
        assert!(v.dot(t2).abs() < 1e-12);
        assert!(t1.dot(t2).abs() < 1e-12);
    }

    #[test]
    fn quat_90deg_about_z_rotates_x_to_y() {
        let q = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_2);
        let r = q.rotate(Vec3::new(1.0, 0.0, 0.0));
        assert!((r.x - 0.0).abs() < 1e-12);
        assert!((r.y - 1.0).abs() < 1e-12);
        assert!((r.z - 0.0).abs() < 1e-12);
    }

    /// docs/01-math/01-linear-algebra.md §6: 恒等式(ランダム入力×決定シードで
    /// 10^4 ケース、eps_abs=1e-12)。
    #[test]
    fn quat_rotation_preserves_length() {
        let mut rng = TestRng(0x9E3779B97F4A7C15);
        for _ in 0..10_000 {
            let q = rng.quat();
            let v = rng.vec3(-5.0, 5.0);
            let rotated = q.rotate(v);
            assert!((rotated.length() - v.length()).abs() < 1e-9);
        }
    }

    #[test]
    fn quat_to_mat3_matches_rotate() {
        let mut rng = TestRng(0xD1B54A32D192ED03);
        for _ in 0..10_000 {
            let q = rng.quat();
            let v = rng.vec3(-5.0, 5.0);
            let via_mat3 = q.to_mat3().mul_vec(v);
            let via_rotate = q.rotate(v);
            assert!((via_mat3 - via_rotate).length() < 1e-9);
        }
    }

    #[test]
    fn mat3_mul_inverse_is_identity() {
        let mut rng = TestRng(0x2545F4914F6CDD1D);
        let mut checked = 0;
        while checked < 10_000 {
            let q = rng.quat();
            let scale = Vec3::new(
                rng.range(0.5, 2.0),
                rng.range(0.5, 2.0),
                rng.range(0.5, 2.0),
            );
            let m = q.to_mat3().mul(Mat3::from_diagonal(scale));
            let inv = m.inverse().expect("well-conditioned matrix must invert");
            let product = m.mul(inv);
            let identity = Mat3::identity();
            for i in 0..3 {
                for j in 0..3 {
                    assert!((product.m[i][j] - identity.m[i][j]).abs() < 1e-9);
                }
            }
            checked += 1;
        }
    }

    #[test]
    fn mat3_inverse_none_for_singular_matrix() {
        let singular = Mat3 {
            m: [[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [1.0, 0.0, 1.0]],
        };
        assert!(singular.inverse().is_none());
    }

    #[test]
    fn skew_matches_cross() {
        let mut rng = TestRng(0x853C49E6748FEA9B);
        for _ in 0..10_000 {
            let a = rng.vec3(-5.0, 5.0);
            let b = rng.vec3(-5.0, 5.0);
            let via_skew = Mat3::skew(a).mul_vec(b);
            let via_cross = a.cross(b);
            assert!((via_skew - via_cross).length() < 1e-9);
        }
    }

    /// `integrate_angular_velocity` を n 分割して合成 → 解析回転
    /// `from_axis_angle(ω̂, |ω| t)` に一次収束することを確認する。
    #[test]
    fn integrate_angular_velocity_converges_to_analytic_rotation() {
        let omega = Vec3::new(0.4, -0.3, 0.9);
        let t = 1.0;
        let analytic = Quat::from_axis_angle(omega.normalize_or_zero(), omega.length() * t);

        let mut prev_err = f64::INFINITY;
        for n in [8u32, 16, 32, 64] {
            let dt = t / n as f64;
            let mut q = Quat::IDENTITY;
            for _ in 0..n {
                q = q.integrate_angular_velocity(omega, dt);
            }
            let err = (q.x - analytic.x).powi(2)
                + (q.y - analytic.y).powi(2)
                + (q.z - analytic.z).powi(2)
                + (q.w - analytic.w).powi(2);
            let err = err.sqrt();
            if prev_err.is_finite() {
                // 一次(誤差 ∝ dt)なので、刻みを半分にすれば誤差もおよそ半分になる。
                assert!(err < prev_err * 0.6);
            }
            prev_err = err;
        }
    }

    #[test]
    fn transform_inverse_round_trip() {
        let mut rng = TestRng(0x0123456789ABCDEF);
        for _ in 0..1000 {
            let t = Transform {
                position: rng.vec3(-10.0, 10.0),
                rotation: rng.quat(),
            };
            let p = rng.vec3(-10.0, 10.0);
            let round_tripped = t.inverse().apply_point(t.apply_point(p));
            assert!((round_tripped - p).length() < 1e-9);
        }
    }
}
