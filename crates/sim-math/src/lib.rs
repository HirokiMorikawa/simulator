//! 線形代数基盤。設計: docs/01-math/01-linear-algebra.md
//!
//! Phase 0 では `Vec3` のみを実装する。`Quat`/`Mat3`/`Transform` は
//! math ウェーブ(Phase A/B)で同文書に沿って追加する。

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

#[cfg(test)]
mod tests {
    use super::*;

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
}
