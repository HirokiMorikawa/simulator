//! 形状・AABB・接触マニフォールドの型。設計: docs/10-mechanics/02-collision-detection.md §3。

use sim_math::Vec3;

/// 剛体の幾何形状。Phase 1 は Sphere/Box/Plane のみ narrowphase 実装対象
/// (docs/10-mechanics/02-collision-detection.md §4.2)。Capsule/Compound/ConvexMesh は
/// 型として先に定義し、中身は担当フェーズ(P2/P5)で実装する。
#[derive(Clone, Debug)]
pub enum Shape {
    Sphere {
        radius: f64,
    },
    Box {
        half_extents: Vec3,
    },
    /// Phase 2。
    Capsule {
        radius: f64,
        half_height: f64,
    },
    /// static 専用・無限平面。
    Plane {
        normal: Vec3,
        d: f64,
    },
    /// Phase 2。
    Compound {
        children: Vec<(sim_math::Transform, Shape)>,
    },
    /// Phase 5(GJK/EPA)。
    ConvexMesh {
        vertices: Vec<Vec3>,
    },
}

impl Shape {
    /// 体積(質量 = 密度 × 体積の算出に使う)。Plane/Compound/ConvexMesh は
    /// static 専用または未実装フェーズのため `None`。
    pub fn volume(&self) -> Option<f64> {
        match self {
            Shape::Sphere { radius } => Some(4.0 / 3.0 * std::f64::consts::PI * radius.powi(3)),
            Shape::Box { half_extents } => {
                Some(8.0 * half_extents.x * half_extents.y * half_extents.z)
            }
            Shape::Plane { .. } => None,
            Shape::Capsule { .. } | Shape::Compound { .. } | Shape::ConvexMesh { .. } => {
                todo!("Phase 2/5 で実装")
            }
        }
    }

    /// 単位質量あたりのローカル慣性テンソル(対角、主軸がローカル軸に一致する形状のみ)。
    /// 設計: docs/10-mechanics/01-rigid-body.md §4.1。
    pub fn unit_mass_inertia_diagonal(&self) -> Vec3 {
        match self {
            Shape::Sphere { radius } => {
                let i = 2.0 / 5.0 * radius * radius;
                Vec3::new(i, i, i)
            }
            Shape::Box { half_extents } => {
                let (a, b, c) = (half_extents.x, half_extents.y, half_extents.z);
                Vec3::new(
                    (b * b + c * c) / 3.0,
                    (a * a + c * c) / 3.0,
                    (a * a + b * b) / 3.0,
                )
            }
            Shape::Plane { .. } => Vec3::ZERO,
            Shape::Capsule { .. } | Shape::Compound { .. } | Shape::ConvexMesh { .. } => {
                todo!("Phase 2/5 で実装")
            }
        }
    }
}

/// 軸並行境界箱。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sphere_volume_matches_formula() {
        let s = Shape::Sphere { radius: 2.0 };
        let expected = 4.0 / 3.0 * std::f64::consts::PI * 8.0;
        assert!((s.volume().unwrap() - expected).abs() < 1e-12);
    }

    #[test]
    fn box_volume_is_product_of_full_extents() {
        let b = Shape::Box {
            half_extents: Vec3::new(0.5, 1.0, 1.5),
        };
        assert!((b.volume().unwrap() - (1.0 * 2.0 * 3.0)).abs() < 1e-12);
    }

    #[test]
    fn sphere_inertia_diagonal_is_isotropic() {
        let s = Shape::Sphere { radius: 3.0 };
        let i = s.unit_mass_inertia_diagonal();
        let expected = 2.0 / 5.0 * 9.0;
        assert!((i.x - expected).abs() < 1e-12);
        assert_eq!(i.x, i.y);
        assert_eq!(i.y, i.z);
    }

    #[test]
    fn cube_inertia_diagonal_is_isotropic() {
        let cube = Shape::Box {
            half_extents: Vec3::new(1.0, 1.0, 1.0),
        };
        let i = cube.unit_mass_inertia_diagonal();
        // 立方体は主慣性モーメントが等方(m/3*(1+1)=2m/3、単位質量なので 2/3)。
        assert!((i.x - 2.0 / 3.0).abs() < 1e-12);
        assert_eq!(i.x, i.y);
        assert_eq!(i.y, i.z);
    }
}
