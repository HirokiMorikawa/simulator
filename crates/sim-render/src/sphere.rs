//! 解析形状: 球。設計 docs/17-rendering/02-path-tracing.md §4「BVH: 三角形メッシュ + 解析形状
//! (球・平面)」。**縮約実装の理由**: 三角形メッシュ・平面はまだ実装しない(白色炉テスト(R1)
//! の検証には孤立した球1個で十分、`sim_mechanics::collision`の球判定と同様の縮約順序)。

use crate::ray::Ray;
use sim_math::Vec3;

#[derive(Clone, Copy, Debug)]
pub struct Sphere {
    pub center: Vec3,
    pub radius: f64,
}

/// 交差記録: ヒット距離・位置・法線(外向き単位ベクトル)。
#[derive(Clone, Copy, Debug)]
pub struct Hit {
    pub t: f64,
    pub point: Vec3,
    pub normal: Vec3,
}

impl Sphere {
    /// レイ-球交差(2次方程式の判別式、`t_min`より大きい最小の実根)。
    pub fn intersect(&self, ray: &Ray, t_min: f64) -> Option<Hit> {
        let oc = ray.origin - self.center;
        let a = ray.direction.dot(ray.direction);
        let b = 2.0 * oc.dot(ray.direction);
        let c = oc.dot(oc) - self.radius * self.radius;
        let discriminant = b * b - 4.0 * a * c;
        if discriminant < 0.0 {
            return None;
        }
        let sqrt_d = discriminant.sqrt();
        let t = {
            let t0 = (-b - sqrt_d) / (2.0 * a);
            if t0 > t_min {
                t0
            } else {
                let t1 = (-b + sqrt_d) / (2.0 * a);
                if t1 > t_min {
                    t1
                } else {
                    return None;
                }
            }
        };
        let point = ray.at(t);
        let normal = (point - self.center).scale(1.0 / self.radius);
        Some(Hit { t, point, normal })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ray_through_center_hits_at_center_minus_radius() {
        let sphere = Sphere {
            center: Vec3::new(0.0, 0.0, -5.0),
            radius: 1.0,
        };
        let ray = Ray::new(Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0));
        let hit = sphere.intersect(&ray, 1e-6).expect("should hit");
        assert!((hit.t - 4.0).abs() < 1e-9);
        assert!((hit.point - Vec3::new(0.0, 0.0, -4.0)).length() < 1e-9);
        assert!((hit.normal - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9);
    }

    #[test]
    fn ray_missing_sphere_returns_none() {
        let sphere = Sphere {
            center: Vec3::new(0.0, 0.0, -5.0),
            radius: 1.0,
        };
        let ray = Ray::new(Vec3::ZERO, Vec3::new(0.0, 1.0, 0.0));
        assert!(sphere.intersect(&ray, 1e-6).is_none());
    }

    #[test]
    fn ray_originating_inside_sphere_hits_far_side_only() {
        let sphere = Sphere {
            center: Vec3::ZERO,
            radius: 1.0,
        };
        let ray = Ray::new(Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0));
        let hit = sphere.intersect(&ray, 1e-6).expect("should hit far side");
        assert!((hit.t - 1.0).abs() < 1e-9);
    }
}
