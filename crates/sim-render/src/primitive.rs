//! シーンを構成する解析形状の総和型。設計 docs/17-rendering/02-path-tracing.md §4
//! 「BVH: 三角形メッシュ + 解析形状(球・平面)」。
//!
//! **縮約実装の理由**: `Sphere`(R1白色炉・R2フレネル等)と`Quad`(R4コーネル
//! ボックスの壁・面光源)の2種のみ。三角形メッシュは引き続き未実装
//! (`quad.rs`モジュールdoc参照)。トレイトオブジェクト(`Box<dyn Shape>`)ではなく
//! enumにするのは、形状の種類が固定で少なく、BVHのリーフから何度も呼ばれる
//! ホットパスであるため動的ディスパッチを避けたいという理由(`sim_mechanics::
//! Shape`が同じ理由でenumを採る慣行に揃える)。

use crate::quad::Quad;
use crate::ray::Ray;
use crate::sphere::{Hit, Sphere};
use sim_math::Vec3;

#[derive(Clone, Copy, Debug)]
pub enum Primitive {
    Sphere(Sphere),
    Quad(Quad),
}

impl Primitive {
    pub fn intersect(&self, ray: &Ray, t_min: f64) -> Option<Hit> {
        match self {
            Primitive::Sphere(s) => s.intersect(ray, t_min),
            Primitive::Quad(q) => q.intersect(ray, t_min),
        }
    }

    /// BVH構築時の分割基準に使う代表点(球は中心、クアッドは対角線の中点)。
    pub fn centroid(&self) -> Vec3 {
        match self {
            Primitive::Sphere(s) => s.center,
            Primitive::Quad(q) => q.corner + (q.edge_u + q.edge_v).scale(0.5),
        }
    }

    /// 軸並行境界ボックス(`min`, `max`)。クアッドは厚みゼロの面になり得る
    /// (軸並行な壁など)が、スラブ法のレイ-AABB判定は退化した軸を
    /// 「方向成分がほぼ0なら範囲内かだけ見る」経路で正しく扱えるため、
    /// 意図的に厚みを持たせない(`bvh.rs`の`intersects_ray`参照)。
    pub fn bounds(&self) -> (Vec3, Vec3) {
        match self {
            Primitive::Sphere(s) => {
                let r = Vec3::new(s.radius, s.radius, s.radius);
                (s.center - r, s.center + r)
            }
            Primitive::Quad(q) => {
                let corners = [
                    q.corner,
                    q.corner + q.edge_u,
                    q.corner + q.edge_v,
                    q.corner + q.edge_u + q.edge_v,
                ];
                let mut min = corners[0];
                let mut max = corners[0];
                for c in &corners[1..] {
                    min = Vec3::new(min.x.min(c.x), min.y.min(c.y), min.z.min(c.z));
                    max = Vec3::new(max.x.max(c.x), max.y.max(c.y), max.z.max(c.z));
                }
                (min, max)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sphere_bounds_and_centroid_match_the_analytic_values() {
        let p = Primitive::Sphere(Sphere {
            center: Vec3::new(1.0, 2.0, 3.0),
            radius: 0.5,
        });
        let (min, max) = p.bounds();
        assert!((min.x - 0.5).abs() < 1e-12 && (max.x - 1.5).abs() < 1e-12);
        assert!((p.centroid() - Vec3::new(1.0, 2.0, 3.0)).length() < 1e-12);
    }

    /// 軸並行クアッドのAABBは、固定軸方向に厚みゼロで潰れる(モジュールdoc参照)。
    #[test]
    fn axis_aligned_quad_bounds_collapse_along_the_fixed_axis() {
        let p = Primitive::Quad(Quad::axis_aligned(2, 5.0, -1.0, 1.0, -2.0, 2.0));
        let (min, max) = p.bounds();
        assert!((min.z - 5.0).abs() < 1e-12 && (max.z - 5.0).abs() < 1e-12);
        assert!((min.x + 1.0).abs() < 1e-12 && (max.x - 1.0).abs() < 1e-12);
        assert!((min.y + 2.0).abs() < 1e-12 && (max.y - 2.0).abs() < 1e-12);
        // 重心は矩形の中心(x,yは範囲の中点、zは固定軸の値そのまま)。
        assert!((p.centroid() - Vec3::new(0.0, 0.0, 5.0)).length() < 1e-12);
    }

    /// `Primitive::intersect`が各形状の`intersect`へ正しく委譲している。
    #[test]
    fn intersect_dispatches_to_the_underlying_shape() {
        let sphere = Primitive::Sphere(Sphere {
            center: Vec3::new(0.0, 0.0, 5.0),
            radius: 1.0,
        });
        let quad = Primitive::Quad(Quad::axis_aligned(2, 5.0, -1.0, 1.0, -1.0, 1.0));
        let ray = Ray::new(Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0));
        // 球は手前の面(z=4)、クアッドはz=5でヒットする。
        assert!((sphere.intersect(&ray, 1e-6).unwrap().t - 4.0).abs() < 1e-12);
        assert!((quad.intersect(&ray, 1e-6).unwrap().t - 5.0).abs() < 1e-12);
    }
}
