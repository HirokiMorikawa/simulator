//! 物理カメラ(薄レンズモデル)。設計 docs/17-rendering/03-materials-camera.md §4.1
//! 「薄レンズモデル: 焦点距離$f$、絞り$N$(F値)から開口半径、被写界深度(ボケ)。
//! レンズ上のサンプリングで焦点外をぼかす」。
//!
//! **縮約実装の理由**: 実際のセンサー・結像距離までは扱わず(設計§4.1の完全な
//! カメラ方程式ではなく)、`focus_distance`(完全に合焦するワールド空間上の距離)+
//! `lens_radius`(開口半径)を直接パラメータとする縮約(Ray Tracing in One Weekend
//! 等の標準的な薄レンズカメラの構成)。ピンホール方向のレイをレンズ円板上の点から
//! 再構成し、焦点距離面上の同じ点(`focus_point`)を通すことでボケを再現する——
//! 露出・シャッター速度・モーションブラーは後続増分。

use sim_math::{SimRng, Vec3};

use crate::ray::Ray;

/// 薄レンズカメラ。`right`・`up`は`forward`と直交する正規直交基底(レンズ円板の
/// サンプリング基底)。
#[derive(Clone, Copy, Debug)]
pub struct Camera {
    pub origin: Vec3,
    pub forward: Vec3,
    pub right: Vec3,
    pub up: Vec3,
    pub lens_radius: f64,
    pub focus_distance: f64,
}

impl Camera {
    /// 絞り$N$(F値)と焦点距離$f$から開口半径を求める(設計§4.1
    /// 「絞りN(F値)から開口半径」、$r=f/(2N)$)。
    pub fn aperture_radius_from_f_number(focal_length: f64, f_number: f64) -> f64 {
        focal_length / (2.0 * f_number)
    }

    /// ピンホール方向`pinhole_direction`(正規化、`forward`との内積が正であること)に
    /// 対応するレイを、レンズ円板上の乱数点から生成する。`lens_radius=0`なら
    /// ピンホールカメラ(ボケ無し)に一致する。
    pub fn generate_ray(&self, pinhole_direction: Vec3, rng: &mut SimRng) -> Ray {
        let t_focus = self.focus_distance / pinhole_direction.dot(self.forward);
        let focus_point = self.origin + pinhole_direction.scale(t_focus);

        let (dx, dy) = rng.unit_disk();
        let lens_offset =
            self.right.scale(dx * self.lens_radius) + self.up.scale(dy * self.lens_radius);
        let ray_origin = self.origin + lens_offset;
        Ray::new(ray_origin, focus_point - ray_origin)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_camera(lens_radius: f64, focus_distance: f64) -> Camera {
        Camera {
            origin: Vec3::ZERO,
            forward: Vec3::new(0.0, 0.0, -1.0),
            right: Vec3::new(1.0, 0.0, 0.0),
            up: Vec3::new(0.0, 1.0, 0.0),
            lens_radius,
            focus_distance,
        }
    }

    /// レンズ半径0(ピンホールカメラ)では、レンズオフセットが常にゼロなので
    /// レイの原点はカメラ原点に厳密に一致する(ボケ無し)。
    #[test]
    fn zero_lens_radius_produces_a_pinhole_ray() {
        let camera = test_camera(0.0, 5.0);
        let mut rng = SimRng::new(1, 1);
        for _ in 0..100 {
            let ray = camera.generate_ray(Vec3::new(0.0, 0.0, -1.0), &mut rng);
            assert_eq!(ray.origin, Vec3::ZERO);
        }
    }

    /// R6: 被写界深度——焦点面(`focus_distance`)ちょうどの点は、レンズ上のどこから
    /// 出たレイでも厳密に同じ1点(`focus_point`)を通る(合焦、ボケ無し)。
    #[test]
    fn rays_converge_exactly_at_the_focus_plane_regardless_of_lens_sample() {
        let focus_distance = 5.0;
        let camera = test_camera(0.3, focus_distance);
        let pinhole_direction = Vec3::new(0.0, 0.0, -1.0);
        let expected_focus_point = camera.origin + pinhole_direction.scale(focus_distance);

        let mut rng = SimRng::new(2, 2);
        for _ in 0..50 {
            let ray = camera.generate_ray(pinhole_direction, &mut rng);
            // レイが焦点距離ちょうどの面(z=-focus_distance)を通る点を求める。
            let t = (expected_focus_point.z - ray.origin.z) / ray.direction.z;
            let point_on_focus_plane = ray.at(t);
            assert!(
                (point_on_focus_plane - expected_focus_point).length() < 1e-9,
                "point_on_focus_plane={point_on_focus_plane:?} \
                 expected_focus_point={expected_focus_point:?}"
            );
        }
    }

    /// R6: 被写界深度——焦点面より`d_obj`だけ奥/手前の面上では、レンズ半径
    /// `lens_radius`のレンズオフセット`lens_offset`から出たレイが、薄レンズ公式
    /// $\text{offset} = \text{lens\_offset}\cdot(1-d_{obj}/f_{dist})$どおりの
    /// 錯乱円(ボケ)を作ることを、既知の(乱数を使わない)特定のレンズサンプル点で
    /// 厳密に確認する(この関係式は薄レンズの相似三角形から直接導出できる幾何学的
    /// 恒等式であり、統計的収束を待つ必要が無い)。
    #[test]
    fn blur_circle_offset_matches_the_thin_lens_similar_triangles_formula() {
        let focus_distance = 5.0;
        let lens_radius = 0.4;
        let camera = test_camera(lens_radius, focus_distance);
        let pinhole_direction = Vec3::new(0.0, 0.0, -1.0);

        // レンズ円板の縁の1点(乱数に依らない既知の値)を手で選ぶ。
        let (dx, dy) = (1.0, 0.0);
        let lens_offset = camera.right.scale(dx * lens_radius) + camera.up.scale(dy * lens_radius);
        let ray_origin = camera.origin + lens_offset;
        let focus_point = camera.origin + pinhole_direction.scale(focus_distance);
        let ray = Ray::new(ray_origin, focus_point - ray_origin);

        for d_obj in [2.0, 5.0, 8.0] {
            // z = -d_obj の面とレイの交点。
            let t = (-d_obj - ray.origin.z) / ray.direction.z;
            let point = ray.at(t);
            let measured_offset = (point - Vec3::new(0.0, 0.0, -d_obj)).length();
            let expected_offset = (lens_offset.length() * (1.0 - d_obj / focus_distance)).abs();
            let rel_err = if expected_offset > 1e-12 {
                (measured_offset - expected_offset).abs() / expected_offset
            } else {
                measured_offset
            };
            assert!(
                rel_err < 1e-9,
                "d_obj={d_obj}: measured_offset={measured_offset} \
                 expected_offset={expected_offset}"
            );
        }
    }

    /// 絞り(F値)と焦点距離から開口半径を求める式$r=f/(2N)$。
    #[test]
    fn aperture_radius_from_f_number_matches_the_formula() {
        let focal_length = 0.05; // 50mm相当。
        let f_number = 2.8;
        let r = Camera::aperture_radius_from_f_number(focal_length, f_number);
        assert!((r - focal_length / (2.0 * f_number)).abs() < 1e-12);
    }
}
