//! パストレーサ本体。設計 docs/17-rendering/02-path-tracing.md §4「レンダリング方程式の
//! モンテカルロ解法」。
//!
//! **縮約実装の理由**: 設計の実装順序(§8)「BVH + 拡散/鏡面BSDF + NEE(基本パストレ)」の
//! うち、本増分は拡散(Lambertian)BSDF + 一様環境光のみを実装する。BVH(加速構造)は
//! シーンが解析球1個のみのこの段階では意味を持たない(線形探索と同義)ため、複数物体の
//! シーンが実際に必要になる増分まで導入を見送る(`sim-fluid::grid_fluid`が固体境界セルを
//! 「必要になってから」導入したのと同じ判断)。NEE(光源の明示サンプル)も、本増分の検証
//! 対象である白色炉テスト(R1、環境光が方向に依らず一様)には不要(環境全体が光源であり、
//! BSDFサンプリングで到達した方向は常に同じ環境放射輝度を返すため、明示的な光源サンプル
//! による分散低減の恩恵がない)なので後続増分に残す。分光(波長ごとのレンダリング)も
//! 後続増分(モノクロの放射輝度スカラーのみを扱う)。
//!
//! シーンは`sim_fluid::grid_fluid`等と同じ「対象を絞って正直に文書化する」縮約で、
//! 孤立した球1個 + 一様環境放射輝度のみを表現する(`Scene`は複数球に拡張可能な形に
//! しておくが、現時点では白色炉テストの検証に必要な最小構成のみ)。

use crate::bsdf::Lambertian;
use crate::ray::Ray;
use crate::sphere::Sphere;
use sim_math::SimRng;

/// 球1個 + 拡散BSDF + 一様環境放射輝度からなる最小シーン。
pub struct Scene {
    pub sphere: Sphere,
    pub material: Lambertian,
    /// 環境放射輝度(方向によらず一定、モノクロスカラー、設計§5「大気散乱等は後続」)。
    pub environment_radiance: f64,
}

impl Scene {
    /// レイを追跡し、方向によらず一様な環境からの放射輝度を経路積分で推定する
    /// (設計§4の`trace`のうち、本増分が実装する範囲——拡散BSDFの再帰的サンプリング
    /// のみ——を抜き出したもの)。`max_depth`はロシアンルーレット無しの単純打切り
    /// (設計§9の値より小さくてよい——白色炉テストは1回の追加バウンスで解析値に一致する
    /// ため、`max_depth`を大きくしても結果は変わらない、モジュールdoc参照)。
    pub fn trace(&self, ray: &Ray, rng: &mut SimRng, max_depth: u32) -> f64 {
        let Some(hit) = self.sphere.intersect(ray, 1e-6) else {
            return self.environment_radiance;
        };
        if max_depth == 0 {
            return 0.0; // 打ち切り(エネルギーを捨てる、通常のロシアンルーレット無し打切りと同じ)。
        }

        let (direction, pdf) = self.material.sample(hit.normal, rng);
        let cos_theta = direction.dot(hit.normal);
        let bsdf = self.material.eval();
        let next_ray = Ray::new(hit.point, direction);
        let incoming = self.trace(&next_ray, rng, max_depth - 1);
        incoming * bsdf * cos_theta / pdf
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_math::Vec3;

    /// R1: 白色炉テスト(docs/21-verification/01-analytic-tests.md、
    /// docs/17-rendering/02-path-tracing.md §7)。完全拡散面(ρ=1)が一様環境放射輝度の
    /// 中に置かれると、その表面から見た放射輝度は環境放射輝度と厳密に一致する
    /// (エネルギー保存・BSDF正規化の検証)。
    ///
    /// 解析的根拠: Lambertian BSDFはコサイン重み付き半球サンプリング
    /// (pdf=cosθ/π)と対にすると、`bsdf*cosθ/pdf = albedo`が方向によらず恒等的に
    /// 成り立つ(重要度サンプリングの完全な相殺)。球が孤立している(凸形状は自身を
    /// 決して自己遮蔽しない)ため、サンプルされた方向は必ず環境へ抜ける。したがって
    /// albedo=1のときは追加のバウンス1回で分散ゼロのまま解析値と厳密に一致する
    /// (統計的収束を待つ必要がない——ここが「白色炉」の名の由来である「エネルギーが
    /// 一切失われない」ことの直接的な帰結)。
    #[test]
    fn r1_white_furnace_diffuse_surface_matches_background_radiance_exactly() {
        let environment_radiance = 3.7; // 任意の一様環境放射輝度。
        let scene = Scene {
            sphere: Sphere {
                center: Vec3::new(0.0, 0.0, -5.0),
                radius: 1.0,
            },
            material: Lambertian { albedo: 1.0 },
            environment_radiance,
        };
        let camera_origin = Vec3::ZERO;
        let mut rng = SimRng::new(1, 1);

        for i in 0..20 {
            // 球面上の様々な点を狙う(法線方向が毎回異なることを確認する)複数レイ。
            let angle = i as f64 * 0.05;
            let target = Vec3::new(angle.sin() * 0.8, angle.cos() * 0.8, -5.0);
            let ray = Ray::new(camera_origin, target - camera_origin);
            let radiance = scene.trace(&ray, &mut rng, 4);
            let rel_err = (radiance - environment_radiance).abs() / environment_radiance;
            assert!(
                rel_err < 0.001,
                "R1 white furnace failed at ray {i}: radiance={radiance} \
                 environment_radiance={environment_radiance} rel_err={rel_err}"
            );
        }
    }

    /// アルベドが1未満の場合、放射輝度は`albedo*environment_radiance`(エネルギー保存、
    /// 白色炉テストの一般形)に厳密に一致する。
    #[test]
    fn sub_unity_albedo_scales_radiance_by_albedo_exactly() {
        let environment_radiance = 2.0;
        let albedo = 0.6;
        let scene = Scene {
            sphere: Sphere {
                center: Vec3::new(0.0, 0.0, -5.0),
                radius: 1.0,
            },
            material: Lambertian { albedo },
            environment_radiance,
        };
        let ray = Ray::new(Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0));
        let mut rng = SimRng::new(2, 2);
        let radiance = scene.trace(&ray, &mut rng, 4);
        let expected = albedo * environment_radiance;
        let rel_err = (radiance - expected).abs() / expected;
        assert!(rel_err < 1e-9, "radiance={radiance} expected={expected}");
    }

    /// レイが球を外れた場合は環境放射輝度そのものを返す。
    #[test]
    fn ray_missing_the_sphere_returns_environment_radiance() {
        let scene = Scene {
            sphere: Sphere {
                center: Vec3::new(0.0, 0.0, -5.0),
                radius: 1.0,
            },
            material: Lambertian { albedo: 1.0 },
            environment_radiance: 9.0,
        };
        let ray = Ray::new(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0));
        let mut rng = SimRng::new(3, 3);
        assert_eq!(scene.trace(&ray, &mut rng, 4), 9.0);
    }
}
