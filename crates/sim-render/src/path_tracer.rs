//! パストレーサ本体。設計 docs/17-rendering/02-path-tracing.md §4「レンダリング方程式の
//! モンテカルロ解法」。
//!
//! **縮約実装の理由**: 設計の実装順序(§8)「BVH + 拡散/鏡面BSDF + NEE(基本パストレ)」の
//! うち、本増分は拡散(Lambertian)+誘電体(`Dielectric`)BSDF + 一様環境光 + 複数物体の
//! シーン(`SceneObject`のリスト、線形探索で最近傍交差を選ぶ)を実装する。BVH(加速
//! 構造)自体は、物体数が線形探索では性能上問題になる規模のシーンが実際に必要になる
//! 増分まで導入を見送る(線形探索は正しさは損なわないため、`sim-fluid::grid_fluid`が
//! 固体境界セルを「必要になってから」導入したのと同じ判断)。NEE(光源の明示サンプル)
//! も、本増分の検証対象(環境光が方向に依らず一様、専用の離散光源が無い)には不要
//! (環境全体が光源であり、BSDFサンプリングで到達した方向は常に同じ環境放射輝度を
//! 返すため、明示的な光源サンプルによる分散低減の恩恵がない)なので、離散光源が実際に
//! 追加される増分まで残す。分光(波長ごとのレンダリング)も後続増分(モノクロの放射
//! 輝度スカラーのみを扱う)。
//!
//! シーンは`sim_fluid::grid_fluid`等と同じ「対象を絞って正直に文書化する」縮約で、
//! 複数の解析球 + 一様環境放射輝度のみを表現する(平面・三角形メッシュは未実装)。

use crate::bsdf::{Dielectric, Lambertian};
use crate::ray::Ray;
use crate::sphere::{Hit, Sphere};
use sim_math::SimRng;

/// このシーンが表現できるBSDF(`bsdf`モジュールdoc「縮約実装の理由」参照、
/// 金属・粗面透過は未実装)。
#[derive(Clone, Copy, Debug)]
pub enum Material {
    Lambertian(Lambertian),
    Dielectric(Dielectric),
}

/// シーン中の1物体(球 + BSDF)。
#[derive(Clone, Copy, Debug)]
pub struct SceneObject {
    pub sphere: Sphere,
    pub material: Material,
}

/// 複数の球 + BSDF + 一様環境放射輝度からなるシーン。
pub struct Scene {
    pub objects: Vec<SceneObject>,
    /// 環境放射輝度(方向によらず一定、モノクロスカラー、設計§5「大気散乱等は後続」)。
    pub environment_radiance: f64,
}

impl Scene {
    /// 全物体を線形探索し、最も近い交差(`t`最小)を返す(モジュールdoc「BVHは
    /// 実際に必要になるまで見送る」参照)。`pub(crate)`はテストから直接この幾何選択
    /// ロジックだけを検証するため(`trace`経由だと再帰的なBSDFサンプリングの結果と
    /// 混ざり、どちらの物体が選ばれたか単体では判別しづらい)。
    pub(crate) fn closest_hit(&self, ray: &Ray, t_min: f64) -> Option<(Hit, &Material)> {
        self.objects
            .iter()
            .filter_map(|obj| {
                obj.sphere
                    .intersect(ray, t_min)
                    .map(|hit| (hit, &obj.material))
            })
            .min_by(|(a, _), (b, _)| a.t.total_cmp(&b.t))
    }

    /// レイを追跡し、方向によらず一様な環境からの放射輝度を経路積分で推定する
    /// (設計§4の`trace`のうち、本増分が実装する範囲——拡散/誘電体BSDFの再帰的
    /// サンプリングのみ——を抜き出したもの)。`max_depth`はロシアンルーレット無しの
    /// 単純打切り(設計§9の値より小さくてよい——白色炉テストは1回の追加バウンスで
    /// 解析値に一致するため、`max_depth`を大きくしても結果は変わらない、モジュール
    /// doc参照)。
    pub fn trace(&self, ray: &Ray, rng: &mut SimRng, max_depth: u32) -> f64 {
        let Some((hit, material)) = self.closest_hit(ray, 1e-6) else {
            return self.environment_radiance;
        };
        if max_depth == 0 {
            return 0.0; // 打ち切り(エネルギーを捨てる、通常のロシアンルーレット無し打切りと同じ)。
        }

        match material {
            Material::Lambertian(lambertian) => {
                let (direction, pdf) = lambertian.sample(hit.normal, rng);
                let cos_theta = direction.dot(hit.normal);
                let bsdf = lambertian.eval();
                let next_ray = Ray::new(hit.point, direction);
                let incoming = self.trace(&next_ray, rng, max_depth - 1);
                incoming * bsdf * cos_theta / pdf
            }
            Material::Dielectric(dielectric) => {
                // オリエンテーション補正: 外向き法線に対してレイが向かってくる側
                // (d・n<0)なら「入射」(真空→本材質)、逆なら球の内部から「出射」
                // (本材質→真空)。法線を出射側基準に反転し、n1/n2を入れ替える。
                let entering = ray.direction.dot(hit.normal) < 0.0;
                let (outward_normal, n1, n2) = if entering {
                    (hit.normal, 1.0, dielectric.ior)
                } else {
                    (-hit.normal, dielectric.ior, 1.0)
                };
                let eta = n1 / n2;
                let cos_theta_i = -ray.direction.dot(outward_normal);
                let refracted = Dielectric::refract(ray.direction, outward_normal, eta);
                let reflectance = match refracted {
                    None => 1.0, // 全反射(TIR)。
                    Some(_) => Dielectric::reflectance_between(cos_theta_i, n1, n2),
                };
                let is_reflected = rng.next_f64() < reflectance;
                match refracted {
                    Some(direction) if !is_reflected => {
                        let next_ray = Ray::new(hit.point, direction);
                        // 放射輝度は界面通過ごとに(n1/n2)^2でスケールする(幾何光学の
                        // 放射輝度不変量 L/n^2 = const、標準的なBTDFの因子)。ガラス球を
                        // 1回貫通する経路では入射時のeta=1/iorと出射時のeta=iorの二乗が
                        // 厳密に相殺する(1/ior^2 * ior^2 = 1)ため、この因子を含めても
                        // 白色炉テストの厳密一致(誤差ゼロ)が保たれる。
                        self.trace(&next_ray, rng, max_depth - 1) * eta * eta
                    }
                    _ => {
                        let direction = Dielectric::reflect(ray.direction, outward_normal);
                        let next_ray = Ray::new(hit.point, direction);
                        self.trace(&next_ray, rng, max_depth - 1)
                    }
                }
            }
        }
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
            objects: vec![SceneObject {
                sphere: Sphere {
                    center: Vec3::new(0.0, 0.0, -5.0),
                    radius: 1.0,
                },
                material: Material::Lambertian(Lambertian { albedo: 1.0 }),
            }],
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
            objects: vec![SceneObject {
                sphere: Sphere {
                    center: Vec3::new(0.0, 0.0, -5.0),
                    radius: 1.0,
                },
                material: Material::Lambertian(Lambertian { albedo }),
            }],
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
            objects: vec![SceneObject {
                sphere: Sphere {
                    center: Vec3::new(0.0, 0.0, -5.0),
                    radius: 1.0,
                },
                material: Material::Lambertian(Lambertian { albedo: 1.0 }),
            }],
            environment_radiance: 9.0,
        };
        let ray = Ray::new(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0));
        let mut rng = SimRng::new(3, 3);
        assert_eq!(scene.trace(&ray, &mut rng, 4), 9.0);
    }

    /// 白色炉テスト(R1)の誘電体版: 吸収の無い誘電体球(ガラス相当)が一様環境放射
    /// 輝度の中に置かれると、反射・屈折のどちらの経路をたどっても最終的に環境へ
    /// 抜けるため、放射輝度は環境放射輝度と厳密に一致する(統計誤差ゼロ)。
    ///
    /// 解析的根拠: 反射/屈折は物理的な確率(フレネル反射率R/透過率1-R)に比例した
    /// 確率でサンプリングされ、選ばれた経路の重みは追加のR/(1-R)による除算を
    /// 伴わない(サンプリング確率と物理確率が一致するため相殺する、Lambertianの
    /// bsdf/pdf相殺と同じ構造)。屈折時の放射輝度スケール因子$(n_1/n_2)^2$は、
    /// 球に入る際($1/\text{ior}$)と出る際($\text{ior}$)とで厳密に相殺する
    /// (モジュールdoc参照)。臨界角を超えるグレージング角は本テストでは避ける
    /// (全反射で球内部に閉じ込められ`max_depth`で打ち切られるため、厳密一致が
    /// 崩れる既知の限界、後続増分でロシアンルーレット等の対応を検討)。
    #[test]
    fn dielectric_furnace_test_non_absorbing_glass_sphere_matches_background_radiance_exactly() {
        let environment_radiance = 4.2;
        let scene = Scene {
            objects: vec![SceneObject {
                sphere: Sphere {
                    center: Vec3::new(0.0, 0.0, -5.0),
                    radius: 1.0,
                },
                material: Material::Dielectric(Dielectric { ior: 1.5 }),
            }],
            environment_radiance,
        };
        let camera_origin = Vec3::ZERO;
        let mut rng = SimRng::new(4, 4);

        for i in 0..30 {
            // 臨界角(≈41.8°)より十分小さい角度に留め、全反射による無限内部反射を
            // 避ける(モジュールdoc「後続増分で対応を検討」参照)。
            let angle = i as f64 * 0.02;
            let target = Vec3::new(angle.sin() * 0.5, angle.cos() * 0.5, -5.0);
            let ray = Ray::new(camera_origin, target - camera_origin);
            let radiance = scene.trace(&ray, &mut rng, 8);
            let rel_err = (radiance - environment_radiance).abs() / environment_radiance;
            assert!(
                rel_err < 1e-6,
                "dielectric furnace test failed at ray {i}: radiance={radiance} \
                 environment_radiance={environment_radiance} rel_err={rel_err}"
            );
        }
    }

    /// 複数物体: 2つの球が同一レイ上に重なる場合、`closest_hit`が正しく手前の
    /// (`t`が小さい)物体を選ぶこと。`trace`経由だとBSDFの再帰的サンプリングの結果
    /// (どちらの物体に当たっても最終的にほぼ同じ放射輝度に収束し得る、特に両方が
    /// 拡散白色だと区別できない)と混ざってしまうため、`closest_hit`を直接検証する
    /// (線形探索による最近傍交差選択そのものの配線確認、モジュールdoc参照)。
    #[test]
    fn closest_hit_picks_the_nearer_object_when_two_spheres_overlap_along_the_ray() {
        let near = SceneObject {
            sphere: Sphere {
                center: Vec3::new(0.0, 0.0, -3.0),
                radius: 1.0,
            },
            material: Material::Lambertian(Lambertian { albedo: 0.2 }),
        };
        let far = SceneObject {
            sphere: Sphere {
                center: Vec3::new(0.0, 0.0, -8.0),
                radius: 1.0,
            },
            material: Material::Lambertian(Lambertian { albedo: 0.9 }),
        };
        let scene = Scene {
            objects: vec![near, far],
            environment_radiance: 1.0,
        };

        // 両方の球の中心を貫く一直線: 手前の球(t≈2)が奥の球(t≈7)より先にヒットする。
        let ray = Ray::new(Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0));
        let (hit, material) = scene
            .closest_hit(&ray, 1e-6)
            .expect("should hit the near sphere");
        assert!(
            (hit.t - 2.0).abs() < 1e-9,
            "expected to hit the near sphere at t=2.0, got t={}",
            hit.t
        );
        match material {
            Material::Lambertian(lambertian) => {
                assert_eq!(
                    lambertian.albedo, 0.2,
                    "should pick the near sphere's material"
                )
            }
            Material::Dielectric(_) => panic!("expected Lambertian"),
        }

        // 物体を除いた登録順で試しても(奥→手前)、結果は変わらず手前が選ばれる
        // (線形探索は登録順に依存しない`t`最小選択であることの確認)。
        let scene_reordered = Scene {
            objects: vec![far, near],
            environment_radiance: 1.0,
        };
        let (hit_reordered, _) = scene_reordered
            .closest_hit(&ray, 1e-6)
            .expect("should still hit the near sphere");
        assert!((hit_reordered.t - 2.0).abs() < 1e-9);

        // レイが両方の球を外れる場合は`None`。
        let missing_ray = Ray::new(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0));
        assert!(scene.closest_hit(&missing_ray, 1e-6).is_none());
    }
}
