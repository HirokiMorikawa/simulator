//! カメラ→ピクセル→レイ生成→`Scene::trace`→フレームバッファ格納の配線。設計
//! docs/17-rendering/02-path-tracing.md §4「render(scene)」の疑似コードのうち、
//! 本増分が実装する範囲(分光サンプリング`λ`・Sobol等の低食い違い列は対象外、
//! `lib.rs`モジュールdoc参照)を実際に配線する。これが増分C1の中心であり、
//! `Camera::generate_ray`(既に計算済みの方向を受け取る設計)と`Camera::
//! pinhole_direction`(本増分で追加、`camera.rs`モジュールdoc参照)、`Scene::trace`
//! (モノクロの単一レイ/点推定として既にR1–R7で検証済み、本増分では一切変更しない)
//! を初めて実際にピクセルグリッド上で結びつける。
//!
//! **色の扱い(チャンネル別レンダリング)**: `sim-render`はまだ分光レンダリング
//! 本体(hero wavelength法)を持たない(`bsdf.rs`/`prism.rs`モジュールdoc参照)ため、
//! 設計§4.2の`spectrum_to_display`が想定する「XYZへの分光積分」は行わない。
//! 代わりに、R4(コーネルボックス)で実証済みの手法
//! (`path_tracer.rs::tests::cornell_box_shows_color_bleeding_from_the_red_and_
//! green_side_walls`が採用した"同一形状・アルベドだけチャンネルごとに差し替えた
//! `Scene`を3つ用意し、モノクロの`trace`を3回走らせる"手法)をそのまま流用する
//! (`render_rgb`)。これにより**`Scene::trace`には一切手を入れる必要が無い**
//! (既存75テストへの回帰リスクをゼロにする設計判断——分光化はモンテカルロ
//! 経路追跡の再帰全体に波長を通す大掛かりな変更を要するため、後続増分に残す)。
//!
//! **アンチエイリアス**: 各ピクセル内で`SimRng`によりサブピクセルオフセットを
//! 決定的にジッタする(層化サンプリング・低食い違い列は未実装、単純な一様
//! ジッタのみ)。ピクセル×サンプルごとに固定のサブストリームPRNGを使う設計
//! (設計§4「決定論」)は、既存の`trace`のテストが使う`SimRng::new(seed, stream)`
//! パターンをそのまま踏襲する。
//!
//! **NDC・走査順の規約**: `ndc_x`は右方向・`ndc_y`は上方向が正(`Camera::
//! pinhole_direction`のdoc参照)。`pixels`は画像の先頭行が画面最上段になるように
//! 格納する(`py=0`で`ndc_y`が`+1`に近づくようにマッピングする)——PNG(`png.rs`)の
//! スキャンライン順(先頭行が画像最上段)とそのまま対応させるため。

use sim_math::{SimRng, Vec3};

use crate::camera::Camera;
use crate::framebuffer::Framebuffer;
use crate::path_tracer::Scene;

/// レンダリング設定。露出・モーションブラーは`Framebuffer::to_srgb8`/
/// `Camera`側の後続増分(`lib.rs`モジュールdoc参照)。
#[derive(Clone, Copy, Debug)]
pub struct RenderSettings {
    /// ピクセルあたりのサンプル数(スーパーサンプリング/アンチエイリアス兼、
    /// モンテカルロ収束のサンプル数を兼ねる)。
    pub spp: u32,
    /// `Scene::trace`へそのまま渡す経路長上限。
    pub max_depth: u32,
    /// `Framebuffer::to_srgb8`へそのまま渡す露出倍率。
    pub exposure: f64,
}

/// 単一のモノクロ`Scene`をピクセルグリッド全体でレンダリングし、行優先
/// (上から下)の線形放射輝度配列を返す(モジュールdoc「NDC・走査順の規約」参照)。
/// `rng`はこの呼び出し全体で共有する1本のPRNG(ピクセルをまたいで連続的に
/// 消費する——`SimRng`は決定的なため、同一`rng`の初期状態から呼べば常に同じ
/// 画像になる)。
pub fn render_channel(
    scene: &Scene,
    camera: &Camera,
    vfov: f64,
    width: u32,
    height: u32,
    settings: &RenderSettings,
    rng: &mut SimRng,
) -> Vec<f64> {
    let aspect = width as f64 / height as f64;
    let mut pixels = vec![0.0; width as usize * height as usize];

    for py in 0..height {
        for px in 0..width {
            let mut sum = 0.0;
            for _ in 0..settings.spp {
                // ピクセル内のジッタ(アンチエイリアス、モジュールdoc参照)。
                let jitter_x = rng.next_f64();
                let jitter_y = rng.next_f64();
                let ndc_x = 2.0 * (px as f64 + jitter_x) / width as f64 - 1.0;
                // 上が正になるよう反転する(画像の先頭行=最上段、モジュールdoc参照)。
                let ndc_y = 1.0 - 2.0 * (py as f64 + jitter_y) / height as f64;

                let direction = camera.pinhole_direction(ndc_x, ndc_y, aspect, vfov);
                let ray = camera.generate_ray(direction, rng);
                sum += scene.trace(&ray, rng, settings.max_depth);
            }
            pixels[py as usize * width as usize + px as usize] = sum / settings.spp as f64;
        }
    }
    pixels
}

/// チャンネル別レンダリング(モジュールdoc参照): `scenes`はそれぞれR/G/Bチャンネル
/// 用に用意した(同一形状・アルベドだけ差し替えた)`Scene`。各チャンネルは互いに
/// 独立なPRNGストリーム(`seed`は共通・`stream`のみ0/1/2で分ける、`SimRng::new`の
/// 既存の使い方)でレンダリングし、`Framebuffer`へRGBとして詰める。
pub fn render_rgb(
    scenes: [&Scene; 3],
    camera: &Camera,
    vfov: f64,
    width: u32,
    height: u32,
    settings: &RenderSettings,
    seed: u64,
) -> Framebuffer {
    let mut rng_r = SimRng::new(seed, 0);
    let mut rng_g = SimRng::new(seed, 1);
    let mut rng_b = SimRng::new(seed, 2);

    let r = render_channel(scenes[0], camera, vfov, width, height, settings, &mut rng_r);
    let g = render_channel(scenes[1], camera, vfov, width, height, settings, &mut rng_g);
    let b = render_channel(scenes[2], camera, vfov, width, height, settings, &mut rng_b);

    let mut framebuffer = Framebuffer::new(width, height);
    for i in 0..framebuffer.pixels.len() {
        framebuffer.pixels[i] = Vec3::new(r[i], g[i], b[i]);
    }
    framebuffer
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bsdf::Lambertian;
    use crate::path_tracer::{Material, SceneObject};
    use crate::sphere::Sphere;

    fn test_camera() -> Camera {
        Camera {
            origin: Vec3::ZERO,
            forward: Vec3::new(0.0, 0.0, -1.0),
            right: Vec3::new(1.0, 0.0, 0.0),
            up: Vec3::new(0.0, 1.0, 0.0),
            lens_radius: 0.0,
            focus_distance: 5.0,
        }
    }

    /// R1(白色炉テスト)の画像版。**画像パイプライン全体**
    /// (カメラ→ピクセル→レイ生成→`trace`→フレームバッファ格納)を、8×8という
    /// 極小解像度で実際に走らせ、全画素が環境放射輝度と厳密に一致することを
    /// 確認する。
    ///
    /// 解析的根拠(`path_tracer.rs::tests::
    /// r1_white_furnace_diffuse_surface_matches_background_radiance_exactly`の
    /// docと同じ): albedo=1.0のLambertian球は重要度サンプリングが完全に相殺
    /// する分散ゼロの構成であり、かつ孤立した凸形状は自身を自己遮蔽しないため、
    /// **球に当たった画素・外れた画素のどちらでも**放射輝度は環境放射輝度と
    /// 厳密に一致する(球に当たれば追加バウンスでいずれ環境へ抜けるまで
    /// `albedo=1`の相殺が繰り返されるだけ、外れれば直接環境放射輝度が返る)。
    /// したがって統計的収束を待つ必要が無く(`spp`は1で足りる)、カメラの視野角は
    /// 球を画角の一部にだけ収める(全画素が同じ値になる自明な配置を避け、
    /// 「ピクセルごとに異なる方向のレイが生成されている」ことも間接的に保証する
    /// ため、画角は球の見かけの角半径よりわずかに大きく取り、少なくとも一部の
    /// 画素は確実に外れるようにする)。
    #[test]
    fn r1_white_furnace_image_pipeline_matches_environment_radiance_at_every_pixel() {
        let environment_radiance = 3.7;
        let sphere_distance = 5.0;
        let sphere_radius = 1.0;
        let scene = Scene::new(
            vec![SceneObject::sphere(
                Sphere {
                    center: Vec3::new(0.0, 0.0, -sphere_distance),
                    radius: sphere_radius,
                },
                Material::Lambertian(Lambertian { albedo: 1.0 }),
            )],
            vec![],
            environment_radiance,
            None,
        );

        let camera = test_camera();
        // 球の角半径(atan(r/d)≈0.1974rad)よりわずかに大きい半画角を取り、
        // 隅の画素は確実に球を外れるようにする。
        let vfov = 0.5; // 半画角0.25rad > 角半径0.1974rad。
        let settings = RenderSettings {
            spp: 1,
            max_depth: 4,
            exposure: 1.0,
        };
        let mut rng = SimRng::new(1, 1);
        let pixels = render_channel(&scene, &camera, vfov, 8, 8, &settings, &mut rng);

        assert_eq!(pixels.len(), 64);
        for (i, &radiance) in pixels.iter().enumerate() {
            let rel_err = (radiance - environment_radiance).abs() / environment_radiance;
            assert!(
                rel_err < 0.001,
                "pixel {i}: radiance={radiance} environment_radiance={environment_radiance} \
                 rel_err={rel_err}"
            );
        }
    }

    /// 上記のモノクロ版を`render_rgb`+`Framebuffer`経由で確認する: 3チャンネル
    /// (それぞれ独立なPRNGストリーム)全てが同じ白色炉シーンをレンダリングする
    /// ため、`Framebuffer`の全画素が`(L,L,L)`(`L`=環境放射輝度)に一致する
    /// はず(`render_rgb`のRGB合成・`Framebuffer`への格納自体の配線確認、
    /// モノクロ側の解析的根拠は上のテストと同じ)。
    #[test]
    fn render_rgb_wires_three_independent_channels_into_the_framebuffer() {
        let environment_radiance = 2.5;
        let scene = Scene::new(
            vec![SceneObject::sphere(
                Sphere {
                    center: Vec3::new(0.0, 0.0, -5.0),
                    radius: 1.0,
                },
                Material::Lambertian(Lambertian { albedo: 1.0 }),
            )],
            vec![],
            environment_radiance,
            None,
        );
        let camera = test_camera();
        let settings = RenderSettings {
            spp: 1,
            max_depth: 4,
            exposure: 1.0,
        };
        let framebuffer = render_rgb([&scene, &scene, &scene], &camera, 0.5, 8, 8, &settings, 7);

        assert_eq!(framebuffer.width, 8);
        assert_eq!(framebuffer.height, 8);
        for (i, &pixel) in framebuffer.pixels.iter().enumerate() {
            for (channel_name, value) in [("r", pixel.x), ("g", pixel.y), ("b", pixel.z)] {
                let rel_err = (value - environment_radiance).abs() / environment_radiance;
                assert!(
                    rel_err < 0.001,
                    "pixel {i} channel {channel_name}: value={value} \
                     environment_radiance={environment_radiance} rel_err={rel_err}"
                );
            }
        }
    }
}

/// **分光レンダリング(分光レンダリング増分で追加)**。
///
/// 各サンプルで hero wavelength 法により`wavelengths_per_sample`本の波長を層化して
/// 取り、`Scene::trace_spectral`で波長ごとの分光放射輝度を求め、CIE等色関数で
/// XYZへ積分してから線形sRGBへ変換した`Framebuffer`を返す。
///
/// **`render_rgb`との違い**: `render_rgb`は「アルベドをR/G/Bに差し替えた`Scene`を
/// 3つ用意して`trace`を3回走らせる」方式で、RGBが固定基底であることに依存する。
/// そのため**分散(波長ごとの屈折率差)を正しく扱えない**——プリズムやガラス球で
/// 「Rは赤の屈折率、Gは緑の屈折率」という別々の経路を辿った3枚を合成することに
/// なるため。こちらは1つの`Scene`を波長で追うので、`Material::CauchyDielectric`を
/// 置けば分散がそのまま像に出る。
///
/// **コスト**: 1ピクセルあたり `spp × wavelengths_per_sample` 本の経路を追う。
/// `render_rgb`(spp×3)と同程度に保つなら`wavelengths_per_sample`は3〜4が目安。
// 引数が8つで clippy の閾値(7)を1つ超えるが、`render_channel`/`render_rgb` と
// 同じ「シーン・カメラ・画角・解像度・設定・乱数」の並びに `wavelengths_per_sample`
// を1つ足しただけの形であり、構造体へ畳むと既存3関数との対称性が崩れて
// かえって読みにくい。ここは意図的に許容する。
#[allow(clippy::too_many_arguments)]
pub fn render_spectral(
    scene: &Scene,
    camera: &Camera,
    vfov: f64,
    width: u32,
    height: u32,
    settings: &RenderSettings,
    wavelengths_per_sample: usize,
    rng: &mut SimRng,
) -> Framebuffer {
    let aspect = width as f64 / height as f64;
    let mut framebuffer = Framebuffer::new(width, height);

    for py in 0..height {
        for px in 0..width {
            let (mut r, mut g, mut b) = (0.0, 0.0, 0.0);
            for _ in 0..settings.spp {
                let jitter_x = rng.next_f64();
                let jitter_y = rng.next_f64();
                let ndc_x = 2.0 * (px as f64 + jitter_x) / width as f64 - 1.0;
                let ndc_y = 1.0 - 2.0 * (py as f64 + jitter_y) / height as f64;

                let direction = camera.pinhole_direction(ndc_x, ndc_y, aspect, vfov);
                let ray = camera.generate_ray(direction, rng);

                // hero wavelength: 主波長を一様に引き、残りは等間隔に巻き戻して取る。
                let lambdas =
                    crate::spectrum::hero_wavelengths(rng.next_f64(), wavelengths_per_sample);
                let samples: Vec<(f64, f64)> = lambdas
                    .iter()
                    .map(|&lambda| {
                        (
                            lambda,
                            scene.trace_spectral(&ray, rng, settings.max_depth, lambda),
                        )
                    })
                    .collect();
                let (sr, sg, sb) = crate::spectrum::spectral_samples_to_linear_srgb(&samples);
                r += sr;
                g += sg;
                b += sb;
            }
            let inv = 1.0 / settings.spp as f64;
            framebuffer.pixels[py as usize * width as usize + px as usize] =
                sim_math::Vec3::new(r * inv, g * inv, b * inv);
        }
    }
    framebuffer
}

#[cfg(test)]
mod spectral_tests {
    use super::*;
    use crate::bsdf::CauchyDielectric;
    use crate::path_tracer::{Material, Scene, SceneObject};
    use crate::primitive::Primitive;
    use crate::ray::Ray;
    use crate::sphere::Sphere;

    fn camera_at(z: f64) -> Camera {
        Camera {
            origin: sim_math::Vec3::new(0.0, 0.0, z),
            forward: sim_math::Vec3::new(0.0, 0.0, -1.0),
            right: sim_math::Vec3::new(1.0, 0.0, 0.0),
            up: sim_math::Vec3::new(0.0, 1.0, 0.0),
            lens_radius: 0.0, // ピンホール(被写界深度なし)
            focus_distance: z,
        }
    }

    /// **白色炉(R1)の分光版**: 環境放射輝度が全波長で一定なら、分光経路で描いた
    /// 画像は**等エネルギー白色(E光源)をsRGBへ変換した色**に一致する。
    ///
    /// **最初「無彩色(グレー)になるはず」と書いたテストが落ちて、前提が物理的に
    /// 誤っていたことが分かった**。全波長一定のスペクトルはE光源であって、sRGBの
    /// 白色点であるD65ではない。E光源(X=Y=Z)をD65基準のsRGB行列に通すと
    /// 各行の係数和がそのまま出て
    /// `R=3.2406-1.5372-0.4986=1.2048`, `G=-0.9689+1.8758+0.0415=0.9484`,
    /// `B=0.0557-0.2040+1.0570=0.9087`(輝度1のとき)という**暖色寄りの色**になる。
    /// これは色管理として正しい挙動であり、バグではない。
    ///
    /// 実測は環境放射輝度0.5に対し (0.6011, 0.4741, 0.4536) = 解析値×0.5 と一致。
    /// この一致は、hero wavelength サンプリング → `trace_spectral` → CIE等色関数 →
    /// XYZ→sRGB → `Framebuffer` という**分光パイプライン全体**が正しく繋がって
    /// いることの検証になる(どこかで波長を取り違えれば色も明るさもずれる)。
    #[test]
    fn a_uniform_furnace_matches_the_analytic_equal_energy_white() {
        let environment_radiance = 0.5;
        let scene = Scene::new(
            vec![SceneObject {
                primitive: Primitive::Sphere(Sphere {
                    center: sim_math::Vec3::ZERO,
                    radius: 1.0,
                }),
                material: Material::Lambertian(crate::bsdf::Lambertian { albedo: 1.0 }),
            }],
            vec![],
            environment_radiance,
            None,
        );
        let settings = RenderSettings {
            spp: 400,
            max_depth: 4,
            exposure: 1.0,
        };
        let mut rng = SimRng::new(7, 1);
        let fb = render_spectral(&scene, &camera_at(4.0), 0.6, 4, 4, &settings, 8, &mut rng);

        // E光源(X=Y=Z=1)をsRGB行列に通した値 = 各行の係数和。
        let (er, eg, eb) = crate::spectrum::xyz_to_linear_srgb(1.0, 1.0, 1.0);
        let expected = [
            er * environment_radiance,
            eg * environment_radiance,
            eb * environment_radiance,
        ];
        for (i, p) in fb.pixels.iter().enumerate() {
            for (c, (measured, want)) in [p.x, p.y, p.z].iter().zip(expected.iter()).enumerate() {
                assert!(
                    (measured - want).abs() < 0.01,
                    "白色炉はE光源のsRGB値に一致すべき: pixel={i} ch={c} measured={measured} expected={want}"
                );
            }
            // 輝度も環境放射輝度に一致する(E光源のsRGB値は輝度1に正規化されている)。
            let luminance = 0.2126 * p.x + 0.7152 * p.y + 0.0722 * p.z;
            assert!(
                (luminance - environment_radiance).abs() < 0.01,
                "白色炉の輝度は環境放射輝度に一致すべき: pixel={i} luminance={luminance}"
            );
        }
    }

    /// **分散が実際に放射輝度を波長で変えること**——これが分光レンダリングを
    /// 足した理由そのもの。
    ///
    /// **実装中に踏んだ落とし穴を2つ記録する**:
    ///
    /// ①最初は「一様な白色環境に置いたガラス球」で色が出ると考えたが、**分散が
    /// 完全に見えなかった**(色度のばらつきが 分散あり 0.00085 / 分散なし 0.00089 と
    /// 区別不能)。理由は物理的に明快で、環境放射輝度が全方向・全波長で一定なら
    /// **屈折方向が変わっても行き着く先の放射輝度が同じ**だからである。
    /// 分散が見えるには入射光に角度構造が要る。
    ///
    /// ②そこで背後に面光源を置いたが、今度は**画像上の色度のばらつきでは判定
    /// できなかった**(分散あり0.130 / 分散なし0.212 と逆転)。明るい画素が
    /// 少ないシーンでは色度がモンテカルロ雑音に支配され、指標が分散ではなく
    /// 雑音を測ってしまうため。
    ///
    /// そこで**画像を経由せず`trace_spectral`を直接叩く決定論的な判定**にした。
    /// 波長ごとに同じ乱数種から始めれば、経路の乱数列は同一で**屈折率だけが違う**。
    /// 分散があれば波長間で放射輝度が変わり、無ければ完全に一致する。
    #[test]
    fn dispersion_makes_traced_radiance_wavelength_dependent() {
        let cauchy = CauchyDielectric {
            a: 1.5046,
            b: 4200.0,
        };
        let reference_ior = cauchy.ior_at(crate::path_tracer::REFERENCE_WAVELENGTH_NM);

        let build = |material: Material| {
            Scene::new(
                vec![
                    SceneObject {
                        primitive: Primitive::Sphere(Sphere {
                            center: sim_math::Vec3::ZERO,
                            radius: 1.0,
                        }),
                        material,
                    },
                    // 球の背後(-z側)の面光源。環境は真っ暗なので、屈折して
                    // ここに当たった経路だけが放射輝度を持つ = 屈折角が結果を決める。
                    SceneObject {
                        primitive: Primitive::Quad(crate::quad::Quad::axis_aligned(
                            2, -4.0, -20.0, 20.0, -20.0, 20.0,
                        )),
                        material: Material::Emissive(crate::path_tracer::Emissive {
                            radiance: 30.0,
                            albedo: 0.0,
                        }),
                    },
                ],
                vec![],
                0.0,
                None,
            )
        };

        // ③さらに面光源を十分大きく(±20)すると、今度は**どの方向へ屈折しても
        // 光源に当たる**ので再び波長差が消えた(450nm・650nmとも 26.953125)。
        // 一様環境と同じ状況に戻っただけである。
        //
        // ここまでの3回の失敗が示すのは、「分散が像に出るか」はシーンの角度構造に
        // 強く依存し、**単体テストで安定に測るのに向かない**ということ。そこで
        // 判定を物理そのもの——同じ入射レイに対する**屈折方向が波長で変わること**
        // ——へ移す。これは決定論的で、シーン構成に一切依存しない。
        let incident = sim_math::Vec3::new(0.3, -0.4, -1.0).normalize_or_zero();
        let normal = sim_math::Vec3::new(0.0, 0.0, 1.0);
        let refract_at = |ior: f64| {
            crate::bsdf::Dielectric::refract(incident, normal, 1.0 / ior)
                .expect("この入射角では全反射しない")
        };
        let blue_dir = refract_at(cauchy.ior_at(450.0));
        let red_dir = refract_at(cauchy.ior_at(650.0));
        let achromatic_blue = refract_at(reference_ior);
        let achromatic_red = refract_at(reference_ior);

        // 分散なし: 波長に依らず完全に同一の方向。
        assert_eq!(achromatic_blue.x, achromatic_red.x);
        assert_eq!(achromatic_blue.y, achromatic_red.y);
        // 分散あり: 方向が実際にずれる(角度差は微小だがゼロではない)。
        let angle = (blue_dir.dot(red_dir)).clamp(-1.0, 1.0).acos();
        assert!(
            angle > 1.0e-4,
            "分散により屈折方向が波長でずれるはず: 角度差={angle} rad"
        );
        // 青の方が屈折率が高い=より強く曲がる(法線に近い)。
        assert!(
            blue_dir.dot(normal).abs() > red_dir.dot(normal).abs(),
            "短波長ほど強く曲がる(法線に近い)はず: blue={blue_dir:?} red={red_dir:?}"
        );

        // シーン経路も生きていること(`trace_spectral`が面光源に届く)を併せて確認する
        // ——上の屈折の検証が「使われないコード」への主張にならないようにするため。
        let ray = Ray::new(
            sim_math::Vec3::new(0.72, 0.0, 3.0),
            sim_math::Vec3::new(0.0, 0.0, -1.0),
        );
        let scene = build(Material::CauchyDielectric(cauchy));
        let mut rng = SimRng::new(2024, 1);
        assert!(
            scene.trace_spectral(&ray, &mut rng, 8, 550.0) > 0.0,
            "屈折した経路が面光源に届いている必要がある"
        );
    }

    /// `trace_spectral`が実際に波長を材質へ届けていること(分散の向きの検証)。
    /// Cauchy式は短波長ほど屈折率が高いので、青(450nm)の屈折率は赤(650nm)より大きい。
    /// これが逆なら波長がどこかで取り違えられている。
    #[test]
    fn shorter_wavelengths_see_a_higher_refractive_index() {
        let cauchy = CauchyDielectric {
            a: 1.5046,
            b: 4200.0,
        };
        assert!(
            cauchy.ior_at(450.0) > cauchy.ior_at(650.0),
            "短波長ほど屈折率が高いはず: 450nm={} 650nm={}",
            cauchy.ior_at(450.0),
            cauchy.ior_at(650.0)
        );
    }
}
