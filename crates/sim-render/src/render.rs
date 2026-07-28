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
