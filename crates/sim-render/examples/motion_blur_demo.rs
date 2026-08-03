//! 増分C3(モーションブラー)の目視確認用デモ。設計上の解析的テスト
//! (`motion_blur.rs::tests`)とは別に、実際に「動いている球がブレて、静止している
//! 球はシャープに写る」対比を目で確認するためのもの。
//!
//! 実行例: `cargo run --release --example motion_blur_demo -- /path/to/out.png`
//! (出力先を省略すると`motion_blur_demo.png`をカレントディレクトリに書き出す)。
//!
//! シーン: `render_demo.rs`と同じ地面+赤い拡散球+金球の構成だが、赤い拡散球だけ
//! シャッター開時間内にx軸方向へ`DISPLACEMENT`だけ動く(`scene_at`で時刻ごとに
//! 中心位置を変えて`Scene`を作り直す、`motion_blur.rs`モジュールdoc「設計判断」
//! 参照)。金球・地面は静止したまま——両者の対比でブラーの有無が一目で分かる。
//!
//! `render_motion_blur`(モジュールdoc「単色限定の理由」参照)はグレースケール
//! 専用なので、本デモは`render_demo.rs`の`render_rgb`と同様に、チャンネル
//! (R/G/B)ごとに`render_motion_blur_channel`を個別に呼んで手動で`Framebuffer`へ
//! 詰める(`render.rs`の「チャンネル別レンダリング」パターンをモーションブラー版に
//! そのまま適用したもの)。

use sim_math::Vec3;
use sim_render::{
    render_motion_blur_channel, Camera, Framebuffer, Lambertian, Material, Metal, PointLight, Quad,
    RenderSettings, Scene, SceneObject, Sphere,
};

/// 赤い拡散球がシャッター開時間内に動く総距離(ワールド座標系のx軸方向)。
const DISPLACEMENT: f64 = 1.6;
/// シャッター開時間内の離散時刻標本数(`motion_blur.rs`モジュールdoc
/// 「縮約実装の理由」参照——多いほど滑らかだがコストが線形に増える)。
const SHUTTER_SAMPLES: u32 = 24;

fn ground_quad() -> Quad {
    Quad::axis_aligned(1, -1.0, -6.0, 6.0, -11.0, -1.0)
}

/// チャンネル`channel`(0=R,1=G,2=B)・時刻`t`(`[0,1]`、シャッター開時間内の
/// 正規化時刻)用のシーンを組み立てる。赤い拡散球の中心x座標だけが`t`に応じて
/// 動き、金球・地面は`t`によらず固定(静止対照)。
fn build_scene(channel: usize, t: f64) -> Scene {
    let red_albedo = [0.75, 0.15, 0.15][channel];
    let ground_albedo = 0.5;

    let red_center_x = -2.6 + DISPLACEMENT * t;
    let red_sphere = SceneObject::sphere(
        Sphere {
            center: Vec3::new(red_center_x, 0.0, -5.0),
            radius: 1.0,
        },
        Material::Lambertian(Lambertian { albedo: red_albedo }),
    );
    let gold_sphere = SceneObject::sphere(
        Sphere {
            center: Vec3::new(1.3, 0.0, -5.0),
            radius: 1.0,
        },
        Material::Metal(Metal { n: 0.47, k: 2.4 }),
    );
    let ground = SceneObject::quad(
        ground_quad(),
        Material::Lambertian(Lambertian {
            albedo: ground_albedo,
        }),
    );

    let light = PointLight {
        position: Vec3::new(2.5, 4.0, -2.0),
        intensity: 55.0,
    };

    Scene::new(
        vec![red_sphere, gold_sphere, ground],
        vec![light],
        0.15,
        None,
    )
}

fn main() {
    let out_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "motion_blur_demo.png".to_string());

    let camera = Camera {
        origin: Vec3::ZERO,
        forward: Vec3::new(0.0, 0.0, -1.0),
        right: Vec3::new(1.0, 0.0, 0.0),
        up: Vec3::new(0.0, 1.0, 0.0),
        lens_radius: 0.0,
        focus_distance: 5.0,
    };

    let width = 256;
    let height = 192;
    let vfov = 0.9;

    let settings = RenderSettings {
        spp: 6,
        max_depth: 4,
        exposure: 1.5,
        russian_roulette_after: None,
    };

    let mut rng_r = sim_math::SimRng::new(42, 0);
    let mut rng_g = sim_math::SimRng::new(42, 1);
    let mut rng_b = sim_math::SimRng::new(42, 2);

    let r = render_motion_blur_channel(
        |t| build_scene(0, t),
        &camera,
        vfov,
        width,
        height,
        &settings,
        SHUTTER_SAMPLES,
        &mut rng_r,
    );
    let g = render_motion_blur_channel(
        |t| build_scene(1, t),
        &camera,
        vfov,
        width,
        height,
        &settings,
        SHUTTER_SAMPLES,
        &mut rng_g,
    );
    let b = render_motion_blur_channel(
        |t| build_scene(2, t),
        &camera,
        vfov,
        width,
        height,
        &settings,
        SHUTTER_SAMPLES,
        &mut rng_b,
    );

    let mut framebuffer = Framebuffer::new(width, height);
    for i in 0..framebuffer.pixels.len() {
        framebuffer.pixels[i] = Vec3::new(r[i], g[i], b[i]);
    }

    framebuffer
        .write_png(std::path::Path::new(&out_path), settings.exposure)
        .expect("write_png should succeed");

    println!(
        "wrote {out_path} ({width}x{height}, spp={}, shutter_samples={SHUTTER_SAMPLES}, \
         displacement={DISPLACEMENT})",
        settings.spp
    );
}
