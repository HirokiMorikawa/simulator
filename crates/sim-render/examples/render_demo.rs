//! 増分C1(画像出力パイプライン)の目視確認用の最小デモ。設計上のR1–R7の解析解
//! テストとは別に、実際に`Camera`→`render_rgb`→`Framebuffer`→PNGの一気通貫を
//! 目で確認するためのもの(コーネルボックス等の凝ったシーンは後続増分C6で作る、
//! `docs/22-roadmap/02-feature-checklist.md`参照)。
//!
//! 実行例: `cargo run --release --example render_demo -- /path/to/out.png`
//! (出力先を省略すると`render_demo.png`をカレントディレクトリに書き出す)。
//!
//! シーン: 地面(灰色Lambertian)の上に赤い拡散球と金球(完全鏡面金属)を1つずつ
//! 置き、右上方の点光源1つ+一様な環境放射輝度(空の代わり)で照らす。

use sim_math::Vec3;
use sim_render::{
    render_rgb, Camera, Lambertian, Material, Metal, PointLight, Quad, RenderSettings, Scene,
    SceneObject, Sphere,
};

fn ground_quad() -> Quad {
    // y=-1平面、x∈[-6,6]・z∈[-11,-1](カメラの手前から奥まで十分な広さ)。
    Quad::axis_aligned(1, -1.0, -6.0, 6.0, -11.0, -1.0)
}

/// チャンネル`channel`(0=R,1=G,2=B)用のシーンを組み立てる(`render`モジュールdoc
/// 「チャンネル別レンダリング」参照)。金属球・地面はチャンネルによらず同じ
/// (金属の複素屈折率はモノクロ、地面はグレー)。赤い拡散球だけアルベドを
/// チャンネルごとに差し替える。
fn build_scene(channel: usize) -> Scene {
    let red_albedo = [0.75, 0.15, 0.15][channel];
    let ground_albedo = 0.5;

    let red_sphere = SceneObject::sphere(
        Sphere {
            center: Vec3::new(-1.3, 0.0, -5.0),
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
        0.15, // 一様環境放射輝度(空の代わりのアンビエント)。
        None,
    )
}

fn main() {
    let out_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "render_demo.png".to_string());

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
    let vfov = 0.9; // 垂直画角(ラジアン)。

    let settings = RenderSettings {
        spp: 64,
        max_depth: 4,
        exposure: 1.5,
    };

    let scene_r = build_scene(0);
    let scene_g = build_scene(1);
    let scene_b = build_scene(2);

    let framebuffer = render_rgb(
        [&scene_r, &scene_g, &scene_b],
        &camera,
        vfov,
        width,
        height,
        &settings,
        42,
    );

    framebuffer
        .write_png(std::path::Path::new(&out_path), settings.exposure)
        .expect("write_png should succeed");

    println!("wrote {out_path} ({width}x{height}, spp={})", settings.spp);
}
