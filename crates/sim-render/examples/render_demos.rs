//! デモ **D40–D43** の目視チェック用画像を生成する(増分C6)。
//!
//! 設計 `docs/21-verification/03-demo-scenarios.md` Phase D:
//!
//! | デモ | 内容 | 合格基準 |
//! |---|---|---|
//! | D40 光の実験室 | プリズム分光・水中コースティクス・虹をパストレで | R2/R3 |
//! | D41 材質ギャラリー | 金属/ガラス/プラスチックの球アレイ(物性DB連動) | R1・R2 |
//! | D42 空と大気 | レイリー散乱による空の青・夕焼け | R5 |
//! | D43 カメラ | 被写界深度・露出・モーションブラー | R6 |
//!
//! §7冒頭の規約「合格 = 合格基準のヘッドレステスト Green + **目視チェック**」の
//! 目視側の実体がこの example である。**`cargo test` では実行されない**
//! (examples はビルドされるが実行されないため、既存のテスト時間を1秒も増やさない
//! ——`sim-render` は既に20万サンプル級のテストを複数抱えている)。
//!
//! 実行: `cargo run --release --example render_demos -- <出力ディレクトリ>`

use sim_core::MaterialDb;
use sim_math::Vec3;
use sim_render::{
    from_material_db, relative_exposure, render_motion_blur, render_rgb, trace_ball_lens_caustic,
    AtmosphereMedium, Camera, CausticMap, Dielectric, HomogeneousMedium, Lambertian, Material,
    PointLight, Quad, RenderSettings, Scene, SceneObject, Sphere,
};
use std::path::{Path, PathBuf};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 240;

fn front_camera(origin: Vec3, lens_radius: f64, focus_distance: f64) -> Camera {
    Camera {
        origin,
        forward: Vec3::new(0.0, 0.0, -1.0),
        right: Vec3::new(1.0, 0.0, 0.0),
        up: Vec3::new(0.0, 1.0, 0.0),
        lens_radius,
        focus_distance,
    }
}

fn ground(albedo: f64) -> SceneObject {
    SceneObject::quad(
        Quad::axis_aligned(1, -1.0, -12.0, 12.0, -24.0, -1.0),
        Material::Lambertian(Lambertian { albedo }),
    )
}

fn write(fb: &sim_render::Framebuffer, exposure: f64, out: &Path, name: &str) {
    let path = out.join(format!("{name}.png"));
    fb.write_png(&path, exposure).expect("PNGを書き出せる");
    println!("  -> {}", path.display());
}

// ---------------------------------------------------------------- D40 光の実験室

/// D40: ガラス球(誘電体)+ 拡散球を並べ、屈折で背景が反転して見えることを確認する。
/// コースティクス本体は `caustic_demo` と同じ `trace_ball_lens_caustic` で別途出す
/// (`caustic.rs` モジュールdoc「カメラ経路と合成しない」参照——パストレ画像の中に
/// コースティクスは現れないので、両者を別の画像として並べるのが正直な提示になる)。
fn d40_scene(channel: usize) -> Scene {
    let glass = SceneObject::sphere(
        Sphere {
            center: Vec3::new(-1.2, -0.1, -4.5),
            radius: 0.9,
        },
        Material::Dielectric(Dielectric { ior: 1.52 }),
    );
    // 屈折で反転して見えるための「背景の目印」= 色の付いた球を奥に置く。
    let marker = SceneObject::sphere(
        Sphere {
            center: Vec3::new(-1.2, -0.1, -8.0),
            radius: 0.7,
        },
        Material::Lambertian(Lambertian {
            albedo: [0.85, 0.2, 0.2][channel],
        }),
    );
    let water = SceneObject::sphere(
        Sphere {
            center: Vec3::new(1.3, -0.2, -4.5),
            radius: 0.8,
        },
        Material::Dielectric(Dielectric { ior: 1.333 }),
    );
    Scene::new(
        vec![glass, marker, water, ground(0.55)],
        vec![PointLight {
            position: Vec3::new(2.0, 5.0, -1.5),
            intensity: 70.0,
        }],
        0.25,
        None,
    )
}

fn render_d40(out: &Path) {
    println!("D40 光の実験室(ガラス球n=1.52・水球n=1.333の屈折)");
    let camera = front_camera(Vec3::new(0.0, 0.3, 0.0), 0.0, 4.5);
    let settings = RenderSettings {
        spp: 200,
        max_depth: 8,
        exposure: 1.4,
    };
    let (r, g, b) = (d40_scene(0), d40_scene(1), d40_scene(2));
    let fb = render_rgb([&r, &g, &b], &camera, 0.9, WIDTH, HEIGHT, &settings, 40);
    write(&fb, settings.exposure, out, "d40_refraction");

    // コースティクス(近軸焦点面)。
    let (radius, ior) = (1.0_f64, 1.5_f64);
    let focal = sim_render::ball_lens_paraxial_focal_distance(radius, ior);
    let mut map = CausticMap::new(WIDTH as usize, WIDTH as usize, 0.5 * radius);
    trace_ball_lens_caustic(radius, ior, 0.9 * radius, focal, 900, &mut map);
    let scale = 0.6 / map.peak_energy().max(1e-12);
    write(&map.to_framebuffer(scale), 1.0, out, "d40_caustic");
}

// ------------------------------------------------------------ D41 材質ギャラリー

/// D41: **`MaterialDb` から実際に引いた**光学値で球アレイを並べる(増分C4の橋)。
/// 縮退(光学値がDBに無い材質)が起きた場合は警告を標準出力へ出す——設計§5の
/// 「既定BSDF + 警告」が実際に働いていることを目視できるようにするため。
fn d41_scene(channel: usize, names: &[&str]) -> Scene {
    let db = MaterialDb::standard();
    let mut objects = vec![ground(0.5)];
    let spacing = 1.35;
    let x0 = -(names.len() as f64 - 1.0) * 0.5 * spacing;
    for (i, name) in names.iter().enumerate() {
        let id = db.find_by_name(name).expect("標準DBに存在する材質名");
        let resolved = from_material_db(&db, id);
        if channel == 0 {
            if let Some(w) = &resolved.warning {
                println!("  [警告] {w}");
            }
        }
        objects.push(SceneObject::sphere(
            Sphere {
                center: Vec3::new(x0 + i as f64 * spacing, -0.35, -5.5),
                radius: 0.6,
            },
            resolved.material,
        ));
    }
    // 屈折・反射が「何かを写している」ことが分かるよう、背景に色の付いた壁を置く。
    objects.push(SceneObject::quad(
        Quad::axis_aligned(2, -9.0, -8.0, 8.0, -1.0, 5.0),
        Material::Lambertian(Lambertian {
            albedo: [0.30, 0.45, 0.70][channel],
        }),
    ));
    Scene::new(
        objects,
        vec![PointLight {
            position: Vec3::new(0.0, 5.0, -2.0),
            intensity: 90.0,
        }],
        0.30,
        None,
    )
}

fn render_d41(out: &Path) {
    println!("D41 材質ギャラリー(MaterialDbから引いた光学値の球アレイ)");
    let names = ["ガラス", "水", "アルミニウム", "銅", "ゴム(天然)"];
    println!("  材質: {}", names.join(" / "));
    let camera = front_camera(Vec3::new(0.0, 0.35, 0.0), 0.0, 5.5);
    let settings = RenderSettings {
        spp: 200,
        max_depth: 8,
        exposure: 1.2,
    };
    let (r, g, b) = (
        d41_scene(0, &names),
        d41_scene(1, &names),
        d41_scene(2, &names),
    );
    let fb = render_rgb([&r, &g, &b], &camera, 0.95, WIDTH, HEIGHT, &settings, 41);
    write(&fb, settings.exposure, out, "d41_material_gallery");
}

// ---------------------------------------------------------------- D42 空と大気

/// D42: レイリー散乱(R5)。`AtmosphereMedium` を実際に `Scene` へ配線して
/// 波長別に散乱係数を変え、**チャンネルごとに別の大気**でレンダリングする。
/// これが「空が青い」の物理的な出どころ($\sigma_s \propto \lambda^{-4}$)。
fn d42_scene(channel: usize, sun_elevation: f64) -> Scene {
    // R/G/B の代表波長(CIE等色関数は未実装なので代表波長で代用する縮約)。
    let wavelength_nm = [700.0, 546.1, 435.8][channel];
    let medium = HomogeneousMedium::rayleigh_atmosphere(wavelength_nm, 1.0e-2);
    let sun_direction =
        Vec3::new(0.0, sun_elevation.sin(), -sun_elevation.cos()).normalize_or_zero();
    Scene::new(
        vec![ground(0.35)],
        vec![],
        0.02, // 大気そのものが光るので環境放射輝度はごく小さく。
        Some(AtmosphereMedium {
            medium,
            sun_direction,
            sun_radiance: 60.0,
            up: Vec3::new(0.0, 1.0, 0.0),
            thickness: 40.0,
        }),
    )
}

fn render_d42(out: &Path) {
    println!("D42 空と大気(レイリー散乱、波長別に散乱係数を変える)");
    // 空を見上げる向き(forwardをやや上へ)。
    let camera = Camera {
        origin: Vec3::new(0.0, 0.0, 0.0),
        forward: Vec3::new(0.0, 0.35, -1.0).normalize_or_zero(),
        right: Vec3::new(1.0, 0.0, 0.0),
        up: Vec3::new(0.0, 1.0, 0.35).normalize_or_zero(),
        lens_radius: 0.0,
        focus_distance: 10.0,
    };
    let settings = RenderSettings {
        spp: 120,
        max_depth: 2,
        exposure: 1.0,
    };
    for (name, elevation_deg) in [("noon", 70.0_f64), ("sunset", 4.0_f64)] {
        let e = elevation_deg.to_radians();
        let (r, g, b) = (d42_scene(0, e), d42_scene(1, e), d42_scene(2, e));
        let fb = render_rgb([&r, &g, &b], &camera, 1.1, WIDTH, HEIGHT, &settings, 42);
        write(&fb, settings.exposure, out, &format!("d42_sky_{name}"));
    }
}

// ------------------------------------------------------------------ D43 カメラ

fn d43_scene_at(channel: usize, t: f64) -> Scene {
    // 手前・中央・奥に球を並べる(被写界深度の効きが分かる配置)。
    let albedo = |c: [f64; 3]| Material::Lambertian(Lambertian { albedo: c[channel] });
    let near = SceneObject::sphere(
        Sphere {
            center: Vec3::new(-1.1, -0.35, -3.0),
            radius: 0.55,
        },
        albedo([0.85, 0.25, 0.25]),
    );
    let mid = SceneObject::sphere(
        Sphere {
            center: Vec3::new(0.0, -0.35, -5.0),
            radius: 0.55,
        },
        albedo([0.25, 0.75, 0.30]),
    );
    let far = SceneObject::sphere(
        Sphere {
            center: Vec3::new(1.3, -0.35, -8.0),
            radius: 0.55,
        },
        albedo([0.30, 0.40, 0.90]),
    );
    // モーションブラー用に水平移動する球(t は [0,1] の正規化シャッター時刻)。
    let moving = SceneObject::sphere(
        Sphere {
            center: Vec3::new(-2.4 + 2.4 * t, 0.75, -5.0),
            radius: 0.4,
        },
        albedo([0.95, 0.85, 0.25]),
    );
    Scene::new(
        vec![near, mid, far, moving, ground(0.5)],
        vec![PointLight {
            position: Vec3::new(1.5, 5.0, -1.5),
            intensity: 90.0,
        }],
        0.22,
        None,
    )
}

fn render_d43(out: &Path) {
    println!("D43 カメラ(被写界深度・露出・モーションブラー)");
    let settings_base = RenderSettings {
        spp: 160,
        max_depth: 5,
        exposure: 1.0,
    };

    // (1) 被写界深度: 中央の球(z=-5)へ合焦し、絞りを開けて手前/奥をぼかす。
    for (name, lens_radius) in [("pinhole", 0.0), ("wide_aperture", 0.20)] {
        let camera = front_camera(Vec3::new(0.0, 0.25, 0.0), lens_radius, 5.0);
        let (r, g, b) = (
            d43_scene_at(0, 0.5),
            d43_scene_at(1, 0.5),
            d43_scene_at(2, 0.5),
        );
        let fb = render_rgb(
            [&r, &g, &b],
            &camera,
            0.95,
            WIDTH,
            HEIGHT,
            &settings_base,
            43,
        );
        write(&fb, settings_base.exposure, out, &format!("d43_dof_{name}"));
    }

    // (2) 露出: 同じシーンをEVで1段ずつ変える(relative_exposure が出す倍率)。
    //     f/4・ISO100 固定でシャッターだけ 1/60 → 1/30 → 1/15 と2倍ずつ開ける。
    let camera = front_camera(Vec3::new(0.0, 0.25, 0.0), 0.0, 5.0);
    let (r, g, b) = (
        d43_scene_at(0, 0.5),
        d43_scene_at(1, 0.5),
        d43_scene_at(2, 0.5),
    );
    let fb = render_rgb(
        [&r, &g, &b],
        &camera,
        0.95,
        WIDTH,
        HEIGHT,
        &settings_base,
        43,
    );
    let base = relative_exposure(1.0 / 60.0, 100.0, 4.0);
    for (name, shutter) in [
        ("under", 1.0 / 60.0),
        ("normal", 1.0 / 30.0),
        ("over", 1.0 / 15.0),
    ] {
        let e = relative_exposure(shutter, 100.0, 4.0) / base;
        write(&fb, e, out, &format!("d43_exposure_{name}"));
    }

    // (3) モーションブラー: シャッター開時間内の複数時刻を平均する。
    let settings_blur = RenderSettings {
        spp: 40,
        max_depth: 5,
        exposure: 1.0,
    };
    let fb = render_motion_blur(
        |t| d43_scene_at(1, t),
        &camera,
        0.95,
        WIDTH,
        HEIGHT,
        &settings_blur,
        16,
        43,
    );
    write(&fb, settings_blur.exposure, out, "d43_motion_blur");
}

fn main() {
    let out = PathBuf::from(std::env::args().nth(1).unwrap_or_else(|| ".".to_string()));
    std::fs::create_dir_all(&out).expect("出力ディレクトリを作れる");
    render_d40(&out);
    render_d41(&out);
    render_d42(&out);
    render_d43(&out);
}
