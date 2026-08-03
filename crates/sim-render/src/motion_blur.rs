//! モーションブラー(シャッター開時間内の光の時間平均)。設計
//! docs/17-rendering/03-materials-camera.md §4.1「モーションブラー: シャッター開時間内の
//! 時刻をサンプル(物理状態の補間、剛体transform)」。
//!
//! **重要な設計判断: `Ray`に時刻フィールドを追加しない**。`Ray::new`の呼び出し箇所は
//! `camera.rs`(`generate_ray`)・`path_tracer.rs`(BSDFサンプリングのたびに新しい
//! `Ray`を構築)・`bsdf.rs`・`prism.rs`など全モジュールに散らばっており、`Ray`へ
//! 時刻を足すとこれら全てのシグネチャ変更を要する(波及が大きすぎる)。
//!
//! 代わりに、**シャッター開時間内の複数の離散時刻でシーンそのものを構築し直し、
//! それぞれのレンダリング結果を平均する**方式を採る: 呼び出し側が与える
//! `scene_at(t)`(`t∈[0,1]`は正規化したシャッター開時間内の時刻)を
//! `shutter_samples`個の時刻で呼び出して`Scene`を作り直し、それぞれ既存の
//! `render_channel`でレンダリングして単純平均する。
//!
//! **これは縮約ではなく、物理的にシャッター積分そのものである**: 実際のカメラの
//! センサーが記録する放射照度は、シャッターが開いている時間$[0,T]$にわたって
//! 届く放射輝度の時間平均$\frac{1}{T}\int_0^T L(t)\,dt$であり(モジュールdoc
//! 冒頭の設計§4.1「シャッター開時間内の時刻をサンプル」がまさにこれを指す)、
//! 本実装はこの積分を`shutter_samples`点の等間隔標本による中点則のリーマン和で
//! 近似する。`shutter_samples\to\infty`で連続シャッター積分に収束する——
//! 剛体の位置・向きの時刻依存の補間(設計§4.1「物理状態の補間、剛体transform」)は
//! `scene_at`の中身(呼び出し側の責務)に委ねられ、本モジュールはそれに関知しない。
//!
//! この方式の利点: ①既存コード(`Ray`・`Camera::generate_ray`・`Scene::trace`)へ
//! **一切変更が要らない**(既存90テストへの回帰リスクがゼロ)。②`scene_at`の中で
//! 好きな補間(線形・スプライン・回転はSLERP等)を使ってよく、本モジュールは
//! それを一切関知しない疎結合な設計になる。
//!
//! **縮約実装の理由**: `shutter_samples`個の決定論的な等間隔時刻(中点則)のみを
//! 使い、連続時刻分布からの乱数サンプリング(層化サンプリング等)は行わない——
//! 各時刻ごとに`Scene::new`(BVH再構築込み)+フルの`render_channel`呼び出しを
//! 要するため、時刻数が多いほど線形にコストが増える(`shutter_samples`倍)。
//! 滑らかなブラーには相応の時刻数(本モジュールのテストでは数十)が要るが、
//! 適応的なサンプリング(コントラストの高い領域だけ時刻を増やす等)は行わない。

use sim_math::{SimRng, Vec3};

use crate::camera::Camera;
use crate::framebuffer::Framebuffer;
use crate::path_tracer::Scene;
use crate::render::{render_channel, RenderSettings};

/// 単一チャンネル版(モジュールdoc参照)。`scene_at(t)`を`shutter_samples`個の
/// 等間隔時刻(中点則: $t_k=(k+0.5)/N$、`k=0..N-1`)で呼び出し、それぞれ
/// `render_channel`でレンダリングした行優先の放射輝度配列を単純平均する。
///
/// `rng`は全ての時刻・全てのピクセルを通じて共有する1本のPRNG(`render_channel`と
/// 同じ「1本のPRNGを連続的に消費する」設計、`render.rs`モジュールdoc参照)。
#[allow(clippy::too_many_arguments)]
pub fn render_motion_blur_channel(
    scene_at: impl Fn(f64) -> Scene,
    camera: &Camera,
    vfov: f64,
    width: u32,
    height: u32,
    settings: &RenderSettings,
    shutter_samples: u32,
    rng: &mut SimRng,
) -> Vec<f64> {
    assert!(shutter_samples > 0, "shutter_samples must be positive");
    let pixel_count = width as usize * height as usize;
    let mut accum = vec![0.0; pixel_count];
    for k in 0..shutter_samples {
        // シャッター開時間内の時刻(中点則、モジュールdoc参照)。
        let t = (k as f64 + 0.5) / shutter_samples as f64;
        let scene = scene_at(t);
        let channel = render_channel(&scene, camera, vfov, width, height, settings, rng);
        for (a, c) in accum.iter_mut().zip(channel.iter()) {
            *a += c;
        }
    }
    let n = shutter_samples as f64;
    for a in &mut accum {
        *a /= n;
    }
    accum
}

/// `render_motion_blur_channel`をRGB全チャンネルへ配線した便利関数。
///
/// **単色(グレースケール)限定の理由**: `render.rs`の`render_rgb`はチャンネルごとに
/// 異なる`Scene`(同一形状・アルベドだけ差し替え、`render.rs`モジュールdoc
/// 「チャンネル別レンダリング」参照)を取るが、本関数は`scene_at`を1つしか取らない
/// ため、返す`Framebuffer`はR=G=Bの単色(グレースケール)になる。色付きの
/// モーションブラー画像が欲しい場合は`render_motion_blur_channel`をチャンネルごとに
/// (チャンネル別の`scene_at`で)個別に呼んで手動で`Framebuffer`へ詰める
/// (`examples/motion_blur_demo.rs`参照)。
#[allow(clippy::too_many_arguments)]
pub fn render_motion_blur(
    scene_at: impl Fn(f64) -> Scene,
    camera: &Camera,
    vfov: f64,
    width: u32,
    height: u32,
    settings: &RenderSettings,
    shutter_samples: u32,
    seed: u64,
) -> Framebuffer {
    let mut rng = SimRng::new(seed, 0);
    let channel = render_motion_blur_channel(
        scene_at,
        camera,
        vfov,
        width,
        height,
        settings,
        shutter_samples,
        &mut rng,
    );

    let mut framebuffer = Framebuffer::new(width, height);
    for (pixel, &v) in framebuffer.pixels.iter_mut().zip(channel.iter()) {
        *pixel = Vec3::new(v, v, v);
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

    /// 中心行(`py=height/2`)上で、背景(`environment_radiance`)と異なる値を持つ
    /// 画素の最左・最右インデックスを返す(球の水平方向の広がりを画素単位で測る)。
    fn occupied_column_extent(
        pixels: &[f64],
        width: u32,
        py: u32,
        background: f64,
        epsilon: f64,
    ) -> Option<(u32, u32)> {
        let row_start = py as usize * width as usize;
        let row = &pixels[row_start..row_start + width as usize];
        let mut left = None;
        let mut right = None;
        for (x, &v) in row.iter().enumerate() {
            if (v - background).abs() > epsilon {
                if left.is_none() {
                    left = Some(x as u32);
                }
                right = Some(x as u32);
            }
        }
        left.zip(right)
    }

    /// 黒い(albedo=0)球1個+一様環境放射輝度のシーン。albedo=0のLambertianは
    /// `direct_lighting`のbsdf項も間接項の`bsdf*cosθ/pdf`も恒等的に0になるため
    /// (`path_tracer.rs`の`Material::Lambertian`分岐参照)、球に当たったレイは
    /// **常に厳密に0**を返す(統計誤差ゼロの白色炉テスト系と同じ理屈——ここでは
    /// 逆に「常に吸収」側の厳密な二値性を使う)。背景(環境放射輝度)とは値が
    /// 大きく異なるため、画素ごとに「球に当たったか」を閾値なしで厳密に判別できる。
    fn scene_with_sphere_at(
        center_x: f64,
        radius: f64,
        depth: f64,
        environment_radiance: f64,
    ) -> Scene {
        Scene::new(
            vec![SceneObject::sphere(
                Sphere {
                    center: Vec3::new(center_x, 0.0, -depth),
                    radius,
                },
                Material::Lambertian(Lambertian { albedo: 0.0 }),
            )],
            vec![],
            environment_radiance,
            None,
        )
    }

    /// 設計§4.1「モーションブラー」の解析的検証。運動軸(x軸)方向に速度`v`で動く
    /// 球を、シャッター時間`T`でブラーさせたときの**運動軸方向の非背景画素の
    /// 広がり**が「静止時の広がり + v・T」に対応することを確認する。
    ///
    /// **ピクセル量子化誤差の見積もり**: 許容誤差を2.0画素とする。内訳は
    /// ①アンチエイリアスのサブピクセルジッタ(`render_channel`が`rng.next_f64()`で
    /// 決める連続的なサンプル位置)により、画素境界の判定が最大で約0.5画素分
    /// ランダムにずれ得る(静止像・ブラー像それぞれで発生するため、差を取ると
    /// 最大約1画素)。②シャッターを`shutter_samples`個の離散時刻(中点則)でしか
    /// 標本化しないため、連続的な掃引に対して最大`v・T/shutter_samples`(本テストの
    /// パラメータでは1画素未満)の標本化誤差が乗る。③画素インデックスは整数
    /// (量子化)であり、広がりを「最左画素〜最右画素」の整数差として測るため、
    /// 端で最大1画素の丸めが乗り得る。以上を線形に積み上げても2画素以内に収まる
    /// はずである。実測値(本テストのパラメータ、固定シードのため決定論的):
    /// 静止時の広がり=28px・ブラー時の広がり=38px・実測の増分=10px、対して
    /// 予測(displacement/world_per_pixel)≈11.04px——差は約1.04pxで、2px許容の
    /// 半分程度のマージンに収まっている。
    #[test]
    fn moving_sphere_blur_spreads_by_approximately_velocity_times_shutter_time() {
        let camera = test_camera();
        let vfov = 0.9;
        let width = 64u32;
        let height = 64u32;
        let aspect = width as f64 / height as f64;
        let environment_radiance = 5.0;
        let depth = 6.0;
        let radius = 1.2;
        let center_x0 = -1.5;
        // v・T(シャッター開時間内の総移動量、世界座標系)。速度vとシャッター時間T
        // 自体は任意の分解(例: v=20 world-units/s, T=0.05s)で構わない——本関数の
        // 積分は正規化時刻t∈[0,1]でのみ働くため、物理的に意味を持つのは積v・Tだけ。
        let displacement = 1.0;
        let settings = RenderSettings {
            spp: 1,
            max_depth: 4,
            exposure: 1.0,
            russian_roulette_after: None,
        };
        let shutter_samples = 48;
        let py = height / 2;

        // 静止像(v=0): 常に同じ位置に球がある(ブラー無し)。
        let mut rng_static = SimRng::new(11, 1);
        let static_pixels = render_motion_blur_channel(
            |_t| scene_with_sphere_at(center_x0, radius, depth, environment_radiance),
            &camera,
            vfov,
            width,
            height,
            &settings,
            shutter_samples,
            &mut rng_static,
        );
        let (static_left, static_right) =
            occupied_column_extent(&static_pixels, width, py, environment_radiance, 1e-9)
                .expect("static sphere should occlude some pixels on the center row");
        let static_extent = (static_right - static_left) as f64;

        // ブラー像: 球がx軸方向にdisplacementだけシャッター開時間内で移動する。
        let mut rng_moving = SimRng::new(11, 1);
        let moving_pixels = render_motion_blur_channel(
            |t| {
                scene_with_sphere_at(
                    center_x0 + displacement * t,
                    radius,
                    depth,
                    environment_radiance,
                )
            },
            &camera,
            vfov,
            width,
            height,
            &settings,
            shutter_samples,
            &mut rng_moving,
        );
        let (moving_left, moving_right) =
            occupied_column_extent(&moving_pixels, width, py, environment_radiance, 1e-9)
                .expect("motion-blurred sphere should occlude some pixels on the center row");
        let moving_extent = (moving_right - moving_left) as f64;

        // 世界座標→画素の変換係数(`camera.rs`の`pinhole_direction`が
        // `tan(vfov/2)*aspect`を水平半画角の正接として使うことの直接の帰結——
        // forward=(0,0,-1)・right=(1,0,0)のとき、ndc_x・halfwidth・depthから
        // 深さdepthの平面上のワールドx座標はndc_x*halfwidth*depthに厳密に一致する
        // (レイのパラメータ化t'でz=-depthとなるt'=depthを代入するだけの幾何学的
        // 恒等式、正規化の有無に依らない)。
        let half_width = (vfov / 2.0).tan() * aspect;
        let world_per_pixel = 2.0 * depth * half_width / width as f64;
        let predicted_extra_pixels = displacement / world_per_pixel;

        let measured_extra_pixels = moving_extent - static_extent;
        let tolerance_pixels = 2.0;
        assert!(
            (measured_extra_pixels - predicted_extra_pixels).abs() < tolerance_pixels,
            "static_extent={static_extent}px moving_extent={moving_extent}px \
             measured_extra={measured_extra_pixels}px \
             predicted_extra={predicted_extra_pixels}px (v*T={displacement} \
             world_per_pixel={world_per_pixel}) tolerance={tolerance_pixels}px"
        );

        // ブラーが実際に「見えている」ことの直接確認: 静止像には存在しない中間値
        // (0でも`environment_radiance`でもない画素、複数の時刻標本にまたがって
        // 球と背景の両方に当たった画素の平均値)が、ブラー像には少なくとも1つ
        // 現れるはず。
        let has_intermediate_value = moving_pixels[(py as usize) * width as usize..]
            [..width as usize]
            .iter()
            .any(|&v| v > 1e-9 && (v - environment_radiance).abs() > 1e-9);
        assert!(
            has_intermediate_value,
            "a motion-blurred image should contain at least one partially-covered \
             (neither black nor background) pixel on the center row"
        );
    }

    /// 対照実験: 速度`v=0`(`scene_at`が時刻によらず同じ`Scene`を返す)なら、
    /// `shutter_samples`を増やしても(モーションブラーのケースと同じ標本数を
    /// 使っても)画素の広がりは「単一時刻(`shutter_samples=1`)の無ブラー基準画像」
    /// と一致する——ブラーが一切生じないことの直接確認(設計の要求「v=0なら静止像と
    /// 一致する」)。
    ///
    /// **実装中に発見した点**: 当初は「`shutter_samples`を増やしても全画素が厳密に
    /// `0.0`か`environment_radiance`のどちらかだけを取る(中間値が一切現れない)」
    /// ことを検証しようとしたが、これは誤り(テスト設計自体のバグ)だと判明した——
    /// `render_motion_blur_channel`は時刻ごとに`rng`を消費し続けるため、たとえ
    /// `v=0`でシーン自体は不変でも、各シャッター時刻標本で画素内のサブピクセル
    /// ジッタ(アンチエイリアス、`render.rs`モジュールdoc参照)が毎回引き直され、
    /// 境界画素では「ある時刻標本は球に当たり別の時刻標本は外れる」という
    /// **通常のアンチエイリアスによる中間値**が生じ得る(モーションブラーとは
    /// 無関係な既知の現象で、バグではない)。したがって「中間値が一切現れない」
    /// という主張は`shutter_samples>1`では成立せず、代わりに「広がり(occupied
    /// extent)が単一時刻基準と一致する」という、アンチエイリアスの精度向上は
    /// 許しつつブラーによる余分な広がりが無いことだけを主張する形に直した。
    /// 実測値(固定シードのため決定論的): reference_extent(shutter_samples=1)=26px、
    /// static_extent(v=0, shutter_samples=48)=27px——差1pxで2px許容に十分収まる。
    #[test]
    fn zero_velocity_matches_the_single_sample_static_reference_extent() {
        let camera = test_camera();
        let vfov = 0.9;
        let width = 64u32;
        let height = 64u32;
        let environment_radiance = 5.0;
        let depth = 6.0;
        let radius = 1.2;
        let center_x = 0.4;
        let settings = RenderSettings {
            spp: 1,
            max_depth: 4,
            exposure: 1.0,
            russian_roulette_after: None,
        };
        let py = height / 2;
        let scene_at =
            |_t: f64| scene_with_sphere_at(center_x, radius, depth, environment_radiance);

        // 単一時刻(シャッター積分を伴わない)の無ブラー基準画像。
        let mut rng_reference = SimRng::new(17, 1);
        let reference_pixels = render_motion_blur_channel(
            scene_at,
            &camera,
            vfov,
            width,
            height,
            &settings,
            1,
            &mut rng_reference,
        );
        let (reference_left, reference_right) =
            occupied_column_extent(&reference_pixels, width, py, environment_radiance, 1e-9)
                .expect("reference image should show the sphere on the center row");
        let reference_extent = (reference_right - reference_left) as f64;

        // v=0を、モーションブラーのテストと同じ`shutter_samples`個の時刻標本で
        // レンダリングする(シーン自体は時刻によらず不変)。
        let mut rng_static = SimRng::new(19, 1);
        let static_pixels = render_motion_blur_channel(
            scene_at,
            &camera,
            vfov,
            width,
            height,
            &settings,
            48,
            &mut rng_static,
        );
        let (static_left, static_right) =
            occupied_column_extent(&static_pixels, width, py, environment_radiance, 1e-9)
                .expect("v=0 image should still show the sphere on the center row");
        let static_extent = (static_right - static_left) as f64;

        let tolerance_pixels = 2.0;
        assert!(
            (static_extent - reference_extent).abs() < tolerance_pixels,
            "v=0 should match the unblurred reference extent (no motion blur): \
             reference_extent={reference_extent}px static_extent={static_extent}px \
             tolerance={tolerance_pixels}px"
        );
    }
}
