//! 物理カメラ(薄レンズモデル)。設計 docs/17-rendering/03-materials-camera.md §4.1
//! 「薄レンズモデル: 焦点距離$f$、絞り$N$(F値)から開口半径、被写界深度(ボケ)。
//! レンズ上のサンプリングで焦点外をぼかす」。
//!
//! **縮約実装の理由**: 実際のセンサー・結像距離までは扱わず(設計§4.1の完全な
//! カメラ方程式ではなく)、`focus_distance`(完全に合焦するワールド空間上の距離)+
//! `lens_radius`(開口半径)を直接パラメータとする縮約(Ray Tracing in One Weekend
//! 等の標準的な薄レンズカメラの構成)。ピンホール方向のレイをレンズ円板上の点から
//! 再構成し、焦点距離面上の同じ点(`focus_point`)を通すことでボケを再現する——
//! モーションブラーは後続増分(`motion_blur`モジュールdoc参照)。
//!
//! **増分C2(露出)**: `relative_exposure`・`exposure_value_at_iso100`(本ファイル
//! 下部)を追加した。設計§4.1「露出: シャッター速度・ISO・絞りから露出値(EV)。
//! 物理的な光量→センサー応答」に対応する、写真測光の標準的な露出方程式
//! ($H \propto t \cdot \mathrm{ISO}/N^2$)の実装。**縮約実装の理由**:
//! 比例定数(センサー較正定数、いわゆる"K"値)は導入せず、比例関係そのもの
//! (相対露出)のみを返す——`RenderSettings.exposure`に渡す倍率としては
//! 比例定数は掛け捨てられる(スケールが変わるだけで「EV変化に対する像の明るさが
//! 物理的にスケールする」という設計§7の検証基準には影響しない)ため、絶対的な
//! 物理単位(cd/m²等)への較正は引き続き行わない。また実際のセンサーの反射率
//! 較正・回折・相反則不軌(reciprocity failure)等は対象外。

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

    /// ピンホール方向: 正規化デバイス座標`(ndc_x, ndc_y)`(それぞれ`[-1,1]`、
    /// `ndc_x`は右方向・`ndc_y`は上方向が正)・画角`vfov`(垂直、ラジアン)・
    /// アスペクト比`aspect`(width/height)から、`generate_ray`が要求する
    /// 「既に計算済みの方向」を作る(`lib.rs`モジュールdoc「ピクセル→方向の対応
    /// だけが欠けていた」を埋める本増分の核)。
    ///
    /// 標準的なピンホールカメラの視錐台パラメータ化: 画面中央(`ndc=(0,0)`)は
    /// `forward`そのもの、画面端は`forward`から`right`/`up`方向に
    /// `tan(vfov/2)`(垂直)・`tan(vfov/2)*aspect`(水平)だけ傾く。
    pub fn pinhole_direction(&self, ndc_x: f64, ndc_y: f64, aspect: f64, vfov: f64) -> Vec3 {
        let half_height = (vfov / 2.0).tan();
        let half_width = half_height * aspect;
        (self.forward + self.right.scale(ndc_x * half_width) + self.up.scale(ndc_y * half_height))
            .normalize_or_zero()
    }
}

/// 相対露出(露出倍率)。設計§4.1「露出: シャッター速度・ISO・絞りから露出値(EV)。
/// 物理的な光量→センサー応答」(モジュールdoc「増分C2」参照)。
///
/// 写真測光の標準的な露出方程式: センサーに届く光量(露出量)$H$は、シャッター時間
/// $t$・ISO感度に比例し、絞りのF値$N$の2乗に反比例する:
/// $$H \propto t \cdot \mathrm{ISO} / N^2$$
/// (絞りのF値は「開口径 = 焦点距離/N」の定義上、開口面積(ひいては集める光量)が
/// $1/N^2$に比例することの直接の帰結——`aperture_radius_from_f_number`が使うのと
/// 同じ$N$の定義)。本関数は比例定数を1として、この関係をそのまま
/// `RenderSettings.exposure`へ渡す倍率として計算する。
pub fn relative_exposure(shutter_time: f64, iso: f64, f_number: f64) -> f64 {
    shutter_time * iso / (f_number * f_number)
}

/// 露出値(EV、ISO100基準): $\mathrm{EV} = \log_2(N^2/t)$(設計§4.1、モジュールdoc
/// 「増分C2」参照)。
///
/// EVは写真の「段(stop)」の定義そのもの——EVが1増える(絞りを1段絞る、または
/// シャッター時間を半分にする)ごとに、センサーに届く光量(`relative_exposure`)は
/// ちょうど半分になる。ISO100以外での実効EVは`exposure_value_at_iso100(N,t) -
/// (iso/100.0).log2()`で得られる(ISOを2倍にすると同じ光量でも実効EVが1段下がる、
/// すなわちセンサー側の感度が2倍になった分だけ「同じ露出」を得るのに必要な光量が
/// 半分で済む、という標準的なISOの扱い)。
pub fn exposure_value_at_iso100(f_number: f64, shutter_time: f64) -> f64 {
    (f_number * f_number / shutter_time).log2()
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

    /// 画面中央(`ndc=(0,0)`)は常に`forward`方向そのもの(オフセット無し)。
    #[test]
    fn pinhole_direction_at_screen_center_matches_forward_exactly() {
        let camera = test_camera(0.0, 5.0);
        let direction = camera.pinhole_direction(0.0, 0.0, 4.0 / 3.0, 0.9);
        assert!(
            (direction - camera.forward).length() < 1e-12,
            "direction={direction:?} forward={:?}",
            camera.forward
        );
    }

    /// 画面端では`forward`から`right`/`up`方向に`tan(vfov/2)`
    /// (アスペクト比ぶん水平方向はスケールされる)だけ傾く、という視錐台の
    /// 定義式そのものと厳密一致する。
    #[test]
    fn pinhole_direction_at_screen_edges_matches_the_frustum_formula() {
        let camera = test_camera(0.0, 5.0);
        let aspect = 16.0 / 9.0;
        let vfov = 1.2_f64;
        for &(ndc_x, ndc_y) in &[(1.0, 0.0), (0.0, 1.0), (-1.0, -1.0), (0.7, -0.4)] {
            let direction = camera.pinhole_direction(ndc_x, ndc_y, aspect, vfov);
            let half_height = (vfov / 2.0).tan();
            let half_width = half_height * aspect;
            let expected = (camera.forward
                + camera.right.scale(ndc_x * half_width)
                + camera.up.scale(ndc_y * half_height))
            .normalize_or_zero();
            assert!(
                (direction - expected).length() < 1e-12,
                "ndc=({ndc_x},{ndc_y}) direction={direction:?} expected={expected:?}"
            );
            // 常に単位ベクトルであること。
            assert!((direction.length() - 1.0).abs() < 1e-12);
        }
    }

    /// 増分C2・設計§7「露出: EV変化に対する像の明るさが物理的にスケール」の
    /// 検証基準その1——**絞りを1段絞る**(F値をN→N√2にする)と、開口面積が
    /// ちょうど半分になるので相対露出もちょうど半分になる。
    #[test]
    fn stopping_down_the_aperture_by_one_stop_halves_relative_exposure() {
        let shutter_time = 1.0 / 125.0;
        let iso = 200.0;
        let f_number = 4.0;
        let baseline = relative_exposure(shutter_time, iso, f_number);
        let stopped_down =
            relative_exposure(shutter_time, iso, f_number * std::f64::consts::SQRT_2);
        let rel_err = (stopped_down - baseline / 2.0).abs() / (baseline / 2.0);
        assert!(
            rel_err < 1e-9,
            "baseline={baseline} stopped_down={stopped_down} rel_err={rel_err}"
        );
    }

    /// 検証基準その2——**シャッター時間を2倍**にすると露出はちょうど2倍
    /// ($H \propto t$の直接の帰結)。
    #[test]
    fn doubling_shutter_time_doubles_relative_exposure() {
        let iso = 100.0;
        let f_number = 2.8;
        let a = relative_exposure(1.0 / 60.0, iso, f_number);
        let b = relative_exposure(2.0 / 60.0, iso, f_number);
        assert!((b - 2.0 * a).abs() < 1e-12, "a={a} b={b}");
    }

    /// 検証基準その3——**ISOを2倍**にすると露出はちょうど2倍
    /// ($H \propto \mathrm{ISO}$の直接の帰結)。
    #[test]
    fn doubling_iso_doubles_relative_exposure() {
        let shutter_time = 1.0 / 250.0;
        let f_number = 5.6;
        let a = relative_exposure(shutter_time, 100.0, f_number);
        let b = relative_exposure(shutter_time, 200.0, f_number);
        assert!((b - 2.0 * a).abs() < 1e-12, "a={a} b={b}");
    }

    /// EV(ISO100基準)が閉形式$\log_2(N^2/t)$と厳密一致すること、および
    /// 「絞りを1段絞るとEVがちょうど+1、対応する相対露出がちょうど半分になる」
    /// という段(stop)の定義との整合を確認する。
    #[test]
    fn exposure_value_matches_closed_form_and_one_stop_is_a_factor_of_two_in_relative_exposure() {
        let f_number = 2.8;
        let shutter_time = 1.0 / 200.0;
        let ev = exposure_value_at_iso100(f_number, shutter_time);
        let expected_ev = (f_number * f_number / shutter_time).log2();
        assert!(
            (ev - expected_ev).abs() < 1e-12,
            "ev={ev} expected_ev={expected_ev}"
        );

        let f_number_stopped_down = f_number * std::f64::consts::SQRT_2;
        let ev_stopped_down = exposure_value_at_iso100(f_number_stopped_down, shutter_time);
        assert!(
            (ev_stopped_down - (ev + 1.0)).abs() < 1e-9,
            "ev={ev} ev_stopped_down={ev_stopped_down}"
        );

        let iso = 100.0;
        let exposure_before = relative_exposure(shutter_time, iso, f_number);
        let exposure_after = relative_exposure(shutter_time, iso, f_number_stopped_down);
        let rel_err = (exposure_after - exposure_before / 2.0).abs() / (exposure_before / 2.0);
        assert!(
            rel_err < 1e-9,
            "exposure_before={exposure_before} exposure_after={exposure_after}"
        );
    }
}
