//! プリズム/雨粒の分散幾何(設計docs/13-electromagnetism/04-light-optics.md §2.2、
//! R3「プリズム最小偏角・虹の分散」)。`sim-render`自身が経路追跡に使う幾何
//! プリミティブ(`Dielectric::refract`/`reflect`、Snellの法則のベクトル実装)を
//! そのまま2回(プリズム)/3回(雨粒: 屈折→内部反射→屈折)呼んで実際にレイを追跡し、
//! 得られた偏角を独立に導出された閉形式(`sim_em::optics::prism_min_deviation`・
//! 古典的な雨粒偏角公式)と突き合わせる。
//!
//! **縮約実装の理由**: 完全な分光レンダリング(hero wavelength法、`Scene::trace`
//! 全体への波長の配線)はパストレーサ全体に大掛かりな変更を要するため後続増分に
//! 残す。本増分はR3の受け入れ基準(プリズム最小偏角・虹の分散が波長ごとに正しく
//! 異なること)を、レンダラ自身の幾何プリミティブを使った決定論的なレイ追跡
//! (乱数不使用、分岐なしのため厳密解と比較できる)で満たす。

use crate::bsdf::Dielectric;
use crate::ray::Ray;
use crate::sphere::Sphere;
use sim_math::{Quat, Vec3};

/// レイの自己交差(数値誤差でヒット点自身を再ヒット)を避けるための最小距離
/// (`path_tracer.rs`のシャドウレイと同じ慣例値`1e-6`)。
const SELF_HIT_EPSILON: f64 = 1e-6;

const Z_AXIS: Vec3 = Vec3 {
    x: 0.0,
    y: 0.0,
    z: 1.0,
};

/// 頂角`apex_angle`の対称プリズム(頂角の二等分線をy軸负方向に取り、2つの面の
/// 外向き法線が二等分線から`±apex_angle/2`傾く)に、第1面法線からの入射角
/// `incidence_angle`で入射したレイの偏角(入射方向から出射方向への回転角、
/// ラジアン)を返す。全反射(第2面での臨界角超過)なら`None`。
pub fn trace_prism_deviation(apex_angle: f64, incidence_angle: f64, ior: f64) -> Option<f64> {
    let half_apex = apex_angle / 2.0;
    // 第1面(入射面)の外向き法線(空気側を向く)。第2面の法線との間の角度が
    // 教科書どおり`π - apex_angle`になるよう、面に沿った方向ベクトルを
    // ±90°回転させて導出する(単純に二等分線からの傾き角をsin/cos入れ替えで
    // 割り当てると法線同士の相対角を誤るバグを、閉形式との突き合わせで発見・修正)。
    let normal1 = Vec3::new(-half_apex.cos(), half_apex.sin(), 0.0);
    let straight_in = normal1.scale(-1.0);
    let incoming = Quat::from_axis_angle(Z_AXIS, incidence_angle).rotate(straight_in);

    let inside = Dielectric::refract(incoming, normal1, 1.0 / ior)?;

    // 第2面(出射面)の外向き法線(空気側を向く)。
    // ガラス内部からの入射のため、`refract`が要求する「入射側から見て外向き」の
    // 法線は内部(ガラス側)を向く必要があり、外向き法線を反転して渡す。
    let normal2 = Vec3::new(half_apex.cos(), half_apex.sin(), 0.0);
    let outgoing = Dielectric::refract(inside, normal2.scale(-1.0), ior)?;

    Some(incoming.dot(outgoing).clamp(-1.0, 1.0).acos())
}

/// 虹(1次虹、水滴内部で屈折→内部反射1回→屈折)の偏角(設計docs/13-electromagnetism/
/// 04-light-optics.md §2.2、古典的なDescartesの虹理論)。半径`radius`の球(水滴)に、
/// 光軸から距離`impact_height`(0〜radius、衝突径数)だけ離れた平行光線が+x方向から
/// 入射したときの偏角を返す。全反射・交差不成立なら`None`。`Sphere::intersect`
/// (レンダラ自身が経路追跡に使う球交差)をそのまま使い、屈折/反射も`Dielectric`の
/// 同じ関数を呼ぶ。
pub fn trace_raindrop_deviation(impact_height: f64, radius: f64, ior: f64) -> Option<f64> {
    let sphere = Sphere {
        center: Vec3::new(0.0, 0.0, 0.0),
        radius,
    };
    let incoming = Vec3::new(1.0, 0.0, 0.0);
    let origin = Vec3::new(-2.0 * radius, impact_height, 0.0);

    let ray_in = Ray::new(origin, incoming);
    let hit1 = sphere.intersect(&ray_in, SELF_HIT_EPSILON)?;
    let refracted = Dielectric::refract(incoming, hit1.normal, 1.0 / ior)?;

    let ray_inside1 = Ray::new(hit1.point, refracted);
    let hit2 = sphere.intersect(&ray_inside1, SELF_HIT_EPSILON)?;
    let reflected = Dielectric::reflect(refracted, hit2.normal);

    let ray_inside2 = Ray::new(hit2.point, reflected);
    let hit3 = sphere.intersect(&ray_inside2, SELF_HIT_EPSILON)?;
    let outgoing = Dielectric::refract(reflected, hit3.normal.scale(-1.0), ior)?;

    Some(incoming.dot(outgoing).clamp(-1.0, 1.0).acos())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CauchyDielectric;

    /// BK7ガラス(既存の`CauchyDielectric`、bsdf.rsで既に検証済みのCauchy係数)の
    /// 頂角60°プリズムに、理論上の対称入射角(`n=sin((A+δm)/2)/sin(A/2)`の逆算)で
    /// 入射させたときの実際のレイ追跡による偏角が、独立に導出された閉形式
    /// (`sim_em::optics::prism_min_deviation`)とrel<1e-9で一致することを確認する
    /// (R3「プリズム最小偏角」)。乱数を使わない決定論的な幾何計算のため厳密一致する。
    #[test]
    fn prism_deviation_at_the_symmetric_incidence_matches_the_closed_form_minimum() {
        let apex_angle = 60.0_f64.to_radians();
        let ior = 1.5168;
        let i1 = (ior * (apex_angle / 2.0).sin()).asin();
        let traced = trace_prism_deviation(apex_angle, i1, ior).expect("no TIR at this angle");
        let closed_form = sim_em::prism_min_deviation(apex_angle, ior);
        let rel_err = (traced - closed_form).abs() / closed_form;
        assert!(rel_err < 1e-9, "traced={traced} closed_form={closed_form}");
    }

    /// 上記の対称入射角が実際に偏角の(局所)最小であることを、その前後の入射角で
    /// 偏角が大きくなることを確認して検証する(閉形式の値と一致するだけでは
    /// 「それが最小である」ことまでは検証できないため)。
    #[test]
    fn prism_deviation_increases_away_from_the_theoretical_minimum_incidence_angle() {
        let apex_angle = 60.0_f64.to_radians();
        let ior = 1.5168;
        let i1 = (ior * (apex_angle / 2.0).sin()).asin();
        let deviation_at_min = trace_prism_deviation(apex_angle, i1, ior).unwrap();
        for delta_deg in [2.0_f64, 5.0, 10.0] {
            let delta = delta_deg.to_radians();
            for candidate in [i1 - delta, i1 + delta] {
                let deviation = trace_prism_deviation(apex_angle, candidate, ior)
                    .expect("nearby incidence angles should not TIR");
                assert!(
                    deviation > deviation_at_min,
                    "incidence={} deviation={} should exceed the minimum {}",
                    candidate.to_degrees(),
                    deviation,
                    deviation_at_min
                );
            }
        }
    }

    /// BK7の分散(短波長ほど屈折率が大きい、既にbsdf.rsで検証済み)により、
    /// 同じプリズムでも青(486.1nm)の最小偏角が赤(656.3nm)より大きいことを確認する
    /// (R3「分散」、波長ごとに屈折角/偏角が異なることのプリズムでの帰結)。
    #[test]
    fn bk7_dispersion_gives_a_larger_prism_minimum_deviation_for_blue_than_red() {
        let apex_angle = 60.0_f64.to_radians();
        let bk7 = CauchyDielectric {
            a: 1.5046,
            b: 4200.0,
        };
        let n_blue = bk7.ior_at(486.1);
        let n_red = bk7.ior_at(656.3);
        assert!(
            n_blue > n_red,
            "normal dispersion: shorter wavelength => larger n"
        );

        let deviation_blue = sim_em::prism_min_deviation(apex_angle, n_blue);
        let deviation_red = sim_em::prism_min_deviation(apex_angle, n_red);
        assert!(
            deviation_blue > deviation_red,
            "blue={deviation_blue} red={deviation_red}"
        );
    }

    /// 虹の偏角(設計docs/13-electromagnetism/04-light-optics.md §2.2の水滴、
    /// R3「虹の分散」)が、独立に導出された古典的なDescartes閉形式
    /// $D(i)=\pi+2i-4r,\ r=\arcsin(\sin i/n)$と複数の衝突径数でrel<1e-9で一致する
    /// ことを確認する(設計の水の屈折率1.333、`docs/13-electromagnetism/
    /// 04-light-optics.md`の材質表と同じ値)。
    #[test]
    fn raindrop_deviation_matches_the_descartes_closed_form_across_impact_heights() {
        let radius = 1.0_f64;
        let ior = 1.333;
        for frac in [0.1, 0.3, 0.5, 0.7, 0.86, 0.9, 0.95] {
            let h = frac * radius;
            let i = (h / radius).asin();
            let r = (i.sin() / ior).asin();
            let closed_form = std::f64::consts::PI + 2.0 * i - 4.0 * r;
            let traced =
                trace_raindrop_deviation(h, radius, ior).expect("no TIR at this impact height");
            let rel_err = (traced - closed_form).abs() / closed_form;
            assert!(
                rel_err < 1e-9,
                "frac={frac} traced={traced} closed_form={closed_form}"
            );
        }
    }

    /// 水滴(n=1.333)の1次虹の偏角の最小値を衝突径数の走査で数値的に求め、
    /// 「虹の角度」(対日点からの角度、180°-最小偏角)が古典的によく知られる約42°に
    /// 近いことを確認する(Descartesの虹理論の教科書的な定量結果、乱数不使用の
    /// 決定論的走査)。
    #[test]
    fn raindrop_minimum_deviation_matches_the_classical_forty_two_degree_rainbow_angle() {
        let radius = 1.0_f64;
        let ior = 1.333;
        let mut min_deviation = f64::INFINITY;
        let steps = 2000;
        for step in 1..steps {
            let frac = step as f64 / steps as f64;
            if let Some(deviation) = trace_raindrop_deviation(frac * radius, radius, ior) {
                min_deviation = min_deviation.min(deviation);
            }
        }
        let rainbow_angle_deg = (std::f64::consts::PI - min_deviation).to_degrees();
        assert!(
            (rainbow_angle_deg - 42.0).abs() < 1.0,
            "rainbow_angle_deg={rainbow_angle_deg}"
        );
    }

    /// 虹の分散(波長ごとに虹の角度がわずかに異なり、これが虹の色分離の物理的起源)を、
    /// 水滴自体の分散係数が設計の材質表に未収録なため、既に検証済みのBK7の分散
    /// 曲線を代用して定性的に確認する——縮約実装の理由: 本増分の主目的は「レンダラの
    /// 幾何プリミティブで実際にレイを追跡したときに波長依存の屈折が正しく異なる
    /// 偏角を生むこと」自体の検証であり、水の実測分散係数の追加は材質DB拡張を
    /// 要するため後続増分に残す。
    #[test]
    fn wavelength_dependent_index_separates_raindrop_deviation_too() {
        let radius = 1.0_f64;
        let bk7 = CauchyDielectric {
            a: 1.5046,
            b: 4200.0,
        };
        let n_blue = bk7.ior_at(486.1);
        let n_red = bk7.ior_at(656.3);
        let impact = 0.8 * radius;
        let deviation_blue =
            trace_raindrop_deviation(impact, radius, n_blue).expect("no TIR at this impact height");
        let deviation_red =
            trace_raindrop_deviation(impact, radius, n_red).expect("no TIR at this impact height");
        assert!(
            (deviation_blue - deviation_red).abs() > 1e-6,
            "blue={deviation_blue} red={deviation_red} should differ"
        );
    }
}
