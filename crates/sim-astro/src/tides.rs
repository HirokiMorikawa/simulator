//! 潮汐力(差分重力)。設計: docs/16-astro/01-gravitation-nbody.md、
//! docs/21-verification/03-demo-scenarios.md D38「潮汐」(合格基準「潮汐力の定性」)。
//!
//! 摂動天体(月・太陽等)による潮汐加速度は、中心天体の中心と、中心から`offset`離れた点との
//! 重力加速度の差(自由落下する中心天体自身のフレームで見た相対加速度)として定義する
//! (`offset`を無限小近似しない厳密な差分)。地球半径/月までの距離比(≈1/60)では、
//! 標準の小offset近似公式$\vec a_{tidal}\approx\frac{GM}{d^3}(3(\hat r\cdot\hat x)\hat x
//! \cdot|x| ...)$からのずれが数%程度生じるため、本モジュールのテストは方向・大小関係の
//! **定性**を主眼とし、定量比較は緩めの許容誤差(rel<5%)で行う——設計の合格基準
//! 「潮汐力の定性」に対応する。

use sim_math::Vec3;

/// 摂動天体(標準重力パラメータ`gm_perturber`、中心天体から見た相対位置`r_to_perturber`)
/// による、中心天体の中心から`offset_from_center`離れた点での潮汐加速度。
pub fn tidal_acceleration(
    gm_perturber: f64,
    r_to_perturber: Vec3,
    offset_from_center: Vec3,
) -> Vec3 {
    let to_point = r_to_perturber - offset_from_center;
    let dist_point = to_point.length();
    let accel_at_point = to_point.scale(gm_perturber / dist_point.powi(3));

    let dist_center = r_to_perturber.length();
    let accel_at_center = r_to_perturber.scale(gm_perturber / dist_center.powi(3));

    accel_at_point - accel_at_center
}

#[cfg(test)]
mod tests {
    use super::*;

    const GM_MOON: f64 = 4.904_869_5e12;
    const D_MOON: f64 = 3.844e8;
    const GM_SUN: f64 = 1.327_124_400_18e20;
    const D_SUN: f64 = 1.496e11;
    const R_EARTH: f64 = 6.371e6;

    /// D38 潮汐(モジュールdoc参照)。月による潮汐加速度が、地球中心から見て月側
    /// (近点)・その反対側(遠点)の両方で中心から外向き(古典的な「両側に膨らむ」
    /// 潮汐バルジ)、月方向に垂直な側では内向き(圧縮)であることを確認する。
    /// 近点・遠点での大きさは、小offset近似の解析式$2GM_{moon}R_\oplus/d^3$と
    /// rel<5%で一致(モジュールdoc「定性」参照、高次項による数%のずれを許容)。
    #[test]
    fn d38_tidal_acceleration_bulges_outward_on_near_and_far_side_and_inward_at_the_sides() {
        let r_to_moon = Vec3::new(D_MOON, 0.0, 0.0);

        let near_side = Vec3::new(R_EARTH, 0.0, 0.0);
        let far_side = Vec3::new(-R_EARTH, 0.0, 0.0);
        let side_point = Vec3::new(0.0, R_EARTH, 0.0);

        let a_near = tidal_acceleration(GM_MOON, r_to_moon, near_side);
        let a_far = tidal_acceleration(GM_MOON, r_to_moon, far_side);
        let a_side = tidal_acceleration(GM_MOON, r_to_moon, side_point);

        assert!(
            a_near.dot(near_side.scale(1.0 / near_side.length())) > 0.0,
            "near-side tidal acceleration should point outward (away from Earth's center): {a_near:?}"
        );
        assert!(
            a_far.dot(far_side.scale(1.0 / far_side.length())) > 0.0,
            "far-side tidal acceleration should also point outward (the second tidal bulge): {a_far:?}"
        );
        assert!(
            a_side.dot(side_point.scale(1.0 / side_point.length())) < 0.0,
            "perpendicular-side tidal acceleration should point inward (compression): {a_side:?}"
        );

        let analytic_radial = 2.0 * GM_MOON * R_EARTH / D_MOON.powi(3);
        for (name, a) in [("near", a_near), ("far", a_far)] {
            let rel_err = (a.length() - analytic_radial).abs() / analytic_radial;
            assert!(
                rel_err < 0.05,
                "{name}-side tidal magnitude should roughly match the small-offset analytic \
                 formula 2GMR/d^3: measured={:?} analytic={analytic_radial} rel_err={rel_err:.4}",
                a.length()
            );
        }
    }

    /// D38 大潮/小潮(モジュールdoc参照)。太陽の潮汐が月と同じ方向に揃う(大潮)場合、
    /// 太陽が月と直交する方向にある(小潮)場合より、月直下点での正味の潮汐加速度
    /// (月方向成分)が明確に大きいことを確認する(実際の太陽/月潮汐比≈0.46に基づく
    /// 大潮/小潮比≈1.9と同オーダー、rel許容ではなく「大潮 > 小潮」の定性判定)。
    #[test]
    fn d38_spring_tide_exceeds_neap_tide_when_sun_and_moon_align_vs_perpendicular() {
        let moon_direction = Vec3::new(1.0, 0.0, 0.0);
        let r_to_moon = moon_direction.scale(D_MOON);
        let sublunar_point = moon_direction.scale(R_EARTH);

        let tidal_from_moon = tidal_acceleration(GM_MOON, r_to_moon, sublunar_point);

        // 大潮: 太陽が月と同じ方向に揃う。
        let r_to_sun_spring = moon_direction.scale(D_SUN);
        let tidal_from_sun_spring = tidal_acceleration(GM_SUN, r_to_sun_spring, sublunar_point);
        let radial_spring = (tidal_from_moon + tidal_from_sun_spring).dot(moon_direction);

        // 小潮: 太陽が月と直交する方向にある。
        let r_to_sun_neap = Vec3::new(0.0, D_SUN, 0.0);
        let tidal_from_sun_neap = tidal_acceleration(GM_SUN, r_to_sun_neap, sublunar_point);
        let radial_neap = (tidal_from_moon + tidal_from_sun_neap).dot(moon_direction);

        assert!(
            radial_spring > radial_neap * 1.3,
            "spring tide (sun aligned with moon) should be meaningfully stronger than neap tide \
             (sun perpendicular to moon): radial_spring={radial_spring:e} radial_neap={radial_neap:e}"
        );
    }
}
