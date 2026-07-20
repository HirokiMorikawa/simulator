//! 幾何光学(アイコナール近似)— スネル則・フレネル係数・薄レンズ・プリズム。
//! 設計: docs/13-electromagnetism/04-light-optics.md §2.1。
//!
//! P4 スコープの最小実装: フル `RayTracer`(光線束追跡・分岐・分光)は未実装で、
//! §7 の解析解テスト(E9–E12)が要求する代数的な界面公式のみを提供する。
//! いずれも状態を持たない純粋関数(§3「光線は状態を持たない」の精神を界面単体に適用)。

/// スネル則 $n_1\sin\theta_1=n_2\sin\theta_2$ による屈折角。全反射なら `None`。
/// 角度はラジアン、入射角は界面法線からで測る。
pub fn snell_refract_angle(n1: f64, n2: f64, theta_i: f64) -> Option<f64> {
    let sin_t = n1 / n2 * theta_i.sin();
    if sin_t.abs() > 1.0 {
        None
    } else {
        Some(sin_t.asin())
    }
}

/// 臨界角 $\sin\theta_c = n_2/n_1$($n_1>n_2$ のときのみ存在)。
pub fn critical_angle(n1: f64, n2: f64) -> Option<f64> {
    if n1 <= n2 {
        None
    } else {
        Some((n2 / n1).asin())
    }
}

/// ブリュースター角 $\theta_B=\arctan(n_2/n_1)$(この角度で $R_p=0$)。
pub fn brewster_angle(n1: f64, n2: f64) -> f64 {
    (n2 / n1).atan()
}

/// フレネル反射率(s偏光・p偏光・非偏光平均)。設計 §2.1。
#[derive(Clone, Copy, Debug)]
pub struct FresnelReflectance {
    pub r_s: f64,
    pub r_p: f64,
    pub r_unpolarized: f64,
}

/// 界面のフレネル反射率。全反射なら `None`(反射率1として扱うのは呼び出し側の判断)。
pub fn fresnel_reflectance(n1: f64, n2: f64, theta_i: f64) -> Option<FresnelReflectance> {
    let theta_t = snell_refract_angle(n1, n2, theta_i)?;
    let (cos_i, cos_t) = (theta_i.cos(), theta_t.cos());
    let r_s = ((n1 * cos_i - n2 * cos_t) / (n1 * cos_i + n2 * cos_t)).powi(2);
    let r_p = ((n2 * cos_i - n1 * cos_t) / (n2 * cos_i + n1 * cos_t)).powi(2);
    Some(FresnelReflectance {
        r_s,
        r_p,
        r_unpolarized: 0.5 * (r_s + r_p),
    })
}

/// 薄レンズの焦点距離(レンズメーカーの式)$1/f=(n-1)(1/R_1-1/R_2)$。設計 §7。
pub fn thin_lens_focal_length(n: f64, r1: f64, r2: f64) -> f64 {
    1.0 / ((n - 1.0) * (1.0 / r1 - 1.0 / r2))
}

/// レンズメーカーの式とは独立に、各球面での近軸屈折(reduced angle 法)を個別に
/// 追跡して焦点距離を求める。設計 §7「近軸光線で焦点距離誤差 < 1%」の「近軸光線」側の
/// 実装(レンズメーカーの式そのものを呼ばずに同じ物理から独立に導出する)。
/// 薄レンズ近似(2面間距離0)、高さ `h` は近軸(線形)なので値によらない。
pub fn thin_lens_paraxial_ray_trace_focal_length(n: f64, r1: f64, r2: f64) -> f64 {
    let h = 0.01;
    let power1 = (n - 1.0) / r1;
    // u = reduced angle(n・θ)。入射光線は光軸に平行(u0=0)。
    let u1 = -h * power1;
    let power2 = (1.0 - n) / r2;
    let u2 = u1 - h * power2;
    // 面2通過後は空気中(n=1)なので u2 がそのまま実角度。光軸との交点までの距離。
    -h / u2
}

/// プリズム最小偏角 $\delta_m$(頂角 `apex_angle` の対称配置、設計 §7)。
pub fn prism_min_deviation(apex_angle: f64, n: f64) -> f64 {
    let theta1 = (n * (apex_angle / 2.0).sin()).asin();
    2.0 * theta1 - apex_angle
}

/// 最小偏角の測定値から屈折率を逆算する式 $n=\sin\frac{A+\delta_m}{2}/\sin\frac{A}{2}$。
pub fn prism_index_from_min_deviation(apex_angle: f64, min_deviation: f64) -> f64 {
    ((apex_angle + min_deviation) / 2.0).sin() / (apex_angle / 2.0).sin()
}

#[cfg(test)]
mod tests {
    use super::*;

    const BK7_INDEX: f64 = 1.5168; // 設計 §9(589nm)

    /// E9: フレネル垂直反射率 $((n_1-n_2)/(n_1+n_2))^2$、rel 1%(空気→BK7ガラスで約4%、
    /// 設計 §9 の記載どおり)。ブリュースター角では $R_p=0$。
    /// (docs/21-verification/01-analytic-tests.md E9)
    #[test]
    fn e9_fresnel_normal_incidence_and_brewster_angle() {
        let (n1, n2) = (1.0, BK7_INDEX);
        let r = fresnel_reflectance(n1, n2, 0.0).unwrap();
        let expected = ((n1 - n2) / (n1 + n2)).powi(2);
        let rel_err = (r.r_unpolarized - expected).abs() / expected;
        assert!(rel_err < 0.01, "R={} expected={expected}", r.r_unpolarized);
        assert!((0.03..0.05).contains(&expected), "expected={expected}");

        let theta_b = brewster_angle(n1, n2);
        let r_brewster = fresnel_reflectance(n1, n2, theta_b).unwrap();
        assert!(
            r_brewster.r_p.abs() < 1e-9,
            "R_p={} at Brewster angle should vanish",
            r_brewster.r_p
        );
    }

    /// E10: スネル則・臨界角。屈折角は解析解に機械精度で一致し、臨界角を超えると
    /// 全反射(`None`)になること(docs/21-verification/01-analytic-tests.md E10)。
    /// 臨界角(水→空気)は設計 §9 のパラメータ表(48.6°)と一致する。
    #[test]
    fn e10_snell_law_and_critical_angle_totally_internally_reflect() {
        let (n_water, n_air) = (1.333, 1.0);
        let theta_i = 20.0_f64.to_radians();
        let theta_t = snell_refract_angle(n_water, n_air, theta_i).unwrap();
        let expected_sin_t = n_water / n_air * theta_i.sin();
        assert!(
            (theta_t.sin() - expected_sin_t).abs() < 1e-12,
            "theta_t={theta_t}"
        );

        let theta_c = critical_angle(n_water, n_air).unwrap();
        let expected_theta_c_deg = 48.6;
        let rel_err = (theta_c.to_degrees() - expected_theta_c_deg).abs() / expected_theta_c_deg;
        assert!(rel_err < 0.01, "theta_c_deg={}", theta_c.to_degrees());

        // 臨界角を超える入射角では全反射(スネル則の実数解が存在しない)。
        let beyond_critical = theta_c + 5.0_f64.to_radians();
        assert!(snell_refract_angle(n_water, n_air, beyond_critical).is_none());
        // 臨界角未満では屈折光が存在する。
        let below_critical = theta_c - 5.0_f64.to_radians();
        assert!(snell_refract_angle(n_water, n_air, below_critical).is_some());
    }

    /// E11: 薄レンズ結像 $1/f=(n-1)(1/R_1-1/R_2)$、近軸光線で焦点距離誤差 < 1%
    /// (docs/21-verification/01-analytic-tests.md E11)。レンズメーカーの式(閉形式)と、
    /// 各球面での近軸屈折を個別に追跡した近軸光線追跡が独立に一致することを確認する。
    #[test]
    fn e11_thin_lens_focal_length_matches_paraxial_ray_trace() {
        let n = BK7_INDEX;
        let (r1, r2) = (0.1, -0.1); // 両凸レンズ
        let f_formula = thin_lens_focal_length(n, r1, r2);
        let f_traced = thin_lens_paraxial_ray_trace_focal_length(n, r1, r2);
        let rel_err = (f_traced - f_formula).abs() / f_formula;
        assert!(
            rel_err < 0.01,
            "f_formula={f_formula} f_traced={f_traced} rel_err={rel_err}"
        );
    }

    /// E12: プリズム最小偏角から屈折率を逆算する誤差 < 0.5%(頂角60°BK7、
    /// docs/21-verification/01-analytic-tests.md E12)。
    #[test]
    fn e12_prism_minimum_deviation_index_round_trip() {
        let apex_angle = 60.0_f64.to_radians();
        let n = BK7_INDEX;
        let delta_m = prism_min_deviation(apex_angle, n);
        let recovered_n = prism_index_from_min_deviation(apex_angle, delta_m);
        let rel_err = (recovered_n - n).abs() / n;
        assert!(
            rel_err < 0.005,
            "recovered_n={recovered_n} n={n} rel_err={rel_err}"
        );
    }
}
