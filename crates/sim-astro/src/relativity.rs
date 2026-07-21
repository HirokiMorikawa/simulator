//! オプトインPost-Newton(1PN)補正。設計: docs/16-astro/03-relativistic-corrections.md。
//!
//! Pα スコープの縮約実装: `RelativitySettings`構造体・`NBodySystem`への完全統合
//! (既定オフのフラグ経由の有効化)は行わず、1PN加速度補正・近日点移動率・GPS固有時率を
//! 純関数として実装する。水星近日点移動の検証(A8)は、実際の太陽・水星の$GM/c^2$比では
//! 43″/世紀という極小の歳差を測定するのに現実的でない数の周回積分が必要となるため、
//! $GM/c^2$比を誇張した二体系(主星固定・test-particle近似)で少数周回のシミュレーションを
//! 行い、同じ誇張パラメータでの解析式と比較することで加速度補正項の実装自体を検証する
//! (実際の水星の数値そのものの再現ではない)。GPS固有時率(A9)は解析式のみで検証済みの
//! 実数値と直接比較できるため、シミュレーションを要しない。

use sim_math::Vec3;

/// 光速 [m/s](定義値、設計§9)。
pub const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// 1PN加速度補正(Schwarzschild項、設計§2.1)。主星(標準重力パラメータ`gm`)まわりの
/// 1体。`r_vec`は主星からの相対位置、`v_vec`は相対速度。ニュートン加速度に加算する。
pub fn pn1_acceleration(gm: f64, c: f64, r_vec: Vec3, v_vec: Vec3) -> Vec3 {
    let r = r_vec.length();
    let r_hat = r_vec.scale(1.0 / r);
    let v_sq = v_vec.length_sq();
    let coeff = gm / (c * c * r * r);
    let radial_term = r_hat.scale(4.0 * gm / r - v_sq);
    let tangential_term = v_vec.scale(4.0 * v_vec.dot(r_hat));
    (radial_term + tangential_term).scale(coeff)
}

/// 近日点移動率(解析式、設計§2.1): $\Delta\varpi = \frac{6\pi GM}{c^2 a(1-e^2)}$
/// (1周あたりのラジアン)。
pub fn pn1_precession_per_orbit(gm: f64, c: f64, semi_major_axis: f64, eccentricity: f64) -> f64 {
    6.0 * std::f64::consts::PI * gm
        / (c * c * semi_major_axis * (1.0 - eccentricity * eccentricity))
}

/// GPS固有時率(設計§2.2): 衛星時計の地表時計に対する相対的な進み率、
/// $d\tau_{sat}/d\tau_{ground} - 1 \approx GM(1/r_{ground}-1/r_{sat})/c^2 - (v_{sat}^2-v_{ground}^2)/(2c^2)$
/// (円軌道近似のニュートンポテンシャル$\Phi=-GM/r$より導出)。正なら衛星時計が進む。
pub fn gps_proper_time_rate(
    gm_earth: f64,
    c: f64,
    r_ground: f64,
    r_satellite: f64,
    v_ground: f64,
    v_satellite: f64,
) -> f64 {
    let gravitational_term = gm_earth * (1.0 / r_ground - 1.0 / r_satellite) / (c * c);
    let kinematic_term = -(v_satellite * v_satellite - v_ground * v_ground) / (2.0 * c * c);
    gravitational_term + kinematic_term
}

/// 円軌道の周回速度(設計§2.2で速度項に使う): $v=\sqrt{GM/r}$。
pub fn circular_orbital_speed(gm: f64, r: f64) -> f64 {
    (gm / r).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// GPS時間差(A9、設計§7): +38.6 μs/日 ± 1%。地表(半径6371km、地球の自転による
    /// 表面速度は本近似では無視)と高度20200kmのGPS衛星(円軌道)の固有時率の差を
    /// 解析式のみで評価する(シミュレーション不要)。
    #[test]
    fn a9_gps_proper_time_difference_matches_known_value() {
        let gm_earth = 3.986_004_418e14;
        let r_ground = 6_371_000.0;
        let altitude = 20_200_000.0;
        let r_satellite = r_ground + altitude;
        let v_ground = 0.0; // 地球自転速度は本近似では無視
        let v_satellite = circular_orbital_speed(gm_earth, r_satellite);

        let rate = gps_proper_time_rate(
            gm_earth,
            SPEED_OF_LIGHT,
            r_ground,
            r_satellite,
            v_ground,
            v_satellite,
        );
        let seconds_per_day = 86400.0;
        let measured_us_per_day = rate * seconds_per_day * 1e6;

        let expected_us_per_day = 38.6;
        let rel_err = (measured_us_per_day - expected_us_per_day).abs() / expected_us_per_day;
        assert!(
            rel_err < 0.01,
            "measured={measured_us_per_day:.4} expected={expected_us_per_day} rel_err={rel_err:.4}"
        );
    }

    /// 近日点移動(A8、設計§7): $\Delta\varpi=6\pi GM/(c^2a(1-e^2))$ ± 1%。実際の太陽・水星の
    /// GM/c^2比では43″/世紀という極小の歳差を数値積分で検出するのに非現実的な数の周回が
    /// 要るため、GM/c^2比を誇張した二体系(主星固定、test-particle近似)で少数周回積分し、
    /// 同じ誇張パラメータでの解析式と比較する(モジュールdoc参照)。離心率ベクトル
    /// (Laplace-Runge-Lenzベクトル)の向きの変化から歳差率を測定する(近日点通過の
    /// タイミング検出が不要な頑健な方法)。
    /// 実装検証中、誇張しすぎる(例: c=20)と解析式(1PNの線形近似)からの系統的なずれが
    /// 大きくなる(c=20でrel_err≈14%、c=40で≈3%、誤差はGM/c²にほぼ比例して縮小)ことを
    /// 発見した — ステップ数を増やしても縮まらないため数値誤差ではなく、線形の1PN近似
    /// 自体が過度に強い摂動では破れる(2次以降の項が無視できなくなる)ことが原因と判明。
    /// c=100(rel_err<1%)まで弱めることで解決した。
    #[test]
    fn a8_perihelion_precession_matches_analytic_1pn_formula() {
        let gm: f64 = 1.0;
        let c: f64 = 100.0; // 誇張したGM/c^2比(現実の太陽系よりずっと大きい)
        let a: f64 = 1.0;
        let e: f64 = 0.5;

        let r_peri = a * (1.0 - e);
        let v_peri = ((gm / a) * (1.0 + e) / (1.0 - e)).sqrt();
        let mut r_vec = Vec3::new(r_peri, 0.0, 0.0);
        let mut v_vec = Vec3::new(0.0, v_peri, 0.0);

        let accel = |r: Vec3, v: Vec3| -> Vec3 {
            let dist = r.length();
            let newtonian = r.scale(-gm / (dist * dist * dist));
            newtonian + pn1_acceleration(gm, c, r, v)
        };

        let period = 2.0 * std::f64::consts::PI * (a.powi(3) / gm).sqrt();
        let orbits = 20;
        let steps_per_orbit = 8000;
        let dt = period / steps_per_orbit as f64;

        let eccentricity_vector_angle = |r: Vec3, v: Vec3| -> f64 {
            let h = r.cross(v);
            let e_vec = v.cross(h).scale(1.0 / gm) - r.scale(1.0 / r.length());
            e_vec.y.atan2(e_vec.x)
        };

        let initial_angle = eccentricity_vector_angle(r_vec, v_vec);
        let mut unwrapped_angle = initial_angle;
        let mut prev_angle = initial_angle;

        // velocity Verlet(1PNは非保存的だが加速度をr,v両方に依存させる標準拡張で十分な精度)
        let mut a_old = accel(r_vec, v_vec);
        for _ in 0..(orbits * steps_per_orbit) {
            r_vec = r_vec
                .addcarry_scaled(v_vec, dt)
                .addcarry_scaled(a_old, 0.5 * dt * dt);
            let a_new = accel(r_vec, v_vec);
            v_vec = v_vec.addcarry_scaled(a_old + a_new, 0.5 * dt);
            a_old = a_new;

            let raw_angle = eccentricity_vector_angle(r_vec, v_vec);
            let mut delta = raw_angle - prev_angle;
            while delta > std::f64::consts::PI {
                delta -= 2.0 * std::f64::consts::PI;
            }
            while delta < -std::f64::consts::PI {
                delta += 2.0 * std::f64::consts::PI;
            }
            unwrapped_angle += delta;
            prev_angle = raw_angle;
        }

        let measured_precession_per_orbit = (unwrapped_angle - initial_angle) / orbits as f64;
        let analytic_precession_per_orbit = pn1_precession_per_orbit(gm, c, a, e);
        let rel_err = (measured_precession_per_orbit - analytic_precession_per_orbit).abs()
            / analytic_precession_per_orbit;
        assert!(
            rel_err < 0.01,
            "measured={measured_precession_per_orbit:.6} analytic={analytic_precession_per_orbit:.6} rel_err={rel_err:.4}"
        );
    }
}
