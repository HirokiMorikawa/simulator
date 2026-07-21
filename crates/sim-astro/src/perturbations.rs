//! 軌道摂動($J_2$扁平率)。設計: docs/16-astro/02-orbital-mechanics.md §2.2。
//!
//! `pn1_acceleration`(relativity.rs)と同じパターンで、ニュートン加速度に加算する
//! 摂動項を純関数として実装する(`NBodySystem`本体への統合は未実装)。

use sim_math::Vec3;

/// 地球のJ2係数(扁平率、設計§9パラメータ表)。
pub const EARTH_J2: f64 = 1.08263e-3;
/// 地球の赤道半径 [m](設計§9パラメータ表)。
pub const EARTH_EQUATORIAL_RADIUS: f64 = 6.378_137e6;

/// $J_2$扁平率による摂動加速度(設計§2.2、標準的な直交座標表示)。`r_vec`は中心天体からの
/// 相対位置(z軸は天体の自転軸/赤道面の法線と一致すると仮定)。ニュートン加速度に加算する。
pub fn j2_acceleration(gm: f64, j2: f64, equatorial_radius: f64, r_vec: Vec3) -> Vec3 {
    let r = r_vec.length();
    let z_over_r = r_vec.z / r;
    let factor = -1.5 * j2 * gm * equatorial_radius * equatorial_radius / (r * r * r * r);
    let common = 1.0 - 5.0 * z_over_r * z_over_r;
    Vec3::new(
        factor * (r_vec.x / r) * common,
        factor * (r_vec.y / r) * common,
        factor * (r_vec.z / r) * (common + 2.0),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A5: $J_2$歳差 — 昇交点(RAAN)の歳差率が解析式
    /// $\dot\Omega = -\frac32 n J_2 (R_e/p)^2\cos i$(標準的な軌道摂動論の結果)とrel<2%で
    /// 一致することを確認する(docs/21-verification/01-analytic-tests.md A5)。円軌道
    /// (離心率0、傾斜45°)を地球のJ2でvelocity Verlet積分し、角運動量ベクトルから
    /// 昇交点方向(節線ベクトル $\hat z\times h$)の回転角を追跡する。
    #[test]
    fn a5_nodal_precession_rate_matches_j2_analytic_formula() {
        let gm = 3.986_004_418e14;
        let j2 = EARTH_J2;
        let re = EARTH_EQUATORIAL_RADIUS;

        let altitude = 700e3;
        let a = re + altitude;
        let inclination: f64 = 45.0_f64.to_radians();
        let p = a; // 離心率0なのでsemi-latus rectum = a
        let n_mean_motion = (gm / a.powi(3)).sqrt();

        let analytic_rate = -1.5 * n_mean_motion * j2 * (re / p).powi(2) * inclination.cos();

        let v_circular = (gm / a).sqrt();
        let mut r_vec = Vec3::new(a, 0.0, 0.0);
        let mut v_vec = Vec3::new(
            0.0,
            v_circular * inclination.cos(),
            v_circular * inclination.sin(),
        );

        let accel = |r: Vec3| -> Vec3 {
            let newtonian = r.scale(-gm / r.length().powi(3));
            newtonian + j2_acceleration(gm, j2, re, r)
        };

        let period = 2.0 * std::f64::consts::PI / n_mean_motion;
        let orbits = 50;
        let steps_per_orbit = 2000;
        let dt = period / steps_per_orbit as f64;

        let raan = |r: Vec3, v: Vec3| -> f64 {
            let h = r.cross(v);
            h.x.atan2(-h.y)
        };

        let initial_raan = raan(r_vec, v_vec);
        let mut unwrapped_raan = initial_raan;
        let mut prev_raan = initial_raan;

        let mut a_old = accel(r_vec);
        for _ in 0..(orbits * steps_per_orbit) {
            r_vec = r_vec
                .addcarry_scaled(v_vec, dt)
                .addcarry_scaled(a_old, 0.5 * dt * dt);
            let a_new = accel(r_vec);
            v_vec = v_vec.addcarry_scaled(a_old + a_new, 0.5 * dt);
            a_old = a_new;

            let raw_raan = raan(r_vec, v_vec);
            let mut delta = raw_raan - prev_raan;
            if delta > std::f64::consts::PI {
                delta -= 2.0 * std::f64::consts::PI;
            } else if delta < -std::f64::consts::PI {
                delta += 2.0 * std::f64::consts::PI;
            }
            unwrapped_raan += delta;
            prev_raan = raw_raan;
        }

        let total_time = (orbits * steps_per_orbit) as f64 * dt;
        let measured_rate = (unwrapped_raan - initial_raan) / total_time;
        let rel_err = (measured_rate - analytic_rate).abs() / analytic_rate.abs();
        assert!(
            rel_err < 0.02,
            "measured={measured_rate:e} analytic={analytic_rate:e} rel_err={rel_err:.4}"
        );
    }
}
