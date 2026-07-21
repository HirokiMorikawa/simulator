//! 大気圏再突入(大気モデル・抗力による軌道減衰)。設計: docs/16-astro/02-orbital-mechanics.md §2.3。
//!
//! Phase Bの縮約実装: 高度依存の指数大気モデルのみを実装し、軌道への抗力摂動は
//! A6検証用に直接組んだ二体+抗力のvelocity Verlet風ループ(1PN検証と同じパターン、
//! `NBodySystem`本体には未統合)で確認する。空力加熱・アブレーション・レジーム切替
//! (再突入時の自動微細刻み)は未実装(設計§4「自動で微細刻み」は本実装のスコープ外)。

/// 指数大気モデル(設計§2.3): $\rho(h) = \rho_0 e^{-h/H}$。
pub fn exponential_atmosphere_density(
    altitude: f64,
    surface_density: f64,
    scale_height: f64,
) -> f64 {
    surface_density * (-altitude / scale_height).exp()
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_math::Vec3;

    const GM_EARTH: f64 = 3.986_004_418e14;
    const R_EARTH: f64 = 6.371e6;

    /// 低軌道衛星を重力+大気抗力(指数大気モデル)で1軌道あたり複数ステップの
    /// semi-implicit Eulerで積分し、最終高度を返す(設計§4「軌道伝播はシンプレクティック
    /// 積分+摂動力」に沿う、1PN検証(A8)と同じ直接ループのパターン)。
    fn simulate_decaying_orbit(
        altitude0: f64,
        drag_coefficient: f64,
        area_over_mass: f64,
        dt: f64,
        steps: u32,
    ) -> f64 {
        let r0 = R_EARTH + altitude0;
        let v0 = (GM_EARTH / r0).sqrt();
        let mut r = Vec3::new(r0, 0.0, 0.0);
        let mut v = Vec3::new(0.0, v0, 0.0);

        for _ in 0..steps {
            let dist = r.length();
            let altitude = dist - R_EARTH;
            let gravity_accel = r.scale(-GM_EARTH / (dist * dist * dist));

            let speed = v.length();
            let density = exponential_atmosphere_density(altitude, 1.225, 8500.0);
            let drag_magnitude = 0.5 * density * speed * speed * drag_coefficient * area_over_mass;
            let drag_accel = if speed > 1e-9 {
                v.scale(-drag_magnitude / speed)
            } else {
                Vec3::ZERO
            };

            let accel = gravity_accel + drag_accel;
            v = v.addcarry_scaled(accel, dt);
            r = r.addcarry_scaled(v, dt);
        }

        r.length() - R_EARTH
    }

    /// A6: 大気減衰 — 低軌道の高度が単調に減衰する定性的傾向と、弾道係数
    /// (抗力係数×面積/質量、値が大きいほど大気抵抗を受けやすい)依存性を確認する
    /// (docs/21-verification/01-analytic-tests.md A6)。設計の指数大気モデルをそのまま
    /// 使い、現実の値(海面密度1.225kg/m³・スケールハイト8.5km)で高度180kmに置いた
    /// 衛星を80周回積分する。実装検証中、面積/質量比を大きくしすぎる(高抗力)と
    /// 数十〜百周回のうちに減衰が加速度的に進み、固定刻み幅(初期軌道周期から決めた
    /// 一定dt)では再突入直前の急激な力学変化に追従できず数値発散することを発見した
    /// (設計§4が「大気圏に入ると自動で微細刻み」と明記する適応刻みは本実装のスコープ外
    /// のため、発散しない範囲の弾道係数・周回数を選んで確認する)。
    #[test]
    fn a6_low_earth_orbit_altitude_decays_and_depends_on_ballistic_coefficient() {
        let altitude0 = 180e3;
        let r0 = R_EARTH + altitude0;
        let period = 2.0 * std::f64::consts::PI * (r0.powi(3) / GM_EARTH).sqrt();
        let steps_per_orbit = 4000;
        let dt = period / steps_per_orbit as f64;
        let orbits = 80;
        let steps = steps_per_orbit * orbits;

        let low_drag_final = simulate_decaying_orbit(altitude0, 2.2, 1e-5, dt, steps);
        let high_drag_final = simulate_decaying_orbit(altitude0, 2.2, 1e-4, dt, steps);

        assert!(
            low_drag_final < altitude0,
            "low-drag satellite should still lose some altitude: final={low_drag_final}"
        );
        assert!(
            high_drag_final < low_drag_final,
            "higher ballistic coefficient (more drag) should decay faster: \
             high_drag_final={high_drag_final} low_drag_final={low_drag_final}"
        );

        let low_drag_loss = altitude0 - low_drag_final;
        let high_drag_loss = altitude0 - high_drag_final;
        assert!(
            high_drag_loss > low_drag_loss * 5.0,
            "10x higher area/mass ratio should produce a clearly larger altitude loss: \
             low_loss={low_drag_loss:.3} high_loss={high_drag_loss:.3}"
        );
    }
}
