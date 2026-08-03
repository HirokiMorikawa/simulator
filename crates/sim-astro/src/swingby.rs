//! スイングバイ(パッチドコニック近似によるフライバイ速度変化)。設計:
//! docs/16-astro/02-orbital-mechanics.md §4「大気圏に入ると自動で微細刻み」の直前の節が
//! 挙げる遊び方「スイングバイで加速する」、同docs §4の数値解法の受け入れ基準
//! 「スイングバイ: 双曲線通過前後の速度ベクトル変化がパッチドコニック解析と一致(±1%)」、
//! docs/21-verification/03-demo-scenarios.md D36。
//!
//! パッチドコニック近似(惑星の重力だけを考え、恒星の重力は通過中無視する)では、
//! 惑星基準系での探査機の速度は通過前後で大きさが変わらず(等方的な2体問題のエネルギー
//! 保存)、方向だけが偏向角$\delta$だけ回転する。この`nbody::NBodySystem`(実際の
//! leapfrog積分)を使った検証は、探査機質量を惑星質量に対して無視できるほど小さくする
//! ことで(惑星の反作用が無視できる制限2体問題)、周回中の惑星自身の速度(公転相当)を
//! 保ったまま探査機の慣性系速度が変化する——という「スイングバイで加速する」効果も
//! 副次的に確認できる。

/// 双曲線軌道の離心率(標準重力パラメータ`mu`=GM、近点距離`r_periapsis`、
/// 双曲線超過速度`v_infinity`から)。
pub fn hyperbolic_eccentricity(mu: f64, r_periapsis: f64, v_infinity: f64) -> f64 {
    1.0 + r_periapsis * v_infinity * v_infinity / mu
}

/// パッチドコニック近似によるスイングバイの偏向角$\delta = 2\arcsin(1/e)$
/// (惑星基準系での速度ベクトルが、通過前の漸近方向から通過後の漸近方向へ曲がる
/// 全角度)。
pub fn patched_conic_deflection_angle(eccentricity: f64) -> f64 {
    2.0 * (1.0 / eccentricity).asin()
}

/// 近点での速度(vis-viva、双曲線軌道: $v^2 = v_\infty^2 + 2\mu/r_p$)。
pub fn periapsis_speed(mu: f64, r_periapsis: f64, v_infinity: f64) -> f64 {
    (v_infinity * v_infinity + 2.0 * mu / r_periapsis).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nbody::NBodySystem;
    use sim_core::{EventQueue, MaterialDb, Solver, SolverContext};
    use sim_math::{SimRng, Vec3};

    /// D36 スイングバイ(モジュールdoc参照)。惑星(公転速度`planet_velocity`を持つ、
    /// 探査機質量は惑星質量に対して無視できるほど小さく、惑星への反作用が無視できる
    /// 制限2体問題)+探査機を、近点(通過前後の対称点、周期関係から近点速度は
    /// vis-vivaで厳密に求まる)から`NBodySystem::step()`(実際のleapfrog)で
    /// 十分遠方(近点距離の200倍)まで積分し、
    /// (1) 惑星基準系での速度の大きさが$v_\infty$へ収束する(エネルギー保存、rel<1%)
    /// (2) 惑星基準系での速度方向が、パッチドコニック解析の半偏向角$\arcsin(1/e)$
    ///     (近点は通過前後の対称点なので全偏向角$\delta$のちょうど半分)だけ回転する
    ///     方向に一致する(rel<1%)
    /// (3) 惑星自身の速度はほぼ変化せず(反作用無視できる)、探査機の慣性系速度が
    ///     「惑星速度+回転後の惑星基準相対速度」と一致する(スイングバイで探査機の
    ///     慣性系速度が変わる効果の直接確認)
    /// ことを確認する(設計docs/16-astro/02-orbital-mechanics.md §4の受け入れ基準
    /// 「双曲線通過前後の速度ベクトル変化がパッチドコニック解析と一致(±1%)」)。
    #[test]
    fn d36_swingby_velocity_turn_matches_patched_conic_analysis_within_one_percent() {
        let g = crate::nbody::GRAVITATIONAL_CONSTANT;
        let planet_mass = 1.0e24;
        let mu = g * planet_mass;
        let v_infinity = 5000.0;
        let r_periapsis = 5.0e6;

        let eccentricity = hyperbolic_eccentricity(mu, r_periapsis, v_infinity);
        let half_deflection = patched_conic_deflection_angle(eccentricity) / 2.0;
        let v_peri = periapsis_speed(mu, r_periapsis, v_infinity);

        let planet_velocity = Vec3::new(0.0, 20_000.0, 0.0); // 公転速度相当(任意値)
        let mut sys = NBodySystem::new(0.0);
        let planet = sys.add_body(Vec3::ZERO, planet_velocity, planet_mass);
        // 近点(通過前後の対称点): 惑星から見て+x方向に`r_periapsis`、速度は近点では
        // 純粋に接線方向(+y、惑星速度に対する相対速度として)。
        let probe_mass = 1.0; // 惑星質量比1e-24、反作用は無視できる
        let probe = sys.add_body(
            Vec3::new(r_periapsis, 0.0, 0.0),
            planet_velocity + Vec3::new(0.0, v_peri, 0.0),
            probe_mass,
        );

        let dt = 5.0;
        let r_far = 200.0 * r_periapsis;
        let materials = MaterialDb::standard();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();
        let max_steps = 200_000u32;
        let mut steps_taken = 0u32;
        loop {
            let relative_distance = (sys.position[probe] - sys.position[planet]).length();
            if relative_distance >= r_far || steps_taken >= max_steps {
                break;
            }
            let mut ctx = SolverContext {
                materials: &materials,
                rng: &mut rng,
                events: &mut events,
            };
            sys.step(dt, &mut ctx);
            steps_taken += 1;
        }
        assert!(
            steps_taken < max_steps,
            "probe should have reached r_far well within the step budget"
        );

        let planet_velocity_final = sys.velocity[planet];
        let relative_velocity_final = sys.velocity[probe] - planet_velocity_final;
        let speed_final = relative_velocity_final.length();

        let rel_err_speed = (speed_final - v_infinity).abs() / v_infinity;
        assert!(
            rel_err_speed < 0.01,
            "planet-frame speed should converge to v_infinity (energy conservation): \
             speed_final={speed_final} v_infinity={v_infinity} rel_err={rel_err_speed:.5}"
        );

        // 近点接線方向(+y)を惑星の公転面内(z軸まわり)でhalf_deflectionだけ
        // 反時計回り(探査機の角運動量の符号と一致する向き)に回転させた方向。
        let expected_direction = Vec3::new(-half_deflection.sin(), half_deflection.cos(), 0.0);
        let actual_direction = relative_velocity_final.scale(1.0 / speed_final);
        let cos_angle_err = expected_direction.dot(actual_direction).clamp(-1.0, 1.0);
        let angle_err = cos_angle_err.acos();
        let rel_angle_err = angle_err / half_deflection;
        assert!(
            rel_angle_err.abs() < 0.01,
            "planet-frame velocity direction should match the patched-conic half-deflection: \
             expected_direction={expected_direction:?} actual_direction={actual_direction:?} \
             half_deflection={half_deflection} angle_err={angle_err} rel_err={rel_angle_err:.5}"
        );

        // 惑星自身の速度はほぼ変化しない(探査機質量が無視できるほど小さいため)。
        let planet_speed_change =
            (planet_velocity_final - planet_velocity).length() / planet_velocity.length();
        assert!(
            planet_speed_change < 1e-6,
            "planet velocity should be essentially unaffected by the negligible-mass probe: \
             change={planet_speed_change:e}"
        );

        // スイングバイによる探査機の慣性系速度変化(「惑星速度+回転後の相対速度」と
        // 実際の探査機速度が一致することを確認、加速効果の直接検算)。
        let predicted_probe_velocity = planet_velocity_final + expected_direction.scale(v_infinity);
        let actual_probe_velocity = sys.velocity[probe];
        let rel_velocity_err = (predicted_probe_velocity - actual_probe_velocity).length()
            / actual_probe_velocity.length();
        assert!(
            rel_velocity_err < 0.01,
            "inertial probe velocity after the flyby should match planet_velocity + rotated \
             relative velocity: predicted={predicted_probe_velocity:?} \
             actual={actual_probe_velocity:?} rel_err={rel_velocity_err:.5}"
        );
    }
}
