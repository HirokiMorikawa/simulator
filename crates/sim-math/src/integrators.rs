//! 数値積分カタログ(汎用部分)。設計: docs/01-math/03-integrators.md。
//!
//! ここに置くのは「状態非依存」の汎用積分プリミティブ(§2.1–2.4, §2.6)のみ。
//! `RigidIntegrator` トレイト(§5)は `RigidBodySet`(P1、`sim-mechanics`)に
//! 依存するため、レイヤ依存規則(docs/00-foundation/04-architecture.md §2:
//! sim-math ← sim-core ← ドメイン solver の一方向)に従い `sim-mechanics` 側で
//! P1 実装時に定義する。同様に陰的 Euler(拡散項、`Grid3`/PCG が必要)・
//! leapfrog(Yee)・split-step Fourier・XPBD・semi-Lagrangian・BAOAB/Euler–Maruyama は
//! それぞれの状態型を持つドメイン crate(P3–P5)で実装する。

use crate::Vec3;

/// explicit Euler(教材・比較用、§2.1)。振動系でエネルギーが単調増加する不安定な参考実装。
pub fn explicit_euler_step(x: Vec3, v: Vec3, a: Vec3, dt: f64) -> (Vec3, Vec3) {
    let v_new = v.addcarry_scaled(a, dt);
    let x_new = x.addcarry_scaled(v, dt);
    (x_new, v_new)
}

/// semi-implicit(symplectic)Euler — 剛体の既定積分器(§2.2)。
pub fn semi_implicit_euler_step(x: Vec3, v: Vec3, a: Vec3, dt: f64) -> (Vec3, Vec3) {
    let v_new = v.addcarry_scaled(a, dt);
    let x_new = x.addcarry_scaled(v_new, dt);
    (x_new, v_new)
}

/// velocity Verlet — 分子動力学の既定(§2.3)。二次精度・シンプレクティック・時間反転対称。
/// `accel` は位置のみに依存する保存力場(分子間ポテンシャル・重力など)を想定する。
pub fn velocity_verlet_step(
    x: Vec3,
    v: Vec3,
    accel: impl Fn(Vec3) -> Vec3,
    dt: f64,
) -> (Vec3, Vec3) {
    let a_n = accel(x);
    let x_new = x.addcarry_scaled(v, dt).addcarry_scaled(a_n, 0.5 * dt * dt);
    let a_next = accel(x_new);
    let v_new = v.addcarry_scaled(a_n + a_next, 0.5 * dt);
    (x_new, v_new)
}

/// 古典的4段 RK4(§2.4)。無衝突の滑らかな系(弾道・軌道)の基準解生成用。
/// `accel(x, v)` は速度依存の力(抗力など)も許す。
pub fn rk4_step(x: Vec3, v: Vec3, accel: impl Fn(Vec3, Vec3) -> Vec3, dt: f64) -> (Vec3, Vec3) {
    let (k1x, k1v) = (v, accel(x, v));

    let x2 = x.addcarry_scaled(k1x, dt * 0.5);
    let v2 = v.addcarry_scaled(k1v, dt * 0.5);
    let (k2x, k2v) = (v2, accel(x2, v2));

    let x3 = x.addcarry_scaled(k2x, dt * 0.5);
    let v3 = v.addcarry_scaled(k2v, dt * 0.5);
    let (k3x, k3v) = (v3, accel(x3, v3));

    let x4 = x.addcarry_scaled(k3x, dt);
    let v4 = v.addcarry_scaled(k3v, dt);
    let (k4x, k4v) = (v4, accel(x4, v4));

    let dx = (k1x + k2x.scale(2.0) + k3x.scale(2.0) + k4x).scale(dt / 6.0);
    let dv = (k1v + k2v.scale(2.0) + k3v.scale(2.0) + k4v).scale(dt / 6.0);
    (x + dx, v + dv)
}

/// 無衝突専用の RK4 積分器(§5)。接触・拘束とは混ぜない。
pub struct BallisticIntegrator;

impl BallisticIntegrator {
    pub fn step(
        &self,
        x: Vec3,
        v: Vec3,
        accel: impl Fn(Vec3, Vec3) -> Vec3,
        dt: f64,
    ) -> (Vec3, Vec3) {
        rk4_step(x, v, accel, dt)
    }
}

/// 電磁場中の荷電粒子・帯電点質量専用(§2.6)。剛体接触とは併用しない。
/// 磁場回転部は厳密な回転で速さを(丸め誤差を除き)保存する。
pub struct BorisPusher;

impl BorisPusher {
    /// `e`・`b` は粒子位置で評価済みの場。
    pub fn step(&self, x: &mut Vec3, v: &mut Vec3, q_over_m: f64, e: Vec3, b: Vec3, dt: f64) {
        let half_dt = dt * 0.5;
        let v_minus = v.addcarry_scaled(e, q_over_m * half_dt);
        let t = b.scale(q_over_m * half_dt);
        let s = t.scale(2.0 / (1.0 + t.length_sq()));
        let v_prime = v_minus + v_minus.cross(t);
        let v_plus = v_minus + v_prime.cross(s);
        let v_new = v_plus.addcarry_scaled(e, q_over_m * half_dt);
        *x = x.addcarry_scaled(v_new, dt);
        *v = v_new;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const OMEGA: f64 = 2.0;

    fn harmonic_accel(x: Vec3) -> Vec3 {
        x.scale(-OMEGA * OMEGA)
    }

    fn harmonic_energy(x: Vec3, v: Vec3) -> f64 {
        0.5 * v.length_sq() + 0.5 * OMEGA * OMEGA * x.length_sq()
    }

    /// §6: explicit Euler は振動系でエネルギーが単調増加する(不安定)ことを確認する。
    #[test]
    fn explicit_euler_diverges_on_harmonic_oscillator() {
        let dt = 0.01;
        let mut x = Vec3::new(1.0, 0.0, 0.0);
        let mut v = Vec3::ZERO;
        let e0 = harmonic_energy(x, v);
        for _ in 0..(2000.0 / dt) as u32 {
            let (nx, nv) = explicit_euler_step(x, v, harmonic_accel(x), dt);
            x = nx;
            v = nv;
        }
        let e_final = harmonic_energy(x, v);
        assert!(
            e_final > e0 * 10.0,
            "expected divergent energy growth, got {e_final} vs {e0}"
        );
    }

    /// §6: semi-implicit Euler はエネルギーが有界に留まる(シンプレクティック)。
    #[test]
    fn semi_implicit_euler_keeps_energy_bounded() {
        let dt = 0.01;
        let mut x = Vec3::new(1.0, 0.0, 0.0);
        let mut v = Vec3::ZERO;
        let e0 = harmonic_energy(x, v);
        let mut max_e = e0;
        for _ in 0..(2000.0 / dt) as u32 {
            let (nx, nv) = semi_implicit_euler_step(x, v, harmonic_accel(x), dt);
            x = nx;
            v = nv;
            max_e = max_e.max(harmonic_energy(x, v));
        }
        assert!(
            max_e < e0 * 1.1,
            "energy should stay bounded, got max {max_e} vs {e0}"
        );
    }

    /// §6 収束次数(◆): semi-implicit Euler は一次収束。
    #[test]
    fn semi_implicit_euler_converges_at_first_order() {
        let t_final = 1.0;
        let x0 = Vec3::new(1.0, 0.0, 0.0);
        let analytic = x0.scale((OMEGA * t_final).cos());

        let run = |dt: f64| -> f64 {
            let mut x = x0;
            let mut v = Vec3::ZERO;
            let n = (t_final / dt).round() as u32;
            for _ in 0..n {
                let (nx, nv) = semi_implicit_euler_step(x, v, harmonic_accel(x), dt);
                x = nx;
                v = nv;
            }
            (x - analytic).length()
        };

        let errors: Vec<f64> = [0.01, 0.005, 0.0025, 0.00125]
            .iter()
            .map(|&dt| run(dt))
            .collect();
        for w in errors.windows(2) {
            let order = (w[0] / w[1]).log2();
            assert!(
                (order - 1.0).abs() < 0.3,
                "expected ~1st order, got {order}"
            );
        }
    }

    /// §6 収束次数(◆): velocity Verlet は二次収束。
    #[test]
    fn velocity_verlet_converges_at_second_order() {
        let t_final = 1.0;
        let x0 = Vec3::new(1.0, 0.0, 0.0);
        let analytic = x0.scale((OMEGA * t_final).cos());

        let run = |dt: f64| -> f64 {
            let mut x = x0;
            let mut v = Vec3::ZERO;
            let n = (t_final / dt).round() as u32;
            for _ in 0..n {
                let (nx, nv) = velocity_verlet_step(x, v, harmonic_accel, dt);
                x = nx;
                v = nv;
            }
            (x - analytic).length()
        };

        let errors: Vec<f64> = [0.02, 0.01, 0.005, 0.0025]
            .iter()
            .map(|&dt| run(dt))
            .collect();
        for w in errors.windows(2) {
            let order = (w[0] / w[1]).log2();
            assert!(
                (order - 2.0).abs() < 0.3,
                "expected ~2nd order, got {order}"
            );
        }
    }

    /// §6 収束次数(◆): RK4 は四次収束。
    #[test]
    fn rk4_converges_at_fourth_order() {
        let t_final = 1.0;
        let x0 = Vec3::new(1.0, 0.0, 0.0);
        let analytic = x0.scale((OMEGA * t_final).cos());

        let run = |dt: f64| -> f64 {
            let mut x = x0;
            let mut v = Vec3::ZERO;
            let n = (t_final / dt).round() as u32;
            for _ in 0..n {
                let (nx, nv) = rk4_step(x, v, |x, _v| harmonic_accel(x), dt);
                x = nx;
                v = nv;
            }
            (x - analytic).length()
        };

        let errors: Vec<f64> = [0.1, 0.05, 0.025, 0.0125]
            .iter()
            .map(|&dt| run(dt))
            .collect();
        for w in errors.windows(2) {
            let order = (w[0] / w[1]).log2();
            assert!(
                (order - 4.0).abs() < 0.3,
                "expected ~4th order, got {order}"
            );
        }
    }

    /// velocity Verlet はケプラー軌道(逆二乗中心力)で長時間のエネルギードリフトが無い
    /// (§6「ケプラー軌道1000周」の縮小版。フルの A2 相当検証は Pα ウェーブが担う)。
    #[test]
    fn velocity_verlet_conserves_energy_on_circular_orbit() {
        let gm: f64 = 1.0;
        let r0: f64 = 1.0;
        let v0 = (gm / r0).sqrt(); // 円軌道速度
        let accel = |x: Vec3| -> Vec3 {
            let r = x.length();
            x.scale(-gm / (r * r * r))
        };
        let energy = |x: Vec3, v: Vec3| -> f64 { 0.5 * v.length_sq() - gm / x.length() };

        let mut x = Vec3::new(r0, 0.0, 0.0);
        let mut v = Vec3::new(0.0, v0, 0.0);
        let e0 = energy(x, v);

        let dt = 0.001;
        let period = 2.0 * std::f64::consts::PI * (r0 * r0 * r0 / gm).sqrt();
        let steps = (100.0 * period / dt) as u32; // 100 周
        for _ in 0..steps {
            let (nx, nv) = velocity_verlet_step(x, v, accel, dt);
            x = nx;
            v = nv;
        }
        let e_final = energy(x, v);
        let rel_drift = (e_final - e0).abs() / e0.abs();
        assert!(
            rel_drift < 1e-4,
            "relative energy drift too large: {rel_drift}"
        );
    }

    /// E2 相当: Boris pusher はサイクロトロン運動で速さを厳密に保存する。
    #[test]
    fn boris_pusher_preserves_speed_in_pure_magnetic_field() {
        let pusher = BorisPusher;
        let b = Vec3::new(0.0, 0.0, 1.0);
        let e = Vec3::ZERO;
        let q_over_m = 1.0;
        let mut x = Vec3::new(1.0, 0.0, 0.0);
        let mut v = Vec3::new(0.0, 1.0, 0.0);
        let speed0 = v.length();
        let dt = 0.001;
        for _ in 0..20_000 {
            pusher.step(&mut x, &mut v, q_over_m, e, b, dt);
            assert!((v.length() - speed0).abs() < 1e-9);
        }
    }

    /// Boris pusher の半径がサイクロトロン半径 r = mv/(qB) の解析値と一致する。
    #[test]
    fn boris_pusher_matches_cyclotron_radius() {
        let pusher = BorisPusher;
        let b_mag = 2.0;
        let b = Vec3::new(0.0, 0.0, b_mag);
        let q_over_m = 1.0;
        let speed = 3.0;
        let expected_radius = speed / (q_over_m * b_mag); // r = v/((q/m) B)

        // dv_x/dt|0 = (q/m) v_y Bz。x0=(+r,0,0) から原点へ向心加速度が向くよう
        // v_y の符号を選ぶ(q/m, Bz > 0 のとき v_y < 0 が中心=原点の配置になる)。
        let mut x = Vec3::new(expected_radius, 0.0, 0.0);
        let mut v = Vec3::new(0.0, -speed, 0.0);
        let dt = 1e-4;
        let mut max_r_dev: f64 = 0.0;
        for _ in 0..20_000 {
            pusher.step(&mut x, &mut v, q_over_m, Vec3::ZERO, b, dt);
            max_r_dev = max_r_dev.max((x.length() - expected_radius).abs());
        }
        assert!(
            max_r_dev / expected_radius < 1e-3,
            "radius deviation {max_r_dev}"
        );
    }
}
