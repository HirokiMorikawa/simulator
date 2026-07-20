//! DCモーター(集中定数モデル)。設計: docs/13-electromagnetism/05-em-mechanics-coupling.md §2.3。
//!
//! 設計が示す一般的な結合アーキテクチャ(`MotorCoupling`: 回路のモーター素子と力学の
//! ヒンジを対にし、回路 sub-step + 力学 step の2時間スケールで進行)は、汎用のヒンジ
//! モーター(`10-mechanics/05-joints-constraints.md`)が Phase 5 未実装のため、ここでは
//! 電気側方程式($v=R_ai+L_a\frac{di}{dt}+k_e\omega$)と機械側方程式
//! ($I\dot\omega=k_ti-\tau_{friction}$)を単一のモーター状態として直接連立させる縮約実装
//! (`k=k_e=k_t`により両者が同じ$k$を共有、設計§2.3のエネルギー保存の帰結)。
//! 電流は後退Euler(電気時定数$L_a/R_a$がミリ秒未満で陽解法が不安定なため)、角速度は
//! semi-implicit Euler(他ドメインと同じ積分則)で更新する。

/// DCモーターの状態(電流・角速度)とパラメータ。
pub struct DcMotor {
    pub resistance: f64,      // R_a [Ω]
    pub inductance: f64,      // L_a [H]
    pub k: f64,               // k = k_e = k_t [V·s/rad = N·m/A]
    pub rotor_inertia: f64,   // I [kg·m^2]
    pub friction_torque: f64, // クーロン摩擦(角速度と逆符号、簡略化のため定数)[N·m]
    pub current: f64,
    pub angular_velocity: f64,
}

impl DcMotor {
    pub fn new(resistance: f64, inductance: f64, k: f64, rotor_inertia: f64) -> DcMotor {
        DcMotor {
            resistance,
            inductance,
            k,
            rotor_inertia,
            friction_torque: 0.0,
            current: 0.0,
            angular_velocity: 0.0,
        }
    }

    /// 電圧 `voltage` を印加し外部負荷トルク `external_load_torque` を受けながら1ステップ
    /// 進める。逆起電力 $k\omega$ はステップ内で前ステップの値を使う(設計§4「$\omega$は
    /// ステップ内一定と近似」の縮約: サブステップなし版)。
    pub fn step(&mut self, dt: f64, voltage: f64, external_load_torque: f64) {
        // 後退Euler: i_{n+1}(L/dt + R) = i_n*L/dt + v - k*ω_n
        let back_emf = self.k * self.angular_velocity;
        self.current = (self.current * self.inductance / dt + voltage - back_emf)
            / (self.inductance / dt + self.resistance);

        let torque = self.k * self.current;
        let friction = self.friction_torque * self.angular_velocity.signum();
        let net_torque = torque - friction - external_load_torque;
        self.angular_velocity += dt * net_torque / self.rotor_inertia;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// マブチ FA-130 相当のパラメータ(設計§9)。
    fn fa130() -> DcMotor {
        DcMotor::new(1.0, 0.3e-3, 1.7e-3, 1e-7)
    }

    /// E6前半: 無負荷回転数 $\omega_{nl}\approx V/k$、rel 1%
    /// (docs/21-verification/01-analytic-tests.md E6)。摩擦トルクをゼロにして、
    /// 無負荷(external_load_torque=0)で定常状態(di/dt, dω/dt ≈ 0)まで積分する。
    #[test]
    fn e6_no_load_speed_matches_v_over_k() {
        let mut motor = fa130();
        let voltage = 3.0;
        let dt = 1e-6;
        for _ in 0..2_000_000 {
            motor.step(dt, voltage, 0.0);
        }
        let expected = voltage / motor.k;
        let rel_err = (motor.angular_velocity - expected).abs() / expected;
        assert!(
            rel_err < 0.01,
            "omega={} expected={expected} rel_err={rel_err}",
            motor.angular_velocity
        );
    }

    /// E6後半: ストールトルク $\tau_{stall}=kV/R_a$、rel 1%
    /// (docs/21-verification/01-analytic-tests.md E6)。回転子を強制的に静止させ
    /// (`rotor_inertia`を極端に大きくし角速度がステップ数の範囲では実質動かないようにする)、
    /// 電流が定常値 $V/R_a$ に収束するのを確認する。
    #[test]
    fn e6_stall_torque_matches_kv_over_ra() {
        let mut motor = fa130();
        motor.rotor_inertia = 1e6; // 回転子を事実上静止させる(ストール条件)
        let voltage = 3.0;
        let dt = 1e-6;
        for _ in 0..200_000 {
            motor.step(dt, voltage, 0.0);
        }
        assert!(
            motor.angular_velocity.abs() < 1e-6,
            "rotor should stay effectively stationary: omega={}",
            motor.angular_velocity
        );
        let torque = motor.k * motor.current;
        let expected = motor.k * voltage / motor.resistance;
        let rel_err = (torque - expected).abs() / expected;
        assert!(
            rel_err < 0.01,
            "torque={torque} expected={expected} rel_err={rel_err}"
        );
    }
}
