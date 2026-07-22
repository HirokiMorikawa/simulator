//! `MotorCoupling`(設計 docs/20-integration/01-coupling-matrix.md §3「P4: 回路 ⇔ ヒンジ ⇔
//! 熱[13-em/05]」、「手回し発電」統合シナリオ docs/20-integration/01-coupling-matrix.md §5)。
//!
//! **縮約実装の理由**: `sim_em::DcMotor`は既にこの物理(逆起電力・トルク定数)を
//! 自己完結した専用型として実装済みだが、`InductionRod`と同様`sim_mechanics`の剛体・
//! `sim_em::Circuit`の回路網とは独立な「自前でcurrent/angular_velocityを持つミニ統合
//! クラス」である。本`Coupling`は同じ物理($\mathcal{E}=k\omega$、$\tau=ki$)を、
//! 実際の`MechanicsSolver`の剛体(固定軸まわりの回転)+`Circuit`の抵抗回路という
//! 2つの正典ドメイン間の橋として実装し直す — `InductionCoupling`(並進版)の回転版に
//! あたる。剛体の回転軸はワールド座標の固定軸(`axis`)まわりの1自由度に限定する
//! (正式なHingeジョイントは未実装、`HingeMotorPd`の縮約と同じ精神)。
//!
//! `InductionCoupling`と同じ理由(モジュールdoc参照)で、単一`apply`呼び出し内に
//! 「今step確定した角速度から次の回路stepへ渡す起電力を設定」+「前回の回路stepで
//! 解かれた電流から反作用トルクを`torque_accum`に積む(次stepの`integrate_velocities`
//! で消費される、`Command::ApplyForce`の`torque_accum`と同じ消費経路)」を両方行う
//! 1step遅れの縮約版とする。反作用トルクは`Dynamic`剛体にのみ意味を持つ
//! (`Kinematic`剛体は`torque_accum`を無視して外部指定の角速度をそのまま維持する設計
//! ——「手回し発電」シナリオで手が任意の負荷に対して一定回転数を保つ理想化に対応)。

use crate::domain_states::{Coupling, DomainStates};
use sim_core::DomainId;
use sim_math::Vec3;

/// 固定軸`axis`まわりに回転する剛体(`body_index`)と回路の電圧源
/// (`voltage_source_index`)を、トルク定数`torque_constant`($k=k_e=k_t$)で結ぶ
/// (モジュールdoc参照)。
pub struct MotorCoupling {
    pub body_index: usize,
    /// ワールド座標の回転軸(固定、単位ベクトル)。
    pub axis: Vec3,
    pub voltage_source_index: usize,
    pub torque_constant: f64,
}

impl Coupling for MotorCoupling {
    fn domains(&self) -> (DomainId, DomainId) {
        (DomainId::Mechanics, DomainId::Electromagnetism)
    }

    fn apply(&mut self, world: &mut DomainStates, _dt: f64) {
        let Some(circuit) = &mut world.em_circuit else {
            return;
        };

        // 反作用トルク(前回の回路stepで解かれた電流、モジュールdoc「1step遅れ」参照)。
        // `torque_accum`へ積む(`Command::ApplyForce`と同じ消費経路、次stepの
        // `integrate_velocities`で消費される)。符号は`InductionCoupling`と同じ経験的
        // 確認による(発電時に回転を妨げる向き)。
        let current = circuit.source_current(self.voltage_source_index);
        let torque = self.torque_constant * current;
        let idx = self.body_index;
        world.mechanics.bodies.torque_accum[idx] =
            world.mechanics.bodies.torque_accum[idx] + self.axis.scale(torque);

        // ファラデー則の起電力(今step確定した角速度、次の回路stepで使われる)。
        let omega = world.mechanics.bodies.angular_velocity[idx].dot(self.axis);
        let emf = self.torque_constant * omega;
        circuit.set_voltage_source_voltage(self.voltage_source_index, emf);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::MaterialDb;
    use sim_em::{Circuit, GROUND};
    use sim_mechanics::{BodyType, MechanicsSolver, RigidBodyDesc, Shape};

    /// 「手回し発電」統合シナリオの核: 一定回転数(理想化された手回し、`Kinematic`剛体
    /// なので反作用トルクの影響を受けない、モジュールdoc参照)で回る軸が
    /// `MotorCoupling`経由で回路にEMF($\mathcal{E}=k\omega$)を供給し、抵抗負荷での
    /// 定常電力が理論値$V^2/R=(k\omega)^2/R$とrel<1%で一致することを確認する
    /// (実測はほぼ厳密一致 — `Kinematic`剛体の角速度は毎step確定的に一定なため、
    /// `InductionCoupling`の1step遅れのような誤差要因がそもそも生じない)。
    #[test]
    fn motor_coupling_generates_emf_matching_k_omega_at_steady_state() {
        let materials = MaterialDb::standard();
        let mut mechanics = MechanicsSolver::new(0.0);
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();

        let omega0 = 10.0; // rad/s、一定回転数(理想化された手回し)
        let k = 0.05; // N·m/A = V·s/rad
        let r = 10.0;

        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.05 }, steel);
        desc.body_type = BodyType::Kinematic;
        desc.angular_velocity = Vec3::new(0.0, omega0, 0.0);
        let body_idx = mechanics.create_body(desc, &materials);

        let mut circuit = Circuit::new(2);
        circuit.add_voltage_source(1, GROUND, 0.0); // index 0、MotorCouplingがEMFで駆動
        circuit.add_resistor(1, GROUND, r);

        let mut coupling = MotorCoupling {
            body_index: body_idx,
            axis: Vec3::new(0.0, 1.0, 0.0),
            voltage_source_index: 0,
            torque_constant: k,
        };

        let dt = 0.001;
        for _ in 0..500 {
            // 抵抗回路のみ(RC/RL要素なし)なので初回解で即座に定常状態に達する。
            coupling.apply(
                &mut DomainStates {
                    mechanics: &mut mechanics,
                    thermal: None,
                    em_circuit: Some(&mut circuit),
                    em_electrostatics: None,
                    gas: None,
                },
                dt,
            );
            circuit.step(dt);
        }

        let expected_emf = k * omega0;
        let expected_power = expected_emf * expected_emf / r;
        let measured_power = circuit.resistor_power(0);
        let rel_err = (measured_power - expected_power).abs() / expected_power;
        assert!(
            rel_err < 0.01,
            "measured_power={measured_power} expected_power={expected_power} rel_err={rel_err:.4}"
        );
    }
}
