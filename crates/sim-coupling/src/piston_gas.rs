//! `PistonGas`(設計 docs/20-integration/01-coupling-matrix.md §3「P6: 気体区画 ⇔ 剛体
//! [12-thermal/01, 10-mechanics/05]」、「断熱圧縮」統合シナリオ同 §5)。
//!
//! **前提**: シリンダー壁に沿って1自由度(軸方向)のみ動けるピストン
//! (`sim_mechanics::joint::SliderJoint`で拘束、`joint`モジュールdoc参照)を、
//! `sim_thermal::GasCompartment`(閉じた気体区画)に橋渡しする。ピストンの軸方向変位から
//! 気体体積を算出し`GasCompartment::apply_step_volume_change`(準静的断熱近似の1step版、
//! `sim-thermal::gas`モジュールdoc参照)で温度を更新、更新後の圧力から力
//! $F=pA$ をピストンに印加する。ピストンの機械的time scaleが気体分子の熱化time scale
//! よりずっと長い(準静的近似が成り立つ)ことが前提(設計§4.3と同じ前提)。
//!
//! 気体は常にピストンの`axis`が指す向きに体積が増える側にあるものとして符号を取る
//! (生成時の位置・体積を基準に、軸方向の変位 × 断面積を加減算)。

use crate::domain_states::{Coupling, DomainStates};
use sim_core::DomainId;
use sim_math::Vec3;
use sim_mechanics::RigidBodySet;

/// ピストン(`body_index`)を、軸`axis`(ワールド座標、固定、単位ベクトル、
/// `SliderJoint::axis_a`と同じ想定)に沿って気体区画へ結合する(モジュールdoc参照)。
pub struct PistonGas {
    pub body_index: usize,
    pub axis: Vec3,
    /// ピストン断面積 [m^2]。
    pub area: f64,
    reference_axis_position: f64,
    reference_volume: f64,
}

impl PistonGas {
    /// 現在のピストン位置・現在の気体体積を基準(変位0)として`PistonGas`を生成する。
    pub fn new(
        bodies: &RigidBodySet,
        body_index: usize,
        axis: Vec3,
        area: f64,
        initial_volume: f64,
    ) -> PistonGas {
        PistonGas {
            body_index,
            axis,
            area,
            reference_axis_position: bodies.position[body_index].dot(axis),
            reference_volume: initial_volume,
        }
    }
}

impl Coupling for PistonGas {
    fn domains(&self) -> (DomainId, DomainId) {
        (DomainId::Mechanics, DomainId::Thermal)
    }

    fn apply(&mut self, world: &mut DomainStates, _dt: f64) {
        let Some(gas) = &mut world.gas else {
            return;
        };

        let idx = self.body_index;
        let axis_position = world.mechanics.bodies.position[idx].dot(self.axis);
        let displacement = axis_position - self.reference_axis_position;
        // 気体体積は下限を持つ(ピストンが底突きしても体積0以下にはならない、数値的な
        // 発散防止)。
        let new_volume = (self.reference_volume + self.area * displacement).max(1e-9);
        gas.apply_step_volume_change(new_volume);

        // 気体圧力はピストンを`axis`の正方向(体積が増える向き)へ押す。符号は
        // `InductionCoupling`/`MotorCoupling`と同じ経験的確認による
        // (気体を圧縮する変位に対しては反対向きの復元力になっているはず)。
        let force = self.axis.scale(gas.pressure() * self.area);
        world.mechanics.bodies.force_accum[idx] = world.mechanics.bodies.force_accum[idx] + force;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::{EventQueue, MaterialDb, Solver, SolverContext};
    use sim_math::SimRng;
    use sim_mechanics::{BodyType, MechanicsSolver, RigidBodyDesc, Shape};
    use sim_thermal::{GasCompartment, GasSpecies};

    /// T5(断熱圧縮、`sim-thermal::gas`の単体テストと同じ合格基準)を、実際の
    /// `MechanicsSolver`剛体(`Kinematic`ピストン、モジュールdoc「一定速度で圧縮」)+
    /// `GasCompartment`という2つの正典ドメイン間の`PistonGas`結合経由で再現する。
    /// `Kinematic`剛体を使う理由は`MotorCoupling`と同じ(反作用力の影響を受けず、
    /// 決定的な圧縮速度を保証するため)。
    #[test]
    fn piston_gas_compression_matches_adiabatic_tv_gamma_minus_one_formula() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut mechanics = MechanicsSolver::new(0.0);

        let axis = Vec3::new(1.0, 0.0, 0.0);
        let area = 0.01; // m^2
        let v1 = 1.0e-3; // m^3
        let v2 = v1 / 2.0;
        // 圧縮に必要な変位: 体積半減 = area * distance。
        let distance = (v1 - v2) / area;
        let compress_time = 1.0; // s(準静的とみなせる程度にゆっくり)
        let speed = distance / compress_time;

        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.05 }, steel);
        desc.body_type = BodyType::Kinematic;
        // 気体はaxis正方向にあるので、圧縮(体積減少)にはaxis負方向へ動かす。
        desc.linear_velocity = axis.scale(-speed);
        let piston = mechanics.create_body(desc, &materials);

        let mut gas = GasCompartment {
            n_moles: 1.0,
            volume: v1,
            temperature: 300.0,
            gas: GasSpecies::AIR,
        };
        let gamma = gas.heat_capacity_ratio();

        let mut coupling = PistonGas::new(&mechanics.bodies, piston, axis, area, v1);

        let dt = compress_time / 10_000.0;
        let mut events = EventQueue::new();
        let mut rng = SimRng::new(1, 1);
        for _ in 0..10_000 {
            let mut ctx = SolverContext {
                materials: &materials,
                rng: &mut rng,
                events: &mut events,
            };
            mechanics.step(dt, &mut ctx);
            coupling.apply(
                &mut DomainStates {
                    mechanics: &mut mechanics,
                    thermal: None,
                    em_circuit: None,
                    em_electrostatics: None,
                    gas: Some(&mut gas),
                },
                dt,
            );
        }

        let expected_t2 = 300.0 * (v1 / v2).powf(gamma - 1.0);
        let rel_err = (gas.temperature - expected_t2).abs() / expected_t2;
        assert!(
            rel_err < 0.02,
            "measured={:.4} expected={expected_t2:.4} rel_err={rel_err:.4}",
            gas.temperature
        );
        let rel_volume_err = (gas.volume - v2).abs() / v2;
        assert!(
            rel_volume_err < 0.01,
            "gas volume should track piston displacement: volume={} expected={v2}",
            gas.volume
        );
    }
}
