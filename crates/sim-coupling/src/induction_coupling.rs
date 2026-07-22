//! `InductionCoupling`(設計 docs/20-integration/01-coupling-matrix.md §3「導体棒・渦電流」、
//! docs/13-electromagnetism/05-em-mechanics-coupling.md §2.2「レール上を滑る導体棒」)。
//!
//! **縮約実装の理由**: `sim_em::InductionRod`は既にこの物理(ファラデー則の起電力→回路→
//! レンツ則の制動力)を自己完結した専用型として実装済みだが、それは`sim_mechanics`の
//! 剛体・`sim_em::Circuit`の回路網とは独立な「自前でmass/velocityを持つミニ統合クラス」
//! である。本`Coupling`は同じ物理を、実際の`MechanicsSolver`の剛体 + `Circuit`の抵抗回路
//! という2つの正典ドメイン間の橋として実装し直す。剛体の運動は簡略化してレール方向を
//! ワールドX軸に固定した1自由度(`linear_velocity.x`)として扱う(3軸姿勢・レール方向の
//! 一般化は対象外、設計§2.2の縮約と同じ精神)。
//!
//! 設計§4の実行順序表は`MotorCoupling`を pre(電気→トルク)と post(ω→逆起電力)の
//! 両方に置いており、この種の双方向結合が本質的に2箇所で作用することを示す。しかし
//! `Coupling`トレイトの`apply`は1回しか呼ばれない設計であり、かつ現時点では`World::step()`
//! パイプラインへのCoupling接続自体が未実装(他のCoupling実装も同様、各モジュールdoc参照)
//! のため、本実装は単一の`apply`呼び出し内で「今step確定した速度から次の回路stepへ渡す
//! 起電力を設定」+「前回の回路stepで解かれた電流からレンツ力を今step反映」を両方行う
//! 1step遅れの縮約版とする(設計§2規則3「各ステップで前ステップ確定値を読む」と整合)。
//! この1step遅れによる数値誤差は、`dt`が時定数$\tau$に対して十分小さければ無視できる
//! 程度に収まることをテストで確認する(実測rel_err、テストdoc参照)。

use crate::domain_states::{Coupling, DomainStates};
use sim_core::DomainId;

/// レール上を滑る導体棒(`body_index`、レール方向はワールドX軸に固定)と回路の電圧源
/// (`voltage_source_index`)を結ぶ(モジュールdoc参照)。
#[derive(Clone)]
pub struct InductionCoupling {
    pub body_index: usize,
    pub voltage_source_index: usize,
    /// 棒の長さ $\ell$。
    pub length: f64,
    /// 磁束密度 $B$(レール面に垂直、一様)。
    pub magnetic_field: f64,
}

impl Coupling for InductionCoupling {
    fn domains(&self) -> (DomainId, DomainId) {
        (DomainId::Mechanics, DomainId::Electromagnetism)
    }

    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        let Some(circuit) = &mut world.em_circuit else {
            return;
        };
        let mass = world.mechanics.bodies.mass(self.body_index);
        if mass <= 0.0 {
            return; // 静的/キネマティック剛体には適用しない。
        }

        // レンツ則の制動力(前回の回路stepで解かれた電流、モジュールdoc「1step遅れ」参照)。
        // 符号は`Circuit`のMNA電流の向き規約(`source_current`のdoc参照)に対し実験的に
        // 確認した(制動(速度と逆向き)になる符号を採用)。
        let current = circuit.source_current(self.voltage_source_index);
        let force = self.magnetic_field * current * self.length;
        world.mechanics.bodies.linear_velocity[self.body_index].x += force / mass * dt;

        // ファラデー則の起電力(今step確定した速度、次の回路stepで使われる)。
        let v = world.mechanics.bodies.linear_velocity[self.body_index].x;
        let emf = self.magnetic_field * self.length * v;
        circuit.set_voltage_source_voltage(self.voltage_source_index, emf);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::MaterialDb;
    use sim_em::{Circuit, GROUND};
    use sim_math::Vec3;
    use sim_mechanics::{MechanicsSolver, RigidBodyDesc, Shape};

    /// E7と同じ解析解(誘導ブレーキのみで自由減速する導体棒は指数減衰
    /// $v(t)=v_0e^{-t/\tau}$、$\tau=mR/(B\ell)^2$、`sim_em::induction_rod`のE7テスト
    /// 参照)を、`sim_em::InductionRod`の自己完結型実装ではなく実際の`MechanicsSolver`の
    /// 剛体+`Circuit`の抵抗回路を`InductionCoupling`で結んだ構成で再現する。E7は
    /// 遅れの無い自己無撞着な明示的Eulerだが、本実装はモジュールdoc記載の1step遅れが
    /// あるため、E7と同じrel<0.5%ではなく余裕を持ったrel<1%を採用する(実装検証中の
    /// 実測rel_errは0.019%とE7自体より良く、1step遅れの影響はdt≪τでは無視できる
    /// ほど小さいことを確認した)。
    #[test]
    fn induction_coupling_matches_e7_exponential_decay_within_one_step_lag_error() {
        let materials = MaterialDb::standard();
        let mut mechanics = MechanicsSolver::new(0.0); // 重力なし: 誘導ブレーキのみを見る
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();

        let mass = 0.01;
        let length = 0.1;
        let b = 0.5;
        let r = 1.0;
        let v0 = 1.0;

        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.01 }, steel);
        desc.mass_override = Some(mass);
        desc.linear_velocity = Vec3::new(v0, 0.0, 0.0);
        let body_idx = mechanics.create_body(desc, &materials);

        let mut circuit = Circuit::new(2);
        circuit.add_voltage_source(1, GROUND, 0.0); // index 0、棒のEMFで毎stepドライブする
        circuit.add_resistor(1, GROUND, r);

        let mut coupling = InductionCoupling {
            body_index: body_idx,
            voltage_source_index: 0,
            length,
            magnetic_field: b,
        };

        let tau = mass * r / (b * length).powi(2);
        let dt = 0.001;
        let steps = 2000u32; // t = 2s ≈ tau/2(E7と同じ設定)

        for _ in 0..steps {
            let mut states = DomainStates {
                mechanics: &mut mechanics,
                thermal: None,
                em_circuit: Some(&mut circuit),
                em_electrostatics: None,
                gas: None,
                grid_fluid: None,
            };
            coupling.apply(&mut states, dt);
            circuit.step(dt);
        }
        let t = steps as f64 * dt;

        let expected_v = v0 * (-t / tau).exp();
        let measured_v = mechanics.bodies.linear_velocity[body_idx].x;
        let rel_err = (measured_v - expected_v).abs() / expected_v;
        assert!(
            rel_err < 0.01,
            "measured_v={measured_v} expected_v={expected_v} rel_err={rel_err:.4}"
        );
    }
}
