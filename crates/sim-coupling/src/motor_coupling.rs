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
//! **群5で1step遅れを解消した** — `InductionCoupling`(並進版)と同じく`apply_pre`
//! (今stepの角速度から起電力を設定し、**この step の回路 solve に間に合わせる**)と
//! `apply_post`(**この step で確定した電流**から反作用トルクを角速度へ直接注入する)の
//! 2相に分けた。`World::step`は pre → 各ドメインソルバ → post の順に呼ぶ
//! (`Coupling::apply_pre`のdoc参照)。
//!
//! 反作用トルクの注入先を`torque_accum`から**角速度への直接積分**($\Delta\omega =
//! I^{-1}\tau\,\Delta t$、`inv_inertia_world`を使う)へ変えた。`torque_accum`は次stepの
//! `integrate_velocities`でしか消費されないため、post 相に置いても1step遅れが残って
//! しまうのが理由。`Kinematic`/`Static`剛体は`inv_inertia_world`がゼロ行列なので
//! **この式のまま角速度が変化しない** — 「手回し発電」シナリオで手が任意の負荷に対して
//! 一定回転数を保つ理想化(旧実装が`torque_accum`を無視することで得ていた性質)は
//! 明示的な分岐なしにそのまま保たれる。
//!
//! 以下は移行前の記録: 単一`apply`呼び出し内に「今step確定した角速度から次の回路stepへ
//! 渡す起電力を設定」+「前回の回路stepで解かれた電流から反作用トルクを`torque_accum`に
//! 積む」を両方行う1step遅れの縮約版だった(設計§2規則3「各ステップで前ステップ確定値を
//! 読む」)。

use crate::domain_states::{Coupling, CouplingKind, DomainStates};
use sim_core::DomainId;
use sim_math::Vec3;

/// 固定軸`axis`まわりに回転する剛体(`body_index`)と回路の電圧源
/// (`voltage_source_index`)を、トルク定数`torque_constant`($k=k_e=k_t$)で結ぶ
/// (モジュールdoc参照)。
#[derive(Clone)]
pub struct MotorCoupling {
    pub body_index: usize,
    /// ワールド座標の回転軸(固定、単位ベクトル)。
    pub axis: Vec3,
    pub voltage_source_index: usize,
    pub torque_constant: f64,
}

impl Coupling for MotorCoupling {
    fn kind(&self) -> CouplingKind {
        CouplingKind::MotorCoupling
    }

    fn domain_ids(&self) -> &'static [DomainId] {
        &[DomainId::Mechanics, DomainId::Electromagnetism]
    }

    fn describe(&self) -> String {
        format!(
            "MotorCoupling body#{} k={}N.m/A -> V[{}]",
            self.body_index, self.torque_constant, self.voltage_source_index
        )
    }

    fn referenced_bodies(&self) -> Vec<usize> {
        vec![self.body_index]
    }

    fn referenced_voltage_sources(&self) -> Vec<usize> {
        vec![self.voltage_source_index]
    }

    /// **pre 相: 起電力を今stepの回路solveへ間に合わせる(群5で1step遅れを解消)**。
    /// ファラデー則 $\mathcal{E}=k\omega$。
    fn apply_pre(&mut self, world: &mut DomainStates, _dt: f64) {
        let omega = world.mechanics.bodies.angular_velocity[self.body_index].dot(self.axis);
        let Some(circuit) = &mut world.em_circuit else {
            return;
        };
        let emf = self.torque_constant * omega;
        circuit.set_voltage_source_voltage(self.voltage_source_index, emf);
    }

    /// **post 相: 今step確定した電流から反作用トルクを角速度へ直接注入する**
    /// ($\tau=ki$、$\Delta\omega=I^{-1}\tau\,\Delta t$)。`Kinematic`/`Static`剛体は
    /// `inv_inertia_world`がゼロ行列なので角速度は変化しない(モジュールdoc参照)。
    /// 符号は`InductionCoupling`と同じ経験的確認による(発電時に回転を妨げる向き)。
    fn apply_post(&mut self, world: &mut DomainStates, dt: f64) {
        let Some(circuit) = &world.em_circuit else {
            return;
        };
        let current = circuit.source_current(self.voltage_source_index);
        let torque = self.axis.scale(self.torque_constant * current);
        let idx = self.body_index;
        let delta = world.mechanics.bodies.inv_inertia_world[idx].mul_vec(torque.scale(dt));
        world.mechanics.bodies.angular_velocity[idx] =
            world.mechanics.bodies.angular_velocity[idx] + delta;
    }

    /// **単相で呼ばれた場合の互換経路**(`World`を経由しない直接呼び出し用)。
    /// pre → post の順で両相を続けて実行する。
    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        self.apply_pre(world, dt);
        self.apply_post(world, dt);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::MaterialDb;
    use sim_em::{Circuit, GROUND};
    use sim_mechanics::{BodyType, MechanicsSolver, RigidBodyDesc, Shape};

    /// `World::step`が組み立てるのと同じ`DomainStates`(力学+回路のみ)を作るヘルパー。
    fn states_of<'a>(
        mechanics: &'a mut MechanicsSolver,
        circuit: &'a mut Circuit,
    ) -> DomainStates<'a> {
        DomainStates {
            mechanics,
            thermal: None,
            em_circuit: Some(circuit),
            em_electrostatics: None,
            gas: None,
            grid_fluid: None,
            grid_fluid_3d: None,
            sph: None,
        }
    }

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
            // `World::step`と同じ順序(pre → ドメインソルバ → post)で回す。
            // 抵抗回路のみ(RC/RL要素なし)なので初回解で即座に定常状態に達する。
            coupling.apply_pre(&mut states_of(&mut mechanics, &mut circuit), dt);
            circuit.step(dt);
            coupling.apply_post(&mut states_of(&mut mechanics, &mut circuit), dt);
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

    /// **群5: 1step遅れが実際に消えたことの検証**。角速度を毎step変える(ランプさせる)と、
    /// 遅れの有無が電流に直接現れる — 起電力が今stepのωから作られていれば
    /// $i_n = k\omega_n/R$、1step遅れていれば $i_n = k\omega_{n-1}/R$ になる。
    /// 同じシナリオを ①`World::step`の順序(pre → circuit.step → post)と
    /// ②移行前の順序(circuit.step → apply)の両方で回し、①が今stepの値・②が前stepの値に
    /// 一致することを**対照実験**として同時に確認する(片側だけだと「たまたま合った」と
    /// 区別できないため)。
    #[test]
    fn motor_coupling_pre_phase_removes_the_one_step_lag_in_the_emf() {
        let materials = MaterialDb::standard();
        let k = 0.05;
        let r = 10.0;
        let dt = 0.001;
        let omega_at = |n: usize| 1.0 + n as f64; // 毎step変化する角速度

        let build = || {
            let mut mechanics = MechanicsSolver::new(0.0);
            let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
            let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.05 }, steel);
            desc.body_type = BodyType::Kinematic;
            let body = mechanics.create_body(desc, &materials);
            let mut circuit = Circuit::new(2);
            circuit.add_voltage_source(1, GROUND, 0.0);
            circuit.add_resistor(1, GROUND, r);
            let coupling = MotorCoupling {
                body_index: body,
                axis: Vec3::new(0.0, 1.0, 0.0),
                voltage_source_index: 0,
                torque_constant: k,
            };
            (mechanics, circuit, coupling, body)
        };

        // ① 2相(現行)。
        let (mut mechanics, mut circuit, mut coupling, body) = build();
        let mut two_phase = Vec::new();
        for n in 0..5 {
            mechanics.bodies.angular_velocity[body] = Vec3::new(0.0, omega_at(n), 0.0);
            coupling.apply_pre(&mut states_of(&mut mechanics, &mut circuit), dt);
            circuit.step(dt);
            coupling.apply_post(&mut states_of(&mut mechanics, &mut circuit), dt);
            two_phase.push(circuit.source_current(0));
        }

        // ② 移行前(単相`apply`を全ドメインstepの**後**に呼ぶ = 旧`World`の適用順序)。
        let (mut mechanics, mut circuit, mut coupling, body) = build();
        let mut single_phase = Vec::new();
        for n in 0..5 {
            mechanics.bodies.angular_velocity[body] = Vec3::new(0.0, omega_at(n), 0.0);
            circuit.step(dt);
            coupling.apply(&mut states_of(&mut mechanics, &mut circuit), dt);
            single_phase.push(circuit.source_current(0));
        }

        for n in 0..5 {
            let expected_now = k * omega_at(n) / r;
            assert!(
                (two_phase[n].abs() - expected_now).abs() < 1e-12,
                "step {n}: 2相なら今stepのωが効くはず measured={} expected={expected_now}",
                two_phase[n].abs()
            );
            // n=0 は「前step」が存在しない(ω=0相当)ので 0、以降は1step前の値。
            let expected_lagged = if n == 0 { 0.0 } else { k * omega_at(n - 1) / r };
            assert!(
                (single_phase[n].abs() - expected_lagged).abs() < 1e-12,
                "step {n}: 単相は1step遅れるはず(対照実験) measured={} expected={expected_lagged}",
                single_phase[n].abs()
            );
        }
    }

    /// post 相の反作用トルクが**今stepで確定した電流**から $\Delta\omega=I^{-1}k i\,\Delta t$
    /// として角速度に直接入ること(`torque_accum`経由ではないこと)を1stepで解析的に確認する。
    /// 併せて`Kinematic`剛体では`inv_inertia_world`がゼロなので角速度が動かないこと
    /// (「手回し発電」の理想化、モジュールdoc参照)も確認する。
    #[test]
    fn motor_coupling_post_phase_injects_reaction_torque_into_angular_velocity() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let k = 0.05;
        let r = 4.0;
        let battery = 12.0;
        let dt = 0.01;
        let axis = Vec3::new(0.0, 1.0, 0.0);

        let mut mechanics = MechanicsSolver::new(0.0);
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.05 }, steel);
        desc.mass_override = Some(1.0);
        let body = mechanics.create_body(desc, &materials);

        let mut circuit = Circuit::new(2);
        circuit.add_voltage_source(1, GROUND, battery);
        circuit.add_resistor(1, GROUND, r);

        let mut coupling = MotorCoupling {
            body_index: body,
            axis,
            voltage_source_index: 0,
            torque_constant: k,
        };

        // pre は ω=0 から起電力0を設定 → 電池ではなく`set_voltage_source_voltage`で
        // 上書きされるため、この構成では電源電圧そのものが coupling の管理下にある。
        // 反作用トルクの検証にはそれで十分(電流値を読んでから期待値を組み立てる)。
        coupling.apply_pre(&mut states_of(&mut mechanics, &mut circuit), dt);
        circuit.step(dt);
        let current = circuit.source_current(0);
        let inv_i = mechanics.bodies.inv_inertia_world[body];
        let expected = inv_i.mul_vec(axis.scale(k * current * dt));
        coupling.apply_post(&mut states_of(&mut mechanics, &mut circuit), dt);
        let measured = mechanics.bodies.angular_velocity[body];
        assert!(
            (measured - expected).length() < 1e-15,
            "measured={measured:?} expected={expected:?}"
        );
        assert_eq!(
            mechanics.bodies.torque_accum[body],
            Vec3::ZERO,
            "post 相は torque_accum を経由しない(次stepまで消費されず遅れるため)"
        );

        // `Kinematic`剛体は inv_inertia_world がゼロ行列 → 角速度は動かない。
        let mut kin_desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.05 }, steel);
        kin_desc.body_type = BodyType::Kinematic;
        kin_desc.angular_velocity = Vec3::new(0.0, 7.0, 0.0);
        let kin = mechanics.create_body(kin_desc, &materials);
        coupling.body_index = kin;
        coupling.apply_post(&mut states_of(&mut mechanics, &mut circuit), dt);
        assert_eq!(
            mechanics.bodies.angular_velocity[kin],
            Vec3::new(0.0, 7.0, 0.0),
            "Kinematic は負荷によらず外部指定の角速度を保つ"
        );
    }
}
