//! `LorentzForce`(設計 docs/20-integration/01-coupling-matrix.md §3「P4: 静場 → 帯電剛体」)。
//!
//! **縮約実装の理由**: 設計 §3 の`ChargedBody`(剛体+電荷の正式な結合型)はまだ存在しない
//! (`sim_mechanics::RigidBodySet`に電荷フィールドは無い)。そのため本実装は、対象剛体の
//! index と電荷量を`Coupling`自身のフィールドとして持つ縮約版とする(`DissipationToHeat`・
//! `JouleHeat`が単一`ThermalNode`を対象として持つのと同じパターン)。
//!
//! `sim_em::PointChargeSystem`の点電荷群が作る電場(クーロンの法則の直接和)+ 一様外部場
//! (`UniformField`)を対象剛体の位置で評価し、ローレンツ力 $\mathbf F=q(\mathbf E+\mathbf
//! v\times\mathbf B)$ を剛体の速度に直接注入する。設計§1「保存量の橋」の運動量版として、
//! 点電荷群由来のクーロン力(対象剛体と各点電荷の対ごとの力)は、対象剛体に加えた力と
//! ちょうど逆向きの反作用を発生源の点電荷自身の速度にも適用する(Newton第3法則、
//! ペアごとに構成するため総運動量の変化は構成上ゼロになる)。一様外部場由来の項
//! (`UniformField`)は「外部」由来のため反作用対象を持たない
//! (`PointChargeSystem::step()`自身が一様外場をそう扱っているのと同じ規約)。

use crate::domain_states::{Coupling, DomainStates};
use sim_core::DomainId;
use sim_em::COULOMB_CONSTANT;
use sim_math::Vec3;

/// 対象剛体(`body_index`)に電荷`charge`を持たせ、`em_electrostatics`の電場からの
/// ローレンツ力を注入する(モジュールdoc参照)。
#[derive(Clone)]
pub struct LorentzForce {
    pub body_index: usize,
    pub charge: f64,
}

impl Coupling for LorentzForce {
    fn domains(&self) -> (DomainId, DomainId) {
        (DomainId::Mechanics, DomainId::Electromagnetism)
    }

    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        let Some(em) = &mut world.em_electrostatics else {
            return;
        };
        let mass = world.mechanics.bodies.mass(self.body_index);
        if mass <= 0.0 {
            return; // 静的/キネマティック剛体には適用しない。
        }
        let body_pos = world.mechanics.bodies.position[self.body_index];
        let body_vel = world.mechanics.bodies.linear_velocity[self.body_index];

        // 点電荷群それぞれとの対ごとのクーロン力(反作用を各点電荷にも記帳、モジュールdoc参照)。
        let mut coulomb_force = Vec3::ZERO;
        for j in 0..em.len() {
            let d = body_pos - em.position[j];
            let dist_sq = d.length_sq();
            if dist_sq < 1e-30 {
                continue; // 同一位置(縮退)は無視。
            }
            let dist = dist_sq.sqrt();
            let factor = COULOMB_CONSTANT * self.charge * em.charge[j] / (dist_sq * dist);
            let force_on_body = d.scale(factor);
            coulomb_force = coulomb_force + force_on_body;
            // 反作用: 点電荷jには逆向きの力(対記帳、モジュールdoc参照)。
            em.velocity[j] = em.velocity[j] - force_on_body.scale(dt / em.mass[j]);
        }

        // 一様外部場由来のローレンツ力(反作用なし、モジュールdoc参照)。
        let uniform_force =
            (em.uniform_field.e + body_vel.cross(em.uniform_field.b)).scale(self.charge);

        let total_force = coulomb_force + uniform_force;
        world.mechanics.bodies.linear_velocity[self.body_index] =
            body_vel + total_force.scale(dt / mass);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::MaterialDb;
    use sim_em::{PointChargeSystem, UniformField};
    use sim_mechanics::{MechanicsSolver, RigidBodyDesc, Shape};

    /// 単一の固定点電荷(質量を極めて大きくして事実上静止させる)が作るクーロン場の中で、
    /// 対象剛体(電荷を持つ)が万有引力のない系のクーロン力のみで運動することを確認する
    /// (逆二乗則、設計docs/13-electromagnetism/01-electrostatics-magnetostatics.md §2の
    /// 点電荷解と同じ形)。同符号の電荷なので反発し、系の全運動量が保存する(対記帳の検証、
    /// モジュールdoc参照)ことも合わせて確認する。
    #[test]
    fn lorentz_force_conserves_total_momentum_between_body_and_source_charge() {
        let materials = MaterialDb::standard();
        let mut mechanics = MechanicsSolver::new(0.0); // 重力なし: クーロン力のみを見る
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();

        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.01 }, steel);
        desc.mass_override = Some(1.0);
        desc.transform.position = Vec3::new(1.0, 0.0, 0.0);
        let body_idx = mechanics.create_body(desc, &materials);

        let mut em = PointChargeSystem::new(UniformField::default());
        let source_idx = em.add_particle(Vec3::ZERO, Vec3::ZERO, 2.0, 1.0e-6);
        let mut coupling = LorentzForce {
            body_index: body_idx,
            charge: 1.0e-6,
        };

        let initial_momentum = mechanics.bodies.linear_velocity[body_idx].scale(1.0)
            + em.velocity[source_idx].scale(em.mass[source_idx]);
        assert_eq!(initial_momentum, Vec3::ZERO);

        let dt = 1.0e-4;
        for _ in 0..2000 {
            let mut states = DomainStates {
                mechanics: &mut mechanics,
                thermal: None,
                em_circuit: None,
                em_electrostatics: Some(&mut em),
                gas: None,
            };
            coupling.apply(&mut states, dt);
            // Couplingは速度のみを更新する(モジュールdoc参照)ため、位置積分は
            // テスト側で明示的に行う(mechanics.step()はここでは呼ばない — 重力・接触の
            // 影響を混ぜずCoupling自体の挙動だけを見るため)。
            mechanics.bodies.position[body_idx] = mechanics.bodies.position[body_idx]
                + mechanics.bodies.linear_velocity[body_idx].scale(dt);
        }

        // 剛体は電荷源から遠ざかる方向(+x)へ加速しているはず(同符号電荷の反発)。
        assert!(
            mechanics.bodies.linear_velocity[body_idx].x > 0.0,
            "like charges should repel: v={:?}",
            mechanics.bodies.linear_velocity[body_idx]
        );
        assert!(
            mechanics.bodies.position[body_idx].x > 1.0,
            "body should have moved away from the source charge"
        );

        // 対記帳の検証: 剛体+点電荷源の全運動量はゼロのまま(設計§1)。
        let final_momentum = mechanics.bodies.linear_velocity[body_idx].scale(1.0)
            + em.velocity[source_idx].scale(em.mass[source_idx]);
        assert!(
            final_momentum.length() < 1e-9,
            "total momentum should stay conserved: {final_momentum:?}"
        );
    }
}
