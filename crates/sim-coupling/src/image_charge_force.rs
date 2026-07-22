//! `ImageChargeForce`(設計 docs/13-electromagnetism/01-electrostatics-magnetostatics.md
//! §2「導体の誘導電荷・誘電体の分極は鏡像力の近似式($F=-q^2/(16\pi\varepsilon_0 d^2)$)
//! のみ提供(風船が壁に貼りつくデモ用)」、D26「帯電風船」の核となる物理)。
//!
//! **縮約実装の理由**: 設計が明記するとおり、一般形状の誘導は境界要素法が必要
//! (Phase 5+、当面非対応)。本実装は設計が明示的に許容する特殊形(平板近傍の点電荷)
//! のみを対象とし、対象剛体を点電荷近似(重心)として扱う(`LorentzForce`と同じ
//! 「`ChargedBody`という正式な結合型はまだ存在しないため、剛体indexと電荷量を
//! `Coupling`自身のフィールドとして持つ」縮約パターン)。
//!
//! 鏡像力は符号によらず常に平板側への引力(電荷の符号によらず$q^2>0$)。壁(平板)は
//! 接地導体という理想化(電荷を持たない・動かない)で、`LorentzForce`の一様外部場と
//! 同じく反作用対象を持たない「外部」由来の力として扱う(平板自身の電荷分布の変化は
//! モデル化しない)。

use crate::domain_states::{Coupling, DomainStates};
use sim_core::DomainId;
use sim_em::COULOMB_CONSTANT;
use sim_math::Vec3;

/// 対象剛体(`body_index`、電荷`charge`)と平板(法線`plane_normal`(単位ベクトル、
/// 剛体側を向く)・平面上の1点を`plane_normal`方向へ射影した符号付き距離`plane_d`、
/// つまり平面は$\mathbf p\cdot\mathbf n=d$)との間の鏡像力を注入する(モジュールdoc参照)。
#[derive(Clone)]
pub struct ImageChargeForce {
    pub body_index: usize,
    pub charge: f64,
    pub plane_normal: Vec3,
    pub plane_d: f64,
}

impl Coupling for ImageChargeForce {
    fn domains(&self) -> (DomainId, DomainId) {
        (DomainId::Mechanics, DomainId::Electromagnetism)
    }

    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        let mass = world.mechanics.bodies.mass(self.body_index);
        if mass <= 0.0 {
            return; // 静的/キネマティック剛体には適用しない。
        }
        let pos = world.mechanics.bodies.position[self.body_index];
        let distance = pos.dot(self.plane_normal) - self.plane_d;
        if distance <= 0.0 {
            return; // 平板に到達済み/背面(設計の近似式は平板前方でのみ意味を持つ)。
        }

        // F = -q^2/(16*pi*epsilon_0*d^2) = -COULOMB_CONSTANT*q^2/(4*d^2)(モジュールdoc参照)。
        let magnitude = COULOMB_CONSTANT * self.charge * self.charge / (4.0 * distance * distance);
        let force = self.plane_normal.scale(-magnitude); // 常に平板側への引力

        let velocity = world.mechanics.bodies.linear_velocity[self.body_index];
        world.mechanics.bodies.linear_velocity[self.body_index] = velocity + force.scale(dt / mass);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::MaterialDb;
    use sim_mechanics::{MechanicsSolver, RigidBodyDesc, Shape};

    /// 鏡像力の解析式 $F=q^2/(16\pi\varepsilon_0 d^2)$(引力)どおりの加速度を注入すること。
    #[test]
    fn image_charge_force_matches_analytic_formula_and_is_attractive_toward_the_plane() {
        let materials = MaterialDb::standard();
        let mut mechanics = MechanicsSolver::new(0.0); // 重力なし: 鏡像力のみを見る
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();

        let distance = 0.05; // 5cm
        let charge = 2.0e-8; // クーロン(帯電風船オーダー)

        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.01 }, steel);
        desc.transform.position = Vec3::new(distance, 0.0, 0.0);
        let body_idx = mechanics.create_body(desc, &materials);

        let mut coupling = ImageChargeForce {
            body_index: body_idx,
            charge,
            plane_normal: Vec3::new(1.0, 0.0, 0.0),
            plane_d: 0.0,
        };

        let mass = mechanics.bodies.mass(body_idx);
        let dt = 0.001;
        let mut states = DomainStates {
            mechanics: &mut mechanics,
            thermal: None,
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: None,
        };
        coupling.apply(&mut states, dt);

        let expected_magnitude =
            sim_em::COULOMB_CONSTANT * charge * charge / (4.0 * distance * distance);
        let expected_vx = -expected_magnitude / mass * dt; // 引力: 平板(x=0)方向、つまり-x
        let measured_vx = mechanics.bodies.linear_velocity[body_idx].x;
        let rel_err = (measured_vx - expected_vx).abs() / expected_vx.abs();
        assert!(
            rel_err < 1e-9,
            "measured_vx={measured_vx} expected_vx={expected_vx} rel_err={rel_err:e}"
        );
        assert!(
            measured_vx < 0.0,
            "image charge force should always be attractive toward the plane regardless of \
             the charge's sign: measured_vx={measured_vx}"
        );
    }

    /// 電荷の符号によらず(q^2は常に正)引力になることを、負電荷でも確認する。
    #[test]
    fn image_charge_force_is_attractive_for_negative_charge_too() {
        let materials = MaterialDb::standard();
        let mut mechanics = MechanicsSolver::new(0.0);
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();

        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.01 }, steel);
        desc.transform.position = Vec3::new(0.1, 0.0, 0.0);
        let body_idx = mechanics.create_body(desc, &materials);

        let mut coupling = ImageChargeForce {
            body_index: body_idx,
            charge: -2.0e-8,
            plane_normal: Vec3::new(1.0, 0.0, 0.0),
            plane_d: 0.0,
        };
        let mut states = DomainStates {
            mechanics: &mut mechanics,
            thermal: None,
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: None,
        };
        coupling.apply(&mut states, 0.001);

        assert!(mechanics.bodies.linear_velocity[body_idx].x < 0.0);
    }
}
