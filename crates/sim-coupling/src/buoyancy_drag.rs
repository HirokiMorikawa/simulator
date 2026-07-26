//! `BuoyancyDrag`(設計 docs/20-integration/01-coupling-matrix.md §3「P1: 静的媒質 → 剛体力
//! (浮力・抗力・揚力)」)。
//!
//! **既存の`MechanicsSolver`埋め込み実装との関係**: `sim_mechanics::MechanicsSolver`は
//! `water: Option<StaticWaterRegion>`・`atmosphere: Option<Atmosphere>`フィールドを持ち、
//! `apply_forces`内でシーン全体の該当形状(直立直方体の浮力・球の抗力)全てに無条件で
//! 同じ物理式(`sim_fluid::{buoyancy_force, drag_force_sphere, submerged_box_axis_aligned}`)
//! を適用する(シーン全体で単一の水域・大気を共有する既存の縮約、多数の既存テスト・
//! デモ(F4・F5・F1・F3・D6・D7等)がこの経路に依存する)。本Couplingはこれを置き換える
//! ものではない(切り出しは広く使われた既存経路への高リスクな変更になるため対象外、
//! 「既存のMechanicsSolver埋め込み実装の切り出しリスク」として繰り返し見送ってきた
//! 判断はそのまま維持する)。代わりに、同じ物理式を剛体単位でCouplingレジストリ経由
//! (シーンJSON`couplings`セクション等)から選択的に適用したい場合の、独立した追加の
//! 配線を提供する(`LorentzForce`・`BrownianForce`と同じ「剛体ごとの明示的な結合登録」
//! パターン、`MechanicsSolver`の内部状態を一切変更しない)。
//!
//! **二重計上に関する注意**: 同一剛体に対して`MechanicsSolver.water`/`.atmosphere`
//! (埋め込み経路)とこの`Coupling`の両方を有効にすると同じ物理を2回適用してしまう
//! (設計§2規則2)。シーン構築側がどちらか一方のみを選ぶ責任を持つ(設計のシーン設定
//! `SceneCouplingConfig`の`static_water_buoyancy`フラグが表す区別と同じ)。
//!
//! 浮力は直立姿勢の直方体のみ(`sim_fluid::buoyancy`冒頭注記と同じ縮約)、抗力は球のみ
//! (`sim_fluid::aero`冒頭注記と同じ縮約)を対象とする。設計名`BuoyancyDrag`が挙げる
//! 「揚力」は`sim_fluid`側にも式自体が未実装のため対象外(設計から乖離している範囲を
//! 正直に文書化する、他の縮約実装と同じ方針)。

use crate::domain_states::{Coupling, DomainStates};
use sim_core::DomainId;
use sim_fluid::{Atmosphere, StaticWaterRegion};
use sim_math::Vec3;
use sim_mechanics::{DragModel, Shape};

/// 剛体(`body_index`)を静的水域・大気による浮力・抗力に結合する(モジュールdoc参照)。
#[derive(Clone)]
pub struct BuoyancyDrag {
    pub body_index: usize,
    pub water: Option<StaticWaterRegion>,
    pub atmosphere: Option<Atmosphere>,
}

impl Coupling for BuoyancyDrag {
    fn domains(&self) -> (DomainId, DomainId) {
        (DomainId::Mechanics, DomainId::Fluid)
    }

    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        let idx = self.body_index;
        let mass = world.mechanics.bodies.mass(idx);
        if mass <= 0.0 {
            return; // 静的/キネマティック剛体には適用しない。
        }

        let mut force = Vec3::ZERO;
        if let Some(water) = &self.water {
            if let Shape::Box { half_extents } = *world.mechanics.bodies.shape_of(idx) {
                let pos = world.mechanics.bodies.position[idx];
                let (v_sub, _) =
                    sim_fluid::submerged_box_axis_aligned(pos, half_extents, water.water_level);
                if v_sub > 0.0 {
                    force = force
                        + sim_fluid::buoyancy_force(v_sub, water.density, world.mechanics.gravity);
                }
            }
        }
        if let Some(atm) = &self.atmosphere {
            if let DragModel::Sphere { radius } = world.mechanics.bodies.drag[idx] {
                force = force
                    + sim_fluid::drag_force_sphere(
                        radius,
                        atm,
                        world.mechanics.bodies.linear_velocity[idx],
                    );
            }
        }

        world.mechanics.bodies.linear_velocity[idx] =
            world.mechanics.bodies.linear_velocity[idx] + force.scale(dt / mass);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::MaterialDb;
    use sim_mechanics::{MechanicsSolver, RigidBodyDesc};

    fn states<'a>(mechanics: &'a mut MechanicsSolver) -> DomainStates<'a> {
        DomainStates {
            mechanics,
            thermal: None,
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: None,
            sph: None,
        }
    }

    /// 半分水没した直立直方体に、`sim_fluid::buoyancy_force`の解析式どおりの浮力が
    /// 注入されること(`sim_fluid::buoyancy`側で既に検証済みの物理式そのものをここでは
    /// 直接呼び出して期待値を作るため、動的な定量検証特有の縁効果は生じない)。
    #[test]
    fn buoyancy_drag_applies_known_buoyancy_force_for_a_submerged_box() {
        let materials = MaterialDb::standard();
        let wood = materials.find_by_name("木材(松)").unwrap();
        let half_extents = Vec3::new(0.5, 0.5, 0.5);
        let mut desc = RigidBodyDesc::dynamic(Shape::Box { half_extents }, wood);
        let mass = 10.0;
        desc.mass_override = Some(mass);
        desc.transform.position = Vec3::new(0.0, 0.25, 0.0); // 半分(0.25m)水没。
        let gravity = 9.80665;
        let mut mechanics = MechanicsSolver::new(gravity);
        let body = mechanics.create_body(desc, &materials);

        let water = StaticWaterRegion::new(0.5, 1000.0);
        let (v_sub, _) = sim_fluid::submerged_box_axis_aligned(
            Vec3::new(0.0, 0.25, 0.0),
            half_extents,
            water.water_level,
        );
        assert!(v_sub > 0.0, "test setup should submerge the box partially");
        let expected_force = sim_fluid::buoyancy_force(v_sub, water.density, gravity);

        let mut coupling = BuoyancyDrag {
            body_index: body,
            water: Some(water),
            atmosphere: None,
        };
        let dt = 0.01;
        let velocity_before = mechanics.bodies.linear_velocity[body];
        coupling.apply(&mut states(&mut mechanics), dt);

        let expected_velocity = velocity_before + expected_force.scale(dt / mass);
        let measured_velocity = mechanics.bodies.linear_velocity[body];
        assert!(
            (measured_velocity - expected_velocity).length() < 1e-12,
            "measured_velocity={measured_velocity:?} expected_velocity={expected_velocity:?}"
        );
    }

    /// 移動中の球に、`sim_fluid::drag_force_sphere`の解析式どおりの抗力が注入される
    /// こと。
    #[test]
    fn buoyancy_drag_applies_known_drag_force_for_a_moving_sphere() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let radius = 0.05;
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, steel);
        let mass = 1.0;
        desc.mass_override = Some(mass);
        desc.drag = DragModel::Sphere { radius };
        desc.linear_velocity = Vec3::new(3.0, 0.0, 0.0);
        let mut mechanics = MechanicsSolver::new(9.80665);
        let body = mechanics.create_body(desc, &materials);

        let atmosphere = Atmosphere::still(1.225, 1.81e-5);
        let expected_force =
            sim_fluid::drag_force_sphere(radius, &atmosphere, Vec3::new(3.0, 0.0, 0.0));

        let mut coupling = BuoyancyDrag {
            body_index: body,
            water: None,
            atmosphere: Some(atmosphere),
        };
        let dt = 0.001;
        let velocity_before = mechanics.bodies.linear_velocity[body];
        coupling.apply(&mut states(&mut mechanics), dt);

        let expected_velocity = velocity_before + expected_force.scale(dt / mass);
        let measured_velocity = mechanics.bodies.linear_velocity[body];
        assert!(
            (measured_velocity - expected_velocity).length() < 1e-12,
            "measured_velocity={measured_velocity:?} expected_velocity={expected_velocity:?}"
        );
    }

    /// 水面より上にある(水没していない)直方体には浮力を適用しない。
    #[test]
    fn buoyancy_drag_does_nothing_for_a_box_above_the_waterline() {
        let materials = MaterialDb::standard();
        let wood = materials.find_by_name("木材(松)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(0.5, 0.5, 0.5),
            },
            wood,
        );
        desc.transform.position = Vec3::new(0.0, 10.0, 0.0);
        let mut mechanics = MechanicsSolver::new(9.80665);
        let body = mechanics.create_body(desc, &materials);

        let mut coupling = BuoyancyDrag {
            body_index: body,
            water: Some(StaticWaterRegion::new(0.0, 1000.0)),
            atmosphere: None,
        };
        let velocity_before = mechanics.bodies.linear_velocity[body];
        coupling.apply(&mut states(&mut mechanics), 0.01);

        assert_eq!(mechanics.bodies.linear_velocity[body], velocity_before);
    }

    /// 質量0以下(静的/キネマティック)の剛体には適用しない(他のCoupling実装と同じ
    /// ガード、`SphRigid`・`GridFluidRigid`等参照)。
    #[test]
    fn buoyancy_drag_does_nothing_for_a_static_body() {
        let materials = MaterialDb::standard();
        let wood = materials.find_by_name("木材(松)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(0.5, 0.5, 0.5),
            },
            wood,
        );
        desc.body_type = sim_mechanics::BodyType::Static;
        desc.transform.position = Vec3::new(0.0, 0.25, 0.0);
        let mut mechanics = MechanicsSolver::new(9.80665);
        let body = mechanics.create_body(desc, &materials);

        let mut coupling = BuoyancyDrag {
            body_index: body,
            water: Some(StaticWaterRegion::new(0.5, 1000.0)),
            atmosphere: None,
        };
        coupling.apply(&mut states(&mut mechanics), 0.01);

        assert_eq!(mechanics.bodies.linear_velocity[body], Vec3::ZERO);
    }
}
