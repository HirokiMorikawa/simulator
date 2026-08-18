//! ドメイン間結合行列・排他結合の静的検査。設計: docs/20-integration/01-coupling-matrix.md。
//!
//! シーン設定における排他結合(同じ物理を2経路で計算しない、設計§2規則2)の静的検査
//! (`validate_exclusive_couplings`)に加え、`Coupling`トレイト + `DomainStates`
//! (設計docs/00-foundation/04-architecture.md §1.3「保存量の橋」、`domain_states`
//! モジュールdoc参照)と、具体的な実装13種(`DissipationToHeat`・`BuoyancyDrag`・
//! `JouleHeat`・`BrownianForce`・`LorentzForce`・`InductionCoupling`・`MotorCoupling`・
//! `PistonGas`・`BoussinesqBuoyancy`・`ConvectionLink`・`SphRigid`・`GridFluidRigid`・
//! `ImageChargeForce`、各モジュールdoc参照)を実装する。これで設計§3が挙げる元の12種の
//! Couplingが全て出揃った(`BuoyancyDrag`は既存の`MechanicsSolver`埋め込み実装を
//! 置き換えるものではなく、同じ物理式を剛体単位でCoupling登録経由から選択的に適用する
//! 独立した追加経路として実装した、`buoyancy_drag`モジュールdoc参照)+ D26「帯電風船」
//! 向けに設計docs/13-electromagnetism/01-electrostatics-magnetostatics.md §2が別途
//! 要求する「鏡像力」である追加実装の`ImageChargeForce`。sub-iteration剛性閾値表
//! (設計§2規則3、`GridFluidRigid`自身は現状固定的な単一適用で、`sim_fluid::
//! GridFluidRigidBox2D`(X2)が持つ閾値ベースのsub-iteration機構までは踏襲していない)・
//! シーンJSON`couplings`セクションからの自動解決は後続増分で追加する。
//!
//! **pre/post 2相分離は群5で完了した**(`Coupling::apply_pre`のdoc参照)。同時に
//! `MotorCoupling`・`InductionCoupling`の1step遅れを解消し、`SphRigid`・
//! `GridFluidRigid`のモジュールdocが誤って主張していた「反作用力の1step遅れ」
//! (実際には`World`経由なら遅れていない)を訂正し、`ConvectionLink`に設計 §4.2 の
//! 相関式4件を揃えた(自然対流を含む)。

mod boussinesq_buoyancy;
mod brownian_force;
mod buoyancy_drag;
mod convection_link;
mod dissipation_to_heat;
mod domain_states;
mod grid_fluid_rigid;
mod image_charge_force;
mod induction_coupling;
mod joule_heat;
mod lorentz_force;
mod motor_coupling;
mod phase_change_morph;
mod piston_gas;
mod sph_rigid;
pub use boussinesq_buoyancy::BoussinesqBuoyancy;
pub use brownian_force::BrownianForce;
pub use buoyancy_drag::{BuoyancyDrag, LiftModel};
pub use convection_link::{ConvectionLink, ConvectionMode};
pub use dissipation_to_heat::{effusivity, BodyThermalLink, DissipationToHeat};
pub use domain_states::{
    Coupling, CouplingKind, CouplingParam, CouplingRawState, DomainStates, NoopCoupling,
};
pub use grid_fluid_rigid::GridFluidRigid;
pub use image_charge_force::ImageChargeForce;
pub use induction_coupling::InductionCoupling;
pub use joule_heat::JouleHeat;
pub use lorentz_force::LorentzForce;
pub use motor_coupling::MotorCoupling;
pub use phase_change_morph::{MeltSpawn, PhaseChangeMorph};
pub use piston_gas::PistonGas;
pub use sph_rigid::SphRigid;

/// シーンの結合設定(設計§2規則2が列挙する3組の排他結合、設定は各ドメインシーンJSON相当)。
#[derive(Clone, Copy, Debug, Default)]
pub struct SceneCouplingConfig {
    /// 浮力: 静的水域モデル(集中定数)。
    pub static_water_buoyancy: bool,
    /// 浮力: SPH/格子流体(解像)。
    pub resolved_fluid_buoyancy: bool,
    /// 空気抗力: 集中定数モデル。
    pub lumped_air_drag: bool,
    /// 空気抗力: 格子流体結合。
    pub grid_coupled_air_drag: bool,
    /// コンデンサ電場エネルギー: 回路(MNA)モデル。
    pub circuit_capacitor_field_energy: bool,
    /// コンデンサ電場エネルギー: 静電場モデル。
    pub electrostatic_field_energy: bool,
}

/// 排他結合違反(設計§2規則2「同じ物理を2経路で計算しない」)。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExclusiveCouplingViolation {
    /// 浮力が静的水域とSPH/格子流体の両方で二重計上される。
    BuoyancyDoubleCounted,
    /// 空気抗力が集中定数と格子流体結合の両方で二重計上される。
    AirDragDoubleCounted,
    /// コンデンサ電場エネルギーが回路と静電場の両方で二重計上される。
    CapacitorFieldEnergyDoubleCounted,
}

/// シーン設定を検査し、排他結合違反(二重計上の組み合わせ)を全て報告する
/// (設計§5「シーンロード時に二重計上の組み合わせを拒否するvalidator」)。
/// 空なら合法な設定。
pub fn validate_exclusive_couplings(
    config: &SceneCouplingConfig,
) -> Vec<ExclusiveCouplingViolation> {
    let mut violations = Vec::new();
    if config.static_water_buoyancy && config.resolved_fluid_buoyancy {
        violations.push(ExclusiveCouplingViolation::BuoyancyDoubleCounted);
    }
    if config.lumped_air_drag && config.grid_coupled_air_drag {
        violations.push(ExclusiveCouplingViolation::AirDragDoubleCounted);
    }
    if config.circuit_capacitor_field_energy && config.electrostatic_field_energy {
        violations.push(ExclusiveCouplingViolation::CapacitorFieldEnergyDoubleCounted);
    }
    violations
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_has_no_violations() {
        let config = SceneCouplingConfig::default();
        assert_eq!(validate_exclusive_couplings(&config), Vec::new());
    }

    #[test]
    fn exactly_one_path_per_pair_is_valid() {
        let config = SceneCouplingConfig {
            static_water_buoyancy: true,
            resolved_fluid_buoyancy: false,
            lumped_air_drag: false,
            grid_coupled_air_drag: true,
            circuit_capacitor_field_energy: true,
            electrostatic_field_energy: false,
        };
        assert_eq!(validate_exclusive_couplings(&config), Vec::new());
    }

    #[test]
    fn both_buoyancy_paths_enabled_is_rejected() {
        let config = SceneCouplingConfig {
            static_water_buoyancy: true,
            resolved_fluid_buoyancy: true,
            ..Default::default()
        };
        assert_eq!(
            validate_exclusive_couplings(&config),
            vec![ExclusiveCouplingViolation::BuoyancyDoubleCounted]
        );
    }

    #[test]
    fn both_air_drag_paths_enabled_is_rejected() {
        let config = SceneCouplingConfig {
            lumped_air_drag: true,
            grid_coupled_air_drag: true,
            ..Default::default()
        };
        assert_eq!(
            validate_exclusive_couplings(&config),
            vec![ExclusiveCouplingViolation::AirDragDoubleCounted]
        );
    }

    #[test]
    fn both_capacitor_field_energy_paths_enabled_is_rejected() {
        let config = SceneCouplingConfig {
            circuit_capacitor_field_energy: true,
            electrostatic_field_energy: true,
            ..Default::default()
        };
        assert_eq!(
            validate_exclusive_couplings(&config),
            vec![ExclusiveCouplingViolation::CapacitorFieldEnergyDoubleCounted]
        );
    }

    #[test]
    fn all_three_violations_are_reported_simultaneously() {
        let config = SceneCouplingConfig {
            static_water_buoyancy: true,
            resolved_fluid_buoyancy: true,
            lumped_air_drag: true,
            grid_coupled_air_drag: true,
            circuit_capacitor_field_energy: true,
            electrostatic_field_energy: true,
        };
        let violations = validate_exclusive_couplings(&config);
        assert_eq!(violations.len(), 3);
        assert!(violations.contains(&ExclusiveCouplingViolation::BuoyancyDoubleCounted));
        assert!(violations.contains(&ExclusiveCouplingViolation::AirDragDoubleCounted));
        assert!(violations.contains(&ExclusiveCouplingViolation::CapacitorFieldEnergyDoubleCounted));
    }
}
