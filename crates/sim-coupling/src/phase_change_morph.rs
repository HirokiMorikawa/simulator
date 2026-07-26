//! `PhaseChangeMorph`(設計 docs/20-integration/01-coupling-matrix.md §3「P3: 融解 →
//! 剛体消滅/流体生成イベント」)。
//!
//! **縮約実装の理由**: 設計は融解時に剛体を消滅させ、融けた分を流体粒子として生成する
//! 双方向のイベントを想定するが、本実装は前者(剛体消滅)のみを扱う。「流体生成」
//! (融けた分をSPH粒子や格子流体セルとして新規に注入する)は、`Coupling::apply`が
//! イベントキュー(`sim_core::EventQueue`)にも`World`の世代管理(`generations`)にも
//! `DomainStates`経由でアクセスできない(いずれも後続増分でのアーキテクチャ拡張を
//! 要する)ため対象外とする(設計から乖離している範囲を正直に文書化する、他の縮約
//! 実装と同じ方針)。
//!
//! 熱源は単一の`ThermalNode`(`thermal_node`、氷を取り巻く飲み物・空気等に相当)とし、
//! 簡易な線形熱コンダクタンス(`conductance`、`DissipationToHeat`と同じ「呼び出し側が
//! 値を直接渡す」縮約)で氷側のエンタルピー(`sim_thermal::PhaseState`、エンタルピー法)
//! へ熱を流し込む。熱源ノード側からは同量を差し引く(設計§2規則1「取り出しと注入を
//! 同一実装内で対記帳」)。
//!
//! 剛体の質量は`initial_mass*(1-liquid_fraction)`(固相残存質量比)として
//! `RigidBodySet::inv_mass`を直接更新する(形状(`Shape`)自体は縮小しない——密度が
//! 見かけ上下がっていく近似、`Shape`のランタイム変形は`RigidBodySet`に未実装のため
//! 対象外)。この質量変化は既存の`BuoyancyDrag`・埋め込み浮力(`MechanicsSolver.water`)
//! 双方に無変更で伝播する(いずれも毎step`bodies.mass(idx)`を読み直すため、本
//! Couplingが質量を更新するだけで浮力側が自動的に追従する——D18「氷と飲み物」の
//! 「アルキメデス統合」が求める浮力との連動は、新規コードなしでこの構成上の性質から
//! 得られる)。
//!
//! 完全融解(`Phase::Liquid`)に達したら、`World::remove_body`と同じ無効化手順
//! (`body_type`をStaticへ・遠方へ退避・速度ゼロ化)を直接行う。`World`の世代カウンタ
//! (`generations`)は`DomainStates`から触れないため据え置かれる——呼び出し側が同じ
//! `BodyId`を将来再利用したい場合は別途`World::remove_body`を呼ぶ必要がある(正直に
//! 文書化する制約、`SphRigid`・`GridFluidRigid`と同様に既存の`World`公開APIを拡張
//! せずに実装できる範囲に留めた)。

use crate::domain_states::{Coupling, DomainStates};
use sim_core::DomainId;
use sim_math::Vec3;
use sim_mechanics::BodyType;
use sim_thermal::{Phase, PhaseMaterial, PhaseState};

/// 剛体(`body_index`)を、単一の熱源`ThermalNode`(`thermal_node`)からの熱で融解させる
/// (モジュールdoc参照)。
#[derive(Clone)]
pub struct PhaseChangeMorph {
    pub body_index: usize,
    pub thermal_node: usize,
    pub material: PhaseMaterial,
    pub initial_mass: f64,
    /// 熱源ノードとの線形熱コンダクタンス [W/K]。
    pub conductance: f64,
    state: PhaseState,
    despawned: bool,
}

impl PhaseChangeMorph {
    /// `initial_enthalpy`は`PhaseState::enthalpy`の初期値(負なら融点未満の固相、
    /// `sim_thermal::phase`モジュールdoc参照)。
    pub fn new(
        body_index: usize,
        thermal_node: usize,
        material: PhaseMaterial,
        initial_mass: f64,
        conductance: f64,
        initial_enthalpy: f64,
    ) -> PhaseChangeMorph {
        PhaseChangeMorph {
            body_index,
            thermal_node,
            material,
            initial_mass,
            conductance,
            state: PhaseState {
                enthalpy: initial_enthalpy,
                mass: initial_mass,
            },
            despawned: false,
        }
    }

    /// 現在の相(テスト・診断用)。
    pub fn phase(&self) -> Phase {
        self.state.phase(&self.material)
    }
}

impl Coupling for PhaseChangeMorph {
    fn domains(&self) -> (DomainId, DomainId) {
        (DomainId::Thermal, DomainId::Mechanics)
    }

    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        if self.despawned {
            return; // 既に完全融解済み(モジュールdoc参照)。
        }
        let Some(thermal) = &mut world.thermal else {
            return;
        };
        let Some(node) = thermal.nodes.get_mut(self.thermal_node) else {
            return;
        };

        // Q = 熱コンダクタンス×温度差×dt。熱源から取り出した分をそのまま氷側の
        // エンタルピーへ注入する対記帳(設計§2規則1)。
        let body_temperature = self.state.temperature(&self.material);
        let q = self.conductance * (node.temperature - body_temperature) * dt;
        node.temperature -= q / node.heat_capacity;
        self.state.add_heat(q);

        let liquid_fraction = match self.state.phase(&self.material) {
            Phase::Solid => 0.0,
            Phase::Mixed { liquid_fraction } => liquid_fraction,
            Phase::Liquid => 1.0,
        };
        let remaining_mass = self.initial_mass * (1.0 - liquid_fraction);
        let idx = self.body_index;
        if remaining_mass > 1e-9 {
            world.mechanics.bodies.inv_mass[idx] = 1.0 / remaining_mass;
            // 質量が変化した剛体はスリープ(設計 docs/10-mechanics/01-rigid-body.md、
            // `sim_mechanics::sleep`、静止0.5秒継続で自動停止)から起こす——スリープ中は
            // 力の適用・速度/位置積分が完全に止まる(`MechanicsSolver::apply_forces`/
            // `integrate_velocities`/`integrate_positions`)ため、外部要因(質量減少)で
            // 力の釣り合いが崩れたにもかかわらず永久に静止したままになってしまう
            // (実装検証中に発見: 浮いた氷が釣り合い位置で静止したまま融解が進んでも
            // 位置が全く変化しないバグとして顕在化した)。
            world.mechanics.bodies.asleep[idx] = false;
        } else {
            // 完全融解: `World::remove_body`と同じ無効化手順(モジュールdoc参照)。
            world.mechanics.bodies.body_type[idx] = BodyType::Static;
            world.mechanics.bodies.position[idx] = Vec3::new(0.0, -1.0e9, 0.0);
            world.mechanics.bodies.linear_velocity[idx] = Vec3::ZERO;
            world.mechanics.bodies.angular_velocity[idx] = Vec3::ZERO;
            self.despawned = true;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::MaterialDb;
    use sim_mechanics::{MechanicsSolver, RigidBodyDesc, Shape};
    use sim_thermal::{ThermalNode, ThermalSolver};

    fn ice_material() -> PhaseMaterial {
        PhaseMaterial {
            melting_temperature: 273.15,
            latent_heat_fusion: 334_000.0,
            specific_heat_solid: 2100.0,
            specific_heat_liquid: 4186.0,
        }
    }

    fn states<'a>(
        mechanics: &'a mut MechanicsSolver,
        thermal: &'a mut ThermalSolver,
    ) -> DomainStates<'a> {
        DomainStates {
            mechanics,
            thermal: Some(thermal),
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: None,
            sph: None,
        }
    }

    /// 融解プラトー期間中、剛体質量が液相率どおりに`initial_mass*(1-liquid_fraction)`
    /// で減少すること、かつ熱源ノードから取り出した熱量と氷側が受け取った熱量が
    /// 厳密に対記帳されること(設計§2規則1)を確認する。T7の融解プラトー物理自体
    /// (`sim_thermal::phase`の`PhaseState`)は既に検証済みのため、ここでは配線
    /// (質量の追従・対記帳)のみを検証する。
    #[test]
    fn phase_change_morph_shrinks_mass_matching_liquid_fraction_and_conserves_heat() {
        let materials = MaterialDb::standard();
        let ice_id = materials.find_by_name("氷(0°C)").unwrap();
        let initial_mass = 0.1;
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.03 }, ice_id);
        desc.mass_override = Some(initial_mass);
        let mut mechanics = MechanicsSolver::new(0.0);
        let body = mechanics.create_body(desc, &materials);

        let mut thermal = ThermalSolver::new(293.15);
        let warm_node = thermal.add_node(ThermalNode::new(293.15, 5000.0));

        let mat = ice_material();
        let mut coupling = PhaseChangeMorph::new(
            body,
            warm_node,
            mat,
            initial_mass,
            20.0,
            -mat.specific_heat_solid * 10.0,
        );

        let dt = 0.05;
        let initial_node_heat =
            thermal.nodes[warm_node].temperature * thermal.nodes[warm_node].heat_capacity;
        for _ in 0..4000 {
            if matches!(coupling.phase(), Phase::Mixed { .. }) {
                break;
            }
            coupling.apply(&mut states(&mut mechanics, &mut thermal), dt);
        }
        assert!(
            matches!(coupling.phase(), Phase::Mixed { .. }),
            "should have reached the melting plateau: phase={:?}",
            coupling.phase()
        );

        let liquid_fraction = match coupling.phase() {
            Phase::Mixed { liquid_fraction } => liquid_fraction,
            other => panic!("expected Mixed, got {other:?}"),
        };
        let expected_mass = initial_mass * (1.0 - liquid_fraction);
        let measured_mass = mechanics.bodies.mass(body);
        assert!(
            (measured_mass - expected_mass).abs() < 1e-9,
            "measured_mass={measured_mass} expected_mass={expected_mass}"
        );

        // 対記帳: 熱源ノードが失った熱量 == 氷が(顕熱+潜熱として)受け取った熱量。
        let final_node_heat =
            thermal.nodes[warm_node].temperature * thermal.nodes[warm_node].heat_capacity;
        let heat_lost_by_node = initial_node_heat - final_node_heat;
        let heat_gained_by_ice =
            coupling.state.mass * (coupling.state.enthalpy - (-mat.specific_heat_solid * 10.0));
        let rel_err = (heat_lost_by_node - heat_gained_by_ice).abs() / heat_lost_by_node;
        assert!(
            rel_err < 1e-9,
            "heat_lost_by_node={heat_lost_by_node} heat_gained_by_ice={heat_gained_by_ice} rel_err={rel_err:e}"
        );
    }

    /// 完全融解(`Phase::Liquid`)に達すると、`World::remove_body`と同じ無効化
    /// (Static化・遠方退避・速度ゼロ化)が行われ、以後`apply`を繰り返し呼んでも
    /// 状態が変化しない(冪等)ことを確認する。
    #[test]
    fn phase_change_morph_despawns_the_body_once_fully_melted() {
        let materials = MaterialDb::standard();
        let ice_id = materials.find_by_name("氷(0°C)").unwrap();
        let initial_mass = 0.01; // 小質量・短時間で完全融解に到達させる。
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.01 }, ice_id);
        desc.mass_override = Some(initial_mass);
        desc.transform.position = Vec3::new(1.0, 2.0, 3.0);
        let mut mechanics = MechanicsSolver::new(0.0);
        let body = mechanics.create_body(desc, &materials);

        let mut thermal = ThermalSolver::new(353.15); // 熱い湯相当、短時間で完全融解させる。
        let warm_node = thermal.add_node(ThermalNode::new(353.15, 50_000.0));

        let mat = ice_material();
        let mut coupling = PhaseChangeMorph::new(body, warm_node, mat, initial_mass, 50.0, 0.0);

        let dt = 0.1;
        for _ in 0..20_000 {
            coupling.apply(&mut states(&mut mechanics, &mut thermal), dt);
            if matches!(coupling.phase(), Phase::Liquid) {
                break;
            }
        }
        assert!(
            matches!(coupling.phase(), Phase::Liquid),
            "should have fully melted: phase={:?}",
            coupling.phase()
        );
        assert_eq!(mechanics.bodies.body_type[body], BodyType::Static);
        assert_eq!(mechanics.bodies.position[body], Vec3::new(0.0, -1.0e9, 0.0));
        assert_eq!(mechanics.bodies.linear_velocity[body], Vec3::ZERO);

        // 冪等性: 完全融解後に追加でapplyしても状態は変化しない。
        let position_after_despawn = mechanics.bodies.position[body];
        coupling.apply(&mut states(&mut mechanics, &mut thermal), dt);
        assert_eq!(mechanics.bodies.position[body], position_after_despawn);
    }
}
