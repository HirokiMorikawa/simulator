//! `ConvectionLink`(設計 docs/20-integration/01-coupling-matrix.md §3
//! 「P3: 流体/媒質 ⇔ ThermalNode(相関式 h)」、docs/12-thermal/02-heat-transfer.md
//! §2.2「対流(ニュートンの冷却則)」$\dot Q=hA(T_{fluid}-T_{surf})$、
//! §4.2の強制対流(平板)相関式 $\overline{Nu}=0.664\,Re^{1/2}Pr^{1/3}$(Blasius解、
//! 層流)。
//!
//! **縮約実装の理由**: 設計の相関式表は状況(自然対流/強制対流 × 形状)ごとに複数の
//! 式を挙げるが、本実装は流体ドメイン(`GridFluid2D`)の流速に依存する結合という
//! 設計表の趣旨(「h と流速」)を素直に反映できる「強制対流・平板」の式のみを採用する
//! (自然対流は温度差自体が駆動源であり流速に依存しないため、既存の`BoussinesqBuoyancy`
//! が担う範疇と役割分担する)。特性速度は`GridFluid2D`の速度場全体のRMS速度
//! (staggered配置の`u`・`v`を同一インデックスで単純にペアリングする近似、cell分の
//! ずれは特性速度の見積もりとしては無視できる)で代表させる。プラントル数`Pr`・
//! 熱伝導率`k_f`・動粘性係数`nu`は物性定数として`ConvectionLink`自身が保持する
//! (`sim_thermal`にまだ流体物性DBが無いため、`PistonGas`の`area`等と同じ「呼び出し側が
//! 材料値を直接渡す」縮約)。
//!
//! 熱源側・受熱側をともに単一の`ThermalNode`(`fluid_node`・`surface_node`)として
//! `ThermalSolver`内の2ノード間の熱交換で表す(セルごとの温度場を持たない`GridFluid2D`
//! の制約は`BoussinesqBuoyancy`と同じ)。取り出した熱量をそのまま反対側へ注入するため、
//! 2ノード間で厳密に対記帳される(丸め誤差を除き完全にゼロ和)。

use crate::domain_states::{Coupling, DomainStates};
use sim_core::DomainId;

/// 流体ノード`fluid_node`と受熱面ノード`surface_node`を、強制対流(平板、Blasius解)の
/// 熱伝達係数 $h=\overline{Nu}\,k_f/L$ で結ぶ(モジュールdoc参照)。特性速度は
/// `DomainStates::grid_fluid`の速度場全体のRMS速度から算出する。
#[derive(Clone)]
pub struct ConvectionLink {
    pub fluid_node: usize,
    pub surface_node: usize,
    /// 伝熱面積 [m^2]。
    pub area: f64,
    /// 特性長さ $L$ [m](Blasius解の平板長さに相当)。
    pub characteristic_length: f64,
    /// 流体の熱伝導率 $k_f$ [W/(m·K)]。
    pub fluid_thermal_conductivity: f64,
    /// 流体の動粘性係数 $\nu$ [m^2/s]。
    pub kinematic_viscosity: f64,
    /// 流体のプラントル数 $Pr$(無次元、物性値)。
    pub prandtl_number: f64,
}

impl ConvectionLink {
    /// `GridFluid2D`速度場全体のRMS速度(特性速度、モジュールdoc参照)。
    fn characteristic_speed(grid_fluid: &sim_fluid::GridFluid2D) -> f64 {
        let n = grid_fluid.u.len();
        if n == 0 {
            return 0.0;
        }
        let sum_sq: f64 = grid_fluid
            .u
            .iter()
            .zip(grid_fluid.v.iter())
            .map(|(&u, &v)| u * u + v * v)
            .sum();
        (sum_sq / n as f64).sqrt()
    }

    /// 強制対流(平板、Blasius解)の熱伝達係数 $h=\overline{Nu}\,k_f/L$、
    /// $\overline{Nu}=0.664\,Re^{1/2}Pr^{1/3}$、$Re=UL/\nu$(モジュールdoc参照)。
    fn heat_transfer_coefficient(&self, characteristic_speed: f64) -> f64 {
        if characteristic_speed <= 0.0 || self.kinematic_viscosity <= 0.0 {
            return 0.0;
        }
        let reynolds = characteristic_speed * self.characteristic_length / self.kinematic_viscosity;
        let nusselt = 0.664 * reynolds.sqrt() * self.prandtl_number.cbrt();
        nusselt * self.fluid_thermal_conductivity / self.characteristic_length
    }
}

impl Coupling for ConvectionLink {
    fn domains(&self) -> (DomainId, DomainId) {
        (DomainId::Fluid, DomainId::Thermal)
    }

    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        let Some(grid_fluid) = &world.grid_fluid else {
            return;
        };
        let speed = Self::characteristic_speed(grid_fluid);
        let h = self.heat_transfer_coefficient(speed);
        if h == 0.0 {
            return;
        }
        let Some(thermal) = &mut world.thermal else {
            return;
        };
        let Some(fluid) = thermal.nodes.get(self.fluid_node) else {
            return;
        };
        let Some(surface) = thermal.nodes.get(self.surface_node) else {
            return;
        };
        let (t_fluid, c_fluid) = (fluid.temperature, fluid.heat_capacity);
        let (t_surface, c_surface) = (surface.temperature, surface.heat_capacity);

        // Q = h*A*(T_fluid - T_surf)*dt(設計§2.2)。流体側から取り出した熱量を
        // そのまま受熱面側へ注入する対記帳(2ノード間で厳密にゼロ和)。
        let heat = h * self.area * (t_fluid - t_surface) * dt;
        thermal.nodes[self.fluid_node].temperature -= heat / c_fluid;
        thermal.nodes[self.surface_node].temperature += heat / c_surface;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_fluid::GridFluid2D;
    use sim_mechanics::MechanicsSolver;
    use sim_thermal::{ThermalNode, ThermalSolver};

    /// 流速ゼロなら(強制対流のみを扱う本実装の縮約により)熱伝達係数もゼロで、
    /// 両ノードとも温度不変。
    #[test]
    fn convection_link_transfers_no_heat_when_fluid_is_at_rest() {
        let mut thermal = ThermalSolver::new(293.15);
        let fluid_node = thermal.add_node(ThermalNode::new(350.0, 1000.0));
        let surface_node = thermal.add_node(ThermalNode::new(293.15, 1000.0));
        let mut mechanics = MechanicsSolver::new(9.80665);
        let mut fluid = GridFluid2D::new(4, 4, 0.1);

        let mut coupling = ConvectionLink {
            fluid_node,
            surface_node,
            area: 0.1,
            characteristic_length: 0.1,
            fluid_thermal_conductivity: 0.026,
            kinematic_viscosity: 1.5e-5,
            prandtl_number: 0.71,
        };
        let mut states = DomainStates {
            mechanics: &mut mechanics,
            thermal: Some(&mut thermal),
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: Some(&mut fluid),
            sph: None,
        };
        coupling.apply(&mut states, 0.01);

        assert_eq!(thermal.nodes[fluid_node].temperature, 350.0);
        assert_eq!(thermal.nodes[surface_node].temperature, 293.15);
    }

    /// 一様な流速がある場合、Blasius解の強制対流相関式どおりの熱伝達係数で、
    /// 流体ノード→受熱面ノードへ熱が移動し、対記帳(2ノード間でのエネルギー厳密保存)が
    /// 成立すること。
    #[test]
    fn convection_link_matches_blasius_forced_convection_formula_and_conserves_energy() {
        let t_fluid0 = 350.0;
        let t_surface0 = 293.15;
        let c_fluid = 1000.0;
        let c_surface = 2000.0;
        let mut thermal = ThermalSolver::new(293.15);
        let fluid_node = thermal.add_node(ThermalNode::new(t_fluid0, c_fluid));
        let surface_node = thermal.add_node(ThermalNode::new(t_surface0, c_surface));
        let mut mechanics = MechanicsSolver::new(9.80665);

        let mut fluid = GridFluid2D::new(4, 4, 0.1);
        let speed = 2.0;
        for u in fluid.u.iter_mut() {
            *u = speed;
        }

        let area = 0.05;
        let length = 0.2;
        let k_f = 0.026;
        let nu = 1.5e-5;
        let pr = 0.71;
        let mut coupling = ConvectionLink {
            fluid_node,
            surface_node,
            area,
            characteristic_length: length,
            fluid_thermal_conductivity: k_f,
            kinematic_viscosity: nu,
            prandtl_number: pr,
        };
        let dt = 0.01;
        let mut states = DomainStates {
            mechanics: &mut mechanics,
            thermal: Some(&mut thermal),
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: Some(&mut fluid),
            sph: None,
        };
        coupling.apply(&mut states, dt);

        let reynolds = speed * length / nu;
        let nusselt = 0.664 * reynolds.sqrt() * pr.cbrt();
        let h = nusselt * k_f / length;
        let expected_heat = h * area * (t_fluid0 - t_surface0) * dt;

        let fluid_temp = thermal.nodes[fluid_node].temperature;
        let surface_temp = thermal.nodes[surface_node].temperature;
        let expected_fluid_temp = t_fluid0 - expected_heat / c_fluid;
        let expected_surface_temp = t_surface0 + expected_heat / c_surface;

        assert!(
            (fluid_temp - expected_fluid_temp).abs() < 1e-9,
            "fluid_temp={fluid_temp} expected={expected_fluid_temp}"
        );
        assert!(
            (surface_temp - expected_surface_temp).abs() < 1e-9,
            "surface_temp={surface_temp} expected={expected_surface_temp}"
        );

        // 対記帳: 流体側が失った熱量 == 受熱面側が得た熱量(2ノード間でゼロ和)。
        let heat_lost_by_fluid = c_fluid * (t_fluid0 - fluid_temp);
        let heat_gained_by_surface = c_surface * (surface_temp - t_surface0);
        assert!(
            (heat_lost_by_fluid - heat_gained_by_surface).abs() < 1e-9,
            "heat_lost_by_fluid={heat_lost_by_fluid} heat_gained_by_surface={heat_gained_by_surface}"
        );
    }
}
