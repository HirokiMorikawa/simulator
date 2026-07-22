//! `BoussinesqBuoyancy`(設計 docs/20-integration/01-coupling-matrix.md §3
//! 「P3: 温度場 → 流体運動量」、docs/11-fluid/02-eulerian-grid.md §4.2の
//! Boussinesq浮力項 $f_y = -\beta(T-T_{amb})g$)。
//!
//! **縮約実装の理由**: 設計は温度場(`Grid3<f64>`、`GridFluid2D`本体のセルごとの温度)から
//! 空間的に変化する浮力を想定するが、`sim_fluid::GridFluid2D`(このcrateが使う縮約版)は
//! 温度場自体を持たない(`grid_fluid`モジュールdoc参照、周期境界の速度場のみ)。そのため
//! 本実装は、単一の`ThermalNode`(シーン全体を代表する「熱源」温度、`PistonGas`の
//! `GasCompartment`単一区画と同じ縮約の精神)と周囲温度`ambient_temperature`との差から、
//! 空間一様な浮力加速度を流体の速度場全体(`u`は不変、`v`のみ、鉛直=y軸)に一括で加える。
//! セルごとの温度差による渦(プルーム)の形成は再現できないが、「暖かい熱源の近くで
//! 流体全体に一様な浮力が働く」という単純化されたシーン(例: 均一に暖められた部屋の
//! 空気循環の粗い近似)には十分な精度で、テスト可能な解析的挙動(一様加速度)を持つ。
//!
//! 重力は`DomainStates::mechanics`(`World`全体で常時有効なドメイン)の
//! `gravity: f64`(大きさ、y軸下向きに`-gravity`として作用する規約、
//! `sim_mechanics::MechanicsSolver::gravity`のdoc参照)をそのまま使う —
//! 独自の重力パラメータを持たせると`World`の重力設定と食い違うリスクがあるため。

use crate::domain_states::{Coupling, DomainStates};
use sim_core::DomainId;

/// 単一の`ThermalNode`(`thermal_node`インデックス)の温度と周囲温度
/// `ambient_temperature`の差から、格子流体の速度場全体に一様なBoussinesq浮力加速度を
/// 加える(モジュールdoc参照)。
#[derive(Clone)]
pub struct BoussinesqBuoyancy {
    pub thermal_node: usize,
    /// 周囲温度 $T_{amb}$ [K]。
    pub ambient_temperature: f64,
    /// 熱膨張係数 $\beta$ [K⁻¹](設計の目安: 空気は$1/T_{amb}\approx3.4\times10^{-3}$、
    /// docs/11-fluid/02-eulerian-grid.md の表参照)。
    pub thermal_expansion_coefficient: f64,
}

impl Coupling for BoussinesqBuoyancy {
    fn domains(&self) -> (DomainId, DomainId) {
        (DomainId::Thermal, DomainId::Fluid)
    }

    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        let Some(thermal) = &world.thermal else {
            return;
        };
        let Some(node) = thermal.nodes.get(self.thermal_node) else {
            return;
        };
        let temperature = node.temperature;
        let Some(grid_fluid) = &mut world.grid_fluid else {
            return;
        };
        // f_y = -beta*(T-T_amb)*g_y、g_y = -gravity(MechanicsSolver::gravityのdoc参照:
        // 大きさで表され、y方向には-gravityとして作用する)なので
        // f_y = beta*(T-T_amb)*gravity(暖かい熱源(T>T_amb)ほど上向きに浮力が働く)。
        let accel_y = self.thermal_expansion_coefficient
            * (temperature - self.ambient_temperature)
            * world.mechanics.gravity;
        for v in grid_fluid.v.iter_mut() {
            *v += accel_y * dt;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_fluid::GridFluid2D;
    use sim_mechanics::MechanicsSolver;
    use sim_thermal::{ThermalNode, ThermalSolver};

    /// 熱源が周囲より暖かい場合、格子流体の鉛直速度場全体が解析的な浮力加速度
    /// $\beta(T-T_{amb})g$分だけ一様に増加すること(`u`は不変であること)を確認する。
    #[test]
    fn boussinesq_buoyancy_adds_uniform_upward_acceleration_matching_analytic_formula() {
        let ambient = 293.15;
        let node_temp = 313.15; // 熱源が20K暖かい
        let beta = 3.4e-3; // 空気の熱膨張係数の目安
        let gravity = 9.80665;

        let mut thermal = ThermalSolver::new(ambient);
        let node_idx = thermal.add_node(ThermalNode::new(node_temp, 1000.0));
        let mut mechanics = MechanicsSolver::new(gravity);

        let mut fluid = GridFluid2D::new(4, 4, 0.1);
        let u_before = fluid.u.clone();

        let mut coupling = BoussinesqBuoyancy {
            thermal_node: node_idx,
            ambient_temperature: ambient,
            thermal_expansion_coefficient: beta,
        };

        let dt = 0.01;
        let mut states = DomainStates {
            mechanics: &mut mechanics,
            thermal: Some(&mut thermal),
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: Some(&mut fluid),
        };
        coupling.apply(&mut states, dt);

        let expected_accel = beta * (node_temp - ambient) * gravity;
        let expected_dv = expected_accel * dt;
        for &v in &fluid.v {
            assert!(
                (v - expected_dv).abs() < 1e-12,
                "v={v} expected_dv={expected_dv}"
            );
        }
        assert_eq!(
            fluid.u, u_before,
            "u (horizontal velocity) should be unaffected"
        );
    }

    /// 熱源が周囲と同温なら浮力はゼロ(速度場は変化しない)。
    #[test]
    fn boussinesq_buoyancy_is_zero_when_node_matches_ambient_temperature() {
        let ambient = 293.15;
        let mut thermal = ThermalSolver::new(ambient);
        let node_idx = thermal.add_node(ThermalNode::new(ambient, 1000.0));
        let mut mechanics = MechanicsSolver::new(9.80665);
        let mut fluid = GridFluid2D::new(4, 4, 0.1);

        let mut coupling = BoussinesqBuoyancy {
            thermal_node: node_idx,
            ambient_temperature: ambient,
            thermal_expansion_coefficient: 3.4e-3,
        };
        let mut states = DomainStates {
            mechanics: &mut mechanics,
            thermal: Some(&mut thermal),
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: Some(&mut fluid),
        };
        coupling.apply(&mut states, 0.01);

        assert!(fluid.v.iter().all(|&v| v == 0.0));
    }
}
