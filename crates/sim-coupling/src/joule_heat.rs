//! `JouleHeat`(設計 docs/20-integration/01-coupling-matrix.md §3「P2: 回路の抵抗損失
//! (ジュール熱) → ThermalNode」)。
//!
//! **縮約実装の理由**: `DissipationToHeat`と同様、各抵抗がどの`ThermalNode`に対応するか
//! (回路基板上の位置ごとの熱容量割り当て)の対応表はまだ存在しない。そのため本実装も
//! 単一の対象`ThermalNode`に回路全体の抵抗損失を注入する縮約版とする。
//!
//! 散逸源は`sim_em::Circuit::resistor_power(i)`(瞬時電力 $P=V^2/R$、`Circuit`の
//! `Solver`実装のdoc参照)を全抵抗について合計し、`dt`を掛けて区間エネルギーとする
//! (瞬時電力なので蓄積量ではなく、`DissipationToHeat`の`last_contact_dissipation`とは
//! 異なり毎回`Circuit`側から改めて読み出すだけで良い — リセットは不要)。

use crate::domain_states::{Coupling, DomainStates};
use sim_core::DomainId;

/// 回路の全抵抗の瞬時消費電力(ΣV²/R)を`dt`で積分し、単一の`ThermalNode`
/// (`thermal_node`インデックス)へ注入する(設計§1「保存量の橋は必ず対で書く」)。
pub struct JouleHeat {
    pub thermal_node: usize,
}

impl Coupling for JouleHeat {
    fn domains(&self) -> (DomainId, DomainId) {
        (DomainId::Electromagnetism, DomainId::Thermal)
    }

    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        let Some(circuit) = &world.em_circuit else {
            return;
        };
        let mut power = 0.0;
        for i in 0..circuit.resistor_count() {
            power += circuit.resistor_power(i);
        }
        let heat = power * dt;
        if heat != 0.0 {
            if let Some(thermal) = &mut world.thermal {
                if let Some(node) = thermal.nodes.get_mut(self.thermal_node) {
                    // 対記帳: 回路側から読み出した瞬時電力の区間積分(heat)を
                    // そのままthermal側へ注入する(ΔE = C・ΔT)。
                    node.temperature += heat / node.heat_capacity;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_em::{Circuit, GROUND};
    use sim_mechanics::MechanicsSolver;
    use sim_thermal::{ThermalNode, ThermalSolver};

    /// 定電圧源+単一抵抗の回路が定常状態に達した後、`JouleHeat`が注入した熱量が
    /// オームの法則から予測される定常電力($P=V^2/R$)× 経過時間とほぼ一致することを
    /// 確認する(設計§1「保存量の橋」の対記帳)。
    ///
    /// `DomainStates::mechanics`は必須フィールド(`World`では常時有効なドメインのため)
    /// だが、この回路単体のテストでは力学は無関係なので、空(ボディなし)の
    /// `MechanicsSolver`をダミーとして渡す(`step`は呼ばない — 力学の時間発展はこの
    /// テストの検証対象外)。
    #[test]
    fn joule_heat_matches_steady_state_i_squared_r_power() {
        let v0 = 10.0;
        let r = 100.0;
        let mut circuit = Circuit::new(2);
        circuit.add_voltage_source(1, GROUND, v0);
        circuit.add_resistor(1, GROUND, r);

        let mut thermal = ThermalSolver::new(293.15);
        let node_idx = thermal.add_node(ThermalNode::new(293.15, 1000.0));
        let mut coupling = JouleHeat {
            thermal_node: node_idx,
        };
        let mut mechanics = MechanicsSolver::new(9.80665);

        let dt = 1.0 / 1000.0;
        // RC/RL要素が無い純抵抗回路なので初回のNewton解で即座に定常状態に達する。
        // 定常電力に対して十分な熱容量比を保ちつつ、対記帳誤差を平均化するため
        // 十分な時間(2秒)を積分する。
        let steps = 2000;
        for _ in 0..steps {
            circuit.step(dt);
            let mut states = DomainStates {
                mechanics: &mut mechanics,
                thermal: Some(&mut thermal),
                em_circuit: Some(&mut circuit),
            };
            coupling.apply(&mut states, dt);
        }

        let final_temp = thermal.nodes[node_idx].temperature;
        let heat_gained = 1000.0 * (final_temp - 293.15);
        let expected_power = v0 * v0 / r;
        let expected_heat = expected_power * dt * steps as f64;
        let rel_err = (heat_gained - expected_heat).abs() / expected_heat;
        assert!(
            rel_err < 0.01,
            "heat_gained={heat_gained:.6} expected_heat={expected_heat:.6} rel_err={rel_err:.6}"
        );
    }
}
