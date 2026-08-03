//! `JouleHeat`(設計 docs/20-integration/01-coupling-matrix.md §3「P2: 回路の抵抗損失
//! (ジュール熱) → ThermalNode」)。
//!
//! **群5で素子ごとの熱ノード割り当てを実装した**。移行前は「各抵抗がどの`ThermalNode`に
//! 対応するか(回路基板上の位置ごとの熱容量割り当て)の対応表が存在しない」という理由で、
//! 単一の`ThermalNode`へ回路全体の抵抗損失を注入する縮約版だった。群5では対応表
//! `resistor_nodes`を**この結合自身が持つ**(`DissipationToHeat::body_links`・
//! `ConvectionLink`の物性値と同じ「呼び出し側が直接渡す」パターン)。
//!
//! 設計 docs/12-thermal/02-heat-transfer.md §4.4 は「ジュール熱は**素子のノードへ全量**」と
//! 明記しており、`DissipationToHeat`のような比分配は無い——抵抗$i$の損失は素子$i$自身が
//! 発熱源なので、対応表を引いて全量をそのノードへ入れるだけで設計どおりになる。
//! 対応表に無い抵抗の分は既定ノード`thermal_node`へ落とす(空なら移行前と同じ挙動)。
//!
//! 散逸源は`sim_em::Circuit::resistor_power(i)`(瞬時電力 $P=V^2/R$、`Circuit`の
//! `Solver`実装のdoc参照)を全抵抗について合計し、`dt`を掛けて区間エネルギーとする
//! (瞬時電力なので蓄積量ではなく、`DissipationToHeat`の`last_contact_dissipation`とは
//! 異なり毎回`Circuit`側から改めて読み出すだけで良い — リセットは不要)。

use crate::domain_states::{Coupling, CouplingKind, DomainStates};
use sim_core::DomainId;

/// 回路の全抵抗の瞬時消費電力(ΣV²/R)を`dt`で積分し、単一の`ThermalNode`
/// (`thermal_node`インデックス)へ注入する(設計§1「保存量の橋は必ず対で書く」)。
#[derive(Clone)]
pub struct JouleHeat {
    /// 対応表(`resistor_nodes`)に載っていない抵抗の損失を受ける既定ノード。
    pub thermal_node: usize,
    /// 抵抗index → `ThermalNode`index の対応表(**群5で追加**、モジュールdoc参照)。
    /// 空なら移行前と同じ「回路全体の損失を`thermal_node`へ」。
    pub resistor_nodes: Vec<(usize, usize)>,
}

impl JouleHeat {
    /// 対応表なし(移行前と同じ「回路全体の損失を単一ノードへ」)。
    pub fn to_single_node(thermal_node: usize) -> JouleHeat {
        JouleHeat {
            thermal_node,
            resistor_nodes: Vec::new(),
        }
    }

    fn node_of(&self, resistor: usize) -> usize {
        self.resistor_nodes
            .iter()
            .find(|(r, _)| *r == resistor)
            .map_or(self.thermal_node, |(_, node)| *node)
    }
}

impl Coupling for JouleHeat {
    fn kind(&self) -> CouplingKind {
        CouplingKind::JouleHeat
    }

    fn domain_ids(&self) -> &'static [DomainId] {
        &[DomainId::Electromagnetism, DomainId::Thermal]
    }

    fn describe(&self) -> String {
        format!("JouleHeat -> thermal_node[{}]", self.thermal_node)
    }

    fn referenced_thermal_nodes(&self) -> Vec<usize> {
        let mut nodes = vec![self.thermal_node];
        for (_, node) in &self.resistor_nodes {
            if !nodes.contains(node) {
                nodes.push(*node);
            }
        }
        nodes
    }

    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        let Some(circuit) = &world.em_circuit else {
            return;
        };
        // 抵抗ごとに (行き先ノード, 熱量) を組み立てる(設計 §4.4「素子のノードへ全量」、
        // モジュールdoc参照)。対応表が空なら全て既定ノードに集まるので、移行前と
        // 完全に同じ結果になる。
        let mut injections: Vec<(usize, f64)> = Vec::new();
        for i in 0..circuit.resistor_count() {
            let heat = circuit.resistor_power(i) * dt;
            if heat == 0.0 {
                continue;
            }
            let node = self.node_of(i);
            match injections.iter_mut().find(|(n, _)| *n == node) {
                Some(entry) => entry.1 += heat,
                None => injections.push((node, heat)),
            }
        }
        if let Some(thermal) = &mut world.thermal {
            for (node_index, heat) in injections {
                if let Some(node) = thermal.nodes.get_mut(node_index) {
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
        let mut coupling = JouleHeat::to_single_node(node_idx);
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
                em_electrostatics: None,
                gas: None,
                grid_fluid: None,
                sph: None,
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

    /// **群5: 素子ごとの熱ノード割り当て**(設計 docs/12-thermal/02-heat-transfer.md §4.4
    /// 「ジュール熱は素子のノードへ全量」)。抵抗値の違う2本を別々のノードへ割り当て、
    /// それぞれの $P=V^2/R$ どおりの熱量が**その素子のノードだけ**に入ること、
    /// 対応表に無い3本目が既定ノードへ落ちること、総熱量が単一ノード版と一致することを
    /// 確認する。
    #[test]
    fn joule_heat_routes_each_resistors_loss_to_its_own_node() {
        let v0 = 10.0;
        let (r0, r1, r2) = (100.0, 200.0, 400.0);
        let build_circuit = || {
            let mut circuit = Circuit::new(2);
            circuit.add_voltage_source(1, GROUND, v0);
            circuit.add_resistor(1, GROUND, r0); // index 0
            circuit.add_resistor(1, GROUND, r1); // index 1
            circuit.add_resistor(1, GROUND, r2); // index 2(対応表に載せない)
            circuit
        };

        let mut thermal = ThermalSolver::new(293.15);
        let env = thermal.add_node(ThermalNode::new(293.15, 1000.0));
        let node0 = thermal.add_node(ThermalNode::new(293.15, 1000.0));
        let node1 = thermal.add_node(ThermalNode::new(293.15, 1000.0));
        let mut coupling = JouleHeat {
            thermal_node: env,
            resistor_nodes: vec![(0, node0), (1, node1)],
        };
        let mut circuit = build_circuit();
        let mut mechanics = MechanicsSolver::new(9.80665);

        let dt = 1.0 / 1000.0;
        let steps = 2000;
        for _ in 0..steps {
            circuit.step(dt);
            coupling.apply(
                &mut DomainStates {
                    mechanics: &mut mechanics,
                    thermal: Some(&mut thermal),
                    em_circuit: Some(&mut circuit),
                    em_electrostatics: None,
                    gas: None,
                    grid_fluid: None,
                    sph: None,
                },
                dt,
            );
        }

        let elapsed = dt * steps as f64;
        let heat_of = |node: usize| 1000.0 * (thermal.nodes[node].temperature - 293.15);
        for (node, r) in [(node0, r0), (node1, r1), (env, r2)] {
            let expected = v0 * v0 / r * elapsed;
            let measured = heat_of(node);
            let rel_err = (measured - expected).abs() / expected;
            assert!(
                rel_err < 0.01,
                "R={r}Ω の損失はその素子のノードへ全量入るはず: \
                 measured={measured:.4} expected={expected:.4} rel_err={rel_err:.4}"
            );
        }

        // 総熱量は単一ノード版と一致する(行き先が分かれるだけ)。
        let mut thermal_single = ThermalSolver::new(293.15);
        let only = thermal_single.add_node(ThermalNode::new(293.15, 1000.0));
        let mut single = JouleHeat::to_single_node(only);
        let mut circuit_single = build_circuit();
        for _ in 0..steps {
            circuit_single.step(dt);
            single.apply(
                &mut DomainStates {
                    mechanics: &mut mechanics,
                    thermal: Some(&mut thermal_single),
                    em_circuit: Some(&mut circuit_single),
                    em_electrostatics: None,
                    gas: None,
                    grid_fluid: None,
                    sph: None,
                },
                dt,
            );
        }
        let total_split = heat_of(node0) + heat_of(node1) + heat_of(env);
        let total_single = 1000.0 * (thermal_single.nodes[only].temperature - 293.15);
        assert!(
            (total_split - total_single).abs() / total_single < 1e-9,
            "総熱量は割り当て方によらず同じはず: split={total_split} single={total_single}"
        );
    }
}
