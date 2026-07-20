//! 回路 — 修正節点解析(MNA)。設計: docs/13-electromagnetism/02-circuits.md。
//!
//! P4 スコープの最小実装: 線形素子(抵抗・コンデンサ・インダクタ・独立電圧源)のみの
//! MNA(設計 §3)。動的素子(C・L)は後退Euler(設計 §4「既定」)のコンパニオンモデルへ
//! 変換して代数化する。非線形素子(ダイオード・モーター)・Newton-Raphson フォールバック
//! 連鎖(gmin/source stepping)・スイッチ・トポロジ不変時のLU分解キャッシュは未実装。
//! 線形方程式はステップごとに部分ピボット付きガウス消去で解く(回路規模が小さいため
//! 十分、設計 §10「密LUで十分」)。

/// ノード0は常にグラウンド(電位0、未知数に含めない)。設計 §3。
pub const GROUND: usize = 0;

/// 回路。素子はノード番号の対 `(a, b)` で接続を表す(a, b どちらも `GROUND` を含みうる)。
#[derive(Default)]
pub struct Circuit {
    num_nodes: usize,
    resistors: Vec<(usize, usize, f64)>,
    capacitors: Vec<(usize, usize, f64)>,
    inductors: Vec<(usize, usize, f64)>,
    voltage_sources: Vec<(usize, usize, f64)>,
    /// 前ステップの端子間電圧(コンデンサの後退Eulerコンパニオンモデルの履歴項)。
    capacitor_voltage: Vec<f64>,
    /// 前ステップの枝電流(インダクタの後退Eulerコンパニオンモデルの履歴項)。
    inductor_current: Vec<f64>,
    /// 直近の解(ノード電圧、`node_voltage` で参照する)。
    last_node_voltage: Vec<f64>,
    /// 直近の解(電圧源の枝電流)。
    last_source_current: Vec<f64>,
}

impl Circuit {
    /// `num_nodes` はグラウンドを含むノード総数(ノード番号は `0..num_nodes`)。
    pub fn new(num_nodes: usize) -> Circuit {
        Circuit {
            num_nodes,
            last_node_voltage: vec![0.0; num_nodes],
            ..Default::default()
        }
    }

    pub fn add_resistor(&mut self, a: usize, b: usize, resistance: f64) {
        self.resistors.push((a, b, resistance));
    }

    /// 初期端子間電圧 `initial_voltage`(未充電なら0)。
    pub fn add_capacitor(&mut self, a: usize, b: usize, capacitance: f64, initial_voltage: f64) {
        self.capacitors.push((a, b, capacitance));
        self.capacitor_voltage.push(initial_voltage);
    }

    /// 初期電流 `initial_current`(a→b方向を正とする)。
    pub fn add_inductor(&mut self, a: usize, b: usize, inductance: f64, initial_current: f64) {
        self.inductors.push((a, b, inductance));
        self.inductor_current.push(initial_current);
    }

    /// 独立電圧源。`a` が正極、`b` が負極(`v_a - v_b = voltage`)。
    pub fn add_voltage_source(&mut self, a: usize, b: usize, voltage: f64) {
        self.voltage_sources.push((a, b, voltage));
    }

    pub fn node_voltage(&self, node: usize) -> f64 {
        if node == GROUND {
            0.0
        } else {
            self.last_node_voltage[node]
        }
    }

    /// dt 進める。MNA 行列を毎回組み立てて解く(線形素子のみなので行列自体は
    /// dt・素子値で決まり時間不変だが、キャッシュは未実装、設計 §10 の性能課題として残す)。
    pub fn step(&mut self, dt: f64) {
        let n_node_unknowns = self.num_nodes.saturating_sub(1); // GND を除く
        let n_extra = self.voltage_sources.len() + self.inductors.len();
        let n = n_node_unknowns + n_extra;

        let mut a_mat = vec![vec![0.0_f64; n]; n];
        let mut b_vec = vec![0.0_f64; n];

        let node_idx = |node: usize| -> Option<usize> {
            if node == GROUND {
                None
            } else {
                Some(node - 1)
            }
        };

        let stamp_conductance = |a_mat: &mut Vec<Vec<f64>>, a: usize, b: usize, g: f64| {
            if let Some(ia) = node_idx(a) {
                a_mat[ia][ia] += g;
            }
            if let Some(ib) = node_idx(b) {
                a_mat[ib][ib] += g;
            }
            if let (Some(ia), Some(ib)) = (node_idx(a), node_idx(b)) {
                a_mat[ia][ib] -= g;
                a_mat[ib][ia] -= g;
            }
        };

        for &(a, b, r) in &self.resistors {
            stamp_conductance(&mut a_mat, a, b, 1.0 / r);
        }

        // コンデンサ: 後退Eulerコンパニオンモデル(設計 §4)。等価コンダクタンス G_c=C/dt を
        // 抵抗と同じ形でスタンプし、前ステップ電圧による等価電流源 G_c・v_prev を
        // ノードaへ注入する(a→bを正方向とする電圧の定義に合わせた符号)。
        for (idx, &(a, b, c)) in self.capacitors.iter().enumerate() {
            let g_c = c / dt;
            stamp_conductance(&mut a_mat, a, b, g_c);
            let i_eq = g_c * self.capacitor_voltage[idx];
            if let Some(ia) = node_idx(a) {
                b_vec[ia] += i_eq;
            }
            if let Some(ib) = node_idx(b) {
                b_vec[ib] -= i_eq;
            }
        }

        // 電圧源・インダクタは枝電流を追加の未知数として持つ(設計 §3 の j)。
        // 行 K(拘束式): v_a - v_b - d・j = rhs。列K(KCL結合): ノードaへ+1・j、ノードbへ-1・j。
        let mut extra_idx = n_node_unknowns;
        for &(a, b, voltage) in &self.voltage_sources {
            let k = extra_idx;
            extra_idx += 1;
            if let Some(ia) = node_idx(a) {
                a_mat[ia][k] += 1.0;
                a_mat[k][ia] += 1.0;
            }
            if let Some(ib) = node_idx(b) {
                a_mat[ib][k] -= 1.0;
                a_mat[k][ib] -= 1.0;
            }
            b_vec[k] = voltage;
        }
        for (idx, &(a, b, inductance)) in self.inductors.iter().enumerate() {
            let k = extra_idx;
            extra_idx += 1;
            if let Some(ia) = node_idx(a) {
                a_mat[ia][k] += 1.0;
                a_mat[k][ia] += 1.0;
            }
            if let Some(ib) = node_idx(b) {
                a_mat[ib][k] -= 1.0;
                a_mat[k][ib] -= 1.0;
            }
            let l_over_dt = inductance / dt;
            a_mat[k][k] -= l_over_dt;
            b_vec[k] = -l_over_dt * self.inductor_current[idx];
        }

        let x = solve_linear_system(a_mat, b_vec);

        self.last_node_voltage = vec![0.0; self.num_nodes];
        self.last_node_voltage[1..self.num_nodes].copy_from_slice(&x[..n_node_unknowns]);

        self.last_source_current =
            x[n_node_unknowns..n_node_unknowns + self.voltage_sources.len()].to_vec();

        for (idx, &(a, b, _)) in self.capacitors.iter().enumerate() {
            self.capacitor_voltage[idx] =
                self.node_voltage_from(&x, a) - self.node_voltage_from(&x, b);
        }
        let inductor_start = n_node_unknowns + self.voltage_sources.len();
        for (idx, current_slot) in self.inductor_current.iter_mut().enumerate() {
            *current_slot = x[inductor_start + idx];
        }
    }

    fn node_voltage_from(&self, x: &[f64], node: usize) -> f64 {
        if node == GROUND {
            0.0
        } else {
            x[node - 1]
        }
    }
}

/// 部分ピボット付きガウス消去。回路規模が小さい(<10^3 節点、設計 §10)前提の密行列版。
fn solve_linear_system(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Vec<f64> {
    let n = b.len();
    for col in 0..n {
        let mut pivot_row = col;
        let mut pivot_val = a[col][col].abs();
        for (row, row_vec) in a.iter().enumerate().skip(col + 1) {
            if row_vec[col].abs() > pivot_val {
                pivot_row = row;
                pivot_val = row_vec[col].abs();
            }
        }
        a.swap(col, pivot_row);
        b.swap(col, pivot_row);

        let pivot = a[col][col];
        if pivot.abs() < 1e-15 {
            continue; // 特異(未接続ノード等)、その行は寄与なしとして0を残す
        }
        let pivot_row_vals = a[col].clone();
        for row in (col + 1)..n {
            let factor = a[row][col] / pivot;
            if factor == 0.0 {
                continue;
            }
            for (k, &pivot_val) in pivot_row_vals.iter().enumerate().skip(col) {
                a[row][k] -= factor * pivot_val;
            }
            b[row] -= factor * b[col];
        }
    }

    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let mut sum = b[row];
        for col in (row + 1)..n {
            sum -= a[row][col] * x[col];
        }
        x[row] = if a[row][row].abs() < 1e-15 {
            0.0
        } else {
            sum / a[row][row]
        };
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;

    /// E5: 分圧回路。直並列の解析値と機械精度一致(docs/21-verification/01-analytic-tests.md E5)。
    /// 動的素子が無いため、任意の dt での単一 MNA 解が厳密解と一致する(時間発展不要)。
    #[test]
    fn e5_voltage_divider_matches_analytic_solution_at_machine_precision() {
        let v0 = 9.0;
        let r1 = 1000.0;
        let r2 = 2000.0;
        let mut circuit = Circuit::new(3); // 0=GND, 1=V0側, 2=分圧点
        circuit.add_voltage_source(1, GROUND, v0);
        circuit.add_resistor(1, 2, r1);
        circuit.add_resistor(2, GROUND, r2);
        circuit.step(1.0);

        let expected = v0 * r2 / (r1 + r2);
        let measured = circuit.node_voltage(2);
        let rel_err = (measured - expected).abs() / expected;
        assert!(rel_err < 1e-9, "measured={measured} expected={expected}");
    }

    /// E3: RC過渡 $v(t)=V(1-e^{-t/RC})$、時定数の相対誤差 < 0.5%
    /// (docs/21-verification/01-analytic-tests.md E3)。2時刻の電圧比から時定数を逆算し、
    /// 指数則の形そのものを検証する(単一時刻の一致だけでなく)。
    #[test]
    fn e3_rc_transient_time_constant_matches_rc() {
        let v0 = 5.0;
        let r = 1000.0;
        let c = 1.0e-6;
        let tau = r * c;

        let mut circuit = Circuit::new(3); // 0=GND, 1=V0側, 2=コンデンサ端子
        circuit.add_voltage_source(1, GROUND, v0);
        circuit.add_resistor(1, 2, r);
        circuit.add_capacitor(2, GROUND, c, 0.0);

        let dt = tau / 2000.0;
        let (t1, t2) = (tau, 2.0 * tau);
        let mut v_at_t1 = None;
        let mut v_at_t2 = None;
        let mut t = 0.0;
        let steps = (t2 / dt).ceil() as u32 + 1;
        for _ in 0..steps {
            circuit.step(dt);
            t += dt;
            if v_at_t1.is_none() && t >= t1 {
                v_at_t1 = Some(circuit.node_voltage(2));
            }
            if v_at_t2.is_none() && t >= t2 {
                v_at_t2 = Some(circuit.node_voltage(2));
            }
        }
        let v1 = v_at_t1.expect("t1 should be reached");
        let v2 = v_at_t2.expect("t2 should be reached");

        // V0-v(t) = V0・exp(-t/τ) なので (V0-v1)/(V0-v2) = exp((t2-t1)/τ)。
        let measured_tau = (t2 - t1) / ((v0 - v1) / (v0 - v2)).ln();
        let rel_err = (measured_tau - tau).abs() / tau;
        assert!(rel_err < 0.005, "measured_tau={measured_tau} tau={tau}");
    }

    /// E4: RLC減衰振動 $\omega=\sqrt{1/LC-(R/2L)^2}$、rel 1%
    /// (docs/21-verification/01-analytic-tests.md E4)。初期充電したコンデンサを
    /// R・Lと閉ループにして自由減衰させ、コンデンサ電圧の隣接ゼロ交差の間隔(半周期)から
    /// 角周波数を実測する。
    #[test]
    fn e4_rlc_decay_angular_frequency_matches_formula() {
        let v0 = 1.0;
        let r: f64 = 10.0;
        let l: f64 = 0.01;
        let c: f64 = 1.0e-6;
        let omega = (1.0 / (l * c) - (r / (2.0 * l)).powi(2)).sqrt();
        let period = 2.0 * std::f64::consts::PI / omega;

        let mut circuit = Circuit::new(3); // 0=GND, 1=コンデンサ端子, 2=R-L接続点
        circuit.add_capacitor(1, GROUND, c, v0);
        circuit.add_resistor(1, 2, r);
        circuit.add_inductor(2, GROUND, l, 0.0);

        let dt = period / 4000.0;
        let steps = (period * 1.1 / dt) as u32;

        let mut prev_v = circuit.node_voltage(1);
        let mut prev_t = 0.0;
        let mut crossings = Vec::new();
        for step in 0..steps {
            circuit.step(dt);
            let t = (step + 1) as f64 * dt;
            let v = circuit.node_voltage(1);
            if prev_v.signum() != v.signum() && prev_v != 0.0 {
                let frac = -prev_v / (v - prev_v);
                crossings.push(prev_t + frac * (t - prev_t));
                if crossings.len() >= 2 {
                    break;
                }
            }
            prev_v = v;
            prev_t = t;
        }

        assert!(crossings.len() >= 2, "should observe two zero crossings");
        let measured_period = 2.0 * (crossings[1] - crossings[0]);
        let measured_omega = 2.0 * std::f64::consts::PI / measured_period;
        let rel_err = (measured_omega - omega).abs() / omega;
        assert!(
            rel_err < 0.01,
            "measured_omega={measured_omega} omega={omega} rel_err={rel_err}"
        );
    }
}
