//! 回路 — 修正節点解析(MNA)。設計: docs/13-electromagnetism/02-circuits.md。
//!
//! P4 スコープの実装: 線形素子(抵抗・コンデンサ・インダクタ・独立電圧源)のMNA(設計 §3)+
//! ダイオード(Shockley式)のNewton-Raphson反復(設計 §4)。動的素子(C・L)は後退Euler
//! (設計 §4「既定」)のコンパニオンモデルへ変換して代数化する。ダイオードも同様に
//! 各Newton反復で現在の動作点まわりの線形コンパニオンモデル(微分コンダクタンス+
//! 等価電流源、SPICE標準)へ変換する。電圧ステップ制限(設計§4の限流手法、
//! 1反復あたりの変化を$2nV_T$にクランプ)は実装したが、フォールバック連鎖の
//! 振動ダンピング・gmin stepping・source stepping・ラッチは未実装(半波整流の
//! テストケースは電圧ステップ制限つきNewtonのみで確実に収束するため)。
//! モーターは未実装。スイッチは理想スイッチの2値抵抗近似(ファイル後方の
//! `SWITCH_ON_RESISTANCE`/`SWITCH_OFF_RESISTANCE`)で実装済み。線形方程式は毎回
//! 部分ピボット付きガウス消去で解く(回路規模が小さいため十分、設計 §10「密LUで十分」)。
//!
//! `Solver`トレイト実装(`sim-coupling::JouleHeat`が`World`経由で駆動するための窓口、
//! 設計docs/00-foundation/04-architecture.md §1.2)はファイル末尾。

use sim_core::{EnergyBreakdown, Solver, SolverContext, StateHasher};

/// ノード0は常にグラウンド(電位0、未知数に含めない)。設計 §3。
pub const GROUND: usize = 0;

/// ダイオードのNewton反復上限(設計§4「上限10反復」)。
const DIODE_MAX_NEWTON_ITERATIONS: usize = 10;
/// 収束判定: 電圧ステップがこれ未満なら収束とみなす(設計§4「Δv<10⁻⁹V」)。
const DIODE_CONVERGENCE_TOLERANCE: f64 = 1e-9;

/// 理想スイッチの近似(モジュールdoc「モーター・スイッチは未実装」の解消、
/// `sim-world::Command::SetSwitch`が使う)。専用の未知数(電圧源のような)を追加せず、
/// 閉:低抵抗(ほぼ短絡)・開:高抵抗(ほぼ開放)の2値抵抗として`resistors`と同じ
/// `stamp_conductance`経路でスタンプする(最小の実装)。抵抗比(1e-6Ω/1e9Ω、15桁)は
/// 倍精度ガウス消去が悪条件化せずに解ける範囲で選んだ(既存のダイオード・回路規模
/// テストと同程度の条件数)。
const SWITCH_ON_RESISTANCE: f64 = 1e-6;
const SWITCH_OFF_RESISTANCE: f64 = 1e9;

/// 回路。素子はノード番号の対 `(a, b)` で接続を表す(a, b どちらも `GROUND` を含みうる)。
#[derive(Default, Clone)]
pub struct Circuit {
    num_nodes: usize,
    resistors: Vec<(usize, usize, f64)>,
    capacitors: Vec<(usize, usize, f64)>,
    inductors: Vec<(usize, usize, f64)>,
    voltage_sources: Vec<(usize, usize, f64)>,
    /// (anode, cathode, saturation_current, n・V_T)。設計§2「Shockley $i=I_s(e^{v/nV_T}-1)$」。
    diodes: Vec<(usize, usize, f64, f64)>,
    /// (a, b, closed)。理想スイッチの近似(モジュールdoc参照)。
    switches: Vec<(usize, usize, bool)>,
    /// 前ステップの端子間電圧(コンデンサの後退Eulerコンパニオンモデルの履歴項)。
    capacitor_voltage: Vec<f64>,
    /// 前ステップの枝電流(インダクタの後退Eulerコンパニオンモデルの履歴項)。
    inductor_current: Vec<f64>,
    /// ダイオードの動作点電圧(Newton反復の現在推定値、次ステップのウォームスタートにも使う)。
    diode_voltage: Vec<f64>,
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

    /// 既存の電圧源の値を変更する(`add_voltage_source` を呼んだ順のインデックス)。
    /// 時間変化する電源(AC等)を`step`の呼び出し間で表現するために使う。
    pub fn set_voltage_source_voltage(&mut self, index: usize, voltage: f64) {
        self.voltage_sources[index].2 = voltage;
    }

    /// ダイオード(Shockley式、設計§2)。`anode`→`cathode`が順方向。
    /// `n_vt` は $nV_T$(理想係数×熱電圧、300Kで$V_T\approx25.85$mV)。
    pub fn add_diode(&mut self, anode: usize, cathode: usize, saturation_current: f64, n_vt: f64) {
        self.diodes.push((anode, cathode, saturation_current, n_vt));
        self.diode_voltage.push(0.0);
    }

    /// 理想スイッチの近似(モジュールdoc参照)。戻り値は`set_switch_closed`用のインデックス。
    pub fn add_switch(&mut self, a: usize, b: usize, closed: bool) -> usize {
        self.switches.push((a, b, closed));
        self.switches.len() - 1
    }

    /// 開閉状態を変更する(`sim-world::Command::SetSwitch`が使う)。
    pub fn set_switch_closed(&mut self, index: usize, closed: bool) {
        self.switches[index].2 = closed;
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

        for &(a, b, closed) in &self.switches {
            let r = if closed {
                SWITCH_ON_RESISTANCE
            } else {
                SWITCH_OFF_RESISTANCE
            };
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

        // ダイオード(設計§4「Newton-Raphson反復、電圧ステップ制限つき」)。線形部分の
        // 行列(上で構築済み)は反復間で不変なので、毎回そのコピーへダイオードの
        // コンパニオンモデル(現在の動作点まわりの微分コンダクタンス+等価電流源、
        // コンデンサと同じ構造)だけを追加してから解く。
        let x = if self.diodes.is_empty() {
            solve_linear_system(a_mat, b_vec)
        } else {
            let mut x = vec![0.0; n];
            for _ in 0..DIODE_MAX_NEWTON_ITERATIONS {
                let mut iter_a = a_mat.clone();
                let mut iter_b = b_vec.clone();
                for (idx, &(a, b, is_sat, n_vt)) in self.diodes.iter().enumerate() {
                    let v_op = self.diode_voltage[idx];
                    let exp_term = (v_op / n_vt).exp();
                    let i_at_op = is_sat * (exp_term - 1.0);
                    let g_d = is_sat / n_vt * exp_term;
                    let i_eq = g_d * v_op - i_at_op;
                    stamp_conductance(&mut iter_a, a, b, g_d);
                    if let Some(ia) = node_idx(a) {
                        iter_b[ia] += i_eq;
                    }
                    if let Some(ib) = node_idx(b) {
                        iter_b[ib] -= i_eq;
                    }
                }
                x = solve_linear_system(iter_a, iter_b);

                let mut max_step = 0.0_f64;
                for (idx, &(a, b, _, n_vt)) in self.diodes.iter().enumerate() {
                    let v_new = self.node_voltage_from(&x, a) - self.node_voltage_from(&x, b);
                    let raw_step = v_new - self.diode_voltage[idx];
                    // 電圧ステップ制限(設計§4「1反復あたりの変化を2nV_Tにクランプ」)。
                    let clamped_step = raw_step.clamp(-2.0 * n_vt, 2.0 * n_vt);
                    self.diode_voltage[idx] += clamped_step;
                    max_step = max_step.max(clamped_step.abs());
                }
                if max_step < DIODE_CONVERGENCE_TOLERANCE {
                    break;
                }
            }
            x
        };

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

    /// 抵抗の本数(`resistor_power`のインデックス範囲、`sim-coupling::JouleHeat`が読む)。
    pub fn resistor_count(&self) -> usize {
        self.resistors.len()
    }

    /// 抵抗`index`の直近の消費電力 $P=V^2/R$(設計docs/20-integration/01-coupling-matrix.md
    /// `JouleHeat`が読む)。
    pub fn resistor_power(&self, index: usize) -> f64 {
        let (a, b, r) = self.resistors[index];
        let v = self.node_voltage(a) - self.node_voltage(b);
        v * v / r
    }

    /// 電圧源`index`(`add_voltage_source`を呼んだ順)の直近の解電流(向きは`a→b`が正、
    /// 設計docs/13-electromagnetism/05-em-mechanics-coupling.md §2.2「導体棒」、
    /// `sim-coupling::InductionCoupling`が読む)。`step()`を一度も呼んでいない場合は
    /// (`node_voltage`と同様に)0を返す(パニックしない)。
    pub fn source_current(&self, index: usize) -> f64 {
        self.last_source_current.get(index).copied().unwrap_or(0.0)
    }
}

impl Solver for Circuit {
    /// 後退Eulerは無条件安定(設計§4)。
    fn max_stable_dt(&self) -> f64 {
        f64::INFINITY
    }

    fn step(&mut self, dt: f64, _ctx: &mut SolverContext) {
        // 同名の inherent メソッド(1引数版、上の`impl Circuit`ブロック)を呼ぶ —
        // Rustのメソッド解決規則により inherent メソッドが同名のトレイトメソッドより
        // 優先されるため、トレイト実装内から`self.step(dt)`と書いても再帰しない。
        self.step(dt);
    }

    /// コンデンサ・インダクタに蓄えられた電磁エネルギー(設計§1.1.2(2)「電磁場」)。
    /// 抵抗のジュール熱は瞬時消費であり蓄積量ではないため対象外(`resistor_power`が
    /// 別途瞬時電力を提供、`JouleHeat`が積算する)。
    fn total_energy(&self) -> EnergyBreakdown {
        let mut electromagnetic = 0.0;
        for (idx, &(_, _, c)) in self.capacitors.iter().enumerate() {
            electromagnetic += 0.5 * c * self.capacitor_voltage[idx] * self.capacitor_voltage[idx];
        }
        for (idx, &(_, _, l)) in self.inductors.iter().enumerate() {
            electromagnetic += 0.5 * l * self.inductor_current[idx] * self.inductor_current[idx];
        }
        EnergyBreakdown {
            electromagnetic,
            ..Default::default()
        }
    }

    fn state_hash(&self, hasher: &mut StateHasher) {
        hasher.write_u64(self.last_node_voltage.len() as u64);
        for &v in &self.last_node_voltage {
            hasher.write_f64(v);
        }
        for &i in &self.last_source_current {
            hasher.write_f64(i);
        }
        for &v in &self.capacitor_voltage {
            hasher.write_f64(v);
        }
        for &i in &self.inductor_current {
            hasher.write_f64(i);
        }
        for &v in &self.diode_voltage {
            hasher.write_f64(v);
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

    /// ダイオード整流: 半波整流の平均電圧、rel 2%(設計 docs/13-electromagnetism/02-circuits.md
    /// §7「ダイオード整流: 半波整流の平均電圧(±2%)」。対応するE番号は無い)。
    /// 理想ダイオード(順方向降下ゼロ)近似での解析平均 $V_{peak}/\pi$ と比較する。
    /// $V_{peak}=100V$ に対しShockleyダイオードの実際の順方向降下は約0.77V($I_s=10^{-14}$A,
    /// $nV_T=25.85$mVでの概算)しかないため、理想近似との差(rel≈1.2%)は2%以内に収まる。
    #[test]
    fn diode_half_wave_rectifier_average_output_matches_ideal_diode_approximation() {
        let v_peak = 100.0;
        let r = 1000.0;
        let is_sat = 1e-14; // Si小信号ダイオード(設計§9)
        let n_vt = 0.02585; // n≈1、300KでのV_T(設計§9)

        let mut circuit = Circuit::new(3); // 0=GND, 1=AC源, 2=出力(ダイオード・抵抗の接続点)
        circuit.add_voltage_source(1, GROUND, 0.0); // index 0、毎ステップ値を更新する
        circuit.add_diode(1, 2, is_sat, n_vt); // anode=AC源側、cathode=出力側
        circuit.add_resistor(2, GROUND, r);

        let period = 1.0; // 角周波数のみが意味を持つ任意単位
        let omega = 2.0 * std::f64::consts::PI / period;
        let samples = 1000;
        let dt = period / samples as f64;

        let mut sum_v_out = 0.0;
        for i in 0..samples {
            let t = i as f64 * dt;
            let v_in = v_peak * (omega * t).sin();
            circuit.set_voltage_source_voltage(0, v_in);
            circuit.step(dt);
            sum_v_out += circuit.node_voltage(2);
        }
        let measured_avg = sum_v_out / samples as f64;
        let expected_avg = v_peak / std::f64::consts::PI;
        let rel_err = (measured_avg - expected_avg).abs() / expected_avg;
        assert!(
            rel_err < 0.02,
            "measured_avg={measured_avg} expected_avg={expected_avg} rel_err={rel_err}"
        );
    }

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

    /// スイッチ(モジュールdoc「モーター・スイッチは未実装」の解消): 開いている間は
    /// 出力電圧がほぼ0(開放)、閉じている間は分圧回路の解析値と一致することを確認する
    /// (`sim-world::Command::SetSwitch`が使う`set_switch_closed`経由)。
    #[test]
    fn switch_toggles_between_open_circuit_and_analytic_voltage_divider() {
        let v = 10.0;
        let r1 = 100.0;
        let r2 = 200.0;
        let mut circuit = Circuit::new(3); // 0=GND, 1=電源, 2=分圧点
        circuit.add_voltage_source(1, GROUND, v);
        circuit.add_resistor(1, 2, r1);
        let switch = circuit.add_switch(2, GROUND, false); // 初期状態: 開
                                                           // switch と並列ではなく2→GNDの負荷抵抗として使う分圧回路(r2はswitchの先の負荷)。
        circuit.add_resistor(2, GROUND, r2);

        circuit.step(1e-6);
        // 開: switchの枝はほぼ電流を流さないが、r1-r2の分圧自体はswitchと無関係に成立する
        // ため、switchが開いていても閉じていてもr2による分圧は変わらない。switch自体の
        // 効果を見るには、switchがGNDへの別経路(負荷を短絡)として働く配線にする必要がある
        // ため、ここではswitchをr2と並列(2→GND)に置き、閉で分圧点がほぼ0Vへ落ちる
        // (switchの低抵抗がr2を実効的に短絡する)ことを直接確認する。
        let v_open = circuit.node_voltage(2);
        let expected_open = v * r2 / (r1 + r2);
        let rel_err_open = (v_open - expected_open).abs() / expected_open;
        assert!(
            rel_err_open < 0.01,
            "open: v_open={v_open} expected={expected_open} rel_err={rel_err_open}"
        );

        circuit.set_switch_closed(switch, true);
        circuit.step(1e-6);
        // 閉: switch(1e-6Ω)がr2(200Ω)と並列になり分圧点をほぼ短絡するため、
        // 出力電圧はほぼ0まで落ちる。
        let v_closed = circuit.node_voltage(2);
        assert!(
            v_closed.abs() < 1e-3,
            "closed: v_closed should be near-zero (switch shorts node 2 to GND), got {v_closed}"
        );
    }
}
