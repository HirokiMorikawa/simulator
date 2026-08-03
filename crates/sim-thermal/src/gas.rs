//! 気体区画(閉じた容器・ピストン)。設計: docs/12-thermal/01-thermodynamics-laws.md §2.2/§3/§4.3。

/// 気体定数 R [J/(mol·K)]。
pub const GAS_CONSTANT: f64 = 8.314_462_618;

/// 気体種(設計§9パラメータ表)。自由度 f から比熱比 γ=(f+2)/f が定まる。
#[derive(Clone, Copy, Debug)]
pub struct GasSpecies {
    pub degrees_of_freedom: f64,
    pub molar_mass: f64,
}

impl GasSpecies {
    /// 空気(設計§9パラメータ表、f=5・γ=1.40相当)。
    pub const AIR: GasSpecies = GasSpecies {
        degrees_of_freedom: 5.0,
        molar_mass: 28.97e-3,
    };
}

/// 気体区画(閉じた容器・ピストン、設計§3)。
#[derive(Clone, Copy, Debug)]
pub struct GasCompartment {
    pub n_moles: f64,
    pub volume: f64,
    pub temperature: f64,
    pub gas: GasSpecies,
}

impl GasCompartment {
    /// 理想気体の状態方程式(設計§3)。
    pub fn pressure(&self) -> f64 {
        self.n_moles * GAS_CONSTANT * self.temperature / self.volume
    }

    /// 比熱比 γ=c_p/c_v=(f+2)/f(設計§2.2)。
    pub fn heat_capacity_ratio(&self) -> f64 {
        (self.gas.degrees_of_freedom + 2.0) / self.gas.degrees_of_freedom
    }

    /// 定積モル熱容量(拡張量)C_v = (f/2)nR(設計§2.2)。
    pub fn heat_capacity_at_constant_volume(&self) -> f64 {
        0.5 * self.gas.degrees_of_freedom * self.n_moles * GAS_CONSTANT
    }

    /// 準静的な断熱体積変化(設計§4.3・§7)。閉形式公式 $TV^{\gamma-1}=const$ を
    /// 直接使わず、その微分形 $dT/T=-(\gamma-1)dV/V$(第1法則 $\frac f2 nR\,dT=-p\,dV$
    /// と状態方程式から導出)を刻み積分することで実際に検証する。
    pub fn adiabatic_quasi_static_volume_change(&mut self, target_volume: f64, steps: u32) {
        let gamma = self.heat_capacity_ratio();
        let dv = (target_volume - self.volume) / steps as f64;
        for _ in 0..steps {
            self.temperature *= 1.0 - (gamma - 1.0) * dv / self.volume;
            self.volume += dv;
        }
    }

    /// 等温での体積変化にともなう熱の出入り $Q=nRT\ln(V_2/V_1)$(設計§4.2)。正なら吸熱。
    pub fn isothermal_heat_for_volume_change(&self, target_volume: f64) -> f64 {
        self.n_moles * GAS_CONSTANT * self.temperature * (target_volume / self.volume).ln()
    }

    /// `sim-coupling::PistonGas`用: 1シミュレーションstep分の断熱体積変化を直接適用する
    /// (`adiabatic_quasi_static_volume_change`の1反復版)。ピストンの機械的time scaleは
    /// 気体分子の熱化time scaleよりずっと長い(準静的近似が成り立つ、設計§4.3)という
    /// 前提の下、1回の`Coupling::apply`呼び出し内で$dT/T=-(\gamma-1)dV/V$を1次近似で
    /// 適用する(`adiabatic_quasi_static_volume_change`が`steps`回の細分で検証している
    /// のと同じ式を、実際のシミュレーションstep一回分だけ適用する形)。
    pub fn apply_step_volume_change(&mut self, new_volume: f64) {
        let gamma = self.heat_capacity_ratio();
        let dv = new_volume - self.volume;
        self.temperature *= 1.0 - (gamma - 1.0) * dv / self.volume;
        self.volume = new_volume;
    }
}

/// カルノー効率の上限(設計§7): $\eta \le 1-T_c/T_h$。
pub fn carnot_efficiency_bound(t_hot: f64, t_cold: f64) -> f64 {
    1.0 - t_cold / t_hot
}

#[cfg(test)]
mod tests {
    use super::*;

    /// T5: 断熱圧縮 — 体積半分でT2=T1(V1/V2)^(γ-1)、rel<1%
    /// (docs/21-verification/01-analytic-tests.md T5)。
    #[test]
    fn t5_adiabatic_compression_matches_tv_gamma_minus_one_formula() {
        let mut gas = GasCompartment {
            n_moles: 1.0,
            volume: 1.0,
            temperature: 300.0,
            gas: GasSpecies::AIR,
        };
        let gamma = gas.heat_capacity_ratio();
        let v1 = gas.volume;
        let v2 = v1 / 2.0;
        gas.adiabatic_quasi_static_volume_change(v2, 10_000);

        let expected_t2 = 300.0 * (v1 / v2).powf(gamma - 1.0);
        let rel_err = (gas.temperature - expected_t2).abs() / expected_t2;
        assert!(
            rel_err < 0.01,
            "measured={:.4} expected={expected_t2:.4} rel_err={rel_err:.4}",
            gas.temperature
        );
    }

    /// T6: カルノー限界 — 任意サイクル機構の効率は $1-T_c/T_h$ を超えない
    /// (docs/21-verification/01-analytic-tests.md T6)。「任意サイクル」の完全な
    /// 網羅は単体テストでは非現実的なため、(1) 可逆なカルノーサイクル自体
    /// (等温+断熱の4行程)を構成し効率が理論値に厳密に一致すること、(2) 等積燃焼型
    /// (オットーサイクル相当、同じ最高温度・最低温度を使う)は可逆でないぶん
    /// 理論上限より厳密に低い効率になること、の2ケースで確認する。
    #[test]
    fn t6_carnot_cycle_efficiency_matches_bound_and_irreversible_cycle_stays_below() {
        let n_moles = 1.0;
        let t_hot = 1500.0;
        let t_cold = 300.0;
        let gas_species = GasSpecies::AIR;
        let gamma = (gas_species.degrees_of_freedom + 2.0) / gas_species.degrees_of_freedom;

        // (1) カルノーサイクル: 等温膨張(Th)→断熱膨張(Th→Tc)→等温圧縮(Tc)→断熱圧縮(Tc→Th)。
        let v_a = 1.0;
        let v_b = 2.0;
        let mut gas = GasCompartment {
            n_moles,
            volume: v_a,
            temperature: t_hot,
            gas: gas_species,
        };
        let q_hot = gas.isothermal_heat_for_volume_change(v_b);
        gas.volume = v_b;

        // 断熱膨張後の体積は TV^(γ-1)=const で厳密に定まる(数値積分で検証する対象は
        // T5の温度変化そのものなので、ここでは終端体積を解析式で与えてサイクルを閉じる)。
        let k = (t_hot / t_cold).powf(1.0 / (gamma - 1.0));
        let v_c = v_b * k;
        gas.adiabatic_quasi_static_volume_change(v_c, 50_000);

        let v_d = v_a * k;
        let q_cold = gas.isothermal_heat_for_volume_change(v_d); // 負(放熱)
        gas.volume = v_d;

        gas.adiabatic_quasi_static_volume_change(v_a, 50_000); // Tc→Th、サイクルを閉じる

        assert!(
            (gas.temperature - t_hot).abs() / t_hot < 0.01,
            "cycle should close back to Th, got {}",
            gas.temperature
        );

        let net_work = q_hot + q_cold; // q_cold は負
        let efficiency_carnot = net_work / q_hot;
        let expected_carnot = carnot_efficiency_bound(t_hot, t_cold);
        let rel_err = (efficiency_carnot - expected_carnot).abs() / expected_carnot;
        assert!(
            rel_err < 0.01,
            "efficiency_carnot={efficiency_carnot:.6} expected={expected_carnot:.6} rel_err={rel_err:.4}"
        );
        assert!(
            efficiency_carnot <= expected_carnot + 1e-3,
            "carnot cycle efficiency must not exceed its own bound: {efficiency_carnot} vs {expected_carnot}"
        );

        // (2) オットーサイクル相当: 断熱圧縮(Th'<Th、Tc'<Tcとなるよう圧縮比を選ぶ)→
        // 等積受熱(Th'→Th)→断熱膨張(Th→Tc')→等積放熱(Tc'→Th'側の初期温度、を閉じる)。
        // 最高温度Th・最低温度(圧縮前の初期温度)T_low_startを使ったカルノー上限と比較する。
        let compression_ratio: f64 = 8.0;
        let t_low_start = 300.0;
        let t_after_compression = t_low_start * compression_ratio.powf(gamma - 1.0);
        assert!(
            t_after_compression < t_hot,
            "must still need isochoric heating to reach Th"
        );
        let cv = 0.5 * gas_species.degrees_of_freedom * n_moles * GAS_CONSTANT;
        let q_in_otto = cv * (t_hot - t_after_compression);
        let t_after_expansion = t_hot / compression_ratio.powf(gamma - 1.0);
        assert!(
            t_after_expansion > t_low_start,
            "must still need isochoric rejection to close the cycle"
        );
        let q_out_otto = cv * (t_after_expansion - t_low_start); // 正(放熱量の大きさ)
        let efficiency_otto = 1.0 - q_out_otto / q_in_otto;

        let expected_carnot_otto_range = carnot_efficiency_bound(t_hot, t_low_start);
        assert!(
            efficiency_otto < expected_carnot_otto_range - 0.05,
            "irreversible (isochoric heat exchange) cycle efficiency {efficiency_otto} should stay meaningfully below the Carnot bound {expected_carnot_otto_range}"
        );
    }
}
