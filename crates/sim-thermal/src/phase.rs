//! 相変化(エンタルピー法)。設計: docs/12-thermal/03-phase-change.md §2.2/§3。
//!
//! 温度でなく比エンタルピー H [J/kg] を状態変数とし、T(H) を区分関数で引く。
//! 本実装では H の原点を「融点における固相の終端」に固定する(設計の一般式の
//! `H_sol` が恒等的に 0 になる特殊化。相境界(Stefan 問題)を陽に追跡せず、
//! 熱収支だけで融解・凝固が進む設計の利点はそのまま保たれる)。

/// 相変化を起こす物質のパラメータ(設計§9パラメータ表)。
#[derive(Clone, Copy, Debug)]
pub struct PhaseMaterial {
    /// 融点 T_m [K]。
    pub melting_temperature: f64,
    /// 融解熱 L_f [J/kg]。
    pub latent_heat_fusion: f64,
    /// 固相比熱 c_p,s [J/(kg·K)]。
    pub specific_heat_solid: f64,
    /// 液相比熱 c_p,l [J/(kg·K)]。
    pub specific_heat_liquid: f64,
}

/// 相(設計§3)。液相率は混合相のみ意味を持つ。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Phase {
    Solid,
    Mixed { liquid_fraction: f64 },
    Liquid,
}

/// エンタルピー法の状態。`enthalpy` の原点は融点における固相の終端(H=0)に固定する。
#[derive(Clone, Copy, Debug)]
pub struct PhaseState {
    pub enthalpy: f64,
    pub mass: f64,
}

impl PhaseState {
    /// T(H) の区分関数(設計§2.2)。
    pub fn temperature(&self, mat: &PhaseMaterial) -> f64 {
        if self.enthalpy < 0.0 {
            mat.melting_temperature + self.enthalpy / mat.specific_heat_solid
        } else if self.enthalpy <= mat.latent_heat_fusion {
            mat.melting_temperature
        } else {
            mat.melting_temperature
                + (self.enthalpy - mat.latent_heat_fusion) / mat.specific_heat_liquid
        }
    }

    /// 現在の相(混合相では液相率 φ=(H-H_sol)/L_f も返す、設計§2.2)。
    pub fn phase(&self, mat: &PhaseMaterial) -> Phase {
        if self.enthalpy < 0.0 {
            Phase::Solid
        } else if self.enthalpy <= mat.latent_heat_fusion {
            Phase::Mixed {
                liquid_fraction: self.enthalpy / mat.latent_heat_fusion,
            }
        } else {
            Phase::Liquid
        }
    }

    /// 熱量 `q` [J] を加える(質量一定、設計§4「熱流の積算先をTでなくHにする」)。
    pub fn add_heat(&mut self, q: f64) {
        self.enthalpy += q / self.mass;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// T7: 融解プラトー — 加熱曲線が T_m で水平になり、プラトー長 = mL_f/Q̇ ± 1%
    /// (docs/21-verification/01-analytic-tests.md T7、docs/12-thermal/03-phase-change.md §7)。
    /// 一定加熱率 Q̇ で固相(T_m-10K)から液相(T_m+10K)まで加熱し、混合相
    /// (`Phase::Mixed`)に留まっていた時間を測る。
    #[test]
    fn t7_melting_plateau_duration_matches_m_lf_over_q_dot() {
        let mat = PhaseMaterial {
            melting_temperature: 273.15,
            latent_heat_fusion: 334_000.0,
            specific_heat_solid: 2100.0,
            specific_heat_liquid: 4186.0,
        };
        let mass = 0.1;
        let q_dot = 50.0;
        let mut state = PhaseState {
            enthalpy: -mat.specific_heat_solid * 10.0,
            mass,
        };

        let dt = 0.02;
        let total_energy_needed = mass
            * (mat.specific_heat_solid * 10.0
                + mat.latent_heat_fusion
                + mat.specific_heat_liquid * 10.0);
        let total_time = total_energy_needed / q_dot * 1.1;
        let steps = (total_time / dt) as u32;

        let mut plateau_start: Option<f64> = None;
        let mut plateau_end: Option<f64> = None;
        let mut prev_phase = state.phase(&mat);
        for i in 0..steps {
            state.add_heat(q_dot * dt);
            let phase = state.phase(&mat);
            let t = (i + 1) as f64 * dt;
            if !matches!(prev_phase, Phase::Mixed { .. }) && matches!(phase, Phase::Mixed { .. }) {
                plateau_start = Some(t);
            }
            if matches!(prev_phase, Phase::Mixed { .. }) && !matches!(phase, Phase::Mixed { .. }) {
                plateau_end = Some(t);
            }
            prev_phase = phase;
        }

        let plateau_start = plateau_start.expect("should enter the mixed phase");
        let plateau_end = plateau_end.expect("should exit the mixed phase");
        let measured_duration = plateau_end - plateau_start;
        let expected_duration = mass * mat.latent_heat_fusion / q_dot;
        let rel_err = (measured_duration - expected_duration).abs() / expected_duration;
        assert!(
            rel_err < 0.01,
            "measured={measured_duration:.4} expected={expected_duration:.4} rel_err={rel_err:.4}"
        );
    }
}
