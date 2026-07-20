//! 導体棒の電磁誘導(レール上を滑る導体棒)。設計: docs/13-electromagnetism/05-em-mechanics-coupling.md §2.2。
//!
//! ファラデー則 $\mathcal{E}=B\ell v$ を起電力として回路に注入し、流れた電流が受ける力
//! $F=-Bi\ell$(レンツ則、運動を減速)で棒の運動を減速させる自己無撞着なモデル
//! (インダクタンスは無視した準静的近似、回路は棒自身の抵抗のみの単純ループ)。

/// レール上の導体棒。誘導ブレーキのみで外力は加えない(自由減速、設計§7「発電」の縮約)。
pub struct InductionRod {
    pub mass: f64,
    pub length: f64,
    pub magnetic_field: f64,
    pub circuit_resistance: f64,
    pub velocity: f64,
}

impl InductionRod {
    pub fn new(
        mass: f64,
        length: f64,
        magnetic_field: f64,
        circuit_resistance: f64,
    ) -> InductionRod {
        InductionRod {
            mass,
            length,
            magnetic_field,
            circuit_resistance,
            velocity: 0.0,
        }
    }

    /// 起電力 $\mathcal{E}=B\ell v$(設計§2.2)。
    pub fn emf(&self) -> f64 {
        self.magnetic_field * self.length * self.velocity
    }

    /// 準静的電流(インダクタンス無視)$i=\mathcal{E}/R$。
    pub fn current(&self) -> f64 {
        self.emf() / self.circuit_resistance
    }

    /// レンツ則の制動力 $F=-Bi\ell$ による1ステップの速度更新(semi-implicit Euler)。
    /// $\dot v=-\frac{B^2\ell^2}{mR}v$ という線形減衰になり、時定数 $\tau=mR/(B\ell)^2$ の
    /// 指数減衰が解析解(設計§7の検証対象そのもの)。
    pub fn step(&mut self, dt: f64) {
        let current = self.current();
        let force = -self.magnetic_field * current * self.length;
        self.velocity += dt * force / self.mass;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// E7: 誘導起電力 $\mathcal{E}=B\ell v$、rel 0.5%(docs/21-verification/01-analytic-tests.md
    /// E7)。棒はレンツ則の制動力のみで自由減速し、速度は解析的に厳密な指数減衰
    /// $v(t)=v_0 e^{-t/\tau}$、$\tau=mR/(B\ell)^2$ に従う(電磁誘導と力学の自己無撞着な結合の
    /// 検証そのもの)。シミュレートされた $v(t)$ が解析解に一致することを確認し、その上で
    /// $\mathcal{E}(t)=B\ell v(t)$ を式に代入するだけであることを明示的に確認する。
    #[test]
    fn e7_induced_emf_matches_b_l_v_during_self_consistent_decay() {
        let mass = 0.01;
        let length = 0.1;
        let b = 0.5;
        let r = 1.0;
        let v0 = 1.0;

        let mut rod = InductionRod::new(mass, length, b, r);
        rod.velocity = v0;

        let tau = mass * r / (b * length).powi(2);
        let dt = 0.001;
        let steps = 2000u32; // t = 2s ≈ tau/2
        for _ in 0..steps {
            rod.step(dt);
        }
        let t = steps as f64 * dt;

        let expected_v = v0 * (-t / tau).exp();
        let rel_err_v = (rod.velocity - expected_v).abs() / expected_v;
        assert!(
            rel_err_v < 0.005,
            "v={} expected_v={expected_v} rel_err={rel_err_v}",
            rod.velocity
        );

        let expected_emf = b * length * expected_v;
        let rel_err_emf = (rod.emf() - expected_emf).abs() / expected_emf;
        assert!(
            rel_err_emf < 0.005,
            "emf={} expected_emf={expected_emf} rel_err={rel_err_emf}",
            rod.emf()
        );
    }
}
