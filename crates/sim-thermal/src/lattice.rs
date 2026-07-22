//! 格子熱伝導(温度場)。設計: docs/12-thermal/02-heat-transfer.md §3/§4.3。
//!
//! ノードネットワーク(`ThermalSolver`)とは別に、格子上の温度場を陰的Euler + PCGで
//! 解く。1D棒(T3の検証範囲)のみを実装し、両端はDirichlet境界(固定温度)として
//! 線形系の未知数から除外する(内部点のみを解く、標準的な境界処理)。3D `Grid3<f64>`
//! への一般化(7点ステンステンシル)はPhase 3の後続増分で拡張する。

use sim_math::{pcg, Preconditioner};

/// 1D棒の格子熱伝導ソルバ。両端(`temperature[0]`・`temperature[n-1]`)はDirichlet境界。
#[derive(Clone)]
pub struct ConductionRod1D {
    pub temperature: Vec<f64>,
    pub dx: f64,
    /// 熱拡散率 α=k/(ρc_p) [m²/s](設計§2.1)。
    pub thermal_diffusivity: f64,
}

impl ConductionRod1D {
    pub fn new(
        node_count: usize,
        length: f64,
        initial_temperature: f64,
        thermal_diffusivity: f64,
    ) -> Self {
        ConductionRod1D {
            temperature: vec![initial_temperature; node_count],
            dx: length / (node_count - 1) as f64,
            thermal_diffusivity,
        }
    }

    pub fn set_boundary_temperatures(&mut self, left: f64, right: f64) {
        let n = self.temperature.len();
        self.temperature[0] = left;
        self.temperature[n - 1] = right;
    }

    /// 陰的Euler(設計§4.3「線形項は陰的Euler」の1D 3点ステンシル版)を matrix-free PCGで
    /// 解く。境界の既知温度は行列(内部点のみのSPD系)から右辺への定数項として移す
    /// (標準的なDirichlet境界の扱い)。
    pub fn step(&mut self, dt: f64) {
        let n = self.temperature.len();
        if n < 3 {
            return;
        }
        let interior = n - 2;
        let r = self.thermal_diffusivity * dt / (self.dx * self.dx);

        let boundary_left = self.temperature[0];
        let boundary_right = self.temperature[n - 1];
        let t_old: Vec<f64> = self.temperature[1..n - 1].to_vec();

        let apply_a = |x: &[f64], out: &mut [f64]| {
            for i in 0..interior {
                let mut val = (1.0 + 2.0 * r) * x[i];
                if i > 0 {
                    val -= r * x[i - 1];
                }
                if i < interior - 1 {
                    val -= r * x[i + 1];
                }
                out[i] = val;
            }
        };

        let mut b = t_old.clone();
        b[0] += r * boundary_left;
        b[interior - 1] += r * boundary_right;

        let mut x = t_old;
        let result = pcg(apply_a, &b, &mut x, &Preconditioner::None, 1e-12, 500);
        debug_assert!(
            result.converged,
            "lattice conduction PCG did not converge: {result:?}"
        );

        self.temperature[1..n - 1].copy_from_slice(&x);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// フーリエ級数解: 両端0・初期一様T0の1D棒の過渡温度分布(docs/12-thermal/02-heat-transfer.md §7)。
    /// $T(x,t)=\sum_{n\,\text{odd}} \frac{4T_0}{n\pi}\sin(n\pi x/L)e^{-n^2\pi^2\alpha t/L^2}$。
    fn fourier_series_solution(
        x: f64,
        t: f64,
        length: f64,
        t0: f64,
        alpha: f64,
        terms: u32,
    ) -> f64 {
        let mut sum = 0.0;
        for k in 0..terms {
            let n = (2 * k + 1) as f64;
            let amplitude = 4.0 * t0 / (n * std::f64::consts::PI);
            let spatial = (n * std::f64::consts::PI * x / length).sin();
            let decay = (-n * n * std::f64::consts::PI * std::f64::consts::PI * alpha * t
                / (length * length))
                .exp();
            sum += amplitude * spatial * decay;
        }
        sum
    }

    /// T3: 1D棒の過渡伝導 — フーリエ級数解とrel<2%(h)(docs/21-verification/01-analytic-tests.md T3)。
    #[test]
    fn t3_1d_rod_transient_conduction_matches_fourier_series_solution() {
        let length = 1.0;
        let t0 = 100.0;
        let alpha = 1e-4;
        let node_count = 41;
        let mut rod = ConductionRod1D::new(node_count, length, t0, alpha);
        rod.set_boundary_temperatures(0.0, 0.0);

        let dt = 1.0;
        let total_time = 300.0;
        let steps = (total_time / dt) as u32;
        for _ in 0..steps {
            rod.step(dt);
        }

        // 境界に近すぎる点は解析解の絶対値が小さく相対誤差が発散しやすいため、
        // 中央寄りの複数点(境界の影響が支配的でない範囲)で比較する。
        for &i in &[10usize, 15, 20, 25, 30] {
            let x = i as f64 * rod.dx;
            let analytic = fourier_series_solution(x, total_time, length, t0, alpha, 50);
            let measured = rod.temperature[i];
            let rel_err = (measured - analytic).abs() / analytic;
            assert!(
                rel_err < 0.02,
                "i={i} x={x:.4} measured={measured:.4} analytic={analytic:.4} rel_err={rel_err:.4}"
            );
        }
    }
}
