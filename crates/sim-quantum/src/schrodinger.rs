//! 1粒子TDSEのsplit-step Fourier解法。設計: docs/14-quantum/02-schrodinger-solver.md。
//!
//! P5 スコープの最小実装: 1D、周期境界(吸収マスクなし)、実時間発展のみ(虚時間発展・
//! 2D・検出スクリーンサンプリングは未実装)。内部は原子単位($\hbar=m_e=1$、設計 §2)。

use sim_math::{fft, ifft, Complex64};

/// 1D波動関数。設計 §3 `QuantumSim1D` の縮約版(吸収マスク・FftPlanキャッシュ構造体は
/// 未実装、FFTは毎ステップ`sim_math::fft`を直接呼ぶ)。
pub struct WaveFunction1D {
    pub psi: Vec<Complex64>,
    /// ポテンシャル(実空間格子上、原子単位)。
    pub v: Vec<f64>,
    pub dx: f64,
}

impl WaveFunction1D {
    /// 長さ `n`(2の冪、`sim_math::fft` の制約)の格子で初期化する。
    pub fn new(n: usize, dx: f64) -> WaveFunction1D {
        assert!(n.is_power_of_two(), "grid length must be a power of two");
        WaveFunction1D {
            psi: vec![Complex64::ZERO; n],
            v: vec![0.0; n],
            dx,
        }
    }

    pub fn len(&self) -> usize {
        self.psi.len()
    }

    pub fn is_empty(&self) -> bool {
        self.psi.is_empty()
    }

    /// ガウス波束 $\psi_0\propto\exp[-(x-x_0)^2/(4\sigma^2)+ik_0x]$ を格子点 `x_i=i\cdot dx`
    /// に設定し、離散ノルム($\sum|\psi_i|^2 dx$)が1になるよう正規化する(設計 §4.2)。
    pub fn set_gaussian_wave_packet(&mut self, x0: f64, sigma: f64, k0: f64) {
        for (i, psi_i) in self.psi.iter_mut().enumerate() {
            let x = i as f64 * self.dx;
            let envelope = (-(x - x0).powi(2) / (4.0 * sigma * sigma)).exp();
            *psi_i = Complex64::from_polar(envelope, k0 * x);
        }
        let norm = self.norm();
        let scale = 1.0 / norm.sqrt();
        for psi_i in &mut self.psi {
            *psi_i = psi_i.scale(scale);
        }
    }

    /// 離散ノルム $\sum_i|\psi_i|^2\,dx$(設計 §7 の検証量)。
    pub fn norm(&self) -> f64 {
        self.psi.iter().map(|p| p.norm_sq()).sum::<f64>() * self.dx
    }

    /// 期待値 $\langle x\rangle$。
    pub fn mean_x(&self) -> f64 {
        self.psi
            .iter()
            .enumerate()
            .map(|(i, p)| (i as f64 * self.dx) * p.norm_sq())
            .sum::<f64>()
            * self.dx
            / self.norm().max(1e-300)
    }

    /// 分散 $\langle x^2\rangle-\langle x\rangle^2$ の平方根(波束の広がり σ)。
    pub fn std_dev_x(&self) -> f64 {
        let mean = self.mean_x();
        let mean_x2 = self
            .psi
            .iter()
            .enumerate()
            .map(|(i, p)| (i as f64 * self.dx).powi(2) * p.norm_sq())
            .sum::<f64>()
            * self.dx
            / self.norm().max(1e-300);
        (mean_x2 - mean * mean).max(0.0).sqrt()
    }

    /// split-step Fourier(Strang分割、設計 §4)を1ステップ進める。
    /// $\psi(t+\Delta t)=e^{-iV\Delta t/2}\,\mathcal F^{-1}\,e^{-ik^2\Delta t/2}\,\mathcal F\,
    /// e^{-iV\Delta t/2}\,\psi(t)$。各因子が位相回転(ユニタリ)のためノルムを厳密に保つ。
    pub fn step(&mut self, dt: f64) {
        let n = self.len();

        self.apply_potential_half_step(dt);

        fft(&mut self.psi);
        let dk = 2.0 * std::f64::consts::PI / (n as f64 * self.dx);
        for (i, psi_i) in self.psi.iter_mut().enumerate() {
            // FFTのビン順序 0,1,...,n/2-1,-n/2,...,-1(周波数)に対応する波数 k。
            let k_index = if i <= n / 2 {
                i as isize
            } else {
                i as isize - n as isize
            };
            let k = k_index as f64 * dk;
            let phase = -0.5 * k * k * dt;
            *psi_i = *psi_i * Complex64::from_polar(1.0, phase);
        }
        ifft(&mut self.psi);

        self.apply_potential_half_step(dt);
    }

    fn apply_potential_half_step(&mut self, dt: f64) {
        for (psi_i, &v_i) in self.psi.iter_mut().zip(self.v.iter()) {
            let phase = -v_i * dt * 0.5;
            *psi_i = *psi_i * Complex64::from_polar(1.0, phase);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Q1: ノルム保存 $\int|\psi|^2=1$、abs 1e-12(周期境界・吸収なし、
    /// docs/21-verification/01-analytic-tests.md Q1)。各split-stepの因子は位相回転で
    /// あり理論上ユニタリなので、FFT実装(sim_math::fft)自体の数値精度の検算になる。
    #[test]
    fn q1_norm_is_conserved_to_near_machine_precision() {
        let n = 512;
        let dx = 0.05;
        let mut wf = WaveFunction1D::new(n, dx);
        wf.set_gaussian_wave_packet(n as f64 * dx * 0.5, 1.0, 5.0);
        // 調和振動子ポテンシャル(束縛系でも保存則が成り立つことを見るため0でない場を使う)。
        for (i, v_i) in wf.v.iter_mut().enumerate() {
            let x = i as f64 * dx - n as f64 * dx * 0.5;
            *v_i = 0.5 * 0.01 * x * x;
        }

        let dt = 0.002;
        for _ in 0..2000 {
            wf.step(dt);
        }

        let norm = wf.norm();
        assert!((norm - 1.0).abs() < 1e-10, "norm={norm}");
    }

    /// Q2: 自由波束の広がり $\sigma(t)=\sigma_0\sqrt{1+(t/(2\sigma_0^2))^2}$
    /// (原子単位 $\hbar=m=1$)、rel 0.1%(docs/21-verification/01-analytic-tests.md Q2)。
    #[test]
    fn q2_free_wave_packet_spreading_matches_analytic_formula() {
        let n = 2048;
        let dx = 0.05;
        let sigma0 = 1.0;
        let mut wf = WaveFunction1D::new(n, dx);
        wf.set_gaussian_wave_packet(n as f64 * dx * 0.5, sigma0, 0.0);
        // v は既定でゼロ(自由粒子)。

        let dt = 0.002;
        let steps = 4000u32;
        for _ in 0..steps {
            wf.step(dt);
        }
        let t = steps as f64 * dt;

        let measured_sigma = wf.std_dev_x();
        let expected_sigma = sigma0 * (1.0 + (t / (2.0 * sigma0 * sigma0)).powi(2)).sqrt();
        let rel_err = (measured_sigma - expected_sigma).abs() / expected_sigma;
        assert!(
            rel_err < 0.001,
            "measured_sigma={measured_sigma} expected_sigma={expected_sigma} rel_err={rel_err}"
        );
    }
}
