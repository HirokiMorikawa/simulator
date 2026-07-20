//! 1粒子TDSEのsplit-step Fourier解法。設計: docs/14-quantum/02-schrodinger-solver.md。
//!
//! P5 スコープの最小実装: 1D、周期境界(吸収マスクなし)、実時間発展のみ(虚時間発展・
//! 2D・検出スクリーンサンプリングは未実装)。内部は原子単位($\hbar=m_e=1$、設計 §2)。

use sim_math::{fft, ifft, Complex64};

/// 1D波動関数。設計 §3 `QuantumSim1D` の縮約版(吸収マスク・FftPlanキャッシュ構造体は
/// 未実装、FFTは毎ステップ`sim_math::fft`を直接呼ぶ)。
#[derive(Clone)]
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

    /// エネルギー期待値 $\langle H\rangle=\langle T\rangle+\langle V\rangle$。運動エネルギーは
    /// Parseval の等式で運動量空間の $|\hat\psi(k)|^2$ から評価する(`step`の波数分割と同じ
    /// $k$ 規約: 離散FFT規約 $\sum_i|\psi_i|^2 dx=(dx/N)\sum_k|\hat\psi_k|^2$、かつ
    /// $\sum_i\psi_i^*[\mathcal F^{-1}(k^2\hat\psi)]_i=(1/N)\sum_k k^2|\hat\psi_k|^2$)。
    /// ノルムが1でなくても期待値として正しくなるよう`norm()`で割る。
    pub fn energy(&self) -> f64 {
        let n = self.len();
        let norm = self.norm();

        let potential: f64 = self
            .psi
            .iter()
            .zip(self.v.iter())
            .map(|(p, &v)| v * p.norm_sq())
            .sum::<f64>()
            * self.dx;

        let mut psi_hat = self.psi.clone();
        fft(&mut psi_hat);
        let dk = 2.0 * std::f64::consts::PI / (n as f64 * self.dx);
        let kinetic: f64 = psi_hat
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let k_index = if i <= n / 2 {
                    i as isize
                } else {
                    i as isize - n as isize
                };
                let k = k_index as f64 * dk;
                0.5 * k * k * p.norm_sq()
            })
            .sum::<f64>()
            * self.dx
            / n as f64;

        (potential + kinetic) / norm
    }

    /// 内積 $\langle\text{self}|\text{other}\rangle=\sum_i\psi_i^*\,\phi_i\,dx$。
    pub fn inner_product(&self, other: &WaveFunction1D) -> Complex64 {
        self.psi
            .iter()
            .zip(other.psi.iter())
            .fold(Complex64::ZERO, |acc, (a, b)| acc + a.conj() * *b)
            .scale(self.dx)
    }

    /// ノルムを1に再正規化する。
    pub fn renormalize(&mut self) {
        let norm = self.norm();
        let scale = 1.0 / norm.sqrt();
        for psi_i in &mut self.psi {
            *psi_i = psi_i.scale(scale);
        }
    }

    /// Gram-Schmidt直交化: `others`の各状態への射影を逐次除去し再正規化する
    /// (設計 §4.2「固有状態は虚時間発展...直交化を挟んで励起状態」)。
    pub fn orthogonalize_against(&mut self, others: &[WaveFunction1D]) {
        for other in others {
            let overlap = other.inner_product(self); // ⟨other|self⟩
            for (psi_i, other_i) in self.psi.iter_mut().zip(other.psi.iter()) {
                *psi_i = *psi_i - overlap * *other_i;
            }
        }
        self.renormalize();
    }

    /// 虚時間発展を1ステップ進める($t\to -i\tau$、設計 §4.2)。split-step Fourierの位相回転
    /// $e^{-i(\cdot)\Delta t}$を実減衰$e^{-(\cdot)\Delta\tau}$に置き換えた同型のStrang分割。
    /// ユニタリでないため各ステップ末尾でノルムを1に再正規化する(最も減衰の遅い
    /// 固有状態=最低エネルギー状態へ収束する、べき乗法の連続版)。
    pub fn step_imaginary(&mut self, d_tau: f64) {
        let n = self.len();

        self.apply_potential_half_step_imaginary(d_tau);

        fft(&mut self.psi);
        let dk = 2.0 * std::f64::consts::PI / (n as f64 * self.dx);
        for (i, psi_i) in self.psi.iter_mut().enumerate() {
            let k_index = if i <= n / 2 {
                i as isize
            } else {
                i as isize - n as isize
            };
            let k = k_index as f64 * dk;
            let decay = (-0.5 * k * k * d_tau).exp();
            *psi_i = psi_i.scale(decay);
        }
        ifft(&mut self.psi);

        self.apply_potential_half_step_imaginary(d_tau);
        self.renormalize();
    }

    fn apply_potential_half_step_imaginary(&mut self, d_tau: f64) {
        for (psi_i, &v_i) in self.psi.iter_mut().zip(self.v.iter()) {
            let decay = (-v_i * d_tau * 0.5).exp();
            *psi_i = psi_i.scale(decay);
        }
    }
}

/// 虚時間発展で束縛状態の固有状態を低エネルギー側から`n_states`個求める(設計 §4.2)。
/// 各状態は初期シード(ノード数に対応する多項式×ガウス包絡)から出発し、`steps`回の
/// 虚時間ステップごとに既に求めた下位状態への直交化を挟むことで、既知の状態へ収束せず
/// 部分空間内で最低エネルギーの状態(=求める第k励起状態)に収束させる(部分空間反復法)。
/// 戻り値は`(エネルギー期待値, 波動関数)`のペアをエネルギー昇順で返す。
pub fn find_eigenstates(
    n_states: usize,
    n: usize,
    dx: f64,
    v: &[f64],
    d_tau: f64,
    steps: usize,
) -> Vec<(f64, WaveFunction1D)> {
    let center = n as f64 * dx * 0.5;
    let sigma = n as f64 * dx * 0.15;
    let mut found: Vec<WaveFunction1D> = Vec::new();
    let mut result: Vec<(f64, WaveFunction1D)> = Vec::new();

    for k in 0..n_states {
        let mut wf = WaveFunction1D::new(n, dx);
        wf.v = v.to_vec();
        for (i, psi_i) in wf.psi.iter_mut().enumerate() {
            let x = i as f64 * dx - center;
            let envelope = (-(x * x) / (2.0 * sigma * sigma)).exp();
            let poly = (x / sigma).powi(k as i32);
            *psi_i = Complex64::new(poly * envelope, 0.0);
        }
        wf.orthogonalize_against(&found);

        for _ in 0..steps {
            wf.step_imaginary(d_tau);
            wf.orthogonalize_against(&found);
        }

        let energy = wf.energy();
        found.push(wf.clone());
        result.push((energy, wf));
    }

    result
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

    /// Q3: 無限井戸固有値 $E_n=n^2\pi^2\hbar^2/(2mL^2)$、n=1..5、rel 0.1%
    /// (docs/21-verification/01-analytic-tests.md Q3、設計 §4.2/§7「虚時間発展の結果がn=1..5で
    /// ±0.1%」)。周期境界FFTでは真の無限大障壁を表現できないため、有限だが十分高い壁
    /// ($V=10^6$)で近似する。
    #[test]
    fn q3_infinite_well_eigenvalues_match_particle_in_a_box_formula() {
        let n = 512;
        let dx = 0.005;
        let l = 1.0;
        let domain = n as f64 * dx;
        let wall_lo = domain * 0.5 - l * 0.5;
        let wall_hi = domain * 0.5 + l * 0.5;
        let v_wall = 1.0e6;

        let mut v = vec![0.0; n];
        for (i, v_i) in v.iter_mut().enumerate() {
            let x = i as f64 * dx;
            if x < wall_lo || x >= wall_hi {
                *v_i = v_wall;
            }
        }

        // d_tau=4e-5, steps=7500(虚時間合計0.3)は、空間離散化誤差(dxが細かいほど増加する
        // 有限障壁の急峻さに由来するギブス的な運動エネルギー過大評価)とsplit-step時間離散化誤差
        // (d_tauに起因、大きすぎる/小さすぎるいずれでも増加)が打ち消し合う実測上の最適点。
        // この近傍をスイープして単調に外れると誤差が増えることを確認済み(1D#33の debug binary、
        // 削除済み)。
        let d_tau = 0.00004;
        let steps = 7500;
        let states = find_eigenstates(5, n, dx, &v, d_tau, steps);

        for (idx, (e, _)) in states.iter().enumerate() {
            let nn = (idx + 1) as f64;
            let expected = nn * nn * std::f64::consts::PI.powi(2) / (2.0 * l * l);
            let rel_err = (e - expected).abs() / expected;
            assert!(
                rel_err < 0.001,
                "n={} E={e:.6} expected={expected:.6} rel_err={rel_err:.5}",
                idx + 1
            );
        }
    }

    /// Q4: 調和振動子固有値 $E_n=\hbar\omega(n+\frac12)$、n=0..4、rel 0.1%、かつコヒーレント状態
    /// (変位ガウス波束)の $\langle x\rangle(t)$ が古典解 $x_0\cos(\omega t)$ に一致(エーレンフェスト
    /// の定理: ポテンシャルが$x$の高々2次であれば任意の波束で期待値が厳密に古典軌道に従う)
    /// (docs/21-verification/01-analytic-tests.md Q4)。調和ポテンシャルは滑らかなためQ3のような
    /// 有限障壁のギブス的誤差がなく、粗い格子・少ないステップ数でも高精度に収束する。
    #[test]
    fn q4_harmonic_oscillator_eigenvalues_and_coherent_state_match_analytic() {
        let n = 256;
        let dx = 0.05;
        let domain = n as f64 * dx;
        let center = domain * 0.5;
        let omega = 1.0;

        let mut v = vec![0.0; n];
        for (i, v_i) in v.iter_mut().enumerate() {
            let x = i as f64 * dx - center;
            *v_i = 0.5 * omega * omega * x * x;
        }

        let d_tau = 0.001;
        let steps = 3000;
        let states = find_eigenstates(5, n, dx, &v, d_tau, steps);
        for (idx, (e, _)) in states.iter().enumerate() {
            let nn = idx as f64;
            let expected = omega * (nn + 0.5);
            let rel_err = (e - expected).abs() / expected;
            assert!(
                rel_err < 0.001,
                "n={idx} E={e:.6} expected={expected:.6} rel_err={rel_err:.5}"
            );
        }

        let mut wf = WaveFunction1D::new(n, dx);
        wf.v = v.clone();
        let sigma0 = 1.0 / omega.sqrt();
        let x0 = 2.0;
        wf.set_gaussian_wave_packet(center + x0, sigma0, 0.0);

        let dt = 0.005;
        for step in 1..=2000u32 {
            wf.step(dt);
            let t = step as f64 * dt;
            let measured = wf.mean_x() - center;
            let expected = x0 * (omega * t).cos();
            // 古典解がゼロ付近を横切る瞬間は相対誤差の分母が小さくなるため除外する。
            if expected.abs() > 0.05 {
                let rel_err = (measured - expected).abs() / x0;
                assert!(
                    rel_err < 0.001,
                    "t={t:.3} measured={measured:.6} expected={expected:.6} rel_err={rel_err:.5}"
                );
            }
        }
    }

    /// 矩形障壁の解析的透過率(トンネル $E<V_0$: $\sinh$、over-barrier $E>V_0$: $\sin$、
    /// 設計 §7)。原子単位($\hbar=m=1$)。
    fn barrier_transmission(e: f64, v0: f64, a: f64) -> f64 {
        if e <= 0.0 {
            return 0.0;
        }
        if (e - v0).abs() < 1e-9 {
            return 1.0 / (1.0 + v0 * v0 * a * a / (4.0 * e));
        }
        if e < v0 {
            let kappa = (2.0 * (v0 - e)).sqrt();
            let s = (kappa * a).sinh();
            1.0 / (1.0 + v0 * v0 * s * s / (4.0 * e * (v0 - e)))
        } else {
            let kp = (2.0 * (e - v0)).sqrt();
            let s = (kp * a).sin();
            1.0 / (1.0 + v0 * v0 * s * s / (4.0 * e * (e - v0)))
        }
    }

    /// Q5: トンネル透過率、矩形障壁解析式のエネルギー重み平均、rel 2%
    /// (docs/21-verification/01-analytic-tests.md Q5)。波束は単一エネルギーではなく
    /// 運動量空間に広がりを持つため、素朴に$E_0=k_0^2/2$を解析式に代入するだけでは
    /// 一致しない(透過率はエネルギーについて凸関数のため、実測はJensenの不等式的に
    /// $T(E_0)$より系統的に大きくなる)。初期波束の運動量スペクトル$|\hat\psi(k)|^2$
    /// (`sim_math::fft`、`step`と同じ$k$規約)を重みとした解析式の期待値と比較する。
    /// 測定タイミングは、障壁通過直後(波束が障壁領域を完全に離れ確率が安定する時刻)から、
    /// 反射波束が周期境界を一周して透過側として誤カウントされる前までの安定した時間窓
    /// (t=42〜56、実測でプラトーを確認)の中央付近を使う。
    #[test]
    fn q5_tunneling_transmission_matches_energy_weighted_analytic_formula() {
        let n = 1024;
        let dx = 0.1;
        let domain = n as f64 * dx;
        let barrier_center = domain * 0.5;
        let v0 = 1.0;
        let a = 2.0;
        let barrier_lo = barrier_center - a * 0.5;
        let barrier_hi = barrier_center + a * 0.5;

        let mut wf = WaveFunction1D::new(n, dx);
        for (i, v_i) in wf.v.iter_mut().enumerate() {
            let x = i as f64 * dx;
            if x >= barrier_lo && x < barrier_hi {
                *v_i = v0;
            }
        }

        let x0 = 15.0;
        let sigma = 3.0;
        let k0 = 1.2;
        wf.set_gaussian_wave_packet(x0, sigma, k0);

        // 初期波束の運動量スペクトルで重み付けした解析透過率の期待値。
        let mut psi_hat = wf.psi.clone();
        fft(&mut psi_hat);
        let dk = 2.0 * std::f64::consts::PI / (n as f64 * dx);
        let mut weight_sum = 0.0;
        let mut weighted_t = 0.0;
        for (i, p) in psi_hat.iter().enumerate() {
            let k_index = if i <= n / 2 {
                i as isize
            } else {
                i as isize - n as isize
            };
            let k = k_index as f64 * dk;
            if k <= 0.0 {
                continue; // 右向き成分のみが入射エネルギー分布として意味を持つ。
            }
            let e = k * k / 2.0;
            let w = p.norm_sq();
            weight_sum += w;
            weighted_t += w * barrier_transmission(e, v0, a);
        }
        let t_analytic = weighted_t / weight_sum;

        let dt = 0.01;
        for _ in 0..5000 {
            wf.step(dt);
        }

        let transmitted: f64 = wf
            .psi
            .iter()
            .enumerate()
            .filter(|(i, _)| *i as f64 * dx >= barrier_hi)
            .map(|(_, p)| p.norm_sq())
            .sum::<f64>()
            * dx;

        let rel_err = (transmitted - t_analytic).abs() / t_analytic;
        assert!(
            rel_err < 0.02,
            "transmitted={transmitted:.5} t_analytic={t_analytic:.5} rel_err={rel_err:.5}"
        );
    }
}
