//! 2D split-step Fourier(二重スリットの本命)。設計: docs/14-quantum/02-schrodinger-solver.md
//! §4/§8「2D(二重スリットの本命)」。
//!
//! 1D版(`schrodinger.rs`)と同じStrang分割を2次元に拡張する。2D FFTは自前実装せず、
//! 既存の1D `sim_math::fft`/`ifft` を各行→各列に適用する分離可能な標準手法で構成する
//! (設計は自前radix-2 FFTを「量子ドメイン共通の基盤」と位置づけており、2D固有の
//! 実装を新たに追加する必要はない)。虚時間発展・吸収境界・検出スクリーンの
//! 決定論的サンプリングは1D版と同様未実装。

use sim_math::{fft, ifft, Complex64};

/// 2D波動関数。行優先(`index = iy*nx+ix`)。
pub struct WaveFunction2D {
    pub psi: Vec<Complex64>,
    pub v: Vec<f64>,
    pub nx: usize,
    pub ny: usize,
    pub dx: f64,
    pub dy: f64,
}

fn index(nx: usize, ix: usize, iy: usize) -> usize {
    iy * nx + ix
}

/// 各行→各列に1D FFT/IFFTを適用する分離可能な2D FFT。`ifft`は呼び出しごとに1/長さで
/// 正規化するため、行(1/nx)→列(1/ny)の順で両方呼べば全体で1/(nx・ny)の正規化になる。
fn fft_2d(data: &mut [Complex64], nx: usize, ny: usize, inverse: bool) {
    for iy in 0..ny {
        let row = &mut data[iy * nx..(iy + 1) * nx];
        if inverse {
            ifft(row);
        } else {
            fft(row);
        }
    }
    let mut col = vec![Complex64::ZERO; ny];
    for ix in 0..nx {
        for (iy, slot) in col.iter_mut().enumerate() {
            *slot = data[index(nx, ix, iy)];
        }
        if inverse {
            ifft(&mut col);
        } else {
            fft(&mut col);
        }
        for (iy, &val) in col.iter().enumerate() {
            data[index(nx, ix, iy)] = val;
        }
    }
}

impl WaveFunction2D {
    /// `nx`・`ny` は共に2の冪(`sim_math::fft`の制約)。
    pub fn new(nx: usize, ny: usize, dx: f64, dy: f64) -> WaveFunction2D {
        assert!(nx.is_power_of_two() && ny.is_power_of_two());
        WaveFunction2D {
            psi: vec![Complex64::ZERO; nx * ny],
            v: vec![0.0; nx * ny],
            nx,
            ny,
            dx,
            dy,
        }
    }

    fn idx(&self, ix: usize, iy: usize) -> usize {
        index(self.nx, ix, iy)
    }

    /// ガウス波束 $\psi_0\propto\exp[-(x-x_0)^2/(4\sigma_x^2)-(y-y_0)^2/(4\sigma_y^2)+ik_0x]$
    /// (設計§4.2の1D版を2Dへ拡張、+x方向へ運動量$k_0$)。離散ノルムが1になるよう正規化する。
    pub fn set_gaussian_wave_packet(
        &mut self,
        x0: f64,
        y0: f64,
        sigma_x: f64,
        sigma_y: f64,
        k0: f64,
    ) {
        for iy in 0..self.ny {
            for ix in 0..self.nx {
                let x = ix as f64 * self.dx;
                let y = iy as f64 * self.dy;
                let envelope = (-(x - x0).powi(2) / (4.0 * sigma_x * sigma_x)
                    - (y - y0).powi(2) / (4.0 * sigma_y * sigma_y))
                    .exp();
                let idx = self.idx(ix, iy);
                self.psi[idx] = Complex64::from_polar(envelope, k0 * x);
            }
        }
        let norm = self.norm();
        let scale = 1.0 / norm.sqrt();
        for p in &mut self.psi {
            *p = p.scale(scale);
        }
    }

    pub fn norm(&self) -> f64 {
        self.psi.iter().map(|p| p.norm_sq()).sum::<f64>() * self.dx * self.dy
    }

    /// split-step Fourier(1D版と同じStrang分割、2次元運動量 $k_x^2+k_y^2$)を1ステップ進める。
    pub fn step(&mut self, dt: f64) {
        self.apply_potential_half_step(dt);

        fft_2d(&mut self.psi, self.nx, self.ny, false);
        let dkx = 2.0 * std::f64::consts::PI / (self.nx as f64 * self.dx);
        let dky = 2.0 * std::f64::consts::PI / (self.ny as f64 * self.dy);
        for iy in 0..self.ny {
            let ky_index = if iy <= self.ny / 2 {
                iy as isize
            } else {
                iy as isize - self.ny as isize
            };
            let ky = ky_index as f64 * dky;
            for ix in 0..self.nx {
                let kx_index = if ix <= self.nx / 2 {
                    ix as isize
                } else {
                    ix as isize - self.nx as isize
                };
                let kx = kx_index as f64 * dkx;
                let phase = -0.5 * (kx * kx + ky * ky) * dt;
                let idx = self.idx(ix, iy);
                self.psi[idx] = self.psi[idx] * Complex64::from_polar(1.0, phase);
            }
        }
        fft_2d(&mut self.psi, self.nx, self.ny, true);

        self.apply_potential_half_step(dt);
    }

    fn apply_potential_half_step(&mut self, dt: f64) {
        for (p, &v) in self.psi.iter_mut().zip(self.v.iter()) {
            let phase = -v * dt * 0.5;
            *p = *p * Complex64::from_polar(1.0, phase);
        }
    }

    /// x座標 `x` に最も近い格子列(`ix`一定)での確率密度 $|\psi(x,y)|^2$ を`y`について返す。
    pub fn density_column_near_x(&self, x: f64) -> Vec<f64> {
        let ix = ((x / self.dx).round() as usize).min(self.nx - 1);
        (0..self.ny)
            .map(|iy| self.psi[self.idx(ix, iy)].norm_sq())
            .collect()
    }

    /// x座標 `x` に最も近い格子列(`ix`一定)での波動関数 $\psi(x,y)$ を`y`について返す
    /// (Fraunhofer遠方界をFFTで求めるために振幅・位相が必要、設計§7の二重スリット検証)。
    pub fn psi_column_near_x(&self, x: f64) -> Vec<Complex64> {
        let ix = ((x / self.dx).round() as usize).min(self.nx - 1);
        (0..self.ny).map(|iy| self.psi[self.idx(ix, iy)]).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 2D版のノルム保存(1D版のQ1と同じ検証量、設計§7)。調和振動子ポテンシャル下で
    /// 2D splt-step Fourier(行→列の分離可能2D FFT)自体の正しさを確認する。
    #[test]
    fn norm_is_conserved_under_2d_harmonic_potential() {
        let n = 64;
        let dx = 0.1;
        let mut wf = WaveFunction2D::new(n, n, dx, dx);
        let center = n as f64 * dx * 0.5;
        for iy in 0..n {
            for ix in 0..n {
                let x = ix as f64 * dx - center;
                let y = iy as f64 * dx - center;
                let idx = wf.idx(ix, iy);
                wf.v[idx] = 0.5 * 0.05 * (x * x + y * y);
            }
        }
        wf.set_gaussian_wave_packet(center, center, 1.0, 1.0, 3.0);

        let dt = 0.002;
        for _ in 0..500 {
            wf.step(dt);
        }
        let norm = wf.norm();
        assert!((norm - 1.0).abs() < 1e-8, "norm={norm}");
    }

    /// Q6: 二重スリット縞間隔 $\Delta y=\lambda_{dB}D/d$、rel 2%
    /// (docs/21-verification/01-analytic-tests.md Q6)。文字通り遠方の距離Dまで実空間で
    /// 波束を伝播させると、paraxial近似の妥当性(角度が小さい)とFraunhofer遠方界条件
    /// ($D\gg d^2/\lambda$)を同時に満たすために極めて大きい格子・長時間の伝播が必要になり
    /// 実用的でない(実装検証中に、両条件を満たせない配置ではむしろ中心が極小になる
    /// (Fresnel領域特有の)パターンが現れることを発見した)。標準的なFraunhofer回折の
    /// 手法を採用し、スリット通過直後の近接場 $\psi(x_{near},y)$ の1D FFT(既存の
    /// `sim_math::fft`を流用)が遠方界パターンそのものであることを使う
    /// ($k_y/k_0=\sin\theta$、$y_{screen}=D\sin\theta$、paraxial近似で$\tan\theta$の代わりに
    /// $\sin\theta$を使うのは設計の式そのものと同じ近似)。バリアの高さは入射波の運動エネルギー
    /// $E=k_0^2/2$より十分大きくする必要がある(実装検証中に、$V_0<E$だとバリアが実質
    /// 透明になり非スリット領域からも大きく漏れることを発見)。
    #[test]
    fn q6_double_slit_fringe_spacing_matches_de_broglie_formula() {
        let nx = 128;
        let ny = 256;
        let dx = 0.2;
        let dy = 0.8;

        let barrier_x = 10.0;
        let v0 = 100.0; // 入射波の運動エネルギー E=k0^2/2=8 より十分高い
        let slit_width = 3.0;
        let d = 12.0;
        let y_center = ny as f64 * dy * 0.5;

        let mut wf = WaveFunction2D::new(nx, ny, dx, dy);
        for iy in 0..ny {
            let y = iy as f64 * dy;
            for ix in 0..nx {
                let x = ix as f64 * dx;
                if (barrier_x - 0.5 * dx..barrier_x + 0.5 * dx).contains(&x) {
                    let in_slit1 = (y - (y_center - d / 2.0)).abs() < slit_width / 2.0;
                    let in_slit2 = (y - (y_center + d / 2.0)).abs() < slit_width / 2.0;
                    if !in_slit1 && !in_slit2 {
                        let idx = wf.idx(ix, iy);
                        wf.v[idx] = v0;
                    }
                }
            }
        }

        let k0 = 4.0;
        let x0 = 3.0;
        let sigma_x = 2.0;
        let sigma_y = 30.0;
        wf.set_gaussian_wave_packet(x0, y_center, sigma_x, sigma_y, k0);

        let dt = 0.005;
        let near_x = barrier_x + 3.0; // スリット通過直後(バリア領域は完全に離れている)
        let total_t = (near_x - x0) / k0 + 3.0;
        let steps = (total_t / dt).round() as u32;
        for _ in 0..steps {
            wf.step(dt);
        }

        let mut slice = wf.psi_column_near_x(near_x);
        fft(&mut slice);
        let dky = 2.0 * std::f64::consts::PI / (ny as f64 * dy);

        let screen_d = 100.0; // 実際に伝播させる距離ではなく角度→位置の変換係数のみに使う
        let lambda = 2.0 * std::f64::consts::PI / k0;
        let expected = lambda * screen_d / d;

        let mut profile: Vec<(f64, f64)> = Vec::new();
        for (m, val) in slice.iter().enumerate() {
            let ky_index = if m <= ny / 2 {
                m as isize
            } else {
                m as isize - ny as isize
            };
            let ky = ky_index as f64 * dky;
            let sin_theta = ky / k0;
            if sin_theta.abs() > 0.3 {
                continue; // paraxial近似が崩れる大角度成分は除外
            }
            profile.push((screen_d * sin_theta, val.norm_sq()));
        }

        // m=1縞(中心の主極大の隣、両側)を期待位置近傍の探索窓内で探す。
        let find_peak_in_window = |lo: f64, hi: f64| -> f64 {
            profile
                .iter()
                .filter(|&&(y, _)| y >= lo && y <= hi)
                .cloned()
                .fold((0.0, -1.0), |acc, x| if x.1 > acc.1 { x } else { acc })
                .0
        };
        let pos_peak = find_peak_in_window(0.7 * expected, 1.3 * expected);
        let neg_peak = find_peak_in_window(-1.3 * expected, -0.7 * expected);
        let measured_spacing = (pos_peak.abs() + neg_peak.abs()) / 2.0;

        let rel_err = (measured_spacing - expected).abs() / expected;
        assert!(
            rel_err < 0.02,
            "measured_spacing={measured_spacing} expected={expected} rel_err={rel_err}"
        );
    }
}
