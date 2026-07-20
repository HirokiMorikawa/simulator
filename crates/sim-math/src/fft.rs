//! 自前 radix-2 FFT。設計: docs/14-quantum/02-schrodinger-solver.md §3「自前radix-2 FFT
//! (依存最小化・決定論)」。split-step Fourier(量子ドメイン)の基盤。長さは2の冪のみ対応。

use crate::Complex64;

fn fft_impl(data: &mut [Complex64], inverse: bool) {
    let n = data.len();
    assert!(n.is_power_of_two(), "FFT length must be a power of two");
    if n <= 1 {
        return;
    }

    // ビット反転並べ替え。
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            data.swap(i, j);
        }
    }

    // バタフライ演算(Cooley-Tukey、反復版)。
    let mut len = 2;
    while len <= n {
        let ang = if inverse {
            2.0 * std::f64::consts::PI / len as f64
        } else {
            -2.0 * std::f64::consts::PI / len as f64
        };
        let wlen = Complex64::from_polar(1.0, ang);
        let mut i = 0;
        while i < n {
            let mut w = Complex64::new(1.0, 0.0);
            for k in 0..len / 2 {
                let u = data[i + k];
                let v = data[i + k + len / 2] * w;
                data[i + k] = u + v;
                data[i + k + len / 2] = u - v;
                w = w * wlen;
            }
            i += len;
        }
        len <<= 1;
    }
}

/// 正方向FFT(正規化なし)。
pub fn fft(data: &mut [Complex64]) {
    fft_impl(data, false);
}

/// 逆方向FFT(1/N 正規化込み、`fft` の厳密な逆演算)。
pub fn ifft(data: &mut [Complex64]) {
    fft_impl(data, true);
    let scale = 1.0 / data.len() as f64;
    for x in data.iter_mut() {
        *x = x.scale(scale);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 素朴なDFT(O(N^2))。小さいNでFFTの検算に使う。
    fn naive_dft(data: &[Complex64]) -> Vec<Complex64> {
        let n = data.len();
        (0..n)
            .map(|k| {
                let mut sum = Complex64::ZERO;
                for (t, &x) in data.iter().enumerate() {
                    let ang = -2.0 * std::f64::consts::PI * (k * t) as f64 / n as f64;
                    sum = sum + x * Complex64::from_polar(1.0, ang);
                }
                sum
            })
            .collect()
    }

    #[test]
    fn fft_matches_naive_dft_for_small_n() {
        let mut data: Vec<Complex64> = (0..8)
            .map(|i| Complex64::new((i as f64 * 0.7).sin(), (i as f64 * 0.3).cos()))
            .collect();
        let expected = naive_dft(&data);
        fft(&mut data);
        for (a, b) in data.iter().zip(expected.iter()) {
            assert!((a.re - b.re).abs() < 1e-9, "re mismatch: {a:?} vs {b:?}");
            assert!((a.im - b.im).abs() < 1e-9, "im mismatch: {a:?} vs {b:?}");
        }
    }

    #[test]
    fn ifft_undoes_fft_round_trip() {
        let original: Vec<Complex64> = (0..64)
            .map(|i| Complex64::new((i as f64 * 0.13).sin(), (i as f64 * 0.05).cos()))
            .collect();
        let mut data = original.clone();
        fft(&mut data);
        ifft(&mut data);
        for (a, b) in data.iter().zip(original.iter()) {
            assert!((a.re - b.re).abs() < 1e-9, "re mismatch: {a:?} vs {b:?}");
            assert!((a.im - b.im).abs() < 1e-9, "im mismatch: {a:?} vs {b:?}");
        }
    }
}
