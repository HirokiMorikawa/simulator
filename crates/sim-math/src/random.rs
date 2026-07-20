//! 決定論的 PRNG と分布サンプリング。設計: docs/01-math/04-random.md。
//!
//! PCG-XSH-RR 64/32(O'Neill 2014)。`rand` crate・OS エントロピーはコアで禁止
//! (docs/20-integration/02-determinism-replay.md §2)であり、コア全体の乱数は
//! この `SimRng` のみを経由する。

use crate::Vec3;

const PCG_MULTIPLIER: u64 = 6364136223846793005;

/// PCG32 状態(128bit: state 64bit + inc 64bit)。周期 2^64、ストリーム分割可。
/// `normal_carry` は Box-Muller の 2 値目のキャリー(§4: 決定論のため状態に含める)。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimRng {
    state: u64,
    inc: u64,
    normal_carry: Option<f64>,
}

impl SimRng {
    /// `seed`(initstate)と `stream`(initseq)から独立系列を導出する。
    /// ドメインごと・粒子ごとに異なる `stream` を渡すことで、単一シードから
    /// 決定的に独立乱数列を分割できる(§2 のストリーム配分規約)。
    pub fn new(seed: u64, stream: u64) -> SimRng {
        let mut rng = SimRng {
            state: 0,
            inc: (stream << 1) | 1,
            normal_carry: None,
        };
        rng.step_u32();
        rng.state = rng.state.wrapping_add(seed);
        rng.step_u32();
        rng
    }

    /// PCG-XSH-RR の 1 ステップ(状態更新 + 出力置換)。
    fn step_u32(&mut self) -> u32 {
        let oldstate = self.state;
        self.state = oldstate.wrapping_mul(PCG_MULTIPLIER).wrapping_add(self.inc);
        let xorshifted = (((oldstate >> 18) ^ oldstate) >> 27) as u32;
        let rot = (oldstate >> 59) as u32;
        xorshifted.rotate_right(rot)
    }

    pub fn next_u32(&mut self) -> u32 {
        self.step_u32()
    }

    fn next_u64(&mut self) -> u64 {
        let hi = self.next_u32() as u64;
        let lo = self.next_u32() as u64;
        (hi << 32) | lo
    }

    /// \[0,1)。53bit 精度: (next_u64 >> 11) * 2^-53。
    pub fn next_f64(&mut self) -> f64 {
        const SCALE: f64 = 1.0 / (1u64 << 53) as f64;
        (self.next_u64() >> 11) as f64 * SCALE
    }

    pub fn range_f64(&mut self, lo: f64, hi: f64) -> f64 {
        lo + self.next_f64() * (hi - lo)
    }

    /// \[0, n)。棄却法でモジュロバイアスを除去する(`pcg32_boundedrand_r` 相当)。
    pub fn range_u32(&mut self, n: u32) -> u32 {
        assert!(n > 0, "range_u32: bound must be positive");
        let threshold = n.wrapping_neg() % n;
        loop {
            let r = self.next_u32();
            if r >= threshold {
                return r % n;
            }
        }
    }

    /// 標準正規分布 N(0,1)。Box-Muller(対数版)。2 値生成のため 2 回に 1 回はキャッシュを返す。
    pub fn normal(&mut self) -> f64 {
        if let Some(cached) = self.normal_carry.take() {
            return cached;
        }
        // u1 は (0,1](0 だと ln が発散するため next_f64 の [0,1) を反転させて避ける)。
        let u1 = 1.0 - self.next_f64();
        let u2 = self.next_f64();
        let radius = (-2.0 * u1.ln()).sqrt();
        let theta = 2.0 * std::f64::consts::PI * u2;
        self.normal_carry = Some(radius * theta.sin());
        radius * theta.cos()
    }

    /// マクスウェル=ボルツマン速度の1粒子分: 各成分独立に N(0, sigma)。
    /// sigma = sqrt(kB T / m) は呼び出し側(統計力学ドメイン)が物理定数から計算する。
    pub fn maxwell_boltzmann_velocity(&mut self, sigma: f64) -> Vec3 {
        Vec3::new(
            self.normal() * sigma,
            self.normal() * sigma,
            self.normal() * sigma,
        )
    }

    /// 単位球面上の一様分布。Marsaglia 法(棄却)。
    pub fn unit_sphere(&mut self) -> Vec3 {
        loop {
            let x = self.range_f64(-1.0, 1.0);
            let y = self.range_f64(-1.0, 1.0);
            let s = x * x + y * y;
            if s < 1.0 {
                let factor = 2.0 * (1.0 - s).sqrt();
                return Vec3::new(x * factor, y * factor, 1.0 - 2.0 * s);
            }
        }
    }

    /// 指数分布(逆関数法): -ln(1-u)/lambda。
    pub fn exponential(&mut self, rate: f64) -> f64 {
        -(1.0 - self.next_f64()).ln() / rate
    }

    /// 離散分布(重み `weights` に比例)。累積和 + 二分探索でインデックスを選ぶ。
    pub fn discrete(&mut self, weights: &[f64]) -> usize {
        assert!(!weights.is_empty(), "discrete: weights must not be empty");
        let mut prefix = Vec::with_capacity(weights.len());
        let mut acc = 0.0;
        for w in weights {
            acc += w;
            prefix.push(acc);
        }
        let target = self.range_f64(0.0, acc);
        let idx = prefix.partition_point(|&cumulative| cumulative <= target);
        idx.min(weights.len() - 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// docs/01-math/04-random.md §5「PCG 公式テストベクタとの一致」。
    /// pcg-c-basic の `pcg32-demo`(seed=42, seq=54)の公式出力
    /// (0xa15c02b7, 0x7b47f409, 0xba1d3330, 0x83d2f293, 0xbfa4784b, 0xcbed606e)。
    #[test]
    fn matches_official_pcg32_reference_vector() {
        let mut rng = SimRng::new(42, 54);
        let expected: [u32; 6] = [
            0xa15c02b7, 0x7b47f409, 0xba1d3330, 0x83d2f293, 0xbfa4784b, 0xcbed606e,
        ];
        for e in expected {
            assert_eq!(rng.next_u32(), e);
        }
    }

    #[test]
    fn same_seed_and_stream_reproduces_sequence() {
        let mut a = SimRng::new(7, 3);
        let mut b = SimRng::new(7, 3);
        for _ in 0..1000 {
            assert_eq!(a.next_u32(), b.next_u32());
        }
    }

    #[test]
    fn different_streams_diverge() {
        let mut a = SimRng::new(7, 1);
        let mut b = SimRng::new(7, 2);
        let seq_a: Vec<u32> = (0..16).map(|_| a.next_u32()).collect();
        let seq_b: Vec<u32> = (0..16).map(|_| b.next_u32()).collect();
        assert_ne!(seq_a, seq_b);
    }

    #[test]
    fn next_f64_is_within_unit_interval() {
        let mut rng = SimRng::new(1, 1);
        for _ in 0..10_000 {
            let v = rng.next_f64();
            assert!((0.0..1.0).contains(&v));
        }
    }

    #[test]
    fn range_u32_is_within_bound_and_covers_range() {
        let mut rng = SimRng::new(99, 5);
        let mut seen = [false; 6];
        for _ in 0..10_000 {
            let v = rng.range_u32(6);
            assert!(v < 6);
            seen[v as usize] = true;
        }
        assert!(seen.iter().all(|&s| s));
    }

    /// §5 統計テスト(軽量版): 一様性のカイ二乗適合。
    /// 10 分位・自由度 9・有意水準 1% の棄却域 21.67 に対し、十分な余裕を確認する
    /// (固定シードのため決定論的に同一の統計量が得られる)。
    #[test]
    fn next_f64_passes_chi_square_uniformity() {
        let mut rng = SimRng::new(2024, 11);
        const BINS: usize = 10;
        const N: usize = 100_000;
        let mut counts = [0u32; BINS];
        for _ in 0..N {
            let v = rng.next_f64();
            let bin = ((v * BINS as f64) as usize).min(BINS - 1);
            counts[bin] += 1;
        }
        let expected = N as f64 / BINS as f64;
        let chi_sq: f64 = counts
            .iter()
            .map(|&c| (c as f64 - expected).powi(2) / expected)
            .sum();
        // df=9, alpha=1% の棄却域は 21.67。乱数実装バグ検出には十分な倍のマージンを取る。
        assert!(chi_sq < 30.0, "chi-square statistic too high: {chi_sq}");
    }

    /// §5 統計テスト(軽量版): 正規サンプルのモーメント。
    /// 10^6 点で平均 < 3σ/√N、分散相対誤差 < 1%。
    #[test]
    fn normal_sample_moments_match_standard_normal() {
        let mut rng = SimRng::new(555, 21);
        const N: usize = 1_000_000;
        let mut sum = 0.0;
        let mut sum_sq = 0.0;
        for _ in 0..N {
            let v = rng.normal();
            sum += v;
            sum_sq += v * v;
        }
        let mean = sum / N as f64;
        let variance = sum_sq / N as f64 - mean * mean;
        assert!(mean.abs() < 3.0 / (N as f64).sqrt());
        assert!((variance - 1.0).abs() < 0.01);
    }

    #[test]
    fn unit_sphere_samples_are_unit_length() {
        let mut rng = SimRng::new(3, 3);
        for _ in 0..10_000 {
            let v = rng.unit_sphere();
            assert!((v.length() - 1.0).abs() < 1e-9);
        }
    }

    #[test]
    fn exponential_samples_are_nonnegative_with_expected_mean() {
        let mut rng = SimRng::new(4, 4);
        const N: usize = 200_000;
        let rate = 2.0;
        let mut sum = 0.0;
        for _ in 0..N {
            let v = rng.exponential(rate);
            assert!(v >= 0.0);
            sum += v;
        }
        let mean = sum / N as f64;
        // 期待値 1/rate = 0.5。標準誤差 (1/rate)/sqrt(N) の十分な倍数を許容。
        assert!((mean - 1.0 / rate).abs() < 0.01);
    }

    #[test]
    fn discrete_respects_weights() {
        let mut rng = SimRng::new(6, 6);
        let weights = [1.0, 0.0, 3.0];
        let mut counts = [0u32; 3];
        const N: u32 = 10_000;
        for _ in 0..N {
            counts[rng.discrete(&weights)] += 1;
        }
        assert_eq!(counts[1], 0, "zero-weight bucket must never be selected");
        // 重み比 1:3 の許容誤差(二項分布の標準誤差の十分な倍数)。
        let ratio = counts[2] as f64 / counts[0] as f64;
        assert!((ratio - 3.0).abs() < 0.5, "ratio was {ratio}");
    }
}
