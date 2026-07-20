//! 2Dイジング模型・メトロポリス法・Wolffクラスタ法。設計: docs/15-statistical/04-monte-carlo.md。
//!
//! P5 スコープの最小実装: 2D正方格子・周期境界・$h=0$(外場ゼロ、Onsager厳密解が
//! 存在する設定)。3D・非ゼロ外場でのWolff($h\ne0$は再重み付けが必要で未対応)は未実装。

use sim_math::SimRng;

/// $L\times L$ 2Dイジング模型。設計 §3 `IsingSim` の縮約版(観測量の移動平均フィールドは
/// 持たず、呼び出し側が`magnetization`/`energy`等を都度計算する)。
pub struct IsingSim {
    pub spins: Vec<i8>,
    pub l: usize,
    pub j_coupling: f64,
    pub temperature: f64,
    rng: SimRng,
}

impl IsingSim {
    /// 全スピンをPRNGでランダムに$\pm1$初期化する(高温初期状態)。
    pub fn new(l: usize, j_coupling: f64, temperature: f64, mut rng: SimRng) -> IsingSim {
        let spins = (0..l * l)
            .map(|_| if rng.next_f64() < 0.5 { 1 } else { -1 })
            .collect();
        IsingSim {
            spins,
            l,
            j_coupling,
            temperature,
            rng,
        }
    }

    fn index(&self, x: usize, y: usize) -> usize {
        y * self.l + x
    }

    fn neighbor_sum(&self, x: usize, y: usize) -> i32 {
        let l = self.l;
        let xp = (x + 1) % l;
        let xm = (x + l - 1) % l;
        let yp = (y + 1) % l;
        let ym = (y + l - 1) % l;
        self.spins[self.index(xp, y)] as i32
            + self.spins[self.index(xm, y)] as i32
            + self.spins[self.index(x, yp)] as i32
            + self.spins[self.index(x, ym)] as i32
    }

    /// メトロポリス法(設計 §2.2/§4)を1スイープ($L^2$回の反転試行、格子を順次走査)進める。
    /// $h=0$ のため $\Delta E\in\{-8J,-4J,0,4J,8J\}$ の5値のみ(設計§4のテーブル化の対象、
    /// ここでは都度`exp`を呼ぶ単純実装)。
    pub fn metropolis_sweep(&mut self) {
        for y in 0..self.l {
            for x in 0..self.l {
                let idx = self.index(x, y);
                let s = self.spins[idx] as f64;
                let nb = self.neighbor_sum(x, y) as f64;
                let delta_e = 2.0 * self.j_coupling * s * nb;
                if delta_e <= 0.0 || self.rng.next_f64() < (-delta_e / self.temperature).exp() {
                    self.spins[idx] = -self.spins[idx];
                }
            }
        }
    }

    /// Wolffクラスタ法(設計 §4、S7/S8の必須実装 — 臨界域での臨界減速を回避する)。
    /// シードスピンから同符号の隣接スピンを確率 $p=1-e^{-2J/k_BT}$ で再帰的にクラスタへ
    /// 加え(棄却なしの一括反転)、1回の呼び出しで1クラスタ分だけ更新する。
    pub fn wolff_step(&mut self) {
        let l = self.l;
        let p_add = 1.0 - (-2.0 * self.j_coupling / self.temperature).exp();

        let start = self.rng.range_u32((l * l) as u32) as usize;
        let (sx, sy) = (start % l, start / l);
        let seed_spin = self.spins[self.index(sx, sy)];

        let mut in_cluster = vec![false; l * l];
        let mut stack = vec![(sx, sy)];
        in_cluster[self.index(sx, sy)] = true;

        while let Some((x, y)) = stack.pop() {
            let neighbors = [
                ((x + 1) % l, y),
                ((x + l - 1) % l, y),
                (x, (y + 1) % l),
                (x, (y + l - 1) % l),
            ];
            for (nx, ny) in neighbors {
                let nidx = self.index(nx, ny);
                if !in_cluster[nidx] && self.spins[nidx] == seed_spin && self.rng.next_f64() < p_add
                {
                    in_cluster[nidx] = true;
                    stack.push((nx, ny));
                }
            }
        }

        for (idx, &in_c) in in_cluster.iter().enumerate() {
            if in_c {
                self.spins[idx] = -self.spins[idx];
            }
        }
    }

    /// 磁化(1スピンあたり)$M=\frac1N\sum_i s_i$。
    pub fn magnetization(&self) -> f64 {
        self.spins.iter().map(|&s| s as f64).sum::<f64>() / (self.l * self.l) as f64
    }

    /// エネルギー(1スピンあたり)$E/N=-\frac{J}{N}\sum_{\langle ij\rangle}s_is_j$($h=0$)。
    /// 各ボンドを1回だけ数える(右隣・下隣のみ)。
    pub fn energy_per_spin(&self) -> f64 {
        let mut e = 0.0;
        for y in 0..self.l {
            for x in 0..self.l {
                let s = self.spins[self.index(x, y)] as f64;
                let right = self.spins[self.index((x + 1) % self.l, y)] as f64;
                let down = self.spins[self.index(x, (y + 1) % self.l)] as f64;
                e += -self.j_coupling * s * (right + down);
            }
        }
        e / (self.l * self.l) as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Onsagerの臨界温度(設計 §2.1)。
    const T_C: f64 = 2.269_185_314_213_022; // 2/ln(1+sqrt(2))

    /// L=64縮約版でWolffを使い、指定温度で系を平衡化したのち`samples`回サンプルして
    /// 帯磁率 $\chi=\frac{N}{k_BT}(\langle M^2\rangle-\langle|M|\rangle^2)$ と
    /// $\langle|M|\rangle$ を返す。$\langle M\rangle$(符号付き)でなく$\langle|M|\rangle$を
    /// 使うのは、Wolffクラスタ更新は低温で系全体の磁化符号を一度に反転させうるため、
    /// 符号付き平均でその反転を素朴に扱うと分散が対称性の破れ自体で支配されて発散し
    /// 本来の(有限)応答関数にならないことを実装検証中に発見したため(標準的な回避策)。
    fn measure(
        l: usize,
        j: f64,
        t: f64,
        seed: u64,
        equilibration: usize,
        samples: usize,
    ) -> (f64, f64) {
        let mut sim = IsingSim::new(l, j, t, SimRng::new(seed, 0));
        for _ in 0..equilibration {
            sim.wolff_step();
        }
        let mut sum_m2 = 0.0;
        let mut sum_abs_m = 0.0;
        for _ in 0..samples {
            sim.wolff_step();
            let m = sim.magnetization();
            sum_m2 += m * m;
            sum_abs_m += m.abs();
        }
        let mean_m2 = sum_m2 / samples as f64;
        let mean_abs_m = sum_abs_m / samples as f64;
        let n = (l * l) as f64;
        let chi = n / t * (mean_m2 - mean_abs_m * mean_abs_m);
        (chi, mean_abs_m)
    }

    /// S7: イジング臨界温度、帯磁率ピークから推定、L=64縮約でrel 5%
    /// (docs/21-verification/01-analytic-tests.md S7)。T_c近傍を粗くスキャンし
    /// 帯磁率が最大になる温度をT_c推定値とする。
    #[test]
    fn s7_susceptibility_peak_estimates_critical_temperature() {
        let l = 64;
        let j = 1.0;
        let temps = [2.05, 2.15, 2.2, 2.25, 2.3, 2.35, 2.4, 2.5, 2.6];
        let mut best_t = temps[0];
        let mut best_chi = f64::NEG_INFINITY;
        for (i, &t) in temps.iter().enumerate() {
            let (chi, _) = measure(l, j, t, 1000 + i as u64, 200, 400);
            if chi > best_chi {
                best_chi = chi;
                best_t = t;
            }
        }
        let rel_err = (best_t - T_C).abs() / T_C;
        assert!(
            rel_err < 0.05,
            "best_t={best_t} T_C={T_C} rel_err={rel_err} best_chi={best_chi}"
        );
    }

    /// S8: 自発磁化 $M(T)=(1-\sinh^{-4}(2J/k_BT))^{1/8}$($T<T_c$)、L=64縮約でrel 5%
    /// (docs/21-verification/01-analytic-tests.md S8)。有限系のWolffは符号がランダムに
    /// 反転しうるため $\langle|M|\rangle$ で比較する。
    #[test]
    fn s8_spontaneous_magnetization_matches_onsager_formula() {
        let l = 64;
        let j = 1.0;
        let t = 2.0; // T < T_c
        let (_, mean_abs_m) = measure(l, j, t, 42, 500, 1000);

        let x = 2.0 * j / t;
        let expected = (1.0 - x.sinh().powi(-4)).powf(1.0 / 8.0);
        let rel_err = (mean_abs_m - expected).abs() / expected;
        assert!(
            rel_err < 0.05,
            "mean_abs_m={mean_abs_m} expected={expected} rel_err={rel_err}"
        );
    }

    /// S9: 小系(4x4=65536状態)の詳細釣り合い、厳密分配関数との照合、rel 1%
    /// (docs/21-verification/01-analytic-tests.md S9)。全 $2^{16}$ 状態を直接列挙して
    /// $\langle|M|\rangle$ の厳密期待値を計算し、メトロポリスで長時間サンプルした
    /// 経験平均と比較する(全状態の訪問頻度そのものを2^16通り照合するのは統計的に
    /// 非現実的なため、集約観測量での照合に簡略化)。
    #[test]
    fn s9_small_system_metropolis_average_matches_exact_partition_function() {
        let l = 4;
        let n = l * l;
        let j = 1.0;
        let t = 2.0;

        // 厳密解: 全2^16状態を列挙してボルツマン重み付き<|M|>を計算。
        let mut z = 0.0;
        let mut weighted_abs_m = 0.0;
        for state in 0u32..(1u32 << n) {
            let spins: Vec<i8> = (0..n)
                .map(|i| if (state >> i) & 1 == 1 { 1 } else { -1 })
                .collect();
            let mut e = 0.0;
            for y in 0..l {
                for x in 0..l {
                    let s = spins[y * l + x] as f64;
                    let right = spins[y * l + (x + 1) % l] as f64;
                    let down = spins[((y + 1) % l) * l + x] as f64;
                    e += -j * s * (right + down);
                }
            }
            let m = spins.iter().map(|&s| s as f64).sum::<f64>() / n as f64;
            let w = (-e / t).exp();
            z += w;
            weighted_abs_m += w * m.abs();
        }
        let exact_mean_abs_m = weighted_abs_m / z;

        // メトロポリスで長時間サンプル。
        let mut sim = IsingSim::new(l, j, t, SimRng::new(7, 3));
        for _ in 0..2000 {
            sim.metropolis_sweep();
        }
        let sweeps = 40000;
        let mut sum_abs_m = 0.0;
        for _ in 0..sweeps {
            sim.metropolis_sweep();
            sum_abs_m += sim.magnetization().abs();
        }
        let sampled_mean_abs_m = sum_abs_m / sweeps as f64;

        let rel_err = (sampled_mean_abs_m - exact_mean_abs_m).abs() / exact_mean_abs_m;
        assert!(
            rel_err < 0.01,
            "sampled={sampled_mean_abs_m} exact={exact_mean_abs_m} rel_err={rel_err}"
        );
    }
}
