//! 2Dイジング模型・メトロポリス法・Wolffクラスタ法。設計: docs/15-statistical/04-monte-carlo.md。
//!
//! P5 スコープの最小実装: 2D正方格子・周期境界・$h=0$(外場ゼロ、Onsager厳密解が
//! 存在する設定)。3D・非ゼロ外場でのWolff($h\ne0$は再重み付けが必要で未対応)は未実装。

use sim_core::{Approximation, EnergyBreakdown, Solver, SolverContext, StateHasher};
use sim_math::SimRng;

/// $L\times L$ 2Dイジング模型。設計 §3 `IsingSim` の縮約版(観測量の移動平均フィールドは
/// 持たず、呼び出し側が`magnetization`/`energy`等を都度計算する)。
#[derive(Clone)]
pub struct IsingSim {
    pub spins: Vec<i8>,
    pub l: usize,
    pub j_coupling: f64,
    pub temperature: f64,
    /// `Solver::step` 1回あたりに回す更新回数(**群3で追加**)。
    ///
    /// **モンテカルロには物理時間が無い**——`Solver::step(dt, ..)` の `dt` は
    /// 何も意味しない。設計 docs/15-statistical/04-monte-carlo.md はイジング模型を
    /// 「平衡状態のサンプリング」と位置づけており、時間発展方程式を解いているの
    /// ではないため、`dt` に対応する物理量が存在しない。そこで **1 step = この
    /// 回数ぶんの更新** と定義し、`dt` は無視する(無視していることを
    /// `approximations()` で申告する)。
    pub updates_per_step: u32,
    /// `true` なら Wolff クラスタ法、`false` ならメトロポリス法で更新する。
    /// 臨界点近傍(T≈2.269 J/k_B)ではメトロポリスが臨界減速を起こすため Wolff を使う。
    pub use_wolff: bool,
}

impl IsingSim {
    /// 全スピンをPRNGでランダムに$\pm1$初期化する(高温初期状態)。
    /// **群3で `rng` を所有しなくなった**。以前は `SimRng` をフィールドとして
    /// 抱え込んでいたが、設計 docs/00-foundation/04-architecture.md は
    /// 「決定論は World が持つ単一の seed 付き PRNG から導く」と定めており、
    /// ドメインが独自の乱数源を持つとその系譜から外れる(`SolverContext::rng`
    /// を渡されても使えない)。初期化にだけ `rng` を借り、以後の更新は
    /// 呼び出し側が渡す `&mut SimRng` を使う。
    pub fn new(l: usize, j_coupling: f64, temperature: f64, rng: &mut SimRng) -> IsingSim {
        let spins = (0..l * l)
            .map(|_| if rng.next_f64() < 0.5 { 1 } else { -1 })
            .collect();
        IsingSim {
            spins,
            l,
            j_coupling,
            temperature,
            updates_per_step: 1,
            use_wolff: false,
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
    pub fn metropolis_sweep(&mut self, rng: &mut SimRng) {
        for y in 0..self.l {
            for x in 0..self.l {
                let idx = self.index(x, y);
                let s = self.spins[idx] as f64;
                let nb = self.neighbor_sum(x, y) as f64;
                let delta_e = 2.0 * self.j_coupling * s * nb;
                if delta_e <= 0.0 || rng.next_f64() < (-delta_e / self.temperature).exp() {
                    self.spins[idx] = -self.spins[idx];
                }
            }
        }
    }

    /// Wolffクラスタ法(設計 §4、S7/S8の必須実装 — 臨界域での臨界減速を回避する)。
    /// シードスピンから同符号の隣接スピンを確率 $p=1-e^{-2J/k_BT}$ で再帰的にクラスタへ
    /// 加え(棄却なしの一括反転)、1回の呼び出しで1クラスタ分だけ更新する。
    pub fn wolff_step(&mut self, rng: &mut SimRng) {
        let l = self.l;
        let p_add = 1.0 - (-2.0 * self.j_coupling / self.temperature).exp();

        let start = rng.range_u32((l * l) as u32) as usize;
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
                if !in_cluster[nidx] && self.spins[nidx] == seed_spin && rng.next_f64() < p_add {
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

    /// 格子全体の交換エネルギー $E=-J\sum_{\langle ij\rangle}s_is_j$(1スピンあたりでない総和)。
    pub fn total_exchange_energy(&self) -> f64 {
        self.energy_per_spin() * (self.l * self.l) as f64
    }
}

/// **`Solver` 実装(群3)**。設計 docs/00-foundation/04-architecture.md §1.2 は
/// 統計を7ドメインの1つとして数えているのに `Solver` 未実装で、`World` に載る
/// 経路が原理的に無かった(D31「イジング模型の相転移」がギャラリーに出せなかった原因)。
///
/// **モンテカルロを時間発展ソルバの器に入れることの正直な説明**: イジング模型の
/// 更新は平衡分布からのサンプリングであって時間発展ではない。したがって
/// `Solver::step(dt, ..)` の `dt` は**物理的な意味を持たず、無視される**。
/// 代わりに `updates_per_step` 回の更新を行う。この不一致を隠さないため、
/// `approximations()` の先頭でそれを申告する。
impl Solver for IsingSim {
    /// `dt` に一切依存しないので上限は無い。
    fn max_stable_dt(&self) -> f64 {
        f64::INFINITY
    }

    fn step(&mut self, _dt: f64, ctx: &mut SolverContext) {
        for _ in 0..self.updates_per_step {
            if self.use_wolff {
                self.wolff_step(ctx.rng);
            } else {
                self.metropolis_sweep(ctx.rng);
            }
        }
    }

    /// **交換エネルギーを `electromagnetic` に入れる**。`EnergyBreakdown` が持つ
    /// 6形態(運動・ポテンシャル・弾性・熱・電磁場・化学)のうち、スピン間交換
    /// 相互作用は磁性——すなわち電磁的な起源——なのでこの枠に入れる。
    /// なお $J$ の単位はここでは無次元(温度も $k_BT/J$ 相当のスケール)なので、
    /// SI 単位の他ドメインと合算した値に物理的な意味は無い(近似として申告する)。
    ///
    /// **モンテカルロはエネルギーを保存しない**——熱浴と平衡にある正準集団を
    /// サンプルしているので、エネルギーは揺らぐのが正しい挙動である。
    /// `EnergyLedger` の残差はここでは保存則の破れを意味しない。
    fn total_energy(&self) -> EnergyBreakdown {
        EnergyBreakdown {
            electromagnetic: self.total_exchange_energy(),
            ..Default::default()
        }
    }

    fn state_hash(&self, hasher: &mut StateHasher) {
        hasher.write_u64(self.l as u64);
        // スピンは ±1 なので 1 バイトに詰めて書く(f64 化すると格子が大きいとき
        // ハッシュ計算だけで無視できない時間を食う)。
        for &s in &self.spins {
            hasher.write_u64(if s > 0 { 1 } else { 0 });
        }
    }

    fn approximations(&self) -> Vec<Approximation> {
        vec![
            Approximation {
                name: "モンテカルロ: dt は無視される",
                reason: "イジング模型の更新は平衡分布のサンプリングであって時間発展ではない。1 step = updates_per_step 回の更新と定義しており、Solver::step の dt には対応する物理量が無い。",
                doc: "docs/15-statistical/04-monte-carlo.md",
                can_disable: false,
            },
            Approximation {
                name: if self.use_wolff {
                    "更新: Wolff クラスタ法"
                } else {
                    "更新: メトロポリス法(臨界減速あり)"
                },
                reason: if self.use_wolff {
                    "同符号の隣接スピンをクラスタとして一括反転する。臨界点近傍でも相関時間が発散しない。"
                } else {
                    "1スピンずつ反転を試行する。臨界点近傍(T≈2.269 J/k_B)では相関時間が発散し、平衡化に必要なスイープ数が爆発する(use_wolff = true で回避できる)。"
                },
                doc: "docs/15-statistical/04-monte-carlo.md",
                can_disable: true,
            },
            Approximation {
                name: "外場ゼロ (h=0)・2D正方格子・周期境界",
                reason: "Onsager の厳密解が存在する設定に限定している。3D と h≠0(Wolff では再重み付けが要る)は未実装。",
                doc: "docs/15-statistical/04-monte-carlo.md",
                can_disable: false,
            },
        ]
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
        let mut rng = SimRng::new(seed, 0);
        let mut sim = IsingSim::new(l, j, t, &mut rng);
        for _ in 0..equilibration {
            sim.wolff_step(&mut rng);
        }
        let mut sum_m2 = 0.0;
        let mut sum_abs_m = 0.0;
        for _ in 0..samples {
            sim.wolff_step(&mut rng);
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
        let mut rng = SimRng::new(7, 3);
        let mut sim = IsingSim::new(l, j, t, &mut rng);
        for _ in 0..2000 {
            sim.metropolis_sweep(&mut rng);
        }
        let sweeps = 40000;
        let mut sum_abs_m = 0.0;
        for _ in 0..sweeps {
            sim.metropolis_sweep(&mut rng);
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

#[cfg(test)]
mod limit_tests {
    use super::*;

    /// **設計 docs/15-statistical/04-monte-carlo.md §7「高温極限: $M\to0$、
    /// $E\to0$(乱雑)。低温極限: $|M|\to1$」**(§7網羅監査で未カバーと判明し
    /// 本増分で追加)。S7(帯磁率ピーク)/S8(自発磁化のOnsager式)は臨界点
    /// 近傍を見るテストで、**両極限そのものは検証されていなかった**。
    ///
    /// 極限は解析解を要さない定性的な主張だが、**モンテカルロが正しく
    /// 熱平衡へ緩和しているかの最も基本的な健全性検査**である——ここが外れる
    /// なら臨界点近傍の一致は偶然でしかない。
    #[test]
    fn ising_reaches_the_disordered_and_ordered_limits() {
        let l = 16;
        let j = 1.0;

        // 高温極限(T >> Tc≈2.269): スピンは乱雑になり、磁化・エネルギーとも0付近。
        let mut hot_rng = SimRng::new(7, 1);
        let mut hot = IsingSim::new(l, j, 50.0, &mut hot_rng);
        for _ in 0..2000 {
            hot.metropolis_sweep(&mut hot_rng);
        }
        let hot_m = hot.magnetization().abs();
        let hot_e = hot.energy_per_spin().abs();
        assert!(hot_m < 0.15, "高温では磁化が0へ向かうべき: |M|={hot_m}");
        assert!(
            hot_e < 0.15,
            "高温ではスピン間の相関が消えエネルギーも0へ向かうべき: |E|={hot_e}"
        );

        // 低温極限(T << Tc): 整列して |M| → 1。
        let mut cold_rng = SimRng::new(7, 2);
        let mut cold = IsingSim::new(l, j, 0.4, &mut cold_rng);
        for _ in 0..2000 {
            cold.wolff_step(&mut cold_rng);
        }
        let cold_m = cold.magnetization().abs();
        assert!(
            cold_m > 0.95,
            "低温では自発磁化が飽和し |M|→1 になるべき: |M|={cold_m}"
        );
        // 完全整列なら最近接4本すべて揃うので E/N → -2J。
        let cold_e = cold.energy_per_spin();
        assert!(
            (cold_e + 2.0 * j).abs() < 0.1,
            "低温のエネルギーは完全整列の -2J へ向かうべき: E/N={cold_e}"
        );
    }
}
