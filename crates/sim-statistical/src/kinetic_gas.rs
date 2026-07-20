//! 気体分子運動論(剛体球MD)。設計: docs/15-statistical/02-kinetic-gas.md。
//!
//! P5 スコープの最小実装: 剛体球気体(既定モデル、設計 §2.1)+ 反射壁の箱 + 空間ハッシュ
//! broadphase(設計 §10「空間ハッシュ + Morton順でO(N)」の縮約、Morton順序ソートは未実装)。
//! Lennard-Jones・熱壁・ピストン・輸送係数測定は未実装。

use sim_math::Vec3;
use std::collections::HashMap;

/// ボルツマン定数 [J/K]。
pub const BOLTZMANN_CONSTANT: f64 = 1.380649e-23;

/// 剛体球気体シミュレーション。設計 §3 `GasSim` の縮約版(`Interaction`/`Container`の
/// enum抽象・ピストン・熱壁は未実装、剛体球+反射壁の箱のみ)。
pub struct GasSim {
    pub position: Vec<Vec3>,
    pub velocity: Vec<Vec3>,
    /// 全粒子共通の質量 [kg]。
    pub mass: f64,
    /// 剛体球半径 [m]。
    pub radius: f64,
    /// 箱は `[0, box_size.x] x [0, box_size.y] x [0, box_size.z]`。
    pub box_size: Vec3,
    /// 壁への運動量移動の累積(圧力測定用、設計 §4「壁への運動量流束」)。
    wall_impulse_accum: f64,
    wall_impulse_time: f64,
    /// 実際に速度交換を行った粒子間衝突の累積数(平衡化の進捗診断用)。
    pub collision_count: u64,
}

fn cell_key(p: Vec3, cell_size: f64) -> (i64, i64, i64) {
    (
        (p.x / cell_size).floor() as i64,
        (p.y / cell_size).floor() as i64,
        (p.z / cell_size).floor() as i64,
    )
}

/// 1軸の壁反射。鏡面反射(設計§4「鏡面反射(断熱壁)」)+ 箱内へのクランプ。
/// 戻り値は `(新しい座標, 新しい速度, 壁へ移動した運動量の大きさ/質量=速度変化量)`。
fn reflect_axis(p: f64, v: f64, radius: f64, extent: f64) -> (f64, f64, f64) {
    let lo = radius;
    let hi = extent - radius;
    if p < lo {
        (lo + (lo - p), -v, 2.0 * v.abs())
    } else if p > hi {
        (hi - (p - hi), -v, 2.0 * v.abs())
    } else {
        (p, v, 0.0)
    }
}

impl GasSim {
    pub fn new(mass: f64, radius: f64, box_size: Vec3) -> GasSim {
        GasSim {
            position: Vec::new(),
            velocity: Vec::new(),
            mass,
            radius,
            box_size,
            wall_impulse_accum: 0.0,
            wall_impulse_time: 0.0,
            collision_count: 0,
        }
    }

    pub fn add_particle(&mut self, position: Vec3, velocity: Vec3) -> usize {
        let idx = self.position.len();
        self.position.push(position);
        self.velocity.push(velocity);
        idx
    }

    pub fn particle_count(&self) -> usize {
        self.position.len()
    }

    /// 温度 $T = \frac{m}{3k_BN}\sum_i|v_i-\bar v|^2$(重心運動を除く、設計 §4)。
    pub fn temperature(&self) -> f64 {
        let n = self.position.len() as f64;
        let com = self
            .velocity
            .iter()
            .fold(Vec3::new(0.0, 0.0, 0.0), |acc, &v| acc + v)
            .scale(1.0 / n);
        let sum_v2: f64 = self.velocity.iter().map(|&v| (v - com).length_sq()).sum();
        self.mass * sum_v2 / (3.0 * BOLTZMANN_CONSTANT * n)
    }

    /// 圧力測定の累積をリセットする(測定窓の開始、設計 §4「移動平均窓」の簡略版:
    /// 単一窓のみ)。
    pub fn reset_pressure_accumulator(&mut self) {
        self.wall_impulse_accum = 0.0;
        self.wall_impulse_time = 0.0;
    }

    /// $p = \frac{\sum\Delta p_{wall}}{A_{total}\Delta t}$(設計 §4)。`reset_pressure_accumulator`
    /// 呼び出し以降に経過した時間で正規化する。
    pub fn pressure(&self) -> f64 {
        let b = self.box_size;
        let surface_area = 2.0 * (b.x * b.y + b.y * b.z + b.z * b.x);
        self.wall_impulse_accum / (surface_area * self.wall_impulse_time)
    }

    /// 1ステップ進める(設計 §4 `gas_step`): 自由飛行 → 壁衝突 → 剛体球衝突。
    pub fn step(&mut self, dt: f64) {
        for i in 0..self.position.len() {
            self.position[i] = self.position[i].addcarry_scaled(self.velocity[i], dt);
        }
        self.resolve_wall_collisions();
        self.resolve_particle_collisions();
        self.wall_impulse_time += dt;
    }

    fn resolve_wall_collisions(&mut self) {
        for i in 0..self.position.len() {
            let pos = self.position[i];
            let vel = self.velocity[i];
            let (px, vx, ix) = reflect_axis(pos.x, vel.x, self.radius, self.box_size.x);
            let (py, vy, iy) = reflect_axis(pos.y, vel.y, self.radius, self.box_size.y);
            let (pz, vz, iz) = reflect_axis(pos.z, vel.z, self.radius, self.box_size.z);
            self.position[i] = Vec3::new(px, py, pz);
            self.velocity[i] = Vec3::new(vx, vy, vz);
            self.wall_impulse_accum += self.mass * (ix + iy + iz);
        }
    }

    /// 剛体球の重なりペアを空間ハッシュ(セル幅 = 直径)で検出し弾性衝突解を適用する
    /// (設計 §4「重なりペア → 弾性衝突解(衝突法線方向の速度交換)」)。ペアは
    /// `j > i` のみを数えることで cell-cell 二重処理を避け、決定論のため粒子インデックス
    /// 昇順に逐次解決する(mechanicsのsequential impulsesと同じ方針)。
    fn resolve_particle_collisions(&mut self) {
        let cell_size = 2.0 * self.radius;
        let mut cells: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
        for (i, &p) in self.position.iter().enumerate() {
            cells.entry(cell_key(p, cell_size)).or_default().push(i);
        }

        let min_dist_sq = (2.0 * self.radius).powi(2);
        let n = self.position.len();
        for i in 0..n {
            let key = cell_key(self.position[i], cell_size);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        let neighbor = (key.0 + dx, key.1 + dy, key.2 + dz);
                        let Some(candidates) = cells.get(&neighbor) else {
                            continue;
                        };
                        for &j in candidates {
                            if j <= i {
                                continue;
                            }
                            let delta = self.position[j] - self.position[i];
                            let dist_sq = delta.length_sq();
                            if dist_sq < min_dist_sq && dist_sq > 1e-30 {
                                self.resolve_pair_collision(i, j, delta, dist_sq);
                            }
                        }
                    }
                }
            }
        }
    }

    /// 等質量剛体球の弾性衝突: 相対速度の法線成分を交換する(1次元弾性衝突の完全交換解、
    /// 運動量保存 $v_i'+v_j'=v_i+v_j$ とエネルギー保存 $|v_j'-v_i'|_n=-|v_j-v_i|_n$ から導出)。
    /// 貫入は法線方向に半分ずつ押し戻して解消する。
    fn resolve_pair_collision(&mut self, i: usize, j: usize, delta: Vec3, dist_sq: f64) {
        let dist = dist_sq.sqrt();
        let normal = delta.scale(1.0 / dist); // i -> j
        let overlap = 2.0 * self.radius - dist;
        if overlap > 0.0 {
            let correction = normal.scale(overlap * 0.5);
            self.position[i] = self.position[i] - correction;
            self.position[j] = self.position[j] + correction;
        }
        let rel_vel = self.velocity[j] - self.velocity[i];
        let approach = rel_vel.dot(normal);
        if approach < 0.0 {
            let impulse = normal.scale(approach);
            self.velocity[i] = self.velocity[i] + impulse;
            self.velocity[j] = self.velocity[j] - impulse;
            self.collision_count += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_math::SimRng;

    /// N2相当のパラメータで `n`×`n`×`n` 格子に粒子を配置する(設計 §9 パラメータ表の質量・
    /// 直径)。`spacing_factor`(直径の何倍の格子間隔か)で密度を制御する: S1/S3 は速い
    /// 熱平衡化のため密(格子間隔小)、S2 は理想気体近似(排除体積効果を無視できる希薄極限)
    /// のため疎(格子間隔大)にする必要がある(実装検証中に、密なほうがS1には都合が良い一方
    /// S2では剛体球の排除体積によるvirial補正(Carnahan-Starling状態方程式)でpVがNkTから
    /// 大きくずれることを発見し、S2専用の希薄配置に分けた)。
    fn lattice_gas(n: usize, spacing_factor: f64, temperature: f64, rng: &mut SimRng) -> GasSim {
        let mass = 4.65e-26; // N2
        let radius = 1.85e-10; // N2
        let spacing = spacing_factor * radius;
        let box_len = n as f64 * spacing;
        let mut sim = GasSim::new(mass, radius, Vec3::new(box_len, box_len, box_len));
        let sigma = (BOLTZMANN_CONSTANT * temperature / mass).sqrt();
        for ix in 0..n {
            for iy in 0..n {
                for iz in 0..n {
                    let pos = Vec3::new(
                        (ix as f64 + 0.5) * spacing,
                        (iy as f64 + 0.5) * spacing,
                        (iz as f64 + 0.5) * spacing,
                    );
                    sim.add_particle(pos, rng.maxwell_boltzmann_velocity(sigma));
                }
            }
        }
        sim
    }

    /// 誤差関数の近似(Abramowitz & Stegun 7.1.26、最大誤差1.5e-7)。マクスウェル速度分布の
    /// CDFに必要(標準ライブラリにerfがないため自前実装)。
    fn erf(x: f64) -> f64 {
        let sign = if x < 0.0 { -1.0 } else { 1.0 };
        let x = x.abs();
        let p = 0.3275911;
        let (a1, a2, a3, a4, a5) = (
            0.254829592,
            -0.284496736,
            1.421413741,
            -1.453152027,
            1.061405429,
        );
        let t = 1.0 / (1.0 + p * x);
        let y = 1.0 - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * (-x * x).exp();
        sign * y
    }

    /// マクスウェル速さ分布のCDF $F(v)=\mathrm{erf}(v/\sqrt2 a)-\sqrt{2/\pi}(v/a)e^{-v^2/2a^2}$、
    /// $a=\sqrt{k_BT/m}$(設計 §2.2)。
    fn mb_speed_cdf(v: f64, a: f64) -> f64 {
        if v <= 0.0 {
            return 0.0;
        }
        erf(v / (std::f64::consts::SQRT_2 * a))
            - (2.0 / std::f64::consts::PI).sqrt() * (v / a) * (-v * v / (2.0 * a * a)).exp()
    }

    /// $F(v)=p$ を満たす $v$ を二分法で求める(等確率ビンの境界を作るため)。
    fn mb_speed_cdf_inverse(p: f64, a: f64) -> f64 {
        let mut lo = 0.0;
        let mut hi = 10.0 * a;
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if mb_speed_cdf(mid, a) < p {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }

    /// S1: マクスウェル=ボルツマン速度分布への収束、$\chi^2$適合(有意水準1%)
    /// (docs/21-verification/01-analytic-tests.md S1)。全粒子を同一速さ$v_0$・ランダム方向で
    /// 初期化し(速度分布は非MB=デルタ関数)、剛体球衝突を多数回起こして速さ分布を
    /// マクスウェル分布へ緩和させる。全運動エネルギーは弾性衝突で厳密に保存されるため、
    /// 目標温度は初期条件から解析的に既知(測定でフィットするパラメータが無いため
    /// 自由度 = ビン数-1)。等確率ビン(逆CDFを二分法で求めて境界とする)を使うことで
    /// 期待度数を全ビンで均一にし、$\chi^2$検定の前提(期待度数≥5程度)を確実に満たす。
    #[test]
    fn s1_speed_distribution_converges_to_maxwell_boltzmann() {
        let mass = 4.65e-26;
        let radius = 1.85e-10;
        let target_t = 300.0;
        let n_side = 10;
        let spacing = 2.3 * radius; // 密な配置(速い熱平衡化、S2とは別配置)
        let box_len = n_side as f64 * spacing;

        let v0 = (3.0 * BOLTZMANN_CONSTANT * target_t / mass).sqrt();
        let mut sim = GasSim::new(mass, radius, Vec3::new(box_len, box_len, box_len));
        let mut rng = SimRng::new(42, 0);
        for ix in 0..n_side {
            for iy in 0..n_side {
                for iz in 0..n_side {
                    let pos = Vec3::new(
                        (ix as f64 + 0.5) * spacing,
                        (iy as f64 + 0.5) * spacing,
                        (iz as f64 + 0.5) * spacing,
                    );
                    sim.add_particle(pos, rng.unit_sphere().scale(v0));
                }
            }
        }
        let n = sim.particle_count();

        let dt = radius / (20.0 * v0);
        for _ in 0..2000 {
            sim.step(dt);
        }
        // 数百衝突/粒子(設計§7の目標)に達していることを確認。
        assert!(
            sim.collision_count > 100 * n as u64,
            "collision_count={} n={n}",
            sim.collision_count
        );

        let a = (BOLTZMANN_CONSTANT * target_t / mass).sqrt();
        let bins = 8;
        let speeds: Vec<f64> = sim.velocity.iter().map(|v| v.length()).collect();
        let mut edges = vec![0.0];
        for k in 1..bins {
            edges.push(mb_speed_cdf_inverse(k as f64 / bins as f64, a));
        }
        edges.push(f64::INFINITY);

        let mut observed = vec![0usize; bins];
        for &s in &speeds {
            let bin = edges.windows(2).position(|w| s < w[1]).unwrap_or(bins - 1);
            observed[bin] += 1;
        }
        let expected = n as f64 / bins as f64;
        let chi2: f64 = observed
            .iter()
            .map(|&obs| (obs as f64 - expected).powi(2) / expected)
            .sum();

        // dof = bins-1 = 7、有意水準1%の臨界値(標準分布表)= 18.475。
        assert!(chi2 < 18.475, "chi2={chi2} observed={observed:?}");
    }

    /// S2: 状態方程式 $pV=Nk_BT$、rel 2%(N=10⁴基準、ここではN=1000で確認)
    /// (docs/21-verification/01-analytic-tests.md S2)。剛体球の排除体積によるvirial補正
    /// (Carnahan-Starling)が無視できる希薄極限(充填率φ≈0.0012)を使う必要があると
    /// 実装検証中に発見(密な配置ではpVがNkTの数倍になった)。初期速度をマクスウェル分布で
    /// 直接生成し(S1とは異なりここでは分布形の収束自体は検証対象でない)、ウォームアップ後に
    /// 圧力測定窓をリセットして壁への運動量移動から圧力を求める。
    #[test]
    fn s2_equation_of_state_matches_pv_equals_nkt() {
        let target_t = 300.0;
        let mut rng = SimRng::new(7, 1);
        let mut sim = lattice_gas(10, 15.0, target_t, &mut rng);
        let n = sim.particle_count();
        let box_len = sim.box_size.x;
        let volume = box_len.powi(3);

        let sigma = (BOLTZMANN_CONSTANT * target_t / sim.mass).sqrt();
        let dt = sim.radius / (20.0 * sigma * 3f64.sqrt());

        for _ in 0..1000 {
            sim.step(dt);
        }
        sim.reset_pressure_accumulator();
        for _ in 0..6000 {
            sim.step(dt);
        }

        let p = sim.pressure();
        let t_measured = sim.temperature();
        let pv = p * volume;
        let nkt = n as f64 * BOLTZMANN_CONSTANT * t_measured;
        let rel_err = (pv - nkt).abs() / nkt;
        assert!(
            rel_err < 0.02,
            "p={p:.4e} pV={pv:.4e} NkT={nkt:.4e} rel_err={rel_err:.5}"
        );
    }

    /// S3: 等分配則 $\langle v_x^2\rangle=\langle v_y^2\rangle=\langle v_z^2\rangle$、
    /// $3/\sqrt N$以内(docs/21-verification/01-analytic-tests.md S3)。等分配は等方的な
    /// 熱平衡状態であれば密度によらず成り立つ統計力学の一般定理のため、S1と同じ(密な)
    /// 配置を使い、剛体球衝突で速度分布が等方化したあとの状態で確認する。
    #[test]
    fn s3_equipartition_holds_across_velocity_axes() {
        let mass = 4.65e-26;
        let radius = 1.85e-10;
        let target_t = 300.0;
        let n_side = 10;
        let spacing = 2.3 * radius;
        let box_len = n_side as f64 * spacing;

        let v0 = (3.0 * BOLTZMANN_CONSTANT * target_t / mass).sqrt();
        let mut sim = GasSim::new(mass, radius, Vec3::new(box_len, box_len, box_len));
        let mut rng = SimRng::new(99, 2);
        for ix in 0..n_side {
            for iy in 0..n_side {
                for iz in 0..n_side {
                    let pos = Vec3::new(
                        (ix as f64 + 0.5) * spacing,
                        (iy as f64 + 0.5) * spacing,
                        (iz as f64 + 0.5) * spacing,
                    );
                    sim.add_particle(pos, rng.unit_sphere().scale(v0));
                }
            }
        }
        let n = sim.particle_count();

        let dt = radius / (20.0 * v0);
        for _ in 0..2000 {
            sim.step(dt);
        }

        let vx2: f64 = sim.velocity.iter().map(|v| v.x * v.x).sum::<f64>() / n as f64;
        let vy2: f64 = sim.velocity.iter().map(|v| v.y * v.y).sum::<f64>() / n as f64;
        let vz2: f64 = sim.velocity.iter().map(|v| v.z * v.z).sum::<f64>() / n as f64;
        let mean = (vx2 + vy2 + vz2) / 3.0;
        let tol = 3.0 / (n as f64).sqrt();

        for (label, v2) in [("vx2", vx2), ("vy2", vy2), ("vz2", vz2)] {
            let rel_dev = (v2 - mean).abs() / mean;
            assert!(
                rel_dev < tol,
                "{label}={v2:.4e} mean={mean:.4e} rel_dev={rel_dev:.4} tol={tol:.4}"
            );
        }
    }
}
