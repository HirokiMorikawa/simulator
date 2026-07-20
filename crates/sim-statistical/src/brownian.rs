//! ランジュバン方程式・BAOAB積分。設計: docs/15-statistical/03-diffusion-brownian.md。
//!
//! P4 スコープ: ブラウン粒子集合 + BAOAB(kick-drift-kick + OU厳密解 + kick-drift-kick)。
//! 濃度場の拡散(陰的Euler、熱伝導と同一コード共有)・移流拡散・回転ブラウン運動は
//! Phase 5+ で拡張する。`Solver` トレイトは外力を汎用の関数引数として渡す必要があるため
//! (熱・力学等の固定シグネチャに馴染まない)、`step_baoab` は独立メソッドとして提供する。

use sim_math::{SimRng, Vec3};

/// ブラウン粒子集合。設計 §3「通常の剛体(微小)+ BrownianForce、または軽量 ParticleSet」の
/// 後者。粒子は同一の質量・抵抗係数・温度を共有する(異種粒子径混合は Phase 5 拡張)。
pub struct BrownianParticleSet {
    pub position: Vec<Vec3>,
    pub velocity: Vec<Vec3>,
    /// 粒子質量 [kg]。
    pub mass: f64,
    /// ストークス抵抗係数 γ=6πμr [kg/s]。設計 §2.1。
    pub gamma: f64,
    /// k_B・T [J]。設計 §2.1 の揺動散逸定理(ノイズ強度 √(2γk_BT) を一意に決める)。
    pub kb_t: f64,
}

impl BrownianParticleSet {
    pub fn new(mass: f64, gamma: f64, kb_t: f64) -> BrownianParticleSet {
        BrownianParticleSet {
            position: Vec::new(),
            velocity: Vec::new(),
            mass,
            gamma,
            kb_t,
        }
    }

    pub fn add_particle(&mut self, position: Vec3, velocity: Vec3) -> usize {
        let idx = self.position.len();
        self.position.push(position);
        self.velocity.push(velocity);
        idx
    }

    /// Stokes-Einstein 拡散係数 D = k_BT/γ(設計 §2.2)。
    pub fn diffusion_coefficient(&self) -> f64 {
        self.kb_t / self.gamma
    }

    /// BAOAB 1ステップ(設計 §4.1)。`force` は粒子位置から外力 [N] を返す(重力・トラップ力等)。
    /// O 段は Ornstein-Uhlenbeck 過程の厳密解(`c1=e^{-γdt/m}`, `c2=√(1-c1²)`)のため、
    /// 大きな `γΔt/m`(過減衰領域)でも平衡分布を正確にサンプルする。
    pub fn step_baoab(&mut self, dt: f64, rng: &mut SimRng, force: impl Fn(Vec3) -> Vec3) {
        let c1 = (-self.gamma * dt / self.mass).exp();
        let c2 = (1.0 - c1 * c1).sqrt();
        let noise_sigma = c2 * (self.kb_t / self.mass).sqrt();

        for i in 0..self.position.len() {
            // B
            let f1 = force(self.position[i]);
            self.velocity[i] = self.velocity[i].addcarry_scaled(f1, 0.5 * dt / self.mass);
            // A
            self.position[i] = self.position[i].addcarry_scaled(self.velocity[i], 0.5 * dt);
            // O(厳密 OU 更新)
            let noise = rng.maxwell_boltzmann_velocity(noise_sigma);
            self.velocity[i] = self.velocity[i].scale(c1) + noise;
            // A
            self.position[i] = self.position[i].addcarry_scaled(self.velocity[i], 0.5 * dt);
            // B
            let f2 = force(self.position[i]);
            self.velocity[i] = self.velocity[i].addcarry_scaled(f2, 0.5 * dt / self.mass);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 1μmポリスチレン球相当のパラメータ(設計 §9 パラメータ表)。
    fn polystyrene_bead() -> (f64, f64, f64) {
        let mass = 5.5e-16; // kg
        let gamma = 9.4e-9; // kg/s
        let kb_t = 1.380649e-23 * 293.15; // J (20°C)
        (mass, gamma, kb_t)
    }

    /// S4: 平均二乗変位 <Δx²>=6Dt、rel 3%(docs/21-verification/01-analytic-tests.md S4)。
    /// 外力なし(自由拡散)。
    ///
    /// dt は小さく取る必要がある: BAOAB の O 段(速度の OU 厳密解)は `γΔt/m` が大きくても
    /// 平衡速度分布を正確にサンプルするが、A 段(位置更新 `x += vΔt/2`)の離散化誤差は
    /// 別物で、`γΔt/m` が大きい(強い過減衰かつ粗い dt)と MSD が理論値から系統的に
    /// 大きくずれる(実装検証中に `γΔt/m≈17` で実測 rel_err≈760% を確認、`γΔt/m≈0.17`
    /// (dt=1e-8s)まで下げると rel_err<0.1% に収束することを数値実験で確認した)。
    #[test]
    fn s4_mean_squared_displacement_matches_6dt() {
        let (mass, gamma, kb_t) = polystyrene_bead();
        let mut set = BrownianParticleSet::new(mass, gamma, kb_t);
        let n = 20_000;
        for _ in 0..n {
            set.add_particle(Vec3::ZERO, Vec3::ZERO);
        }

        let dt = 1.0e-8;
        let steps = 1000u32;
        let mut rng = SimRng::new(1, 1);
        for _ in 0..steps {
            set.step_baoab(dt, &mut rng, |_| Vec3::ZERO);
        }

        let t = steps as f64 * dt;
        let msd: f64 =
            set.position.iter().map(|p| p.length_sq()).sum::<f64>() / set.position.len() as f64;
        let expected = 6.0 * set.diffusion_coefficient() * t;
        let rel_err = (msd - expected).abs() / expected;
        assert!(
            rel_err < 0.03,
            "msd={msd} expected={expected} rel_err={rel_err}"
        );
    }

    /// S5: 調和トラップ中の位置分散 = k_BT/k_trap、rel 2%(docs/21-verification/01-analytic-tests.md
    /// S5)。揺動散逸定理の直接検証(抵抗係数から導かれるノイズ強度が正しい平衡分布を作るか)。
    #[test]
    fn s5_harmonic_trap_variance_matches_kbt_over_ktrap() {
        let (mass, gamma, kb_t) = polystyrene_bead();
        let mut set = BrownianParticleSet::new(mass, gamma, kb_t);
        let n = 20_000;
        for _ in 0..n {
            set.add_particle(Vec3::ZERO, Vec3::ZERO);
        }

        let k_trap = 1.0e-4; // N/m(強めのトラップ、緩和時間を短くしテストを高速化)
        let dt = 1.0e-6;
        // 緩和時間 τ=γ/k_trap ≈ 94μs。十分に平衡化させるため 10τ 分ステップする。
        let relax_time = gamma / k_trap;
        let steps = (10.0 * relax_time / dt) as u32;
        let mut rng = SimRng::new(2, 2);
        for _ in 0..steps {
            set.step_baoab(dt, &mut rng, |p| p.scale(-k_trap));
        }

        // 3軸まとめて分散を測る(各軸独立に同じ分布)。
        let variance: f64 =
            set.position.iter().map(|p| p.length_sq()).sum::<f64>() / (3.0 * n as f64);
        let expected = kb_t / k_trap;
        let rel_err = (variance - expected).abs() / expected;
        assert!(
            rel_err < 0.02,
            "variance={variance} expected={expected} rel_err={rel_err}"
        );
    }

    /// S6: 沈降平衡 — 一様重力下、床(y=0)で弾性反射する粒子集団の高度分布が
    /// 指数則 $c(h)\propto e^{-mgh/k_BT}$(ペラン実験)に従うこと、rel 5%
    /// (docs/21-verification/01-analytic-tests.md S6)。指数分布の平均は $k_BT/(mg)$。
    ///
    /// 実重力(9.8 m/s²)では平衡到達までの拡散時間が $h_0^2/D$ ~ 秒級になり自動テストには
    /// 遅すぎるため、S5 の k_trap 増強と同じ発想で合成的に強めた重力加速度を使い、
    /// 平衡到達スケール $h_0$(ひいては緩和時間)を縮める。dt は S4 で精度確認済みの
    /// $\gamma\Delta t/m\approx0.17$ をそのまま使い回す。床の反射は境界条件の型が設計に
    /// 明記されていない検証専用の最小実装として、テスト内で位置・速度を直接操作する。
    #[test]
    fn s6_sedimentation_equilibrium_matches_boltzmann_height_distribution() {
        let (mass, gamma, kb_t) = polystyrene_bead();
        let mut set = BrownianParticleSet::new(mass, gamma, kb_t);
        let n = 5_000;
        for _ in 0..n {
            set.add_particle(Vec3::ZERO, Vec3::ZERO);
        }

        let g_eff = 2000.0; // m/s²(合成値。h0=kBT/(mg)を小さくし平衡到達を高速化)
        let gravity_force = Vec3::new(0.0, -mass * g_eff, 0.0);
        let expected_mean_height = kb_t / (mass * g_eff);

        let diffusion = set.diffusion_coefficient();
        let relax_time = expected_mean_height * expected_mean_height / diffusion;
        let dt = 1.0e-8;
        let steps = (5.0 * relax_time / dt).ceil() as u32;

        let mut rng = SimRng::new(3, 3);
        for _ in 0..steps {
            set.step_baoab(dt, &mut rng, |_| gravity_force);
            for i in 0..set.position.len() {
                if set.position[i].y < 0.0 {
                    set.position[i].y = -set.position[i].y;
                    set.velocity[i].y = -set.velocity[i].y;
                }
            }
        }

        let mean_height: f64 =
            set.position.iter().map(|p| p.y).sum::<f64>() / set.position.len() as f64;
        let rel_err = (mean_height - expected_mean_height).abs() / expected_mean_height;
        assert!(
            rel_err < 0.05,
            "mean_height={mean_height} expected={expected_mean_height} rel_err={rel_err}"
        );
    }
}
