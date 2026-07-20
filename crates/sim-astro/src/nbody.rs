//! 万有引力・N体問題(総当たり + leapfrog)。設計: docs/16-astro/01-gravitation-nbody.md。
//!
//! Pα の基礎部分: 総当たり($O(N^2)$、少数体は Barnes-Hut より高精度・十分速いと
//! 設計 §4.1 が明記する既定モード)+ leapfrog(kick-drift-kick、シンプレクティック)。
//! Barnes-Hut($N\gtrsim256$)・WHFast・浮動原点・レジーム切替(§4.2 の残り)は Phase 3+ で拡張する。

use sim_core::{EnergyBreakdown, Solver, SolverContext, StateHasher};
use sim_math::Vec3;

/// 万有引力定数 [N m^2/kg^2]。設計 §2、CODATA 値。
pub const GRAVITATIONAL_CONSTANT: f64 = 6.674e-11;

/// N体系。設計 §3 の `NBodySystem` から、Barnes-Hut ツリー・積分器種別の選択機構を除いた
/// P0 スコープ(総当たり + leapfrog 固定)。
pub struct NBodySystem {
    pub position: Vec<Vec3>,
    pub velocity: Vec<Vec3>,
    pub mass: Vec<f64>,
    /// 近接特異点の緩和(設計 §2)。既定 0(実天体は接触を剛体/再突入に委ねる)。
    pub softening: f64,
}

impl NBodySystem {
    pub fn new(softening: f64) -> NBodySystem {
        NBodySystem {
            position: Vec::new(),
            velocity: Vec::new(),
            mass: Vec::new(),
            softening,
        }
    }

    pub fn add_body(&mut self, position: Vec3, velocity: Vec3, mass: f64) -> usize {
        let idx = self.position.len();
        self.position.push(position);
        self.velocity.push(velocity);
        self.mass.push(mass);
        idx
    }

    pub fn len(&self) -> usize {
        self.position.len()
    }

    pub fn is_empty(&self) -> bool {
        self.position.is_empty()
    }

    /// 設計 §2: 総当たり重ね合わせによる各体の加速度。$O(N^2)$。
    fn accelerations(&self) -> Vec<Vec3> {
        let n = self.len();
        let mut acc = vec![Vec3::ZERO; n];
        let eps_sq = self.softening * self.softening;
        for (i, acc_i) in acc.iter_mut().enumerate() {
            for j in 0..n {
                if i == j {
                    continue;
                }
                let d = self.position[j] - self.position[i];
                let dist_sq = d.length_sq() + eps_sq;
                let dist = dist_sq.sqrt();
                let factor = GRAVITATIONAL_CONSTANT * self.mass[j] / (dist_sq * dist);
                *acc_i = acc_i.addcarry_scaled(d, factor);
            }
        }
        acc
    }
}

impl Solver for NBodySystem {
    /// シンプレクティック積分は明示的な CFL 条件を持たない(軌道周期に対する刻みの妥当性は
    /// Orchestrator の sub-step 決定に委ねる、設計 §4.2「天体は独立時間軸」)。
    fn max_stable_dt(&self) -> f64 {
        f64::INFINITY
    }

    /// leapfrog(kick-drift-kick)。設計 §4.2:
    /// v_{1/2}=v_0+dt/2・a_0、x_1=x_0+dt・v_{1/2}、v_1=v_{1/2}+dt/2・a_1。
    fn step(&mut self, dt: f64, _ctx: &mut SolverContext) {
        let n = self.len();
        if n == 0 {
            return;
        }
        let a0 = self.accelerations();
        for (v, &a) in self.velocity.iter_mut().zip(a0.iter()) {
            *v = v.addcarry_scaled(a, dt * 0.5);
        }
        for (p, &v) in self.position.iter_mut().zip(self.velocity.iter()) {
            *p = p.addcarry_scaled(v, dt);
        }
        let a1 = self.accelerations();
        for (v, &a) in self.velocity.iter_mut().zip(a1.iter()) {
            *v = v.addcarry_scaled(a, dt * 0.5);
        }
    }

    fn state_hash(&self, hasher: &mut StateHasher) {
        let n = self.len();
        hasher.write_u64(n as u64);
        for i in 0..n {
            hasher.write_vec3(self.position[i]);
            hasher.write_vec3(self.velocity[i]);
        }
    }

    /// 運動エネルギー + 重力ポテンシャル(対ごとに1回、$-Gm_im_j/|r_i-r_j|$)。
    /// 設計 §2 の支配方程式から導かれるポテンシャル(EnergyBreakdown.potential に計上)。
    fn total_energy(&self) -> EnergyBreakdown {
        let n = self.len();
        let mut kinetic = 0.0;
        for i in 0..n {
            kinetic += 0.5 * self.mass[i] * self.velocity[i].length_sq();
        }
        let mut potential = 0.0;
        for i in 0..n {
            for j in (i + 1)..n {
                let dist = (self.position[j] - self.position[i]).length();
                potential -= GRAVITATIONAL_CONSTANT * self.mass[i] * self.mass[j] / dist;
            }
        }
        EnergyBreakdown {
            kinetic,
            potential,
            ..Default::default()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::{EventQueue, MaterialDb};
    use sim_math::SimRng;

    fn step_n(sys: &mut NBodySystem, dt: f64, n: u32) {
        let materials = MaterialDb::standard();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();
        for _ in 0..n {
            let mut ctx = SolverContext {
                materials: &materials,
                rng: &mut rng,
                events: &mut events,
            };
            sys.step(dt, &mut ctx);
        }
    }

    /// A3: 円軌道速度 v=sqrt(GM/r)、rel 0.1%(docs/21-verification/01-analytic-tests.md A3)。
    /// 太陽-地球相当(1AU、太陽質量)で1公転させ、半径がほぼ一定に保たれることを確認する。
    #[test]
    fn a3_circular_orbit_speed_matches_vis_viva_formula() {
        let mass_sun = 1.989e30;
        let r = 1.496e11; // 1 AU
        let mut sys = NBodySystem::new(0.0);
        sys.add_body(Vec3::ZERO, Vec3::ZERO, mass_sun);
        let v_circ = (GRAVITATIONAL_CONSTANT * mass_sun / r).sqrt();
        let idx = sys.add_body(Vec3::new(r, 0.0, 0.0), Vec3::new(0.0, v_circ, 0.0), 1.0);

        let period =
            2.0 * std::f64::consts::PI * (r.powi(3) / (GRAVITATIONAL_CONSTANT * mass_sun)).sqrt();
        let steps = 10_000u32;
        let dt = period / steps as f64;
        step_n(&mut sys, dt, steps);

        // 1周後、出発点付近に戻り半径がほぼ一定であること。
        let final_r = sys.position[idx].length();
        assert!((final_r - r).abs() / r < 0.001, "final_r={final_r} r={r}");
        let final_speed = sys.velocity[idx].length();
        assert!(
            (final_speed - v_circ).abs() / v_circ < 0.001,
            "final_speed={final_speed} v_circ={v_circ}"
        );
    }

    /// A2(縮約版): 二体のエネルギー・角運動量保存。10⁶周のフル検証は長時間級のため、
    /// 縮約(100周)でシンプレクティック積分のドリフトが小さいことを確認する
    /// (docs/21-verification/01-analytic-tests.md A2 注記: 10⁴周縮約は分級/通常CI)。
    #[test]
    fn a2_two_body_energy_and_angular_momentum_drift_stays_small_over_many_orbits() {
        let mass_sun = 1.989e30;
        let r = 1.496e11;
        let mut sys = NBodySystem::new(0.0);
        sys.add_body(Vec3::ZERO, Vec3::ZERO, mass_sun);
        let v_circ = (GRAVITATIONAL_CONSTANT * mass_sun / r).sqrt();
        let idx = sys.add_body(Vec3::new(r, 0.0, 0.0), Vec3::new(0.0, v_circ, 0.0), 1.0);

        let period =
            2.0 * std::f64::consts::PI * (r.powi(3) / (GRAVITATIONAL_CONSTANT * mass_sun)).sqrt();
        let steps_per_orbit = 1000u32;
        let dt = period / steps_per_orbit as f64;
        let orbits = 100u32;

        let e0 = sys.total_energy().total();
        let l0 = sys.position[idx].cross(sys.velocity[idx]).length();

        step_n(&mut sys, dt, steps_per_orbit * orbits);

        let e1 = sys.total_energy().total();
        let l1 = sys.position[idx].cross(sys.velocity[idx]).length();

        let e_drift = (e1 - e0).abs() / e0.abs();
        let l_drift = (l1 - l0).abs() / l0;
        assert!(
            e_drift < 1e-6,
            "energy drift {e_drift} over {orbits} orbits"
        );
        assert!(
            l_drift < 1e-9,
            "angular momentum drift {l_drift} over {orbits} orbits"
        );
    }

    /// A7: 三体カオス決定論 — 同一初期条件を2回実行すると状態ハッシュが厳密一致する
    /// (docs/21-verification/01-analytic-tests.md A7)。
    #[test]
    fn a7_three_body_chaos_is_deterministic_across_runs() {
        let run = || {
            let mut sys = NBodySystem::new(1e9); // 弱いソフトニングで近接発散を避ける
            sys.add_body(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0), 1.0e30);
            sys.add_body(
                Vec3::new(1.0e11, 0.0, 0.0),
                Vec3::new(0.0, 2.0e4, 0.0),
                5.0e29,
            );
            sys.add_body(
                Vec3::new(-0.6e11, 0.8e11, 0.0),
                Vec3::new(-1.5e4, -1.0e4, 0.0),
                3.0e29,
            );
            step_n(&mut sys, 3600.0, 2000);
            let mut hasher = StateHasher::new();
            sys.state_hash(&mut hasher);
            hasher.finish()
        };
        assert_eq!(run(), run());
    }
}
