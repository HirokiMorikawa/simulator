//! 静電場・点電荷系(直接和 + Boris pusher)。設計: docs/13-electromagnetism/01-electrostatics-magnetostatics.md。
//!
//! P4 スコープの最小実装: 点電荷の直接和クーロン力($O(N^2)$、設計 §4「数十源で十分」)+
//! 一様外部場(地磁気等)。運動積分は設計 §4 が明記する通り Boris pusher
//! (sim_math::BorisPusher、01-math/03-integrators.md §2.6)を用いる — クーロン力・一様外場を
//! 合成した電場を各粒子位置で評価し、磁場回転はノルム保存の厳密回転で扱う。
//! 磁気双極子・鏡像力・摩擦帯電・放電イベントは Phase 4 残りとして未実装。

use sim_core::{EnergyBreakdown, Solver, SolverContext, StateHasher};
use sim_math::{BorisPusher, Vec3};

/// 真空の誘電率 [F/m](CODATA)。
pub const VACUUM_PERMITTIVITY: f64 = 8.8541878128e-12;

/// クーロン定数 $k=1/(4\pi\varepsilon_0)$ [N m^2/C^2]。
pub const COULOMB_CONSTANT: f64 = 1.0 / (4.0 * std::f64::consts::PI * VACUUM_PERMITTIVITY);

/// 一様外部場(地磁気など)。設計 §3 `UniformField`。
#[derive(Clone, Copy, Debug, Default)]
pub struct UniformField {
    pub e: Vec3,
    pub b: Vec3,
}

/// 点電荷系。設計 §3 `ChargedBody` の集合版(P0 スコープでは剛体と未結合の独立粒子)。
#[derive(Clone)]
pub struct PointChargeSystem {
    pub position: Vec<Vec3>,
    pub velocity: Vec<Vec3>,
    pub mass: Vec<f64>,
    pub charge: Vec<f64>,
    pub uniform_field: UniformField,
}

impl PointChargeSystem {
    pub fn new(uniform_field: UniformField) -> PointChargeSystem {
        PointChargeSystem {
            position: Vec::new(),
            velocity: Vec::new(),
            mass: Vec::new(),
            charge: Vec::new(),
            uniform_field,
        }
    }

    pub fn add_particle(
        &mut self,
        position: Vec3,
        velocity: Vec3,
        mass: f64,
        charge: f64,
    ) -> usize {
        let idx = self.position.len();
        self.position.push(position);
        self.velocity.push(velocity);
        self.mass.push(mass);
        self.charge.push(charge);
        idx
    }

    pub fn len(&self) -> usize {
        self.position.len()
    }

    pub fn is_empty(&self) -> bool {
        self.position.is_empty()
    }

    /// 粒子 `i` の位置における電場: 他の全電荷からの直接和(クーロンの法則) + 一様外場。
    /// 設計 §2 の点電荷解 $\mathbf{E}=\frac{q}{4\pi\varepsilon_0 r^2}\hat r$。
    pub fn electric_field_at(&self, i: usize) -> Vec3 {
        let mut e = self.uniform_field.e;
        for j in 0..self.len() {
            if i == j {
                continue;
            }
            let d = self.position[i] - self.position[j];
            let dist_sq = d.length_sq();
            let dist = dist_sq.sqrt();
            let factor = COULOMB_CONSTANT * self.charge[j] / (dist_sq * dist);
            e = e.addcarry_scaled(d, factor);
        }
        e
    }
}

impl Solver for PointChargeSystem {
    /// Boris pusher は磁場回転を厳密回転で扱うため明示的な CFL 条件を持たない
    /// (sim-astro の leapfrog と同じ理由付け、設計 §4 に安定性条件の言及なし)。
    fn max_stable_dt(&self) -> f64 {
        f64::INFINITY
    }

    fn step(&mut self, dt: f64, _ctx: &mut SolverContext) {
        let pusher = BorisPusher;
        for i in 0..self.len() {
            let e = self.electric_field_at(i);
            let b = self.uniform_field.b;
            let q_over_m = self.charge[i] / self.mass[i];
            pusher.step(
                &mut self.position[i],
                &mut self.velocity[i],
                q_over_m,
                e,
                b,
                dt,
            );
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

    /// 運動エネルギー + クーロンポテンシャル(対ごとに1回、設計 §4「台帳の em_field に記帳」)。
    /// 一様外場のポテンシャルは(基準点が系の外にあり自明に定義できないため)計上しない。
    fn total_energy(&self) -> EnergyBreakdown {
        let n = self.len();
        let mut kinetic = 0.0;
        for i in 0..n {
            kinetic += 0.5 * self.mass[i] * self.velocity[i].length_sq();
        }
        let mut electromagnetic = 0.0;
        for i in 0..n {
            for j in (i + 1)..n {
                let dist = (self.position[j] - self.position[i]).length();
                electromagnetic += COULOMB_CONSTANT * self.charge[i] * self.charge[j] / dist;
            }
        }
        EnergyBreakdown {
            kinetic,
            electromagnetic,
            ..Default::default()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::{EventQueue, MaterialDb};
    use sim_math::SimRng;

    /// E1: 逆二乗則 — 2点電荷の力 $F=kq_1q_2/r^2$、直接和なので機械精度で一致する
    /// (docs/21-verification/01-analytic-tests.md E1: "力の代数検算")。
    #[test]
    fn e1_coulomb_force_matches_inverse_square_law_at_machine_precision() {
        let mut sys = PointChargeSystem::new(UniformField::default());
        let q1 = 2.0e-6;
        let q2 = -3.0e-6;
        let r = 0.5;
        sys.add_particle(Vec3::ZERO, Vec3::ZERO, 1.0, q1);
        let idx2 = sys.add_particle(Vec3::new(r, 0.0, 0.0), Vec3::ZERO, 1.0, q2);

        let e_at_2 = sys.electric_field_at(idx2);
        let force_on_2 = e_at_2.scale(q2);

        let expected_magnitude = COULOMB_CONSTANT * q1.abs() * q2.abs() / (r * r);
        let rel_err = (force_on_2.length() - expected_magnitude).abs() / expected_magnitude;
        assert!(rel_err < 1e-12, "rel_err={rel_err}");
        // 異符号電荷は引力: 粒子2に働く力は -x 方向(粒子1へ向かう)。
        assert!(force_on_2.x < 0.0, "force_on_2={force_on_2:?}");
    }

    /// E2: サイクロトロン半径 $r=mv/(qB)$、rel 0.5%/abs 1e-9(速さは Boris pusher の
    /// 厳密回転により丸め誤差を除き不変、docs/21-verification/01-analytic-tests.md E2)。
    /// sim_math::BorisPusher 自体は既に "E2相当" として検証済み(crates/sim-math/src/integrators.rs)
    /// だが、ここでは sim-em の公開 API(`PointChargeSystem` + クーロン力との合成場)を
    /// 通した経路として改めて記録する。
    #[test]
    fn e2_cyclotron_radius_matches_mv_over_qb() {
        let b_field = 0.02; // T
        let mass = 9.1093837015e-31; // 電子質量相当(スケール確認用、実測値である必要はない)
        let charge = 1.602176634e-19;
        let speed = 2.0e5; // m/s

        let mut sys = PointChargeSystem::new(UniformField {
            e: Vec3::ZERO,
            b: Vec3::new(0.0, 0.0, b_field),
        });
        let idx = sys.add_particle(Vec3::ZERO, Vec3::new(speed, 0.0, 0.0), mass, charge);

        let omega_c = (charge * b_field / mass).abs();
        let period = 2.0 * std::f64::consts::PI / omega_c;
        let steps = 2000u32;
        let dt = period / steps as f64;

        let materials = MaterialDb::standard();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();
        let mut max_r: f64 = 0.0;
        for _ in 0..steps {
            let mut ctx = SolverContext {
                materials: &materials,
                rng: &mut rng,
                events: &mut events,
            };
            sys.step(dt, &mut ctx);
            max_r = max_r.max(sys.position[idx].length());
        }

        // 出発点は円周上にあるため、出発点からの最大距離は半周後に到達する直径 2r
        // (中心からの半径 r ではない)。
        let expected_r = mass * speed / (charge * b_field).abs();
        let measured_r = max_r / 2.0;
        let radius_rel_err = (measured_r - expected_r).abs() / expected_r;
        assert!(
            radius_rel_err < 0.005,
            "measured_r={measured_r} expected_r={expected_r} radius_rel_err={radius_rel_err}"
        );

        let final_speed = sys.velocity[idx].length();
        let speed_rel_err = (final_speed - speed).abs() / speed;
        assert!(speed_rel_err < 0.005, "speed_rel_err={speed_rel_err}");
    }
}
