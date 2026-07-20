//! World facade。設計: docs/00-foundation/04-architecture.md §1.1、
//!       docs/20-integration/04-world-api.md。
//!
//! Phase A 時点では最小 World(1 剛体・重力のみ)を正式な `MechanicsSolver`/
//! `RigidBodySet` 経由で動かす縮小版。フル API(create_body/joint/circuit/...、
//! コマンドキュー、スナップショット、`Coupling`)は後続の増分で
//! docs/20-integration/04-world-api.md §2 に沿って拡張する。
//! `EnergyLedger`(docs/00-foundation/04-architecture.md §1.1.2(2))は P1 で導入済み:
//! 毎 step 後に mechanics ドメインの `total_energy()` を記帳する。

use sim_core::{EnergyLedger, EventQueue, MaterialDb, Solver, SolverContext, StateHasher};
use sim_math::{SimRng, Transform, Vec3};
use sim_mechanics::{BodyType, MechanicsSolver, RigidBodyDesc, Shape};

/// World 生成オプション。
pub struct WorldOptions {
    pub gravity: f64,
    pub dt: f64,
    pub initial_position: Vec3,
    pub seed: u64,
}

impl Default for WorldOptions {
    fn default() -> Self {
        WorldOptions {
            gravity: 9.80665,
            dt: 1.0 / 120.0,
            initial_position: Vec3::new(0.0, 10.0, 0.0),
            seed: 0,
        }
    }
}

/// シミュレートされた環境そのもの。世界時刻の一意性は `clock`
/// (docs/00-foundation/04-architecture.md §1.1.2(4))、状態オーナーシップの一意性は
/// `mechanics`(正典状態)が保持することで満たす(同 §1.1.2(1))。
pub struct World {
    clock: sim_core::SimClock,
    mechanics: MechanicsSolver,
    materials: MaterialDb,
    rng: SimRng,
    events: EventQueue,
    box_body: usize,
    ledger: EnergyLedger,
}

const STREAM_DIAG: u64 = 0;
/// エネルギー台帳の代表エネルギー(ゼロ初期エネルギー対策の下限)。設計
/// docs/21-verification/02-conservation-laws.md §2 の E_scale。シーンごとの代表値を求める
/// API はまだ無いため、P1 では固定値 1 J とする(将来シーン記述に応じて拡張)。
const ENERGY_SCALE_FLOOR: f64 = 1.0;

impl World {
    pub fn new(options: WorldOptions) -> World {
        let materials = MaterialDb::standard();
        let steel = materials
            .find_by_name("鋼(炭素鋼)")
            .expect("standard DB has steel");
        let mut mechanics = MechanicsSolver::new(options.gravity);
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(0.5, 0.5, 0.5),
            },
            steel,
        );
        desc.body_type = BodyType::Dynamic;
        desc.transform = Transform {
            position: options.initial_position,
            rotation: sim_math::Quat::IDENTITY,
        };
        let box_body = mechanics.create_body(desc, &materials);
        let initial_energy = mechanics.total_energy().total();

        World {
            clock: sim_core::SimClock::new(options.dt),
            mechanics,
            materials,
            rng: SimRng::new(options.seed, STREAM_DIAG),
            events: EventQueue::new(),
            box_body,
            ledger: EnergyLedger::new(initial_energy),
        }
    }

    /// 1 world step(固定 dt)。docs/20-integration/04-world-api.md §2 の `step()`。
    pub fn step(&mut self) {
        let dt = self.clock.dt();
        let mut ctx = SolverContext {
            materials: &self.materials,
            rng: &mut self.rng,
            events: &mut self.events,
        };
        self.mechanics.step(dt, &mut ctx);
        let _ = self.events.drain_sorted(); // Phase A: 購読者未実装のため排出のみ。
        self.ledger
            .record(self.mechanics.total_energy().total(), ENERGY_SCALE_FLOOR);
        self.clock.advance();
    }

    /// 直近の記帳残差(設計 docs/21-verification/02-conservation-laws.md §2)。
    /// トレンド監視指標であり、単発のバグ検出には使わない(ドメイン別保存則テストが担う)。
    pub fn energy_residual(&self) -> f64 {
        self.ledger.latest_residual()
    }

    pub fn energy_residual_history(&self) -> &[f64] {
        self.ledger.residual_history()
    }

    pub fn time(&self) -> f64 {
        self.clock.time()
    }

    pub fn step_count(&self) -> u64 {
        self.clock.step_count()
    }

    pub fn body_position(&self) -> Vec3 {
        self.mechanics.bodies.position[self.box_body]
    }

    /// 全状態(clock + mechanics)を決定的順序でハッシュする。
    /// 設計: docs/20-integration/02-determinism-replay.md §3。
    pub fn state_hash(&self) -> u64 {
        let mut hasher = StateHasher::new();
        hasher.write_u64(self.clock.step_count());
        hasher.write_f64(self.clock.time());
        self.mechanics.state_hash(&mut hasher);
        hasher.finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn box_falls_and_test_is_green() {
        let mut world = World::new(WorldOptions::default());
        let y0 = world.body_position().y;
        for _ in 0..120 {
            world.step();
        }
        assert!(world.body_position().y < y0);
        assert_eq!(world.step_count(), 120);
    }

    /// 決定論テスト(階層1): 同一初期条件 → 同数ステップ後のハッシュが一致する。
    /// 設計: docs/20-integration/02-determinism-replay.md §5/§6。
    #[test]
    fn determinism_same_scenario_twice_matches_hash() {
        let run = || {
            let mut world = World::new(WorldOptions::default());
            for _ in 0..300 {
                world.step();
            }
            world.state_hash()
        };
        let hash_a = run();
        let hash_b = run();
        assert_eq!(hash_a, hash_b);
    }

    /// エネルギー台帳: 接触なし自由落下では semi-implicit Euler が定数外力(一様重力)に対して
    /// 1 step あたり厳密に `-0.5 m g^2 dt^2` の力学的エネルギー損失を持つ(周期軌道でないため
    /// symplectic 特有の有界誤差ではなく、線形ドリフトになる — 既知の積分器由来のドリフトで
    /// あり不明な漏れではない)。E(0)=m g h0 が ENERGY_SCALE_FLOOR を大きく上回るので
    /// residual の scale は E(0) に決まり、質量 m が式から消える:
    /// residual(N) = N * 0.5 * g * dt^2 / h0。台帳の記帳がこの解析予測と一致することを検証する。
    #[test]
    fn energy_ledger_residual_matches_analytic_symplectic_drift() {
        let options = WorldOptions::default();
        let (g, dt, h0) = (options.gravity, options.dt, options.initial_position.y);
        let n = 100u32;

        let mut world = World::new(WorldOptions::default());
        for _ in 0..n {
            world.step();
        }

        let expected = n as f64 * 0.5 * g * dt * dt / h0;
        let measured = world.energy_residual();
        assert!(
            (measured - expected).abs() / expected < 1e-6,
            "measured={measured} expected={expected}"
        );
        assert_eq!(world.energy_residual_history().len(), n as usize);
        // 外力なし・接触なしの単調な力学的エネルギー減少なので残差は単調非減少のはず。
        assert!(world.energy_residual_history()[0] <= measured);
    }
}
