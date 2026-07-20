//! World facade。設計: docs/00-foundation/04-architecture.md §1.1、
//!       docs/20-integration/04-world-api.md。
//!
//! Phase A 時点では最小 World(1 剛体・重力のみ)を正式な `MechanicsSolver`/
//! `RigidBodySet` 経由で動かす縮小版。フル API(create_body/joint/circuit/...、
//! コマンドキュー、スナップショット、`Coupling`/`EnergyLedger`)は後続の増分で
//! docs/20-integration/04-world-api.md §2 に沿って拡張する。

use sim_core::{EventQueue, MaterialDb, Solver, SolverContext, StateHasher};
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
}

const STREAM_DIAG: u64 = 0;

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

        World {
            clock: sim_core::SimClock::new(options.dt),
            mechanics,
            materials,
            rng: SimRng::new(options.seed, STREAM_DIAG),
            events: EventQueue::new(),
            box_body,
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
        self.clock.advance();
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
}
