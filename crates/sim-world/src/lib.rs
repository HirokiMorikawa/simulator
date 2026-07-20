//! World facade。設計: docs/00-foundation/04-architecture.md §1.1、
//!       docs/20-integration/04-world-api.md。
//!
//! Phase 0 は完了条件(docs/22-roadmap/01-phases.md Phase 0)である
//! 「箱 1 個が落ちる最小 World」だけを満たす縮小版。フル API
//! (create_body/joint/circuit/... 、コマンドキュー、スナップショット等)は
//! Phase A 以降で docs/20-integration/04-world-api.md §2 に沿って拡張する。

use sim_core::{SimClock, StateHasher};
use sim_math::Vec3;
use sim_mechanics::FallingBody;

/// Phase 0 の World 生成オプション。
pub struct WorldOptions {
    pub gravity: f64,
    pub dt: f64,
    pub initial_position: Vec3,
}

impl Default for WorldOptions {
    fn default() -> Self {
        WorldOptions {
            gravity: 9.80665,
            dt: 1.0 / 120.0,
            initial_position: Vec3::new(0.0, 10.0, 0.0),
        }
    }
}

/// シミュレートされた環境そのもの(Phase 0 縮小版)。
/// 世界時刻の一意性は `clock`(docs/00-foundation/04-architecture.md §1.1.2(4))、
/// 状態オーナーシップの一意性は `body` が正典状態を保持することで満たす(同 §1.1.2(1))。
pub struct World {
    clock: SimClock,
    gravity: f64,
    body: FallingBody,
}

impl World {
    pub fn new(options: WorldOptions) -> World {
        World {
            clock: SimClock::new(options.dt),
            gravity: options.gravity,
            body: FallingBody::new(options.initial_position),
        }
    }

    /// 1 world step(固定 dt)。docs/20-integration/04-world-api.md §2 の `step()`。
    pub fn step(&mut self) {
        self.body.step(self.gravity, self.clock.dt());
        self.clock.advance();
    }

    pub fn time(&self) -> f64 {
        self.clock.time()
    }

    pub fn step_count(&self) -> u64 {
        self.clock.step_count()
    }

    pub fn body_position(&self) -> Vec3 {
        self.body.position
    }

    /// 全状態(clock + body)を決定的順序でハッシュする。
    /// 設計: docs/20-integration/02-determinism-replay.md §3。
    pub fn state_hash(&self) -> u64 {
        let mut hasher = StateHasher::new();
        hasher.write_u64(self.clock.step_count());
        hasher.write_f64(self.clock.time());
        hasher.write_vec3(self.body.position);
        hasher.write_vec3(self.body.velocity);
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
