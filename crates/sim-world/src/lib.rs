//! World facade。設計: docs/00-foundation/04-architecture.md §1.1、
//!       docs/20-integration/04-world-api.md。
//!
//! Phase A 時点では `create_body` による複数剛体の構築 + `MechanicsSolver` 駆動を
//! 正式な `RigidBodySet` 経由で提供する縮小版。フル API(joint/circuit/fluid region/
//! Coupling、コマンドキュー、スナップショット、シーン JSON)は後続の増分で
//! docs/20-integration/04-world-api.md §2 に沿って拡張する。
//!
//! `create_body`/`remove_body`/`body_position` は `sim_core::BodyId`(世代付き index)を
//! 使う(設計 docs/00-foundation/04-architecture.md §3「削除済み ID へのアクセスは
//! `None`」)。世代は `World` 層で管理する — `sim_mechanics::RigidBodySet` 自体はまだ
//! スロットの削除・再利用に対応していないため(密な `Vec` ベースで、削除は配列の
//! 詰め直しか tombstone 化を要する大きめの改修になる)、`remove_body` は下層スロットを
//! 「無効化」(`BodyType::Static` 化 + 遠方(y=-1e9)へ退避 + 速度ゼロ化)するに留め、世代
//! カウンタだけを正式にインクリメントして以後のアクセスを `None` にする。ジョイント・
//! 結合の連鎖削除(設計 §2 の `remove_body` 完全仕様)は、`World` がまだジョイント・
//! Coupling を保持していないため対象外(それらの導入時に合わせて拡張する)。
//! `EnergyLedger`(docs/00-foundation/04-architecture.md §1.1.2(2))は P1 で導入済み:
//! シーン構築(`create_body` 呼び出し列)が終わり最初の `step()` が呼ばれた時点の
//! 合計エネルギーを基準点として、以後毎 step 後に記帳する(構築途中の`create_body`
//! 呼び出し自体は台帳上の「エネルギーの出現」として扱わない)。

use sim_core::{EnergyLedger, EventQueue, MaterialDb, Solver, SolverContext, StateHasher};
use sim_math::{SimRng, Vec3};
use sim_mechanics::{BodyType, MechanicsSolver, RigidBodyDesc};

// 下流crate(sim-wasm等)が別途sim-core依存を追加しなくてもBodyIdを使えるよう、
// Worldの公開APIとしてそのまま再エクスポートする。
pub use sim_core::BodyId;

/// World 生成オプション。剛体はここでは作らず `create_body` で追加する。
pub struct WorldOptions {
    pub gravity: f64,
    pub dt: f64,
    pub seed: u64,
}

impl Default for WorldOptions {
    fn default() -> Self {
        WorldOptions {
            gravity: 9.80665,
            dt: 1.0 / 120.0,
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
    /// 最初の `step()` で遅延初期化する(構築フェーズの `create_body` を
    /// 台帳の基準点計算に含めないため)。
    ledger: Option<EnergyLedger>,
    /// `BodyId` の世代管理(`RigidBodySet` のインデックスに対応、モジュールdoc参照)。
    generations: Vec<u32>,
}

const STREAM_DIAG: u64 = 0;
/// エネルギー台帳の代表エネルギー(ゼロ初期エネルギー対策の下限)。設計
/// docs/21-verification/02-conservation-laws.md §2 の E_scale。シーンごとの代表値を求める
/// API はまだ無いため、P1 では固定値 1 J とする(将来シーン記述に応じて拡張)。
const ENERGY_SCALE_FLOOR: f64 = 1.0;

impl World {
    pub fn new(options: WorldOptions) -> World {
        World {
            clock: sim_core::SimClock::new(options.dt),
            mechanics: MechanicsSolver::new(options.gravity),
            materials: MaterialDb::standard(),
            rng: SimRng::new(options.seed, STREAM_DIAG),
            events: EventQueue::new(),
            ledger: None,
            generations: Vec::new(),
        }
    }

    /// 全ドメインが読む物性データベース(設計 §1.1.5)。`create_body` に渡す
    /// `MaterialId` の解決に使う。
    pub fn materials(&self) -> &MaterialDb {
        &self.materials
    }

    /// 剛体を追加する。設計 docs/20-integration/04-world-api.md §2 の `create_body`。
    pub fn create_body(&mut self, desc: RigidBodyDesc) -> BodyId {
        let index = self.mechanics.create_body(desc, &self.materials);
        debug_assert_eq!(
            index,
            self.generations.len(),
            "RigidBodySet is expected to only grow (no slot reuse yet, module doc)"
        );
        self.generations.push(0);
        BodyId {
            index: index as u32,
            generation: 0,
        }
    }

    fn is_valid(&self, id: BodyId) -> bool {
        (id.index as usize) < self.generations.len()
            && self.generations[id.index as usize] == id.generation
    }

    /// ボディを削除する。世代カウンタをインクリメントし、以後この `id` (と古い世代の
    /// 再利用)へのアクセスは `None` になる(設計の不変条件)。下層の `RigidBodySet`
    /// スロット自体はまだ真に解放されない(モジュールdoc参照) — 無効化として
    /// `BodyType::Static` 化・遠方への退避・速度ゼロ化を行い、実質的な影響を無くす。
    pub fn remove_body(&mut self, id: BodyId) {
        if !self.is_valid(id) {
            return;
        }
        let idx = id.index as usize;
        self.generations[idx] += 1;
        self.mechanics.bodies.body_type[idx] = BodyType::Static;
        self.mechanics.bodies.position[idx] = Vec3::new(0.0, -1.0e9, 0.0);
        self.mechanics.bodies.linear_velocity[idx] = Vec3::ZERO;
        self.mechanics.bodies.angular_velocity[idx] = Vec3::ZERO;
    }

    /// 直接可変アクセス(抗力・浮力の周囲媒質設定など)。設計が定める
    /// 「書き込みはコマンド経由」規律の対象は実行中の状態変更であり、
    /// シーン構築時の設定はこの限りでない(§1 設計原則)。
    pub fn mechanics_mut(&mut self) -> &mut MechanicsSolver {
        &mut self.mechanics
    }

    /// 1 world step(固定 dt)。docs/20-integration/04-world-api.md §2 の `step()`。
    pub fn step(&mut self) {
        if self.ledger.is_none() {
            self.ledger = Some(EnergyLedger::new(self.mechanics.total_energy().total()));
        }
        let dt = self.clock.dt();
        let mut ctx = SolverContext {
            materials: &self.materials,
            rng: &mut self.rng,
            events: &mut self.events,
        };
        self.mechanics.step(dt, &mut ctx);
        let _ = self.events.drain_sorted(); // Phase A: 購読者未実装のため排出のみ。
        self.ledger
            .as_mut()
            .expect("initialized above")
            .record(self.mechanics.total_energy().total(), ENERGY_SCALE_FLOOR);
        self.clock.advance();
    }

    /// 直近の記帳残差(設計 docs/21-verification/02-conservation-laws.md §2)。
    /// トレンド監視指標であり、単発のバグ検出には使わない(ドメイン別保存則テストが担う)。
    /// `step()` を一度も呼んでいない場合は 0。
    pub fn energy_residual(&self) -> f64 {
        self.ledger.as_ref().map_or(0.0, |l| l.latest_residual())
    }

    pub fn energy_residual_history(&self) -> &[f64] {
        self.ledger.as_ref().map_or(&[], |l| l.residual_history())
    }

    pub fn time(&self) -> f64 {
        self.clock.time()
    }

    pub fn step_count(&self) -> u64 {
        self.clock.step_count()
    }

    /// 設計 docs/00-foundation/04-architecture.md §3「削除済み ID へのアクセスは `None`
    /// (パニックしない)」。
    pub fn body_position(&self, id: BodyId) -> Option<Vec3> {
        if !self.is_valid(id) {
            return None;
        }
        Some(self.mechanics.bodies.position[id.index as usize])
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
    use sim_math::Transform;
    use sim_mechanics::{BodyType, Shape};

    const INITIAL_HEIGHT: f64 = 10.0;

    /// Phase 0 相当の「箱1個が落ちる」シーンを構築する(鋼の箱、高さ `INITIAL_HEIGHT`)。
    fn create_falling_box(world: &mut World) -> BodyId {
        let steel = world
            .materials()
            .find_by_name("鋼(炭素鋼)")
            .expect("standard DB has steel");
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(0.5, 0.5, 0.5),
            },
            steel,
        );
        desc.body_type = BodyType::Dynamic;
        desc.transform = Transform {
            position: Vec3::new(0.0, INITIAL_HEIGHT, 0.0),
            rotation: sim_math::Quat::IDENTITY,
        };
        world.create_body(desc)
    }

    #[test]
    fn box_falls_and_test_is_green() {
        let mut world = World::new(WorldOptions::default());
        let idx = create_falling_box(&mut world);
        let y0 = world.body_position(idx).unwrap().y;
        for _ in 0..120 {
            world.step();
        }
        assert!(world.body_position(idx).unwrap().y < y0);
        assert_eq!(world.step_count(), 120);
    }

    /// 複数剛体: create_body を複数回呼んでも各 body が独立に扱えること。
    #[test]
    fn multiple_bodies_are_independently_addressable() {
        let mut world = World::new(WorldOptions::default());
        let a = create_falling_box(&mut world);
        let steel = world.materials().find_by_name("鋼(炭素鋼)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.2 }, steel);
        desc.body_type = BodyType::Static; // 静止参照点(比較用)
        desc.transform.position = Vec3::new(3.0, 2.0, 0.0);
        let b = world.create_body(desc);

        for _ in 0..60 {
            world.step();
        }
        assert!(
            world.body_position(a).unwrap().y < INITIAL_HEIGHT,
            "a should fall"
        );
        assert_eq!(
            world.body_position(b).unwrap(),
            Vec3::new(3.0, 2.0, 0.0),
            "static body must not move"
        );
    }

    /// 世代付き`BodyId`(設計 docs/00-foundation/04-architecture.md §3)の不変条件:
    /// 削除済み ID へのアクセスは `None`(パニックしない)。同じインデックスへの新規
    /// `create_body`(現時点では `RigidBodySet` がスロット再利用に未対応のため実際には
    /// 新しいインデックスになるが、`is_valid`の世代比較ロジック自体はどちらの場合も
    /// 正しく機能する)。
    #[test]
    fn removed_body_id_returns_none_and_does_not_panic() {
        let mut world = World::new(WorldOptions::default());
        let a = create_falling_box(&mut world);
        assert!(world.body_position(a).is_some());

        world.remove_body(a);
        assert!(
            world.body_position(a).is_none(),
            "removed body id must resolve to None, not panic"
        );

        // 削除後も他のボディ・ステップ実行は正常に動作する(パニックしない)。
        let b = create_falling_box(&mut world);
        for _ in 0..10 {
            world.step();
        }
        assert!(world.body_position(b).is_some());
        assert!(world.body_position(a).is_none());
    }

    /// 未知(存在しない index)の`BodyId`も`None`(パニックしない)。
    #[test]
    fn unknown_body_id_returns_none() {
        let world = World::new(WorldOptions::default());
        let bogus = BodyId {
            index: 999,
            generation: 0,
        };
        assert!(world.body_position(bogus).is_none());
    }

    /// 決定論テスト(階層1): 同一初期条件 → 同数ステップ後のハッシュが一致する。
    /// 設計: docs/20-integration/02-determinism-replay.md §5/§6。
    #[test]
    fn determinism_same_scenario_twice_matches_hash() {
        let run = || {
            let mut world = World::new(WorldOptions::default());
            create_falling_box(&mut world);
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
    /// (台帳は最初の `step()` で遅延初期化するため、`create_body` はここでは計上されない。)
    #[test]
    fn energy_ledger_residual_matches_analytic_symplectic_drift() {
        let options = WorldOptions::default();
        let (g, dt) = (options.gravity, options.dt);
        let n = 100u32;

        let mut world = World::new(options);
        create_falling_box(&mut world);
        for _ in 0..n {
            world.step();
        }

        let expected = n as f64 * 0.5 * g * dt * dt / INITIAL_HEIGHT;
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
