//! マニフォールド持続化(設計 docs/10-mechanics/02-collision-detection.md §4.7)の
//! 受け入れテスト。**群9で実装**。
//!
//! 移行前は feature_id が一致しさえすれば無条件に前ステップの累積インパルスを
//! 引き継いでいた(=接触点が別の場所へ移っていても古いインパルスを適用していた)。
//! `MechanicsSolver::set_manifold_persistence(false)` で移行前の挙動へ戻せるため、
//! ここでは**同一シーン・同一乱数での対照実験**として両者を並べて計測する。

use sim_core::{Event, EventQueue, MaterialDb, MaterialId, Solver, SolverContext};
use sim_math::{SimRng, Vec3};
use sim_mechanics::{BodyType, MechanicsSolver, RigidBodyDesc, Shape};

/// docs/10-mechanics/03-contact-solver.md §9 の既定 slop。
const SLOP: f64 = 0.005;

fn ground_plane(solver: &mut MechanicsSolver, materials: &MaterialDb, material: MaterialId) {
    let ground = RigidBodyDesc {
        body_type: BodyType::Static,
        ..RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            material,
        )
    };
    solver.create_body(ground, materials);
}

fn step_n(solver: &mut MechanicsSolver, materials: &MaterialDb, dt: f64, steps: usize) {
    let mut rng = SimRng::new(1, 1);
    let mut events = EventQueue::new();
    for _ in 0..steps {
        let mut ctx = SolverContext {
            materials,
            rng: &mut rng,
            events: &mut events,
        };
        solver.step(dt, &mut ctx);
        let _: Vec<Event> = events.drain_sorted();
    }
}

/// M12(4段の木箱スタック)を 10 秒回し、隣接ペアごとの貫入量を返す。
fn four_box_stack_penetrations(persistence: bool) -> Vec<f64> {
    let materials = MaterialDb::standard();
    let wood = materials.find_by_name("木材(松)").unwrap();
    let mut solver = MechanicsSolver::new(9.80665);
    solver.set_manifold_persistence(persistence);
    ground_plane(&mut solver, &materials, wood);

    let half = 0.5;
    let mut box_indices = Vec::new();
    for level in 0..4 {
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(half, half, half),
            },
            wood,
        );
        desc.transform.position = Vec3::new(0.0, half + level as f64 * 2.0 * half, 0.0);
        box_indices.push(solver.create_body(desc, &materials));
    }

    step_n(&mut solver, &materials, 1.0 / 120.0, 1200);

    let mut penetrations = Vec::new();
    let mut below_top = 0.0;
    for &idx in &box_indices {
        let bottom = solver.bodies.position[idx].y - half;
        penetrations.push(below_top - bottom);
        below_top = solver.bodies.position[idx].y + half;
    }
    penetrations
}

/// **対照実験**: マニフォールド持続化が M12(4段スタック)の貫入量を実際に減らすことを、
/// 移行前の挙動(`set_manifold_persistence(false)`)と並べて確認する。
///
/// 移行前の実測値は既に記録されていた(docs/22-roadmap/02-feature-checklist.md の Q4:
/// 0.00226/0.00353/0.00378/0.00468、最上段が slop=0.005 の 93.5%)。持続化オフの側が
/// その値を再現することも同時に確認し、対照が正しく「移行前」を表していることを保証する。
#[test]
fn manifold_persistence_reduces_stack_penetration_compared_to_unconditional_warm_start() {
    let without = four_box_stack_penetrations(false);
    let with = four_box_stack_penetrations(true);

    // 対照側が Q4 の記録値を再現していること(= 対照が本当に「移行前」であること)。
    let recorded = [0.0022612, 0.0035274, 0.0037759, 0.0046779];
    for (measured, expected) in without.iter().zip(recorded.iter()) {
        assert!(
            (measured - expected).abs() < 1e-6,
            "control run must reproduce the recorded pre-migration penetrations: \
             measured={without:?} recorded={recorded:?}"
        );
    }

    let max_without = without.iter().cloned().fold(f64::MIN, f64::max);
    let max_with = with.iter().cloned().fold(f64::MIN, f64::max);
    assert!(
        max_with < 0.5 * max_without,
        "manifold persistence must substantially reduce the worst penetration: \
         without={without:?} (max {max_without}) with={with:?} (max {max_with})"
    );
    assert!(
        max_with < SLOP,
        "penetration {max_with} must stay below slop {SLOP}"
    );
    // 全ペアで悪化していないこと(平均だけ改善して局所的に悪化する、を許さない)。
    for (a, b) in without.iter().zip(with.iter()) {
        assert!(
            b <= a,
            "no contact pair may get worse: without={without:?} with={with:?}"
        );
    }
}

/// **GC**: 接触が完全に消えたボディ対のキャッシュエントリが捨てられること
/// (設計 §4.7 のマニフォールド持続化。移行前はキーが増え続けるだけで削除されなかった)。
///
/// 箱を地面に落として静止させ(キャッシュにエントリが載る)、そのあと箱を遠方へ
/// テレポートさせて接触を絶つ。持続化オンならキャッシュは空になり、オフなら
/// 移行前どおり残り続ける。
#[test]
fn manifold_cache_is_garbage_collected_when_contact_ends() {
    let materials = MaterialDb::standard();
    let wood = materials.find_by_name("木材(松)").unwrap();

    for persistence in [true, false] {
        let mut solver = MechanicsSolver::new(9.80665);
        solver.set_manifold_persistence(persistence);
        ground_plane(&mut solver, &materials, wood);

        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(0.5, 0.5, 0.5),
            },
            wood,
        );
        desc.transform.position = Vec3::new(0.0, 0.5, 0.0);
        let body = solver.create_body(desc, &materials);

        step_n(&mut solver, &materials, 1.0 / 120.0, 120);
        assert!(
            solver.cached_contact_point_count() > 0,
            "contact points must be cached while touching (persistence={persistence})"
        );

        // 接触を絶つ。スリープ中だと接触解決自体が走らないので同時に起こしておく。
        solver.bodies.position[body] = Vec3::new(0.0, 50.0, 0.0);
        solver.bodies.asleep[body] = false;
        step_n(&mut solver, &materials, 1.0 / 120.0, 10);

        if persistence {
            assert_eq!(
                solver.cached_contact_point_count(),
                0,
                "stale entries must be collected once the pair stops touching"
            );
        } else {
            assert!(
                solver.cached_contact_point_count() > 0,
                "the pre-migration behaviour keeps stale entries forever (control)"
            );
        }
    }
}
