//! P2 解析解テスト。定義: docs/21-verification/01-analytic-tests.md。

use sim_core::{Event, EventQueue, MaterialDb, Solver, SolverContext};
use sim_math::{SimRng, Vec3};
use sim_mechanics::{BodyType, MechanicsSolver, RigidBodyDesc, Shape};

/// M12: スタック静止 — 4段の木箱を10秒間シミュレートし、速度が閾値未満・貫入がslop未満に
/// 収まること(docs/21-verification/01-analytic-tests.md M12)。warm starting(§4.4)+
/// 軸選択ヒステリシス(§4.4)+ split impulse(§4.5)が揃って初めて「4段積みが10反復で
/// 安定する」(設計 §4.4 の説明そのもの)ことを実地で確認する。
#[test]
fn m12_four_box_stack_settles_below_velocity_threshold() {
    let materials = MaterialDb::standard();
    let wood = materials.find_by_name("木材(松)").unwrap();

    let mut solver = MechanicsSolver::new(9.80665);
    let half = 0.5;
    let ground = RigidBodyDesc {
        body_type: BodyType::Static,
        ..RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            wood,
        )
    };
    solver.create_body(ground, &materials);

    let mut box_indices = Vec::new();
    for level in 0..4 {
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(half, half, half),
            },
            wood,
        );
        // ちょうど接した状態(隙間0)から開始し、初期落下による大きな衝撃を避ける
        // (M12 が検証するのは静止状態の維持であり、積み上げの過渡応答ではない)。
        desc.transform.position = Vec3::new(0.0, half + level as f64 * 2.0 * half, 0.0);
        box_indices.push(solver.create_body(desc, &materials));
    }

    let dt = 1.0 / 120.0;
    let mut rng = SimRng::new(1, 1);
    let mut events = EventQueue::new();
    for _ in 0..1200 {
        let mut ctx = SolverContext {
            materials: &materials,
            rng: &mut rng,
            events: &mut events,
        };
        solver.step(dt, &mut ctx);
        let _: Vec<Event> = events.drain_sorted();
    }

    const SLOP: f64 = 0.005; // docs/10-mechanics/03-contact-solver.md §9 の既定値と同じ
    for &idx in &box_indices {
        let speed = solver.bodies.linear_velocity[idx].length();
        assert!(speed < 1e-3, "speed {speed} exceeds M12 threshold");
    }
    // 「貫入 < slop」は個々の接触の重なりについての条件(設計 §9)。積み上げでは各接触が
    // 独立に slop まで許容するため、box の絶対位置の沈み込みは段数に比例して累積しうる
    // (物理的に正しい挙動)。したがって隣接ペア(地面-最下段、段k-段k+1)ごとの
    // 貫入量を個別に検査する。
    let mut below_top = 0.0; // 直下の面(地面 or 下段の上面)の y
    for &idx in &box_indices {
        let bottom = solver.bodies.position[idx].y - half;
        let penetration = below_top - bottom;
        assert!(
            penetration < SLOP,
            "penetration {penetration} exceeds slop {SLOP}"
        );
        below_top = solver.bodies.position[idx].y + half;
    }
}
