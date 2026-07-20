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

/// 転がり摩擦(docs/10-mechanics/04-friction.md §4.1「トルク制約 |τ_roll|≤μ_roll・N・r」)。
/// M-series には対応する番号がないため、設計の力学導出をそのままテストにする:
/// 滑りなし転がり(v=-ωr)を初期条件に与えると、エネルギー収支
/// $\frac{d}{dt}\left(\frac{7}{10}mv^2\right) = -\tau_{roll}\,\omega$
/// (剛体球の慣性 $I=\frac25 mr^2$ を含む有効質量 $\frac75 m$ から)より並進減速度は
/// $a=\frac57\mu_{roll} g$ になる(単純な $a=\mu_{roll} g$ ではない)。転がり摩擦が
/// 無ければ(設計 §1「これが無いと球が永遠に転がり続ける」)速度は一定のまま減衰しない。
#[test]
fn rolling_friction_decelerates_ball_at_designed_rate() {
    let materials = MaterialDb::standard();
    let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
    let gravity = 9.80665;

    let mut solver = MechanicsSolver::new(gravity);
    let radius = 0.1;
    let v0 = 2.0;
    let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, steel);
    desc.transform.position = Vec3::new(0.0, radius, 0.0);
    desc.linear_velocity = Vec3::new(v0, 0.0, 0.0);
    // 滑りなし転がり(初期スリップ0)。接触点オフセット r_a=(0,-radius,0) に対し
    // v + ω×r_a = 0 を満たす ω は (0,0,-v0/radius)。
    desc.angular_velocity = Vec3::new(0.0, 0.0, -v0 / radius);
    let idx = solver.create_body(desc, &materials);

    let plane = RigidBodyDesc {
        body_type: BodyType::Static,
        ..RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            steel,
        )
    };
    solver.create_body(plane, &materials);

    let dt = 1.0 / 120.0;
    let duration = 20.0;
    let mut rng = SimRng::new(1, 1);
    let mut events = EventQueue::new();
    for _ in 0..(duration / dt) as u32 {
        let mut ctx = SolverContext {
            materials: &materials,
            rng: &mut rng,
            events: &mut events,
        };
        solver.step(dt, &mut ctx);
        let _: Vec<Event> = events.drain_sorted();
    }

    let final_speed = solver.bodies.linear_velocity[idx].x;
    let final_omega = solver.bodies.angular_velocity[idx].z;
    // 滑りなし転がりを維持していること(v ≈ -ω・radius)の確認。
    assert!(
        (final_speed + final_omega * radius).abs() / v0 < 0.01,
        "rolling constraint violated: v={final_speed} omega={final_omega}"
    );

    const ROLLING_FRICTION: f64 = 0.005; // crates/sim-mechanics/src/contact.rs の既定値と同じ
    let expected_decel = (5.0 / 7.0) * ROLLING_FRICTION * gravity;
    let measured_decel = (v0 - final_speed) / duration;
    let rel_err = (measured_decel - expected_decel).abs() / expected_decel;
    assert!(
        rel_err < 0.02,
        "measured_decel={measured_decel} expected_decel={expected_decel} rel_err={rel_err}"
    );
}

/// スリープ(docs/10-mechanics/01-rigid-body.md §4「速度が閾値未満の接触島単位で積分を
/// 停止」)。単独の木箱が地面に着地して静止すると、既定の継続時間(0.5s)後に asleep になり
/// 速度が厳密に0に凍結されることを確認する。
#[test]
fn sleep_engages_after_box_settles_on_ground() {
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
    let mut desc = RigidBodyDesc::dynamic(
        Shape::Box {
            half_extents: Vec3::new(half, half, half),
        },
        wood,
    );
    desc.transform.position = Vec3::new(0.0, half + 0.05, 0.0);
    let idx = solver.create_body(desc, &materials);

    let dt = 1.0 / 120.0;
    let mut rng = SimRng::new(1, 1);
    let mut events = EventQueue::new();
    for _ in 0..600 {
        let mut ctx = SolverContext {
            materials: &materials,
            rng: &mut rng,
            events: &mut events,
        };
        solver.step(dt, &mut ctx);
        let _: Vec<Event> = events.drain_sorted();
    }

    assert!(
        solver.bodies.asleep[idx],
        "box should be asleep after settling for 5s"
    );
    assert_eq!(solver.bodies.linear_velocity[idx], Vec3::ZERO);
    assert_eq!(solver.bodies.angular_velocity[idx], Vec3::ZERO);
}

/// スリープからの起床(設計 §4「起床は新規接触・力適用時」)。眠っている箱に別の落下する
/// 箱がぶつかると、新規接触で起床することを確認する。
#[test]
fn sleeping_box_wakes_on_new_contact_from_falling_body() {
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
    let mut resting_desc = RigidBodyDesc::dynamic(
        Shape::Box {
            half_extents: Vec3::new(half, half, half),
        },
        wood,
    );
    resting_desc.transform.position = Vec3::new(0.0, half, 0.0);
    let resting_idx = solver.create_body(resting_desc, &materials);

    let dt = 1.0 / 120.0;
    let mut rng = SimRng::new(1, 1);
    let mut events = EventQueue::new();
    for _ in 0..600 {
        let mut ctx = SolverContext {
            materials: &materials,
            rng: &mut rng,
            events: &mut events,
        };
        solver.step(dt, &mut ctx);
        let _: Vec<Event> = events.drain_sorted();
    }
    assert!(
        solver.bodies.asleep[resting_idx],
        "resting box should be asleep before impact"
    );

    let mut falling_desc = RigidBodyDesc::dynamic(
        Shape::Box {
            half_extents: Vec3::new(half, half, half),
        },
        wood,
    );
    falling_desc.transform.position = Vec3::new(0.0, half * 2.0 + 3.0, 0.0);
    solver.create_body(falling_desc, &materials);

    let mut woke = false;
    for _ in 0..240 {
        let mut ctx = SolverContext {
            materials: &materials,
            rng: &mut rng,
            events: &mut events,
        };
        solver.step(dt, &mut ctx);
        let _: Vec<Event> = events.drain_sorted();
        if !solver.bodies.asleep[resting_idx] {
            woke = true;
            break;
        }
    }
    assert!(
        woke,
        "resting box should wake once the falling box lands on it"
    );
}
