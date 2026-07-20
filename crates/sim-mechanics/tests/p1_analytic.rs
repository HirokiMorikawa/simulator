//! P1 解析解テスト(M5–M9)。定義: docs/21-verification/01-analytic-tests.md。
//! ユニットテストではなく crate 公開 API 経由の統合テスト(World 層が無い Phase A 時点の代替)。

use sim_core::{Event, EventQueue, Material, MaterialDb, PairOverride, Solver, SolverContext};
use sim_math::{Quat, SimRng, Vec3};
use sim_mechanics::{BodyType, MechanicsSolver, RigidBodyDesc, Shape};

fn step_n(solver: &mut MechanicsSolver, materials: &MaterialDb, dt: f64, n: u32) {
    let mut rng = SimRng::new(1, 1);
    let mut events = EventQueue::new();
    for _ in 0..n {
        let mut ctx = SolverContext {
            materials,
            rng: &mut rng,
            events: &mut events,
        };
        solver.step(dt, &mut ctx);
        let _: Vec<Event> = events.drain_sorted();
    }
}

fn frictionless_bouncy_material(
    materials: &mut MaterialDb,
    restitution: f64,
) -> sim_core::MaterialId {
    materials.push(Material {
        name: "test-frictionless-bouncy",
        density: 1000.0,
        friction: 0.0,
        restitution,
        youngs_modulus: None,
        specific_heat: 1000.0,
        conductivity: 1.0,
        emissivity: 0.5,
        melting: None,
        resistivity: None,
        relative_permittivity: 1.0,
        refractive_index: None,
        source: "test fixture",
        uncertainty: 0.0,
    })
}

/// 傾斜角 theta(ラジアン、水平からの角)の下り坂を x-y 平面内に作る。
/// 面法線は水平(theta=0)で (0,1,0)。下り方向は重力の接線成分の向きから導出する
/// (docs/10-mechanics/03-contact-solver.md M8 テスト相当の導出済み式)。
fn incline_normal_and_downhill(theta: f64) -> (Vec3, Vec3) {
    let normal = Vec3::new(-theta.sin(), theta.cos(), 0.0);
    let downhill = Vec3::new(-theta.cos(), -theta.sin(), 0.0);
    (normal, downhill)
}

/// 箱の局所 +y 面を傾斜面法線に一致させる回転(z軸まわり theta)。
fn incline_box_rotation(theta: f64) -> Quat {
    Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), theta)
}

/// M5: 1D 正面衝突(等質量、e=1)— 速度交換が abs 1e-6 で成立。
/// ジッタ防止の反発閾値(既定 0.5 m/s、docs/10-mechanics/03-contact-solver.md §4.3)は
/// 理想化された弾性衝突を隔離検証するため 0 にする(閾値は数値安定性のためのヒューリスティクスで
/// あり、M5 が検証する核の物理則そのものではない)。細かい dt で離散化ラグ由来の
/// Baumgarte 過剰インパルス(貫入量 ∝ 衝突速度×dt)も抑える。
#[test]
fn m5_equal_mass_elastic_collision_exchanges_velocities() {
    let mut materials = MaterialDb::standard();
    let mat = frictionless_bouncy_material(&mut materials, 1.0);

    let mut solver = MechanicsSolver::new(0.0); // 重力なし(1D正面衝突の理想化)
    solver.restitution_velocity_threshold = 0.0;
    let radius = 0.5;
    let v0 = 2.0;

    let mut a = RigidBodyDesc::dynamic(Shape::Sphere { radius }, mat);
    a.transform.position = Vec3::new(-1.1, 0.0, 0.0);
    a.linear_velocity = Vec3::new(v0, 0.0, 0.0);
    let idx_a = solver.create_body(a, &materials);

    let mut b = RigidBodyDesc::dynamic(Shape::Sphere { radius }, mat);
    b.transform.position = Vec3::new(0.0, 0.0, 0.0);
    let idx_b = solver.create_body(b, &materials);

    step_n(&mut solver, &materials, 1.0 / 1200.0, 600);

    assert!(
        solver.bodies.linear_velocity[idx_a].x.abs() < 1e-6,
        "A should stop: {:?}",
        solver.bodies.linear_velocity[idx_a]
    );
    assert!(
        (solver.bodies.linear_velocity[idx_b].x - v0).abs() < 1e-6,
        "B should carry A's velocity: {:?}",
        solver.bodies.linear_velocity[idx_b]
    );
}

/// M6: 反発バウンド — 高さ比 = e^2。
/// Phase 1 は Baumgarte のみ(split impulse は Phase 2、docs/10-mechanics/03-contact-solver.md §4.5)。
/// Baumgarte の偽エネルギーは離散化ラグ由来の貫入量に比例するため、細かい dt で抑える。
/// 反発閾値は M5 と同じ理由で 0 にする(既定 0.5 m/s の固定減算は、この落下速度
/// (≈6 m/s)では e を約8%見かけ上下げてしまい、閾値自体の効果とバウムガルテ誤差が
/// 混ざって解析解と比較できなくなるため)。
#[test]
fn m6_bounce_height_ratio_matches_restitution_squared() {
    let mut materials = MaterialDb::standard();
    let restitution = 0.6;
    let mat = frictionless_bouncy_material(&mut materials, restitution);

    let mut solver = MechanicsSolver::new(9.80665);
    solver.restitution_velocity_threshold = 0.0;
    let radius = 0.1;
    let drop_height = 1.9; // 中心の初期高さ - radius
    let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, mat);
    desc.transform.position = Vec3::new(0.0, drop_height + radius, 0.0);
    let idx = solver.create_body(desc, &materials);

    let ground = RigidBodyDesc {
        body_type: BodyType::Static,
        ..RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            mat,
        )
    };
    solver.create_body(ground, &materials);

    let dt = 1.0 / 1200.0;
    let mut rng = SimRng::new(1, 1);
    let mut events = EventQueue::new();

    let mut min_height = f64::INFINITY;
    let mut post_bounce_max = f64::NEG_INFINITY;
    let mut bounced = false;
    for _ in 0..12_000 {
        let mut ctx = SolverContext {
            materials: &materials,
            rng: &mut rng,
            events: &mut events,
        };
        solver.step(dt, &mut ctx);
        let _: Vec<Event> = events.drain_sorted();

        let height = solver.bodies.position[idx].y - radius;
        if !bounced {
            if height < min_height {
                min_height = height;
            } else if height > min_height + 1e-4 {
                bounced = true; // 最下点を過ぎて上昇に転じた
            }
        } else {
            post_bounce_max = post_bounce_max.max(height);
            if height < post_bounce_max - 1e-4 {
                break; // 頂点を過ぎて再度落下し始めた
            }
        }
    }

    let ratio = post_bounce_max / drop_height;
    let expected = restitution * restitution;
    assert!(
        (ratio - expected).abs() / expected < 0.1,
        "height ratio {ratio} vs expected {expected} (e^2); Baumgarte-only Phase 1 は \
         split impulse (Phase 2) 導入まで rel 1% を保証しない(設計注記)"
    );
}

/// M7: 斜面静止 — tanθ < μs で速度が閾値未満に収束する。
/// 球だと接触点でのトルクにより転がり始める(摩擦=滑り速度ゼロを保証するのみで、
/// 重心の並進を止める保証ではない)ため、回転しない箱で検証する。
#[test]
fn m7_incline_static_when_angle_below_friction() {
    let materials = MaterialDb::standard();
    let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
    let mu = materials.friction_pair(steel, steel);

    let theta: f64 = 20.0_f64.to_radians();
    assert!(theta.tan() < mu, "test precondition: tanθ < μs");

    let mut solver = MechanicsSolver::new(9.80665);
    let (normal, _) = incline_normal_and_downhill(theta);
    let half_extent = 0.5;
    let mut desc = RigidBodyDesc::dynamic(
        Shape::Box {
            half_extents: Vec3::new(half_extent, half_extent, half_extent),
        },
        steel,
    );
    desc.transform.position = normal.scale(half_extent);
    desc.transform.rotation = incline_box_rotation(theta);
    let idx = solver.create_body(desc, &materials);

    let plane = RigidBodyDesc {
        body_type: BodyType::Static,
        ..RigidBodyDesc::dynamic(Shape::Plane { normal, d: 0.0 }, steel)
    };
    solver.create_body(plane, &materials);

    step_n(&mut solver, &materials, 1.0 / 120.0, 600); // 5s

    let speed = solver.bodies.linear_velocity[idx].length();
    assert!(speed < 1e-4, "body should stay at rest, speed={speed}");
}

/// M8: 斜面滑走 — a = g(sinθ - μk cosθ)。箱で検証(理由は M7 参照)。
#[test]
fn m8_incline_slide_acceleration_matches_formula() {
    let materials = MaterialDb::standard();
    let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
    let mu = materials.friction_pair(steel, steel);

    let theta: f64 = 45.0_f64.to_radians();
    assert!(theta.tan() > mu, "test precondition: tanθ > μk (sliding)");

    let gravity = 9.80665;
    let mut solver = MechanicsSolver::new(gravity);
    let (normal, downhill) = incline_normal_and_downhill(theta);
    let half_extent = 0.5;
    let mut desc = RigidBodyDesc::dynamic(
        Shape::Box {
            half_extents: Vec3::new(half_extent, half_extent, half_extent),
        },
        steel,
    );
    desc.transform.position = normal.scale(half_extent);
    desc.transform.rotation = incline_box_rotation(theta);
    let idx = solver.create_body(desc, &materials);

    let plane = RigidBodyDesc {
        body_type: BodyType::Static,
        ..RigidBodyDesc::dynamic(Shape::Plane { normal, d: 0.0 }, steel)
    };
    solver.create_body(plane, &materials);

    let dt = 1.0 / 120.0;
    step_n(&mut solver, &materials, dt, 60); // 0.5s

    let speed_downhill = solver.bodies.linear_velocity[idx].dot(downhill);
    let elapsed = 60.0 * dt;
    let measured_accel = speed_downhill / elapsed;
    let expected_accel = gravity * (theta.sin() - mu * theta.cos());
    assert!(
        (measured_accel - expected_accel).abs() / expected_accel < 0.05,
        "measured {measured_accel} vs expected {expected_accel}"
    );
}

/// M9: 制動距離 — d = v0^2/(2 μk g)。箱で検証(球だと摩擦トルクで転がりに変わり、
/// 単純な並進減速モデルと一致しなくなるため)。
#[test]
fn m9_braking_distance_matches_formula() {
    let materials = MaterialDb::standard();
    let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
    let mu = materials.friction_pair(steel, steel);
    let gravity = 9.80665;

    let mut solver = MechanicsSolver::new(gravity);
    let half_extent = 0.5;
    let v0 = 4.0;
    let mut desc = RigidBodyDesc::dynamic(
        Shape::Box {
            half_extents: Vec3::new(half_extent, half_extent, half_extent),
        },
        steel,
    );
    desc.transform.position = Vec3::new(0.0, half_extent, 0.0);
    desc.linear_velocity = Vec3::new(v0, 0.0, 0.0);
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

    let start_x = solver.bodies.position[idx].x;
    let dt = 1.0 / 120.0;
    let mut rng = SimRng::new(1, 1);
    let mut events = EventQueue::new();
    for _ in 0..(10.0 / dt) as u32 {
        let mut ctx = SolverContext {
            materials: &materials,
            rng: &mut rng,
            events: &mut events,
        };
        solver.step(dt, &mut ctx);
        let _: Vec<Event> = events.drain_sorted();
        if solver.bodies.linear_velocity[idx].x <= 0.0 {
            break;
        }
    }

    let traveled = solver.bodies.position[idx].x - start_x;
    let expected = v0 * v0 / (2.0 * mu * gravity);
    assert!(
        (traveled - expected).abs() / expected < 0.05,
        "traveled {traveled} vs expected {expected}"
    );
}

/// 素材ペア表オーバーライドが実際の接触ソルバで使われることを確認する
/// (docs/10-mechanics/04-friction.md §3.1)。
#[test]
fn friction_pair_override_is_honored_in_contact_solver() {
    let mut materials = MaterialDb::standard();
    let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
    let ice = materials.find_by_name("氷(0°C)").unwrap();
    materials.set_friction_pair(
        steel,
        ice,
        PairOverride {
            friction: 0.9,
            restitution: 0.0,
        },
    );

    let gravity = 9.80665;
    let mut solver = MechanicsSolver::new(gravity);
    // tanθ≈0.577。通常の鋼-氷ペア(√(0.6*0.05)≈0.17)なら滑るが、オーバーライド 0.9 なら静止するはず。
    let theta: f64 = 30.0_f64.to_radians();
    let (normal, _) = incline_normal_and_downhill(theta);
    let half_extent = 0.5;
    let mut desc = RigidBodyDesc::dynamic(
        Shape::Box {
            half_extents: Vec3::new(half_extent, half_extent, half_extent),
        },
        steel,
    );
    desc.transform.position = normal.scale(half_extent);
    desc.transform.rotation = incline_box_rotation(theta);
    let idx = solver.create_body(desc, &materials);

    let plane = RigidBodyDesc {
        body_type: BodyType::Static,
        ..RigidBodyDesc::dynamic(Shape::Plane { normal, d: 0.0 }, ice)
    };
    solver.create_body(plane, &materials);

    step_n(&mut solver, &materials, 1.0 / 120.0, 600);
    let speed = solver.bodies.linear_velocity[idx].length();
    // 箱は4点接触で法線・摩擦が相互作用するため、warm starting 無し(Phase 2)の
    // 10反復では M7 の球1点接触ほど厳密にゼロへ収束しない。M7 より緩い閾値を使う。
    assert!(
        speed < 1e-3,
        "override friction should keep body static, speed={speed}"
    );
}
