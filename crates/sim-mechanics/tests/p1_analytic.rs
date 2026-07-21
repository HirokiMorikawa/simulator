//! P1 解析解テスト(M2, M5–M9, M15, F1–F5)。定義: docs/21-verification/01-analytic-tests.md。
//! (F6 は代数検算のためユニットテスト crates/sim-fluid/src/buoyancy.rs で検証)
//! ユニットテストではなく crate 公開 API 経由の統合テスト(World 層が無い Phase A 時点の代替)。

use sim_core::{Event, EventQueue, Material, MaterialDb, PairOverride, Solver, SolverContext};
use sim_fluid::{Atmosphere, StaticWaterRegion};
use sim_math::{BallisticIntegrator, Quat, SimRng, Vec3};
use sim_mechanics::{BodyType, DragModel, MechanicsSolver, RigidBodyDesc, Shape};

/// 密度 `density` の試験用材料を登録する(摩擦・反発はゼロ、F4/F5 の浮力単体テスト用)。
fn buoyancy_test_material(materials: &mut MaterialDb, density: f64) -> sim_core::MaterialId {
    materials.push(Material {
        name: "test-buoyancy-body",
        density,
        friction: 0.0,
        restitution: 0.0,
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

/// M6: 反発バウンド — 高さ比 = e^2、rel 1%(docs/21-verification/01-analytic-tests.md M6、
/// split impulse モード)。split impulse(docs/10-mechanics/03-contact-solver.md §4.5)導入により
/// 位置補正が速度チャンネルを汚さなくなったため、設計が目標とする rel 1% を達成できる。
/// 反発閾値は M5 と同じ理由で 0 にする(既定 0.5 m/s の固定減算は、この落下速度
/// (≈6 m/s)では e を約8%見かけ上下げてしまい、閾値自体の効果と解析解の比較が
/// 混ざってしまうため)。
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
        (ratio - expected).abs() / expected < 0.01,
        "height ratio {ratio} vs expected {expected} (e^2)"
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

/// F1: 終端速度(鋼球 半径5mm)— v_t=sqrt(2mg/(ρCdA))、rel 1%
/// (docs/21-verification/01-analytic-tests.md F1、docs/11-fluid/05-aero-hydrodynamics.md §2.1)。
/// 空気密度・Cd(亜臨界)は同 §9 のパラメータ表(ISA 15°C 海面)。
#[test]
fn f1_terminal_velocity_matches_high_re_drag_formula() {
    let materials = MaterialDb::standard();
    let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();

    let mut solver = MechanicsSolver::new(9.80665);
    let air_viscosity = 1.81e-5; // CRC Handbook、空気 15°C 近傍
    let atmosphere = Atmosphere::still(1.225, air_viscosity);
    solver.atmosphere = Some(atmosphere);

    let radius = 0.005;
    let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, steel);
    desc.drag = DragModel::Sphere { radius };
    let idx = solver.create_body(desc, &materials);

    // v_t/g ~ 数秒のスケール(高Re域では緩和が緩やか)なので30秒(3600 step)与える。
    step_n(&mut solver, &materials, 1.0 / 120.0, 3600);

    let mass = 7850.0 * (4.0 / 3.0) * std::f64::consts::PI * radius.powi(3);
    let area = std::f64::consts::PI * radius * radius;
    let cd = 0.47;
    let analytic_vt = (2.0 * mass * 9.80665 / (atmosphere.density * cd * area)).sqrt();

    let measured = -solver.bodies.linear_velocity[idx].y; // 下向きが負
    assert!(
        (measured - analytic_vt).abs() / analytic_vt < 0.01,
        "measured={measured} analytic={analytic_vt}"
    );
}

/// F2: 雨滴(直径2mm)の終端速度 ≈ 6.5 m/s(Gunn-Kinzer 1949 実測)、rel 5%
/// (docs/21-verification/01-analytic-tests.md F2)。単純球抗力モデルは雨滴の扁平化を
/// 表現しないため、実測との差は球近似の既知の誤差として 5% 許容に収まることを検証する。
#[test]
fn f2_raindrop_terminal_velocity_matches_gunn_kinzer_measurement() {
    let materials = MaterialDb::standard();
    let water = materials.find_by_name("水").unwrap();

    let mut solver = MechanicsSolver::new(9.80665);
    solver.atmosphere = Some(Atmosphere::still(1.225, 1.81e-5));

    let radius = 0.001; // 直径2mm
    let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, water);
    desc.drag = DragModel::Sphere { radius };
    let idx = solver.create_body(desc, &materials);

    step_n(&mut solver, &materials, 1.0 / 120.0, 600); // 5s、τ~0.34s に対し十分

    let measured = -solver.bodies.linear_velocity[idx].y;
    let gunn_kinzer = 6.5;
    assert!(
        (measured - gunn_kinzer).abs() / gunn_kinzer < 0.05,
        "measured={measured} gunn_kinzer={gunn_kinzer}"
    );
}

/// F3: ストークス沈降 v=2r²Δρg/(9μ)、rel 2%
/// (docs/21-verification/01-analytic-tests.md F3、docs/11-fluid/05-aero-hydrodynamics.md §2.1)。
///
/// 浮力(F4–F6)は未実装のため、本テストは Δρ≈ρ_particle となるよう媒質密度を無視できるほど
/// 小さく(0.5 kg/m³、実在流体ではなく低Re環境を作るための試験値)取ることで、浮力の欠如が
/// 与える誤差を目標許容(2%)より十分小さく(<0.1%)抑えて隔離検証する。粘性は Re<0.02(スト
/// ークス域として十分小さい、Schiller-Naumann補正の寄与 <1%)かつ緩和時定数 τ=(2/9)ρr²/μ が
/// dt(1/120s)を大きく上回る(数値安定性)ように選ぶ。
#[test]
fn f3_stokes_settling_matches_analytic_formula() {
    let materials = MaterialDb::standard();
    let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
    let steel_density = 7850.0;

    let mut solver = MechanicsSolver::new(9.80665);
    let fluid_density = 0.5;
    let viscosity = 1.0;
    solver.atmosphere = Some(Atmosphere::still(fluid_density, viscosity));

    let radius = 0.01;
    let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, steel);
    desc.drag = DragModel::Sphere { radius };
    let idx = solver.create_body(desc, &materials);

    step_n(&mut solver, &materials, 1.0 / 120.0, 240); // 2s ~ 11*τ、定常収束に十分

    let delta_rho = steel_density - fluid_density;
    let analytic = 2.0 * radius * radius * delta_rho * 9.80665 / (9.0 * viscosity);
    let measured = -solver.bodies.linear_velocity[idx].y;
    assert!(
        (measured - analytic).abs() / analytic < 0.02,
        "measured={measured} analytic={analytic}"
    );
}

/// F4: 立方体の喫水 = 密度比 × 辺長、rel 1%(静水域)
/// (docs/21-verification/01-analytic-tests.md F4、docs/11-fluid/04-free-surface-buoyancy.md §2.2)。
/// 解析的な釣り合い位置(浮力=重力)にゼロ速度で置き、力が厳密に釣り合っていて動かないことを
/// 検証する(密度比の定義から浮力=重力は代数的に厳密に成立する、テスト内コメント参照)。
#[test]
fn f4_cube_waterline_depth_matches_density_ratio() {
    let mut materials = MaterialDb::standard();
    let water_density = 998.2;
    let ratio = 0.6;
    let body = buoyancy_test_material(&mut materials, ratio * water_density);

    let mut solver = MechanicsSolver::new(9.80665);
    solver.water = Some(StaticWaterRegion::new(0.0, water_density));

    let half = 0.5; // 一辺 1m
    let side = 2.0 * half;
    // 釣り合い: ratio*ρ_f*V = ρ_f*g*(ratio*side*base_area)、V=side*base_area なので
    // 両辺の ratio*ρ_f*side*base_area が一致し、喫水 h_sub=ratio*side で厳密に釣り合う。
    let h_sub = ratio * side;
    let equilibrium_y = -h_sub + half;
    let mut desc = RigidBodyDesc::dynamic(
        Shape::Box {
            half_extents: Vec3::new(half, half, half),
        },
        body,
    );
    desc.transform.position = Vec3::new(0.0, equilibrium_y, 0.0);
    let idx = solver.create_body(desc, &materials);

    step_n(&mut solver, &materials, 1.0 / 120.0, 120);

    let drift = (solver.bodies.position[idx].y - equilibrium_y).abs();
    assert!(
        drift / side < 0.01,
        "drift={drift} equilibrium_y={equilibrium_y}"
    );
}

/// F5: 浮体の上下振動周期 T=2π√(m/(ρ_f g A_wl))、rel 5%
/// (docs/21-verification/01-analytic-tests.md F5、docs/11-fluid/04-free-surface-buoyancy.md §7)。
/// 釣り合い位置から変位させ、速度が正から非正に転じる最初の時刻(=1周期後、開始が変位最大点
/// のため)を測定して比較する。減衰(水中抗力)は未実装のため無損失SHMとして厳密に周期的。
#[test]
fn f5_floating_body_heave_period_matches_analytic_formula() {
    let mut materials = MaterialDb::standard();
    let water_density = 998.2;
    let ratio = 0.5;
    let body = buoyancy_test_material(&mut materials, ratio * water_density);

    let mut solver = MechanicsSolver::new(9.80665);
    solver.water = Some(StaticWaterRegion::new(0.0, water_density));

    let half = 0.5;
    let side = 2.0 * half;
    let equilibrium_y = -(ratio * side) + half;
    let amplitude = 0.1; // half=0.5 に対し十分小さく、全没・完全露出を避ける
    let mut desc = RigidBodyDesc::dynamic(
        Shape::Box {
            half_extents: Vec3::new(half, half, half),
        },
        body,
    );
    desc.transform.position = Vec3::new(0.0, equilibrium_y + amplitude, 0.0);
    let idx = solver.create_body(desc, &materials);

    let dt = 1.0 / 120.0;
    let mut rng = SimRng::new(1, 1);
    let mut events = EventQueue::new();
    let mut t = 0.0;
    let mut period = None;
    let mut prev_v = 0.0;
    for _ in 0..400 {
        let mut ctx = SolverContext {
            materials: &materials,
            rng: &mut rng,
            events: &mut events,
        };
        solver.step(dt, &mut ctx);
        let _: Vec<Event> = events.drain_sorted();
        t += dt;
        let v = solver.bodies.linear_velocity[idx].y;
        if prev_v > 0.0 && v <= 0.0 && period.is_none() {
            period = Some(t);
        }
        prev_v = v;
    }

    let measured = period.expect("should observe one full heave period within simulated window");
    let mass = ratio * water_density * side.powi(3);
    let waterline_area = side * side;
    let analytic =
        2.0 * std::f64::consts::PI * (mass / (water_density * 9.80665 * waterline_area)).sqrt();
    assert!(
        (measured - analytic).abs() / analytic < 0.05,
        "measured={measured} analytic={analytic}"
    );
}

/// M15: 弾丸トンネリング防止(貫通せず反発、docs/21-verification/01-analytic-tests.md M15)。
/// 高速球(300 m/s、r=5mm)が厚さ2mmの静的鋼板に衝突する。1ステップの移動距離
/// (dt=1/1200sで0.25m)が板厚(2mm)を大きく超えるため、最小CCD(speculative contact、
/// `ccd::apply_speculative_contacts`)なしでは離散衝突検出のステップ端点判定を
/// すり抜けて板を素通りしてしまう。
///
/// 設計の主たる合格基準は「貫通イベントゼロ・貫入 < slop」であり、本実装(TOI反復なしの
/// 速度クランプ、設計§4.6が許容する簡略化)では、クランプが効くステップの離散化位相に
/// よって実際の衝突速度がv0から若干目減りする(実装検証中に、dtを変えると反発速度の
/// 相対誤差が2%~20%まで変動することを発見 — 真のTOIサブステップでないため、クランプ後の
/// 速度がちょうど衝突する瞬間のv0と厳密には一致しないという原理的な限界)。そのため反発
/// 速度の一致は緩めの許容誤差で確認し、主要な合格基準(貫通ゼロ・貫入有界)を主眼に置く。
#[test]
fn m15_bullet_speed_sphere_does_not_tunnel_through_thin_plate() {
    let mut materials = MaterialDb::standard();
    let restitution = 0.5;
    let mat = frictionless_bouncy_material(&mut materials, restitution);

    let mut solver = MechanicsSolver::new(0.0); // 重力なし(水平弾道、簡潔化)
    solver.restitution_velocity_threshold = 0.0;

    let radius = 0.005; // 5mm
    let v0 = 300.0;
    let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, mat);
    desc.transform.position = Vec3::new(-1.0, 0.0, 0.0);
    desc.linear_velocity = Vec3::new(v0, 0.0, 0.0);
    let idx = solver.create_body(desc, &materials);

    let plate_half_thickness = 0.001; // 板厚2mm
    let plate = RigidBodyDesc {
        body_type: BodyType::Static,
        ..RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(plate_half_thickness, 0.5, 0.5),
            },
            mat,
        )
    };
    solver.create_body(plate, &materials);

    let dt = 1.0 / 1200.0;
    let slop = 0.005; // contact::SLOP と同じ値(private のためテスト内で複製)
    let mut min_gap_seen = f64::INFINITY;
    let mut tunneled = false;
    for _ in 0..60 {
        step_n(&mut solver, &materials, dt, 1);
        let x = solver.bodies.position[idx].x;
        let vx = solver.bodies.linear_velocity[idx].x;
        let gap = -plate_half_thickness - (x + radius); // 板の近い面までの隙間(負なら貫入)
        min_gap_seen = min_gap_seen.min(gap);
        if x > plate_half_thickness + radius && vx > 0.0 {
            tunneled = true; // 板の反対側に正の速度で抜けた = 貫通
        }
    }

    assert!(!tunneled, "bullet should not tunnel through the plate");
    assert!(
        min_gap_seen > -slop,
        "penetration should stay below slop: min_gap_seen={min_gap_seen}"
    );

    let final_vx = solver.bodies.linear_velocity[idx].x;
    let expected_rebound = -restitution * v0;
    assert!(
        final_vx < 0.0,
        "bullet should bounce back: final_vx={final_vx}"
    );
    let rel_err = (final_vx - expected_rebound).abs() / expected_rebound.abs();
    assert!(
        rel_err < 0.25,
        "final_vx={final_vx} expected_rebound={expected_rebound} rel_err={rel_err}"
    );
}

/// M2: 斜方投射45°(真空、無衝突)— 到達距離 R = v0²/g(設計§21-verification/01)。
/// `MechanicsSolver`(既定はsemi-implicit Euler、衝突ソルバ込み)ではなく、設計が
/// 明記するとおり無衝突専用の`BallisticIntegrator`(RK4)を直接使う。等速度の重力加速度
/// のみを与えるとRK4は4次多項式まで厳密なため(実際の解は位置が時間の2次式)、離散化
/// 誤差は原理的に浮動小数点丸め程度しか生じない。飛行時間T=2v0sinθ/gちょうどでdtを
/// 割り切れるように刻み数を選ぶことで、着地点の線形補間も不要にした。
#[test]
fn m2_45_degree_projectile_range_matches_v0_squared_over_g() {
    let g = 9.80665;
    let v0 = 20.0;
    let theta = std::f64::consts::FRAC_PI_4;
    let gravity_accel = Vec3::new(0.0, -g, 0.0);

    let flight_time = 2.0 * v0 * theta.sin() / g;
    let steps = 2000;
    let dt = flight_time / steps as f64;

    let integrator = BallisticIntegrator;
    let mut x = Vec3::new(0.0, 0.0, 0.0);
    let mut v = Vec3::new(v0 * theta.cos(), v0 * theta.sin(), 0.0);
    for _ in 0..steps {
        let (nx, nv) = integrator.step(x, v, |_x, _v| gravity_accel, dt);
        x = nx;
        v = nv;
    }

    let expected_range = v0 * v0 / g;
    assert!(x.y.abs() < 1e-6, "should land back at y=0, got y={}", x.y);
    let rel_err = (x.x - expected_range).abs() / expected_range;
    assert!(
        rel_err < 0.005,
        "range={:.6} expected={expected_range:.6} rel_err={rel_err:.6}",
        x.x
    );
}
