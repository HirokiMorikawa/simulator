//! フルCCD(conservative advancement、設計 docs/10-mechanics/02-collision-detection.md
//! §4.6「フルCCD(Phase 5)」)の受け入れテスト。**群9で配線**。
//!
//! 既存の最小CCD(speculative contact)は **球 × 静的 Box/Plane** に限定されており
//! (`ccd.rs` が球以外の形状と `BodyType::Dynamic` の相手を `continue` する)、
//!
//! - 動的な相手(揺れる板・飛んでいる的)
//! - 球以外の弾丸(高速の箱)
//!
//! は**素通り**していた。`gjk::conservative_advancement_toi` は以前から実装・テスト
//! されていたが、ワークスペースのどこからも呼ばれていなかった。
//!
//! `MechanicsSolver::set_full_ccd(false)` で最小CCDだけの状態へ戻せるので、
//! **同一シーン・同一乱数の対照実験**として「切ると貫通する / 入れると貫通しない」を
//! 直接示す。

use sim_core::{Event, EventQueue, Material, MaterialDb, MaterialId, Solver, SolverContext};
use sim_math::{SimRng, Vec3};
use sim_mechanics::{BodyType, MechanicsSolver, RigidBodyDesc, Shape};

fn frictionless_bouncy_material(materials: &mut MaterialDb, restitution: f64) -> MaterialId {
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

const DT: f64 = 1.0 / 1200.0;
const V0: f64 = 300.0;
const PLATE_HALF_THICKNESS: f64 = 0.001; // 板厚 2mm
/// 弾丸の初期 x。**ちょうど -1.0 にすると 1 ステップの移動量 0.25 m の整数倍で
/// 板の中心 x=0 に着地してしまい、離散検出が偶然に重なりを捉えてしまう**
/// (実装検証中に発見: 箱の対照実験が「フルCCD 無しでも貫通しない」と出た原因が
/// これだった。すり抜けの検証としては無意味なので、格子と非整合な位置から撃つ)。
const START_X: f64 = -1.013;

/// 弾丸が板の反対側へ正の速度で抜けたか(= トンネリングしたか)。
fn fire_and_check_tunnelling(
    bullet_shape: &Shape,
    target_is_dynamic: bool,
    full_ccd: bool,
) -> bool {
    let mut materials = MaterialDb::standard();
    let mat = frictionless_bouncy_material(&mut materials, 0.5);

    let mut solver = MechanicsSolver::new(0.0); // 重力なし(水平弾道)
    solver.restitution_velocity_threshold = 0.0;
    solver.set_full_ccd(full_ccd);

    let bullet_half = match *bullet_shape {
        Shape::Sphere { radius } => radius,
        Shape::Box { half_extents } => half_extents.x,
        _ => unreachable!("this test only fires spheres and boxes"),
    };
    let mut bullet = RigidBodyDesc::dynamic(bullet_shape.clone(), mat);
    bullet.transform.position = Vec3::new(START_X, 0.0, 0.0);
    bullet.linear_velocity = Vec3::new(V0, 0.0, 0.0);
    let idx = solver.create_body(bullet, &materials);

    let mut plate = RigidBodyDesc::dynamic(
        Shape::Box {
            half_extents: Vec3::new(PLATE_HALF_THICKNESS, 0.5, 0.5),
        },
        mat,
    );
    if target_is_dynamic {
        // 重い動的板(撃たれてもほとんど動かないが、最小CCD からは Dynamic として
        // スキップされる)。
        plate.mass_override = Some(1.0e4);
    } else {
        plate.body_type = BodyType::Static;
    }
    solver.create_body(plate, &materials);

    let mut tunneled = false;
    for _ in 0..60 {
        step_n(&mut solver, &materials, DT, 1);
        let x = solver.bodies.position[idx].x;
        let vx = solver.bodies.linear_velocity[idx].x;
        if x > PLATE_HALF_THICKNESS + bullet_half && vx > 0.0 {
            tunneled = true;
        }
    }
    tunneled
}

/// このシーンが本当に「1ステップで板を飛び越す」条件になっていること。
/// これが成り立たなければ以下の対照実験は何も示さない。
#[test]
fn the_scene_is_actually_a_tunnelling_scene() {
    let travel_per_step = V0 * DT;
    let plate_thickness = 2.0 * PLATE_HALF_THICKNESS;
    assert!(
        travel_per_step > 100.0 * plate_thickness,
        "travel_per_step={travel_per_step} must dwarf plate_thickness={plate_thickness}"
    );
}

/// **対照実験1**: 高速の球 × **動的**な薄板。最小CCD は Dynamic な相手を丸ごと
/// スキップするため、フルCCD を切ると貫通する。
#[test]
fn full_ccd_is_what_stops_a_bullet_against_a_dynamic_plate() {
    let sphere = Shape::Sphere { radius: 0.005 };
    assert!(
        fire_and_check_tunnelling(&sphere, true, false),
        "control: without full CCD the bullet must tunnel through the dynamic plate \
         (the speculative pass skips dynamic targets)"
    );
    assert!(
        !fire_and_check_tunnelling(&sphere, true, true),
        "full CCD must stop the bullet at the dynamic plate"
    );
}

/// **対照実験2**: 球以外の弾丸(高速の**箱**)× 静的な壁。最小CCD は球以外を
/// 丸ごとスキップするため、これもフルCCD を切ると貫通する。
#[test]
fn full_ccd_is_what_stops_a_fast_box() {
    let cube = Shape::Box {
        half_extents: Vec3::new(0.005, 0.005, 0.005),
    };
    assert!(
        fire_and_check_tunnelling(&cube, false, false),
        "control: without full CCD a fast box must tunnel \
         (the speculative pass only handles spheres)"
    );
    assert!(
        !fire_and_check_tunnelling(&cube, false, true),
        "full CCD must stop the fast box"
    );
}

/// **既存経路の不変性**: 球 × 静的板は最小CCD だけで既に防げている(M15)。
/// フルCCD を足しても切っても、どちらでも貫通しないこと——新機能が既存の
/// 合格を肩代わりしているのではないことの確認。
#[test]
fn the_existing_speculative_path_still_stands_on_its_own() {
    let sphere = Shape::Sphere { radius: 0.005 };
    assert!(
        !fire_and_check_tunnelling(&sphere, false, false),
        "M15 (sphere vs static plate) must still pass with the speculative pass alone"
    );
    assert!(
        !fire_and_check_tunnelling(&sphere, false, true),
        "and must keep passing with full CCD enabled"
    );
}

/// **偽陽性が無いこと**: 低速シーン(弾丸級判定に引っかからない)ではフルCCD が
/// 一切走らないので、有無で結果が**ビット一致**する。
#[test]
fn full_ccd_leaves_slow_scenes_bit_identical() {
    let materials = MaterialDb::standard();
    let wood = materials.find_by_name("木材(松)").unwrap();

    let run = |full_ccd: bool| {
        let mut solver = MechanicsSolver::new(9.80665);
        solver.set_full_ccd(full_ccd);
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
        let mut indices = Vec::new();
        for level in 0..4 {
            let mut desc = RigidBodyDesc::dynamic(
                Shape::Box {
                    half_extents: Vec3::new(0.5, 0.5, 0.5),
                },
                wood,
            );
            desc.transform.position = Vec3::new(0.0, 0.5 + level as f64, 0.0);
            indices.push(solver.create_body(desc, &materials));
        }
        step_n(&mut solver, &materials, 1.0 / 120.0, 1200);
        indices
            .iter()
            .map(|&i| solver.bodies.position[i].y)
            .collect::<Vec<f64>>()
    };

    let without = run(false);
    let with = run(true);
    assert_eq!(
        without, with,
        "full CCD must not perturb scenes that never trigger the bullet-grade test"
    );
    for (level, y) in with.iter().enumerate() {
        let expected = 0.5 + level as f64;
        assert!(
            (y - expected).abs() < 0.01,
            "stack level {level} must stay in place: y={y} expected≈{expected}"
        );
    }
}
