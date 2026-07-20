//! 保存則テスト。定義: docs/21-verification/02-conservation-laws.md §1。
//! ユニットテストではなく crate 公開 API 経由の統合テスト(World 層が無い Phase A 時点の代替)。

use sim_core::{Event, EventQueue, MaterialDb, Solver, SolverContext};
use sim_math::{SimRng, Vec3};
use sim_mechanics::{MechanicsSolver, RigidBodyDesc, Shape};

/// 角運動量・回転運動エネルギー保存(外トルクなし・接触なしの自由回転)。
/// 設計 docs/21-verification/02-conservation-laws.md §1「角運動量: 外トルクなし → 保存」、
/// docs/10-mechanics/01-rigid-body.md §7「ジャイロ項の陽的積分では L にドリフトが出る —
/// 許容ドリフト率を測定し文書化、陰的モードで消えることを確認」。
///
/// 本実装(陽的ジャイロ)は非対称剛体(3軸慣性が異なる箱)の自由回転で、既定 dt=1/120・1秒で
/// |L|・回転運動エネルギーとも相対 1% 未満のドリフトに収まることを測定した(実測値は
/// |L| ≈0.52%、KE ≈0.79%)。許容を 2%(実測の約2.5倍のマージン)として、将来の陰的ジャイロ
/// モード(Phase 2、design docs/10-mechanics/01 §8)導入時にドリフトが大幅に減ることを
/// 検出できるようにする(退行検知)。
#[test]
fn free_asymmetric_rotation_conserves_angular_momentum_and_kinetic_energy() {
    let materials = MaterialDb::standard();
    let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
    let mut solver = MechanicsSolver::new(0.0); // 重力なし(外力・外トルクなしの孤立系)

    // 3軸の半辺が全て異なる箱(慣性が非対称、テニスラケット定理的な取り扱いにも使える設定)。
    let half_extents = Vec3::new(0.3, 0.5, 0.8);
    let mut desc = RigidBodyDesc::dynamic(Shape::Box { half_extents }, steel);
    desc.angular_velocity = Vec3::new(0.7, 1.3, 2.1); // 3軸とも非ゼロの一般回転
    let idx = solver.create_body(desc, &materials);

    let angular_momentum = |solver: &MechanicsSolver| -> f64 {
        let inv_iw = solver.bodies.inv_inertia_world[idx];
        let iw = inv_iw.inverse().expect("inertia tensor must be invertible");
        iw.mul_vec(solver.bodies.angular_velocity[idx]).length()
    };
    let rotational_kinetic_energy = |solver: &MechanicsSolver| -> f64 {
        let inv_iw = solver.bodies.inv_inertia_world[idx];
        let iw = inv_iw.inverse().expect("inertia tensor must be invertible");
        let w = solver.bodies.angular_velocity[idx];
        0.5 * w.dot(iw.mul_vec(w))
    };

    let l0 = angular_momentum(&solver);
    let ke0 = rotational_kinetic_energy(&solver);
    // 並進エネルギーがゼロであることを確認(total_energy() とのクロスチェック用の前提)。
    assert_eq!(solver.bodies.linear_velocity[idx], Vec3::ZERO);
    assert!((solver.total_energy().kinetic - ke0).abs() < 1e-9);

    let dt = 1.0 / 120.0;
    let mut rng = SimRng::new(1, 1);
    let mut events = EventQueue::new();
    for _ in 0..120 {
        let mut ctx = SolverContext {
            materials: &materials,
            rng: &mut rng,
            events: &mut events,
        };
        solver.step(dt, &mut ctx);
        let _: Vec<Event> = events.drain_sorted();
    }

    let l1 = angular_momentum(&solver);
    let ke1 = rotational_kinetic_energy(&solver);

    let l_drift = (l1 - l0).abs() / l0;
    let ke_drift = (ke1 - ke0).abs() / ke0;
    assert!(
        l_drift < 0.02,
        "angular momentum drift too large: {l_drift}"
    );
    assert!(
        ke_drift < 0.02,
        "kinetic energy drift too large: {ke_drift}"
    );
}
