//! P3 解析解テスト(M3, M4)。定義: docs/21-verification/01-analytic-tests.md。
//! 単振り子を「質点 + ワールド固定支点への Distance ジョイント(質量無しの棒/紐)」として
//! 表現する(docs/10-mechanics/05-joints-constraints.md、`DistanceJoint`)。

use sim_core::{Event, EventQueue, MaterialDb, Solver, SolverContext};
use sim_math::{SimRng, Vec3};
use sim_mechanics::{DistanceJoint, MechanicsSolver, RigidBodyDesc, Shape};

/// 算術幾何平均(AGM)で完全楕円積分 $K(k)=\pi/(2\,\mathrm{agm}(1,\sqrt{1-k^2}))$ を計算する
/// (M4 の大振幅振り子周期の理論値に使う)。
fn complete_elliptic_k(k: f64) -> f64 {
    let (mut a, mut b) = (1.0, (1.0 - k * k).sqrt());
    for _ in 0..40 {
        let a_next = 0.5 * (a + b);
        let b_next = (a * b).sqrt();
        a = a_next;
        b = b_next;
    }
    std::f64::consts::PI / (2.0 * a)
}

/// 振れ角(鉛直下向きを0とする、支点からの相対位置から算出)。
fn pendulum_angle(pos: Vec3, pivot: Vec3) -> f64 {
    let dx = pos.x - pivot.x;
    let dy = pos.y - pivot.y;
    dx.atan2(-dy)
}

/// 振り子を θ0(鉛直から測った振れ角)・長さ L・重力 g で構築し、`steps` 回シミュレートして
/// 最初の2回の鉛直通過(角度0通過)の時刻差から周期を実測する(A1 と同じ手法: 線形補間で
/// 離散検出誤差を O(dt/T) から O(dt^2/T) に落とす)。
fn measure_pendulum_period(theta0: f64, length: f64, gravity: f64, dt: f64, steps: u32) -> f64 {
    let materials = MaterialDb::standard();
    let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();

    let mut solver = MechanicsSolver::new(gravity);
    let pivot = Vec3::ZERO;
    let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.01 }, steel);
    desc.mass_override = Some(1.0);
    desc.transform.position = pivot + Vec3::new(theta0.sin() * length, -theta0.cos() * length, 0.0);
    let idx = solver.create_body(desc, &materials);
    solver.add_distance_joint(DistanceJoint {
        body_a: idx,
        anchor_a: Vec3::ZERO,
        body_b: None,
        anchor_b: pivot,
        length,
    });

    let mut rng = SimRng::new(1, 1);
    let mut events = EventQueue::new();
    let mut prev_angle = pendulum_angle(solver.bodies.position[idx], pivot);
    let mut prev_t = 0.0;
    let mut crossings = Vec::new();
    for step in 0..steps {
        let mut ctx = SolverContext {
            materials: &materials,
            rng: &mut rng,
            events: &mut events,
        };
        solver.step(dt, &mut ctx);
        let _: Vec<Event> = events.drain_sorted();

        let t = (step + 1) as f64 * dt;
        let angle = pendulum_angle(solver.bodies.position[idx], pivot);
        if prev_angle.signum() != angle.signum() && prev_angle != 0.0 {
            let frac = -prev_angle / (angle - prev_angle);
            crossings.push(prev_t + frac * (t - prev_t));
            if crossings.len() >= 2 {
                break;
            }
        }
        prev_angle = angle;
        prev_t = t;
    }

    assert!(
        crossings.len() >= 2,
        "pendulum should cross vertical twice within the simulated window"
    );
    2.0 * (crossings[1] - crossings[0])
}

/// M3: 単振り子(小振幅)周期 $T=2\pi\sqrt{L/g}$、rel 1%(docs/21-verification/01-analytic-tests.md M3)。
#[test]
fn m3_small_amplitude_pendulum_period_matches_2pi_sqrt_l_over_g() {
    let length: f64 = 1.0;
    let gravity: f64 = 9.80665;
    let theta0 = 0.05; // 小振幅(rad)
    let dt = 1.0 / 2000.0;

    let analytic_period = 2.0 * std::f64::consts::PI * (length / gravity).sqrt();
    let steps = (1.2 * analytic_period / dt) as u32;
    let measured = measure_pendulum_period(theta0, length, gravity, dt, steps);

    let rel_err = (measured - analytic_period).abs() / analytic_period;
    assert!(
        rel_err < 0.01,
        "measured={measured} analytic={analytic_period} rel_err={rel_err}"
    );
}

/// M4: 単振り子(振幅90°)周期 = 楕円積分の解析値、rel 1%(docs/21-verification/01-analytic-tests.md M4)。
#[test]
fn m4_large_amplitude_pendulum_period_matches_elliptic_integral() {
    let length: f64 = 1.0;
    let gravity: f64 = 9.80665;
    let theta0 = std::f64::consts::FRAC_PI_2; // 90°
    let dt = 1.0 / 4000.0;

    let k = (theta0 / 2.0).sin();
    let analytic_period = 4.0 * (length / gravity).sqrt() * complete_elliptic_k(k);
    let steps = (1.2 * analytic_period / dt) as u32;
    let measured = measure_pendulum_period(theta0, length, gravity, dt, steps);

    let rel_err = (measured - analytic_period).abs() / analytic_period;
    assert!(
        rel_err < 0.01,
        "measured={measured} analytic={analytic_period} rel_err={rel_err}"
    );
}
