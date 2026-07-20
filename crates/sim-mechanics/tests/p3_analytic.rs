//! P3 解析解テスト(M3, M4, M10)。定義: docs/21-verification/01-analytic-tests.md。
//! 単振り子を「質点 + ワールド固定支点への Distance ジョイント(質量無しの棒/紐)」として
//! 表現する(docs/10-mechanics/05-joints-constraints.md、`DistanceJoint`)。独楽(M10)は
//! 「重心からオフセットした支点をワールド固定する Ball ジョイント」で表現する(`BallJoint`)。

use sim_core::{Event, EventQueue, MaterialDb, Solver, SolverContext};
use sim_math::{Quat, SimRng, Vec3};
use sim_mechanics::{BallJoint, DistanceJoint, MechanicsSolver, RigidBodyDesc, Shape};

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

/// M10: 独楽の歳差 $\dot\phi=mgr/(I\omega)$(速い自転極限)、rel 2%
/// (docs/21-verification/01-analytic-tests.md M10)。重心から距離 `d` オフセットした支点を
/// `BallJoint`(`body_b=None`)でワールド固定し、自転軸を鉛直から角度 θ0 傾けて大きな自転
/// 角速度 ω0 を与えると、自転軸(=重心の水平位置)が鉛直まわりに歳差運動する。
/// 等方慣性の球(慣性テンソルがスカラー)を使うため、この歳差速度公式は「速い自転」の
/// 近似ではなく厳密になる(非等方項 $(I_1-I_3)\dot\phi^2\cos\theta$ が恒等的に消える)—
/// ただし章動(自転軸の周期的な揺れ)は残るため、ω0 を十分大きく(章動振幅を歳差信号に
/// 対して無視できる水準まで)取り、短時間平均で歳差速度を実測する。
#[test]
fn m10_top_precession_rate_matches_mgr_over_i_omega() {
    let materials = MaterialDb::standard();
    let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
    let gravity = 9.80665;

    let mass = 1.0;
    let radius = 0.05; // 球半径(慣性計算用)
    let pivot_offset = 0.1; // 支点から重心までの距離 d
    let theta0: f64 = 0.3; // 鉛直からの傾き(rad)
    let omega0 = 1000.0; // 自転角速度(rad/s、速い自転極限を満たす値)
    let inertia = 2.0 / 5.0 * mass * radius * radius;
    let expected_phi_dot = mass * gravity * pivot_offset / (inertia * omega0);

    let mut solver = MechanicsSolver::new(gravity);
    let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, steel);
    desc.mass_override = Some(mass);
    desc.transform.rotation = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), theta0);
    let anchor_a = Vec3::new(0.0, -pivot_offset, 0.0);
    let pivot = Vec3::ZERO;
    let r_a = desc.transform.rotation.to_mat3().mul_vec(anchor_a);
    desc.transform.position = pivot - r_a;
    let spin_dir = desc
        .transform
        .rotation
        .to_mat3()
        .mul_vec(Vec3::new(0.0, 1.0, 0.0));
    desc.angular_velocity = spin_dir.scale(omega0);
    let idx = solver.create_body(desc, &materials);
    solver.add_ball_joint(BallJoint {
        body_a: idx,
        anchor_a,
        body_b: None,
        anchor_b: pivot,
    });

    let dt = 1.0 / 20_000.0;
    let duration = 1.0;
    let steps = (duration / dt) as u32;
    let mut rng = SimRng::new(1, 1);
    let mut events = EventQueue::new();

    let start_pos = solver.bodies.position[idx];
    let mut prev_angle = (start_pos.z - pivot.z).atan2(start_pos.x - pivot.x);
    let mut unwrapped_angle = 0.0;
    for _ in 0..steps {
        let mut ctx = SolverContext {
            materials: &materials,
            rng: &mut rng,
            events: &mut events,
        };
        solver.step(dt, &mut ctx);
        let _: Vec<Event> = events.drain_sorted();

        let pos = solver.bodies.position[idx];
        let angle = (pos.z - pivot.z).atan2(pos.x - pivot.x);
        let mut delta = angle - prev_angle;
        if delta > std::f64::consts::PI {
            delta -= 2.0 * std::f64::consts::PI;
        } else if delta < -std::f64::consts::PI {
            delta += 2.0 * std::f64::consts::PI;
        }
        unwrapped_angle += delta;
        prev_angle = angle;
    }

    // 支点がほぼ固定されたままであること(拘束が機能していることの確認)。
    let tip_world =
        solver.bodies.position[idx] + solver.bodies.rotation[idx].to_mat3().mul_vec(anchor_a);
    assert!(
        (tip_world - pivot).length() < 1e-3,
        "pivot should stay fixed, drifted to {tip_world:?}"
    );

    let measured_phi_dot = (unwrapped_angle / (steps as f64 * dt)).abs();
    let rel_err = (measured_phi_dot - expected_phi_dot).abs() / expected_phi_dot;
    assert!(
        rel_err < 0.02,
        "measured={measured_phi_dot} expected={expected_phi_dot} rel_err={rel_err}"
    );
}
