//! `GridFluid3D`(**群9で追加**)の受け入れテスト。
//! 設計 docs/11-fluid/02-eulerian-grid.md §3(3D状態表現)・§7(検証)。
//!
//! 一番強い検証は **2D版との交差検証**である: z方向に一様な初期条件を与えると、
//! 3Dソルバは既に Green な2Dソルバと一致するはず——離散化が2Dと整合していることを、
//! 検証済みの資産をそのまま使って立証できる。

use sim_core::Solver;
use sim_fluid::{CellType, GridBoundary, GridBoundary3D, GridFluid2D, GridFluid3D};
use sim_math::Vec3;

/// 3D Taylor-Green(z方向に一様、= 2Dの Taylor-Green を厚み方向へ押し出したもの)。
fn taylor_green_3d(nx: usize, ny: usize, nz: usize, h: f64) -> GridFluid3D {
    let length = h * nx as f64;
    let k = 2.0 * std::f64::consts::PI / length;
    let mut fluid = GridFluid3D::new(nx, ny, nz, h);
    for kk in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                let idx = i + nx * (j + ny * kk);
                fluid.u[idx] = -(k * (i as f64 * h)).cos() * (k * ((j as f64 + 0.5) * h)).sin();
                fluid.v[idx] = (k * ((i as f64 + 0.5) * h)).sin() * (k * (j as f64 * h)).cos();
                fluid.w[idx] = 0.0;
            }
        }
    }
    fluid
}

fn taylor_green_2d(nx: usize, ny: usize, h: f64) -> GridFluid2D {
    let length = h * nx as f64;
    let k = 2.0 * std::f64::consts::PI / length;
    let mut fluid = GridFluid2D::new(nx, ny, h);
    for j in 0..ny {
        for i in 0..nx {
            let idx = i + nx * j;
            fluid.u[idx] = -(k * (i as f64 * h)).cos() * (k * ((j as f64 + 0.5) * h)).sin();
            fluid.v[idx] = (k * ((i as f64 + 0.5) * h)).sin() * (k * (j as f64 * h)).cos();
        }
    }
    fluid
}

fn max_abs_divergence_3d(fluid: &GridFluid3D) -> f64 {
    let mut worst: f64 = 0.0;
    for k in 0..fluid.nz as i64 {
        for j in 0..fluid.ny as i64 {
            for i in 0..fluid.nx as i64 {
                if fluid.cell_type_at(i, j, k) == CellType::Solid {
                    continue;
                }
                worst = worst.max(fluid.divergence(i, j, k).abs());
            }
        }
    }
    worst
}

/// **設計§7「発散: 投影後 $|\nabla\cdot u| < 10^{-6}$」**。
/// 非発散フリーな適当な場を1回投影して確認する(2D版の F9 の3D対応物)。
#[test]
fn divergence_after_a_single_projection_is_near_zero() {
    let (nx, ny, nz) = (12, 12, 12);
    let h = 1.0 / nx as f64;
    let mut fluid = GridFluid3D::new(nx, ny, nz, h);
    let k = 2.0 * std::f64::consts::PI;
    for kk in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                let idx = i + nx * (j + ny * kk);
                fluid.u[idx] = (k * (i as f64 * h)).sin();
                fluid.v[idx] = (k * (j as f64 * h)).sin();
                fluid.w[idx] = (k * (kk as f64 * h)).sin();
            }
        }
    }
    assert!(
        max_abs_divergence_3d(&fluid) > 1.0,
        "the initial field must actually be compressible for this test to mean anything"
    );

    fluid.project(0.01, 1.0);
    let divergence = max_abs_divergence_3d(&fluid);
    assert!(
        divergence < 1e-6,
        "divergence after projection={divergence:e} (design §7 requires < 1e-6)"
    );
}

/// **設計§7「Taylor-Green渦: 減衰率 $e^{-2\nu k^2t}$」**(< 5%)。
/// 設計が書く $e^{-2\nu k^2t}$ は**速度**の減衰で、ここで測る運動エネルギーはその2乗
/// なので $e^{-4\nu k^2t}$ になる(2D版 F8 も `analytic_rate = 4νk²` で同じ扱い)。
#[test]
fn taylor_green_decay_matches_the_analytic_rate() {
    let (nx, ny, nz) = (32, 32, 4);
    let h = 1.0 / nx as f64;
    let k = 2.0 * std::f64::consts::PI;
    // 2D版(F8)と同じ理由で粘性を強めに取る: semi-Lagrangian の数値拡散が
    // 真の粘性減衰と同程度になる領域では解析解と比較できない。
    let nu = 0.2;
    let mut fluid = taylor_green_3d(nx, ny, nz, h);
    fluid.kinematic_viscosity = nu;

    let kinetic = |f: &GridFluid3D| -> f64 {
        f.u.iter().map(|x| x * x).sum::<f64>()
            + f.v.iter().map(|x| x * x).sum::<f64>()
            + f.w.iter().map(|x| x * x).sum::<f64>()
    };
    let ke0 = kinetic(&fluid);

    let dt = 0.0005;
    let steps = 120;
    for _ in 0..steps {
        fluid.step(dt);
    }
    let ke1 = kinetic(&fluid);

    // **最初 $e^{-2\nu k^2t}$ と書いてこのテストを落とした**(実測 0.1439 対 期待 0.3877)。
    // 2Dソルバで同じ条件を測ると 3D と同じ 0.14387 になり、食い違っているのは
    // ソルバではなく期待値の式だと切り分けられた。波数ベクトルの大きさが $k\sqrt2$
    // なので速度が $e^{-2\nu k^2t}$、エネルギーはその2乗で $e^{-4\nu k^2t}$ が正しい。
    let total_time = dt * steps as f64;
    let measured_rate = -(ke1 / ke0).ln() / total_time;
    let analytic_rate = 4.0 * nu * k * k;
    let rel_err = (measured_rate - analytic_rate).abs() / analytic_rate;
    assert!(
        rel_err < 0.05,
        "measured_rate={measured_rate:.6} analytic_rate={analytic_rate:.6} rel_err={rel_err:.4}"
    );
}

/// **2Dとの交差検証(この増分で一番強い検証)**: z方向に一様な初期条件では、
/// 3Dソルバは検証済みの2Dソルバと一致しなければならない(`w` は恒等的に0のまま、
/// `u`/`v` は2Dと同じ値)。離散化が2Dと整合していることの直接の証明。
#[test]
fn a_z_invariant_setup_reproduces_the_verified_two_dimensional_solver() {
    let (nx, ny, nz) = (16, 16, 3);
    let h = 1.0 / nx as f64;
    let nu = 0.2;

    let mut fluid3d = taylor_green_3d(nx, ny, nz, h);
    fluid3d.kinematic_viscosity = nu;
    let mut fluid2d = taylor_green_2d(nx, ny, h);
    fluid2d.kinematic_viscosity = nu;

    let dt = 0.0005;
    for _ in 0..30 {
        fluid3d.step(dt);
        fluid2d.step(dt);
    }

    let mut worst_u: f64 = 0.0;
    let mut worst_v: f64 = 0.0;
    let mut worst_w: f64 = 0.0;
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                let idx3 = i + nx * (j + ny * k);
                let idx2 = i + nx * j;
                worst_u = worst_u.max((fluid3d.u[idx3] - fluid2d.u[idx2]).abs());
                worst_v = worst_v.max((fluid3d.v[idx3] - fluid2d.v[idx2]).abs());
                worst_w = worst_w.max(fluid3d.w[idx3].abs());
            }
        }
    }
    // **どこまで一致するかは実測で決めた**。移流と粘性拡散は**倍精度の丸め誤差の
    // 範囲(2e-16)で一致する**ことをステージ別に確認済みで、差が出るのは投影だけ:
    // 周期ポアソンは特異(定数が零空間)で、PCG は相対残差 1e-8 で止まるため、
    // 2つの解は零空間近傍の成分ぶんだけずれうる。1回の投影で 2.5e-5、30ステップで
    // 1.1e-4 まで蓄積する——**離散化のずれではなく反復解法の許容誤差**である
    // (3Dの7点ステンシルは z 一様なベクトルに対して 2Dの5点ステンシルと代数的に
    // 厳密に等しくなる: z方向の2つの隣接項が自分自身になり対角が -6 から -4 へ落ちる)。
    assert!(
        worst_u < 1e-3 && worst_v < 1e-3,
        "the z-invariant 3D solution must match the 2D solver: \
         worst_u={worst_u:e} worst_v={worst_v:e}"
    );
    assert!(
        worst_w < 1e-12,
        "w must stay identically zero in a z-invariant setup: worst_w={worst_w:e}"
    );
}

/// 受動スカラー(煙、設計§3 `smoke_density`)が速度場に乗って運ばれること。
/// **3D にする実用上の意味そのもの**なので、移流されることを直接確認する。
#[test]
fn smoke_is_advected_downstream_by_the_flow() {
    let (nx, ny, nz) = (32, 16, 16);
    let h = 1.0 / ny as f64;
    let mut fluid = GridFluid3D::new(nx, ny, nz, h)
        .with_boundary(GridBoundary3D::Channel { inflow_speed: 1.0 });

    // 上流側に煙の塊を置く。
    let blob_i = 4;
    for k in 6..10 {
        for j in 6..10 {
            for i in blob_i..blob_i + 3 {
                let idx = i + nx * (j + ny * k);
                fluid.smoke_density[idx] = 1.0;
            }
        }
    }
    let column_sum = |f: &GridFluid3D, i: usize| -> f64 {
        let mut s = 0.0;
        for k in 0..nz {
            for j in 0..ny {
                s += f.smoke_density[i + nx * (j + ny * k)];
            }
        }
        s
    };
    let upstream_before = column_sum(&fluid, blob_i + 1);
    let downstream_before = column_sum(&fluid, blob_i + 12);
    assert!(upstream_before > 0.0 && downstream_before == 0.0);

    // 流速1 m/s、h=1/16 なので、12セル進むのに約 0.75 s。
    for _ in 0..400 {
        fluid.step(0.002);
    }

    let upstream_after = column_sum(&fluid, blob_i + 1);
    let downstream_after = column_sum(&fluid, blob_i + 12);
    assert!(
        downstream_after > 0.1,
        "smoke must reach the downstream column: {downstream_after}"
    );
    assert!(
        upstream_after < upstream_before,
        "and must leave the upstream column: before={upstream_before} after={upstream_after}"
    );
}

/// 3Dの固体境界: 球を埋め込むと、その内部へ流体が入らず、流体セルは非圧縮のまま。
#[test]
fn an_embedded_sphere_blocks_the_flow_without_breaking_incompressibility() {
    let (nx, ny, nz) = (24, 16, 16);
    let h = 1.0 / ny as f64;
    let mut fluid = GridFluid3D::new(nx, ny, nz, h)
        .with_boundary(GridBoundary3D::Channel { inflow_speed: 1.0 });
    fluid.kinematic_viscosity = 1.0e-3;

    let center = Vec3::new(0.5, 0.5, 0.5);
    let radius = 0.15;
    fluid.set_solid_cells(|x, y, z| {
        let d = Vec3::new(x, y, z) - center;
        if d.length() < radius {
            Some(Vec3::ZERO)
        } else {
            None
        }
    });

    let i_c = (center.x / h) as i64;
    let j_c = (center.y / h) as i64;
    let k_c = (center.z / h) as i64;
    assert_eq!(fluid.cell_type_at(i_c, j_c, k_c), CellType::Solid);
    // 球の外接立方体の角は Fluid(= 立方体マスクではない)。
    let corner = 0.55 * radius;
    assert_eq!(
        fluid.cell_type_at(
            ((center.x + corner) / h) as i64,
            ((center.y + corner) / h) as i64,
            ((center.z + corner) / h) as i64
        ),
        CellType::Fluid
    );

    for _ in 0..150 {
        fluid.step(0.003);
    }

    assert!(
        fluid.u_at(i_c, j_c, k_c).abs() < 1e-9
            && fluid.v_at(i_c, j_c, k_c).abs() < 1e-9
            && fluid.w_at(i_c, j_c, k_c).abs() < 1e-9,
        "fluid must not leak into the solid"
    );

    // 流入層・流出層は設計上そもそも発散ゼロにならないので内部だけを見る
    // (流入=速度Dirichlet、流出=圧力Dirichletの恒等行。2D版と同じ)。
    let mut worst: f64 = 0.0;
    for k in 0..nz as i64 {
        for j in 0..ny as i64 {
            for i in 1..nx as i64 - 1 {
                if fluid.cell_type_at(i, j, k) == CellType::Solid {
                    continue;
                }
                worst = worst.max(fluid.divergence(i, j, k).abs());
            }
        }
    }
    assert!(worst < 1e-6, "interior divergence={worst:e}");

    // 球は下流へ押される(圧力積分の符号が正しいこと)。
    let force = fluid
        .pressure_force_on_solid()
        .expect("a solid was embedded");
    assert!(
        force.x > 0.0,
        "the sphere must be pushed downstream: force={force:?}"
    );
}

/// 近似バッジが設定に追従すること(2D版と同じ規律)。
/// 3Dでは「前処理なしPCG」が常時の近似として申告される——設計§10の 64³/4ms 予算に
/// 届かないことを黙って隠さない。
#[test]
fn approximations_follow_the_configuration() {
    let mut fluid = GridFluid3D::new(8, 8, 8, 0.1);
    let names = |f: &GridFluid3D| -> Vec<&'static str> {
        f.approximations().iter().map(|a| a.name).collect()
    };
    assert!(names(&fluid).contains(&"3D・周期境界"));
    assert!(names(&fluid).contains(&"前処理なしPCG"));
    assert!(!names(&fluid).contains(&"渦度強化(非物理)"));

    fluid.vorticity_confinement_epsilon = 1.0;
    assert!(names(&fluid).contains(&"渦度強化(非物理)"));

    let channel =
        GridFluid3D::new(8, 8, 8, 0.1).with_boundary(GridBoundary3D::Channel { inflow_speed: 1.0 });
    assert!(names(&channel).contains(&"3D・開境界(流路)"));
    assert!(!names(&channel).contains(&"3D・周期境界"));

    for a in fluid.approximations() {
        assert!(a.doc.starts_with("docs/"), "出典を持つべき: {a:?}");
        assert!(!a.reason.is_empty(), "理由を持つべき: {a:?}");
    }
}

/// 2D版と `GridBoundary` が意味的に対応していること(型は別だが挙動の対応を固定する)。
#[test]
fn the_two_and_three_dimensional_boundary_enums_stay_in_step() {
    let fluid2d =
        GridFluid2D::new(4, 4, 0.1).with_boundary(GridBoundary::Channel { inflow_speed: 2.0 });
    let fluid3d =
        GridFluid3D::new(4, 4, 4, 0.1).with_boundary(GridBoundary3D::Channel { inflow_speed: 2.0 });
    // どちらも流入速度で場を初期化する。
    assert!(fluid2d.u.iter().all(|x| *x == 2.0));
    assert!(fluid3d.u.iter().all(|x| *x == 2.0));
    assert!(fluid3d.w.iter().all(|x| *x == 0.0));
}
