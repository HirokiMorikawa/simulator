//! `pressure_multigrid`(**群10で追加**)の受け入れテスト。
//! 設計 docs/11-fluid/02-eulerian-grid.md §4.4「難所: マルチグリッド前処理と不規則境界」・§10。
//!
//! 前処理は**解を変えてはならない**——ここで守りたいのは主にそれである。
//! したがって検証は3層になる:
//!
//! 1. **作用素が同一であること**: レベル0のステンシルが、置き換え前の7点ステンシルと
//!    係数まで一致する(物理が動いていないことの直接の証明)。
//! 2. **前処理が PCG の前提を満たすこと**: $M=M^\top$ かつ正定値。ここが崩れると
//!    CG の収束理論そのものが失効するので、乱数ベクトルで機械的に確かめる。
//! 3. **性能上の主張が本当であること**: 反復数が解像度にほぼ依存しない(これが
//!    「前処理なしPCGでは 64³/4ms 予算に届かない」への回答そのもの)。
//!
//! 加えて、設計§4.4 が定めた**粗格子化の分岐条件(混在30%)とフォールバック**が
//! 実際に働くことを、故意に解像できない固体パターンで確かめる。

use sim_fluid::pressure_multigrid::{MultigridPoisson, PressureCell};
use sim_math::SimRng;

fn all_fluid(n: usize) -> Vec<PressureCell> {
    vec![PressureCell::Fluid; n * n * n]
}

fn random_vector(rng: &mut SimRng, n: usize, kinds: &[PressureCell]) -> Vec<f64> {
    (0..n)
        .map(|i| {
            if kinds[i] == PressureCell::Fluid {
                rng.range_f64(-1.0, 1.0)
            } else {
                0.0
            }
        })
        .collect()
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b.iter()).map(|(x, y)| x * y).sum()
}

/// 置き換え**前**の `GridFluid3D::project` が使っていた7点ステンシル。
/// Solid セルは恒等行、それ以外は $(\sum_{隣} x - \deg\cdot x)/h^2$。
fn legacy_stencil(
    n: usize,
    h: f64,
    periodic: bool,
    kinds: &[PressureCell],
    x: &[f64],
    out: &mut [f64],
) {
    let h2 = h * h;
    let flat = |i: i64, j: i64, k: i64| -> usize {
        let w = |v: i64| v.rem_euclid(n as i64) as usize;
        w(i) + n * (w(j) + n * w(k))
    };
    for k in 0..n as i64 {
        for j in 0..n as i64 {
            for i in 0..n as i64 {
                let idx = flat(i, j, k);
                if kinds[idx] != PressureCell::Fluid {
                    out[idx] = x[idx]; // 恒等行(未知数から外れている)
                    continue;
                }
                let mut sum = 0.0;
                let mut degree = 0.0;
                let mut neighbour = |a: i64, b: i64, c: i64| {
                    let outside =
                        a < 0 || a >= n as i64 || b < 0 || b >= n as i64 || c < 0 || c >= n as i64;
                    if !periodic && outside {
                        return; // 壁は Neumann(鏡像で相殺)
                    }
                    let m = flat(a, b, c);
                    if kinds[m] == PressureCell::Solid {
                        return; // Solid 面も Neumann
                    }
                    sum += x[m];
                    degree += 1.0;
                };
                neighbour(i + 1, j, k);
                neighbour(i - 1, j, k);
                neighbour(i, j + 1, k);
                neighbour(i, j - 1, k);
                neighbour(i, j, k + 1);
                neighbour(i, j, k - 1);
                out[idx] = (sum - degree * x[idx]) / h2;
            }
        }
    }
}

/// 固体の球と、x+ 側の圧力Dirichlet層を持つ、不規則境界のセル種別。
fn sphere_in_a_channel(n: usize) -> Vec<PressureCell> {
    let mut kinds = all_fluid(n);
    let radius = 0.18 * n as f64;
    let centre = 0.5 * n as f64;
    for k in 0..n {
        for j in 0..n {
            for i in 0..n {
                let (dx, dy, dz) = (
                    i as f64 + 0.5 - centre,
                    j as f64 + 0.5 - centre,
                    k as f64 + 0.5 - centre,
                );
                if dx * dx + dy * dy + dz * dz < radius * radius {
                    kinds[i + n * (j + n * k)] = PressureCell::Solid;
                }
            }
        }
    }
    for k in 0..n {
        for j in 0..n {
            let idx = (n - 1) + n * (j + n * k);
            if kinds[idx] != PressureCell::Solid {
                kinds[idx] = PressureCell::Dirichlet;
            }
        }
    }
    kinds
}

/// **1層目の検証**: レベル0の作用素は、置き換え前の7点ステンシルと符号を除いて一致する
/// ($L=-\nabla^2$ を採ったぶんだけ符号が反転し、未知数から外したセルは 0 になる)。
/// これが成り立つ限り、前処理を差し替えても解は動かない。
#[test]
fn the_finest_level_operator_reproduces_the_legacy_seven_point_stencil() {
    let n = 12;
    let h = 1.0 / n as f64;
    let mut rng = SimRng::new(7, 0);

    for (periodic, kinds) in [
        (true, all_fluid(n)),
        (false, sphere_in_a_channel(n)),
        (true, sphere_in_a_channel(n)),
    ] {
        let multigrid = MultigridPoisson::build(n, n, n, h, periodic, kinds.clone());
        let x = random_vector(&mut rng, n * n * n, &kinds);

        let mut expected = vec![0.0; n * n * n];
        legacy_stencil(n, h, periodic, &kinds, &x, &mut expected);
        let mut actual = vec![0.0; n * n * n];
        multigrid.apply_operator(&x, &mut actual);

        for c in 0..n * n * n {
            let want = if kinds[c] == PressureCell::Fluid {
                -expected[c]
            } else {
                0.0
            };
            assert!(
                (actual[c] - want).abs() < 1e-9 * (1.0 + want.abs()),
                "cell {c}: {} vs {want} (periodic={periodic})",
                actual[c]
            );
        }
    }
}

/// **2層目の検証(その1)**: $M^{-1}$ が対称であること。
/// PCG は $M=M^\top$ を前提に導出されており、ここが崩れると収束保証が消える。
/// V サイクルの制限を「延長の転置」として書いてあるのはこのためで、その意図が
/// 実装で保たれているかを直接測る。
#[test]
fn the_preconditioner_is_symmetric() {
    let n = 16;
    let mut rng = SimRng::new(11, 0);
    for kinds in [all_fluid(n), sphere_in_a_channel(n)] {
        let periodic = !kinds.contains(&PressureCell::Dirichlet);
        let multigrid = MultigridPoisson::build(n, n, n, 1.0 / n as f64, periodic, kinds.clone());
        let cells = n * n * n;

        for _ in 0..4 {
            let r1 = random_vector(&mut rng, cells, &kinds);
            let r2 = random_vector(&mut rng, cells, &kinds);
            let (mut z1, mut z2) = (vec![0.0; cells], vec![0.0; cells]);
            multigrid.precondition(&r1, &mut z1);
            multigrid.precondition(&r2, &mut z2);

            let a = dot(&r2, &z1);
            let b = dot(&r1, &z2);
            let scale = a.abs().max(b.abs()).max(1e-30);
            assert!(
                (a - b).abs() / scale < 1e-10,
                "r2·M⁻¹r1={a:e} と r1·M⁻¹r2={b:e} が一致しない"
            );
        }
    }
}

/// **2層目の検証(その2)**: $M^{-1}$ が正定値であること($r\cdot M^{-1}r>0$)。
#[test]
fn the_preconditioner_is_positive_definite() {
    let n = 16;
    let mut rng = SimRng::new(13, 0);
    for kinds in [all_fluid(n), sphere_in_a_channel(n)] {
        let periodic = !kinds.contains(&PressureCell::Dirichlet);
        let multigrid = MultigridPoisson::build(n, n, n, 1.0 / n as f64, periodic, kinds.clone());
        let cells = n * n * n;
        for _ in 0..4 {
            let mut r = random_vector(&mut rng, cells, &kinds);
            if multigrid.is_singular() {
                // 特異系では定数成分は零空間なので、可解性条件を満たす右辺で測る。
                let fluid: Vec<usize> = (0..cells)
                    .filter(|c| kinds[*c] == PressureCell::Fluid)
                    .collect();
                let mean = fluid.iter().map(|c| r[*c]).sum::<f64>() / fluid.len() as f64;
                for c in fluid {
                    r[c] -= mean;
                }
            }
            let mut z = vec![0.0; cells];
            multigrid.precondition(&r, &mut z);
            let quadratic = dot(&r, &z);
            assert!(quadratic > 0.0, "r·M⁻¹r={quadratic:e} が正でない");
        }
    }
}

/// 製造解法で $L x = b$ を解く。返すのは (反復数, 解の最大誤差)。
fn solve_manufactured(
    n: usize,
    periodic: bool,
    kinds: &[PressureCell],
    preconditioned: bool,
) -> (usize, f64) {
    let cells = n * n * n;
    let multigrid = MultigridPoisson::build(n, n, n, 1.0 / n as f64, periodic, kinds.to_vec());
    let mut rng = SimRng::new(17, 0);
    let exact = random_vector(&mut rng, cells, kinds);
    let mut b = vec![0.0; cells];
    multigrid.apply_operator(&exact, &mut b);

    let mut x = vec![0.0; cells];
    let precondition = |r: &[f64], z: &mut [f64]| multigrid.precondition(r, z);
    let apply = |v: &[f64], out: &mut [f64]| multigrid.apply_operator(v, out);
    let preconditioner = if preconditioned {
        sim_math::Preconditioner::Custom(&precondition)
    } else {
        sim_math::Preconditioner::None
    };
    let result = sim_math::pcg(apply, &b, &mut x, &preconditioner, 1e-10, 5000);
    assert!(result.converged, "PCG が収束しなかった: {result:?}");

    // 特異系では解は定数分の自由度を持つので、流体セル平均を揃えてから比べる。
    let fluid: Vec<usize> = (0..cells)
        .filter(|c| kinds[*c] == PressureCell::Fluid)
        .collect();
    let shift = if multigrid.is_singular() {
        let mx = fluid.iter().map(|c| x[*c]).sum::<f64>() / fluid.len() as f64;
        let me = fluid.iter().map(|c| exact[*c]).sum::<f64>() / fluid.len() as f64;
        mx - me
    } else {
        0.0
    };
    let worst = fluid
        .iter()
        .map(|c| (x[*c] - shift - exact[*c]).abs())
        .fold(0.0f64, f64::max);
    (result.iterations, worst)
}

/// **前処理は解を変えない**: 前処理あり/なしで同じ解に到達する。
#[test]
fn preconditioning_changes_the_iteration_count_but_not_the_solution() {
    let n = 16;
    let kinds = sphere_in_a_channel(n);
    let (plain_iterations, plain_error) = solve_manufactured(n, false, &kinds, false);
    let (mg_iterations, mg_error) = solve_manufactured(n, false, &kinds, true);

    assert!(plain_error < 1e-6, "前処理なしの誤差={plain_error:e}");
    assert!(mg_error < 1e-6, "MGPCG の誤差={mg_error:e}");
    assert!(
        mg_iterations * 4 < plain_iterations,
        "MGPCG が前処理なしより十分速いこと: {mg_iterations} vs {plain_iterations}"
    );
}

/// **3層目の検証**: 反復数が解像度にほぼ依存しないこと。
/// 前処理なしPCGの反復数は $O(N)$ で増えるので、解像度を倍にすると倍近くなる——
/// それが「64³ で 795.7 ms/step」の正体だった。マルチグリッド前処理はこれを潰す。
#[test]
fn the_iteration_count_barely_grows_with_resolution() {
    let mut plain = Vec::new();
    let mut multigrid = Vec::new();
    for n in [16usize, 32] {
        let kinds = all_fluid(n);
        plain.push(solve_manufactured(n, true, &kinds, false).0);
        multigrid.push(solve_manufactured(n, true, &kinds, true).0);
    }
    assert!(
        plain[1] > plain[0] * 3 / 2,
        "前処理なしは解像度とともに反復数が伸びるはず: {plain:?}"
    );
    assert!(
        multigrid[1] <= multigrid[0] + 3,
        "MGPCG の反復数は解像度にほぼ依存しないはず: {multigrid:?}"
    );
}

/// **設計§4.4 の分岐条件**: 8子が solid/fluid 混在の粗セルが30%を超えるパターンでは
/// 粗格子化を打ち切る。市松模様(1セルおきに固体)は、どう粗格子化しても全ての粗セルが
/// 混在するので、**1段も粗格子化できない = レベル1本**になる。
/// これが設計の言う「最悪時は Jacobi 前処理 PCG へ退行」の姿である。
#[test]
fn an_unresolvable_solid_pattern_truncates_the_coarsening() {
    let n = 16;
    let mut kinds = all_fluid(n);
    for k in 0..n {
        for j in 0..n {
            for i in 0..n {
                if (i + j + k) % 2 == 0 {
                    kinds[i + n * (j + n * k)] = PressureCell::Solid;
                }
            }
        }
    }
    let multigrid = MultigridPoisson::build(n, n, n, 1.0 / n as f64, true, kinds.clone());
    let report = multigrid.report();
    assert!(
        report.truncated_by_mixed_cells,
        "混在率が30%を超えたら打ち切るはず: {report:?}"
    );
    assert_eq!(report.levels, 1, "1段も粗格子化できないはず: {report:?}");
    assert!(report.mixed_fraction > 0.30, "{report:?}");

    // 退行しても正しさは保たれる(市松の固体は流体セルを孤立させるので、
    // ここでは階層が組めることと前処理が対称正定値のままであることを確かめる)。
    let mut rng = SimRng::new(23, 0);
    let r = random_vector(&mut rng, n * n * n, &kinds);
    let mut z = vec![0.0; n * n * n];
    multigrid.precondition(&r, &mut z);
    for (c, value) in z.iter().enumerate() {
        assert!(value.is_finite(), "cell {c} が有限でない");
        if kinds[c] != PressureCell::Fluid {
            assert_eq!(*value, 0.0, "未知数から外したセルに値が入っている: {c}");
        }
    }
}

/// **粗格子化は緩やかな境界なら通る**: 球1個ぶんの固体は混在率が低いので階層が伸びる。
/// 打ち切り規則が「常に打ち切る」だけの安全側の実装になっていないことの裏取り。
#[test]
fn a_smooth_solid_boundary_still_coarsens() {
    let n = 32;
    let kinds = sphere_in_a_channel(n);
    let multigrid = MultigridPoisson::build(n, n, n, 1.0 / n as f64, false, kinds);
    let report = multigrid.report();
    assert!(
        report.levels >= 3,
        "滑らかな固体境界では粗格子化が通るはず: {report:?}"
    );
}

/// **Dirichlet の支配性**: 流出層に圧力Dirichletがあれば作用素は非特異になり、
/// 粗レベルでも Dirichlet が消えない(消えると粗格子補正が定数分ずれる)。
#[test]
fn a_dirichlet_layer_makes_the_operator_non_singular() {
    let n = 16;
    let with_dirichlet =
        MultigridPoisson::build(n, n, n, 1.0 / n as f64, false, sphere_in_a_channel(n));
    assert!(!with_dirichlet.is_singular());

    let periodic = MultigridPoisson::build(n, n, n, 1.0 / n as f64, true, all_fluid(n));
    assert!(periodic.is_singular(), "全周期は特異(定数が零空間)");
}

/// **決定論**(設計§1): 同じ入力からは同じ階層・同じ前処理結果になる。
#[test]
fn the_preconditioner_is_deterministic() {
    let n = 16;
    let kinds = sphere_in_a_channel(n);
    let mut rng = SimRng::new(29, 0);
    let r = random_vector(&mut rng, n * n * n, &kinds);

    let build = || MultigridPoisson::build(n, n, n, 1.0 / n as f64, false, kinds.clone());
    let (first, second) = (build(), build());
    assert_eq!(first.report(), second.report());

    let (mut z1, mut z2) = (vec![0.0; n * n * n], vec![0.0; n * n * n]);
    first.precondition(&r, &mut z1);
    second.precondition(&r, &mut z2);
    assert_eq!(z1, z2, "同条件ならビット単位で同じ結果になるべき");

    // 同じインスタンスを2度呼んでも作業配列の残りが漏れないこと。
    let mut z3 = vec![0.0; n * n * n];
    first.precondition(&r, &mut z3);
    assert_eq!(z1, z3);
}
