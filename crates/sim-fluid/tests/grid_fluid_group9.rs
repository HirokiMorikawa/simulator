//! 群9で `GridFluid2D` へ追加した2機能の受け入れテスト:
//! **渦度強化**(設計 docs/11-fluid/02-eulerian-grid.md §4.5)と
//! **任意形状の固体境界**(§3 `cell_type` / §4.4 Solid面のNeumann / §6 圧力の面積分)。
//!
//! いずれも「入れた/入れない」の**対照実験**で、機能が実際に効いていることを示す。

use sim_core::Solver;
use sim_fluid::{CellType, GridBoundary, GridFluid2D};

/// Taylor-Green 渦(F8 と同じ初期条件)を張る。
fn taylor_green(nx: usize, ny: usize, h: f64) -> GridFluid2D {
    let length = h * nx as f64;
    let k = 2.0 * std::f64::consts::PI / length;
    let mut fluid = GridFluid2D::new(nx, ny, h);
    for j in 0..ny {
        for i in 0..nx {
            let idx = i + nx * j;
            let x = i as f64 * h;
            let y = (j as f64 + 0.5) * h;
            fluid.u[idx] = -(k * x).cos() * (k * y).sin();
            let xv = (i as f64 + 0.5) * h;
            let yv = j as f64 * h;
            fluid.v[idx] = (k * xv).sin() * (k * yv).cos();
        }
    }
    fluid
}

/// 全セルの $\omega_z^2$ の総和(エンストロフィー、渦の「濃さ」の指標)。
fn enstrophy(fluid: &GridFluid2D) -> f64 {
    let h = fluid.h;
    let mut sum = 0.0;
    for j in 0..fluid.ny as i64 {
        for i in 0..fluid.nx as i64 {
            let dvdx = (fluid.v_at(i + 1, j) - fluid.v_at(i - 1, j)) / (2.0 * h);
            let dudy = (fluid.u_at(i, j + 1) - fluid.u_at(i, j - 1)) / (2.0 * h);
            let omega = dvdx - dudy;
            sum += omega * omega;
        }
    }
    sum
}

/// 流体セルの発散の最大値。`skip_boundary_columns` を立てると x 端の2列
/// (`Channel` の流入列と流出列)を除外する——**この2列は設計上そもそも
/// 発散ゼロにならない**: 流入列は速度Dirichlet、流出列は圧力Dirichlet の恒等行で
/// 未知数から外してあるため、投影は両列の発散を消さない(群7からの既存挙動)。
fn max_abs_divergence(fluid: &GridFluid2D, skip_boundary_columns: bool) -> f64 {
    let mut worst: f64 = 0.0;
    let (lo, hi) = if skip_boundary_columns {
        (1, fluid.nx as i64 - 1)
    } else {
        (0, fluid.nx as i64)
    };
    for j in 0..fluid.ny as i64 {
        for i in lo..hi {
            if fluid.cell_type_at(i, j) == CellType::Solid {
                continue; // 固体セルは未知数から外してあるので発散ゼロではない
            }
            worst = worst.max(fluid.divergence(i, j).abs());
        }
    }
    worst
}

/// **既定が検証モードであること**: `vorticity_confinement_epsilon` の既定は 0.0 で、
/// このとき渦度強化の段は結果を**ビット単位で変えない**(既存の F8/F9 に対する回帰の
/// 完全な遮断)。設計§4.5「検証モードでは無効化する」。
#[test]
fn vorticity_confinement_is_off_by_default_and_bit_identical() {
    let mut baseline = taylor_green(32, 32, 1.0 / 32.0);
    baseline.kinematic_viscosity = 0.2;
    assert_eq!(
        baseline.vorticity_confinement_epsilon, 0.0,
        "the default must be the verification mode (design §4.5)"
    );

    let mut with_explicit_zero = baseline.clone();
    with_explicit_zero.vorticity_confinement_epsilon = 0.0;

    for _ in 0..40 {
        baseline.step(0.0005);
        with_explicit_zero.step(0.0005);
    }
    assert_eq!(
        baseline.u, with_explicit_zero.u,
        "epsilon=0 must not perturb the velocity field at all"
    );
    assert_eq!(baseline.v, with_explicit_zero.v);
}

/// **対照実験**: 渦度強化を入れると、数値拡散で失われる渦度が実際に補償される
/// (エンストロフィーが多く残る)。設計§4.5 の目的そのもの。
/// あわせて、投影より前に加えているので**非圧縮性は壊れない**ことを確認する。
#[test]
fn vorticity_confinement_preserves_more_enstrophy_without_breaking_incompressibility() {
    let dt = 0.0005;
    let steps = 200;

    let run = |epsilon: f64| -> (f64, f64) {
        let mut fluid = taylor_green(32, 32, 1.0 / 32.0);
        fluid.kinematic_viscosity = 0.05;
        fluid.vorticity_confinement_epsilon = epsilon;
        for _ in 0..steps {
            fluid.step(dt);
        }
        (enstrophy(&fluid), max_abs_divergence(&fluid, false))
    };

    let (without, div_without) = run(0.0);
    let (with, div_with) = run(2.0);

    assert!(
        with > without,
        "vorticity confinement must retain more enstrophy: without={without} with={with}"
    );
    // **非圧縮性**: 渦度強化は投影より前に加える外力なので、投影が残す発散を
    // 悪化させない。ここは絶対値ではなく **epsilon=0 の同一シーンとの比較**で見る——
    // 200ステップ後の残差発散は epsilon に関係なく 1e-4 台で、これは PCG の
    // 相対許容誤差(1e-8)と右辺のノルムから決まる**この格子流体ソルバの既存の性質**
    // (旧来の advect→diffuse→project 手順でも同じ値になることを実測で確認した)であり、
    // 渦度強化とは無関係だからである。F9(単発投影で <1e-6)とは測っているものが違う。
    assert!(
        div_with < 3.0 * div_without.max(1e-12),
        "adding a force before the projection must not degrade incompressibility: \
         div_without={div_without:e} div_with={div_with:e}"
    );
}

/// バッジは `epsilon > 0` のときだけ出る(設計§4.5「非物理的な補償項であることを
/// UIの近似表示で明示し、検証モードでは無効化する」)。
#[test]
fn the_confinement_badge_appears_only_when_it_is_actually_on() {
    let mut fluid = GridFluid2D::new(8, 8, 0.1);
    let names = |f: &GridFluid2D| -> Vec<&'static str> {
        f.approximations().iter().map(|a| a.name).collect()
    };
    assert!(!names(&fluid).contains(&"渦度強化(非物理)"));
    fluid.vorticity_confinement_epsilon = 0.5;
    assert!(names(&fluid).contains(&"渦度強化(非物理)"));
    assert!(
        !names(&fluid).contains(&"セル単位の固体境界"),
        "no solid cells were set, so that badge must stay off"
    );
}

/// 円柱(矩形ではない形)を `set_solid_cells` で埋め込めること、そこが `Solid` になり
/// **流体が中へ入らない**こと。移行前は `GridSolidBox` の単一矩形しか置けなかった。
#[test]
fn an_arbitrary_shaped_solid_can_be_embedded_and_blocks_the_flow() {
    let (nx, ny) = (48, 24);
    let h = 1.0 / ny as f64;
    let mut fluid =
        GridFluid2D::new(nx, ny, h).with_boundary(GridBoundary::Channel { inflow_speed: 1.0 });
    fluid.kinematic_viscosity = 1.0e-3;

    let center = (0.6, 0.5);
    let radius = 0.12;
    fluid.set_solid_cells(|x, y| {
        let dx = x - center.0;
        let dy = y - center.1;
        if dx * dx + dy * dy < radius * radius {
            Some(sim_math::Vec3::ZERO)
        } else {
            None
        }
    });

    // 円柱の内部・外部が正しくラスタライズされていること(矩形ではない)。
    let cell_of = |x: f64, y: f64| -> CellType {
        let i = (x / h) as i64;
        let j = (y / h) as i64;
        fluid.cell_type_at(i, j)
    };
    assert_eq!(cell_of(center.0, center.1), CellType::Solid, "円柱の中心");
    assert_eq!(
        cell_of(center.0 + radius * 0.8, center.1 + radius * 0.8),
        CellType::Fluid,
        "円柱に外接する正方形の角(矩形マスクなら Solid になってしまう位置)"
    );

    for _ in 0..300 {
        fluid.step(0.002);
    }

    // 固体セル内部の速度は固体速度(=0)のまま。
    let i_c = (center.0 / h) as i64;
    let j_c = (center.1 / h) as i64;
    assert!(
        fluid.u_at(i_c, j_c).abs() < 1e-9 && fluid.v_at(i_c, j_c).abs() < 1e-9,
        "fluid must not leak into the solid: u={} v={}",
        fluid.u_at(i_c, j_c),
        fluid.v_at(i_c, j_c)
    );
    // 固体を埋め込んでも**流体セルの内部は非圧縮のまま**であること
    // (= Solid面をPoissonのNeumannとして正しく組めていることの直接の確認)。
    // 流入/流出列は設計上そもそも発散ゼロにならないので除外する(ヘルパのdoc参照)。
    let interior_divergence = max_abs_divergence(&fluid, true);
    assert!(
        interior_divergence < 1e-6,
        "interior divergence with an embedded solid={interior_divergence:e}"
    );
}

/// **形状依存性(対照実験)**: 前面投影高さが同じ**円柱と角柱**を同じ流路に置くと、
/// 圧力積分から出る抗力は**円柱の方が小さい**(流線が滑らかに回り込むため)。
///
/// これが「任意形状に対応した」ことの実質的な確認になる——移行前のマスキング方式では
/// そもそも円柱を置けず、置けたとしても投影の後に速度を上書きするだけなので、
/// 固体表面の圧力分布が形状を反映しなかった。
#[test]
fn a_cylinder_has_lower_pressure_drag_than_a_square_of_the_same_frontal_height() {
    let (nx, ny) = (64, 32);
    let h = 1.0 / ny as f64;
    let half_height = 0.12;
    let center = (0.5, 0.5);

    let drag_of = |shape: &dyn Fn(f64, f64) -> bool| -> f64 {
        let mut fluid =
            GridFluid2D::new(nx, ny, h).with_boundary(GridBoundary::Channel { inflow_speed: 1.0 });
        fluid.kinematic_viscosity = 1.0e-3;
        fluid.set_solid_cells(|x, y| {
            if shape(x, y) {
                Some(sim_math::Vec3::ZERO)
            } else {
                None
            }
        });
        for _ in 0..400 {
            fluid.step(0.002);
        }
        fluid
            .pressure_force_on_solid()
            .expect("a solid was embedded")
            .x
    };

    let cylinder = |x: f64, y: f64| {
        let dx = x - center.0;
        let dy = y - center.1;
        dx * dx + dy * dy < half_height * half_height
    };
    let square =
        |x: f64, y: f64| (x - center.0).abs() < half_height && (y - center.1).abs() < half_height;

    let drag_cylinder = drag_of(&cylinder);
    let drag_square = drag_of(&square);

    assert!(
        drag_square > 0.0 && drag_cylinder > 0.0,
        "both bodies must feel a downstream push: cylinder={drag_cylinder} square={drag_square}"
    );
    assert!(
        drag_cylinder < drag_square,
        "a cylinder must have less pressure drag than a square of the same frontal height: \
         cylinder={drag_cylinder} square={drag_square}"
    );
}
