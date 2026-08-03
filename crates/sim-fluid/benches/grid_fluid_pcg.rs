//! 格子流体の圧力投影(PCG)ベンチマーク(設計docs/00-foundation/05-rust-wasm-platform.md
//! §5「ホットパス候補: 接触ソルバ・PCG・SPH近傍探索」、`sim-mechanics::contact_solver`
//! ベンチのdoc参照)。
//!
//! **縮約実装の理由**: `sim_math::pcg`単体ではなく`GridFluid2D`の1step分のパイプライン
//! (移流→拡散→圧力投影)をエンドツーエンドで計測する(`contact_solver`ベンチと同じ
//! 「公開APIをエンドツーエンドで計測する方が実際のシーンから乖離しない」という方針)。
//! Taylor-Green渦(F8の単体テストと同じ非自明な初期速度場、既存テストの経緯参照)を
//! 初期条件に使う — 全域ゼロの速度場では発散がどこでも0になりPCGが実質1反復で収束して
//! しまい、圧力投影の典型的な負荷を代表しないため。

use criterion::{black_box, criterion_group, criterion_main, BatchSize, Criterion};
use sim_fluid::GridFluid2D;

fn build_taylor_green(nx: usize, ny: usize) -> GridFluid2D {
    let length = 1.0;
    let h = length / nx as f64;
    let k = 2.0 * std::f64::consts::PI / length;
    let mut fluid = GridFluid2D::new(nx, ny, h);

    for j in 0..ny as i64 {
        for i in 0..=nx as i64 {
            let idx = (i.rem_euclid(nx as i64)) as usize + nx * (j.rem_euclid(ny as i64)) as usize;
            let x = i as f64 * h;
            let y = (j as f64 + 0.5) * h;
            fluid.u[idx] = -(k * x).cos() * (k * y).sin();
        }
    }
    for j in 0..=ny as i64 {
        for i in 0..nx as i64 {
            let idx = (i.rem_euclid(nx as i64)) as usize + nx * (j.rem_euclid(ny as i64)) as usize;
            let x = (i as f64 + 0.5) * h;
            let y = j as f64 * h;
            fluid.v[idx] = (k * x).sin() * (k * y).cos();
        }
    }
    fluid
}

fn bench_grid_fluid_pcg(c: &mut Criterion) {
    c.bench_function("grid_fluid_2d_step_64x64_taylor_green", |b| {
        b.iter_batched(
            || build_taylor_green(64, 64),
            |mut fluid| {
                let dt = 0.0005;
                fluid.advect_velocity(black_box(dt));
                fluid.diffuse_explicit(dt, 0.2);
                fluid.project(dt, 1.0);
            },
            BatchSize::SmallInput,
        );
    });
}

criterion_group!(benches, bench_grid_fluid_pcg);
criterion_main!(benches);
