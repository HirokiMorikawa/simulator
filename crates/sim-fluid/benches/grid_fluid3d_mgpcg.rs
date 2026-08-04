//! 3D格子流体の1ステップ(マルチグリッド前処理PCG込み)のベンチマーク(**群10で追加**)。
//! 設計 docs/11-fluid/02-eulerian-grid.md §10、docs/00-foundation/05-rust-wasm-platform.md §5。
//!
//! **なぜ 2D の `grid_fluid_pcg` と別ターゲットにしたか**: 回帰ゲート
//! (`scripts/check_bench_regression.py`)はベースラインと**ベンチターゲット単位**で
//! 積集合を取り、ベースラインに無いターゲットを比較対象外にする。既存ターゲットへ
//! 関数を足すと、ベースライン側に保存が無いまま `--baseline base`(criterion の
//! 厳密モード)で比較しにいって落ちる。別ターゲットにすれば、導入回だけ素通りして
//! 次回以降から基準が効く——スクリプトの doc がまさにその運用を想定している。
//!
//! **縮約実装の理由**: 解像度は 32³。設計の予算は 64³ だが、64³ は1ステップ 0.2 秒で
//! criterion のサンプリングに載せると数十秒かかり CI に置けない。守りたいのは
//! 「MGPCG の反復数が解像度に依存しない」という性質が壊れていないことで、それは
//! 32³ でも十分に検出できる(前処理が壊れれば反復数が跳ね上がり数倍遅くなる)。
//! 解像度スケーリングそのものは `examples/grid_fluid3d_bench.rs` が 32³/64³/128³ で測る。
//!
//! 初期条件に Taylor-Green 渦を使う理由は 2D 版と同じ——全域ゼロの速度場では発散が
//! どこでも 0 になり、圧力投影が実質1反復で終わって典型的な負荷を代表しないため。

use criterion::{black_box, criterion_group, criterion_main, BatchSize, Criterion};
use sim_fluid::GridFluid3D;

fn build_taylor_green_3d(n: usize) -> GridFluid3D {
    let h = 1.0 / n as f64;
    let k = 2.0 * std::f64::consts::PI;
    let mut fluid = GridFluid3D::new(n, n, n, h);
    for kk in 0..n {
        for j in 0..n {
            for i in 0..n {
                let idx = i + n * (j + n * kk);
                fluid.u[idx] = -(k * (i as f64 * h)).cos() * (k * ((j as f64 + 0.5) * h)).sin();
                fluid.v[idx] = (k * ((i as f64 + 0.5) * h)).sin() * (k * (j as f64 * h)).cos();
            }
        }
    }
    fluid
}

fn bench_grid_fluid3d_mgpcg(c: &mut Criterion) {
    c.bench_function("grid_fluid_3d_step_32x32x32_taylor_green", |b| {
        b.iter_batched(
            || build_taylor_green_3d(32),
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

criterion_group!(benches, bench_grid_fluid3d_mgpcg);
criterion_main!(benches);
