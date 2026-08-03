//! SPH近傍探索ベンチマーク(設計docs/00-foundation/05-rust-wasm-platform.md §5
//! 「ホットパス候補: 接触ソルバ・PCG・SPH近傍探索」、`sim-mechanics::contact_solver`
//! ベンチのdoc参照)。
//!
//! **縮約実装の理由**: `SpatialHash`単体ではなく`SphFluid::step()`をエンドツーエンドで
//! 計測する(`compute_density_and_pressure`内の`hash.rebuild`+`hash.query`が支配的な
//! コストになる、密な立方体状の粒子塊 — `sph.rs`の運動量保存テストと同じ配置 — で
//! 典型的な近傍数を再現する)。

use criterion::{black_box, criterion_group, criterion_main, BatchSize, Criterion};
use sim_fluid::SphFluid;
use sim_math::Vec3;

fn build_particle_cube(n: usize) -> SphFluid {
    let h = 0.04;
    let dx = h / 2.0;
    let rho0 = 1000.0;
    let c_s = 20.0;
    let mut fluid = SphFluid::new(h, rho0, c_s);
    fluid.mass = rho0 * dx.powi(3);

    for ix in 0..n {
        for iy in 0..n {
            for iz in 0..n {
                let pos = Vec3::new(ix as f64 * dx, iy as f64 * dx, iz as f64 * dx);
                fluid.add_particle(pos, Vec3::new(0.1, -0.05, 0.02));
            }
        }
    }
    fluid
}

fn bench_sph_neighbor_search(c: &mut Criterion) {
    c.bench_function("sph_fluid_step_cube_of_1728_particles", |b| {
        b.iter_batched(
            || build_particle_cube(12), // 12^3 = 1728粒子
            |mut fluid| {
                let dt = 0.25 * fluid.h / fluid.c_s;
                fluid.step(black_box(dt), 0.0);
            },
            BatchSize::SmallInput,
        );
    });
}

criterion_group!(benches, bench_sph_neighbor_search);
criterion_main!(benches);
