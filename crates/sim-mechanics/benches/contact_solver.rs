//! 接触ソルバのベンチマーク(設計docs/00-foundation/05-rust-wasm-platform.md §5
//! 「ホットパス候補: 接触ソルバ・PCG・SPH近傍探索」)。
//!
//! **縮約実装の理由**: `MechanicsSolver::step()`全体(ブロードフェーズ検出+PGS接触解決を
//! 含む)を、積み重なった箱のスタック(現実的な多点接触・warm starting・摩擦を伴う
//! 典型的な負荷)でベンチマークする。`contact::resolve()`単体を直接呼ぶには
//! `ContactManifold`を手動構築する必要があり(通常は`collision::detect()`が内部生成)、
//! 実際のシーンから乖離するため、公開APIの`step()`をエンドツーエンドで計測する方が
//! 現実的な回帰検知になる。PCG(`sim-thermal`/`sim-fluid`)・SPH近傍探索
//! (`sim-fluid::sph`)のベンチマークは同じパターンで後続増分にて追加する。

use criterion::{black_box, criterion_group, criterion_main, BatchSize, Criterion};
use sim_core::{EventQueue, MaterialDb, Solver, SolverContext};
use sim_math::{SimRng, Vec3};
use sim_mechanics::{BodyType, MechanicsSolver, RigidBodyDesc, Shape};

/// `n`段の箱を積み重ねたシーンを構築する(段ごとに0.05mの隙間、初期状態から
/// 接触解決が毎stepフル稼働する典型的な負荷)。
fn build_stack(n: usize) -> (MechanicsSolver, MaterialDb) {
    let materials = MaterialDb::standard();
    let steel = materials
        .find_by_name("鋼(炭素鋼)")
        .expect("standard DB has steel");
    let mut solver = MechanicsSolver::new(9.80665);

    let mut floor = RigidBodyDesc::dynamic(
        Shape::Plane {
            normal: Vec3::new(0.0, 1.0, 0.0),
            d: 0.0,
        },
        steel,
    );
    floor.body_type = BodyType::Static;
    solver.create_body(floor, &materials);

    for i in 0..n {
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(0.5, 0.5, 0.5),
            },
            steel,
        );
        desc.transform.position = Vec3::new(0.0, 0.55 + i as f64 * 1.05, 0.0);
        solver.create_body(desc, &materials);
    }
    (solver, materials)
}

fn bench_contact_solver(c: &mut Criterion) {
    c.bench_function("mechanics_step_stack_of_20_boxes", |b| {
        b.iter_batched(
            || build_stack(20),
            |(mut solver, materials)| {
                let mut rng = SimRng::new(1, 1);
                let mut events = EventQueue::new();
                let mut ctx = SolverContext {
                    materials: &materials,
                    rng: &mut rng,
                    events: &mut events,
                };
                solver.step(black_box(1.0 / 120.0), &mut ctx);
            },
            BatchSize::SmallInput,
        );
    });
}

criterion_group!(benches, bench_contact_solver);
criterion_main!(benches);
