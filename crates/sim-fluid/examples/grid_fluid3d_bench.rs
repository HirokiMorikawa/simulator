//! `GridFluid3D` の1ステップ実測(**群9で追加、群10で更新**)。
//! 設計 docs/11-fluid/02-eulerian-grid.md §10 は「64³ で予算 4 ms」と定めている。
//!
//! `examples/` に置いてあるので `cargo test` では走らない(ビルドはされる)——
//! 128³ は1ステップで秒オーダーかかり、テスト時間に載せられないため
//! (`sim-render` のデモ画像生成と同じ方針)。
//!
//! ```text
//! cargo run --release -p sim-fluid --example grid_fluid3d_bench
//! ```
//!
//! # 群9の実測(前処理なしPCG、2026-08-03、release ビルド)
//!
//! | 解像度 | 1ステップ | 設計§10の予算 |
//! |---|---|---|
//! | 32³ | 73.3 ms | — |
//! | 64³ | **795.7 ms** | **4 ms**(約200倍の超過) |
//!
//! この実測が、設計§4.4 の言う「マルチグリッド前処理は**性能ベンチが要求したときに**
//! 導入する」の「要求」そのものだった。
//!
//! # 群10の実測(MGPCG + 移流のホットパス整理、2026-08-04、同一マシン)
//!
//! | 解像度 | 1ステップ | 圧力投影 | 投影の反復数 |
//! |---|---|---|---|
//! | 32³ | 24.9 ms | 11.4 ms | 7 |
//! | 64³ | **206.8 ms** | 91.1 ms | 7 |
//! | 128³ | 1840 ms | 935 ms | 8 |
//!
//! 同一マシンで測り直した群9の基準値は 32³ 68.9 ms / 64³ 716.8 ms なので、
//! **64³ で約 3.5 倍**の短縮である。中身は2つ:
//!
//! - **圧力投影 344 → 91 ms**。効いたのは速さより**反復数が解像度に依存しなくなった**
//!   ことで、32³ も 64³ も 128³ も 7〜8 反復で相対残差 10⁻⁸ に入る。前処理なしPCGは
//!   反復数が $O(N)$ で伸びるため、解像度を上げるほど差が開く: 体積8倍に対して、
//!   群9は 32³→64³ で 10.4 倍(超線形)、群10は 8.3 倍・64³→128³ で 8.9 倍
//!   (ほぼ線形)。
//! - **移流 352 → 110 ms**。周期ラップ(整数除算)と `clamp` を三線形補間の8点ごとに
//!   踏んでいたのを、軸ごとに1度だけ解くようにした。物理は1ビットも変えていない。
//!
//! **予算 4 ms にはまだ 50 倍届かない**(群9の約200倍からは縮んだ)。
//! 残っているのは設計§10 が並べた手段のうち
//! 未着手のもの——SIMD(7点ステンシルと補間)、rayon による格子タイル並列、
//! WebGPU——で、いずれもアルゴリズムではなく実行基盤の話である。
//! 反復数が解像度に依存しなくなった以上、**ここから先は素直に台数効果で縮む**。

use sim_fluid::GridFluid3D;
use std::time::Instant;

fn main() {
    let sizes: Vec<usize> = match std::env::args().nth(1) {
        Some(arg) => arg
            .split(',')
            .map(|s| s.trim().parse().expect("解像度は整数で指定する"))
            .collect(),
        None => vec![32, 64, 128],
    };

    for n in sizes {
        let h = 1.0 / n as f64;
        let k = 2.0 * std::f64::consts::PI;
        let mut fluid = GridFluid3D::new(n, n, n, h);
        fluid.kinematic_viscosity = 0.01;
        for kk in 0..n {
            for j in 0..n {
                for i in 0..n {
                    let idx = i + n * (j + n * kk);
                    fluid.u[idx] = -(k * (i as f64 * h)).cos() * (k * ((j as f64 + 0.5) * h)).sin();
                    fluid.v[idx] = (k * ((i as f64 + 0.5) * h)).sin() * (k * (j as f64 * h)).cos();
                }
            }
        }

        fluid.step(0.001); // ウォームアップ(初回はページフォルトと階層構築が乗る)
        let started = Instant::now();
        let steps = 3;
        let mut projection = 0.0;
        for _ in 0..steps {
            fluid.advect_smoke(0.001);
            fluid.advect_velocity(0.001);
            fluid.diffuse_explicit(0.001, fluid.kinematic_viscosity);
            let at_projection = Instant::now();
            fluid.last_pressure = fluid.project(0.001, fluid.density);
            projection += at_projection.elapsed().as_secs_f64();
        }
        let ms = started.elapsed().as_secs_f64() * 1000.0 / steps as f64;
        let projection_ms = projection * 1000.0 / steps as f64;
        let report = fluid.last_pressure_solve();
        println!(
            "{n}^3: {ms:8.1} ms/step  (投影 {projection_ms:7.1} ms, \
             {} 反復, 残差 {:.1e}, マルチグリッド {} 段{})",
            report.iterations,
            report.residual_norm,
            report.multigrid_levels,
            if report.coarsening_truncated {
                "・打ち切りあり"
            } else {
                ""
            },
        );
    }
}
