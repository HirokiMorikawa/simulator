//! `GridFluid3D` の1ステップ実測(**群9で追加**)。設計 docs/11-fluid/02-eulerian-grid.md
//! §10 は「64³ で予算 4 ms」と定めているが、**現状の前処理なしPCGでは全く届かない**。
//! その事実を数字で残すためのベンチである。
//!
//! `examples/` に置いてあるので `cargo test` では走らない(ビルドはされる)——
//! 64³ は1ステップで秒オーダーかかり、テスト時間に載せられないため
//! (`sim-render` のデモ画像生成と同じ方針)。
//!
//! ```text
//! cargo run --release -p sim-fluid --example grid_fluid3d_bench
//! ```
//!
//! 実測(2026-08-03、release ビルド):
//!
//! | 解像度 | 1ステップ | 設計§10の予算 |
//! |---|---|---|
//! | 32³ | 73.3 ms | — |
//! | 64³ | **795.7 ms** | **4 ms**(約200倍の超過) |
//!
//! 設計§4.4 は「マルチグリッド前処理は性能ベンチ(64³ 4ms 予算)が要求したときに
//! 導入する(機能でなく性能の最適化)」と実装順序を定めている。**この実測が、
//! まさにその要求が示された記録**である。前処理なしPCGの反復数は解像度とともに
//! 増えるため、上げるほど差は開く(32³→64³ で体積8倍に対し時間は10.9倍)。

use sim_fluid::GridFluid3D;
use std::time::Instant;

fn main() {
    for n in [32usize, 64] {
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

        fluid.step(0.001); // ウォームアップ(初回はページフォルトが乗る)
        let started = Instant::now();
        let steps = 3;
        for _ in 0..steps {
            fluid.step(0.001);
        }
        let ms = started.elapsed().as_secs_f64() * 1000.0 / steps as f64;
        println!("{n}^3: {ms:.1} ms/step");
    }
}
