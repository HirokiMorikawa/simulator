//! D40「光の実験室」の水中コースティクス相当: 球レンズ(n=1.5)へ平行ビームを
//! 通し、床に落ちる集光模様を焦点距離を変えて3枚描く。
//!
//! `cargo run --release --example caustic_demo -- <出力ディレクトリ>`

use sim_render::{ball_lens_paraxial_focal_distance, trace_ball_lens_caustic, CausticMap};
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(std::env::args().nth(1).unwrap_or_else(|| ".".to_string()));
    let (radius, ior) = (1.0_f64, 1.5_f64);
    let beam_radius = 0.9 * radius;
    let focal = ball_lens_paraxial_focal_distance(radius, ior);

    // 近軸焦点の手前・ちょうど・奥の3枚。球面収差により、縁の光線は手前で交差する
    // ため「最小錯乱円」は近軸焦点より手前に来る(caustic.rsのテスト参照)。
    for (label, distance) in [
        ("before", focal * 0.80),
        ("paraxial", focal),
        ("after", focal * 1.25),
    ] {
        let mut map = CausticMap::new(256, 256, 0.5 * radius);
        let (deposited, launched) =
            trace_ball_lens_caustic(radius, ior, beam_radius, distance, 900, &mut map);
        // ピークが飽和しない程度に規格化(見た目のための線形スケール)。
        let scale = 0.6 / map.peak_energy().max(1e-12);
        let path = out.join(format!("caustic_{label}.png"));
        map.to_framebuffer(scale)
            .write_png(&path, 1.0)
            .expect("PNGを書き出せる");
        println!(
            "{label}: distance={distance:.4}R deposited={deposited}/{launched} \
             total_energy={:.6} peak={:.3e} -> {}",
            map.total_energy(),
            map.peak_energy(),
            path.display()
        );
    }
}
