//! docs/15-statistical/ (ミクロマクロ橋渡し・気体分子運動・拡散/ブラウン運動・モンテカルロ)。P4(ランジュバン)・P5(気体分子/イジング)で実装。
//!
//! `brownian`(ランジュバン方程式・BAOAB積分、docs/15-statistical/03-diffusion-brownian.md)・
//! `kinetic_gas`(剛体球気体MD、docs/15-statistical/02-kinetic-gas.md)・
//! `ising`(2Dイジング模型・メトロポリス・Wolffクラスタ法、docs/15-statistical/04-monte-carlo.md)
//! を実装。

mod brownian;
mod ising;
mod kinetic_gas;
pub use brownian::BrownianParticleSet;
pub use ising::IsingSim;
pub use kinetic_gas::{GasSim, BOLTZMANN_CONSTANT};

/// 自由膨張のエントロピー変化 $\Delta S = Nk_B\ln(V_2/V_1)$(設計
/// docs/15-statistical/01-micro-macro-bridge.md §4「状態数と熱力学量の橋渡し」)。
///
/// **§7網羅監査(増分L)で未カバーと判明し追加した**。同文書§7は
/// 「エントロピー: 自由膨張 $\Delta S = Nk_B\ln 2$(2倍体積)を**状態数カウントと
/// 熱力学式の両方で**」を要求しており、その熱力学式側がこの関数、
/// 状態数カウント側が`entropy_change_from_microstate_count`である。
pub fn free_expansion_entropy_change(particle_count: f64, volume_ratio: f64) -> f64 {
    particle_count * BOLTZMANN_CONSTANT * volume_ratio.ln()
}

/// 同じ量を**状態数の数え上げ**から出す(ボルツマンの関係 $S=k_B\ln W$)。
///
/// 理想気体を「各粒子が独立に体積$V$のどこかに居る」と数えると、
/// 状態数は $W \propto V^N$ なので $\Delta S = k_B\ln(V_2^N/V_1^N)
/// = Nk_B\ln(V_2/V_1)$。**運動量部分は自由膨張で不変**(温度が変わらない)
/// なので比を取ると完全に相殺し、位置の数え上げだけが残る——これが
/// 「状態数カウントと熱力学式が一致する」ことの中身である。
///
/// 引数は`ln`を取る前の比を対数で受けることで大きな$N$でも桁溢れしない形にした
/// ($V_2^N$を直に計算すると$N$が数十で倍精度が溢れる)。
pub fn entropy_change_from_microstate_count(particle_count: f64, volume_ratio: f64) -> f64 {
    // ln W_2 - ln W_1 = N*ln V_2 - N*ln V_1 = N*ln(V_2/V_1)
    BOLTZMANN_CONSTANT * (particle_count * volume_ratio.ln())
}

#[cfg(test)]
mod entropy_tests {
    use super::*;

    /// **設計 §7「自由膨張 $\Delta S = Nk_B\ln 2$ を状態数カウントと熱力学式の
    /// 両方で」**。2つの経路が厳密に一致すること、および2倍体積で
    /// $Nk_B\ln 2$ になることを確認する。
    #[test]
    fn free_expansion_entropy_matches_between_microstate_counting_and_thermodynamics() {
        let n = 1000.0;
        let thermodynamic = free_expansion_entropy_change(n, 2.0);
        let counted = entropy_change_from_microstate_count(n, 2.0);
        assert!(
            (thermodynamic - counted).abs() <= f64::EPSILON * thermodynamic.abs().max(1.0),
            "状態数カウントと熱力学式は厳密に一致すべき: {thermodynamic} vs {counted}"
        );
        let expected = n * BOLTZMANN_CONSTANT * std::f64::consts::LN_2;
        assert!(
            (thermodynamic - expected).abs() / expected < 1e-15,
            "2倍体積の自由膨張は N k_B ln2: {thermodynamic} vs {expected}"
        );

        // 圧縮(比<1)ならエントロピーは減る。等体積なら変化なし。
        assert!(free_expansion_entropy_change(n, 0.5) < 0.0);
        assert_eq!(free_expansion_entropy_change(n, 1.0), 0.0);
    }
}
