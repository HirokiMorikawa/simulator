//! docs/14-quantum/ (役割と限界・シュレディンガーソルバ・有効モデル)。P5で実装。
//!
//! `schrodinger`(1D TDSEのsplit-step Fourier解法、docs/14-quantum/02-schrodinger-solver.md)と
//! `schrodinger2d`(2D版、同docs §4/§8「2D(二重スリットの本命)」)を実装。
//! 吸収境界・検出スクリーンの決定論的サンプリング・有効モデルの型・トレイトのスケルトンは
//! 今後の増分で追加する(docs/22-roadmap/01-phases.md)。

mod schrodinger;
mod schrodinger2d;
pub use schrodinger::{find_eigenstates, WaveFunction1D};
pub use schrodinger2d::WaveFunction2D;

/// 有効モデルの引用値(設計 docs/14-quantum/03-effective-models.md §7
/// 「各式の引用値再現(バルマー線波長、$V_T$ = 25.85 mV @300K、
/// デュロン=プティとの比較)」)。
///
/// **§7網羅監査(増分L)で未カバーと判明し追加した**。設計§3が挙げる
/// 「量子論から出る巨視的な引用値」を閉形式で再現できることを示す節で、
/// **完全な量子計算ではなく既知の有効モデル(縮約)をそのまま実装する**
/// のが趣旨——だからこそ引用値との一致がそのまま検証になる。
pub mod effective_models {
    /// リュードベリ定数 [m^-1](水素、換算質量込みの実測値)。
    pub const RYDBERG_HYDROGEN_PER_METER: f64 = 1.0967758e7;
    /// ボルツマン定数 [J/K]。
    pub const BOLTZMANN_CONSTANT: f64 = 1.380649e-23;
    /// 電気素量 [C]。
    pub const ELEMENTARY_CHARGE: f64 = 1.602176634e-19;
    /// 気体定数 [J/(mol·K)]。
    pub const GAS_CONSTANT: f64 = 8.314462618;

    /// 水素のスペクトル線波長 [m](リュードベリの式)。
    /// $1/\lambda = R_H\left(\frac{1}{n_1^2}-\frac{1}{n_2^2}\right)$、$n_2>n_1$。
    /// バルマー系列は $n_1=2$。
    pub fn hydrogen_line_wavelength(n_lower: u32, n_upper: u32) -> f64 {
        let (a, b) = (n_lower as f64, n_upper as f64);
        1.0 / (RYDBERG_HYDROGEN_PER_METER * (1.0 / (a * a) - 1.0 / (b * b)))
    }

    /// 熱電圧 $V_T = k_B T/q$ [V]。ダイオードのShockley式が使う量で、
    /// `sim-em`の回路が `n_vt` として受け取る値の素になる。
    pub fn thermal_voltage(temperature_kelvin: f64) -> f64 {
        BOLTZMANN_CONSTANT * temperature_kelvin / ELEMENTARY_CHARGE
    }

    /// デュロン=プティの法則によるモル比熱 $3R$ [J/(mol·K)]。
    /// 高温極限で全ての単原子固体が取る古典値。
    pub fn dulong_petit_molar_heat_capacity() -> f64 {
        3.0 * GAS_CONSTANT
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        /// **バルマー線の波長が引用値と一致する**。Hα(n=3→2)656.3 nm、
        /// Hβ(4→2)486.1 nm、Hγ(5→2)434.0 nm(可視域の代表3本)。
        #[test]
        fn balmer_series_wavelengths_match_the_quoted_values() {
            for (upper, quoted_nm) in [(3u32, 656.3), (4, 486.1), (5, 434.0)] {
                let nm = hydrogen_line_wavelength(2, upper) * 1e9;
                assert!(
                    (nm - quoted_nm).abs() / quoted_nm < 1e-3,
                    "バルマー線 n={upper}→2 は {quoted_nm} nm のはず: {nm} nm"
                );
            }
            // ライマン系列(n_1=1)は紫外、パッシェン系列(n_1=3)は赤外。
            assert!(
                hydrogen_line_wavelength(1, 2) * 1e9 < 400.0,
                "ライマンは紫外"
            );
            assert!(
                hydrogen_line_wavelength(3, 4) * 1e9 > 700.0,
                "パッシェンは赤外"
            );
        }

        /// **$V_T$ = 25.85 mV @ 300 K**。`sim-em`の回路テスト・D19シーンが
        /// `n_vt: 0.02585` としてこの値を使っているが、**その値がどこから来るのかを
        /// 検証するテストは無かった**(定数がハードコードされているだけだった)。
        #[test]
        fn thermal_voltage_at_300_kelvin_matches_the_quoted_25_85_millivolts() {
            let v_t = thermal_voltage(300.0);
            assert!(
                (v_t - 0.02585).abs() < 5e-6,
                "V_T @300K は 25.85 mV のはず: {} mV",
                v_t * 1e3
            );
            // 温度に比例する(定義そのもの)。
            assert!((thermal_voltage(600.0) / v_t - 2.0).abs() < 1e-12);
        }

        /// **デュロン=プティ $3R$ ≈ 24.94 J/(mol·K)**。銅の実測モル比熱
        /// (24.44)と数%以内で一致することを確認する——高温極限の古典値として
        /// 妥当であることの引用値比較。
        #[test]
        fn dulong_petit_matches_the_classical_limit_and_copper() {
            let c = dulong_petit_molar_heat_capacity();
            assert!((c - 24.94).abs() < 0.01, "3R は 24.94 J/(mol·K): {c}");
            let copper_measured = 24.44;
            assert!(
                (c - copper_measured).abs() / copper_measured < 0.03,
                "銅の実測モル比熱と3%以内で一致するはず: 3R={c} 実測={copper_measured}"
            );
        }
    }
}
