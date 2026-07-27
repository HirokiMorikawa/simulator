//! トーンマッピング(HDR輝度→表示可能範囲[0,1]への圧縮)。設計:
//! docs/17-rendering/03-materials-camera.md、docs/21-verification/
//! 03-demo-scenarios.md D43「カメラ」。
//!
//! Reinhard演算子($L_{out} = L_{in}/(1+L_{in})$)を実装する。色を扱う場合は
//! 輝度(Rec.709相対輝度)のみを圧縮しチャンネルへ均等にスケールを掛け戻すことで
//! 色相を保つ(ハイライトで色が白飛びして色相が失われるのを防ぐ標準的な手法)。
//!
//! **縮約実装の理由**: `sim-render`はまだ実際の画像出力パイプライン(フレーム
//! バッファ)を持たない検証用レイトレーサ(R1–R7は単一レイ/解析値比較)のため、
//! 本モジュールも単一の輝度値・単一の色に対する純粋関数として実装する。
//! 露出調整・シャッター速度・モーションブラー・ACES等のより高度な演算子は未実装。

use sim_math::Vec3;

/// 輝度ベースのReinhardトーンマッピング: $L_{out} = L_{in}/(1+L_{in})$。
/// `luminance`は非負を仮定する(放射輝度は物理的に非負)。
pub fn reinhard_tonemap(luminance: f64) -> f64 {
    luminance / (1.0 + luminance)
}

/// Rec.709相対輝度(標準的なRGB→輝度変換係数、$0.2126R+0.7152G+0.0722B$)。
pub fn relative_luminance(color: Vec3) -> f64 {
    0.2126 * color.x + 0.7152 * color.y + 0.0722 * color.z
}

/// 色相を保つReinhardトーンマッピング(モジュールdoc参照)。輝度のみを
/// `reinhard_tonemap`で圧縮し、その比率を全チャンネルへ均等に掛け戻す。
pub fn reinhard_tonemap_color(radiance: Vec3) -> Vec3 {
    let luminance = relative_luminance(radiance);
    if luminance <= 0.0 {
        return Vec3::ZERO;
    }
    let scale = reinhard_tonemap(luminance) / luminance;
    radiance.scale(scale)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 式そのものの厳密評価(abs<1e-15)。
    #[test]
    fn reinhard_tonemap_matches_closed_form_at_several_luminance_values() {
        for &l in &[0.0, 0.5, 1.0, 2.0, 10.0, 1000.0] {
            let expected = l / (1.0 + l);
            assert!((reinhard_tonemap(l) - expected).abs() < 1e-15);
        }
    }

    /// 単調増加(輝度が高いほど出力も高い、圧縮しても順序が保たれる)。
    #[test]
    fn reinhard_tonemap_is_monotonically_increasing() {
        let samples = [0.0, 0.1, 0.5, 1.0, 2.0, 5.0, 50.0, 500.0, 5000.0];
        for pair in samples.windows(2) {
            assert!(
                reinhard_tonemap(pair[1]) > reinhard_tonemap(pair[0]),
                "tonemap should be strictly increasing: f({})={} f({})={}",
                pair[0],
                reinhard_tonemap(pair[0]),
                pair[1],
                reinhard_tonemap(pair[1])
            );
        }
    }

    /// 非常に高い輝度では出力が1に漸近する(表示可能範囲の上限)。
    #[test]
    fn reinhard_tonemap_approaches_one_for_very_high_luminance() {
        let output = reinhard_tonemap(1.0e9);
        assert!(
            (output - 1.0).abs() < 1e-6,
            "output should approach 1.0 for extreme luminance: {output}"
        );
    }

    /// 輝度0では出力0(黒は黒のまま)。
    #[test]
    fn reinhard_tonemap_at_zero_luminance_is_zero() {
        assert_eq!(reinhard_tonemap(0.0), 0.0);
    }

    /// 色付きの高輝度放射輝度に対し、トーンマッピング後もチャンネル比(色相)が
    /// 保たれること(輝度だけが`reinhard_tonemap`の式どおり圧縮される)を確認する。
    #[test]
    fn reinhard_tonemap_color_preserves_hue_ratio_while_compressing_luminance() {
        let radiance = Vec3::new(4.0, 2.0, 1.0); // 明るいオレンジ寄りの色
        let tonemapped = reinhard_tonemap_color(radiance);

        let luminance_in = relative_luminance(radiance);
        let luminance_out = relative_luminance(tonemapped);
        let expected_luminance_out = reinhard_tonemap(luminance_in);
        assert!(
            (luminance_out - expected_luminance_out).abs() < 1e-12,
            "tonemapped luminance should match reinhard_tonemap(luminance_in): \
             luminance_out={luminance_out} expected={expected_luminance_out}"
        );

        // 色相(チャンネル比)が保たれていること。
        let ratio_in_gr = radiance.y / radiance.x;
        let ratio_out_gr = tonemapped.y / tonemapped.x;
        assert!((ratio_in_gr - ratio_out_gr).abs() < 1e-12);
        let ratio_in_br = radiance.z / radiance.x;
        let ratio_out_br = tonemapped.z / tonemapped.x;
        assert!((ratio_in_br - ratio_out_br).abs() < 1e-12);
    }

    /// 放射輝度0(黒)はトーンマッピング後も0のまま。
    #[test]
    fn reinhard_tonemap_color_at_zero_radiance_is_zero() {
        let tonemapped = reinhard_tonemap_color(Vec3::ZERO);
        assert_eq!(tonemapped, Vec3::ZERO);
    }
}
