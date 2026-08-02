//! 分光レンダリング — hero wavelength法とCIE等色関数によるXYZ→sRGB変換。
//! 設計: docs/17-rendering/02-path-tracing.md §3、docs/17-rendering/03-materials-camera.md §4.2。
//!
//! **これが無かったあいだ何ができなかったか**: `Scene::trace`はモノクロの`f64`を返し、
//! 波長という概念自体を持たなかった。R4(コーネルボックス)やD41(材質ギャラリー)の
//! 色は「同一形状でアルベドだけR/G/Bに差し替えた`Scene`を3つ作り`trace`を3回走らせる」
//! という方式で出しており、これは**RGBが3つの固定された基底であるという前提**に
//! 依存する。分散(波長ごとの屈折率差)を持つ誘電体をこの方式に載せると、
//! 「Rチャンネルは赤の屈折率で、Gは緑で」という不整合な3枚を合成することになり、
//! プリズムの虹やコースティクスの色付きは原理的に出せない。
//!
//! # hero wavelength法(設計§3)
//!
//! 1本の経路で複数波長を**相関サンプル**する。まず可視域から1本の「主波長」
//! (hero wavelength)$\lambda_0$を一様サンプルし、残りの$N-1$本を
//! $\lambda_j = \lambda_0 + j\Delta$($\Delta$=可視域幅$/N$、上端を超えたら下端へ巻き戻す)
//! として等間隔に取る。こうすると
//!
//! - 1経路あたりの分散が単純な独立サンプリングより小さい(波長間で幾何が共有される)
//! - 各波長が可視域全体を一様に覆う(層化サンプリング)
//!
//! という2つの性質が同時に得られる。屈折のように波長で経路が分岐する場合、
//! 厳密には各波長で別経路を追う必要があるため、**本実装は`trace_spectral`を
//! 波長ごとに呼ぶ**(=相関しているのはピクセル内の乱数列とサンプリング位置であって
//! 経路そのものではない)。設計が意図する「1経路で複数波長を運ぶ」完全形では
//! ないが、分散を持つ媒質を正しく扱いつつ層化の利得を得る縮約として選んだ。
//!
//! # CIE等色関数(設計§4.2)
//!
//! 実測テーブル(5nm刻み、95点×3列)を持たず、Wyman・Sloan・Shirley (2013)
//! "Simple Analytic Approximations to the CIE XYZ Color Matching Functions" の
//! **多ローブ区分ガウス近似**を使う。依存追加なし・約20行で、CIE 1931 2°観測者に
//! 対し可視域全体で十分な精度(論文報告の最大誤差は数%オーダー)を持つ。
//! **縮約であることを明記する**: 測色用途(色差ΔEの厳密評価)には足りない。
//! ここでの用途は「プリズムの分散が実際に虹として見えるか」の描画であり、
//! その判定には十分である。

/// 可視域の下端・上端[nm]。CIE等色関数がほぼ0になる範囲を採る。
pub const LAMBDA_MIN_NM: f64 = 380.0;
pub const LAMBDA_MAX_NM: f64 = 780.0;

/// 区分ガウス $g(x;\mu,\sigma_1,\sigma_2)$。$x<\mu$ なら $\sigma_1$、そうでなければ
/// $\sigma_2$ を使う(左右で幅の違うローブを1つの式で書くための道具、Wyman et al.)。
fn piecewise_gaussian(x: f64, mu: f64, sigma_left: f64, sigma_right: f64) -> f64 {
    let sigma = if x < mu { sigma_left } else { sigma_right };
    let t = (x - mu) / sigma;
    (-0.5 * t * t).exp()
}

/// CIE 1931 2°観測者の等色関数 $(\bar x,\bar y,\bar z)$ の多ローブ近似
/// (モジュールdoc参照)。
pub fn cie_xyz_at(wavelength_nm: f64) -> (f64, f64, f64) {
    let l = wavelength_nm;
    let x = 1.056 * piecewise_gaussian(l, 599.8, 37.9, 31.0)
        + 0.362 * piecewise_gaussian(l, 442.0, 16.0, 26.7)
        - 0.065 * piecewise_gaussian(l, 501.1, 20.4, 26.2);
    let y = 0.821 * piecewise_gaussian(l, 568.8, 46.9, 40.5)
        + 0.286 * piecewise_gaussian(l, 530.9, 16.3, 31.1);
    let z = 1.217 * piecewise_gaussian(l, 437.0, 11.8, 36.0)
        + 0.681 * piecewise_gaussian(l, 459.0, 26.0, 13.8);
    (x, y, z)
}

/// CIE XYZ → 線形sRGB(D65白色点、IEC 61966-2-1の逆行列)。
/// **ガンマ符号化は行わない**——それは`Framebuffer::to_srgb8`の役目であり、
/// ここが返すのは線形値。負値(sRGB色域外)はそのまま返す(クランプは表示段の責務)。
pub fn xyz_to_linear_srgb(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    (
        3.2406 * x - 1.5372 * y - 0.4986 * z,
        -0.9689 * x + 1.8758 * y + 0.0415 * z,
        0.0557 * x - 0.2040 * y + 1.0570 * z,
    )
}

/// hero wavelength法の波長セット(モジュールdoc参照)。
/// `hero_u`は$[0,1)$の一様乱数、`count`は1経路あたりの波長数。
pub fn hero_wavelengths(hero_u: f64, count: usize) -> Vec<f64> {
    assert!(count > 0, "波長数は1以上");
    let span = LAMBDA_MAX_NM - LAMBDA_MIN_NM;
    let delta = span / count as f64;
    (0..count)
        .map(|j| {
            // 巻き戻し(rotation): 上端を超えたぶんを下端から数え直す。
            let offset = (hero_u * span + j as f64 * delta) % span;
            LAMBDA_MIN_NM + offset
        })
        .collect()
}

/// 分光放射輝度のサンプル列 $(\lambda_i, L_i)$ を線形sRGBへ変換する。
///
/// 波長は一様サンプル(pdf = $1/(\lambda_{max}-\lambda_{min})$)なので、
/// $X=\int \bar x(\lambda)L(\lambda)d\lambda$ のモンテカルロ推定は
/// $\frac{\lambda_{max}-\lambda_{min}}{N}\sum \bar x(\lambda_i)L_i$ になる。
///
/// **正規化**: $\bar y$ の可視域積分で割ることで、平坦なスペクトル
/// $L(\lambda)\equiv c$ の輝度 $Y$ が $c$ に一致するようにする
/// (=「白い物体を白く写す」)。この正規化定数は`Y_INTEGRAL`として実測で求めてある。
pub fn spectral_samples_to_linear_srgb(samples: &[(f64, f64)]) -> (f64, f64, f64) {
    if samples.is_empty() {
        return (0.0, 0.0, 0.0);
    }
    let span = LAMBDA_MAX_NM - LAMBDA_MIN_NM;
    let scale = span / (samples.len() as f64 * Y_INTEGRAL);
    let (mut x, mut y, mut z) = (0.0, 0.0, 0.0);
    for &(lambda, radiance) in samples {
        let (bx, by, bz) = cie_xyz_at(lambda);
        x += bx * radiance;
        y += by * radiance;
        z += bz * radiance;
    }
    xyz_to_linear_srgb(x * scale, y * scale, z * scale)
}

/// $\int_{380}^{780}\bar y(\lambda)d\lambda$(上の近似式に対する値、1nm刻みの数値積分)。
/// `spectral_samples_to_linear_srgb`の正規化に使う。テストで再計算して固定している。
pub const Y_INTEGRAL: f64 = 106.919_734_641_708_54;

#[cfg(test)]
mod tests {
    use super::*;

    /// $\bar y$ のピークは555nm付近(明所視の視感度ピーク)にあり、値はほぼ1。
    /// CIE 1931の定義そのものなので、近似式がこれを外していたら実装が誤っている。
    #[test]
    fn luminous_efficiency_peaks_near_555_nanometers() {
        let mut best = (0.0_f64, 0.0_f64);
        let mut l = LAMBDA_MIN_NM;
        while l <= LAMBDA_MAX_NM {
            let (_, y, _) = cie_xyz_at(l);
            if y > best.1 {
                best = (l, y);
            }
            l += 0.1;
        }
        assert!(
            (best.0 - 555.0).abs() < 8.0,
            "ȳのピークは555nm付近にあるべき: {}nm",
            best.0
        );
        assert!(
            (best.1 - 1.0).abs() < 0.05,
            "ȳのピーク値はほぼ1のはず: {}",
            best.1
        );
    }

    /// `Y_INTEGRAL`定数が実際の数値積分と一致すること(正規化が狂うと全体の
    /// 明るさが狂うため、定数を固定して回帰を防ぐ)。
    #[test]
    fn y_integral_constant_matches_numerical_integration() {
        let step = 0.001;
        let mut sum = 0.0;
        let mut l = LAMBDA_MIN_NM;
        while l < LAMBDA_MAX_NM {
            let (_, y, _) = cie_xyz_at(l + 0.5 * step);
            sum += y * step;
            l += step;
        }
        assert!(
            (sum - Y_INTEGRAL).abs() / Y_INTEGRAL < 1.0e-6,
            "Y_INTEGRAL={Y_INTEGRAL} vs numerical={sum}"
        );
    }

    /// **平坦なスペクトルは無彩色(グレー)になる**。等エネルギー白色は厳密には
    /// D65ではなくE光源なので完全な(1,1,1)にはならないが、3チャンネルが
    /// 互いに近い値になることは色変換の健全性の必要条件である。
    /// あわせて輝度Yが入力値に一致すること(`Y_INTEGRAL`正規化の目的)も見る。
    #[test]
    fn a_flat_spectrum_maps_to_a_near_neutral_color_with_matching_luminance() {
        let n = 400;
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = (i as f64 + 0.5) / n as f64;
                (LAMBDA_MIN_NM + t * (LAMBDA_MAX_NM - LAMBDA_MIN_NM), 1.0)
            })
            .collect();
        let (r, g, b) = spectral_samples_to_linear_srgb(&samples);
        for (name, v) in [("r", r), ("g", g), ("b", b)] {
            assert!(
                (v - 1.0).abs() < 0.35,
                "平坦スペクトルは無彩色に近いはず: {name}={v}"
            );
        }
        // Rec.709の相対輝度が入力の1.0に一致する(正規化の直接の検証)。
        let luminance = 0.2126 * r + 0.7152 * g + 0.0722 * b;
        assert!(
            (luminance - 1.0).abs() < 0.02,
            "平坦スペクトルの輝度は入力値に一致すべき: {luminance}"
        );
    }

    /// **単波長は彩度の高い色になり、波長順に赤→緑→青と主チャンネルが移る**。
    /// これが崩れていたら等色関数かXYZ→sRGB行列のどちらかが誤っている。
    #[test]
    fn monochromatic_samples_map_to_the_expected_dominant_channel() {
        let dominant = |lambda: f64| -> usize {
            let (r, g, b) = spectral_samples_to_linear_srgb(&[(lambda, 1.0)]);
            let mut best = 0;
            let v = [r, g, b];
            for i in 1..3 {
                if v[i] > v[best] {
                    best = i;
                }
            }
            best
        };
        assert_eq!(dominant(630.0), 0, "630nmは赤が主チャンネル");
        assert_eq!(dominant(530.0), 1, "530nmは緑が主チャンネル");
        assert_eq!(dominant(460.0), 2, "460nmは青が主チャンネル");
    }

    /// hero wavelength は可視域を等間隔に覆い、全て可視域内に収まる(巻き戻しの検証)。
    #[test]
    fn hero_wavelengths_stratify_the_visible_range_and_stay_in_bounds() {
        for &u in &[0.0, 0.13, 0.5, 0.87, 0.999] {
            let count = 4;
            let ls = hero_wavelengths(u, count);
            assert_eq!(ls.len(), count);
            for &l in &ls {
                assert!(
                    (LAMBDA_MIN_NM..=LAMBDA_MAX_NM).contains(&l),
                    "巻き戻し後も可視域内に収まるべき: u={u} l={l}"
                );
            }
            // 各波長が別々の等分割区間に1本ずつ入る(層化の定義)。
            let span = LAMBDA_MAX_NM - LAMBDA_MIN_NM;
            let mut bins: Vec<usize> = ls
                .iter()
                .map(|l| (((l - LAMBDA_MIN_NM) / span * count as f64) as usize).min(count - 1))
                .collect();
            bins.sort_unstable();
            bins.dedup();
            assert_eq!(
                bins.len(),
                count,
                "各区間に1本ずつ入るべき: u={u} ls={ls:?}"
            );
        }
    }
}
