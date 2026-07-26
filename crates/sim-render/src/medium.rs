//! 参加媒質(体積散乱)——レイリー散乱。設計 docs/17-rendering/02-path-tracing.md §2.2「参加
//! 媒質: 放射伝達方程式(RTE): 吸収σ_a・散乱σ_s係数と位相関数p。大気: レイリー散乱
//! (σ_s ∝ λ^-4、空の青)」・§7「大気: レイリー散乱のλ^-4による空の青・地平線の赤の定量」。
//!
//! **縮約実装の理由**: 完全な体積レンダリング(マルチスキャッタリング、レイマーチングに
//! よる経路上の散乱イベントのサンプリング、`Scene::trace`への本格的な配線)ではなく、
//! 一様(homogeneous)大気を仮定した単一散乱(single scattering)の閉形式解のみを実装する。
//! 太陽光が媒質中で減衰しない(媒質が太陽方向に対して光学的に薄い、遠くの太陽光源からの
//! 入射強度は場所によらず一定とみなす)と仮定すると、カメラ側の減衰(Beer-Lambert則)
//! だけを積分すればよく、解析的に閉じた式が得られる(統計的収束を待つ必要が無い、
//! このセッション一貫の検証方針)。ミー散乱(エアロゾル・雲)・マルチスキャッタリング・
//! 煙/水の密度場からの体積散乱は対象外(後続増分)。

/// 波長`wavelength_nm`でのレイリー散乱係数(σ_s ∝ λ^-4)。`reference_coefficient`は
/// 参照波長550nmでの係数(設計§9: 海面でβ_R(550nm)≈1.16e-5/m)。
pub fn rayleigh_scattering_coefficient(wavelength_nm: f64, reference_coefficient: f64) -> f64 {
    const REFERENCE_WAVELENGTH_NM: f64 = 550.0;
    reference_coefficient * (REFERENCE_WAVELENGTH_NM / wavelength_nm).powi(4)
}

/// レイリー位相関数(非偏光平均): $p(\cos\theta) = \frac{3}{16\pi}(1+\cos^2\theta)$。
/// 単位は1/sr、全立体角で積分すると1に正規化される(`tests::
/// rayleigh_phase_function_integrates_to_one_over_the_sphere`で確認)。
pub fn rayleigh_phase(cos_theta: f64) -> f64 {
    3.0 / (16.0 * std::f64::consts::PI) * (1.0 + cos_theta * cos_theta)
}

/// 一様(homogeneous)散乱媒質。大気のレイリー散乱は純散乱(吸収σ_a≈0)なので、
/// 消散係数σ_tは散乱係数σ_sに等しいとみなす。
#[derive(Clone, Copy, Debug)]
pub struct HomogeneousMedium {
    pub sigma_s: f64,
}

impl HomogeneousMedium {
    /// 波長`wavelength_nm`のレイリー大気媒質(σ_s ∝ λ^-4、`reference_coefficient`は
    /// 550nmでの参照値)。
    pub fn rayleigh_atmosphere(
        wavelength_nm: f64,
        reference_coefficient: f64,
    ) -> HomogeneousMedium {
        HomogeneousMedium {
            sigma_s: rayleigh_scattering_coefficient(wavelength_nm, reference_coefficient),
        }
    }

    /// Beer-Lambert則による透過率: 距離`distance`を進む間に散乱されずに直進で残る割合、
    /// $T(d) = e^{-\sigma_s d}$。
    pub fn transmittance(&self, distance: f64) -> f64 {
        (-self.sigma_s * distance).exp()
    }

    /// 単一散乱(single scattering)放射輝度の閉形式解。カメラから距離`path_length`まで
    /// 一様媒質を通して、視線と太陽方向のなす角の余弦`cos_theta`の方向から入射する
    /// 太陽放射輝度`sun_radiance`が経路上で散乱されカメラに届く量(太陽光自体は媒質中で
    /// 減衰しないと仮定、モジュールdoc「縮約実装の理由」参照):
    /// $$L = \int_0^D \sigma_s \, p(\cos\theta) \, L_{sun} \, e^{-\sigma_s t}\,dt
    ///     = p(\cos\theta)\, L_{sun} \, (1 - e^{-\sigma_s D})$$
    pub fn single_scattering_radiance(
        &self,
        path_length: f64,
        cos_theta: f64,
        sun_radiance: f64,
    ) -> f64 {
        rayleigh_phase(cos_theta) * sun_radiance * (1.0 - self.transmittance(path_length))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const REFERENCE_COEFFICIENT_550NM: f64 = 1.16e-5; // 設計§9: β_R(550nm)、海面、/m。

    #[test]
    fn rayleigh_scattering_coefficient_matches_the_reference_at_550nm() {
        let sigma_s = rayleigh_scattering_coefficient(550.0, REFERENCE_COEFFICIENT_550NM);
        assert!((sigma_s - REFERENCE_COEFFICIENT_550NM).abs() < 1e-15);
    }

    /// σ_s ∝ λ^-4 (設計§2.2)。450nm(青)と650nm(赤)の比は (650/450)^4 に厳密に一致する。
    #[test]
    fn rayleigh_scattering_coefficient_scales_as_inverse_fourth_power_of_wavelength() {
        let sigma_blue = rayleigh_scattering_coefficient(450.0, REFERENCE_COEFFICIENT_550NM);
        let sigma_red = rayleigh_scattering_coefficient(650.0, REFERENCE_COEFFICIENT_550NM);
        let expected_ratio = (650.0_f64 / 450.0).powi(4);
        let measured_ratio = sigma_blue / sigma_red;
        assert!(
            (measured_ratio - expected_ratio).abs() / expected_ratio < 1e-12,
            "measured_ratio={measured_ratio} expected_ratio={expected_ratio}"
        );
    }

    /// レイリー位相関数の全立体角積分は1に正規化される(位相関数がcosθのみに依存する
    /// ため、方位角φの積分は自明な2πの係数になり、天頂角側をu=cosθの数値求積で
    /// 確認する)。
    #[test]
    fn rayleigh_phase_function_integrates_to_one_over_the_sphere() {
        const N: usize = 1_000_000;
        let du = 2.0 / N as f64;
        let mut sum = 0.0;
        for i in 0..N {
            let u = -1.0 + (i as f64 + 0.5) * du;
            sum += rayleigh_phase(u) * du;
        }
        let integral = sum * 2.0 * std::f64::consts::PI;
        assert!(
            (integral - 1.0).abs() < 1e-9,
            "phase function integral over the sphere = {integral}, expected 1.0"
        );
    }

    #[test]
    fn transmittance_is_one_at_zero_distance_and_decays_monotonically() {
        let medium = HomogeneousMedium::rayleigh_atmosphere(550.0, REFERENCE_COEFFICIENT_550NM);
        assert!((medium.transmittance(0.0) - 1.0).abs() < 1e-15);
        let t1 = medium.transmittance(1000.0);
        let t2 = medium.transmittance(2000.0);
        assert!(t2 < t1, "transmittance must decay as distance increases");
        assert!((t1 - (-medium.sigma_s * 1000.0).exp()).abs() < 1e-15);
    }

    #[test]
    fn single_scattering_radiance_is_zero_at_zero_path_length() {
        let medium = HomogeneousMedium::rayleigh_atmosphere(450.0, REFERENCE_COEFFICIENT_550NM);
        let l = medium.single_scattering_radiance(0.0, 0.0, 1.0);
        assert!(l.abs() < 1e-15);
    }

    /// 単一散乱の閉形式解が、経路積分を素朴に数値積分(中点則、十分細かい分割)した
    /// 結果と一致することを確認する(閉形式導出の実装バグ検出、`conductor_reflectance`
    /// のk=0帰着チェックと同種の自己無撞着性検証)。
    #[test]
    fn single_scattering_closed_form_matches_numerical_path_integration() {
        let medium = HomogeneousMedium::rayleigh_atmosphere(500.0, REFERENCE_COEFFICIENT_550NM);
        let path_length = 5000.0;
        let cos_theta = 0.3;
        let sun_radiance = 2.5;

        const N: usize = 200_000;
        let dt = path_length / N as f64;
        let mut numerical = 0.0;
        for i in 0..N {
            let t = (i as f64 + 0.5) * dt;
            numerical += medium.sigma_s
                * rayleigh_phase(cos_theta)
                * sun_radiance
                * (-medium.sigma_s * t).exp()
                * dt;
        }

        let closed_form = medium.single_scattering_radiance(path_length, cos_theta, sun_radiance);
        let rel_err = (numerical - closed_form).abs() / closed_form;
        assert!(
            rel_err < 1e-6,
            "numerical={numerical} closed_form={closed_form} rel_err={rel_err}"
        );
    }

    /// R5(空の青): 光学的に薄い(σ_s・D ≪ 1)極限では単一散乱放射輝度が経路長Dに
    /// 線形になるため、青(450nm)/赤(650nm)の比はσ_sの比、すなわち(650/450)^4に近づく
    /// (空を見上げたときの散乱光が波長λ^-4で青に強く偏ることの定量)。
    #[test]
    fn sky_scattering_is_stronger_for_blue_than_red_and_matches_the_optically_thin_ratio() {
        let path_length = 10.0; // sigma_s * path_length ~ 1e-4、光学的に薄い極限。
        let cos_theta = 0.5;
        let sun_radiance = 1.0;

        let blue = HomogeneousMedium::rayleigh_atmosphere(450.0, REFERENCE_COEFFICIENT_550NM)
            .single_scattering_radiance(path_length, cos_theta, sun_radiance);
        let red = HomogeneousMedium::rayleigh_atmosphere(650.0, REFERENCE_COEFFICIENT_550NM)
            .single_scattering_radiance(path_length, cos_theta, sun_radiance);

        assert!(
            blue > red,
            "sky scattering must be stronger for blue: blue={blue} red={red}"
        );

        let expected_ratio = (650.0_f64 / 450.0).powi(4);
        let measured_ratio = blue / red;
        assert!(
            (measured_ratio - expected_ratio).abs() / expected_ratio < 1e-3,
            "measured_ratio={measured_ratio} expected_ratio={expected_ratio}"
        );
    }

    /// 地平線の赤(sunset): 長い大気経路を直進してくる太陽光そのものの透過率は、
    /// 青がより強く散乱により失われるため赤より低くなり、経路が長くなるほどその差
    /// (赤みの度合い)が拡大する(青/赤の透過率比が単調に減少する)。
    #[test]
    fn direct_transmittance_reddens_the_sun_over_a_long_horizon_path() {
        let blue_medium =
            HomogeneousMedium::rayleigh_atmosphere(450.0, REFERENCE_COEFFICIENT_550NM);
        let red_medium = HomogeneousMedium::rayleigh_atmosphere(650.0, REFERENCE_COEFFICIENT_550NM);

        let short_path = 10_000.0; // 天頂方向相当の短い経路。
        let long_path = 300_000.0; // 地平線方向相当の長い経路(大気シェル近似)。

        let ratio_short =
            blue_medium.transmittance(short_path) / red_medium.transmittance(short_path);
        let ratio_long = blue_medium.transmittance(long_path) / red_medium.transmittance(long_path);

        assert!(
            ratio_short < 1.0,
            "blue must already transmit less than red: {ratio_short}"
        );
        assert!(
            ratio_long < ratio_short,
            "the blue/red transmittance ratio must shrink further over the longer horizon path \
             (deeper reddening): ratio_short={ratio_short} ratio_long={ratio_long}"
        );
    }
}
