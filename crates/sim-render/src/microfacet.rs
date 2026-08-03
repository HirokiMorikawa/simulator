//! GGX(Trowbridge-Reitz)マイクロファセット分布 + Smith遮蔽/マスキング関数。
//! 設計 docs/17-rendering/03-materials-camera.md §8「BSDF(拡散→誘電体→金属→粗面
//! 透過)」のうち、これまで対象外としていた「粗さ」(GGXマイクロファセット分布、
//! `bsdf.rs`・`path_tracer.rs`各モジュールdoc「完全鏡面(粗さ0)のみ、GGXは対象外」
//! 参照)を実装する。
//!
//! **縮約実装の理由**: Smith遮蔽/マスキング関数は height-correlated(高さ相関)版
//! ではなく、より単純な分離可能(separable)版 $G(i,o)=G_1(i)\,G_1(o)$ を用いる
//! (標準的な近似、正確な相関版よりわずかにエネルギーを失うが実装・検証が単純)。
//! マルチスキャッタリング補償(単一散乱モデルが粗い表面でエネルギーをわずかに
//! 失う既知の問題への対処)も対象外。重要度サンプリングは visible normal
//! distribution(VNDF)ではなく、素朴な $D(h)\cos\theta_h$ に比例した半角ベクトル
//! サンプリング(Walter et al. 2007の標準的な構成)を用いる。

use sim_math::{SimRng, Vec3};

/// GGX(Trowbridge-Reitz)法線分布関数 $D(h)$。`cos_theta_h`は半角ベクトルhと
/// マクロ法線nのなす角の余弦、`alpha`は粗さパラメータ。
pub fn ggx_distribution(cos_theta_h: f64, alpha: f64) -> f64 {
    let alpha2 = alpha * alpha;
    let cos2 = cos_theta_h * cos_theta_h;
    let denom = cos2 * (alpha2 - 1.0) + 1.0;
    alpha2 / (std::f64::consts::PI * denom * denom)
}

/// Smith GGXマスキング関数(1方向分): $G_1(v) = \frac{2\cos\theta_v}{\cos\theta_v +
/// \sqrt{\alpha^2 + (1-\alpha^2)\cos^2\theta_v}}$。
pub fn smith_g1(cos_theta_v: f64, alpha: f64) -> f64 {
    let alpha2 = alpha * alpha;
    let cos2 = cos_theta_v * cos_theta_v;
    2.0 * cos_theta_v / (cos_theta_v + (alpha2 + (1.0 - alpha2) * cos2).sqrt())
}

/// 分離可能(separable)Smith遮蔽/マスキング関数: $G(i,o) = G_1(i)\,G_1(o)$
/// (モジュールdoc「縮約実装の理由」参照)。
pub fn smith_g(cos_theta_i: f64, cos_theta_o: f64, alpha: f64) -> f64 {
    smith_g1(cos_theta_i, alpha) * smith_g1(cos_theta_o, alpha)
}

/// マクロ法線`normal`まわりに、GGX分布 $D(h)|\cos\theta_h|$ に比例する確率で
/// 半角ベクトルhをサンプリングする(Walter et al. 2007の標準的な構成、
/// `sim_math::Vec3::orthonormal_basis`で接線基底を作る)。
pub fn sample_ggx_half_vector(normal: Vec3, alpha: f64, rng: &mut SimRng) -> Vec3 {
    let (tangent, bitangent) = normal.orthonormal_basis();
    let xi1 = rng.next_f64();
    let xi2 = rng.next_f64();
    let tan2_theta = alpha * alpha * xi1 / (1.0 - xi1).max(1e-12);
    let cos_theta = (1.0 / (1.0 + tan2_theta)).sqrt();
    let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();
    let phi = 2.0 * std::f64::consts::PI * xi2;
    tangent.scale(sin_theta * phi.cos())
        + bitangent.scale(sin_theta * phi.sin())
        + normal.scale(cos_theta)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// GGX分布の正規化恒等式 $\int D(h)\cos\theta_h\,d\Omega_h = 1$(マイクロ
    /// ファセット分布の定義そのもの、法線分布関数の標準的な正規化条件)。
    /// 天頂角側をu=cosθhの数値求積(中点則)、方位角側は解析的に2π(位相関数の
    /// 正規化テストと同じ手法)で確認する。
    #[test]
    fn ggx_distribution_integrates_to_one_over_the_hemisphere_weighted_by_cosine() {
        for alpha in [0.05, 0.3, 0.7, 1.0] {
            const N: usize = 1_000_000;
            let du = 1.0 / N as f64;
            let mut sum = 0.0;
            for i in 0..N {
                let u = (i as f64 + 0.5) * du; // u=cosθh in (0,1]、半球側のみ。
                sum += ggx_distribution(u, alpha) * u * du;
            }
            let integral = sum * 2.0 * std::f64::consts::PI;
            assert!(
                (integral - 1.0).abs() < 1e-3,
                "alpha={alpha}: GGX distribution cosine-weighted integral = {integral}, expected 1.0"
            );
        }
    }

    /// 垂直入射(cosθv=1)ではSmithマスキング関数は厳密に1(遮蔽が一切無い)。
    #[test]
    fn smith_g1_is_exactly_one_at_normal_incidence() {
        for alpha in [0.05, 0.3, 0.7, 1.0] {
            let g1 = smith_g1(1.0, alpha);
            assert!((g1 - 1.0).abs() < 1e-12, "alpha={alpha}: g1={g1}");
        }
    }

    /// Smithマスキング関数はグレージング角に近づくほど単調に減少する(遮蔽が
    /// 強くなる)。
    #[test]
    fn smith_g1_decreases_monotonically_toward_grazing_angles() {
        let alpha = 0.4;
        let cos_thetas = [1.0, 0.8, 0.6, 0.4, 0.2, 0.05];
        let mut previous = f64::INFINITY;
        for cos_theta in cos_thetas {
            let g1 = smith_g1(cos_theta, alpha);
            assert!(
                g1 < previous + 1e-12,
                "smith_g1 should decrease monotonically toward grazing angles: \
                 cos_theta={cos_theta} g1={g1} previous={previous}"
            );
            assert!(
                (0.0..=1.0).contains(&g1),
                "g1 must stay within [0,1]: g1={g1}"
            );
            previous = g1;
        }
    }

    /// サンプルされた半角ベクトルは常に単位長かつマクロ法線側の半球内にある。
    #[test]
    fn sampled_half_vectors_are_unit_length_and_stay_in_the_hemisphere_around_the_normal() {
        let normal = Vec3::new(0.0, 1.0, 0.0);
        let mut rng = SimRng::new(11, 11);
        for _ in 0..2000 {
            let h = sample_ggx_half_vector(normal, 0.4, &mut rng);
            assert!((h.length() - 1.0).abs() < 1e-9);
            assert!(
                h.dot(normal) > 0.0,
                "half vector must stay in the hemisphere: h={h:?}"
            );
        }
    }
}
