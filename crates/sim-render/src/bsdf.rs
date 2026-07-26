//! BSDF。設計 docs/17-rendering/02-path-tracing.md §8「BVH + 拡散/鏡面BSDF + NEE(基本パストレ)」。
//! **縮約実装の理由**: 拡散(Lambertian)のみを実装する(鏡面・誘電体・金属・分光は後続増分)。
//! 分光(波長ごとの反射率)は未実装のため、`albedo`はモノクロのスカラー反射率とする。

use sim_math::{SimRng, Vec3};

/// 完全拡散(Lambertian)BSDF。$f_r = \rho/\pi$(設計§2.1のレンダリング方程式のBSDF項)。
#[derive(Clone, Copy, Debug)]
pub struct Lambertian {
    /// アルベド $\rho \in [0,1]$(エネルギー保存を満たすには$\rho \le 1$、白色炉テストは$\rho=1$)。
    pub albedo: f64,
}

impl Lambertian {
    /// 法線`normal`まわりのコサイン重み付き半球サンプリング(Malley法の変種:
    /// 単位球面上の一様点`d`を`normal + d`として正規化すると、確率密度が
    /// $\cos\theta$に比例した半球分布になる、標準的な構成)。
    /// 戻り値: (サンプル方向, pdf = cosθ/π)。
    pub fn sample(&self, normal: Vec3, rng: &mut SimRng) -> (Vec3, f64) {
        let offset = rng.unit_sphere();
        let direction = (normal + offset).normalize_or_zero();
        let cos_theta = direction.dot(normal).max(1e-12);
        let pdf = cos_theta / std::f64::consts::PI;
        (direction, pdf)
    }

    /// BSDF値(方向によらず一定、Lambertianの定義)。
    pub fn eval(&self) -> f64 {
        self.albedo / std::f64::consts::PI
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// サンプルされた方向は常に法線側の半球内(cosθ>0)にあること。
    #[test]
    fn sampled_directions_stay_in_the_hemisphere_around_the_normal() {
        let normal = Vec3::new(0.0, 1.0, 0.0);
        let material = Lambertian { albedo: 0.5 };
        let mut rng = SimRng::new(1, 1);
        for _ in 0..1000 {
            let (direction, pdf) = material.sample(normal, &mut rng);
            assert!(direction.dot(normal) > 0.0);
            assert!(pdf > 0.0);
            assert!((direction.length() - 1.0).abs() < 1e-9);
        }
    }

    /// コサイン重み付きサンプリングは、法線方向の平均コサインが1/2に収束する
    /// (半球上でコサイン重み付き分布の$E[\cos\theta]=2/3$ではなく、
    /// $\int_0^{\pi/2}\cos\theta\cdot(\cos\theta/\pi)\cdot2\pi\sin\theta\,d\theta=2/3$
    /// になることの統計的確認——密度関数の正規化自体はサンプル生成コードとは独立に
    /// 検算できる解析値)。
    #[test]
    fn mean_cosine_of_cosine_weighted_samples_matches_two_thirds() {
        let normal = Vec3::new(0.0, 1.0, 0.0);
        let material = Lambertian { albedo: 1.0 };
        let mut rng = SimRng::new(2, 2);
        let n = 200_000;
        let mut sum_cos = 0.0;
        for _ in 0..n {
            let (direction, _pdf) = material.sample(normal, &mut rng);
            sum_cos += direction.dot(normal);
        }
        let mean_cos = sum_cos / n as f64;
        let expected = 2.0 / 3.0;
        let rel_err = (mean_cos - expected).abs() / expected;
        assert!(rel_err < 0.01, "mean_cos={mean_cos} expected={expected}");
    }
}
