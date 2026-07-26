//! BSDF。設計 docs/17-rendering/02-path-tracing.md §8「BVH + 拡散/鏡面BSDF + NEE(基本パストレ)」、
//! docs/17-rendering/03-materials-camera.md §8「BSDF(拡散→誘電体→金属→粗面透過)」。
//! **縮約実装の理由**: 拡散(Lambertian)+誘電体(`Dielectric`、実屈折率)のみを実装する
//! (金属(複素屈折率 $n+ik$)・粗面透過は後続増分、設計§8の実装順序どおり)。分光
//! (波長ごとの反射率)は未実装のため、`albedo`・屈折率はいずれもモノクロのスカラーとする。
//!
//! `sim_em::raytracer`(光学ドメイン)が既に持つ`Ray`/`SurfaceGeom`/`OpticalSurface`は
//! 目的が異なる別実装として意図的に区別する: あちらは決定論的なパワー分岐トレース
//! (フレネル係数でエネルギーを反射/透過に分配し両方の経路を追跡、E9–E12のエネルギー
//! 収支検証が目的)であるのに対し、こちら(`sim_render`)はモンテカルロ経路追跡
//! (乱数で1つの経路を確率的に選び、多数サンプルの平均で輸送方程式を推定、画像合成が
//! 目的)であり、状態表現(`sim_em::Ray`は`power`を持つが`sim_render::Ray`は経路の
//! スループットを再帰呼び出しの掛け算で表現する)が本質的に異なるため型を共有しない。
//! フレネル反射率の解析式(`sim_em::fresnel_reflectance`)自体は重複させず再利用する。

use sim_math::{SimRng, Vec3};

/// 誘電体(実屈折率のみ、金属の複素屈折率は対象外)のフレネル反射率。
/// `sim_em::fresnel_reflectance`(既にE9/E10で検証済みの解析式)をそのまま
/// 再利用する(設計§2規則1と同じ「既存の検証済み実装を再利用し重複させない」方針)。
#[derive(Clone, Copy, Debug)]
pub struct Dielectric {
    /// 界面の外側(真空・空気相当、n=1)に対する相対屈折率。
    pub ior: f64,
}

impl Dielectric {
    /// 非偏光平均のフレネル反射率(設計docs/17-rendering/02-path-tracing.md §5
    /// 「偏光は既定オフ、フルネルは非偏光平均」)。全反射(TIR)なら1.0
    /// (`sim_em::optics::fresnel_reflectance`のdocが明記する「呼び出し側の判断」)。
    pub fn reflectance(&self, cos_theta_i: f64) -> f64 {
        let theta_i = cos_theta_i.clamp(-1.0, 1.0).acos();
        sim_em::fresnel_reflectance(1.0, self.ior, theta_i)
            .map(|r| r.r_unpolarized)
            .unwrap_or(1.0)
    }

    /// 反射/透過どちらの経路をたどるかを、フレネル反射率に比例した確率で確率的に
    /// 選ぶ(ロシアンルーレット式の分岐、鏡面BSDFの標準的な実装)。`true`なら反射。
    pub fn sample_is_reflected(&self, cos_theta_i: f64, rng: &mut SimRng) -> bool {
        rng.next_f64() < self.reflectance(cos_theta_i)
    }
}

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

    /// コサイン重み付きサンプリングは、法線方向の平均コサインが2/3に収束する
    /// ($\int_0^{\pi/2}\cos\theta\cdot(\cos\theta/\pi)\cdot2\pi\sin\theta\,d\theta=2/3$、
    /// 密度関数の正規化自体はサンプル生成コードとは独立に検算できる解析値)。
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

    /// R2: フルネル反射率(誘電体、docs/21-verification/01-analytic-tests.md)。
    /// 垂直入射(θ=0)ではフレネル反射率が閉形式$((n-1)/(n+1))^2$に厳密に一致する
    /// (`sim_em::optics`のE9が既に検証済みの式をそのまま再利用しているだけなので、
    /// ここでは配線——`Dielectric::reflectance`が正しい引数で呼んでいること——を
    /// 確認する)。
    #[test]
    fn r2_fresnel_reflectance_at_normal_incidence_matches_closed_form() {
        let glass = Dielectric { ior: 1.5 };
        let measured = glass.reflectance(1.0); // cosθ=1 → θ=0(垂直入射)。
        let expected = ((glass.ior - 1.0) / (glass.ior + 1.0)).powi(2);
        let rel_err = (measured - expected).abs() / expected;
        assert!(rel_err < 1e-9, "measured={measured} expected={expected}");
    }

    /// 全反射(TIR、臨界角を超える斜入射)では反射率が1.0になること(`sim_em::optics::
    /// fresnel_reflectance`が`None`を返すケースの「呼び出し側の判断」を実装する)。
    /// フレネル係数はn1・n2の比のみに依存する(両方を同じ係数で拡大縮小しても
    /// Snell則・反射率の式は不変)ため、`ior=1/1.5`(n1=1.0→n2=1/1.5)は
    /// 「ガラス(n=1.5)から真空へ」の全反射条件(臨界角 sinθ_c=1/1.5、θ_c≈41.8°)と
    /// 数学的に同一になる。
    #[test]
    fn r2_dielectric_reflectance_is_total_at_grazing_angle_beyond_critical_angle() {
        let dense_to_rare = Dielectric { ior: 1.0 / 1.5 };
        let steep_angle_cos = 0.1; // θ≈84°、臨界角(≈41.8°)より大きい。
        let measured = dense_to_rare.reflectance(steep_angle_cos);
        assert_eq!(measured, 1.0, "measured={measured}");
    }

    /// フレネル反射率に比例した確率的な反射/透過の分岐(`sample_is_reflected`)が、
    /// 実際に`reflectance`が返す確率どおりの頻度で反射を選ぶこと(モンテカルロの
    /// 分岐ロジック自体の配線確認、フレネル反射率の値自体は上記の決定論的テストで
    /// 既に検証済み)。
    #[test]
    fn sample_is_reflected_frequency_matches_the_analytic_reflectance() {
        let glass = Dielectric { ior: 1.5 };
        let cos_theta_i = 1.0;
        let expected_r = glass.reflectance(cos_theta_i);
        let mut rng = SimRng::new(3, 3);
        let n = 500_000;
        let mut reflected_count = 0u32;
        for _ in 0..n {
            if glass.sample_is_reflected(cos_theta_i, &mut rng) {
                reflected_count += 1;
            }
        }
        let measured_r = reflected_count as f64 / n as f64;
        let rel_err = (measured_r - expected_r).abs() / expected_r;
        assert!(
            rel_err < 0.02,
            "measured_r={measured_r} expected_r={expected_r} rel_err={rel_err}"
        );
    }
}
