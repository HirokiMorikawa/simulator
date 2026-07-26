//! BSDF。設計 docs/17-rendering/02-path-tracing.md §8「BVH + 拡散/鏡面BSDF + NEE(基本パストレ)」、
//! docs/17-rendering/03-materials-camera.md §8「BSDF(拡散→誘電体→金属→粗面透過)」。
//! **縮約実装の理由**: 拡散(Lambertian)+誘電体(`Dielectric`、実屈折率)+金属
//! (`Metal`、複素屈折率$n+ik$、完全鏡面——設計が本来求めるGGXマイクロファセット
//! 分布(粗さ)は対象外、`roughness=0`の鏡面反射のみ)を実装する。粗面透過は後続増分。
//! 分光(波長ごとの反射率)は未実装のため、`albedo`・屈折率はいずれもモノクロの
//! スカラーとする。
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

    /// 一般形のフレネル反射率(入射側媒質`n1`→透過側媒質`n2`)。`reflectance`は
    /// 「真空から本材質へ入射する」場合(`n1=1.0, n2=self.ior`固定)の特殊形だが、
    /// 誘電体球から外へ抜ける場合は逆向き(`n1=self.ior, n2=1.0`)の反射率が必要
    /// なため、この一般形を`Scene::trace`(入射/透過の両方向を扱う)から使う。
    /// 全反射(TIR)なら1.0(モジュールdoc「呼び出し側の判断」を踏襲)。
    pub fn reflectance_between(cos_theta_i: f64, n1: f64, n2: f64) -> f64 {
        let theta_i = cos_theta_i.clamp(-1.0, 1.0).acos();
        sim_em::fresnel_reflectance(n1, n2, theta_i)
            .map(|r| r.r_unpolarized)
            .unwrap_or(1.0)
    }

    /// 鏡面反射方向: $d - 2(d\cdot n)n$(`n`は入射側から見て外向き、標準公式)。
    pub fn reflect(direction: Vec3, normal: Vec3) -> Vec3 {
        direction - normal.scale(2.0 * direction.dot(normal))
    }

    /// Snellの法則のベクトル形による屈折方向。`normal`は入射側から見て外向き
    /// ($d\cdot n<0$)、`eta`は相対屈折率$n_1/n_2$(入射側媒質/透過側媒質)。
    /// 全反射(TIR、$\sin^2\theta_t>1$)なら`None`。
    pub fn refract(direction: Vec3, normal: Vec3, eta: f64) -> Option<Vec3> {
        let cos_theta_i = -direction.dot(normal);
        let sin2_theta_t = eta * eta * (1.0 - cos_theta_i * cos_theta_i).max(0.0);
        if sin2_theta_t > 1.0 {
            return None;
        }
        let cos_theta_t = (1.0 - sin2_theta_t).sqrt();
        Some(direction.scale(eta) + normal.scale(eta * cos_theta_i - cos_theta_t))
    }
}

/// 分散する誘電体(設計docs/13-electromagnetism/04-light-optics.md §2.2「Cauchy式
/// $n(\lambda)=A+B/\lambda^2$」、R3「分光/屈折」の検証対象)。`sim_em::
/// cauchy_refractive_index`(既に検証済みの解析式)をそのまま再利用する。
///
/// **縮約実装の理由**: 設計の完全な分光レンダリング(hero wavelength法、1経路で
/// 複数波長を相関サンプルしCIE等色関数でRGBへ変換、設計§3)はパストレーサ全体に
/// 波長を通す大掛かりな変更を要するため後続増分に残す。本増分では、`Dielectric`
/// (単色・固定屈折率)を特定の波長で具体化する`to_dielectric_at`のみを提供し、
/// 分散(波長ごとに屈折角が異なること)そのものの正しさを、既存の`Dielectric::
/// refract`(Snellの法則、既に検証済み)を複数の波長で呼び分けるだけで確認する
/// (`sim_render`側に新しい幾何計算を追加しない、既存コードの再利用)。
#[derive(Clone, Copy, Debug)]
pub struct CauchyDielectric {
    pub a: f64,
    pub b: f64,
}

impl CauchyDielectric {
    /// 波長`wavelength_nm`(ナノメートル)における屈折率。
    pub fn ior_at(&self, wavelength_nm: f64) -> f64 {
        sim_em::cauchy_refractive_index(self.a, self.b, wavelength_nm)
    }

    /// 指定波長での単色`Dielectric`を具体化する(既存の`reflect`/`refract`/
    /// `reflectance`をそのまま使うため)。
    pub fn to_dielectric_at(&self, wavelength_nm: f64) -> Dielectric {
        Dielectric {
            ior: self.ior_at(wavelength_nm),
        }
    }
}

/// 金属(複素屈折率$n+ik$、完全鏡面——設計のGGXマイクロファセット分布(粗さ)は
/// 対象外、モジュールdoc参照)。誘電体と異なり透過が無い(不透明)ため、鏡面反射
/// 方向(`Dielectric::reflect`を再利用、反射則自体は材質に依らない幾何なので
/// 重複させない)のみを持ち、その振幅がフレネル反射率でスケールされる
/// (吸収された分(1-R)はエネルギーとして失われる、誘電体のような反射/透過の
/// 確率的分岐は不要——透過という代替経路自体が存在しないため)。
#[derive(Clone, Copy, Debug)]
pub struct Metal {
    pub n: f64,
    pub k: f64,
}

impl Metal {
    /// 非偏光平均のフレネル反射率。`sim_em::conductor_reflectance`(既に
    /// 「k=0で誘電体反射率に厳密に帰着する」ことを検証済みの解析式)をそのまま
    /// 再利用する。
    pub fn reflectance(&self, cos_theta_i: f64) -> f64 {
        let theta_i = cos_theta_i.clamp(-1.0, 1.0).acos();
        sim_em::conductor_reflectance(self.n, self.k, theta_i)
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

    /// `reflect`は入射角=反射角(鏡映反射の定義そのもの)を厳密に満たす。
    #[test]
    fn reflect_preserves_the_angle_of_incidence() {
        let normal = Vec3::new(0.0, 1.0, 0.0);
        let direction = Vec3::new(0.6, -0.8, 0.0); // 正規化済み(0.6²+0.8²=1)。
        let reflected = Dielectric::reflect(direction, normal);
        assert!((reflected.length() - 1.0).abs() < 1e-9);
        // 反射則: 法線方向の成分は反転、接線方向の成分は不変。
        assert!((reflected.x - 0.6).abs() < 1e-9);
        assert!((reflected.y - 0.8).abs() < 1e-9);
    }

    /// `refract`はSnellの法則$n_1\sin\theta_1=n_2\sin\theta_2$を厳密に満たす
    /// (R3「分光/屈折」が要求する屈折方向の正確さの、単一波長版の検証)。
    #[test]
    fn refract_satisfies_snells_law() {
        let normal = Vec3::new(0.0, 1.0, 0.0);
        let theta_i = 0.4_f64; // ラジアン、臨界角より十分小さい。
        let direction = Vec3::new(theta_i.sin(), -theta_i.cos(), 0.0);
        let n1 = 1.0;
        let n2 = 1.5;
        let eta = n1 / n2;
        let refracted =
            Dielectric::refract(direction, normal, eta).expect("should not TIR at this angle");
        assert!((refracted.length() - 1.0).abs() < 1e-9);

        let theta_t = (-refracted.dot(normal)).acos();
        let lhs = n1 * theta_i.sin();
        let rhs = n2 * theta_t.sin();
        assert!(
            (lhs - rhs).abs() < 1e-9,
            "Snell's law violated: n1 sinθ1={lhs} n2 sinθ2={rhs}"
        );
    }

    /// 臨界角を超える入射では`refract`が`None`(全反射)を返す。
    #[test]
    fn refract_returns_none_beyond_the_critical_angle() {
        let normal = Vec3::new(0.0, 1.0, 0.0);
        let n1 = 1.5;
        let n2 = 1.0;
        let eta = n1 / n2;
        // 臨界角 = asin(n2/n1) = asin(1/1.5) ≈ 0.7297 rad(≈41.8°)。
        let steep_theta_i = 1.3_f64; // ≈74.5°、臨界角より大きい。
        let direction = Vec3::new(steep_theta_i.sin(), -steep_theta_i.cos(), 0.0);
        assert!(Dielectric::refract(direction, normal, eta).is_none());
    }

    /// R3: 分散(虹の色分散)——同じ入射角でも波長ごとに屈折率が異なるため屈折角も
    /// 異なり、Snellの法則$n_1\sin\theta_1=n_2\sin\theta_2$は各波長でそれぞれ厳密に
    /// 成り立ちながら、短波長(青)の方が長波長(赤)より大きく曲がる(屈折角が
    /// 法線に近い、`n`が大きいほど屈折角は小さくなる)ことを確認する。
    #[test]
    fn cauchy_dielectric_disperses_different_wavelengths_into_different_refraction_angles() {
        // 文献の標準的なBK7 Cauchy係数(可視域近似、sim-em::optics::tests参照)。
        let bk7 = CauchyDielectric {
            a: 1.5046,
            b: 4200.0,
        };
        let n_blue = bk7.ior_at(486.1);
        let n_red = bk7.ior_at(656.3);
        assert!(n_blue > n_red, "n_blue={n_blue} n_red={n_red}");

        let normal = Vec3::new(0.0, 1.0, 0.0);
        let theta_i = 0.6_f64; // 臨界角の心配がない適度な入射角(真空→ガラス、TIR無し)。
        let direction = Vec3::new(theta_i.sin(), -theta_i.cos(), 0.0);

        let refract_angle = |wavelength_nm: f64| -> f64 {
            let dielectric = bk7.to_dielectric_at(wavelength_nm);
            let eta = 1.0 / dielectric.ior;
            let refracted = Dielectric::refract(direction, normal, eta)
                .expect("should not TIR entering a denser medium");
            let theta_t = (-refracted.dot(normal)).acos();
            // 各波長でSnellの法則がそれぞれ厳密に成り立つことも合わせて確認する。
            let lhs = 1.0 * theta_i.sin();
            let rhs = dielectric.ior * theta_t.sin();
            assert!(
                (lhs - rhs).abs() < 1e-9,
                "Snell's law violated at {wavelength_nm}nm: n1 sinθ1={lhs} n2 sinθ2={rhs}"
            );
            theta_t
        };

        let theta_t_blue = refract_angle(486.1);
        let theta_t_red = refract_angle(656.3);
        assert!(
            theta_t_blue < theta_t_red,
            "blue (higher n) should refract closer to the normal than red: \
             theta_t_blue={theta_t_blue} theta_t_red={theta_t_red}"
        );
    }

    /// R2(金属側): `Metal::reflectance`が`sim_em::conductor_reflectance`(既に
    /// 検証済みの解析式)をそのまま呼んでいることの配線確認。金(Au、550nm、設計
    /// docs/17-rendering/03-materials-camera.md §9)の垂直入射反射率を閉形式と
    /// 直接比較する。
    #[test]
    fn metal_reflectance_matches_conductor_closed_form_at_normal_incidence() {
        let gold = Metal { n: 0.47, k: 2.4 };
        let measured = gold.reflectance(1.0); // cosθ=1 → 垂直入射。
        let expected =
            ((gold.n - 1.0).powi(2) + gold.k * gold.k) / ((gold.n + 1.0).powi(2) + gold.k * gold.k);
        let rel_err = (measured - expected).abs() / expected;
        assert!(rel_err < 1e-9, "measured={measured} expected={expected}");
    }
}
