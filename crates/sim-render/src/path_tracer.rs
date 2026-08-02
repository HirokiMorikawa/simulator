//! パストレーサ本体。設計 docs/17-rendering/02-path-tracing.md §4「レンダリング方程式の
//! モンテカルロ解法」。
//!
//! **縮約実装の理由**: 設計の実装順序(§8)「BVH + 拡散/鏡面BSDF + NEE(基本パストレ)」の
//! うち、本増分は拡散(Lambertian)+誘電体(`Dielectric`)BSDF + 一様環境光 + 複数物体の
//! シーン(`SceneObject`のリスト、線形探索で最近傍交差を選ぶ)+ NEE(`PointLight`の
//! 明示サンプル)を実装する。BVH(加速構造)自体は、物体数が線形探索では性能上問題に
//! なる規模のシーンが実際に必要になる増分まで導入を見送る(線形探索は正しさは損なわ
//! ないため、`sim-fluid::grid_fluid`が固体境界セルを「必要になってから」導入したのと
//! 同じ判断)。分光(波長ごとのレンダリング)は後続増分(モノクロの放射輝度スカラー
//! のみを扱う)。
//!
//! `PointLight`は幾何を持たない抽象的な光源(シーンの`objects`には含まれない)とする
//! ため、BSDFサンプリングで到達したレイが誤って光源自体に「衝突」してNEEの寄与と
//! 二重計上する心配がない(設計§4の疑似コードが`sample_lights`と`environment`/BSDF
//! 再帰を単純に加算できるのは、光源が可視物体でない場合に限られる——可視な面光源
//! (エリアライト)を扱うには多重重点サンプリング(MIS)が必要になるため後続増分)。
//! 拡散(Lambertian)面のみNEEを適用する(鏡面/誘電体/金属は反射/屈折方向がデルタ
//! 関数のため光源の直接サンプルと意味を成さない、標準的な扱い)。
//!
//! `AtmosphereMedium`(参加媒質、`medium`モジュールの`HomogeneousMedium`を`Scene`へ
//! 配線したもの)は本増分で追加した。各レイセグメント(交差点まで、または環境へ
//! 抜けるまで)ごとに、平行平面近似のsecant則(空へ抜ける場合)または交差距離
//! (物体に当たる場合、aerial perspective)を光路長として、透過率(Beer-Lambert則)で
//! そのセグメントの先にある放射輝度を減衰させつつ、同じ光路上の太陽光の単一散乱を
//! 加算する。再帰呼び出し(間接光のバウンス)にも同じ処理が各セグメントの光路長で
//! 適用されるため、複数バウンスに渡る透過率の減衰は自然に積(Beer-Lambert則の
//! 加法的な光学的厚みに対応)として合成される。マルチスキャッタリング・レイ
//! マーチングによる不均質媒質は対象外(`medium`モジュールdoc参照、変わらず)。

use crate::bsdf::{CauchyDielectric, Dielectric, Lambertian, Metal, RoughConductor};
use crate::bvh::Bvh;
use crate::primitive::Primitive;
use crate::quad::Quad;
use crate::ray::Ray;
use crate::sphere::{Hit, Sphere};
use sim_math::{SimRng, Vec3};

/// このシーンが表現できるBSDF(`bsdf`モジュールdoc「縮約実装の理由」参照、
/// 粗い誘電体の透過は未実装)。
#[derive(Clone, Copy, Debug)]
pub enum Material {
    Lambertian(Lambertian),
    Dielectric(Dielectric),
    Metal(Metal),
    RoughConductor(RoughConductor),
    /// 分散を持つ誘電体(**分光レンダリング増分で追加**)。Cauchy式で波長ごとに
    /// 屈折率が変わるため、**`trace_spectral`で波長を指定したときだけ**その波長で
    /// 具体化される。波長を指定しない従来の`trace`では`REFERENCE_WAVELENGTH_NM`
    /// (視感度ピーク付近の緑)で具体化する——モノクロ経路の既存の挙動を壊さず、
    /// かつ「代表波長で見た1枚」として妥当な既定にするため。
    CauchyDielectric(CauchyDielectric),
    /// 面光源(自ら放射輝度を放つ拡散面、`Emissive`のdoc参照)。
    Emissive(Emissive),
}

/// 波長を指定しないモノクロ経路で`CauchyDielectric`を具体化するときの代表波長[nm]。
/// 明所視の視感度ピーク(555nm)付近の緑を採る——「代表波長で見た1枚」として
/// もっとも妥当で、既存のモノクロ`trace`の意味を変えないため。
pub const REFERENCE_WAVELENGTH_NM: f64 = 550.0;

/// 発光する拡散面(面光源)。R4(コーネルボックス)の天井パネルのように、
/// 幾何を持つ可視の光源を表す。
///
/// **NEE(次事象推定)を適用しない**のがこの型の設計の核である。既存の
/// `PointLight`は幾何を持たない抽象光源なので、BSDFサンプリングで到達したレイが
/// 光源自体に「衝突」して二重計上する心配が無く、NEEと再帰を単純加算できた。
/// 一方、可視な面光源に対してNEEを併用すると、BSDFサンプリングで偶然その面へ
/// 到達した経路とNEEの寄与が二重計上されるため、本来は多重重点サンプリング
/// (MIS)で重み付けする必要がある(モジュールdoc参照)。
///
/// **縮約実装の理由**: MISは実装せず、面光源については**NEEを一切行わない
/// 純粋なBSDFサンプリングのパストレース**とする。レイがこの面に当たったときに
/// `radiance`を加算するだけなので二重計上が原理的に起こらない(不偏。分散は
/// MIS併用より大きいが、R4の検証は高サンプル数の点推定なので許容できる)。
/// 既存の`PointLight`+NEE経路には一切手を入れないため、R1–R7の既存テストは
/// そのまま成立し続ける。
#[derive(Clone, Copy, Debug)]
pub struct Emissive {
    /// この面が自ら放つ放射輝度(モノクロスカラー、両面から等しく放つ)。
    pub radiance: f64,
    /// 反射成分のアルベド(Lambertian)。`0.0`なら純粋な光源(入射光を全て吸収)。
    pub albedo: f64,
}

/// シーン中の1物体(プリミティブ + BSDF)。
#[derive(Clone, Copy, Debug)]
pub struct SceneObject {
    pub primitive: Primitive,
    pub material: Material,
}

impl SceneObject {
    pub fn sphere(sphere: Sphere, material: Material) -> SceneObject {
        SceneObject {
            primitive: Primitive::Sphere(sphere),
            material,
        }
    }

    pub fn quad(quad: Quad, material: Material) -> SceneObject {
        SceneObject {
            primitive: Primitive::Quad(quad),
            material,
        }
    }
}

/// 点光源(幾何を持たない抽象光源、モジュールdoc参照)。逆二乗則の点光源として
/// 放射強度`intensity`[W/sr相当、モノクロスカラー]を持つ。
#[derive(Clone, Copy, Debug)]
pub struct PointLight {
    pub position: Vec3,
    pub intensity: f64,
}

/// シーン全体を満たす一様参加媒質(`medium`モジュールdoc「縮約実装の理由」参照——
/// 単一散乱の閉形式解のみ)+ 平行光源としての太陽。平行平面(plane-parallel)大気
/// 近似のsecant則(標準的な大気光学の近似、`up`との天頂角のコサインで光路長を
/// スケールする)で、環境へ抜けるレイ(空を見上げる)の光路長を決める——`thickness`は
/// 天頂方向(`up`)を真っ直ぐ見た場合の実効光路長(大気スケールハイトに相当)。
/// 物体に当たったレイは、単純に交差距離`hit.t`を光路長とする(aerial perspective、
/// 手前の大気による霞み)。
#[derive(Clone, Copy, Debug)]
pub struct AtmosphereMedium {
    pub medium: crate::medium::HomogeneousMedium,
    /// 太陽がある方向(カメラから見た方向、正規化済みを想定)。
    pub sun_direction: Vec3,
    pub sun_radiance: f64,
    /// 天頂方向(secant近似の基準軸、正規化済みを想定)。
    pub up: Vec3,
    pub thickness: f64,
}

impl AtmosphereMedium {
    /// 天頂角のコサインに基づくsecant近似の光路長(モジュールdoc参照)。天頂
    /// 方向ぎりぎり(地平線方向、コサインが0に近い)で光路長が発散しないよう、
    /// コサインの絶対値を`MIN_COS_ZENITH`未満にはクランプする。
    fn sky_path_length(&self, direction: Vec3) -> f64 {
        const MIN_COS_ZENITH: f64 = 0.02;
        self.thickness / direction.dot(self.up).abs().max(MIN_COS_ZENITH)
    }

    /// 大気越しの放射輝度合成: 経路長`path_length`ぶんの透過率で`beyond`
    /// (物体表面/環境から届く放射輝度)を減衰させ、同じ経路上での太陽光の単一散乱
    /// (`medium`モジュールdoc参照)を加算する。
    fn composite(&self, beyond: f64, path_length: f64, view_direction: Vec3) -> f64 {
        let cos_theta = view_direction.dot(self.sun_direction);
        let transmittance = self.medium.transmittance(path_length);
        let in_scattered =
            self.medium
                .single_scattering_radiance(path_length, cos_theta, self.sun_radiance);
        beyond * transmittance + in_scattered
    }
}

/// 複数のプリミティブ + BSDF + 一様環境放射輝度 + 点光源からなるシーン。
///
/// **構築は必ず`Scene::new`を使う**(フィールドを直接書ける構造体リテラルでは
/// なくコンストラクタにしているのは、`objects`から一度だけBVHを構築して保持する
/// ため——構築後に`objects`を差し替えるとBVHが陳腐化するので、`objects`は
/// 非公開にして読み取り専用の`objects()`だけを公開する)。
pub struct Scene {
    objects: Vec<SceneObject>,
    pub lights: Vec<PointLight>,
    /// 環境放射輝度(方向によらず一定、モノクロスカラー、設計§5「大気散乱等は後続」)。
    pub environment_radiance: f64,
    /// 参加媒質(`None`なら真空、既存の全テストの挙動を完全に保つ)。
    pub medium: Option<AtmosphereMedium>,
    /// `objects`に対する加速構造(`bvh`モジュールdoc参照)。`new`で一度だけ構築する。
    bvh: Bvh,
    /// `objects`と同じ順序のプリミティブ配列。`Bvh::closest_hit`がスライスを要求
    /// するため、レイごとに作り直さずに済むよう`new`で一度だけ作って保持する
    /// (レイ1本ごとに`Vec`を確保するとBVH導入の目的である高速化が帳消しになる)。
    primitives: Vec<Primitive>,
}

impl Scene {
    /// シーンを構築し、あわせて`objects`のBVHを一度だけ構築する(型のdoc参照)。
    pub fn new(
        objects: Vec<SceneObject>,
        lights: Vec<PointLight>,
        environment_radiance: f64,
        medium: Option<AtmosphereMedium>,
    ) -> Scene {
        let primitives: Vec<Primitive> = objects.iter().map(|o| o.primitive).collect();
        let bvh = Bvh::build(&primitives);
        Scene {
            objects,
            lights,
            environment_radiance,
            medium,
            bvh,
            primitives,
        }
    }

    /// 読み取り専用のシーン物体一覧(型のdoc参照 — BVHとの整合を保つため
    /// 構築後の差し替えは許さない)。
    pub fn objects(&self) -> &[SceneObject] {
        &self.objects
    }

    /// BVHで最近傍交差(`t`最小)を返す。`pub(crate)`はテストから直接この幾何選択
    /// ロジックだけを検証するため(`trace`経由だと再帰的なBSDFサンプリングの結果と
    /// 混ざり、どちらの物体が選ばれたか単体では判別しづらい)。
    pub(crate) fn closest_hit(&self, ray: &Ray, t_min: f64) -> Option<(Hit, &Material)> {
        // `Bvh`はindexだけを返すので、`objects`と同じ順序である前提で引き当てる
        // (`new`が`objects`の順序そのままで`primitives`を作るため成立する)。
        self.bvh
            .closest_hit(&self.primitives, ray, t_min)
            .map(|(index, hit)| (hit, &self.objects[index].material))
    }

    /// NEE(Next Event Estimation、設計§4「直接光: 光源を明示サンプル」)。全ての
    /// 点光源について、シャドウレイで遮蔽を確認した上で逆二乗則の直接照明
    /// $L_o \mathrel{+}= f_r \cdot (I/d^2) \cdot \cos\theta$を合算する
    /// (モジュールdoc「光源は幾何を持たない」参照、遮蔽判定のみ`closest_hit`を使う)。
    /// `pub(crate)`はテストから直接呼ぶため(`trace`経由だと、複数物体シーンでは
    /// 間接項(BSDFサンプリングの再帰)が他の物体に当たり得るため、直接照明単体の
    /// 値と混ざってしまう——`closest_hit`のテストで踏んだのと同じ落とし穴)。
    pub(crate) fn direct_lighting(&self, point: Vec3, normal: Vec3, bsdf: f64) -> f64 {
        let mut total = 0.0;
        for light in &self.lights {
            let to_light = light.position - point;
            let distance = to_light.length();
            if distance < 1e-9 {
                continue;
            }
            let direction = to_light.scale(1.0 / distance);
            let cos_theta = direction.dot(normal);
            if cos_theta <= 0.0 {
                continue; // 光源が面の裏側。
            }
            let shadow_ray = Ray::new(point, direction);
            if let Some((hit, _)) = self.closest_hit(&shadow_ray, 1e-6) {
                if hit.t < distance - 1e-6 {
                    continue; // 光源との間に別の物体があり遮蔽される。
                }
            }
            let irradiance = light.intensity / (distance * distance);
            total += bsdf * irradiance * cos_theta;
        }
        total
    }

    /// レイを追跡し、方向によらず一様な環境からの放射輝度を経路積分で推定する
    /// (設計§4の`trace`のうち、本増分が実装する範囲——拡散/誘電体BSDFの再帰的
    /// サンプリングのみ——を抜き出したもの)。`max_depth`はロシアンルーレット無しの
    /// 単純打切り(設計§9の値より小さくてよい——白色炉テストは1回の追加バウンスで
    /// 解析値に一致するため、`max_depth`を大きくしても結果は変わらない、モジュール
    /// doc参照)。
    /// モノクロ経路(既存)。`CauchyDielectric`は代表波長で具体化される。
    pub fn trace(&self, ray: &Ray, rng: &mut SimRng, max_depth: u32) -> f64 {
        self.trace_at_wavelength(ray, rng, max_depth, None)
    }

    /// **分光経路(分光レンダリング増分で追加)**。`wavelength_nm`をシーン中の
    /// 分散を持つ材質(`Material::CauchyDielectric`)へ伝え、その波長での屈折率で
    /// 経路を追う。返すのはその波長における分光放射輝度。
    ///
    /// これが無かったあいだ、分散は`prism.rs`が波長ごとに`Dielectric`を手で
    /// 具体化して個別に呼ぶ形でしか検証できず、**パストレの経路には一度も
    /// 波長が入っていなかった**。
    pub fn trace_spectral(
        &self,
        ray: &Ray,
        rng: &mut SimRng,
        max_depth: u32,
        wavelength_nm: f64,
    ) -> f64 {
        self.trace_at_wavelength(ray, rng, max_depth, Some(wavelength_nm))
    }

    fn trace_at_wavelength(
        &self,
        ray: &Ray,
        rng: &mut SimRng,
        max_depth: u32,
        wavelength_nm: Option<f64>,
    ) -> f64 {
        let Some((hit, material)) = self.closest_hit(ray, 1e-6) else {
            return match &self.medium {
                None => self.environment_radiance,
                // 空へ抜けるレイ(モジュールdoc「AtmosphereMedium」参照): 平行平面近似の
                // secant則で光路長を決め、環境放射輝度を大気越しの透過率で減衰させつつ、
                // その経路上での太陽光の単一散乱(空の色)を加算する。
                Some(atmosphere) => {
                    let path_length = atmosphere.sky_path_length(ray.direction);
                    atmosphere.composite(self.environment_radiance, path_length, ray.direction)
                }
            };
        };
        if max_depth == 0 {
            return 0.0; // 打ち切り(エネルギーを捨てる、通常のロシアンルーレット無し打切りと同じ)。
        }

        // **分散を持つ材質は、追跡中の波長で単色の誘電体へ具体化してから
        // 通常の分岐に入る**(`Dielectric`のreflect/refract/reflectanceは屈折率
        // だけに依存するので、これだけで既存の誘電体経路をそのまま再利用できる)。
        let specialized;
        let material = match material {
            Material::CauchyDielectric(cauchy) => {
                let lambda = wavelength_nm.unwrap_or(REFERENCE_WAVELENGTH_NM);
                specialized = Material::Dielectric(cauchy.to_dielectric_at(lambda));
                &specialized
            }
            other => other,
        };

        let surface_radiance = match material {
            // 直前で単色の`Dielectric`へ具体化済みなので、ここには到達しない。
            Material::CauchyDielectric(_) => unreachable!("上で具体化済み"),
            Material::Lambertian(lambertian) => {
                let direct = self.direct_lighting(hit.point, hit.normal, lambertian.eval());

                let (direction, pdf) = lambertian.sample(hit.normal, rng);
                let cos_theta = direction.dot(hit.normal);
                let bsdf = lambertian.eval();
                let next_ray = Ray::new(hit.point, direction);
                let indirect =
                    self.trace_at_wavelength(&next_ray, rng, max_depth - 1, wavelength_nm)
                        * bsdf
                        * cos_theta
                        / pdf;

                direct + indirect
            }
            Material::Dielectric(dielectric) => {
                // オリエンテーション補正: 外向き法線に対してレイが向かってくる側
                // (d・n<0)なら「入射」(真空→本材質)、逆なら球の内部から「出射」
                // (本材質→真空)。法線を出射側基準に反転し、n1/n2を入れ替える。
                let entering = ray.direction.dot(hit.normal) < 0.0;
                let (outward_normal, n1, n2) = if entering {
                    (hit.normal, 1.0, dielectric.ior)
                } else {
                    (-hit.normal, dielectric.ior, 1.0)
                };
                let eta = n1 / n2;
                let cos_theta_i = -ray.direction.dot(outward_normal);
                let refracted = Dielectric::refract(ray.direction, outward_normal, eta);
                let reflectance = match refracted {
                    None => 1.0, // 全反射(TIR)。
                    Some(_) => Dielectric::reflectance_between(cos_theta_i, n1, n2),
                };
                let is_reflected = rng.next_f64() < reflectance;
                match refracted {
                    Some(direction) if !is_reflected => {
                        let next_ray = Ray::new(hit.point, direction);
                        // 放射輝度は界面通過ごとに(n1/n2)^2でスケールする(幾何光学の
                        // 放射輝度不変量 L/n^2 = const、標準的なBTDFの因子)。ガラス球を
                        // 1回貫通する経路では入射時のeta=1/iorと出射時のeta=iorの二乗が
                        // 厳密に相殺する(1/ior^2 * ior^2 = 1)ため、この因子を含めても
                        // 白色炉テストの厳密一致(誤差ゼロ)が保たれる。
                        self.trace_at_wavelength(&next_ray, rng, max_depth - 1, wavelength_nm)
                            * eta
                            * eta
                    }
                    _ => {
                        let direction = Dielectric::reflect(ray.direction, outward_normal);
                        let next_ray = Ray::new(hit.point, direction);
                        self.trace_at_wavelength(&next_ray, rng, max_depth - 1, wavelength_nm)
                    }
                }
            }
            Material::Metal(metal) => {
                // 金属は不透明(透過が無い)ため、誘電体のような反射/透過の確率的
                // 分岐は不要——鏡面反射方向1つだけを追跡し、フレネル反射率で振幅を
                // スケールする(吸収された分(1-R)は失われる、`bsdf.rs`モジュールdoc
                // 参照)。
                let cos_theta_i = -ray.direction.dot(hit.normal);
                let r = metal.reflectance(cos_theta_i);
                let direction = Dielectric::reflect(ray.direction, hit.normal);
                let next_ray = Ray::new(hit.point, direction);
                self.trace_at_wavelength(&next_ray, rng, max_depth - 1, wavelength_nm) * r
            }
            Material::Emissive(emissive) => {
                // 自己発光分をそのまま加算する(NEEは行わない、`Emissive`のdoc参照)。
                // 反射成分はLambertianと同じ扱い——コサイン重み付き半球サンプリングと
                // 対にすると`bsdf*cosθ/pdf`が`albedo`に恒等的に一致する(R1と同じ
                // 完全な相殺)。
                let mut radiance = emissive.radiance;
                if emissive.albedo > 0.0 {
                    let lambertian = Lambertian {
                        albedo: emissive.albedo,
                    };
                    let (direction, pdf) = lambertian.sample(hit.normal, rng);
                    let cos_theta = direction.dot(hit.normal);
                    let bsdf = lambertian.eval();
                    let next_ray = Ray::new(hit.point, direction);
                    radiance +=
                        self.trace_at_wavelength(&next_ray, rng, max_depth - 1, wavelength_nm)
                            * bsdf
                            * cos_theta
                            / pdf;
                }
                radiance
            }
            Material::RoughConductor(rough) => {
                let (direction, weight) = rough.sample(ray.direction, hit.normal, rng);
                if weight <= 0.0 {
                    // 出射方向がマクロ表面の裏側、寄与なし。この早期returnは関数全体を
                    // 抜けるため、大気媒質が設定されていてもこのセグメント分の
                    // 大気越しの単一散乱(空の色の混入)は加算されない(既知の限定、
                    // 縮約実装として許容——このサンプリング失敗自体が稀なケース)。
                    return 0.0;
                }
                let next_ray = Ray::new(hit.point, direction);
                self.trace_at_wavelength(&next_ray, rng, max_depth - 1, wavelength_nm) * weight
            }
        };

        match &self.medium {
            None => surface_radiance,
            // 物体に当たったレイ(モジュールdoc参照): 交差距離`hit.t`をそのまま
            // 大気の光路長とする(aerial perspective、手前の大気による霞み)。
            Some(atmosphere) => atmosphere.composite(surface_radiance, hit.t, ray.direction),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::medium::HomogeneousMedium;

    /// R1: 白色炉テスト(docs/21-verification/01-analytic-tests.md、
    /// docs/17-rendering/02-path-tracing.md §7)。完全拡散面(ρ=1)が一様環境放射輝度の
    /// 中に置かれると、その表面から見た放射輝度は環境放射輝度と厳密に一致する
    /// (エネルギー保存・BSDF正規化の検証)。
    ///
    /// 解析的根拠: Lambertian BSDFはコサイン重み付き半球サンプリング
    /// (pdf=cosθ/π)と対にすると、`bsdf*cosθ/pdf = albedo`が方向によらず恒等的に
    /// 成り立つ(重要度サンプリングの完全な相殺)。球が孤立している(凸形状は自身を
    /// 決して自己遮蔽しない)ため、サンプルされた方向は必ず環境へ抜ける。したがって
    /// albedo=1のときは追加のバウンス1回で分散ゼロのまま解析値と厳密に一致する
    /// (統計的収束を待つ必要がない——ここが「白色炉」の名の由来である「エネルギーが
    /// 一切失われない」ことの直接的な帰結)。
    #[test]
    fn r1_white_furnace_diffuse_surface_matches_background_radiance_exactly() {
        let environment_radiance = 3.7; // 任意の一様環境放射輝度。
        let scene = Scene::new(
            vec![SceneObject::sphere(
                Sphere {
                    center: Vec3::new(0.0, 0.0, -5.0),
                    radius: 1.0,
                },
                Material::Lambertian(Lambertian { albedo: 1.0 }),
            )],
            vec![],
            environment_radiance,
            None,
        );
        let camera_origin = Vec3::ZERO;
        let mut rng = SimRng::new(1, 1);

        for i in 0..20 {
            // 球面上の様々な点を狙う(法線方向が毎回異なることを確認する)複数レイ。
            let angle = i as f64 * 0.05;
            let target = Vec3::new(angle.sin() * 0.8, angle.cos() * 0.8, -5.0);
            let ray = Ray::new(camera_origin, target - camera_origin);
            let radiance = scene.trace(&ray, &mut rng, 4);
            let rel_err = (radiance - environment_radiance).abs() / environment_radiance;
            assert!(
                rel_err < 0.001,
                "R1 white furnace failed at ray {i}: radiance={radiance} \
                 environment_radiance={environment_radiance} rel_err={rel_err}"
            );
        }
    }

    /// アルベドが1未満の場合、放射輝度は`albedo*environment_radiance`(エネルギー保存、
    /// 白色炉テストの一般形)に厳密に一致する。
    #[test]
    fn sub_unity_albedo_scales_radiance_by_albedo_exactly() {
        let environment_radiance = 2.0;
        let albedo = 0.6;
        let scene = Scene::new(
            vec![SceneObject::sphere(
                Sphere {
                    center: Vec3::new(0.0, 0.0, -5.0),
                    radius: 1.0,
                },
                Material::Lambertian(Lambertian { albedo }),
            )],
            vec![],
            environment_radiance,
            None,
        );
        let ray = Ray::new(Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0));
        let mut rng = SimRng::new(2, 2);
        let radiance = scene.trace(&ray, &mut rng, 4);
        let expected = albedo * environment_radiance;
        let rel_err = (radiance - expected).abs() / expected;
        assert!(rel_err < 1e-9, "radiance={radiance} expected={expected}");
    }

    /// レイが球を外れた場合は環境放射輝度そのものを返す。
    #[test]
    fn ray_missing_the_sphere_returns_environment_radiance() {
        let scene = Scene::new(
            vec![SceneObject::sphere(
                Sphere {
                    center: Vec3::new(0.0, 0.0, -5.0),
                    radius: 1.0,
                },
                Material::Lambertian(Lambertian { albedo: 1.0 }),
            )],
            vec![],
            9.0,
            None,
        );
        let ray = Ray::new(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0));
        let mut rng = SimRng::new(3, 3);
        assert_eq!(scene.trace(&ray, &mut rng, 4), 9.0);
    }

    /// 白色炉テスト(R1)の誘電体版: 吸収の無い誘電体球(ガラス相当)が一様環境放射
    /// 輝度の中に置かれると、反射・屈折のどちらの経路をたどっても最終的に環境へ
    /// 抜けるため、放射輝度は環境放射輝度と厳密に一致する(統計誤差ゼロ)。
    ///
    /// 解析的根拠: 反射/屈折は物理的な確率(フレネル反射率R/透過率1-R)に比例した
    /// 確率でサンプリングされ、選ばれた経路の重みは追加のR/(1-R)による除算を
    /// 伴わない(サンプリング確率と物理確率が一致するため相殺する、Lambertianの
    /// bsdf/pdf相殺と同じ構造)。屈折時の放射輝度スケール因子$(n_1/n_2)^2$は、
    /// 球に入る際($1/\text{ior}$)と出る際($\text{ior}$)とで厳密に相殺する
    /// (モジュールdoc参照)。臨界角を超えるグレージング角は本テストでは避ける
    /// (全反射で球内部に閉じ込められ`max_depth`で打ち切られるため、厳密一致が
    /// 崩れる既知の限界、後続増分でロシアンルーレット等の対応を検討)。
    #[test]
    fn dielectric_furnace_test_non_absorbing_glass_sphere_matches_background_radiance_exactly() {
        let environment_radiance = 4.2;
        let scene = Scene::new(
            vec![SceneObject::sphere(
                Sphere {
                    center: Vec3::new(0.0, 0.0, -5.0),
                    radius: 1.0,
                },
                Material::Dielectric(Dielectric { ior: 1.5 }),
            )],
            vec![],
            environment_radiance,
            None,
        );
        let camera_origin = Vec3::ZERO;
        let mut rng = SimRng::new(4, 4);

        for i in 0..30 {
            // 臨界角(≈41.8°)より十分小さい角度に留め、全反射による無限内部反射を
            // 避ける(モジュールdoc「後続増分で対応を検討」参照)。
            let angle = i as f64 * 0.02;
            let target = Vec3::new(angle.sin() * 0.5, angle.cos() * 0.5, -5.0);
            let ray = Ray::new(camera_origin, target - camera_origin);
            let radiance = scene.trace(&ray, &mut rng, 8);
            let rel_err = (radiance - environment_radiance).abs() / environment_radiance;
            assert!(
                rel_err < 1e-6,
                "dielectric furnace test failed at ray {i}: radiance={radiance} \
                 environment_radiance={environment_radiance} rel_err={rel_err}"
            );
        }
    }

    /// 金属球(不透明・完全鏡面)を一様環境放射輝度の中に置くと、放射輝度は
    /// フレネル反射率でスケールされた環境放射輝度に厳密に一致する(白色炉テストの
    /// 金属版——誘電体と異なり反射/透過の確率的分岐が無い単一経路のため、統計誤差
    /// ゼロで解析値に一致する、`sub_unity_albedo_scales_radiance_by_albedo_exactly`
    /// と同種の判断)。
    #[test]
    fn metal_furnace_test_matches_fresnel_scaled_background_radiance_exactly() {
        let environment_radiance = 5.0;
        let gold = Metal { n: 0.47, k: 2.4 };
        let scene = Scene::new(
            vec![SceneObject::sphere(
                Sphere {
                    center: Vec3::new(0.0, 0.0, -5.0),
                    radius: 1.0,
                },
                Material::Metal(gold),
            )],
            vec![],
            environment_radiance,
            None,
        );
        let ray = Ray::new(Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0)); // 垂直入射。
        let mut rng = SimRng::new(7, 7);
        let radiance = scene.trace(&ray, &mut rng, 4);
        let expected = gold.reflectance(1.0) * environment_radiance;
        let rel_err = (radiance - expected).abs() / expected;
        assert!(rel_err < 1e-9, "radiance={radiance} expected={expected}");
    }

    /// 複数物体: 2つの球が同一レイ上に重なる場合、`closest_hit`が正しく手前の
    /// (`t`が小さい)物体を選ぶこと。`trace`経由だとBSDFの再帰的サンプリングの結果
    /// (どちらの物体に当たっても最終的にほぼ同じ放射輝度に収束し得る、特に両方が
    /// 拡散白色だと区別できない)と混ざってしまうため、`closest_hit`を直接検証する
    /// (線形探索による最近傍交差選択そのものの配線確認、モジュールdoc参照)。
    #[test]
    fn closest_hit_picks_the_nearer_object_when_two_spheres_overlap_along_the_ray() {
        let near = SceneObject::sphere(
            Sphere {
                center: Vec3::new(0.0, 0.0, -3.0),
                radius: 1.0,
            },
            Material::Lambertian(Lambertian { albedo: 0.2 }),
        );
        let far = SceneObject::sphere(
            Sphere {
                center: Vec3::new(0.0, 0.0, -8.0),
                radius: 1.0,
            },
            Material::Lambertian(Lambertian { albedo: 0.9 }),
        );
        let scene = Scene::new(vec![near, far], vec![], 1.0, None);

        // 両方の球の中心を貫く一直線: 手前の球(t≈2)が奥の球(t≈7)より先にヒットする。
        let ray = Ray::new(Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0));
        let (hit, material) = scene
            .closest_hit(&ray, 1e-6)
            .expect("should hit the near sphere");
        assert!(
            (hit.t - 2.0).abs() < 1e-9,
            "expected to hit the near sphere at t=2.0, got t={}",
            hit.t
        );
        match material {
            Material::Lambertian(lambertian) => {
                assert_eq!(
                    lambertian.albedo, 0.2,
                    "should pick the near sphere's material"
                )
            }
            other => panic!("expected Lambertian, got {other:?}"),
        }

        // 物体を除いた登録順で試しても(奥→手前)、結果は変わらず手前が選ばれる
        // (線形探索は登録順に依存しない`t`最小選択であることの確認)。
        let scene_reordered = Scene::new(vec![far, near], vec![], 1.0, None);
        let (hit_reordered, _) = scene_reordered
            .closest_hit(&ray, 1e-6)
            .expect("should still hit the near sphere");
        assert!((hit_reordered.t - 2.0).abs() < 1e-9);

        // レイが両方の球を外れる場合は`None`。
        let missing_ray = Ray::new(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0));
        assert!(scene.closest_hit(&missing_ray, 1e-6).is_none());
    }

    /// NEE: 点光源からの直接照明が解析式$f_r \cdot (I/d^2) \cdot \cos\theta$に厳密に
    /// 一致すること(遮蔽物が無い場合)。
    #[test]
    fn direct_lighting_matches_the_inverse_square_point_light_formula() {
        let light = PointLight {
            position: Vec3::new(0.0, 0.0, -1.0),
            intensity: 12.0,
        };
        let scene = Scene::new(
            vec![SceneObject::sphere(
                Sphere {
                    center: Vec3::new(0.0, 0.0, -5.0),
                    radius: 1.0,
                },
                Material::Lambertian(Lambertian { albedo: 0.8 }),
            )],
            vec![light],
            0.0,
            None,
        );
        let point = Vec3::new(0.0, 0.0, -4.0); // 球の手前(原点側)の頂点。
        let normal = Vec3::new(0.0, 0.0, 1.0); // 外向き法線(光源の方向と一致)。
        let bsdf = 0.8 / std::f64::consts::PI;

        let measured = scene.direct_lighting(point, normal, bsdf);
        let distance = (light.position - point).length();
        let cos_theta = 1.0; // 光源は法線の真正面。
        let expected = bsdf * (light.intensity / (distance * distance)) * cos_theta;
        let rel_err = (measured - expected).abs() / expected;
        assert!(rel_err < 1e-9, "measured={measured} expected={expected}");
    }

    /// NEE: 光源と着目点の間に別の物体があると、直接照明への寄与が0になる
    /// (シャドウレイによる遮蔽判定)。
    #[test]
    fn direct_lighting_is_zero_when_the_light_is_occluded() {
        let light = PointLight {
            position: Vec3::new(0.0, 0.0, -1.0),
            intensity: 12.0,
        };
        let occluder = SceneObject::sphere(
            Sphere {
                center: Vec3::new(0.0, 0.0, -2.5),
                radius: 0.3,
            },
            Material::Lambertian(Lambertian { albedo: 0.5 }),
        );
        let main_sphere = SceneObject::sphere(
            Sphere {
                center: Vec3::new(0.0, 0.0, -5.0),
                radius: 1.0,
            },
            Material::Lambertian(Lambertian { albedo: 0.8 }),
        );
        let scene = Scene::new(vec![occluder, main_sphere], vec![light], 0.0, None);
        let point = Vec3::new(0.0, 0.0, -4.0);
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let bsdf = 0.8 / std::f64::consts::PI;

        let measured = scene.direct_lighting(point, normal, bsdf);
        assert_eq!(
            measured, 0.0,
            "the occluder sitting between the point and the light should block all direct \
             lighting"
        );
    }

    /// NEE: 光源が面の裏側(法線との内積が0以下)にある場合は寄与しない。
    #[test]
    fn direct_lighting_ignores_lights_behind_the_surface() {
        let light = PointLight {
            position: Vec3::new(0.0, 0.0, -6.0), // 法線の反対側。
            intensity: 12.0,
        };
        let scene = Scene::new(
            vec![SceneObject::sphere(
                Sphere {
                    center: Vec3::new(0.0, 0.0, -5.0),
                    radius: 1.0,
                },
                Material::Lambertian(Lambertian { albedo: 0.8 }),
            )],
            vec![light],
            0.0,
            None,
        );
        let point = Vec3::new(0.0, 0.0, -4.0);
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let bsdf = 0.8 / std::f64::consts::PI;

        assert_eq!(scene.direct_lighting(point, normal, bsdf), 0.0);
    }

    /// `trace`自体がNEEを正しく配線していること: 環境放射輝度0・孤立した球1個の
    /// シーンでは、間接項(BSDFサンプリングの再帰)は常に環境(0)へ抜けるため
    /// (凸形状の自己遮蔽なし)、`trace`の結果は`direct_lighting`単体の値と厳密に
    /// 一致するはず。
    #[test]
    fn trace_wires_direct_lighting_in_for_an_isolated_sphere_with_no_environment() {
        let light = PointLight {
            position: Vec3::new(0.0, 0.0, -1.0),
            intensity: 12.0,
        };
        let albedo = 0.8;
        let scene = Scene::new(
            vec![SceneObject::sphere(
                Sphere {
                    center: Vec3::new(0.0, 0.0, -5.0),
                    radius: 1.0,
                },
                Material::Lambertian(Lambertian { albedo }),
            )],
            vec![light],
            0.0,
            None,
        );
        let point = Vec3::new(0.0, 0.0, -4.0);
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let bsdf = albedo / std::f64::consts::PI;
        let expected = scene.direct_lighting(point, normal, bsdf);

        let ray = Ray::new(Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0));
        let mut rng = SimRng::new(6, 6);
        let measured = scene.trace(&ray, &mut rng, 4);
        let rel_err = (measured - expected).abs() / expected;
        assert!(rel_err < 1e-9, "measured={measured} expected={expected}");
    }

    /// R7の検証シーン: 主球(Lambertian)の頂点から見て、法線方向(コサイン重み付き
    /// サンプリングが集中する向き)の先に部分的な遮蔽球を置く。遮蔽球に当たった経路は
    /// `max_depth`打ち切り(0を返す)、外れた経路は環境放射輝度に albedo を掛けた値
    /// (Lambertianのbsdf*cosθ/pdf=albedo恒等式、R1参照)を返すため、`trace`の
    /// 結果自体がベルヌーイ的な二値混合になり、真に分散を持つモンテカルロ推定量に
    /// なる(白色炉テスト系がわざと分散ゼロにしているのとは対照的に、ここでは意図的に
    /// 分散を持たせて収束率そのものを検証する)。
    fn r7_variance_test_scene() -> (Scene, Ray) {
        let main_sphere = SceneObject::sphere(
            Sphere {
                center: Vec3::new(0.0, 0.0, -5.0),
                radius: 1.0,
            },
            Material::Lambertian(Lambertian { albedo: 0.9 }),
        );
        // 主球頂点(0,0,-4)から法線(0,0,1)方向(カメラを越えてさらに奥、z>0側)に
        // 距離6・半径1.8の遮蔽球を置く(半頂角≈17.5°の円錐を部分的に遮蔽)。
        // カメラ(原点)からこの遮蔽球までの距離は2 > 半径1.8なので、カメラ自身は
        // 遮蔽球の外側にあり、かつ主レイ(-z方向)の経路(z<=0)は遮蔽球
        // (z∈[0.2, 3.8])に一切触れない——カメラから見て遮蔽球は真後ろ(+z)に
        // あるため、主レイは必ず主球に先に当たる。主球で拡散反射した間接レイ
        // (法線(0,0,1)まわりのコサイン重み付き、cosθ>0すなわちz成分が正)だけが
        // z=-4からz>0方向へ進み、この遮蔽球に当たり得る(主球とは7>1+1.8で
        // 重ならない)。
        let occluder = SceneObject::sphere(
            Sphere {
                center: Vec3::new(0.0, 0.0, 2.0),
                radius: 1.8,
            },
            Material::Lambertian(Lambertian { albedo: 0.5 }),
        );
        let scene = Scene::new(vec![main_sphere, occluder], vec![], 6.0, None);
        let ray = Ray::new(Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0));
        (scene, ray)
    }

    fn sample_variance(data: &[f64]) -> f64 {
        let n = data.len() as f64;
        let mean = data.iter().sum::<f64>() / n;
        data.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (n - 1.0)
    }

    /// 決定論: 同一シード・同一サンプル数なら、平均放射輝度は厳密に同一の値になる
    /// (設計§7「決定論(同一シード同一画像)」、`SimRng`が完全に決定的なため各サンプルの
    /// 系列も再現される)。
    #[test]
    fn average_radiance_is_deterministic_given_the_same_seed_and_sample_count() {
        let (scene, ray) = r7_variance_test_scene();
        let average = |seed: u64, spp: u64| -> f64 {
            let sum: f64 = (0..spp)
                .map(|i| {
                    let mut rng = SimRng::new(seed, i);
                    scene.trace(&ray, &mut rng, 1)
                })
                .sum();
            sum / spp as f64
        };
        let a = average(42, 500);
        let b = average(42, 500);
        assert_eq!(
            a, b,
            "identical seed and sample count must reproduce the exact same average"
        );
    }

    /// R7: モンテカルロ推定のノイズ(平均のばらつき)はサンプル数Nに対してO(1/√N)で
    /// 減少する。上記の意図的に分散を持つシーンから20000個の独立サンプル(サブ
    /// ストリームごとに1個)を1回だけ引き、それをバッチサイズ100(200バッチ)と
    /// バッチサイズ400(50バッチ)に分割してバッチ平均の分散を比較する: バッチ
    /// サイズを4倍にすると、各バッチ平均の分散は理論上ちょうど1/4になる
    /// (Var(平均)=Var(1サンプル)/N)ため、標準偏差(ノイズ)は1/2、つまり1/√4に
    /// 減少する。
    #[test]
    fn r7_monte_carlo_noise_decreases_as_the_inverse_square_root_of_sample_count() {
        let (scene, ray) = r7_variance_test_scene();
        const TOTAL_SAMPLES: u64 = 20_000;
        let samples: Vec<f64> = (0..TOTAL_SAMPLES)
            .map(|i| {
                let mut rng = SimRng::new(99, i);
                scene.trace(&ray, &mut rng, 1)
            })
            .collect();

        // 個々のサンプルが実際に(遮蔽/非遮蔽の)両方の値を取ること、つまり本当に
        // 分散を持つシーンであることをまず確認する(そうでなければ収束率の検証自体が
        // 意味を持たない)。
        let min = samples.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = samples.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(
            max - min > 1.0,
            "the test scene must actually produce variance across samples: min={min} max={max}"
        );

        let batch_means = |batch_size: usize| -> Vec<f64> {
            samples
                .chunks_exact(batch_size)
                .map(|chunk| chunk.iter().sum::<f64>() / batch_size as f64)
                .collect()
        };

        let means_100 = batch_means(100);
        let means_400 = batch_means(400);
        let var_100 = sample_variance(&means_100);
        let var_400 = sample_variance(&means_400);

        // 理論比: Var(batch=100)/Var(batch=400) = 400/100 = 4。有限バッチ数
        // (200・50バッチ)によるサンプリング誤差を見込んで広めの許容範囲を取る。
        // 実測値(固定シードのため決定論的に同一の値になる): ratio≈4.16
        // (var_100≈0.0224, var_400≈0.00539)、理論値4に近い。
        let ratio = var_100 / var_400;
        assert!(
            (3.0..5.5).contains(&ratio),
            "batch-mean variance ratio should be close to 4 (O(1/N) variance decay, \
             i.e. O(1/sqrt(N)) noise decay): ratio={ratio} var_100={var_100} var_400={var_400}"
        );
    }

    fn rough_conductor_furnace_scene(alpha: f64, environment_radiance: f64) -> (Scene, Ray) {
        let gold = RoughConductor {
            n: 0.47,
            k: 2.4,
            alpha,
        };
        let scene = Scene::new(
            vec![SceneObject::sphere(
                Sphere {
                    center: Vec3::new(0.0, 0.0, -5.0),
                    radius: 1.0,
                },
                Material::RoughConductor(gold),
            )],
            vec![],
            environment_radiance,
            None,
        );
        let ray = Ray::new(Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0));
        (scene, ray)
    }

    fn average_trace(scene: &Scene, ray: &Ray, seed: u64, n: u64) -> f64 {
        let sum: f64 = (0..n)
            .map(|i| {
                let mut rng = SimRng::new(seed, i);
                scene.trace(ray, &mut rng, 1)
            })
            .sum();
        sum / n as f64
    }

    /// GGX粗い金属は、粗さ(alpha)を0に近づけると完全鏡面`Metal`の白色炉テスト
    /// (フレネル反射率でスケールされた環境放射輝度)にモンテカルロ収束する
    /// (`RoughConductor::sample`の重み式(Walter et al. 2007)が正しく実装されて
    /// いることの統合的な検証——白色炉系の他テストと異なり、ここではalpha>0が
    /// 本質的な統計誤差を導入するため、統計的収束を待つ必要がある。実測値
    /// (alpha=0.02, N=50000, 固定シード)でrel_err≈0.037%だったため、10倍以上の
    /// 余裕を見てrel<1%を要求する)。
    #[test]
    fn rough_conductor_converges_to_the_smooth_mirror_furnace_test_as_roughness_approaches_zero() {
        let environment_radiance = 5.0;
        let (scene, ray) = rough_conductor_furnace_scene(0.02, environment_radiance);
        let mean = average_trace(&scene, &ray, 21, 50_000);
        let expected = Metal { n: 0.47, k: 2.4 }.reflectance(1.0) * environment_radiance;
        let rel_err = (mean - expected).abs() / expected;
        assert!(
            rel_err < 0.01,
            "mean={mean} expected={expected} rel_err={rel_err}"
        );
    }

    /// エネルギー保存: 粗さが大きくても(単一散乱モデルではマルチスキャッタリングを
    /// 拾えない分だけ、むしろ滑らかな鏡面より輝度を過小評価することはあっても)
    /// 平均放射輝度が環境放射輝度を超えてエネルギーを増幅することは無い
    /// (`microfacet`モジュールdoc「マルチスキャッタリング補償は対象外」参照——
    /// この既知の制限により、ここでは「超過しないこと」のみを検証し、滑らかな
    /// 鏡面の値と厳密一致することは要求しない)。
    #[test]
    fn rough_conductor_never_exceeds_energy_conservation_bound_for_moderate_roughness() {
        let environment_radiance = 5.0;
        for alpha in [0.3, 0.6] {
            let (scene, ray) = rough_conductor_furnace_scene(alpha, environment_radiance);
            let mean = average_trace(&scene, &ray, 21, 100_000);
            assert!(
                mean < environment_radiance,
                "alpha={alpha}: mean radiance must not exceed the environment radiance \
                 (no energy amplification): mean={mean} environment_radiance={environment_radiance}"
            );
            assert!(
                mean > 0.0,
                "alpha={alpha}: mean radiance must be positive: mean={mean}"
            );
        }
    }

    /// D42(空と大気)の土台: `medium`モジュールの`HomogeneousMedium`(R5/R6で純粋関数
    /// として既に検証済み)を`Scene::trace`へ実際に配線する(モジュールdoc
    /// 「AtmosphereMedium」参照——それまでは`Scene`に参加媒質を持つ手段自体が
    /// 無かった)。オブジェクトの無いシーンでは環境へ抜けるレイが必ず`self.medium`の
    /// miss分岐を通り、確率的なBSDFサンプリングが絡まないため、secant則による
    /// 光路長・透過率・単一散乱の合成値を解析的に厳密一致で検証できる。
    #[test]
    fn trace_applies_secant_sky_path_length_and_single_scattering_to_environment_rays() {
        let medium = HomogeneousMedium::rayleigh_atmosphere(450.0, 1.16e-5);
        let atmosphere = AtmosphereMedium {
            medium,
            sun_direction: Vec3::new(0.3, 0.4, 0.0).normalize_or_zero(),
            sun_radiance: 3.0,
            up: Vec3::new(0.0, 1.0, 0.0),
            thickness: 8000.0,
        };
        let environment_radiance = 2.0;
        let scene = Scene::new(vec![], vec![], environment_radiance, Some(atmosphere));

        // 天頂から60°傾いた視線方向(cosθ_zenith=0.5、secant則のクランプの影響を
        // 受けない値)。
        let direction = Vec3::new(3.0_f64.sqrt() / 2.0, 0.5, 0.0);
        let ray = Ray::new(Vec3::ZERO, direction);
        let mut rng = SimRng::new(3, 3);
        let radiance = scene.trace(&ray, &mut rng, 4);

        let path_length = atmosphere.thickness / 0.5;
        let cos_theta = direction.dot(atmosphere.sun_direction);
        let expected = environment_radiance * medium.transmittance(path_length)
            + medium.single_scattering_radiance(path_length, cos_theta, atmosphere.sun_radiance);
        let rel_err = (radiance - expected).abs() / expected;
        assert!(
            rel_err < 1e-9,
            "radiance={radiance} expected={expected} rel_err={rel_err}"
        );
    }

    /// R5/R6(空の色)を`medium`モジュール単体の純粋関数としてではなく、`Scene::trace`
    /// 経由で再確認する(「配線」自体の検証)。環境放射輝度をゼロにして単一散乱項
    /// のみを抽出し、青(450nm)が赤(650nm)より強く散乱される(空が青い)ことを、
    /// 天頂を見上げるレイで確認する。
    #[test]
    fn trace_reproduces_stronger_blue_sky_scattering_than_red_through_the_medium_wiring() {
        let up = Vec3::new(0.0, 1.0, 0.0);
        let sun_direction = Vec3::new(0.6, 0.8, 0.0); // 既に単位ベクトル(0.6²+0.8²=1)。
        let scene_for_wavelength = |wavelength_nm: f64| {
            Scene::new(
                vec![],
                vec![],
                0.0,
                Some(AtmosphereMedium {
                    medium: HomogeneousMedium::rayleigh_atmosphere(wavelength_nm, 1.16e-5),
                    sun_direction,
                    sun_radiance: 1.0,
                    up,
                    thickness: 8000.0,
                }),
            )
        };
        let ray = Ray::new(Vec3::ZERO, up); // 天頂を見上げる。
        let mut rng = SimRng::new(9, 9);

        let blue = scene_for_wavelength(450.0).trace(&ray, &mut rng, 1);
        let red = scene_for_wavelength(650.0).trace(&ray, &mut rng, 1);

        assert!(
            blue > red,
            "sky must appear blue, not red: blue={blue} red={red}"
        );
    }

    /// Aerial perspective: 孤立した白色炉球(albedo=1、`bsdf*cosθ/pdf`が方向に
    /// よらずalbedoに恒常的に相殺されるR1と同じ性質)を異なる距離に置き、大気による
    /// 手前の透過率減衰で遠い球ほど暗く見えることを、Beer-Lambert則が予測する
    /// 比率と比較して定量的に確認する(太陽の単一散乱項は`sun_radiance=0`で
    /// 切り離し、純粋な透過率減衰のみを対象にする——単一散乱込みの検証は上の
    /// 2テストが担う)。
    #[test]
    fn atmosphere_dims_a_distant_white_furnace_sphere_matching_beer_lambert_transmittance_ratio() {
        // 効果を統計的に測定しやすくするため、現実の大気(β_R(550nm)≈1.16e-5/m)より
        // 一桁大きい係数を使う(縮約実装のテスト設計としては`sim-fluid`等で使って
        // きた「効果が測定しやすいパラメータを選ぶ」方針と同じ)。
        let sigma_s_reference: f64 = 1.0e-4;
        let medium = HomogeneousMedium::rayleigh_atmosphere(550.0, sigma_s_reference);
        let atmosphere = AtmosphereMedium {
            medium,
            sun_direction: Vec3::new(0.0, 1.0, 0.0),
            sun_radiance: 0.0, // 単一散乱項を切り離し、透過率減衰のみを対象にする。
            up: Vec3::new(0.0, 1.0, 0.0),
            thickness: 8000.0,
        };

        let environment_radiance = 4.0;
        let radius = 0.5;
        let near_distance: f64 = 1000.0;
        let far_distance: f64 = 5000.0;

        let scene_for = |distance: f64| {
            Scene::new(
                vec![SceneObject::sphere(
                    Sphere {
                        center: Vec3::new(0.0, 0.0, -distance),
                        radius,
                    },
                    Material::Lambertian(Lambertian { albedo: 1.0 }),
                )],
                vec![],
                environment_radiance,
                Some(atmosphere),
            )
        };
        let ray = Ray::new(Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0));

        let n = 200_000;
        let mean_near = average_trace(&scene_for(near_distance), &ray, 7, n);
        let mean_far = average_trace(&scene_for(far_distance), &ray, 7, n);

        // 球面上の交差点までの距離は`distance - radius`だが、比を取ると`radius`は
        // 厳密に相殺する(`transmittance(d)=exp(-sigma_s*d)`が指数関数のため)。
        let measured_ratio = mean_far / mean_near;
        let expected_ratio = (-medium.sigma_s * (far_distance - near_distance)).exp();
        let rel_err = (measured_ratio - expected_ratio).abs() / expected_ratio;
        assert!(
            rel_err < 0.05,
            "measured_ratio={measured_ratio} expected_ratio={expected_ratio} \
             rel_err={rel_err:.4} mean_near={mean_near} mean_far={mean_far}"
        );
    }

    /// 閉じた箱を作るヘルパ(6面の軸並行クアッド、`[-h,h]^3`)。全面に同じ材質を
    /// 与える。クアッドの法線は常に入射レイと逆を向く(`quad.rs`モジュールdoc)ため、
    /// 箱の内側から見て正しく機能する。
    fn closed_box(half: f64, material: Material) -> Vec<SceneObject> {
        let mut objects = Vec::with_capacity(6);
        for axis in 0..3 {
            for sign in [-1.0, 1.0] {
                objects.push(SceneObject::quad(
                    Quad::axis_aligned(axis, sign * half, -half, half, -half, half),
                    material,
                ));
            }
        }
        objects
    }

    /// 面光源(`Emissive`)の検証その1: **閉じた箱の等比級数**。
    ///
    /// 全6面が同一の「放射輝度`L_e`を放ちアルベド`ρ`で反射する」面である閉じた箱の
    /// 内部では、経路追跡の推定値が幾何形状に一切依存しない閉形式になる:
    /// 各バウンスで必ず何かの面に当たり(閉じているので環境へ抜けられない)、
    /// `L_e`を受け取ってスループットが厳密に`ρ`倍される。したがって深さ`d`で
    /// 打ち切った推定値は
    /// $$L = L_e (1 + \rho + \rho^2 + \cdots + \rho^{d-1}) = L_e\frac{1-\rho^d}{1-\rho}$$
    /// に**分散ゼロで厳密一致する**(R1白色炉テストと同じ仕組み——Lambertianの
    /// `bsdf*cosθ/pdf`がコサイン重み付きサンプリングと完全に相殺して方向に依らず
    /// `ρ`になるため、乱数の引き方によらず同じ値になる)。
    ///
    /// これは面光源の配線について次を同時に立証する: 発光が正しく加算されている
    /// (係数1、二重計上なし)・反射成分が正しくスケールされている・複数バウンスが
    /// 正しく積算される・箱が実際に閉じている(クアッドの交差判定に光が漏れる穴が
    /// 無い)。形態係数を必要としないため、参照解データも不要である。
    #[test]
    fn emissive_closed_box_matches_the_analytic_geometric_series_exactly() {
        let emitted = 0.7;
        let albedo = 0.6;
        let scene = Scene::new(
            closed_box(
                1.0,
                Material::Emissive(Emissive {
                    radiance: emitted,
                    albedo,
                }),
            ),
            vec![],
            0.0, // 環境光は箱の外なので寄与しない(閉じている検証も兼ねる)。
            None,
        );

        let mut rng = SimRng::new(20260727, 3);
        for max_depth in 1..=8u32 {
            let expected = emitted * (1.0 - albedo.powi(max_depth as i32)) / (1.0 - albedo);
            // 方向をばらしても分散ゼロで同じ値になるはず(幾何非依存)。
            for _ in 0..16 {
                let direction = rng.unit_sphere();
                let ray = Ray::new(Vec3::ZERO, direction);
                let measured = scene.trace(&ray, &mut rng, max_depth);
                let error = (measured - expected).abs();
                assert!(
                    error < 1e-12,
                    "closed emissive box must match the geometric series exactly: \
                     max_depth={max_depth} measured={measured} expected={expected} error={error:e}"
                );
            }
        }
    }

    /// 面光源の検証その2: 発光面を直接見たときの放射輝度は、その面の`radiance`に
    /// 厳密一致する(アルベド0の純粋な光源なら反射成分が無いため、1バウンスで
    /// 確定する)。`max_depth`を増やしても値が変わらないことも確認する
    /// (吸収体なので余分な寄与が積み上がらない)。
    #[test]
    fn looking_straight_at_a_pure_emitter_returns_its_radiance_exactly() {
        let emitted = 2.5;
        let scene = Scene::new(
            vec![SceneObject::quad(
                Quad::axis_aligned(2, 5.0, -1.0, 1.0, -1.0, 1.0),
                Material::Emissive(Emissive {
                    radiance: emitted,
                    albedo: 0.0,
                }),
            )],
            vec![],
            0.0,
            None,
        );

        let mut rng = SimRng::new(7, 7);
        let ray = Ray::new(Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0));
        for max_depth in 1..=5u32 {
            let measured = scene.trace(&ray, &mut rng, max_depth);
            assert!(
                (measured - emitted).abs() < 1e-12,
                "max_depth={max_depth} measured={measured} expected={emitted}"
            );
        }
    }

    /// 面光源の検証その3: **NEEを行わないため二重計上が起きない**ことを、
    /// 「発光面のアルベドを0にすると等比級数の第1項だけが残る」形で確認する。
    /// もし面光源にNEEを併用していれば(MISなしでは)直接光の寄与が余分に
    /// 加算され`L_e`を超えるはずである(`Emissive`のdoc参照)。
    #[test]
    fn emissive_surfaces_are_not_double_counted_without_multiple_importance_sampling() {
        let emitted = 1.3;
        let scene = Scene::new(
            closed_box(
                1.0,
                Material::Emissive(Emissive {
                    radiance: emitted,
                    albedo: 0.0, // 完全吸収 = 反射成分なし。
                }),
            ),
            vec![],
            0.0,
            None,
        );

        let mut rng = SimRng::new(99, 1);
        for _ in 0..32 {
            let ray = Ray::new(Vec3::ZERO, rng.unit_sphere());
            let measured = scene.trace(&ray, &mut rng, 8);
            assert!(
                (measured - emitted).abs() < 1e-12,
                "a purely absorbing emitter must contribute exactly L_e once: \
                 measured={measured} expected={emitted}"
            );
        }
    }

    /// 発光壁からの**一次反射(single bounce)の投影立体角**を数値求積で独立に
    /// 計算する参照解(R4のcolor bleeding検証の定量的な土台、下のテストのdoc参照)。
    ///
    /// 床の測定点`p`(法線+y)から見て、`wall`(x=`wall_x`平面上、y∈[0,`wall_h`]、
    /// z∈[`wall_z0`,`wall_z1`])が張る投影立体角
    /// $$\Omega_{proj} = \int_{wall} \frac{\cos\theta_P \cos\theta_w}{r^2} dA$$
    /// を中点則で求める。パストレーサとは全く独立な決定的求積であり、
    /// パストレーサ側は「コサイン重み付き半球サンプリング + 再帰」という別経路で
    /// 同じ量を推定するため、両者の一致は幾何・BSDF・サンプリングの連鎖全体を
    /// 相互検証する(`medium.rs`の`single_scattering_closed_form_matches_numerical_
    /// path_integration`と同じ「独立実装どうしの突き合わせ」パターン)。
    fn projected_solid_angle_of_wall(
        p: Vec3,
        wall_x: f64,
        wall_h: f64,
        wall_z0: f64,
        wall_z1: f64,
        n: usize,
    ) -> f64 {
        let dy = wall_h / n as f64;
        let dz = (wall_z1 - wall_z0) / n as f64;
        let da = dy * dz;
        let mut total = 0.0;
        for i in 0..n {
            let qy = (i as f64 + 0.5) * dy;
            for j in 0..n {
                let qz = wall_z0 + (j as f64 + 0.5) * dz;
                let d = Vec3::new(wall_x - p.x, qy - p.y, qz - p.z);
                let r2 = d.dot(d);
                let r = r2.sqrt();
                // 床法線は+y、壁法線は±x。
                let cos_p = d.y / r;
                let cos_w = (d.x / r).abs();
                if cos_p <= 0.0 {
                    continue;
                }
                total += cos_p * cos_w / r2 * da;
            }
        }
        total
    }

    /// **R4(コーネルボックス)のcolor bleeding、定量的検証**。
    ///
    /// 設計(docs/21-verification/01-analytic-tests.md R4)の合格基準は「既知の
    /// 参照解(color bleeding)と収束一致」だが、参照解データが入手できなかった
    /// (同docsのR4注記に調査経緯を記録)。そこでF10(ダム崩壊)と同じく設計を
    /// 改訂し、**解析的に計算できる一次反射の色移りを参照解とする**形で満たす。
    ///
    /// 構成: 床(Lambertian、アルベド`rho_floor`)の測定点へ真上からレイを撃つ。
    /// 側壁は`Emissive { radiance: L_w, albedo: 0 }`——壁の出射放射輝度を
    /// **直接指定する**ことで、光源→壁の輸送という連鎖を1段外し参照解を厳密に
    /// する(壁は完全吸収なので二次以降の反射も無い)。他に光源・環境光は無い
    /// (`environment_radiance = 0`)ので、測定点に届く光は壁からの一次反射のみ。
    /// `max_depth = 2`(床で1バウンス + 壁の発光を読む)で経路長も厳密に限定する。
    ///
    /// このとき床から出る放射輝度は
    /// $$L = \rho_{floor} \cdot L_w \cdot \frac{\Omega_{proj}}{\pi}$$
    /// (コサイン重み付きサンプリングでは壁に当たる確率がちょうど
    /// $\Omega_{proj}/\pi$、かつ`bsdf*cosθ/pdf`が恒等的に$\rho_{floor}$——R1と
    /// 同じ完全相殺)。$\Omega_{proj}$は`projected_solid_angle_of_wall`が
    /// パストレーサとは独立な数値求積で与える。
    ///
    /// 壁からの距離を変えた3点で確認するため、**距離に応じて色移りが減衰する**
    /// という color bleeding の空間的性質そのものが定量的に検証される。
    #[test]
    fn cornell_box_color_bleeding_matches_the_analytic_single_bounce_transfer() {
        let wall_radiance = 1.0;
        let rho_floor = 0.8;
        let wall_x = 1.0;
        let wall_h = 1.0;
        let (wall_z0, wall_z1) = (-1.0, 1.0);

        let scene = Scene::new(
            vec![
                // 床: y=0、x,z∈[-1,1]。
                SceneObject::quad(
                    Quad::axis_aligned(1, 0.0, -1.0, 1.0, -1.0, 1.0),
                    Material::Lambertian(Lambertian { albedo: rho_floor }),
                ),
                // 発光する側壁: x=1、y∈[0,1]、z∈[-1,1]。完全吸収(albedo=0)。
                SceneObject::quad(
                    Quad::axis_aligned(0, wall_x, 0.0, wall_h, wall_z0, wall_z1),
                    Material::Emissive(Emissive {
                        radiance: wall_radiance,
                        albedo: 0.0,
                    }),
                ),
            ],
            vec![],
            0.0,
            None,
        );

        let samples = 200_000;
        let mut rng = SimRng::new(20260727, 11);
        let mut previous: Option<f64> = None;

        for px in [0.9, 0.5, 0.0] {
            let point = Vec3::new(px, 0.0, 0.0);
            let omega_proj =
                projected_solid_angle_of_wall(point, wall_x, wall_h, wall_z0, wall_z1, 400);
            let expected = rho_floor * wall_radiance * omega_proj / std::f64::consts::PI;

            // 真上から床へ撃つ(壁はx=1平面なので鉛直レイとは交差しない)。
            let ray = Ray::new(Vec3::new(px, 1.0, 0.0), Vec3::new(0.0, -1.0, 0.0));
            let mut sum = 0.0;
            for _ in 0..samples {
                sum += scene.trace(&ray, &mut rng, 2);
            }
            let measured = sum / samples as f64;

            let rel_err = (measured - expected).abs() / expected;
            assert!(
                rel_err < 0.03,
                "R4 color bleeding (single-bounce transfer) at px={px}: \
                 measured={measured:.6} expected={expected:.6} rel_err={rel_err:.4}"
            );

            // 壁から遠ざかるほど色移りは弱くなる(color bleedingの空間的性質)。
            if let Some(nearer) = previous {
                assert!(
                    measured < nearer,
                    "bleeding must weaken with distance from the wall: \
                     px={px} measured={measured:.6} nearer={nearer:.6}"
                );
            }
            previous = Some(measured);
        }
    }

    /// コーネルボックスの幾何(1チャンネル分)。`left`/`right`は側壁のアルベド
    /// (このチャンネルでの値)、`white`は床/天井/奥壁のアルベド、`light`は天井
    /// パネルの放射輝度。箱は x∈[-1,1]、y∈[0,2]、z∈[-1,1] で、**手前(z=+1)は
    /// 開いている**(標準的なコーネルボックスと同じ。抜けたレイは
    /// `environment_radiance`=0 を拾う)。
    fn cornell_box_channel(white: f64, left: f64, right: f64, light: f64) -> Vec<SceneObject> {
        let diffuse = |albedo: f64| Material::Lambertian(Lambertian { albedo });
        vec![
            // 床・天井(白)。
            SceneObject::quad(
                Quad::axis_aligned(1, 0.0, -1.0, 1.0, -1.0, 1.0),
                diffuse(white),
            ),
            SceneObject::quad(
                Quad::axis_aligned(1, 2.0, -1.0, 1.0, -1.0, 1.0),
                diffuse(white),
            ),
            // 奥壁(白)。
            SceneObject::quad(
                Quad::axis_aligned(2, -1.0, -1.0, 1.0, 0.0, 2.0),
                diffuse(white),
            ),
            // 左壁・右壁(色付き)。
            SceneObject::quad(
                Quad::axis_aligned(0, -1.0, 0.0, 2.0, -1.0, 1.0),
                diffuse(left),
            ),
            SceneObject::quad(
                Quad::axis_aligned(0, 1.0, 0.0, 2.0, -1.0, 1.0),
                diffuse(right),
            ),
            // 天井直下の発光パネル(面光源、`Emissive`のdoc参照——NEEは行わない)。
            SceneObject::quad(
                Quad::axis_aligned(1, 1.98, -0.3, 0.3, -0.3, 0.3),
                Material::Emissive(Emissive {
                    radiance: light,
                    albedo: 0.0,
                }),
            ),
        ]
    }

    /// 床の測定点から出る放射輝度を推定する(真上からレイを撃ち`samples`本平均)。
    fn measure_floor_radiance(scene: &Scene, px: f64, samples: usize, seed: u64) -> f64 {
        let mut rng = SimRng::new(seed, 5);
        let ray = Ray::new(Vec3::new(px, 1.0, 0.0), Vec3::new(0.0, -1.0, 0.0));
        let mut sum = 0.0;
        for _ in 0..samples {
            sum += scene.trace(&ray, &mut rng, 5);
        }
        sum / samples as f64
    }

    /// **R4(コーネルボックス)のcolor bleeding、フルシーンでの検証**。
    ///
    /// 上の`cornell_box_color_bleeding_matches_the_analytic_single_bounce_transfer`が
    /// 色移りの**量**を独立な数値求積と突き合わせて定量的に裏付けているので、
    /// こちらは実際のコーネルボックス(白い床/天井/奥壁 + 赤い左壁 + 緑い右壁 +
    /// 天井の発光パネル)で**色移りの符号と対称性**を確認する。
    ///
    /// **チャンネル別レンダリングという縮約**: `trace`はモノクロスカラーを返す
    /// (分光レンダリングもRGBも未実装)ため、同じ幾何でアルベドだけをR/G/Bの各
    /// 成分に差し替えた`Scene`を3つ作り、既存の`trace`を3回走らせる。color
    /// bleedingの検証はチャンネル間の比較そのものなのでこれで足りる——`trace`の
    /// 型を変えないため既存60テストへの回帰リスクがゼロという利点もある。これは
    /// 「RGBレンダリング」であって分光(hero wavelength法)ではない。
    ///
    /// 合格条件は対称な2つの主張である:
    /// - 赤い壁の近くの床は、緑の壁の近くの床より**赤チャンネルが明るい**
    /// - 緑の壁の近くの床は、赤い壁の近くの床より**緑チャンネルが明るい**
    ///
    /// さらに**対照実験**として、両側壁を同じ灰色にすると(箱が左右対称になる)
    /// この非対称が消えることを確認する——非対称が幾何やサンプリングの偏りでは
    /// なく確かに壁の色に由来することの裏付け。
    #[test]
    fn cornell_box_shows_color_bleeding_from_the_red_and_green_side_walls() {
        // 標準的なコーネルボックスに近い反射率(白/赤/緑)。
        let white = (0.74, 0.74, 0.74);
        let red = (0.61, 0.06, 0.06);
        let green = (0.12, 0.45, 0.09);
        let light_radiance = 8.0;
        let samples = 60_000;

        // 左壁=赤、右壁=緑。
        let channel_scene = |ch: usize| {
            let pick = |c: (f64, f64, f64)| match ch {
                0 => c.0,
                1 => c.1,
                _ => c.2,
            };
            Scene::new(
                cornell_box_channel(pick(white), pick(red), pick(green), light_radiance),
                vec![],
                0.0,
                None,
            )
        };
        let red_scene = channel_scene(0);
        let green_scene = channel_scene(1);

        let near_red = -0.8; // 赤い左壁のそば。
        let near_green = 0.8; // 緑い右壁のそば。

        let r_near_red = measure_floor_radiance(&red_scene, near_red, samples, 101);
        let r_near_green = measure_floor_radiance(&red_scene, near_green, samples, 102);
        let g_near_red = measure_floor_radiance(&green_scene, near_red, samples, 103);
        let g_near_green = measure_floor_radiance(&green_scene, near_green, samples, 104);
        assert!(
            r_near_red > r_near_green,
            "R4 color bleeding: the red channel must be brighter near the red wall: \
             r_near_red={r_near_red:.6} r_near_green={r_near_green:.6}"
        );
        assert!(
            g_near_green > g_near_red,
            "R4 color bleeding: the green channel must be brighter near the green wall: \
             g_near_green={g_near_green:.6} g_near_red={g_near_red:.6}"
        );

        // 対照実験: 両側壁を同じ灰色にすると左右対称になり、非対称は消える
        // (残るのはモンテカルロ誤差のみ)。
        let gray = (0.5, 0.5, 0.5);
        let symmetric = Scene::new(
            cornell_box_channel(white.0, gray.0, gray.0, light_radiance),
            vec![],
            0.0,
            None,
        );
        let sym_left = measure_floor_radiance(&symmetric, near_red, samples, 201);
        let sym_right = measure_floor_radiance(&symmetric, near_green, samples, 202);
        let asymmetry = (sym_left - sym_right).abs() / (0.5 * (sym_left + sym_right));

        // 色付きの箱で観測された非対称の大きさ。
        let colored_asymmetry =
            (r_near_red - r_near_green).abs() / (0.5 * (r_near_red + r_near_green));
        assert!(
            asymmetry < 0.02,
            "the control (both walls gray) must be left-right symmetric: \
             sym_left={sym_left:.6} sym_right={sym_right:.6} asymmetry={asymmetry:.4}"
        );
        assert!(
            colored_asymmetry > 5.0 * asymmetry.max(1e-4),
            "the colored box's asymmetry must clearly exceed the symmetric control's \
             residual noise: colored={colored_asymmetry:.4} control={asymmetry:.4}"
        );
    }
}
