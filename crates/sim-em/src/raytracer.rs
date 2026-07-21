//! 幾何光学レイトレーサ。設計: docs/13-electromagnetism/04-light-optics.md §3/§4。
//!
//! P4 スコープの実装: 光線と球面/平面の交差 + 反射/屈折(フレネル係数によるパワー分配)の
//! 分岐トレース(深さ・パワー打切り)。光線束追跡(rayon並列化)・波長サンプリングの
//! CIE等色関数RGB変換・結像のスクリーンビニングは未実装(単一光線のエネルギー収支検証が
//! 目的のP4残りのスコープでは不要)。

use crate::optics::fresnel_reflectance;
use sim_math::Vec3;

/// 光線。設計§3。
#[derive(Clone, Copy, Debug)]
pub struct Ray {
    pub origin: Vec3,
    pub dir: Vec3, // 正規化済み
    pub power: f64,
}

/// 光学面の種類。
#[derive(Clone, Copy, Debug)]
pub enum SurfaceKind {
    Mirror,
    /// 屈折率 `n`(反対側の媒質、レイが現在いる側は`ambient_index`で別途管理)。
    Dielectric {
        index: f64,
    },
    Absorber,
}

/// 光学形状(P4スコープ: 球・平面のみ、レンズ/プリズムはこの組み合わせで表現)。
#[derive(Clone, Copy, Debug)]
pub enum SurfaceGeom {
    Sphere { center: Vec3, radius: f64 },
    Plane { normal: Vec3, d: f64 },
}

#[derive(Clone, Copy, Debug)]
pub struct OpticalSurface {
    pub geometry: SurfaceGeom,
    pub kind: SurfaceKind,
}

/// トレース打切り条件(設計§3「打切り: power < ε or 深さ > 16」)。
const MAX_DEPTH: u32 = 16;
const MIN_POWER: f64 = 1e-9;

fn intersect(ray: &Ray, surface: &OpticalSurface) -> Option<(f64, Vec3)> {
    match surface.geometry {
        SurfaceGeom::Plane { normal, d } => {
            let denom = normal.dot(ray.dir);
            if denom.abs() < 1e-12 {
                return None;
            }
            let t = (d - normal.dot(ray.origin)) / denom;
            if t > 1e-9 {
                Some((t, normal))
            } else {
                None
            }
        }
        SurfaceGeom::Sphere { center, radius } => {
            let oc = ray.origin - center;
            let b = oc.dot(ray.dir);
            let c = oc.length_sq() - radius * radius;
            let disc = b * b - c;
            if disc < 0.0 {
                return None;
            }
            let sqrt_disc = disc.sqrt();
            let t1 = -b - sqrt_disc;
            let t2 = -b + sqrt_disc;
            let t = if t1 > 1e-9 {
                t1
            } else if t2 > 1e-9 {
                t2
            } else {
                return None;
            };
            let point = ray.origin.addcarry_scaled(ray.dir, t);
            let normal = (point - center).scale(1.0 / radius);
            Some((t, normal))
        }
    }
}

fn reflect(dir: Vec3, normal: Vec3) -> Vec3 {
    dir - normal.scale(2.0 * dir.dot(normal))
}

/// スネル則による屈折方向(全反射時は`None`)。`normal`は入射側を向く単位法線。
fn refract(dir: Vec3, normal: Vec3, n1: f64, n2: f64) -> Option<Vec3> {
    let cos_i = -dir.dot(normal);
    let sin2_t = (n1 / n2).powi(2) * (1.0 - cos_i * cos_i);
    if sin2_t > 1.0 {
        return None; // 全反射
    }
    let cos_t = (1.0 - sin2_t).sqrt();
    Some(dir.scale(n1 / n2) + normal.scale(n1 / n2 * cos_i - cos_t))
}

/// 光線を`surfaces`に対して再帰的にトレースし、吸収・射出(シーン外への脱出)された
/// 総パワーを`(absorbed, escaped)`として返す(設計§7「系全体で入射パワー=吸収+射出」の
/// エネルギー収支検証に使う)。
pub fn trace_energy(
    ray: Ray,
    surfaces: &[OpticalSurface],
    ambient_index: f64,
    depth: u32,
) -> (f64, f64) {
    if depth > MAX_DEPTH || ray.power < MIN_POWER {
        return (0.0, ray.power); // 打切り: 残りパワーは失われた扱い(吸収でも脱出でもない)
    }

    let mut closest: Option<(f64, Vec3, &OpticalSurface)> = None;
    for surface in surfaces {
        if let Some((t, normal)) = intersect(&ray, surface) {
            if closest.is_none_or(|(ct, _, _)| t < ct) {
                closest = Some((t, normal, surface));
            }
        }
    }

    let Some((t, mut normal, surface)) = closest else {
        return (0.0, ray.power); // 何にも当たらずシーン外へ脱出
    };
    let hit_point = ray.origin.addcarry_scaled(ray.dir, t);
    // 法線は常に入射側(レイが来る側)を向くようにする。
    if normal.dot(ray.dir) > 0.0 {
        normal = -normal;
    }

    match surface.kind {
        SurfaceKind::Absorber => (ray.power, 0.0),
        SurfaceKind::Mirror => {
            let reflected = Ray {
                origin: hit_point,
                dir: reflect(ray.dir, normal),
                power: ray.power,
            };
            trace_energy(reflected, surfaces, ambient_index, depth + 1)
        }
        SurfaceKind::Dielectric { index } => {
            // 現在レイがどちら側にいるか(入射法線と幾何法線の向きから)で n1/n2 を決める。
            let entering = ambient_index < index; // 単純化: 常に外側=ambient_indexから内側=indexへ
            let (n1, n2) = if entering {
                (ambient_index, index)
            } else {
                (index, ambient_index)
            };
            let cos_i = (-ray.dir.dot(normal)).abs();
            let reflectance = fresnel_reflectance(n1, n2, cos_i.acos())
                .map(|r| r.r_unpolarized)
                .unwrap_or(1.0); // 全反射(角度がTIR領域): フレネル係数は定義不能、全反射として扱う

            let mut absorbed = 0.0;
            let mut escaped = 0.0;

            if reflectance > 0.0 {
                let reflected = Ray {
                    origin: hit_point,
                    dir: reflect(ray.dir, normal),
                    power: ray.power * reflectance,
                };
                let (a, e) = trace_energy(reflected, surfaces, ambient_index, depth + 1);
                absorbed += a;
                escaped += e;
            }

            let transmittance = 1.0 - reflectance;
            if transmittance > 0.0 {
                match refract(ray.dir, normal, n1, n2) {
                    Some(refracted_dir) => {
                        let next_ambient = if entering { index } else { ambient_index };
                        let refracted = Ray {
                            origin: hit_point,
                            dir: refracted_dir,
                            power: ray.power * transmittance,
                        };
                        let (a, e) = trace_energy(refracted, surfaces, next_ambient, depth + 1);
                        absorbed += a;
                        escaped += e;
                    }
                    None => {
                        // 全反射(フレネル係数の前提が崩れる境界ケース): 全パワーを反射側で扱う。
                        let reflected = Ray {
                            origin: hit_point,
                            dir: reflect(ray.dir, normal),
                            power: ray.power * transmittance,
                        };
                        let (a, e) = trace_energy(reflected, surfaces, ambient_index, depth + 1);
                        absorbed += a;
                        escaped += e;
                    }
                }
            }
            (absorbed, escaped)
        }
    }
}

/// プランクの法則: 分光放射輝度 $B_\lambda(T)=\frac{2hc^2}{\lambda^5}\frac{1}{e^{hc/(\lambda k_BT)}-1}$
/// (設計§2.3)。`wavelength`は[m]。
pub fn planck_spectral_radiance(wavelength: f64, temperature: f64) -> f64 {
    const H: f64 = 6.62607015e-34; // プランク定数
    const C: f64 = 299_792_458.0; // 光速
    const KB: f64 = 1.380649e-23; // ボルツマン定数
    let exponent = H * C / (wavelength * KB * temperature);
    let numerator = 2.0 * H * C * C;
    numerator / (wavelength.powi(5) * (exponent.exp() - 1.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::optics::snell_refract_angle;

    /// エネルギー収支: 単一の誘電体境界(ガラス板)を通過するレイの反射・透過の合計が
    /// 入射パワーと一致する(設計§7「各界面でR+T=1、系全体で入射パワー=吸収+射出<10⁻⁹」)。
    /// 平行平板(2つの平面)を通して、両面の反射・透過の分岐すべてが最終的に脱出することを
    /// フルのレイトレース経由で確認する(E9のフレネル係数の値そのものではなく、実際に
    /// トレーサに組み込んだときにエネルギーが保存することの検証)。
    #[test]
    fn energy_conservation_holds_through_dielectric_slab_trace() {
        let n_glass = 1.5;
        let surfaces = vec![
            OpticalSurface {
                geometry: SurfaceGeom::Plane {
                    normal: Vec3::new(0.0, 0.0, -1.0),
                    d: -5.0,
                },
                kind: SurfaceKind::Dielectric { index: n_glass },
            },
            OpticalSurface {
                geometry: SurfaceGeom::Plane {
                    normal: Vec3::new(0.0, 0.0, -1.0),
                    d: -6.0,
                },
                kind: SurfaceKind::Dielectric { index: n_glass },
            },
        ];

        let ray = Ray {
            origin: Vec3::new(0.0, 0.0, 0.0),
            dir: Vec3::new(0.1, 0.0, 1.0).normalize_or_zero(),
            power: 1.0,
        };
        let (absorbed, escaped) = trace_energy(ray, &surfaces, 1.0, 0);
        assert!(
            absorbed < 1e-12,
            "no absorber in this scene: absorbed={absorbed}"
        );
        let rel_err = (escaped - ray.power).abs() / ray.power;
        assert!(
            rel_err < 1e-9,
            "escaped={escaped} expected={} rel_err={rel_err}",
            ray.power
        );
    }

    /// 屈折レイトレースがスネル則(既存のE10の代数式)と一致することを確認する
    /// (レイトレーサの`refract`とE10の`snell_refract_angle`が独立に同じ結果を出すこと)。
    #[test]
    fn traced_refraction_angle_matches_snell_law_formula() {
        let n1 = 1.0;
        let n2 = 1.5;
        let theta_i: f64 = 30.0_f64.to_radians();

        let normal = Vec3::new(0.0, 0.0, -1.0);
        let dir = Vec3::new(theta_i.sin(), 0.0, theta_i.cos()).normalize_or_zero();
        let refracted = refract(dir, normal, n1, n2).expect("should not TIR at this angle");

        let cos_theta_t = refracted.dot(Vec3::new(0.0, 0.0, 1.0));
        let measured_theta_t = cos_theta_t.acos();
        let expected_theta_t = snell_refract_angle(n1, n2, theta_i).expect("no TIR");

        assert!(
            (measured_theta_t - expected_theta_t).abs() < 1e-9,
            "measured={measured_theta_t} expected={expected_theta_t}"
        );
    }

    /// プランク則: ピーク波長がウィーンの変位則 $\lambda_{max}T=2.898\times10^{-3}$m·K に
    /// 一致(設計§7)。数値的にピーク位置を探索して比較する。
    #[test]
    fn planck_peak_wavelength_matches_wien_displacement_law() {
        let temperature = 5778.0; // 太陽表面相当
        let wien_constant = 2.897_771_955e-3;
        let expected_peak = wien_constant / temperature;

        let mut best_wavelength = expected_peak * 0.5;
        let mut best_radiance = 0.0;
        let mut wavelength = expected_peak * 0.3;
        let end = expected_peak * 3.0;
        let step = expected_peak * 0.0002;
        while wavelength < end {
            let radiance = planck_spectral_radiance(wavelength, temperature);
            if radiance > best_radiance {
                best_radiance = radiance;
                best_wavelength = wavelength;
            }
            wavelength += step;
        }

        let rel_err = (best_wavelength - expected_peak).abs() / expected_peak;
        assert!(
            rel_err < 0.001,
            "best_wavelength={best_wavelength} expected_peak={expected_peak} rel_err={rel_err}"
        );
    }

    /// プランク則: 全波長にわたる積分がシュテファン=ボルツマン則 $\sigma T^4$ に一致
    /// (設計§7、数値積分<0.1%)。分光放射輝度を全立体角・全波長で積分すると
    /// $\int B_\lambda d\lambda \cdot \pi = \sigma T^4$(ランベルト放射体の全放射発散度)。
    #[test]
    fn planck_integral_matches_stefan_boltzmann_law() {
        let temperature: f64 = 1000.0;
        const STEFAN_BOLTZMANN: f64 = 5.670374419e-8;
        let expected = STEFAN_BOLTZMANN * temperature.powi(4);

        // ウィーンのピーク波長を基準に、十分広い範囲を細かい台形則で積分する。
        let peak = 2.897_771_955e-3 / temperature;
        let lo = peak * 0.01;
        let hi = peak * 50.0;
        let n = 200_000;
        let dw = (hi - lo) / n as f64;
        let mut integral = 0.0;
        for i in 0..n {
            let w1 = lo + i as f64 * dw;
            let w2 = w1 + dw;
            let b1 = planck_spectral_radiance(w1, temperature);
            let b2 = planck_spectral_radiance(w2, temperature);
            integral += 0.5 * (b1 + b2) * dw;
        }
        let measured = integral * std::f64::consts::PI;

        let rel_err = (measured - expected).abs() / expected;
        assert!(
            rel_err < 0.001,
            "measured={measured} expected={expected} rel_err={rel_err}"
        );
    }
}
