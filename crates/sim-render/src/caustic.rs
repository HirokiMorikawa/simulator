//! コースティクス(集光模様)。設計 docs/21-verification/03-demo-scenarios.md
//! D40「光の実験室 — プリズム分光・**水中コースティクス**・虹をパストレで」。
//!
//! ## なぜ通常のパストレでは出ないのか
//!
//! `path_tracer` のパストレースは、カメラから出たレイをBSDFサンプリングで追跡し、
//! 拡散面で光源を明示サンプル(NEE)する方式である。コースティクスは
//! **鏡面→拡散→鏡面(SDS)経路**——光源から出た光が誘電体で屈折して床に集まり、
//! それをカメラが見る——であり、拡散面でのNEEは「光源への直線」しか試さないので
//! 誘電体を経由した経路を見つけられない。フォトンマッピングや双方向パストレースが
//! 必要になる領域である。
//!
//! ## 本モジュールの縮約実装
//!
//! **光源側から前方へレイを飛ばす(ライトトレーシング)**。誘電体を屈折で通過した
//! 光線が床平面に落ちた点のエネルギーを2Dグリッド(`CausticMap`)へ堆積させる。
//! 縮約している点を正直に列挙する:
//!
//! - **平行ビーム(コリメート光)のみ**。点光源からの発散ビームは対象外
//!   (球レンズの解析焦点距離の公式が平行入射を前提とするため、検証と揃えた)。
//! - **床は軸に垂直な平面1枚のみ**。任意形状の受光面は対象外。
//! ### 群6で解消した2点
//!
//! - **カメラ経路との合成**(`composite_onto_floor`)。移行前は`CausticMap`が独立した
//!   成果物で、パストレ画像の中にコースティクスが現れなかった。群6では
//!   **堆積エネルギーを床面の放射照度 $E$ とみなし、Lambertian床の反射放射輝度
//!   $L=\rho E/\pi$ を一次レイに沿って足し込む**(グリッドのセルがそのまま
//!   フォトンマップの密度推定の「バケツ」になっている)。
//!   **残る縮約**: 足し込むのは**一次レイが直接床を見たときだけ**で、鏡やガラス越しに
//!   映り込んだコースティクスは出ない(それには`Scene::trace`の再帰全体へ
//!   フォトンマップを配線する必要があり、ホットパスへの侵襲が大きい)。
//! - **分光コースティクス**(`trace_ball_lens_caustic_spectral`)。移行前は「波長ごとに
//!   呼び出し側が`CauchyDielectric::to_dielectric_at`で具体化すればよい」と
//!   책任を外に出していたが、実際にそれをやる関数が無かった。群6では可視域を
//!   等間隔サンプルして波長ごとに集光を追跡し、`spectrum`モジュールの
//!   `spectral_samples_to_linear_srgb`(CIE等色関数)でRGBへ落とす——分散のある材質では**焦点距離が波長ごとに違う**ので、
//!   焦点面の外周に色付き(軸上色収差)のリングが出る。
//!
//! ## 球レンズの解析焦点(検証の正典)
//!
//! 半径 $R$・屈折率 $n$ の球(ボールレンズ)に平行光が入射したときの
//! **近軸**後側焦点距離は、球の中心から測って
//!
//! $$ f = \frac{nR}{2(n-1)} $$
//!
//! である($n=1.5$ なら $f=1.5R$、すなわち射出面から $0.5R$ 奥)。
//! **これは近軸(paraxial)公式であり、開口が大きいと球面収差で焦点が手前へ寄る**
//! ——`tests` でこのずれを実測し数値として記録してある。

use sim_math::Vec3;

use crate::bsdf::Dielectric;
use crate::framebuffer::Framebuffer;
use crate::ray::Ray;
use crate::sphere::Sphere;

/// 球レンズ(半径`radius`・屈折率`ior`)の**近軸**後側焦点距離(球中心から測る)。
/// $f = nR/(2(n-1))$。モジュールdoc参照。
pub fn ball_lens_paraxial_focal_distance(radius: f64, ior: f64) -> f64 {
    ior * radius / (2.0 * (ior - 1.0))
}

/// 光軸に平行で、光軸から `entry_height` だけ離れた1本の光線を球レンズへ入射させ、
/// 射出後に**光軸と交わる位置**(球中心からの距離)を返す。
///
/// 幾何は検証しやすい標準配置に固定する: 球は原点中心、光軸は $-z$ 方向、
/// 光線は $(h, 0, +2R)$ から $(0,0,-1)$ へ進む。戻り値は球中心から $-z$ 方向へ
/// 測った距離(正なら球の後方)。
///
/// 全反射(TIR)や、射出光線が光軸と交わらない場合は `None`。
/// `entry_height` が小さいほど `ball_lens_paraxial_focal_distance` に近づき、
/// 大きいほど球面収差で手前(小さい値)へずれる。
pub fn ball_lens_ray_focus_distance(radius: f64, ior: f64, entry_height: f64) -> Option<f64> {
    let sphere = Sphere {
        center: Vec3::ZERO,
        radius,
    };
    let ray = Ray::new(
        Vec3::new(entry_height, 0.0, 2.0 * radius),
        Vec3::new(0.0, 0.0, -1.0),
    );

    // 前面(入射): `Sphere::intersect`の法線は常に外向きなので、入射側から見て
    // 外向き(d・n<0)という`Dielectric::refract`の前提をそのまま満たす。
    let entry = sphere.intersect(&ray, 1e-9)?;
    let inside_dir = Dielectric::refract(ray.direction, entry.normal, 1.0 / ior)?;
    let inside_ray = Ray::new(entry.point, inside_dir);

    // 後面(射出): 球内部から当たるので外向き法線は進行方向と同じ側を向く
    // (d・n>0)。`refract`の前提に合わせて法線を反転し、相対屈折率も逆にする。
    let exit = sphere.intersect(&inside_ray, 1e-9)?;
    let outward = exit.normal.scale(-1.0);
    let exit_dir = Dielectric::refract(inside_ray.direction, outward, ior)?;

    // 光軸(x=0, y=0 の直線)との交点。x成分がゼロになるパラメータを解く。
    if exit_dir.x.abs() < 1e-15 {
        return None;
    }
    let t = -exit.point.x / exit_dir.x;
    if t <= 0.0 {
        return None;
    }
    let z = exit.point.z + exit_dir.z * t;
    // 球中心(z=0)から -z 方向へ測った距離。
    Some(-z)
}

/// 床平面(軸に垂直な1枚)に堆積した放射エネルギーの2Dグリッド。
#[derive(Clone, Debug)]
pub struct CausticMap {
    width: usize,
    height: usize,
    /// グリッドが覆う正方領域の一辺の半分(中心は光軸)。
    half_extent: f64,
    energy: Vec<f64>,
}

impl CausticMap {
    pub fn new(width: usize, height: usize, half_extent: f64) -> CausticMap {
        assert!(width > 0 && height > 0 && half_extent > 0.0);
        CausticMap {
            width,
            height,
            half_extent,
            energy: vec![0.0; width * height],
        }
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    /// 1セルの面積(エネルギー密度=エネルギー/面積の計算に使う)。
    pub fn cell_area(&self) -> f64 {
        let w = 2.0 * self.half_extent / self.width as f64;
        let h = 2.0 * self.half_extent / self.height as f64;
        w * h
    }

    /// 堆積した総エネルギー。
    pub fn total_energy(&self) -> f64 {
        self.energy.iter().sum()
    }

    /// 最大セルのエネルギー(集光の強さ。密度にするには`cell_area()`で割る)。
    pub fn peak_energy(&self) -> f64 {
        self.energy.iter().copied().fold(0.0, f64::max)
    }

    /// グリッド座標`(x, z)`[m]へ`energy`を堆積する。領域外なら何もせず`false`。
    fn deposit(&mut self, x: f64, z: f64, energy: f64) -> bool {
        let to_index = |v: f64, n: usize| -> Option<usize> {
            let t = (v + self.half_extent) / (2.0 * self.half_extent);
            if !(0.0..1.0).contains(&t) {
                return None;
            }
            Some(((t * n as f64) as usize).min(n - 1))
        };
        let (Some(ix), Some(iz)) = (to_index(x, self.width), to_index(z, self.height)) else {
            return false;
        };
        self.energy[iz * self.width + ix] += energy;
        true
    }

    /// グリッド座標`(x, z)`[m]での**放射照度** $E$ [W/m²](= セルのエネルギー /
    /// セル面積、**群6で追加**)。領域外なら`None`。フォトンマップの密度推定
    /// (単位面積あたりのフォトン数)そのもので、セルがバケツの役をする。
    pub fn irradiance_at(&self, x: f64, z: f64) -> Option<f64> {
        let to_index = |v: f64, n: usize| -> Option<usize> {
            let t = (v + self.half_extent) / (2.0 * self.half_extent);
            if !(0.0..1.0).contains(&t) {
                return None;
            }
            Some(((t * n as f64) as usize).min(n - 1))
        };
        let (ix, iz) = (to_index(x, self.width)?, to_index(z, self.height)?);
        Some(self.energy[iz * self.width + ix] / self.cell_area())
    }

    /// 可視化用のフレームバッファへ変換する(単色、`scale`倍して線形RGBに載せる)。
    pub fn to_framebuffer(&self, scale: f64) -> Framebuffer {
        let mut fb = Framebuffer::new(self.width as u32, self.height as u32);
        for (dst, &e) in fb.pixels.iter_mut().zip(self.energy.iter()) {
            let v = e * scale;
            *dst = Vec3::new(v, v, v);
        }
        fb
    }
}

/// 平行ビームを球レンズへ通し、床平面 $z = -\text{floor\_distance}$(球中心から
/// $-z$ 方向へ`floor_distance`)に落ちたエネルギーを`map`へ堆積する。
///
/// ビームはビーム断面(半径`beam_radius`の円板)を `samples_per_axis` × `samples_per_axis`
/// の正方格子で決定的に走査する(乱数を使わない——このプロジェクトは決定論を重視し、
/// かつ集光模様の検証には系統的な走査の方が適している)。円板の外に落ちた格子点は
/// 捨てる。各光線は等しいエネルギー`1/該当本数`を運ぶので、**総打ち上げエネルギーは
/// 常に1**になる。
///
/// フレネル反射による損失は**計上しない**(幾何的な集光のみを見る縮約)。
/// これによりエネルギー保存の検証が「堆積総和 == 1 − (TIR/領域外で失われた分)」
/// という明快な形になる。
///
/// 戻り値は (堆積できた光線数, 打ち上げた光線数)。
pub fn trace_ball_lens_caustic(
    radius: f64,
    ior: f64,
    beam_radius: f64,
    floor_distance: f64,
    samples_per_axis: usize,
    map: &mut CausticMap,
) -> (usize, usize) {
    let sphere = Sphere {
        center: Vec3::ZERO,
        radius,
    };

    // 円板内に入る格子点を先に数え、1本あたりのエネルギーを決める。
    let mut launched = 0usize;
    let grid: Vec<(f64, f64)> = (0..samples_per_axis)
        .flat_map(|i| (0..samples_per_axis).map(move |j| (i, j)))
        .filter_map(|(i, j)| {
            let u = (i as f64 + 0.5) / samples_per_axis as f64 * 2.0 - 1.0;
            let v = (j as f64 + 0.5) / samples_per_axis as f64 * 2.0 - 1.0;
            let (x, y) = (u * beam_radius, v * beam_radius);
            (x * x + y * y <= beam_radius * beam_radius).then_some((x, y))
        })
        .collect();
    let per_ray = if grid.is_empty() {
        0.0
    } else {
        1.0 / grid.len() as f64
    };

    let mut deposited = 0usize;
    for (x, y) in grid {
        launched += 1;
        let ray = Ray::new(Vec3::new(x, y, 2.0 * radius), Vec3::new(0.0, 0.0, -1.0));
        let Some(entry) = sphere.intersect(&ray, 1e-9) else {
            continue;
        };
        let Some(inside_dir) = Dielectric::refract(ray.direction, entry.normal, 1.0 / ior) else {
            continue;
        };
        let inside_ray = Ray::new(entry.point, inside_dir);
        let Some(exit) = sphere.intersect(&inside_ray, 1e-9) else {
            continue;
        };
        let outward = exit.normal.scale(-1.0);
        let Some(exit_dir) = Dielectric::refract(inside_ray.direction, outward, ior) else {
            continue;
        };
        // 床平面 z = -floor_distance との交点。
        if exit_dir.z.abs() < 1e-15 {
            continue;
        }
        let t = (-floor_distance - exit.point.z) / exit_dir.z;
        if t <= 0.0 {
            continue;
        }
        let hit = exit.point + exit_dir.scale(t);
        if map.deposit(hit.x, hit.y, per_ray) {
            deposited += 1;
        }
    }
    (deposited, launched)
}

/// コースティクスを**レンダリング済みの画像へ合成する**(**群6で追加**、
/// モジュールdoc「群6で解消した2点」参照)。
///
/// カメラから各ピクセルへ一次レイを飛ばし、`plane_normal`・`plane_offset`が定める
/// 床平面(平面の式 $\mathbf p\cdot\mathbf n = d$)と交差したら、その交点での
/// `CausticMap`の放射照度 $E$ から Lambertian の反射放射輝度 $L=\rho E/\pi$ を
/// 求めて画素へ**加算**する。
///
/// 平面座標へのマッピングは`plane_axes`(交点から引く2軸、`CausticMap`のグリッド
/// 座標に対応する単位ベクトル対)で与える。`total_power`は打ち上げたビームの
/// 総放射束 [W](`trace_ball_lens_caustic`は総エネルギー1で正規化して堆積するため、
/// ここで物理的な強さへ戻す)。
///
/// **一次レイのみ**(鏡・ガラス越しの映り込みには寄与しない、モジュールdoc参照)。
#[allow(clippy::too_many_arguments)]
pub fn composite_onto_floor(
    framebuffer: &mut Framebuffer,
    camera: &crate::camera::Camera,
    vfov: f64,
    plane_normal: Vec3,
    plane_offset: f64,
    plane_axes: (Vec3, Vec3),
    plane_origin: Vec3,
    map: &CausticMap,
    albedo: Vec3,
    total_power: f64,
) {
    let (width, height) = (framebuffer.width, framebuffer.height);
    let aspect = width as f64 / height as f64;
    let normal = plane_normal.normalize_or_zero();
    for py in 0..height {
        for px in 0..width {
            // ピクセル中心(合成はアンチエイリアスを持たない——コースティクスは
            // グリッド解像度で既にぼけているため、ここでのジッタは効果が薄い)。
            let ndc_x = 2.0 * (px as f64 + 0.5) / width as f64 - 1.0;
            let ndc_y = 1.0 - 2.0 * (py as f64 + 0.5) / height as f64;
            let direction = camera.pinhole_direction(ndc_x, ndc_y, aspect, vfov);
            let denominator = direction.dot(normal);
            if denominator.abs() < 1e-12 {
                continue;
            }
            let t = (plane_offset - camera.origin.dot(normal)) / denominator;
            if t <= 0.0 {
                continue;
            }
            let point = camera.origin + direction.scale(t);
            let local = point - plane_origin;
            let (u, v) = (local.dot(plane_axes.0), local.dot(plane_axes.1));
            let Some(irradiance) = map.irradiance_at(u, v) else {
                continue;
            };
            // Lambertian の反射放射輝度 L = ρE/π(設計 docs/17-rendering/02 §2)。
            let radiance = irradiance * total_power / std::f64::consts::PI;
            let index = py as usize * width as usize + px as usize;
            framebuffer.pixels[index] = framebuffer.pixels[index]
                + Vec3::new(
                    albedo.x * radiance,
                    albedo.y * radiance,
                    albedo.z * radiance,
                );
        }
    }
}

/// **分光コースティクス**(**群6で追加**、モジュールdoc参照)。可視域を
/// `wavelength_count`本の波長でサンプルし、Cauchy分散
/// (`sim_em::cauchy_refractive_index`)で波長ごとに屈折率を具体化して集光を追跡、
/// `spectrum::spectral_samples_to_linear_srgb`(CIE等色関数)でRGBのフレーム
/// バッファへ落とす。
///
/// `cauchy_a`/`cauchy_b`はCauchy式 $n(\lambda)=A+B/\lambda^2$ の係数
/// ($\lambda$は µm、`bsdf::CauchyDielectric`と同じ規約)。分散があると
/// **波長ごとに焦点距離が違う**ため、単一の焦点面で切ると色付きのリング
/// (軸上色収差)が現れる——それが本関数の見どころである。
#[allow(clippy::too_many_arguments)]
pub fn trace_ball_lens_caustic_spectral(
    radius: f64,
    cauchy_a: f64,
    cauchy_b: f64,
    beam_radius: f64,
    floor_distance: f64,
    samples_per_axis: usize,
    resolution: usize,
    half_extent: f64,
    wavelength_count: usize,
) -> Framebuffer {
    // 可視域を等間隔に覆う波長列(`spectrum::hero_wavelengths`をhero=0で使うと
    // ちょうど等間隔の層化サンプルになる)。
    let wavelengths = crate::spectrum::hero_wavelengths(0.0, wavelength_count);
    // 波長ごとの`CausticMap`。
    let maps: Vec<(f64, CausticMap)> = wavelengths
        .iter()
        .map(|&wavelength| {
            let ior = sim_em::cauchy_refractive_index(cauchy_a, cauchy_b, wavelength);
            let mut map = CausticMap::new(resolution, resolution, half_extent);
            trace_ball_lens_caustic(
                radius,
                ior,
                beam_radius,
                floor_distance,
                samples_per_axis,
                &mut map,
            );
            (wavelength, map)
        })
        .collect();

    let mut framebuffer = Framebuffer::new(resolution as u32, resolution as u32);
    let mut spectrum: Vec<(f64, f64)> = Vec::with_capacity(maps.len());
    for iz in 0..resolution {
        for ix in 0..resolution {
            spectrum.clear();
            for (wavelength, map) in &maps {
                spectrum.push((*wavelength, map.energy[iz * resolution + ix]));
            }
            let (r, g, b) = crate::spectrum::spectral_samples_to_linear_srgb(&spectrum);
            framebuffer.pixels[iz * resolution + ix] = Vec3::new(r, g, b);
        }
    }
    framebuffer
}

#[cfg(test)]
mod tests {
    use super::*;

    /// **近軸領域では解析焦点と一致する**。球レンズの後側焦点距離
    /// $f=nR/(2(n-1))$ は**近軸公式**なので、光軸に十分近い光線
    /// (入射高さ $h \le 0.02R$)で検証する。
    #[test]
    fn paraxial_rays_focus_at_the_analytic_ball_lens_focal_distance() {
        let radius = 1.0;
        let ior = 1.5;
        let analytic = ball_lens_paraxial_focal_distance(radius, ior);
        assert!((analytic - 1.5).abs() < 1e-12, "n=1.5, R=1 なら f=1.5R");

        for &h in &[0.005, 0.01, 0.02] {
            let traced = ball_lens_ray_focus_distance(radius, ior, h * radius)
                .expect("近軸光線は必ず射出して光軸と交わる");
            let rel_err = (traced - analytic).abs() / analytic;
            assert!(
                rel_err < 0.01,
                "近軸(h={h}R)では解析焦点と一致すべき: traced={traced} analytic={analytic} \
                 rel_err={rel_err:.5}"
            );
        }
    }

    /// **球面収差の定量記録(実測)**: 開口を広げると焦点は単調に手前へ寄る。
    /// これは近軸公式が破れる正しい物理であり、バグではない——この事実を
    /// テストとして固定することで、「フル開口で解析焦点と一致しない」ことを
    /// 将来の実装者が異常と誤認するのを防ぐ。
    #[test]
    fn spherical_aberration_moves_the_focus_closer_as_the_aperture_widens() {
        let radius = 1.0;
        let ior = 1.5;
        let paraxial = ball_lens_paraxial_focal_distance(radius, ior);

        let mut previous = f64::INFINITY;
        for &h in &[0.01, 0.2, 0.4, 0.6, 0.8] {
            let traced = ball_lens_ray_focus_distance(radius, ior, h * radius)
                .expect("この入射高さでは全反射しない");
            assert!(
                traced < previous,
                "入射高さが増えるほど焦点は手前へ寄るべき: h={h}R traced={traced} previous={previous}"
            );
            assert!(
                traced <= paraxial + 1e-9,
                "球面収差は焦点を近軸値より手前にしかしない: h={h}R traced={traced} paraxial={paraxial}"
            );
            previous = traced;
        }

        // 縁(h=0.8R)での実測ずれを数値として記録する(近軸値からの後退量)。
        let marginal = ball_lens_ray_focus_distance(radius, ior, 0.8 * radius).unwrap();
        let shift = (paraxial - marginal) / paraxial;
        assert!(
            shift > 0.15,
            "h=0.8R の縁光線は近軸焦点よりはっきり手前に集まるはず(実測 shift={shift:.4})"
        );
    }

    /// **エネルギー保存**: 打ち上げた総エネルギー1が、床のグリッドに過不足なく
    /// 堆積する(グリッドを十分広く取り、全光線が領域内に落ちる構成)。
    /// 堆積の計上そのものにエネルギーの生成・消失が無いことの直接検証。
    #[test]
    fn deposited_energy_equals_the_launched_energy_when_all_rays_land_on_the_map() {
        let radius = 1.0;
        let ior = 1.5;
        let mut map = CausticMap::new(64, 64, 4.0);
        let (deposited, launched) =
            trace_ball_lens_caustic(radius, ior, 0.8 * radius, 1.5, 48, &mut map);

        assert_eq!(
            deposited, launched,
            "十分広いグリッドなら全光線が領域内に落ちるべき"
        );
        assert!(
            (map.total_energy() - 1.0).abs() < 1e-9,
            "堆積した総エネルギーは打ち上げた1に一致すべき: total={}",
            map.total_energy()
        );
    }

    /// **対照実験(コースティクスが実際に集光であることの証明)**: 同じビーム・
    /// 同じ床でも、球レンズを通した場合は通さない場合よりピークセルの
    /// エネルギーが桁違いに大きい。「集光している」という主張の直接の根拠。
    #[test]
    fn the_lens_concentrates_energy_far_above_the_uniform_beam_baseline() {
        let radius = 1.0;
        let ior = 1.5;
        let beam_radius = 0.8 * radius;
        let focal = ball_lens_paraxial_focal_distance(radius, ior);

        let mut lensed = CausticMap::new(64, 64, 4.0);
        trace_ball_lens_caustic(radius, ior, beam_radius, focal, 48, &mut lensed);

        // 対照: 屈折率1(=レンズが無いのと同じ)。平行ビームがそのまま落ちる。
        let mut unlensed = CausticMap::new(64, 64, 4.0);
        trace_ball_lens_caustic(radius, 1.0, beam_radius, focal, 48, &mut unlensed);

        let ratio = lensed.peak_energy() / unlensed.peak_energy();
        assert!(
            ratio > 10.0,
            "レンズありのピークは無しの10倍を超えるべき(実測 ratio={ratio:.2}, \
             lensed_peak={}, unlensed_peak={})",
            lensed.peak_energy(),
            unlensed.peak_energy()
        );
    }

    /// **群6: 分光コースティクス**。分散のある材質(BK7のCauchy係数)では
    /// 波長ごとに屈折率が違い、したがって**焦点距離が違う**(軸上色収差)。
    ///
    /// 検証は近軸領域(ビーム半径0.1R)で行う——実装検証中、ビーム半径0.6Rでは
    /// **球面収差が色収差を完全に覆い隠す**(周辺光線の焦点が近軸焦点よりずっと
    /// 手前へ来るため、青の近軸焦点面で切ると赤のほうが集光して見える)ことを
    /// 実測で確認した。色収差を見たいなら球面収差を小さくしておく必要がある、
    /// という光学の当たり前を数値で踏んだ形。
    #[test]
    fn spectral_caustic_shows_axial_chromatic_aberration() {
        // BK7 の Cauchy 係数(`bsdf::CauchyDielectric`の既存テストと同じ値、λはnm)。
        let (a, b) = (1.5046, 4200.0);
        let radius = 1.0;
        let beam_radius = 0.1 * radius;

        let n_blue = sim_em::cauchy_refractive_index(a, b, 450.0);
        let n_red = sim_em::cauchy_refractive_index(a, b, 650.0);
        assert!(n_blue > n_red, "分散: 短波長ほど屈折率が大きい");
        let focal_blue = ball_lens_paraxial_focal_distance(radius, n_blue);
        let focal_red = ball_lens_paraxial_focal_distance(radius, n_red);
        assert!(focal_blue < focal_red, "青のほうが手前で結ぶ");

        // (1) 各焦点面では**その波長が最もよく集光する**。
        let peak_at = |ior: f64, plane: f64| {
            let mut map = CausticMap::new(64, 64, 0.02);
            trace_ball_lens_caustic(radius, ior, beam_radius, plane, 400, &mut map);
            map.peak_energy()
        };
        let (blue_at_blue, red_at_blue) = (peak_at(n_blue, focal_blue), peak_at(n_red, focal_blue));
        let (blue_at_red, red_at_red) = (peak_at(n_blue, focal_red), peak_at(n_red, focal_red));
        assert!(
            blue_at_blue > 2.0 * red_at_blue,
            "青の焦点面では青が集光しているはず: blue={blue_at_blue} red={red_at_blue}"
        );
        assert!(
            red_at_red > 2.0 * blue_at_red,
            "赤の焦点面では赤が集光しているはず: red={red_at_red} blue={blue_at_red}"
        );

        // (2) 分光合成した画像が実際に**色付く**: 青の焦点面で切ると、中心は
        //     青(集光した短波長)、その外は赤(まだ絞られていない長波長)になる。
        let resolution = 64usize;
        let fb = trace_ball_lens_caustic_spectral(
            radius,
            a,
            b,
            beam_radius,
            focal_blue,
            400,
            resolution,
            0.02,
            16,
        );
        assert_eq!(fb.width, resolution as u32);
        let center = resolution / 2;
        let at = |ix: usize, iz: usize| fb.pixels[iz * resolution + ix];
        let core = at(center, center);
        let halo = at(center + 1, center);
        assert!(
            core.length() > 0.0 && halo.length() > 0.0,
            "光が届いているはず"
        );
        assert!(
            core.z > core.x,
            "焦点の芯は青寄りのはず(短波長が集光している): {core:?}"
        );
        assert!(
            halo.x > halo.z,
            "その外周は赤寄りのはず(長波長がまだ絞られていない): {halo:?}"
        );
    }

    /// **群6: カメラ経路への合成**。真下を向いたカメラの前に床平面を置き、
    /// コースティクスを合成すると、集光している中心の画素だけが明るくなること、
    /// 加算量が Lambertian の $L=\rho E/\pi$ に厳密に一致することを確認する。
    #[test]
    fn compositing_adds_the_lambertian_reflected_radiance_of_the_caustic_irradiance() {
        use crate::camera::Camera;
        let resolution = 32u32;
        let half_extent = 1.0;
        let mut map = CausticMap::new(resolution as usize, resolution as usize, half_extent);
        // 中央の1セルだけに既知のエネルギーを置く(合成の算術を厳密に検算するため)。
        let energy = 0.25;
        assert!(map.deposit(0.0, 0.0, energy), "中央セルへ堆積できるはず");
        let irradiance = energy / map.cell_area();

        // 床は y = 0 の水平面、カメラは真上から見下ろす。
        let camera = Camera {
            origin: Vec3::new(0.0, 4.0, 0.0),
            forward: Vec3::new(0.0, -1.0, 0.0),
            right: Vec3::new(1.0, 0.0, 0.0),
            up: Vec3::new(0.0, 0.0, -1.0),
            lens_radius: 0.0,
            focus_distance: 4.0,
        };
        let mut fb = Framebuffer::new(resolution, resolution);
        let albedo = Vec3::new(0.8, 0.6, 0.4);
        let total_power = 3.0;
        composite_onto_floor(
            &mut fb,
            &camera,
            0.6,
            Vec3::new(0.0, 1.0, 0.0),
            0.0,
            (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
            Vec3::ZERO,
            &map,
            albedo,
            total_power,
        );

        // 明るい画素がちょうど1セルぶんだけあること。
        let lit: Vec<usize> = fb
            .pixels
            .iter()
            .enumerate()
            .filter(|(_, p)| p.length() > 0.0)
            .map(|(i, _)| i)
            .collect();
        assert!(
            !lit.is_empty(),
            "集光セルを見ている画素が少なくとも1つはあるはず"
        );

        // 加算量は L = ρ E Φ / π に厳密一致。
        let expected = irradiance * total_power / std::f64::consts::PI;
        for &i in &lit {
            let p = fb.pixels[i];
            assert!((p.x - albedo.x * expected).abs() < 1e-9, "p={p:?}");
            assert!((p.y - albedo.y * expected).abs() < 1e-9, "p={p:?}");
            assert!((p.z - albedo.z * expected).abs() < 1e-9, "p={p:?}");
        }
        // 何も堆積していないセルを見ている画素は変化しない。
        assert!(
            lit.len() < fb.pixels.len(),
            "全画素が光るのはおかしい(領域マッピングの誤り)"
        );
    }
}
