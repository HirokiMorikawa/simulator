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
//! - **カメラ経路と合成しない**。`CausticMap` は独立した成果物であり、
//!   `Scene::trace` が返す放射輝度へ足し込まれるわけではない
//!   (合成にはフォトンマップの密度推定が要る)。したがって本モジュールは
//!   「コースティクスが定量的に正しく計算できる」ことは示すが、
//!   「パストレ画像の中にコースティクスが現れる」ことは示さない。
//! - **分散(波長ごとの屈折率差)は呼び出し側の責務**。`CauchyDielectric::
//!   to_dielectric_at` で波長ごとに屈折率を具体化して本関数を波長分呼べば
//!   分光コースティクスになるが、本モジュール自身は単色である。
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
}
