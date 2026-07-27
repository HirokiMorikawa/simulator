//! 解析形状: 平行四辺形(クアッド)。設計 docs/17-rendering/02-path-tracing.md §4
//! 「BVH: 三角形メッシュ + 解析形状(球・平面)」。
//!
//! **縮約実装の理由**: 三角形メッシュは引き続き未実装。R4(コーネルボックス)が
//! 要求するのは箱の壁面(6枚)と天井の発光パネルであり、いずれも平行四辺形1枚で
//! 表せるため、無限平面でも三角形メッシュでもなく「隅 + 2辺ベクトル」で定義される
//! 有限のクアッドを導入する(無限平面だと箱の壁として使えず、三角形メッシュだと
//! 頂点バッファ・インデックスバッファ・法線補間といった付随機構が一式必要になる)。
//!
//! **法線の向き**: `intersect`が返す法線は常に入射レイと逆を向く(両面材質として
//! 扱う)。コーネルボックスの壁は「内側から見る」ため、幾何法線をそのまま返すと
//! 箱の内部にいるレイに対して裏を向いてしまい、Lambertianの半球サンプリングが
//! 壁の外側へ飛んでしまう。球(`sphere.rs`)は常に外向き法線を返し`trace`側が
//! 入射/出射を判定する(誘電体の屈折で内外の区別が要るため)が、クアッドは
//! 厚みを持たない面なので内外の区別自体が無く、レイ側へ向けるのが自然。

use crate::ray::Ray;
use crate::sphere::Hit;
use sim_math::Vec3;

/// 平行四辺形。`corner`を起点に`edge_u`・`edge_v`が張る面(それぞれの係数が
/// `[0,1]`の範囲内が面上)。
#[derive(Clone, Copy, Debug)]
pub struct Quad {
    pub corner: Vec3,
    pub edge_u: Vec3,
    pub edge_v: Vec3,
}

impl Quad {
    /// 軸並行な矩形を作るヘルパ(コーネルボックスの壁のように、1軸を固定して
    /// 残り2軸方向へ広がる面を素直に書けるようにする)。`axis`は固定する軸
    /// (0=x, 1=y, 2=z)、`value`はその軸上の位置、`(a0,a1)`/`(b0,b1)`は残り2軸の
    /// 範囲(軸番号の小さい順に割り当てる)。
    pub fn axis_aligned(axis: usize, value: f64, a0: f64, a1: f64, b0: f64, b1: f64) -> Quad {
        let (corner, edge_u, edge_v) = match axis {
            0 => (
                Vec3::new(value, a0, b0),
                Vec3::new(0.0, a1 - a0, 0.0),
                Vec3::new(0.0, 0.0, b1 - b0),
            ),
            1 => (
                Vec3::new(a0, value, b0),
                Vec3::new(a1 - a0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, b1 - b0),
            ),
            _ => (
                Vec3::new(a0, b0, value),
                Vec3::new(a1 - a0, 0.0, 0.0),
                Vec3::new(0.0, b1 - b0, 0.0),
            ),
        };
        Quad {
            corner,
            edge_u,
            edge_v,
        }
    }

    /// 面積(2辺ベクトルの外積の長さ)。面光源の放射輝度⇔放射束の換算に使う。
    pub fn area(&self) -> f64 {
        self.edge_u.cross(self.edge_v).length()
    }

    /// 幾何法線(単位ベクトル、`edge_u × edge_v`の向き)。`intersect`が返す法線は
    /// レイ側へ反転され得る(モジュールdoc参照)ため、反転前の向きが要る場合に使う。
    pub fn geometric_normal(&self) -> Vec3 {
        let n = self.edge_u.cross(self.edge_v);
        n.scale(1.0 / n.length())
    }

    /// レイ-平行四辺形交差。まず面を含む平面との交点を求め、その交点を
    /// `corner + α·edge_u + β·edge_v` と表したときの `(α, β)` が共に `[0,1]` に
    /// 収まるかで面内判定する(外積による標準的な平面パラメータ化)。
    pub fn intersect(&self, ray: &Ray, t_min: f64) -> Option<Hit> {
        let n = self.edge_u.cross(self.edge_v);
        let denominator = n.dot(ray.direction);
        if denominator.abs() < 1e-12 {
            return None; // レイが面と平行。
        }
        let t = n.dot(self.corner - ray.origin) / denominator;
        if t <= t_min {
            return None;
        }

        let point = ray.at(t);
        // `w = n / (n·n)` を使うと、面内座標が α = w·(p_rel × edge_v)、
        // β = w·(edge_u × p_rel) で求まる(平行四辺形の標準的なパラメータ化)。
        let w = n.scale(1.0 / n.dot(n));
        let planar = point - self.corner;
        let alpha = w.dot(planar.cross(self.edge_v));
        let beta = w.dot(self.edge_u.cross(planar));
        if !(0.0..=1.0).contains(&alpha) || !(0.0..=1.0).contains(&beta) {
            return None;
        }

        // 法線は常に入射レイと逆向きにする(両面材質、モジュールdoc参照)。
        let unit_normal = n.scale(1.0 / n.length());
        let normal = if unit_normal.dot(ray.direction) < 0.0 {
            unit_normal
        } else {
            -unit_normal
        };
        Some(Hit { t, point, normal })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 正面から撃ったレイが平面の解析的な交点距離でヒットし、法線がレイと
    /// 逆向き(両面材質、モジュールdoc参照)であることを確認する。
    #[test]
    fn intersect_hits_the_analytic_plane_distance_with_a_normal_facing_the_ray() {
        // z=5 の平面上、x,y ともに [-1,1] の矩形。
        let quad = Quad::axis_aligned(2, 5.0, -1.0, 1.0, -1.0, 1.0);
        let ray = Ray::new(Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0));
        let hit = quad.intersect(&ray, 1e-6).expect("ray should hit the quad");

        assert!((hit.t - 5.0).abs() < 1e-12, "t={}", hit.t);
        assert!((hit.point.z - 5.0).abs() < 1e-12);
        // +z へ進むレイに対して法線は -z を向く。
        assert!(
            (hit.normal.z + 1.0).abs() < 1e-12,
            "normal must oppose the ray: {:?}",
            hit.normal
        );
    }

    /// 面の外側(矩形の範囲外)を通るレイはヒットしない——無限平面ではなく
    /// 有限のクアッドであることの検証。
    #[test]
    fn intersect_misses_when_the_hit_point_lies_outside_the_parallelogram() {
        let quad = Quad::axis_aligned(2, 5.0, -1.0, 1.0, -1.0, 1.0);
        // x=+3 を通るので z=5 平面上の交点は矩形の外。
        let ray = Ray::new(Vec3::new(3.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
        assert!(quad.intersect(&ray, 1e-6).is_none());
    }

    /// 背面から撃っても(両面材質として)ヒットし、法線はやはりレイと逆を向く。
    #[test]
    fn intersect_is_two_sided_and_flips_the_normal_for_backside_rays() {
        let quad = Quad::axis_aligned(2, 5.0, -1.0, 1.0, -1.0, 1.0);
        let ray = Ray::new(Vec3::new(0.0, 0.0, 10.0), Vec3::new(0.0, 0.0, -1.0));
        let hit = quad.intersect(&ray, 1e-6).expect("backside ray should hit");
        assert!((hit.t - 5.0).abs() < 1e-12);
        assert!(
            (hit.normal.z - 1.0).abs() < 1e-12,
            "normal must oppose the ray: {:?}",
            hit.normal
        );
    }

    /// レイと平行な面はヒットしない(ゼロ除算の回避)。
    #[test]
    fn intersect_returns_none_for_rays_parallel_to_the_quad() {
        let quad = Quad::axis_aligned(2, 5.0, -1.0, 1.0, -1.0, 1.0);
        let ray = Ray::new(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0));
        assert!(quad.intersect(&ray, 1e-6).is_none());
    }

    /// `t_min`より手前の交点は棄却する(シャドウレイの自己交差回避と同じ規約)。
    #[test]
    fn intersect_rejects_hits_closer_than_t_min() {
        let quad = Quad::axis_aligned(2, 5.0, -1.0, 1.0, -1.0, 1.0);
        let ray = Ray::new(Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0));
        assert!(quad.intersect(&ray, 6.0).is_none());
    }

    /// 面積・幾何法線が解析値と一致する(面光源の放射束換算で使うため)。
    #[test]
    fn area_and_geometric_normal_match_the_analytic_values() {
        let quad = Quad::axis_aligned(1, 2.0, 0.0, 3.0, 0.0, 4.0);
        assert!((quad.area() - 12.0).abs() < 1e-12, "area={}", quad.area());
        let n = quad.geometric_normal();
        assert!((n.length() - 1.0).abs() < 1e-12);
        assert!(n.y.abs() > 1.0 - 1e-12, "normal should be ±y: {n:?}");
    }
}
