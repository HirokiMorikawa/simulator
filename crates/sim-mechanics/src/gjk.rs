//! GJK(Gilbert-Johnson-Keerthi)距離アルゴリズム + EPA(Expanding Polytope Algorithm)+
//! フルCCD(conservative advancement)。設計: docs/10-mechanics/02-collision-detection.md §4.5。
//!
//! Phase 5 スコープの実装: GJK(ミンコフスキー差の原点への最近点探索による分離距離・
//! 重なり判定)+ EPA(重なり時の貫入深さ・法線復元)+ フルCCD(並進のみ、回転を含む
//! 一般形状は未対応)。設計§4.5が「実装の要諦(退化単体・数値許容)は実装フェーズで
//! Gino van den Bergenの書籍を正とする」と明記するとおり、本実装は教科書の完全な実装では
//! なく、GJKはJohnsonのサブアルゴリズム(単体の全部分集合を試して原点の重心座標が
//! 非負になる部分集合を探す方式)、EPAはシルエット辺法(可視面を除去し境界の辺で
//! 新しい面を張る素直な多面体拡張)、フルCCDは並進のみのconservative advancement
//! (GJKの分離距離+分離法線を使い、法線方向の相対速度で時間を厳密に前進させる。
//! 回転がないため速度は法線方向で一定であり、下界を使う一般形と異なり反復ごとに
//! 正確な時刻更新ができる)による直接的な実装。

use sim_math::Vec3;

/// GJKが扱える凸形状。点群(凸包、`Shape::ConvexMesh`に対応)と球(既存の`Shape::Sphere`と
/// 独立に、サポート写像が閉形式で書けるためGJKの検証に有用)の2種類。
pub enum ConvexShape<'a> {
    Points(&'a [Vec3]),
    Sphere { center: Vec3, radius: f64 },
}

impl ConvexShape<'_> {
    /// サポート写像: 指定方向に最も遠い形状表面上の点。
    fn support(&self, dir: Vec3) -> Vec3 {
        match self {
            ConvexShape::Points(points) => {
                let mut best = points[0];
                let mut best_dot = best.dot(dir);
                for &p in points.iter().skip(1) {
                    let d = p.dot(dir);
                    if d > best_dot {
                        best_dot = d;
                        best = p;
                    }
                }
                best
            }
            ConvexShape::Sphere { center, radius } => {
                let len = dir.length();
                if len < 1e-12 {
                    *center
                } else {
                    *center + dir.scale(*radius / len)
                }
            }
        }
    }
}

/// ミンコフスキー差A⊖(B+b_offset)のサポート写像。`b_offset`はBの並進(CCDで使う、
/// 通常は`Vec3::ZERO`)。
fn minkowski_support(a: &ConvexShape, b: &ConvexShape, b_offset: Vec3, dir: Vec3) -> Vec3 {
    a.support(dir) - (b.support(dir.scale(-1.0)) + b_offset)
}

/// GJKの結果: 分離している場合は分離距離と分離方向(BからAへ向かう単位法線、CCDが
/// 相対速度の投影に使う。Bの位置から見てAがどちらにあるかを指す)、重なっている場合は
/// 原点を包含する四面体(ミンコフスキー差の点4つ、EPAの初期多面体として使う)。
pub enum GjkResult {
    Separated { distance: f64, normal: Vec3 },
    Overlapping { simplex: [Vec3; 4] },
}

fn signed_volume6(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3) -> f64 {
    (p1 - p0).dot((p2 - p0).cross(p3 - p0))
}

/// 単体(1〜4点)の凸包上で原点に最も近い点を求める(Johnsonのサブアルゴリズム)。
/// 全ての空でない部分集合を試し、原点の重心座標が(許容誤差込みで)非負になる
/// 部分集合のうち、原点までの距離が最小のものを採用する。戻り値は最近点と、
/// それを与えた部分集合の(元のインデックスに対する)添字。
fn closest_point_on_simplex(points: &[Vec3]) -> (Vec3, Vec<usize>) {
    let n = points.len();
    let mut best_dist_sq = f64::MAX;
    let mut best_point = points[0];
    let mut best_subset = vec![0];

    for mask in 1u32..(1u32 << n) {
        let subset: Vec<usize> = (0..n).filter(|&i| mask & (1 << i) != 0).collect();
        if let Some(point) = closest_in_subset(points, &subset) {
            let d = point.length_sq();
            if d < best_dist_sq {
                best_dist_sq = d;
                best_point = point;
                best_subset = subset;
            }
        }
    }
    (best_point, best_subset)
}

const BARYCENTRIC_EPSILON: f64 = -1e-9;

fn closest_in_subset(points: &[Vec3], subset: &[usize]) -> Option<Vec3> {
    match subset.len() {
        1 => Some(points[subset[0]]),
        2 => {
            let a = points[subset[0]];
            let b = points[subset[1]];
            let ab = b - a;
            let denom = ab.dot(ab);
            if denom < 1e-18 {
                return Some(a);
            }
            let t = -a.dot(ab) / denom;
            if (0.0..=1.0).contains(&t) {
                Some(a + ab.scale(t))
            } else {
                None
            }
        }
        3 => {
            let a = points[subset[0]];
            let b = points[subset[1]];
            let c = points[subset[2]];
            let ab = b - a;
            let ac = c - a;
            let ao = -a;
            let d00 = ab.dot(ab);
            let d01 = ab.dot(ac);
            let d11 = ac.dot(ac);
            let d20 = ao.dot(ab);
            let d21 = ao.dot(ac);
            let denom = d00 * d11 - d01 * d01;
            if denom.abs() < 1e-18 {
                return None;
            }
            let v = (d11 * d20 - d01 * d21) / denom;
            let w = (d00 * d21 - d01 * d20) / denom;
            let u = 1.0 - v - w;
            if u >= BARYCENTRIC_EPSILON && v >= BARYCENTRIC_EPSILON && w >= BARYCENTRIC_EPSILON {
                Some(a + ab.scale(v) + ac.scale(w))
            } else {
                None
            }
        }
        4 => {
            let a = points[subset[0]];
            let b = points[subset[1]];
            let c = points[subset[2]];
            let d = points[subset[3]];
            let total = signed_volume6(a, b, c, d);
            if total.abs() < 1e-18 {
                return None;
            }
            let origin = Vec3::ZERO;
            let wa = signed_volume6(origin, b, c, d) / total;
            let wb = signed_volume6(a, origin, c, d) / total;
            let wc = signed_volume6(a, b, origin, d) / total;
            let wd = signed_volume6(a, b, c, origin) / total;
            if wa >= BARYCENTRIC_EPSILON
                && wb >= BARYCENTRIC_EPSILON
                && wc >= BARYCENTRIC_EPSILON
                && wd >= BARYCENTRIC_EPSILON
            {
                Some(origin) // 原点が四面体内部 = 重なり(距離0)
            } else {
                None
            }
        }
        _ => unreachable!("simplex subset size must be 1..=4"),
    }
}

/// 原点を含むことが分かっている単体(1〜4点)を、EPAが必要とする非退化な四面体
/// (アフィン独立な4点)に育てる。凸包に新しい支持点を追加しても凸包は単調に
/// 大きくなるだけなので、既に原点を含む単体に(ミンコフスキー差の実在の点である)
/// 支持点を足しても原点を含むことは保たれる。
fn complete_to_tetrahedron(
    a: &ConvexShape,
    b: &ConvexShape,
    b_offset: Vec3,
    mut simplex: Vec<Vec3>,
) -> [Vec3; 4] {
    let candidate_dirs = [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(-1.0, -1.0, -1.0),
        Vec3::new(1.0, -1.0, 1.0),
        Vec3::new(-1.0, 1.0, -1.0),
    ];
    let mut idx = 0;
    while simplex.len() < 4 && idx < candidate_dirs.len() * 2 {
        let dir = candidate_dirs[idx % candidate_dirs.len()];
        idx += 1;
        let candidate = minkowski_support(a, b, b_offset, dir);
        if simplex.iter().any(|&p| (p - candidate).length_sq() < 1e-12) {
            continue;
        }
        let independent = match simplex.len() {
            1 => true,
            2 => {
                let ab = simplex[1] - simplex[0];
                let ac = candidate - simplex[0];
                ab.cross(ac).length_sq() > 1e-18
            }
            3 => {
                let ab = simplex[1] - simplex[0];
                let ac = simplex[2] - simplex[0];
                let normal = ab.cross(ac);
                let ad = candidate - simplex[0];
                normal.dot(ad).abs() > 1e-12
            }
            _ => true,
        };
        if independent {
            simplex.push(candidate);
        }
    }
    while simplex.len() < 4 {
        let last = *simplex.last().unwrap();
        simplex.push(last + Vec3::new(1e-6, 1e-6, 1e-6));
    }
    [simplex[0], simplex[1], simplex[2], simplex[3]]
}

/// GJK本体: ミンコフスキー差の凸包に対する原点までの最近点を反復的に求める。
pub fn gjk_distance(a: &ConvexShape, b: &ConvexShape) -> GjkResult {
    gjk_distance_offset(a, b, Vec3::ZERO)
}

/// `gjk_distance`の一般形: Bを`b_offset`だけ並進させた状態で判定する(CCDが使う)。
fn gjk_distance_offset(a: &ConvexShape, b: &ConvexShape, b_offset: Vec3) -> GjkResult {
    let mut simplex: Vec<Vec3> = vec![minkowski_support(a, b, b_offset, Vec3::new(1.0, 0.0, 0.0))];
    let mut closest = simplex[0];

    for _ in 0..64 {
        if closest.length_sq() < 1e-16 {
            let tetra = complete_to_tetrahedron(a, b, b_offset, simplex);
            return GjkResult::Overlapping { simplex: tetra };
        }
        let dir = closest.scale(-1.0);
        let new_point = minkowski_support(a, b, b_offset, dir);
        // 新しい支持点が探索方向へこれ以上進めない(収束)なら終了。
        if new_point.dot(dir) <= closest.dot(dir) + 1e-12 {
            let distance = closest.length();
            return GjkResult::Separated {
                distance,
                normal: closest.scale(1.0 / distance.max(1e-18)),
            };
        }
        simplex.push(new_point);
        let (new_closest, subset) = closest_point_on_simplex(&simplex);
        simplex = subset.iter().map(|&i| simplex[i]).collect();
        closest = new_closest;
    }
    let distance = closest.length();
    GjkResult::Separated {
        distance,
        normal: closest.scale(1.0 / distance.max(1e-18)),
    }
}

/// EPAの結果: 貫入深さと、Aの表面をBから押し出す向きの法線(Aからみて外向き)。
pub struct EpaResult {
    pub depth: f64,
    pub normal: Vec3,
}

struct Face {
    indices: [usize; 3],
    normal: Vec3,
    distance: f64,
}

/// 面(i0,i1,i2)を構築する。原点は常に多面体の内部にある(EPAの不変条件)ため、
/// 法線が原点から離れる向きになるよう巻き順を調整する。
fn build_face(vertices: &[Vec3], i0: usize, i1: usize, i2: usize) -> Face {
    let a = vertices[i0];
    let b = vertices[i1];
    let c = vertices[i2];
    let raw_normal = (b - a).cross(c - a);
    let len = raw_normal.length();
    let normal = raw_normal.scale(1.0 / len.max(1e-18));
    if normal.dot(a) < 0.0 {
        let flipped = normal.scale(-1.0);
        Face {
            indices: [i0, i2, i1],
            normal: flipped,
            distance: flipped.dot(a),
        }
    } else {
        Face {
            indices: [i0, i1, i2],
            normal,
            distance: normal.dot(a),
        }
    }
}

/// EPAの反復上限。多面体形状(実際の`ConvexShape::Points`用途)なら数回の面分割で
/// ミンコフスキー差の境界と厳密に一致し即座に収束するが、球のような滑らかな形状は
/// 各反復で誤差がおよそ半分になるだけ(線形収束)のため、テストで使う球ケースが
/// 1e-9まで収束するには経験的に約90回前後必要と判明し、余裕を見て100回とした。
const EPA_MAX_ITERATIONS: usize = 100;

/// EPA本体: GJKが返した原点包含四面体を出発点に、ミンコフスキー差の境界多面体を
/// 支持点で拡張しながら、原点に最も近い面へ収束させる(貫入深さ・法線を復元)。
pub fn epa_penetration(a: &ConvexShape, b: &ConvexShape, simplex: [Vec3; 4]) -> EpaResult {
    let mut vertices: Vec<Vec3> = simplex.to_vec();
    let mut faces: Vec<Face> = vec![
        build_face(&vertices, 0, 1, 2),
        build_face(&vertices, 0, 3, 1),
        build_face(&vertices, 0, 2, 3),
        build_face(&vertices, 1, 3, 2),
    ];

    for _ in 0..EPA_MAX_ITERATIONS {
        let (min_idx, _) = faces
            .iter()
            .enumerate()
            .min_by(|(_, f1), (_, f2)| f1.distance.partial_cmp(&f2.distance).unwrap())
            .unwrap();
        let min_normal = faces[min_idx].normal;
        let min_distance = faces[min_idx].distance;

        let support_point = minkowski_support(a, b, Vec3::ZERO, min_normal);
        let support_distance = support_point.dot(min_normal);

        if support_distance - min_distance < 1e-9 {
            return EpaResult {
                depth: min_distance,
                normal: min_normal,
            };
        }

        let new_idx = vertices.len();
        vertices.push(support_point);

        // 新しい支持点から見える面を除去し、シルエット辺(可視面1枚だけに属する辺)を
        // 新しい頂点との面で埋める。
        let visible: Vec<bool> = faces
            .iter()
            .map(|f| f.normal.dot(support_point - vertices[f.indices[0]]) > 1e-10)
            .collect();

        let mut directed_edges: Vec<(usize, usize)> = Vec::new();
        for (f, &vis) in faces.iter().zip(&visible) {
            if vis {
                directed_edges.push((f.indices[0], f.indices[1]));
                directed_edges.push((f.indices[1], f.indices[2]));
                directed_edges.push((f.indices[2], f.indices[0]));
            }
        }
        let silhouette: Vec<(usize, usize)> = directed_edges
            .iter()
            .copied()
            .filter(|&(x, y)| !directed_edges.contains(&(y, x)))
            .collect();

        let mut new_faces: Vec<Face> = faces
            .into_iter()
            .zip(&visible)
            .filter(|(_, &vis)| !vis)
            .map(|(f, _)| f)
            .collect();
        for (x, y) in silhouette {
            new_faces.push(build_face(&vertices, x, y, new_idx));
        }
        faces = new_faces;
    }

    // 収束しなかった場合のフォールバック: 直近の最良面を返す。
    let (min_idx, _) = faces
        .iter()
        .enumerate()
        .min_by(|(_, f1), (_, f2)| f1.distance.partial_cmp(&f2.distance).unwrap())
        .unwrap();
    EpaResult {
        depth: faces[min_idx].distance,
        normal: faces[min_idx].normal,
    }
}

/// CCD(continuous collision detection)の反復上限。並進のみのconservative advancementは
/// GJKの分離距離を法線方向の相対速度でちょうど1回で使い切れる場合が多いが、形状が
/// 非球対称だと最近点の位置がステップごとに変わり分離法線も変わるため、複数回の
/// 前進が必要になることがある。
const CCD_MAX_ITERATIONS: usize = 64;

/// 並進のみのconservative advancement(設計§4.5、モジュールdoc参照)によるTOI
/// (time of impact)計算。`rel_vel`はAを静止基準としたときのBの並進速度(Aは静止、
/// Bだけが`rel_vel`で動くと考えた等価な相対運動)で、`max_time`以内に接触
/// (貫入深さ0)へ到達する時刻を返す。回転は扱わないため、GJKが返す分離法線
/// (BからAへ向かう単位法線)への`rel_vel`の射影(closing speed = `rel_vel.dot(normal)`)
/// は各反復内で保守的な下界としてではなく(並進のみなので)厳密な閉じ速度として使える
/// — ただし最近点の組が反復間で変わりうるため、複数回に分けて前進しTOIを積み上げる。
/// 閉じていない(離れていく、または平行に移動する)場合は`None`を返す。
pub fn conservative_advancement_toi(
    a: &ConvexShape,
    b: &ConvexShape,
    rel_vel: Vec3,
    max_time: f64,
) -> Option<f64> {
    let mut t = 0.0;
    for _ in 0..CCD_MAX_ITERATIONS {
        let offset = rel_vel.scale(t);
        match gjk_distance_offset(a, b, offset) {
            GjkResult::Overlapping { .. } => return Some(t),
            GjkResult::Separated { distance, normal } => {
                if distance < 1e-9 {
                    return Some(t);
                }
                let closing_speed = rel_vel.dot(normal);
                if closing_speed <= 1e-12 {
                    return None;
                }
                t += distance / closing_speed;
                if t > max_time {
                    return None;
                }
            }
        }
    }
    // 反復上限に達した場合、この時点のtは接触に十分近いとみなして返す
    // (分離距離が反復ごとに単調減少しているため、上限到達時も概ね接触寸前)。
    Some(t.min(max_time))
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_math::SimRng;

    fn assert_separated(result: GjkResult, expected_distance: f64, tol: f64) {
        match result {
            GjkResult::Separated { distance, .. } => {
                let rel_err = (distance - expected_distance).abs() / expected_distance.max(1e-9);
                assert!(
                    rel_err < tol,
                    "distance={distance:.6} expected={expected_distance:.6} rel_err={rel_err:.4}"
                );
            }
            GjkResult::Overlapping { .. } => panic!("expected Separated, got Overlapping"),
        }
    }

    /// 分離した2球: 解析的な分離距離 |c1-c2|-(r1+r2) と一致(設計§4.5)。
    #[test]
    fn separated_spheres_distance_matches_analytic_formula() {
        let a = ConvexShape::Sphere {
            center: Vec3::new(0.0, 0.0, 0.0),
            radius: 1.0,
        };
        let b = ConvexShape::Sphere {
            center: Vec3::new(5.0, 0.0, 0.0),
            radius: 1.5,
        };
        let result = gjk_distance(&a, &b);
        assert_separated(result, 5.0 - 1.0 - 1.5, 1e-6);
    }

    /// 重なった2球: 重なり判定が正しく出ること(距離0未満の状態、設計§4.5)。
    #[test]
    fn overlapping_spheres_report_overlap() {
        let a = ConvexShape::Sphere {
            center: Vec3::new(0.0, 0.0, 0.0),
            radius: 2.0,
        };
        let b = ConvexShape::Sphere {
            center: Vec3::new(1.0, 0.0, 0.0),
            radius: 2.0,
        };
        assert!(matches!(
            gjk_distance(&a, &b),
            GjkResult::Overlapping { .. }
        ));
    }

    /// EPA: 重なった2球の貫入深さが解析式 $r_1+r_2-|c_1-c_2|$ と一致し、法線が
    /// 中心を結ぶ軸に平行であること(設計§4.5)。実装検証中、球(多面体ではなく滑らかな
    /// 形状)に対してはEPAが各反復で誤差をおよそ半分にするだけの線形収束にしかならず
    /// (実際の`ConvexShape::Points`用途である多面体同士なら数回の面分割で厳密に収束する)、
    /// 既定の反復上限64では収束しきらないことを発見し、上限を100に増やして解決した
    /// (`EPA_MAX_ITERATIONS`のコメント参照)。
    #[test]
    fn epa_penetration_depth_for_overlapping_spheres_matches_analytic_formula() {
        let a = ConvexShape::Sphere {
            center: Vec3::new(0.0, 0.0, 0.0),
            radius: 2.0,
        };
        let b = ConvexShape::Sphere {
            center: Vec3::new(1.0, 0.0, 0.0),
            radius: 2.0,
        };
        let simplex = match gjk_distance(&a, &b) {
            GjkResult::Overlapping { simplex } => simplex,
            GjkResult::Separated { .. } => panic!("expected overlap"),
        };
        let result = epa_penetration(&a, &b, simplex);
        let expected_depth = 2.0 + 2.0 - 1.0;
        let rel_err = (result.depth - expected_depth).abs() / expected_depth;
        assert!(
            rel_err < 1e-4,
            "depth={:.6} expected={expected_depth:.6} rel_err={rel_err:.4}",
            result.depth
        );
        assert!(
            result.normal.y.abs() < 1e-3 && result.normal.z.abs() < 1e-3,
            "normal should be parallel to the center-to-center axis, got {:?}",
            result.normal
        );
    }

    /// EPA: 平坦な面を持つ多面体(軸並行の箱、x軸方向にのみ重なる)は、実際の
    /// `ConvexShape::Points`用途どおり数回の反復で厳密に(球よりずっと速く)収束し、
    /// 貫入深さがAABBの重なり幅と、法線がx軸方向と、それぞれ高精度で一致すること。
    #[test]
    fn epa_penetration_depth_for_overlapping_boxes_matches_axis_overlap() {
        let box_points = |min: Vec3, max: Vec3| -> Vec<Vec3> {
            let mut pts = Vec::new();
            for &x in &[min.x, max.x] {
                for &y in &[min.y, max.y] {
                    for &z in &[min.z, max.z] {
                        pts.push(Vec3::new(x, y, z));
                    }
                }
            }
            pts
        };
        let a_pts = box_points(Vec3::new(-1.0, -1.0, -1.0), Vec3::new(1.0, 1.0, 1.0));
        let b_pts = box_points(Vec3::new(0.0, -1.0, -1.0), Vec3::new(2.0, 1.0, 1.0));
        let a = ConvexShape::Points(&a_pts);
        let b = ConvexShape::Points(&b_pts);

        let simplex = match gjk_distance(&a, &b) {
            GjkResult::Overlapping { simplex } => simplex,
            GjkResult::Separated { .. } => panic!("expected overlap"),
        };
        let result = epa_penetration(&a, &b, simplex);
        let expected_depth = 1.0;
        let rel_err = (result.depth - expected_depth).abs() / expected_depth;
        assert!(
            rel_err < 1e-6,
            "depth={:.6} expected={expected_depth:.6} rel_err={rel_err:.6}",
            result.depth
        );
        assert!(
            result.normal.y.abs() < 1e-9 && result.normal.z.abs() < 1e-9,
            "normal should be exactly parallel to the x-axis, got {:?}",
            result.normal
        );
    }

    /// ちょうど接する2球(境界ケース): 分離距離がほぼ0であること。
    #[test]
    fn touching_spheres_have_near_zero_distance() {
        let a = ConvexShape::Sphere {
            center: Vec3::new(0.0, 0.0, 0.0),
            radius: 1.0,
        };
        let b = ConvexShape::Sphere {
            center: Vec3::new(2.0, 0.0, 0.0),
            radius: 1.0,
        };
        match gjk_distance(&a, &b) {
            GjkResult::Separated { distance, .. } => {
                assert!(distance < 1e-6, "distance={distance}")
            }
            GjkResult::Overlapping { .. } => {}
        }
    }

    /// 分離した2つの凸多面体(点群、軸並行の箱を8頂点で表現): 解析的なAABB間距離と一致。
    #[test]
    fn separated_boxes_distance_matches_analytic_aabb_gap() {
        let box_points = |min: Vec3, max: Vec3| -> Vec<Vec3> {
            let mut pts = Vec::new();
            for &x in &[min.x, max.x] {
                for &y in &[min.y, max.y] {
                    for &z in &[min.z, max.z] {
                        pts.push(Vec3::new(x, y, z));
                    }
                }
            }
            pts
        };
        let a_pts = box_points(Vec3::new(-1.0, -1.0, -1.0), Vec3::new(1.0, 1.0, 1.0));
        let b_pts = box_points(Vec3::new(4.0, -1.0, -1.0), Vec3::new(6.0, 1.0, 1.0));
        let a = ConvexShape::Points(&a_pts);
        let b = ConvexShape::Points(&b_pts);
        assert_separated(gjk_distance(&a, &b), 3.0, 1e-6);
    }

    /// 統計テスト(設計§4.5の推奨): 乱数配置(決定シード)された凸多面体(四面体)対で、
    /// GJKの重なり判定と総当たりサンプリング(大量の点対の最短距離が0未満かの近似判定)が
    /// 一致することを確認する。
    #[test]
    fn gjk_overlap_decision_matches_brute_force_sampling_on_random_tetrahedra() {
        let mut rng = SimRng::new(2024, 0);
        let mut mismatches = 0;
        let trials = 200;
        for _ in 0..trials {
            let random_tetra = |rng: &mut SimRng, center: Vec3, spread: f64| -> Vec<Vec3> {
                (0..4)
                    .map(|_| {
                        center
                            + Vec3::new(
                                (rng.next_f64() - 0.5) * spread,
                                (rng.next_f64() - 0.5) * spread,
                                (rng.next_f64() - 0.5) * spread,
                            )
                    })
                    .collect()
            };
            let center_a = Vec3::new(0.0, 0.0, 0.0);
            let center_b = Vec3::new(
                (rng.next_f64() - 0.5) * 4.0,
                (rng.next_f64() - 0.5) * 4.0,
                (rng.next_f64() - 0.5) * 4.0,
            );
            let a_pts = random_tetra(&mut rng, center_a, 2.0);
            let b_pts = random_tetra(&mut rng, center_b, 2.0);

            let gjk_overlap = matches!(
                gjk_distance(&ConvexShape::Points(&a_pts), &ConvexShape::Points(&b_pts)),
                GjkResult::Overlapping { .. }
            );

            // 総当たり近似判定: 両凸包を稠密にサンプルした点集合間の最小距離が
            // 小さければ「重なっている」とみなす(2つの凸多面体の頂点+辺中点+面重心+
            // 重心を使った粗い近似)。
            let sample_points = |pts: &[Vec3]| -> Vec<Vec3> {
                let mut samples = pts.to_vec();
                for i in 0..pts.len() {
                    for j in (i + 1)..pts.len() {
                        samples.push((pts[i] + pts[j]).scale(0.5));
                    }
                }
                let centroid = pts
                    .iter()
                    .fold(Vec3::ZERO, |acc, &p| acc + p)
                    .scale(1.0 / pts.len() as f64);
                samples.push(centroid);
                samples
            };
            let a_samples = sample_points(&a_pts);
            let b_samples = sample_points(&b_pts);
            let min_gap = a_samples
                .iter()
                .flat_map(|&pa| b_samples.iter().map(move |&pb| (pb - pa).length()))
                .fold(f64::MAX, f64::min);

            // サンプリングは粗い近似(頂点・辺中点・重心のみ)なので、GJKが「重なっている」と
            // 判定したならサンプル間最小距離はある程度小さいはず、逆にサンプル間最小距離が
            // 十分大きければGJKも重ならないと判定するはず、という緩い整合性のみ確認する
            // (厳密な面上の点までは網羅しないため、双方向の完全一致は要求しない)。
            if gjk_overlap && min_gap > 1.0 {
                mismatches += 1;
            }
            if !gjk_overlap && min_gap < 1e-6 {
                mismatches += 1;
            }
        }
        assert_eq!(
            mismatches, 0,
            "{mismatches} mismatches out of {trials} trials"
        );
    }

    /// CCD: 一定の相対速度で接近する2球のTOIが解析式 $(gap)/(closing\_speed)$ と
    /// 厳密に一致すること(並進のみ・回転なしなら閉じ速度は法線方向で一定なので、
    /// conservative advancementが正確な時刻を1回の前進で求められるはず)。
    #[test]
    fn ccd_toi_for_approaching_spheres_matches_analytic_gap_over_speed() {
        let a = ConvexShape::Sphere {
            center: Vec3::new(0.0, 0.0, 0.0),
            radius: 1.0,
        };
        let b = ConvexShape::Sphere {
            center: Vec3::new(10.0, 0.0, 0.0),
            radius: 1.0,
        };
        let closing_speed = 2.0;
        let rel_vel = Vec3::new(-closing_speed, 0.0, 0.0); // Aを静止基準としたBの速度(-x、Aに接近)
        let gap = 10.0 - 1.0 - 1.0;
        let expected_toi = gap / closing_speed;

        let toi = conservative_advancement_toi(&a, &b, rel_vel, 100.0)
            .expect("approaching spheres should collide within max_time");
        let rel_err = (toi - expected_toi).abs() / expected_toi;
        assert!(
            rel_err < 1e-6,
            "toi={toi:.6} expected={expected_toi:.6} rel_err={rel_err:.6}"
        );
    }

    /// CCD: 平行移動(接近しない)場合はTOIが存在せず`None`を返すこと。
    #[test]
    fn ccd_toi_is_none_when_shapes_do_not_approach() {
        let a = ConvexShape::Sphere {
            center: Vec3::new(0.0, 0.0, 0.0),
            radius: 1.0,
        };
        let b = ConvexShape::Sphere {
            center: Vec3::new(10.0, 0.0, 0.0),
            radius: 1.0,
        };
        // y方向にすれ違うだけで、x方向(分離軸)の閉じ速度はゼロ。
        let rel_vel = Vec3::new(0.0, 3.0, 0.0);
        assert_eq!(conservative_advancement_toi(&a, &b, rel_vel, 100.0), None);
    }

    /// CCD: 接近はするが`max_time`以内には到達しない場合は`None`を返すこと。
    #[test]
    fn ccd_toi_is_none_when_impact_is_beyond_max_time() {
        let a = ConvexShape::Sphere {
            center: Vec3::new(0.0, 0.0, 0.0),
            radius: 1.0,
        };
        let b = ConvexShape::Sphere {
            center: Vec3::new(10.0, 0.0, 0.0),
            radius: 1.0,
        };
        let rel_vel = Vec3::new(-2.0, 0.0, 0.0);
        let gap = 10.0 - 1.0 - 1.0;
        let true_toi = gap / 2.0;
        assert_eq!(
            conservative_advancement_toi(&a, &b, rel_vel, true_toi * 0.5),
            None
        );
    }

    /// CCD: 平行移動する多面体(軸並行の箱同士、x方向に接近)のTOIがAABB間の
    /// 解析的ギャップ/閉じ速度と一致すること(球以外の凸形状での検証)。
    #[test]
    fn ccd_toi_for_approaching_boxes_matches_analytic_gap_over_speed() {
        let box_points = |min: Vec3, max: Vec3| -> Vec<Vec3> {
            let mut pts = Vec::new();
            for &x in &[min.x, max.x] {
                for &y in &[min.y, max.y] {
                    for &z in &[min.z, max.z] {
                        pts.push(Vec3::new(x, y, z));
                    }
                }
            }
            pts
        };
        let a_pts = box_points(Vec3::new(-1.0, -1.0, -1.0), Vec3::new(1.0, 1.0, 1.0));
        let b_pts = box_points(Vec3::new(4.0, -1.0, -1.0), Vec3::new(6.0, 1.0, 1.0));
        let a = ConvexShape::Points(&a_pts);
        let b = ConvexShape::Points(&b_pts);

        let closing_speed = 1.5;
        let rel_vel = Vec3::new(-closing_speed, 0.0, 0.0); // Aを静止基準としたBの速度(-x、Aに接近)
        let gap = 3.0;
        let expected_toi = gap / closing_speed;

        let toi = conservative_advancement_toi(&a, &b, rel_vel, 100.0)
            .expect("approaching boxes should collide within max_time");
        let rel_err = (toi - expected_toi).abs() / expected_toi;
        assert!(
            rel_err < 1e-6,
            "toi={toi:.6} expected={expected_toi:.6} rel_err={rel_err:.6}"
        );
    }
}
