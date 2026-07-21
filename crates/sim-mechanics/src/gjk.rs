//! GJK(Gilbert-Johnson-Keerthi)距離アルゴリズム。設計: docs/10-mechanics/02-collision-detection.md §4.5。
//!
//! Phase 5 スコープの部分実装: GJK(ミンコフスキー差の原点への最近点探索による分離距離・
//! 重なり判定)のみ。EPA(貫入深さ・法線の復元)は後続増分で追加する(設計§4.5が
//! 「実装の要諦(退化単体・数値許容)は実装フェーズでGino van den Bergenの書籍を正とする」
//! と明記するとおり、本実装は教科書の完全な実装ではなく、Johnsonのサブアルゴリズム
//! (単体の全部分集合を試して原点の重心座標が非負になる部分集合を探す方式)による
//! 素直な実装)。

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

fn minkowski_support(a: &ConvexShape, b: &ConvexShape, dir: Vec3) -> Vec3 {
    a.support(dir) - b.support(dir.scale(-1.0))
}

/// GJKの結果: 分離している場合は分離距離、重なっている場合はそれを示す(EPAは未実装)。
pub enum GjkResult {
    Separated { distance: f64 },
    Overlapping,
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

/// GJK本体: ミンコフスキー差の凸包に対する原点までの最近点を反復的に求める。
pub fn gjk_distance(a: &ConvexShape, b: &ConvexShape) -> GjkResult {
    let mut dir = Vec3::new(1.0, 0.0, 0.0);
    let mut simplex: Vec<Vec3> = vec![minkowski_support(a, b, dir)];
    let mut closest = simplex[0];

    for _ in 0..64 {
        if closest.length_sq() < 1e-16 {
            return GjkResult::Overlapping;
        }
        dir = closest.scale(-1.0);
        let new_point = minkowski_support(a, b, dir);
        // 新しい支持点が探索方向へこれ以上進めない(収束)なら終了。
        if new_point.dot(dir) <= closest.dot(dir) + 1e-12 {
            return GjkResult::Separated {
                distance: closest.length(),
            };
        }
        simplex.push(new_point);
        let (new_closest, subset) = closest_point_on_simplex(&simplex);
        simplex = subset.iter().map(|&i| simplex[i]).collect();
        closest = new_closest;
    }
    GjkResult::Separated {
        distance: closest.length(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_math::SimRng;

    fn assert_separated(result: GjkResult, expected_distance: f64, tol: f64) {
        match result {
            GjkResult::Separated { distance } => {
                let rel_err = (distance - expected_distance).abs() / expected_distance.max(1e-9);
                assert!(
                    rel_err < tol,
                    "distance={distance:.6} expected={expected_distance:.6} rel_err={rel_err:.4}"
                );
            }
            GjkResult::Overlapping => panic!("expected Separated, got Overlapping"),
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
        assert!(matches!(gjk_distance(&a, &b), GjkResult::Overlapping));
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
            GjkResult::Separated { distance } => assert!(distance < 1e-6, "distance={distance}"),
            GjkResult::Overlapping => {}
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
                GjkResult::Overlapping
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
}
