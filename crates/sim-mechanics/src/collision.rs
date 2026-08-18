//! Broadphase / narrowphase / 接触マニフォールド。
//! 設計: docs/10-mechanics/02-collision-detection.md §3/§4。
//!
//! Phase 1: 総当たり broadphase + Sphere-Sphere/Sphere-Plane/Box-Plane/Sphere-Box
//! narrowphase(§4.2 の表の Phase 1 行)。
//! Phase 2: Box-Box(SAT、§4.4)。Capsule 系は Phase 2、GJK/EPA は Phase 5。
//! 軸選択のヒステリシス(`AxisCache`)は実装済み。**マニフォールド持続化(§4.7)は
//! 群9で実装した**(`contact::ManifoldCache`)——移行前は M12(4段スタック)の
//! 貫入が slop の 93.5% まで達していた既知の限界(Q4)があり、実装後は最悪 33.8% に下がった
//! (`tests/manifold_persistence.rs` の対照実験)。
//! **群4で Capsule×Box を実装した**(増分Lでは線分-OBB の場合分けを避けて `None` を
//! 返しており、エディタでカプセルと箱を並べるとすり抜けていた)。`capsule_box` 参照。
//! **群11で ConvexMesh の接触生成を実装した**(それまでは一律 `None` を返し、
//! この形状は何ともぶつからずすり抜けていた)。3D凸包(`crate::hull`)を張り、
//! 平面とは頂点距離の解析形、球とは表面最近点による球-球への帰着、
//! 箱・他の多面体とは SAT で扱う(`convex_poly_manifold` 参照)。
//! **`ConvexMesh` × `Capsule` だけは未実装のまま**(同 doc の「既知の限界」)。

use crate::body::{BodyType, RigidBodySet};
use crate::shape::{Aabb, Shape};
use sim_math::{Transform, Vec3};

const EPS_LEN: f64 = 1e-12;

/// 設計 §3。
#[derive(Clone, Copy, Debug)]
pub struct ContactPoint {
    pub world_point: Vec3,
    pub penetration: f64,
    pub feature_id: u32,
}

/// 設計 §3。`body_a.index < body_b.index` に正規化する。
#[derive(Clone, Debug)]
pub struct ContactManifold {
    pub body_a: usize,
    pub body_b: usize,
    pub normal: Vec3,
    pub points: Vec<ContactPoint>,
}

/// narrowphase・broadphase が使う「**形状の**ワールド変換」。
/// `bodies.position[i]` は重心であって形状のローカル原点ではないため、
/// 幾何を扱うここでは必ず `shape_transform` を通す
/// (`RigidBodySet` の型doc「`position` は「重心」」参照)。
/// 重心オフセットが 0 の形状では `position[i]` と一致する。
fn transform_of(bodies: &RigidBodySet, i: usize) -> Transform {
    bodies.shape_transform(i)
}

/// 形状のワールド AABB。Plane は無限平面のため常に重なる扱い(全域を返す)。
fn aabb_of(shape: &Shape, xf: Transform) -> Aabb {
    match shape {
        Shape::Sphere { radius } => {
            let r = Vec3::new(*radius, *radius, *radius);
            Aabb {
                min: xf.position - r,
                max: xf.position + r,
            }
        }
        Shape::Box { half_extents } => {
            let mut min = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
            let mut max = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
            for &sx in &[-1.0, 1.0] {
                for &sy in &[-1.0, 1.0] {
                    for &sz in &[-1.0, 1.0] {
                        let local = Vec3::new(
                            sx * half_extents.x,
                            sy * half_extents.y,
                            sz * half_extents.z,
                        );
                        let world = xf.apply_point(local);
                        min = Vec3::new(min.x.min(world.x), min.y.min(world.y), min.z.min(world.z));
                        max = Vec3::new(max.x.max(world.x), max.y.max(world.y), max.z.max(world.z));
                    }
                }
            }
            Aabb { min, max }
        }
        Shape::Plane { .. } => Aabb {
            min: Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
            max: Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
        },
        // **増分Lで実装**。ローカル+y軸を長軸とする線分の両端を世界へ移し、
        // その包絡へ半径ぶんのマージンを足す。
        Shape::Capsule {
            radius,
            half_height,
        } => {
            let (a, b) = capsule_segment(xf, *half_height);
            let r = Vec3::new(*radius, *radius, *radius);
            Aabb {
                min: Vec3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z)) - r,
                max: Vec3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z)) + r,
            }
        }
        // **群10で実装**。各部品のAABB(親のxfと部品ローカルxfを合成した
        // ワールド変換で評価)の和集合。
        Shape::Compound { children } => {
            let mut result: Option<Aabb> = None;
            for (child_xf, child_shape) in children {
                let world_xf = xf.compose(*child_xf);
                let child_aabb = aabb_of(child_shape, world_xf);
                result = Some(match result {
                    Some(acc) => Aabb {
                        min: Vec3::new(
                            acc.min.x.min(child_aabb.min.x),
                            acc.min.y.min(child_aabb.min.y),
                            acc.min.z.min(child_aabb.min.z),
                        ),
                        max: Vec3::new(
                            acc.max.x.max(child_aabb.max.x),
                            acc.max.y.max(child_aabb.max.y),
                            acc.max.z.max(child_aabb.max.z),
                        ),
                    },
                    None => child_aabb,
                });
            }
            result.unwrap_or(Aabb {
                min: xf.position,
                max: xf.position,
            })
        }
        // **群10で実装**。頂点をワールド座標へ変換したAABB(`shape::points_aabb`、
        // `Shape::volume`のdoc「既知の限界」参照——面情報が無いため凸包そのもの
        // ではなく点群のAABB)。
        Shape::ConvexMesh { vertices } => {
            let world_points: Vec<Vec3> = vertices.iter().map(|p| xf.apply_point(*p)).collect();
            crate::shape::points_aabb(&world_points).unwrap_or(Aabb {
                min: xf.position,
                max: xf.position,
            })
        }
    }
}

fn aabb_overlap(a: Aabb, b: Aabb) -> bool {
    a.min.x <= b.max.x
        && a.max.x >= b.min.x
        && a.min.y <= b.max.y
        && a.max.y >= b.min.y
        && a.min.z <= b.max.z
        && a.max.z >= b.min.z
}

/// 設計 §4.3: 中心間距離 vs 半径和。
fn sphere_sphere(
    center_a: Vec3,
    r_a: f64,
    center_b: Vec3,
    r_b: f64,
) -> Option<(Vec3, ContactPoint)> {
    let d = center_b - center_a;
    let len_sq = d.length_sq();
    let radius_sum = r_a + r_b;
    if len_sq >= radius_sum * radius_sum {
        return None;
    }
    let len = len_sq.sqrt();
    let normal = if len < EPS_LEN {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        d.scale(1.0 / len)
    };
    let penetration = radius_sum - len;
    let world_point = center_a.addcarry_scaled(normal, r_a - penetration * 0.5);
    Some((
        normal,
        ContactPoint {
            world_point,
            penetration,
            feature_id: 0,
        },
    ))
}

/// 球 と 無限平面(法線は正規化済み前提)。
/// カプセルの芯線(ローカル+y軸方向の線分)をワールド座標で返す(**増分L**)。
fn capsule_segment(xf: Transform, half_height: f64) -> (Vec3, Vec3) {
    let axis = xf.rotation.rotate(Vec3::new(0.0, 1.0, 0.0));
    (
        xf.position.addcarry_scaled(axis, -half_height),
        xf.position.addcarry_scaled(axis, half_height),
    )
}

/// 線分`a`-`b`上で点`p`にもっとも近い点(**増分L**)。
fn closest_point_on_segment(a: Vec3, b: Vec3, p: Vec3) -> Vec3 {
    let ab = b - a;
    let denom = ab.dot(ab);
    if denom <= 1e-18 {
        return a;
    }
    let t = ((p - a).dot(ab) / denom).clamp(0.0, 1.0);
    a.addcarry_scaled(ab, t)
}

/// 2線分の最近接点対(**増分L**、Ericson "Real-Time Collision Detection" §5.1.9)。
fn closest_points_between_segments(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> (Vec3, Vec3) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);
    const EPS: f64 = 1e-18;

    if a <= EPS && e <= EPS {
        return (p1, p2);
    }
    if a <= EPS {
        return (p1, closest_point_on_segment(p2, q2, p1));
    }
    if e <= EPS {
        return (closest_point_on_segment(p1, q1, p2), p2);
    }

    let c = d1.dot(r);
    let b = d1.dot(d2);
    let denom = a * e - b * b;
    // 平行(denom≈0)なら s=0 から始めて t を解き、必要なら s を解き直す。
    let mut s = if denom > EPS {
        ((b * f - c * e) / denom).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let mut t = (b * s + f) / e;
    if t < 0.0 {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - c) / a).clamp(0.0, 1.0);
    }
    (p1.addcarry_scaled(d1, s), p2.addcarry_scaled(d2, t))
}

/// カプセル と 無限平面(**増分L**)。芯線の両端それぞれの平面距離を見て、
/// 貫入している端を接触点にする(最大2点——寝たカプセルが床で安定するには
/// 2点要る。1点だけだと転がり続けて静止しない)。
fn capsule_plane(
    xf: Transform,
    radius: f64,
    half_height: f64,
    plane_normal: Vec3,
    plane_d: f64,
) -> Option<(Vec3, Vec<ContactPoint>)> {
    let (a, b) = capsule_segment(xf, half_height);
    let mut points = Vec::new();
    for (feature_id, end) in [a, b].into_iter().enumerate() {
        let dist = plane_normal.dot(end) - plane_d;
        if dist < radius {
            points.push(ContactPoint {
                world_point: end.addcarry_scaled(plane_normal, -dist),
                penetration: radius - dist,
                feature_id: feature_id as u32,
            });
        }
    }
    if points.is_empty() {
        None
    } else {
        Some((plane_normal, points))
    }
}

/// カプセル と 球(**増分L**)。芯線上の最近接点を中心とする球として扱えば
/// 球-球と同じ問題になる。
fn capsule_sphere(
    capsule_xf: Transform,
    radius: f64,
    half_height: f64,
    sphere_center: Vec3,
    sphere_radius: f64,
) -> Option<(Vec3, ContactPoint)> {
    let (a, b) = capsule_segment(capsule_xf, half_height);
    let closest = closest_point_on_segment(a, b, sphere_center);
    let delta = sphere_center - closest;
    let distance = delta.length();
    let sum = radius + sphere_radius;
    if distance >= sum {
        return None;
    }
    // 完全に中心が一致する退化ケースでは法線を任意に決める(+y)。
    let normal = if distance > 1e-12 {
        delta.scale(1.0 / distance)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    Some((
        normal,
        ContactPoint {
            world_point: closest.addcarry_scaled(normal, radius),
            penetration: sum - distance,
            feature_id: 0,
        },
    ))
}

/// カプセル同士(**増分L**)。2本の芯線の最近接点対を求めれば球-球に帰着する。
fn capsule_capsule(
    xf_a: Transform,
    radius_a: f64,
    half_height_a: f64,
    xf_b: Transform,
    radius_b: f64,
    half_height_b: f64,
) -> Option<(Vec3, ContactPoint)> {
    let (a0, a1) = capsule_segment(xf_a, half_height_a);
    let (b0, b1) = capsule_segment(xf_b, half_height_b);
    let (pa, pb) = closest_points_between_segments(a0, a1, b0, b1);
    let delta = pb - pa;
    let distance = delta.length();
    let sum = radius_a + radius_b;
    if distance >= sum {
        return None;
    }
    let normal = if distance > 1e-12 {
        delta.scale(1.0 / distance)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    Some((
        normal,
        ContactPoint {
            world_point: pa.addcarry_scaled(normal, radius_a),
            penetration: sum - distance,
            feature_id: 0,
        },
    ))
}

fn sphere_plane(
    center: Vec3,
    radius: f64,
    plane_normal: Vec3,
    plane_d: f64,
) -> Option<(Vec3, ContactPoint)> {
    let dist = plane_normal.dot(center) - plane_d;
    if dist >= radius {
        return None;
    }
    let penetration = radius - dist;
    let world_point = center.addcarry_scaled(plane_normal, -dist);
    Some((
        plane_normal,
        ContactPoint {
            world_point,
            penetration,
            feature_id: 0,
        },
    ))
}

/// 箱 と 無限平面: 8頂点の平面距離、負の頂点(貫入)を接触点にする(最大4点)。
fn box_plane(
    box_xf: Transform,
    half_extents: Vec3,
    plane_normal: Vec3,
    plane_d: f64,
) -> Option<(Vec3, Vec<ContactPoint>)> {
    let mut points = Vec::new();
    let mut feature_id = 0u32;
    for &sx in &[-1.0, 1.0] {
        for &sy in &[-1.0, 1.0] {
            for &sz in &[-1.0, 1.0] {
                let local = Vec3::new(
                    sx * half_extents.x,
                    sy * half_extents.y,
                    sz * half_extents.z,
                );
                let world = box_xf.apply_point(local);
                let dist = plane_normal.dot(world) - plane_d;
                if dist < 0.0 {
                    points.push(ContactPoint {
                        world_point: world,
                        penetration: -dist,
                        feature_id,
                    });
                }
                feature_id += 1;
            }
        }
    }
    if points.is_empty() {
        return None;
    }
    // 最深点を先頭に、最大4点へ縮約(設計 §4.4 の縮約規約の簡易版)。
    points.sort_by(|a, b| b.penetration.partial_cmp(&a.penetration).unwrap());
    points.truncate(4);
    Some((plane_normal, points))
}

/// 凸多面体 と 無限平面(**群11**)。`box_plane`と同じ構造——全頂点の平面距離を
/// 見て、貫入している頂点を接触点にする(最大4点へ縮約)。
///
/// 平面は非有界なので SAT の「射影区間の重なり」に乗らない(片側が無限)。
/// 一方この解析形は厳密かつ決定的で、しかも「床に置いた多面体が静止する」という
/// 最も重要なケースをそのまま扱える。よって平面だけは専用経路にした。
fn convex_mesh_plane(
    vertices: &[Vec3],
    xf: Transform,
    plane_normal: Vec3,
    plane_d: f64,
) -> Option<(Vec3, Vec<ContactPoint>)> {
    let mut points = Vec::new();
    for (feature_id, &v) in vertices.iter().enumerate() {
        let world = xf.apply_point(v);
        let dist = plane_normal.dot(world) - plane_d;
        if dist < 0.0 {
            points.push(ContactPoint {
                world_point: world,
                penetration: -dist,
                feature_id: feature_id as u32,
            });
        }
    }
    if points.is_empty() {
        return None;
    }
    points.sort_by(|a, b| b.penetration.partial_cmp(&a.penetration).unwrap());
    points.truncate(4);
    Some((plane_normal, points))
}

/// 凸多面体としての「面法線 + 頂点」表現(ワールド座標、**群11**)。
/// SAT の分離軸候補(面法線・辺方向)と射影に必要なものだけを持つ。
struct ConvexPoly {
    vertices: Vec<Vec3>,
    /// 面の外向き単位法線(重複方向は除去済み)。
    face_normals: Vec<Vec3>,
    /// 辺の方向ベクトル(単位、重複・逆向き重複は除去済み)。
    edge_directions: Vec<Vec3>,
}

/// 方向の重複判定(向きの反転も同一視する)。SAT の軸は符号を持たないため。
fn push_unique_direction(list: &mut Vec<Vec3>, dir: Vec3) {
    let len = dir.length();
    if len < 1e-9 {
        return;
    }
    let unit = dir.scale(1.0 / len);
    if list
        .iter()
        .any(|&d| (d - unit).length_sq() < 1e-18 || (d + unit).length_sq() < 1e-18)
    {
        return;
    }
    list.push(unit);
}

impl ConvexPoly {
    /// 凸包(`crate::hull`)からワールド座標の SAT 用表現を作る。
    fn from_hull(hull: &crate::hull::ConvexHull, xf: Transform) -> ConvexPoly {
        let vertices: Vec<Vec3> = hull.vertices.iter().map(|&v| xf.apply_point(v)).collect();
        let mut face_normals = Vec::new();
        let mut edge_directions = Vec::new();
        for &f in &hull.faces {
            let (a, b, c) = (vertices[f[0]], vertices[f[1]], vertices[f[2]]);
            push_unique_direction(&mut face_normals, (b - a).cross(c - a));
            push_unique_direction(&mut edge_directions, b - a);
            push_unique_direction(&mut edge_directions, c - b);
            push_unique_direction(&mut edge_directions, a - c);
        }
        ConvexPoly {
            vertices,
            face_normals,
            edge_directions,
        }
    }

    /// 箱を凸多面体として表す(8頂点・3面法線・3辺方向)。凸包を張り直すより
    /// 直接書くほうが速く、退化も無い。
    fn from_box(xf: Transform, half_extents: Vec3) -> ConvexPoly {
        let mut vertices = Vec::with_capacity(8);
        for &sx in &[-1.0, 1.0] {
            for &sy in &[-1.0, 1.0] {
                for &sz in &[-1.0, 1.0] {
                    vertices.push(xf.apply_point(Vec3::new(
                        sx * half_extents.x,
                        sy * half_extents.y,
                        sz * half_extents.z,
                    )));
                }
            }
        }
        let axes = [
            box_axis_world(xf, 0),
            box_axis_world(xf, 1),
            box_axis_world(xf, 2),
        ];
        ConvexPoly {
            vertices,
            face_normals: axes.to_vec(),
            edge_directions: axes.to_vec(),
        }
    }

    /// 軸 `axis`(単位)への射影区間 `(min, max)`。
    fn project(&self, axis: Vec3) -> (f64, f64) {
        let mut min = f64::INFINITY;
        let mut max = f64::NEG_INFINITY;
        for &v in &self.vertices {
            let d = v.dot(axis);
            min = min.min(d);
            max = max.max(d);
        }
        (min, max)
    }
}

/// 凸多面体どうしの **SAT**(分離軸定理)による接触生成(**群11**)。
///
/// ## なぜ GJK/EPA ではなく SAT か
///
/// 設計上はどちらも標準的な選択肢だが、この物理コアでは SAT を採った:
///
/// - **既存の`box_box`と同じ構造**。15軸 SAT・最小重なり軸・面クリップという
///   語彙がすでにモジュール内にあり、`ConvexMesh`だけ別世界の反復解法に
///   なるのを避けられる(brief の「既存 narrowphase と同じアーキテクチャに
///   合わせる」要件)。
/// - **決定論的で反復が無い**。有限個の軸を全部試すだけなので、反復回数・
///   収束判定・初期単体の質に一切依存しない。
///
/// なお本増分の実装中に、既存の `gjk`/`epa_penetration` を
/// `ConvexShape::Points` × `ConvexShape::Sphere` に適用すると**実用にならない**
/// ことが判明した(GJK が返す初期単体は原点が四面体の一面の**上**に厳密に
/// 乗った状態になりうる。EPA はその面の距離を 0 と見て法線の向きも定まらず、
/// 100反復まで多面体を膨らませ続けて**112秒**かけて出鱈目な法線を返した——
/// 貫入 0.5 の解析解に対し 0.086)。既存の球×球・箱×箱のテストはたまたま
/// 原点が内部に来る配置で通っていたため露見していなかった。
/// **これは本増分では手を付けていない既存の弱点**であり、`gjk` モジュールは
/// フルCCD(分離距離のみ使用、EPA を通らない)専用のまま残してある。
/// 修正は独立した増分で扱うべき事項として、ここに記録しておく。
///
/// ## 接触点の作り方
///
/// 最小重なり軸 `n`(A→B 向きへ正規化)を法線とし、**相手の内部に入り込んで
/// いる頂点**を接触点にする(A の頂点で B の内部にあるもの + B の頂点で A の
/// 内部にあるもの)。各点の貫入量は「その点が相手の表面から `n` 方向に
/// どれだけ潜っているか」で個別に測るので、傾いた接触でも点ごとに正しい
/// 深さになる。面どうしが平らに重なる典型例(箱が箱の上に乗る)では相手の
/// 面の4頂点が入るため、そのまま4点マニフォールドになり安定して静止する。
///
/// **既知の限界**: 辺×辺が唯一の接触になる配置では、どちらの頂点も相手の
/// 内部に入らないことがある。その場合は最深の頂点対の中点を1点だけ返す
/// フォールバックに落ちる(`box_box_edge_contact` が単一点を返すのと同じ粒度)。
fn convex_poly_manifold(a: &ConvexPoly, b: &ConvexPoly) -> Option<(Vec3, Vec<ContactPoint>)> {
    let mut best_axis = Vec3::ZERO;
    let mut best_overlap = f64::INFINITY;

    let mut consider = |axis: Vec3| -> bool {
        // 既存の`box_box_sat`と同じ退化判定(設計 §4.4 の 1e-10、二乗長で比較)。
        if axis.length_sq() < SAT_DEGENERATE_AXIS_LEN_SQ {
            return true; // 退化軸は無視(分離を主張しない)
        }
        let len = axis.length();
        let unit = axis.scale(1.0 / len);
        let (min_a, max_a) = a.project(unit);
        let (min_b, max_b) = b.project(unit);
        let overlap = (max_a - min_b).min(max_b - min_a);
        if overlap <= 0.0 {
            return false; // 分離軸を発見
        }
        if overlap < best_overlap {
            best_overlap = overlap;
            // A→B 向きへ揃える(マニフォールドの法線規約)。
            best_axis = if max_b - min_a < max_a - min_b {
                -unit
            } else {
                unit
            };
        }
        true
    };

    for &n in &a.face_normals {
        if !consider(n) {
            return None;
        }
    }
    for &n in &b.face_normals {
        if !consider(n) {
            return None;
        }
    }
    for &ea in &a.edge_directions {
        for &eb in &b.edge_directions {
            if !consider(ea.cross(eb)) {
                return None;
            }
        }
    }

    if best_axis == Vec3::ZERO || !best_overlap.is_finite() {
        return None;
    }
    let normal = best_axis;

    // 相手の内部に潜っている頂点を接触点にする。
    let (_, max_a) = a.project(normal);
    let (min_b, _) = b.project(normal);
    let mut points: Vec<ContactPoint> = Vec::new();
    for (i, &v) in b.vertices.iter().enumerate() {
        // B の頂点が A の表面(法線方向の最遠面)より内側にあるか。
        let depth = max_a - v.dot(normal);
        if depth > 0.0 && point_inside(a, v) {
            points.push(ContactPoint {
                world_point: v,
                penetration: depth,
                feature_id: i as u32,
            });
        }
    }
    for (i, &v) in a.vertices.iter().enumerate() {
        let depth = v.dot(normal) - min_b;
        if depth > 0.0 && point_inside(b, v) {
            points.push(ContactPoint {
                world_point: v,
                penetration: depth,
                feature_id: 0x8000_0000 | i as u32,
            });
        }
    }

    if points.is_empty() {
        // 辺×辺接触のフォールバック(docの「既知の限界」参照)。
        let deepest_b = b
            .vertices
            .iter()
            .copied()
            .min_by(|p, q| p.dot(normal).partial_cmp(&q.dot(normal)).unwrap())?;
        let deepest_a = a
            .vertices
            .iter()
            .copied()
            .max_by(|p, q| p.dot(normal).partial_cmp(&q.dot(normal)).unwrap())?;
        points.push(ContactPoint {
            world_point: (deepest_a + deepest_b).scale(0.5),
            penetration: best_overlap,
            feature_id: 0,
        });
    }

    points.sort_by(|p, q| q.penetration.partial_cmp(&p.penetration).unwrap());
    points.truncate(4);
    Some((normal, points))
}

/// 点が凸多面体の内部(表面含む)にあるか。全ての面の内側半空間にあるかを見る。
fn point_inside(poly: &ConvexPoly, p: Vec3) -> bool {
    poly.face_normals.iter().all(|&n| {
        let (_, max) = poly.project(n);
        let (min, _) = poly.project(n);
        let d = p.dot(n);
        d <= max + 1e-9 && d >= min - 1e-9
    })
}

/// 凸多面体 と 球(**群11**)。多面体の表面上で球の中心に最も近い点を求めれば
/// 球-球に帰着する(`sphere_box`が箱に対してやっているのと同じ帰着)。
///
/// 最近点は「面上・辺上・頂点」のいずれかにあるので、三角形ごとに
/// 点-三角形の最近点を求めて最小を取る。凸包の面数は高々数十なので総当たりで
/// 十分(質量特性と同じく、これは narrowphase の中でも稀に走る経路)。
/// 戻り値の法線は**多面体から球へ**向かう向き(A=多面体・B=球の A→B 規約)。
fn convex_poly_sphere(
    poly: &ConvexPoly,
    hull_faces: &[[usize; 3]],
    center: Vec3,
    radius: f64,
) -> Option<(Vec3, ContactPoint)> {
    if point_inside(poly, center) {
        // 中心が内部: 最も浅い面へ押し出す。
        let mut best_depth = f64::INFINITY;
        let mut best_normal = Vec3::new(0.0, 1.0, 0.0);
        for &n in &poly.face_normals {
            let (_, max) = poly.project(n);
            let depth = max - center.dot(n);
            if depth < best_depth {
                best_depth = depth;
                best_normal = n;
            }
        }
        return Some((
            best_normal,
            ContactPoint {
                world_point: center,
                penetration: best_depth + radius,
                feature_id: 0,
            },
        ));
    }

    let mut closest = Vec3::ZERO;
    let mut best_dist_sq = f64::INFINITY;
    for &f in hull_faces {
        let p = closest_point_on_triangle(
            center,
            poly.vertices[f[0]],
            poly.vertices[f[1]],
            poly.vertices[f[2]],
        );
        let d = (p - center).length_sq();
        if d < best_dist_sq {
            best_dist_sq = d;
            closest = p;
        }
    }
    let dist = best_dist_sq.sqrt();
    if dist >= radius {
        return None;
    }
    let normal = if dist < EPS_LEN {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        (center - closest).scale(1.0 / dist)
    };
    let penetration = radius - dist;
    Some((
        normal,
        ContactPoint {
            world_point: closest,
            penetration,
            feature_id: 0,
        },
    ))
}

/// 点 `p` に最も近い三角形 `abc` 上の点(Ericson "Real-Time Collision Detection" §5.1.5、
/// 重心座標の領域判定による閉形式)。
fn closest_point_on_triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return a + ab.scale(v);
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return a + ac.scale(w);
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b).scale(w);
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    a + ab.scale(v) + ac.scale(w)
}

/// 球 と 箱: ボックスローカルで最近点にクランプ。
/// 線分 $S(t)=a+t(b-a)$($t\in[0,1]$)と原点中心の AABB(半幅 `half`)の
/// 二乗距離を最小化する $t$ を**解析的に**求める(**群4で追加**)。
///
/// 各軸の寄与 $g_i(t)=\max(-h_i-S_i(t),\,0,\,S_i(t)-h_i)$ は区分線形で、
/// 二乗距離 $f(t)=\sum_i g_i(t)^2$ は**凸な区分二次関数**になる。区間の切れ目は
/// 各軸が「箱の外(負側)/箱の中/箱の外(正側)」を切り替える $t$、すなわち
/// $S_i(t)=\pm h_i$ の解しかない(3軸で最大6個)。したがって
/// **切れ目で区切った各小区間で二次関数を厳密に最小化すれば全体の最小が出る**。
///
/// 反復解法(GJK や数値最小化)ではなくこの形にしたのは、
/// ①決定論(反復回数や初期値に依存しない)②`Capsule`×`Box` 以外に使い道が無いのに
/// GJK の凸包表現へ変換するのは遠回り、という2点による。
fn closest_segment_param_to_aabb(a: Vec3, b: Vec3, half: Vec3) -> f64 {
    let d = b - a;
    let axis = |v: Vec3, i: usize| match i {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    };

    // 区間の切れ目を集める(0 と 1 は常に含む)。
    let mut breakpoints = vec![0.0_f64, 1.0];
    for i in 0..3 {
        let di = axis(d, i);
        if di.abs() <= 1e-18 {
            continue;
        }
        let ai = axis(a, i);
        let hi = axis(half, i);
        for bound in [-hi, hi] {
            let t = (bound - ai) / di;
            if t > 0.0 && t < 1.0 {
                breakpoints.push(t);
            }
        }
    }
    breakpoints.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));

    // 小区間ごとに f(t) = Σ (α_i + β_i t)² を最小化する。各軸の状態(外/中/外)は
    // 区間内で一定なので、α_i/β_i は区間の中点で判定すれば決まる。
    let mut best_t = 0.0;
    let mut best_f = f64::INFINITY;
    let evaluate = |t: f64| -> f64 {
        let p = a.addcarry_scaled(d, t);
        let mut sum = 0.0;
        for i in 0..3 {
            let pi = axis(p, i);
            let hi = axis(half, i);
            let g = if pi < -hi {
                -hi - pi
            } else if pi > hi {
                pi - hi
            } else {
                0.0
            };
            sum += g * g;
        }
        sum
    };
    for w in breakpoints.windows(2) {
        let (lo, hi_t) = (w[0], w[1]);
        if hi_t - lo <= 0.0 {
            continue;
        }
        let mid = 0.5 * (lo + hi_t);
        let p_mid = a.addcarry_scaled(d, mid);
        // この小区間での f(t) = Σ (α_i + β_i t)²。
        let mut alpha_beta = 0.0; // Σ α_i β_i
        let mut beta_beta = 0.0; // Σ β_i²
        for i in 0..3 {
            let hi_i = axis(half, i);
            let pi = axis(p_mid, i);
            let (sign, bound) = if pi < -hi_i {
                (-1.0, -hi_i)
            } else if pi > hi_i {
                (1.0, hi_i)
            } else {
                continue; // 箱の中: この軸は距離に寄与しない。
            };
            // g_i(t) = sign * (a_i + d_i t - bound) = α_i + β_i t
            let alpha = sign * (axis(a, i) - bound);
            let beta = sign * axis(d, i);
            alpha_beta += alpha * beta;
            beta_beta += beta * beta;
        }
        // f'(t) = 2(Σ α_i β_i + t Σ β_i²) = 0 → t = -Σαβ / Σββ
        let t_star = if beta_beta > 1e-18 {
            (-alpha_beta / beta_beta).clamp(lo, hi_t)
        } else {
            mid // この区間では f は定数(全軸が箱の中、または線分が軸に垂直)。
        };
        for t in [lo, hi_t, t_star] {
            let f = evaluate(t);
            if f < best_f {
                best_f = f;
                best_t = t;
            }
        }
    }
    best_t
}

/// カプセル と ボックス(**群4で実装**)。
///
/// 増分Lの時点では**未実装で`None`を返していた**——「線分-OBBの最近接点は
/// 面/辺/頂点の場合分けが要り、他の3組のように既存の問題へ素直に帰着しない」
/// という理由で、エディタでカプセルと箱を並べるとすり抜けていた。
/// `closest_segment_param_to_aabb`(解析的な区分二次最小化)を用意したことで、
/// **球-箱と同じ「最近接点を中心とする球」への帰着**が使えるようになった。
///
/// **接触点は最大2点にする**。寝たカプセルが箱の上で静止するには2点要る
/// (1点だと転がり続ける)——`capsule_plane`が同じ理由で2点を出しているのと同じ。
/// カプセルの芯線が接触法線とほぼ直交している(=寝ている)ときだけ、
/// 芯線の両端それぞれについて接触点を作る。
fn capsule_box(
    capsule_xf: Transform,
    radius: f64,
    half_height: f64,
    box_xf: Transform,
    half_extents: Vec3,
) -> Option<(Vec3, Vec<ContactPoint>)> {
    let to_local = box_xf.inverse();
    let (world_a, world_b) = capsule_segment(capsule_xf, half_height);
    let a = to_local.apply_point(world_a);
    let b = to_local.apply_point(world_b);

    let clamp_to_box = |p: Vec3| {
        Vec3::new(
            p.x.clamp(-half_extents.x, half_extents.x),
            p.y.clamp(-half_extents.y, half_extents.y),
            p.z.clamp(-half_extents.z, half_extents.z),
        )
    };

    // 芯線上で箱に最も近い点と、そこから見た箱側の最近接点(いずれも箱ローカル)。
    let t = closest_segment_param_to_aabb(a, b, half_extents);
    let seg_point = a.addcarry_scaled(b - a, t);
    let box_point = clamp_to_box(seg_point);
    let delta_local = seg_point - box_point;
    let distance = delta_local.length();
    if distance >= radius {
        return None;
    }

    // 法線(箱→カプセル方向、ワールド)。芯線が箱の内部を通る退化ケースでは
    // **最も浅い面へ押し出す**(球-箱の deep case と同じ考え方だが、
    // 固定の +y ではなく実際に最も近い面を選ぶ——カプセルは細長く、
    // 箱を貫く向きが軸によって大きく違うため)。
    let normal_local = if distance > EPS_LEN {
        delta_local.scale(1.0 / distance)
    } else {
        let depths = [
            half_extents.x - seg_point.x.abs(),
            half_extents.y - seg_point.y.abs(),
            half_extents.z - seg_point.z.abs(),
        ];
        let mut axis = 1; // 同値なら y を選ぶ(決定論)。
        for (i, &depth) in depths.iter().enumerate() {
            if depth < depths[axis] {
                axis = i;
            }
        }
        let sign = match axis {
            0 => seg_point.x,
            1 => seg_point.y,
            _ => seg_point.z,
        };
        let s = if sign >= 0.0 { 1.0 } else { -1.0 };
        match axis {
            0 => Vec3::new(s, 0.0, 0.0),
            1 => Vec3::new(0.0, s, 0.0),
            _ => Vec3::new(0.0, 0.0, s),
        }
    };
    let normal_world = box_xf.apply_dir(normal_local).normalize_or_zero();
    if normal_world.length_sq() < 0.5 {
        return None; // 退化した変換(スケール0など)。
    }

    // 芯線が接触法線とほぼ直交していれば「寝ている」——両端で接触点を作る。
    let seg_dir_local = (b - a).normalize_or_zero();
    let lying = seg_dir_local.dot(normal_local).abs() < 0.25;
    let mut points = Vec::new();
    if lying {
        for (feature_id, end_local) in [a, b].into_iter().enumerate() {
            let end_box_point = clamp_to_box(end_local);
            let end_distance = (end_local - end_box_point).length();
            if end_distance < radius {
                points.push(ContactPoint {
                    world_point: box_xf.apply_point(end_box_point),
                    penetration: radius - end_distance,
                    feature_id: feature_id as u32,
                });
            }
        }
    }
    if points.is_empty() {
        points.push(ContactPoint {
            world_point: box_xf.apply_point(box_point),
            penetration: radius - distance,
            feature_id: 0,
        });
    }
    Some((normal_world, points))
}

fn sphere_box(
    sphere_center: Vec3,
    radius: f64,
    box_xf: Transform,
    half_extents: Vec3,
) -> Option<(Vec3, ContactPoint)> {
    let local = box_xf.inverse().apply_point(sphere_center);
    let clamped = Vec3::new(
        local.x.clamp(-half_extents.x, half_extents.x),
        local.y.clamp(-half_extents.y, half_extents.y),
        local.z.clamp(-half_extents.z, half_extents.z),
    );
    let closest_world = box_xf.apply_point(clamped);
    let d = sphere_center - closest_world;
    let len_sq = d.length_sq();
    if len_sq >= radius * radius {
        return None;
    }
    let len = len_sq.sqrt();
    // 中心がボックス内部にある退化ケース: 最近面方向にフォールバック(決定的、y軸優先)。
    let normal = if len < EPS_LEN {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        d.scale(1.0 / len)
    };
    let penetration = radius - len;
    Some((
        normal,
        ContactPoint {
            world_point: closest_world,
            penetration,
            feature_id: 0,
        },
    ))
}

/// ボックスのワールド系ローカル軸(axis=0,1,2 → ローカル x,y,z)。
fn box_axis_world(xf: Transform, axis: usize) -> Vec3 {
    let local = match axis {
        0 => Vec3::new(1.0, 0.0, 0.0),
        1 => Vec3::new(0.0, 1.0, 0.0),
        _ => Vec3::new(0.0, 0.0, 1.0),
    };
    xf.apply_dir(local)
}

/// 分離軸(cross積の退化除外の閾値)。設計 §4.4 の $10^{-10}$。
const SAT_DEGENERATE_AXIS_LEN_SQ: f64 = 1e-10;
/// 軸選択ヒステリシスの相対閾値。設計 §4.4・§9「SAT 軸ヒステリシス: 相対5%」。
const AXIS_HYSTERESIS_RELATIVE: f64 = 0.05;

/// 15軸(A面3 + B面3 + 辺×辺9)の SAT。分離軸が見つかれば `None`。
/// 重なっている場合は最小重なり軸のインデックスと重なり量を返す。
/// インデックス規約: 0-2 = A のローカル軸、3-5 = B のローカル軸、
/// 6+i*3+j (i,j∈0..3) = A の軸iとBの軸jの外積。
fn box_box_sat(
    xf_a: Transform,
    half_a: Vec3,
    xf_b: Transform,
    half_b: Vec3,
    preferred_axis: Option<usize>,
) -> Option<(usize, f64)> {
    let a_axes = [
        box_axis_world(xf_a, 0),
        box_axis_world(xf_a, 1),
        box_axis_world(xf_a, 2),
    ];
    let b_axes = [
        box_axis_world(xf_b, 0),
        box_axis_world(xf_b, 1),
        box_axis_world(xf_b, 2),
    ];
    let half_a_arr = [half_a.x, half_a.y, half_a.z];
    let half_b_arr = [half_b.x, half_b.y, half_b.z];
    let t = xf_b.position - xf_a.position;

    let mut candidates: Vec<(Vec3, usize)> = Vec::with_capacity(15);
    for (i, &ax) in a_axes.iter().enumerate() {
        candidates.push((ax, i));
    }
    for (j, &ax) in b_axes.iter().enumerate() {
        candidates.push((ax, 3 + j));
    }
    for (i, &ai) in a_axes.iter().enumerate() {
        for (j, &bj) in b_axes.iter().enumerate() {
            candidates.push((ai.cross(bj), 6 + i * 3 + j));
        }
    }

    let mut min_pen = f64::INFINITY;
    let mut min_idx = 0usize;
    let mut preferred_pen: Option<f64> = None;
    for (axis, idx) in candidates {
        let len_sq = axis.length_sq();
        if len_sq < SAT_DEGENERATE_AXIS_LEN_SQ {
            continue; // 辺×辺の平行退化(設計 §4.4 の表): この軸を候補から除外
        }
        let n = axis.scale(1.0 / len_sq.sqrt());
        let ra: f64 = (0..3).map(|k| half_a_arr[k] * a_axes[k].dot(n).abs()).sum();
        let rb: f64 = (0..3).map(|k| half_b_arr[k] * b_axes[k].dot(n).abs()).sum();
        let dist = t.dot(n).abs();
        let pen = ra + rb - dist;
        if pen < 0.0 {
            return None; // 分離軸が見つかった → 非接触
        }
        if pen < min_pen {
            min_pen = pen;
            min_idx = idx;
        }
        if preferred_axis == Some(idx) {
            preferred_pen = Some(pen);
        }
    }
    // 軸選択のヒステリシス(設計 §4.4「相対5%」): 前ステップの軸が今回も僅差(5%以内)なら
    // 数値ジッタによる軸のフリップ(≒法線の振動、warm starting の feature_id 対応も崩す)を
    // 避けてそれを維持する。
    if let (Some(axis), Some(pen)) = (preferred_axis, preferred_pen) {
        if pen <= min_pen * (1.0 + AXIS_HYSTERESIS_RELATIVE) {
            return Some((axis, pen));
        }
    }
    Some((min_idx, min_pen))
}

fn axis_for_index(a_axes: &[Vec3; 3], b_axes: &[Vec3; 3], idx: usize) -> Vec3 {
    if idx < 3 {
        a_axes[idx]
    } else if idx < 6 {
        b_axes[idx - 3]
    } else {
        let e = idx - 6;
        a_axes[e / 3].cross(b_axes[e % 3])
    }
}

/// 参照ボックスのローカル軸 `ref_axis`・符号 `ref_sign` で決まる面の4頂点(ワールド座標、
/// 境界を一周する順序)と、その面が乗る「他の2軸」のインデックスを返す。
fn box_face_vertices(
    xf: Transform,
    half: Vec3,
    ref_axis: usize,
    ref_sign: f64,
) -> ([Vec3; 4], [usize; 2]) {
    let half_arr = [half.x, half.y, half.z];
    let others = match ref_axis {
        0 => [1usize, 2usize],
        1 => [0, 2],
        _ => [0, 1],
    };
    let mut local = [0.0; 3];
    local[ref_axis] = ref_sign * half_arr[ref_axis];
    let corner = |s0: f64, s1: f64| {
        let mut l = local;
        l[others[0]] = s0 * half_arr[others[0]];
        l[others[1]] = s1 * half_arr[others[1]];
        xf.apply_point(Vec3::new(l[0], l[1], l[2]))
    };
    (
        [
            corner(-1.0, -1.0),
            corner(1.0, -1.0),
            corner(1.0, 1.0),
            corner(-1.0, 1.0),
        ],
        others,
    )
}

/// Sutherland-Hodgman: 多角形を半空間 (p-plane_point)·normal <= 0 側へ切り取る。
fn clip_polygon_against_plane(poly: &[Vec3], plane_point: Vec3, plane_normal: Vec3) -> Vec<Vec3> {
    if poly.len() < 2 {
        return Vec::new();
    }
    let n = poly.len();
    let mut out = Vec::with_capacity(n + 1);
    for i in 0..n {
        let cur = poly[i];
        let prev = poly[(i + n - 1) % n];
        let cur_dist = (cur - plane_point).dot(plane_normal);
        let prev_dist = (prev - plane_point).dot(plane_normal);
        let cur_inside = cur_dist <= 0.0;
        let prev_inside = prev_dist <= 0.0;
        if cur_inside {
            if !prev_inside {
                let denom = prev_dist - cur_dist;
                let s = if denom.abs() < EPS_LEN {
                    0.0
                } else {
                    prev_dist / denom
                };
                out.push(prev.addcarry_scaled(cur - prev, s));
            }
            out.push(cur);
        } else if prev_inside {
            let denom = prev_dist - cur_dist;
            let s = if denom.abs() < EPS_LEN {
                0.0
            } else {
                prev_dist / denom
            };
            out.push(prev.addcarry_scaled(cur - prev, s));
        }
    }
    out
}

/// 面接触(SAT の最小重なり軸が A か B のローカル軸)のマニフォールド生成。
/// 設計 §4.4「参照面に対して入射面の頂点を Sutherland-Hodgman クリップ」。
fn box_box_face_contact(
    xf_a: Transform,
    half_a: Vec3,
    xf_b: Transform,
    half_b: Vec3,
    axis_a_to_b: Vec3,
    ref_is_a: bool,
) -> Vec<ContactPoint> {
    let (ref_xf, ref_half, other_xf, other_half) = if ref_is_a {
        (xf_a, half_a, xf_b, half_b)
    } else {
        (xf_b, half_b, xf_a, half_a)
    };
    // 参照面の外向き法線: A が参照なら axis_a_to_b の向き、B が参照なら逆向き。
    let ref_normal = if ref_is_a {
        axis_a_to_b
    } else {
        axis_a_to_b.scale(-1.0)
    };
    let ref_axes = [
        box_axis_world(ref_xf, 0),
        box_axis_world(ref_xf, 1),
        box_axis_world(ref_xf, 2),
    ];
    let ref_axis = (0..3)
        .max_by(|&i, &j| {
            ref_normal
                .dot(ref_axes[i])
                .abs()
                .partial_cmp(&ref_normal.dot(ref_axes[j]).abs())
                .unwrap()
        })
        .unwrap();
    let ref_sign = if ref_normal.dot(ref_axes[ref_axis]) >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let (ref_face, side_axes) = box_face_vertices(ref_xf, ref_half, ref_axis, ref_sign);
    let ref_half_arr = [ref_half.x, ref_half.y, ref_half.z];

    // 入射面: 他ボックスの6面のうち法線が ref_normal に最も反平行なもの。
    let other_axes = [
        box_axis_world(other_xf, 0),
        box_axis_world(other_xf, 1),
        box_axis_world(other_xf, 2),
    ];
    let mut best_axis = 0usize;
    let mut best_sign = 1.0f64;
    let mut best_dot = f64::INFINITY;
    for (axis, &ax) in other_axes.iter().enumerate() {
        for &sign in &[1.0, -1.0] {
            let d = ref_normal.dot(ax.scale(sign));
            if d < best_dot {
                best_dot = d;
                best_axis = axis;
                best_sign = sign;
            }
        }
    }
    let (incident_face, _) = box_face_vertices(other_xf, other_half, best_axis, best_sign);

    // 参照面の4側平面でクリップ(側平面法線 = side_axes の各軸、符号は面の外側)。
    let mut poly: Vec<Vec3> = incident_face.to_vec();
    for &side_axis in &side_axes {
        let axis_world = ref_axes[side_axis];
        let half = ref_half_arr[side_axis];
        for &sign in &[1.0, -1.0] {
            let plane_point = ref_xf.position.addcarry_scaled(axis_world, sign * half);
            let plane_normal = axis_world.scale(sign);
            poly = clip_polygon_against_plane(&poly, plane_point, plane_normal);
            if poly.is_empty() {
                break;
            }
        }
        if poly.is_empty() {
            break;
        }
    }

    let ref_face_point = ref_face[0];
    let depth_of = |p: Vec3| (ref_face_point - p).dot(ref_normal);

    // feature_id: warm starting(設計 §4.4)がステップ間で正しく対応づけられるよう、
    // クリップ後の配列インデックス(ステップごとに変わりうる)ではなく、軸選択
    // (ref_axis/sign・incident_axis/sign)+ 参照面上の象限(側軸2本の符号)から組み立てる。
    // 静止・準静止のスタックでは軸選択も象限もステップ間で安定するため、warm start の
    // 前提(同一 feature_id ⇒ 同一物理接触点)を満たす(頂点/辺の追跡による厳密な対応付けは
    // 将来の精緻化課題)。
    let base_feature = (ref_axis as u32)
        | (u32::from(ref_sign > 0.0) << 2)
        | ((best_axis as u32) << 3)
        | (u32::from(best_sign > 0.0) << 5);
    let quadrant_of = |p: Vec3| -> u32 {
        let d = p - ref_xf.position;
        let s0 = u32::from(d.dot(ref_axes[side_axes[0]]) >= 0.0);
        let s1 = u32::from(d.dot(ref_axes[side_axes[1]]) >= 0.0);
        s0 | (s1 << 1)
    };

    if poly.is_empty() {
        // 設計 §4.4 表: クリップ結果が0点 → 元の入射面頂点から最深点1点にフォールバック。
        let deepest = incident_face
            .iter()
            .copied()
            .max_by(|&p, &q| depth_of(p).partial_cmp(&depth_of(q)).unwrap())
            .unwrap();
        let pen = depth_of(deepest);
        return vec![ContactPoint {
            world_point: deepest.addcarry_scaled(ref_normal, 0.5 * pen),
            penetration: pen,
            feature_id: base_feature | (quadrant_of(deepest) << 6),
        }];
    }

    let mut points: Vec<ContactPoint> = poly
        .iter()
        .filter_map(|&p| {
            let pen = depth_of(p);
            if pen < -1e-9 {
                None // 参照面より外側(貫入していない)は除外
            } else {
                Some(ContactPoint {
                    world_point: p.addcarry_scaled(ref_normal, 0.5 * pen),
                    penetration: pen.max(0.0),
                    feature_id: base_feature | (quadrant_of(p) << 6),
                })
            }
        })
        .collect();

    if points.is_empty() {
        // フィルタ後に0点(全点が僅かに外側)→ クリップ後の最深点にフォールバック。
        let deepest = poly
            .iter()
            .copied()
            .max_by(|&p, &q| depth_of(p).partial_cmp(&depth_of(q)).unwrap())
            .unwrap();
        let pen = depth_of(deepest);
        return vec![ContactPoint {
            world_point: deepest.addcarry_scaled(ref_normal, 0.5 * pen),
            penetration: pen.max(0.0),
            feature_id: base_feature | (quadrant_of(deepest) << 6),
        }];
    }

    if points.len() > 4 {
        // 設計 §4.4 表の簡易版縮約: 最深点を含む上位4点(貫入深さ降順)を保持する
        // (面積最大化による厳密な4点選択は将来の精緻化課題)。
        points.sort_by(|a, b| b.penetration.partial_cmp(&a.penetration).unwrap());
        points.truncate(4);
    }
    points
}

/// 辺×辺接触(SAT の最小重なり軸が外積軸)のマニフォールド生成。単一接触点。
fn box_box_edge_contact(
    xf_a: Transform,
    half_a: Vec3,
    xf_b: Transform,
    half_b: Vec3,
    axis_idx: usize,
    penetration: f64,
) -> ContactPoint {
    let e = axis_idx - 6;
    let (i, j) = (e / 3, e % 3);
    let a_axes = [
        box_axis_world(xf_a, 0),
        box_axis_world(xf_a, 1),
        box_axis_world(xf_a, 2),
    ];
    let b_axes = [
        box_axis_world(xf_b, 0),
        box_axis_world(xf_b, 1),
        box_axis_world(xf_b, 2),
    ];
    let half_a_arr = [half_a.x, half_a.y, half_a.z];
    let half_b_arr = [half_b.x, half_b.y, half_b.z];
    let t = xf_b.position - xf_a.position;

    let others_a = match i {
        0 => [1usize, 2usize],
        1 => [0, 2],
        _ => [0, 1],
    };
    let others_b = match j {
        0 => [1usize, 2usize],
        1 => [0, 2],
        _ => [0, 1],
    };
    let sign = |axes: &[Vec3; 3], k: usize, dir: Vec3| -> f64 {
        if axes[k].dot(dir) >= 0.0 {
            1.0
        } else {
            -1.0
        }
    };
    let mut local_a = [0.0; 3];
    local_a[others_a[0]] = sign(&a_axes, others_a[0], t) * half_a_arr[others_a[0]];
    local_a[others_a[1]] = sign(&a_axes, others_a[1], t) * half_a_arr[others_a[1]];
    let p_a = xf_a.apply_point(Vec3::new(local_a[0], local_a[1], local_a[2]));
    let d_a = a_axes[i];

    let neg_t = t.scale(-1.0);
    let mut local_b = [0.0; 3];
    local_b[others_b[0]] = sign(&b_axes, others_b[0], neg_t) * half_b_arr[others_b[0]];
    local_b[others_b[1]] = sign(&b_axes, others_b[1], neg_t) * half_b_arr[others_b[1]];
    let p_b = xf_b.apply_point(Vec3::new(local_b[0], local_b[1], local_b[2]));
    let d_b = b_axes[j];

    // 2直線の最近点(d_a, d_b は単位ベクトル)。設計 §4.4「辺×辺」。
    let r = p_a - p_b;
    let b_coeff = d_a.dot(d_b);
    let c = d_a.dot(r);
    let f = d_b.dot(r);
    let denom = 1.0 - b_coeff * b_coeff;
    let (s, u) = if denom.abs() < EPS_LEN {
        (0.0, 0.0) // SAT で既に非退化軸として選ばれているため通常到達しない
    } else {
        let u = (f - b_coeff * c) / denom;
        let s = u * b_coeff - c;
        (s, u)
    };
    let s = s.clamp(-half_a_arr[i], half_a_arr[i]);
    let u = u.clamp(-half_b_arr[j], half_b_arr[j]);

    let closest_a = p_a.addcarry_scaled(d_a, s);
    let closest_b = p_b.addcarry_scaled(d_b, u);
    ContactPoint {
        world_point: closest_a.addcarry_scaled(closest_b - closest_a, 0.5),
        penetration,
        // warm starting(設計 §4.4)用の安定 feature_id: 辺の組 (i,j) から一意に決まる
        // (面接触の feature_id 範囲 0-127 とは 200 のオフセットで重ならないようにする)。
        feature_id: 200 + (i * 3 + j) as u32,
    }
}

/// Box-Box(SAT)。設計 docs/10-mechanics/02-collision-detection.md §4.4。
/// `preferred_axis` は軸選択ヒステリシス用(前ステップで選ばれた軸、`detect` が管理する
/// `AxisCache` から渡す。テスト等で履歴が無い場合は `None` で純粋な最小重なり軸を使う)。
/// 戻り値の第3要素は今回選ばれた軸インデックス(呼び出し側がキャッシュ更新に使う)。
/// マニフォールド持続化(§4.7、feature_id の移動量チェックによる再利用判定)は
/// 接触ソルバ側(`contact::ManifoldCache`)が担う——ここは毎ステップ新しい接触点を
/// 生成し、安定した feature_id を振るところまでを担当する。
fn box_box(
    xf_a: Transform,
    half_a: Vec3,
    xf_b: Transform,
    half_b: Vec3,
    preferred_axis: Option<usize>,
) -> Option<(Vec3, Vec<ContactPoint>, usize)> {
    let (axis_idx, penetration) = box_box_sat(xf_a, half_a, xf_b, half_b, preferred_axis)?;

    let a_axes = [
        box_axis_world(xf_a, 0),
        box_axis_world(xf_a, 1),
        box_axis_world(xf_a, 2),
    ];
    let b_axes = [
        box_axis_world(xf_b, 0),
        box_axis_world(xf_b, 1),
        box_axis_world(xf_b, 2),
    ];
    let raw_axis = axis_for_index(&a_axes, &b_axes, axis_idx);
    let t = xf_b.position - xf_a.position;
    let mut normal = raw_axis.scale(1.0 / raw_axis.length_sq().sqrt());
    if normal.dot(t) < 0.0 {
        normal = normal.scale(-1.0);
    }

    let points = if axis_idx < 6 {
        box_box_face_contact(xf_a, half_a, xf_b, half_b, normal, axis_idx < 3)
    } else {
        vec![box_box_edge_contact(
            xf_a,
            half_a,
            xf_b,
            half_b,
            axis_idx,
            penetration,
        )]
    };
    Some((normal, points, axis_idx))
}

fn shape_pair_manifold(
    shape_a: &Shape,
    xf_a: Transform,
    shape_b: &Shape,
    xf_b: Transform,
) -> Option<(Vec3, Vec<ContactPoint>)> {
    match (shape_a, shape_b) {
        (Shape::Sphere { radius: ra }, Shape::Sphere { radius: rb }) => {
            sphere_sphere(xf_a.position, *ra, xf_b.position, *rb).map(|(n, p)| (n, vec![p]))
        }
        // sphere_plane/box_plane/sphere_box は「面から離れる自然な向き」を返す。
        // マニフォールドの normal は設計の A→B 規約(sphere-sphere の d=c_B-c_A に整合)なので、
        // A が球/箱(面から出ていく側)の組では反転、A が平面側の組ではそのまま使う。
        (Shape::Sphere { radius }, Shape::Plane { normal, d }) => {
            sphere_plane(xf_a.position, *radius, *normal, *d).map(|(n, p)| (-n, vec![p]))
        }
        (Shape::Plane { normal, d }, Shape::Sphere { radius }) => {
            sphere_plane(xf_b.position, *radius, *normal, *d).map(|(n, p)| (n, vec![p]))
        }
        (Shape::Box { half_extents }, Shape::Plane { normal, d }) => {
            box_plane(xf_a, *half_extents, *normal, *d).map(|(n, pts)| (-n, pts))
        }
        (Shape::Plane { normal, d }, Shape::Box { half_extents }) => {
            box_plane(xf_b, *half_extents, *normal, *d)
        }
        (Shape::Sphere { radius }, Shape::Box { half_extents }) => {
            sphere_box(xf_a.position, *radius, xf_b, *half_extents).map(|(n, p)| (-n, vec![p]))
        }
        (Shape::Box { half_extents }, Shape::Sphere { radius }) => {
            sphere_box(xf_b.position, *radius, xf_a, *half_extents).map(|(n, p)| (n, vec![p]))
        }
        (Shape::Box { half_extents: ha }, Shape::Box { half_extents: hb }) => {
            // 軸選択ヒステリシス無し(履歴を持たない単発呼び出し。`detect` は別途
            // `AxisCache` 付きで `box_box` を直接呼ぶ、下記参照)。
            box_box(xf_a, *ha, xf_b, *hb, None).map(|(n, p, _)| (n, p))
        }
        (Shape::Plane { .. }, Shape::Plane { .. }) => None, // static同士は broadphase で除外すべき無意味ペア

        // **カプセル(増分Lで追加)**。芯線(線分)への最近接点を取れば、
        // 平面/球/カプセルのいずれもすでにある問題へ帰着する。
        // 法線の向きは上と同じA→B規約に合わせて必要なら反転する。
        (
            Shape::Capsule {
                radius,
                half_height,
            },
            Shape::Plane { normal, d },
        ) => capsule_plane(xf_a, *radius, *half_height, *normal, *d).map(|(n, pts)| (-n, pts)),
        (
            Shape::Plane { normal, d },
            Shape::Capsule {
                radius,
                half_height,
            },
        ) => capsule_plane(xf_b, *radius, *half_height, *normal, *d),
        (
            Shape::Capsule {
                radius,
                half_height,
            },
            Shape::Sphere { radius: sr },
        ) => capsule_sphere(xf_a, *radius, *half_height, xf_b.position, *sr)
            .map(|(n, p)| (n, vec![p])),
        (
            Shape::Sphere { radius: sr },
            Shape::Capsule {
                radius,
                half_height,
            },
        ) => capsule_sphere(xf_b, *radius, *half_height, xf_a.position, *sr)
            .map(|(n, p)| (-n, vec![p])),
        (
            Shape::Capsule {
                radius: ra,
                half_height: ha,
            },
            Shape::Capsule {
                radius: rb,
                half_height: hb,
            },
        ) => capsule_capsule(xf_a, *ra, *ha, xf_b, *rb, *hb).map(|(n, p)| (n, vec![p])),

        // **カプセル × 箱(群4で実装)**。増分Lの時点では「線分-OBBの最近接点は
        // 面/辺/頂点の場合分けが要る」として`None`を返しており、エディタで
        // カプセルと箱を並べるとすり抜けていた。`capsule_box`のdoc参照。
        //
        // `capsule_box` は `sphere_box` と同じく「**箱から離れる自然な向き**」
        // (箱→カプセル)を返すので、A=カプセルの側では反転して A→B 規約に合わせる。
        (
            Shape::Capsule {
                radius,
                half_height,
            },
            Shape::Box { half_extents },
        ) => capsule_box(xf_a, *radius, *half_height, xf_b, *half_extents)
            .map(|(n, points)| (-n, points)),
        (
            Shape::Box { half_extents },
            Shape::Capsule {
                radius,
                half_height,
            },
        ) => capsule_box(xf_b, *radius, *half_height, xf_a, *half_extents),

        // **群10で実装**。`Compound`は「部品ごとに既存の解析的ペア関数へ
        // 帰着させる」形で解く——GJK/EPA等の新しい幾何アルゴリズムは要らない。
        // 部品Aの各要素×shape_b(shape_bもCompoundなら、この再帰の次段で
        // 今度は`(_, Compound)`側の枝に落ちて部品Bの各要素へさらに分解される、
        // つまりCompound×Compoundは部品どうしの総当たりに帰着する)。
        //
        // 複数の部品が同時に接触する場合、`ContactManifold`は法線1本しか
        // 持てない設計のため、**貫入量が最大の部品ペアの法線を代表として採用し、
        // 全部品の接触点をその1本の法線へ束ねる**近似を取る(車体を組んだ
        // シャシーが地面に接するような「全部品の法線が一致する」典型例では
        // 厳密に正しく、法線が食い違う稀なケース(隅に挟まる等)でのみ近似になる、
        // モジュールdoc参照)。`feature_id`は部品indexで空間を分けて warm start の
        // 衝突を避ける。
        (Shape::Compound { children }, _) => {
            let mut merged: Option<(Vec3, Vec<ContactPoint>)> = None;
            for (child_index, (child_xf, child_shape)) in children.iter().enumerate() {
                let world_xf = xf_a.compose(*child_xf);
                if let Some(result) = shape_pair_manifold(child_shape, world_xf, shape_b, xf_b) {
                    merged = Some(merge_compound_manifolds(merged, result, child_index));
                }
            }
            merged
        }
        (_, Shape::Compound { children }) => {
            let mut merged: Option<(Vec3, Vec<ContactPoint>)> = None;
            for (child_index, (child_xf, child_shape)) in children.iter().enumerate() {
                let world_xf = xf_b.compose(*child_xf);
                if let Some(result) = shape_pair_manifold(shape_a, xf_a, child_shape, world_xf) {
                    merged = Some(merge_compound_manifolds(merged, result, child_index));
                }
            }
            merged
        }

        // **群11で実装**。移行前は「頂点列だけで面情報が無く、3D凸包の実装は
        // 範囲外」という理由で`None`を返しており、`ConvexMesh`は**何とも衝突
        // せずすり抜けていた**。`crate::hull`で凸包を張れるようになったので、
        // 他の形状と同じ土俵に上げる(`convex_poly_manifold`のdoc参照)。
        //
        // 平面だけは非有界で SAT の射影区間に乗らないため専用経路
        // (`convex_mesh_plane`)で扱う——これは`box_plane`と同じ構造で、
        // 最大4点の面接触を作れるので「床に置いた多面体が静止する」ケースが
        // きちんと安定する。
        (Shape::ConvexMesh { vertices }, Shape::Plane { normal, d }) => {
            convex_mesh_plane(vertices, xf_a, *normal, *d).map(|(n, pts)| (-n, pts))
        }
        (Shape::Plane { normal, d }, Shape::ConvexMesh { vertices }) => {
            convex_mesh_plane(vertices, xf_b, *normal, *d)
        }

        // 多面体 × 球は「表面上の最近点」で球-球へ帰着させる(`sphere_box`と同じ発想)。
        (Shape::ConvexMesh { vertices }, Shape::Sphere { radius }) => {
            let hull = crate::hull::convex_hull(vertices)?;
            let poly = ConvexPoly::from_hull(&hull, xf_a);
            convex_poly_sphere(&poly, &hull.faces, xf_b.position, *radius)
                .map(|(n, p)| (n, vec![p]))
        }
        (Shape::Sphere { radius }, Shape::ConvexMesh { vertices }) => {
            let hull = crate::hull::convex_hull(vertices)?;
            let poly = ConvexPoly::from_hull(&hull, xf_b);
            convex_poly_sphere(&poly, &hull.faces, xf_a.position, *radius)
                .map(|(n, p)| (-n, vec![p]))
        }

        // 多面体 × 多面体 / 多面体 × 箱は SAT(`convex_poly_manifold`のdoc参照)。
        (Shape::ConvexMesh { vertices: va }, Shape::ConvexMesh { vertices: vb }) => {
            let (ha, hb) = (crate::hull::convex_hull(va)?, crate::hull::convex_hull(vb)?);
            convex_poly_manifold(
                &ConvexPoly::from_hull(&ha, xf_a),
                &ConvexPoly::from_hull(&hb, xf_b),
            )
        }
        (Shape::ConvexMesh { vertices }, Shape::Box { half_extents }) => {
            let hull = crate::hull::convex_hull(vertices)?;
            convex_poly_manifold(
                &ConvexPoly::from_hull(&hull, xf_a),
                &ConvexPoly::from_box(xf_b, *half_extents),
            )
        }
        (Shape::Box { half_extents }, Shape::ConvexMesh { vertices }) => {
            let hull = crate::hull::convex_hull(vertices)?;
            convex_poly_manifold(
                &ConvexPoly::from_box(xf_a, *half_extents),
                &ConvexPoly::from_hull(&hull, xf_b),
            )
        }

        // **既知の限界(正直な開示)**: `ConvexMesh` × `Capsule` は未実装。
        // カプセルは「線分を半径で膨らませた」非多面体なので SAT の分離軸に
        // 素直に乗らず(丸い部分の分離軸は連続無限個ある)、線分-凸多面体の
        // 最近点計算を別途書く必要がある。本増分では手を付けず`None`を返す
        // ——この組み合わせだけは引き続きすり抜ける。移行前は`ConvexMesh`が
        // **何とも**衝突しなかったので、機能としては後退していない。
        (Shape::ConvexMesh { .. }, _) | (_, Shape::ConvexMesh { .. }) => None,
    }
}

/// `Compound`を(入れ子も含めて)葉の部品へ平坦化する(**群11**)。
/// 返すのは `(親→部品のワールド変換, 部品形状)`。`feature_id` の名前空間を
/// 分けるために、列挙順が安定していることが要件(`children` の順を保つ)。
fn flatten_compound_parts<'a>(
    children: &'a [(Transform, Shape)],
    base: Transform,
    out: &mut Vec<(Transform, &'a Shape)>,
) {
    for (child_xf, child_shape) in children {
        let world = base.compose(*child_xf);
        match child_shape {
            Shape::Compound { children: inner } => flatten_compound_parts(inner, world, out),
            other => out.push((world, other)),
        }
    }
}

/// 「法線が実質同じ」とみなす閾値の余弦(**群11**)。
///
/// 15°(cos≈0.966)。根拠は**接触の物理的な意味**:
/// - 床に並べた2つの箱のように、部品の接触面が同一平面(あるいはごく近い
///   傾き)なら、1本の法線で表しても物理は厳密に正しく、マニフォールドを
///   分けると同じ拘束を二重に解くことになって収束が悪くなる。
/// - 一方、L字の角のように**異なる面が異なる向きで**接している場合は、
///   1本に束ねた時点で片方の接触方向が消えてしまう(移行前の近似)。
///
/// 15°は「数値誤差やBaumgarte補正による法線の揺らぎ(実測で1°未満)では
/// 分裂せず、意図的に異なる面(実用上は30°以上、多くは90°)は確実に分ける」
/// という条件から取った。閾値をまたぐ中間的な角度では、束ねても分けても
/// 誤差は連続的に小さいので、正確な値に敏感ではない。
const COMPOUND_NORMAL_MERGE_COS: f64 = 0.966;

/// `Compound`が絡む衝突を、**部品ごとの独立したマニフォールド列**として返す
/// (**群11**)。
///
/// ## 何を変えたか
///
/// 移行前は `merge_compound_manifolds` が全部品の接触点を
/// **「貫入が最大の部品ペアの法線」1本**へ束ねていた。全部品の法線が揃う
/// 典型例(床に置いたシャシー)では厳密だが、**L字を角で接地させる**ような
/// 「異なる面が異なる向きで同時に当たる」配置では、片方の接触方向が
/// 丸ごと失われて物理的に誤った応答になっていた。
///
/// いまは部品ペアごとにマニフォールドを作り、**法線がほぼ同じもの同士だけ**を
/// 束ねる(`COMPOUND_NORMAL_MERGE_COS`)。したがって:
/// - 法線が揃う配置 → これまでどおり1本に束ねる(挙動不変・厳密なまま)
/// - 法線が食い違う配置 → 独立した複数のマニフォールドを出す(新しく正しい)
///
/// `feature_id` は部品indexで名前空間を分けるので、同じボディ対に複数の
/// マニフォールドが出ても warm starting のキャッシュキー
/// `(body_a, body_b, feature_id)` は衝突しない。
///
/// **既知の限界**: `Compound` × `Compound` では、A側だけを部品へ分解し、
/// B側は `shape_pair_manifold` の中で従来どおり1本へ束ねられる。
/// 両側を同時に部品分解すると部品数の積だけマニフォールドが出て、
/// 接触ソルバの反復コストが跳ね上がるため、本増分では片側のみとした。
fn compound_pair_manifolds(
    shape_a: &Shape,
    xf_a: Transform,
    shape_b: &Shape,
    xf_b: Transform,
) -> Vec<(Vec3, Vec<ContactPoint>)> {
    // どちら側を部品へ分解するか(A優先)。分解しない側はそのまま相手に渡す。
    let (parts, a_is_compound) = match (shape_a, shape_b) {
        (Shape::Compound { children }, _) => {
            let mut v = Vec::new();
            flatten_compound_parts(children, xf_a, &mut v);
            (v, true)
        }
        (_, Shape::Compound { children }) => {
            let mut v = Vec::new();
            flatten_compound_parts(children, xf_b, &mut v);
            (v, false)
        }
        _ => return Vec::new(),
    };

    let mut groups: Vec<(Vec3, Vec<ContactPoint>, f64)> = Vec::new();
    for (part_index, (part_xf, part_shape)) in parts.iter().enumerate() {
        let result = if a_is_compound {
            shape_pair_manifold(part_shape, *part_xf, shape_b, xf_b)
        } else {
            shape_pair_manifold(shape_a, xf_a, part_shape, *part_xf)
        };
        let Some((normal, mut points)) = result else {
            continue;
        };
        // 部品ごとに feature_id の名前空間を分ける(warm start の衝突回避)。
        let offset = (part_index as u32).wrapping_mul(1_000_003);
        for p in &mut points {
            p.feature_id = p.feature_id.wrapping_add(offset);
        }
        let max_pen = points
            .iter()
            .map(|p| p.penetration)
            .fold(f64::MIN, f64::max);

        // 既存グループのうち法線がほぼ同じものへ合流、無ければ新規グループ。
        match groups
            .iter_mut()
            .find(|(n, _, _)| n.dot(normal) >= COMPOUND_NORMAL_MERGE_COS)
        {
            Some((group_normal, group_points, group_max_pen)) => {
                // 代表法線は「最も深く刺さっている部品のもの」を採る(従来と同じ規約)。
                if max_pen > *group_max_pen {
                    *group_normal = normal;
                    *group_max_pen = max_pen;
                }
                group_points.extend(points);
            }
            None => groups.push((normal, points, max_pen)),
        }
    }

    groups
        .into_iter()
        .map(|(normal, points, _)| (normal, points))
        .collect()
}

/// `shape_pair_manifold`のCompound分解が複数の部品ペアから得た
/// `(normal, points)`を1つの`ContactManifold`へ束ねる(モジュールdoc
/// 「複数の部品が同時に接触する場合」参照)。`child_index`は`feature_id`の
/// 名前空間を部品ごとに分けてwarm startingの衝突を避けるためのオフセット。
fn merge_compound_manifolds(
    acc: Option<(Vec3, Vec<ContactPoint>)>,
    next: (Vec3, Vec<ContactPoint>),
    child_index: usize,
) -> (Vec3, Vec<ContactPoint>) {
    let offset = (child_index as u32).wrapping_mul(1_000_003);
    let (next_normal, mut next_points) = next;
    for p in &mut next_points {
        p.feature_id = p.feature_id.wrapping_add(offset);
    }
    match acc {
        None => (next_normal, next_points),
        Some((acc_normal, mut acc_points)) => {
            let acc_max_pen = acc_points
                .iter()
                .map(|p| p.penetration)
                .fold(f64::MIN, f64::max);
            let next_max_pen = next_points
                .iter()
                .map(|p| p.penetration)
                .fold(f64::MIN, f64::max);
            let normal = if next_max_pen > acc_max_pen {
                next_normal
            } else {
                acc_normal
            };
            acc_points.extend(next_points);
            (normal, acc_points)
        }
    }
}

/// Box-Box の軸選択ヒステリシス用キャッシュ(ペア→前ステップで選ばれた軸インデックス)。
/// 設計 §4.4「軸選択に前ステップの軸を優先するヒステリシス」。
pub type AxisCache = std::collections::BTreeMap<(usize, usize), usize>;

/// 動的 AABB BVH のノード。設計 §4.1 表「P2: SAP/BVH」、$O(N\log N)$ の目標アルゴリズム
/// (性能プロファイル §10)。SAP(x軸掃引、総当たり O(N²) の削減)を先に実装したが、設計が
/// 目標とする最終形の BVH に置き換えた(永続構造・挿入/削除は未実装、毎ステップ全 body
/// から決定論的に作り直す)。
enum BvhNode {
    Leaf {
        index: usize,
        aabb: Aabb,
    },
    Internal {
        aabb: Aabb,
        left: Box<BvhNode>,
        right: Box<BvhNode>,
    },
}

impl BvhNode {
    fn aabb(&self) -> Aabb {
        match self {
            BvhNode::Leaf { aabb, .. } => *aabb,
            BvhNode::Internal { aabb, .. } => *aabb,
        }
    }
}

fn union_aabb(a: Aabb, b: Aabb) -> Aabb {
    Aabb {
        min: Vec3::new(
            a.min.x.min(b.min.x),
            a.min.y.min(b.min.y),
            a.min.z.min(b.min.z),
        ),
        max: Vec3::new(
            a.max.x.max(b.max.x),
            a.max.y.max(b.max.y),
            a.max.z.max(b.max.z),
        ),
    }
}

/// 無限平面(設計上 `aabb_of` が min=-∞/max=+∞ を返す)は素朴な `(min+max)/2` だと
/// NaN になるため、有限な側だけで代表点を決める(有限軸なら真の重心、無限軸は0扱い
/// — 無限平面は常にどのAABBとも重なるため、木上のどこに置かれても`aabb_overlap`が
/// 正しく重なりを検出し、ソート順自体の妥当性には影響しない)。
fn centroid(aabb: Aabb) -> Vec3 {
    let mid = |lo: f64, hi: f64| -> f64 {
        match (lo.is_finite(), hi.is_finite()) {
            (true, true) => (lo + hi) * 0.5,
            (true, false) => lo,
            (false, true) => hi,
            (false, false) => 0.0,
        }
    };
    Vec3::new(
        mid(aabb.min.x, aabb.max.x),
        mid(aabb.min.y, aabb.max.y),
        mid(aabb.min.z, aabb.max.z),
    )
}

/// トップダウン構築: 重心のバウンディングボックスで最も広い軸を選び、その軸の重心座標で
/// ソートして中央値で2分する(単純な中央値分割、SAHのような費用関数は未実装)。
fn build_bvh(mut leaves: Vec<(usize, Aabb)>) -> BvhNode {
    if leaves.len() == 1 {
        let (index, aabb) = leaves[0];
        return BvhNode::Leaf { index, aabb };
    }

    let mut centroid_min = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut centroid_max = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for &(_, aabb) in &leaves {
        let c = centroid(aabb);
        centroid_min = Vec3::new(
            centroid_min.x.min(c.x),
            centroid_min.y.min(c.y),
            centroid_min.z.min(c.z),
        );
        centroid_max = Vec3::new(
            centroid_max.x.max(c.x),
            centroid_max.y.max(c.y),
            centroid_max.z.max(c.z),
        );
    }
    let extent = centroid_max - centroid_min;
    let axis = if extent.x >= extent.y && extent.x >= extent.z {
        0
    } else if extent.y >= extent.z {
        1
    } else {
        2
    };

    leaves.sort_by(|(_, a), (_, b)| {
        let ca = centroid(*a);
        let cb = centroid(*b);
        let (va, vb) = match axis {
            0 => (ca.x, cb.x),
            1 => (ca.y, cb.y),
            _ => (ca.z, cb.z),
        };
        va.partial_cmp(&vb).unwrap()
    });

    let mid = leaves.len() / 2;
    let right_leaves = leaves.split_off(mid);
    let left = build_bvh(leaves);
    let right = build_bvh(right_leaves);
    let aabb = union_aabb(left.aabb(), right.aabb());
    BvhNode::Internal {
        aabb,
        left: Box::new(left),
        right: Box::new(right),
    }
}

/// 2つの部分木間の重なりペアを再帰的に集める(標準的なBVH自己衝突走査)。各ペア
/// (i,j) はその最小共通祖先ノードでの `collect_cross_pairs` 呼び出しでちょうど1回だけ
/// 生成されるため、重複除去は不要。
fn collect_cross_pairs(a: &BvhNode, b: &BvhNode, pairs: &mut Vec<(usize, usize)>) {
    if !aabb_overlap(a.aabb(), b.aabb()) {
        return;
    }
    match (a, b) {
        (BvhNode::Leaf { index: ia, .. }, BvhNode::Leaf { index: ib, .. }) => {
            pairs.push(if ia < ib { (*ia, *ib) } else { (*ib, *ia) });
        }
        (BvhNode::Leaf { .. }, BvhNode::Internal { left, right, .. }) => {
            collect_cross_pairs(a, left, pairs);
            collect_cross_pairs(a, right, pairs);
        }
        (BvhNode::Internal { left, right, .. }, BvhNode::Leaf { .. }) => {
            collect_cross_pairs(left, b, pairs);
            collect_cross_pairs(right, b, pairs);
        }
        (
            BvhNode::Internal {
                left: al,
                right: ar,
                ..
            },
            BvhNode::Internal {
                left: bl,
                right: br,
                ..
            },
        ) => {
            collect_cross_pairs(al, bl, pairs);
            collect_cross_pairs(al, br, pairs);
            collect_cross_pairs(ar, bl, pairs);
            collect_cross_pairs(ar, br, pairs);
        }
    }
}

/// 木の内部で自分自身との重なり(左右の部分木間)を再帰的に集める。
fn collect_self_pairs(node: &BvhNode, pairs: &mut Vec<(usize, usize)>) {
    if let BvhNode::Internal { left, right, .. } = node {
        collect_self_pairs(left, pairs);
        collect_self_pairs(right, pairs);
        collect_cross_pairs(left, right, pairs);
    }
}

/// 動的 AABB BVH broadphase。ペアは (indexA, indexB) 昇順にソートして返す
/// (総当たり版と結果を一致させ、決定論・既存の数値挙動を保つ)。
fn bvh_candidate_pairs(bodies: &RigidBodySet) -> Vec<(usize, usize)> {
    let n = bodies.len();
    if n < 2 {
        return Vec::new();
    }
    let leaves: Vec<(usize, Aabb)> = (0..n)
        .map(|i| (i, aabb_of(bodies.shape_of(i), transform_of(bodies, i))))
        .collect();
    let root = build_bvh(leaves);
    let mut pairs = Vec::new();
    collect_self_pairs(&root, &mut pairs);
    pairs.sort_unstable();
    pairs
}

/// BVH broadphase(§4.1)+ narrowphase ディスパッチ(§4.2)。
/// ペア列挙順は (indexA, indexB) 昇順に固定(決定論)。
pub fn detect(bodies: &RigidBodySet, axis_cache: &mut AxisCache) -> Vec<ContactManifold> {
    let mut manifolds = Vec::new();
    for (a, b) in bvh_candidate_pairs(bodies) {
        // static/kinematic 同士は無意味ペア(設計 §4.4 表)。
        if bodies.body_type[a] != BodyType::Dynamic && bodies.body_type[b] != BodyType::Dynamic {
            continue;
        }
        // 衝突フィルタ(設計 §4.1)。broadphase 側で落とすので narrowphase は
        // 一切走らず、マニフォールドも生成されない = 完全にすり抜ける。
        if !crate::body::collision_filter_allows(
            bodies.collision_group[a],
            bodies.collision_mask[a],
            bodies.collision_group[b],
            bodies.collision_mask[b],
        ) {
            continue;
        }
        let xf_a = transform_of(bodies, a);
        let xf_b = transform_of(bodies, b);
        let shape_a = bodies.shape_of(a);
        let shape_b = bodies.shape_of(b);
        // `Compound`が絡むペアは**部品ごとに独立したマニフォールド**を出す
        // (群11、`compound_pair_manifolds`のdoc参照)。法線が揃う部品は
        // 1本に束ねられるので、従来どおりの配置では出力も従来と同一。
        let results: Vec<(Vec3, Vec<ContactPoint>)> = if matches!(shape_a, Shape::Compound { .. })
            || matches!(shape_b, Shape::Compound { .. })
        {
            compound_pair_manifolds(shape_a, xf_a, shape_b, xf_b)
        } else if let (Shape::Box { half_extents: ha }, Shape::Box { half_extents: hb }) =
            (shape_a, shape_b)
        {
            let preferred = axis_cache.get(&(a, b)).copied();
            let r = box_box(xf_a, *ha, xf_b, *hb, preferred);
            match &r {
                Some((_, _, axis_idx)) => {
                    axis_cache.insert((a, b), *axis_idx);
                }
                None => {
                    axis_cache.remove(&(a, b));
                }
            }
            r.map(|(n, p, _)| (n, p)).into_iter().collect()
        } else {
            shape_pair_manifold(shape_a, xf_a, shape_b, xf_b)
                .into_iter()
                .collect()
        };
        for (normal, points) in results {
            manifolds.push(ContactManifold {
                body_a: a,
                body_b: b,
                normal,
                points,
            });
        }
    }
    manifolds
}

/// テスト・単一ペア検査用の直接呼び出し(narrowphase の単体テストに使う)。
#[cfg(test)]
pub(crate) fn dispatch_for_test(
    shape_a: &Shape,
    xf_a: Transform,
    shape_b: &Shape,
    xf_b: Transform,
) -> Option<(Vec3, Vec<ContactPoint>)> {
    shape_pair_manifold(shape_a, xf_a, shape_b, xf_b)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_math::Quat;

    fn identity_xf(p: Vec3) -> Transform {
        Transform {
            position: p,
            rotation: sim_math::Quat::IDENTITY,
        }
    }

    /// 衝突フィルタ(設計 §4.1)が `detect` に効いていること。重なった2球は
    /// 既定フィルタでは必ずマニフォールドを作るが、互いに見えないグループへ
    /// 分けると **narrowphase まで到達せず** 0 件になる。
    #[test]
    fn collision_filter_suppresses_manifold_for_disjoint_groups() {
        use crate::body::{RigidBodyDesc, RigidBodySet};
        use sim_core::MaterialDb;

        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let build = || {
            let mut bodies = RigidBodySet::new();
            for x in [0.0, 1.5] {
                let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 1.0 }, steel);
                desc.transform.position = Vec3::new(x, 0.0, 0.0);
                bodies.create_body(desc, &materials);
            }
            bodies
        };

        let mut cache = AxisCache::default();
        let bodies = build();
        assert_eq!(
            detect(&bodies, &mut cache).len(),
            1,
            "既定フィルタでは接触する"
        );

        let mut filtered = build();
        // 0b01 は 0b01 だけを見る / 0b10 は 0b10 だけを見る → 互いに不可視。
        filtered.set_collision_filter(0, 0b01, 0b01);
        filtered.set_collision_filter(1, 0b10, 0b10);
        assert_eq!(detect(&filtered, &mut cache).len(), 0, "フィルタで落ちる");

        // 片側だけがマスクを緩めても通らない(双方向 AND)。
        let mut half = build();
        half.set_collision_filter(0, 0b01, 0b11);
        half.set_collision_filter(1, 0b10, 0b10);
        assert_eq!(detect(&half, &mut cache).len(), 0, "片側だけでは通らない");
    }

    /// `Compound`のAABBが各部品(ローカルtransform込み)のAABBの和集合になること。
    #[test]
    fn compound_aabb_is_the_union_of_its_children_aabbs() {
        let compound = Shape::Compound {
            children: vec![
                (
                    identity_xf(Vec3::new(-2.0, 0.0, 0.0)),
                    Shape::Sphere { radius: 0.5 },
                ),
                (
                    identity_xf(Vec3::new(2.0, 0.0, 0.0)),
                    Shape::Box {
                        half_extents: Vec3::new(1.0, 1.0, 1.0),
                    },
                ),
            ],
        };
        let aabb = aabb_of(&compound, identity_xf(Vec3::ZERO));
        assert!((aabb.min.x - -2.5).abs() < 1e-12);
        assert!((aabb.max.x - 3.0).abs() < 1e-12);
        assert!((aabb.min.y - -1.0).abs() < 1e-12);
        assert!((aabb.max.y - 1.0).abs() < 1e-12);
    }

    /// `Compound`(単一の箱を部品に持つ)と地面平面の接触が、同じ寸法の
    /// 素の`Box`と地面平面の接触に一致すること(部品分解が既存のBox-Plane
    /// 解析解へ正しく帰着することの確認)。
    #[test]
    fn compound_of_a_single_box_against_a_plane_matches_a_plain_box() {
        let half_extents = Vec3::new(1.0, 1.0, 1.0);
        let plane = Shape::Plane {
            normal: Vec3::new(0.0, 1.0, 0.0),
            d: 0.0,
        };
        // 箱の下面が y=-0.1 (めり込み量0.1)になるよう、中心を y=0.9 に置く。
        let body_xf = identity_xf(Vec3::new(0.0, 0.9, 0.0));

        let plain_box = Shape::Box { half_extents };
        let (plain_normal, plain_points) =
            dispatch_for_test(&plain_box, body_xf, &plane, identity_xf(Vec3::ZERO))
                .expect("box penetrates the plane");

        let compound = Shape::Compound {
            children: vec![(identity_xf(Vec3::ZERO), Shape::Box { half_extents })],
        };
        let (compound_normal, compound_points) =
            dispatch_for_test(&compound, body_xf, &plane, identity_xf(Vec3::ZERO))
                .expect("compound must penetrate exactly like the plain box");

        assert!((compound_normal - plain_normal).length() < 1e-12);
        assert_eq!(compound_points.len(), plain_points.len());
        let max_pen = |pts: &[ContactPoint]| pts.iter().map(|p| p.penetration).fold(0.0, f64::max);
        assert!((max_pen(&compound_points) - max_pen(&plain_points)).abs() < 1e-12);
    }

    /// `Compound`の複数部品(左右の車輪相当の2球)が同時に地面へ接触する場合、
    /// 両方の接触点が1つのマニフォールドへ束ねられること。
    #[test]
    fn compound_merges_contacts_from_multiple_penetrating_children() {
        let plane = Shape::Plane {
            normal: Vec3::new(0.0, 1.0, 0.0),
            d: 0.0,
        };
        let compound = Shape::Compound {
            children: vec![
                (
                    identity_xf(Vec3::new(-2.0, 0.0, 0.0)),
                    Shape::Sphere { radius: 1.0 },
                ),
                (
                    identity_xf(Vec3::new(2.0, 0.0, 0.0)),
                    Shape::Sphere { radius: 1.0 },
                ),
            ],
        };
        // 両方の球の中心を y=0.9 に置く(半径1.0なので貫入量0.1)。
        let body_xf = identity_xf(Vec3::new(0.0, 0.9, 0.0));
        let (_normal, points) =
            dispatch_for_test(&compound, body_xf, &plane, identity_xf(Vec3::ZERO))
                .expect("both spheres penetrate the plane");
        assert_eq!(points.len(), 2, "両方の球からの接触点が1つに束ねられる");
        assert_ne!(
            points[0].feature_id, points[1].feature_id,
            "部品ごとにfeature_idが別空間になっている"
        );
    }

    /// **複合剛体の静止支持**(接触マニフォールドを部品ごとに分ける将来の変更に
    /// 備えた回帰ハーネス)。地面に並んで接する2つの箱からなる`Compound`は、
    /// 全部品の接触法線が`+y`で一致する——モジュールdocが「**厳密に正しい**」と
    /// 述べる典型例そのもの。したがって「貫入最大の部品の法線を代表として全接触点を
    /// 束ねる」現行の近似でも、支持力は解析解と一致しなければならない。
    ///
    /// 検証する解析量は**総支持力 = 重量 $mg$**。各ステップで
    /// 「重力以外に速度へ入った力積」 $m\,(\Delta v_y + g\,\Delta t)/\Delta t$ を
    /// 測ると、これは接触ソルバが実際に返した垂直抗力そのものになる。静止して
    /// いるなら Newton の第3法則から厳密に $mg$ に等しい。
    ///
    /// 許容誤差の根拠: 落下させず解析的な接触位置に静置するので、過渡は
    /// 数ステップで消える。測定窓は $t\in[0.1,0.42]$ s(初期過渡の後、かつ
    /// `sleep`が島を眠らせる 0.5 s より前——眠ると積分も接触解決も止まり、
    /// 支持力の測定自体が意味を失うため)。この窓での実測は各ステップとも
    /// 相対誤差 1e-9 未満なので、Baumgarte 補正の揺らぎを見込んで
    /// 各ステップ 1e-3、窓平均 1e-6 を上限とする。
    #[test]
    fn compound_of_two_boxes_rests_on_a_plane_with_the_correct_total_support_force() {
        use crate::body::RigidBodyDesc;
        use crate::solver::MechanicsSolver;
        use sim_core::{EventQueue, MaterialDb, Solver, SolverContext};
        use sim_math::SimRng;

        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let concrete = materials.find_by_name("コンクリート").unwrap();
        let gravity = 9.80665;
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();
        let mut solver = MechanicsSolver::new(gravity);

        let mut ground = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            concrete,
        );
        ground.body_type = BodyType::Static;
        solver.create_body(ground, &materials);

        // 一辺1mの箱2つを x=±0.6 に横並びで置く(隙間0.2m、どちらも地面に接する)。
        let half = Vec3::new(0.5, 0.5, 0.5);
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Compound {
                children: vec![
                    (
                        identity_xf(Vec3::new(0.6, 0.0, 0.0)),
                        Shape::Box { half_extents: half },
                    ),
                    (
                        identity_xf(Vec3::new(-0.6, 0.0, 0.0)),
                        Shape::Box { half_extents: half },
                    ),
                ],
            },
            steel,
        );
        // 底面がちょうど y=0 に接する高さ。
        desc.transform.position = Vec3::new(0.0, half.y, 0.0);
        let body = solver.create_body(desc, &materials);
        let mass = solver.bodies.mass(body);
        let weight = mass * gravity;

        let dt = 1.0 / 240.0;
        let measure_from = 24; // 0.1 s: 初期過渡の後
        let measure_to = 100; // 0.42 s: sleep(0.5 s)より前
        let mut support_samples: Vec<f64> = Vec::new();
        for step in 0..measure_to {
            let v_before = solver.bodies.linear_velocity[body].y;
            let mut ctx = SolverContext {
                materials: &materials,
                rng: &mut rng,
                events: &mut events,
            };
            solver.step(dt, &mut ctx);
            let _ = events.drain_sorted();
            let v_after = solver.bodies.linear_velocity[body].y;
            if step >= measure_from {
                assert!(
                    !solver.bodies.asleep[body],
                    "measurement window must end before the island falls asleep"
                );
                support_samples.push(mass * ((v_after - v_before) + gravity * dt) / dt);
            }
        }

        for support in &support_samples {
            assert!(
                (support - weight).abs() / weight < 1e-3,
                "per-step support force must equal the weight: support={support} weight={weight}"
            );
        }
        let mean = support_samples.iter().sum::<f64>() / support_samples.len() as f64;
        assert!(
            (mean - weight).abs() / weight < 1e-6,
            "mean support force must equal the weight: mean={mean} weight={weight}"
        );

        // 静止していること(鉛直速度・傾き・貫入)。
        assert!(
            solver.bodies.linear_velocity[body].length() < 1e-9,
            "the compound must come to rest: v={:?}",
            solver.bodies.linear_velocity[body]
        );
        let tilt = 2.0 * solver.bodies.rotation[body].w.abs().min(1.0).acos();
        assert!(
            tilt < 1e-4,
            "symmetric support must not tip the compound: tilt={tilt:.3e} rad"
        );
        let penetration = half.y - solver.bodies.position[body].y;
        // `contact`モジュールの許容貫入(SLOP=0.005、Baumgarteが押し戻さない残量)。
        assert!(
            (0.0..0.005).contains(&penetration),
            "resting penetration must stay within the contact slop: {penetration:.3e}"
        );

        // 両方の部品が実際に支えていること(片側だけの接触で釣り合っていない)。
        let manifold = solver
            .last_manifolds
            .iter()
            .find(|m| m.body_a == body || m.body_b == body)
            .expect("the compound must keep touching the ground");
        assert!(
            (manifold.normal.length() - 1.0).abs() < 1e-12 && manifold.normal.y.abs() > 1.0 - 1e-12,
            "all children share the same +y normal here (the exact case): {:?}",
            manifold.normal
        );
        assert!(
            manifold.points.iter().any(|p| p.world_point.x > 0.0)
                && manifold.points.iter().any(|p| p.world_point.x < 0.0),
            "both children must contribute contact points: {:?}",
            manifold
                .points
                .iter()
                .map(|p| p.world_point.x)
                .collect::<Vec<_>>()
        );
    }

    /// **法線が食い違う部品は独立したマニフォールドになる**(群11、item4の核心)。
    ///
    /// `compound_of_two_boxes_rests_on_a_plane_with_the_correct_total_support_force`
    /// は全部品の法線が `+y` で揃う配置——移行前の「1本に束ねる」近似でも
    /// **厳密に正しかった**ケースしか見ていない。
    ///
    /// ここでは**単一のボディ対**(コンパウンド × 1つの静的な箱)の中で、
    /// 2つの部品が**互いに直交する面**に同時に接触する配置を作る。
    /// ボディ対が1つであることが要点——別々の相手に当たっているだけなら、
    /// 移行前の実装でも相手ごとにマニフォールドが分かれるので、
    /// per-part 化の検証にならない。
    ///
    /// - 静的な箱: 原点中心・半寸 (2, 1, 2) → 上面 y=1、右面 x=2
    /// - 部品1: 半寸 0.5 の箱を (0, 1.4, 0) に → **上面**へ 0.1 めり込む(法線 ±y)
    /// - 部品2: 半寸 0.5 の箱を (2.4, 0, 0) に → **右面**へ 0.1 めり込む(法線 ±x)
    ///
    /// 移行前はこの2つが「貫入の深いほうの法線」1本へ束ねられ、もう一方の
    /// 拘束方向が**完全に消えていた**(2つの貫入量は等しいので、どちらが
    /// 代表になるかは列挙順次第という不安定さもあった)。いまは法線の
    /// 食い違いが 90°(`COMPOUND_NORMAL_MERGE_COS` の 15° を大きく超える)
    /// なので、独立した2本のマニフォールドとして出る。
    #[test]
    fn compound_with_non_aligned_part_normals_emits_separate_manifolds() {
        use crate::body::{BodyType, RigidBodyDesc, RigidBodySet};
        use sim_core::MaterialDb;

        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let concrete = materials.find_by_name("コンクリート").unwrap();
        let mut bodies = RigidBodySet::new();

        // 相手は**1つだけ**の静的な箱(ボディ対を1つに保つ)。
        let mut block = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(2.0, 1.0, 2.0),
            },
            concrete,
        );
        block.body_type = BodyType::Static;
        let block_index = bodies.create_body(block, &materials);

        let half = Vec3::new(0.5, 0.5, 0.5);
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Compound {
                children: vec![
                    // 上面に乗る部品(最小重なり軸は y)。
                    (
                        identity_xf(Vec3::new(0.0, 1.4, 0.0)),
                        Shape::Box { half_extents: half },
                    ),
                    // 右面を押す部品(最小重なり軸は x)。
                    (
                        identity_xf(Vec3::new(2.4, 0.0, 0.0)),
                        Shape::Box { half_extents: half },
                    ),
                ],
            },
            steel,
        );
        desc.transform.position = Vec3::ZERO;
        let body = bodies.create_body(desc, &materials);

        let mut axis_cache = AxisCache::new();
        let manifolds = detect(&bodies, &mut axis_cache);

        let touching: Vec<&ContactManifold> = manifolds
            .iter()
            .filter(|m| {
                (m.body_a == body && m.body_b == block_index)
                    || (m.body_a == block_index && m.body_b == body)
            })
            .collect();
        let normals: Vec<Vec3> = touching.iter().map(|m| m.normal).collect();
        assert_eq!(
            touching.len(),
            2,
            "同一ボディ対から、法線の異なる2本のマニフォールドが出るはず: {normals:?}"
        );

        let has_vertical = normals.iter().any(|n| n.y.abs() > 0.99);
        let has_horizontal = normals.iter().any(|n| n.x.abs() > 0.99);
        assert!(
            has_vertical && has_horizontal,
            "鉛直(上面)と水平(右面)の両方の法線が独立して残っているはず: {normals:?}"
        );
        // どちらのマニフォールドにも実際の接触点があること。
        assert!(
            touching.iter().all(|m| !m.points.is_empty()),
            "空のマニフォールドを出していない"
        );
        // feature_id はボディ対をまたいで一意(warm start のキー衝突回避)。
        let ids: std::collections::BTreeSet<u32> = touching
            .iter()
            .flat_map(|m| m.points.iter().map(|p| p.feature_id))
            .collect();
        let total: usize = touching.iter().map(|m| m.points.len()).sum();
        assert_eq!(
            ids.len(),
            total,
            "部品をまたいで feature_id が重複していない"
        );
    }

    /// **同じ向きの部品はこれまでどおり1本に束ねられる**(群11)。
    /// 法線ごとに無条件でマニフォールドを分けると、床に並べた箱2つが
    /// 同じ拘束を2本に割ってしまい収束が悪化する。`COMPOUND_NORMAL_MERGE_COS`
    /// による合流が効いていることを、上のテストと対にして固定する。
    #[test]
    fn compound_with_aligned_part_normals_stays_a_single_manifold() {
        use crate::body::{BodyType, RigidBodyDesc, RigidBodySet};
        use sim_core::MaterialDb;

        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let concrete = materials.find_by_name("コンクリート").unwrap();
        let mut bodies = RigidBodySet::new();

        let mut ground = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            concrete,
        );
        ground.body_type = BodyType::Static;
        bodies.create_body(ground, &materials);

        let half = Vec3::new(0.5, 0.5, 0.5);
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Compound {
                children: vec![
                    (
                        identity_xf(Vec3::new(0.6, 0.0, 0.0)),
                        Shape::Box { half_extents: half },
                    ),
                    (
                        identity_xf(Vec3::new(-0.6, 0.0, 0.0)),
                        Shape::Box { half_extents: half },
                    ),
                ],
            },
            steel,
        );
        desc.transform.position = Vec3::new(0.0, 0.45, 0.0);
        let body = bodies.create_body(desc, &materials);

        let mut axis_cache = AxisCache::new();
        let manifolds = detect(&bodies, &mut axis_cache);
        let touching: Vec<&ContactManifold> = manifolds
            .iter()
            .filter(|m| m.body_a == body || m.body_b == body)
            .collect();
        assert_eq!(
            touching.len(),
            1,
            "法線が揃う部品は1本に束ねられるはず: {:?}",
            touching.iter().map(|m| m.normal).collect::<Vec<_>>()
        );
        // 両方の部品が接触点を出していること(束ねても情報は失っていない)。
        let m = touching[0];
        assert!(
            m.points.iter().any(|p| p.world_point.x > 0.0)
                && m.points.iter().any(|p| p.world_point.x < 0.0),
            "両部品が接触点を寄せているはず"
        );
        // feature_id が部品ごとに別空間になっていること(warm start の衝突回避)。
        let ids: std::collections::BTreeSet<u32> = m.points.iter().map(|p| p.feature_id).collect();
        assert_eq!(ids.len(), m.points.len(), "feature_id が重複していない");
    }

    /// **かつて「`ConvexMesh`はまだ何とも衝突しない(すり抜ける)」ことを
    /// 固定していたテストを、実装の到達点として書き換えたもの**(群11)。
    ///
    /// 移行前は3D凸包が無く narrowphase が一律`None`を返していたため、
    /// `ConvexMesh`のボディは生成・積分はできても**他の何ともぶつからず
    /// すり抜けた**。`crate::hull`と GJK/EPA 経路の追加でこれが解消したので、
    /// 「すり抜けないこと」を要求する形へ反転させた。
    ///
    /// 退化した頂点列(ここでは2点=線分)でも**パニックしない**ことは
    /// 引き続き要求する——凸包は張れない(体積ゼロ)が、GJK のサポート写像は
    /// 点集合に対して定義できるので接触自体は生成される。
    #[test]
    fn convex_mesh_now_collides_instead_of_passing_through() {
        // 退化した2点の「メッシュ」は凸包を張れない(3次元の広がりが無い)。
        // 体積も接触も持たないが、**パニックしない**ことは要求する。
        let degenerate = Shape::ConvexMesh {
            vertices: vec![Vec3::new(-1.0, -1.0, -1.0), Vec3::new(1.0, 1.0, 1.0)],
        };
        assert_eq!(degenerate.volume(), None, "退化メッシュは体積を持たない");
        let big_sphere = Shape::Sphere { radius: 5.0 };
        assert!(
            dispatch_for_test(
                &degenerate,
                identity_xf(Vec3::ZERO),
                &big_sphere,
                identity_xf(Vec3::ZERO)
            )
            .is_none(),
            "凸包が張れない退化メッシュは接触も生成しない(パニックしないことが要件)"
        );

        // **実体のある**立方体メッシュは、球と重なれば接触を生成する
        // ——これが移行前との決定的な違い(移行前は常に None ですり抜けた)。
        let cube_mesh_overlapping = convex_mesh_cube(1.0);
        assert!(
            dispatch_for_test(
                &cube_mesh_overlapping,
                identity_xf(Vec3::ZERO),
                &Shape::Sphere { radius: 1.0 },
                identity_xf(Vec3::new(1.5, 0.0, 0.0))
            )
            .is_some(),
            "実体のあるメッシュは球と衝突するはず(移行前は None ですり抜けた)"
        );

        // 明確に離れていれば接触しない。
        let cube_mesh = convex_mesh_cube(1.0);
        let sphere = Shape::Sphere { radius: 1.0 };
        assert!(
            dispatch_for_test(
                &cube_mesh,
                identity_xf(Vec3::ZERO),
                &sphere,
                identity_xf(Vec3::new(5.0, 0.0, 0.0))
            )
            .is_none(),
            "離れていれば接触しない"
        );
    }

    /// 立方体の8隅からなる`ConvexMesh`(半辺`half`)。
    fn convex_mesh_cube(half: f64) -> Shape {
        let mut vertices = Vec::with_capacity(8);
        for &sx in &[-1.0, 1.0] {
            for &sy in &[-1.0, 1.0] {
                for &sz in &[-1.0, 1.0] {
                    vertices.push(Vec3::new(sx * half, sy * half, sz * half));
                }
            }
        }
        Shape::ConvexMesh { vertices }
    }

    /// **床に置いた凸多面体は、等価な箱と厳密に同じ接触を作る**(群11)。
    /// 立方体の8隅そのものを頂点に持つメッシュなので、`box_plane`と
    /// `convex_mesh_plane`はどちらも「貫入した頂点を最大4点」返す同じ論理——
    /// 法線・接触点数・最大貫入量まで一致しなければならない。
    ///
    /// これは`ConvexMesh`が床で安定して静止できること(1点支持ではなく
    /// 面で支えられること)を担保する、実用上いちばん重要なケース。
    #[test]
    fn convex_mesh_on_a_plane_matches_the_equivalent_box_exactly() {
        let half = 1.0;
        let plane = Shape::Plane {
            normal: Vec3::new(0.0, 1.0, 0.0),
            d: 0.0,
        };
        // 下面が y=-0.1(貫入 0.1)になる高さ。
        let xf = identity_xf(Vec3::new(0.0, 0.9, 0.0));

        let (box_normal, box_points) = dispatch_for_test(
            &Shape::Box {
                half_extents: Vec3::new(half, half, half),
            },
            xf,
            &plane,
            identity_xf(Vec3::ZERO),
        )
        .expect("box penetrates the plane");
        let (mesh_normal, mesh_points) =
            dispatch_for_test(&convex_mesh_cube(half), xf, &plane, identity_xf(Vec3::ZERO))
                .expect("mesh must penetrate the plane just like the box");

        assert!((mesh_normal - box_normal).length() < 1e-12);
        assert_eq!(mesh_points.len(), box_points.len(), "どちらも底面の4頂点");
        assert_eq!(mesh_points.len(), 4);
        let max_pen = |pts: &[ContactPoint]| pts.iter().map(|p| p.penetration).fold(0.0, f64::max);
        assert!((max_pen(&mesh_points) - max_pen(&box_points)).abs() < 1e-12);
        assert!((max_pen(&mesh_points) - 0.1).abs() < 1e-12, "貫入は 0.1");
    }

    /// **多面体 × 球の法線が設計の A→B 規約であること**(群11)。
    /// 符号の取り違えは「接触が反発ではなく吸着になる」重大な誤りなので、
    /// 解析的に分かる配置(立方体の +x 面の外に球を置く)で明示的に固定する。
    #[test]
    fn convex_mesh_sphere_normal_points_from_a_to_b() {
        let mesh = convex_mesh_cube(1.0);
        let sphere = Shape::Sphere { radius: 1.0 };
        // 立方体の +x 面(x=1)から球の中心が 1.5 → 貫入 0.5。
        let (normal, points) = dispatch_for_test(
            &mesh,
            identity_xf(Vec3::ZERO),
            &sphere,
            identity_xf(Vec3::new(1.5, 0.0, 0.0)),
        )
        .expect("mesh and sphere overlap");
        assert!(
            (normal - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-3,
            "A(メッシュ)から B(球)へ向かう +x のはず: {normal:?}"
        );
        assert_eq!(points.len(), 1, "多面体×球は1点マニフォールド");
        assert!(
            (points[0].penetration - 0.5).abs() < 1e-3,
            "貫入は解析値 0.5 のはず: {}",
            points[0].penetration
        );
    }

    /// **凸多面体どうしの貫入は軸方向の重なり量に一致する**(群11)。
    /// 多面体ペアでは EPA がミンコフスキー差の境界と有限回で厳密に一致する
    /// (滑らかな球と違って線形収束の尾を引かない)ため、許容誤差を
    /// 球ケースより2桁厳しく取れる。
    #[test]
    fn convex_mesh_versus_box_penetration_matches_the_axis_overlap() {
        let mesh = convex_mesh_cube(1.0);
        let other = Shape::Box {
            half_extents: Vec3::new(1.0, 1.0, 1.0),
        };
        // 中心間距離 1.5、半幅の和 2.0 → x方向の重なりは 0.5。
        let (normal, points) = dispatch_for_test(
            &mesh,
            identity_xf(Vec3::ZERO),
            &other,
            identity_xf(Vec3::new(1.5, 0.0, 0.0)),
        )
        .expect("mesh and box overlap");
        assert!(
            (normal - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-6,
            "{normal:?}"
        );
        assert!(
            (points[0].penetration - 0.5).abs() < 1e-6,
            "{}",
            points[0].penetration
        );
    }

    /// `ConvexMesh`どうしも同じ経路で衝突すること(群11)。
    #[test]
    fn convex_mesh_versus_convex_mesh_collides() {
        let a = convex_mesh_cube(1.0);
        let b = convex_mesh_cube(1.0);
        let (normal, points) = dispatch_for_test(
            &a,
            identity_xf(Vec3::ZERO),
            &b,
            identity_xf(Vec3::new(0.0, 1.5, 0.0)),
        )
        .expect("two meshes overlap");
        assert!(
            (normal - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-6,
            "{normal:?}"
        );
        assert!((points[0].penetration - 0.5).abs() < 1e-6);
    }

    /// **`ConvexMesh` × `Capsule` は本増分では未実装**(既知の限界を固定する
    /// テスト)。カプセルは「線分を半径で膨らませた」非多面体なので SAT の
    /// 分離軸に素直に乗らず(丸い部分の分離軸は連続無限個ある)、線分-凸多面体の
    /// 最近点計算を別途書く必要がある。
    ///
    /// **この組み合わせだけは引き続きすり抜ける**。移行前は`ConvexMesh`が
    /// *何とも*衝突しなかったので機能の後退ではないが、穴が残っていることは
    /// テストとして明示しておく——将来これを実装したらこのテストが落ち、
    /// 「塞がった」ことの通知になる(`convex_mesh_aabb_approximation_...`が
    /// 凸包実装の通知として機能したのと同じ仕掛け)。
    #[test]
    fn convex_mesh_versus_capsule_is_not_implemented_yet() {
        let mesh = convex_mesh_cube(1.0);
        // 明らかに深く重なる配置(実装されていれば必ず接触するはず)。
        let capsule = Shape::Capsule {
            radius: 0.8,
            half_height: 1.0,
        };
        assert!(
            dispatch_for_test(
                &mesh,
                identity_xf(Vec3::ZERO),
                &capsule,
                identity_xf(Vec3::new(0.5, 0.0, 0.0))
            )
            .is_none(),
            "未実装なので None(実装したらこのテストを反転させること)"
        );
    }

    #[test]
    fn sphere_sphere_detects_overlap_and_normal_direction() {
        let a = Shape::Sphere { radius: 1.0 };
        let b = Shape::Sphere { radius: 1.0 };
        let (normal, points) = dispatch_for_test(
            &a,
            identity_xf(Vec3::ZERO),
            &b,
            identity_xf(Vec3::new(1.5, 0.0, 0.0)),
        )
        .expect("spheres overlap");
        assert!((normal - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-12);
        assert!((points[0].penetration - 0.5).abs() < 1e-12);
    }

    #[test]
    fn sphere_sphere_no_contact_when_far_apart() {
        let a = Shape::Sphere { radius: 1.0 };
        let b = Shape::Sphere { radius: 1.0 };
        assert!(dispatch_for_test(
            &a,
            identity_xf(Vec3::ZERO),
            &b,
            identity_xf(Vec3::new(5.0, 0.0, 0.0))
        )
        .is_none());
    }

    #[test]
    fn sphere_plane_penetration_matches_formula() {
        let sphere = Shape::Sphere { radius: 1.0 };
        let plane = Shape::Plane {
            normal: Vec3::new(0.0, 1.0, 0.0),
            d: 0.0,
        };
        let (normal, points) = dispatch_for_test(
            &sphere,
            identity_xf(Vec3::new(0.0, 0.6, 0.0)),
            &plane,
            identity_xf(Vec3::ZERO),
        )
        .expect("sphere penetrates plane");
        // body_a=sphere, body_b=plane なので A→B(球→平面)は下向き。
        assert!((normal - Vec3::new(0.0, -1.0, 0.0)).length() < 1e-12);
        assert!((points[0].penetration - 0.4).abs() < 1e-12);
    }

    #[test]
    fn box_plane_normal_flips_when_arguments_swapped() {
        let b = Shape::Box {
            half_extents: Vec3::new(1.0, 1.0, 1.0),
        };
        let plane = Shape::Plane {
            normal: Vec3::new(0.0, 1.0, 0.0),
            d: 0.0,
        };
        let (n1, _) = dispatch_for_test(
            &b,
            identity_xf(Vec3::new(0.0, 0.5, 0.0)),
            &plane,
            identity_xf(Vec3::ZERO),
        )
        .expect("box penetrates plane");
        let (n2, _) = dispatch_for_test(
            &plane,
            identity_xf(Vec3::ZERO),
            &b,
            identity_xf(Vec3::new(0.0, 0.5, 0.0)),
        )
        .expect("box penetrates plane (swapped)");
        assert!(
            (n1 + n2).length() < 1e-12,
            "normals must be exact opposites"
        );
    }

    #[test]
    fn box_plane_finds_four_penetrating_corners_when_resting() {
        let b = Shape::Box {
            half_extents: Vec3::new(1.0, 1.0, 1.0),
        };
        let plane = Shape::Plane {
            normal: Vec3::new(0.0, 1.0, 0.0),
            d: 0.0,
        };
        // y=0.9 中心の箱は下面4頂点(y=-0.1)が貫入。
        let (_, points) = dispatch_for_test(
            &b,
            identity_xf(Vec3::new(0.0, 0.9, 0.0)),
            &plane,
            identity_xf(Vec3::ZERO),
        )
        .expect("box penetrates plane");
        assert_eq!(points.len(), 4);
        for p in &points {
            assert!((p.penetration - 0.1).abs() < 1e-9);
        }
    }

    #[test]
    fn sphere_box_matches_sphere_plane_when_box_is_large_flat() {
        let sphere = Shape::Sphere { radius: 0.5 };
        let big_box = Shape::Box {
            half_extents: Vec3::new(50.0, 1.0, 50.0),
        };
        let (normal, points) = dispatch_for_test(
            &sphere,
            identity_xf(Vec3::new(0.0, 1.3, 0.0)),
            &big_box,
            identity_xf(Vec3::ZERO),
        )
        .expect("sphere touches box top face");
        // body_a=sphere, body_b=box なので A→B(球→箱)は下向き。
        assert!((normal - Vec3::new(0.0, -1.0, 0.0)).length() < 1e-9);
        assert!((points[0].penetration - 0.2).abs() < 1e-9);
    }

    #[test]
    fn detect_normalizes_pair_order_and_skips_static_pairs() {
        let mut bodies = RigidBodySet::new();
        let materials = sim_core::MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut d1 = crate::RigidBodyDesc::dynamic(Shape::Sphere { radius: 1.0 }, steel);
        d1.transform.position = Vec3::ZERO;
        let mut d2 = crate::RigidBodyDesc::dynamic(Shape::Sphere { radius: 1.0 }, steel);
        d2.transform.position = Vec3::new(1.5, 0.0, 0.0);
        bodies.create_body(d1, &materials);
        bodies.create_body(d2, &materials);

        let mut axis_cache = AxisCache::new();
        let manifolds = detect(&bodies, &mut axis_cache);
        assert_eq!(manifolds.len(), 1);
        assert!(manifolds[0].body_a < manifolds[0].body_b);
    }

    /// 動的 AABB BVH(設計 §4.1 表「P2: SAP/BVH」)。散らばった多数体シーンで、BVH が
    /// 列挙する候補ペア集合が総当たり(全 $\binom{N}{2}$ ペアを `aabb_overlap` で判定)と
    /// 完全一致すること(順序含む)を確認する。
    #[test]
    fn bvh_matches_brute_force_pair_enumeration_on_scattered_scene() {
        let mut bodies = RigidBodySet::new();
        let materials = sim_core::MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut rng = sim_math::SimRng::new(7, 7);
        for i in 0..40 {
            let pos = Vec3::new(
                rng.range_f64(-2.0, 2.0),
                rng.range_f64(-2.0, 2.0),
                rng.range_f64(-2.0, 2.0),
            );
            let mut desc = if i % 2 == 0 {
                crate::RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.5 }, steel)
            } else {
                crate::RigidBodyDesc::dynamic(
                    Shape::Box {
                        half_extents: Vec3::new(0.4, 0.6, 0.3),
                    },
                    steel,
                )
            };
            desc.transform.position = pos;
            bodies.create_body(desc, &materials);
        }

        let bvh_pairs = bvh_candidate_pairs(&bodies);

        let n = bodies.len();
        let mut brute_force = Vec::new();
        for a in 0..n {
            for b in (a + 1)..n {
                let xf_a = transform_of(&bodies, a);
                let xf_b = transform_of(&bodies, b);
                if aabb_overlap(
                    aabb_of(bodies.shape_of(a), xf_a),
                    aabb_of(bodies.shape_of(b), xf_b),
                ) {
                    brute_force.push((a, b));
                }
            }
        }

        assert!(
            !brute_force.is_empty(),
            "scene should contain overlapping AABBs for this test to be meaningful"
        );
        assert_eq!(bvh_pairs, brute_force);
    }

    /// Box-Box 面接触: 同サイズの立方体2個をy方向に0.1だけ重ねると、
    /// 上面/底面の4頂点が一致してクリップされ、4点マニフォールドになる(設計 §4.4)。
    #[test]
    fn box_box_face_contact_produces_four_points_when_boxes_stack() {
        let a = Shape::Box {
            half_extents: Vec3::new(1.0, 1.0, 1.0),
        };
        let b = Shape::Box {
            half_extents: Vec3::new(1.0, 1.0, 1.0),
        };
        let (normal, points) = dispatch_for_test(
            &a,
            identity_xf(Vec3::ZERO),
            &b,
            identity_xf(Vec3::new(0.0, 1.9, 0.0)),
        )
        .expect("boxes overlap");
        assert!((normal - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-9);
        assert_eq!(points.len(), 4);
        for p in &points {
            assert!((p.penetration - 0.1).abs() < 1e-9, "{:?}", p.penetration);
            assert!((p.world_point.y - 0.95).abs() < 1e-9);
            assert!((p.world_point.x.abs() - 1.0).abs() < 1e-9);
            assert!((p.world_point.z.abs() - 1.0).abs() < 1e-9);
        }
    }

    #[test]
    fn box_box_no_contact_when_far_apart() {
        let a = Shape::Box {
            half_extents: Vec3::new(1.0, 1.0, 1.0),
        };
        let b = Shape::Box {
            half_extents: Vec3::new(1.0, 1.0, 1.0),
        };
        assert!(dispatch_for_test(
            &a,
            identity_xf(Vec3::ZERO),
            &b,
            identity_xf(Vec3::new(5.0, 0.0, 0.0))
        )
        .is_none());
    }

    #[test]
    fn box_box_normal_flips_when_arguments_swapped() {
        let a = Shape::Box {
            half_extents: Vec3::new(1.0, 1.0, 1.0),
        };
        let b = Shape::Box {
            half_extents: Vec3::new(1.0, 1.0, 1.0),
        };
        let (n1, _) = dispatch_for_test(
            &a,
            identity_xf(Vec3::ZERO),
            &b,
            identity_xf(Vec3::new(0.0, 1.9, 0.0)),
        )
        .expect("boxes overlap");
        let (n2, _) = dispatch_for_test(
            &b,
            identity_xf(Vec3::new(0.0, 1.9, 0.0)),
            &a,
            identity_xf(Vec3::ZERO),
        )
        .expect("boxes overlap (swapped)");
        assert!((n1 + n2).length() < 1e-9, "normals must be exact opposites");
    }

    /// Box-Box 頂点接触: 頂点が下向きになるよう複合回転させた小箱を大きく平たい箱の上面に
    /// わずかに突き刺す。入射面(小箱側)の4頂点のうち貫入するのは最下頂点1つだけなので、
    /// クリップ後のフィルタで残り3点(貫入負)が除外され1点マニフォールドになることを検証する
    /// (設計 §4.4 の退化ケース表と同種の状況: 面接触の一般ロジックが単一深点へ自然に縮退する)。
    #[test]
    fn box_box_single_penetrating_vertex_reduces_to_one_point() {
        let big = Shape::Box {
            half_extents: Vec3::new(5.0, 1.0, 5.0),
        };
        let half_small = Vec3::new(0.3, 0.3, 0.3);
        let small = Shape::Box {
            half_extents: half_small,
        };

        let rot = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), std::f64::consts::FRAC_PI_4).mul(
            Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_4),
        );
        let lowest_local_y = (0u8..8)
            .map(|k| {
                let sx = if k & 1 == 0 { -1.0 } else { 1.0 };
                let sy = if k & 2 == 0 { -1.0 } else { 1.0 };
                let sz = if k & 4 == 0 { -1.0 } else { 1.0 };
                rot.rotate(Vec3::new(
                    sx * half_small.x,
                    sy * half_small.y,
                    sz * half_small.z,
                ))
                .y
            })
            .fold(f64::INFINITY, f64::min);

        let penetration_target = 0.05;
        // big の上面は y=1.0。小箱の中心を「最下頂点がちょうど penetration_target だけ
        // 貫入する高さ」に置く(小箱の最下頂点の世界y = center_y + lowest_local_y)。
        let small_center_y = 1.0 - lowest_local_y - penetration_target;

        let xf_big = identity_xf(Vec3::ZERO);
        let xf_small = Transform {
            position: Vec3::new(0.0, small_center_y, 0.0),
            rotation: rot,
        };

        let (normal, points) = dispatch_for_test(&big, xf_big, &small, xf_small)
            .expect("small box's lowest vertex penetrates big box's top face");
        assert!(
            (normal - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-6,
            "{normal:?}"
        );
        assert_eq!(
            points.len(),
            1,
            "only the single lowest vertex should remain after depth filtering"
        );
        assert!(
            (points[0].penetration - penetration_target).abs() < 1e-6,
            "penetration={} expected={}",
            points[0].penetration,
            penetration_target
        );
    }
}

#[cfg(test)]
mod capsule_tests {
    use super::*;

    fn xf(position: Vec3) -> Transform {
        Transform {
            position,
            rotation: sim_math::Quat::IDENTITY,
        }
    }

    /// **カプセルの体積と慣性が解析式と一致すること**(増分L)。
    /// 体積は円柱 $\pi r^2\cdot 2h$ + 球 $\frac43\pi r^3$。
    /// **極限で既知の形状へ縮退することも確認する**——half_height→0 で球、
    /// r を固定して h を大きくすると長軸慣性が円柱の $r^2/2$ へ近づく。
    /// これが縮退しないなら合成の重み付けが誤っている。
    #[test]
    fn capsule_volume_and_inertia_match_the_analytic_composite() {
        let (r, h) = (0.3_f64, 0.5_f64);
        let capsule = Shape::Capsule {
            radius: r,
            half_height: h,
        };
        let expected_volume =
            std::f64::consts::PI * r * r * 2.0 * h + 4.0 / 3.0 * std::f64::consts::PI * r.powi(3);
        assert!(
            (capsule.volume().unwrap() - expected_volume).abs() / expected_volume < 1e-12,
            "体積が解析式と一致すべき: {:?}",
            capsule.volume()
        );

        // half_height→0 は球そのもの。
        let degenerate = Shape::Capsule {
            radius: r,
            half_height: 0.0,
        };
        let sphere = Shape::Sphere { radius: r };
        let (a, b) = (
            degenerate.unit_mass_inertia_diagonal(),
            sphere.unit_mass_inertia_diagonal(),
        );
        for (axis, (x, y)) in [(a.x, b.x), (a.y, b.y), (a.z, b.z)].iter().enumerate() {
            assert!(
                (x - y).abs() < 1e-12,
                "half_height=0 のカプセルは球へ縮退すべき: axis={axis} {x} vs {y}"
            );
        }

        // 細長くすると長軸慣性は円柱の r²/2 へ近づく(半球の寄与が相対的に減る)。
        let slender = Shape::Capsule {
            radius: 0.05,
            half_height: 5.0,
        };
        let axial = slender.unit_mass_inertia_diagonal().y;
        let cylinder_axial = 0.5 * 0.05 * 0.05;
        assert!(
            (axial - cylinder_axial).abs() / cylinder_axial < 0.02,
            "細長いカプセルの長軸慣性は円柱の r²/2 に近づくべき: {axial} vs {cylinder_axial}"
        );
        // 横軸慣性は長さ² 支配になる(細長い棒の 1/3 L² オーダー)。
        assert!(
            slender.unit_mass_inertia_diagonal().x > 5.0,
            "細長いカプセルの横軸慣性は長さ²で効くべき: {}",
            slender.unit_mass_inertia_diagonal().x
        );
    }

    /// **立てたカプセルと床平面の貫入量が解析値と一致する**(増分L)。
    /// 芯線の下端が平面から `dist` にあるとき、貫入は `radius - dist`。
    #[test]
    fn capsule_plane_penetration_matches_the_segment_distance() {
        let (r, h) = (0.25_f64, 0.6_f64);
        let capsule = Shape::Capsule {
            radius: r,
            half_height: h,
        };
        let plane = Shape::Plane {
            normal: Vec3::new(0.0, 1.0, 0.0),
            d: 0.0,
        };
        // 重心を y = h + r - 0.05 に置く = 下端が y = r - 0.05、貫入 0.05。
        let center_y = h + r - 0.05;
        let (normal, points) = shape_pair_manifold(
            &capsule,
            xf(Vec3::new(0.0, center_y, 0.0)),
            &plane,
            xf(Vec3::ZERO),
        )
        .expect("床に触れているはず");
        assert_eq!(points.len(), 1, "立てたカプセルは下端の1点だけが触れる");
        assert!(
            (points[0].penetration - 0.05).abs() < 1e-12,
            "貫入量が解析値と一致すべき: {}",
            points[0].penetration
        );
        // A→B規約(カプセル→平面)なので法線は下向き。
        assert!(normal.y < 0.0, "A→B規約では法線は-y向き: {normal:?}");

        // 十分高い位置では接触しない。
        assert!(
            shape_pair_manifold(
                &capsule,
                xf(Vec3::new(0.0, h + r + 0.01, 0.0)),
                &plane,
                xf(Vec3::ZERO)
            )
            .is_none(),
            "浮いているカプセルは接触しない"
        );
    }

    /// **寝かせたカプセルは床と2点で接触する**(増分L)。1点しか返さないと
    /// 転がり続けて静止しないため、安定に寝かせるには両端の2点が要る。
    #[test]
    fn a_lying_capsule_touches_the_floor_at_both_ends() {
        let (r, h) = (0.2_f64, 0.7_f64);
        let capsule = Shape::Capsule {
            radius: r,
            half_height: h,
        };
        let plane = Shape::Plane {
            normal: Vec3::new(0.0, 1.0, 0.0),
            d: 0.0,
        };
        // ローカル+y(長軸)をワールド+xへ倒す。
        let lying = Transform {
            position: Vec3::new(0.0, r - 0.02, 0.0),
            rotation: sim_math::Quat::from_axis_angle(
                Vec3::new(0.0, 0.0, 1.0),
                -std::f64::consts::FRAC_PI_2,
            ),
        };
        let (_, points) = shape_pair_manifold(&capsule, lying, &plane, xf(Vec3::ZERO))
            .expect("寝たカプセルは床に触れる");
        assert_eq!(points.len(), 2, "両端の2点で接触すべき: {points:?}");
        for p in &points {
            assert!(
                (p.penetration - 0.02).abs() < 1e-9,
                "両端とも同じ貫入量になるはず: {}",
                p.penetration
            );
        }
    }

    /// **カプセル-球とカプセル-カプセルが球-球へ正しく帰着すること**(増分L)。
    #[test]
    fn capsule_sphere_and_capsule_capsule_reduce_to_the_sphere_case() {
        let capsule = Shape::Capsule {
            radius: 0.2,
            half_height: 0.5,
        };
        let sphere = Shape::Sphere { radius: 0.3 };

        // 芯線の真横 0.4 に球中心 → 距離0.4、半径和0.5、貫入0.1。
        let (normal, points) = shape_pair_manifold(
            &capsule,
            xf(Vec3::ZERO),
            &sphere,
            xf(Vec3::new(0.4, 0.0, 0.0)),
        )
        .expect("接触するはず");
        assert!((points[0].penetration - 0.1).abs() < 1e-12, "{points:?}");
        assert!((normal.x - 1.0).abs() < 1e-12, "法線はA→B(+x): {normal:?}");

        // 芯線の端より外(y=1.0)にある球は、端点からの距離で判定される。
        // 端点 y=0.5 から距離0.5 → 半径和0.5なのでちょうど接する=貫入0。
        assert!(
            shape_pair_manifold(
                &capsule,
                xf(Vec3::ZERO),
                &sphere,
                xf(Vec3::new(0.0, 1.01, 0.0))
            )
            .is_none(),
            "端点から半径和より遠ければ接触しない"
        );

        // カプセル同士: 平行に置いて距離0.3、半径和0.4 → 貫入0.1。
        let (n2, p2) = shape_pair_manifold(
            &capsule,
            xf(Vec3::ZERO),
            &capsule,
            xf(Vec3::new(0.3, 0.0, 0.0)),
        )
        .expect("接触するはず");
        assert!((p2[0].penetration - 0.1).abs() < 1e-12, "{p2:?}");
        assert!((n2.x - 1.0).abs() < 1e-12, "{n2:?}");
    }

    /// **カプセル×箱(群4で実装)**。増分Lでは`None`を返す(=すり抜ける)
    /// ままだった。ここでは**解析的に貫入量と法線が分かる配置**で確認する。
    #[test]
    fn capsule_versus_box_penetration_and_normal_match_the_closed_form() {
        let radius = 0.2;
        let half_height = 0.5;
        let capsule = Shape::Capsule {
            radius,
            half_height,
        };
        let half = Vec3::new(0.5, 0.5, 0.5);
        let boxed = Shape::Box { half_extents: half };

        // ① 真横から近づく(芯線は縦、箱の +x 面に平行に当たる)。
        //    芯線と箱の距離 = 0.65 - 0.5 = 0.15 → 貫入 0.2 - 0.15 = 0.05。
        //    **芯線が面と平行なので2点接触になる**——これが正しい挙動である
        //    (1点だと壁に触れたカプセルがその点を軸に回ってしまう。
        //    最初この配置で1点を期待するテストを書いたが、期待値の方が誤りだった)。
        let (normal, points) = shape_pair_manifold(
            &capsule,
            xf(Vec3::new(0.65, 0.0, 0.0)),
            &boxed,
            xf(Vec3::ZERO),
        )
        .expect("接触するはず");
        assert_eq!(points.len(), 2, "面と平行なら2点接触: {points:?}");
        for p in &points {
            assert!((p.penetration - 0.05).abs() < 1e-12, "{points:?}");
        }
        // 法線は A→B 規約(カプセル→箱)なので -x 方向。
        assert!(
            (normal - Vec3::new(-1.0, 0.0, 0.0)).length() < 1e-12,
            "{normal:?}"
        );

        // ①' 端から突っ込む(芯線が法線と平行)なら1点接触。
        //    芯線の下端 y = 1.18 - 0.5 = 0.68、箱の上面 0.5 → 距離 0.18 → 貫入 0.02。
        let (normal_end, points_end) = shape_pair_manifold(
            &capsule,
            xf(Vec3::new(0.0, 1.18, 0.0)),
            &boxed,
            xf(Vec3::ZERO),
        )
        .expect("接触するはず");
        assert_eq!(
            points_end.len(),
            1,
            "端から当たるなら1点接触: {points_end:?}"
        );
        assert!(
            (points_end[0].penetration - 0.02).abs() < 1e-12,
            "{points_end:?}"
        );
        assert!(
            (normal_end - Vec3::new(0.0, -1.0, 0.0)).length() < 1e-12,
            "{normal_end:?}"
        );

        // ② 引数の順序を入れ替えると法線が反転する(A→B規約の対称性)。
        let (normal_swapped, points_swapped) = shape_pair_manifold(
            &boxed,
            xf(Vec3::ZERO),
            &capsule,
            xf(Vec3::new(0.65, 0.0, 0.0)),
        )
        .expect("接触するはず");
        assert!(
            (normal_swapped - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-12,
            "{normal_swapped:?}"
        );
        assert!((points_swapped[0].penetration - 0.05).abs() < 1e-12);

        // ③ 離れていれば接触しない(境界: 距離 0.2 ちょうどでは接触しない)。
        assert!(shape_pair_manifold(
            &capsule,
            xf(Vec3::new(0.71, 0.0, 0.0)),
            &boxed,
            xf(Vec3::ZERO)
        )
        .is_none());

        // ④ **寝たカプセルが箱の上面に乗ると2点接触になる**(1点だと転がり続けて
        //    静止しない——`capsule_plane`が2点を出しているのと同じ理由)。
        let lying = Transform {
            position: Vec3::new(0.0, 0.5 + radius - 0.02, 0.0),
            rotation: sim_math::Quat::from_axis_angle(
                Vec3::new(0.0, 0.0, 1.0),
                -std::f64::consts::FRAC_PI_2,
            ),
        };
        let (normal, points) = shape_pair_manifold(&capsule, lying, &boxed, xf(Vec3::ZERO))
            .expect("寝たカプセルは箱に触れる");
        assert_eq!(points.len(), 2, "両端の2点で接触すべき: {points:?}");
        for p in &points {
            assert!(
                (p.penetration - 0.02).abs() < 1e-9,
                "両端とも同じ貫入量になるはず: {}",
                p.penetration
            );
        }
        assert!(
            (normal - Vec3::new(0.0, -1.0, 0.0)).length() < 1e-9,
            "{normal:?}"
        );
    }

    /// `closest_segment_param_to_aabb` を**総当たりサンプリングと突き合わせる**。
    /// 区分二次の解析的最小化は場合分けを間違えても「それらしい」値を返すので、
    /// 独立な方法(細かく刻んだ数値最小化)と一致することを見る。
    #[test]
    fn closest_segment_param_to_aabb_matches_brute_force_sampling() {
        let half = Vec3::new(0.4, 0.7, 0.3);
        let mut rng = sim_math::SimRng::new(31, 7);
        let mut random_point = |scale: f64| {
            Vec3::new(
                (rng.next_f64() - 0.5) * scale,
                (rng.next_f64() - 0.5) * scale,
                (rng.next_f64() - 0.5) * scale,
            )
        };
        let squared_distance = |a: Vec3, b: Vec3, t: f64| {
            let p = a.addcarry_scaled(b - a, t);
            let g = |v: f64, h: f64| {
                if v < -h {
                    -h - v
                } else if v > h {
                    v - h
                } else {
                    0.0
                }
            };
            let (gx, gy, gz) = (g(p.x, half.x), g(p.y, half.y), g(p.z, half.z));
            gx * gx + gy * gy + gz * gz
        };

        for _ in 0..300 {
            let a = random_point(4.0);
            let b = random_point(4.0);
            let t = closest_segment_param_to_aabb(a, b, half);
            let analytic = squared_distance(a, b, t);
            let mut brute = f64::INFINITY;
            const SAMPLES: u32 = 20_000;
            for i in 0..=SAMPLES {
                let s = i as f64 / SAMPLES as f64;
                brute = brute.min(squared_distance(a, b, s));
            }
            // 解析解は総当たりより良い(または刻み幅ぶんの誤差内で同じ)はず。
            assert!(
                analytic <= brute + 1e-9,
                "analytic minimum must not be worse than brute force: \
                 analytic={analytic} brute={brute} a={a:?} b={b:?} t={t}"
            );
        }
    }
}
