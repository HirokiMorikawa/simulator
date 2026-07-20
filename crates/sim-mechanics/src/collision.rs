//! Broadphase / narrowphase / 接触マニフォールド。
//! 設計: docs/10-mechanics/02-collision-detection.md §3/§4。
//!
//! Phase 1: 総当たり broadphase + Sphere-Sphere/Sphere-Plane/Box-Plane/Sphere-Box
//! narrowphase(§4.2 の表の Phase 1 行)。
//! Phase 2: Box-Box(SAT、§4.4)。軸選択のヒステリシス・マニフォールド持続化(§4.7、warm
//! starting の前提)は未実装 — 多段スタック(M12)で貫入が slop を超える既知の制限として
//! 残る(docs/22-roadmap/02-feature-checklist.md に記録)。Capsule 系は Phase 2、GJK/EPA は Phase 5。

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

fn transform_of(bodies: &RigidBodySet, i: usize) -> Transform {
    Transform {
        position: bodies.position[i],
        rotation: bodies.rotation[i],
    }
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
        Shape::Capsule { .. } | Shape::Compound { .. } | Shape::ConvexMesh { .. } => {
            todo!("Phase 2/5 で実装")
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

/// 球 と 箱: ボックスローカルで最近点にクランプ。
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
/// マニフォールド持続化(§4.7、feature_id の移動量チェックによる再利用判定)は未実装。
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
        _ => todo!("Capsule/Compound/ConvexMesh は Phase 2/5"),
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
        let xf_a = transform_of(bodies, a);
        let xf_b = transform_of(bodies, b);
        let shape_a = bodies.shape_of(a);
        let shape_b = bodies.shape_of(b);
        let result = if let (Shape::Box { half_extents: ha }, Shape::Box { half_extents: hb }) =
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
            r.map(|(n, p, _)| (n, p))
        } else {
            shape_pair_manifold(shape_a, xf_a, shape_b, xf_b)
        };
        if let Some((normal, points)) = result {
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
            rotation: Quat::IDENTITY,
        }
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
