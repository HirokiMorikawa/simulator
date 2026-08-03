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
        Shape::Compound { .. } | Shape::ConvexMesh { .. } => {
            todo!("Phase 5 で実装")
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

        _ => todo!("Compound/ConvexMesh は Phase 5"),
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
