//! Broadphase / narrowphase / 接触マニフォールド。
//! 設計: docs/10-mechanics/02-collision-detection.md §3/§4。
//!
//! Phase 1 スコープ: 総当たり broadphase + Sphere-Sphere/Sphere-Plane/Box-Plane/Sphere-Box
//! narrowphase(§4.2 の表の Phase 1 行)。Box-Box(SAT)は Phase 2、Capsule 系は Phase 2、
//! GJK/EPA は Phase 5。

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
        (Shape::Box { .. }, Shape::Box { .. }) => {
            todo!("Box-Box SAT は Phase 2(docs/10-mechanics/02-collision-detection.md §4.4)")
        }
        (Shape::Plane { .. }, Shape::Plane { .. }) => None, // static同士は broadphase で除外すべき無意味ペア
        _ => todo!("Capsule/Compound/ConvexMesh は Phase 2/5"),
    }
}

/// 総当たり broadphase(§4.1)+ narrowphase ディスパッチ(§4.2)。
/// ペア列挙順は (indexA, indexB) 昇順に固定(決定論)。
pub fn detect(bodies: &RigidBodySet) -> Vec<ContactManifold> {
    let n = bodies.len();
    let mut manifolds = Vec::new();
    for a in 0..n {
        for b in (a + 1)..n {
            // static/kinematic 同士は無意味ペア(設計 §4.4 表)。
            if bodies.body_type[a] != BodyType::Dynamic && bodies.body_type[b] != BodyType::Dynamic
            {
                continue;
            }
            let xf_a = transform_of(bodies, a);
            let xf_b = transform_of(bodies, b);
            let shape_a = bodies.shape_of(a);
            let shape_b = bodies.shape_of(b);
            if !aabb_overlap(aabb_of(shape_a, xf_a), aabb_of(shape_b, xf_b)) {
                continue;
            }
            if let Some((normal, points)) = shape_pair_manifold(shape_a, xf_a, shape_b, xf_b) {
                manifolds.push(ContactManifold {
                    body_a: a,
                    body_b: b,
                    normal,
                    points,
                });
            }
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

        let manifolds = detect(&bodies);
        assert_eq!(manifolds.len(), 1);
        assert!(manifolds[0].body_a < manifolds[0].body_b);
    }
}
