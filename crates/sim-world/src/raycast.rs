//! レイキャストクエリ。設計 docs/20-integration/04-world-api.md §2
//! `raycast(origin, dir, max, filter) -> Option<RayHit>`。
//!
//! **縮約実装の理由**: `filter`引数は設計に具体的な型が示されておらず(`Filter`は
//! シグネチャに現れるのみで定義が無い)、本実装では省略する(将来レイヤー/BodyId除外
//! フィルタとして追加する)。対象形状は`sim_mechanics::collision`のnarrowphaseが
//! 現時点で実装済みの`Sphere`/`Box`/`Plane`のみ(`Capsule`/`Compound`/`ConvexMesh`は
//! P2/P5未実装、同モジュールのdoc参照)。
//!
//! `Box`は剛体のtransform(位置+回転)のローカル空間で軸並行境界ボックスとして判定する
//! (`collision::sphere_box`が`box_xf.inverse().apply_point`で行うのと同じ変換)。
//! `Sphere`は姿勢が意味を持たないため位置のみで判定する。`Plane`は
//! `collision::sphere_plane`と同様、法線`normal`・原点距離`d`をワールド座標系の値として
//! 直接使う(所有剛体のtransformとは独立、`Shape::Plane`のdoc「static専用・無限平面」
//! 参照)。

use sim_math::{Transform, Vec3};
use sim_mechanics::{RigidBodySet, Shape};

/// レイヒット結果。`body_index`は`RigidBodySet`の生インデックス(呼び出し側で`BodyId`へ
/// 変換する、`World::raycast`参照)。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    pub body_index: usize,
    pub point: Vec3,
    pub normal: Vec3,
    pub distance: f64,
}

/// 全剛体に対しレイを飛ばし、最も近いヒットを返す(`max_distance`以内、`dir`はゼロ
/// ベクトルなら`None`)。
pub fn raycast(
    bodies: &RigidBodySet,
    origin: Vec3,
    dir: Vec3,
    max_distance: f64,
) -> Option<RayHit> {
    let dir = dir.normalize_or_zero();
    if dir == Vec3::ZERO {
        return None;
    }
    let mut best: Option<RayHit> = None;
    for i in 0..bodies.len() {
        let hit = match bodies.shape_of(i) {
            Shape::Sphere { radius } => ray_sphere(origin, dir, bodies.position[i], *radius),
            Shape::Box { half_extents } => {
                let xf = Transform {
                    position: bodies.position[i],
                    rotation: bodies.rotation[i],
                };
                let inv = xf.inverse();
                let local_origin = inv.apply_point(origin);
                let local_dir = inv.apply_dir(dir);
                ray_box_local(local_origin, local_dir, *half_extents)
                    .map(|(t, local_normal)| (t, xf.apply_dir(local_normal)))
            }
            Shape::Plane { normal, d } => ray_plane(origin, dir, *normal, *d),
            Shape::Capsule { .. } | Shape::Compound { .. } | Shape::ConvexMesh { .. } => None,
        };
        if let Some((t, normal)) = hit {
            if t >= 0.0 && t <= max_distance {
                let better = best.as_ref().is_none_or(|b| t < b.distance);
                if better {
                    best = Some(RayHit {
                        body_index: i,
                        point: origin.addcarry_scaled(dir, t),
                        normal: normal.normalize_or_zero(),
                        distance: t,
                    });
                }
            }
        }
    }
    best
}

/// ワールド空間の球(姿勢は無意味なので中心+半径のみ)。
fn ray_sphere(origin: Vec3, dir: Vec3, center: Vec3, radius: f64) -> Option<(f64, Vec3)> {
    let oc = origin - center;
    let b = oc.dot(dir);
    let c = oc.dot(oc) - radius * radius;
    let disc = b * b - c;
    if disc < 0.0 {
        return None;
    }
    let sqrt_disc = disc.sqrt();
    let t0 = -b - sqrt_disc;
    let t1 = -b + sqrt_disc;
    let t = if t0 >= 0.0 {
        t0
    } else if t1 >= 0.0 {
        t1
    } else {
        return None;
    };
    let point = origin.addcarry_scaled(dir, t);
    Some((t, (point - center).normalize_or_zero()))
}

/// ワールド空間の無限平面(`Shape::Plane`のdoc参照、姿勢とは独立)。
fn ray_plane(origin: Vec3, dir: Vec3, normal: Vec3, d: f64) -> Option<(f64, Vec3)> {
    let denom = normal.dot(dir);
    if denom.abs() < 1e-12 {
        return None; // レイが平面に平行。
    }
    let t = (d - normal.dot(origin)) / denom;
    if t < 0.0 {
        return None;
    }
    Some((t, normal))
}

/// ローカル空間(箱の中心が原点、軸並行)のスラブ法。返す法線もローカル空間
/// (呼び出し側でワールドへ回転する)。
fn ray_box_local(origin: Vec3, dir: Vec3, half_extents: Vec3) -> Option<(f64, Vec3)> {
    let mut t_min = f64::NEG_INFINITY;
    let mut t_max = f64::INFINITY;
    let mut normal = Vec3::ZERO;

    let axes = [
        (origin.x, dir.x, half_extents.x, Vec3::new(1.0, 0.0, 0.0)),
        (origin.y, dir.y, half_extents.y, Vec3::new(0.0, 1.0, 0.0)),
        (origin.z, dir.z, half_extents.z, Vec3::new(0.0, 0.0, 1.0)),
    ];
    for (o, dc, he, axis) in axes {
        if dc.abs() < 1e-12 {
            if o < -he || o > he {
                return None; // スラブに平行かつ範囲外。
            }
            continue;
        }
        let inv = 1.0 / dc;
        let t1 = (-he - o) * inv;
        let t2 = (he - o) * inv;
        let (t_near, t_far, n_near) = if t1 <= t2 {
            (t1, t2, axis.scale(-1.0))
        } else {
            (t2, t1, axis)
        };
        if t_near > t_min {
            t_min = t_near;
            normal = n_near;
        }
        t_max = t_max.min(t_far);
    }

    if t_min > t_max || t_max < 0.0 {
        return None;
    }
    let t = if t_min >= 0.0 { t_min } else { t_max };
    Some((t, normal))
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::MaterialDb;
    use sim_mechanics::{BodyType, RigidBodyDesc};

    fn steel_sphere_at(materials: &MaterialDb, position: Vec3, radius: f64) -> RigidBodyDesc {
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, steel);
        desc.transform.position = position;
        desc
    }

    /// 球への正面からのレイキャスト: ヒット距離・法線が解析的に一致することを確認する。
    #[test]
    fn raycast_hits_sphere_at_expected_distance_and_normal() {
        let materials = MaterialDb::standard();
        let mut bodies = RigidBodySet::new();
        bodies.create_body(
            steel_sphere_at(&materials, Vec3::new(5.0, 0.0, 0.0), 1.0),
            &materials,
        );

        let hit = raycast(&bodies, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), 100.0)
            .expect("ray along +x should hit the sphere");
        assert_eq!(hit.body_index, 0);
        assert!(
            (hit.distance - 4.0).abs() < 1e-9,
            "distance={}",
            hit.distance
        );
        assert!(
            (hit.normal - Vec3::new(-1.0, 0.0, 0.0)).length() < 1e-9,
            "normal={:?}",
            hit.normal
        );
        assert!(
            (hit.point - Vec3::new(4.0, 0.0, 0.0)).length() < 1e-9,
            "point={:?}",
            hit.point
        );
    }

    /// レイが的から外れる(球に当たらない)場合は`None`。
    #[test]
    fn raycast_misses_sphere_when_ray_passes_beside_it() {
        let materials = MaterialDb::standard();
        let mut bodies = RigidBodySet::new();
        bodies.create_body(
            steel_sphere_at(&materials, Vec3::new(5.0, 5.0, 0.0), 1.0),
            &materials,
        );

        assert!(raycast(&bodies, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), 100.0).is_none());
    }

    /// `max_distance`より遠いヒットは無視される。
    #[test]
    fn raycast_respects_max_distance() {
        let materials = MaterialDb::standard();
        let mut bodies = RigidBodySet::new();
        bodies.create_body(
            steel_sphere_at(&materials, Vec3::new(5.0, 0.0, 0.0), 1.0),
            &materials,
        );

        assert!(raycast(&bodies, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), 3.0).is_none());
    }

    /// 軸に平行な回転済み箱へのレイキャスト(ローカル空間変換の検証、45°回転させた
    /// 箱の対角線上からの正面ヒット)。
    #[test]
    fn raycast_hits_rotated_box_in_local_space() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(1.0, 1.0, 1.0),
            },
            steel,
        );
        desc.transform.position = Vec3::new(10.0, 0.0, 0.0);
        desc.transform.rotation =
            sim_math::Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), std::f64::consts::FRAC_PI_4);
        let mut bodies = RigidBodySet::new();
        bodies.create_body(desc, &materials);

        let hit = raycast(&bodies, Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), 100.0)
            .expect("ray along +x should hit the rotated box");
        // 45度回転した半径1の立方体の最近接面までの距離は 1*sqrt(2) ≈ 1.41421。
        let expected_distance = 10.0 - std::f64::consts::SQRT_2;
        assert!(
            (hit.distance - expected_distance).abs() < 1e-9,
            "distance={} expected={}",
            hit.distance,
            expected_distance
        );
    }

    /// 平面へのレイキャスト: 剛体のtransformとは独立にワールド座標の`normal`/`d`で
    /// 判定される(`Shape::Plane`のdoc参照)。
    #[test]
    fn raycast_hits_plane_using_world_space_normal_independent_of_body_transform() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            steel,
        );
        desc.body_type = BodyType::Static;
        // 剛体自身のtransformはPlaneの幾何には影響しない(モジュールdoc参照)ことを
        // わざと非自明な位置に置いて確認する。
        desc.transform.position = Vec3::new(123.0, 456.0, 789.0);
        let mut bodies = RigidBodySet::new();
        bodies.create_body(desc, &materials);

        let hit = raycast(
            &bodies,
            Vec3::new(0.0, 10.0, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
            100.0,
        )
        .expect("ray straight down should hit the y=0 plane");
        assert!((hit.distance - 10.0).abs() < 1e-9);
        assert!((hit.normal - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-9);
    }
}
