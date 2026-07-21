//! `overlap_sphere`クエリ。設計 docs/20-integration/04-world-api.md §2
//! `overlap_sphere(center, r, filter) -> Vec<BodyId>`。
//!
//! **縮約実装の理由**: `raycast`モジュールと同じ理由で`filter`引数を省略する。対象形状も
//! 同様に`Sphere`/`Box`/`Plane`のみ(`Capsule`/`Compound`/`ConvexMesh`は未実装)。
//! `Box`との判定は`sim_mechanics::collision::sphere_box`と全く同じ「ローカル空間で
//! クランプして最近接点を求める」手法を使う(接触解決の narrowphase と同一の幾何)。

use sim_math::{Transform, Vec3};
use sim_mechanics::{RigidBodySet, Shape};

/// クエリ球(`center`, `r`)と重なる全剛体の`RigidBodySet`indexを返す(呼び出し側で
/// `BodyId`へ変換する、`World::overlap_sphere`参照)。
pub fn overlap_sphere(bodies: &RigidBodySet, center: Vec3, r: f64) -> Vec<usize> {
    let mut hits = Vec::new();
    for i in 0..bodies.len() {
        let overlaps = match bodies.shape_of(i) {
            Shape::Sphere { radius } => {
                let dist_sq = (bodies.position[i] - center).length_sq();
                dist_sq <= (radius + r) * (radius + r)
            }
            Shape::Box { half_extents } => {
                let xf = Transform {
                    position: bodies.position[i],
                    rotation: bodies.rotation[i],
                };
                let local_center = xf.inverse().apply_point(center);
                let clamped = Vec3::new(
                    local_center.x.clamp(-half_extents.x, half_extents.x),
                    local_center.y.clamp(-half_extents.y, half_extents.y),
                    local_center.z.clamp(-half_extents.z, half_extents.z),
                );
                (local_center - clamped).length_sq() <= r * r
            }
            Shape::Plane { normal, d } => {
                let dist = normal.dot(center) - d;
                dist.abs() <= r
            }
            Shape::Capsule { .. } | Shape::Compound { .. } | Shape::ConvexMesh { .. } => false,
        };
        if overlaps {
            hits.push(i);
        }
    }
    hits
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::MaterialDb;
    use sim_mechanics::RigidBodyDesc;

    fn steel_sphere_at(materials: &MaterialDb, position: Vec3, radius: f64) -> RigidBodyDesc {
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, steel);
        desc.transform.position = position;
        desc
    }

    /// 球同士の重なり判定: 中心間距離が半径和以下なら重なる。
    #[test]
    fn overlap_sphere_detects_overlapping_sphere_and_excludes_far_one() {
        let materials = MaterialDb::standard();
        let mut bodies = RigidBodySet::new();
        bodies.create_body(
            steel_sphere_at(&materials, Vec3::new(1.5, 0.0, 0.0), 1.0),
            &materials,
        );
        bodies.create_body(
            steel_sphere_at(&materials, Vec3::new(100.0, 0.0, 0.0), 1.0),
            &materials,
        );

        let hits = overlap_sphere(&bodies, Vec3::ZERO, 1.0);
        assert_eq!(hits, vec![0]);
    }

    /// 回転した箱との重なり判定(ローカル空間クランプ法の検証): 箱の角付近の球が
    /// 重なることを確認する。
    #[test]
    fn overlap_sphere_detects_rotated_box_overlap() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(1.0, 1.0, 1.0),
            },
            steel,
        );
        desc.transform.rotation =
            sim_math::Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), std::f64::consts::FRAC_PI_4);
        let mut bodies = RigidBodySet::new();
        bodies.create_body(desc, &materials);

        // 45度回転した半径1の立方体の面までの最短距離はsqrt(2)。中心から
        // 1.5だけ離れた球(半径1)は面に0.5だけ食い込むはず。
        let query_center = Vec3::new(1.5, 0.0, 0.0);
        assert_eq!(overlap_sphere(&bodies, query_center, 1.0), vec![0]);
        // 十分離れていれば重ならない。
        assert!(overlap_sphere(&bodies, Vec3::new(10.0, 0.0, 0.0), 1.0).is_empty());
    }

    /// 平面との重なり判定は剛体のtransformとは独立にワールド座標の`normal`/`d`で行う
    /// (`raycast`モジュールの`Shape::Plane`扱いと同じ)。
    #[test]
    fn overlap_sphere_detects_plane_overlap_using_world_space_normal() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let desc = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            steel,
        );
        let mut bodies = RigidBodySet::new();
        bodies.create_body(desc, &materials);

        assert_eq!(
            overlap_sphere(&bodies, Vec3::new(0.0, 0.5, 0.0), 1.0),
            vec![0]
        );
        assert!(overlap_sphere(&bodies, Vec3::new(0.0, 10.0, 0.0), 1.0).is_empty());
    }
}
