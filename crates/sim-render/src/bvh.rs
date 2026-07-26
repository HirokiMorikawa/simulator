//! BVH(境界ボリューム階層)によるレイ交差の高速化。設計docs/17-rendering/
//! 02-path-tracing.md §4「BVH: 三角形メッシュ + 解析形状(球・平面)」。
//!
//! **縮約実装の理由**: 対象形状は`Sphere`のみ(三角形メッシュは`sphere.rs`
//! モジュールdoc参照のとおり未実装)。分割戦略は最長軸の重心中央値
//! (median split)による単純なトップダウン構築——SAH(Surface Area
//! Heuristic)による最適分割は後続増分。この木構造は`sim_mechanics::
//! collision`のBVH broadphase(ボディペア列挙)とは意図的に別実装(あちらの
//! 内部型は非公開かつ「レイの最近傍ヒットを再帰探索する」という本モジュールの
//! クエリとは異なる形状のクエリ(重なりペア列挙)のため共有しない、
//! `bsdf.rs`モジュールdocの「意図的な型分離」と同じ方針)。既存の
//! `Scene::closest_hit`(総当たり)への配線は、対象になり得る多数物体シーン
//! (D40–D43)がまだ存在しないため後続増分に残す——本増分は加速構造自体の
//! 正しさ(総当たりと厳密一致)と、実際に部分木を刈っていること(全ノードを
//! 訪問しないこと)の検証に留める。

use crate::ray::Ray;
use crate::sphere::{Hit, Sphere};
use sim_math::Vec3;

#[derive(Clone, Copy, Debug)]
struct Aabb {
    min: Vec3,
    max: Vec3,
}

impl Aabb {
    fn of_sphere(s: &Sphere) -> Aabb {
        let r = Vec3::new(s.radius, s.radius, s.radius);
        Aabb {
            min: s.center - r,
            max: s.center + r,
        }
    }

    fn union(a: Aabb, b: Aabb) -> Aabb {
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

    /// スラブ法によるレイ-AABB交差判定。`[t_min, t_max]`区間との重なりがあれば
    /// `true`(設計docs/17-rendering/02-path-tracing.md §4の標準的な手法)。
    fn intersects_ray(&self, ray: &Ray, t_min: f64, t_max: f64) -> bool {
        let mut lo = t_min;
        let mut hi = t_max;
        for axis in 0..3 {
            let (origin, dir, min, max) = match axis {
                0 => (ray.origin.x, ray.direction.x, self.min.x, self.max.x),
                1 => (ray.origin.y, ray.direction.y, self.min.y, self.max.y),
                _ => (ray.origin.z, ray.direction.z, self.min.z, self.max.z),
            };
            if dir.abs() < 1e-12 {
                if origin < min || origin > max {
                    return false;
                }
                continue;
            }
            let inv_dir = 1.0 / dir;
            let mut t0 = (min - origin) * inv_dir;
            let mut t1 = (max - origin) * inv_dir;
            if t0 > t1 {
                std::mem::swap(&mut t0, &mut t1);
            }
            lo = lo.max(t0);
            hi = hi.min(t1);
            if lo > hi {
                return false;
            }
        }
        true
    }
}

enum BvhNode {
    Leaf {
        bounds: Aabb,
        sphere_index: usize,
    },
    Internal {
        bounds: Aabb,
        left: Box<BvhNode>,
        right: Box<BvhNode>,
    },
}

impl BvhNode {
    fn bounds(&self) -> Aabb {
        match self {
            BvhNode::Leaf { bounds, .. } => *bounds,
            BvhNode::Internal { bounds, .. } => *bounds,
        }
    }
}

/// 球群に対するBVH。`Bvh::build`で構築し、`Bvh::closest_hit`でレイの最近傍
/// ヒット(元の`spheres`スライスにおけるindex込み)を返す。
pub struct Bvh {
    root: Option<BvhNode>,
}

/// 診断用の交差テスト回数(`closest_hit_with_diagnostics`が返す、テストで
/// 「総当たりよりノード訪問数が少ない」= 実際に枝刈りしていることの検証に使う)。
#[derive(Clone, Copy, Debug, Default)]
pub struct BvhDiagnostics {
    pub sphere_tests: usize,
}

fn build_node(mut leaves: Vec<(usize, Aabb, Vec3)>) -> BvhNode {
    if leaves.len() == 1 {
        let (index, bounds, _) = leaves[0];
        return BvhNode::Leaf {
            bounds,
            sphere_index: index,
        };
    }

    let bounds = leaves
        .iter()
        .fold(leaves[0].1, |acc, &(_, b, _)| Aabb::union(acc, b));
    let extent = bounds.max - bounds.min;
    let axis = if extent.x >= extent.y && extent.x >= extent.z {
        0
    } else if extent.y >= extent.z {
        1
    } else {
        2
    };
    leaves.sort_by(|a, b| {
        let ca = match axis {
            0 => a.2.x,
            1 => a.2.y,
            _ => a.2.z,
        };
        let cb = match axis {
            0 => b.2.x,
            1 => b.2.y,
            _ => b.2.z,
        };
        ca.partial_cmp(&cb)
            .expect("centroid coordinates are finite")
    });
    let mid = leaves.len() / 2;
    let right_leaves = leaves.split_off(mid);
    let left = build_node(leaves);
    let right = build_node(right_leaves);
    BvhNode::Internal {
        bounds,
        left: Box::new(left),
        right: Box::new(right),
    }
}

impl Bvh {
    /// `spheres`が空なら常にヒットしない`Bvh`を返す。
    pub fn build(spheres: &[Sphere]) -> Bvh {
        if spheres.is_empty() {
            return Bvh { root: None };
        }
        let leaves: Vec<(usize, Aabb, Vec3)> = spheres
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let bounds = Aabb::of_sphere(s);
                (i, bounds, s.center)
            })
            .collect();
        Bvh {
            root: Some(build_node(leaves)),
        }
    }

    /// 最近傍ヒットを返す(ヒットした球の`spheres`内でのindexと`Hit`)。
    pub fn closest_hit(&self, spheres: &[Sphere], ray: &Ray, t_min: f64) -> Option<(usize, Hit)> {
        let mut diagnostics = BvhDiagnostics::default();
        self.closest_hit_with_diagnostics(spheres, ray, t_min, &mut diagnostics)
    }

    /// テスト計測用: 実際に`Sphere::intersect`を呼んだ回数を`diagnostics`に積む
    /// (枝刈りの実効性を検証するため、モジュールdoc参照)。
    pub fn closest_hit_with_diagnostics(
        &self,
        spheres: &[Sphere],
        ray: &Ray,
        t_min: f64,
        diagnostics: &mut BvhDiagnostics,
    ) -> Option<(usize, Hit)> {
        let root = self.root.as_ref()?;
        let mut best: Option<(usize, Hit)> = None;
        traverse(root, spheres, ray, t_min, &mut best, diagnostics);
        best
    }
}

fn traverse(
    node: &BvhNode,
    spheres: &[Sphere],
    ray: &Ray,
    t_min: f64,
    best: &mut Option<(usize, Hit)>,
    diagnostics: &mut BvhDiagnostics,
) {
    let current_t_max = best.as_ref().map(|(_, hit)| hit.t).unwrap_or(f64::INFINITY);
    if !node.bounds().intersects_ray(ray, t_min, current_t_max) {
        return;
    }
    match node {
        BvhNode::Leaf { sphere_index, .. } => {
            diagnostics.sphere_tests += 1;
            if let Some(hit) = spheres[*sphere_index].intersect(ray, t_min) {
                let better = match best {
                    Some((_, existing)) => hit.t < existing.t,
                    None => true,
                };
                if better {
                    *best = Some((*sphere_index, hit));
                }
            }
        }
        BvhNode::Internal { left, right, .. } => {
            traverse(left, spheres, ray, t_min, best, diagnostics);
            traverse(right, spheres, ray, t_min, best, diagnostics);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_math::SimRng;

    fn brute_force_closest_hit(spheres: &[Sphere], ray: &Ray, t_min: f64) -> Option<(usize, Hit)> {
        let mut best: Option<(usize, Hit)> = None;
        for (i, s) in spheres.iter().enumerate() {
            if let Some(hit) = s.intersect(ray, t_min) {
                let better = match &best {
                    Some((_, existing)) => hit.t < existing.t,
                    None => true,
                };
                if better {
                    best = Some((i, hit));
                }
            }
        }
        best
    }

    fn random_scene(rng: &mut SimRng, n: usize) -> Vec<Sphere> {
        (0..n)
            .map(|_| Sphere {
                center: Vec3::new(
                    rng.next_f64() * 40.0 - 20.0,
                    rng.next_f64() * 40.0 - 20.0,
                    rng.next_f64() * 40.0 - 20.0,
                ),
                radius: 0.5 + rng.next_f64() * 1.5,
            })
            .collect()
    }

    /// BVHの最近傍ヒットが、多数の乱数シーン・乱数レイに対して総当たりと厳密に
    /// 一致すること(設計が求める「BVH: レイ交差高速化」の正しさ、結果自体は
    /// 総当たりと同一でなければならない)を確認する。
    #[test]
    fn closest_hit_matches_brute_force_across_random_scenes_and_rays() {
        let mut rng = SimRng::new(42, 0);
        for _ in 0..20 {
            let spheres = random_scene(&mut rng, 60);
            let bvh = Bvh::build(&spheres);
            for _ in 0..50 {
                let origin = Vec3::new(
                    rng.next_f64() * 60.0 - 30.0,
                    rng.next_f64() * 60.0 - 30.0,
                    rng.next_f64() * 60.0 - 30.0,
                );
                let direction = Vec3::new(
                    rng.next_f64() * 2.0 - 1.0,
                    rng.next_f64() * 2.0 - 1.0,
                    rng.next_f64() * 2.0 - 1.0,
                );
                let ray = Ray::new(origin, direction);
                let expected = brute_force_closest_hit(&spheres, &ray, 1e-6);
                let actual = bvh.closest_hit(&spheres, &ray, 1e-6);
                match (expected, actual) {
                    (None, None) => {}
                    (Some((ei, eh)), Some((ai, ah))) => {
                        assert_eq!(ei, ai, "hit index mismatch");
                        assert!(
                            (eh.t - ah.t).abs() < 1e-9,
                            "hit distance mismatch: expected={} actual={}",
                            eh.t,
                            ah.t
                        );
                    }
                    (expected, actual) => {
                        panic!("hit/miss mismatch: expected={expected:?} actual={actual:?}")
                    }
                }
            }
        }
    }

    /// BVHが実際に枝刈りしていること(総当たり(=全球テスト)より訪問する
    /// 球の数が少ないこと)を確認する——2つの離れたクラスタのうち片方だけを
    /// 通るレイは、もう片方のクラスタの球を全くテストしないはず。
    #[test]
    fn closest_hit_prunes_the_far_cluster_and_tests_fewer_spheres_than_brute_force() {
        let mut rng = SimRng::new(7, 0);
        let near_cluster: Vec<Sphere> = (0..30)
            .map(|_| Sphere {
                center: Vec3::new(
                    rng.next_f64() * 2.0 - 1.0,
                    rng.next_f64() * 2.0 - 1.0,
                    5.0 + rng.next_f64() * 2.0,
                ),
                radius: 0.3,
            })
            .collect();
        let far_cluster: Vec<Sphere> = (0..30)
            .map(|_| Sphere {
                center: Vec3::new(
                    rng.next_f64() * 2.0 - 1.0,
                    rng.next_f64() * 2.0 - 1.0,
                    1000.0 + rng.next_f64() * 2.0,
                ),
                radius: 0.3,
            })
            .collect();
        let mut spheres = near_cluster;
        spheres.extend(far_cluster);
        let bvh = Bvh::build(&spheres);

        // +z方向へ向かうレイは近いクラスタに当たり、遠いクラスタは明らかに
        // 枝刈りされるべき(バウンディングボックスが全く重ならない)。
        let ray = Ray::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
        let mut diagnostics = BvhDiagnostics::default();
        let hit = bvh.closest_hit_with_diagnostics(&spheres, &ray, 1e-6, &mut diagnostics);
        assert!(hit.is_some(), "ray should hit the near cluster");
        assert!(
            diagnostics.sphere_tests < spheres.len(),
            "BVH should prune the far cluster instead of testing all {} spheres (tested {})",
            spheres.len(),
            diagnostics.sphere_tests
        );
    }
}
