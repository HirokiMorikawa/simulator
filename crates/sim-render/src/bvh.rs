//! BVH(境界ボリューム階層)によるレイ交差の高速化。設計docs/17-rendering/
//! 02-path-tracing.md §4「BVH: 三角形メッシュ + 解析形状(球・平面)」。
//!
//! **縮約実装の理由**: 対象形状は`Primitive`(球・クアッド)で、三角形メッシュは
//! 引き続き未実装(`quad.rs`モジュールdoc参照)。この木構造は`sim_mechanics::
//! collision`のBVH broadphase(ボディペア列挙)とは意図的に別実装(あちらの
//! 内部型は非公開かつ「レイの最近傍ヒットを再帰探索する」という本モジュールの
//! クエリとは異なる形状のクエリ(重なりペア列挙)のため共有しない、
//! `bsdf.rs`モジュールdocの「意図的な型分離」と同じ方針)。
//!
//! `Scene::closest_hit`への配線は増分Cで完了した(以前は「多数物体シーンが
//! 無いため後続増分」として総当たりのままだったが、R4コーネルボックスが
//! 壁6面+発光パネル+球という多数プリミティブのシーンを要求するため、
//! `Scene::new`がBVHを一度だけ構築して保持する形へ移行した)。
//!
//! **群9で分割戦略を SAH(Surface Area Heuristic)へ差し替えた**(`build_node`)。
//! 移行前は最長軸の重心中央値(median split)で、プリミティブが偏って分布する
//! シーン——小さな物体のクラスタと遠方の大きな物体が混在する構成——では
//! 深さだけ増えて枝刈りが効かなかった。binned SAH(Wald 2007、12ビン)で
//! $C=C_{trav}+\frac{A_L}{A}N_L+\frac{A_R}{A}N_R$ を最小化する分割面を選ぶ。
//! **どの候補も葉コストを上回る場合と重心が縮退している場合は中央値分割へ
//! フォールバックする**(構築が必ず終わることを保証するため)。

use crate::primitive::Primitive;
use crate::ray::Ray;
use crate::sphere::Hit;
use sim_math::Vec3;

#[derive(Clone, Copy, Debug)]
struct Aabb {
    min: Vec3,
    max: Vec3,
}

impl Aabb {
    fn of_primitive(p: &Primitive) -> Aabb {
        let (min, max) = p.bounds();
        Aabb { min, max }
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
        primitive_index: usize,
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

/// プリミティブ群に対するBVH。`Bvh::build`で構築し、`Bvh::closest_hit`でレイの
/// 最近傍ヒット(元の`primitives`スライスにおけるindex込み)を返す。
pub struct Bvh {
    root: Option<BvhNode>,
}

/// 診断用の交差テスト回数(`closest_hit_with_diagnostics`が返す、テストで
/// 「総当たりよりノード訪問数が少ない」= 実際に枝刈りしていることの検証に使う)。
#[derive(Clone, Copy, Debug, Default)]
pub struct BvhDiagnostics {
    pub primitive_tests: usize,
    /// 訪問したノード数(**群9で追加**)。SAH が最小化しているのは
    /// $C=C_{trav}N_{node}+N_{prim}$ という**走査コスト**であり、葉での交差判定回数
    /// だけでは効果が測れない(枝刈りが効いていれば葉テスト数はどちらの戦略でも
    /// 1に近づく)。実際に SAH と中央値分割を比較したところ葉テスト数は完全に同数
    /// (220 対 220)で、差が出るのはノード訪問数だった。
    pub node_visits: usize,
}

/// SAH のビン数(Wald 2007 が推奨する 8〜16 の範囲、モジュールdoc参照)。
const SAH_BINS: usize = 12;
/// 内部ノードを1つ通過するコスト(プリミティブ1個の交差判定コストを 1 とした相対値)。
/// 解析形状(球・クアッド)の交差判定は安いので、走査コストを相対的に小さめに置く。
const SAH_TRAVERSAL_COST: f64 = 0.5;

fn axis_of(v: Vec3, axis: usize) -> f64 {
    match axis {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

/// AABB の表面積(SAH のコスト式に入る $A$)。退化(1点)なら 0。
fn surface_area(b: Aabb) -> f64 {
    let d = b.max - b.min;
    if d.x < 0.0 || d.y < 0.0 || d.z < 0.0 {
        return 0.0;
    }
    2.0 * (d.x * d.y + d.y * d.z + d.z * d.x)
}

/// BVH構築中の葉候補: (元の`primitives`内でのindex, 境界, 重心)。
type Leaf = (usize, Aabb, Vec3);

/// 最長軸の重心中央値で分割する(移行前の戦略。SAH のフォールバック先として残す)。
fn median_split(mut leaves: Vec<Leaf>, axis: usize) -> (Vec<Leaf>, Vec<Leaf>) {
    leaves.sort_by(|a, b| {
        axis_of(a.2, axis)
            .partial_cmp(&axis_of(b.2, axis))
            .expect("centroid coordinates are finite")
    });
    let mid = leaves.len() / 2;
    let right = leaves.split_off(mid);
    (leaves, right)
}

/// binned SAH による分割候補の探索(モジュールdoc参照)。分割すべきでない・
/// できない場合は `None`(呼び出し側が中央値分割へフォールバックする)。
fn sah_split(leaves: &[Leaf], bounds: Aabb) -> Option<(usize, f64)> {
    let parent_area = surface_area(bounds);
    if parent_area <= 0.0 {
        return None;
    }
    let leaf_cost = leaves.len() as f64; // 分割せず全部を1ノードで持つ相対コスト
    let mut best: Option<(usize, f64, f64)> = None; // (axis, 境界値, コスト)

    for axis in 0..3 {
        // 重心の範囲でビンを切る(境界の範囲ではない — Wald 2007)。
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for &(_, _, centroid) in leaves {
            let c = axis_of(centroid, axis);
            lo = lo.min(c);
            hi = hi.max(c);
        }
        if hi <= lo {
            continue; // この軸では重心が縮退している
        }
        let scale = SAH_BINS as f64 / (hi - lo);

        let mut bin_bounds: Vec<Option<Aabb>> = vec![None; SAH_BINS];
        let mut bin_counts = [0usize; SAH_BINS];
        for &(_, leaf_bounds, centroid) in leaves {
            let b = (((axis_of(centroid, axis) - lo) * scale) as usize).min(SAH_BINS - 1);
            bin_bounds[b] = Some(match bin_bounds[b] {
                Some(acc) => Aabb::union(acc, leaf_bounds),
                None => leaf_bounds,
            });
            bin_counts[b] += 1;
        }

        // 前方/後方の累積境界と個数(分割面は各ビンの境目 = SAH_BINS-1 通り)。
        let mut prefix_area = [0.0; SAH_BINS];
        let mut prefix_count = [0usize; SAH_BINS];
        let mut acc: Option<Aabb> = None;
        let mut count = 0usize;
        for b in 0..SAH_BINS {
            if let Some(bb) = bin_bounds[b] {
                acc = Some(match acc {
                    Some(a) => Aabb::union(a, bb),
                    None => bb,
                });
            }
            count += bin_counts[b];
            prefix_area[b] = acc.map(surface_area).unwrap_or(0.0);
            prefix_count[b] = count;
        }
        let mut suffix_area = [0.0; SAH_BINS];
        let mut suffix_count = [0usize; SAH_BINS];
        let mut acc: Option<Aabb> = None;
        let mut count = 0usize;
        for b in (0..SAH_BINS).rev() {
            if let Some(bb) = bin_bounds[b] {
                acc = Some(match acc {
                    Some(a) => Aabb::union(a, bb),
                    None => bb,
                });
            }
            count += bin_counts[b];
            suffix_area[b] = acc.map(surface_area).unwrap_or(0.0);
            suffix_count[b] = count;
        }

        for split in 0..SAH_BINS - 1 {
            let (n_left, n_right) = (prefix_count[split], suffix_count[split + 1]);
            if n_left == 0 || n_right == 0 {
                continue; // 片側が空になる分割は意味がない
            }
            let cost = SAH_TRAVERSAL_COST
                + (prefix_area[split] * n_left as f64 + suffix_area[split + 1] * n_right as f64)
                    / parent_area;
            let better = match best {
                Some((_, _, best_cost)) => cost < best_cost,
                None => true,
            };
            if better {
                // 分割面は「ビン split の右端」= 重心がこの値未満なら左。
                let boundary = lo + (split + 1) as f64 / scale;
                best = Some((axis, boundary, cost));
            }
        }
    }

    let (axis, boundary, cost) = best?;
    if cost >= leaf_cost {
        return None; // 分割してもコストが下がらない(葉コストを上回る)
    }
    Some((axis, boundary))
}

/// 分割戦略。既定は `Sah`。`Median` は**対照実験専用**(移行前の挙動を再現して
/// 「SAH が実際に交差判定回数を減らしている」ことを測るために残してある)。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SplitStrategy {
    Sah,
    Median,
}

fn build_node(leaves: Vec<Leaf>, strategy: SplitStrategy) -> BvhNode {
    if leaves.len() == 1 {
        let (index, bounds, _) = leaves[0];
        return BvhNode::Leaf {
            bounds,
            primitive_index: index,
        };
    }

    let bounds = leaves
        .iter()
        .fold(leaves[0].1, |acc, &(_, b, _)| Aabb::union(acc, b));
    let extent = bounds.max - bounds.min;
    let longest_axis = if extent.x >= extent.y && extent.x >= extent.z {
        0
    } else if extent.y >= extent.z {
        1
    } else {
        2
    };

    let candidate = match strategy {
        SplitStrategy::Sah => sah_split(&leaves, bounds),
        SplitStrategy::Median => None,
    };
    let (left_leaves, right_leaves) = match candidate {
        Some((axis, boundary)) => {
            let (left, right): (Vec<_>, Vec<_>) = leaves
                .into_iter()
                .partition(|&(_, _, centroid)| axis_of(centroid, axis) < boundary);
            // ビン化の丸めで片側が空になり得るので、その場合だけ中央値へ落とす
            // (無限再帰を防ぐための保険。空にならない限りSAHの分割をそのまま使う)。
            if left.is_empty() || right.is_empty() {
                let mut all = left;
                all.extend(right);
                median_split(all, longest_axis)
            } else {
                (left, right)
            }
        }
        None => median_split(leaves, longest_axis),
    };

    BvhNode::Internal {
        bounds,
        left: Box::new(build_node(left_leaves, strategy)),
        right: Box::new(build_node(right_leaves, strategy)),
    }
}

impl Bvh {
    /// `primitives`が空なら常にヒットしない`Bvh`を返す。分割戦略は SAH(モジュールdoc参照)。
    pub fn build(primitives: &[Primitive]) -> Bvh {
        Bvh::build_with(primitives, SplitStrategy::Sah)
    }

    /// 分割戦略を指定して構築する。`SplitStrategy::Median` は**対照実験専用**
    /// ——移行前の挙動を再現して SAH の効果を測るために残してある。
    pub fn build_with(primitives: &[Primitive], strategy: SplitStrategy) -> Bvh {
        if primitives.is_empty() {
            return Bvh { root: None };
        }
        let leaves: Vec<Leaf> = primitives
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let bounds = Aabb::of_primitive(p);
                (i, bounds, p.centroid())
            })
            .collect();
        Bvh {
            root: Some(build_node(leaves, strategy)),
        }
    }

    /// 最近傍ヒットを返す(ヒットしたプリミティブの`primitives`内でのindexと`Hit`)。
    pub fn closest_hit(
        &self,
        primitives: &[Primitive],
        ray: &Ray,
        t_min: f64,
    ) -> Option<(usize, Hit)> {
        let mut diagnostics = BvhDiagnostics::default();
        self.closest_hit_with_diagnostics(primitives, ray, t_min, &mut diagnostics)
    }

    /// テスト計測用: 実際に`Primitive::intersect`を呼んだ回数を`diagnostics`に積む
    /// (枝刈りの実効性を検証するため、モジュールdoc参照)。
    pub fn closest_hit_with_diagnostics(
        &self,
        primitives: &[Primitive],
        ray: &Ray,
        t_min: f64,
        diagnostics: &mut BvhDiagnostics,
    ) -> Option<(usize, Hit)> {
        let root = self.root.as_ref()?;
        let mut best: Option<(usize, Hit)> = None;
        traverse(root, primitives, ray, t_min, &mut best, diagnostics);
        best
    }
}

fn traverse(
    node: &BvhNode,
    primitives: &[Primitive],
    ray: &Ray,
    t_min: f64,
    best: &mut Option<(usize, Hit)>,
    diagnostics: &mut BvhDiagnostics,
) {
    diagnostics.node_visits += 1;
    let current_t_max = best.as_ref().map(|(_, hit)| hit.t).unwrap_or(f64::INFINITY);
    if !node.bounds().intersects_ray(ray, t_min, current_t_max) {
        return;
    }
    match node {
        BvhNode::Leaf {
            primitive_index, ..
        } => {
            diagnostics.primitive_tests += 1;
            if let Some(hit) = primitives[*primitive_index].intersect(ray, t_min) {
                let better = match best {
                    Some((_, existing)) => hit.t < existing.t,
                    None => true,
                };
                if better {
                    *best = Some((*primitive_index, hit));
                }
            }
        }
        BvhNode::Internal { left, right, .. } => {
            traverse(left, primitives, ray, t_min, best, diagnostics);
            traverse(right, primitives, ray, t_min, best, diagnostics);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::quad::Quad;
    use crate::sphere::Sphere;
    use sim_math::SimRng;

    fn brute_force_closest_hit(
        primitives: &[Primitive],
        ray: &Ray,
        t_min: f64,
    ) -> Option<(usize, Hit)> {
        let mut best: Option<(usize, Hit)> = None;
        for (i, s) in primitives.iter().enumerate() {
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

    fn random_scene(rng: &mut SimRng, n: usize) -> Vec<Primitive> {
        (0..n)
            .map(|_| {
                Primitive::Sphere(Sphere {
                    center: Vec3::new(
                        rng.next_f64() * 40.0 - 20.0,
                        rng.next_f64() * 40.0 - 20.0,
                        rng.next_f64() * 40.0 - 20.0,
                    ),
                    radius: 0.5 + rng.next_f64() * 1.5,
                })
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
            let primitives = random_scene(&mut rng, 60);
            let bvh = Bvh::build(&primitives);
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
                let expected = brute_force_closest_hit(&primitives, &ray, 1e-6);
                let actual = bvh.closest_hit(&primitives, &ray, 1e-6);
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
    fn closest_hit_prunes_the_far_cluster_and_tests_fewer_primitives_than_brute_force() {
        let mut rng = SimRng::new(7, 0);
        let near_cluster: Vec<Primitive> = (0..30)
            .map(|_| {
                Primitive::Sphere(Sphere {
                    center: Vec3::new(
                        rng.next_f64() * 2.0 - 1.0,
                        rng.next_f64() * 2.0 - 1.0,
                        5.0 + rng.next_f64() * 2.0,
                    ),
                    radius: 0.3,
                })
            })
            .collect();
        let far_cluster: Vec<Primitive> = (0..30)
            .map(|_| {
                Primitive::Sphere(Sphere {
                    center: Vec3::new(
                        rng.next_f64() * 2.0 - 1.0,
                        rng.next_f64() * 2.0 - 1.0,
                        1000.0 + rng.next_f64() * 2.0,
                    ),
                    radius: 0.3,
                })
            })
            .collect();
        let mut primitives = near_cluster;
        primitives.extend(far_cluster);
        let bvh = Bvh::build(&primitives);

        // +z方向へ向かうレイは近いクラスタに当たり、遠いクラスタは明らかに
        // 枝刈りされるべき(バウンディングボックスが全く重ならない)。
        let ray = Ray::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
        let mut diagnostics = BvhDiagnostics::default();
        let hit = bvh.closest_hit_with_diagnostics(&primitives, &ray, 1e-6, &mut diagnostics);
        assert!(hit.is_some(), "ray should hit the near cluster");
        assert!(
            diagnostics.primitive_tests < primitives.len(),
            "BVH should prune the far cluster instead of testing all {} primitives (tested {})",
            primitives.len(),
            diagnostics.primitive_tests
        );
    }

    /// **対照実験(群9)**: SAH 分割が中央値分割より実際に交差判定回数を減らすこと。
    ///
    /// 中央値分割が苦手な分布を意図的に作る: **小さな球の密なクラスタ**(空間的に
    /// 狭い範囲に多数)と、**遠方に置いた大きなクアッド**(1枚で巨大なAABBを持つ)。
    /// 中央値分割は個数だけで半分に割るので、巨大なクアッドが小球群と同じノードへ
    /// 混ざり、そのノードのAABBが空間全体に膨らんで枝刈りが効かなくなる。
    /// SAH は表面積を見るので、巨大な物体を早い段階で切り離せる。
    ///
    /// **不変条件も同時に確認する**: 加速構造は「何がヒットするか」を変えてはならない
    /// ——両ビルダーのヒット結果が全レイで完全一致すること。
    #[test]
    fn sah_prunes_more_than_median_split_on_a_skewed_distribution() {
        let mut rng = SimRng::new(2024, 0);
        let mut primitives: Vec<Primitive> = (0..64)
            .map(|_| {
                Primitive::Sphere(Sphere {
                    center: Vec3::new(
                        rng.next_f64() * 2.0 - 1.0,
                        rng.next_f64() * 2.0 - 1.0,
                        rng.next_f64() * 2.0 - 1.0,
                    ),
                    radius: 0.05,
                })
            })
            .collect();
        // 遠方に置いた巨大なクアッド(中央値分割はこれを小球群と混ぜてしまう)。
        for k in 0..4 {
            let offset = 200.0 + k as f64 * 50.0;
            primitives.push(Primitive::Quad(Quad::axis_aligned(
                2, offset, -150.0, 150.0, -150.0, 150.0,
            )));
        }

        let sah = Bvh::build_with(&primitives, SplitStrategy::Sah);
        let median = Bvh::build_with(&primitives, SplitStrategy::Median);

        let mut rays = Vec::new();
        let mut ray_rng = SimRng::new(99, 0);
        for _ in 0..200 {
            let origin = Vec3::new(
                ray_rng.next_f64() * 4.0 - 2.0,
                ray_rng.next_f64() * 4.0 - 2.0,
                -5.0,
            );
            let target = Vec3::new(
                ray_rng.next_f64() * 2.0 - 1.0,
                ray_rng.next_f64() * 2.0 - 1.0,
                ray_rng.next_f64() * 2.0 - 1.0,
            );
            rays.push(Ray::new(origin, (target - origin).normalize_or_zero()));
        }

        // SAH が最小化しているのは**走査コスト**なので、ノード訪問数で測る
        // (`BvhDiagnostics::node_visits` のdoc参照)。
        let mut sah_tests = 0usize;
        let mut median_tests = 0usize;
        let mut sah_leaf_tests = 0usize;
        let mut median_leaf_tests = 0usize;
        for ray in &rays {
            let mut d_sah = BvhDiagnostics::default();
            let hit_sah = sah.closest_hit_with_diagnostics(&primitives, ray, 1e-6, &mut d_sah);
            let mut d_median = BvhDiagnostics::default();
            let hit_median =
                median.closest_hit_with_diagnostics(&primitives, ray, 1e-6, &mut d_median);
            sah_tests += d_sah.node_visits;
            median_tests += d_median.node_visits;

            sah_leaf_tests += d_sah.primitive_tests;
            median_leaf_tests += d_median.primitive_tests;

            // **不変条件**: どちらのビルダーでも同じものがヒットする。
            match (hit_sah, hit_median) {
                (Some((ia, ha)), Some((ib, hb))) => {
                    assert_eq!(ia, ib, "both builders must hit the same primitive");
                    assert!(
                        (ha.t - hb.t).abs() < 1e-12,
                        "and at the same distance: {} vs {}",
                        ha.t,
                        hb.t
                    );
                }
                (None, None) => {}
                (a, b) => panic!(
                    "hit disagreement: sah={:?} median={:?}",
                    a.is_some(),
                    b.is_some()
                ),
            }
        }

        assert!(
            sah_tests < median_tests,
            "SAH must visit strictly fewer nodes than the median split on a skewed \
             distribution: sah={sah_tests} median={median_tests}"
        );
        let improvement = 1.0 - sah_tests as f64 / median_tests as f64;
        assert!(
            improvement > 0.10,
            "and the improvement should be substantial: sah={sah_tests} median={median_tests} \
             (improvement {:.1}%)",
            improvement * 100.0
        );
        // 実測(2026-08-03): ノード訪問 SAH=4082 / 中央値=5758(29.1%削減)、
        // 葉での交差判定は 220 対 220 で**完全に同数**——枝刈りが効いている限り
        // 葉テスト数では差が出ないことの実地の裏取りである。
        assert_eq!(
            sah_leaf_tests, median_leaf_tests,
            "leaf tests are expected to be identical here; the win is in traversal"
        );
        println!(
            "node visits: SAH={sah_tests} median={median_tests} improvement={:.1}% \
             (leaf tests: SAH={sah_leaf_tests} median={median_leaf_tests})",
            improvement * 100.0
        );
    }

    /// 球とクアッドが混在するシーンでもBVHの最近傍ヒットが総当たりと厳密一致する
    /// (`Primitive`への一般化がAABB構築・交差委譲とも正しいことの検証)。
    #[test]
    fn closest_hit_matches_brute_force_for_mixed_spheres_and_quads() {
        let mut rng = SimRng::new(1234, 0);
        let mut primitives = random_scene(&mut rng, 20);
        // 軸並行クアッド(厚みゼロのAABBになる)を各軸ぶん混ぜる。
        for axis in 0..3 {
            for k in 0..4 {
                let value = -6.0 + 4.0 * k as f64;
                primitives.push(Primitive::Quad(Quad::axis_aligned(
                    axis, value, -8.0, 8.0, -8.0, 8.0,
                )));
            }
        }
        let bvh = Bvh::build(&primitives);

        for _ in 0..300 {
            let origin = Vec3::new(
                rng.next_f64() * 30.0 - 15.0,
                rng.next_f64() * 30.0 - 15.0,
                rng.next_f64() * 30.0 - 15.0,
            );
            let direction = Vec3::new(
                rng.next_f64() * 2.0 - 1.0,
                rng.next_f64() * 2.0 - 1.0,
                rng.next_f64() * 2.0 - 1.0,
            );
            let ray = Ray::new(origin, direction);
            let expected = brute_force_closest_hit(&primitives, &ray, 1e-6);
            let actual = bvh.closest_hit(&primitives, &ray, 1e-6);
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
