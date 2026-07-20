//! スリープ(接触島の静止検出による積分停止)。設計: docs/10-mechanics/01-rigid-body.md §4
//! 「速度が閾値未満の接触島単位で積分を停止。起床は新規接触・力適用時」。
//!
//! 接触島(dynamic-dynamic 接触で連結された剛体群、static/kinematic 越しには連結しない —
//! 標準的な物理エンジンの慣例)単位で、島内の**全** dynamic body の速度が閾値未満の状態が
//! 既定 0.5 秒続いたら asleep とする。停止するのは力適用・速度積分・位置積分に加え、
//! **両側とも asleep な接触の再解決**(`MechanicsSolver::manifold_is_active` が判定)。
//! 実装検証中に、asleep でも contact solve だけは毎ステップ回し続けると warm start・
//! split impulse の数値的な揺らぎ(0 に凍結した速度への微小な再摂動)で島が
//! 再起床→再入眠を繰り返し、かえって収束が乱れる(最終速度が M12 の閾値 1e-3 を上回る)
//! ことを発見した — 「積分を停止」だけでは不十分で、接触解決自体も止める必要がある。

use crate::body::{BodyType, RigidBodySet};
use crate::collision::ContactManifold;
use sim_math::Vec3;
use std::collections::BTreeMap;

/// スリープ速度閾値(設計 §9)。
pub const SLEEP_LINEAR_THRESHOLD: f64 = 0.01;
pub const SLEEP_ANGULAR_THRESHOLD: f64 = 0.02;
/// スリープに入るまでの継続静止時間(設計 §4)。
pub const SLEEP_TIME_THRESHOLD: f64 = 0.5;

struct UnionFind {
    parent: Vec<usize>,
}

impl UnionFind {
    fn new(n: usize) -> UnionFind {
        UnionFind {
            parent: (0..n).collect(),
        }
    }

    fn find(&mut self, x: usize) -> usize {
        if self.parent[x] != x {
            self.parent[x] = self.find(self.parent[x]);
        }
        self.parent[x]
    }

    /// 決定論のため常に小さい方の index に統合する固定規則。
    fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            if ra < rb {
                self.parent[rb] = ra;
            } else {
                self.parent[ra] = rb;
            }
        }
    }
}

/// dynamic-dynamic 接触の連結成分(島)を求め、各 body の代表 index(root)を返す。
/// static/kinematic は島の連結には参加しない(1つの床に乗る無関係な物体群を
/// 誤って同一島にまとめないため)。
fn compute_islands(bodies: &RigidBodySet, manifolds: &[ContactManifold]) -> Vec<usize> {
    let n = bodies.len();
    let mut uf = UnionFind::new(n);
    for m in manifolds {
        if bodies.body_type[m.body_a] == BodyType::Dynamic
            && bodies.body_type[m.body_b] == BodyType::Dynamic
        {
            uf.union(m.body_a, m.body_b);
        }
    }
    (0..n).map(|i| uf.find(i)).collect()
}

/// スリープ状態を1ステップ分更新する。速度が post-solve(接触解決後)であることが前提
/// (解決前の速度は重力積分直後でまだ抗力が相殺していないため、静止判定に使えない)。
pub fn update_sleep_state(bodies: &mut RigidBodySet, manifolds: &[ContactManifold], dt: f64) {
    let n = bodies.len();
    if n == 0 {
        return;
    }
    let roots = compute_islands(bodies, manifolds);

    for i in 0..n {
        if bodies.body_type[i] != BodyType::Dynamic {
            continue;
        }
        let still = bodies.linear_velocity[i].length() < SLEEP_LINEAR_THRESHOLD
            && bodies.angular_velocity[i].length() < SLEEP_ANGULAR_THRESHOLD;
        if still {
            bodies.still_time[i] += dt;
        } else {
            bodies.still_time[i] = 0.0;
        }
    }

    // 島は「全 member が揃って閾値時間以上静止」して初めて眠る(1体でも足りなければ島全体が
    // 起きたまま)。新規接触で島が合流した直後は合流相手の still_time=0 が効いて即座に起床する。
    let mut island_min_still: BTreeMap<usize, f64> = BTreeMap::new();
    for (i, &root) in roots.iter().enumerate() {
        if bodies.body_type[i] != BodyType::Dynamic {
            continue;
        }
        let entry = island_min_still.entry(root).or_insert(f64::INFINITY);
        *entry = entry.min(bodies.still_time[i]);
    }
    for i in 0..n {
        if bodies.body_type[i] != BodyType::Dynamic {
            continue;
        }
        let now_asleep = island_min_still[&roots[i]] >= SLEEP_TIME_THRESHOLD;
        if now_asleep && !bodies.asleep[i] {
            // 眠りに入った瞬間、残留速度(閾値未満とはいえ非ゼロ)を厳密に0にする
            // (標準的な物理エンジンの慣例。以後 apply_forces/integrate は停止するため、
            // ここで凍結しないと入眠時点の残留速度がそのまま永続してしまう)。
            bodies.linear_velocity[i] = Vec3::ZERO;
            bodies.angular_velocity[i] = Vec3::ZERO;
        }
        bodies.asleep[i] = now_asleep;
    }
}
