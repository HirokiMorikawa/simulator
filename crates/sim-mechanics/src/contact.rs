//! Sequential impulses 接触ソルバ。設計: docs/10-mechanics/03-contact-solver.md、
//! docs/10-mechanics/04-friction.md(接線 solve)。
//!
//! 法線 + 反発 + split impulse 位置補正 + 箱近似クーロン摩擦 + warm starting
//! (設計 §4.1/§4.4/§4.5)。§4.4 warm starting は本来 Phase 1 スコープ — 4段スタック(M12)の
//! 収束にはこれが鍵、docs/22-roadmap/01-phases.md 横断ルール5に基づき実装漏れを訂正。
//! velocity_bias の Baumgarte 項は split impulse(§4.5)の位置チャンネルに置き換えたため
//! 廃止(設計 §4.5 の指摘どおり「エネルギーを汚さないため反発テストが厳密になる」)。
//!
//! **群9でマニフォールド持続化(設計 02-collision-detection.md §4.7)を実装した**。
//! 移行前は feature_id が一致しさえすれば無条件に前ステップのインパルスを引き継いでおり、
//! 接触点が実際には別の場所へ移った場合でも古いインパルスがそのまま適用されていた
//! (加えて接触が消えたキーの GC が無かった)。`ManifoldCache` を参照。

use crate::body::RigidBodySet;
use crate::collision::ContactManifold;
use crate::shape::Shape;
use sim_core::MaterialDb;
use sim_math::{Mat3, Transform, Vec3};
use std::collections::{BTreeMap, BTreeSet};

/// 接触点を再利用してよい「移動量」の上限 [m]。設計 02-collision-detection.md §4.7
/// 「点の再利用判定: 同一 feature_id かつ移動 < 2mm」。
///
/// **「移動」を何に対して測るか**: ワールド座標での移動量ではなく、**2体のアンカーが
/// 互いにどれだけずれたか**(相対すべり量)で測る。ワールド基準にすると、転がる球・
/// 動く床の上の箱など「接触自体は継続しているが接触点がワールド内を動く」ケースで
/// warm starting が毎ステップ無効化されてしまい、設計 §4.4 が warm starting に期待する
/// 収束改善が丸ごと失われる(実装検討時にこの読み違いに気づいた)。キャッシュ時に
/// 一致していた2つのローカルアンカーを現在の姿勢でワールドへ戻し、その距離を見る
/// ——これが「同一の物理接触点であり続けているか」の判定になる。
pub const PERSISTENCE_TOLERANCE: f64 = 0.002;

#[derive(Clone, Copy, Default)]
pub struct WarmStartImpulse {
    normal: f64,
    tangent: (f64, f64),
}

/// キャッシュされた接触点。インパルスに加えて**両ボディのローカル座標での接触点位置**を
/// 持つ(`PERSISTENCE_TOLERANCE` のdoc参照)。
#[derive(Clone, Copy)]
struct CachedPoint {
    local_a: Vec3,
    local_b: Vec3,
    impulse: WarmStartImpulse,
}

/// マニフォールド持続化キャッシュ(設計 §4.7)。キーは (body_a, body_b, feature_id)。
/// 設計 §4.4「前ステップの累積インパルス(feature_idで対応づけ)をソルバ開始時に適用」の
/// 土台でもある。
#[derive(Clone)]
pub struct ManifoldCache {
    points: BTreeMap<(usize, usize, u32), CachedPoint>,
    /// 持続化の再利用判定を行うか。`false` にすると移行前の挙動(feature_id 一致だけで
    /// 無条件に引き継ぎ・GCなし)に戻る——対照実験専用のスイッチで、既定は `true`。
    pub persistence_enabled: bool,
}

impl Default for ManifoldCache {
    fn default() -> ManifoldCache {
        ManifoldCache::new()
    }
}

impl ManifoldCache {
    pub fn new() -> ManifoldCache {
        ManifoldCache {
            points: BTreeMap::new(),
            persistence_enabled: true,
        }
    }

    /// キャッシュしている接触点の総数(GCが効いていることの検証に使う。
    /// 空判定の用途が無いため `is_empty` は置かない)。
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// 指定の接触点に引き継ぐ warm start インパルス。持続化が有効なとき、
    /// アンカーのずれが `PERSISTENCE_TOLERANCE` 以上なら**引き継がない**(同 doc 参照)。
    fn warm_start_for(
        &self,
        key: (usize, usize, u32),
        xf_a: Transform,
        xf_b: Transform,
    ) -> WarmStartImpulse {
        let Some(cached) = self.points.get(&key) else {
            return WarmStartImpulse::default();
        };
        if !self.persistence_enabled {
            return cached.impulse; // 移行前の挙動(対照実験用)
        }
        let drift = xf_a.apply_point(cached.local_a) - xf_b.apply_point(cached.local_b);
        if drift.length() < PERSISTENCE_TOLERANCE {
            cached.impulse
        } else {
            WarmStartImpulse::default()
        }
    }

    /// 接触が完全に消えたボディ対のエントリを捨てる(GC)。`live_pairs` には
    /// **スリープ中でソルバをスキップしたペアも含めた**全マニフォールドのペアを渡す
    /// (スリープ中のペアを捨ててしまうと、起床時に warm start が0から立ち上がって
    /// 再収束の跳ねが出るため)。持続化が無効なときは移行前どおり何もしない。
    pub fn retain_pairs(&mut self, live_pairs: &BTreeSet<(usize, usize)>) {
        if !self.persistence_enabled {
            return;
        }
        self.points
            .retain(|&(a, b, _), _| live_pairs.contains(&(a, b)));
    }
}

/// 反発を無視する接近速度の閾値(静止接触のジッタ防止)。設計 §4.3・§9 の既定値。
/// `resolve` の引数として渡す(検証シナリオでジッタ防止ヒューリスティクスを外して
/// 純粋な弾性衝突を検証できるようにするため定数ではなくパラメータ化)。
pub const DEFAULT_RESTITUTION_VELOCITY_THRESHOLD: f64 = 0.5;
/// 位置補正の押し戻し係数。設計 §9(§4.3 の Baumgarte と同じ既定値を §4.5 の split impulse
/// 位置チャンネルにも流用する — 設計は両者に別値を指定していない)。
const BAUMGARTE_BETA_POS: f64 = 0.2;
/// 接触を保つ許容貫入。設計 §9。
const SLOP: f64 = 0.005;
/// velocity iterations 既定回数。設計 §9。
pub const VELOCITY_ITERATIONS: u32 = 10;
/// position iterations 既定回数。設計 §9(Box2D 準拠)。
pub const POSITION_ITERATIONS: u32 = 4;
/// 転がり摩擦係数の既定値(設計 04-friction.md §9「硬い面の代表値」)。材料ペア表は
/// 持たず単一既定値のみ(設計のパラメータ表自体が単一値であり、滑り摩擦のような
/// ペア表は要求していない)。
const DEFAULT_ROLLING_FRICTION: f64 = 0.005;

struct PointConstraint {
    r_a: Vec3,
    r_b: Vec3,
    /// 接触点の body_a / body_b ローカル座標(マニフォールド持続化、設計 §4.7)。
    /// `prepare` の時点(=位置補正より前)の姿勢で求めて保持し、そのままキャッシュへ
    /// 書き戻す(位置補正後の姿勢で取り直すと、split impulse が動かした分だけ
    /// アンカーがずれて次ステップの再利用判定が誤る)。
    local_a: Vec3,
    local_b: Vec3,
    feature_id: u32,
    normal_mass: f64,
    tangent_mass: (f64, f64),
    rolling_mass: (f64, f64),
    velocity_bias: f64,
    /// split impulse(§4.5)の位置補正チャンネル専用。速度には影響しない。
    penetration: f64,
    normal_impulse: f64,
    tangent_impulse: (f64, f64),
    rolling_impulse: (f64, f64),
}

struct Constraint {
    body_a: usize,
    body_b: usize,
    normal: Vec3,
    tangent: (Vec3, Vec3),
    friction: f64,
    /// 転がり摩擦のトルク上限 μ_roll・N・r の r(設計 04-friction.md §4.1)。
    /// Sphere 形状の半径から求める(接触する2体のうち球のものを採用、両方球なら大きい方)。
    /// 球でない接触(箱同士等)は 0 になり転がり摩擦は自動的に無効化される。
    rolling_radius: f64,
    points: Vec<PointConstraint>,
}

fn effective_mass(
    inv_mass_a: f64,
    inv_ia: Mat3,
    r_a: Vec3,
    inv_mass_b: f64,
    inv_ib: Mat3,
    r_b: Vec3,
    dir: Vec3,
) -> f64 {
    let term_a = dir.dot(inv_ia.mul_vec(r_a.cross(dir)).cross(r_a));
    let term_b = dir.dot(inv_ib.mul_vec(r_b.cross(dir)).cross(r_b));
    let k = inv_mass_a + inv_mass_b + term_a + term_b;
    if k > 0.0 {
        1.0 / k
    } else {
        0.0
    }
}

/// 転がり摩擦は純粋な偶力(トルクのみ、線形速度に寄与しない)なので、有効質量も
/// 角速度項のみで決まる(設計 04-friction.md §4.1)。
fn angular_effective_mass(inv_ia: Mat3, inv_ib: Mat3, dir: Vec3) -> f64 {
    let k = dir.dot(inv_ia.mul_vec(dir)) + dir.dot(inv_ib.mul_vec(dir));
    if k > 0.0 {
        1.0 / k
    } else {
        0.0
    }
}

fn sphere_radius(shape: &Shape) -> f64 {
    match shape {
        Shape::Sphere { radius } => *radius,
        _ => 0.0,
    }
}

fn point_velocity(v: Vec3, omega: Vec3, r: Vec3) -> Vec3 {
    v + omega.cross(r)
}

/// 設計 §4.1「prepare: 各接触点の m_eff・接線基底・velocity_bias を計算」。
/// `warm_start_cache` から前ステップの累積インパルスを feature_id で引き継ぐ(§4.4)。
/// dt に依存する項(旧 Baumgarte 速度バイアス)は split impulse 化で不要になった。
fn prepare(
    manifolds: &[ContactManifold],
    bodies: &RigidBodySet,
    materials: &MaterialDb,
    restitution_velocity_threshold: f64,
    warm_start_cache: &ManifoldCache,
) -> Vec<Constraint> {
    manifolds
        .iter()
        .map(|m| {
            let a = m.body_a;
            let b = m.body_b;
            let xf_a = Transform {
                position: bodies.position[a],
                rotation: bodies.rotation[a],
            };
            let xf_b = Transform {
                position: bodies.position[b],
                rotation: bodies.rotation[b],
            };
            let inv_xf_a = xf_a.inverse();
            let inv_xf_b = xf_b.inverse();
            let (t1, t2) = m.normal.orthonormal_basis();
            let friction = materials.friction_pair(bodies.material[a], bodies.material[b]);
            let restitution = materials.restitution_pair(bodies.material[a], bodies.material[b]);
            let rolling_radius =
                sphere_radius(bodies.shape_of(a)).max(sphere_radius(bodies.shape_of(b)));

            let points = m
                .points
                .iter()
                .map(|p| {
                    let r_a = p.world_point - bodies.position[a];
                    let r_b = p.world_point - bodies.position[b];
                    let normal_mass = effective_mass(
                        bodies.inv_mass[a],
                        bodies.inv_inertia_world[a],
                        r_a,
                        bodies.inv_mass[b],
                        bodies.inv_inertia_world[b],
                        r_b,
                        m.normal,
                    );
                    let tangent_mass = (
                        effective_mass(
                            bodies.inv_mass[a],
                            bodies.inv_inertia_world[a],
                            r_a,
                            bodies.inv_mass[b],
                            bodies.inv_inertia_world[b],
                            r_b,
                            t1,
                        ),
                        effective_mass(
                            bodies.inv_mass[a],
                            bodies.inv_inertia_world[a],
                            r_a,
                            bodies.inv_mass[b],
                            bodies.inv_inertia_world[b],
                            r_b,
                            t2,
                        ),
                    );
                    let rolling_mass = (
                        angular_effective_mass(
                            bodies.inv_inertia_world[a],
                            bodies.inv_inertia_world[b],
                            t1,
                        ),
                        angular_effective_mass(
                            bodies.inv_inertia_world[a],
                            bodies.inv_inertia_world[b],
                            t2,
                        ),
                    );

                    let v_a =
                        point_velocity(bodies.linear_velocity[a], bodies.angular_velocity[a], r_a);
                    let v_b =
                        point_velocity(bodies.linear_velocity[b], bodies.angular_velocity[b], r_b);
                    let v_n_pre = m.normal.dot(v_b - v_a);

                    // 設計 §4.3(符号は実装時に訂正、docs/10-mechanics/03-contact-solver.md 参照)。
                    // Baumgarte 項は含めない(§4.5 split impulse の位置チャンネルに分離)。
                    let restitution_bias =
                        restitution * (-v_n_pre - restitution_velocity_threshold).max(0.0);

                    let warm = warm_start_cache.warm_start_for((a, b, p.feature_id), xf_a, xf_b);

                    PointConstraint {
                        r_a,
                        r_b,
                        local_a: inv_xf_a.apply_point(p.world_point),
                        local_b: inv_xf_b.apply_point(p.world_point),
                        feature_id: p.feature_id,
                        normal_mass,
                        tangent_mass,
                        rolling_mass,
                        velocity_bias: restitution_bias,
                        penetration: p.penetration,
                        normal_impulse: warm.normal,
                        tangent_impulse: warm.tangent,
                        rolling_impulse: (0.0, 0.0),
                    }
                })
                .collect();

            Constraint {
                body_a: a,
                body_b: b,
                normal: m.normal,
                tangent: (t1, t2),
                friction,
                rolling_radius,
                points,
            }
        })
        .collect()
}

fn apply_impulse(bodies: &mut RigidBodySet, body: usize, impulse: Vec3, r: Vec3, sign: f64) {
    let inv_mass = bodies.inv_mass[body];
    let inv_i = bodies.inv_inertia_world[body];
    bodies.linear_velocity[body] =
        bodies.linear_velocity[body].addcarry_scaled(impulse, sign * inv_mass);
    let angular_impulse = r.cross(impulse);
    bodies.angular_velocity[body] =
        bodies.angular_velocity[body] + inv_i.mul_vec(angular_impulse).scale(sign);
}

/// 設計 §4.2「solve_normal」。
fn solve_normal(c: &mut Constraint, bodies: &mut RigidBodySet) {
    for p in &mut c.points {
        let v_a = point_velocity(
            bodies.linear_velocity[c.body_a],
            bodies.angular_velocity[c.body_a],
            p.r_a,
        );
        let v_b = point_velocity(
            bodies.linear_velocity[c.body_b],
            bodies.angular_velocity[c.body_b],
            p.r_b,
        );
        let v_n = c.normal.dot(v_b - v_a);

        let delta = -(v_n - p.velocity_bias) * p.normal_mass;
        let old = p.normal_impulse;
        p.normal_impulse = (old + delta).max(0.0);
        let applied = p.normal_impulse - old;

        let impulse = c.normal.scale(applied);
        apply_impulse(bodies, c.body_a, impulse, p.r_a, -1.0);
        apply_impulse(bodies, c.body_b, impulse, p.r_b, 1.0);
    }
}

/// 設計 04-friction.md §4「solve_tangent」(箱近似、2 独立制約)。
fn solve_tangent(c: &mut Constraint, bodies: &mut RigidBodySet) {
    for p in &mut c.points {
        for (k, tangent) in [c.tangent.0, c.tangent.1].into_iter().enumerate() {
            let v_a = point_velocity(
                bodies.linear_velocity[c.body_a],
                bodies.angular_velocity[c.body_a],
                p.r_a,
            );
            let v_b = point_velocity(
                bodies.linear_velocity[c.body_b],
                bodies.angular_velocity[c.body_b],
                p.r_b,
            );
            let v_t = tangent.dot(v_b - v_a);

            let mass = if k == 0 {
                p.tangent_mass.0
            } else {
                p.tangent_mass.1
            };
            let delta = -v_t * mass;
            let old = if k == 0 {
                p.tangent_impulse.0
            } else {
                p.tangent_impulse.1
            };
            let limit = c.friction * p.normal_impulse;
            let new_impulse = (old + delta).clamp(-limit, limit);
            if k == 0 {
                p.tangent_impulse.0 = new_impulse;
            } else {
                p.tangent_impulse.1 = new_impulse;
            }
            let applied = new_impulse - old;

            let impulse = tangent.scale(applied);
            apply_impulse(bodies, c.body_a, impulse, p.r_a, -1.0);
            apply_impulse(bodies, c.body_b, impulse, p.r_b, 1.0);
        }
    }
}

fn apply_angular_impulse(bodies: &mut RigidBodySet, body: usize, angular_impulse: Vec3, sign: f64) {
    let inv_i = bodies.inv_inertia_world[body];
    bodies.angular_velocity[body] =
        bodies.angular_velocity[body] + inv_i.mul_vec(angular_impulse).scale(sign);
}

/// 設計 04-friction.md §4.1「転がる球・円柱の減速…トルク制約 |τ_roll|≤μ_roll・N・r を
/// 同じクランプ構造で実装」。純粋な偶力(等大反対のトルクのみ、線形速度は変えない)なので
/// `solve_tangent` と異なり `apply_impulse` の r×impulse 経由ではなく角速度を直接更新する。
/// `rolling_radius` が 0(非球形接触)なら limit が常に 0 になり事実上無効化される。
fn solve_rolling(c: &mut Constraint, bodies: &mut RigidBodySet) {
    if c.rolling_radius <= 0.0 {
        return;
    }
    for p in &mut c.points {
        for (k, tangent) in [c.tangent.0, c.tangent.1].into_iter().enumerate() {
            let w_t =
                tangent.dot(bodies.angular_velocity[c.body_b] - bodies.angular_velocity[c.body_a]);

            let mass = if k == 0 {
                p.rolling_mass.0
            } else {
                p.rolling_mass.1
            };
            let delta = -w_t * mass;
            let old = if k == 0 {
                p.rolling_impulse.0
            } else {
                p.rolling_impulse.1
            };
            let limit = DEFAULT_ROLLING_FRICTION * p.normal_impulse * c.rolling_radius;
            let new_impulse = (old + delta).clamp(-limit, limit);
            if k == 0 {
                p.rolling_impulse.0 = new_impulse;
            } else {
                p.rolling_impulse.1 = new_impulse;
            }
            let applied = new_impulse - old;

            let angular_impulse = tangent.scale(applied);
            apply_angular_impulse(bodies, c.body_a, angular_impulse, -1.0);
            apply_angular_impulse(bodies, c.body_b, angular_impulse, 1.0);
        }
    }
}

/// 設計 §4.1「warm start: 前ステップの累積インパルスをそのまま適用」。
fn apply_warm_start(constraints: &[Constraint], bodies: &mut RigidBodySet) {
    for c in constraints {
        for p in &c.points {
            if p.normal_impulse != 0.0 {
                let impulse = c.normal.scale(p.normal_impulse);
                apply_impulse(bodies, c.body_a, impulse, p.r_a, -1.0);
                apply_impulse(bodies, c.body_b, impulse, p.r_b, 1.0);
            }
            if p.tangent_impulse.0 != 0.0 {
                let impulse = c.tangent.0.scale(p.tangent_impulse.0);
                apply_impulse(bodies, c.body_a, impulse, p.r_a, -1.0);
                apply_impulse(bodies, c.body_b, impulse, p.r_b, 1.0);
            }
            if p.tangent_impulse.1 != 0.0 {
                let impulse = c.tangent.1.scale(p.tangent_impulse.1);
                apply_impulse(bodies, c.body_a, impulse, p.r_a, -1.0);
                apply_impulse(bodies, c.body_b, impulse, p.r_b, 1.0);
            }
        }
    }
}

/// 設計 §4.5「split impulse / NGS」: 速度とは別チャンネルで貫入を直接解消する。
/// `Δλ = β_pos・max(δ-δ_slop,0)・m_eff` を位置・姿勢へ直接適用し(速度は変更しない)、
/// エネルギーを汚さない。r_a/r_b はワールド系オフセットとして固定のまま扱う近似
/// (位置補正は小さく、反復間の姿勢変化による re-projection は Phase 2 の精緻化課題)。
/// inv_inertia_world の再計算は反復ごとには行わない(同じ理由、ステップ末の
/// `update_inertia_and_clear_accum` に委ねる)。
///
/// 各反復・各点で現在の body 位置から貫入量を**再計算**する(NGS の要点)。同一 body に
/// 複数の接触点がある場合(例: 箱の4隅)、ある点の補正が他の点の実質的な貫入量も
/// 変えるため、固定値を独立に減算すると過剰補正になる — 毎回 prepare 時の
/// `p.penetration` と現在位置のズレから引き直すことでこれを避ける。
fn position_correction(constraints: &[Constraint], bodies: &mut RigidBodySet) {
    for _ in 0..POSITION_ITERATIONS {
        for c in constraints {
            for p in &c.points {
                let current_a = bodies.position[c.body_a] + p.r_a;
                let current_b = bodies.position[c.body_b] + p.r_b;
                // prepare 時は current_a == current_b == p.world_point だったので、
                // ズレ (current_b - current_a)・n がこれまでの累積補正による分離量の変化。
                let drift = (current_b - current_a).dot(c.normal);
                let current_penetration = p.penetration - drift;

                let excess = (current_penetration - SLOP).max(0.0);
                if excess <= 0.0 {
                    continue;
                }
                let lambda = BAUMGARTE_BETA_POS * excess * p.normal_mass;
                if lambda <= 0.0 {
                    continue;
                }
                let correction = c.normal.scale(lambda);

                let inv_mass_a = bodies.inv_mass[c.body_a];
                let inv_mass_b = bodies.inv_mass[c.body_b];
                bodies.position[c.body_a] =
                    bodies.position[c.body_a].addcarry_scaled(correction, -inv_mass_a);
                bodies.position[c.body_b] =
                    bodies.position[c.body_b].addcarry_scaled(correction, inv_mass_b);

                let inv_ia = bodies.inv_inertia_world[c.body_a];
                let inv_ib = bodies.inv_inertia_world[c.body_b];
                let ang_a = inv_ia.mul_vec(p.r_a.cross(correction));
                let ang_b = inv_ib.mul_vec(p.r_b.cross(correction));
                bodies.rotation[c.body_a] =
                    bodies.rotation[c.body_a].integrate_angular_velocity(ang_a.scale(-1.0), 1.0);
                bodies.rotation[c.body_b] =
                    bodies.rotation[c.body_b].integrate_angular_velocity(ang_b, 1.0);
            }
        }
    }
}

/// 接触解決の1ステップ分: prepare → warm start 適用 → velocity iterations(法線→摩擦、固定順)
/// → position iterations(split impulse、§4.5)→ 次ステップ用に累積インパルスとローカル
/// アンカーをキャッシュへ書き戻す。設計 §4.1/§4.4/§4.5/§4.7。
///
/// 書き戻しの際、**このステップで扱ったボディ対のうち今回現れなかった feature_id を
/// 削除する**(接触点が消えた分の GC、設計 §4.7 のマニフォールド持続化)。
/// ボディ対そのものが接触を終えた分の GC は `ManifoldCache::retain_pairs` が担う
/// (`resolve` にはスリープでスキップされたペアが渡ってこないため、ここでは判断できない)。
pub fn resolve(
    manifolds: &[ContactManifold],
    bodies: &mut RigidBodySet,
    materials: &MaterialDb,
    restitution_velocity_threshold: f64,
    warm_start_cache: &mut ManifoldCache,
) {
    let mut constraints = prepare(
        manifolds,
        bodies,
        materials,
        restitution_velocity_threshold,
        warm_start_cache,
    );
    apply_warm_start(&constraints, bodies);
    for _ in 0..VELOCITY_ITERATIONS {
        for c in &mut constraints {
            solve_normal(c, bodies);
            solve_tangent(c, bodies);
            solve_rolling(c, bodies);
        }
    }
    position_correction(&constraints, bodies);

    let mut touched: BTreeSet<(usize, usize, u32)> = BTreeSet::new();
    let mut solved_pairs: BTreeSet<(usize, usize)> = BTreeSet::new();
    for c in &constraints {
        solved_pairs.insert((c.body_a, c.body_b));
        for p in &c.points {
            let key = (c.body_a, c.body_b, p.feature_id);
            touched.insert(key);
            warm_start_cache.points.insert(
                key,
                CachedPoint {
                    local_a: p.local_a,
                    local_b: p.local_b,
                    impulse: WarmStartImpulse {
                        normal: p.normal_impulse,
                        tangent: p.tangent_impulse,
                    },
                },
            );
        }
    }
    if warm_start_cache.persistence_enabled {
        warm_start_cache
            .points
            .retain(|key, _| touched.contains(key) || !solved_pairs.contains(&(key.0, key.1)));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::{BodyType, RigidBodyDesc};
    use crate::collision::ContactPoint;
    use sim_math::Quat;

    fn identity_at(position: Vec3) -> Transform {
        Transform {
            position,
            rotation: Quat::IDENTITY,
        }
    }

    /// 地面(Static な箱)の上に箱を1つ置いた最小構成を作り、`resolve` を1回呼んで
    /// キャッシュに接触点を1つ載せる。戻り値はキャッシュ・その唯一のキー・
    /// キャッシュ時点の両ボディの姿勢(再利用判定はこの姿勢からのずれで決まる)。
    fn cache_with_one_contact() -> (ManifoldCache, (usize, usize, u32), (Transform, Transform)) {
        let materials = MaterialDb::standard();
        let wood = materials.find_by_name("木材(松)").unwrap();
        let mut bodies = RigidBodySet::new();

        let ground = RigidBodyDesc {
            body_type: BodyType::Static,
            ..RigidBodyDesc::dynamic(
                Shape::Box {
                    half_extents: Vec3::new(5.0, 0.5, 5.0),
                },
                wood,
            )
        };
        let a = bodies.create_body(ground, &materials);
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(0.5, 0.5, 0.5),
            },
            wood,
        );
        desc.transform.position = Vec3::new(0.0, 1.0, 0.0);
        let b = bodies.create_body(desc, &materials);
        // 法線方向に押し合っている状態にする(warm start インパルスが非ゼロになるよう、
        // 接近速度を与える)。
        bodies.linear_velocity[b] = Vec3::new(0.0, -1.0, 0.0);

        let manifold = ContactManifold {
            body_a: a,
            body_b: b,
            normal: Vec3::new(0.0, 1.0, 0.0),
            points: vec![ContactPoint {
                world_point: Vec3::new(0.0, 0.5, 0.0),
                penetration: 0.001,
                feature_id: 7,
            }],
        };

        let xf_a = identity_at(bodies.position[a]);
        let xf_b = identity_at(bodies.position[b]);

        let mut cache = ManifoldCache::new();
        resolve(&[manifold], &mut bodies, &materials, 0.5, &mut cache);
        assert_eq!(cache.len(), 1);
        assert!(
            cache.points[&(a, b, 7)].impulse.normal > 0.0,
            "the cached entry must carry a non-zero normal impulse for the test to mean anything"
        );
        (cache, (a, b, 7), (xf_a, xf_b))
    }

    /// 設計 §4.7「点の再利用判定: 同一 feature_id かつ移動 < 2mm」。
    /// アンカーのずれが許容値未満なら引き継ぎ、超えたら引き継がない。
    #[test]
    fn warm_start_is_inherited_only_while_the_anchors_stay_within_two_millimetres() {
        let (cache, key, (xf_a, xf_b)) = cache_with_one_contact();
        let cached = cache.points[&key].impulse.normal;
        let slid = |offset: Vec3| identity_at(xf_b.position + offset);

        // ずれ 0(両ボディとも動いていない)→ 引き継ぐ。
        let kept = cache.warm_start_for(key, xf_a, xf_b);
        assert_eq!(kept.normal, cached);

        // ずれ 1mm(許容内)→ 引き継ぐ。
        let kept = cache.warm_start_for(key, xf_a, slid(Vec3::new(0.001, 0.0, 0.0)));
        assert_eq!(kept.normal, cached);

        // ずれ 5mm(許容超)→ 引き継がない。
        let dropped = cache.warm_start_for(key, xf_a, slid(Vec3::new(0.005, 0.0, 0.0)));
        assert_eq!(
            dropped.normal, 0.0,
            "an anchor that slid {PERSISTENCE_TOLERANCE} m or more is a different physical \
             contact point and must not inherit the accumulated impulse"
        );
    }

    /// 持続化を切ると移行前の挙動(ずれに関係なく無条件に引き継ぐ)に戻ること
    /// ——対照実験のスイッチが本当に「移行前」を再現していることの確認。
    #[test]
    fn disabling_persistence_restores_the_unconditional_pre_migration_inheritance() {
        let (mut cache, key, (xf_a, xf_b)) = cache_with_one_contact();
        cache.persistence_enabled = false;
        let cached = cache.points[&key].impulse.normal;

        let kept = cache.warm_start_for(
            key,
            xf_a,
            identity_at(xf_b.position + Vec3::new(1.0, 0.0, 0.0)), // 1m ずれても引き継いでしまう
        );
        assert_eq!(kept.normal, cached);

        // GC も行われない。
        cache.retain_pairs(&BTreeSet::new());
        assert_eq!(cache.len(), 1);
    }

    /// 接触が消えたボディ対のエントリが `retain_pairs` で捨てられること(GC)。
    #[test]
    fn retain_pairs_drops_entries_for_pairs_that_are_no_longer_touching() {
        let (mut cache, key, _) = cache_with_one_contact();
        let live: BTreeSet<(usize, usize)> = [(key.0, key.1)].into_iter().collect();
        cache.retain_pairs(&live);
        assert_eq!(cache.len(), 1, "a live pair must be kept");

        cache.retain_pairs(&BTreeSet::new());
        assert_eq!(
            cache.len(),
            0,
            "a pair that stopped touching must be dropped"
        );
    }
}
