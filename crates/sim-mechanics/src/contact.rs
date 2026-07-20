//! Sequential impulses 接触ソルバ。設計: docs/10-mechanics/03-contact-solver.md、
//! docs/10-mechanics/04-friction.md(接線 solve)。
//!
//! 法線 + 反発 + split impulse 位置補正 + 箱近似クーロン摩擦 + warm starting
//! (設計 §4.1/§4.4/§4.5)。§4.4 warm starting は本来 Phase 1 スコープ — 4段スタック(M12)の
//! 収束にはこれが鍵、docs/22-roadmap/01-phases.md 横断ルール5に基づき実装漏れを訂正。
//! velocity_bias の Baumgarte 項は split impulse(§4.5)の位置チャンネルに置き換えたため
//! 廃止(設計 §4.5 の指摘どおり「エネルギーを汚さないため反発テストが厳密になる」)。
//! マニフォールド持続化の feature_id マッチング(移動量2mm以内チェック、設計 §4.7)は未実装。

use crate::body::RigidBodySet;
use crate::collision::ContactManifold;
use sim_core::MaterialDb;
use sim_math::{Mat3, Vec3};
use std::collections::BTreeMap;

/// Warm starting 用の永続キャッシュ。キーは (body_a, body_b, feature_id)。
/// 設計 §4.4「前ステップの累積インパルス(feature_idで対応づけ)をソルバ開始時に適用」。
/// 簡易実装: 毎ステップ現在の接触点で上書きするのみで、接触が消えた古いキーの明示的な
/// 削除(GC)は行わない(body 削除・再利用が未実装のため実害はない、Phase 2 の精緻化課題)。
pub type WarmStartCache = BTreeMap<(usize, usize, u32), WarmStartImpulse>;

#[derive(Clone, Copy, Default)]
pub struct WarmStartImpulse {
    normal: f64,
    tangent: (f64, f64),
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

struct PointConstraint {
    r_a: Vec3,
    r_b: Vec3,
    feature_id: u32,
    normal_mass: f64,
    tangent_mass: (f64, f64),
    velocity_bias: f64,
    /// split impulse(§4.5)の位置補正チャンネル専用。速度には影響しない。
    penetration: f64,
    normal_impulse: f64,
    tangent_impulse: (f64, f64),
}

struct Constraint {
    body_a: usize,
    body_b: usize,
    normal: Vec3,
    tangent: (Vec3, Vec3),
    friction: f64,
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
    warm_start_cache: &WarmStartCache,
) -> Vec<Constraint> {
    manifolds
        .iter()
        .map(|m| {
            let a = m.body_a;
            let b = m.body_b;
            let (t1, t2) = m.normal.orthonormal_basis();
            let friction = materials.friction_pair(bodies.material[a], bodies.material[b]);
            let restitution = materials.restitution_pair(bodies.material[a], bodies.material[b]);

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

                    let v_a =
                        point_velocity(bodies.linear_velocity[a], bodies.angular_velocity[a], r_a);
                    let v_b =
                        point_velocity(bodies.linear_velocity[b], bodies.angular_velocity[b], r_b);
                    let v_n_pre = m.normal.dot(v_b - v_a);

                    // 設計 §4.3(符号は実装時に訂正、docs/10-mechanics/03-contact-solver.md 参照)。
                    // Baumgarte 項は含めない(§4.5 split impulse の位置チャンネルに分離)。
                    let restitution_bias =
                        restitution * (-v_n_pre - restitution_velocity_threshold).max(0.0);

                    let warm = warm_start_cache
                        .get(&(a, b, p.feature_id))
                        .copied()
                        .unwrap_or_default();

                    PointConstraint {
                        r_a,
                        r_b,
                        feature_id: p.feature_id,
                        normal_mass,
                        tangent_mass,
                        velocity_bias: restitution_bias,
                        penetration: p.penetration,
                        normal_impulse: warm.normal,
                        tangent_impulse: warm.tangent,
                    }
                })
                .collect();

            Constraint {
                body_a: a,
                body_b: b,
                normal: m.normal,
                tangent: (t1, t2),
                friction,
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
/// → position iterations(split impulse、§4.5)→ 次ステップ用に累積インパルスをキャッシュへ
/// 書き戻す。設計 §4.1/§4.4/§4.5。
pub fn resolve(
    manifolds: &[ContactManifold],
    bodies: &mut RigidBodySet,
    materials: &MaterialDb,
    restitution_velocity_threshold: f64,
    warm_start_cache: &mut WarmStartCache,
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
        }
    }
    position_correction(&constraints, bodies);
    for c in &constraints {
        for p in &c.points {
            warm_start_cache.insert(
                (c.body_a, c.body_b, p.feature_id),
                WarmStartImpulse {
                    normal: p.normal_impulse,
                    tangent: p.tangent_impulse,
                },
            );
        }
    }
}
