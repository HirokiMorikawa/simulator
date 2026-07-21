//! 最小CCD — speculative contact(P1標準機能)。設計 docs/10-mechanics/02-collision-detection.md
//! §4.6。高速小物体(弾丸級)が離散衝突検出のステップ端点判定をすり抜ける(トンネリング)
//! ことを防ぐ。対象範囲は設計どおり球(単純形状)× 静的形状(Box/Plane)のみ
//! (回転由来のトンネリング・動的物体同士・カプセルは対象外、設計§4.6「適用範囲」)。
//!
//! 実装方針: TOI(Time of Impact)反復は行わず、弾丸級と判定された球の静止形状への
//! 接近速度を「このステップで表面を通り越さない」よう速度レベルでクランプするのみ
//! (非貫入拘束、反発・摩擦は適用しない)。実際の反発は、クランプの結果として次ステップで
//! 通常の接触検出・解決(既存のsequential impulses)が実接触を検出したときに、
//! 既存の反発モデルがそのまま処理する(設計§4.6「ghost contact対策: マージン接触は
//! 非貫入拘束のみ、反発・摩擦は実接触になったステップから適用」に対応)。

use crate::body::{BodyType, RigidBodySet};
use crate::shape::Shape;
use sim_math::{Transform, Vec3};

/// 弾丸級判定のしきい値係数(設計§4.6「$\alpha=0.5$固定」)。
const ALPHA: f64 = 0.5;

/// 1ステップぶんの速度クランプを適用する。`solver::step`内で接触解決後・位置積分前に呼ぶ
/// (通常の接触解決が既存の実接触を先に処理したあと、まだ検出されていない今ステップ中の
/// すり抜けだけをここで防ぐ)。
pub fn apply_speculative_contacts(bodies: &mut RigidBodySet, dt: f64) {
    let n = bodies.len();
    for i in 0..n {
        if bodies.body_type[i] != BodyType::Dynamic || bodies.asleep[i] {
            continue;
        }
        let Shape::Sphere { radius } = *bodies.shape_of(i) else {
            continue; // 対象は球のみ(設計§4.6「対象範囲: 球・カプセル等の単純形状」)
        };
        let vel = bodies.linear_velocity[i];
        if vel.length() * dt <= ALPHA * radius {
            continue; // 弾丸級でない(設計§4.6の決定的判定、状態の関数のみで実行時適応なし)
        }
        let center = bodies.position[i];

        for j in 0..n {
            if bodies.body_type[j] == BodyType::Dynamic {
                continue; // 静的形状のみを対象にする簡略化
            }
            match *bodies.shape_of(j) {
                Shape::Plane { normal, d } => {
                    let gap = normal.dot(center) - d - radius;
                    clamp_approach_velocity(bodies, i, normal, gap, dt);
                }
                Shape::Box { half_extents } => {
                    let xf = Transform {
                        position: bodies.position[j],
                        rotation: bodies.rotation[j],
                    };
                    let local = xf.inverse().apply_point(center);
                    let clamped = Vec3::new(
                        local.x.clamp(-half_extents.x, half_extents.x),
                        local.y.clamp(-half_extents.y, half_extents.y),
                        local.z.clamp(-half_extents.z, half_extents.z),
                    );
                    let closest_world = xf.apply_point(clamped);
                    let delta = center - closest_world;
                    let dist = delta.length();
                    if dist < 1e-12 {
                        continue; // 中心が箱の内部(退化ケース、通常の接触解決に任せる)
                    }
                    let normal = delta.scale(1.0 / dist);
                    let gap = dist - radius;
                    clamp_approach_velocity(bodies, i, normal, gap, dt);
                }
                _ => {}
            }
        }
    }
}

/// 法線 `normal`(相手表面から自分へ向かう向き)に対して、現在のギャップ `gap` の
/// (ほぼ)手前で止まるよう接近速度成分だけを減速する。ちょうど`gap`ぶんで止めてしまうと
/// 実接触(貫入 ≥ 0)が一度も発生せず、離散衝突検出の重なり判定が永久にトリガーされない
/// (=速度が0のまま面に張り付いて反発が起きない)ことを実装検証中に発見した — 設計§4.6の
/// 「マージン接触は非貫入拘束のみ、反発は実接触になったステップから適用」を実現するには、
/// 実接触に確実に引き渡すため`OVERSHOOT`ぶんだけわずかに実貫入させる必要がある
/// (`OVERSHOOT`はslopより十分小さく設定、次ステップの通常の接触解決に安全に委ねられる)。
const OVERSHOOT: f64 = 0.2; // 半径に対する比率
fn clamp_approach_velocity(bodies: &mut RigidBodySet, i: usize, normal: Vec3, gap: f64, dt: f64) {
    if gap < 0.0 {
        return; // 既に貫入している(通常の接触解決が扱う範囲)
    }
    let vel = bodies.linear_velocity[i];
    let closing_speed = -normal.dot(vel); // 表面へ近づく向きを正とする
    if closing_speed <= 0.0 {
        return; // 離れていく、または平行
    }
    let radius = match bodies.shape_of(i) {
        Shape::Sphere { radius } => *radius,
        _ => 0.0,
    };
    let allowed_travel = gap + OVERSHOOT * radius;
    let max_closing_speed = allowed_travel / dt;
    if closing_speed <= max_closing_speed {
        return; // このステップで通り越さない
    }
    let excess = closing_speed - max_closing_speed;
    bodies.linear_velocity[i] = vel + normal.scale(excess);
}
