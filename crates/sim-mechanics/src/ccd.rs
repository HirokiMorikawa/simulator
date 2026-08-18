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
//!
//! **群9でフルCCD(設計§4.6「フルCCD(Phase 5): 一般形状の conservative advancement」)を
//! 配線した**(`apply_conservative_advancement`)。`gjk::conservative_advancement_hit` は
//! 以前から実装・テスト済みだったが**ワークスペースのどこからも呼ばれておらず**、
//! 実際の力学ステップには一切効いていなかった。speculative pass が原理的に届かない
//! 「球以外の弾丸」「動的な相手」をこちらが受け持つ。回転は扱わない(conservative
//! advancement が並進のみ)——設計§4.6も「回転を含む一般形状」をPhase 5としており、
//! ここで閉じるのは並進側だけであることを正直に記録する。

use crate::body::{collision_filter_allows, BodyType, RigidBodySet};
use crate::gjk::{conservative_advancement_hit, ConvexShape};
use crate::shape::Shape;
use sim_math::Vec3;

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
        // 球の重心オフセットは常にゼロなので `position[i]` と同じだが、
        // 「幾何は `shape_transform` 経由」の規約に揃えておく
        // (`RigidBodySet` の型doc参照)。
        let center = bodies.shape_transform(i).position;

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
                    let xf = bodies.shape_transform(j);
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
const OVERSHOOT: f64 = 0.2; // 最小半厚に対する比率
fn clamp_approach_velocity(bodies: &mut RigidBodySet, i: usize, normal: Vec3, gap: f64, dt: f64) {
    let half_thickness = min_half_thickness(bodies.shape_of(i)).unwrap_or(0.0);
    clamp_relative_approach(bodies, i, normal, gap, dt, half_thickness, 0.0);
}

/// `clamp_approach_velocity` の一般形(**群9で分離**)。`relative_closing_offset` は
/// 相手が動いている場合の相手側の接近速度成分(`-normal·v_other`)で、静止形状に対する
/// 従来の呼び出しでは 0。クランプするのは常に自分(`i`)の速度だけで、相手には
/// 反作用を返さない——非貫入のための速度ガードであり、運動量のやり取りは次ステップの
/// 通常の接触解決が行う(設計§4.6「マージン接触は非貫入拘束のみ」)。
fn clamp_relative_approach(
    bodies: &mut RigidBodySet,
    i: usize,
    normal: Vec3,
    gap: f64,
    dt: f64,
    half_thickness: f64,
    relative_closing_offset: f64,
) {
    if gap < 0.0 {
        return; // 既に貫入している(通常の接触解決が扱う範囲)
    }
    let vel = bodies.linear_velocity[i];
    // 表面へ近づく向きを正とする。相手も近づいてくるなら、その分だけ余裕が減る。
    let closing_speed = -normal.dot(vel) + relative_closing_offset;
    if closing_speed <= 0.0 {
        return; // 離れていく、または平行
    }
    let allowed_travel = gap + OVERSHOOT * half_thickness;
    let max_closing_speed = allowed_travel / dt;
    if closing_speed <= max_closing_speed {
        return; // このステップで通り越さない
    }
    let excess = closing_speed - max_closing_speed;
    bodies.linear_velocity[i] = vel + normal.scale(excess);
}

/// 形状の「最小半厚」$r_{min}$(設計§4.6の弾丸級判定に使う)。凸多面体化できない
/// 形状(Plane・Capsule・Compound 等)は `None`。
fn min_half_thickness(shape: &Shape) -> Option<f64> {
    match shape {
        Shape::Sphere { radius } => Some(*radius),
        Shape::Box { half_extents } => Some(half_extents.x.min(half_extents.y).min(half_extents.z)),
        _ => None,
    }
}

/// フルCCD が扱える凸形状(`ConvexShape` は借用を持つため、頂点列の所有者が要る)。
enum CcdShape {
    Sphere { center: Vec3, radius: f64 },
    Points(Vec<Vec3>),
}

impl CcdShape {
    fn as_convex(&self) -> ConvexShape<'_> {
        match self {
            CcdShape::Sphere { center, radius } => ConvexShape::Sphere {
                center: *center,
                radius: *radius,
            },
            CcdShape::Points(points) => ConvexShape::Points(points),
        }
    }
}

/// ボディのワールド空間での凸形状。`Plane` は非有界で凸多面体として表せないため `None`
/// (無限平面との CCD は既存の speculative pass が解析的に扱う)。
fn ccd_shape_of(bodies: &RigidBodySet, i: usize) -> Option<CcdShape> {
    match *bodies.shape_of(i) {
        Shape::Sphere { radius } => Some(CcdShape::Sphere {
            center: bodies.shape_transform(i).position,
            radius,
        }),
        Shape::Box { half_extents } => {
            let xf = bodies.shape_transform(i);
            let mut corners = Vec::with_capacity(8);
            for sx in [-1.0, 1.0] {
                for sy in [-1.0, 1.0] {
                    for sz in [-1.0, 1.0] {
                        corners.push(xf.apply_point(Vec3::new(
                            sx * half_extents.x,
                            sy * half_extents.y,
                            sz * half_extents.z,
                        )));
                    }
                }
            }
            Some(CcdShape::Points(corners))
        }
        _ => None,
    }
}

/// フルCCD(設計§4.6「フルCCD(Phase 5)」、モジュールdoc参照)。
/// `apply_speculative_contacts` の直後に呼ぶ。弾丸級判定は speculative pass と同一
/// ($|v|\Delta t > \alpha r_{min}$、$\alpha=0.5$、状態の決定的関数)だが、
/// **形状を球に限定せず、相手が Dynamic でも対象にする**——これが speculative pass では
/// 原理的に届かない範囲であり、フルCCDの存在意義。
///
/// 見つかった最小のTOIについて、その時刻の直前で止まるよう自分の接近速度成分だけを
/// クランプする(`OVERSHOOT` ぶんだけわずかに行き過ぎさせ、次ステップの通常の接触解決へ
/// 実接触として引き渡す。反発・摩擦はそちらが担当する)。
pub fn apply_conservative_advancement(bodies: &mut RigidBodySet, dt: f64) {
    let n = bodies.len();
    for i in 0..n {
        if bodies.body_type[i] != BodyType::Dynamic || bodies.asleep[i] {
            continue;
        }
        let Some(half_thickness) = min_half_thickness(bodies.shape_of(i)) else {
            continue;
        };
        let vel = bodies.linear_velocity[i];
        if vel.length() * dt <= ALPHA * half_thickness {
            continue; // 弾丸級でない(設計§4.6の決定的判定)
        }
        let Some(bullet) = ccd_shape_of(bodies, i) else {
            continue;
        };

        // 最小のTOIを取る。同着は index 順で先に見つかった方を採る(決定論)。
        let mut earliest: Option<(f64, Vec3, f64)> = None;
        for j in 0..n {
            if j == i {
                continue;
            }
            if !collision_filter_allows(
                bodies.collision_group[i],
                bodies.collision_mask[i],
                bodies.collision_group[j],
                bodies.collision_mask[j],
            ) {
                continue;
            }
            let Some(target) = ccd_shape_of(bodies, j) else {
                continue; // Plane 等は speculative pass に委ねる
            };
            // `conservative_advancement_hit` は「A静止・Bだけが rel_vel で動く」等価系。
            // A = 相手、B = 弾丸 とすると rel_vel は相対速度そのもの。
            let other_vel = if bodies.body_type[j] == BodyType::Dynamic {
                bodies.linear_velocity[j]
            } else {
                Vec3::ZERO
            };
            let rel_vel = vel - other_vel;
            let Some(hit) =
                conservative_advancement_hit(&target.as_convex(), &bullet.as_convex(), rel_vel, dt)
            else {
                continue;
            };
            let Some(normal) = hit.normal else {
                continue; // 開始時点で既に重なっている(通常の接触解決の範囲)
            };
            if hit.time >= dt {
                continue;
            }
            // `normal` は B(弾丸)から A(相手)へ向かう。クランプ側は「相手の面から
            // 自分へ向かう法線」を期待するので反転する。
            let toward_bullet = normal.scale(-1.0);
            // 相対接近速度 = -n·v_self + n·v_other(n は相手から自分へ向かう法線)。
            let other_closing = toward_bullet.dot(other_vel);
            match earliest {
                Some((t, _, _)) if t <= hit.time => {}
                _ => earliest = Some((hit.time, toward_bullet, other_closing)),
            }
        }

        if let Some((toi, normal, other_closing)) = earliest {
            // TOI までに詰まる法線方向の距離が、そのまま「残りギャップ」になる。
            let closing_speed = -normal.dot(vel) + other_closing;
            if closing_speed <= 0.0 {
                continue;
            }
            let gap = closing_speed * toi;
            clamp_relative_approach(bodies, i, normal, gap, dt, half_thickness, other_closing);
        }
    }
}
