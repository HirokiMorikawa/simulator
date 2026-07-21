//! 力学ソルバ。設計: docs/10-mechanics/01-rigid-body.md、02-collision-detection.md、
//!       03-contact-solver.md、04-friction.md、docs/20-integration/03-entity-layer.md(車両)。
//!
//! P1 スコープ(docs/22-roadmap/01-phases.md): 剛体状態・慣性テンソル・重力積分・
//! 総当たり衝突検出(Sphere/Box/Plane)・sequential impulses 接触ソルバ・箱近似クーロン摩擦。
//! 最小CCD・warm starting・split impulse・Box-Box(SAT)は後続の増分で追加する。
//! Phase 0 の `FallingBody` 最小実装はこの正式な `RigidBodySet`/`MechanicsSolver` に置き換えた。
//! P4: `vehicle`(簡易Pacejkaタイヤモデル、フルの`WheelJoint`剛体シミュレーションではなく
//! 制動距離・定常円旋回の受け入れ基準を単独のスカラーODEで直接検証する縮約実装)・
//! `joint::HingeMotorPd`(PD位置サーボ付きヒンジモーター、正式なHingeジョイントの軸直交
//! 拘束行を持たない縮約実装、`joint`モジュールdoc参照。エンティティ層の関節PD静的姿勢
//! 維持の受け入れ基準、docs/20-integration/03-entity-layer.md §7)。

mod body;
mod ccd;
mod collision;
mod contact;
mod gjk;
mod joint;
mod shape;
mod sleep;
mod soft_body;
mod solver;
mod vehicle;

pub use body::{BodyType, DragModel, RigidBodyDesc, RigidBodySet, ShapeHandle, ShapeStore};
pub use collision::{ContactManifold, ContactPoint};
pub use gjk::{
    conservative_advancement_toi, epa_penetration, gjk_distance, ConvexShape, EpaResult, GjkResult,
};
pub use joint::{BallJoint, DistanceJoint, HingeMotorPd};
pub use shape::{Aabb, Shape};
pub use soft_body::{
    rope, DistanceConstraint, SoftBody, DEFAULT_DAMPING, DEFAULT_ITERATIONS, DEFAULT_SUBSTEPS,
};
pub use solver::MechanicsSolver;
pub use vehicle::{pacejka_force, pacejka_peak_slip, PacejkaParams};
