//! 力学ソルバ。設計: docs/10-mechanics/01-rigid-body.md、02-collision-detection.md。
//!
//! P1 スコープ(docs/22-roadmap/01-phases.md): 剛体状態・慣性テンソル・重力積分。
//! 衝突検出(broadphase/narrowphase)・接触ソルバ・摩擦・最小CCDは後続の増分で追加する。
//! Phase 0 の `FallingBody` 最小実装はこの正式な `RigidBodySet`/`MechanicsSolver` に置き換えた。

mod body;
mod shape;
mod solver;

pub use body::{BodyType, DragModel, RigidBodyDesc, RigidBodySet, ShapeHandle, ShapeStore};
pub use shape::{Aabb, Shape};
pub use solver::MechanicsSolver;
