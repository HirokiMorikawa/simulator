//! docs/16-astro/ (N体重力・軌道力学・相対論的補正)、docs/20-integration/05-frame-hierarchy.md、06-regime-switching.md。Pαで実装。
//!
//! `nbody`(総当たり + leapfrog、docs/16-astro/01-gravitation-nbody.md)を実装。
//! Barnes-Hut・WHFast・浮動原点・軌道力学(ホーマン遷移等)・相対論的補正の型・トレイトの
//! スケルトンは Phase A で追加する(docs/22-roadmap/01-phases.md)。

mod nbody;
pub use nbody::{NBodySystem, GRAVITATIONAL_CONSTANT};
