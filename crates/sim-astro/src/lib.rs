//! docs/16-astro/ (N体重力・軌道力学・相対論的補正)、docs/20-integration/05-frame-hierarchy.md、06-regime-switching.md。Pαで実装。
//!
//! `nbody`(総当たり + leapfrog、docs/16-astro/01-gravitation-nbody.md)・
//! `relativity`(オプトイン1PN補正: 1PN加速度・近日点移動率・GPS固有時率、
//! docs/16-astro/03-relativistic-corrections.md、`NBodySystem`への完全統合は未実装)
//! を実装。Barnes-Hut・WHFast・浮動原点・軌道力学(ホーマン遷移以外)の型・トレイトの
//! スケルトンは Phase A で追加する(docs/22-roadmap/01-phases.md)。

mod nbody;
mod relativity;
pub use nbody::{NBodySystem, GRAVITATIONAL_CONSTANT};
pub use relativity::{
    circular_orbital_speed, gps_proper_time_rate, pn1_acceleration, pn1_precession_per_orbit,
    SPEED_OF_LIGHT,
};
