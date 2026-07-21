//! docs/16-astro/ (N体重力・軌道力学・相対論的補正)、docs/20-integration/05-frame-hierarchy.md、06-regime-switching.md。Pαで実装。
//!
//! `nbody`(総当たり + leapfrog、docs/16-astro/01-gravitation-nbody.md)・
//! `relativity`(オプトイン1PN補正: 1PN加速度・近日点移動率・GPS固有時率・光の重力偏向、
//! docs/16-astro/03-relativistic-corrections.md、`NBodySystem`への完全統合は未実装)・
//! `atmosphere`(指数大気モデル、docs/16-astro/02-orbital-mechanics.md §2.3、
//! 空力加熱・アブレーション・レジーム切替は未実装)を実装。Barnes-Hut・WHFast・
//! 浮動原点・軌道力学(ホーマン遷移以外)の型・トレイトのスケルトンは Phase A で
//! 追加する(docs/22-roadmap/01-phases.md)。

mod atmosphere;
mod nbody;
mod relativity;
pub use atmosphere::exponential_atmosphere_density;
pub use nbody::{NBodySystem, GRAVITATIONAL_CONSTANT};
pub use relativity::{
    circular_orbital_speed, gps_proper_time_rate, light_deflection_angle, pn1_acceleration,
    pn1_precession_per_orbit, SPEED_OF_LIGHT,
};
