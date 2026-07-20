//! docs/11-fluid/ (連続体基礎・格子流体・SPH・自由表面/浮力・空力)。P1(浮力/抗力)・P3(格子流体)・P4(SPH)で実装。
//!
//! P1: `aero`(集中定数の抗力モデル、docs/11-fluid/05-aero-hydrodynamics.md)を実装。
//! 格子流体・SPH・自由表面/浮力の型・トレイトのスケルトンは Phase A で追加する
//! (docs/22-roadmap/01-phases.md)。

mod aero;
pub use aero::{
    drag_coefficient_sphere, drag_force_sphere, reynolds_number, terminal_velocity_high_re,
    Atmosphere,
};
