//! docs/11-fluid/ (連続体基礎・格子流体・SPH・自由表面/浮力・空力)。P1(浮力/抗力)・P3(格子流体)・P4(SPH)で実装。
//!
//! P1: `aero`(集中定数の抗力モデル、docs/11-fluid/05-aero-hydrodynamics.md)・
//! `buoyancy`(集中定数の浮力モデル、docs/11-fluid/04-free-surface-buoyancy.md)を実装。
//! P4: `sph`(弱圧縮SPH、docs/11-fluid/03-sph.md)を実装。P3: `grid_fluid`
//! (格子流体、docs/11-fluid/02-eulerian-grid.md、2D周期境界のみ・F8/F9)・
//! `poiseuille`(ポアズイユ流、完全発達した平行平板間流れが厳密に1D陰的粘性拡散に
//! 帰着することを使った専用縮約実装、F7)を実装。カルマン渦列(F11、円柱障害物+
//! 渦度強化の要否判断)は後続増分に残す。

mod aero;
mod buoyancy;
mod grid_fluid;
mod poiseuille;
mod sph;
pub use aero::{
    drag_coefficient_sphere, drag_force_sphere, reynolds_number, terminal_velocity_high_re,
    Atmosphere,
};
pub use buoyancy::{
    buoyancy_force, hydrostatic_pressure, submerged_box_axis_aligned, StaticWaterRegion,
};
pub use grid_fluid::GridFluid2D;
pub use poiseuille::PoiseuilleChannel1D;
pub use sph::SphFluid;
