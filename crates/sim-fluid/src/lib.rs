//! docs/11-fluid/ (連続体基礎・格子流体・SPH・自由表面/浮力・空力)。P1(浮力/抗力)・P3(格子流体)・P4(SPH)で実装。
//!
//! P1: `aero`(集中定数の抗力モデル、docs/11-fluid/05-aero-hydrodynamics.md)・
//! `buoyancy`(集中定数の浮力モデル、docs/11-fluid/04-free-surface-buoyancy.md)を実装。
//! P4: `sph`(弱圧縮SPH、docs/11-fluid/03-sph.md)を実装。P3: `grid_fluid`
//! (格子流体、docs/11-fluid/02-eulerian-grid.md、2D周期境界のみ・F8/F9)・
//! `poiseuille`(ポアズイユ流、完全発達した平行平板間流れが厳密に1D陰的粘性拡散に
//! 帰着することを使った専用縮約実装、F7)・`karman`(カルマン渦列、流入/流出境界+円柱の
//! マスキング方式固体セル、渦度強化を設計§4.5の代替経路として使用、F11)・
//! `grid_fluid_rigid`(格子流体×剛体の疎結合、ばね拘束された箱による付加質量不安定性の
//! 検証、X2)を実装。F10(ダム崩壊)はMartin & Moyce 1952実測データ入手待ちのまま未着手。

mod aero;
mod buoyancy;
mod grid_fluid;
mod grid_fluid_rigid;
mod karman;
mod poiseuille;
mod sph;
pub use aero::{
    drag_coefficient_sphere, drag_force_sphere, reynolds_number, terminal_velocity_high_re,
    Atmosphere,
};
pub use buoyancy::{
    buoyancy_force, hydrostatic_pressure, submerged_box_axis_aligned, StaticWaterRegion,
};
pub use grid_fluid::{GridFluid2D, GridSolidBox};
pub use grid_fluid_rigid::GridFluidRigidBox2D;
pub use karman::KarmanChannel2D;
pub use poiseuille::PoiseuilleChannel1D;
pub use sph::SphFluid;
