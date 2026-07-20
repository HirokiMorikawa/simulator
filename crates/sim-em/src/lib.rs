//! docs/13-electromagnetism/ (静電磁場・回路MNA・FDTD・光学・EM-力学結合)。P4(回路/静場/光学)・P5(FDTD)で実装。
//!
//! `electrostatics`(点電荷の直接和クーロン力 + Boris pusher、
//! docs/13-electromagnetism/01-electrostatics-magnetostatics.md)を実装。
//! 磁気双極子・回路MNA・FDTD・光学の型・トレイトのスケルトンは Phase A で
//! 追加する(docs/22-roadmap/01-phases.md)。

mod electrostatics;
pub use electrostatics::{PointChargeSystem, UniformField, COULOMB_CONSTANT, VACUUM_PERMITTIVITY};
