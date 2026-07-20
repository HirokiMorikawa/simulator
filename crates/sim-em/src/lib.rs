//! docs/13-electromagnetism/ (静電磁場・回路MNA・FDTD・光学・EM-力学結合)。P4(回路/静場/光学)・P5(FDTD)で実装。
//!
//! `electrostatics`(点電荷の直接和クーロン力 + Boris pusher、
//! docs/13-electromagnetism/01-electrostatics-magnetostatics.md)・
//! `optics`(幾何光学: スネル則・フレネル係数・薄レンズ・プリズム、
//! docs/13-electromagnetism/04-light-optics.md)・`circuit`(回路MNA: 抵抗・コンデンサ・
//! インダクタ・独立電圧源の線形素子のみ、docs/13-electromagnetism/02-circuits.md)を実装。
//! フル `RayTracer`(光線束追跡・分岐・分光)・磁気双極子・ダイオード/モーター等の非線形
//! 素子(Newton-Raphsonフォールバック連鎖)・FDTD の型・トレイトのスケルトンは Phase A で
//! 追加する(docs/22-roadmap/01-phases.md)。

mod circuit;
mod electrostatics;
mod optics;
pub use circuit::{Circuit, GROUND};
pub use electrostatics::{PointChargeSystem, UniformField, COULOMB_CONSTANT, VACUUM_PERMITTIVITY};
pub use optics::{
    brewster_angle, critical_angle, fresnel_reflectance, prism_index_from_min_deviation,
    prism_min_deviation, snell_refract_angle, thin_lens_focal_length,
    thin_lens_paraxial_ray_trace_focal_length, FresnelReflectance,
};
