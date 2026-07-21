//! docs/13-electromagnetism/ (静電磁場・回路MNA・FDTD・光学・EM-力学結合)。P4(回路/静場/光学)・P5(FDTD)で実装。
//!
//! `electrostatics`(点電荷の直接和クーロン力 + Boris pusher、
//! docs/13-electromagnetism/01-electrostatics-magnetostatics.md)・
//! `magnetism`(磁気双極子の場・力・トルク、同docs §2)・
//! `optics`(幾何光学: スネル則・フレネル係数・薄レンズ・プリズム、
//! docs/13-electromagnetism/04-light-optics.md)・`circuit`(回路MNA: 抵抗・コンデンサ・
//! インダクタ・独立電圧源の線形素子のみ、docs/13-electromagnetism/02-circuits.md)・
//! `motor`(DCモーターの集中定数モデル)・`induction_rod`(導体棒の電磁誘導、いずれも
//! docs/13-electromagnetism/05-em-mechanics-coupling.md)・`raytracer`(幾何光学レイトレーサ:
//! 球/平面と光線の交差 + 反射/屈折の分岐トレース、プランクの法則、同docs §3/§4)・
//! `fdtd`(2D TMz Yee格子FDTD、PEC境界のみ、docs/13-electromagnetism/03-maxwell-fdtd.md)
//! を実装。光線束(rayon並列化)・波長サンプリングのCIE等色関数RGB変換・結像の
//! スクリーンビニング・鏡像力・摩擦帯電・ダイオード等の非線形素子
//! (Newton-Raphsonフォールバック連鎖)・汎用`MotorCoupling`(ヒンジモーター経由)・
//! 渦電流ブレーキ・FDTDの誘電体界面/PML/ソース の型・トレイトのスケルトンは
//! Phase A で追加する(docs/22-roadmap/01-phases.md)。

mod circuit;
mod electrostatics;
mod fdtd;
mod induction_rod;
mod magnetism;
mod motor;
mod optics;
mod raytracer;
pub use circuit::{Circuit, GROUND};
pub use electrostatics::{PointChargeSystem, UniformField, COULOMB_CONSTANT, VACUUM_PERMITTIVITY};
pub use fdtd::FdtdSim2D;
pub use induction_rod::InductionRod;
pub use magnetism::{
    dipole_field, dipole_force, dipole_torque, MagneticDipole, VACUUM_PERMEABILITY,
};
pub use motor::DcMotor;
pub use optics::{
    brewster_angle, critical_angle, fresnel_reflectance, prism_index_from_min_deviation,
    prism_min_deviation, snell_refract_angle, thin_lens_focal_length,
    thin_lens_paraxial_ray_trace_focal_length, FresnelReflectance,
};
pub use raytracer::{
    planck_spectral_radiance, trace_energy, OpticalSurface, Ray, SurfaceGeom, SurfaceKind,
};
