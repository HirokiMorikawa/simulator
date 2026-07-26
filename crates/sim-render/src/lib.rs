//! docs/17-rendering/ (レンダリングアーキテクチャ・パストレーシング・マテリアル/カメラ)。Phase Dで実装。
//!
//! **実装順序**(設計docs/17-rendering/02-path-tracing.md §8「BVH + 拡散/鏡面BSDF + NEE
//! (基本パストレ) → 分光・屈折・コースティクス → 参加媒質 → 被写界深度・モーションブラー」)
//! のうち、本増分は拡散(Lambertian)+誘電体(`Dielectric`、実屈折率のみ)BSDF + 一様
//! 環境放射輝度による経路追跡(`path_tracer`モジュールdoc参照)を実装し、R1(白色炉
//! テスト)+R2(誘電体側)をGreen化した。BVH・NEE・金属BSDF・分光・参加媒質・
//! 物理カメラは後続増分(各モジュールdoc「縮約実装の理由」参照)。

mod bsdf;
mod path_tracer;
mod ray;
mod sphere;

pub use bsdf::{Dielectric, Lambertian};
pub use path_tracer::{Material, Scene};
pub use ray::Ray;
pub use sphere::{Hit, Sphere};
