//! docs/17-rendering/ (レンダリングアーキテクチャ・パストレーシング・マテリアル/カメラ)。Phase Dで実装。
//!
//! **実装順序**(設計docs/17-rendering/02-path-tracing.md §8「BVH + 拡散/鏡面BSDF + NEE
//! (基本パストレ) → 分光・屈折・コースティクス → 参加媒質 → 被写界深度・モーションブラー」)
//! のうち、本増分は最初の一歩として拡散(Lambertian)BSDF + 一様環境放射輝度による
//! 経路追跡(`path_tracer`モジュールdoc参照)を実装し、R1(白色炉テスト)をGreen化した。
//! BVH・NEE・鏡面/誘電体BSDF・分光・参加媒質・物理カメラは後続増分(各モジュールdoc
//! 「縮約実装の理由」参照)。

mod bsdf;
mod path_tracer;
mod ray;
mod sphere;

pub use bsdf::{Dielectric, Lambertian};
pub use path_tracer::Scene;
pub use ray::Ray;
pub use sphere::{Hit, Sphere};
