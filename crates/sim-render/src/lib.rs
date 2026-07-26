//! docs/17-rendering/ (レンダリングアーキテクチャ・パストレーシング・マテリアル/カメラ)。Phase Dで実装。
//!
//! **実装順序**(設計docs/17-rendering/02-path-tracing.md §8「BVH + 拡散/鏡面BSDF + NEE
//! (基本パストレ) → 分光・屈折・コースティクス → 参加媒質 → 被写界深度・モーションブラー」)
//! のうち、本増分は拡散(Lambertian)+誘電体(`Dielectric`、実屈折率のみ)BSDF + 一様
//! 環境放射輝度 + 複数物体シーン + NEE(`PointLight`)による経路追跡(`path_tracer`
//! モジュールdoc参照)を実装し、R1(白色炉テスト)+R2(誘電体側)をGreen化した。
//! さらに分散(`CauchyDielectric`、Cauchy式、`bsdf`モジュールdoc参照)+金属BSDF
//! (`Metal`、複素屈折率$n+ik$の完全鏡面、GGX粗さは対象外)を追加し、R3(分光/屈折の
//! 分散側)の核となる物理(波長ごとに屈折角が異なること)を検証した(`Scene`/
//! `trace`全体への波長の配線(hero wavelength法)自体は後続増分)。
//! さらに薄レンズモデルの物理カメラ(`Camera`、`camera`モジュールdoc参照)を追加し、
//! R6(被写界深度: 錯乱円径が薄レンズ公式と一致)を検証した。
//! BVH・GGXマイクロファセット(粗さ)・完全な分光レンダリング・参加媒質は
//! 後続増分(各モジュールdoc「縮約実装の理由」参照)。

mod bsdf;
mod camera;
mod path_tracer;
mod ray;
mod sphere;

pub use bsdf::{CauchyDielectric, Dielectric, Lambertian, Metal};
pub use camera::Camera;
pub use path_tracer::{Material, PointLight, Scene, SceneObject};
pub use ray::Ray;
pub use sphere::{Hit, Sphere};
