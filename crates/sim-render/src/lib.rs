//! docs/17-rendering/ (レンダリングアーキテクチャ・パストレーシング・マテリアル/カメラ)。Phase Dで実装。
//!
//! **実装順序**(設計docs/17-rendering/02-path-tracing.md §8「BVH + 拡散/鏡面BSDF + NEE
//! (基本パストレ) → 分光・屈折・コースティクス → 参加媒質 → 被写界深度・モーションブラー」)
//! のうち、本増分は拡散(Lambertian)+誘電体(`Dielectric`、実屈折率のみ)BSDF + 一様
//! 環境放射輝度 + 複数物体シーン + NEE(`PointLight`)による経路追跡(`path_tracer`
//! モジュールdoc参照)を実装し、R1(白色炉テスト)+R2(誘電体側)をGreen化した。
//! さらに分散(`CauchyDielectric`、Cauchy式、`bsdf`モジュールdoc参照)+金属BSDF
//! (`Metal`、複素屈折率$n+ik$の完全鏡面、GGX粗さは対象外)を追加し、波長ごとに
//! 屈折角が異なることを検証した。続けてプリズム(頂角2面での屈折)・雨粒
//! (屈折→内部反射→屈折)を`Dielectric::refract`/`reflect`(レンダラ自身が経路
//! 追跡に使う同じ幾何プリミティブ)で実際にレイ追跡する`prism`モジュールを追加し、
//! プリズム最小偏角(`sim_em::optics::prism_min_deviation`の閉形式にrel<1e-9で
//! 一致)・虹の偏角(古典的なDescartes閉形式$D=\pi+2i-4r$にrel<1e-9で一致・
//! 数値走査で求めた最小偏角が古典的な約42°の虹の角度と一致)・分散による波長ごとの
//! 偏角の違い(プリズムはBK7の実測Cauchy係数、雨粒は水自体の分散係数が設計の
//! 材質表に未収録なため同じBK7係数を代用)を検証し、R3(分光/屈折)をGreen化した
//! (`Scene`/`trace`全体への波長の配線(hero wavelength法)自体は後続増分、
//! `prism`モジュールdoc参照)。
//! さらに薄レンズモデルの物理カメラ(`Camera`、`camera`モジュールdoc参照)を追加し、
//! R6(被写界深度: 錯乱円径が薄レンズ公式と一致)を検証し、参加媒質(レイリー散乱、
//! `medium`モジュールdoc参照)の単一散乱閉形式解を追加してR5(大気レイリー散乱の
//! λ^-4による空の青・地平線の赤の定量)を検証した。意図的に分散を持たせた検証専用
//! シーンでR7(モンテカルロ収束O(1/√N)・決定論)も検証した。さらにGGXマイクロ
//! ファセット分布(`RoughConductor`、`microfacet`モジュールdoc参照、粗い金属のみ)
//! を追加した。
//! BVH・GGX粗い誘電体・完全な分光レンダリング(hero wavelength法)・
//! マルチスキャッタリング・ミー散乱・煙/水の体積散乱・コーネルボックス(R4)は
//! 後続増分(各モジュールdoc「縮約実装の理由」参照)。

mod bsdf;
mod camera;
mod medium;
mod microfacet;
mod path_tracer;
mod prism;
mod ray;
mod sphere;

pub use bsdf::{CauchyDielectric, Dielectric, Lambertian, Metal, RoughConductor};
pub use camera::Camera;
pub use medium::{rayleigh_phase, rayleigh_scattering_coefficient, HomogeneousMedium};
pub use microfacet::{ggx_distribution, sample_ggx_half_vector, smith_g, smith_g1};
pub use path_tracer::{Material, PointLight, Scene, SceneObject};
pub use prism::{trace_prism_deviation, trace_raindrop_deviation};
pub use ray::Ray;
pub use sphere::{Hit, Sphere};
