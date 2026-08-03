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
//! (`Scene`/`trace`全体への波長の配線(hero wavelength法)は`spectrum`モジュールと
//! `Scene::trace_spectral`として実装済み、
//! `prism`モジュールdoc参照)。
//! さらに薄レンズモデルの物理カメラ(`Camera`、`camera`モジュールdoc参照)を追加し、
//! R6(被写界深度: 錯乱円径が薄レンズ公式と一致)を検証し、参加媒質(レイリー散乱、
//! `medium`モジュールdoc参照)の単一散乱閉形式解を追加してR5(大気レイリー散乱の
//! λ^-4による空の青・地平線の赤の定量)を検証した。意図的に分散を持たせた検証専用
//! シーンでR7(モンテカルロ収束O(1/√N)・決定論)も検証した。さらにGGXマイクロ
//! ファセット分布(`RoughConductor`、`microfacet`モジュールdoc参照、粗い金属のみ)
//! を追加した。
//! BVH(`bvh`モジュールdoc参照、最長軸中央値分割のトップダウン構築)を
//! 追加し、多数の乱数シーン・乱数レイで最近傍ヒットが総当たりと厳密一致すること、
//! かつ実際に遠いクラスタの部分木を刈って総当たりよりテスト数が少ないことを
//! 検証した。さらにトーンマッピング(Reinhard演算子、`tonemap`モジュールdoc参照)を
//! HDR輝度→表示可能範囲[0,1]への圧縮として追加した(色相を保つ輝度ベース版)。
//!
//! **R4(コーネルボックス)に向けた増分**: ①`Primitive`(球・クアッドの総和型、
//! `primitive`モジュール)+`Quad`(平行四辺形、`quad`モジュール)を追加し、BVHを
//! `Sphere`決め打ちから`Primitive`へ一般化した上で`Scene::closest_hit`へ配線した
//! (`Scene`は`Scene::new`で構築しBVHとプリミティブ配列を一度だけ保持する)。
//! ②面光源(`Emissive`、`path_tracer`モジュールの`Emissive`のdoc参照)を追加した
//! ——MISを実装せずに済ませるため、面光源についてはNEEを一切行わない純粋な
//! BSDFサンプリングのパストレースとする(不偏、二重計上が原理的に起きない)。
//!
//! GGX粗い誘電体・完全な分光レンダリング(hero wavelength法)・マルチスキャッ
//! タリング・ミー散乱・煙/水の体積散乱・SAH分割は引き続き後続増分
//! (各モジュールdoc「縮約実装の理由」参照)。
//!
//! **群6(2026-08-03)で解消した縮約**:
//! 1. **MIS**(`Scene::enable_mis`、`path_tracer`)——面光源へのNEEを、べき乗
//!    ヒューリスティック($\beta=2$)で二重計上を打ち消しつつ併用できるようにした。
//!    小さく明るい光源で標準誤差が半分以下になることを実測。**opt-in**なので
//!    既定経路(R1–R7・R4)はビット単位で不変。
//! 2. **ロシアンルーレット**(`Scene::trace_with_roulette`、`RenderSettings::
//!    russian_roulette_after`)——`max_depth`による単純打切り(必ず下側にバイアス)を
//!    不偏な確率的打切りへ置き換えられるようにした。
//! 3. **三角形メッシュ**(`triangle`モジュール)——Möller–Trumbore交差・面積重み付き
//!    頂点法線・アイコスフィア/グリッド生成。`Primitive::Triangle`としてBVHへそのまま
//!    載る。細分したアイコスフィアが解析球と**同じ画像**になることまで確認した。
//! 4. **PNGの実圧縮**(`png`)——`stored`(非圧縮)ブロックしか書いていなかったのを、
//!    LZ77 + 固定ハフマン(RFC 1951 `BTYPE=01`)にした。実測でレンダリング結果が
//!    2.6–40倍に縮み、標準の zlib で復元できることも確認済み。
//! 5. **コースティクスの画像への合成**(`caustic::composite_onto_floor`)と
//!    **分光コースティクス**(`trace_ball_lens_caustic_spectral`、既存の`spectrum`
//!    モジュールのCIE等色関数を使う)——移行前は`CausticMap`が独立した成果物で、
//!    パストレ画像の中に集光模様が現れなかった。生成画像を実際に開いて、ガラス球の
//!    下の床に明るいプールが出ること・分光版に青い芯→緑→赤いハロの同心リング
//!    (軸上色収差)が出ることを目視で確認した。
//!
//! **増分C1(画像出力パイプライン)**: 上記のとおりR1–R7は全て単一レイ/点推定の
//! 検証であり、実際に画像を1枚も描いたことが無かった——フレームバッファ・
//! 解像度・ピクセルループ・ファイル出力が存在せず、`Camera`(薄レンズ、R6で
//! 検証済み)と`Scene::trace`が一度も接続されていなかった。本増分でこれを解消
//! する: ①`Camera::pinhole_direction`(`camera.rs`、NDC座標→ピンホール方向、
//! 既存の`generate_ray`のシグネチャは変更しない)、②`render`モジュール
//! (カメラ→ピクセル→レイ生成→`trace`→フレームバッファの配線、色は分光本体が
//! 無いためR4で実証済みの「チャンネル別レンダリング」を流用——`Scene::trace`
//! 自体には一切手を入れない)、③`framebuffer`モジュール(線形RGB放射輝度の
//! 画素配列、露出倍率→トーンマッピング→sRGBガンマ符号化→u8量子化)、
//! ④`png`モジュール(依存追加なしの自前最小PNGエンコーダ、非圧縮のstored
//! deflateブロックのみ)を追加した。あわせて、`path_tracer::AtmosphereMedium`
//! が`pub`でありながら`lib.rs`の再エクスポートに含まれておらず(`path_tracer`
//! モジュール自体が非公開のため)クレート外から型を名指しできなかった実装漏れ
//! (`Scene::new`の第4引数に`None`しか渡せなかった)を修正した。
//!
//! **増分C2(露出)**: `camera.rs`に写真測光の標準的な露出方程式(EV換算、設計§4.1)
//! を追加した(`relative_exposure`/`exposure_value_at_iso100`、詳細は
//! `camera.rs`モジュールdoc参照)。
//!
//! **増分C3(モーションブラー)**: 新規`motion_blur`モジュール(`render_motion_blur`/
//! `render_motion_blur_channel`)を追加した。`Ray`に時刻フィールドを足す代わりに
//! 「シャッター開時間内の複数時刻でシーンを構築し直して平均する」方式を採用
//! (これは縮約ではなく物理的にシャッター積分そのもの、詳細な設計判断の理由は
//! `motion_blur.rs`モジュールdoc参照)——既存の`Ray`・`Camera::generate_ray`・
//! `Scene::trace`には一切手を入れていない。

mod bsdf;
mod bvh;
mod camera;
mod caustic;
mod framebuffer;
mod medium;
mod microfacet;
mod motion_blur;
mod optical_material;
mod path_tracer;
mod png;
mod primitive;
mod prism;
mod quad;
mod ray;
mod render;
mod spectrum;
mod sphere;
mod tonemap;
mod triangle;

pub use bsdf::{CauchyDielectric, Dielectric, Lambertian, Metal, RoughConductor};
pub use bvh::{Bvh, BvhDiagnostics};
pub use camera::{exposure_value_at_iso100, relative_exposure, Camera};
pub use caustic::{
    ball_lens_paraxial_focal_distance, ball_lens_ray_focus_distance, composite_onto_floor,
    trace_ball_lens_caustic, trace_ball_lens_caustic_spectral, CausticMap,
};
pub use framebuffer::Framebuffer;
pub use medium::{rayleigh_phase, rayleigh_scattering_coefficient, HomogeneousMedium};
pub use microfacet::{ggx_distribution, sample_ggx_half_vector, smith_g, smith_g1};
pub use motion_blur::{render_motion_blur, render_motion_blur_channel};
pub use optical_material::{
    all_from_standard_db, from_material_db, IorSource, OpticalMaterial, DEFAULT_IOR,
};
pub use path_tracer::{AtmosphereMedium, Emissive, Material, PointLight, Scene, SceneObject};
pub use primitive::Primitive;
pub use prism::{trace_prism_deviation, trace_raindrop_deviation};
pub use quad::Quad;
pub use ray::Ray;
pub use render::{render_channel, render_rgb, render_spectral, RenderSettings};
pub use spectrum::{
    cie_xyz_at, hero_wavelengths, spectral_samples_to_linear_srgb, xyz_to_linear_srgb,
    LAMBDA_MAX_NM, LAMBDA_MIN_NM, Y_INTEGRAL,
};
pub use sphere::{Hit, Sphere};
pub use tonemap::{
    aces_filmic_tonemap, aces_filmic_tonemap_color, reinhard_tonemap, reinhard_tonemap_color,
    relative_luminance,
};
pub use triangle::{Triangle, TriangleMesh};
