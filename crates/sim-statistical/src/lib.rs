//! docs/15-statistical/ (ミクロマクロ橋渡し・気体分子運動・拡散/ブラウン運動・モンテカルロ)。P4(ランジュバン)・P5(気体分子/イジング)で実装。
//!
//! `brownian`(ランジュバン方程式・BAOAB積分、docs/15-statistical/03-diffusion-brownian.md)を
//! 実装。気体分子運動・モンテカルロ(イジング)の型・トレイトのスケルトンは Phase A で
//! 追加する(docs/22-roadmap/01-phases.md)。

mod brownian;
pub use brownian::BrownianParticleSet;
