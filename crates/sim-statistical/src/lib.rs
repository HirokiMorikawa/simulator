//! docs/15-statistical/ (ミクロマクロ橋渡し・気体分子運動・拡散/ブラウン運動・モンテカルロ)。P4(ランジュバン)・P5(気体分子/イジング)で実装。
//!
//! `brownian`(ランジュバン方程式・BAOAB積分、docs/15-statistical/03-diffusion-brownian.md)と
//! `kinetic_gas`(剛体球気体MD、docs/15-statistical/02-kinetic-gas.md)を実装。
//! モンテカルロ(イジング)の型・トレイトのスケルトンは Phase A で追加する
//! (docs/22-roadmap/01-phases.md)。

mod brownian;
mod kinetic_gas;
pub use brownian::BrownianParticleSet;
pub use kinetic_gas::{GasSim, BOLTZMANN_CONSTANT};
