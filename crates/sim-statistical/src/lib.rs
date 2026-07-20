//! docs/15-statistical/ (ミクロマクロ橋渡し・気体分子運動・拡散/ブラウン運動・モンテカルロ)。P4(ランジュバン)・P5(気体分子/イジング)で実装。
//!
//! `brownian`(ランジュバン方程式・BAOAB積分、docs/15-statistical/03-diffusion-brownian.md)・
//! `kinetic_gas`(剛体球気体MD、docs/15-statistical/02-kinetic-gas.md)・
//! `ising`(2Dイジング模型・メトロポリス・Wolffクラスタ法、docs/15-statistical/04-monte-carlo.md)
//! を実装。

mod brownian;
mod ising;
mod kinetic_gas;
pub use brownian::BrownianParticleSet;
pub use ising::IsingSim;
pub use kinetic_gas::{GasSim, BOLTZMANN_CONSTANT};
