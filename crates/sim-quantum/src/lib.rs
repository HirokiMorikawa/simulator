//! docs/14-quantum/ (役割と限界・シュレディンガーソルバ・有効モデル)。P5で実装。
//!
//! `schrodinger`(1D TDSEのsplit-step Fourier解法、docs/14-quantum/02-schrodinger-solver.md)と
//! `schrodinger2d`(2D版、同docs §4/§8「2D(二重スリットの本命)」)を実装。
//! 吸収境界・検出スクリーンの決定論的サンプリング・有効モデルの型・トレイトのスケルトンは
//! 今後の増分で追加する(docs/22-roadmap/01-phases.md)。

mod schrodinger;
mod schrodinger2d;
pub use schrodinger::{find_eigenstates, WaveFunction1D};
pub use schrodinger2d::WaveFunction2D;
