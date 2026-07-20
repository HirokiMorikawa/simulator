//! docs/14-quantum/ (役割と限界・シュレディンガーソルバ・有効モデル)。P5で実装。
//!
//! `schrodinger`(1D TDSEのsplit-step Fourier解法、docs/14-quantum/02-schrodinger-solver.md)を
//! 実装。虚時間発展・2D・吸収境界・検出スクリーンサンプリング・有効モデルの型・トレイトの
//! スケルトンは今後の増分で追加する(docs/22-roadmap/01-phases.md)。

mod schrodinger;
pub use schrodinger::{find_eigenstates, WaveFunction1D};
