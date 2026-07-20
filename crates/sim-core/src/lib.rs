//! ID・時間・決定論ハッシュ・材料物性データベースの基盤。
//! 設計: docs/00-foundation/04-architecture.md §1.1.2(4)/§3、
//!       docs/20-integration/02-determinism-replay.md §2/§3、
//!       docs/12-thermal/04-material-thermal-props.md。
//!
//! `Solver`/`Coupling` トレイト・`EventQueue`・`CommandQueue` 等は Phase A で
//! 各ドメインスケルトンと合わせて追加する(docs/00-foundation/04-architecture.md §1.2–1.3)。

mod ledger;
mod material;
mod solver;
pub use ledger::EnergyLedger;
pub use material::{Material, MaterialDb, MaterialId, PairOverride, PhaseChangeProps};
pub use solver::{
    DomainId, EnergyBreakdown, Event, EventKind, EventQueue, Solver, SolverContext, SourceId,
};

/// 世代付きインデックス。削除済み ID へのアクセスは `None`(パニックしない)。
/// 設計: docs/00-foundation/04-architecture.md §3。
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub struct BodyId {
    pub index: u32,
    pub generation: u32,
}

/// 所属フレーム。単一フレームのシーンでは全て `ROOT`。
/// 設計: docs/00-foundation/02-scale-ladder.md §2.2、docs/00-foundation/04-architecture.md §3。
/// フル フレーム階層(floating origin)は Pα(docs/20-integration/05-frame-hierarchy.md)で拡張する。
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Hash)]
pub struct FrameId(pub u32);

impl FrameId {
    pub const ROOT: FrameId = FrameId(0);
}

/// World が唯一持つ時刻。固定 dt・単調増加。壁時計・OS 乱数から独立
/// (docs/00-foundation/04-architecture.md §1.1.2(4)、
///  docs/20-integration/02-determinism-replay.md §2「可変タイムステップ」の禁止)。
#[derive(Clone, Copy, Debug)]
pub struct SimClock {
    dt: f64,
    step_count: u64,
}

impl SimClock {
    pub fn new(dt: f64) -> SimClock {
        SimClock { dt, step_count: 0 }
    }

    pub fn dt(&self) -> f64 {
        self.dt
    }

    pub fn step_count(&self) -> u64 {
        self.step_count
    }

    pub fn time(&self) -> f64 {
        self.step_count as f64 * self.dt
    }

    /// 1 ステップ進める。dt は固定のまま、ステップ数だけを単調増加させる。
    pub fn advance(&mut self) {
        self.step_count += 1;
    }
}

/// FNV-1a 64bit ストリーミングハッシャ。
/// 設計: docs/20-integration/02-determinism-replay.md §3。
pub struct StateHasher {
    state: u64,
}

const FNV_OFFSET_BASIS: u64 = 0xcbf29ce484222325;
const FNV_PRIME: u64 = 0x100000001b3;

impl Default for StateHasher {
    fn default() -> Self {
        StateHasher::new()
    }
}

impl StateHasher {
    pub fn new() -> StateHasher {
        StateHasher {
            state: FNV_OFFSET_BASIS,
        }
    }

    fn write_u8(&mut self, byte: u8) {
        self.state ^= byte as u64;
        self.state = self.state.wrapping_mul(FNV_PRIME);
    }

    pub fn write_bytes(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.write_u8(b);
        }
    }

    pub fn write_u64(&mut self, v: u64) {
        self.write_bytes(&v.to_le_bytes());
    }

    /// f64 は to_bits() でハッシュ。±0.0 は +0.0 に正規化する
    /// (docs/20-integration/02-determinism-replay.md §3)。
    pub fn write_f64(&mut self, v: f64) {
        let normalized = if v == 0.0 { 0.0 } else { v };
        self.write_u64(normalized.to_bits());
    }

    pub fn write_vec3(&mut self, v: sim_math::Vec3) {
        self.write_f64(v.x);
        self.write_f64(v.y);
        self.write_f64(v.z);
    }

    pub fn write_quat(&mut self, q: sim_math::Quat) {
        self.write_f64(q.x);
        self.write_f64(q.y);
        self.write_f64(q.z);
        self.write_f64(q.w);
    }

    pub fn finish(&self) -> u64 {
        self.state
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clock_advances_monotonically_with_fixed_dt() {
        let mut clock = SimClock::new(1.0 / 120.0);
        for i in 1..=10 {
            clock.advance();
            assert_eq!(clock.step_count(), i);
        }
        assert!((clock.time() - 10.0 / 120.0).abs() < 1e-15);
    }

    #[test]
    fn hasher_is_deterministic_for_same_input() {
        let mut a = StateHasher::new();
        a.write_f64(1.5);
        a.write_f64(-2.25);
        let mut b = StateHasher::new();
        b.write_f64(1.5);
        b.write_f64(-2.25);
        assert_eq!(a.finish(), b.finish());
    }

    #[test]
    fn hasher_normalizes_negative_zero() {
        let mut a = StateHasher::new();
        a.write_f64(0.0);
        let mut b = StateHasher::new();
        b.write_f64(-0.0);
        assert_eq!(a.finish(), b.finish());
    }

    #[test]
    fn hasher_differs_for_different_input() {
        let mut a = StateHasher::new();
        a.write_f64(1.0);
        let mut b = StateHasher::new();
        b.write_f64(2.0);
        assert_ne!(a.finish(), b.finish());
    }
}
