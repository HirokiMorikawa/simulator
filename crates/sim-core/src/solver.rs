//! ドメインソルバの共通契約。設計: docs/00-foundation/04-architecture.md §1.2。

use crate::{MaterialDb, StateHasher};
use sim_math::SimRng;

/// 物理ドメイン(力学・流体・熱・電磁気・量子・統計・天体)の識別子。
/// レンダリングは物理から分離された別系統のため含まない
/// (docs/00-foundation/01-vision.md §5.3)。
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Hash)]
pub enum DomainId {
    Mechanics,
    Fluid,
    Thermal,
    Electromagnetism,
    Quantum,
    Statistical,
    Astro,
}

/// 保存則検算用のエネルギー内訳。設計 §1.2「このソルバが保持する全エネルギー」。
/// 形態は docs/00-foundation/04-architecture.md §1.1.2(2) が列挙する
/// 「運動・ポテンシャル・弾性・熱・電磁場・化学」に対応する。
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EnergyBreakdown {
    pub kinetic: f64,
    pub potential: f64,
    pub elastic: f64,
    pub thermal: f64,
    pub electromagnetic: f64,
    pub chemical: f64,
}

impl EnergyBreakdown {
    pub fn total(&self) -> f64 {
        self.kinetic
            + self.potential
            + self.elastic
            + self.thermal
            + self.electromagnetic
            + self.chemical
    }
}

impl std::ops::Add for EnergyBreakdown {
    type Output = EnergyBreakdown;
    fn add(self, rhs: EnergyBreakdown) -> EnergyBreakdown {
        EnergyBreakdown {
            kinetic: self.kinetic + rhs.kinetic,
            potential: self.potential + rhs.potential,
            elastic: self.elastic + rhs.elastic,
            thermal: self.thermal + rhs.thermal,
            electromagnetic: self.electromagnetic + rhs.electromagnetic,
            chemical: self.chemical + rhs.chemical,
        }
    }
}

/// ステップ内に発生した通知イベント。設計 §3(押し込み型)・
/// docs/20-integration/04-world-api.md §2.1 の一覧を型として起こす。
/// 各バリアントの詳細ペイロードはドメイン実装時(P1–P5)に拡充する。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EventKind {
    ContactStarted,
    ContactEnded,
    JointBroken,
    PhaseChanged,
    Discharge,
    FuseBlown,
    SolverDiverged,
}

/// 因果順序の全順序化に使う発生源 ID。世代なしの単純な整数(発行元ドメインが採番)。
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Hash)]
pub struct SourceId(pub u64);

/// イベントの最小形。設計 §1.1.2(5): (step, source_id, kind) の辞書式全順序でソートされる。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Event {
    pub step: u64,
    pub source: SourceId,
    pub kind: EventKind,
}

fn event_order_key(e: &Event) -> (u64, u64, u8) {
    // EventKind の判別値。全順序を安定させるため明示的に固定する
    // (derive(PartialOrd) はバリアント順依存で暗黙的になるため避ける)。
    let kind_rank = match e.kind {
        EventKind::ContactStarted => 0,
        EventKind::ContactEnded => 1,
        EventKind::JointBroken => 2,
        EventKind::PhaseChanged => 3,
        EventKind::Discharge => 4,
        EventKind::FuseBlown => 5,
        EventKind::SolverDiverged => 6,
    };
    (e.step, e.source.0, kind_rank)
}

/// イベントの一時保管。ステップ末尾でまとめて全順序化して配送する
/// (docs/00-foundation/04-architecture.md §3「イベントは push 型」)。
#[derive(Default, Clone)]
pub struct EventQueue {
    pending: Vec<Event>,
}

impl EventQueue {
    pub fn new() -> EventQueue {
        EventQueue {
            pending: Vec::new(),
        }
    }

    pub fn push(&mut self, event: Event) {
        self.pending.push(event);
    }

    /// (step, source_id, kind) の辞書式全順序でソートして排出する。
    pub fn drain_sorted(&mut self) -> Vec<Event> {
        let mut events = std::mem::take(&mut self.pending);
        events.sort_by_key(event_order_key);
        events
    }
}

/// `Solver::step` に渡される共有コンテキスト。設計 §1.2。
pub struct SolverContext<'a> {
    pub materials: &'a MaterialDb,
    pub rng: &'a mut SimRng,
    pub events: &'a mut EventQueue,
}

/// ソルバが自己申告する近似・縮約(**群1で追加**)。
///
/// **なぜ必要だったか**: `World::active_approximations()`は
/// 「どのドメインが有効か」から**Worldの側で推測して**バッジ文字列を組み立てていた。
/// これは①ソルバ側で近似を変えてもWorldを直さないと表示が古いまま
/// ②同じドメインでも設定によって効いている近似が違う(例: 格子流体の粘性が0なら
/// 陽的粘性拡散をスキップする)ことを表現できない、という2つの問題がある。
/// 設計 docs/23-frontend/01-editor.md §1.3 の `ApproximationBadge` は
/// 「近似の名前・**出典**・**オフ可否**」を要求しており、出典もオフ可否も
/// ソルバ自身しか知らない。**各ソルバが自分の近似を申告する**形へ移す。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Approximation {
    /// 近似の短い名前(バッジに出す)。
    pub name: &'static str,
    /// なぜその近似を採ったか(ツールチップに出す)。
    pub reason: &'static str,
    /// 根拠となる設計文書のパス(設計§1.3「出典」)。
    pub doc: &'static str,
    /// 実行時にオフにできるか(設計§1.3「オフ可否」)。
    /// **現状は全て`false`**——オフにする機構(近似を切り替えて再計算する経路)は
    /// まだ無い。ここで`false`と申告しておけば、UIは「オフにできます」という
    /// 嘘のトグルを出さずに済む。
    pub can_disable: bool,
}

/// すべてのドメインソルバが実装する共通トレイト。設計 §1.2。
pub trait Solver {
    /// このソルバが安定に積分できる最大タイムステップ(状態依存でよい)。
    fn max_stable_dt(&self) -> f64;

    /// dt だけ状態を進める。dt <= max_stable_dt() が保証されて呼ばれる。
    fn step(&mut self, dt: f64, ctx: &mut SolverContext);

    /// 決定論検証・リプレイ照合用の状態ハッシュ。
    fn state_hash(&self, hasher: &mut StateHasher);

    /// このソルバが保持する全エネルギー(保存則の全体検算に使う)。
    fn total_energy(&self) -> EnergyBreakdown;

    /// このソルバが今の設定で使っている近似・縮約(**群1で追加**、
    /// `Approximation`のdoc参照)。既定は空——近似を持たない(あるいはまだ
    /// 申告していない)ソルバは何も出さない。
    fn approximations(&self) -> Vec<Approximation> {
        Vec::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn event_queue_drains_in_step_source_kind_order() {
        let mut q = EventQueue::new();
        q.push(Event {
            step: 2,
            source: SourceId(5),
            kind: EventKind::ContactStarted,
        });
        q.push(Event {
            step: 1,
            source: SourceId(9),
            kind: EventKind::JointBroken,
        });
        q.push(Event {
            step: 1,
            source: SourceId(1),
            kind: EventKind::ContactEnded,
        });
        q.push(Event {
            step: 1,
            source: SourceId(1),
            kind: EventKind::ContactStarted,
        });

        let drained = q.drain_sorted();
        let keys: Vec<(u64, u64, u8)> = drained.iter().map(event_order_key).collect();
        let mut sorted_keys = keys.clone();
        sorted_keys.sort();
        assert_eq!(keys, sorted_keys);
        assert_eq!(drained[0].step, 1);
        assert_eq!(drained[0].source, SourceId(1));
    }

    #[test]
    fn event_queue_is_empty_after_drain() {
        let mut q = EventQueue::new();
        q.push(Event {
            step: 0,
            source: SourceId(0),
            kind: EventKind::SolverDiverged,
        });
        let _ = q.drain_sorted();
        assert!(q.drain_sorted().is_empty());
    }

    #[test]
    fn energy_breakdown_total_sums_all_forms() {
        let e = EnergyBreakdown {
            kinetic: 1.0,
            potential: 2.0,
            elastic: 3.0,
            thermal: 4.0,
            electromagnetic: 5.0,
            chemical: 6.0,
        };
        assert_eq!(e.total(), 21.0);
    }
}
