//! Orchestrator。設計: docs/00-foundation/04-architecture.md §1.3、
//! docs/20-integration/01-coupling-matrix.md §4。`World`と不可分のため別crateにはしない
//! (設計の明記どおり、`sim-world`内のモジュールとして実装する)。
//!
//! **この増分のスコープ**: 各ドメインの`max_stable_dt()`から決定的にsub-step数を算出する
//! 中核機構(設計§1.3「sub-step数・反復数は状態からの決定的算出のみ、壁時計ベースの
//! 打ち切り禁止」)を実装し、`World::step()`に統合する。Lie-Trotter operator splitting自体
//! (pre/post couplingを挟むパイプライン、docs/20-integration/01-coupling-matrix.md §4の
//! 順序表)は、`Coupling`実装が1つも無い現時点では意味を持たない(挟むものが無い)ため、
//! `Coupling`導入時に合わせて拡張する。現時点で実装済みの全ドメインソルバ(mechanics・
//! thermal・em・astro)は`max_stable_dt()`が全て`f64::INFINITY`(陰的Euler・leapfrog・
//! Boris pusher・sequential impulsesはいずれも設計上無条件安定または適応刻みを未実装)を
//! 返すため、本増分時点では`sub_step_count`は常に1を返す(将来、有限の`max_stable_dt()`を
//! 返すソルバが追加されたときに初めて複数sub-stepが実際に発生する)。

/// フレームdt(Worldの固定dt)を、指定ドメインの`max_stable_dt()`以下の間隔に均等分割
/// するのに必要な最小のsub-step数を決定的に算出する(設計§1.3)。状態(`max_stable_dt`の
/// 値)のみに依存し壁時計を参照しないため、同一入力から同一sub-step数が再現される
/// (決定論、docs/20-integration/02-determinism-replay.md §2)。
///
/// `max_stable_dt`が非有限(`INFINITY`)または非正の場合は1を返す(無条件安定 or
/// 未実装の適応刻み)。
pub fn sub_step_count(frame_dt: f64, domain_max_stable_dt: f64) -> u32 {
    if !domain_max_stable_dt.is_finite() || domain_max_stable_dt <= 0.0 {
        return 1;
    }
    (frame_dt / domain_max_stable_dt).ceil().max(1.0) as u32
}

/// `sub_step_count`から一様なsub-step刻み幅を算出する(フレームdtを均等分割、
/// 合計が厳密にframe_dtに一致する)。
pub fn sub_step_dt(frame_dt: f64, sub_steps: u32) -> f64 {
    frame_dt / sub_steps as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sub_step_count_is_one_for_infinite_max_stable_dt() {
        assert_eq!(sub_step_count(1.0 / 120.0, f64::INFINITY), 1);
    }

    #[test]
    fn sub_step_count_is_one_when_max_stable_dt_exceeds_frame_dt() {
        // ドメインが要求する安定刻みより実際のフレームdtの方が小さい(余裕がある)ケース。
        assert_eq!(sub_step_count(1.0 / 120.0, 1.0 / 60.0), 1);
    }

    #[test]
    fn sub_step_count_divides_evenly_when_frame_dt_is_an_exact_multiple() {
        // frame_dt=1/60, max_stable_dt=1/120 => ちょうど2 sub-step必要。
        assert_eq!(sub_step_count(1.0 / 60.0, 1.0 / 120.0), 2);
    }

    #[test]
    fn sub_step_count_rounds_up_when_not_an_exact_multiple() {
        // frame_dt=0.025, max_stable_dt=0.01 => 0.025/0.01=2.5 -> ceil=3。
        assert_eq!(sub_step_count(0.025, 0.01), 3);
    }

    #[test]
    fn sub_step_dt_sums_exactly_to_frame_dt() {
        let frame_dt = 1.0 / 60.0;
        let n = sub_step_count(frame_dt, 1.0 / 120.0);
        let dt = sub_step_dt(frame_dt, n);
        assert!((dt * n as f64 - frame_dt).abs() < 1e-15);
    }

    #[test]
    fn sub_step_count_treats_non_positive_max_stable_dt_as_unconditionally_stable() {
        assert_eq!(sub_step_count(1.0 / 120.0, 0.0), 1);
        assert_eq!(sub_step_count(1.0 / 120.0, -1.0), 1);
    }
}
