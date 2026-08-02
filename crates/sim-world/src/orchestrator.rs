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
/// 1フレームあたりのsub-step数の上限(**群3で追加**)。
///
/// **なぜ必要になったか**: 群3で統計ドメイン(`GasSim`)を`World`へ載せたところ、
/// **`World::step()`が返ってこなくなった**。原因は時間スケールの桁違い——分子気体の
/// `max_stable_dt`は「最速粒子が半径ぶん動く時間」で $\sim10^{-13}$ s なのに対し、
/// `World`のフレームdtは 1/120 s。素直に割ると **2×10¹⁰ sub-step** になる
/// (`u32`で飽和して約43億回ループする)。
///
/// これは実装バグではなく**物理的に正しい要求**である——その dt でその気体を
/// 正しく積分するには本当にそれだけのステップが要る。したがって「速く回るように
/// 誤魔化す」のではなく、**上限で打ち切ったうえで、打ち切ったことを申告する**のが
/// 正しい振る舞いになる(設計 docs/00-foundation/04-architecture.md §1.3 は
/// 「壁時計ベースの打ち切り禁止」と定めるが、これは壁時計ではなく**状態から
/// 決定的に決まる固定上限**なので決定論は保たれる)。
///
/// 上限に当たったシーンは「そのフレームdtでは正しく積分できていない」ので、
/// `World::active_approximations()` がバッジとして出す——ユーザーは
/// **フレームdtを下げるか、ドメインを別レジームで回すか**を選べる。
pub const MAX_SUB_STEPS_PER_FRAME: u32 = 1000;

/// `sub_step_count`の、上限に当たったかどうかも返す版(`MAX_SUB_STEPS_PER_FRAME`のdoc参照)。
/// 戻り値は `(sub_step数, 上限で打ち切ったか)`。
pub fn sub_step_count_capped(frame_dt: f64, domain_max_stable_dt: f64) -> (u32, bool) {
    if !domain_max_stable_dt.is_finite() || domain_max_stable_dt <= 0.0 {
        return (1, false);
    }
    let ideal = (frame_dt / domain_max_stable_dt).ceil().max(1.0);
    if ideal > MAX_SUB_STEPS_PER_FRAME as f64 {
        (MAX_SUB_STEPS_PER_FRAME, true)
    } else {
        (ideal as u32, false)
    }
}

/// `sub_step_count`から一様なsub-step刻み幅を算出する(フレームdtを均等分割、
/// 合計が厳密にframe_dtに一致する)。
pub fn sub_step_dt(frame_dt: f64, sub_steps: u32) -> f64 {
    frame_dt / sub_steps as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 上限(`MAX_SUB_STEPS_PER_FRAME`)で打ち切られ、そのことが返り値で分かる。
    /// **これが無いと `World::step()` が返ってこない**——群3で分子気体
    /// ($\Delta t_{max}\sim10^{-13}$ s)を載せたときに実際に固まった。
    #[test]
    fn sub_step_count_is_capped_and_reports_it() {
        // 1/120 s を 1e-13 s 刻みで割ると 8.3e10 sub-step になる。
        let (n, capped) = sub_step_count_capped(1.0 / 120.0, 1.0e-13);
        assert_eq!(n, MAX_SUB_STEPS_PER_FRAME);
        assert!(capped);

        // 上限以下なら打ち切らない(既存の挙動は不変)。
        let (n, capped) = sub_step_count_capped(1.0 / 60.0, 1.0 / 120.0);
        assert_eq!(n, 2);
        assert!(!capped);
    }

    #[test]
    fn sub_step_count_is_one_for_infinite_max_stable_dt() {
        assert_eq!(sub_step_count_capped(1.0 / 120.0, f64::INFINITY).0, 1);
    }

    #[test]
    fn sub_step_count_is_one_when_max_stable_dt_exceeds_frame_dt() {
        // ドメインが要求する安定刻みより実際のフレームdtの方が小さい(余裕がある)ケース。
        assert_eq!(sub_step_count_capped(1.0 / 120.0, 1.0 / 60.0).0, 1);
    }

    #[test]
    fn sub_step_count_divides_evenly_when_frame_dt_is_an_exact_multiple() {
        // frame_dt=1/60, max_stable_dt=1/120 => ちょうど2 sub-step必要。
        assert_eq!(sub_step_count_capped(1.0 / 60.0, 1.0 / 120.0).0, 2);
    }

    #[test]
    fn sub_step_count_rounds_up_when_not_an_exact_multiple() {
        // frame_dt=0.025, max_stable_dt=0.01 => 0.025/0.01=2.5 -> ceil=3。
        assert_eq!(sub_step_count_capped(0.025, 0.01).0, 3);
    }

    #[test]
    fn sub_step_dt_sums_exactly_to_frame_dt() {
        let frame_dt = 1.0 / 60.0;
        let n = sub_step_count_capped(frame_dt, 1.0 / 120.0).0;
        let dt = sub_step_dt(frame_dt, n);
        assert!((dt * n as f64 - frame_dt).abs() < 1e-15);
    }

    #[test]
    fn sub_step_count_treats_non_positive_max_stable_dt_as_unconditionally_stable() {
        assert_eq!(sub_step_count_capped(1.0 / 120.0, 0.0).0, 1);
        assert_eq!(sub_step_count_capped(1.0 / 120.0, -1.0).0, 1);
    }
}
