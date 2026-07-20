//! エネルギー台帳。設計: docs/00-foundation/04-architecture.md §1.1.2(2)、
//! docs/21-verification/02-conservation-laws.md §2。
//!
//! 台帳の役割は「不明なエネルギー漏れの常時検出器」ではなく、残差の**トレンド監視指標**
//! (単調増大・階段状ジャンプ等のパターン検知、シーンあたりの累積残差の緩い上限)。
//! バグ検出の役割はドメイン別の保存則テスト(docs/21-verification/02-conservation-laws.md)が担う。

/// residual(t) = |E_total(t) - E_total(0) - W_injected(t)| / max(E_scale, |E_total(0)|)
/// (docs/21-verification/02-conservation-laws.md §2)。
pub struct EnergyLedger {
    initial_energy: f64,
    injected_work: f64,
    residual_history: Vec<f64>,
}

impl EnergyLedger {
    pub fn new(initial_energy: f64) -> EnergyLedger {
        EnergyLedger {
            initial_energy,
            injected_work: 0.0,
            residual_history: Vec::new(),
        }
    }

    /// 外部注入仕事(モーター・ユーザー操作等)を加算する。符号つき。
    /// P1 時点では呼び出し元(motor/user操作)が未実装のため常に 0。
    pub fn add_injected_work(&mut self, work: f64) {
        self.injected_work += work;
    }

    /// 1 step 分の記帳。`energy_scale` はシーンの代表エネルギー(ゼロ初期エネルギー対策の下限、
    /// 設計 §2 の E_scale)。residual を履歴に積み、その値を返す。
    pub fn record(&mut self, current_total_energy: f64, energy_scale: f64) -> f64 {
        let scale = energy_scale
            .max(self.initial_energy.abs())
            .max(f64::EPSILON);
        let residual =
            (current_total_energy - self.initial_energy - self.injected_work).abs() / scale;
        self.residual_history.push(residual);
        residual
    }

    pub fn latest_residual(&self) -> f64 {
        self.residual_history.last().copied().unwrap_or(0.0)
    }

    pub fn residual_history(&self) -> &[f64] {
        &self.residual_history
    }

    /// パターン検知の一例(設計 §1.1.2(2)「単調増大・階段状ジャンプ等のパターン検知」):
    /// 直近 `window` 個の residual が単調非減少なら true。
    pub fn is_monotonically_nondecreasing(&self, window: usize) -> bool {
        let n = self.residual_history.len();
        if window < 2 || n < window {
            return false;
        }
        self.residual_history[n - window..]
            .windows(2)
            .all(|w| w[1] >= w[0])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn residual_zero_when_energy_conserved() {
        let mut ledger = EnergyLedger::new(100.0);
        let r = ledger.record(100.0, 1.0);
        assert_eq!(r, 0.0);
    }

    #[test]
    fn residual_matches_formula_with_scale_floor() {
        let mut ledger = EnergyLedger::new(0.0);
        // E_total(0)=0 なので scale は energy_scale の下限(=2.0)が使われる。
        let r = ledger.record(1.0, 2.0);
        assert!((r - 0.5).abs() < 1e-12, "r={r}");
    }

    #[test]
    fn injected_work_offsets_residual() {
        let mut ledger = EnergyLedger::new(10.0);
        ledger.add_injected_work(5.0);
        // E_total(0)+W_injected = 15、current=15 なので残差ゼロ。
        let r = ledger.record(15.0, 1.0);
        assert_eq!(r, 0.0);
    }

    #[test]
    fn monotonic_trend_detected() {
        let mut ledger = EnergyLedger::new(0.0);
        for e in [0.0, 1.0, 2.0, 3.0] {
            ledger.record(e, 1.0);
        }
        assert!(ledger.is_monotonically_nondecreasing(4));
    }

    #[test]
    fn non_monotonic_trend_not_flagged() {
        let mut ledger = EnergyLedger::new(0.0);
        for e in [0.0, 3.0, 1.0, 2.0] {
            ledger.record(e, 1.0);
        }
        assert!(!ledger.is_monotonically_nondecreasing(4));
    }
}
