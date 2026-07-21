//! ポアズイユ流(2Dチャネル、両端no-slip壁)。設計: docs/11-fluid/02-eulerian-grid.md §7(F7)。
//!
//! 完全に発達した平行平板間流れは x 方向に一様(u=u(y)のみ、v=0)なので非線形移流項が
//! 恒等的に消え、圧力投影も発散が常に0のため不要になる — 断面方向(y)の1D陰的粘性拡散
//! (`ConductionRod1D`と同型のDirichlet境界+matrix-free PCG)に厳密に帰着する。
//! `GridFluid2D`(周期境界のみ)とは別に、壁面no-slip境界を持つ専用の1D縮約実装とした。

use sim_math::{pcg, Preconditioner};

/// ポアズイユ流の断面速度分布 u(y)。両端(`u[0]`・`u[n-1]`)は壁面no-slip境界(u=0固定)。
pub struct PoiseuilleChannel1D {
    pub u: Vec<f64>,
    pub dy: f64,
    /// 動粘性率 ν=μ/ρ [m²/s]。
    pub kinematic_viscosity: f64,
    /// 駆動力(圧力勾配相当)を質量で割った値 $-\frac{1}{\rho}\frac{dp}{dx}$ [m/s²]。
    pub driving_force_per_mass: f64,
}

impl PoiseuilleChannel1D {
    pub fn new(
        node_count: usize,
        channel_height: f64,
        kinematic_viscosity: f64,
        driving_force_per_mass: f64,
    ) -> Self {
        PoiseuilleChannel1D {
            u: vec![0.0; node_count],
            dy: channel_height / (node_count - 1) as f64,
            kinematic_viscosity,
            driving_force_per_mass,
        }
    }

    /// 陰的Euler(`ConductionRod1D::step`と同じ境界処理)+ 一定駆動力源項:
    /// $\partial u/\partial t = \nu\,\partial^2u/\partial y^2 + f$。
    pub fn step(&mut self, dt: f64) {
        let n = self.u.len();
        if n < 3 {
            return;
        }
        let interior = n - 2;
        let r = self.kinematic_viscosity * dt / (self.dy * self.dy);

        let u_old: Vec<f64> = self.u[1..n - 1].to_vec();

        let apply_a = |x: &[f64], out: &mut [f64]| {
            for i in 0..interior {
                let mut val = (1.0 + 2.0 * r) * x[i];
                if i > 0 {
                    val -= r * x[i - 1];
                }
                if i < interior - 1 {
                    val -= r * x[i + 1];
                }
                out[i] = val;
            }
        };

        let mut b = u_old.clone();
        for bi in b.iter_mut() {
            *bi += dt * self.driving_force_per_mass;
        }
        // 壁面(u=0)からの寄与は0なので、ConductionRod1Dと異なり境界定数項の加算は不要。

        let mut x = u_old;
        let result = pcg(apply_a, &b, &mut x, &Preconditioner::None, 1e-12, 500);
        debug_assert!(
            result.converged,
            "Poiseuille channel PCG did not converge: {result:?}"
        );

        self.u[1..n - 1].copy_from_slice(&x);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn analytic_profile(y: f64, height: f64, nu: f64, force: f64) -> f64 {
        force / (2.0 * nu) * y * (height - y)
    }

    /// F7: ポアズイユ流 — 放物型プロファイルとrel<2%、4解像度水準で検証
    /// (docs/21-verification/01-analytic-tests.md F7)。陰的Euler(無条件安定)により
    /// 拡散の緩和時間スケール$\tau=H^2/\nu$の何倍もの時間を大きなdtで少数ステップで
    /// 進められるため、解像度を上げても実行時間はほぼ変わらない。
    ///
    /// 実装検証中、設計が要求する「2次収束(◆)」を4水準の誤差比から確認しようとしたところ、
    /// 最も粗い解像度(9点)から既に誤差が浮動小数点丸め誤差の水準(約1e-12)に達しており、
    /// 解像度を上げても誤差比が理論値(4倍)にならない(比が1.7程度でばらつく)ことを発見した。
    /// 原因を検討し、中心差分ラプラシアンは2次多項式を厳密に再現する(打ち切り誤差が
    /// 恒等的に0になる)ことに気づいた — 完全発達ポアズイユ流の解析解はyについて厳密な
    /// 2次多項式(放物線)なので、離散化誤差そのものが原理的に存在せず、解像度に依らず
    /// 丸め誤差のみが残る。したがって「解像度を上げると誤差が2次で縮小する」という
    /// 収束次数の傾向は観測できない(打ち切り誤差がそもそも存在しないため比較対象がない)
    /// — これはバグではなく、このテストケースの解析解の性質(厳密な2次関数)による
    /// 数値的に正しい帰結である。収束次数の代わりに、全解像度で誤差が丸め誤差の水準
    /// (1e-8未満、設計目標のrel 2%よりはるかに厳しい)に収まることを確認する。
    #[test]
    fn f7_poiseuille_profile_matches_parabola_at_all_resolution_levels() {
        let height = 1.0;
        let nu = 0.1;
        let force = 1.0;
        let tau = height * height / nu;
        let dt = tau / 10.0;
        let steps = 100;

        let node_counts = [9usize, 17, 33, 65];

        for &node_count in &node_counts {
            let mut channel = PoiseuilleChannel1D::new(node_count, height, nu, force);
            for _ in 0..steps {
                channel.step(dt);
            }

            let mut max_rel_err: f64 = 0.0;
            for i in 1..node_count - 1 {
                let y = i as f64 * channel.dy;
                let analytic = analytic_profile(y, height, nu, force);
                let measured = channel.u[i];
                let rel_err = (measured - analytic).abs() / analytic;
                max_rel_err = max_rel_err.max(rel_err);
            }
            assert!(
                max_rel_err < 1e-8,
                "node_count={node_count} max_rel_err={max_rel_err:e} (expected near machine precision, see test doc comment)"
            );
        }
    }
}
