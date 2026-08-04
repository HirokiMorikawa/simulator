//! 前処理付き共役勾配法(PCG)。設計: docs/01-math/02-fields.md §5。
//!
//! 圧力 Poisson・陰的熱伝導などの SPD・matrix-free 線形システム A x = b を解く。
//! IC(0)(不完全コレスキー)やマルチグリッドは具体的な疎行列パターン(格子ラプラシアンの
//! ステンシル)と粗格子化の規則を要するため、その構造を持つドメイン crate 側で組む。
//! ここでは前処理なし・対角(Jacobi)前処理の2種を直接提供し、それ以外は
//! `Preconditioner::Custom` で外から差し込めるようにする
//! (実例: `sim_fluid::pressure_multigrid` の V サイクル前処理)。

/// 前処理の種類。`Jacobi` は行列 A の対角成分そのもの(逆数は内部で計算)。
pub enum Preconditioner<'a> {
    None,
    Jacobi(&'a [f64]),
    /// 任意の前処理作用素 $z = M^{-1} r$。**呼び出し側が $M$ の対称正定値性を保証する**
    /// (PCG の収束理論はそれを前提にしている)。`Fn` なので内部状態を書き換えたい場合は
    /// `RefCell` 等で包むこと——作業配列を毎回確保しないための逃げ道である。
    Custom(&'a dyn Fn(&[f64], &mut [f64])),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PcgResult {
    pub iterations: usize,
    pub residual_norm: f64,
    pub converged: bool,
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b.iter()).map(|(x, y)| x * y).sum()
}

fn apply_preconditioner(precond: &Preconditioner, r: &[f64], z: &mut [f64]) {
    match precond {
        Preconditioner::None => z.copy_from_slice(r),
        Preconditioner::Jacobi(diag) => {
            for i in 0..r.len() {
                z[i] = r[i] / diag[i];
            }
        }
        Preconditioner::Custom(apply) => apply(r, z),
    }
}

/// A x = b を解く。`apply_a` はステンシル適用(matrix-free)。収束判定は相対残差
/// `||r|| / ||b|| < tol_rel`。決定論: 逐次実行では入力から決定的(設計 §5)。
pub fn pcg(
    apply_a: impl Fn(&[f64], &mut [f64]),
    b: &[f64],
    x: &mut [f64],
    precond: &Preconditioner,
    tol_rel: f64,
    max_iter: usize,
) -> PcgResult {
    let n = b.len();
    let b_norm = dot(b, b).sqrt();
    let b_norm = if b_norm < 1e-300 { 1.0 } else { b_norm };

    let mut r = vec![0.0; n];
    let mut ax = vec![0.0; n];
    apply_a(x, &mut ax);
    for i in 0..n {
        r[i] = b[i] - ax[i];
    }

    let mut residual_norm = dot(&r, &r).sqrt() / b_norm;
    if residual_norm < tol_rel {
        return PcgResult {
            iterations: 0,
            residual_norm,
            converged: true,
        };
    }

    let mut z = vec![0.0; n];
    apply_preconditioner(precond, &r, &mut z);
    let mut p = z.clone();
    let mut rz_old = dot(&r, &z);

    let mut ap = vec![0.0; n];
    let mut iterations = 0;
    for iter in 0..max_iter {
        apply_a(&p, &mut ap);
        let p_ap = dot(&p, &ap);
        if p_ap.abs() < 1e-300 {
            break;
        }
        let alpha = rz_old / p_ap;
        for i in 0..n {
            x[i] += alpha * p[i];
            r[i] -= alpha * ap[i];
        }
        residual_norm = dot(&r, &r).sqrt() / b_norm;
        iterations = iter + 1;
        if residual_norm < tol_rel {
            return PcgResult {
                iterations,
                residual_norm,
                converged: true,
            };
        }
        apply_preconditioner(precond, &r, &mut z);
        let rz_new = dot(&r, &z);
        let beta = rz_new / rz_old;
        for i in 0..n {
            p[i] = z[i] + beta * p[i];
        }
        rz_old = rz_new;
    }
    PcgResult {
        iterations,
        residual_norm,
        converged: false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::SimRng;

    /// 1D 離散ポアソン作用素(同次ディリクレ境界): (A x)_i = 2x_i - x_{i-1} - x_{i+1}。
    fn poisson_1d_apply(n: usize) -> impl Fn(&[f64], &mut [f64]) {
        move |v: &[f64], out: &mut [f64]| {
            for i in 0..n {
                let left = if i == 0 { 0.0 } else { v[i - 1] };
                let right = if i + 1 == n { 0.0 } else { v[i + 1] };
                out[i] = 2.0 * v[i] - left - right;
            }
        }
    }

    /// 設計 §7: 既知解の Poisson 問題(製造解法)で収束することを確認する。
    #[test]
    fn converges_on_manufactured_1d_poisson_solution() {
        let n = 50;
        let apply_a = poisson_1d_apply(n);
        let x_exact: Vec<f64> = (0..n).map(|i| (0.13 * i as f64).sin()).collect();
        let mut b = vec![0.0; n];
        apply_a(&x_exact, &mut b);

        let mut x = vec![0.0; n];
        let result = pcg(apply_a, &b, &mut x, &Preconditioner::None, 1e-10, 500);

        assert!(result.converged, "PCG did not converge: {result:?}");
        for i in 0..n {
            assert!(
                (x[i] - x_exact[i]).abs() < 1e-6,
                "mismatch at {i}: {} vs {}",
                x[i],
                x_exact[i]
            );
        }
    }

    #[test]
    fn jacobi_preconditioner_also_converges() {
        let n = 50;
        let apply_a = poisson_1d_apply(n);
        let diag = vec![2.0; n];
        let x_exact: Vec<f64> = (0..n).map(|i| (0.07 * i as f64).cos()).collect();
        let mut b = vec![0.0; n];
        apply_a(&x_exact, &mut b);

        let mut x = vec![0.0; n];
        let result = pcg(
            apply_a,
            &b,
            &mut x,
            &Preconditioner::Jacobi(&diag),
            1e-10,
            500,
        );

        assert!(result.converged, "PCG did not converge: {result:?}");
        for i in 0..n {
            assert!((x[i] - x_exact[i]).abs() < 1e-6);
        }
    }

    /// `Custom` は「外から差し込んだ $M^{-1}$」であって、それ以上の意味を持たない
    /// ——同じ作用素を `Jacobi` として渡したときと反復ごとに同一の結果になること
    /// (ドメイン crate 側のマルチグリッドを信頼して差し込むための前提)。
    #[test]
    fn a_custom_preconditioner_matches_the_equivalent_builtin() {
        let n = 50;
        let x_exact: Vec<f64> = (0..n).map(|i| (0.07 * i as f64).cos()).collect();
        let mut b = vec![0.0; n];
        poisson_1d_apply(n)(&x_exact, &mut b);
        let diag = vec![2.0; n];

        let mut x_builtin = vec![0.0; n];
        let builtin = pcg(
            poisson_1d_apply(n),
            &b,
            &mut x_builtin,
            &Preconditioner::Jacobi(&diag),
            1e-10,
            500,
        );

        let apply_m_inverse = |r: &[f64], z: &mut [f64]| {
            for i in 0..r.len() {
                z[i] = r[i] / 2.0;
            }
        };
        let mut x_custom = vec![0.0; n];
        let custom = pcg(
            poisson_1d_apply(n),
            &b,
            &mut x_custom,
            &Preconditioner::Custom(&apply_m_inverse),
            1e-10,
            500,
        );

        assert!(custom.converged);
        assert_eq!(custom.iterations, builtin.iterations);
        assert_eq!(x_custom, x_builtin);
    }

    /// 設計 §7: SPD 性の乱数テスト — ランダムな対角優位対称行列でも収束する。
    #[test]
    fn converges_on_random_spd_system() {
        let mut rng = SimRng::new(42, 100);
        let n = 12;
        let mut a = vec![vec![0.0; n]; n];
        // 対称行列の上三角を埋めて下三角へ転写するため、行列インデックスでの直接アクセスが
        // イテレータ版より明快(clippy::needless_range_loop を意図的に抑制)。
        #[allow(clippy::needless_range_loop)]
        for i in 0..n {
            for j in (i + 1)..n {
                let v = rng.range_f64(-1.0, 1.0);
                a[i][j] = v;
                a[j][i] = v;
            }
        }
        // 対角優位にして SPD を保証する。
        for (i, row) in a.iter_mut().enumerate() {
            let row_sum: f64 = row.iter().map(|v| v.abs()).sum();
            row[i] = row_sum + 5.0;
        }
        let apply_a = |v: &[f64], out: &mut [f64]| {
            for (i, out_i) in out.iter_mut().enumerate() {
                *out_i = (0..n).map(|j| a[i][j] * v[j]).sum();
            }
        };

        let x_exact: Vec<f64> = (0..n).map(|_| rng.range_f64(-2.0, 2.0)).collect();
        let mut b = vec![0.0; n];
        apply_a(&x_exact, &mut b);

        let mut x = vec![0.0; n];
        let result = pcg(apply_a, &b, &mut x, &Preconditioner::None, 1e-10, 500);
        assert!(result.converged, "PCG did not converge: {result:?}");
        for i in 0..n {
            assert!((x[i] - x_exact[i]).abs() < 1e-6);
        }
    }
}
