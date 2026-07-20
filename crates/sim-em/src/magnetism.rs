//! 静磁場 — 磁気双極子の場・力・トルク。設計: docs/13-electromagnetism/01-electrostatics-magnetostatics.md §2。
//!
//! P4 スコープの最小実装: 磁気双極子の場(閉形式)・トルク(閉形式 $\tau=\mathbf{m}\times\mathbf{B}$)・
//! 力(設計 §2「$\mathbf{F}=\nabla(\mathbf{m}\cdot\mathbf{B})$」)。力は一般の相対配置に対する
//! 閉形式の双極子間力の式を導出・実装する代わりに、ポテンシャル $\mathbf{m}_2\cdot\mathbf{B}_1$ の
//! 中心差分による数値勾配として計算する(閉形式より単純で、任意の配置に対して同じコードで
//! 動作する)。永久磁石の姿勢追従・多体の直接和ループ・鏡像力・強磁性体の非線形磁化は未実装。

use sim_math::Vec3;

/// 真空の透磁率 $\mu_0$ [N/A²](CODATA)。
pub const VACUUM_PERMEABILITY: f64 = 1.25663706212e-6;

/// 磁気双極子。設計 §3 `MagneticDipole`。
#[derive(Clone, Copy, Debug)]
pub struct MagneticDipole {
    pub position: Vec3,
    pub moment: Vec3,
}

/// 磁気双極子の場(設計 §2)。$\mathbf{B}(\mathbf{r})=\frac{\mu_0}{4\pi}\left[\frac{3(\mathbf{m}\cdot\hat r)\hat r-\mathbf{m}}{r^3}\right]$。
pub fn dipole_field(source: &MagneticDipole, at: Vec3) -> Vec3 {
    let r_vec = at - source.position;
    let r = r_vec.length();
    let r_hat = r_vec.normalize_or_zero();
    let bracket = r_hat.scale(3.0 * source.moment.dot(r_hat)) - source.moment;
    bracket.scale(VACUUM_PERMEABILITY / (4.0 * std::f64::consts::PI * r.powi(3)))
}

/// 双極子に働くトルク(設計 §2「$\boldsymbol\tau=\mathbf{m}\times\mathbf{B}$」)。
pub fn dipole_torque(moment: Vec3, field: Vec3) -> Vec3 {
    moment.cross(field)
}

/// 双極子 `moment` が `source` から受ける力(設計 §2「$\mathbf{F}=\nabla(\mathbf{m}\cdot\mathbf{B})$」)。
/// ポテンシャル $\phi(\mathbf{r})=\mathbf{m}\cdot\mathbf{B}_{source}(\mathbf{r})$ の中心差分勾配として
/// 数値的に求める(任意の相対配置に対応する単一の実装で済む — 閉形式の双極子間力の式は
/// 導出せずここでは省略する)。
pub fn dipole_force(source: &MagneticDipole, moment: Vec3, at: Vec3) -> Vec3 {
    let r = (at - source.position).length();
    let h = r * 1.0e-6; // 数値勾配の刻み幅(rに対して十分小さく、桁落ちを避ける水準)
    let potential = |p: Vec3| moment.dot(dipole_field(source, p));
    let grad_x = (potential(at + Vec3::new(h, 0.0, 0.0)) - potential(at - Vec3::new(h, 0.0, 0.0)))
        / (2.0 * h);
    let grad_y = (potential(at + Vec3::new(0.0, h, 0.0)) - potential(at - Vec3::new(0.0, h, 0.0)))
        / (2.0 * h);
    let grad_z = (potential(at + Vec3::new(0.0, 0.0, h)) - potential(at - Vec3::new(0.0, 0.0, h)))
        / (2.0 * h);
    Vec3::new(grad_x, grad_y, grad_z)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 整列した2磁石の吸引力は $F=\frac{3\mu_0 m_1 m_2}{2\pi r^4}$(設計 §7「距離依存r^-4の
    /// 冪フィット」)。同じ向きに軸方向整列した双極子対(N-S同士が向き合う配置)は引力になる
    /// 標準的な結果(教科書の同軸双極子力の式)。対応するE番号は無いため設計の力の定義
    /// (F=∇(m・B))から自前で導出した閉形式と数値勾配実装を突き合わせる形で検証する。
    #[test]
    fn aligned_dipoles_attract_with_inverse_fourth_power_force() {
        let m1 = 1.5;
        let m2 = 0.8;
        let r = 0.05;
        let source = MagneticDipole {
            position: Vec3::ZERO,
            moment: Vec3::new(m1, 0.0, 0.0),
        };
        let target_moment = Vec3::new(m2, 0.0, 0.0);
        let target_pos = Vec3::new(r, 0.0, 0.0);

        let force = dipole_force(&source, target_moment, target_pos);
        let expected_magnitude =
            3.0 * VACUUM_PERMEABILITY * m1 * m2 / (2.0 * std::f64::consts::PI * r.powi(4));

        let rel_err = (force.length() - expected_magnitude).abs() / expected_magnitude;
        assert!(
            rel_err < 1e-4,
            "force={force:?} expected_magnitude={expected_magnitude} rel_err={rel_err}"
        );
        // 引力: target(x=+r)に働く力は -x 方向(sourceへ向かう)。
        assert!(force.x < 0.0, "force={force:?}");
    }

    /// べき指数の確認: 距離を2倍にすると力は 2^-4=1/16 になる(設計 §7「r^-4」)。
    #[test]
    fn dipole_force_scales_as_inverse_fourth_power_of_distance() {
        let source = MagneticDipole {
            position: Vec3::ZERO,
            moment: Vec3::new(1.0, 0.0, 0.0),
        };
        let target_moment = Vec3::new(1.0, 0.0, 0.0);
        let f1 = dipole_force(&source, target_moment, Vec3::new(0.02, 0.0, 0.0)).length();
        let f2 = dipole_force(&source, target_moment, Vec3::new(0.04, 0.0, 0.0)).length();
        let ratio = f1 / f2;
        let rel_err = (ratio - 16.0).abs() / 16.0;
        assert!(rel_err < 1e-3, "ratio={ratio}");
    }

    /// トルクは m×B(設計§2)。直交する場では磁気モーメントを場と平行にしようとする方向
    /// (右手則)。ここでは単純に定義式どおりの外積であることを機械精度で確認する。
    #[test]
    fn dipole_torque_matches_cross_product_definition() {
        let m = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(0.5, -1.0, 0.25);
        let tau = dipole_torque(m, b);
        let expected = m.cross(b);
        assert_eq!(tau, expected);
    }
}
