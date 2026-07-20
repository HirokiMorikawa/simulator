//! 空力・水力(集中定数モデル)。設計: docs/11-fluid/05-aero-hydrodynamics.md。
//!
//! 流れを解像せずに剛体へ働く流体力を集中定数で与える。P1 スコープ: 球の抗力
//! (Schiller-Naumann 補正、Re<800 でストークス域へ連続接続、Re>=800 は §9 表の
//! 亜臨界球定数 0.47)+ 一様風。Box3・Panels(布・翼)・乱流風・揚力/マグヌスは Phase 3–4。

use sim_math::Vec3;

/// 剛体を包む流体媒質。設計 §3 の `Atmosphere` 型スケッチに、抗力式(§2.1)の評価に
/// 必要な動粘性係数を P1 の実務上の追加として持たせる(スケッチは省略のみで矛盾はない)。
#[derive(Clone, Copy, Debug)]
pub struct Atmosphere {
    pub density: f64,
    pub viscosity: f64,
    pub wind: Vec3,
}

impl Atmosphere {
    /// 無風・静止媒質(P1 の既定シナリオ)。
    pub fn still(density: f64, viscosity: f64) -> Atmosphere {
        Atmosphere {
            density,
            viscosity,
            wind: Vec3::ZERO,
        }
    }
}

/// 直径基準レイノルズ数。設計 §2.1。
pub fn reynolds_number(diameter: f64, atm: &Atmosphere, speed: f64) -> f64 {
    atm.density * speed * diameter / atm.viscosity
}

/// Schiller-Naumann 補正抗力係数(Re<800)。Re>=800 は §9 表の亜臨界球定数 0.47 に固定。
/// Re→0 で補正項 `0.15 Re^0.687` は無視できるほど小さくなり、Cd≈24/Re に収束する。これは
/// ストークス抵抗 F=6πμrv と代数的に一致する(0.5ρ(24/Re)(πr²)v²、Re=2rρv/μ を代入して
/// F=6πμrv になることを確認できる)。
pub fn drag_coefficient_sphere(re: f64) -> f64 {
    if re < 800.0 {
        (24.0 / re) * (1.0 + 0.15 * re.powf(0.687))
    } else {
        0.47
    }
}

/// 球への抗力。設計 §2.1: F_d = -0.5 ρ Cd A |v_rel| v_rel、v_rel = v - wind。
/// 相対速度がゼロなら 0 を返す(Re=0 での Cd 特異点を回避)。
pub fn drag_force_sphere(radius: f64, atm: &Atmosphere, velocity: Vec3) -> Vec3 {
    let v_rel = velocity - atm.wind;
    let speed = v_rel.length();
    if speed < 1e-12 {
        return Vec3::ZERO;
    }
    let re = reynolds_number(2.0 * radius, atm, speed);
    let cd = drag_coefficient_sphere(re);
    let area = std::f64::consts::PI * radius * radius;
    let magnitude = 0.5 * atm.density * cd * area * speed;
    v_rel.scale(-magnitude)
}

/// 終端速度の解析解。設計 §2.1: mg = 0.5 ρ Cd A v_t^2 (亜臨界域、Cd 一定)。
/// F1(鋼球)のような高 Re 終端速度シナリオの検証に使う。
pub fn terminal_velocity_high_re(mass: f64, gravity: f64, atm: &Atmosphere, radius: f64) -> f64 {
    let area = std::f64::consts::PI * radius * radius;
    let cd = 0.47;
    (2.0 * mass * gravity / (atm.density * cd * area)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn low_re_drag_matches_stokes_formula() {
        let atm = Atmosphere::still(1000.0, 1.0); // 高粘性の仮想媒質で低Reを保証
        let radius = 1e-4;
        let v = Vec3::new(0.0, -1e-6, 0.0);
        let f = drag_force_sphere(radius, &atm, v);
        // ストークス抵抗は運動を妨げる向き: F = -6πμr v(設計 §2.1)。
        let stokes = -6.0 * std::f64::consts::PI * atm.viscosity * radius * v.y;
        assert!(
            (f.y - stokes).abs() / stokes.abs() < 1e-3,
            "f={f:?} stokes={stokes}"
        );
    }

    #[test]
    fn high_re_drag_uses_subcritical_sphere_cd() {
        assert!((drag_coefficient_sphere(1.0e5) - 0.47).abs() < 1e-9);
    }

    #[test]
    fn zero_relative_velocity_gives_zero_force() {
        let atm = Atmosphere::still(1.225, 1.81e-5);
        let f = drag_force_sphere(0.01, &atm, Vec3::ZERO);
        assert_eq!(f, Vec3::ZERO);
    }

    #[test]
    fn wind_only_relative_velocity_gives_zero_force() {
        let mut atm = Atmosphere::still(1.225, 1.81e-5);
        atm.wind = Vec3::new(3.0, 0.0, 0.0);
        let f = drag_force_sphere(0.01, &atm, Vec3::new(3.0, 0.0, 0.0));
        assert_eq!(f, Vec3::ZERO);
    }

    #[test]
    fn drag_opposes_relative_velocity_direction() {
        let atm = Atmosphere::still(1.225, 1.81e-5);
        let f = drag_force_sphere(0.05, &atm, Vec3::new(0.0, -10.0, 0.0));
        assert!(f.y > 0.0, "drag must oppose downward motion: f={f:?}");
    }
}
