//! 空力・水力(集中定数モデル)。設計: docs/11-fluid/05-aero-hydrodynamics.md。
//!
//! 流れを解像せずに剛体へ働く流体力を集中定数で与える。P1 スコープ: 球の抗力
//! (Schiller-Naumann 補正、Re<800 でストークス域へ連続接続、Re>=800 は §9 表の
//! 亜臨界球定数 0.47)+ 一様風。
//!
//! **群5で §2.2(揚力・マグヌス効果)を実装した** — `thin_airfoil_lift_coefficient`・
//! `wing_lift_force`(薄翼理論 $C_L\approx2\pi\alpha$ + 失速)・`magnus_force_sphere`
//! (経験式 $C_M\approx0.2S$)。移行前は「Phase 3–4」として未着手で、そのために
//! `sim-coupling::BuoyancyDrag`が設計名に挙がる揚力を扱えなかった。
//!
//! Box3・Panels(布)・乱流風・§2.3 の回転抗力は引き続き対象外(Phase 3–4)。

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
    // **媒質が無ければ抗力は生じない**。密度 0(真空)では Re=0 となり
    // `Cd = 24/Re` が無限大、`0.5 ρ Cd` が `0 × ∞ = NaN` になって速度へ
    // 流れ込んでいた——「空気の濃さ」を真空にすると落ちる速さが数値として
    // 出てこなくなっていた(利用者役③の観察)。物理としても真空中の抗力は
    // ゼロなので、ここで打ち切るのが正しい(粘性 0 でも Re が無限大になる
    // ので同じ扱いにする)。
    if atm.density <= 0.0
        || atm.viscosity <= 0.0
        || !atm.density.is_finite()
        || !atm.viscosity.is_finite()
    {
        return Vec3::ZERO;
    }
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

/// 失速が始まる迎角 [rad](設計 §2.2「失速前 $|\alpha|<12°$」)。
pub const STALL_ANGLE: f64 = 12.0 * std::f64::consts::PI / 180.0;

/// 揚力が完全に失われる迎角 [rad](**本実装が決めた値**、下記参照)。
pub const FULL_STALL_ANGLE: f64 = 30.0 * std::f64::consts::PI / 180.0;

/// 薄翼理論の揚力係数 $C_L(\alpha)\approx2\pi\alpha$(設計 §2.2、**群5で追加**)。
///
/// 失速の扱いは設計が「クランプ+線形減衰で近似」とだけ書いて数値を与えていないため、
/// **本実装が具体化した**: $|\alpha|\le12°$ は薄翼理論そのまま、$12°<|\alpha|<30°$ は
/// ピーク値 $2\pi\cdot12°$ から $30°$ でゼロになるよう線形に減衰、$|\alpha|\ge30°$ は
/// ゼロ。$30°$ という完全失速角は設計に根拠が無い本実装独自の選択で、実機の失速後
/// 挙動(再付着・ヒステリシス)は再現しない — 「失速すると揚力が急落する」という
/// 定性的挙動だけを与える縮約である。
pub fn thin_airfoil_lift_coefficient(angle_of_attack: f64) -> f64 {
    let a = angle_of_attack.abs();
    let sign = if angle_of_attack < 0.0 { -1.0 } else { 1.0 };
    let peak = 2.0 * std::f64::consts::PI * STALL_ANGLE;
    if a <= STALL_ANGLE {
        sign * 2.0 * std::f64::consts::PI * a
    } else if a < FULL_STALL_ANGLE {
        let t = (FULL_STALL_ANGLE - a) / (FULL_STALL_ANGLE - STALL_ANGLE);
        sign * peak * t
    } else {
        0.0
    }
}

/// 翼の揚力。設計 §2.2: $\mathbf F_L=\frac12\rho C_L(\alpha)A|\mathbf v_{rel}|^2\hat{\mathbf L}$
/// (**群5で追加**)。
///
/// `chord`は翼弦方向(前縁→後縁の逆、迎角0のとき進行方向と一致する単位ベクトル)、
/// `span`は翼スパン方向(単位ベクトル、`chord`と直交)。迎角$\alpha$は
/// $\mathbf v_{rel}$を**スパンに直交する平面へ射影**してから`chord`との成す角として
/// 測る(スパン方向成分=横滑りは揚力に寄与しないという薄翼理論の前提)。
/// $\hat{\mathbf L}$は設計どおり$\mathbf v_{rel}$とスパンの両方に直交する向きに取る。
///
/// 相対速度がゼロ、または`span`と`v_rel`が平行(射影が消える)なら 0 を返す。
pub fn wing_lift_force(
    area: f64,
    chord: Vec3,
    span: Vec3,
    atm: &Atmosphere,
    velocity: Vec3,
) -> Vec3 {
    let v_rel = velocity - atm.wind;
    let speed = v_rel.length();
    if speed < 1e-12 {
        return Vec3::ZERO;
    }
    let span_hat = span.normalize_or_zero();
    // スパンに直交する平面への射影(横滑り成分を落とす)。
    let v_plane = v_rel - span_hat.scale(v_rel.dot(span_hat));
    let v_plane_len = v_plane.length();
    if v_plane_len < 1e-12 {
        return Vec3::ZERO;
    }
    let v_hat = v_plane.scale(1.0 / v_plane_len);
    let chord_hat = chord.normalize_or_zero();
    // 迎角の符号。基準は「機体が +x へ飛び、翼弦を機首上げ方向へ$\theta$傾けたら
    // $\alpha=+\theta$、揚力は上向き」——**相対風は$-\mathbf v_{rel}$なので、
    // 迎角は流れから翼弦へ向かう回転で測る**(逆向きに測ると、水平な翼で上昇飛行
    // (流れが上を向く)したときに揚力が上を向くという非物理な符号になる。
    // 実際には上昇中の翼は上面から風を受けるので下向きの揚力を受ける)。
    let cos_alpha = chord_hat.dot(v_hat).clamp(-1.0, 1.0);
    let sin_alpha = v_hat.cross(chord_hat).dot(span_hat);
    let alpha = sin_alpha.atan2(cos_alpha);
    let cl = thin_airfoil_lift_coefficient(alpha);
    if cl == 0.0 {
        return Vec3::ZERO;
    }
    // 揚力方向: v_rel とスパンの両方に直交(設計 §2.2)。正の迎角で
    // 「流れを下向きに曲げる = 翼は上向きの力を受ける」向きになるよう span × v を取る。
    let lift_dir = span_hat.cross(v_hat);
    let lift_dir_len = lift_dir.length();
    if lift_dir_len < 1e-12 {
        return Vec3::ZERO;
    }
    let magnitude = 0.5 * atm.density * cl * area * speed * speed;
    lift_dir.scale(magnitude / lift_dir_len)
}

/// 回転球のマグヌス力。設計 §2.2: $\mathbf F_M=\frac12\rho C_M A|\mathbf v_{rel}|^2
/// (\hat{\boldsymbol\omega}\times\hat{\mathbf v}_{rel})$、$C_M\approx0.2S$、
/// スピン比 $S=\omega r/|\mathbf v_{rel}|$(**群5で追加**)。
///
/// 設計が「$S<1$ の経験式」と明記しているため、$S>1$ では **$S=1$ にクランプ**して
/// 経験式の外挿を避ける(高スピン域では$C_M$が飽和することが知られており、線形外挿は
/// 過大評価になる)。相対速度または角速度がゼロなら 0。
pub fn magnus_force_sphere(
    radius: f64,
    atm: &Atmosphere,
    velocity: Vec3,
    angular_velocity: Vec3,
) -> Vec3 {
    let v_rel = velocity - atm.wind;
    let speed = v_rel.length();
    let omega = angular_velocity.length();
    if speed < 1e-12 || omega < 1e-12 {
        return Vec3::ZERO;
    }
    let spin_ratio = (omega * radius / speed).min(1.0);
    let cm = 0.2 * spin_ratio;
    let area = std::f64::consts::PI * radius * radius;
    let dir = angular_velocity
        .scale(1.0 / omega)
        .cross(v_rel.scale(1.0 / speed));
    let dir_len = dir.length();
    if dir_len < 1e-12 {
        return Vec3::ZERO; // ω ∥ v_rel(スピン軸が進行方向、マグヌス力は生じない)
    }
    let magnitude = 0.5 * atm.density * cm * area * speed * speed;
    dir.scale(magnitude / dir_len)
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

    /// 真空(密度 0)では抗力はゼロで、NaN を作らない。
    ///
    /// 以前は `Cd = 24/Re` が Re=0 で無限大になり、`0.5 ρ Cd` が `0 × ∞ = NaN`
    /// になっていた。落ちる速さが数値として出てこなくなり、画面には「—」しか
    /// 残らなかった(利用者役③の観察)。
    #[test]
    fn drag_in_a_vacuum_is_zero_and_finite() {
        let vacuum = Atmosphere::still(0.0, 1.81e-5);
        let f = drag_force_sphere(0.05, &vacuum, Vec3::new(0.0, -30.0, 0.0));
        assert_eq!(f, Vec3::ZERO, "真空では抗力は生じない: f={f:?}");
        assert!(f.x.is_finite() && f.y.is_finite() && f.z.is_finite());

        // 粘性 0 でも同じ(Re が無限大になる)。
        let inviscid = Atmosphere::still(1.225, 0.0);
        let g = drag_force_sphere(0.05, &inviscid, Vec3::new(0.0, -30.0, 0.0));
        assert!(g.x.is_finite() && g.y.is_finite() && g.z.is_finite());
    }

    #[test]
    fn drag_opposes_relative_velocity_direction() {
        let atm = Atmosphere::still(1.225, 1.81e-5);
        let f = drag_force_sphere(0.05, &atm, Vec3::new(0.0, -10.0, 0.0));
        assert!(f.y > 0.0, "drag must oppose downward motion: f={f:?}");
    }

    /// 薄翼理論 $C_L=2\pi\alpha$ が失速前で厳密に成り立ち、失速後は設計 §2.2 の
    /// 「クランプ+線形減衰」どおりピークから完全失速角までゼロへ落ちること
    /// (**群5で追加**、`thin_airfoil_lift_coefficient`のdoc参照)。
    #[test]
    fn thin_airfoil_lift_is_linear_before_stall_and_decays_after() {
        let two_pi = 2.0 * std::f64::consts::PI;
        for deg in [0.0, 1.0, 5.0, 11.9] {
            let a = deg * std::f64::consts::PI / 180.0;
            assert!(
                (thin_airfoil_lift_coefficient(a) - two_pi * a).abs() < 1e-12,
                "失速前は薄翼理論そのもの: alpha={deg}deg"
            );
            // 奇関数(負の迎角では下向きの揚力)。
            assert!(
                (thin_airfoil_lift_coefficient(-a) + two_pi * a).abs() < 1e-12,
                "C_L は迎角の奇関数であるべき: alpha=-{deg}deg"
            );
        }
        let peak = two_pi * STALL_ANGLE;
        assert!((thin_airfoil_lift_coefficient(STALL_ANGLE) - peak).abs() < 1e-12);
        // 失速域の中点でちょうど半分。
        let mid = 0.5 * (STALL_ANGLE + FULL_STALL_ANGLE);
        assert!((thin_airfoil_lift_coefficient(mid) - 0.5 * peak).abs() < 1e-12);
        assert_eq!(thin_airfoil_lift_coefficient(FULL_STALL_ANGLE), 0.0);
        assert_eq!(thin_airfoil_lift_coefficient(1.5), 0.0);
    }

    /// 翼の揚力(設計 §2.2)。迎角0では揚力ゼロ、正の迎角では翼弦・スパンの双方に
    /// 直交する向きへ $\frac12\rho C_L A v^2$ ちょうどの大きさで働くこと、
    /// スパン方向の横滑り成分が揚力に寄与しないことを解析的に確認する(**群5**)。
    #[test]
    fn wing_lift_matches_the_design_formula_and_ignores_spanwise_flow() {
        let atm = Atmosphere::still(1.225, 1.81e-5);
        let area = 2.0;
        let chord = Vec3::new(1.0, 0.0, 0.0); // 翼弦は +x
        let span = Vec3::new(0.0, 0.0, 1.0); // スパンは +z
        let speed = 30.0;

        // 迎角0(流れが翼弦とちょうど平行)→ C_L=0 → 揚力ゼロ。
        let f0 = wing_lift_force(area, chord, span, &atm, chord.scale(speed));
        assert!(f0.length() < 1e-12, "迎角0で揚力が出てはいけない: {f0:?}");

        // 迎角5°(流れを x-y 平面内で傾ける)。
        let alpha = 5.0_f64.to_radians();
        let v = Vec3::new(alpha.cos(), alpha.sin(), 0.0).scale(speed);
        let f = wing_lift_force(area, chord, span, &atm, v);
        let expected_magnitude =
            0.5 * atm.density * thin_airfoil_lift_coefficient(alpha).abs() * area * speed * speed;
        assert!(
            (f.length() - expected_magnitude).abs() / expected_magnitude < 1e-12,
            "|F_L|={} expected={expected_magnitude}",
            f.length()
        );
        // 揚力は流れにもスパンにも直交(設計 §2.2)。
        assert!(f.dot(v).abs() / (f.length() * speed) < 1e-12);
        assert!(f.dot(span).abs() / f.length() < 1e-12);

        // スパン方向(+z)の速度成分を足しても、迎角も揚力方向も変わらない
        // (横滑りは薄翼理論では揚力に寄与しない)。大きさは |v_rel|^2 に比例するため
        // 増えるので、**向き**が保たれることを見る。
        let f_yaw = wing_lift_force(area, chord, span, &atm, v + span.scale(10.0));
        let cos = f.dot(f_yaw) / (f.length() * f_yaw.length());
        assert!(
            (cos - 1.0).abs() < 1e-12,
            "横滑り成分は揚力の向きを変えないはず: cos={cos}"
        );
    }

    /// マグヌス力(設計 §2.2)。バックスピンの球が上向きの力を受けること、大きさが
    /// $C_M=0.2S$ の経験式どおりであること、$S>1$ でクランプされること、
    /// スピン軸が進行方向と平行なら力がゼロになることを確認する(**群5**)。
    #[test]
    fn magnus_force_matches_the_empirical_spin_ratio_formula() {
        let atm = Atmosphere::still(1.225, 1.81e-5);
        let radius = 0.037; // テニスボール相当
        let speed = 25.0;
        let v = Vec3::new(speed, 0.0, 0.0); // +x へ飛ぶ
                                            // バックスピン: +x へ進む球が上へ曲がるには ω は +z 向き
                                            // (ω̂ × v̂ = ẑ × x̂ = ŷ)。
        let omega = Vec3::new(0.0, 0.0, 50.0);

        let f = magnus_force_sphere(radius, &atm, v, omega);
        assert!(f.y > 0.0, "バックスピンは上向きの力を生むはず: f={f:?}");
        assert!(f.x.abs() < 1e-12 && f.z.abs() < 1e-12);

        let s = omega.length() * radius / speed;
        assert!(s < 1.0, "この設定は経験式の適用範囲 S<1 にあるはず: S={s}");
        let area = std::f64::consts::PI * radius * radius;
        let expected = 0.5 * atm.density * (0.2 * s) * area * speed * speed;
        assert!(
            (f.length() - expected).abs() / expected < 1e-12,
            "|F_M|={} expected={expected}",
            f.length()
        );

        // S>1 はクランプされる(S=1 の値と一致する)。
        let fast_spin = Vec3::new(0.0, 0.0, 5000.0);
        let f_clamped = magnus_force_sphere(radius, &atm, v, fast_spin);
        let expected_clamped = 0.5 * atm.density * 0.2 * area * speed * speed;
        assert!(
            (f_clamped.length() - expected_clamped).abs() / expected_clamped < 1e-12,
            "S>1 は S=1 にクランプされるはず: |F|={}",
            f_clamped.length()
        );

        // スピン軸 ∥ 進行方向ならマグヌス力は生じない。
        let spiral = magnus_force_sphere(radius, &atm, v, Vec3::new(100.0, 0.0, 0.0));
        assert_eq!(spiral, Vec3::ZERO);
    }
}
