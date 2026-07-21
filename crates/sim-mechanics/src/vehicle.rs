//! 車両(簡易Pacejkaタイヤモデル)。設計: docs/20-integration/03-entity-layer.md §4.1・§7・§9。
//!
//! P4 スコープの縮約実装: フルの `WheelJoint`(サスペンション用Sliderジョイント + 駆動用
//! 汎用ヒンジモーター + 操舵ヒンジ)は未実装(汎用ジョイントアーキテクチャの拡張が必要)
//! なため、車両そのものの剛体シミュレーションは行わず、設計§9のPacejka簡易Magic Formula
//! ($F=D\sin(C\arctan(Bs))$、$s$はスリップ比/スリップ角)を単独の関数として実装し、
//! 設計§7が要求する2つの受け入れ基準(制動距離・定常円旋回)を単純なスカラーODE積分で
//! 直接検証する。

/// Pacejka簡易係数。$D=\mu N$(法線荷重×摩擦係数)。設計§9既定値: 乾燥時 B=10, C=1.9。
#[derive(Clone, Copy, Debug)]
pub struct PacejkaParams {
    pub b: f64,
    pub c: f64,
    pub d: f64,
}

impl PacejkaParams {
    /// 乾燥路面の既定係数(設計§9)。`normal_load`はタイヤ(または車両全体)が受ける法線荷重。
    pub fn dry_default(mu: f64, normal_load: f64) -> PacejkaParams {
        PacejkaParams {
            b: 10.0,
            c: 1.9,
            d: mu * normal_load,
        }
    }
}

/// 簡易Pacejka Magic Formula: $F=D\sin(C\arctan(Bs))$(設計§4.1・§9)。
pub fn pacejka_force(slip: f64, params: &PacejkaParams) -> f64 {
    params.d * (params.c * (params.b * slip).atan()).sin()
}

/// この簡易式(E項なし)が力の最大値を取るスリップ($C\arctan(Bs)=\pi/2$を解いた閉形式)。
/// $C>1$のとき存在し、ここで$F=D$(ピーク摩擦力)に一致する。
pub fn pacejka_peak_slip(params: &PacejkaParams) -> f64 {
    (std::f64::consts::FRAC_PI_2 / params.c).tan() / params.b
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pacejka_force_reaches_d_exactly_at_the_peak_slip() {
        let params = PacejkaParams::dry_default(0.8, 1500.0 * 9.80665);
        let s_peak = pacejka_peak_slip(&params);
        let f_peak = pacejka_force(s_peak, &params);
        assert!(
            (f_peak - params.d).abs() / params.d < 1e-9,
            "expected peak force to equal D, got {f_peak} vs D={}",
            params.d
        );
    }

    /// 制動距離(設計§7): $v^2/(2\mu g)$ ± 10%(タイヤモデル簡易化の範囲)。
    /// 簡易化: 理想的なABS(スリップをPacejkaのピーク値に保持し続ける制御)を仮定し、
    /// 減速中は常にタイヤ縦力がピーク摩擦力$D=\mu m g$になるとする(実際のロックアップ
    /// ブレーキより短い、理論上最良の制動距離という位置づけ、設計が認める簡易化)。
    /// この一定の力を単純なEuler積分で速度・距離に反映し、解析解と比較する。
    #[test]
    fn braking_distance_matches_v_squared_over_two_mu_g() {
        let mu = 0.8;
        let g = 9.80665;
        let mass = 1500.0;
        let params = PacejkaParams::dry_default(mu, mass * g);
        let s_peak = pacejka_peak_slip(&params);
        // 制動なのでスリップは負(タイヤが車体より遅く回る)。
        let f_brake = pacejka_force(-s_peak, &params);
        assert!(f_brake < 0.0, "braking force should decelerate the vehicle");

        let v0 = 30.0; // m/s
        let dt = 1e-4;
        let mut v = v0;
        let mut distance = 0.0;
        let mut steps = 0u64;
        while v > 0.0 {
            let a = f_brake / mass;
            let v_next = (v + a * dt).max(0.0);
            distance += 0.5 * (v + v_next) * dt;
            v = v_next;
            steps += 1;
            assert!(steps < 10_000_000, "braking simulation did not converge");
        }

        let expected = v0 * v0 / (2.0 * mu * g);
        let rel_err = (distance - expected).abs() / expected;
        assert!(
            rel_err < 0.1,
            "distance={distance:.4} expected={expected:.4} rel_err={rel_err:.4}"
        );
    }

    /// 定常円旋回(設計§7): 横加速度と遠心力の釣り合い。半径Rで速度vの定常円旋回に
    /// 必要な向心力 $mv^2/R$ をタイヤ横力(Pacejka、スリップ角$\alpha$)が正確に供給する
    /// スリップ角を求め(単調増加区間での二分探索)、その力で1周分の等速円運動を
    /// 実際に積分してみて、軌道が半径Rの円を保つこと(半径の相対誤差)を確認する。
    #[test]
    fn steady_cornering_lateral_force_balances_centripetal_acceleration() {
        let mu = 0.9;
        let g = 9.80665;
        let mass = 1500.0;
        let radius = 50.0;
        let speed = 15.0; // m/s
        let params = PacejkaParams::dry_default(mu, mass * g);
        let required_force = mass * speed * speed / radius;
        assert!(
            required_force < params.d,
            "requested cornering exceeds available peak grip"
        );

        // 単調増加区間([0, s_peak])で二分探索し、Pacejka横力=required_forceとなる
        // スリップ角を求める。
        let s_peak = pacejka_peak_slip(&params);
        let mut lo = 0.0;
        let mut hi = s_peak;
        for _ in 0..100 {
            let mid = 0.5 * (lo + hi);
            if pacejka_force(mid, &params) < required_force {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let slip_angle = 0.5 * (lo + hi);
        let solved_force = pacejka_force(slip_angle, &params);
        assert!(
            (solved_force - required_force).abs() / required_force < 1e-6,
            "solved slip angle should reproduce the required lateral force exactly"
        );

        // 実際に1周分、等速円運動として積分(向心力=タイヤ横力、常に中心方向)。
        let period = 2.0 * std::f64::consts::PI * radius / speed;
        let dt = period / 20_000.0;
        let mut pos = sim_math::Vec3::new(radius, 0.0, 0.0);
        let mut vel = sim_math::Vec3::new(0.0, speed, 0.0);
        let steps = 20_000;
        let mut max_radius_err: f64 = 0.0;
        for _ in 0..steps {
            let center_dir = pos.scale(-1.0 / pos.length());
            let accel = center_dir.scale(solved_force / mass);
            vel = vel.addcarry_scaled(accel, dt);
            pos = pos.addcarry_scaled(vel, dt);
            let r = pos.length();
            max_radius_err = max_radius_err.max((r - radius).abs() / radius);
        }
        assert!(
            max_radius_err < 0.02,
            "orbit radius should stay close to R over one revolution, max_radius_err={max_radius_err:.4}"
        );
    }
}
