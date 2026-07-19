//! 力学ソルバ。設計: docs/10-mechanics/01-rigid-body.md。
//!
//! Phase 0 は「箱 1 個が重力で落ちる」ことだけを実証する最小実装であり、
//! 正式な `RigidBodySet`(慣性テンソル・トルク・接触・フレームID)は
//! P1(Phase B)で 01-rigid-body.md §3 に沿って構築し直す
//! (docs/22-roadmap/01-phases.md の P1 ウェーブ)。回転・接触・摩擦は
//! 意図的にここでは扱わない。

use sim_math::Vec3;

/// Phase 0 専用の最小剛体。回転なし・接触なし。
pub struct FallingBody {
    pub position: Vec3,
    pub velocity: Vec3,
}

impl FallingBody {
    pub fn new(position: Vec3) -> FallingBody {
        FallingBody {
            position,
            velocity: Vec3::ZERO,
        }
    }

    /// 重力加速度 `gravity`(下向き、m/s^2)の下で semi-implicit Euler で dt 進める。
    /// 積分器の選定は docs/01-math/03-integrators.md(math ウェーブで正式カタログ化)。
    pub fn step(&mut self, gravity: f64, dt: f64) {
        self.velocity.y -= gravity * dt;
        self.position = self.position.addcarry_scaled(self.velocity, dt);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// semi-implicit Euler の厳密漸化式との照合。
    /// v_n = v0 - n*g*dt, y_n = y0 + n*dt*v0 - g*dt^2*n*(n+1)/2
    #[test]
    fn matches_semi_implicit_euler_closed_form() {
        let g = 9.80665;
        let dt = 1.0 / 120.0;
        let y0 = 10.0;
        let mut body = FallingBody::new(Vec3::new(0.0, y0, 0.0));
        let n = 200u64;
        for _ in 0..n {
            body.step(g, dt);
        }
        let nf = n as f64;
        let expected_v = -g * dt * nf;
        let expected_y = y0 - g * dt * dt * nf * (nf + 1.0) / 2.0;
        assert!((body.velocity.y - expected_v).abs() < 1e-9);
        assert!((body.position.y - expected_y).abs() < 1e-6);
    }

    #[test]
    fn body_falls_monotonically() {
        let mut body = FallingBody::new(Vec3::new(0.0, 10.0, 0.0));
        let mut last_y = body.position.y;
        for _ in 0..100 {
            body.step(9.80665, 1.0 / 120.0);
            assert!(body.position.y < last_y);
            last_y = body.position.y;
        }
    }
}
