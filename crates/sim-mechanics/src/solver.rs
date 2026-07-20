//! 力学ソルバ。設計: docs/10-mechanics/01-rigid-body.md §4/§9、
//!       docs/10-mechanics/02-collision-detection.md、docs/10-mechanics/03-contact-solver.md。
//!
//! P1/P2 スコープ: 重力の適用・semi-implicit Euler 積分・総当たり衝突検出・
//! sequential impulses 接触ソルバ(反発+Baumgarte+箱近似クーロン摩擦+warm starting)。
//! 最小CCD・split impulse・スリープは別増分で追加する
//! (docs/22-roadmap/01-phases.md P1/P2 ウェーブ)。

use crate::body::{BodyType, DragModel, RigidBodySet};
use crate::shape::Shape;
use crate::{collision, contact, RigidBodyDesc};
use sim_core::{EnergyBreakdown, MaterialDb, Solver, SolverContext, StateHasher};
use sim_fluid::{Atmosphere, StaticWaterRegion};

pub struct MechanicsSolver {
    pub bodies: RigidBodySet,
    /// 重力加速度(下向き、m/s^2)。既定 9.80665(docs/00-foundation/03-units-conventions.md)。
    pub gravity: f64,
    /// 反発を無視する接近速度の閾値(設計 §4.3・§9、既定 0.5 m/s)。ジッタ防止用の
    /// ヒューリスティクスであり、理想化された弾性衝突の検証(M5 等)では 0 に下げてよい。
    pub restitution_velocity_threshold: f64,
    /// 抗力の評価に使う周囲媒質(設計 docs/11-fluid/05-aero-hydrodynamics.md §3)。
    /// `None`(既定)は真空相当(抗力なし)。P1 は単一の一様媒質のみ(局所媒質・格子流体
    /// との排他は Phase 3、docs/11-fluid/05 §6)。
    pub atmosphere: Option<Atmosphere>,
    /// 浮力の評価に使う静的水域(設計 docs/11-fluid/04-free-surface-buoyancy.md §3)。
    /// `None`(既定)は水域なし。P1 は直立姿勢の直方体のみ対応(`sim_fluid::buoyancy` 冒頭注記)。
    pub water: Option<StaticWaterRegion>,
    /// Warm starting 用の永続キャッシュ(設計 docs/10-mechanics/03-contact-solver.md §4.4)。
    contact_cache: contact::WarmStartCache,
    /// Box-Box 軸選択ヒステリシス用キャッシュ(設計 docs/10-mechanics/02-collision-detection.md §4.4)。
    axis_cache: collision::AxisCache,
}

impl MechanicsSolver {
    pub fn new(gravity: f64) -> MechanicsSolver {
        MechanicsSolver {
            bodies: RigidBodySet::new(),
            gravity,
            restitution_velocity_threshold: contact::DEFAULT_RESTITUTION_VELOCITY_THRESHOLD,
            atmosphere: None,
            water: None,
            contact_cache: contact::WarmStartCache::new(),
            axis_cache: collision::AxisCache::new(),
        }
    }

    pub fn create_body(&mut self, desc: RigidBodyDesc, materials: &MaterialDb) -> usize {
        self.bodies.create_body(desc, materials)
    }

    /// 設計 §4 パイプラインの `apply_forces`。P1 スコープ: 重力 + 球の抗力
    /// (docs/11-fluid/05-aero-hydrodynamics.md §2.1)+ 直立直方体の浮力
    /// (docs/11-fluid/04-free-surface-buoyancy.md §2.1)。結合力は後続増分。
    fn apply_forces(&mut self) {
        let n = self.bodies.len();
        for i in 0..n {
            if self.bodies.body_type[i] == BodyType::Dynamic {
                let mass = self.bodies.mass(i);
                self.bodies.force_accum[i].y -= mass * self.gravity;

                if let (Some(atm), DragModel::Sphere { radius }) =
                    (&self.atmosphere, self.bodies.drag[i])
                {
                    self.bodies.force_accum[i] = self.bodies.force_accum[i]
                        + sim_fluid::drag_force_sphere(radius, atm, self.bodies.linear_velocity[i]);
                }

                if let (Some(water), Shape::Box { half_extents }) =
                    (&self.water, self.bodies.shape_of(i))
                {
                    let (v_sub, _c_buoy) = sim_fluid::submerged_box_axis_aligned(
                        self.bodies.position[i],
                        *half_extents,
                        water.water_level,
                    );
                    // 浮心は直立対称箱では常に body 中心と同じ x,z を持ち、浮力は鉛直成分
                    // のみなのでトルクは厳密に0(r×F、r・Fが共にy軸方向で外積0)。
                    // トルク適用は不要(_c_buoy は式の対称性の記録として保持)。
                    if v_sub > 0.0 {
                        self.bodies.force_accum[i] = self.bodies.force_accum[i]
                            + sim_fluid::buoyancy_force(v_sub, water.density, self.gravity);
                    }
                }
            }
        }
    }

    /// `v += (F/m)dt`、`ω += I_w⁻¹(τ − ω×I_wω)dt`(ジャイロ項は既定で陽的、設計 §4/§9)。
    fn integrate_velocities(&mut self, dt: f64) {
        let n = self.bodies.len();
        for i in 0..n {
            if self.bodies.body_type[i] != BodyType::Dynamic {
                continue;
            }
            let accel = self.bodies.force_accum[i].scale(self.bodies.inv_mass[i]);
            self.bodies.linear_velocity[i] =
                self.bodies.linear_velocity[i].addcarry_scaled(accel, dt);

            let inv_iw = self.bodies.inv_inertia_world[i];
            if let Some(iw) = inv_iw.inverse() {
                let omega = self.bodies.angular_velocity[i];
                let gyro = omega.cross(iw.mul_vec(omega));
                let ang_accel = inv_iw.mul_vec(self.bodies.torque_accum[i] - gyro);
                self.bodies.angular_velocity[i] = omega.addcarry_scaled(ang_accel, dt);
            }
        }
    }

    /// `x += v dt`、`q = normalize(q + dt/2 * ω_quat ⊗ q)`(設計 §9)。
    /// Dynamic/Kinematic の両方が対象(Kinematic はスクリプトで速度が指定される)。
    fn integrate_positions(&mut self, dt: f64) {
        let n = self.bodies.len();
        for i in 0..n {
            if self.bodies.body_type[i] == BodyType::Static {
                continue;
            }
            self.bodies.position[i] =
                self.bodies.position[i].addcarry_scaled(self.bodies.linear_velocity[i], dt);
            self.bodies.rotation[i] = self.bodies.rotation[i]
                .integrate_angular_velocity(self.bodies.angular_velocity[i], dt);
        }
    }

    /// ワールド慣性の相似変換キャッシュ更新 + アキュムレータのクリア(設計 §4 末尾)。
    fn update_inertia_and_clear_accum(&mut self) {
        let n = self.bodies.len();
        for i in 0..n {
            self.bodies.inv_inertia_world[i] =
                self.bodies.inv_inertia_local[i].similarity(self.bodies.rotation[i].to_mat3());
            self.bodies.force_accum[i] = sim_math::Vec3::ZERO;
            self.bodies.torque_accum[i] = sim_math::Vec3::ZERO;
        }
    }
}

impl Solver for MechanicsSolver {
    /// sequential impulses は固定 dt 前提の速度レベル解法で明示的な CFL 条件を持たない
    /// (Box2D 系と同様)。拘束(ジョイント)導入時に硬い系の刻み制約を追加検討する。
    fn max_stable_dt(&self) -> f64 {
        f64::INFINITY
    }

    fn step(&mut self, dt: f64, ctx: &mut SolverContext) {
        self.apply_forces();
        self.integrate_velocities(dt);
        let manifolds = collision::detect(&self.bodies, &mut self.axis_cache);
        contact::resolve(
            &manifolds,
            &mut self.bodies,
            ctx.materials,
            dt,
            self.restitution_velocity_threshold,
            &mut self.contact_cache,
        );
        self.integrate_positions(dt);
        self.update_inertia_and_clear_accum();
    }

    fn state_hash(&self, hasher: &mut StateHasher) {
        let n = self.bodies.len();
        hasher.write_u64(n as u64);
        for i in 0..n {
            hasher.write_vec3(self.bodies.position[i]);
            hasher.write_quat(self.bodies.rotation[i]);
            hasher.write_vec3(self.bodies.linear_velocity[i]);
            hasher.write_vec3(self.bodies.angular_velocity[i]);
        }
    }

    /// Dynamic 剛体の運動エネルギー(並進+回転)+ 重力ポテンシャル(基準 y=0)。
    /// Kinematic の運動は外部注入エネルギーとして台帳側(World)が扱うため、ここでは対象外
    /// (docs/00-foundation/04-architecture.md §1.1.2(2))。
    fn total_energy(&self) -> EnergyBreakdown {
        let mut kinetic = 0.0;
        let mut potential = 0.0;
        let n = self.bodies.len();
        for i in 0..n {
            if self.bodies.body_type[i] != BodyType::Dynamic {
                continue;
            }
            let mass = self.bodies.mass(i);
            kinetic += 0.5 * mass * self.bodies.linear_velocity[i].length_sq();
            if let Some(inertia_world) = self.bodies.inv_inertia_world[i].inverse() {
                let omega = self.bodies.angular_velocity[i];
                kinetic += 0.5 * omega.dot(inertia_world.mul_vec(omega));
            }
            potential += mass * self.gravity * self.bodies.position[i].y;
        }
        EnergyBreakdown {
            kinetic,
            potential,
            ..Default::default()
        }
    }
}

/// Phase 0 の `FallingBody`(回転なし・接触なし)相当を、正式な `RigidBodySet` +
/// `MechanicsSolver` 経由で再現できることを確認する M1 相当のテスト。
#[cfg(test)]
mod tests {
    use super::*;
    use crate::shape::Shape;
    use sim_core::{EventQueue, MaterialDb};
    use sim_math::{SimRng, Vec3};

    fn make_ctx<'a>(
        materials: &'a MaterialDb,
        rng: &'a mut SimRng,
        events: &'a mut EventQueue,
    ) -> SolverContext<'a> {
        SolverContext {
            materials,
            rng,
            events,
        }
    }

    /// M1: 自由落下 h=10m の到達時刻 t*=sqrt(2h/g)=1.4278s、相対誤差 0.5% 以内
    /// (docs/21-verification/01-analytic-tests.md M1)。
    #[test]
    fn m1_free_fall_matches_analytic_time_to_ground() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();

        let mut solver = MechanicsSolver::new(9.80665);
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.05 }, steel);
        desc.transform.position = Vec3::new(0.0, 10.0, 0.0);
        let idx = solver.create_body(desc, &materials);

        let dt = 1.0 / 120.0;
        let mut t = 0.0;
        while solver.bodies.position[idx].y > 0.0 {
            let mut ctx = make_ctx(&materials, &mut rng, &mut events);
            solver.step(dt, &mut ctx);
            t += dt;
        }

        let analytic = (2.0 * 10.0 / 9.80665_f64).sqrt();
        assert!(
            (t - analytic).abs() / analytic < 0.005,
            "t={t} analytic={analytic}"
        );
    }

    #[test]
    fn static_body_does_not_move_under_gravity() {
        let materials = MaterialDb::standard();
        let concrete = materials.find_by_name("コンクリート").unwrap();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();

        let mut solver = MechanicsSolver::new(9.80665);
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(1.0, 1.0, 1.0),
            },
            concrete,
        );
        desc.body_type = BodyType::Static;
        let idx = solver.create_body(desc, &materials);

        for _ in 0..120 {
            let mut ctx = make_ctx(&materials, &mut rng, &mut events);
            solver.step(1.0 / 120.0, &mut ctx);
        }
        assert_eq!(solver.bodies.position[idx], Vec3::ZERO);
    }

    #[test]
    fn kinematic_body_moves_at_prescribed_velocity_without_gravity() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut rng = SimRng::new(1, 1);
        let mut events = EventQueue::new();

        let mut solver = MechanicsSolver::new(9.80665);
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.1 }, steel);
        desc.body_type = BodyType::Kinematic;
        desc.linear_velocity = Vec3::new(1.0, 0.0, 0.0);
        let idx = solver.create_body(desc, &materials);

        let dt = 1.0 / 120.0;
        for _ in 0..120 {
            let mut ctx = make_ctx(&materials, &mut rng, &mut events);
            solver.step(dt, &mut ctx);
        }
        assert!((solver.bodies.position[idx].x - 1.0).abs() < 1e-9);
        assert!(
            (solver.bodies.position[idx].y - 0.0).abs() < 1e-12,
            "gravity must not affect kinematic bodies"
        );
    }

    /// 決定論: 同一初期条件を2回実行 → state_hash が一致する。
    #[test]
    fn determinism_same_scenario_twice_matches_hash() {
        let run = || {
            let materials = MaterialDb::standard();
            let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
            let mut rng = SimRng::new(7, 7);
            let mut events = EventQueue::new();
            let mut solver = MechanicsSolver::new(9.80665);
            let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.2 }, steel);
            desc.transform.position = Vec3::new(0.0, 5.0, 0.0);
            solver.create_body(desc, &materials);
            for _ in 0..300 {
                let mut ctx = make_ctx(&materials, &mut rng, &mut events);
                solver.step(1.0 / 120.0, &mut ctx);
            }
            let mut hasher = StateHasher::new();
            solver.state_hash(&mut hasher);
            hasher.finish()
        };
        assert_eq!(run(), run());
    }
}
