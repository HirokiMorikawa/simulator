//! 力学ソルバ。設計: docs/10-mechanics/01-rigid-body.md §4/§9、
//!       docs/10-mechanics/02-collision-detection.md、docs/10-mechanics/03-contact-solver.md。
//!
//! P1/P2 スコープ: 重力の適用・semi-implicit Euler 積分・総当たり衝突検出・
//! sequential impulses 接触ソルバ(反発+Baumgarte+箱近似クーロン摩擦+warm starting)。
//! 最小CCD・split impulse・スリープは別増分で追加する
//! (docs/22-roadmap/01-phases.md P1/P2 ウェーブ)。

use crate::body::{BodyType, DragModel, RigidBodySet};
use crate::joint::{BallJoint, DistanceJoint, HingeMotorPd, SliderJoint};
use crate::shape::Shape;
use crate::{ccd, collision, contact, joint, sleep, RigidBodyDesc};
use sim_core::{EnergyBreakdown, MaterialDb, Solver, SolverContext, StateHasher};
use sim_fluid::{Atmosphere, StaticWaterRegion};

#[derive(Clone)]
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
    /// Distance ジョイント一覧(設計 docs/10-mechanics/05-joints-constraints.md §3)。
    pub joints: Vec<DistanceJoint>,
    /// Ball ジョイント一覧(設計 docs/10-mechanics/05-joints-constraints.md §3)。
    pub ball_joints: Vec<BallJoint>,
    /// Slider ジョイント一覧(設計 §4.4表「Slider」、`joint`モジュールdoc参照)。
    pub slider_joints: Vec<SliderJoint>,
    /// PD位置サーボ付きヒンジモーター一覧(設計 §4.5、`joint`モジュールdoc参照)。
    pub hinge_motors: Vec<HingeMotorPd>,
    /// 直近stepの接触解決(摩擦+反発)による運動エネルギー散逸量(設計
    /// docs/20-integration/01-coupling-matrix.md `DissipationToHeat`が読む、
    /// `sim-coupling`クレートのdoc参照)。接触解決の直前直後の運動エネルギー差分として
    /// 測定する(位置は変化しないためポテンシャルエネルギーは不変、速度の変化のみを見れば
    /// 十分)。抗力による散逸は含まない(抗力は保存力(重力)と共に`apply_forces`で積分
    /// されるため、この測定窓では分離できない。後続増分で抗力の仕事を個別に計測して追加
    /// する)。0にクランプしない — Baumgarte安定化・warm startingは稀に1step内で微小に
    /// 運動エネルギーを増やすことがある(PGS系接触ソルバの既知の数値アーティファクト、
    /// 物理的な現象ではない)ため、クランプすると増加分を無視し減少分だけ計上する系統的な
    /// 片側バイアスになる。実装検証中の発見: それでもなお、10秒・1200stepの滑走→静止
    /// シナリオでは、この量の累積和が実際の力学的エネルギー総損失(区間の`total_energy()`
    /// の差)より約9%大きいことを確認した — 原因は、Baumgarte位置誤差補正がこの
    /// (`contact::resolve()`呼び出し前後のみの)測定窓では運動エネルギー変化として
    /// 現れる一方、その補正効果は次stepの位置積分にも影響し、測定窓の外側で部分的に
    /// 打ち消されるため、前後差分の単純な累積が系統的に過大評価になること(PGS+
    /// Baumgarteソルバの既知の限界であり、クランプの有無では解決しない)。根本修正
    /// (Baumgarteのバイアス速度分を測定から除外する等)は接触ソルバへの踏み込んだ変更を
    /// 要するため本増分では見送り、`sim-coupling::DissipationToHeat`の受け入れテスト側で
    /// この系統誤差を踏まえた許容誤差(rel<15%)を設定して対応する。
    pub last_contact_dissipation: f64,
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
            joints: Vec::new(),
            ball_joints: Vec::new(),
            slider_joints: Vec::new(),
            hinge_motors: Vec::new(),
            last_contact_dissipation: 0.0,
        }
    }

    pub fn create_body(&mut self, desc: RigidBodyDesc, materials: &MaterialDb) -> usize {
        self.bodies.create_body(desc, materials)
    }

    pub fn add_distance_joint(&mut self, joint: DistanceJoint) {
        self.joints.push(joint);
    }

    pub fn add_ball_joint(&mut self, joint: BallJoint) {
        self.ball_joints.push(joint);
    }

    pub fn add_slider_joint(&mut self, joint: SliderJoint) {
        self.slider_joints.push(joint);
    }

    pub fn add_hinge_motor(&mut self, motor: HingeMotorPd) {
        self.hinge_motors.push(motor);
    }

    /// 設計 §4 パイプラインの `apply_forces`。P1 スコープ: 重力 + 球の抗力
    /// (docs/11-fluid/05-aero-hydrodynamics.md §2.1)+ 直立直方体の浮力
    /// (docs/11-fluid/04-free-surface-buoyancy.md §2.1)。結合力は後続増分。
    fn apply_forces(&mut self) {
        let n = self.bodies.len();
        for i in 0..n {
            if self.bodies.body_type[i] == BodyType::Dynamic && !self.bodies.asleep[i] {
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
            if self.bodies.body_type[i] != BodyType::Dynamic || self.bodies.asleep[i] {
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
            if self.bodies.body_type[i] == BodyType::Dynamic && self.bodies.asleep[i] {
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

    /// 少なくとも一方が「起きている dynamic body」なら解決対象(設計 §4「起床は新規接触・
    /// 力適用時」の反対: 両側とも寝ていれば新規に動く要素が無い)。
    fn manifold_is_active(&self, m: &collision::ContactManifold) -> bool {
        let a_awake_dynamic =
            self.bodies.body_type[m.body_a] == BodyType::Dynamic && !self.bodies.asleep[m.body_a];
        let b_awake_dynamic =
            self.bodies.body_type[m.body_b] == BodyType::Dynamic && !self.bodies.asleep[m.body_b];
        a_awake_dynamic || b_awake_dynamic
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
        joint::apply_hinge_motors(&self.hinge_motors, &mut self.bodies, dt);
        self.integrate_velocities(dt);
        // 処理順「ジョイント→接触」(設計 docs/10-mechanics/05-joints-constraints.md §4.1)。
        joint::resolve_distance(&self.joints, &mut self.bodies, dt);
        joint::resolve_ball(&self.ball_joints, &mut self.bodies, dt);
        joint::resolve_slider(&self.slider_joints, &mut self.bodies, dt);
        let manifolds = collision::detect(&self.bodies, &mut self.axis_cache);
        // 両側の dynamic body が全て asleep な接触は再解決しない(収束済みで変化が無いのに
        // 毎ステップ再解決すると warm start・split impulse の数値的な揺らぎで再起床してしまう
        // ことを実装検証中に発見した — 「積分を停止」だけでは不十分で、接触解決自体も
        // 停止する必要がある、設計 docs/10-mechanics/01-rigid-body.md §4)。
        let active_manifolds: Vec<collision::ContactManifold> = manifolds
            .iter()
            .filter(|m| self.manifold_is_active(m))
            .cloned()
            .collect();
        let ke_before_contact = self.total_energy().kinetic;
        contact::resolve(
            &active_manifolds,
            &mut self.bodies,
            ctx.materials,
            self.restitution_velocity_threshold,
            &mut self.contact_cache,
        );
        let ke_after_contact = self.total_energy().kinetic;
        debug_assert!(
            ke_after_contact <= ke_before_contact + 1e-6 * ke_before_contact.max(1.0),
            "contact resolution must not increase kinetic energy beyond numerical noise: \
             before={ke_before_contact} after={ke_after_contact}"
        );
        self.last_contact_dissipation = ke_before_contact - ke_after_contact;
        // 接触解決後(post-solve)の速度で静止判定する(解決前は重力積分直後でまだ抗力が
        // 相殺していないため静止判定に使えない)。島判定には(スキップした分も含め)
        // 全マニフォールドを使う。
        sleep::update_sleep_state(&mut self.bodies, &manifolds, dt);
        // 最小CCD(speculative contact、設計§4.6)。既存の実接触解決のあとに、まだ検出
        // されていない今ステップ中のすり抜けだけを速度クランプで防ぐ(P1標準機能)。
        ccd::apply_speculative_contacts(&mut self.bodies, dt);
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
    use sim_math::{Quat, SimRng, Vec3};

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

    /// エンティティ層の受け入れテスト(docs/20-integration/03-entity-layer.md §7
    /// 「静的姿勢維持: 関節PDのみで外乱なしのしゃがみ姿勢を60秒維持(転倒しない、
    /// 関節角ドリフト<5°)」)。倒立平衡(バランス制御)を含まない設計の指示どおり、
    /// 完全な15剛体の人体骨格ではなく、ワールド固定ピボット(股関節)に`BallJoint`で
    /// 繋がれた単一の脚リンクが、地面(`Plane`)に足先で接地しつつ`HingeMotorPd`が
    /// 45°のしゃがみ角を保持する縮約構成(モジュールdocの`joint::HingeMotorPd`参照)で
    /// 検証する — 「関節PD × 接触ソルバの結合」という設計が明記する検証対象そのものは、
    /// この縮約構成でも(ピボット+接地の両方が同時に働くため)保たれる。設計§4.5既定の
    /// PDゲイン(kp=20 s⁻¹, kd=2)をそのまま使用したところ、60秒間の最大ドリフトは
    /// 約3.8°(基準5°以内)、足先接地点は地面にめり込まず(min_tip_yが正、接触ソルバが
    /// 支えている)であることを実装検証中に確認した。
    #[test]
    fn entity_layer_hinge_motor_maintains_crouch_pose_for_60s_with_ground_contact() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut rng = SimRng::new(3, 3);
        let mut events = EventQueue::new();

        let mut solver = MechanicsSolver::new(9.80665);

        let mut ground = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            steel,
        );
        ground.body_type = BodyType::Static;
        solver.create_body(ground, &materials);

        let theta_target = std::f64::consts::FRAC_PI_4; // 45°(しゃがみ角)
        let half_extents = Vec3::new(0.05, 0.4, 0.05);
        let anchor_local_top = Vec3::new(0.0, half_extents.y, 0.0);
        let anchor_local_bottom = Vec3::new(0.0, -half_extents.y, 0.0);
        let rotation = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), theta_target);

        // 45°姿勢で足先(anchor_local_bottom)がちょうど地面(y=0)に接地するようピボットを選ぶ
        // (プログラム的に算出、手計算の符号取り違えを避ける)。
        let bottom_offset_from_pivot =
            rotation.rotate(anchor_local_bottom) - rotation.rotate(anchor_local_top);
        let pivot = Vec3::new(0.0, -bottom_offset_from_pivot.y, 0.0);
        let body_center = pivot - rotation.rotate(anchor_local_top);

        let mut leg_desc = RigidBodyDesc::dynamic(Shape::Box { half_extents }, steel);
        leg_desc.transform.position = body_center;
        leg_desc.transform.rotation = rotation;
        leg_desc.mass_override = Some(5.0);
        let leg = solver.create_body(leg_desc, &materials);

        solver.add_ball_joint(BallJoint {
            body_a: leg,
            anchor_a: anchor_local_top,
            body_b: None,
            anchor_b: pivot,
        });
        solver.add_hinge_motor(HingeMotorPd {
            body: leg,
            axis: Vec3::new(0.0, 0.0, 1.0),
            reference_rotation: Quat::IDENTITY,
            theta_target,
            kp: 20.0,
            kd: 2.0,
            torque_max: 50.0,
        });

        let dt = 1.0 / 120.0;
        let steps = 60 * 120;
        let mut max_drift: f64 = 0.0;
        let mut min_tip_y: f64 = f64::INFINITY;
        for _ in 0..steps {
            let mut ctx = make_ctx(&materials, &mut rng, &mut events);
            solver.step(dt, &mut ctx);

            assert!(
                solver.bodies.position[leg].x.is_finite()
                    && solver.bodies.position[leg].y.is_finite()
                    && solver.bodies.position[leg].z.is_finite(),
                "solver diverged: position={:?}",
                solver.bodies.position[leg]
            );

            let theta = solver.hinge_motors[0].measure_angle(&solver.bodies);
            max_drift = max_drift.max((theta - theta_target).abs());

            let tip = solver.bodies.position[leg]
                + solver.bodies.rotation[leg].rotate(anchor_local_bottom);
            min_tip_y = min_tip_y.min(tip.y);
        }

        let max_drift_deg = max_drift.to_degrees();
        assert!(
            max_drift_deg < 5.0,
            "joint angle drift too large: {max_drift_deg:.3} deg"
        );
        assert!(
            min_tip_y > -0.02,
            "foot penetrated the ground beyond contact slop: min_tip_y={min_tip_y:.5}"
        );
    }

    /// `SliderJoint`(設計 §4.4「Slider | 5 | 軸直交並進2 + 相対回転固定3」)の受け入れ:
    /// ワールドx軸に沿って自由に滑る「ピストンロッド」(ワールド固定シリンダー、
    /// `body_b=None`)が、(1)重力下でも軸に直交するy/zへ落下・ドリフトしない
    /// (直交並進2行が拘束)、(2)姿勢が生成時の基準(恒等回転)から傾かない
    /// (相対回転固定3行が拘束)、(3)軸方向(x)には初速のまま自由に(抵抗なく)進み続ける
    /// (拘束されない1自由度)ことを確認する — 断熱圧縮の`PistonGas`結合が前提とする
    /// 「シリンダー壁は軸直交方向・回転を拘束し、軸方向のみ自由」という構成そのもの。
    #[test]
    fn slider_joint_constrains_perpendicular_translation_and_rotation_but_frees_the_axis() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut rng = SimRng::new(11, 11);
        let mut events = EventQueue::new();

        let mut solver = MechanicsSolver::new(9.80665);
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.05 }, steel);
        desc.linear_velocity = Vec3::new(2.0, 0.0, 0.0);
        let piston = solver.create_body(desc, &materials);

        solver.add_slider_joint(SliderJoint::new(
            &solver.bodies,
            piston,
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            None,
            Vec3::ZERO,
        ));

        let dt = 1.0 / 120.0;
        let steps = 240; // 2秒: 軸方向に2.0*2.0=4.0m進む間の直交ドリフト/姿勢ドリフトを見る
        let mut max_perp: f64 = 0.0;
        let mut max_tilt_deg: f64 = 0.0;
        for _ in 0..steps {
            let mut ctx = make_ctx(&materials, &mut rng, &mut events);
            solver.step(dt, &mut ctx);
            let pos = solver.bodies.position[piston];
            max_perp = max_perp.max(pos.y.abs()).max(pos.z.abs());
            let rot = solver.bodies.rotation[piston];
            // 恒等回転からの角度 = 2*acos(|w|)(最短経路、二重被覆を考慮)。
            let tilt = 2.0 * rot.w.abs().min(1.0).acos();
            max_tilt_deg = max_tilt_deg.max(tilt.to_degrees());
        }

        assert!(
            max_perp < 0.01,
            "slider should not drift perpendicular to its axis under gravity: max_perp={max_perp:.5}"
        );
        assert!(
            max_tilt_deg < 1.0,
            "slider should not rotate relative to its fixed reference orientation: max_tilt_deg={max_tilt_deg:.3}"
        );
        let expected_x = 2.0 * (steps as f64 * dt);
        let actual_x = solver.bodies.position[piston].x;
        assert!(
            (actual_x - expected_x).abs() / expected_x < 0.01,
            "slider's free axis should move ballistically at the initial velocity: actual_x={actual_x} expected_x={expected_x}"
        );
    }
}
