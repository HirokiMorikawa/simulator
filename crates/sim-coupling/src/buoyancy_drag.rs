//! `BuoyancyDrag`(設計 docs/20-integration/01-coupling-matrix.md §3「P1: 静的媒質 → 剛体力
//! (浮力・抗力・揚力)」)。
//!
//! **既存の`MechanicsSolver`埋め込み実装との関係**: `sim_mechanics::MechanicsSolver`は
//! `water: Option<StaticWaterRegion>`・`atmosphere: Option<Atmosphere>`フィールドを持ち、
//! `apply_forces`内でシーン全体の該当形状(直立直方体の浮力・球の抗力)全てに無条件で
//! 同じ物理式(`sim_fluid::{buoyancy_force, drag_force_sphere, submerged_box_axis_aligned}`)
//! を適用する(シーン全体で単一の水域・大気を共有する既存の縮約、多数の既存テスト・
//! デモ(F4・F5・F1・F3・D6・D7等)がこの経路に依存する)。本Couplingはこれを置き換える
//! ものではない(切り出しは広く使われた既存経路への高リスクな変更になるため対象外、
//! 「既存のMechanicsSolver埋め込み実装の切り出しリスク」として繰り返し見送ってきた
//! 判断はそのまま維持する)。代わりに、同じ物理式を剛体単位でCouplingレジストリ経由
//! (シーンJSON`couplings`セクション等)から選択的に適用したい場合の、独立した追加の
//! 配線を提供する(`LorentzForce`・`BrownianForce`と同じ「剛体ごとの明示的な結合登録」
//! パターン、`MechanicsSolver`の内部状態を一切変更しない)。
//!
//! **二重計上に関する注意**: 同一剛体に対して`MechanicsSolver.water`/`.atmosphere`
//! (埋め込み経路)とこの`Coupling`の両方を有効にすると同じ物理を2回適用してしまう
//! (設計§2規則2)。シーン構築側がどちらか一方のみを選ぶ責任を持つ(設計のシーン設定
//! `SceneCouplingConfig`の`static_water_buoyancy`フラグが表す区別と同じ)。
//!
//! 浮力は直立姿勢の直方体のみ(`sim_fluid::buoyancy`冒頭注記と同じ縮約)、抗力は球のみ
//! (`sim_fluid::aero`冒頭注記と同じ縮約)を対象とする。
//!
//! **群5で「揚力」を実装した**(`lift`フィールド)。移行前は設計名`BuoyancyDrag`が挙げる
//! 揚力を「`sim_fluid`側にも式自体が未実装のため対象外」としていた——縮約の理由が
//! **この結合の外**(下位クレートの欠落)にあったので、`sim_fluid::aero`へ
//! 薄翼理論の`wing_lift_force`とマグヌス力の`magnus_force_sphere`(設計
//! docs/11-fluid/05-aero-hydrodynamics.md §2.2)を足した上でここへ配線した。
//! 揚力面の姿勢は剛体の回転で追従する(翼弦・スパンをローカル軸で与え、毎step
//! ワールドへ回す)。マグヌス力は剛体の角速度をそのまま使う。
//!
//! **群5**: 浮力・抗力の速度注入を`apply_pre`(ドメインソルバの**前**)へ移した。
//! post 相に置くと、その step の位置積分は注入前の速度で行われるため、外力が位置応答に
//! 1step遅れて現れる。pre 相なら重力と同じ相で効く。

use crate::domain_states::{Coupling, CouplingKind, CouplingParam, DomainStates};
use sim_core::DomainId;
use sim_fluid::{Atmosphere, StaticWaterRegion};
use sim_math::Vec3;
use sim_mechanics::{DragModel, Shape};

/// 揚力の与え方(設計 docs/11-fluid/05-aero-hydrodynamics.md §2.2、**群5で追加**)。
/// `atmosphere`が`None`なら評価されない(揚力は媒質密度に比例するため)。
#[derive(Clone, Copy, Debug)]
pub enum LiftModel {
    /// 翼(薄翼理論 $C_L\approx2\pi\alpha$ + 失速)。`chord_local`・`span_local`は
    /// **剛体ローカル座標**の翼弦方向・スパン方向(毎step剛体の姿勢でワールドへ回す)。
    Wing {
        /// 翼面積 $A$ [m^2]。
        area: f64,
        chord_local: Vec3,
        span_local: Vec3,
        /// 操縦面(エルロン・エレベーター・ラダー)の舵角[rad]、既定0
        /// (**残タスク完遂の縦串⑤増分**)。`chord_local`を`span_local`軸まわりに
        /// この角度だけ追加回転させてから使う——`Coupling::set_scalar_param`
        /// (`CouplingParam::ControlSurfaceDeflection`)経由で実行時に変更できる、
        /// Coupling registryで唯一書き換え可能なパラメータ。
        control_surface_deflection: f64,
    },
    /// 回転球のマグヌス効果($C_M\approx0.2S$)。剛体の角速度をそのまま使う。
    MagnusSphere { radius: f64 },
}

/// 剛体(`body_index`)を静的水域・大気による浮力・抗力・揚力に結合する
/// (モジュールdoc参照)。
#[derive(Clone)]
pub struct BuoyancyDrag {
    pub body_index: usize,
    pub water: Option<StaticWaterRegion>,
    pub atmosphere: Option<Atmosphere>,
    /// 揚力(**群5で追加**)。`None`なら揚力を評価しない(移行前と同じ挙動)。
    pub lift: Option<LiftModel>,
}

impl Coupling for BuoyancyDrag {
    fn kind(&self) -> CouplingKind {
        CouplingKind::BuoyancyDrag
    }

    fn domain_ids(&self) -> &'static [DomainId] {
        &[DomainId::Mechanics, DomainId::Fluid]
    }

    fn describe(&self) -> String {
        format!(
            "BuoyancyDrag body#{} water={} atmosphere={}",
            self.body_index,
            self.water.is_some(),
            self.atmosphere.is_some()
        )
    }

    fn referenced_bodies(&self) -> Vec<usize> {
        vec![self.body_index]
    }

    fn apply_pre(&mut self, world: &mut DomainStates, dt: f64) {
        let idx = self.body_index;
        let mass = world.mechanics.bodies.mass(idx);
        if mass <= 0.0 {
            return; // 静的/キネマティック剛体には適用しない。
        }

        let mut force = Vec3::ZERO;
        if let Some(water) = &self.water {
            if let Shape::Box { half_extents } = *world.mechanics.bodies.shape_of(idx) {
                let pos = world.mechanics.bodies.position[idx];
                let (v_sub, _) =
                    sim_fluid::submerged_box_axis_aligned(pos, half_extents, water.water_level);
                if v_sub > 0.0 {
                    force = force
                        + sim_fluid::buoyancy_force(v_sub, water.density, world.mechanics.gravity);
                }
            }
        }
        if let Some(atm) = &self.atmosphere {
            let velocity = world.mechanics.bodies.linear_velocity[idx];
            if let DragModel::Sphere { radius } = world.mechanics.bodies.drag[idx] {
                force = force + sim_fluid::drag_force_sphere(radius, atm, velocity);
            }
            // 揚力(**群5で追加**、モジュールdoc参照)。
            match self.lift {
                Some(LiftModel::Wing {
                    area,
                    chord_local,
                    span_local,
                    control_surface_deflection,
                }) => {
                    // 舵角(**残タスク完遂の縦串⑤増分**)をspan_local軸まわりの
                    // 追加回転として先に適用してから、剛体の姿勢でワールドへ回す。
                    let deflected_chord_local = if control_surface_deflection != 0.0 {
                        sim_math::Quat::from_axis_angle(span_local, control_surface_deflection)
                            .rotate(chord_local)
                    } else {
                        chord_local
                    };
                    let r = world.mechanics.bodies.rotation[idx];
                    force = force
                        + sim_fluid::wing_lift_force(
                            area,
                            r.rotate(deflected_chord_local),
                            r.rotate(span_local),
                            atm,
                            velocity,
                        );
                }
                Some(LiftModel::MagnusSphere { radius }) => {
                    let omega = world.mechanics.bodies.angular_velocity[idx];
                    force = force + sim_fluid::magnus_force_sphere(radius, atm, velocity, omega);
                }
                None => {}
            }
        }

        world.mechanics.bodies.linear_velocity[idx] =
            world.mechanics.bodies.linear_velocity[idx] + force.scale(dt / mass);
    }

    /// **post 相では何もしない(群5)**。既定実装は`apply`へ委譲するので、これを省略すると
    /// `World`が pre と post で同じ力を2回積んでしまう。
    fn apply_post(&mut self, _world: &mut DomainStates, _dt: f64) {}

    /// **単相で呼ばれた場合の互換経路**(`World`を経由しない直接呼び出し用)。
    /// 全処理は pre 相にある(モジュールdoc参照)。
    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        self.apply_pre(world, dt);
    }

    /// `ControlSurfaceDeflection`(`self.lift`が`LiftModel::Wing`の場合のみ)を
    /// 受け付ける(**残タスク完遂の縦串⑤増分**、`CouplingParam`のdoc参照)。
    fn set_scalar_param(&mut self, param: CouplingParam, value: f64) -> bool {
        match (param, &mut self.lift) {
            (
                CouplingParam::ControlSurfaceDeflection,
                Some(LiftModel::Wing {
                    control_surface_deflection,
                    ..
                }),
            ) => {
                *control_surface_deflection = value;
                true
            }
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::MaterialDb;
    use sim_mechanics::{MechanicsSolver, RigidBodyDesc};

    fn states<'a>(mechanics: &'a mut MechanicsSolver) -> DomainStates<'a> {
        DomainStates {
            mechanics,
            thermal: None,
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: None,
            grid_fluid_3d: None,
            sph: None,
        }
    }

    /// 半分水没した直立直方体に、`sim_fluid::buoyancy_force`の解析式どおりの浮力が
    /// 注入されること(`sim_fluid::buoyancy`側で既に検証済みの物理式そのものをここでは
    /// 直接呼び出して期待値を作るため、動的な定量検証特有の縁効果は生じない)。
    #[test]
    fn buoyancy_drag_applies_known_buoyancy_force_for_a_submerged_box() {
        let materials = MaterialDb::standard();
        let wood = materials.find_by_name("木材(松)").unwrap();
        let half_extents = Vec3::new(0.5, 0.5, 0.5);
        let mut desc = RigidBodyDesc::dynamic(Shape::Box { half_extents }, wood);
        let mass = 10.0;
        desc.mass_override = Some(mass);
        desc.transform.position = Vec3::new(0.0, 0.25, 0.0); // 半分(0.25m)水没。
        let gravity = 9.80665;
        let mut mechanics = MechanicsSolver::new(gravity);
        let body = mechanics.create_body(desc, &materials);

        let water = StaticWaterRegion::new(0.5, 1000.0);
        let (v_sub, _) = sim_fluid::submerged_box_axis_aligned(
            Vec3::new(0.0, 0.25, 0.0),
            half_extents,
            water.water_level,
        );
        assert!(v_sub > 0.0, "test setup should submerge the box partially");
        let expected_force = sim_fluid::buoyancy_force(v_sub, water.density, gravity);

        let mut coupling = BuoyancyDrag {
            body_index: body,
            water: Some(water),
            atmosphere: None,
            lift: None,
        };
        let dt = 0.01;
        let velocity_before = mechanics.bodies.linear_velocity[body];
        coupling.apply(&mut states(&mut mechanics), dt);

        let expected_velocity = velocity_before + expected_force.scale(dt / mass);
        let measured_velocity = mechanics.bodies.linear_velocity[body];
        assert!(
            (measured_velocity - expected_velocity).length() < 1e-12,
            "measured_velocity={measured_velocity:?} expected_velocity={expected_velocity:?}"
        );
    }

    /// 移動中の球に、`sim_fluid::drag_force_sphere`の解析式どおりの抗力が注入される
    /// こと。
    #[test]
    fn buoyancy_drag_applies_known_drag_force_for_a_moving_sphere() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let radius = 0.05;
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, steel);
        let mass = 1.0;
        desc.mass_override = Some(mass);
        desc.drag = DragModel::Sphere { radius };
        desc.linear_velocity = Vec3::new(3.0, 0.0, 0.0);
        let mut mechanics = MechanicsSolver::new(9.80665);
        let body = mechanics.create_body(desc, &materials);

        let atmosphere = Atmosphere::still(1.225, 1.81e-5);
        let expected_force =
            sim_fluid::drag_force_sphere(radius, &atmosphere, Vec3::new(3.0, 0.0, 0.0));

        let mut coupling = BuoyancyDrag {
            body_index: body,
            water: None,
            atmosphere: Some(atmosphere),
            lift: None,
        };
        let dt = 0.001;
        let velocity_before = mechanics.bodies.linear_velocity[body];
        coupling.apply(&mut states(&mut mechanics), dt);

        let expected_velocity = velocity_before + expected_force.scale(dt / mass);
        let measured_velocity = mechanics.bodies.linear_velocity[body];
        assert!(
            (measured_velocity - expected_velocity).length() < 1e-12,
            "measured_velocity={measured_velocity:?} expected_velocity={expected_velocity:?}"
        );
    }

    /// 水面より上にある(水没していない)直方体には浮力を適用しない。
    #[test]
    fn buoyancy_drag_does_nothing_for_a_box_above_the_waterline() {
        let materials = MaterialDb::standard();
        let wood = materials.find_by_name("木材(松)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(0.5, 0.5, 0.5),
            },
            wood,
        );
        desc.transform.position = Vec3::new(0.0, 10.0, 0.0);
        let mut mechanics = MechanicsSolver::new(9.80665);
        let body = mechanics.create_body(desc, &materials);

        let mut coupling = BuoyancyDrag {
            body_index: body,
            water: Some(StaticWaterRegion::new(0.0, 1000.0)),
            atmosphere: None,
            lift: None,
        };
        let velocity_before = mechanics.bodies.linear_velocity[body];
        coupling.apply(&mut states(&mut mechanics), 0.01);

        assert_eq!(mechanics.bodies.linear_velocity[body], velocity_before);
    }

    /// 質量0以下(静的/キネマティック)の剛体には適用しない(他のCoupling実装と同じ
    /// ガード、`SphRigid`・`GridFluidRigid`等参照)。
    #[test]
    fn buoyancy_drag_does_nothing_for_a_static_body() {
        let materials = MaterialDb::standard();
        let wood = materials.find_by_name("木材(松)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(0.5, 0.5, 0.5),
            },
            wood,
        );
        desc.body_type = sim_mechanics::BodyType::Static;
        desc.transform.position = Vec3::new(0.0, 0.25, 0.0);
        let mut mechanics = MechanicsSolver::new(9.80665);
        let body = mechanics.create_body(desc, &materials);

        let mut coupling = BuoyancyDrag {
            body_index: body,
            water: Some(StaticWaterRegion::new(0.5, 1000.0)),
            atmosphere: None,
            lift: None,
        };
        coupling.apply(&mut states(&mut mechanics), 0.01);

        assert_eq!(mechanics.bodies.linear_velocity[body], Vec3::ZERO);
    }

    /// **群5: 揚力の配線**。マグヌス力が剛体の角速度から作られ、`sim_fluid`側の式と
    /// 厳密に一致する量が速度へ注入されること、`lift: None`なら一切注入されないこと
    /// (移行前の挙動が保たれること)を確認する。
    #[test]
    fn buoyancy_drag_injects_magnus_lift_from_the_bodys_angular_velocity() {
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let radius = 0.037;
        let mass = 0.058;
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, steel);
        desc.mass_override = Some(mass);
        desc.linear_velocity = Vec3::new(25.0, 0.0, 0.0);
        desc.angular_velocity = Vec3::new(0.0, 0.0, 50.0); // バックスピン
        desc.drag = DragModel::None; // 抗力を切って揚力だけを見る
        let mut mechanics = MechanicsSolver::new(0.0);
        let body = mechanics.create_body(desc, &materials);

        let atmosphere = Atmosphere::still(1.225, 1.81e-5);
        let expected = sim_fluid::magnus_force_sphere(
            radius,
            &atmosphere,
            Vec3::new(25.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 50.0),
        );
        assert!(
            expected.y > 0.0,
            "test setup should produce upward Magnus lift"
        );

        let dt = 0.001;
        let velocity_before = mechanics.bodies.linear_velocity[body];
        let mut coupling = BuoyancyDrag {
            body_index: body,
            water: None,
            atmosphere: Some(atmosphere),
            lift: Some(LiftModel::MagnusSphere { radius }),
        };
        coupling.apply(&mut states(&mut mechanics), dt);
        let measured = mechanics.bodies.linear_velocity[body];
        let expected_velocity = velocity_before + expected.scale(dt / mass);
        assert!(
            (measured - expected_velocity).length() < 1e-15,
            "measured={measured:?} expected={expected_velocity:?}"
        );

        // `lift: None` なら何も注入されない(移行前の挙動)。
        let mut mechanics2 = MechanicsSolver::new(0.0);
        let mut desc2 = RigidBodyDesc::dynamic(Shape::Sphere { radius }, steel);
        desc2.mass_override = Some(mass);
        desc2.linear_velocity = Vec3::new(25.0, 0.0, 0.0);
        desc2.angular_velocity = Vec3::new(0.0, 0.0, 50.0);
        desc2.drag = DragModel::None;
        let body2 = mechanics2.create_body(desc2, &materials);
        let mut no_lift = BuoyancyDrag {
            body_index: body2,
            water: None,
            atmosphere: Some(atmosphere),
            lift: None,
        };
        no_lift.apply(&mut states(&mut mechanics2), dt);
        assert_eq!(
            mechanics2.bodies.linear_velocity[body2],
            Vec3::new(25.0, 0.0, 0.0),
            "lift: None は移行前どおり揚力を一切与えない"
        );
    }

    /// **群5**: 翼の揚力が剛体の**姿勢**に追従すること(翼弦・スパンはローカル指定
    /// なので、ローカル→ワールド変換を忘れていれば破れる)。3つの姿勢で確認する:
    /// ①機首上げ$\alpha$で水平飛行 → 上向きの揚力
    /// ②翼は水平のまま速度を$\alpha$上向きに傾ける(=上昇) → **下向き**の揚力
    ///   (上昇中の翼は上面から風を受ける。符号を取り違えていればここで破れる)
    /// ③進行軸まわりに90°ロール → 流れの上下成分が丸ごと横滑りになるので揚力は消える
    #[test]
    fn buoyancy_drag_wing_lift_follows_the_bodys_orientation() {
        let materials = MaterialDb::standard();
        let alu = materials.find_by_name("アルミニウム").unwrap();
        let mass = 5.0;
        let atmosphere = Atmosphere::still(1.225, 1.81e-5);
        let alpha = 5.0_f64.to_radians();
        let speed = 30.0;
        let lift = LiftModel::Wing {
            area: 2.0,
            chord_local: Vec3::new(1.0, 0.0, 0.0),
            span_local: Vec3::new(0.0, 0.0, 1.0),
            control_surface_deflection: 0.0,
        };

        // 1step ぶんの速度変化 = 揚力 * dt / mass(抗力を切ってあるので揚力のみ)。
        let lift_impulse = |rotation: sim_math::Quat, velocity: Vec3| -> Vec3 {
            let mut desc = RigidBodyDesc::dynamic(
                Shape::Box {
                    half_extents: Vec3::new(1.0, 0.1, 2.0),
                },
                alu,
            );
            desc.mass_override = Some(mass);
            desc.linear_velocity = velocity;
            desc.drag = DragModel::None;
            desc.transform.rotation = rotation;
            let mut mechanics = MechanicsSolver::new(0.0);
            let body = mechanics.create_body(desc, &materials);
            let mut coupling = BuoyancyDrag {
                body_index: body,
                water: None,
                atmosphere: Some(atmosphere),
                lift: Some(lift),
            };
            coupling.apply(&mut states(&mut mechanics), 0.001);
            mechanics.bodies.linear_velocity[body] - velocity
        };

        let z = Vec3::new(0.0, 0.0, 1.0);
        let x = Vec3::new(1.0, 0.0, 0.0);

        // ① 機首上げ alpha(スパン +z まわりに +alpha 回す)で水平飛行 → 上向き。
        let nose_up = lift_impulse(sim_math::Quat::from_axis_angle(z, alpha), x.scale(speed));
        assert!(
            nose_up.y > 0.0 && nose_up.x.abs() < 1e-12,
            "機首上げでは上向きの揚力: {nose_up:?}"
        );

        // ② 翼は水平のまま上昇(速度を alpha だけ上へ傾ける)→ 下向き。
        let climbing = lift_impulse(
            sim_math::Quat::IDENTITY,
            Vec3::new(alpha.cos(), alpha.sin(), 0.0).scale(speed),
        );
        assert!(
            climbing.y < 0.0,
            "上昇中の水平翼は上面から風を受けるので下向きの揚力: {climbing:?}"
        );
        // 迎角の大きさは同じなので揚力の大きさも一致する(符号だけが違う)。
        assert!(
            (climbing.length() - nose_up.length()).abs() / nose_up.length() < 1e-9,
            "|nose_up|={} |climbing|={}",
            nose_up.length(),
            climbing.length()
        );

        // ③ 進行軸(+x)まわりに90°ロールすると、スパンが +z から -y へ回り、
        //    流れの上下成分が丸ごとスパン方向(横滑り)になるので迎角が消える。
        let rolled = lift_impulse(
            sim_math::Quat::from_axis_angle(x, std::f64::consts::FRAC_PI_2),
            Vec3::new(alpha.cos(), alpha.sin(), 0.0).scale(speed),
        );
        assert!(
            rolled.length() < 1e-12,
            "90度ロールで迎角が消えるので揚力もゼロ: {rolled:?}"
        );
    }

    /// **残タスク完遂の縦串⑤増分**——`Coupling::set_scalar_param`
    /// (`CouplingParam::ControlSurfaceDeflection`)で舵角を設定すると、
    /// 同じ角度だけ機体姿勢を回転させたのと同一の揚力が出ること(舵角は
    /// `chord_local`を`span_local`軸まわりに追加回転させるだけなので、
    /// 「翼を傾ける」か「機体ごと傾ける」かは等価であるはず)。あわせて
    /// `set_scalar_param`が既定の`false`(未対応パラメータ)を正しく返す
    /// ケースも確認する。
    #[test]
    fn control_surface_deflection_matches_an_equivalent_body_rotation() {
        let materials = MaterialDb::standard();
        let alu = materials.find_by_name("アルミニウム").unwrap();
        let mass = 5.0;
        let atmosphere = Atmosphere::still(1.225, 1.81e-5);
        let alpha = 5.0_f64.to_radians();
        let speed = 30.0;
        let x = Vec3::new(1.0, 0.0, 0.0);
        let z = Vec3::new(0.0, 0.0, 1.0);

        let lift_impulse = |rotation: sim_math::Quat, deflection: f64| -> Vec3 {
            let mut desc = RigidBodyDesc::dynamic(
                Shape::Box {
                    half_extents: Vec3::new(1.0, 0.1, 2.0),
                },
                alu,
            );
            desc.mass_override = Some(mass);
            desc.linear_velocity = x.scale(speed);
            desc.drag = DragModel::None;
            desc.transform.rotation = rotation;
            let mut mechanics = MechanicsSolver::new(0.0);
            let body = mechanics.create_body(desc, &materials);
            let mut coupling = BuoyancyDrag {
                body_index: body,
                water: None,
                atmosphere: Some(atmosphere),
                lift: Some(LiftModel::Wing {
                    area: 2.0,
                    chord_local: x,
                    span_local: z,
                    control_surface_deflection: 0.0,
                }),
            };
            if deflection != 0.0 {
                assert!(
                    coupling.set_scalar_param(CouplingParam::ControlSurfaceDeflection, deflection)
                );
            }
            let velocity_before = mechanics.bodies.linear_velocity[body];
            coupling.apply(&mut states(&mut mechanics), 0.001);
            mechanics.bodies.linear_velocity[body] - velocity_before
        };

        let via_body_rotation = lift_impulse(sim_math::Quat::from_axis_angle(z, alpha), 0.0);
        let via_control_surface = lift_impulse(sim_math::Quat::IDENTITY, alpha);
        assert!(
            (via_control_surface - via_body_rotation).length() / via_body_rotation.length() < 1e-9,
            "舵角による揚力は機体回転による揚力と一致するはず: via_control_surface={via_control_surface:?} via_body_rotation={via_body_rotation:?}"
        );

        // 未対応のパラメータ・Couplingでは`false`を返す(既定実装の確認)。
        let mut plain = BuoyancyDrag {
            body_index: 0,
            water: None,
            atmosphere: None,
            lift: None,
        };
        assert!(!plain.set_scalar_param(CouplingParam::ControlSurfaceDeflection, alpha));
    }
}
