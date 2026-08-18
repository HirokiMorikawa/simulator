//! `BuoyancyDrag`(設計 docs/20-integration/01-coupling-matrix.md §3「P1: 静的媒質 → 剛体力
//! (浮力・抗力・揚力)」)。
//!
//! **既存の`MechanicsSolver`埋め込み実装との関係**: `sim_mechanics::MechanicsSolver`は
//! `fluids: Vec<FluidRegion>`・`atmosphere: Option<Atmosphere>`フィールドを持ち、
//! `apply_forces`内でシーン全体の該当形状(直立直方体の浮力・球の抗力)全てに
//! 同じ物理式(`sim_fluid::{buoyancy_force, drag_force_sphere, submerged_box_below_plane}`)
//! を適用する(結合登録を要さず、剛体を置くだけで効く経路。どの水域が効くかは
//! 剛体の重心を含む最初の領域という決着規則で決まり(`MechanicsSolver::fluids`の
//! doc)、大気はシーン全体で1つを共有する縮約のまま。多数の既存テスト・
//! デモ(F4・F5・F1・F3・D6・D7等)がこの経路に依存する)。本Couplingはこれを置き換える
//! ものではない(切り出しは広く使われた既存経路への高リスクな変更になるため対象外、
//! 「既存のMechanicsSolver埋め込み実装の切り出しリスク」として繰り返し見送ってきた
//! 判断はそのまま維持する)。代わりに、同じ物理式を剛体単位でCouplingレジストリ経由
//! (シーンJSON`couplings`セクション等)から選択的に適用したい場合の、独立した追加の
//! 配線を提供する(`LorentzForce`・`BrownianForce`と同じ「剛体ごとの明示的な結合登録」
//! パターン、`MechanicsSolver`の内部状態を一切変更しない)。
//!
//! **なぜこちらは`Vec`にしないのか(流体領域の一般化に際しての判断)**:
//! `MechanicsSolver::fluids`が複数領域を持つのは、シーン全体の全剛体を1つの
//! フィールドで面倒みる埋め込み経路だからで、「この剛体はどの水域か」を
//! 位置から引き当てる規則が要る。対してこの`Coupling`は**剛体1体につき1件
//! 登録する**形なので、どの水域に結ぶかはシーン構築側が登録時に明示済みである
//! ——引き当て規則を重ねる意味がない。したがって`water`は`Option<FluidRegion>`の
//! ままとし、一般化の恩恵(形状・水温)だけを型から受け取る。形状は効く:
//! `water`にAABB領域を与えれば、その剛体が領域の外にいるstepでは浮力が
//! 出なくなる(`FluidRegion::contains`)。
//!
//! **二重計上に関する注意**: 同一剛体に対して`MechanicsSolver.fluids`/`.atmosphere`
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
//! 1step遅れて現れる。pre 相なら重力と同じ相で効く。**力の積分方式はこの群5で
//! 決着済み**——`apply_pre`は集めた力をその場で`v += F·dt/m`と速度へ入れ(半陰的
//! Eulerの速度更新と同じ相・同じ形)、`apply_post`は二重計上を避けるため空にしてある。
//! したがって「浮力の力積分方式を直接速度キックへ書き換える」余地は**もう無い**
//! (`buoyancy_force_integration_conserves_mechanical_energy_without_drag`が
//! その方式のエネルギー挙動を基準線として固定している)。
//!
//! **重力追従増分**: 浮力が`sim_mechanics::GravityField`へ追従するようになった。
//! `apply_pre`は剛体位置での局所的な重力
//! (`GravityField::up_and_magnitude_at`)から「上」を取り、自由表面をそれに
//! 垂直な平面として水面下体積を切り(`sim_fluid::submerged_box_below_plane`)、
//! 浮力をその向きへ出す(`sim_fluid::buoyancy_force`)。既定の`-y`向き一様重力では
//! `up = (0,1,0)`となり、移行前とビット単位で同一の結果になる。
//! `Zero`場では浮力が厳密に0、`PointSource`場では自由表面を局所鉛直に垂直な
//! 平面で近似する(`apply_pre`内のコメントと`sim_fluid::buoyancy`モジュールdoc)。

use crate::domain_states::{Coupling, CouplingKind, CouplingParam, DomainStates};
use sim_core::DomainId;
use sim_fluid::{Atmosphere, FluidRegion};
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
    /// この剛体を結ぶ流体領域(モジュールdoc「なぜこちらは`Vec`にしないのか」参照)。
    /// `FluidShape::Aabb`なら剛体の重心が領域外にあるstepでは浮力を出さない。
    pub water: Option<FluidRegion>,
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
                // **浮力は重力場へ追従する(重力追従増分)**。剛体位置における
                // 局所的な重力の逆向きを「上」とし、自由表面はそれに垂直な平面
                // (原点から`up`軸に沿って`water_level`の符号付き距離)、
                // 浮力はその`up`向きに出る。以前ここにあった
                // 「水面はワールドy軸に垂直な水平面に固定で、スカラー縮約
                // `MechanicsSolver::gravity()`しか読めない」という限界は解消した。
                //
                // **残る限界**(`sim_fluid::buoyancy`モジュールdocに詳しい):
                // (1) `GravityField::Zero`では「上」が定義できないので浮力は
                //     厳密に0(`up_and_magnitude_at`が`None`)。
                // (2) `PointSource`では自由表面を局所鉛直に垂直な**平面**で
                //     近似する(本来の等ポテンシャル面=球面の曲率は無視)。
                // (3) 剛体の姿勢は依然として無視する——水面が傾いても切られる
                //     直方体はワールド軸に平行なままで、浮心のずれが生む復原
                //     モーメントも積んでいない。
                //
                // 領域の内外判定(`FluidRegion::contains`)。`HalfSpace`(移行前の
                // `StaticWaterRegion`相当)では常に`true`なので、移行前と同一の挙動。
                let up_and_g = world.mechanics.gravity_field().up_and_magnitude_at(pos);
                if let (Some((up, g)), true) = (up_and_g, water.contains(pos)) {
                    let (v_sub, _) = sim_fluid::submerged_box_below_plane(
                        pos,
                        half_extents,
                        up,
                        water.water_level,
                    );
                    if v_sub > 0.0 {
                        force = force + sim_fluid::buoyancy_force(v_sub, water.density, g, up);
                    }
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

    /// Coupling registryで唯一、実行時に書き換えられるパラメータを持つ結合
    /// (**Task#9**、`Coupling::supported_params`のdoc参照)。
    ///
    /// **`lift`が`LiftModel::Wing`でなくても`ControlSurfaceDeflection`を挙げる**
    /// ——ここが宣言するのは「`BuoyancyDrag`という種別が受け付け得る
    /// パラメータ」であって、その瞬間のフィールド値に依存する可否ではない。
    /// 揚力モデル込みの可否まで見たい呼び出し側は従来どおり
    /// `set_scalar_param`の戻り値で確かめられる(こちらは`&'static`を返す
    /// 都合上、`self`の状態で分岐した動的なスライスは返せない)。
    fn supported_params(&self) -> &'static [CouplingParam] {
        &[CouplingParam::ControlSurfaceDeflection]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::MaterialDb;
    use sim_mechanics::{GravityField, MechanicsSolver, RigidBodyDesc};

    /// 既定の重力(ワールド`-y`向き一様)に対応する「上」。
    const UP_Y: Vec3 = Vec3 {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    };

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

        let water = FluidRegion::new(0.5, 1000.0);
        let (v_sub, _) = sim_fluid::submerged_box_axis_aligned(
            Vec3::new(0.0, 0.25, 0.0),
            half_extents,
            water.water_level,
        );
        assert!(v_sub > 0.0, "test setup should submerge the box partially");
        // 既定の重力(-y向き一様)なので「上」は +y。
        let expected_force = sim_fluid::buoyancy_force(v_sub, water.density, gravity, UP_Y);

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
            water: Some(FluidRegion::new(0.0, 1000.0)),
            atmosphere: None,
            lift: None,
        };
        let velocity_before = mechanics.bodies.linear_velocity[body];
        coupling.apply(&mut states(&mut mechanics), 0.01);

        assert_eq!(mechanics.bodies.linear_velocity[body], velocity_before);
    }

    /// **流体領域の一般化**: `water`にAABB領域を与えると、領域の外にいる剛体には
    /// (「無限に広い半空間なら水面下」の高さでも)浮力を出さない。
    /// 同じ剛体・同じ高さで`HalfSpace`なら出る、という対比で形状が効いていることを見る。
    #[test]
    fn buoyancy_drag_respects_the_bounds_of_an_aabb_fluid_region() {
        let materials = MaterialDb::standard();
        let wood = materials.find_by_name("木材(松)").unwrap();
        let half_extents = Vec3::new(0.5, 0.5, 0.5);

        let impulse = |water: FluidRegion, position: Vec3| -> Vec3 {
            let mut desc = RigidBodyDesc::dynamic(Shape::Box { half_extents }, wood);
            desc.mass_override = Some(10.0);
            desc.transform.position = position;
            let mut mechanics = MechanicsSolver::new(9.80665);
            let body = mechanics.create_body(desc, &materials);
            let mut coupling = BuoyancyDrag {
                body_index: body,
                water: Some(water),
                atmosphere: None,
                lift: None,
            };
            let before = mechanics.bodies.linear_velocity[body];
            coupling.apply(&mut states(&mut mechanics), 0.01);
            mechanics.bodies.linear_velocity[body] - before
        };

        let tank = FluidRegion::aabb(
            Vec3::new(-2.0, -5.0, -2.0),
            Vec3::new(2.0, 2.0, 2.0),
            0.0,
            1000.0,
        );
        let half_space = FluidRegion::new(0.0, 1000.0);
        let inside = Vec3::new(0.0, -1.0, 0.0);
        let outside = Vec3::new(10.0, -1.0, 0.0);

        // 水槽の中では半空間と同じ浮力(式は一切変えていない)。
        assert_eq!(impulse(tank, inside), impulse(half_space, inside));
        assert!(impulse(tank, inside).y > 0.0);
        // 水槽の外では浮力ゼロ。半空間なら同じ位置で浮力が出る。
        assert_eq!(impulse(tank, outside), Vec3::ZERO);
        assert!(impulse(half_space, outside).y > 0.0);
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
            water: Some(FluidRegion::new(0.5, 1000.0)),
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

    /// `supported_params`が「試しに書き込んでみる」ことなく舵角の可否を
    /// 答えられる(**Task#9**、`Coupling::supported_params`のdoc参照)。
    /// **揚力モデルを持たない`BuoyancyDrag`でも挙げる**ことまで見る——
    /// 種別単位の宣言であって瞬間のフィールド値ではない、という規約の固定。
    #[test]
    fn buoyancy_drag_declares_its_runtime_adjustable_param() {
        let wing = BuoyancyDrag {
            body_index: 0,
            water: None,
            atmosphere: None,
            lift: Some(LiftModel::Wing {
                area: 1.0,
                chord_local: sim_math::Vec3::new(1.0, 0.0, 0.0),
                span_local: sim_math::Vec3::new(0.0, 0.0, 1.0),
                control_surface_deflection: 0.0,
            }),
        };
        assert_eq!(
            wing.supported_params(),
            &[CouplingParam::ControlSurfaceDeflection]
        );

        let plain = BuoyancyDrag {
            body_index: 0,
            water: None,
            atmosphere: None,
            lift: None,
        };
        assert_eq!(
            plain.supported_params(),
            &[CouplingParam::ControlSurfaceDeflection]
        );

        // 実行時に変更できるパラメータを持たない結合は空(トレイトの既定実装)。
        let lorentz = crate::LorentzForce {
            body_index: 0,
            charge: 1.0,
        };
        assert!(lorentz.supported_params().is_empty());
        assert_eq!(
            CouplingParam::ControlSurfaceDeflection.name(),
            "ControlSurfaceDeflection"
        );
    }

    // ------------------------------------------------------------------
    // 解析解による足場(**`apply_pre`の力積分方式を書き換える将来の変更に備えた
    // 回帰ハーネス**)。既存の`buoyancy_drag_applies_known_buoyancy_force_*`群は
    // 「1stepぶんの速度変化が`sim_fluid`の式と一致する」ことしか見ておらず、
    // **多数step回したときの振る舞い**(平衡位置・エネルギー保存)は無検証だった。
    // 力積分の方式を変えれば1stepの式が同じでも長時間の挙動は壊れうる。
    // ------------------------------------------------------------------

    /// 浮体の実験用材料(摩擦・反発ゼロ、密度は水の`ratio`倍)。
    fn floating_material(materials: &mut MaterialDb, density: f64) -> sim_core::MaterialId {
        materials.push(sim_core::Material {
            name: "test-floating-box-for-buoyancy-analytics",
            density,
            friction: 0.0,
            restitution: 0.0,
            youngs_modulus: None,
            specific_heat: 1000.0,
            conductivity: 1.0,
            emissivity: 0.5,
            melting: None,
            resistivity: None,
            relative_permittivity: 1.0,
            refractive_index: None,
            source: "test fixture",
            uncertainty: 0.0,
        })
    }

    /// 直立直方体を水面に浮かべ、`BuoyancyDrag`(pre相)→`MechanicsSolver::step`の
    /// 順で`steps`ステップ回す(`World`のpre相と同じ順序)。各ステップ後の
    /// `(位置, 速度)`を返す。抗力は`atmosphere: None`で完全に切ってあるので、
    /// 働く力は重力と浮力だけ(散逸ゼロ)。テスト専用フィクスチャで呼び出し元は
    /// 本ファイル内の少数のテストのみのため、引数を構造体へまとめる必要は薄いと判断。
    ///
    /// **重力追従増分**で重力を`GravityField`で受け取る形へ広げた(傾いた重力の
    /// 検証に使う)。`Uniform`かつ向きが既定の`-y`なら移行前と同一の数値になる
    /// ——`MechanicsSolver::new(g)`が組み立てるのと同じ場だから。
    #[allow(clippy::too_many_arguments)]
    fn simulate_floating_box_in_field(
        half_extents: Vec3,
        mass: f64,
        density: f64,
        water: FluidRegion,
        field: GravityField,
        start: Vec3,
        dt: f64,
        steps: usize,
    ) -> Vec<(Vec3, Vec3)> {
        let mut materials = MaterialDb::standard();
        let material = floating_material(&mut materials, density);
        let mut rng = sim_math::SimRng::new(1, 1);
        let mut events = sim_core::EventQueue::new();

        // 埋め込み経路(`MechanicsSolver::fluids`)は使わない——二重計上を避け、
        // 検証対象を`BuoyancyDrag`の力積分だけに絞る(モジュールdoc §二重計上)。
        let mut mechanics = MechanicsSolver::new(0.0);
        mechanics.set_gravity_field(field);
        let mut desc = RigidBodyDesc::dynamic(Shape::Box { half_extents }, material);
        desc.mass_override = Some(mass);
        desc.transform.position = start;
        let body = mechanics.create_body(desc, &materials);
        let mut coupling = BuoyancyDrag {
            body_index: body,
            water: Some(water),
            atmosphere: None,
            lift: None,
        };

        let mut trajectory = Vec::with_capacity(steps);
        for _ in 0..steps {
            coupling.apply_pre(&mut states(&mut mechanics), dt);
            let mut ctx = sim_core::SolverContext {
                materials: &materials,
                rng: &mut rng,
                events: &mut events,
            };
            sim_core::Solver::step(&mut mechanics, dt, &mut ctx);
            let _ = events.drain_sorted();
            trajectory.push((
                mechanics.bodies.position[body],
                mechanics.bodies.linear_velocity[body],
            ));
        }
        trajectory
    }

    /// 既定の重力(`-y`向き一様)での`simulate_floating_box_in_field`
    /// ——鉛直運動だけを見る既存2テストのための薄い包み。
    #[allow(clippy::too_many_arguments)]
    fn simulate_floating_box(
        half_extents: Vec3,
        mass: f64,
        density: f64,
        water: FluidRegion,
        gravity: f64,
        start_y: f64,
        dt: f64,
        steps: usize,
    ) -> Vec<(f64, f64)> {
        simulate_floating_box_in_field(
            half_extents,
            mass,
            density,
            water,
            GravityField::Uniform {
                magnitude: gravity,
                direction: Vec3::new(0.0, -1.0, 0.0),
            },
            Vec3::new(0.0, start_y, 0.0),
            dt,
            steps,
        )
        .into_iter()
        .map(|(position, velocity)| (position.y, velocity.y))
        .collect()
    }

    /// **アルキメデスの原理による釣り合い喫水**。質量 $m$・水線面積 $A$ の
    /// 直立直方体は $\rho_f\,g\,A\,d = m\,g$ すなわち $d = m/(\rho_f A)$ の
    /// 深さまで沈む(`sim_fluid::buoyancy`の`equilibrium_draft_matches_archimedes_closed_form`
    /// が同じ式を代数で固定している)。
    ///
    /// **このモデルには水中抗力が無い**(`sim_fluid::buoyancy`冒頭注記「水中抗力は
    /// Phase 3」)ので、平衡点から外して離した浮体は減衰せず単振動を続け、
    /// 文字通り「静止」はしない。復元力が喫水に厳密に比例する線形域
    /// (全没も完全露出もしない振幅)に留めれば運動は正確な単振動になるので、
    /// **上下の転回点の中点**が平衡位置そのものになる——これを「静止喫水」として
    /// 解析解と比較する。
    ///
    /// 許容誤差の根拠: dt=1/240・6周期(約2237step)の実測で中点のずれは
    /// 1.5e-6 m(辺長1mの 1.5e-6)。転回点をstep単位でしか拾えないことによる
    /// 量子化(振幅0.1m・周期0.37sなので 1step で最大 ~2e-5 m)が主因なので、
    /// その数倍の 1e-4 m を上限とする。振幅が保存する(数値減衰も励起も無い)
    /// ことも併せて確認する。
    ///
    /// **前提の明示**: 水面がワールドyの水平面で浮力が`+y`固定という現行モデル
    /// (`sim_fluid::buoyancy`のdoc)に依存する。浮力が重力場の局所的な向きへ
    /// 追従するようになったら「ワールドyが下向き」という前提は保証されなくなり、
    /// 本テストにも重力方向の注記が要る。
    #[test]
    fn floating_box_settles_at_the_archimedes_equilibrium_draft() {
        let gravity = 9.80665;
        let water_density = 998.2;
        let ratio = 0.6;
        let half_extents = Vec3::new(0.5, 0.5, 0.5);
        let side = 1.0_f64;
        let waterline_area = side * side;
        let mass = ratio * water_density * side.powi(3);
        let water = FluidRegion::new(0.0, water_density);

        // 解析解: d = m/(ρ_f A)、箱中心はそこから半分の高さだけ上。
        let draft = mass / (water_density * waterline_area);
        assert!(
            (draft - ratio * side).abs() < 1e-12,
            "一様密度の直方体では喫水=密度比×辺長になる: {draft}"
        );
        let equilibrium_y = -draft + half_extents.y;

        // 復元力の角振動数 ω=√(ρ_f g A / m) から周期を出し、6周期ぶん回す。
        let omega = (water_density * gravity * waterline_area / mass).sqrt();
        let period = 2.0 * std::f64::consts::PI / omega;
        let dt = 1.0 / 240.0;
        let amplitude = 0.1; // 全没(0.4m)にも完全露出(0.6m)にも届かない線形域
        let steps = ((6.0 * period) / dt) as usize;
        let trajectory = simulate_floating_box(
            half_extents,
            mass,
            ratio * water_density,
            water,
            gravity,
            equilibrium_y + amplitude,
            dt,
            steps,
        );

        let y_min = trajectory.iter().map(|(y, _)| *y).fold(f64::MAX, f64::min);
        let y_max = trajectory.iter().map(|(y, _)| *y).fold(f64::MIN, f64::max);
        let midpoint = 0.5 * (y_min + y_max);
        assert!(
            (midpoint - equilibrium_y).abs() < 1e-4,
            "the mean floating depth must match Archimedes' equilibrium draft: \
             midpoint={midpoint} equilibrium_y={equilibrium_y} (draft={draft})"
        );
        // 線形域に留まっている(全没・完全露出していない)ことの確認。
        assert!(
            y_min > -half_extents.y && y_max + half_extents.y < 2.0 * half_extents.y,
            "the box must stay partially submerged: y_min={y_min} y_max={y_max}"
        );
        // 数値減衰も励起も無い(振幅が保存する)。
        let measured_amplitude = 0.5 * (y_max - y_min);
        assert!(
            (measured_amplitude - amplitude).abs() / amplitude < 1e-3,
            "there is no hydrodynamic damping in this model, so the amplitude must be \
             preserved: measured={measured_amplitude} initial={amplitude}"
        );
    }

    /// **力学的エネルギー保存**(`apply_pre`の力積分方式を直接速度キックへ
    /// 書き換える将来の変更が越えてはならない基準線)。抗力ゼロ・静水では
    /// 散逸源が存在しないので、
    /// $E = \frac12 m v^2 + m g y + U_b$ は保存量でなければならない。
    /// 浮力ポテンシャル $U_b$ は水面下体積が喫水の一次関数になる線形域でも
    /// 閉形式を書き下す代わりに、軌道に沿って浮力の仕事
    /// $W=\int F_b\,\mathrm{d}y$ を台形則で積算して $U_b=-W$ とする
    /// (積分誤差を抗力の散逸が隠してしまう心配が無いのがこの設定の要点)。
    ///
    /// 許容誤差の根拠: `apply_pre`(速度更新)→`step`(位置更新)の順序は
    /// symplectic Euler と同型なので、エネルギー誤差は $O(\Delta t)$ で**有界に
    /// 振動する**(単調増加しない)。運動エネルギーのスケール
    /// $\frac12 m(\omega A)^2$ に対する実測の最大偏差は dt=1/120 で 1.7e-2、
    /// dt=1/240 で 8.5e-3 と刻みに比例する。dt=1/120 を基準に 3e-2 を上限とし、
    /// **さらに前半20周期と後半20周期で最大偏差が変わらない**(=系統的な増大が
    /// 無い)ことを要求する。単調に発散する陽的スキームへ差し替えられたら
    /// この後半の条件が破れる。
    #[test]
    fn buoyancy_force_integration_conserves_mechanical_energy_without_drag() {
        let gravity = 9.80665;
        let water_density = 998.2;
        let ratio = 0.6;
        let half_extents = Vec3::new(0.5, 0.5, 0.5);
        let side = 1.0_f64;
        let waterline_area = side * side;
        let mass = ratio * water_density * side.powi(3);
        let water = FluidRegion::new(0.0, water_density);
        let equilibrium_y = -(mass / (water_density * waterline_area)) + half_extents.y;

        let omega = (water_density * gravity * waterline_area / mass).sqrt();
        let period = 2.0 * std::f64::consts::PI / omega;
        let dt = 1.0 / 120.0;
        let amplitude = 0.1;
        let periods = 40.0;
        let steps = ((periods * period) / dt) as usize;
        let start_y = equilibrium_y + amplitude;
        let trajectory = simulate_floating_box(
            half_extents,
            mass,
            ratio * water_density,
            water,
            gravity,
            start_y,
            dt,
            steps,
        );

        // 高さ y のときに働く浮力(鉛直成分)。
        let buoyancy_at = |y: f64| -> f64 {
            let (v_sub, _) = sim_fluid::submerged_box_axis_aligned(
                Vec3::new(0.0, y, 0.0),
                half_extents,
                water.water_level,
            );
            sim_fluid::buoyancy_force(v_sub, water.density, gravity, UP_Y).y
        };
        let energy = |y: f64, vy: f64, work: f64| 0.5 * mass * vy * vy + mass * gravity * y - work;

        // 誤差の基準スケール: この振動の運動エネルギー振幅。
        let kinetic_scale = 0.5 * mass * (omega * amplitude).powi(2);
        let initial_energy = energy(start_y, 0.0, 0.0);
        let mut work = 0.0;
        let mut previous_y = start_y;
        let mut max_drift_first_half: f64 = 0.0;
        let mut max_drift_second_half: f64 = 0.0;
        for (index, &(y, vy)) in trajectory.iter().enumerate() {
            // 浮力の仕事を台形則で積算する(浮力は y のみの関数)。
            work += 0.5 * (buoyancy_at(previous_y) + buoyancy_at(y)) * (y - previous_y);
            previous_y = y;
            let drift = (energy(y, vy, work) - initial_energy).abs() / kinetic_scale;
            if index < trajectory.len() / 2 {
                max_drift_first_half = max_drift_first_half.max(drift);
            } else {
                max_drift_second_half = max_drift_second_half.max(drift);
            }
        }

        assert!(
            max_drift_first_half < 3e-2 && max_drift_second_half < 3e-2,
            "mechanical energy must stay bounded without drag: \
             first={max_drift_first_half:.3e} second={max_drift_second_half:.3e}"
        );
        // 後半で誤差が増えていない = 有界振動であって系統的なドリフトではない。
        assert!(
            max_drift_second_half <= max_drift_first_half * 1.05,
            "energy error must not grow secularly over {periods} periods: \
             first={max_drift_first_half:.3e} second={max_drift_second_half:.3e}"
        );
    }

    // ------------------------------------------------------------------
    // **重力追従増分**: 浮力が`GravityField`へ追従する。
    // ------------------------------------------------------------------

    /// **傾いた一様重力での釣り合い**。重力をワールド鉛直から30°傾けると、
    /// 自由表面もその`up`(=重力の逆向き)に垂直な平面になり、浮体は
    /// **傾いた`up`軸に沿って**釣り合う——移行前は水面がワールドyの水平面に
    /// 固定だったので、この配置は再現できなかった。
    ///
    /// # 解析解
    ///
    /// 一辺1mの立方体を`up`が30°傾いた向きで切ると、水面が箱の側面を横切る帯の
    /// 中では断面積が $A=1/\cos30°$ で一定になり(`sim_fluid::buoyancy`の
    /// `a_tilted_waterline_cuts_a_constant_cross_section_band_of_the_box`が
    /// 固定している)、水面下体積は箱の`up`座標 $\sigma$ の一次関数
    /// $V(\sigma) = V_0/2 - \sigma A$ になる。したがって
    ///
    /// - 釣り合い: $\rho_f g V(\sigma_{eq}) = m g$ すなわち
    ///   $\sigma_{eq} = (V_0/2 - m/\rho_f)/A$(移行前と同じアルキメデスの
    ///   閉形式を、y軸ではなく`up`軸へ射影しただけ)。**$g$には依存しない**。
    /// - 復元力は $\sigma$ に厳密比例するので運動は単振動になり、
    ///   $\omega=\sqrt{\rho_f g A/m}$、上下の転回点の中点がそのまま平衡位置。
    ///
    /// 水中抗力がまだ無い(`sim_fluid::buoyancy`冒頭注記)ので文字通りの「静止」は
    /// しない点、許容誤差が転回点のstep量子化で決まる点は、鉛直版の
    /// `floating_box_settles_at_the_archimedes_equilibrium_draft`と同じ
    /// (振幅0.05m・周期1.45s・dt=1/240なので1stepあたり最大~1e-5m、その数倍の
    /// 1e-4 mを上限とする)。
    #[test]
    fn floating_box_under_tilted_gravity_settles_along_the_tilted_up_axis() {
        let gravity = 9.80665;
        let water_density = 998.2;
        let ratio = 0.6;
        let half_extents = Vec3::new(0.5, 0.5, 0.5);
        let volume = 1.0_f64; // 一辺1m
        let mass = ratio * water_density * volume;

        let tilt = 30.0_f64.to_radians();
        let up = Vec3::new(tilt.sin(), tilt.cos(), 0.0);
        let field = GravityField::Uniform {
            magnitude: gravity,
            direction: Vec3::ZERO - up,
        };
        // `water_level`は`up`軸に沿った符号付き距離(原点基準)。
        let water = FluidRegion::new(0.0, water_density);

        // 帯の中の断面積と、釣り合いの`up`座標。
        let cross_section = 1.0 / tilt.cos();
        let sigma_eq = (0.5 * volume - mass / water_density) / cross_section;
        // 釣り合いが「断面積一定の帯」の内側にあること(単振動になる前提)。
        let band = (0.5 * tilt.sin() - 0.5 * tilt.cos()).abs();
        let amplitude = 0.05;
        assert!(
            sigma_eq.abs() + amplitude < band,
            "sigma_eq={sigma_eq} amplitude={amplitude} band={band}"
        );

        // 力の釣り合いを閉形式で直接確認する(平衡位置での浮力 = 重量、向きは`up`)。
        let (v_eq, _) = sim_fluid::submerged_box_below_plane(
            up.scale(sigma_eq),
            half_extents,
            up,
            water.water_level,
        );
        let buoyancy = sim_fluid::buoyancy_force(v_eq, water_density, gravity, up);
        assert!(
            (buoyancy.length() - mass * gravity).abs() / (mass * gravity) < 1e-12,
            "平衡位置では浮力の大きさが重量に一致する: |F_b|={} mg={}",
            buoyancy.length(),
            mass * gravity
        );
        assert!(
            (buoyancy.normalize_or_zero() - up).length() < 1e-12,
            "浮力は傾いた`up`向き: {buoyancy:?}"
        );

        let omega = (water_density * gravity * cross_section / mass).sqrt();
        let period = 2.0 * std::f64::consts::PI / omega;
        let dt = 1.0 / 240.0;
        let steps = ((6.0 * period) / dt) as usize;
        let trajectory = simulate_floating_box_in_field(
            half_extents,
            mass,
            ratio * water_density,
            water,
            field,
            up.scale(sigma_eq + amplitude),
            dt,
            steps,
        );

        // 運動は`up`軸上に閉じる(浮力も重力も`up`に平行なので、横成分は出ない)。
        let max_lateral = trajectory
            .iter()
            .map(|(p, _)| (*p - up.scale(p.dot(up))).length())
            .fold(0.0_f64, f64::max);
        assert!(max_lateral < 1e-12, "max_lateral={max_lateral}");

        let sigma: Vec<f64> = trajectory.iter().map(|(p, _)| p.dot(up)).collect();
        let min = sigma.iter().copied().fold(f64::MAX, f64::min);
        let max = sigma.iter().copied().fold(f64::MIN, f64::max);
        let midpoint = 0.5 * (min + max);
        assert!(
            (midpoint - sigma_eq).abs() < 1e-4,
            "傾いた`up`軸に沿った平均喫水がアルキメデスの閉形式に一致する: \
             midpoint={midpoint} sigma_eq={sigma_eq}"
        );
        // 数値減衰も励起も無い(振幅が保存する)。
        assert!(
            (0.5 * (max - min) - amplitude).abs() / amplitude < 1e-3,
            "measured={} initial={amplitude}",
            0.5 * (max - min)
        );

        // **移行前のモデルでは出せない結果であること**: 平衡位置はワールドyだけで
        // なくxにもずれる(水面が傾いているから)。浮力を`+y`固定にした実装では
        // x方向の力が一切生じないので、ここは厳密に0になってしまう。
        let equilibrium_x = up.x * midpoint;
        assert!(
            (equilibrium_x - tilt.sin() * sigma_eq).abs() < 1e-4,
            "equilibrium_x={equilibrium_x}"
        );
        assert!(
            equilibrium_x.abs() > 0.04,
            "傾いた重力では平衡位置がx方向にもずれる: equilibrium_x={equilibrium_x}"
        );
        // 実際に軌道がx方向へ動いていること(初期位置に貼り付いていない)。
        let x_span = trajectory.iter().map(|(p, _)| p.x).fold(f64::MIN, f64::max)
            - trajectory.iter().map(|(p, _)| p.x).fold(f64::MAX, f64::min);
        assert!(
            (x_span - 2.0 * amplitude * up.x).abs() < 1e-3,
            "x_span={x_span}"
        );
    }

    /// `GravityField::Zero`では浮力は**厳密に0**——「下」が無い場所に水平な
    /// 自由表面は定義できず、押しのけた流体の重量も0だから
    /// (`sim_mechanics::GravityField::up_and_magnitude_at`が`None`を返す)。
    /// **意図した縮退であって取りこぼしではない**ことを、同じ配置で重力がある
    /// 場合との対比で固定する。
    #[test]
    fn zero_gravity_field_produces_exactly_zero_buoyancy() {
        let materials = MaterialDb::standard();
        let wood = materials.find_by_name("木材(松)").unwrap();
        let half_extents = Vec3::new(0.5, 0.5, 0.5);

        let impulse = |field: GravityField| -> Vec3 {
            let mut desc = RigidBodyDesc::dynamic(Shape::Box { half_extents }, wood);
            desc.mass_override = Some(10.0);
            desc.transform.position = Vec3::new(0.0, -5.0, 0.0); // 全没
            let mut mechanics = MechanicsSolver::new(0.0);
            mechanics.set_gravity_field(field);
            let body = mechanics.create_body(desc, &materials);
            let mut coupling = BuoyancyDrag {
                body_index: body,
                water: Some(FluidRegion::new(0.0, 1000.0)),
                atmosphere: None,
                lift: None,
            };
            let before = mechanics.bodies.linear_velocity[body];
            // `apply_pre`だけを見る(重力は`MechanicsSolver::step`が積むので、
            // ここに現れるのは浮力のぶんだけ)。
            coupling.apply_pre(&mut states(&mut mechanics), 0.01);
            mechanics.bodies.linear_velocity[body] - before
        };

        assert_eq!(
            impulse(GravityField::Zero),
            Vec3::ZERO,
            "無重力場では浮力を出さない"
        );
        // 大きさ0の`Uniform`場も同じ(向きはあるが力が0)。
        assert_eq!(
            impulse(GravityField::Uniform {
                magnitude: 0.0,
                direction: Vec3::new(0.0, -1.0, 0.0),
            }),
            Vec3::ZERO
        );
        // 同じ配置で重力があれば確かに浮力が出る(上の0が「たまたま」ではない)。
        let with_gravity = impulse(GravityField::Uniform {
            magnitude: 9.80665,
            direction: Vec3::new(0.0, -1.0, 0.0),
        });
        assert!(with_gravity.y > 0.0, "{with_gravity:?}");
    }

    /// `GravityField::PointSource`では浮力が**中心から外向き**に働く。
    /// 移行前は`gravity()`が0.0を返して浮力が丸ごと消えていた場である。
    /// 中心を通る自由表面はどんな向きでも箱をちょうど半分にするので、
    /// 大きさは $\rho_f\,(\mu/r^2)\,V/2$ で閉じる。
    #[test]
    fn point_source_gravity_pushes_buoyancy_away_from_the_centre() {
        let materials = MaterialDb::standard();
        let wood = materials.find_by_name("木材(松)").unwrap();
        let half_extents = Vec3::new(0.5, 0.5, 0.5);
        let mass = 10.0;
        let position = Vec3::new(3.0, 4.0, 0.0);
        let radius = 5.0;
        let mu = 245.0; // mu/r^2 = 9.8 m/s^2
        let up = position.scale(1.0 / radius);
        let density = 1000.0;
        let dt = 0.01;

        let mut desc = RigidBodyDesc::dynamic(Shape::Box { half_extents }, wood);
        desc.mass_override = Some(mass);
        desc.transform.position = position;
        let mut mechanics = MechanicsSolver::new(0.0);
        mechanics.set_gravity_field(GravityField::PointSource {
            center: Vec3::ZERO,
            mu,
        });
        let body = mechanics.create_body(desc, &materials);
        // 自由表面は「原点から`up`軸に沿って`radius`」= 箱の中心を通る平面。
        let mut coupling = BuoyancyDrag {
            body_index: body,
            water: Some(FluidRegion::new(radius, density)),
            atmosphere: None,
            lift: None,
        };
        let before = mechanics.bodies.linear_velocity[body];
        coupling.apply_pre(&mut states(&mut mechanics), dt);
        let impulse = mechanics.bodies.linear_velocity[body] - before;

        let expected = up.scale(density * (mu / (radius * radius)) * 0.5 * dt / mass);
        assert!(
            (impulse - expected).length() < 1e-12,
            "impulse={impulse:?} expected={expected:?}"
        );
        // 中心から外向き、かつワールド+y向きではない(点源場の「上」は位置依存)。
        assert!(impulse.dot(up) > 0.0, "{impulse:?}");
        assert!(impulse.x > 0.0, "{impulse:?}");
    }
}
