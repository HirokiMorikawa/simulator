//! `GridFluidRigid`(設計 docs/20-integration/01-coupling-matrix.md §3「P3: 格子流体 ⇔
//! 剛体(ボクセル化境界・圧力積分)」)。
//!
//! **縮約実装の理由**: `sim_fluid::GridFluid2D`は2D格子流体なので、この結合も剛体の
//! x-y平面内の運動(`sim_mechanics`の3D剛体のz座標・z速度は無視する)のみを扱う。
//! 剛体形状は軸並行の矩形(`sim_fluid::GridSolidBox`、回転を反映しない)に限定する —
//! 設計は任意形状のボクセル化境界を想定するが、この縮約は`sim_fluid::GridFluidRigidBox2D`
//! (X2)が既に採用した縮約(マスキング方式、cut-cell法ではない)と同じ。
//!
//! 双方向性は`SphRigid`・`InductionCoupling`と同じ1step遅れの縮約で実現する(設計§2規則3
//! 「各ステップで前ステップ確定値を読む」と整合): `apply`は、(1)前stepの`GridFluid2D::step`
//! が計算した圧力場から剛体表面の圧力積分力(`GridFluid2D::pressure_force_on_solid`)を
//! 読み出して剛体速度に注入し、(2)続けて剛体の(今stepの)位置・速度を`GridFluid2D::solid`
//! へ書き込む(次stepの`GridFluid2D::step`内のマスキングに反映される)。
//!
//! **検証方針**: `SphRigid`実装検証時に確立したパターンを踏襲する。マスキング+圧力積分
//! による流体力抽出という手法自体の物理的妥当性は、同じ手法を使う
//! `sim_fluid::GridFluidRigidBox2D`の既存テスト(X2、ばね-質量系の固有振動数との一致)が
//! 既に検証済みなので、本モジュールのテストは`GridFluidRigid`自身の配管ロジック(剛体への
//! 力の注入・`solid`マスクの位置/速度追従)を既知の(手で設定した)圧力場・`solid`値で
//! 決定論的に検証する(密な剛体をバルク流体に沈める動的シナリオが`SphRigid`実装検証中に
//! 招いたSPH特有の縁効果の再発を避ける判断)。

use crate::domain_states::{Coupling, DomainStates};
use sim_core::DomainId;
use sim_fluid::GridSolidBox;
use sim_math::Vec3;

/// 軸並行矩形剛体(`body_index`)を、`GridFluid2D::solid`のマスキング機構経由で
/// 格子流体に結合する(モジュールdoc参照)。
#[derive(Clone)]
pub struct GridFluidRigid {
    pub body_index: usize,
    pub half_width: f64,
    pub half_height: f64,
}

impl GridFluidRigid {
    pub fn new(body_index: usize, half_width: f64, half_height: f64) -> GridFluidRigid {
        GridFluidRigid {
            body_index,
            half_width,
            half_height,
        }
    }
}

impl Coupling for GridFluidRigid {
    fn domains(&self) -> (DomainId, DomainId) {
        (DomainId::Mechanics, DomainId::Fluid)
    }

    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        let mass = world.mechanics.bodies.mass(self.body_index);
        if mass <= 0.0 {
            return; // 静的/キネマティック剛体には適用しない。
        }
        let Some(grid) = &mut world.grid_fluid else {
            return;
        };

        // 反作用: 前stepのGridFluid2Dステップで確定した圧力場からの面積分力を
        // 剛体へ適用(モジュールdoc「1step遅れ」参照)。
        if let Some(force) = grid.pressure_force_on_solid() {
            let idx = self.body_index;
            world.mechanics.bodies.linear_velocity[idx] =
                world.mechanics.bodies.linear_velocity[idx] + force.scale(dt / mass);
        }

        // 剛体マスクを今stepの剛体位置・速度に更新(次stepのGridFluid2Dステップの
        // マスキングに反映される)。
        let idx = self.body_index;
        let pos = world.mechanics.bodies.position[idx];
        let vel = world.mechanics.bodies.linear_velocity[idx];
        grid.solid = Some(GridSolidBox {
            center: (pos.x, pos.y),
            half_width: self.half_width,
            half_height: self.half_height,
            velocity: Vec3::new(vel.x, vel.y, 0.0),
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::MaterialDb;
    use sim_fluid::GridFluid2D;
    use sim_mechanics::{MechanicsSolver, RigidBodyDesc, Shape};

    /// `GridFluidRigid`自身の配管ロジック(圧力積分力の注入・`solid`マスクの位置/速度
    /// 追従)を、既知の(手で設定した)圧力場を使って決定論的に検証する(`SphRigid`と同じ
    /// パターン、モジュールdoc参照)。
    #[test]
    fn grid_fluid_rigid_applies_known_pressure_force_and_tracks_body_position_and_velocity() {
        let nx = 8;
        let ny = 8;
        let h = 0.5;
        let mut grid = GridFluid2D::new(nx, ny, h);
        // 既知の線形圧力場 p(i,j)=3i+2j (grid_fluid.rsの同種テストと同じ値を使い回す)。
        for j in 0..ny {
            for i in 0..nx {
                grid.last_pressure[i + nx * j] = 3.0 * i as f64 + 2.0 * j as f64;
            }
        }

        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mass = 2.0;
        let half_width = 0.75;
        let half_height = 0.75;
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.2 }, steel);
        desc.mass_override = Some(mass);
        desc.transform.position = Vec3::new(2.0, 2.0, 0.0);
        let mut mechanics = MechanicsSolver::new(0.0);
        let body = mechanics.create_body(desc, &materials);

        // 「前stepで確定したsolidマスク」を手で設定する(1step遅れの縮約、モジュールdoc
        // 参照)。剛体の現在位置と同じ場所に置くことで、grid_fluid.rsの
        // `pressure_force_on_solid_integrates_a_known_linear_pressure_field`と同一の
        // ジオメトリ・圧力場になり、期待force=(-9.0,-6.0,0.0)を再利用できる。
        grid.solid = Some(sim_fluid::GridSolidBox {
            center: (2.0, 2.0),
            half_width,
            half_height,
            velocity: Vec3::ZERO,
        });

        let mut coupling = GridFluidRigid::new(body, half_width, half_height);
        let expected_force = Vec3::new(-9.0, -6.0, 0.0);

        let dt = 0.01;
        let velocity_before = mechanics.bodies.linear_velocity[body];
        {
            let mut states = DomainStates {
                mechanics: &mut mechanics,
                thermal: None,
                em_circuit: None,
                em_electrostatics: None,
                gas: None,
                grid_fluid: Some(&mut grid),
                sph: None,
            };
            coupling.apply(&mut states, dt);
        }

        // (1) 圧力積分力がF*dt/massだけ速度に注入されていること。
        let expected_velocity = velocity_before + expected_force.scale(dt / mass);
        let measured_velocity = mechanics.bodies.linear_velocity[body];
        assert!(
            (measured_velocity - expected_velocity).length() < 1e-12,
            "measured_velocity={measured_velocity:?} expected_velocity={expected_velocity:?}"
        );

        // (2) `solid`マスクが今stepの剛体位置・(注入後の)速度へ追従していること。
        let body_pos = mechanics.bodies.position[body];
        let body_vel = mechanics.bodies.linear_velocity[body];
        let solid = grid.solid.expect("apply should have set solid");
        assert!((solid.center.0 - body_pos.x).abs() < 1e-12);
        assert!((solid.center.1 - body_pos.y).abs() < 1e-12);
        assert_eq!(solid.half_width, half_width);
        assert_eq!(solid.half_height, half_height);
        assert!((solid.velocity.x - body_vel.x).abs() < 1e-12);
        assert!((solid.velocity.y - body_vel.y).abs() < 1e-12);
        assert_eq!(solid.velocity.z, 0.0);
    }

    /// 質量0以下(静的/キネマティック)の剛体には適用しない(他のCoupling実装と同じ
    /// ガード、`SphRigid`・`LorentzForce`等参照)。
    #[test]
    fn grid_fluid_rigid_does_nothing_for_a_static_body() {
        let mut grid = GridFluid2D::new(8, 8, 0.5);
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.2 }, steel);
        desc.body_type = sim_mechanics::BodyType::Static;
        desc.transform.position = Vec3::new(2.0, 2.0, 0.0);
        let mut mechanics = MechanicsSolver::new(0.0);
        let body = mechanics.create_body(desc, &materials);

        let mut coupling = GridFluidRigid::new(body, 0.75, 0.75);
        let mut states = DomainStates {
            mechanics: &mut mechanics,
            thermal: None,
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: Some(&mut grid),
            sph: None,
        };
        coupling.apply(&mut states, 0.01);

        assert!(
            grid.solid.is_none(),
            "static body: early return, solid mask is never set"
        );
    }
}
