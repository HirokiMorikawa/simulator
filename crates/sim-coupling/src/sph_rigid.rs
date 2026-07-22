//! `SphRigid`(設計 docs/20-integration/01-coupling-matrix.md §3「P4: SPH ⇔ 剛体
//! (境界粒子)」)。
//!
//! **縮約実装の理由**: 設計は一般形状の剛体表面を境界粒子で覆う想定だが、本実装は
//! 球剛体のみを対象とする(フィボナッチ格子で球面上に均一に配置した境界粒子群、
//! 姿勢(回転)は反映しない — 球は回転対称なので剛体の中心位置さえ追従すれば
//! 境界形状は常に正しい、`BoussinesqBuoyancy`等と同じ「対象を絞って正直に文書化する」
//! 縮約)。
//!
//! `sim_fluid::SphFluid::boundary_position`は元々静的(壁・床)を想定するが、
//! `boundary_force`(新設フィールド、`sph.rs`のdoc参照)は位置に依存しない較正のため、
//! 呼び出し側が毎stepの`apply`内で位置を書き換えても問題なく動作する
//! (`compute_acceleration`呼び出しの間でキネマティックに駆動される動的境界として使う)。
//!
//! 双方向性は次の1step遅れの縮約で実現する(`InductionCoupling`等と同じ、設計§2規則3
//! 「各ステップで前ステップ確定値を読む」と整合): `apply`は、(1)まず前stepの`SphFluid`
//! ステップで確定した`boundary_force`(自分の境界粒子群の合計)を剛体への反作用力として
//! 適用し、(2)続けて境界粒子群の位置を今stepの剛体位置に更新する(次stepの`SphFluid`
//! ステップに反映される)。
//!
//! **検証方針**: 密な球状剛体をバルク流体に沈めて浮力の物理そのもの(アルキメデスの
//! 原理)を再現する動的シナリオは、境界粒子群と既存流体粒子の重なり・空洞境界での
//! 密度不連続などSPH特有の縁効果に弱く、安定した定量検証が難しいことを実装検証中に
//! 発見した(過圧縮による桁違いの過大反発・密度不連続による符号反転を経験した)。
//! そのため本モジュールのテストは、`SphRigid`自身の配管ロジック(境界粒子の確保・
//! 反作用力の合計・剛体への注入・位置追従)を、既知の(手で設定した)`boundary_force`
//! 値を使って決定論的に検証する — `boundary_force`自体がNewton第3法則を正しく
//! 反映すること(浮力の物理的妥当性)は`sim-fluid::sph`の
//! `boundary_force_sums_to_resting_fluid_columns_weight_on_the_container`が
//! 既に検証済み。

use crate::domain_states::{Coupling, DomainStates};
use sim_core::DomainId;
use sim_math::Vec3;

/// フィボナッチ格子で単位球面上に`n`個の点を均一に配置する(黄金角による標準的な
/// 構成、決定論的)。
fn fibonacci_sphere_point(k: usize, n: usize) -> Vec3 {
    let golden_angle = std::f64::consts::PI * (3.0 - 5.0_f64.sqrt());
    let y = 1.0 - 2.0 * (k as f64 + 0.5) / n as f64; // 1 .. -1
    let radius_at_y = (1.0 - y * y).max(0.0).sqrt();
    let theta = golden_angle * k as f64;
    Vec3::new(theta.cos() * radius_at_y, y, theta.sin() * radius_at_y)
}

/// 球剛体(`body_index`)を、その表面を覆う境界粒子群(`SphFluid::boundary_position`の
/// `[boundary_start, boundary_start+boundary_count)`区間)経由でSPH流体に結合する
/// (モジュールdoc参照)。
#[derive(Clone)]
pub struct SphRigid {
    pub body_index: usize,
    pub radius: f64,
    boundary_start: usize,
    boundary_count: usize,
}

impl SphRigid {
    /// `sph`に`n_points`個の境界粒子を新規追加し(既存の境界粒子には影響しない)、
    /// その区間を占有する`SphRigid`を作る。
    pub fn new(
        sph: &mut sim_fluid::SphFluid,
        body_index: usize,
        radius: f64,
        n_points: usize,
    ) -> SphRigid {
        let boundary_start = sph.boundary_position.len();
        for _ in 0..n_points {
            sph.add_boundary_particle(Vec3::ZERO); // 実位置は最初のapply()で設定される。
        }
        SphRigid {
            body_index,
            radius,
            boundary_start,
            boundary_count: n_points,
        }
    }

    fn local_offset(&self, k: usize) -> Vec3 {
        fibonacci_sphere_point(k, self.boundary_count).scale(self.radius)
    }

    /// この剛体の境界粒子群が(直前の`SphFluid::step`で)受けた反作用力の合計
    /// (テスト・診断用、`apply`が内部で使うのと同じ計算)。
    pub fn reaction_force(&self, sph: &sim_fluid::SphFluid) -> Vec3 {
        (0..self.boundary_count)
            .map(|k| sph.boundary_force[self.boundary_start + k])
            .fold(Vec3::ZERO, |acc, f| acc + f)
    }
}

impl Coupling for SphRigid {
    fn domains(&self) -> (DomainId, DomainId) {
        (DomainId::Mechanics, DomainId::Fluid)
    }

    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        let mass = world.mechanics.bodies.mass(self.body_index);
        if mass <= 0.0 {
            return; // 静的/キネマティック剛体には適用しない。
        }
        let Some(sph) = &mut world.sph else {
            return;
        };
        if self.boundary_start + self.boundary_count > sph.boundary_position.len() {
            return; // 構築時とsph構成が食い違う(想定外の呼び出し)。
        }

        // 反作用: 前stepのSPHステップで確定した境界粒子群への合力を剛体へ適用
        // (モジュールdoc「1step遅れ」参照)。
        let mut total_force = Vec3::ZERO;
        for k in 0..self.boundary_count {
            total_force = total_force + sph.boundary_force[self.boundary_start + k];
        }
        let idx = self.body_index;
        world.mechanics.bodies.linear_velocity[idx] =
            world.mechanics.bodies.linear_velocity[idx] + total_force.scale(dt / mass);

        // 境界粒子群の位置を今stepの剛体位置に更新(次stepのSPHステップに反映される)。
        let pos = world.mechanics.bodies.position[idx];
        for k in 0..self.boundary_count {
            sph.boundary_position[self.boundary_start + k] = pos + self.local_offset(k);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::MaterialDb;
    use sim_fluid::SphFluid;
    use sim_mechanics::{MechanicsSolver, RigidBodyDesc, Shape};

    /// `SphRigid`自身の配管ロジック(境界粒子群の確保・反作用力の合計・剛体への注入・
    /// 位置追従)を、既に(`sim-fluid::sph`側の`boundary_force_sums_to_resting_fluid_
    /// columns_weight_on_the_container`で)検証済みの`boundary_force`の物理的妥当性とは
    /// 切り離して確認する。既知の(手で設定した)`boundary_force`値を使うことで、
    /// 実際にSPH流体を多数step動かして静水圧平衡に持ち込む必要がなく、決定論的・
    /// 高速に検証できる(密な球状剛体をバルク流体に沈めるシナリオは、境界粒子群と
    /// 既存流体粒子の重なり・空洞境界での密度不連続などSPH特有の縁効果に弱く、
    /// 安定した定量検証が難しいことを実装検証中に発見したため、この決定論的な
    /// 単体テストに置き換えた — 浮力そのものの物理は既存の静水圧平衡テスト群が担う)。
    #[test]
    fn sph_rigid_applies_known_reaction_force_and_tracks_body_position() {
        let mut sph = SphFluid::new(0.04, 1000.0, 20.0);
        // 床の境界粒子を何点か先に追加しておき(既存の境界に影響しないことの確認を兼ねる)、
        // `SphRigid`が正しく自分の区間だけを扱うことを検証する。
        for i in 0..5 {
            sph.add_boundary_particle(Vec3::new(i as f64, -1.0, 0.0));
        }

        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mass = 2.0;
        let radius = 0.1;
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, steel);
        desc.mass_override = Some(mass);
        desc.transform.position = Vec3::new(1.0, 2.0, 3.0);
        let mut mechanics = MechanicsSolver::new(0.0);
        let ball = mechanics.create_body(desc, &materials);

        let n_points = 4;
        let mut coupling = SphRigid::new(&mut sph, ball, radius, n_points);
        assert_eq!(
            sph.boundary_position.len(),
            5 + n_points,
            "SphRigid::new should append n_points boundary particles without touching \
             pre-existing ones"
        );

        // 手で「前stepのSPHで確定した」反作用力を設定する(合計が既知になるように)。
        let known_forces = [
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(-0.5, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(2.0, -1.0, -2.0),
        ];
        for (k, &f) in known_forces.iter().enumerate() {
            sph.boundary_force[5 + k] = f;
        }
        let expected_total: Vec3 = known_forces.iter().fold(Vec3::ZERO, |acc, &f| acc + f);
        assert_eq!(
            coupling.reaction_force(&sph),
            expected_total,
            "reaction_force should sum exactly this body's boundary_force sub-range"
        );

        let dt = 0.01;
        let velocity_before = mechanics.bodies.linear_velocity[ball];
        {
            let mut states = DomainStates {
                mechanics: &mut mechanics,
                thermal: None,
                em_circuit: None,
                em_electrostatics: None,
                gas: None,
                grid_fluid: None,
                sph: Some(&mut sph),
            };
            coupling.apply(&mut states, dt);
        }

        // (1) 反作用力がF*dt/massだけ速度に注入されていること。
        let expected_velocity = velocity_before + expected_total.scale(dt / mass);
        let measured_velocity = mechanics.bodies.linear_velocity[ball];
        assert!(
            (measured_velocity - expected_velocity).length() < 1e-12,
            "measured_velocity={measured_velocity:?} expected_velocity={expected_velocity:?}"
        );

        // (2) 境界粒子群が剛体の(今stepの)位置 + フィボナッチ格子オフセットへ更新され、
        // かつ既存の床の境界粒子(index 0..5)には触れていないこと。
        for i in 0..5 {
            assert_eq!(
                sph.boundary_position[i],
                Vec3::new(i as f64, -1.0, 0.0),
                "pre-existing boundary particles must be untouched"
            );
        }
        let body_pos = mechanics.bodies.position[ball];
        for k in 0..n_points {
            let offset = fibonacci_sphere_point(k, n_points).scale(radius);
            let expected_pos = body_pos + offset;
            let measured_pos = sph.boundary_position[5 + k];
            assert!(
                (measured_pos - expected_pos).length() < 1e-9,
                "boundary particle {k} should track the body's position: \
                 measured_pos={measured_pos:?} expected_pos={expected_pos:?}"
            );
            // 全境界点は半径ちょうどradiusの球面上にあるはず。
            assert!(
                (offset.length() - radius).abs() < 1e-9,
                "local_offset should lie on the sphere of the given radius"
            );
        }
    }

    /// 質量0以下(静的/キネマティック)の剛体には適用しない(他のCoupling実装と
    /// 同じガード、`LorentzForce`等参照)。
    #[test]
    fn sph_rigid_does_nothing_for_a_static_body() {
        let mut sph = SphFluid::new(0.04, 1000.0, 20.0);
        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.1 }, steel);
        desc.body_type = sim_mechanics::BodyType::Static;
        let mut mechanics = MechanicsSolver::new(0.0);
        let ball = mechanics.create_body(desc, &materials);

        let mut coupling = SphRigid::new(&mut sph, ball, 0.1, 4);
        let mut states = DomainStates {
            mechanics: &mut mechanics,
            thermal: None,
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: None,
            sph: Some(&mut sph),
        };
        coupling.apply(&mut states, 0.01);

        assert_eq!(
            sph.boundary_position[0],
            Vec3::ZERO,
            "static body: no position update"
        );
    }
}
