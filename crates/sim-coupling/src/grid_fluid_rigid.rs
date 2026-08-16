//! `GridFluidRigid`(設計 docs/20-integration/01-coupling-matrix.md §3「P3: 格子流体 ⇔
//! 剛体(ボクセル化境界・圧力積分)」)。
//!
//! **縮約実装の理由**: `sim_fluid::GridFluid2D`は2D格子流体なので、この結合も剛体の
//! x-y平面内の運動(`sim_mechanics`の3D剛体のz座標・z速度は無視する)のみを扱う。
//! 剛体形状は軸並行の矩形(回転を反映しない)に限定する — 設計は任意形状のボクセル化
//! 境界を想定するが、この縮約は`sim_fluid::GridFluidRigidBox2D`(X2)が既に採用した
//! 縮約(マスキング方式、cut-cell法ではない)と同じ。
//!
//! 双方向性は`apply`ひとつで実現する: (1)`GridFluid2D::step`が計算した圧力場から
//! 剛体表面の圧力積分力(`GridFluid2D::pressure_force_on_solid`)を読み出して剛体速度に
//! 注入し、(2)続けて剛体の位置・速度を`GridFluid2D`のセル種別へ矩形としてラスタライズする。
//!
//! **固体表現の一本化に追従した**: 移行前は`GridFluid2D`が`cell_type`とは別に
//! `solid: Option<GridSolidBox>`(単一矩形)という第二の固体表現を持ち、この結合だけが
//! そちらを使っていた。二重表現の「どちらが勝つか」が暗黙だったため`GridSolidBox`ごと
//! 削除し、ここも`set_solid_cells`に矩形判定の閉包を渡す形へ移した。**コストは変わらない**
//! ——移行前の`set_solid_box`も内部で全セルを走査して矩形をラスタライズしており、
//! さらに`GridFluid2D::step`の冒頭が毎step同じラスタライズを繰り返していた(その重複が
//! 無くなったぶん、むしろ1step当たりの全セル走査は2回から1回へ減る)。
//!
//! **群5でこのdocの記述を訂正した**(実装は変えていない)。移行前は上記を「1step遅れの
//! 縮約」と書いていたが、`SphRigid`とまったく同じ理由で**圧力積分力に1step遅れは無い**
//! ——`World::step`は全ドメインソルバ(`grid_fluid`を含む)の後に post 相を呼ぶので、
//! ここで読む圧力場は今stepのものである。残る遅れは固体マスク側だけで、これは
//! 力学ステップがマスク書き込みと流体ステップの間に挟まることに由来し、pre 相へ移しても
//! 変わらない(`SphRigid`のモジュールdocに詳述)。
//!
//! **検証方針**: `SphRigid`実装検証時に確立したパターンを踏襲する。マスキング+圧力積分
//! による流体力抽出という手法自体の物理的妥当性は、同じ手法を使う
//! `sim_fluid::GridFluidRigidBox2D`の既存テスト(X2、ばね-質量系の固有振動数との一致)が
//! 既に検証済みなので、本モジュールのテストは`GridFluidRigid`自身の配管ロジック(剛体への
//! 力の注入・固体マスクの位置/速度追従)を既知の(手で設定した)圧力場・固体マスクで
//! 決定論的に検証する(密な剛体をバルク流体に沈める動的シナリオが`SphRigid`実装検証中に
//! 招いたSPH特有の縁効果の再発を避ける判断)。

use crate::domain_states::{Coupling, CouplingKind, DomainStates};
use sim_core::DomainId;
use sim_math::Vec3;

/// 軸並行矩形剛体(`body_index`)を、`GridFluid2D`のセル種別マスキング機構経由で
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
    fn kind(&self) -> CouplingKind {
        CouplingKind::GridFluidRigid
    }

    fn domain_ids(&self) -> &'static [DomainId] {
        &[DomainId::Mechanics, DomainId::Fluid]
    }

    fn describe(&self) -> String {
        format!(
            "GridFluidRigid body#{} half={}x{}",
            self.body_index, self.half_width, self.half_height
        )
    }

    fn referenced_bodies(&self) -> Vec<usize> {
        vec![self.body_index]
    }

    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        let mass = world.mechanics.bodies.mass(self.body_index);
        if mass <= 0.0 {
            return; // 静的/キネマティック剛体には適用しない。
        }
        let Some(grid) = &mut world.grid_fluid else {
            return;
        };

        // 反作用: 今stepのGridFluid2Dステップで確定した圧力場からの面積分力を
        // 剛体へ適用(`World`は全ドメインソルバの後に post 相を呼ぶ、モジュールdoc参照)。
        if let Some(force) = grid.pressure_force_on_solid() {
            let idx = self.body_index;
            world.mechanics.bodies.linear_velocity[idx] =
                world.mechanics.bodies.linear_velocity[idx] + force.scale(dt / mass);
        }

        // 剛体マスクを今stepの剛体位置・速度に更新(次stepのGridFluid2Dステップの
        // マスキングに反映される)。矩形をセル種別へ直接ラスタライズする——内外判定は
        // セル中心が半幅・半高より真に内側かどうか(移行前の`set_solid_box`と同一)。
        let idx = self.body_index;
        let pos = world.mechanics.bodies.position[idx];
        let vel = world.mechanics.bodies.linear_velocity[idx];
        let solid_velocity = Vec3::new(vel.x, vel.y, 0.0);
        let (half_width, half_height) = (self.half_width, self.half_height);
        grid.set_solid_cells(|x, y| {
            if (x - pos.x).abs() < half_width && (y - pos.y).abs() < half_height {
                Some(solid_velocity)
            } else {
                None
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::MaterialDb;
    use sim_fluid::{CellType, GridFluid2D};
    use sim_mechanics::{MechanicsSolver, RigidBodyDesc, Shape};

    /// 軸並行矩形をセル種別へ敷く(`apply`が使う内外判定と同一)。
    fn rasterize_box(grid: &mut GridFluid2D, center: (f64, f64), half: (f64, f64), vel: Vec3) {
        grid.set_solid_cells(|x, y| {
            if (x - center.0).abs() < half.0 && (y - center.1).abs() < half.1 {
                Some(vel)
            } else {
                None
            }
        });
    }

    /// 「セル種別が中心`center`・半幅`half`・速度`vel`の矩形と厳密に一致すること」を
    /// 検査する。移行前の`solid_box()`(単一矩形をそのまま読み戻す)に代わる検証手段——
    /// 固体はセル種別が唯一の表現になったので、**敷かれた結果**を直接見る。
    fn assert_solid_region_matches_box(
        grid: &GridFluid2D,
        center: (f64, f64),
        half: (f64, f64),
        vel: Vec3,
    ) {
        for j in 0..grid.ny {
            for i in 0..grid.nx {
                let x = (i as f64 + 0.5) * grid.h;
                let y = (j as f64 + 0.5) * grid.h;
                let idx = i + grid.nx * j;
                let expected_solid = (x - center.0).abs() < half.0 && (y - center.1).abs() < half.1;
                let actual_solid = grid.cell_type()[idx] == CellType::Solid;
                assert_eq!(
                    actual_solid, expected_solid,
                    "cell ({i},{j}) at ({x},{y}): expected_solid={expected_solid} \
                     actual_solid={actual_solid} (box center={center:?} half={half:?})"
                );
                if expected_solid {
                    let actual_vel = grid.solid_velocity()[idx];
                    assert!(
                        (actual_vel - vel).length() < 1e-12,
                        "cell ({i},{j}): solid velocity {actual_vel:?} != expected {vel:?}"
                    );
                }
            }
        }
    }

    /// 固体セルがひとつも無いこと。
    fn assert_no_solid_cells(grid: &GridFluid2D) {
        assert!(
            grid.cell_type().iter().all(|c| *c == CellType::Fluid),
            "no cell should have been marked Solid"
        );
    }

    /// `GridFluidRigid`自身の配管ロジック(圧力積分力の注入・固体マスクの位置/速度
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

        // 「今stepの`GridFluid2D::step`が使ったsolidマスク」を手で設定する(モジュールdoc
        // 参照)。剛体の現在位置と同じ場所に置くことで、grid_fluid.rsの
        // `pressure_force_on_solid_integrates_a_known_linear_pressure_field`と同一の
        // ジオメトリ・圧力場になり、期待force=(-9.0,-6.0,0.0)を再利用できる。
        rasterize_box(&mut grid, (2.0, 2.0), (half_width, half_height), Vec3::ZERO);

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
                grid_fluid_3d: None,
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

        // (2) 固体セルが今stepの剛体位置・(注入後の)速度へ追従していること
        //     (移行前は`solid_box()`を読み戻していた——セル種別が唯一の表現に
        //     なったので、敷かれたセルそのものを検査する)。
        let body_pos = mechanics.bodies.position[body];
        let body_vel = mechanics.bodies.linear_velocity[body];
        assert_solid_region_matches_box(
            &grid,
            (body_pos.x, body_pos.y),
            (half_width, half_height),
            Vec3::new(body_vel.x, body_vel.y, 0.0),
        );
    }

    /// **剛体が動くとき、固体セル領域が毎step追従する**こと(masking と **un-masking**
    /// の両方)。移行前は`solid`(単一矩形)を書き換えるだけで済んでいたが、セル種別へ
    /// 一本化した後は「前stepで Solid にしたセルを Fluid へ戻す」責務が
    /// `set_solid_cells`側にある——その取りこぼしを検出するための回帰テスト。
    ///
    /// 剛体位置は毎step手で進める(圧力場はゼロのままなので`apply`は力を注入せず、
    /// マスク更新の配管だけを決定論的に見る)。
    #[test]
    fn grid_fluid_rigid_solid_region_follows_the_body_across_multiple_steps() {
        let nx = 16;
        let ny = 8;
        let h = 0.5;
        let mut grid = GridFluid2D::new(nx, ny, h);

        let materials = MaterialDb::standard();
        let steel = materials.find_by_name("鋼(炭素鋼)").unwrap();
        let half_width = 0.75;
        let half_height = 0.75;
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.2 }, steel);
        desc.mass_override = Some(2.0);
        desc.transform.position = Vec3::new(1.5, 2.0, 0.0);
        let mut mechanics = MechanicsSolver::new(0.0);
        let body = mechanics.create_body(desc, &materials);

        let mut coupling = GridFluidRigid::new(body, half_width, half_height);
        // 1stepで**ちょうど1セル分**(`h`)進ませる。半セルだと矩形が広がるだけで
        // 後端のセルが解放されず、un-masking の検証にならない。
        let velocity = Vec3::new(2.0, 0.0, 0.0);
        let dt = 0.25;

        let mut previous_solid: Option<Vec<bool>> = None;
        for step in 0..5 {
            // 剛体を+x方向へ一定速度で進める。
            let pos = Vec3::new(1.5 + velocity.x * dt * step as f64, 2.0, 0.0);
            mechanics.bodies.position[body] = pos;
            mechanics.bodies.linear_velocity[body] = velocity;

            {
                let mut states = DomainStates {
                    mechanics: &mut mechanics,
                    thermal: None,
                    em_circuit: None,
                    em_electrostatics: None,
                    gas: None,
                    grid_fluid: Some(&mut grid),
                    grid_fluid_3d: None,
                    sph: None,
                };
                coupling.apply(&mut states, dt);
            }

            // 各stepで固体領域が「今の剛体位置の矩形」と厳密に一致すること。
            assert_solid_region_matches_box(
                &grid,
                (pos.x, pos.y),
                (half_width, half_height),
                velocity,
            );

            let solid: Vec<bool> = grid
                .cell_type()
                .iter()
                .map(|c| *c == CellType::Solid)
                .collect();
            assert!(
                solid.iter().any(|s| *s),
                "step {step}: the body is inside the grid, so some cell must be Solid"
            );

            if let Some(previous) = &previous_solid {
                // 剛体が1セル分以上進んでいるので、**前stepでSolidだったセルのうち
                // 少なくとも1つがFluidへ戻っている**こと(un-masking の検証)。
                let unmasked = previous
                    .iter()
                    .zip(solid.iter())
                    .any(|(was, now)| *was && !*now);
                assert!(
                    unmasked,
                    "step {step}: the body moved, so at least one previously-Solid cell \
                     must have been released back to Fluid"
                );
            }
            previous_solid = Some(solid);
        }
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
            grid_fluid_3d: None,
            sph: None,
        };
        coupling.apply(&mut states, 0.01);

        assert_no_solid_cells(&grid);
    }
}
