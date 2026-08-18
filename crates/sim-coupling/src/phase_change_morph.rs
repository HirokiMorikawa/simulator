//! `PhaseChangeMorph`(設計 docs/20-integration/01-coupling-matrix.md §3「P3: 融解 →
//! 剛体消滅/流体生成イベント」)。
//!
//! **群9で「流体生成」を実装した**(`MeltSpawn`)。融けた質量ぶんを SPH 粒子として
//! 実際に生成する。
//!
//! **群5までの残置理由は事実誤認だった**ので、その訂正もここに残す。当時
//! 「`Coupling::apply` がイベントキュー(`sim_core::EventQueue`)にも `World` の
//! 世代管理にも `DomainStates` 経由でアクセスできないため対象外」と書いたが、
//! **SPH への注入にはそのどちらも要らなかった**——`DomainStates` は最初から
//! `sph: Option<&mut SphFluid>` を公開しており、`SphFluid::add_particle` も既に
//! 存在していた。アーキテクチャ拡張は不要で、今日そのまま書ける状態だった。
//!
//! **実際に残る制約**(こちらは本当): `World` のイベントキューは `DomainStates` から
//! 触れないため、設計が言う「イベント」としては発行できない。代わりに
//! `spawned_particles()` というカウンタで露出する。
//!
//! 熱源は単一の`ThermalNode`(`thermal_node`、氷を取り巻く飲み物・空気等に相当)とし、
//! 簡易な線形熱コンダクタンス(`conductance`、`DissipationToHeat`と同じ「呼び出し側が
//! 値を直接渡す」縮約)で氷側のエンタルピー(`sim_thermal::PhaseState`、エンタルピー法)
//! へ熱を流し込む。熱源ノード側からは同量を差し引く(設計§2規則1「取り出しと注入を
//! 同一実装内で対記帳」)。
//!
//! 剛体の質量は`initial_mass*(1-liquid_fraction)`(固相残存質量比)として
//! `RigidBodySet::inv_mass`を直接更新する(形状(`Shape`)自体は縮小しない——密度が
//! 見かけ上下がっていく近似、`Shape`のランタイム変形は`RigidBodySet`に未実装のため
//! 対象外)。この質量変化は既存の`BuoyancyDrag`・埋め込み浮力(`MechanicsSolver.fluids`)
//! 双方に無変更で伝播する(いずれも毎step`bodies.mass(idx)`を読み直すため、本
//! Couplingが質量を更新するだけで浮力側が自動的に追従する——D18「氷と飲み物」の
//! 「アルキメデス統合」が求める浮力との連動は、新規コードなしでこの構成上の性質から
//! 得られる)。
//!
//! 完全融解(`Phase::Liquid`)に達したら、`World::remove_body`と同じ無効化手順
//! (`body_type`をStaticへ・遠方へ退避・速度ゼロ化)を直接行う。`World`の世代カウンタ
//! (`generations`)は`DomainStates`から触れないため据え置かれる——呼び出し側が同じ
//! `BodyId`を将来再利用したい場合は別途`World::remove_body`を呼ぶ必要がある(正直に
//! 文書化する制約、`SphRigid`・`GridFluidRigid`と同様に既存の`World`公開APIを拡張
//! せずに実装できる範囲に留めた)。

use crate::domain_states::{Coupling, CouplingKind, CouplingRawState, DomainStates};
use sim_core::DomainId;
use sim_math::{SimRng, Vec3};
use sim_mechanics::BodyType;
use sim_thermal::{Phase, PhaseMaterial, PhaseState};

/// 融解した質量を SPH 粒子として生成する設定(**群9で追加**、モジュールdoc参照)。
#[derive(Clone, Copy)]
pub struct MeltSpawn {
    /// 生成位置のばらつき半径 [m](剛体中心まわりの球内に一様分布させる)。
    /// 融けた水は氷の表面付近から出るので、氷の半径程度を渡すのが自然。
    pub spawn_radius: f64,
    /// 決定論のための乱数シード(設計 docs/01-math/04-random.md の規約に従い、
    /// `SimRng::new(seed, 生成カウンタ)` で粒子ごとに独立ストリームを引く)。
    pub seed: u64,
}

/// 剛体(`body_index`)を、単一の熱源`ThermalNode`(`thermal_node`)からの熱で融解させる
/// (モジュールdoc参照)。
#[derive(Clone)]
pub struct PhaseChangeMorph {
    pub body_index: usize,
    pub thermal_node: usize,
    pub material: PhaseMaterial,
    pub initial_mass: f64,
    /// 熱源ノードとの線形熱コンダクタンス [W/K]。
    pub conductance: f64,
    /// 融けた質量をSPH粒子として生成する設定(**群9で追加**)。`None`(既定)なら
    /// 生成しない——移行前の挙動と**ビット単位で同一**になる。
    pub spawn: Option<MeltSpawn>,
    state: PhaseState,
    despawned: bool,
    /// まだ粒子1個ぶんに満たない融解質量の繰り越し [kg](量子化の余り)。
    /// 質量の対記帳はこの値を含めて厳密に閉じる(`spawned_particles`のdoc参照)。
    pending_spawn_mass: f64,
    /// これまでに生成した粒子数(決定論のストリーム番号も兼ねる)。
    spawned_particles: usize,
    /// 直前の`apply`時点の液相率(増分から今stepの融解質量を出すために持つ)。
    last_liquid_fraction: f64,
}

impl PhaseChangeMorph {
    /// `initial_enthalpy`は`PhaseState::enthalpy`の初期値(負なら融点未満の固相、
    /// `sim_thermal::phase`モジュールdoc参照)。
    pub fn new(
        body_index: usize,
        thermal_node: usize,
        material: PhaseMaterial,
        initial_mass: f64,
        conductance: f64,
        initial_enthalpy: f64,
    ) -> PhaseChangeMorph {
        PhaseChangeMorph {
            body_index,
            thermal_node,
            material,
            initial_mass,
            conductance,
            spawn: None,
            state: PhaseState {
                enthalpy: initial_enthalpy,
                mass: initial_mass,
            },
            despawned: false,
            pending_spawn_mass: 0.0,
            spawned_particles: 0,
            last_liquid_fraction: 0.0,
        }
    }

    /// 融けた質量をSPH粒子として生成する設定を有効にする(**群9で追加**)。
    pub fn with_spawn(mut self, spawn: MeltSpawn) -> PhaseChangeMorph {
        self.spawn = Some(spawn);
        self
    }

    /// 現在の相(テスト・診断用)。
    pub fn phase(&self) -> Phase {
        self.state.phase(&self.material)
    }

    /// これまでに生成したSPH粒子数(**群9で追加**)。
    ///
    /// 設計 docs/20-integration/01-coupling-matrix.md §3 は「融解 → 剛体消滅/流体生成
    /// **イベント**」と書いているが、`World`のイベントキューは`DomainStates`から
    /// 触れないため**イベントとしては発行できない**。呼び出し側がポーリングできる
    /// カウンタとして露出する(実際に残る制約、モジュールdoc参照)。
    pub fn spawned_particles(&self) -> usize {
        self.spawned_particles
    }

    /// まだ粒子1個ぶんに満たない融解質量の繰り越し [kg]。
    /// 質量保存は「生成粒子数×粒子質量 + 剛体の残存質量 + この繰り越し = initial_mass」
    /// として**厳密に等式で**閉じる。
    pub fn pending_spawn_mass(&self) -> f64 {
        self.pending_spawn_mass
    }

    /// 融解の増分ぶんをSPH粒子として生成する(**群9で追加**、モジュールdoc参照)。
    ///
    /// 液相率の**増分**から $\Delta m = m_0\,\Delta f$ を求めて繰り越しに積み、
    /// 粒子1個ぶん(`sph.mass`)溜まるたびに1個生成する。既存の熱の対記帳
    /// (設計§2規則1「取り出しと注入を同一実装内で対記帳」)と同じ構造で、
    /// **質量も取り出した分だけ注入する**。
    ///
    /// SPHドメインが無い場合は繰り越しに積むだけで生成しない——質量が消えるのでは
    /// なく「まだ注入先が無い」状態として保持されるので、質量保存の等式
    /// (`pending_spawn_mass` のdoc参照)はそのまま成り立つ。
    fn spawn_melted_particles(&mut self, world: &mut DomainStates, liquid_fraction: f64) {
        let Some(spawn) = self.spawn else {
            self.last_liquid_fraction = liquid_fraction;
            return; // 生成しない設定(既定)——移行前と完全に同じ挙動
        };
        let delta_fraction = liquid_fraction - self.last_liquid_fraction;
        self.last_liquid_fraction = liquid_fraction;
        if delta_fraction <= 0.0 {
            return;
        }
        self.pending_spawn_mass += self.initial_mass * delta_fraction;

        // 剛体側の値を先に読み出してから`sph`を可変借用する(`DomainStates`の
        // 別フィールドなので分離した借用は成立するが、順序を守ると読みやすい)。
        let position = world.mechanics.bodies.position[self.body_index];
        let velocity = world.mechanics.bodies.linear_velocity[self.body_index];
        let Some(sph) = &mut world.sph else {
            return; // 注入先が無い(繰り越しに積んだままにする)
        };
        if sph.mass <= 0.0 {
            return;
        }
        while self.pending_spawn_mass >= sph.mass {
            // 粒子ごとに独立ストリームを引く(設計 docs/01-math/04-random.md §2)。
            // `spawned_particles` を stream 番号に使うので、同じシードなら
            // 何度走らせても同じ位置になる(決定論規約)。
            let mut rng = SimRng::new(spawn.seed, self.spawned_particles as u64);
            // 球「内部」に一様分布させる(表面上の点を $r\sqrt[3]{u}$ で内側へ引く)。
            let radius = spawn.spawn_radius * rng.next_f64().cbrt();
            let offset = rng.unit_sphere().scale(radius);
            // 速度は剛体の現在速度を引き継ぐ(運動量の連続性)。
            sph.add_particle(position + offset, velocity);
            self.pending_spawn_mass -= sph.mass;
            self.spawned_particles += 1;
        }
    }
}

impl Coupling for PhaseChangeMorph {
    fn kind(&self) -> CouplingKind {
        CouplingKind::PhaseChangeMorph
    }

    fn domain_ids(&self) -> &'static [DomainId] {
        &[DomainId::Thermal, DomainId::Mechanics]
    }

    fn describe(&self) -> String {
        format!(
            "PhaseChangeMorph body#{} thermal_node[{}] Tm={}K m0={}kg",
            self.body_index,
            self.thermal_node,
            self.material.melting_temperature,
            self.initial_mass
        )
    }

    fn referenced_bodies(&self) -> Vec<usize> {
        vec![self.body_index]
    }

    fn referenced_thermal_nodes(&self) -> Vec<usize> {
        vec![self.thermal_node]
    }

    /// 融解の内部状態(`CouplingRawState`のdoc参照)。`CouplingJson`が持つ
    /// `initial_enthalpy`は**生成時の**値であって、融解が進んだ今の
    /// エンタルピーではない——これが無いと、融けかけの氷をエクスポート→
    /// 再インポートしたときに融解が最初からやり直しになる。
    /// 粒子生成の繰り越し(`pending_spawn_mass`)と生成数(ストリーム番号を兼ねる)も
    /// 含めるのは、質量保存の等式と生成位置の決定論が両方これに依存するため。
    fn raw_state(&self) -> Option<CouplingRawState> {
        Some(CouplingRawState::PhaseChangeMorph {
            enthalpy: self.state.enthalpy,
            mass: self.state.mass,
            despawned: self.despawned,
            pending_spawn_mass: self.pending_spawn_mass,
            spawned_particles: self.spawned_particles,
            last_liquid_fraction: self.last_liquid_fraction,
        })
    }

    fn restore_raw_state(&mut self, state: &CouplingRawState) -> Result<(), String> {
        let CouplingRawState::PhaseChangeMorph {
            enthalpy,
            mass,
            despawned,
            pending_spawn_mass,
            spawned_particles,
            last_liquid_fraction,
        } = state
        else {
            return Err("PhaseChangeMorph に別種の CouplingRawState が渡された".to_string());
        };
        self.state = PhaseState {
            enthalpy: *enthalpy,
            mass: *mass,
        };
        self.despawned = *despawned;
        self.pending_spawn_mass = *pending_spawn_mass;
        self.spawned_particles = *spawned_particles;
        self.last_liquid_fraction = *last_liquid_fraction;
        Ok(())
    }

    fn apply(&mut self, world: &mut DomainStates, dt: f64) {
        if self.despawned {
            return; // 既に完全融解済み(モジュールdoc参照)。
        }
        let Some(thermal) = &mut world.thermal else {
            return;
        };
        let Some(node) = thermal.nodes.get_mut(self.thermal_node) else {
            return;
        };

        // Q = 熱コンダクタンス×温度差×dt。熱源から取り出した分をそのまま氷側の
        // エンタルピーへ注入する対記帳(設計§2規則1)。
        let body_temperature = self.state.temperature(&self.material);
        let q = self.conductance * (node.temperature - body_temperature) * dt;
        node.temperature -= q / node.heat_capacity;
        self.state.add_heat(q);

        let liquid_fraction = match self.state.phase(&self.material) {
            Phase::Solid => 0.0,
            Phase::Mixed { liquid_fraction } => liquid_fraction,
            Phase::Liquid => 1.0,
        };
        // **群9**: 融けた質量ぶんのSPH粒子を生成する(モジュールdoc参照)。
        // 完全融解の退避(剛体を -1e9 へ飛ばす)より**前**に呼ぶ——生成位置は
        // 剛体の現在位置を使うので、順序を逆にすると最後の粒子群が遠方に湧く。
        self.spawn_melted_particles(world, liquid_fraction);

        let remaining_mass = self.initial_mass * (1.0 - liquid_fraction);
        let idx = self.body_index;
        if remaining_mass > 1e-9 {
            world.mechanics.bodies.inv_mass[idx] = 1.0 / remaining_mass;
            // 質量が変化した剛体はスリープ(設計 docs/10-mechanics/01-rigid-body.md、
            // `sim_mechanics::sleep`、静止0.5秒継続で自動停止)から起こす——スリープ中は
            // 力の適用・速度/位置積分が完全に止まる(`MechanicsSolver::apply_forces`/
            // `integrate_velocities`/`integrate_positions`)ため、外部要因(質量減少)で
            // 力の釣り合いが崩れたにもかかわらず永久に静止したままになってしまう
            // (実装検証中に発見: 浮いた氷が釣り合い位置で静止したまま融解が進んでも
            // 位置が全く変化しないバグとして顕在化した)。
            world.mechanics.bodies.asleep[idx] = false;
        } else {
            // 完全融解: `World::remove_body`と同じ無効化手順(モジュールdoc参照)。
            world.mechanics.bodies.body_type[idx] = BodyType::Static;
            world.mechanics.bodies.position[idx] = Vec3::new(0.0, -1.0e9, 0.0);
            world.mechanics.bodies.linear_velocity[idx] = Vec3::ZERO;
            world.mechanics.bodies.angular_velocity[idx] = Vec3::ZERO;
            self.despawned = true;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::MaterialDb;
    use sim_mechanics::{MechanicsSolver, RigidBodyDesc, Shape};
    use sim_thermal::{ThermalNode, ThermalSolver};

    fn ice_material() -> PhaseMaterial {
        PhaseMaterial {
            melting_temperature: 273.15,
            latent_heat_fusion: 334_000.0,
            specific_heat_solid: 2100.0,
            specific_heat_liquid: 4186.0,
        }
    }

    fn states<'a>(
        mechanics: &'a mut MechanicsSolver,
        thermal: &'a mut ThermalSolver,
    ) -> DomainStates<'a> {
        DomainStates {
            mechanics,
            thermal: Some(thermal),
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: None,
            grid_fluid_3d: None,
            sph: None,
        }
    }

    fn states_with_sph<'a>(
        mechanics: &'a mut MechanicsSolver,
        thermal: &'a mut ThermalSolver,
        sph: &'a mut sim_fluid::SphFluid,
    ) -> DomainStates<'a> {
        DomainStates {
            mechanics,
            thermal: Some(thermal),
            em_circuit: None,
            em_electrostatics: None,
            gas: None,
            grid_fluid: None,
            grid_fluid_3d: None,
            sph: Some(sph),
        }
    }

    /// 溶かす氷 + 熱源 + 受け皿のSPHを1組作る。`particle_mass` は生成の粒度。
    fn melting_setup(
        particle_mass: f64,
        spawn: Option<MeltSpawn>,
    ) -> (
        MechanicsSolver,
        ThermalSolver,
        sim_fluid::SphFluid,
        PhaseChangeMorph,
        usize,
        f64,
    ) {
        let materials = MaterialDb::standard();
        let ice_id = materials.find_by_name("氷(0°C)").unwrap();
        let initial_mass = 0.02;
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.02 }, ice_id);
        desc.mass_override = Some(initial_mass);
        desc.transform.position = Vec3::new(1.0, 2.0, 3.0);
        let mut mechanics = MechanicsSolver::new(0.0);
        let body = mechanics.create_body(desc, &materials);

        let mut thermal = ThermalSolver::new(353.15);
        let warm_node = thermal.add_node(ThermalNode::new(353.15, 50_000.0));

        let mut sph = sim_fluid::SphFluid::new(0.02, 1000.0, 20.0);
        sph.mass = particle_mass;

        let mat = ice_material();
        let mut coupling = PhaseChangeMorph::new(body, warm_node, mat, initial_mass, 50.0, 0.0);
        coupling.spawn = spawn;
        (mechanics, thermal, sph, coupling, body, initial_mass)
    }

    /// **群9**: 融けた質量ぶんのSPH粒子が実際に生成され、**質量が厳密に対記帳される**
    /// こと。設計 docs/20-integration/01-coupling-matrix.md §3「融解 → 剛体消滅/
    /// 流体生成」の後半。
    ///
    /// 質量保存は近似ではなく**厳密な等式**で確認する:
    /// 生成粒子数 × 粒子質量 + 剛体の残存質量 + 繰り越し = `initial_mass`。
    #[test]
    fn melted_mass_is_spawned_as_sph_particles_with_exact_mass_bookkeeping() {
        let particle_mass = 0.001; // 20粒子ぶん
        let (mut mechanics, mut thermal, mut sph, mut coupling, _body, initial_mass) =
            melting_setup(
                particle_mass,
                Some(MeltSpawn {
                    spawn_radius: 0.02,
                    seed: 7,
                }),
            );

        let dt = 0.1;
        for _ in 0..20_000 {
            coupling.apply(
                &mut states_with_sph(&mut mechanics, &mut thermal, &mut sph),
                dt,
            );
            if matches!(coupling.phase(), Phase::Liquid) {
                break;
            }
        }
        assert!(
            matches!(coupling.phase(), Phase::Liquid),
            "should have fully melted: phase={:?}",
            coupling.phase()
        );

        assert_eq!(
            sph.position.len(),
            coupling.spawned_particles(),
            "the SPH domain must hold exactly the particles this coupling reports"
        );
        assert!(
            coupling.spawned_particles() > 0,
            "melting must actually produce particles"
        );

        // **厳密な質量の対記帳**(浮動小数の丸めぶんだけ許容)。
        let spawned_mass = coupling.spawned_particles() as f64 * particle_mass;
        let remaining_solid = 0.0; // 完全融解したので固相は残っていない
        let total = spawned_mass + remaining_solid + coupling.pending_spawn_mass();
        assert!(
            (total - initial_mass).abs() < 1e-12,
            "spawned={spawned_mass} pending={} total={total} initial={initial_mass}",
            coupling.pending_spawn_mass()
        );
        // 量子化の余りは粒子1個ぶん未満(それ以上溜まっていたら生成漏れ)。
        assert!(
            coupling.pending_spawn_mass() < particle_mass,
            "the carry must always be smaller than one particle: {}",
            coupling.pending_spawn_mass()
        );
    }

    /// **回帰**: `spawn` が `None`(既定)なら、粒子は1つも生成されず**移行前と
    /// 完全に同じ挙動**になる。既存2テストが無変更で通ることと合わせて、
    /// 新機能が既定経路に一切影響しないことを固定する。
    #[test]
    fn without_a_spawn_setting_nothing_is_injected_into_the_fluid() {
        let (mut mechanics, mut thermal, mut sph, mut coupling, _body, _) =
            melting_setup(0.001, None);

        let dt = 0.1;
        for _ in 0..20_000 {
            coupling.apply(
                &mut states_with_sph(&mut mechanics, &mut thermal, &mut sph),
                dt,
            );
            if matches!(coupling.phase(), Phase::Liquid) {
                break;
            }
        }
        assert!(matches!(coupling.phase(), Phase::Liquid));
        assert_eq!(sph.position.len(), 0, "no particles must be created");
        assert_eq!(coupling.spawned_particles(), 0);
        assert_eq!(
            coupling.pending_spawn_mass(),
            0.0,
            "and nothing must be carried either"
        );
    }

    /// **決定論**(設計 docs/01-math/04-random.md): 同じシードで2回走らせると
    /// 生成粒子の位置が**ビット一致**する。
    #[test]
    fn spawning_is_deterministic_for_a_fixed_seed() {
        let run = || {
            let (mut mechanics, mut thermal, mut sph, mut coupling, _body, _) = melting_setup(
                0.002,
                Some(MeltSpawn {
                    spawn_radius: 0.02,
                    seed: 12345,
                }),
            );
            let dt = 0.1;
            for _ in 0..20_000 {
                coupling.apply(
                    &mut states_with_sph(&mut mechanics, &mut thermal, &mut sph),
                    dt,
                );
                if matches!(coupling.phase(), Phase::Liquid) {
                    break;
                }
            }
            (sph.position.clone(), sph.velocity.clone())
        };
        let (positions_a, velocities_a) = run();
        let (positions_b, velocities_b) = run();
        assert!(!positions_a.is_empty());
        assert_eq!(positions_a, positions_b, "positions must be bit-identical");
        assert_eq!(velocities_a, velocities_b);
    }

    /// 生成位置が剛体中心まわり `spawn_radius` の球内にあり、速度が剛体の速度を
    /// 引き継いでいること(運動量の連続性)。
    #[test]
    fn spawned_particles_appear_inside_the_body_with_its_velocity() {
        let spawn_radius = 0.02;
        let (mut mechanics, mut thermal, mut sph, mut coupling, body, _) = melting_setup(
            0.002,
            Some(MeltSpawn {
                spawn_radius,
                seed: 3,
            }),
        );
        let body_velocity = Vec3::new(0.5, -0.25, 0.125);
        mechanics.bodies.linear_velocity[body] = body_velocity;
        let body_position = mechanics.bodies.position[body];

        // 完全融解の退避(遠方へのテレポート)が起きる前に止める。
        let dt = 0.1;
        for _ in 0..20_000 {
            coupling.apply(
                &mut states_with_sph(&mut mechanics, &mut thermal, &mut sph),
                dt,
            );
            if coupling.spawned_particles() >= 3 {
                break;
            }
        }
        assert!(coupling.spawned_particles() >= 3);

        for (position, velocity) in sph.position.iter().zip(sph.velocity.iter()) {
            let offset = (*position - body_position).length();
            assert!(
                offset <= spawn_radius + 1e-12,
                "particle must appear within spawn_radius: offset={offset} radius={spawn_radius}"
            );
            assert_eq!(
                *velocity, body_velocity,
                "particles must inherit the body velocity (momentum continuity)"
            );
        }
    }

    /// 融解プラトー期間中、剛体質量が液相率どおりに`initial_mass*(1-liquid_fraction)`
    /// で減少すること、かつ熱源ノードから取り出した熱量と氷側が受け取った熱量が
    /// 厳密に対記帳されること(設計§2規則1)を確認する。T7の融解プラトー物理自体
    /// (`sim_thermal::phase`の`PhaseState`)は既に検証済みのため、ここでは配線
    /// (質量の追従・対記帳)のみを検証する。
    #[test]
    fn phase_change_morph_shrinks_mass_matching_liquid_fraction_and_conserves_heat() {
        let materials = MaterialDb::standard();
        let ice_id = materials.find_by_name("氷(0°C)").unwrap();
        let initial_mass = 0.1;
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.03 }, ice_id);
        desc.mass_override = Some(initial_mass);
        let mut mechanics = MechanicsSolver::new(0.0);
        let body = mechanics.create_body(desc, &materials);

        let mut thermal = ThermalSolver::new(293.15);
        let warm_node = thermal.add_node(ThermalNode::new(293.15, 5000.0));

        let mat = ice_material();
        let mut coupling = PhaseChangeMorph::new(
            body,
            warm_node,
            mat,
            initial_mass,
            20.0,
            -mat.specific_heat_solid * 10.0,
        );

        let dt = 0.05;
        let initial_node_heat =
            thermal.nodes[warm_node].temperature * thermal.nodes[warm_node].heat_capacity;
        for _ in 0..4000 {
            if matches!(coupling.phase(), Phase::Mixed { .. }) {
                break;
            }
            coupling.apply(&mut states(&mut mechanics, &mut thermal), dt);
        }
        assert!(
            matches!(coupling.phase(), Phase::Mixed { .. }),
            "should have reached the melting plateau: phase={:?}",
            coupling.phase()
        );

        let liquid_fraction = match coupling.phase() {
            Phase::Mixed { liquid_fraction } => liquid_fraction,
            other => panic!("expected Mixed, got {other:?}"),
        };
        let expected_mass = initial_mass * (1.0 - liquid_fraction);
        let measured_mass = mechanics.bodies.mass(body);
        assert!(
            (measured_mass - expected_mass).abs() < 1e-9,
            "measured_mass={measured_mass} expected_mass={expected_mass}"
        );

        // 対記帳: 熱源ノードが失った熱量 == 氷が(顕熱+潜熱として)受け取った熱量。
        let final_node_heat =
            thermal.nodes[warm_node].temperature * thermal.nodes[warm_node].heat_capacity;
        let heat_lost_by_node = initial_node_heat - final_node_heat;
        let heat_gained_by_ice =
            coupling.state.mass * (coupling.state.enthalpy - (-mat.specific_heat_solid * 10.0));
        let rel_err = (heat_lost_by_node - heat_gained_by_ice).abs() / heat_lost_by_node;
        assert!(
            rel_err < 1e-9,
            "heat_lost_by_node={heat_lost_by_node} heat_gained_by_ice={heat_gained_by_ice} rel_err={rel_err:e}"
        );
    }

    /// 完全融解(`Phase::Liquid`)に達すると、`World::remove_body`と同じ無効化
    /// (Static化・遠方退避・速度ゼロ化)が行われ、以後`apply`を繰り返し呼んでも
    /// 状態が変化しない(冪等)ことを確認する。
    #[test]
    fn phase_change_morph_despawns_the_body_once_fully_melted() {
        let materials = MaterialDb::standard();
        let ice_id = materials.find_by_name("氷(0°C)").unwrap();
        let initial_mass = 0.01; // 小質量・短時間で完全融解に到達させる。
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.01 }, ice_id);
        desc.mass_override = Some(initial_mass);
        desc.transform.position = Vec3::new(1.0, 2.0, 3.0);
        let mut mechanics = MechanicsSolver::new(0.0);
        let body = mechanics.create_body(desc, &materials);

        let mut thermal = ThermalSolver::new(353.15); // 熱い湯相当、短時間で完全融解させる。
        let warm_node = thermal.add_node(ThermalNode::new(353.15, 50_000.0));

        let mat = ice_material();
        let mut coupling = PhaseChangeMorph::new(body, warm_node, mat, initial_mass, 50.0, 0.0);

        let dt = 0.1;
        for _ in 0..20_000 {
            coupling.apply(&mut states(&mut mechanics, &mut thermal), dt);
            if matches!(coupling.phase(), Phase::Liquid) {
                break;
            }
        }
        assert!(
            matches!(coupling.phase(), Phase::Liquid),
            "should have fully melted: phase={:?}",
            coupling.phase()
        );
        assert_eq!(mechanics.bodies.body_type[body], BodyType::Static);
        assert_eq!(mechanics.bodies.position[body], Vec3::new(0.0, -1.0e9, 0.0));
        assert_eq!(mechanics.bodies.linear_velocity[body], Vec3::ZERO);

        // 冪等性: 完全融解後に追加でapplyしても状態は変化しない。
        let position_after_despawn = mechanics.bodies.position[body];
        coupling.apply(&mut states(&mut mechanics, &mut thermal), dt);
        assert_eq!(mechanics.bodies.position[body], position_after_despawn);
    }
}
