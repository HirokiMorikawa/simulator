//! `World → Scenario`逆写像(統合エディタ実装計画の縦串①、
//! docs/reviews/2026-08-14-editor-implementation-plan.md 参照)。
//!
//! `from_scenario`(`Scenario → World`)は既に存在するが、逆方向(実行中の`World`を
//! 編集可能なシーンドキュメントへ書き戻す)は無かった——手書きの
//! `sim-wasm::export_scene_json`は bodies の3形状しか書けず、joints・couplings・
//! fluids・thermal・circuit・probesは無言で脱落していた。この関数がその置き換え。
//!
//! **既知の制限(縮約、正直な記録)**:
//! - `seed`: `World`はRNGへ消費した後の元の値を保持しないため、常に`0`を書く。
//!   ブラウン運動・相変化の粒子生成のような乱数依存ドメインを含むシーンを
//!   エクスポート→再インポートすると、以後の乱数列が変わりうる。
//! - `PistonGas`・`SphRigid`・`PhaseChangeMorph`・`BrownianForce`:
//!   基準値(ピストンの変位ゼロ点・境界粒子の確保区間・融解の内部状態・RNGの
//!   ストリーム位置)が`sim-coupling`側で非公開のため、公開されているパラメータ
//!   (現在値)のみを書き戻す。ピストンが変位済み・氷が融解途中のシーンを
//!   エクスポートすると、再インポート後は「今の状態を新たな基準」として
//!   再スタートする(基準そのものではなく現在値を保存するため、以後の相対変位は
//!   ゼロから再カウントされる)。
//! - `kinetic_gas`: 圧力測定用の壁運動量アキュムレータ(`sim_statistical::GasSim`の
//!   `wall_impulse_accum`/`wall_impulse_time`)が非公開のため書き出せない。
//!   `GasSim`の`state_hash`はこの2つを含まない(位置・速度・`collision_count`のみ)
//!   ので決定論replayには影響せず、**影響するのは`pressure()`の時間平均窓が
//!   復元時点から引き直されること**だけである(`reset_pressure_accumulator`を
//!   呼んだのと同じ状態になる)。窓を取り直せば同じ圧力へ収束する。
//! - `fdtd`のPML: `sim_em::FdtdSim2D::with_pml`の分離場成分は書き出せない。
//!   ただし`FdtdScenarioJson`にPMLを構成するフィールドが無く、`from_scenario`が
//!   作る`World`のFDTDは常にPEC境界のみ(`pml: None`)なので、
//!   エクスポート→再インポートの往復ではこの制限に到達しない。
//!
//! **解消済みの制限(**生状態スナップショット**、`scenario`モジュールdoc参照)**:
//! `grid_fluid`/`grid_fluid_3d`/`sph`/`soft_body`/`quantum_1d`/`quantum_2d`/
//! `brownian`/`kinetic_gas`/`ising`/`fdtd`/`conduction_rod`の11ドメインは、
//! シーンJSON側のスキーマが「構築レシピ」(例: 波束の中心・分散、SPH粒子を
//! 敷き詰める直方体ブロック)であって「状態のスナップショット」ではないため、
//! 時間発展した後の実行中`World`から正確に逆算できず、長らく`None`のまま
//! 落としていた。各`*ScenarioJson`へ`raw_state`(生状態スナップショット)を
//! 足したことで**時間発展後でも`state_hash`一致で往復できるようになった**。
//! レシピ側のフィールド(格子寸法・粒子数等)にも現在値を書くが、それは人が
//! 読むためのメタデータで、復元に使われるのは`raw_state`である。

use crate::{BodyId, ProbeTarget, World};
use sim_math::Vec3;
use sim_mechanics::{BodyType, DragModel, Shape};
use std::collections::HashMap;

use crate::scenario::{
    AstroBodyJson, AstroScenarioJson, AtmosphereJson, AtmosphericDragJson, BodyScenarioDesc,
    BodyThermalLinkJson, BrownianRawStateJson, BrownianScenarioJson, CapacitorJson,
    CircuitScenarioJson, CompoundChildJson, ConductionRodRawStateJson, ConductionRodScenarioJson,
    ConvectionModeJson, CouplingJson, DiodeJson, FdtdRawStateJson, FdtdScenarioJson, FluidJson,
    GasScenarioJson, GaussianPacket2dJson, GaussianPacketJson, GridBoundaryJson,
    GridFluid3DRawStateJson, GridFluid3DScenarioJson, GridFluidRawStateJson, GridFluidScenarioJson,
    GridSolidBoxJson, InductorJson, IsingRawStateJson, IsingScenarioJson, JointJson,
    KineticGasRawStateJson, KineticGasScenarioJson, LiftModelJson, MaterialOverride, ProbeJson,
    Quantum1dRawStateJson, Quantum1dScenarioJson, Quantum2dRawStateJson, Quantum2dScenarioJson,
    RelativisticCorrectionJson, ResistorJson, Scenario, ShapeJson, SoftBendingConstraintJson,
    SoftBodyRawStateJson, SoftBodyScenarioJson, SoftConstraintJson, SoftVolumeConstraintJson,
    SphRawStateJson, SphScenarioJson, SwitchJson, ThermalLinkJson, ThermalNodeJson,
    ThermalScenarioJson, VoltageSourceJson, WorldScenarioOptions,
};

/// この export セッション内で使う、生存ボディの決定的な名前
/// (`body_{BodyId.index}`)。ジョイント・結合・プローブはこの名前で
/// ボディを参照する(`JointJson`等は文字列参照のみを受け付けるため)。
fn body_name(id: BodyId) -> String {
    format!("body_{}", id.index)
}

/// 実行中の`World`を、`Scenario::from_scenario`で読み戻せるシーンドキュメントへ
/// 変換する(モジュールdoc「既知の制限」参照)。
pub fn to_scenario(world: &World, name: &str) -> Scenario {
    let live_bodies = world.body_ids();
    let names: HashMap<BodyId, String> =
        live_bodies.iter().map(|id| (*id, body_name(*id))).collect();

    let (materials, material_name_of) = export_materials(world, &live_bodies);

    Scenario {
        name: name.to_string(),
        // 既知の制限(モジュールdoc参照): World は元のシードを保持しない。
        seed: 0,
        // 経過ステップ数(`Scenario::elapsed_steps`のdoc参照)。`state_hash`は
        // 先頭で時刻を混ぜるので、これを書かないと時間発展後のシーンは
        // 復元しても必ずハッシュがずれる。
        elapsed_steps: world.step_count(),
        world: export_world_options(world),
        materials,
        bodies: export_bodies(world, &live_bodies, &names, &material_name_of),
        fluids: export_fluids(world),
        thermal: export_thermal(world),
        joints: export_joints(world, &names),
        couplings: export_couplings(world, &names),
        circuit: export_circuit(world),
        astro: export_astro(world),
        soft_body: export_soft_body(world),
        grid_fluid: export_grid_fluid(world),
        grid_fluid_3d: export_grid_fluid_3d(world),
        conduction_rod: export_conduction_rod(world),
        sph: export_sph(world),
        gas: export_gas(world),
        quantum_1d: export_quantum_1d(world),
        quantum_2d: export_quantum_2d(world),
        brownian: export_brownian(world),
        kinetic_gas: export_kinetic_gas(world),
        ising: export_ising(world),
        fdtd: export_fdtd(world),
        probes: export_probes(world, &names),
        prediction_prompts: Vec::new(),
        // `prediction_prompts`と同じ理由(モジュールdocの`Scenario::
        // pass_criteria`参照)——著者向けメタデータであり`World`は
        // 実行時状態として持たない。
        pass_criteria: Vec::new(),
    }
}

fn export_world_options(world: &World) -> WorldScenarioOptions {
    let mechanics = world.mechanics();
    WorldScenarioOptions {
        gravity: mechanics.gravity,
        gravity_direction: Some(vec3_to_array(mechanics.gravity_direction)),
        dt: world.dt(),
        restitution_velocity_threshold: Some(mechanics.restitution_velocity_threshold),
        atmosphere: mechanics.atmosphere.as_ref().map(|a| AtmosphereJson {
            density: a.density,
            viscosity: a.viscosity,
        }),
    }
}

/// 生存ボディが使う材料を集め、標準DBの密度違い派生(`extends`)なら
/// `MaterialOverride`として書き出す。戻り値は`(materials, MaterialId → 名前)`。
fn export_materials(
    world: &World,
    live_bodies: &[BodyId],
) -> (Vec<MaterialOverride>, HashMap<sim_core::MaterialId, String>) {
    let standard = sim_core::MaterialDb::standard();
    let db = world.materials();

    let mut used: Vec<sim_core::MaterialId> = live_bodies
        .iter()
        .map(|id| db_material_of(world, *id))
        .collect();
    used.sort_by_key(|m| m.0);
    used.dedup();

    let mut overrides = Vec::new();
    let mut name_of = HashMap::new();
    for mat_id in used {
        let m = db.get(mat_id);
        if standard
            .find_by_name(m.name)
            .is_some_and(|std_id| materials_equal(standard.get(std_id), m))
        {
            // 標準DBに同名・同物性のものがある(まさにその標準材料)。
            name_of.insert(mat_id, m.name.to_string());
            continue;
        }
        // 標準DBの密度違い派生(`append_scenario_bodies`の`extends`適用と同じく、
        // 密度以外は基底材料と完全一致するはず)。基底を探して`extends`を書く。
        let parent = standard
            .iter()
            .find(|(_, base)| materials_equal_except_density(base, m));
        match parent {
            Some((_, base)) => {
                overrides.push(MaterialOverride {
                    extends: base.name.to_string(),
                    name: m.name.to_string(),
                    density: Some(m.density),
                });
            }
            None => {
                // 標準DBのどれの派生でもない材料(理論上到達しない——
                // `append_scenario_bodies`は`extends`経由でしか材料を追加しない)。
                // フォールバックとして最初の標準材料からの全属性上書きは表現できないため、
                // 名前だけそのまま書き出す(次のfrom_scenarioで`UnknownMaterial`に
                // なりうるが、この状況自体が既存の縮約が破れていることを示す)。
            }
        }
        name_of.insert(mat_id, m.name.to_string());
    }
    (overrides, name_of)
}

fn db_material_of(world: &World, id: BodyId) -> sim_core::MaterialId {
    world.mechanics().bodies.material[id.index as usize]
}

fn materials_equal(a: &sim_core::Material, b: &sim_core::Material) -> bool {
    a.name == b.name && materials_equal_except_density(a, b) && a.density == b.density
}

fn materials_equal_except_density(a: &sim_core::Material, b: &sim_core::Material) -> bool {
    a.friction == b.friction
        && a.restitution == b.restitution
        && a.youngs_modulus == b.youngs_modulus
        && a.specific_heat == b.specific_heat
        && a.conductivity == b.conductivity
        && a.emissivity == b.emissivity
        && phase_change_props_equal(&a.melting, &b.melting)
        && a.resistivity == b.resistivity
        && a.relative_permittivity == b.relative_permittivity
        && a.refractive_index == b.refractive_index
}

fn phase_change_props_equal(
    a: &Option<sim_core::PhaseChangeProps>,
    b: &Option<sim_core::PhaseChangeProps>,
) -> bool {
    match (a, b) {
        (None, None) => true,
        (Some(a), Some(b)) => {
            a.melting_point == b.melting_point
                && a.latent_heat_fusion == b.latent_heat_fusion
                && a.boiling_point == b.boiling_point
                && a.latent_heat_vaporization == b.latent_heat_vaporization
        }
        _ => false,
    }
}

fn export_bodies(
    world: &World,
    live_bodies: &[BodyId],
    names: &HashMap<BodyId, String>,
    material_name_of: &HashMap<sim_core::MaterialId, String>,
) -> Vec<BodyScenarioDesc> {
    let bodies = &world.mechanics().bodies;
    live_bodies
        .iter()
        .map(|id| {
            let idx = id.index as usize;
            let shape_json = shape_to_shape_json(bodies.shape_of(idx));
            let drag = matches!(bodies.drag[idx], DragModel::Sphere { .. });
            let body_type = match bodies.body_type[idx] {
                BodyType::Static => Some("static".to_string()),
                BodyType::Kinematic => Some("kinematic".to_string()),
                BodyType::Dynamic => None,
            };
            let mass_override =
                if bodies.body_type[idx] == BodyType::Dynamic && bodies.inv_mass[idx] > 0.0 {
                    Some(1.0 / bodies.inv_mass[idx])
                } else {
                    None
                };
            let material = material_name_of
                .get(&bodies.material[idx])
                .cloned()
                .unwrap_or_default();
            BodyScenarioDesc {
                shape: shape_json,
                material,
                position: vec3_to_array(bodies.position[idx]),
                rotation: Some(quat_to_array(bodies.rotation[idx])),
                linear_velocity: vec3_to_array(bodies.linear_velocity[idx]),
                angular_velocity: vec3_to_array(bodies.angular_velocity[idx]),
                body_type,
                name: Some(names[id].clone()),
                drag,
                mass_override,
                collision_group: Some(bodies.collision_group[idx]),
                collision_mask: Some(bodies.collision_mask[idx]),
            }
        })
        .collect()
}

fn export_fluids(world: &World) -> Vec<FluidJson> {
    match &world.mechanics().water {
        Some(w) => vec![FluidJson::StaticWater {
            water_level: w.water_level,
            density: w.density,
        }],
        None => Vec::new(),
    }
}

fn export_thermal(world: &World) -> Option<ThermalScenarioJson> {
    world.thermal().map(|t| ThermalScenarioJson {
        ambient_temperature: t.ambient_temperature,
        nodes: t
            .nodes
            .iter()
            .map(|n| ThermalNodeJson {
                temperature: n.temperature,
                heat_capacity: n.heat_capacity,
                convection_coefficient: n.convection_coefficient,
                area: n.area,
                emissivity: n.emissivity,
            })
            .collect(),
        links: t
            .links
            .iter()
            .map(|l| ThermalLinkJson {
                a: l.a,
                b: l.b,
                conductance: l.conductance,
            })
            .collect(),
        environment_radiation_temperature: Some(t.environment_radiation_temperature),
    })
}

fn export_circuit(world: &World) -> Option<CircuitScenarioJson> {
    world.circuit().map(|c| CircuitScenarioJson {
        num_nodes: c.num_nodes(),
        resistors: c
            .resistors()
            .iter()
            .map(|(a, b, r)| ResistorJson {
                a: *a,
                b: *b,
                resistance: *r,
            })
            .collect(),
        voltage_sources: c
            .voltage_sources()
            .iter()
            .map(|(a, b, v)| VoltageSourceJson {
                a: *a,
                b: *b,
                voltage: *v,
            })
            .collect(),
        capacitors: c
            .capacitors()
            .iter()
            .enumerate()
            .map(|(i, (a, b, cap))| CapacitorJson {
                a: *a,
                b: *b,
                capacitance: *cap,
                initial_voltage: c.capacitor_voltage(i),
            })
            .collect(),
        inductors: c
            .inductors()
            .iter()
            .enumerate()
            .map(|(i, (a, b, ind))| InductorJson {
                a: *a,
                b: *b,
                inductance: *ind,
                initial_current: c.inductor_current(i),
            })
            .collect(),
        diodes: c
            .diodes()
            .iter()
            .map(|(anode, cathode, i_s, n_vt)| DiodeJson {
                anode: *anode,
                cathode: *cathode,
                saturation_current: *i_s,
                n_vt: *n_vt,
            })
            .collect(),
        switches: c
            .switches()
            .iter()
            .map(|(a, b, closed)| SwitchJson {
                a: *a,
                b: *b,
                closed: *closed,
            })
            .collect(),
    })
}

fn export_astro(world: &World) -> Option<AstroScenarioJson> {
    world.astro().map(|a| AstroScenarioJson {
        softening: a.softening,
        bodies: a
            .position
            .iter()
            .zip(a.velocity.iter())
            .zip(a.mass.iter())
            .map(|((p, v), m)| AstroBodyJson {
                position: vec3_to_array(*p),
                velocity: vec3_to_array(*v),
                mass: *m,
            })
            .collect(),
        atmospheric_drag: a.atmospheric_drag.as_ref().map(|d| AtmosphericDragJson {
            central_body: d.central_body,
            surface_density: d.surface_density,
            scale_height: d.scale_height,
            planet_radius: d.planet_radius,
            ballistic_coefficients: d
                .ballistic_coefficient
                .iter()
                .enumerate()
                .filter_map(|(i, c)| c.map(|c| (i, c)))
                .collect(),
        }),
        relativistic_correction: a.relativistic_correction.as_ref().map(|r| {
            RelativisticCorrectionJson {
                central_body: r.central_body,
                speed_of_light: r.speed_of_light,
            }
        }),
    })
}

fn export_gas(world: &World) -> Option<GasScenarioJson> {
    world.gas().map(|g| GasScenarioJson {
        n_moles: g.n_moles,
        volume: g.volume,
        temperature: g.temperature,
        degrees_of_freedom: Some(g.gas.degrees_of_freedom),
        molar_mass: Some(g.gas.molar_mass),
    })
}

// ---------------------------------------------------------------------------
// **生状態スナップショット(`raw_state`)を使う11ドメインのエクスポート**。
// 構築レシピでは時間発展後の状態を表せないドメイン群(モジュールdoc「解消済みの
// 制限」・`crate::scenario`モジュールdoc参照)。どれも同じ形をしている:
// レシピ側のフィールドには**現在値**を(人が読むためのメタデータとして)書き、
// `raw_state`に復元用の生状態を入れる。
// ---------------------------------------------------------------------------

fn export_soft_body(world: &World) -> Option<SoftBodyScenarioJson> {
    let body = world.soft_body()?;
    Some(SoftBodyScenarioJson {
        // レシピ側は空にする(`rope`ヘルパの引数は`SoftBody`から逆算できない——
        // 分割数もレスト長も拘束の集合へ潰れてしまっている)。`raw_state`が真。
        rope: None,
        particles: Vec::new(),
        constraints: Vec::new(),
        pinned: Vec::new(),
        gravity: Some(vec3_to_array(body.gravity)),
        substeps: Some(body.substeps),
        iterations: Some(body.iterations),
        damping: Some(body.damping),
        raw_state: Some(SoftBodyRawStateJson {
            position: body.position.iter().copied().map(vec3_to_array).collect(),
            prev_position: body
                .prev_position
                .iter()
                .copied()
                .map(vec3_to_array)
                .collect(),
            velocity: body.velocity.iter().copied().map(vec3_to_array).collect(),
            inv_mass: body.inv_mass.clone(),
            constraints: body
                .constraints
                .iter()
                .map(|c| SoftConstraintJson {
                    i: c.i,
                    j: c.j,
                    rest: c.rest,
                    compliance: c.compliance,
                })
                .collect(),
            bending_constraints: body
                .bending_constraints
                .iter()
                .map(|c| SoftBendingConstraintJson {
                    i: c.i,
                    j: c.j,
                    k: c.k,
                    rest: c.rest,
                    compliance: c.compliance,
                })
                .collect(),
            volume_constraints: body
                .volume_constraints
                .iter()
                .map(|c| SoftVolumeConstraintJson {
                    particles: c.particles,
                    rest_volume: c.rest_volume,
                    compliance: c.compliance,
                })
                .collect(),
            gravity: vec3_to_array(body.gravity),
            substeps: body.substeps,
            iterations: body.iterations,
            damping: body.damping,
        }),
    })
}

fn export_grid_fluid(world: &World) -> Option<GridFluidScenarioJson> {
    let fluid = world.grid_fluid()?;
    let boundary = grid_boundary_to_json(fluid.boundary());
    Some(GridFluidScenarioJson {
        nx: fluid.nx,
        ny: fluid.ny,
        h: fluid.h,
        density: Some(fluid.density),
        kinematic_viscosity: Some(fluid.kinematic_viscosity),
        // 一様初期速度・解析形の固体は`raw_state`の格子で完全に置き換わる
        // (任意形状の固体は矩形/円の列挙では表せない)。
        initial_velocity: None,
        boundary: Some(boundary),
        vorticity_confinement_epsilon: Some(fluid.vorticity_confinement_epsilon),
        solids: Vec::new(),
        raw_state: Some(GridFluidRawStateJson {
            u: fluid.u.clone(),
            v: fluid.v.clone(),
            solid_cells: fluid
                .cell_type()
                .iter()
                .map(|c| *c == sim_fluid::CellType::Solid)
                .collect(),
            solid_velocity: fluid
                .solid_velocity()
                .iter()
                .copied()
                .map(vec3_to_array)
                .collect(),
            solid_box: fluid.solid_box().map(|b| GridSolidBoxJson {
                center: [b.center.0, b.center.1],
                half_width: b.half_width,
                half_height: b.half_height,
                velocity: vec3_to_array(b.velocity),
            }),
            density: fluid.density,
            kinematic_viscosity: fluid.kinematic_viscosity,
            vorticity_confinement_epsilon: fluid.vorticity_confinement_epsilon,
            boundary: Some(grid_boundary_to_json(fluid.boundary())),
        }),
    })
}

fn export_grid_fluid_3d(world: &World) -> Option<GridFluid3DScenarioJson> {
    let fluid = world.grid_fluid_3d()?;
    Some(GridFluid3DScenarioJson {
        nx: fluid.nx,
        ny: fluid.ny,
        nz: fluid.nz,
        h: fluid.h,
        density: Some(fluid.density),
        kinematic_viscosity: Some(fluid.kinematic_viscosity),
        boundary: Some(grid_boundary_3d_to_json(fluid.boundary())),
        vorticity_confinement_epsilon: Some(fluid.vorticity_confinement_epsilon),
        solids: Vec::new(),
        smoke_blocks: Vec::new(),
        raw_state: Some(GridFluid3DRawStateJson {
            u: fluid.u.clone(),
            v: fluid.v.clone(),
            w: fluid.w.clone(),
            smoke_density: fluid.smoke_density.clone(),
            solid_cells: fluid
                .cell_type()
                .iter()
                .map(|c| *c == sim_fluid::CellType::Solid)
                .collect(),
            solid_velocity: fluid
                .solid_velocity()
                .iter()
                .copied()
                .map(vec3_to_array)
                .collect(),
            density: fluid.density,
            kinematic_viscosity: fluid.kinematic_viscosity,
            vorticity_confinement_epsilon: fluid.vorticity_confinement_epsilon,
            boundary: Some(grid_boundary_3d_to_json(fluid.boundary())),
        }),
    })
}

fn grid_boundary_to_json(boundary: sim_fluid::GridBoundary) -> GridBoundaryJson {
    match boundary {
        sim_fluid::GridBoundary::Periodic => GridBoundaryJson::Periodic,
        sim_fluid::GridBoundary::Channel { inflow_speed } => {
            GridBoundaryJson::Channel { inflow_speed }
        }
    }
}

fn grid_boundary_3d_to_json(boundary: sim_fluid::GridBoundary3D) -> GridBoundaryJson {
    match boundary {
        sim_fluid::GridBoundary3D::Periodic => GridBoundaryJson::Periodic,
        sim_fluid::GridBoundary3D::Channel { inflow_speed } => {
            GridBoundaryJson::Channel { inflow_speed }
        }
    }
}

fn export_conduction_rod(world: &World) -> Option<ConductionRodScenarioJson> {
    let rod = world.conduction_rod()?;
    let node_count = rod.temperature.len();
    Some(ConductionRodScenarioJson {
        node_count,
        length: rod.dx * (node_count.max(1) - 1) as f64,
        // レシピ側の「一様な初期温度」は時間発展後には存在しない。人が読むときの
        // 目安として現在の左端温度を書く(復元には使われない)。
        initial_temperature: rod.temperature.first().copied().unwrap_or(0.0),
        thermal_diffusivity: Some(rod.thermal_diffusivity),
        material: None,
        boundary_left: rod.temperature.first().copied(),
        boundary_right: rod.temperature.last().copied(),
        raw_state: Some(ConductionRodRawStateJson {
            temperature: rod.temperature.clone(),
            dx: rod.dx,
            thermal_diffusivity: rod.thermal_diffusivity,
            volumetric_heat_capacity: rod.volumetric_heat_capacity,
            conductivity: rod.conductivity.clone(),
            cross_section_area: rod.cross_section_area,
        }),
    })
}

fn export_sph(world: &World) -> Option<SphScenarioJson> {
    let fluid = world.sph()?;
    Some(SphScenarioJson {
        h: fluid.h,
        rest_density: fluid.rho0,
        sound_speed: fluid.c_s,
        particle_mass: Some(fluid.mass),
        blocks: Vec::new(),
        boundary_blocks: Vec::new(),
        raw_state: Some(SphRawStateJson {
            position: fluid.position.iter().copied().map(vec3_to_array).collect(),
            velocity: fluid.velocity.iter().copied().map(vec3_to_array).collect(),
            density: fluid.density.clone(),
            pressure: fluid.pressure.clone(),
            mass: fluid.mass,
            h: fluid.h,
            rho0: fluid.rho0,
            c_s: fluid.c_s,
            viscosity_alpha: fluid.viscosity_alpha,
            gravity: fluid.gravity,
            boundary_position: fluid
                .boundary_position
                .iter()
                .copied()
                .map(vec3_to_array)
                .collect(),
        }),
    })
}

fn export_quantum_1d(world: &World) -> Option<Quantum1dScenarioJson> {
    let wave = world.quantum_1d()?;
    Some(Quantum1dScenarioJson {
        n: wave.psi.len(),
        dx: wave.dx,
        // レシピ側の波束パラメータは時間発展後の`psi`から逆算できない
        // (分散したガウス波束はもはやガウス型とは限らない)。ゼロで埋める。
        packet: GaussianPacketJson {
            x0: 0.0,
            sigma: 0.0,
            k0: 0.0,
        },
        potential: None,
        raw_state: Some(Quantum1dRawStateJson {
            psi_re: wave.psi.iter().map(|c| c.re).collect(),
            psi_im: wave.psi.iter().map(|c| c.im).collect(),
            v: wave.v.clone(),
            dx: wave.dx,
        }),
    })
}

fn export_quantum_2d(world: &World) -> Option<Quantum2dScenarioJson> {
    let wave = world.quantum_2d()?;
    Some(Quantum2dScenarioJson {
        nx: wave.nx,
        ny: wave.ny,
        dx: wave.dx,
        dy: wave.dy,
        // 1D版と同じ理由でレシピ側の波束パラメータは逆算できない。
        packet: GaussianPacket2dJson {
            x0: 0.0,
            y0: 0.0,
            sigma_x: 0.0,
            sigma_y: 0.0,
            k0: 0.0,
        },
        double_slit: None,
        raw_state: Some(Quantum2dRawStateJson {
            psi_re: wave.psi.iter().map(|c| c.re).collect(),
            psi_im: wave.psi.iter().map(|c| c.im).collect(),
            v: wave.v.clone(),
            nx: wave.nx,
            ny: wave.ny,
            dx: wave.dx,
            dy: wave.dy,
        }),
    })
}

fn export_brownian(world: &World) -> Option<BrownianScenarioJson> {
    let set = world.brownian()?;
    Some(BrownianScenarioJson {
        particle_count: set.position.len(),
        mass: set.mass,
        gamma: set.gamma,
        kb_t: set.kb_t,
        external_force: Some(vec3_to_array(set.external_force)),
        // 拡散後の粒子群に「共通の初期位置」は存在しない。`raw_state`が真。
        initial_position: None,
        raw_state: Some(BrownianRawStateJson {
            position: set.position.iter().copied().map(vec3_to_array).collect(),
            velocity: set.velocity.iter().copied().map(vec3_to_array).collect(),
            mass: set.mass,
            gamma: set.gamma,
            kb_t: set.kb_t,
            external_force: vec3_to_array(set.external_force),
        }),
    })
}

fn export_kinetic_gas(world: &World) -> Option<KineticGasScenarioJson> {
    let gas = world.kinetic_gas()?;
    Some(KineticGasScenarioJson {
        particle_count: gas.position.len(),
        mass: gas.mass,
        radius: gas.radius,
        box_size: vec3_to_array(gas.box_size),
        // レシピ側の`temperature`はマクスウェル分布のサンプリング温度。現在の
        // 実測温度を書いておく(人が読むためのメタデータ、復元には使われない)。
        temperature: gas.temperature(),
        raw_state: Some(KineticGasRawStateJson {
            position: gas.position.iter().copied().map(vec3_to_array).collect(),
            velocity: gas.velocity.iter().copied().map(vec3_to_array).collect(),
            mass: gas.mass,
            radius: gas.radius,
            box_size: vec3_to_array(gas.box_size),
            collision_count: gas.collision_count,
        }),
    })
}

fn export_ising(world: &World) -> Option<IsingScenarioJson> {
    let sim = world.ising()?;
    Some(IsingScenarioJson {
        l: sim.l,
        j_coupling: sim.j_coupling,
        temperature: sim.temperature,
        updates_per_step: Some(sim.updates_per_step),
        use_wolff: Some(sim.use_wolff),
        raw_state: Some(IsingRawStateJson {
            spins: sim.spins.clone(),
            l: sim.l,
            j_coupling: sim.j_coupling,
            temperature: sim.temperature,
            updates_per_step: sim.updates_per_step,
            use_wolff: sim.use_wolff,
        }),
    })
}

fn export_fdtd(world: &World) -> Option<FdtdScenarioJson> {
    let sim = world.fdtd()?;
    Some(FdtdScenarioJson {
        nx: sim.nx(),
        ny: sim.ny(),
        h: sim.h(),
        courant: Some(sim.dt / sim.h()),
        // レシピ側の`initial`は $E_z$ しか書けない(磁場が落ちる)。`raw_state`が真。
        initial: None,
        raw_state: Some(FdtdRawStateJson {
            ez: sim.ez_raw().to_vec(),
            hx: sim.hx_raw().to_vec(),
            hy: sim.hy_raw().to_vec(),
            dt: sim.dt,
        }),
    })
}

fn export_joints(world: &World, names: &HashMap<BodyId, String>) -> Vec<JointJson> {
    let m = world.mechanics();
    let live = |idx: usize| -> Option<BodyId> {
        names.keys().find(|id| id.index as usize == idx).copied()
    };
    let name_of =
        |idx: usize| -> Option<String> { live(idx).and_then(|id| names.get(&id).cloned()) };

    let mut out = Vec::new();
    for j in &m.joints {
        if j.disabled {
            continue;
        }
        let (Some(body_a), body_b) = (name_of(j.body_a), j.body_b.and_then(name_of)) else {
            continue;
        };
        if j.body_b.is_some() && body_b.is_none() {
            continue; // 相手が削除済み(=無効化されているはず、防御的にskip)。
        }
        out.push(JointJson::Distance {
            body_a,
            anchor_a: vec3_to_array(j.anchor_a),
            body_b,
            anchor_b: vec3_to_array(j.anchor_b),
            length: j.length,
        });
    }
    for j in &m.ball_joints {
        if j.disabled {
            continue;
        }
        let (Some(body_a), body_b) = (name_of(j.body_a), j.body_b.and_then(name_of)) else {
            continue;
        };
        if j.body_b.is_some() && body_b.is_none() {
            continue;
        }
        out.push(JointJson::Ball {
            body_a,
            anchor_a: vec3_to_array(j.anchor_a),
            body_b,
            anchor_b: vec3_to_array(j.anchor_b),
        });
    }
    for j in &m.slider_joints {
        if j.disabled {
            continue;
        }
        let (Some(body_a), body_b) = (name_of(j.body_a), j.body_b.and_then(name_of)) else {
            continue;
        };
        if j.body_b.is_some() && body_b.is_none() {
            continue;
        }
        out.push(JointJson::Slider {
            body_a,
            anchor_a: vec3_to_array(j.anchor_a),
            axis: vec3_to_array(j.axis_a),
            body_b,
            anchor_b: vec3_to_array(j.anchor_b),
            reference_relative_rotation: Some(quat_to_array(j.reference_relative_rotation)),
        });
    }
    for j in &m.wheel_joints {
        if j.disabled {
            continue;
        }
        let (Some(chassis), Some(wheel)) = (name_of(j.chassis), name_of(j.wheel)) else {
            continue;
        };
        out.push(JointJson::Wheel {
            chassis,
            wheel,
            anchor_chassis: vec3_to_array(j.anchor_chassis),
            rest_length: j.rest_length,
            suspension_axis: Some(vec3_to_array(j.suspension_axis)),
            axle_axis: Some(vec3_to_array(j.axle_axis)),
            frequency: Some(j.soft.frequency),
            damping_ratio: Some(j.soft.damping_ratio),
            steer_angle: Some(j.steer_angle),
            motor_speed: Some(j.motor_speed),
            motor_max_torque: Some(j.motor_max_torque),
        });
    }
    for j in &m.hinge_motors {
        if j.disabled {
            continue;
        }
        let Some(body) = name_of(j.body) else {
            continue;
        };
        out.push(JointJson::HingeMotor {
            body,
            axis: vec3_to_array(j.axis),
            reference_rotation: Some(quat_to_array(j.reference_rotation)),
            theta_target: j.theta_target,
            kp: j.kp,
            kd: j.kd,
            torque_max: j.torque_max,
            limit: j.limit,
        });
    }
    out
}

fn export_couplings(world: &World, names: &HashMap<BodyId, String>) -> Vec<CouplingJson> {
    let name_of = |idx: usize| -> Option<String> {
        names
            .iter()
            .find(|(id, _)| id.index as usize == idx)
            .map(|(_, n)| n.clone())
    };

    let mut out = Vec::new();
    for coupling in world.couplings_raw() {
        let c = coupling.as_ref();
        match c.kind() {
            sim_coupling::CouplingKind::ImageChargeForce => {
                if let Some(c) = c.as_any().downcast_ref::<sim_coupling::ImageChargeForce>() {
                    let Some(body) = name_of(c.body_index) else {
                        continue;
                    };
                    out.push(CouplingJson::ImageChargeForce {
                        body,
                        charge: c.charge,
                        plane_normal: vec3_to_array(c.plane_normal),
                        plane_d: c.plane_d,
                    });
                }
            }
            sim_coupling::CouplingKind::BoussinesqBuoyancy => {
                if let Some(c) = c
                    .as_any()
                    .downcast_ref::<sim_coupling::BoussinesqBuoyancy>()
                {
                    out.push(CouplingJson::BoussinesqBuoyancy {
                        thermal_node: c.thermal_node,
                        ambient_temperature: c.ambient_temperature,
                        thermal_expansion_coefficient: c.thermal_expansion_coefficient,
                    });
                }
            }
            sim_coupling::CouplingKind::DissipationToHeat => {
                if let Some(c) = c.as_any().downcast_ref::<sim_coupling::DissipationToHeat>() {
                    out.push(CouplingJson::DissipationToHeat {
                        thermal_node: c.thermal_node,
                        body_links: c
                            .body_links
                            .iter()
                            .filter_map(|link| {
                                name_of(link.body_index).map(|body| BodyThermalLinkJson {
                                    body,
                                    thermal_node: link.thermal_node,
                                    effusivity: Some(link.effusivity),
                                })
                            })
                            .collect(),
                    });
                }
            }
            sim_coupling::CouplingKind::PhaseChangeMorph => {
                if let Some(c) = c.as_any().downcast_ref::<sim_coupling::PhaseChangeMorph>() {
                    let Some(body) = name_of(c.body_index) else {
                        continue;
                    };
                    out.push(CouplingJson::PhaseChangeMorph {
                        body,
                        thermal_node: c.thermal_node,
                        melting_temperature: c.material.melting_temperature,
                        latent_heat_fusion: c.material.latent_heat_fusion,
                        specific_heat_solid: c.material.specific_heat_solid,
                        specific_heat_liquid: c.material.specific_heat_liquid,
                        initial_mass: c.initial_mass,
                        conductance: c.conductance,
                        // 既知の制限(モジュールdoc参照): 内部の相状態・繰り越し
                        // エンタルピーは非公開のため書き出せない。
                        initial_enthalpy: 0.0,
                        melt_spawn: None,
                    });
                }
            }
            sim_coupling::CouplingKind::MotorCoupling => {
                if let Some(c) = c.as_any().downcast_ref::<sim_coupling::MotorCoupling>() {
                    let Some(body) = name_of(c.body_index) else {
                        continue;
                    };
                    out.push(CouplingJson::MotorCoupling {
                        body,
                        axis: vec3_to_array(c.axis),
                        voltage_source_index: c.voltage_source_index,
                        torque_constant: c.torque_constant,
                    });
                }
            }
            sim_coupling::CouplingKind::SphRigid => {
                if let Some(c) = c.as_any().downcast_ref::<sim_coupling::SphRigid>() {
                    let Some(body) = name_of(c.body_index) else {
                        continue;
                    };
                    out.push(CouplingJson::SphRigid {
                        body,
                        radius: c.radius,
                        // 既知の制限(モジュールdoc参照): 実際の境界粒子確保数は
                        // 非公開。JSON側の再構成は新たに同数を確保し直す前提の値。
                        boundary_points: 0,
                    });
                }
            }
            sim_coupling::CouplingKind::GridFluidRigid => {
                if let Some(c) = c.as_any().downcast_ref::<sim_coupling::GridFluidRigid>() {
                    let Some(body) = name_of(c.body_index) else {
                        continue;
                    };
                    out.push(CouplingJson::GridFluidRigid {
                        body,
                        half_width: c.half_width,
                        half_height: c.half_height,
                    });
                }
            }
            sim_coupling::CouplingKind::LorentzForce => {
                if let Some(c) = c.as_any().downcast_ref::<sim_coupling::LorentzForce>() {
                    let Some(body) = name_of(c.body_index) else {
                        continue;
                    };
                    out.push(CouplingJson::LorentzForce {
                        body,
                        charge: c.charge,
                    });
                }
            }
            sim_coupling::CouplingKind::ConvectionLink => {
                if let Some(c) = c.as_any().downcast_ref::<sim_coupling::ConvectionLink>() {
                    out.push(CouplingJson::ConvectionLink {
                        fluid_node: c.fluid_node,
                        surface_node: c.surface_node,
                        area: c.area,
                        characteristic_length: c.characteristic_length,
                        fluid_thermal_conductivity: c.fluid_thermal_conductivity,
                        kinematic_viscosity: c.kinematic_viscosity,
                        prandtl_number: c.prandtl_number,
                        mode: convection_mode_to_json(c.mode),
                        thermal_expansion_coefficient: c.thermal_expansion_coefficient,
                    });
                }
            }
            sim_coupling::CouplingKind::PistonGas => {
                if let Some(c) = c.as_any().downcast_ref::<sim_coupling::PistonGas>() {
                    let Some(body) = name_of(c.body_index) else {
                        continue;
                    };
                    let initial_volume = world.gas().map(|g| g.volume).unwrap_or(0.0);
                    out.push(CouplingJson::PistonGas {
                        body,
                        axis: vec3_to_array(c.axis),
                        area: c.area,
                        // 既知の制限(モジュールdoc参照): 変位ゼロ点(基準位置・
                        // 基準体積)は非公開。現在の気体体積を新たな基準として書く。
                        initial_volume,
                    });
                }
            }
            sim_coupling::CouplingKind::JouleHeat => {
                if let Some(c) = c.as_any().downcast_ref::<sim_coupling::JouleHeat>() {
                    out.push(CouplingJson::JouleHeat {
                        thermal_node: c.thermal_node,
                        resistor_nodes: c.resistor_nodes.clone(),
                    });
                }
            }
            sim_coupling::CouplingKind::BrownianForce => {
                if let Some(c) = c.as_any().downcast_ref::<sim_coupling::BrownianForce>() {
                    let Some(body) = name_of(c.body_index) else {
                        continue;
                    };
                    out.push(CouplingJson::BrownianForce {
                        body,
                        radius: c.radius,
                        viscosity: c.viscosity,
                        thermal_node: c.thermal_node,
                        // 既知の制限(モジュールdoc参照): RNGストリーム位置は
                        // 非公開のため常に新規シードで書き出す。
                        seed: 0,
                        stream: 0,
                    });
                }
            }
            sim_coupling::CouplingKind::InductionCoupling => {
                if let Some(c) = c.as_any().downcast_ref::<sim_coupling::InductionCoupling>() {
                    let Some(body) = name_of(c.body_index) else {
                        continue;
                    };
                    out.push(CouplingJson::InductionCoupling {
                        body,
                        voltage_source_index: c.voltage_source_index,
                        length: c.length,
                        magnetic_field: c.magnetic_field,
                        axis: vec3_to_array(c.axis),
                    });
                }
            }
            sim_coupling::CouplingKind::BuoyancyDrag => {
                if let Some(c) = c.as_any().downcast_ref::<sim_coupling::BuoyancyDrag>() {
                    let Some(body) = name_of(c.body_index) else {
                        continue;
                    };
                    out.push(CouplingJson::BuoyancyDrag {
                        body,
                        water_level: c.water.as_ref().map(|w| w.water_level),
                        water_density: c.water.as_ref().map(|w| w.density),
                        atmosphere: c.atmosphere.as_ref().map(|a| AtmosphereJson {
                            density: a.density,
                            viscosity: a.viscosity,
                        }),
                        lift: c.lift.as_ref().map(lift_model_to_json),
                    });
                }
            }
            sim_coupling::CouplingKind::Noop => {}
        }
    }
    out
}

fn convection_mode_to_json(mode: sim_coupling::ConvectionMode) -> ConvectionModeJson {
    match mode {
        sim_coupling::ConvectionMode::NaturalVerticalPlate => {
            ConvectionModeJson::NaturalVerticalPlate
        }
        sim_coupling::ConvectionMode::NaturalSphere => ConvectionModeJson::NaturalSphere,
        sim_coupling::ConvectionMode::ForcedSphere => ConvectionModeJson::ForcedSphere,
        sim_coupling::ConvectionMode::ForcedFlatPlate => ConvectionModeJson::ForcedFlatPlate,
    }
}

fn lift_model_to_json(lift: &sim_coupling::LiftModel) -> LiftModelJson {
    match lift {
        sim_coupling::LiftModel::Wing {
            area,
            chord_local,
            span_local,
            control_surface_deflection,
        } => LiftModelJson::Wing {
            area: *area,
            chord_local: vec3_to_array(*chord_local),
            span_local: vec3_to_array(*span_local),
            control_surface_deflection: *control_surface_deflection,
        },
        sim_coupling::LiftModel::MagnusSphere { radius } => {
            LiftModelJson::MagnusSphere { radius: *radius }
        }
    }
}

fn export_probes(world: &World, names: &HashMap<BodyId, String>) -> Vec<ProbeJson> {
    let mut out = Vec::new();
    let mut handle = 0usize;
    while let Some(probe) = world.probe(handle) {
        if let Some(json) = probe_target_to_json(&probe.target, names) {
            out.push(json);
        }
        handle += 1;
    }
    out
}

fn probe_target_to_json(
    target: &ProbeTarget,
    names: &HashMap<BodyId, String>,
) -> Option<ProbeJson> {
    Some(match target {
        ProbeTarget::BodyPosY(id) => ProbeJson::BodyPosY(names.get(id)?.clone()),
        ProbeTarget::BodyPosX(id) => ProbeJson::BodyPosX(names.get(id)?.clone()),
        ProbeTarget::BodySpeed(id) => ProbeJson::BodySpeed(names.get(id)?.clone()),
        ProbeTarget::NodeTemp(i) => ProbeJson::NodeTemp(*i),
        ProbeTarget::AstroPosX(i) => ProbeJson::AstroPosX(*i),
        ProbeTarget::AstroPosY(i) => ProbeJson::AstroPosY(*i),
        ProbeTarget::AstroVelX(i) => ProbeJson::AstroVelX(*i),
        ProbeTarget::AstroVelY(i) => ProbeJson::AstroVelY(*i),
        ProbeTarget::CircuitCurrent(i) => ProbeJson::CircuitCurrent(*i),
        ProbeTarget::SoftBodyPosX(i) => ProbeJson::SoftBodyPosX(*i),
        ProbeTarget::SoftBodyPosY(i) => ProbeJson::SoftBodyPosY(*i),
        ProbeTarget::RodTemp(i) => ProbeJson::RodTemp(*i),
        ProbeTarget::GridFluidMeanV => ProbeJson::GridFluidMeanV,
        ProbeTarget::GridFluidRmsV => ProbeJson::GridFluidRmsV,
        ProbeTarget::SphParticlePosY(i) => ProbeJson::SphParticlePosY(*i),
        ProbeTarget::SphParticleDensity(i) => ProbeJson::SphParticleDensity(*i),
        ProbeTarget::CircuitNodeVoltage(i) => ProbeJson::CircuitNodeVoltage(*i),
        ProbeTarget::QuantumNorm => ProbeJson::QuantumNorm,
        ProbeTarget::QuantumMeanX => ProbeJson::QuantumMeanX,
        ProbeTarget::QuantumEnergy => ProbeJson::QuantumEnergy,
        ProbeTarget::QuantumTransmission(i) => ProbeJson::QuantumTransmission(*i),
        ProbeTarget::GasTemperature => ProbeJson::GasTemperature,
        ProbeTarget::GasPressure => ProbeJson::GasPressure,
        ProbeTarget::IsingMagnetization => ProbeJson::IsingMagnetization,
        ProbeTarget::IsingEnergyPerSpin => ProbeJson::IsingEnergyPerSpin,
        ProbeTarget::BrownianMsd => ProbeJson::BrownianMsd,
        ProbeTarget::FdtdEz(i, j) => ProbeJson::FdtdEz(*i, *j),
        ProbeTarget::FdtdEnergy => ProbeJson::FdtdEnergy,
        // 既知の制限(モジュールdoc参照): シーンJSON側に対応する`ProbeJson`が
        // 無い(設計として意図的に除外されている、`ProbeTarget`のdoc参照)。
        ProbeTarget::LedgerKinetic | ProbeTarget::StateHashDigest => return None,
    })
}

fn vec3_to_array(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

fn quat_to_array(q: sim_math::Quat) -> [f64; 4] {
    [q.x, q.y, q.z, q.w]
}

/// `Shape` → `ShapeJson`(**残タスク完遂の縦串⑤前後でCompound/ConvexMeshを
/// 追加**)。`Compound`は子を再帰的に変換する——`scenario::shape_json_to_shape`
/// (逆方向の変換)と対になる。
/// Body の実際の `Shape`(Compound/ConvexMesh の入れ子構造も含む)を、
/// シーンJSON向けの`ShapeJson`へ変換する。`shape_json_to_shape`の逆写像。
pub fn shape_to_shape_json(shape: &Shape) -> ShapeJson {
    match shape {
        Shape::Box { half_extents } => ShapeJson::Box {
            half: vec3_to_array(*half_extents),
        },
        Shape::Sphere { radius } => ShapeJson::Sphere { radius: *radius },
        Shape::Capsule {
            radius,
            half_height,
        } => ShapeJson::Capsule {
            radius: *radius,
            half_height: *half_height,
        },
        Shape::Plane { normal, d } => ShapeJson::Plane {
            normal: vec3_to_array(*normal),
            d: *d,
        },
        Shape::Compound { children } => ShapeJson::Compound {
            children: children
                .iter()
                .map(|(transform, child_shape)| CompoundChildJson {
                    position: vec3_to_array(transform.position),
                    rotation: quat_to_array(transform.rotation),
                    shape: Box::new(shape_to_shape_json(child_shape)),
                })
                .collect(),
        },
        Shape::ConvexMesh { vertices } => ShapeJson::ConvexMesh {
            vertices: vertices.iter().map(|v| vec3_to_array(*v)).collect(),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Scenario, WorldOptions};
    use sim_mechanics::{
        BallJoint, DistanceJoint, HingeMotorPd, RigidBodyDesc, SliderJoint, WheelJoint,
    };

    fn steel(world: &World) -> sim_core::MaterialId {
        world.materials().find_by_name("鋼(炭素鋼)").unwrap()
    }

    /// D24相当の縮図: シャシー+車輪(WheelJoint、操舵・駆動あり)、振り子(BallJoint、
    /// ワールド固定点)、ドア(HingeMotorPd)、ピストン(SliderJoint)。
    fn build_mechanics_scene() -> World {
        let mut world = World::new(WorldOptions {
            gravity: 9.8,
            dt: 1.0 / 120.0,
            seed: 0,
        });
        let mat = steel(&world);

        let mut chassis_desc = RigidBodyDesc::dynamic(
            sim_mechanics::Shape::Box {
                half_extents: Vec3::new(1.0, 0.3, 2.0),
            },
            mat,
        );
        chassis_desc.transform.position = Vec3::new(0.0, 1.0, 0.0);
        let chassis = world.create_body(chassis_desc);

        let mut wheel_desc =
            RigidBodyDesc::dynamic(sim_mechanics::Shape::Sphere { radius: 0.4 }, mat);
        wheel_desc.transform.position = Vec3::new(1.0, 0.5, 1.5);
        let wheel = world.create_body(wheel_desc);

        let mut wheel_joint = WheelJoint::new(
            chassis.index as usize,
            wheel.index as usize,
            Vec3::new(1.0, -0.3, 1.5),
            0.5,
        );
        wheel_joint.steer_angle = 0.2;
        wheel_joint.motor_speed = 3.0;
        wheel_joint.motor_max_torque = 50.0;
        world.mechanics_mut().wheel_joints.push(wheel_joint);

        let mut bob_desc =
            RigidBodyDesc::dynamic(sim_mechanics::Shape::Sphere { radius: 0.2 }, mat);
        bob_desc.transform.position = Vec3::new(-3.0, 0.0, 0.0);
        let bob = world.create_body(bob_desc);
        world.mechanics_mut().add_ball_joint(BallJoint {
            body_a: bob.index as usize,
            anchor_a: Vec3::ZERO,
            body_b: None,
            anchor_b: Vec3::new(-3.0, 3.0, 0.0),
            disabled: false,
        });

        let mut door_desc = RigidBodyDesc::dynamic(
            sim_mechanics::Shape::Box {
                half_extents: Vec3::new(0.5, 1.0, 0.05),
            },
            mat,
        );
        door_desc.transform.position = Vec3::new(5.0, 1.0, 0.0);
        let door = world.create_body(door_desc);
        let door_rotation = world.mechanics().bodies.rotation[door.index as usize];
        world.mechanics_mut().add_hinge_motor(HingeMotorPd {
            body: door.index as usize,
            axis: Vec3::new(0.0, 1.0, 0.0),
            reference_rotation: door_rotation,
            theta_target: 0.7,
            kp: 10.0,
            kd: 1.0,
            torque_max: 20.0,
            limit: Some((-1.0, 1.0)),
            disabled: false,
        });

        let mut piston_desc = RigidBodyDesc::dynamic(
            sim_mechanics::Shape::Box {
                half_extents: Vec3::new(0.1, 0.1, 0.5),
            },
            mat,
        );
        piston_desc.transform.position = Vec3::new(8.0, 1.0, 0.0);
        let piston = world.create_body(piston_desc);
        let slider = SliderJoint::new(
            &world.mechanics().bodies,
            piston.index as usize,
            Vec3::ZERO,
            Vec3::new(0.0, 0.0, 1.0),
            None,
            Vec3::new(8.0, 1.0, 0.0),
        );
        world.mechanics_mut().add_slider_joint(slider);

        let mut anchor_desc =
            RigidBodyDesc::dynamic(sim_mechanics::Shape::Sphere { radius: 0.1 }, mat);
        anchor_desc.transform.position = Vec3::new(-6.0, 5.0, 0.0);
        let anchor = world.create_body(anchor_desc);
        world.mechanics_mut().add_distance_joint(DistanceJoint {
            body_a: anchor.index as usize,
            anchor_a: Vec3::ZERO,
            body_b: None,
            anchor_b: Vec3::new(-6.0, 8.0, 0.0),
            length: 3.0,
            disabled: false,
        });

        world
    }

    #[test]
    fn mechanics_round_trip_preserves_state_hash_immediately_and_after_stepping() {
        let world = build_mechanics_scene();
        let scenario = to_scenario(&world, "round-trip");
        let reloaded = World::from_scenario(&scenario).expect("round trip must parse back");
        assert_eq!(
            world.state_hash(),
            reloaded.state_hash(),
            "state_hash must match immediately after reload"
        );

        let mut world = world;
        let mut reloaded = reloaded;
        for _ in 0..30 {
            world.step();
            reloaded.step();
        }
        assert_eq!(
            world.state_hash(),
            reloaded.state_hash(),
            "state_hash must still match after stepping (joint reference state preserved)"
        );
    }

    #[test]
    fn json_round_trip_via_serde_preserves_state_hash() {
        let world = build_mechanics_scene();
        let scenario = to_scenario(&world, "round-trip");
        let json = serde_json::to_string(&scenario).expect("Scenario must serialize");
        let parsed: Scenario = serde_json::from_str(&json).expect("Scenario must deserialize");
        let reloaded = World::from_scenario(&parsed).expect("round trip must parse back");
        assert_eq!(world.state_hash(), reloaded.state_hash());
    }

    #[test]
    fn couplings_thermal_circuit_astro_gas_round_trip() {
        let mut world = World::new(WorldOptions {
            gravity: 9.8,
            dt: 1.0 / 120.0,
            seed: 0,
        });
        let mat = steel(&world);
        let mut desc = RigidBodyDesc::dynamic(sim_mechanics::Shape::Sphere { radius: 0.3 }, mat);
        desc.transform.position = Vec3::new(0.0, 2.0, 0.0);
        let body = world.create_body(desc);
        world.add_coupling(Box::new(sim_coupling::LorentzForce {
            body_index: body.index as usize,
            charge: 1e-6,
        }));

        let mut thermal = sim_thermal::ThermalSolver::new(293.15);
        thermal.add_node(sim_thermal::ThermalNode {
            temperature: 350.0,
            heat_capacity: 100.0,
            emissivity: 0.5,
            area: 1.0,
            heat_accum: 0.0,
            convection_coefficient: 5.0,
        });
        world.enable_thermal(thermal);

        let mut circuit = sim_em::Circuit::new(3);
        circuit.add_voltage_source(1, sim_em::GROUND, 10.0);
        circuit.add_resistor(1, 2, 100.0);
        circuit.add_capacitor(2, sim_em::GROUND, 1e-3, 4.2);
        world.enable_circuit(circuit);

        let mut astro = sim_astro::NBodySystem::new(0.0);
        astro.position.push(Vec3::ZERO);
        astro.velocity.push(Vec3::ZERO);
        astro.mass.push(1.0e24);
        astro.position.push(Vec3::new(1.0e7, 0.0, 0.0));
        astro.velocity.push(Vec3::new(0.0, 1000.0, 0.0));
        astro.mass.push(500.0);
        world.enable_astro(astro);

        world.enable_gas(sim_thermal::GasCompartment {
            n_moles: 1.0,
            volume: 0.01,
            temperature: 300.0,
            gas: sim_thermal::GasSpecies::AIR,
        });

        let scenario = to_scenario(&world, "domains");
        let reloaded = World::from_scenario(&scenario).expect("round trip must parse back");
        assert_eq!(world.state_hash(), reloaded.state_hash());
    }

    // -----------------------------------------------------------------------
    // **生状態スナップショット(`raw_state`)の往復テスト**(11ドメイン)。
    //
    // どのテストも「作った直後」ではなく **数十step回して時間発展させてから**
    // エクスポートする——これがこの増分の要点で、$t=0$ の状態なら構築レシピでも
    // 往復できてしまうため、レシピでは表せない状態になっていることを確かめないと
    // 何もテストしていないことになる。
    // -----------------------------------------------------------------------

    fn empty_world() -> World {
        World::new(WorldOptions {
            gravity: 9.8,
            dt: 1.0 / 120.0,
            seed: 0,
        })
    }

    /// 時間発展 → エクスポート → 再インポートで`state_hash`が
    /// 「復元直後」と「双方をさらに`extra_steps`回したあと」の両方で一致することを
    /// 確かめる共通手順。`steps`は事前に回す回数(時間発展させるため)。
    fn assert_round_trip(mut world: World, steps: usize, extra_steps: usize, label: &str) {
        for _ in 0..steps {
            world.step();
        }
        let scenario = to_scenario(&world, label);
        let mut reloaded = World::from_scenario(&scenario).expect("round trip must parse back");
        assert_eq!(
            world.state_hash(),
            reloaded.state_hash(),
            "{label}: state_hash must match immediately after reload"
        );
        for _ in 0..extra_steps {
            world.step();
            reloaded.step();
        }
        assert_eq!(
            world.state_hash(),
            reloaded.state_hash(),
            "{label}: state_hash must still match after stepping both forward"
        );
    }

    /// 乱数を消費するドメイン(ブラウン運動・イジング)用。`Scenario::seed`が常に0で
    /// 書き出される既知の制限(モジュールdoc参照)により、**元の`World`は既にRNGを
    /// 消費済み・復元後の`World`はストリーム先頭**という差が残るため、
    /// 「復元直後の一致」と「復元経路自体の決定論(2回復元して同じだけ回したら一致)」
    /// に分けて確かめる。
    fn assert_round_trip_rng_domain(
        mut world: World,
        steps: usize,
        extra_steps: usize,
        label: &str,
    ) {
        for _ in 0..steps {
            world.step();
        }
        let scenario = to_scenario(&world, label);
        let mut a = World::from_scenario(&scenario).expect("round trip must parse back");
        let mut b = World::from_scenario(&scenario).expect("round trip must parse back");
        assert_eq!(
            world.state_hash(),
            a.state_hash(),
            "{label}: state_hash must match immediately after reload"
        );
        for _ in 0..extra_steps {
            a.step();
            b.step();
        }
        assert_eq!(
            a.state_hash(),
            b.state_hash(),
            "{label}: reload path must be deterministic when stepped forward"
        );
    }

    #[test]
    fn soft_body_raw_state_round_trip_after_stepping() {
        let mut world = empty_world();
        // たるんだロープ(端点ピン留め)+ 曲げ拘束 + 体積拘束を持つ非自明な構成。
        let mut body = sim_mechanics::rope(
            Vec3::new(0.0, 5.0, 0.0),
            Vec3::new(4.0, 5.0, 0.0),
            8,
            0.1,
            5.0,
            1e-8,
        );
        body.pin(0);
        body.add_bending_constraint(0, 1, 2, 1e-4);
        body.add_bending_constraint(3, 4, 5, 1e-4);
        body.add_volume_constraint([0, 1, 2, 3], 1e-5);
        body.damping = 0.7;
        body.substeps = 3;
        body.iterations = 4;
        world.enable_soft_body(body);
        assert_round_trip(world, 25, 20, "soft_body");
    }

    #[test]
    fn grid_fluid_raw_state_round_trip_after_stepping() {
        let mut world = empty_world();
        let mut fluid = sim_fluid::GridFluid2D::new(16, 12, 0.05)
            .with_boundary(sim_fluid::GridBoundary::Channel { inflow_speed: 1.0 });
        fluid.kinematic_viscosity = 1e-4;
        fluid.vorticity_confinement_epsilon = 0.3;
        // 円柱障害物: `solids`(矩形/円の列挙)では書けても、時間発展後の
        // セル種別配列を戻せるのは`raw_state`だけ。
        fluid.set_solid_cells(|x, y| {
            let (dx, dy) = (x - 0.2, y - 0.3);
            (dx * dx + dy * dy < 0.06 * 0.06).then_some(Vec3::ZERO)
        });
        world.enable_grid_fluid(fluid);
        assert_round_trip(world, 20, 15, "grid_fluid");
    }

    #[test]
    fn grid_fluid_3d_raw_state_round_trip_after_stepping() {
        let mut world = empty_world();
        // 3Dは1stepが重いので小さく取る(`GridFluid3DScenarioJson`のdoc参照)。
        let mut fluid = sim_fluid::GridFluid3D::new(10, 8, 8, 0.1);
        fluid.kinematic_viscosity = 1e-4;
        for k in 0..8 {
            for j in 0..8 {
                for i in 0..10 {
                    let idx = i + 10 * (j + 8 * k);
                    fluid.u[idx] = 0.5 + 0.1 * (j as f64);
                    fluid.v[idx] = 0.05 * (i as f64 - 5.0);
                    if (2..5).contains(&i) && (2..5).contains(&j) {
                        fluid.smoke_density[idx] = 1.0;
                    }
                }
            }
        }
        fluid.set_solid_cells(|x, y, z| {
            ((x - 0.5).abs() < 0.1 && (y - 0.4).abs() < 0.1 && (z - 0.4).abs() < 0.1)
                .then_some(Vec3::ZERO)
        });
        world.enable_grid_fluid_3d(fluid);
        assert_round_trip(world, 6, 5, "grid_fluid_3d");
    }

    #[test]
    fn conduction_rod_raw_state_round_trip_after_stepping() {
        let mut world = empty_world();
        // 材質プロファイル(群7の $k_i$ 可変・$\rho c_p$ つき)を与えた棒。
        let mut rod = sim_thermal::ConductionRod1D::new(24, 0.5, 300.0, 1.1e-4);
        rod.set_boundary_temperatures(500.0, 280.0);
        let conductivity: Vec<f64> = (0..24).map(|i| 40.0 + 3.0 * i as f64).collect();
        let rod = rod.with_material_profile(conductivity, 3.5e6, 2.5e-4);
        world.enable_conduction_rod(rod);
        assert_round_trip(world, 30, 20, "conduction_rod");
    }

    #[test]
    fn sph_raw_state_round_trip_after_stepping() {
        let mut world = empty_world();
        let spacing = 0.05;
        let mut fluid = sim_fluid::SphFluid::new(2.0 * spacing, 1000.0, 30.0);
        fluid.mass = 1000.0 * spacing * spacing * spacing;
        fluid.viscosity_alpha = 0.1;
        for ix in 0..4 {
            for iy in 0..4 {
                for iz in 0..3 {
                    fluid.add_particle(
                        Vec3::new(
                            0.1 + ix as f64 * spacing,
                            0.3 + iy as f64 * spacing,
                            0.1 + iz as f64 * spacing,
                        ),
                        Vec3::new(0.2, 0.0, 0.0),
                    );
                }
            }
        }
        for ix in 0..8 {
            for iz in 0..6 {
                fluid.add_boundary_particle(Vec3::new(
                    ix as f64 * spacing,
                    0.0,
                    iz as f64 * spacing,
                ));
            }
        }
        world.enable_sph(fluid);
        assert_round_trip(world, 20, 15, "sph");
    }

    #[test]
    fn quantum_1d_raw_state_round_trip_after_stepping() {
        let mut world = empty_world();
        let mut wave = sim_quantum::WaveFunction1D::new(128, 0.1);
        wave.set_gaussian_wave_packet(4.0, 0.6, 5.0);
        // 矩形障壁(D27相当)。
        for i in 0..128 {
            let x = i as f64 * 0.1;
            if (7.0..7.6).contains(&x) {
                wave.v[i] = 12.0;
            }
        }
        world.enable_quantum_1d(wave);
        assert_round_trip(world, 25, 20, "quantum_1d");
    }

    #[test]
    fn quantum_2d_raw_state_round_trip_after_stepping() {
        let mut world = empty_world();
        let mut wave = sim_quantum::WaveFunction2D::new(32, 32, 0.2, 0.2);
        wave.set_gaussian_wave_packet(1.5, 3.2, 0.5, 0.8, 4.0);
        for iy in 0..32 {
            let y = iy as f64 * 0.2;
            if (y - 2.6).abs() <= 0.25 || (y - 3.8).abs() <= 0.25 {
                continue;
            }
            for ix in 0..32 {
                let x = ix as f64 * 0.2;
                if (x - 3.0).abs() <= 0.2 {
                    wave.v[iy * 32 + ix] = 60.0;
                }
            }
        }
        world.enable_quantum_2d(wave);
        assert_round_trip(world, 15, 10, "quantum_2d");
    }

    #[test]
    fn brownian_raw_state_round_trip_after_stepping() {
        let mut world = empty_world();
        let mut set = sim_statistical::BrownianParticleSet::new(1e-15, 1e-8, 4.11e-21);
        set.external_force = Vec3::new(0.0, -1e-14, 0.0);
        for i in 0..40 {
            set.add_particle(Vec3::new(i as f64 * 1e-7, 0.0, 0.0), Vec3::ZERO);
        }
        world.enable_brownian(set);
        assert_round_trip_rng_domain(world, 25, 20, "brownian");
    }

    #[test]
    fn kinetic_gas_raw_state_round_trip_after_stepping() {
        let mut world = empty_world();
        let box_size = Vec3::new(1e-7, 1e-7, 1e-7);
        let mut gas = sim_statistical::GasSim::new(4.65e-26, 1e-10, box_size);
        let mut rng = sim_math::SimRng::new(7, 0x67617300);
        for _ in 0..60 {
            let position = Vec3::new(
                1e-10 + rng.next_f64() * (1e-7 - 2e-10),
                1e-10 + rng.next_f64() * (1e-7 - 2e-10),
                1e-10 + rng.next_f64() * (1e-7 - 2e-10),
            );
            let sigma = (sim_statistical::BOLTZMANN_CONSTANT * 300.0 / 4.65e-26).sqrt();
            gas.add_particle(position, rng.maxwell_boltzmann_velocity(sigma));
        }
        world.enable_kinetic_gas(gas);
        assert_round_trip(world, 30, 20, "kinetic_gas");
    }

    #[test]
    fn ising_raw_state_round_trip_after_stepping() {
        let mut world = empty_world();
        let mut rng = sim_math::SimRng::new(3, 0x6973696e);
        let mut sim = sim_statistical::IsingSim::new(12, 1.0, 2.1, &mut rng);
        sim.updates_per_step = 3;
        world.enable_ising(sim);
        assert_round_trip_rng_domain(world, 25, 20, "ising");
    }

    #[test]
    fn fdtd_raw_state_round_trip_after_stepping() {
        let mut world = empty_world();
        let mut sim = sim_em::FdtdSim2D::new(24, 20, 0.02, 0.5);
        for j in 1..19 {
            for i in 1..23 {
                let sx = (std::f64::consts::PI * i as f64 / 23.0).sin();
                let sy = (2.0 * std::f64::consts::PI * j as f64 / 19.0).sin();
                sim.set_ez(i, j, 1.5 * sx * sy);
            }
        }
        world.enable_fdtd(sim);
        // **磁場が復元されていることを実際に効かせるための30step**。`initial`
        // (Ezしか書けない)経路だと、ここで必ずハッシュがずれる。
        assert_round_trip(world, 30, 25, "fdtd");
    }

    /// FDTDの`raw_state`から磁場を落とすと往復が壊れることを、対照として固定する
    /// (「Ezだけでは足りない」という`FdtdRawStateJson`のdocの主張の裏取り)。
    #[test]
    fn fdtd_round_trip_breaks_without_magnetic_field() {
        let mut world = empty_world();
        let mut sim = sim_em::FdtdSim2D::new(16, 16, 0.02, 0.5);
        sim.set_ez(8, 8, 1.0);
        world.enable_fdtd(sim);
        for _ in 0..12 {
            world.step();
        }
        let mut scenario = to_scenario(&world, "fdtd-no-h");
        let raw = scenario.fdtd.as_mut().unwrap().raw_state.as_mut().unwrap();
        let (hx_len, hy_len) = (raw.hx.len(), raw.hy.len());
        raw.hx = vec![0.0; hx_len];
        raw.hy = vec![0.0; hy_len];
        let mut reloaded = World::from_scenario(&scenario).expect("must still parse");
        let mut world = world;
        world.step();
        reloaded.step();
        assert_ne!(
            world.state_hash(),
            reloaded.state_hash(),
            "磁場を落とせば次のstepでずれるはず(落としていないから一致している、という\
             取り違えを防ぐための対照)"
        );
    }

    /// serde 往復(JSON文字列を経由)。**`psi`の実部・虚部の平行配列**と
    /// **`solid_cells`のbool配列**という、`raw_state`で新たに入った2つの表現が
    /// 文字列を通しても壊れないことを確かめる。
    #[test]
    fn json_round_trip_via_serde_preserves_raw_state_domains() {
        let mut world = empty_world();
        let mut wave = sim_quantum::WaveFunction1D::new(64, 0.15);
        wave.set_gaussian_wave_packet(3.0, 0.7, 4.0);
        world.enable_quantum_1d(wave);

        let mut fluid = sim_fluid::GridFluid2D::new(12, 10, 0.05);
        fluid.u.iter_mut().for_each(|x| *x = 0.8);
        fluid.set_solid_cells(|x, y| {
            ((x - 0.2).abs() < 0.06 && (y - 0.25).abs() < 0.06).then_some(Vec3::ZERO)
        });
        world.enable_grid_fluid(fluid);

        let mut sim = sim_em::FdtdSim2D::new(16, 16, 0.02, 0.5);
        sim.set_ez(8, 8, 1.0);
        world.enable_fdtd(sim);

        for _ in 0..15 {
            world.step();
        }
        let scenario = to_scenario(&world, "serde-raw-state");
        let json = serde_json::to_string(&scenario).expect("Scenario must serialize");
        let parsed: Scenario = serde_json::from_str(&json).expect("Scenario must deserialize");
        let mut reloaded = World::from_scenario(&parsed).expect("round trip must parse back");
        assert_eq!(world.state_hash(), reloaded.state_hash());

        let mut world = world;
        for _ in 0..15 {
            world.step();
            reloaded.step();
        }
        assert_eq!(world.state_hash(), reloaded.state_hash());
    }

    /// **後方互換の担保**: `raw_state`を持たないシーンJSON(=既存の`scenes/*.json`と
    /// 同じ形)が、これまでどおり構築レシピ経路で読めること。`#[serde(default)]`が
    /// 効いていることの直接の確認でもある。
    #[test]
    fn scenes_without_raw_state_still_load_via_construction_recipe() {
        let json = r#"{
            "name": "recipe-only",
            "world": { "gravity": 9.8, "dt": 0.008333333333333333 },
            "sph": {
                "h": 0.1, "rest_density": 1000.0, "sound_speed": 30.0,
                "particle_mass": 0.125,
                "blocks": [{ "min": [0.0, 0.0, 0.0], "counts": [2, 2, 2], "spacing": 0.05 }]
            },
            "ising": { "l": 4, "j_coupling": 1.0, "temperature": 2.0 },
            "fdtd": {
                "nx": 8, "ny": 8, "h": 0.02,
                "initial": { "pulse": { "i": 4, "j": 4, "amplitude": 1.0, "width": 0.05 } }
            },
            "conduction_rod": {
                "node_count": 8, "length": 0.4, "initial_temperature": 300.0,
                "thermal_diffusivity": 1.0e-4
            }
        }"#;
        let scenario: Scenario = serde_json::from_str(json).expect("既存形のJSONが読めること");
        assert!(scenario.sph.as_ref().unwrap().raw_state.is_none());
        assert!(scenario.ising.as_ref().unwrap().raw_state.is_none());
        assert!(scenario.fdtd.as_ref().unwrap().raw_state.is_none());
        assert!(scenario
            .conduction_rod
            .as_ref()
            .unwrap()
            .raw_state
            .is_none());
        let world = World::from_scenario(&scenario).expect("レシピ経路で構築できること");
        assert_eq!(world.sph().unwrap().position.len(), 8);
        assert_eq!(world.ising().unwrap().spins.len(), 16);
        assert!(world.fdtd().unwrap().ez(4, 4) > 0.0);
        assert_eq!(world.conduction_rod().unwrap().temperature.len(), 8);
    }
}
