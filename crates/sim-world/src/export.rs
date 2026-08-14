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
//! - `grid_fluid`/`grid_fluid_3d`/`sph`/`soft_body`/`quantum_1d`/`quantum_2d`/
//!   `brownian`/`kinetic_gas`/`ising`/`fdtd`: シーンJSON側のスキーマが「構築レシピ」
//!   (例: 波束の中心・分散、SPH粒子を敷き詰める直方体ブロック)であって「状態の
//!   スナップショット」ではないため、時間発展した後の実行中`World`から正確に
//!   逆算することができない——**構造的にこのスキーマでは表現不能**であり、
//!   生値スナップショット形式のスキーマ拡張が別途要る(後続増分)。
//!   このエクスポートは現時点でこれらのドメインを`None`のまま出力する
//!   (存在ごと落ちるのではなく、実装漏れとして明示する)。

use crate::{BodyId, ProbeTarget, World};
use sim_math::Vec3;
use sim_mechanics::{BodyType, DragModel, Shape};
use std::collections::HashMap;

use crate::scenario::{
    AstroBodyJson, AstroScenarioJson, AtmosphereJson, AtmosphericDragJson, BodyScenarioDesc,
    BodyThermalLinkJson, CapacitorJson, CircuitScenarioJson, ConvectionModeJson, CouplingJson,
    DiodeJson, FluidJson, GasScenarioJson, InductorJson, JointJson, LiftModelJson,
    MaterialOverride, ProbeJson, RelativisticCorrectionJson, ResistorJson, Scenario, ShapeJson,
    SwitchJson, ThermalLinkJson, ThermalNodeJson, ThermalScenarioJson, VoltageSourceJson,
    WorldScenarioOptions,
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
        world: export_world_options(world),
        materials,
        bodies: export_bodies(world, &live_bodies, &names, &material_name_of),
        fluids: export_fluids(world),
        thermal: export_thermal(world),
        joints: export_joints(world, &names),
        couplings: export_couplings(world, &names),
        circuit: export_circuit(world),
        astro: export_astro(world),
        soft_body: None,
        grid_fluid: None,
        grid_fluid_3d: None,
        conduction_rod: None,
        sph: None,
        gas: export_gas(world),
        quantum_1d: None,
        quantum_2d: None,
        brownian: None,
        kinetic_gas: None,
        ising: None,
        fdtd: None,
        probes: export_probes(world, &names),
        prediction_prompts: Vec::new(),
    }
}

fn export_world_options(world: &World) -> WorldScenarioOptions {
    let mechanics = world.mechanics();
    WorldScenarioOptions {
        gravity: mechanics.gravity,
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
            let shape = bodies.shape_of(idx);
            let shape_json = match shape {
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
                Shape::Compound { .. } | Shape::ConvexMesh { .. } => {
                    // 到達しない: これらの形状は質量・慣性計算が `todo!()` で
                    // パニックする(task #7)ため、生きたボディとして存在し得ない。
                    unreachable!("Compound/ConvexMesh cannot exist on a live body yet (task #7)")
                }
            };
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
        } => LiftModelJson::Wing {
            area: *area,
            chord_local: vec3_to_array(*chord_local),
            span_local: vec3_to_array(*span_local),
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
}
