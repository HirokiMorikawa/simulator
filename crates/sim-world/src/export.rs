//! `World → Scenario`逆写像(統合エディタ実装計画の縦串①、
//! docs/reviews/2026-08-14-editor-implementation-plan.md 参照)。
//!
//! `from_scenario`(`Scenario → World`)は既に存在するが、逆方向(実行中の`World`を
//! 編集可能なシーンドキュメントへ書き戻す)は無かった——手書きの
//! `sim-wasm::export_scene_json`は bodies の3形状しか書けず、joints・couplings・
//! fluids・thermal・circuit・probesは無言で脱落していた。この関数がその置き換え。
//!
//! **既知の制限(縮約、正直な記録)**:
//! - `kinetic_gas`: 圧力測定用の壁運動量アキュムレータ(`sim_statistical::GasSim`の
//!   `wall_impulse_accum`/`wall_impulse_time`)が非公開のため書き出せない。
//!   `GasSim`の`state_hash`はこの2つを含まない(位置・速度・`collision_count`のみ)
//!   ので決定論replayには影響せず、**影響するのは`pressure()`の時間平均窓が
//!   復元時点から引き直されること**だけである(`reset_pressure_accumulator`を
//!   呼んだのと同じ状態になる)。窓を取り直せば同じ圧力へ収束する。
//! - `bodies`の力アキュムレータ(`sim_mechanics::RigidBodySet`の
//!   `force_accum`/`torque_accum`): シーンJSONに書く場所が無い(`BodyScenarioDesc`は
//!   位置・姿勢・速度までで、`state_hash`もこの2つを含まない)。**post 相の結合が
//!   積んだ反力は次stepの積分が消費する**ため、post 相を持つ結合
//!   (`PistonGas`・`GridFluidRigid`等)と動的剛体を組み合わせたシーンでは、
//!   エクスポート→再インポート後の**最初の1stepだけ**その反力が抜ける。
//!   2step目以降は同じ量が積み直されるので、ずれは初回の $F\Delta t/m$ 一回分に
//!   留まる。**結合の内部基準値(下記の解消済み)とは別問題**であり、剛体スキーマ
//!   側の増分になる(本増分の対象外、`PistonGas`の往復テストが`Kinematic`
//!   ピストンを使うのはこの欠落を混ぜないため)。
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
//!
//! **解消済みの制限(PRNGのストリーム位置、`Scenario::rng_state`)**:
//! `seed`は今も常に`0`を書く(`SimRng::new`が種を状態へ畳み込んだ時点で元の値が
//! 失われるため、`World`から逆算できない)が、**種の代わりに今のストリーム位置
//! そのもの**を`rng_state`へ書くようにした(`sim_math::SimRngState`)。これで
//! ブラウン運動・気体分子運動論のように`SolverContext::rng`を引くドメインを
//! 含むシーンでも、エクスポート→再インポート後の乱数列がビット単位で続く。
//!
//! この解消に伴い、`brownian`/`ising`の往復テストを分けていた回避策
//! (「復元直後の一致」と「復元経路自体の決定論」の2本立て)は不要になり、
//! 他の9ドメインと同じ`assert_round_trip`(復元直後 + 双方をさらに回した後の
//! `state_hash`一致)へ統合した。
//!
//! **解消済みの制限(結合の内部基準値、`sim_coupling::CouplingRawState`)**:
//! `PistonGas`(変位ゼロ点)・`SphRigid`(境界粒子の確保区間)・
//! `PhaseChangeMorph`(融解のエンタルピーと粒子生成の繰り越し)・
//! `BrownianForce`(自前RNGのストリーム位置)は、これらを非公開で持つため
//! 公開パラメータ(現在値)しか書き戻せず、「ピストンが変位済み・氷が融解途中」の
//! シーンをエクスポートすると再インポート後は**今の状態を新たな基準として
//! 再スタート**していた。`Coupling`トレイトへ`raw_state`/`restore_raw_state`を
//! 足し(既定は`None`/`Err`で、残る10種の挙動は不変)、`CouplingJson`の該当4変種へ
//! `raw_state`を通したことで**基準値ごと往復する**ようになった。
//! **結合は`state_hash`に含まれない**ため、往復の検証は「復元直後の一致」ではなく
//! **復元後に双方を回して力学・熱・気体・SPHがずれないこと**で行う。
//!
//! この作業で分かった**`PistonGas`についての正直な訂正**: 体積は
//! $V = V_{ref} + A\,(x - x_{ref})$ という**アフィン写像**なので、
//! 体積の下限クランプ(`new_volume.max(1e-9)`、底突き)に当たっていない限り
//! 「現在の体積と現在の位置」を新たな基準に取り直しても以後の体積は一致する
//! ——移行前の記述「以後の相対変位はゼロから再カウントされる」は
//! **クランプに当たった後にだけ当てはまる**誤りだった。`raw_state`が実際に
//! 効くのはその領域であり、対照テストもそこに置いてある。
//!
//! **解消済みの制限(`fdtd`のPML、`FdtdScenarioJson::pml`)**:
//! 移行前は`FdtdScenarioJson`にPMLを構成するフィールドが無く、`from_scenario`が
//! 作る`World`のFDTDは常にPEC境界のみ(`pml: None`)だった——つまり
//! 「分離場成分を書き出せない」以前に、**シーンJSONからPMLへ到達する経路が
//! 存在しなかった**(開放空間を模したいシーンは箱の壁で全反射していた)。
//! `pml: Option<PmlJson>`(`layers`/`target_reflection`)を足して`with_pml`へ
//! 配線し、`FdtdRawStateJson::pml`に分離場成分 $(E_{zx}, E_{zy})$ を持たせた
//! ことで、**PML有効なFDTDも時間発展後に`state_hash`一致で往復する**。
//! 係数表8本は`(nx, ny, h, dt, layers, target_reflection)`の決定的関数なので
//! JSONへは書かない(`PmlJson`のdoc参照)——`from_scenario`は`dt`を戻してから
//! `with_pml`を呼び、同じ係数を組み直す。
//!
//! **解消済みの制限(著者向けメタデータ、`Scenario::pass_criteria`/
//! `prediction_prompts`)**: 移行前はこの2つを`from_scenario`が読まず
//! `to_scenario`が常に空を返していた——`World`が実行時状態として持たない、
//! というのが理由だった。**しかしその結果、エディタでシーンを保存するたびに
//! 消えていた**(手で合格基準を書いたシーンを読み込み、`export_scene_json`で
//! 書き戻すと検証タブの基準が丸ごと落ちる)。`World`に置き場所を作り
//! (`World::prediction_prompts`/`pass_criteria`)、`append_scenario_bodies`が
//! 読み・`to_scenario`が書き戻すようにした。**物理からは完全に隔離**されている
//! ——`step()`も`state_hash()`もこの2つに触れない。

use crate::{BodyId, ProbeTarget, World};
use sim_math::Vec3;
use sim_mechanics::{BodyType, DragModel, Shape};
use std::collections::HashMap;

use crate::scenario::{
    AstroBodyJson, AstroScenarioJson, AtmosphereJson, AtmosphericDragJson, BodyScenarioDesc,
    BodyThermalLinkJson, BrownianForceRawStateJson, BrownianRawStateJson, BrownianScenarioJson,
    CapacitorJson, CircuitScenarioJson, CompoundChildJson, ConductionRodRawStateJson,
    ConductionRodScenarioJson, ConvectionModeJson, CouplingJson, DiodeJson, FdtdPmlRawStateJson,
    FdtdRawStateJson, FdtdScenarioJson, FluidJson, GasScenarioJson, GaussianPacket2dJson,
    GaussianPacketJson, GridBoundaryJson, GridFluid3DRawStateJson, GridFluid3DScenarioJson,
    GridFluidRawStateJson, GridFluidScenarioJson, InductorJson, IsingRawStateJson,
    IsingScenarioJson, JointJson, KineticGasRawStateJson, KineticGasScenarioJson, LiftModelJson,
    MaterialOverride, MeltSpawnJson, PhaseChangeMorphRawStateJson, PhaseChangeOverrideJson,
    PistonGasRawStateJson, PmlJson, ProbeJson, Quantum1dRawStateJson, Quantum1dScenarioJson,
    Quantum2dRawStateJson, Quantum2dScenarioJson, RelativisticCorrectionJson, ResistorJson,
    RngStateJson, Scenario, ShapeJson, SoftBendingConstraintJson, SoftBodyRawStateJson,
    SoftBodyScenarioJson, SoftConstraintJson, SoftVolumeConstraintJson, SphRawStateJson,
    SphRigidRawStateJson, SphScenarioJson, SwitchJson, ThermalLinkJson, ThermalNodeJson,
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
        // `World`は`SimRng::new`が畳み込んだ後の元シードを保持しないので、ここは
        // 常に0のままである。**代わりに`rng_state`(今のストリーム位置)を書く**
        // ので、乱数列は往復しても途切れない(`Scenario::rng_state`のdoc参照)。
        seed: 0,
        rng_state: Some(RngStateJson::from_domain(world.rng_state())),
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
        // **著者向けメタデータ**。`World`が保持するようになったのでそのまま
        // 書き戻す(モジュールdoc「解消済みの制限(著者向けメタデータ)」参照)。
        prediction_prompts: world.prediction_prompts().to_vec(),
        pass_criteria: world.pass_criteria().to_vec(),
    }
}

fn export_world_options(world: &World) -> WorldScenarioOptions {
    let mechanics = world.mechanics();
    let field = mechanics.gravity_field();
    WorldScenarioOptions {
        // `gravity`/`gravity_direction`は「一様場として見たときの値」
        // (`MechanicsSolver::gravity`のdoc参照)。非`Uniform`な場では
        // `(0.0, 下向き)`になるが、そのとき`gravity_field`が書かれていて
        // 読み込み側はそちらを優先するので情報は落ちない。
        gravity: mechanics.gravity(),
        gravity_direction: Some(vec3_to_array(mechanics.gravity_direction())),
        // **`Uniform`のときは書き出さない**——既存の`scenes/*.json`は
        // すべて一様場であり、往復出力にキーが1つ増えるのを避けるため
        // (`WorldScenarioOptions::gravity_field`のdoc参照)。
        // `Uniform`は上の2フィールドで無損失に表現できるので情報も落ちない。
        gravity_field: match field {
            sim_mechanics::GravityField::Uniform { .. } => None,
            other => Some(crate::scenario::gravity_field_to_json(other)),
        },
        dt: world.dt(),
        restitution_velocity_threshold: Some(mechanics.restitution_velocity_threshold),
        atmosphere: mechanics.atmosphere.as_ref().map(|a| AtmosphereJson {
            density: a.density,
            viscosity: a.viscosity,
        }),
    }
}

/// 生存ボディが使う材料を集め、標準DBそのものでなければ`MaterialOverride`として
/// 書き出す。戻り値は`(materials, MaterialId → 名前)`。
///
/// 書き出しは3通り: (1) 標準DBの材料そのもの → `materials`には出さず名前で参照、
/// (2) 標準DBの密度違い派生 → `extends`+`density`(従来通りの簡潔な形)、
/// (3) それ以外 → `extends`無しの全物性指定(**増分C9**)。
/// (3)は以前は表現手段が無く名前だけを書き出していたため、読み直すと
/// `SceneError::UnknownMaterial`になった。`MaterialOverride::extends`が
/// `Option`になったことでこの穴が塞がり、往復が無損失になっている。
///
/// **既知の限界**: 標準DBと同名だが物性の異なる材料((2)(3)いずれの形でも
/// `name`は元の名前のまま出る)を読み直すと、`MaterialDb::find_by_name`が
/// 先に登録された標準材料のほうを引くため、ボディが標準材料に化ける。
/// 名前の一意性はシーンJSONを書く側の責務であり、ここで勝手に改名すると
/// `bodies[].material`の参照とプローブ名の対応が壊れるため踏み込まない。
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
                overrides.push(density_derived_override(base, m));
            }
            None => {
                // 標準DBのどれの派生でもない材料。**増分C9**で`materials[].extends`が
                // 省略可能になったこと、および`MaterialDb::push`直叩きの経路により、
                // これは実際に存在しうる状態になった。`extends`無しの全物性指定として
                // 書き出す——`Material::blank()`の上へ全フィールドを載せ直す形なので、
                // 標準表のどれにも似ていない材料でも往復で値が失われない
                // (以前はここで名前だけを書き出し、次の`from_scenario`が
                // `UnknownMaterial`になっていた)。
                overrides.push(full_material_override(m));
            }
        }
        name_of.insert(mat_id, m.name.to_string());
    }
    (overrides, name_of)
}

/// 標準材料`base`の密度違い派生を、従来通りの簡潔な`extends`+`density`形で書き出す。
///
/// `source`/`uncertainty`は物理計算に効かないので`materials_equal_except_density`の
/// 比較対象外だが、基底と食い違う場合だけは書き出す——`sim-wasm`の
/// `derive_material`は派生材料に`source = "editor derived"`を立てるので、
/// これを落とすとエディタで作った材料の出所が往復で消える。
fn density_derived_override(base: &sim_core::Material, m: &sim_core::Material) -> MaterialOverride {
    MaterialOverride {
        extends: Some(base.name.to_string()),
        name: m.name.to_string(),
        density: Some(m.density),
        source: (m.source != base.source).then(|| m.source.to_string()),
        uncertainty: (m.uncertainty != base.uncertainty).then_some(m.uncertainty),
        // 残る物性は基底と一致しているので上書きしない(`Default` = 全て未指定)。
        ..MaterialOverride::default()
    }
}

/// 標準DBのどれとも(密度以外でも)一致しない材料を、`extends`無しの全物性指定で
/// 書き出す。読み直す側(`append_scenario_bodies`)の土台は`Material::blank()`だが、
/// `Option`物性も含めて全フィールドを明示するので基底の値は一切透けない。
///
/// `youngs_modulus`/`resistivity`/`refractive_index`が`None`の材料は、そのまま
/// `None`(JSONでは`null`)として書き出す——`blank()`側も同じく`None`なので、
/// 「上書きしない」がそのまま「値を持たない」として復元される。
fn full_material_override(m: &sim_core::Material) -> MaterialOverride {
    MaterialOverride {
        extends: None,
        name: m.name.to_string(),
        density: Some(m.density),
        friction: Some(m.friction),
        restitution: Some(m.restitution),
        youngs_modulus: m.youngs_modulus,
        specific_heat: Some(m.specific_heat),
        conductivity: Some(m.conductivity),
        emissivity: Some(m.emissivity),
        melting: m.melting.map(|p| PhaseChangeOverrideJson {
            melting_point: p.melting_point,
            latent_heat_fusion: p.latent_heat_fusion,
            boiling_point: p.boiling_point,
            latent_heat_vaporization: p.latent_heat_vaporization,
        }),
        resistivity: m.resistivity,
        relative_permittivity: Some(m.relative_permittivity),
        refractive_index: m.refractive_index,
        source: Some(m.source.to_string()),
        uncertainty: Some(m.uncertainty),
    }
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
                // `BodyScenarioDesc::position` は `RigidBodyDesc::transform.position`
                // と同じ意味、すなわち**形状のローカル原点**。`bodies.position[idx]`
                // は重心なので、生成→書き出し→再読み込みを恒等にするには
                // `origin_position` を通す必要がある(群11、`RigidBodySet` 型doc参照)。
                // 重心オフセットが 0 の形状では両者は一致する。
                position: vec3_to_array(bodies.origin_position(idx)),
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

/// 流体領域を**登録順のまま**書き出す(順序が重なり時の優先順位そのものなので、
/// 並べ替えてはいけない、`sim_mechanics::MechanicsSolver::fluids`のdoc参照)。
fn export_fluids(world: &World) -> Vec<FluidJson> {
    world
        .fluid_regions()
        .iter()
        .map(FluidJson::from_region)
        .collect()
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
    // PMLは**構築レシピ側**(係数表は`layers`/`target_reflection`から決定的に
    // 組み直せる、`PmlJson`のdoc参照)。時間発展する分離場成分だけが`raw_state`。
    let pml = sim
        .pml_target_reflection()
        .map(|target_reflection| PmlJson {
            layers: sim.pml_layers(),
            target_reflection,
        });
    let pml_raw = sim
        .pml_split_fields()
        .map(|(ezx, ezy)| FdtdPmlRawStateJson {
            ezx: ezx.to_vec(),
            ezy: ezy.to_vec(),
        });
    Some(FdtdScenarioJson {
        nx: sim.nx(),
        ny: sim.ny(),
        h: sim.h(),
        courant: Some(sim.dt / sim.h()),
        // レシピ側の`initial`は $E_z$ しか書けない(磁場が落ちる)。`raw_state`が真。
        initial: None,
        pml,
        raw_state: Some(FdtdRawStateJson {
            ez: sim.ez_raw().to_vec(),
            hx: sim.hx_raw().to_vec(),
            hy: sim.hy_raw().to_vec(),
            dt: sim.dt,
            pml: pml_raw,
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

/// 結合の内部基準値(`sim_coupling::CouplingRawState`)をシーンJSON表現へ写す。
///
/// 4変種それぞれに個別の関数を置くのは、`CouplingJson`が**変種ごとに別々の
/// `raw_state`型**を持つため(`FdtdRawStateJson`等のドメイン側と同じ方針——
/// 型でどの結合の状態かが決まるので、読む側が種別で分岐せずに済む)。
/// 種別が食い違えば`None`を返す(そのまま既定値経路になる、防御)。
fn coupling_piston_gas_raw_state(c: &sim_coupling::PistonGas) -> Option<PistonGasRawStateJson> {
    match sim_coupling::Coupling::raw_state(c)? {
        sim_coupling::CouplingRawState::PistonGas {
            reference_axis_position,
            reference_volume,
        } => Some(PistonGasRawStateJson {
            reference_axis_position,
            reference_volume,
        }),
        _ => None,
    }
}

fn coupling_sph_rigid_raw_state(c: &sim_coupling::SphRigid) -> Option<SphRigidRawStateJson> {
    match sim_coupling::Coupling::raw_state(c)? {
        sim_coupling::CouplingRawState::SphRigid {
            boundary_start,
            boundary_count,
        } => Some(SphRigidRawStateJson {
            boundary_start,
            boundary_count,
        }),
        _ => None,
    }
}

fn coupling_phase_change_raw_state(
    c: &sim_coupling::PhaseChangeMorph,
) -> Option<PhaseChangeMorphRawStateJson> {
    match sim_coupling::Coupling::raw_state(c)? {
        sim_coupling::CouplingRawState::PhaseChangeMorph {
            enthalpy,
            mass,
            despawned,
            pending_spawn_mass,
            spawned_particles,
            last_liquid_fraction,
        } => Some(PhaseChangeMorphRawStateJson {
            enthalpy,
            mass,
            despawned,
            pending_spawn_mass,
            spawned_particles,
            last_liquid_fraction,
        }),
        _ => None,
    }
}

fn coupling_brownian_force_raw_state(
    c: &sim_coupling::BrownianForce,
) -> Option<BrownianForceRawStateJson> {
    match sim_coupling::Coupling::raw_state(c)? {
        sim_coupling::CouplingRawState::BrownianForce { rng } => Some(BrownianForceRawStateJson {
            rng: RngStateJson::from_domain(rng),
        }),
        _ => None,
    }
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
                    let raw = coupling_phase_change_raw_state(c);
                    out.push(CouplingJson::PhaseChangeMorph {
                        body,
                        thermal_node: c.thermal_node,
                        melting_temperature: c.material.melting_temperature,
                        latent_heat_fusion: c.material.latent_heat_fusion,
                        specific_heat_solid: c.material.specific_heat_solid,
                        specific_heat_liquid: c.material.specific_heat_liquid,
                        initial_mass: c.initial_mass,
                        conductance: c.conductance,
                        // レシピ側の`initial_enthalpy`は**生成時**の値を表すフィールド
                        // なので、融解が進んだ今の値をここへ書くと意味がずれる。
                        // 復元に使われるのは`raw_state`であり、こちらは0のままでよい
                        // (`raw_state`が`None`の場合だけ効く既定値)。
                        initial_enthalpy: 0.0,
                        melt_spawn: c.spawn.map(|s| MeltSpawnJson {
                            spawn_radius: s.spawn_radius,
                            seed: s.seed,
                        }),
                        raw_state: raw,
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
                    let raw = coupling_sph_rigid_raw_state(c);
                    out.push(CouplingJson::SphRigid {
                        body,
                        // 人が読むためのメタデータ(`raw_state`が真、
                        // `SphRigidRawStateJson`のdoc参照)。
                        boundary_points: raw.as_ref().map_or(0, |r| r.boundary_count),
                        radius: c.radius,
                        raw_state: raw,
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
                        // 人が読むためのメタデータ(現在の気体体積)。変位ゼロ点は
                        // `raw_state`が持つ(`PistonGasRawStateJson`のdoc参照)。
                        initial_volume,
                        raw_state: coupling_piston_gas_raw_state(c),
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
                        // `seed`/`stream`はストリームの**先頭**を決めるだけで、
                        // 走らせた後の位置は表せない(`SimRng::new`は種を状態へ
                        // 畳み込んで捨てる)。位置は`raw_state`が持つ。
                        seed: 0,
                        stream: 0,
                        raw_state: coupling_brownian_force_raw_state(c),
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
        ProbeTarget::LedgerKinetic => ProbeJson::LedgerKinetic,
        ProbeTarget::StateHashDigest => ProbeJson::StateHashDigest,
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

    /// **流体領域の一般化**: 複数の流体領域が、形状(半空間/AABB)・水面・密度・
    /// 水温・**並び順**まで含めて往復すること。並び順は重なり時の優先順位そのもの
    /// (`sim_mechanics::MechanicsSolver::fluids`のdoc)なので、順序が入れ替わる
    /// 往復は物理を変える。
    #[test]
    fn multiple_fluid_regions_round_trip_with_their_shape_temperature_and_order() {
        let mut world = World::new(WorldOptions {
            gravity: 9.80665,
            dt: 1.0 / 120.0,
            seed: 0,
        });
        let regions = [
            // AABBを先に置く(半空間より優先されるべき順序)。
            sim_fluid::FluidRegion::aabb(
                Vec3::new(-2.0, -5.0, -2.0),
                Vec3::new(2.0, 1.0, 2.0),
                0.25,
                1025.0,
            )
            .with_temperature(277.15),
            sim_fluid::FluidRegion::aabb(
                Vec3::new(4.0, -5.0, -2.0),
                Vec3::new(8.0, 1.0, 2.0),
                0.0,
                1200.0,
            ),
            sim_fluid::FluidRegion::new(-1.0, 998.2).with_temperature(293.15),
        ];
        for region in regions {
            world.add_fluid_region(region);
        }

        let scenario = to_scenario(&world, "fluid-regions-round-trip");
        // JSONを実際に通す(`FluidJson`のserde表現ごと確かめる)。
        let json = serde_json::to_string(&scenario).expect("serializable");
        let parsed = Scenario::from_json(&json).expect("round trip must parse back");
        let reloaded = World::from_scenario(&parsed).expect("round trip must build");

        assert_eq!(
            reloaded.fluid_regions(),
            &regions[..],
            "形状・水面・密度・水温・並び順がすべて保たれること"
        );
    }

    /// 領域が1つも無い`World`は`fluids`が空配列のまま往復する(移行前の
    /// `water: None`に相当、`aabb_water`変種の追加が既定を変えていないこと)。
    #[test]
    fn a_world_without_fluid_regions_round_trips_to_an_empty_fluids_section() {
        let world = World::new(WorldOptions {
            gravity: 9.80665,
            dt: 1.0 / 120.0,
            seed: 0,
        });
        let scenario = to_scenario(&world, "no-fluids");
        assert!(scenario.fluids.is_empty());
        let reloaded = World::from_scenario(&scenario).expect("round trip must build");
        assert!(reloaded.fluid_regions().is_empty());
    }

    /// **後方互換の往復**: 実アセット(`scenes/d6-floating-box-f4.json`)を読み、
    /// 書き戻し、読み直しても水域が移行前と同一の半空間領域のままであること。
    /// エクスポータが`static_water`ではなく`aabb_water`を書き始めたら破れる。
    #[test]
    fn an_existing_static_water_scene_still_exports_as_static_water() {
        let json = include_str!("../../../scenes/d6-floating-box-f4.json");
        let scenario = Scenario::from_json(json).expect("既存シーンは妥当なJSON");
        let world = World::from_scenario(&scenario).expect("既存シーンは構築できる");
        let exported = to_scenario(&world, "d6-round-trip");
        assert!(matches!(
            exported.fluids.as_slice(),
            [FluidJson::StaticWater {
                water_level,
                density,
                temperature: None,
            }] if *water_level == 0.0 && *density == 998.2
        ));
        let reloaded = World::from_scenario(&exported).expect("round trip must build");
        assert_eq!(
            reloaded.fluid_regions(),
            &[sim_fluid::FluidRegion::new(0.0, 998.2)]
        );
    }

    /// **増分C9の要**: 標準表のどれとも密度以外で異なる材料を持つ`World`が
    /// 往復しても物性が失われないこと。以前は`export_materials`がこの材料を
    /// 名前だけで書き出し、読み直すと`SceneError::UnknownMaterial`になっていた
    /// (`export_materials`のdocが自ら挙げていた穴)。`extends`が省略可能に
    /// なったことで`extends`無し+全物性指定として書き出せるようになった。
    #[test]
    fn material_unrelated_to_standard_db_round_trips_with_all_properties() {
        let mut world = World::new(WorldOptions {
            gravity: 9.8,
            dt: 1.0 / 120.0,
            seed: 0,
        });
        // 標準表のどれとも「密度以外が一致」しない材料を直接積む。
        let exotic = sim_core::Material {
            name: "架空合金",
            density: 4321.0,
            friction: 0.123,
            restitution: 0.456,
            youngs_modulus: Some(1.5e11),
            specific_heat: 654.0,
            conductivity: 77.7,
            emissivity: 0.33,
            melting: Some(sim_core::PhaseChangeProps {
                melting_point: 1234.0,
                latent_heat_fusion: 2.5e5,
                boiling_point: 3210.0,
                latent_heat_vaporization: 6.1e6,
            }),
            resistivity: Some(3.3e-7),
            relative_permittivity: 4.25,
            refractive_index: Some(2.75),
            source: "架空(テスト専用)",
            uncertainty: 0.42,
        };
        let mat = world.materials_mut().push(exotic.clone());
        let mut desc = RigidBodyDesc::dynamic(sim_mechanics::Shape::Sphere { radius: 0.3 }, mat);
        desc.transform.position = Vec3::new(0.0, 5.0, 0.0);
        world.create_body(desc);

        let scenario = to_scenario(&world, "exotic-material");
        // 名前だけの書き逃げではなく、`extends`無しの全物性指定が出ていること。
        assert_eq!(scenario.materials.len(), 1);
        assert_eq!(scenario.materials[0].extends, None);
        assert_eq!(scenario.materials[0].name, "架空合金");

        // JSONを一度経由しても(serdeの`Option`の往復も含めて)復元できること。
        let json = serde_json::to_string(&scenario).expect("Scenario must serialize");
        let parsed: Scenario = serde_json::from_str(&json).expect("Scenario must deserialize");
        let reloaded = World::from_scenario(&parsed).expect("round trip must parse back");

        let id = reloaded
            .materials()
            .find_by_name("架空合金")
            .expect("exotic material must be reconstructible from the scene JSON");
        let back = reloaded.materials().get(id);
        assert!(
            materials_equal(&exotic, back),
            "reloaded material must match the original exactly, got {back:?}"
        );
        // メタ(物理計算には効かないが往復の忠実性のために持つ)も一致すること。
        assert_eq!(back.source, exotic.source);
        assert_eq!(back.uncertainty, exotic.uncertainty);
    }

    /// 標準表の密度違い派生は従来通り簡潔な`extends`+`density`で書き出す
    /// (増分C9で全物性を書けるようになっても、表現できるからといって常に
    /// 全部書くわけではない——差分が密度だけならその形が最も読みやすい)。
    #[test]
    fn density_only_derivative_still_exports_as_extends_plus_density() {
        let mut world = World::new(WorldOptions {
            gravity: 9.8,
            dt: 1.0 / 120.0,
            seed: 0,
        });
        let pine = world.materials().find_by_name("木材(松)").unwrap();
        let mut derived = world.materials().get(pine).clone();
        derived.name = "light-wood";
        derived.density = 400.0;
        let mat = world.materials_mut().push(derived);
        world.create_body(RigidBodyDesc::dynamic(
            sim_mechanics::Shape::Sphere { radius: 0.3 },
            mat,
        ));

        let scenario = to_scenario(&world, "derived-material");
        assert_eq!(scenario.materials.len(), 1);
        let over = &scenario.materials[0];
        assert_eq!(over.extends.as_deref(), Some("木材(松)"));
        assert_eq!(over.name, "light-wood");
        assert_eq!(over.density, Some(400.0));
        // 密度以外は基底と一致するので、上書きは書かれない。
        assert_eq!(over.friction, None);
        assert_eq!(over.conductivity, None);
        assert_eq!(over.source, None);
        assert_eq!(over.uncertainty, None);

        let reloaded = World::from_scenario(&scenario).expect("round trip must parse back");
        let id = reloaded.materials().find_by_name("light-wood").unwrap();
        assert_eq!(reloaded.materials().get(id).density, 400.0);
        assert_eq!(
            reloaded.materials().get(id).conductivity,
            world.materials().get(pine).conductivity
        );
    }

    /// `sim-wasm`の`derive_material`が立てる`source = "editor derived"`のように、
    /// 物理物性は基底と密度しか違わないがメタだけ食い違う材料でも、出所が
    /// 往復で消えないこと(`density_derived_override`のdoc参照)。
    #[test]
    fn density_derivative_with_differing_source_keeps_it_through_the_round_trip() {
        let mut world = World::new(WorldOptions {
            gravity: 9.8,
            dt: 1.0 / 120.0,
            seed: 0,
        });
        let pine = world.materials().find_by_name("木材(松)").unwrap();
        let mut derived = world.materials().get(pine).clone();
        derived.name = "editor-wood";
        derived.density = 333.0;
        derived.source = "editor derived";
        let mat = world.materials_mut().push(derived);
        world.create_body(RigidBodyDesc::dynamic(
            sim_mechanics::Shape::Sphere { radius: 0.3 },
            mat,
        ));

        let scenario = to_scenario(&world, "editor-derived-material");
        assert_eq!(
            scenario.materials[0].source.as_deref(),
            Some("editor derived")
        );

        let reloaded = World::from_scenario(&scenario).expect("round trip must parse back");
        let id = reloaded.materials().find_by_name("editor-wood").unwrap();
        assert_eq!(reloaded.materials().get(id).source, "editor derived");
        assert_eq!(reloaded.materials().get(id).density, 333.0);
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

    // -----------------------------------------------------------------------
    // **結合の内部基準値(`sim_coupling::CouplingRawState`)の往復テスト**(4種)。
    //
    // ドメインの`raw_state`と同じく「作った直後」ではなく**回してから**
    // エクスポートする。**結合は`state_hash`に含まれない**ので、往復の成否は
    // 復元直後のハッシュでは判定できない——基準値がずれていることは
    // 「復元後に双方を回すと力学・熱・気体・SPHがずれる」形でしか現れない。
    // `assert_round_trip`の後半(extra_steps)がそこを見ている。
    // -----------------------------------------------------------------------

    fn zero_gravity_world() -> World {
        World::new(WorldOptions {
            gravity: 0.0,
            dt: 1.0 / 120.0,
            seed: 0,
        })
    }

    /// 一定速度で気体を圧縮する`Kinematic`ピストン(`sim_coupling::piston_gas`の
    /// 単体テストと同じ構成)。**`Kinematic`にするのは`force_accum`を避けるため**
    /// ——`PistonGas`は post 相で`force_accum`へ反力を積み、それを消費するのは
    /// **次stepの積分**なので、動的ピストンだと「シーンJSONが`force_accum`を
    /// 書けない」という別の(既知の、本増分の対象外の)欠落が混ざる。
    /// `Kinematic`は力を受けないので、ここでは変位ゼロ点の往復だけを見られる。
    fn piston_gas_world() -> (World, BodyId) {
        let mut world = zero_gravity_world();
        let mat = steel(&world);
        let mut desc = RigidBodyDesc::dynamic(sim_mechanics::Shape::Sphere { radius: 0.05 }, mat);
        desc.body_type = sim_mechanics::BodyType::Kinematic;
        desc.linear_velocity = Vec3::new(-0.05, 0.0, 0.0); // 気体を圧縮する向き
        let piston = world.create_body(desc);
        world.enable_gas(sim_thermal::GasCompartment {
            n_moles: 1.0,
            volume: 1.0e-3,
            temperature: 300.0,
            gas: sim_thermal::GasSpecies::AIR,
        });
        let coupling = sim_coupling::PistonGas::new(
            &world.mechanics().bodies,
            piston.index as usize,
            Vec3::new(1.0, 0.0, 0.0),
            0.01,
            1.0e-3,
        );
        world.add_coupling(Box::new(coupling));
        (world, piston)
    }

    /// **底突き(体積の下限クランプ)まで圧縮してから反転させた**ワールドを作る。
    ///
    /// **なぜこの経路でないと対照にならないか(正直な記録)**: `PistonGas`の
    /// 体積は $V = V_{ref} + A\,(x - x_{ref})$ という**アフィン写像**なので、
    /// クランプに当たっていない限り「現在の体積と現在の位置」を新たな基準に
    /// 取り直しても以後の体積は一致する——つまり変位ゼロ点は現在値から
    /// 復元できてしまい、`raw_state`の有無で差が出ない。
    /// 差が出るのは`new_volume.max(1e-9)`のクランプに当たった後だけである
    /// (クランプがアフィン性を壊し、現在値からは基準を逆算できなくなる)。
    fn bottomed_out_piston_gas_world() -> World {
        let (mut world, piston) = piston_gas_world();
        // 変位 -0.125 m ⇒ V = 1e-3 + 0.01*(-0.125) < 0 ⇒ 下限クランプに当たる。
        for _ in 0..300 {
            world.step();
        }
        assert!(
            world.gas().unwrap().volume <= 1e-9,
            "この対照は体積の下限クランプに当たっている必要がある"
        );
        // 反転させて引き戻す(クランプ域から出ると基準の違いが体積に現れる)。
        world.mechanics_mut().bodies.linear_velocity[piston.index as usize] =
            Vec3::new(0.05, 0.0, 0.0);
        world
    }

    /// `PistonGas`(変位ゼロ点)。底突き後に反転させた状態でエクスポートする
    /// (`bottomed_out_piston_gas_world`のdoc参照)。
    ///
    /// **`GasCompartment`は`state_hash`に含まれない**(自律的な時間発展を持たない
    /// 従属量、`World::gas`のdoc参照)ので、判定はハッシュではなく気体の状態量
    /// そのもの(体積・温度)をビット単位で突き合わせる形で行う。
    #[test]
    fn piston_gas_coupling_raw_state_round_trip_after_stepping() {
        let mut world = bottomed_out_piston_gas_world();
        let scenario = to_scenario(&world, "piston_gas");
        let mut reloaded = World::from_scenario(&scenario).expect("round trip must parse back");
        for _ in 0..100 {
            world.step();
            reloaded.step();
        }
        let (a, b) = (world.gas().unwrap(), reloaded.gas().unwrap());
        assert_eq!(
            a.volume.to_bits(),
            b.volume.to_bits(),
            "変位ゼロ点が戻っていれば体積はビット単位で一致する ({} vs {})",
            a.volume,
            b.volume
        );
        assert_eq!(a.temperature.to_bits(), b.temperature.to_bits());
    }

    /// `PistonGas`の`raw_state`を落とすと(=`initial_volume`(現在値)を新たな基準に
    /// する移行前の挙動)以後の気体体積がずれることを、対照として固定する。
    #[test]
    fn piston_gas_round_trip_breaks_without_its_reference_point() {
        let mut world = bottomed_out_piston_gas_world();
        let mut scenario = to_scenario(&world, "piston_gas-no-ref");
        for coupling in &mut scenario.couplings {
            if let CouplingJson::PistonGas { raw_state, .. } = coupling {
                *raw_state = None;
            }
        }
        let mut reloaded = World::from_scenario(&scenario).expect("must still parse");
        for _ in 0..100 {
            world.step();
            reloaded.step();
        }
        assert_ne!(
            world.gas().unwrap().volume,
            reloaded.gas().unwrap().volume,
            "変位ゼロ点を落とせば「今の位置が変位0」になって体積がずれるはず\
             (落としていないから一致している、という取り違えを防ぐための対照)"
        );
    }

    /// `SphRigid`(境界粒子の確保区間)。`sph`側の`raw_state`が境界粒子ごと
    /// 復元するので、結合側は**確保し直さず区間だけを戻す**(二重確保の防止)。
    #[test]
    fn sph_rigid_coupling_raw_state_round_trip_after_stepping() {
        let mut world = empty_world();
        let mat = steel(&world);
        let mut desc = RigidBodyDesc::dynamic(sim_mechanics::Shape::Sphere { radius: 0.05 }, mat);
        desc.transform.position = Vec3::new(0.1, 0.3, 0.1);
        let ball = world.create_body(desc);

        let mut fluid = sim_fluid::SphFluid::new(0.04, 1000.0, 20.0);
        fluid.mass = 0.05;
        for ix in 0..4 {
            for iy in 0..4 {
                for iz in 0..4 {
                    fluid.position.push(Vec3::new(
                        ix as f64 * 0.03,
                        iy as f64 * 0.03,
                        iz as f64 * 0.03,
                    ));
                    fluid.velocity.push(Vec3::ZERO);
                    fluid.density.push(1000.0);
                    fluid.pressure.push(0.0);
                }
            }
        }
        world.enable_sph(fluid);

        let coupling = {
            let sph = world.sph_mut().expect("sph domain");
            sim_coupling::SphRigid::new(sph, ball.index as usize, 0.05, 12)
        };
        world.add_coupling(Box::new(coupling));
        assert_round_trip(world, 20, 15, "sph_rigid");
    }

    /// `SphRigid`の`raw_state`が**境界粒子を再確保しない**こと(二重確保の対照)。
    #[test]
    fn sph_rigid_round_trip_does_not_duplicate_boundary_particles() {
        let mut world = empty_world();
        let mat = steel(&world);
        let ball = world.create_body(RigidBodyDesc::dynamic(
            sim_mechanics::Shape::Sphere { radius: 0.05 },
            mat,
        ));
        world.enable_sph(sim_fluid::SphFluid::new(0.04, 1000.0, 20.0));
        let coupling = {
            let sph = world.sph_mut().expect("sph domain");
            sim_coupling::SphRigid::new(sph, ball.index as usize, 0.05, 12)
        };
        world.add_coupling(Box::new(coupling));
        let before = world.sph().unwrap().boundary_position.len();
        assert_eq!(before, 12);

        let scenario = to_scenario(&world, "sph-rigid-dup");
        let reloaded = World::from_scenario(&scenario).expect("round trip must parse back");
        assert_eq!(
            reloaded.sph().unwrap().boundary_position.len(),
            before,
            "境界粒子は`sph.raw_state`が復元済み——結合側で確保し直すと二重になる"
        );
    }

    /// `PhaseChangeMorph`(融解の内部状態)。融解が進んだ途中でエクスポートする——
    /// `initial_enthalpy`は**生成時**の値なので、これだけでは融解が最初から
    /// やり直しになる。
    #[test]
    fn phase_change_morph_coupling_raw_state_round_trip_after_stepping() {
        let mut world = zero_gravity_world();
        let mat = steel(&world);
        let ball = world.create_body(RigidBodyDesc::dynamic(
            sim_mechanics::Shape::Sphere { radius: 0.05 },
            mat,
        ));

        let mut thermal = sim_thermal::ThermalSolver::new(293.15);
        thermal.add_node(sim_thermal::ThermalNode::new(293.15, 4000.0));
        world.enable_thermal(thermal);

        world.add_coupling(Box::new(sim_coupling::PhaseChangeMorph::new(
            ball.index as usize,
            0,
            sim_thermal::PhaseMaterial {
                melting_temperature: 273.15,
                latent_heat_fusion: 334_000.0,
                specific_heat_solid: 2100.0,
                specific_heat_liquid: 4186.0,
            },
            0.5,
            50.0,
            // 融点ちょうど(=融解が即座に始まる)。30step後は融解途中になる。
            0.0,
        )));
        assert_round_trip(world, 30, 25, "phase_change_morph");
    }

    /// `BrownianForce`を1個持つワールド。**慣性時間 $\tau=m/\gamma$ が既定dtより
    /// 十分長い**大きさを選ぶ(半径1cmの鋼球、水の粘性)——`sim-coupling`側の
    /// 単体テストのようなミクロン球は $\tau\approx 2\times10^{-7}$ s で、
    /// dt=1/120 だと明示的Euler-Maruyamaが発散して往復の判定にならない。
    /// 注入されるゆらぎは極小(σ≈3e-13 m/s)だが、f64のビットとしては確実に効く。
    fn brownian_force_world() -> World {
        let mut world = empty_world();
        let mat = steel(&world);
        let radius = 0.01;
        let bead = world.create_body(RigidBodyDesc::dynamic(
            sim_mechanics::Shape::Sphere { radius },
            mat,
        ));

        let mut thermal = sim_thermal::ThermalSolver::new(293.15);
        thermal.add_node(sim_thermal::ThermalNode::new(293.15, 1000.0));
        world.enable_thermal(thermal);

        world.add_coupling(Box::new(sim_coupling::BrownianForce::new(
            bead.index as usize,
            radius,
            1.002e-3,
            0,
            42,
            99,
        )));
        world
    }

    /// `BrownianForce`(自前RNGのストリーム位置)。この結合は`World`中央の`rng`
    /// とは独立した系列を持つので、`Scenario::rng_state`では戻らない。
    #[test]
    fn brownian_force_coupling_raw_state_round_trip_after_stepping() {
        assert_round_trip(brownian_force_world(), 30, 25, "brownian_force");
    }

    /// `BrownianForce`の`raw_state`を落とすと(=`seed`/`stream`から作り直す
    /// 移行前の挙動)以後の軌跡がずれることを、対照として固定する。
    #[test]
    fn brownian_force_round_trip_breaks_without_its_rng_stream_position() {
        let mut world = brownian_force_world();
        for _ in 0..30 {
            world.step();
        }

        let mut scenario = to_scenario(&world, "brownian-force-no-rng");
        for coupling in &mut scenario.couplings {
            if let CouplingJson::BrownianForce { raw_state, .. } = coupling {
                *raw_state = None;
            }
        }
        let mut reloaded = World::from_scenario(&scenario).expect("must still parse");
        let mut world = world;
        world.step();
        reloaded.step();
        assert_ne!(
            world.state_hash(),
            reloaded.state_hash(),
            "自前RNGの位置を落とせば次のstepでずれるはず(落としていないから一致している、\
             という取り違えを防ぐための対照)"
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
        assert_round_trip(world, 25, 20, "brownian");
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
        assert_round_trip(world, 25, 20, "ising");
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

    // -----------------------------------------------------------------------
    // **著者向けメタデータ**(`pass_criteria`/`prediction_prompts`)の往復。
    // -----------------------------------------------------------------------

    /// 合格基準と予測ヒントを持つシーンJSON(2プローブ)。
    fn authored_scene_json() -> &'static str {
        r#"{
            "name": "authored",
            "world": { "gravity": 9.80665, "dt": 0.008333333 },
            "bodies": [
                { "name": "ball", "shape": { "sphere": { "radius": 0.2 } },
                  "material": "鋼(炭素鋼)", "position": [0, 5, 0] }
            ],
            "probes": [ { "body_pos_y": "ball" }, { "body_speed": "ball" } ],
            "prediction_prompts": [
                { "question": "落下1秒後の高さは?", "probe_index": 0,
                  "expected_value": 0.0967 }
            ],
            "pass_criteria": [
                { "probe_index": 0, "operator": "le", "threshold": 0.25 },
                { "probe_index": 1, "operator": "ge", "threshold": 9.0 }
            ]
        }"#
    }

    /// **著者向けメタデータが往復で消えないこと**。移行前は`from_scenario`が
    /// 読まず`to_scenario`が常に空を返していたため、エディタでシーンを保存する
    /// たびに丸ごと落ちていた。
    ///
    /// **`.step()`した後でも同じ値が出ること**まで見るのが要点——物理から
    /// 完全に隔離されている(実行で変化しない)ことの確認である。
    #[test]
    fn author_metadata_survives_the_round_trip_and_is_inert_to_stepping() {
        let scenario = Scenario::from_json(authored_scene_json()).expect("valid scene");
        let mut world = World::from_scenario(&scenario).expect("must build");

        // 読み込み時点で`World`が持っていること。
        assert_eq!(world.prediction_prompts().len(), 1);
        assert_eq!(world.pass_criteria().len(), 2);

        let hash_before = world.state_hash();
        for _ in 0..120 {
            world.step();
        }
        // 物理を進めてもメタデータは変わらない(かつ、メタデータは物理に効かない
        // ——`state_hash`はメタデータ抜きの`World`と同じ経路で計算される)。
        assert_ne!(
            hash_before,
            world.state_hash(),
            "実際に時間発展していること"
        );

        let exported = to_scenario(&world, "authored");
        assert_eq!(exported.prediction_prompts.len(), 1);
        assert_eq!(
            exported.prediction_prompts[0].question,
            "落下1秒後の高さは?"
        );
        assert_eq!(exported.prediction_prompts[0].probe_index, 0);
        assert_eq!(exported.prediction_prompts[0].expected_value, 0.0967);
        assert_eq!(exported.pass_criteria.len(), 2);
        assert_eq!(exported.pass_criteria[0].probe_index, 0);
        assert!(matches!(
            exported.pass_criteria[0].operator,
            crate::scenario::PassCriterionOperator::Le
        ));
        assert_eq!(exported.pass_criteria[0].threshold, 0.25);
        assert_eq!(exported.pass_criteria[1].probe_index, 1);
        assert!(matches!(
            exported.pass_criteria[1].operator,
            crate::scenario::PassCriterionOperator::Ge
        ));
        assert_eq!(exported.pass_criteria[1].threshold, 9.0);

        // 2周目(JSON文字列を経由)でも同じ。
        let json = serde_json::to_string(&exported).expect("must serialize");
        let parsed: Scenario = serde_json::from_str(&json).expect("must deserialize");
        let again = World::from_scenario(&parsed).expect("must build");
        assert_eq!(again.prediction_prompts().len(), 1);
        assert_eq!(again.pass_criteria().len(), 2);
    }

    /// 著者向けメタデータは**`state_hash`に混ざらない**(決定論replayに影響
    /// しない)ことの対照。同じ物理・メタデータ有無だけが違う2つを比べる。
    #[test]
    fn author_metadata_does_not_leak_into_the_state_hash() {
        let with_meta =
            World::from_scenario(&Scenario::from_json(authored_scene_json()).unwrap()).unwrap();
        let stripped_json = authored_scene_json()
            .replace(r#""prediction_prompts": ["#, r#""unused_prompts": ["#)
            .replace(r#""pass_criteria": ["#, r#""unused_criteria": ["#);
        let without_meta =
            World::from_scenario(&Scenario::from_json(&stripped_json).unwrap()).unwrap();
        assert!(without_meta.pass_criteria().is_empty());
        assert_eq!(with_meta.state_hash(), without_meta.state_hash());
    }

    /// `append_scenario_bodies`(シーンJSON Import)経由でも取り込まれ、
    /// **既存のメタデータを消さずに末尾へ積む**こと。`probe_index`は
    /// 登録済みプローブ本数ぶんずれる(`World::append_author_metadata`のdoc参照)。
    #[test]
    fn importing_a_scene_appends_author_metadata_with_rebased_probe_indices() {
        let scenario = Scenario::from_json(authored_scene_json()).expect("valid scene");
        let mut world = World::from_scenario(&scenario).expect("must build");
        assert_eq!(world.probe_count(), 2);

        // 同じシーンをもう一度Importする(bodies + probes が末尾へ増える)。
        let ids = world
            .append_scenario_bodies(&scenario)
            .expect("import must succeed");
        let names: std::collections::HashMap<String, BodyId> = scenario
            .bodies
            .iter()
            .zip(&ids)
            .filter_map(|(b, id)| b.name.clone().map(|n| (n, *id)))
            .collect();
        world
            .add_scenario_probes(&scenario, &names)
            .expect("probes must resolve");

        assert_eq!(world.probe_count(), 4);
        assert_eq!(world.pass_criteria().len(), 4);
        // 1件目は元のまま、3件目(=2回目のImport分)はオフセット2ぶんずれる。
        assert_eq!(world.pass_criteria()[0].probe_index, 0);
        assert_eq!(world.pass_criteria()[1].probe_index, 1);
        assert_eq!(world.pass_criteria()[2].probe_index, 2);
        assert_eq!(world.pass_criteria()[3].probe_index, 3);
        assert_eq!(world.prediction_prompts().len(), 2);
        assert_eq!(world.prediction_prompts()[1].probe_index, 2);
    }

    /// **PMLを有効にしたFDTD**(`FdtdScenarioJson::pml`)。点源から外へ広がる波を
    /// 層で吸わせながら時間発展させてから往復させる。
    ///
    /// 移行前はシーンJSONにPMLを構成する口が無く、`from_scenario`が作るFDTDは
    /// 常にPEC境界のみだったため、この経路自体が存在しなかった。
    fn pml_world() -> World {
        let mut world = empty_world();
        let mut sim = sim_em::FdtdSim2D::new(32, 32, 0.02, 0.5).with_pml(6, 1.0e-6);
        // 中央の点源(層まで届いて実際に吸収が効く配置)。
        sim.set_ez(16, 16, 2.0);
        sim.set_ez(15, 16, 1.0);
        sim.set_ez(16, 15, 1.0);
        world.enable_fdtd(sim);
        world
    }

    #[test]
    fn fdtd_pml_raw_state_round_trip_after_stepping() {
        let world = pml_world();
        // 波面がPML層へ入るまで回す(層内では`ezx`/`ezy`が別々の減衰を受けるので、
        // 分離成分の分配がここで初めて「半分ずつ」から離れる)。
        assert_round_trip(world, 40, 30, "fdtd_pml");
    }

    /// PML設定(`fdtd.pml`)がシーンJSONを通ること + 係数表がレシピから
    /// 決定的に組み直せていること(`PmlJson`のdocの主張の裏取り)。
    #[test]
    fn fdtd_pml_config_survives_the_json_round_trip() {
        let mut world = pml_world();
        for _ in 0..40 {
            world.step();
        }
        let scenario = to_scenario(&world, "fdtd_pml-json");
        let pml =
            scenario.fdtd.as_ref().unwrap().pml.as_ref().expect(
                "PML有効なFDTDは`fdtd.pml`を書き出すこと(移行前はこのフィールドが無かった)",
            );
        assert_eq!(pml.layers, 6);
        assert_eq!(pml.target_reflection, 1.0e-6);

        // JSON文字列を経由しても分離場成分が壊れないこと。
        let json = serde_json::to_string(&scenario).expect("Scenario must serialize");
        let parsed: Scenario = serde_json::from_str(&json).expect("Scenario must deserialize");
        let mut reloaded = World::from_scenario(&parsed).expect("round trip must parse back");
        assert_eq!(reloaded.fdtd().unwrap().pml_layers(), 6);
        assert_eq!(world.state_hash(), reloaded.state_hash());
        for _ in 0..30 {
            world.step();
            reloaded.step();
        }
        assert_eq!(world.state_hash(), reloaded.state_hash());
    }

    /// PMLの`raw_state`から分離場成分を落とすと往復が壊れることを、対照として
    /// 固定する(「$E_z$の半分ずつでは足りない」という`FdtdPmlRawStateJson`の
    /// docの主張の裏取り)。
    #[test]
    fn fdtd_pml_round_trip_breaks_without_the_split_fields() {
        let mut world = pml_world();
        for _ in 0..40 {
            world.step();
        }
        let mut scenario = to_scenario(&world, "fdtd_pml-no-split");
        scenario
            .fdtd
            .as_mut()
            .unwrap()
            .raw_state
            .as_mut()
            .unwrap()
            .pml = None;
        let mut reloaded = World::from_scenario(&scenario).expect("must still parse");
        world.step();
        reloaded.step();
        assert_ne!(
            world.state_hash(),
            reloaded.state_hash(),
            "分離場成分を落とせば次のstepでずれるはず(落としていないから一致している、\
             という取り違えを防ぐための対照)"
        );
    }

    /// PMLの構成値が不正なシーンJSONは`assert!`でプロセスを落とさず
    /// `SceneError::InvalidValue`になること(`apply_fdtd_pml`のdoc参照)。
    #[test]
    fn invalid_fdtd_pml_config_is_a_scene_error_not_a_panic() {
        let base = r#"{
            "name": "pml", "world": { "gravity": 0.0, "dt": 0.008333333 },
            "fdtd": { "nx": 16, "ny": 16, "h": 0.02, "pml": PML }
        }"#;
        let cases = [
            r#"{ "layers": 0, "target_reflection": 1e-6 }"#,
            r#"{ "layers": 7, "target_reflection": 1e-6 }"#,
            r#"{ "layers": 4, "target_reflection": 0.0 }"#,
            r#"{ "layers": 4, "target_reflection": 1.0 }"#,
        ];
        for case in cases {
            let json = base.replace("PML", case);
            let scenario = Scenario::from_json(&json).expect("スキーマとしては妥当");
            assert!(
                matches!(
                    World::from_scenario(&scenario),
                    Err(crate::scenario::SceneError::InvalidValue(_))
                ),
                "{case} は InvalidValue になるべき"
            );
        }
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

    /// **中央PRNGのストリーム位置が往復すること**(`Scenario::rng_state`)。
    /// `state_hash`はPRNGを含まないので、往復の成否はハッシュではなく
    /// **復元後に引いた乱数列そのもの**で確かめる必要がある。
    #[test]
    fn rng_stream_position_round_trips() {
        let mut world = empty_world();
        // 乱数を実際に消費させる(`SolverContext::rng`を引くドメインを載せる)。
        let mut set = sim_statistical::BrownianParticleSet::new(1e-15, 1e-8, 4.11e-21);
        for i in 0..8 {
            set.add_particle(Vec3::new(i as f64 * 1e-7, 0.0, 0.0), Vec3::ZERO);
        }
        world.enable_brownian(set);
        for _ in 0..17 {
            world.step();
        }

        let scenario = to_scenario(&world, "rng");
        let reloaded = World::from_scenario(&scenario).expect("round trip must parse back");
        assert_eq!(
            world.rng_state(),
            reloaded.rng_state(),
            "ストリーム位置(state/inc/normal_carry)がそのまま戻ること"
        );

        // 「次に出る値」まで一致していることを、実際に引いて確かめる。
        let mut a = sim_math::SimRng::from_raw_state(world.rng_state());
        let mut b = sim_math::SimRng::from_raw_state(reloaded.rng_state());
        for _ in 0..32 {
            assert_eq!(a.next_u32(), b.next_u32());
        }
    }

    /// `rng_state`を落とすと(=移行前の「`seed`から作り直す」挙動)以後の乱数列が
    /// 変わることを、対照として固定する。
    #[test]
    fn dropping_rng_state_restarts_the_stream_from_the_seed() {
        let mut world = empty_world();
        let mut set = sim_statistical::BrownianParticleSet::new(1e-15, 1e-8, 4.11e-21);
        set.add_particle(Vec3::ZERO, Vec3::ZERO);
        world.enable_brownian(set);
        for _ in 0..17 {
            world.step();
        }

        let mut scenario = to_scenario(&world, "rng-dropped");
        scenario.rng_state = None;
        let reloaded = World::from_scenario(&scenario).expect("must still parse");
        assert_ne!(
            world.rng_state(),
            reloaded.rng_state(),
            "`rng_state`が唯一の復元経路であること(落としても偶然一致していた、\
             という取り違えを防ぐための対照)"
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

    /// `ProbeTarget::LedgerKinetic`/`StateHashDigest`は設計の例示に含まれていた
    /// にもかかわらず、対応する`ProbeJson`が無く`to_scenario`から無言で脱落
    /// していた(意図的な除外ではなく後続増分の積み残し、`ProbeTarget`のdoc
    /// 「設計の例示」参照)。`add_probe`→`to_scenario`→`from_scenario`で
    /// 両方とも欠落しないことを確認する。
    #[test]
    fn ledger_kinetic_and_state_hash_digest_probes_round_trip() {
        let mut world = World::new(WorldOptions {
            gravity: 9.8,
            dt: 1.0 / 120.0,
            seed: 0,
        });
        let mat = steel(&world);
        let mut desc = RigidBodyDesc::dynamic(sim_mechanics::Shape::Sphere { radius: 0.3 }, mat);
        desc.transform.position = Vec3::new(0.0, 5.0, 0.0);
        world.create_body(desc);
        world.add_probe(ProbeTarget::LedgerKinetic);
        world.add_probe(ProbeTarget::StateHashDigest);

        for _ in 0..10 {
            world.step();
        }

        let scenario = to_scenario(&world, "probe-round-trip");
        assert!(matches!(scenario.probes[0], ProbeJson::LedgerKinetic));
        assert!(matches!(scenario.probes[1], ProbeJson::StateHashDigest));

        let reloaded = World::from_scenario(&scenario).expect("round trip must parse back");
        assert_eq!(world.state_hash(), reloaded.state_hash());
        assert_eq!(
            reloaded.probe(0).unwrap().target,
            ProbeTarget::LedgerKinetic
        );
        assert_eq!(
            reloaded.probe(1).unwrap().target,
            ProbeTarget::StateHashDigest
        );
    }

    // -----------------------------------------------------------------------
    // 重力場(`GravityField`)の往復。**重力場の抽象化増分**。
    // -----------------------------------------------------------------------

    /// 重力場だけを差し替えた、自由体1個のワールド。点源場でも軌道になるよう
    /// 中心から離れた位置・接線方向の初速を与える(場が本当に復元されているかは
    /// 「その後の軌跡が一致するか」でしか判定できないため、静止した体では
    /// テストにならない)。
    fn world_with_gravity_field(field: sim_mechanics::GravityField) -> World {
        let mut world = empty_world();
        world.mechanics_mut().set_gravity_field(field);
        let mat = steel(&world);
        let mut desc = RigidBodyDesc::dynamic(sim_mechanics::Shape::Sphere { radius: 0.5 }, mat);
        desc.transform.position = Vec3::new(30.0, 40.0, 0.0);
        desc.linear_velocity = Vec3::new(1.0, 0.0, 2.0);
        world.create_body(desc);
        world
    }

    /// **重力場の抽象化増分**: `GravityField`の3種すべてが
    /// `to_scenario`→`from_scenario`で往復し、復元直後だけでなく
    /// **双方をさらに時間発展させたあとも**`state_hash`が一致すること。
    ///
    /// 復元直後の一致だけでは足りない——重力場は初期状態ではなく*その後の
    /// 加速度*を決めるので、場の復元が壊れていても $t=0$ のハッシュは一致して
    /// しまう。`assert_round_trip`が後半で回す`extra_steps`がここでの本体である。
    #[test]
    fn gravity_field_round_trips_for_every_kind_including_after_stepping() {
        let cases = [
            (
                "uniform",
                sim_mechanics::GravityField::Uniform {
                    magnitude: 3.71, // 火星
                    direction: Vec3::new(1.0, -2.0, 0.5),
                },
            ),
            (
                "point-source",
                sim_mechanics::GravityField::PointSource {
                    center: Vec3::new(0.0, -10.0, 0.0),
                    mu: 5.0e5,
                },
            ),
            ("zero", sim_mechanics::GravityField::Zero),
        ];
        for (label, field) in cases {
            assert_round_trip(world_with_gravity_field(field), 20, 40, label);
            // 場そのものが同じvariantで戻っていること(ハッシュ一致だけだと、
            // たまたま軌跡が同じ別の場に化けていても気づけない)。
            let world = world_with_gravity_field(field);
            let reloaded = World::from_scenario(&to_scenario(&world, label)).unwrap();
            assert_eq!(
                reloaded.mechanics().gravity_field(),
                world.mechanics().gravity_field(),
                "{label}: the field itself must survive the round trip"
            );
        }
    }

    /// **重力場の抽象化増分**: 書き出し側の約束
    /// (`WorldScenarioOptions::gravity_field`のdoc)——`Uniform`のときは
    /// `gravity_field`キーを**出さない**(既存の`scenes/*.json`の往復出力に
    /// キーを増やさない)、非`Uniform`のときだけ出す。
    #[test]
    fn gravity_field_key_is_written_only_for_non_uniform_fields() {
        let uniform = to_scenario(
            &world_with_gravity_field(sim_mechanics::GravityField::Uniform {
                magnitude: 9.80665,
                direction: Vec3::new(0.0, -1.0, 0.0),
            }),
            "uniform",
        );
        assert!(uniform.world.gravity_field.is_none());
        assert_eq!(uniform.world.gravity, 9.80665);

        let zero = to_scenario(
            &world_with_gravity_field(sim_mechanics::GravityField::Zero),
            "zero",
        );
        assert!(matches!(
            zero.world.gravity_field,
            Some(crate::scenario::GravityFieldJson::Zero)
        ));
        // 非`Uniform`では`gravity`は「一様場として見た値」=0.0になるが、
        // 読み込み側は`gravity_field`を優先するので情報は落ちない。
        assert_eq!(zero.world.gravity, 0.0);

        let point = to_scenario(
            &world_with_gravity_field(sim_mechanics::GravityField::PointSource {
                center: Vec3::new(1.0, 2.0, 3.0),
                mu: 7.0e5,
            }),
            "point",
        );
        assert!(matches!(
            point.world.gravity_field,
            Some(crate::scenario::GravityFieldJson::PointSource { .. })
        ));
    }
}
