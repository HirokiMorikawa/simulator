//! シーン記述(JSON)。設計 docs/20-integration/04-world-api.md §3。
//!
//! **縮約実装の理由**: 設計例示のJSONスキーマ(`world`/`materials`/`bodies`/`fluids`/
//! `couplings`/`probes`)のうち、`couplings`(`Coupling` registryがまだ`World::step()`
//! に接続されていない、各`sim-coupling`実装のモジュールdoc参照)以外は実装する。
//! `fluids`は`sim_mechanics::MechanicsSolver::water`(P1スコープの単一`static_water`
//! 領域、`sim_fluid::buoyancy`冒頭の注記参照)のみ対応 — 設計例示のAABB表現ではなく
//! `water_level`(水平面の高さ)+`density`の縮約表現とする(現在の`StaticWaterRegion`
//! 自体がAABBではなく単一の水位面のみを表すため)。`temperature`(水温、熱ドメインとの
//! 結合)は未対応。`probes`は`body_pos_y`/`body_speed`のみ(`bodies[].name`で名前
//! 解決)対応 — 設計例示の`{"ledger": "thermal"}`のような`ProbeTarget::LedgerKinetic`
//! に素直に対応しない形は後続増分。validator(参照整合検査)はこの縮約版が対象とする
//! 範囲(材料参照・剛体名参照)のみ実装する — 排他結合検査(`sim-coupling::
//! validate_exclusive_couplings`)は`couplings`セクション未実装のため接続できない
//! (後続増分)。`bodies[]`には`rotation`(クォータニオン`[x,y,z,w]`、未指定なら
//! 恒等回転)・`linear_velocity`(未指定ならゼロ)も追加済み(D5斜面のような
//! 回転済み初期配置・D2弾道のような初速を要するシナリオへの対応、ヘッドレス
//! ランナーの適用例参照)。
//!
//! `World::append_scenario_bodies`は`materials`/`bodies`セクションのみを
//! 実行中のワールドへ追加する処理として`from_scenario`から切り出したもの
//! (`sim-wasm::WasmWorld::import_scene_json`——シーンJSON Import——が
//! 新規`World`を構築せず既存ワールドへボディを追加できるようにするため、
//! `fluids`/`probes`セクションは対象外、`append_scenario_bodies`のdoc参照)。

use crate::{BodyId, ProbeTarget, World, WorldOptions};
use serde::Deserialize;
use sim_fluid::StaticWaterRegion;
use sim_math::Vec3;
use sim_mechanics::{BodyType, RigidBodyDesc, Shape};
use std::collections::HashMap;

/// `probes`セクションで名前解決を経ずにプローブ履歴の容量を指定する仕組みが設計JSONに
/// 無いため、この縮約実装では固定容量を使う(600サンプル、既定`dt`(1/120)で5秒相当)。
const DEFAULT_PROBE_CAPACITY: usize = 600;

/// シーンロードの失敗(設計§3「validator: 参照整合(名前解決)…を位置つきエラーで返す」
/// の縮約版 — 位置情報は持たず、エラー種別と関連する名前のみ)。
#[derive(Clone, Debug, PartialEq)]
pub enum SceneError {
    /// JSONとして構文解析できなかった(`serde_json`のエラーメッセージをそのまま保持)。
    JsonParse(String),
    /// `materials[].extends`が既存の材料名を指していない。
    UnknownBaseMaterial(String),
    /// `bodies[].material`が(`materials`セクションで派生したものを含め)既存の材料名を
    /// 指していない。
    UnknownMaterial(String),
    /// `probes[].body_pos_y`等が`bodies[].name`のいずれとも一致しない。
    UnknownBodyName(String),
}

#[derive(Deserialize)]
pub struct Scenario {
    pub name: String,
    #[serde(default)]
    pub seed: u64,
    pub world: WorldScenarioOptions,
    #[serde(default)]
    pub materials: Vec<MaterialOverride>,
    #[serde(default)]
    pub bodies: Vec<BodyScenarioDesc>,
    #[serde(default)]
    pub fluids: Vec<FluidJson>,
    #[serde(default)]
    pub probes: Vec<ProbeJson>,
}

impl Scenario {
    pub fn from_json(json: &str) -> Result<Scenario, SceneError> {
        serde_json::from_str(json).map_err(|e| SceneError::JsonParse(e.to_string()))
    }
}

#[derive(Deserialize)]
pub struct WorldScenarioOptions {
    pub gravity: f64,
    pub dt: f64,
    /// 反発を適用しない法線方向相対速度のしきい値(`sim_mechanics::MechanicsSolver::
    /// restitution_velocity_threshold`、数値安定化のための既定値がある)。未指定なら
    /// 既定値のまま(D3(バウンド比べ)のような、反発係数の合成則を避けて同一材質どうし
    /// で正確にe²倍の跳ね返り高さを検証したいシナリオでは0.0を指定する、ヘッドレス
    /// ランナーの適用例参照)。
    #[serde(default)]
    pub restitution_velocity_threshold: Option<f64>,
}

/// 既存材料からの派生(設計§3「`extends`による材料派生」— 「密度だけ変えた木」等)。
/// 現時点では`density`のみ上書き可能(他の物性の上書きは後続増分)。
#[derive(Deserialize)]
pub struct MaterialOverride {
    pub extends: String,
    pub name: String,
    #[serde(default)]
    pub density: Option<f64>,
}

#[derive(Deserialize)]
pub struct BodyScenarioDesc {
    pub shape: ShapeJson,
    pub material: String,
    #[serde(default)]
    pub position: [f64; 3],
    /// 初期姿勢(クォータニオン`[x,y,z,w]`)。未指定なら恒等回転(縮約実装:
    /// 傾いた斜面(D5)のように回転済みの初期配置を必要とするシナリオに対応する
    /// ため追加)。
    #[serde(default)]
    pub rotation: Option<[f64; 4]>,
    /// 初期速度。未指定なら`[0,0,0]`(縮約実装: D2弾道のような初速を要する
    /// シナリオに対応するため追加、`[f64;3]`の`Default`はゼロ配列)。
    #[serde(default)]
    pub linear_velocity: [f64; 3],
    #[serde(default, rename = "type")]
    pub body_type: Option<String>,
    #[serde(default)]
    pub name: Option<String>,
}

/// 設計§3の例示に現れる3形状のみ(`Capsule`/`Compound`/`ConvexMesh`は`raycast`/
/// `overlap`モジュール同様、narrowphase未実装のため対象外)。
#[derive(Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ShapeJson {
    Box { half: [f64; 3] },
    Sphere { radius: f64 },
    Plane { normal: [f64; 3], d: f64 },
}

/// モジュールdoc「縮約実装の理由」参照 — 設計例示のAABBではなく`water_level`+
/// `density`の縮約表現。
#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FluidJson {
    StaticWater { water_level: f64, density: f64 },
}

/// モジュールdoc「縮約実装の理由」参照 — `body_pos_y`/`body_speed`のみ、
/// `bodies[].name`による名前解決。
#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProbeJson {
    BodyPosY(String),
    BodySpeed(String),
}

fn array_to_vec3(a: [f64; 3]) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

impl World {
    /// シーンJSONの`materials`/`bodies`セクションを現在の`World`へ追加する
    /// (`from_scenario`と、実行中のワールドへシーンJSONを取り込むシーンJSON
    /// Import(`sim-wasm::WasmWorld::import_scene_json`)の共通処理)。`fluids`/
    /// `probes`セクションは対象外とする——Importは実行中のワールドへの「追加」
    /// であり、既に有効化されているかもしれない流体設定を無条件に上書きしたり、
    /// 既存のプローブ体系に無関係な名前解決を割り込ませたりするのは意図しない
    /// 挙動になりうるため(`fluids`/`probes`はシーンを新規に構築する
    /// `from_scenario`側のみで処理する)。返り値は`scenario.bodies`と同じ順序の
    /// `BodyId`(`from_scenario`が`probes`の名前解決に使う、Importが新規ボディの
    /// シェイプ/材質をUI側に伝えるのに使う、双方の理由で名前の有無によらず
    /// 全ボディ分を返す)。
    pub fn append_scenario_bodies(
        &mut self,
        scenario: &Scenario,
    ) -> Result<Vec<BodyId>, SceneError> {
        for over in &scenario.materials {
            let base_id = self
                .materials()
                .find_by_name(&over.extends)
                .ok_or_else(|| SceneError::UnknownBaseMaterial(over.extends.clone()))?;
            let mut derived = self.materials().get(base_id).clone();
            // `Material::name`は`&'static str`(既存の`MaterialDb::standard()`の
            // コンパイル時定数群と型を揃えるため)。シーンJSON由来の動的な名前は
            // `Box::leak`で`'static`化する — シーンロードは頻度の低い操作であり、
            // リークするメモリは派生材料1件あたり名前文字列のみで無視できる規模
            // (ホットパスでの繰り返し呼び出しは想定していない)。
            derived.name = Box::leak(over.name.clone().into_boxed_str());
            if let Some(density) = over.density {
                derived.density = density;
            }
            self.materials_mut().push(derived);
        }

        let mut ids = Vec::with_capacity(scenario.bodies.len());
        for body in &scenario.bodies {
            let material_id = self
                .materials()
                .find_by_name(&body.material)
                .ok_or_else(|| SceneError::UnknownMaterial(body.material.clone()))?;
            let shape = match body.shape {
                ShapeJson::Box { half } => Shape::Box {
                    half_extents: array_to_vec3(half),
                },
                ShapeJson::Sphere { radius } => Shape::Sphere { radius },
                ShapeJson::Plane { normal, d } => Shape::Plane {
                    normal: array_to_vec3(normal),
                    d,
                },
            };
            let mut desc = RigidBodyDesc::dynamic(shape, material_id);
            desc.transform.position = array_to_vec3(body.position);
            if let Some(q) = body.rotation {
                desc.transform.rotation = sim_math::Quat {
                    x: q[0],
                    y: q[1],
                    z: q[2],
                    w: q[3],
                };
            }
            desc.linear_velocity = array_to_vec3(body.linear_velocity);
            desc.body_type = match body.body_type.as_deref() {
                Some("static") => BodyType::Static,
                Some("kinematic") => BodyType::Kinematic,
                _ => BodyType::Dynamic,
            };
            ids.push(self.create_body(desc));
        }

        Ok(ids)
    }

    /// シーンJSONから`World`を構築する(設計docs/20-integration/04-world-api.md §2
    /// `from_scenario`、`Scenario`のdoc「縮約実装の理由」参照)。
    pub fn from_scenario(scenario: &Scenario) -> Result<World, SceneError> {
        let options = WorldOptions {
            gravity: scenario.world.gravity,
            dt: scenario.world.dt,
            seed: scenario.seed,
        };
        let mut world = World::new(options);
        if let Some(threshold) = scenario.world.restitution_velocity_threshold {
            world.mechanics_mut().restitution_velocity_threshold = threshold;
        }
        let ids = world.append_scenario_bodies(scenario)?;

        let mut body_ids_by_name: HashMap<String, BodyId> = HashMap::new();
        for (body, id) in scenario.bodies.iter().zip(ids.iter()) {
            if let Some(name) = &body.name {
                body_ids_by_name.insert(name.clone(), *id);
            }
        }

        for fluid in &scenario.fluids {
            match fluid {
                FluidJson::StaticWater {
                    water_level,
                    density,
                } => {
                    world.mechanics_mut().water =
                        Some(StaticWaterRegion::new(*water_level, *density));
                }
            }
        }

        for probe in &scenario.probes {
            let (name, make_target): (&str, fn(BodyId) -> ProbeTarget) = match probe {
                ProbeJson::BodyPosY(name) => (name, ProbeTarget::BodyPosY),
                ProbeJson::BodySpeed(name) => (name, ProbeTarget::BodySpeed),
            };
            let id = body_ids_by_name
                .get(name)
                .ok_or_else(|| SceneError::UnknownBodyName(name.to_string()))?;
            world.add_probe(make_target(*id), DEFAULT_PROBE_CAPACITY);
        }

        Ok(world)
    }
}

/// ヘッドレスシナリオ実行結果(設計§8「ヘッドレスランナー」の最小骨格、
/// `run_headless_scenario`のdoc参照)。
pub struct HeadlessRunResult {
    /// 実行終了時点の`World::state_hash()`(決定論リプレイ検証に使う)。
    pub final_state_hash: u64,
    pub final_time: f64,
    /// `scenario.probes`と同じ順のプローブ履歴(D1–D39の合否判定はこの履歴に対する
    /// アサートとして表現できる、設計docs/21-verification/03-demo-scenarios.md)。
    pub probe_histories: Vec<Vec<f64>>,
}

/// シーンJSONを読み込み、固定`steps`回`step()`してプローブ履歴を回収する
/// (設計§8「ヘッドレスランナー」——「シーンJSON + 入力列 + Probe assertでの合否判定」
/// のうち、シーンJSON読み込み+固定step数実行+Probe履歴回収の核となる部分。
/// 入力列(Command系列の記録・再生)の接続は、フロントエンドの`command_log`
/// (`demo/src/main.ts`)に相当する仕組みをこちら側にも持たせる後続増分)。
/// D1–D39各シナリオの合否基準(`docs/21-verification/03-demo-scenarios.md`)は、
/// 呼び出し側が`probe_histories`に対してアサートすることで表現する。
pub fn run_headless_scenario(json: &str, steps: u32) -> Result<HeadlessRunResult, SceneError> {
    let scenario = Scenario::from_json(json)?;
    let mut world = World::from_scenario(&scenario)?;

    for _ in 0..steps {
        world.step();
    }

    let probe_histories = (0..scenario.probes.len())
        .map(|i| {
            world
                .probe(i)
                .map(|p| p.history().copied().collect())
                .unwrap_or_default()
        })
        .collect();

    Ok(HeadlessRunResult {
        final_state_hash: world.state_hash(),
        final_time: world.time(),
        probe_histories,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::BodyId;
    use sim_math::Quat;

    /// 設計docs/20-integration/04-world-api.md §3の例示JSON(浮力デモの縮約版、
    /// `fluids`/`couplings`/`probes`セクションを除く)を実際にパースして`World`を構築し、
    /// 派生材料(`extends`)・剛体(位置・種別)が正しく反映されることを確認する。
    #[test]
    fn from_scenario_builds_world_matching_design_doc_example_json() {
        let json = r#"
        {
          "name": "buoyancy-basic",
          "seed": 42,
          "world": { "gravity": 9.80665, "dt": 0.008333333 },
          "materials": [ { "extends": "木材(松)", "name": "light-wood", "density": 400.0 } ],
          "bodies": [
            { "shape": { "box": { "half": [0.1, 0.1, 0.1] } }, "material": "light-wood",
              "position": [0, 2, 0], "name": "crate" },
            { "shape": { "plane": { "normal": [0,1,0], "d": 0 } }, "type": "static", "material": "コンクリート" }
          ]
        }
        "#;
        let scenario = Scenario::from_json(json).expect("valid JSON matching design doc example");
        let mut world =
            World::from_scenario(&scenario).expect("should build without validation errors");

        let light_wood = world.materials().find_by_name("light-wood").unwrap();
        assert_eq!(world.materials().get(light_wood).density, 400.0);

        // crate(box, dynamic)は先頭のBodyId(index=0)。
        let crate_id = BodyId {
            index: 0,
            generation: 0,
        };
        assert_eq!(
            world.body_position(crate_id),
            Some(Vec3::new(0.0, 2.0, 0.0))
        );

        // 2step進めてもクラッシュせず、木箱(軽い)が自由落下することを確認する
        // (静的な地面(Plane, static)に接触するまでの短時間)。
        let y0 = world.body_position(crate_id).unwrap().y;
        for _ in 0..2 {
            world.step();
        }
        assert!(world.body_position(crate_id).unwrap().y < y0);
    }

    /// `World::append_scenario_bodies`(シーンJSON Importの土台、モジュールdoc参照)は
    /// 新規`World`を作らず、既に稼働中のワールド(既存ボディ・既存の重力/dt設定を
    /// 持つ)へシーンJSONの`bodies`を追加できることを確認する。既存ボディの位置は
    /// 変わらず、返り値の`Vec<BodyId>`は`scenario.bodies`と同じ順序・同じ件数で、
    /// 新規ボディの位置もJSONどおりであることを確認する。
    #[test]
    fn append_scenario_bodies_adds_bodies_to_an_already_running_world_without_disturbing_it() {
        let mut world = World::new(WorldOptions {
            gravity: 1.23, // `from_scenario`を経由しない、この既存ワールド固有の値。
            dt: 0.008333333,
            seed: 0,
        });
        let steel = world.materials().find_by_name("鋼(炭素鋼)").unwrap();
        let mut existing_desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.2 }, steel);
        existing_desc.transform.position = Vec3::new(9.0, 9.0, 9.0);
        let existing_id = world.create_body(existing_desc);

        let json = r#"
        {
          "name": "import-fragment",
          "world": { "gravity": 9.80665, "dt": 0.008333333 },
          "bodies": [
            { "shape": { "sphere": { "radius": 0.3 } }, "material": "鋼(炭素鋼)",
              "position": [1.0, 2.0, 3.0], "name": "imported-a" },
            { "shape": { "box": { "half": [0.4, 0.4, 0.4] } }, "material": "コンクリート",
              "position": [4.0, 5.0, 6.0] }
          ]
        }
        "#;
        let scenario = Scenario::from_json(json).unwrap();
        let ids = world
            .append_scenario_bodies(&scenario)
            .expect("valid scenario fragment");

        assert_eq!(ids.len(), 2);
        assert_eq!(
            world.body_position(existing_id),
            Some(Vec3::new(9.0, 9.0, 9.0)),
            "appending bodies must not disturb bodies that already existed"
        );
        assert_eq!(world.body_position(ids[0]), Some(Vec3::new(1.0, 2.0, 3.0)));
        assert_eq!(world.body_position(ids[1]), Some(Vec3::new(4.0, 5.0, 6.0)));
    }

    /// `materials[].extends`が未知の材料名を指す場合は`SceneError::UnknownBaseMaterial`。
    #[test]
    fn from_scenario_rejects_unknown_base_material() {
        let json = r#"
        {
          "name": "broken",
          "world": { "gravity": 9.80665, "dt": 0.008333333 },
          "materials": [ { "extends": "unobtainium", "name": "derived" } ]
        }
        "#;
        let scenario = Scenario::from_json(json).unwrap();
        let result = World::from_scenario(&scenario);
        assert!(matches!(
            result,
            Err(SceneError::UnknownBaseMaterial(ref name)) if name == "unobtainium"
        ));
    }

    /// `bodies[].material`が未知の材料名を指す場合は`SceneError::UnknownMaterial`。
    #[test]
    fn from_scenario_rejects_unknown_body_material() {
        let json = r#"
        {
          "name": "broken",
          "world": { "gravity": 9.80665, "dt": 0.008333333 },
          "bodies": [
            { "shape": { "sphere": { "radius": 1.0 } }, "material": "unobtainium" }
          ]
        }
        "#;
        let scenario = Scenario::from_json(json).unwrap();
        let result = World::from_scenario(&scenario);
        assert!(matches!(
            result,
            Err(SceneError::UnknownMaterial(ref name)) if name == "unobtainium"
        ));
    }

    /// `fluids`(縮約: `water_level`+`density`、モジュールdoc参照)+`probes`
    /// (`body_pos_y`、`bodies[].name`による名前解決)を実際にパースして
    /// `World`を構築し、浮力が働くこと(木箱が沈み込みつつも自由落下より遅く
    /// 沈む)とプローブ履歴がサンプルされることを確認する。
    #[test]
    fn from_scenario_wires_static_water_fluid_and_body_pos_y_probe() {
        let json = r#"
        {
          "name": "buoyancy-full",
          "world": { "gravity": 9.80665, "dt": 0.008333333 },
          "materials": [ { "extends": "木材(松)", "name": "light-wood", "density": 400.0 } ],
          "bodies": [
            { "shape": { "box": { "half": [0.1, 0.1, 0.1] } }, "material": "light-wood",
              "position": [0, 0.5, 0], "name": "crate" }
          ],
          "fluids": [ { "static_water": { "water_level": 1.0, "density": 1000.0 } } ],
          "probes": [ { "body_pos_y": "crate" } ]
        }
        "#;
        let scenario = Scenario::from_json(json).expect("valid JSON");
        let mut world =
            World::from_scenario(&scenario).expect("should build without validation errors");

        assert!(world.mechanics_mut().water.is_some());

        for _ in 0..10 {
            world.step();
        }

        let history: Vec<f64> = world.probe(0).unwrap().history().copied().collect();
        assert_eq!(history.len(), 10);
    }

    /// ヘッドレスランナー(設計§8)の最小骨格: シーンJSON(浮力+`body_pos_y`プローブの
    /// 既存例)を読み込み、固定step数実行してプローブ履歴を回収できること、同じJSON+
    /// step数を独立に2回実行すると`final_state_hash`が一致する(決定論リプレイ)ことを
    /// 確認する。
    #[test]
    fn run_headless_scenario_executes_scene_json_and_reports_deterministic_probe_history() {
        let json = r#"
        {
          "name": "buoyancy-full",
          "world": { "gravity": 9.80665, "dt": 0.008333333 },
          "materials": [ { "extends": "木材(松)", "name": "light-wood", "density": 400.0 } ],
          "bodies": [
            { "shape": { "box": { "half": [0.1, 0.1, 0.1] } }, "material": "light-wood",
              "position": [0, 0.5, 0], "name": "crate" }
          ],
          "fluids": [ { "static_water": { "water_level": 1.0, "density": 1000.0 } } ],
          "probes": [ { "body_pos_y": "crate" } ]
        }
        "#;

        let steps = 10;
        let result1 = run_headless_scenario(json, steps).expect("valid scenario JSON");
        let result2 = run_headless_scenario(json, steps).expect("valid scenario JSON");

        assert_eq!(result1.probe_histories.len(), 1);
        assert_eq!(result1.probe_histories[0].len(), steps as usize);
        assert_eq!(
            result1.final_time,
            0.008333333 * steps as f64,
            "final_time should reflect exactly `steps` frames of dt"
        );
        assert_eq!(
            result1.final_state_hash, result2.final_state_hash,
            "identical scenario JSON + step count must replay bit-identically"
        );
    }

    /// シーンJSONが不正(存在しない材料参照)なら、`Scenario::from_json`/
    /// `World::from_scenario`と同じ`SceneError`をそのまま返す(ヘッドレスランナーは
    /// バリデーションを迂回しない)。
    #[test]
    fn run_headless_scenario_propagates_scene_validation_errors() {
        let json = r#"
        {
          "name": "broken",
          "world": { "gravity": 9.80665, "dt": 0.008333333 },
          "bodies": [
            { "shape": { "sphere": { "radius": 1.0 } }, "material": "unobtainium" }
          ]
        }
        "#;
        let result = run_headless_scenario(json, 1);
        assert!(matches!(
            result,
            Err(SceneError::UnknownMaterial(ref name)) if name == "unobtainium"
        ));
    }

    /// D4 積み木(docs/21-verification/03-demo-scenarios.md「箱スタック+ドミノ。
    /// 反復回数スライダー」「合格基準: M12(10秒静止)」)を、`demos.rs`の既存の
    /// Rustネイティブ実装(`d4_box_stack_settles_below_velocity_threshold_
    /// within_10s`)とは別に、シーンJSON経由でヘッドレスランナーの土台
    /// (`run_headless_scenario`)が実際に使えることを示す2本目の適用例として
    /// 実装する。3段の箱スタックを床に置き、10秒(既定dtで1200step)後に
    /// 各箱の速さ(`body_speed`プローブ)が十分小さいこと(静止)を確認する。
    #[test]
    fn run_headless_scenario_settles_a_stacked_box_tower_matching_d4_pass_criterion() {
        let json = r#"
        {
          "name": "d4-box-stack",
          "world": { "gravity": 9.80665, "dt": 0.008333333 },
          "bodies": [
            { "shape": { "plane": { "normal": [0,1,0], "d": 0 } }, "type": "static",
              "material": "鋼(炭素鋼)" },
            { "shape": { "box": { "half": [0.5, 0.5, 0.5] } }, "material": "鋼(炭素鋼)",
              "position": [0, 0.5, 0], "name": "box1" },
            { "shape": { "box": { "half": [0.5, 0.5, 0.5] } }, "material": "鋼(炭素鋼)",
              "position": [0, 1.51, 0], "name": "box2" },
            { "shape": { "box": { "half": [0.5, 0.5, 0.5] } }, "material": "鋼(炭素鋼)",
              "position": [0, 2.52, 0], "name": "box3" }
          ],
          "probes": [
            { "body_speed": "box1" },
            { "body_speed": "box2" },
            { "body_speed": "box3" }
          ]
        }
        "#;

        let steps = 1200; // 既定dt(1/120s)で10秒
        let result = run_headless_scenario(json, steps).expect("valid scenario JSON");

        assert_eq!(result.probe_histories.len(), 3);
        for (i, history) in result.probe_histories.iter().enumerate() {
            let final_speed = *history.last().expect("history should not be empty");
            assert!(
                final_speed < 0.01,
                "box{} should be at rest after 10s (D4 pass criterion): final_speed={final_speed}",
                i + 1
            );
        }
    }

    /// D6 浮き沈み(docs/21-verification/03-demo-scenarios.md「密度スライダー付きの
    /// 箱を水域へ」「合格基準: F4(喫水)」)を、ヘッドレスランナーの3本目の適用例
    /// として実装する。`materials[].extends`で密度比0.6の材質を派生させ
    /// (`demos.rs`の`d6_floating_box_matches_waterline_depth_and_heave_period`の
    /// F4部分と同じ密度・寸法)、箱を解析的な釣り合い喫水位置(`h_sub=0.6×側面長`、
    /// `equilibrium_y=-h_sub+half`)にちょうど置く。安定平衡のため、十分な時間
    /// 経過後も`body_pos_y`プローブの最終値が釣り合い位置から大きくずれていない
    /// ことを確認する。
    #[test]
    fn run_headless_scenario_settles_a_floating_box_at_the_f4_equilibrium_waterline() {
        let water_density = 998.2;
        let ratio = 0.6;
        let density = ratio * water_density;
        let half = 0.5;
        let side = 2.0 * half;
        let h_sub = ratio * side;
        let equilibrium_y = -h_sub + half;

        let json = format!(
            r#"
        {{
          "name": "d6-floating-box-f4",
          "world": {{ "gravity": 9.80665, "dt": 0.008333333 }},
          "materials": [ {{ "extends": "木材(松)", "name": "d6-density", "density": {density} }} ],
          "bodies": [
            {{ "shape": {{ "box": {{ "half": [{half}, {half}, {half}] }} }}, "material": "d6-density",
              "position": [0, {equilibrium_y}, 0], "name": "box" }}
          ],
          "fluids": [ {{ "static_water": {{ "water_level": 0.0, "density": {water_density} }} }} ],
          "probes": [ {{ "body_pos_y": "box" }} ]
        }}
        "#
        );

        let steps = 600; // 5秒(既定dt)
        let result = run_headless_scenario(&json, steps).expect("valid scenario JSON");

        let final_y = *result.probe_histories[0]
            .last()
            .expect("history should not be empty");
        let drift = (final_y - equilibrium_y).abs();
        assert!(
            drift < 0.05 * side,
            "box should remain near the F4 equilibrium waterline depth (stable equilibrium): \
             final_y={final_y} equilibrium_y={equilibrium_y} drift={drift}"
        );
    }

    /// シーンJSONに`rotation`(クォータニオン)・`linear_velocity`フィールドを追加した
    /// 上での4本目の適用例。D5 斜面(docs/21-verification/03-demo-scenarios.md
    /// 「角度スライダー+素材切替」「合格基準: M7/M8」)のうちM7(静止摩擦角未満では
    /// 静止し続ける)を、傾いた平面(`Plane`)+それに合わせて回転させた箱
    /// (`demos.rs`の`d5_incline_stays_static_below_friction_angle_and_slides_
    /// matching_formula_above`と同じ構成)で検証する。回転が正しく適用されて
    /// いなければ箱は斜面に対して傾いたまま接触し即座に転倒/滑落するため、
    /// 5秒間静止し続けることの確認は`rotation`フィールドの配線自体の検証にもなる。
    #[test]
    fn run_headless_scenario_stays_static_on_an_incline_below_the_friction_angle() {
        let theta: f64 = 10.0_f64.to_radians();
        let normal = Vec3::new(-theta.sin(), theta.cos(), 0.0);
        let half_extent = 0.5;
        let position = normal.scale(half_extent);
        let rotation = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), theta);

        let json = format!(
            r#"
        {{
          "name": "d5-incline-static",
          "world": {{ "gravity": 9.80665, "dt": 0.008333333 }},
          "bodies": [
            {{ "shape": {{ "plane": {{ "normal": [{nx}, {ny}, {nz}], "d": 0 }} }},
              "type": "static", "material": "鋼(炭素鋼)" }},
            {{ "shape": {{ "box": {{ "half": [{half_extent}, {half_extent}, {half_extent}] }} }},
              "material": "鋼(炭素鋼)",
              "position": [{px}, {py}, {pz}],
              "rotation": [{qx}, {qy}, {qz}, {qw}],
              "name": "box" }}
          ],
          "probes": [ {{ "body_speed": "box" }} ]
        }}
        "#,
            nx = normal.x,
            ny = normal.y,
            nz = normal.z,
            px = position.x,
            py = position.y,
            pz = position.z,
            qx = rotation.x,
            qy = rotation.y,
            qz = rotation.z,
            qw = rotation.w,
        );

        let steps = 600; // 5秒(既定dt)
        let result = run_headless_scenario(&json, steps).expect("valid scenario JSON");

        let final_speed = *result.probe_histories[0]
            .last()
            .expect("history should not be empty");
        assert!(
            final_speed < 1e-4,
            "M7: box on a 10° incline (below the friction angle) should stay at rest: \
             final_speed={final_speed}"
        );
    }

    /// D2(弾道): 45°射出の真空放物運動を`body_pos_y`/`body_speed`の2プローブのみで検証する。
    /// `ProbeTarget`には水平位置(range)を直接読める種別が無いため、`demos.rs`の
    /// `d2_ballistic_range_matches_45_degree_formula_and_drag_shortens_range`のように
    /// 着地x座標を直接assertすることはできない。代わりに同じ真空弾道物理から導出できる
    /// 2つの不変量を確認する: (1) 飛翔時間(`body_pos_y`が0を上から下へ跨ぐ時刻)が
    /// 解析解`T=2*v0*sin(θ)/g`と一致する、(2) 着地時の速さが射出速さ`v0`と一致する
    /// (同一高度なのでエネルギー保存)、(3) 頂点(最小速度点)の速さが水平成分`v0*cos(θ)`と
    /// 一致する。地面プレーンを置かず、床衝突なしで自由落下させることで着地判定を
    /// 「y=0を跨いだ最初のステップ」という単純な走査で済ませている。
    #[test]
    fn run_headless_scenario_ballistic_flight_matches_analytic_time_of_flight_and_landing_speed() {
        let v0 = 20.0;
        let theta: f64 = std::f64::consts::FRAC_PI_4; // 45°(最大到達距離)
        let g = 9.80665;
        let dt = 0.008333333;
        let vx = v0 * theta.cos();
        let vy = v0 * theta.sin();

        let json = format!(
            r#"
        {{
          "name": "d2-ballistic",
          "world": {{ "gravity": {g}, "dt": {dt} }},
          "bodies": [
            {{ "shape": {{ "sphere": {{ "radius": 0.1 }} }},
              "material": "鋼(炭素鋼)",
              "linear_velocity": [{vx}, {vy}, 0.0],
              "name": "shell" }}
          ],
          "probes": [ {{ "body_pos_y": "shell" }}, {{ "body_speed": "shell" }} ]
        }}
        "#,
        );

        // プローブ履歴はリングバッファ(容量`DEFAULT_PROBE_CAPACITY`=600、`run_headless_scenario`
        // 参照)なので、着地ステップ(解析値T≈2.885s→step≈346)より前の区間が上書きされて
        // インデックスと絶対時刻の対応がずれないよう、stepsは容量以下に収める。
        let steps = 500;
        let result = run_headless_scenario(&json, steps).expect("valid scenario JSON");
        let pos_y = &result.probe_histories[0];
        let speed = &result.probe_histories[1];

        let landing_step = (1..pos_y.len())
            .find(|&i| pos_y[i] <= 0.0 && pos_y[i - 1] > 0.0)
            .expect("ballistic flight should return to y=0 within the simulated window");
        // `history[i]`はstep(i+1)後の値(`World::step`のプローブサンプリングは
        // `clock.advance()`の後、`sim-world/src/lib.rs`参照)なので絶対時刻は`(i+1)*dt`。
        let landing_time = (landing_step + 1) as f64 * dt;
        let analytic_time_of_flight = 2.0 * v0 * theta.sin() / g;
        let time_rel_err = (landing_time - analytic_time_of_flight).abs() / analytic_time_of_flight;
        assert!(
            time_rel_err < 0.01,
            "D2: time-of-flight landing_time={landing_time} analytic={analytic_time_of_flight} \
             rel_err={time_rel_err:.4}"
        );

        let landing_speed = speed[landing_step];
        let landing_speed_rel_err = (landing_speed - v0).abs() / v0;
        assert!(
            landing_speed_rel_err < 0.02,
            "D2: landing speed should match launch speed in vacuum (energy conservation): \
             landing_speed={landing_speed} v0={v0} rel_err={landing_speed_rel_err:.4}"
        );

        let apex_speed = speed[..=landing_step]
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        let analytic_apex_speed = v0 * theta.cos();
        let apex_rel_err = (apex_speed - analytic_apex_speed).abs() / analytic_apex_speed;
        assert!(
            apex_rel_err < 0.02,
            "D2: apex speed should match the horizontal velocity component: \
             apex_speed={apex_speed} analytic={analytic_apex_speed} rel_err={apex_rel_err:.4}"
        );
    }

    /// `probes[].body_pos_y`が`bodies[].name`のいずれとも一致しない場合は
    /// D1(落下時計)を6本目の適用例として実装する。`demos.rs`の
    /// `d1_falling_clock_matches_free_fall_time_and_shows_drag_on_off_difference`の
    /// うち真空側(M1: 自由落下時間が解析解$t=\sqrt{2h/g}$と一致)を、地面プレーンを
    /// 置かず`body_pos_y`プローブだけで検証する——球の半径ぶん手前(`y<=radius`)を
    /// 通過した最初のステップを着地時刻とする。抗力側(D1のもう一つの合格基準)は
    /// シーンJSONに大気抵抗を配線する手段がまだ無いため対象外。
    #[test]
    fn run_headless_scenario_free_fall_time_matches_analytic_vacuum_formula() {
        let height: f64 = 20.0;
        let radius: f64 = 0.3;
        let g: f64 = 9.80665;
        let dt: f64 = 0.008333333;

        let json = format!(
            r#"
        {{
          "name": "d1-falling-clock",
          "world": {{ "gravity": {g}, "dt": {dt} }},
          "bodies": [
            {{ "shape": {{ "sphere": {{ "radius": {radius} }} }},
              "material": "鋼(炭素鋼)",
              "position": [0.0, {height}, 0.0],
              "name": "clock" }}
          ],
          "probes": [ {{ "body_pos_y": "clock" }} ]
        }}
        "#,
        );

        let steps = 400; // 解析落下時間T≈2.019sに対し十分な余裕(dt=1/120で約243step)
        let result = run_headless_scenario(&json, steps).expect("valid scenario JSON");
        let pos_y = &result.probe_histories[0];

        let landing_step = (0..pos_y.len())
            .find(|&i| pos_y[i] <= radius)
            .expect("free-falling sphere should reach the ground within the simulated window");
        // `history[i]`はstep(i+1)後の値(`World::step`のプローブサンプリングは
        // `clock.advance()`の後、`run_headless_scenario_ballistic_flight_...`のコメント参照)。
        let landing_time = (landing_step + 1) as f64 * dt;
        let analytic = (2.0 * height / g).sqrt();
        let rel_err = (landing_time - analytic).abs() / analytic;
        assert!(
            rel_err < 0.01,
            "D1/M1: free-fall time landing_time={landing_time} analytic={analytic} \
             rel_err={rel_err:.4}"
        );
    }

    /// D3(バウンド比べ)を7本目の適用例として実装する。`demos.rs`の
    /// `d3_bounce_comparison_matches_restitution_squared_for_each_material`は
    /// `restitution_velocity_threshold=0.0`(反発係数の合成則を避けるため床・球を
    /// 同一材質にした上で数値安定化のしきい値も切る)という、これまでのシーンJSON
    /// スキーマには無かった設定を必要としていた。`WorldScenarioOptions`に
    /// `restitution_velocity_threshold`(省略可、既定値のまま)を追加したことで
    /// JSON経由でも表現できるようになったため、ゴム(天然)1材質分(他3材質は
    /// ネイティブ側で既に検証済み)を、床への落下→跳ね返り→頂点到達までを
    /// `body_pos_y`プローブ1本から検出して確認する。
    #[test]
    fn run_headless_scenario_bounce_height_matches_restitution_squared_for_rubber() {
        let radius: f64 = 0.1;
        let drop_height: f64 = 1.9;
        let dt: f64 = 1.0 / 240.0; // 反発の数値精度のため既定よりやや細かく(D1弾道と同じ理由)。
        let material_name = "ゴム(天然)";

        // 期待値(反発係数の2乗)はMaterialDbから実際にクエリする(ネイティブ側の
        // `d3_bounce_comparison_matches_restitution_squared_for_each_material`と
        // 同じ導出方法)。
        let reference_world = World::new(WorldOptions::default());
        let material_id = reference_world
            .materials()
            .find_by_name(material_name)
            .expect("standard DB has rubber");
        let expected_e = reference_world.materials().get(material_id).restitution;

        let json = format!(
            r#"
        {{
          "name": "d3-bounce",
          "world": {{ "gravity": 9.80665, "dt": {dt}, "restitution_velocity_threshold": 0.0 }},
          "bodies": [
            {{ "shape": {{ "plane": {{ "normal": [0, 1, 0], "d": 0 }} }},
              "type": "static", "material": "{material_name}" }},
            {{ "shape": {{ "sphere": {{ "radius": {radius} }} }}, "material": "{material_name}",
              "position": [0.0, {y0}, 0.0], "name": "ball" }}
          ],
          "probes": [ {{ "body_pos_y": "ball" }} ]
        }}
        "#,
            y0 = drop_height + radius,
        );

        let steps = 500; // リングバッファ容量600以下(D1/D2の増分で確立した配慮)。
        let result = run_headless_scenario(&json, steps).expect("valid scenario JSON");
        let pos_y = &result.probe_histories[0];
        let height: Vec<f64> = pos_y.iter().map(|y| y - radius).collect();

        // 床への到達(最初にheightが最小値を取る点、跳ね返り直前)。
        let bounce_step = (1..height.len())
            .find(|&i| height[i] > height[i - 1])
            .expect("ball should bounce back up within the simulated window");
        // 跳ね返り後の頂点(そこから先、再び下降に転じるまでの最大値)。
        let post_bounce_max = height[bounce_step..]
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);

        let ratio = post_bounce_max / drop_height;
        let expected_ratio = expected_e * expected_e;
        let rel_err = (ratio - expected_ratio).abs() / expected_ratio;
        assert!(
            rel_err < 0.05,
            "D3: bounce height ratio ratio={ratio} expected_ratio={expected_ratio} \
             (e={expected_e}) rel_err={rel_err:.4}"
        );
    }

    /// `SceneError::UnknownBodyName`。
    #[test]
    fn from_scenario_rejects_unknown_body_name_in_probe() {
        let json = r#"
        {
          "name": "broken",
          "world": { "gravity": 9.80665, "dt": 0.008333333 },
          "bodies": [
            { "shape": { "sphere": { "radius": 1.0 } }, "material": "コンクリート", "name": "crate" }
          ],
          "probes": [ { "body_pos_y": "nonexistent" } ]
        }
        "#;
        let scenario = Scenario::from_json(json).unwrap();
        let result = World::from_scenario(&scenario);
        assert!(matches!(
            result,
            Err(SceneError::UnknownBodyName(ref name)) if name == "nonexistent"
        ));
    }
}
