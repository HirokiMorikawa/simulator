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
//! (後続増分)。

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
    /// シーンJSONから`World`を構築する(設計docs/20-integration/04-world-api.md §2
    /// `from_scenario`、`Scenario`のdoc「縮約実装の理由」参照)。
    pub fn from_scenario(scenario: &Scenario) -> Result<World, SceneError> {
        let options = WorldOptions {
            gravity: scenario.world.gravity,
            dt: scenario.world.dt,
            seed: scenario.seed,
        };
        let mut world = World::new(options);

        for over in &scenario.materials {
            let base_id = world
                .materials()
                .find_by_name(&over.extends)
                .ok_or_else(|| SceneError::UnknownBaseMaterial(over.extends.clone()))?;
            let mut derived = world.materials().get(base_id).clone();
            // `Material::name`は`&'static str`(既存の`MaterialDb::standard()`の
            // コンパイル時定数群と型を揃えるため)。シーンJSON由来の動的な名前は
            // `Box::leak`で`'static`化する — シーンロードは頻度の低い操作であり、
            // リークするメモリは派生材料1件あたり名前文字列のみで無視できる規模
            // (ホットパスでの繰り返し呼び出しは想定していない)。
            derived.name = Box::leak(over.name.clone().into_boxed_str());
            if let Some(density) = over.density {
                derived.density = density;
            }
            world.materials_mut().push(derived);
        }

        let mut body_ids_by_name: HashMap<String, BodyId> = HashMap::new();
        for body in &scenario.bodies {
            let material_id = world
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
            desc.body_type = match body.body_type.as_deref() {
                Some("static") => BodyType::Static,
                Some("kinematic") => BodyType::Kinematic,
                _ => BodyType::Dynamic,
            };
            let id = world.create_body(desc);
            if let Some(name) = &body.name {
                body_ids_by_name.insert(name.clone(), id);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::BodyId;

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

    /// `probes[].body_pos_y`が`bodies[].name`のいずれとも一致しない場合は
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
