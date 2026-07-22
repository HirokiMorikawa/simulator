//! シーン記述(JSON)。設計 docs/20-integration/04-world-api.md §3。
//!
//! **縮約実装の理由**: 設計例示のJSONスキーマ(`world`/`materials`/`bodies`/`fluids`/
//! `couplings`/`probes`)のうち、`world`・`materials`(`extends`派生)・`bodies`のみを
//! 実装する。`fluids`(静的水域/SPH等の流体ドメイン)・`couplings`(`Coupling` registry
//! がまだ`World::step()`に接続されていない、各`sim-coupling`実装のモジュールdoc参照)・
//! `probes`(シーンJSON上の文字列ターゲット解決)は、対応する`World`側の機能自体が
//! まだ限定的なため後続増分で追加する。validator(参照整合検査)は、この縮約版が
//! 対象とする範囲(材料参照)のみ実装する — 排他結合検査(`sim-coupling::
//! validate_exclusive_couplings`)は`couplings`セクション未実装のため接続できない
//! (後続増分)。

use crate::{World, WorldOptions};
use serde::Deserialize;
use sim_math::Vec3;
use sim_mechanics::{BodyType, RigidBodyDesc, Shape};

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
            world.create_body(desc);
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
}
