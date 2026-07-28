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
use sim_mechanics::{BodyType, DragModel, RigidBodyDesc, Shape};
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
    /// **排他であるべき結合が同一シーンで同時に有効になっている**(増分F1で接続)。
    /// 設計 docs/20-integration/01-coupling-matrix.md §2規則2 が列挙する3組
    /// (浮力: 静的水域×解像流体、空気抗力: 集中定数×格子結合、コンデンサ電場
    /// エネルギー: 回路×静電場)を`sim_coupling::validate_exclusive_couplings`で
    /// 検出したもの。**同じ物理量を二重計上するとエネルギー台帳が静かに壊れる**
    /// ため、シーン読み込みの時点で弾く。
    ExclusiveCouplingViolation(Vec<sim_coupling::ExclusiveCouplingViolation>),
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
    /// 熱ドメイン(`sim_thermal::ThermalSolver`)。未指定なら無効(縮約実装:
    /// ノード間リンク(`ThermalSolver::add_link`、伝導ネットワーク)・放射
    /// (`emissivity`)は対象外——D9(冷めるコーヒー)が要る「対流のみの単一
    /// ノード」のみサポートする、モジュールdoc「縮約実装の理由」参照)。
    #[serde(default)]
    pub thermal: Option<ThermalScenarioJson>,
    /// 剛体間の拘束(設計の例示JSONには無い項目——D11(振り子と時計)が要る
    /// ワールド固定点/剛体間の距離拘束に対応するため追加した拡張、モジュールdoc
    /// 「縮約実装の理由」参照)。`from_scenario`側のみで処理する(`fluids`/
    /// `thermal`/`probes`と同じ理由——Importは実行中ワールドへの「追加」であり、
    /// 既存の拘束体系に無関係な参照を割り込ませたくないため対象外)。
    #[serde(default)]
    pub joints: Vec<JointJson>,
    /// `sim-coupling`の`Coupling`実装群(設計の例示JSONでは`["buoyancy_drag", ...]`の
    /// ような名前配列だが、`sim-coupling`registry自体が未接続な設計の縮約状態
    /// (モジュールdoc冒頭参照)を反映し、この実装では各`Coupling`をパラメータ付きで
    /// 直接構成する形にする)。現時点では`ImageChargeForce`(D26帯電風船の鏡像力)・
    /// `BrownianForce`(D25ブラウン運動)・`InductionCoupling`(D21銅管落下の渦電流
    /// ブレーキ)のみ対応——残り11種は後続増分(この`couplings`セクション自体、
    /// 設計が「排他結合検査」を要求する多対多の相互作用検証はまだ及ばない、
    /// 最小の一歩)。`joints`と同じ理由で`from_scenario`側のみで処理する。
    #[serde(default)]
    pub couplings: Vec<CouplingJson>,
    /// 回路ドメイン(`sim_em::Circuit`)。未指定なら無効。ノードはインデックス
    /// (`sim_em::GROUND`=0が接地、`Circuit::new`と同じ規約)——ノード名前付けの
    /// 手間を避けるため`thermal.nodes`同様インデックス直接指定とする。現時点では
    /// 抵抗器・電圧源のみ対応(コンデンサ・スイッチ・ダイオードは後続増分、
    /// D21銅管落下が要る最小構成のみ)。
    #[serde(default)]
    pub circuit: Option<CircuitScenarioJson>,
    /// 天体ドメイン(`sim_astro::NBodySystem`)。未指定なら無効。`mechanics`の
    /// `bodies`(形状+材質を持つ剛体)とは別種の質点集合であり、質量中心運動や
    /// 衝突判定は対象外——D34(太陽系儀)/D35(軌道投入)が要る2体重力の最小構成
    /// (大気抗力・相対論補正は後続増分)。
    #[serde(default)]
    pub astro: Option<AstroScenarioJson>,
    #[serde(default)]
    pub probes: Vec<ProbeJson>,
    /// 予測→実験ミニパネル(設計docs/23-frontend/01-editor.md §5「予測→実験
    /// (オプションのミニパネル)」)向けのメタデータ。物理には影響しない
    /// (`from_scenario`/`append_scenario_bodies`はこのフィールドを読まない)——
    /// フロントエンド(`sim-wasm::WasmWorld::import_scene_json`)がインポート時に
    /// 生のJSONを独立に読んで、ユーザーが数値予測を書ける入力欄+実測値との
    /// 比較表を表示するためだけの、意味を検証しない自由記述のヒント。
    #[serde(default)]
    pub prediction_prompts: Vec<PredictionPromptJson>,
}

/// `Scenario::prediction_prompts`の1件(モジュールdoc参照)。
#[derive(Deserialize)]
pub struct PredictionPromptJson {
    pub question: String,
    /// `probes`配列内でのインデックス(0起点)。この予測が対応するプローブ。
    pub probe_index: usize,
    pub expected_value: f64,
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
    /// 静止大気/流体(`sim_mechanics::MechanicsSolver::atmosphere`、
    /// `sim_fluid::Atmosphere::still`)。未指定なら無効(既定は真空、これまでの
    /// ヘッドレスランナー適用例の挙動を変えない)。D7(風と終端速度)のような
    /// 球の抗力(F1高Re/F3低Re、いずれも同じ`sim_fluid::drag_force_sphere`が
    /// レイノルズ数から自動選択)を検証するシナリオに対応するため追加。
    #[serde(default)]
    pub atmosphere: Option<AtmosphereJson>,
}

/// `WorldScenarioOptions::atmosphere`(モジュールdoc参照)。
#[derive(Deserialize)]
pub struct AtmosphereJson {
    pub density: f64,
    pub viscosity: f64,
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
    /// 球の抗力モデル(`sim_mechanics::DragModel::Sphere`)を有効化するかどうか。
    /// 縮約実装: 対応する`DragModel`は球のみ(`sim-mechanics`自体に他の抗力
    /// モデルが無いため)なので、この体の`shape`が`Sphere`でない場合は無視する
    /// (D7(風と終端速度)向け、`WorldScenarioOptions::atmosphere`と併用する)。
    #[serde(default)]
    pub drag: bool,
    /// 質量の直接指定(`sim_mechanics::RigidBodyDesc::mass_override`)。未指定なら
    /// 形状+材質密度から自動計算(既定の挙動を変えない)。D11(振り子)の
    /// 「質点」(形状は球だが振り子の物理としては質量のみが意味を持つ)のような
    /// シナリオに対応するため追加。
    #[serde(default)]
    pub mass_override: Option<f64>,
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

/// `Scenario::thermal`(モジュールdoc「縮約実装の理由」参照)。
#[derive(Deserialize)]
pub struct ThermalScenarioJson {
    pub ambient_temperature: f64,
    #[serde(default)]
    pub nodes: Vec<ThermalNodeJson>,
}

/// `ThermalScenarioJson::nodes`の1件(`sim_thermal::ThermalNode`の縮約表現、
/// `heat_accum`は毎step`Solver::step`前にゼロクリアされる中間値のため
/// シーンJSONの対象外)。
#[derive(Deserialize)]
pub struct ThermalNodeJson {
    pub temperature: f64,
    pub heat_capacity: f64,
    #[serde(default)]
    pub convection_coefficient: f64,
    #[serde(default)]
    pub area: f64,
}

/// モジュールdoc「縮約実装の理由」参照 — `body_pos_y`/`body_speed`は
/// `bodies[].name`による名前解決、`node_temp`は`thermal.nodes`配列の
/// インデックス(0起点、名前解決を経ない——D9のような単一ノードのシナリオでは
/// 名前付けの手間自体が不要なため)。
#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProbeJson {
    BodyPosY(String),
    /// `body_pos_y`の水平成分版(モジュールdoc「縮約実装の理由」参照 —
    /// D11(振り子)の振れ角再構成のため追加)。
    BodyPosX(String),
    BodySpeed(String),
    NodeTemp(usize),
    /// `astro.bodies`配列のインデックス(0起点、名前解決を経ない——`NodeTemp`と
    /// 同じ理由、D34太陽系儀の軌道半径再構成に使う)。
    AstroPosX(usize),
    AstroPosY(usize),
    /// `astro.bodies`配列の速度成分版(D35軌道投入の「速度も出発点へ戻る」
    /// 判定に使う)。
    AstroVelX(usize),
    AstroVelY(usize),
}

/// `Scenario::joints`の1件。設計の例示JSONには無い項目(モジュールdoc
/// 「縮約実装の理由」参照)——`sim_mechanics::MechanicsSolver`が既に持つ拘束の
/// うち、D11(振り子と時計)が要る`DistanceJoint`(ワールド固定点、または
/// 剛体間の一定長ピン拘束)のみ対応する(`BallJoint`/`SliderJoint`/
/// `HingeMotorPd`は後続増分)。
#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointJson {
    Distance {
        body_a: String,
        #[serde(default)]
        anchor_a: [f64; 3],
        /// 未指定ならワールド固定点への拘束(`sim_mechanics::DistanceJoint::
        /// body_b`の`None`と同じ意味)。
        #[serde(default)]
        body_b: Option<String>,
        #[serde(default)]
        anchor_b: [f64; 3],
        length: f64,
    },
    /// `sim_mechanics::BallJoint`(ワールド座標軸沿いの3本の独立スカラー拘束、
    /// ゼロ距離でも退化しないピン拘束)。**増分G1で追加**——D12(ラグドール)を
    /// シーンJSON化するのに必要だった。`Distance`と同じく`body_b`未指定なら
    /// ワールド固定点への拘束。
    Ball {
        body_a: String,
        #[serde(default)]
        anchor_a: [f64; 3],
        #[serde(default)]
        body_b: Option<String>,
        #[serde(default)]
        anchor_b: [f64; 3],
    },
}

/// `Scenario::couplings`の1件(モジュールdoc「縮約実装の理由」参照 — 現時点で
/// `ImageChargeForce`のみ対応)。
#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CouplingJson {
    ImageChargeForce {
        body: String,
        charge: f64,
        plane_normal: [f64; 3],
        plane_d: f64,
    },
    /// D25(ブラウン運動)向け(`sim_coupling::BrownianForce`)。`thermal_node`は
    /// `thermal.nodes`配列のインデックス(`ProbeJson::NodeTemp`と同じ縮約、
    /// 名前解決を経ない)。
    BrownianForce {
        body: String,
        radius: f64,
        viscosity: f64,
        thermal_node: usize,
        seed: u64,
        stream: u64,
    },
    /// D21(磁石遊び、銅管落下)向け(`sim_coupling::InductionCoupling`)。
    /// `voltage_source_index`は`circuit.voltage_sources`配列のインデックス
    /// (`thermal_node`と同じ縮約)。
    InductionCoupling {
        body: String,
        voltage_source_index: usize,
        length: f64,
        magnetic_field: f64,
        axis: [f64; 3],
    },
    /// 浮力・抗力を**Coupling registry経由で**剛体単位に適用する
    /// (`sim_coupling::BuoyancyDrag`、増分F1で追加)。
    ///
    /// **なぜこの変種を足したか**: 設計 docs/20-integration/01-coupling-matrix.md
    /// §2規則2 の「浮力: 静的水域(集中定数)と解像流体(SPH/格子)は排他」という
    /// 検査を`from_scenario`へ接続したかったが、**排他の相手側をシーンJSONで
    /// 表現する手段が無いと検査は絶対に発火しない**——つまり「接続した」と言い
    /// ながら実質何もしないゲートになってしまう。この変種があって初めて、
    /// `fluids[].static_water`(埋め込み浮力)と本変種を同時に書いたシーンが
    /// 実際に弾かれる。
    ///
    /// **縮約**: `water`は`fluids[].static_water`とは独立にここで指定する
    /// (registry経由の適用は`MechanicsSolver::water`の埋め込み経路とは別物で
    /// あることを明示するため)。`atmosphere`は`world.atmosphere`と同じ形。
    BuoyancyDrag {
        body: String,
        water_level: f64,
        water_density: f64,
    },
}

/// `Scenario::circuit`(モジュールdoc「縮約実装の理由」参照)。
#[derive(Deserialize)]
pub struct CircuitScenarioJson {
    pub num_nodes: usize,
    #[serde(default)]
    pub resistors: Vec<ResistorJson>,
    #[serde(default)]
    pub voltage_sources: Vec<VoltageSourceJson>,
}

/// `CircuitScenarioJson::resistors`の1件。
#[derive(Deserialize)]
pub struct ResistorJson {
    pub a: usize,
    pub b: usize,
    pub resistance: f64,
}

/// `CircuitScenarioJson::voltage_sources`の1件(`sim_em::Circuit::
/// add_voltage_source`の順序どおり登録されるため、`couplings`側の
/// `voltage_source_index`はこの配列のインデックスと一致する)。
#[derive(Deserialize)]
pub struct VoltageSourceJson {
    pub a: usize,
    pub b: usize,
    pub voltage: f64,
}

/// `Scenario::astro`(モジュールdoc「縮約実装の理由」参照)。
#[derive(Deserialize)]
pub struct AstroScenarioJson {
    #[serde(default)]
    pub softening: f64,
    #[serde(default)]
    pub bodies: Vec<AstroBodyJson>,
}

/// `AstroScenarioJson::bodies`の1件(`sim_astro::NBodySystem::add_body`の
/// 縮約表現、`position`配列のインデックス(0起点)が`ProbeJson::AstroPosX`等の
/// 参照先——`mechanics`の`bodies`とは名前空間が別なので名前解決を経ない)。
#[derive(Deserialize)]
pub struct AstroBodyJson {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub mass: f64,
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
            let drag = match (body.drag, &body.shape) {
                (true, ShapeJson::Sphere { radius }) => DragModel::Sphere { radius: *radius },
                _ => DragModel::None,
            };
            let mut desc = RigidBodyDesc::dynamic(shape, material_id);
            desc.drag = drag;
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
            desc.mass_override = body.mass_override;
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
        let (world, _ids) = Self::from_scenario_with_body_ids(scenario)?;
        Ok(world)
    }

    /// `from_scenario`と同じ全ドメイン構築を行い、あわせて`scenario.bodies`と
    /// 同じ順の`BodyId`一覧も返す(**残タスク完遂のシーンギャラリー増分で追加**:
    /// `sim-wasm::WasmWorld::from_scene_json`がHierarchy表示用の`SpawnedBodyMeta`
    /// (ラベル・材質名・基準形状)を構築するには、各ボディの`BodyId`と
    /// `scenario.bodies`のどの要素に対応するかの対応が要るが、`from_scenario`は
    /// この対応を内部で使い捨てていたため公開していなかった。既存の`from_scenario`
    /// はこの関数の薄いラッパーに変更し、振る舞いは変えていない)。
    pub fn from_scenario_with_body_ids(
        scenario: &Scenario,
    ) -> Result<(World, Vec<BodyId>), SceneError> {
        // **排他結合の静的検査(増分F1で接続)**。設計 docs/20-integration/
        // 01-coupling-matrix.md §2規則2 が列挙する3組を、シーンを構築する**前に**
        // 弾く——同じ物理量を二重計上したワールドは、走らせてもエラーにならず
        // エネルギー台帳の残差だけが静かに増えるため、読み込み時点で止めるのが
        // 唯一の実用的な防ぎ方である。
        //
        // **判定の対応付け(縮約と正直な記録)**: `SceneCouplingConfig`の6フラグの
        // うち、シーンJSONから実際に判定できるのは浮力の2つだけである——
        // `fluids[].static_water`があれば静的水域の浮力、`couplings`に
        // `BuoyancyDrag`があれば registry 経由の浮力。
        // 空気抗力とコンデンサ電場エネルギーは、そもそも`CouplingJson`に対応する
        // 変種が無く(現状は`ImageChargeForce`/`BrownianForce`/`InductionCoupling`
        // の3種のみ)、シーンJSONから両方を有効にする手段が存在しないため常に
        // `false`のままになる。**この検査が実際に働くのは浮力の組だけ**であり、
        // 残り2組は対応する`CouplingJson`変種が増えた時点で意味を持ち始める。
        let coupling_config = sim_coupling::SceneCouplingConfig {
            static_water_buoyancy: scenario
                .fluids
                .iter()
                .any(|f| matches!(f, FluidJson::StaticWater { .. })),
            resolved_fluid_buoyancy: scenario
                .couplings
                .iter()
                .any(|c| matches!(c, CouplingJson::BuoyancyDrag { .. })),
            ..Default::default()
        };
        let violations = sim_coupling::validate_exclusive_couplings(&coupling_config);
        if !violations.is_empty() {
            return Err(SceneError::ExclusiveCouplingViolation(violations));
        }

        let options = WorldOptions {
            gravity: scenario.world.gravity,
            dt: scenario.world.dt,
            seed: scenario.seed,
        };
        let mut world = World::new(options);
        if let Some(threshold) = scenario.world.restitution_velocity_threshold {
            world.mechanics_mut().restitution_velocity_threshold = threshold;
        }
        if let Some(atm) = &scenario.world.atmosphere {
            world.mechanics_mut().atmosphere =
                Some(sim_fluid::Atmosphere::still(atm.density, atm.viscosity));
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

        if let Some(thermal_json) = &scenario.thermal {
            let mut solver = sim_thermal::ThermalSolver::new(thermal_json.ambient_temperature);
            for n in &thermal_json.nodes {
                let mut node = sim_thermal::ThermalNode::new(n.temperature, n.heat_capacity);
                node.convection_coefficient = n.convection_coefficient;
                node.area = n.area;
                solver.add_node(node);
            }
            world.enable_thermal(solver);
        }

        if let Some(circuit_json) = &scenario.circuit {
            let mut circuit = sim_em::Circuit::new(circuit_json.num_nodes);
            for r in &circuit_json.resistors {
                circuit.add_resistor(r.a, r.b, r.resistance);
            }
            for v in &circuit_json.voltage_sources {
                circuit.add_voltage_source(v.a, v.b, v.voltage);
            }
            world.enable_circuit(circuit);
        }

        if let Some(astro_json) = &scenario.astro {
            let mut sys = sim_astro::NBodySystem::new(astro_json.softening);
            for b in &astro_json.bodies {
                sys.add_body(array_to_vec3(b.position), array_to_vec3(b.velocity), b.mass);
            }
            world.enable_astro(sys);
        }

        for joint in &scenario.joints {
            match joint {
                JointJson::Distance {
                    body_a,
                    anchor_a,
                    body_b,
                    anchor_b,
                    length,
                } => {
                    let a_id = body_ids_by_name
                        .get(body_a)
                        .ok_or_else(|| SceneError::UnknownBodyName(body_a.clone()))?;
                    let b_index = match body_b {
                        Some(name) => Some(
                            body_ids_by_name
                                .get(name)
                                .ok_or_else(|| SceneError::UnknownBodyName(name.clone()))?
                                .index as usize,
                        ),
                        None => None,
                    };
                    world
                        .mechanics_mut()
                        .add_distance_joint(sim_mechanics::DistanceJoint {
                            body_a: a_id.index as usize,
                            anchor_a: array_to_vec3(*anchor_a),
                            body_b: b_index,
                            anchor_b: array_to_vec3(*anchor_b),
                            length: *length,
                        });
                }
                JointJson::Ball {
                    body_a,
                    anchor_a,
                    body_b,
                    anchor_b,
                } => {
                    let a_id = body_ids_by_name
                        .get(body_a)
                        .ok_or_else(|| SceneError::UnknownBodyName(body_a.clone()))?;
                    let b_index = match body_b {
                        Some(name) => Some(
                            body_ids_by_name
                                .get(name)
                                .ok_or_else(|| SceneError::UnknownBodyName(name.clone()))?
                                .index as usize,
                        ),
                        None => None,
                    };
                    world
                        .mechanics_mut()
                        .add_ball_joint(sim_mechanics::BallJoint {
                            body_a: a_id.index as usize,
                            anchor_a: array_to_vec3(*anchor_a),
                            body_b: b_index,
                            anchor_b: array_to_vec3(*anchor_b),
                            disabled: false,
                        });
                }
            }
        }

        for coupling in &scenario.couplings {
            match coupling {
                CouplingJson::BuoyancyDrag {
                    body,
                    water_level,
                    water_density,
                } => {
                    let id = body_ids_by_name
                        .get(body)
                        .ok_or_else(|| SceneError::UnknownBodyName(body.clone()))?;
                    world.add_coupling(Box::new(sim_coupling::BuoyancyDrag {
                        body_index: id.index as usize,
                        water: Some(sim_fluid::StaticWaterRegion::new(
                            *water_level,
                            *water_density,
                        )),
                        atmosphere: None,
                    }));
                }
                CouplingJson::ImageChargeForce {
                    body,
                    charge,
                    plane_normal,
                    plane_d,
                } => {
                    let id = body_ids_by_name
                        .get(body)
                        .ok_or_else(|| SceneError::UnknownBodyName(body.clone()))?;
                    world.add_coupling(Box::new(sim_coupling::ImageChargeForce {
                        body_index: id.index as usize,
                        charge: *charge,
                        plane_normal: array_to_vec3(*plane_normal),
                        plane_d: *plane_d,
                    }));
                }
                CouplingJson::BrownianForce {
                    body,
                    radius,
                    viscosity,
                    thermal_node,
                    seed,
                    stream,
                } => {
                    let id = body_ids_by_name
                        .get(body)
                        .ok_or_else(|| SceneError::UnknownBodyName(body.clone()))?;
                    world.add_coupling(Box::new(sim_coupling::BrownianForce::new(
                        id.index as usize,
                        *radius,
                        *viscosity,
                        *thermal_node,
                        *seed,
                        *stream,
                    )));
                }
                CouplingJson::InductionCoupling {
                    body,
                    voltage_source_index,
                    length,
                    magnetic_field,
                    axis,
                } => {
                    let id = body_ids_by_name
                        .get(body)
                        .ok_or_else(|| SceneError::UnknownBodyName(body.clone()))?;
                    world.add_coupling(Box::new(sim_coupling::InductionCoupling {
                        body_index: id.index as usize,
                        voltage_source_index: *voltage_source_index,
                        length: *length,
                        magnetic_field: *magnetic_field,
                        axis: array_to_vec3(*axis),
                    }));
                }
            }
        }

        world.add_scenario_probes(scenario, &body_ids_by_name)?;

        Ok((world, ids))
    }

    /// シーンJSONの`probes`セクションを、既に解決済みの名前→`BodyId`対応表を使って
    /// 現在のワールドへ追加する(`from_scenario`から切り出したもの、
    /// `sim-wasm::WasmWorld::import_scene_json`——シーンJSON Import——も同じ解決
    /// ロジックでインポートしたシーンのプローブを再現できるよう共有する)。
    /// `append_scenario_bodies`とは別メソッドにしているのは、`append_scenario_bodies`
    /// のdocに明記したとおりImportが実行中ワールドへの「追加」であり、両者を
    /// 呼ぶかどうかを呼び出し側が選べるようにするため(Importは`fluids`は
    /// 対象外のまま、`probes`のみ対象に含める、という非対称を表現できる)。
    /// 返り値は`scenario.probes`と同じ順のプローブハンドル(`World::probe`用)。
    pub fn add_scenario_probes(
        &mut self,
        scenario: &Scenario,
        body_ids_by_name: &HashMap<String, BodyId>,
    ) -> Result<Vec<usize>, SceneError> {
        let mut handles = Vec::with_capacity(scenario.probes.len());
        for probe in &scenario.probes {
            let target = match probe {
                ProbeJson::BodyPosY(name)
                | ProbeJson::BodyPosX(name)
                | ProbeJson::BodySpeed(name) => {
                    let id = body_ids_by_name
                        .get(name)
                        .ok_or_else(|| SceneError::UnknownBodyName(name.to_string()))?;
                    match probe {
                        ProbeJson::BodyPosY(_) => ProbeTarget::BodyPosY(*id),
                        ProbeJson::BodyPosX(_) => ProbeTarget::BodyPosX(*id),
                        _ => ProbeTarget::BodySpeed(*id),
                    }
                }
                ProbeJson::NodeTemp(index) => ProbeTarget::NodeTemp(*index),
                ProbeJson::AstroPosX(index) => ProbeTarget::AstroPosX(*index),
                ProbeJson::AstroPosY(index) => ProbeTarget::AstroPosY(*index),
                ProbeJson::AstroVelX(index) => ProbeTarget::AstroVelX(*index),
                ProbeJson::AstroVelY(index) => ProbeTarget::AstroVelY(*index),
            };
            handles.push(self.add_probe(target, DEFAULT_PROBE_CAPACITY));
        }
        Ok(handles)
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

    /// `World::add_scenario_probes`(`from_scenario`から切り出した、シーンJSON
    /// Importが取り込んだシーンのプローブも再現できるようにする土台、モジュールdoc
    /// 参照)を、`append_scenario_bodies`と組み合わせて直接呼び、`scenario.probes`と
    /// 同じ順のハンドルで実際に値が読めることを確認する。
    #[test]
    fn add_scenario_probes_registers_probes_matching_scenario_order() {
        let mut world = World::new(WorldOptions::default());
        let json = r#"
        {
          "name": "probe-fragment",
          "world": { "gravity": 9.80665, "dt": 0.008333333 },
          "bodies": [
            { "shape": { "sphere": { "radius": 0.3 } }, "material": "鋼(炭素鋼)",
              "position": [0.0, 5.0, 0.0], "name": "ball" }
          ],
          "probes": [ { "body_pos_y": "ball" }, { "body_speed": "ball" } ]
        }
        "#;
        let scenario = Scenario::from_json(json).unwrap();
        let ids = world.append_scenario_bodies(&scenario).unwrap();

        let mut body_ids_by_name: HashMap<String, BodyId> = HashMap::new();
        for (body, id) in scenario.bodies.iter().zip(ids.iter()) {
            if let Some(name) = &body.name {
                body_ids_by_name.insert(name.clone(), *id);
            }
        }
        let handles = world
            .add_scenario_probes(&scenario, &body_ids_by_name)
            .expect("valid probe references");

        assert_eq!(handles.len(), 2);
        world.step();
        let pos_y_history: Vec<f64> = world
            .probe(handles[0])
            .unwrap()
            .history()
            .copied()
            .collect();
        let speed_history: Vec<f64> = world
            .probe(handles[1])
            .unwrap()
            .history()
            .copied()
            .collect();
        assert_eq!(pos_y_history.len(), 1);
        assert_eq!(speed_history.len(), 1);
        assert!(
            pos_y_history[0] < 5.0,
            "ball should have fallen after 1 step"
        );
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

    /// **増分G1**: D12(ラグドール)をシーンJSON経由で検証する。`JointJson::Ball`
    /// (本増分で追加)が実際に効いていることの確認でもある——`BallJoint`が無ければ
    /// 4つの箱はバラバラに落ちるので、頭と胴体の距離が保たれることが拘束が働いて
    /// いる直接の証拠になる。合格基準(docs/20-integration/03-entity-layer.md §7
    /// 「ラグドール落下: エネルギー単調減少・貫入なし」)のうち貫入なしを見る。
    #[test]
    fn run_headless_scenario_ragdoll_stays_connected_and_does_not_penetrate_the_floor() {
        let json = include_str!("../../../scenes/d12-ragdoll.json");
        let steps = 600; // 5秒。落下(3m)して静止するのに十分。
        let result = run_headless_scenario(json, steps).expect("valid scenario JSON");
        let torso_y = &result.probe_histories[0];
        let head_y = &result.probe_histories[1];
        let torso_speed = &result.probe_histories[2];

        let torso_x = &result.probe_histories[3];
        let head_x = &result.probe_histories[4];

        // **床への貫入が無いこと**。ここで注意が要るのは、胴体の半寸法が
        // `[0.3, 0.5, 0.15]` で**等方でない**点である。ラグドールは落下中に倒れ、
        // 最終的に一番広い面(0.3×0.5)で床に伏せるため、重心の静止高さは
        // 半高さ0.5ではなく**最小半寸法の0.15**になる(実測 0.1496)。
        // 「0.5を下回ったら貫通」と書くとこの正しい挙動を貫通と誤判定するので、
        // 判定の下限は最小半寸法を使う。
        const TORSO_MIN_HALF_EXTENT: f64 = 0.15;
        const FLOOR_PENETRATION_SLOP: f64 = 0.05; // `demos.rs`のD12テストと同じ許容。
        let min_torso = torso_y.iter().copied().fold(f64::INFINITY, f64::min);
        assert!(
            min_torso > TORSO_MIN_HALF_EXTENT - FLOOR_PENETRATION_SLOP,
            "胴体が床を貫通した: min_torso_y={min_torso}"
        );

        // **BallJointが効いていること**。頭は胴体の上端(+0.5)に半高さ0.2で
        // 繋がるので、両重心の3D距離は拘束により常に 0.7 に保たれる。
        // プローブは x と y しか取れない(`ProbeTarget` に Z が無い)が、
        // **3D距離の xy 平面への射影は元の距離を超えられない**ので、
        // 「射影距離が 0.7 を超えない」は拘束が破れていないことの厳密な必要条件になる。
        // 実測の最大値は 0.7000000000000004(浮動小数の丸め以内で上限に張り付く)。
        //
        // 逆向き(拘束が消えていないこと)は射影では見られない——胴体が倒れると
        // 頭は z 方向へ回るため、射影距離は最終的に 0.084 まで縮む。そこで
        // 3D距離の検証は同じJSONからワールドを直接組んで下で行う。
        for (i, ((tx, hx), (ty, hy))) in torso_x
            .iter()
            .zip(head_x)
            .zip(torso_y.iter().zip(head_y))
            .enumerate()
        {
            let projected = ((hx - tx).powi(2) + (hy - ty).powi(2)).sqrt();
            assert!(
                projected < 0.7 + 1e-9,
                "BallJointの拘束距離0.7を射影距離が超えた(拘束が破れている): step={i} projected={projected}"
            );
        }

        // 3D距離が全区間で 0.7 に保たれること。**これが `JointJson::Ball` が
        // 実際に拘束を張っている直接の証拠**——ジョイントが無ければ頭と胴体は
        // 独立に落ち、それぞれの静止高さ(0.2 と 0.15)へ落ち着いて距離は 0.05 になる。
        let scenario = Scenario::from_json(json).expect("valid scenario JSON");
        let (mut world, ids) = World::from_scenario_with_body_ids(&scenario).expect("valid world");
        let (torso, head) = (ids[1], ids[2]);
        let distance = |w: &World| {
            let p = &w.mechanics().bodies.position;
            (p[head.index as usize] - p[torso.index as usize]).length()
        };
        for i in 0..steps {
            world.step();
            let d = distance(&world);
            assert!(
                (d - 0.7).abs() < 0.02,
                "BallJointは頭と胴体の距離を0.7に保つべき: step={i} distance={d}"
            );
        }

        // 最終的にほぼ静止する(散逸してエネルギーを失う)。実測 8.1e-4。
        let final_speed = *torso_speed.last().expect("履歴が空でない");
        assert!(
            final_speed < 0.5,
            "5秒後にはほぼ静止しているべき: final_speed={final_speed}"
        );
    }

    /// **増分F1: 排他結合の静的検査が実際に発火すること**。設計
    /// docs/20-integration/01-coupling-matrix.md §2規則2「浮力: 静的水域
    /// (集中定数)と解像流体は排他」を、シーン読み込みの時点で弾く。
    ///
    /// **発火しない検査は無意味**なので、①両方書いたシーンが実際に`Err`になる
    /// ②片方だけなら通る、の両方を確認する(片方だけで落ちるなら過検出になる)。
    #[test]
    fn from_scenario_rejects_a_scene_that_double_counts_buoyancy() {
        let both = r#"
        {
          "name": "buoyancy-double-count",
          "world": { "gravity": 9.80665, "dt": 0.008333333 },
          "bodies": [
            { "shape": { "box": { "half": [0.5, 0.5, 0.5] } }, "material": "木材(松)",
              "position": [0, 2, 0], "name": "box" }
          ],
          "fluids": [ { "static_water": { "water_level": 0.0, "density": 998.2 } } ],
          "couplings": [
            { "buoyancy_drag": { "body": "box", "water_level": 0.0, "water_density": 998.2 } }
          ]
        }
        "#;
        let scenario = Scenario::from_json(both).expect("JSON自体は妥当");
        match World::from_scenario(&scenario) {
            Err(SceneError::ExclusiveCouplingViolation(v)) => {
                assert!(
                    v.contains(&sim_coupling::ExclusiveCouplingViolation::BuoyancyDoubleCounted),
                    "浮力の二重計上として検出されるべき: {v:?}"
                );
            }
            Err(other) => panic!("排他結合違反で弾かれるべきだが別のエラー: {other:?}"),
            Ok(_) => panic!("排他結合違反のシーンが通ってしまった"),
        }

        // 片方だけなら通る(過検出でないことの確認)。
        let only_static = both.replace(
            r#""couplings": [
            { "buoyancy_drag": { "body": "box", "water_level": 0.0, "water_density": 998.2 } }
          ]"#,
            r#""couplings": []"#,
        );
        let scenario = Scenario::from_json(&only_static).expect("JSON自体は妥当");
        assert!(
            World::from_scenario(&scenario).is_ok(),
            "静的水域だけなら二重計上ではない"
        );

        let only_coupling = both.replace(
            r#""fluids": [ { "static_water": { "water_level": 0.0, "density": 998.2 } } ],"#,
            "",
        );
        let scenario = Scenario::from_json(&only_coupling).expect("JSON自体は妥当");
        assert!(
            World::from_scenario(&scenario).is_ok(),
            "registry経由の浮力だけなら二重計上ではない"
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
        // シーンJSONは`scenes/d4-box-stack.json`として出荷する(残タスク完遂の
        // シーンギャラリー増分、`scenes/index.json`マニフェスト参照)。テストと
        // 出荷アセットを同一ファイルにすることで、アセットが壊れれば直ちにこの
        // テストがRedになる。
        let json = include_str!("../../../scenes/d4-box-stack.json");

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
        // シーンJSONは`scenes/d6-floating-box-f4.json`として出荷する(残タスク完遂の
        // シーンギャラリー増分、`scenes/index.json`マニフェスト参照)。密度・釣り合い
        // 喫水位置(下記`equilibrium_y`と同じ計算式で導出)はJSON側に焼き込み済み——
        // このRust側の再計算は期待値(解析解)としてアサーションに使う。
        let ratio = 0.6;
        let half = 0.5;
        let side = 2.0 * half;
        let h_sub = ratio * side;
        let equilibrium_y = -h_sub + half;

        let json = include_str!("../../../scenes/d6-floating-box-f4.json");

        let steps = 600; // 5秒(既定dt)
        let result = run_headless_scenario(json, steps).expect("valid scenario JSON");

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

    /// D6浮き沈みのF5部分(平衡点から変位させると解析解の周期で振動する、
    /// `demos.rs`の同テストのF5部分)を8本目の適用例として実装する。F4と同じ
    /// `materials[].extends`密度派生+`fluids`静水面構成を流用し、箱を平衡位置より
    /// `amplitude`だけ高い位置に置く。ネイティブ側は`body_velocity`(符号付き
    /// y速度)の下降方向ゼロ交差で1周期を判定するが、シーンJSONの`ProbeJson`には
    /// 符号付き速度を読める種別が無いため、`body_pos_y`のみから「最初の谷(底)を
    /// 過ぎた後の次の山(頂点)」を検出する位置ベースの判定に置き換えた(単振動
    /// なので谷から次の山までの時間も1周期に等しい)。
    #[test]
    fn run_headless_scenario_floating_box_oscillates_at_the_f5_analytic_period() {
        let water_density: f64 = 998.2;
        let ratio: f64 = 0.5;
        let half: f64 = 0.5;
        let side = 2.0 * half;
        let dt: f64 = 0.008333333;

        // シーンJSONは`scenes/d6-floating-box-f5.json`として出荷する(残タスク完遂の
        // シーンギャラリー増分B2)。派生密度(=ratio*water_density=499.1)・平衡位置
        // からamplitude(0.1)だけ高い初期位置(=-(ratio*side)+half+amplitude=0.1)は
        // 事前計算した値をそのまま焼き込んである(Rustの`{}`は往復可能な最短表現を
        // 出すため、下の期待値計算と1ビットも変わらない)。
        let json = include_str!("../../../scenes/d6-floating-box-f5.json");

        let steps = 400; // ネイティブ側と同じ既定dt換算での歩数。
        let result = run_headless_scenario(json, steps).expect("valid scenario JSON");
        let pos_y = &result.probe_histories[0];

        // 谷(最初にheightが上昇へ転じる点)。
        let trough_step = (1..pos_y.len())
            .find(|&i| pos_y[i] > pos_y[i - 1])
            .expect("box should start descending then bounce back up");
        // 谷の後、次に下降へ転じる直前の点(=次の山、1周期後)。
        let peak_step = (trough_step + 1..pos_y.len())
            .find(|&i| pos_y[i] < pos_y[i - 1])
            .map(|i| i - 1)
            .expect("box should reach a subsequent peak within the simulated window");

        let measured_period = (peak_step + 1) as f64 * dt;
        let mass = ratio * water_density * side * side * side;
        let k = water_density * 9.80665 * side * side;
        let analytic_period = 2.0 * std::f64::consts::PI * (mass / k).sqrt();
        let rel_err = (measured_period - analytic_period).abs() / analytic_period;
        assert!(
            rel_err < 0.05,
            "F5: measured_period={measured_period} analytic_period={analytic_period} \
             rel_err={rel_err:.4}"
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
        // シーンJSONは`scenes/d5-incline-static.json`として出荷する(残タスク完遂の
        // シーンギャラリー増分、`scenes/index.json`マニフェスト参照)。10°の傾いた
        // 平面(`Plane`)+それに合わせて回転させた箱の座標・クォータニオンは
        // 事前に計算した値をそのまま焼き込んである(計算式は本テストのgit履歴、
        // または`docs/21-verification/03-demo-scenarios.md`のD5参照)。
        let json = include_str!("../../../scenes/d5-incline-static.json");

        let steps = 600; // 5秒(既定dt)
        let result = run_headless_scenario(json, steps).expect("valid scenario JSON");

        let final_speed = *result.probe_histories[0]
            .last()
            .expect("history should not be empty");
        assert!(
            final_speed < 1e-4,
            "M7: box on a 10° incline (below the friction angle) should stay at rest: \
             final_speed={final_speed}"
        );
    }

    /// D5(斜面、続き): 静止摩擦角を超える(45°)と`M8`の解析解どおりの加速度で滑り出す
    /// ことを確認する(`demos.rs`の`d5_incline_stays_static_below_friction_angle_and_
    /// slides_matching_formula_above`のM8部分と同じ構成をシーンJSONで再現)。
    /// `ProbeTarget`には斜面下り方向の符号付き速度成分を直接読める種別が無いが、
    /// 初期速度ゼロ・XY平面内のみの運動(重力はY軸負方向、斜面法線はXY平面内)であれば
    /// 速度は常に下り方向の単一成分のみを持つため、`body_speed`(速さ)がそのまま
    /// 下り方向速度に一致する。
    #[test]
    fn run_headless_scenario_slides_down_an_incline_above_the_friction_angle_matching_m8() {
        let theta: f64 = 45.0_f64.to_radians();

        // シーンJSONは`scenes/d5-incline-slide.json`として出荷する(残タスク完遂の
        // シーンギャラリー増分B2)。45°の傾いた平面(`Plane`)+それに合わせて回転
        // させた箱の法線・座標・クォータニオンは事前に計算した値をそのまま
        // 焼き込んである(計算式は本テストのgit履歴参照。Rustの`{}`は往復可能な
        // 最短表現を出すため、下の期待値計算と1ビットも変わらない)。
        let json = include_str!("../../../scenes/d5-incline-slide.json");

        let steps = 60; // 0.5秒(既定dt) — demos.rsのM8アサーションと同じ経過時間
        let dt: f64 = 0.008333333;
        let result = run_headless_scenario(json, steps).expect("valid scenario JSON");

        let measured_speed = *result.probe_histories[0]
            .last()
            .expect("history should not be empty");
        let elapsed = steps as f64 * dt;
        let measured_accel = measured_speed / elapsed;

        let world = World::new(WorldOptions::default());
        let steel = world.materials().find_by_name("鋼(炭素鋼)").unwrap();
        let steel_friction = world.materials().friction_pair(steel, steel);
        let expected_accel = 9.80665 * (theta.sin() - steel_friction * theta.cos());
        let rel_err = (measured_accel - expected_accel).abs() / expected_accel;
        assert!(
            rel_err < 0.05,
            "M8: box on a 45° incline (above the friction angle) should slide with \
             a=g(sinθ-μkcosθ): measured_accel={measured_accel} expected_accel={expected_accel} \
             rel_err={rel_err:.4}"
        );
    }

    /// D7(風と終端速度): `demos.rs`の`d7_wind_and_terminal_velocity_matches_high_and_
    /// low_reynolds_formulas`と同じ構成(F1高Re・F3低Re、いずれも`sim_fluid::
    /// drag_force_sphere`がレイノルズ数から自動選択するため同じ物理式)をシーンJSON
    /// (`WorldScenarioOptions::atmosphere`+`BodyScenarioDesc::drag`、本増分で追加した
    /// スキーマ拡張)経由で再現する。F2(雨粒の実測値との比較)はF1と同じ物理を別
    /// パラメータで示すのみのため`demos.rs`同様対象外。
    #[test]
    fn run_headless_scenario_wind_and_terminal_velocity_matches_high_and_low_reynolds_formulas() {
        // F1: 高Re(鋼球、Cd=0.47相当の二次抗力)。
        // シーンJSONは`scenes/d7-wind-high-re.json`として出荷する(残タスク完遂の
        // シーンギャラリー増分B2。F1/F3は大気パラメータと球半径が異なり、収束する
        // 終端速度・到達までの時間スケールも大きく異なる別ドメインの見え方をする
        // ため、ギャラリーでは2ファイルに分けた——1ファイルにまとめると片方の挙動
        // しか目視できない)。
        {
            let radius: f64 = 0.005;
            let json = include_str!("../../../scenes/d7-wind-high-re.json");

            let steps = 3600; // 30秒(既定dt)
            let result = run_headless_scenario(json, steps).expect("valid scenario JSON");
            let measured = *result.probe_histories[0]
                .last()
                .expect("history should not be empty");

            let mass = 7850.0 * (4.0 / 3.0) * std::f64::consts::PI * radius.powi(3);
            let area = std::f64::consts::PI * radius * radius;
            let cd = 0.47;
            let atmosphere_density = 1.225;
            let analytic_vt = (2.0 * mass * 9.80665 / (atmosphere_density * cd * area)).sqrt();
            let rel_err = (measured - analytic_vt).abs() / analytic_vt;
            assert!(
                rel_err < 0.01,
                "F1: measured={measured} analytic_vt={analytic_vt} rel_err={rel_err:.4}"
            );
        }

        // F3: 低Re(ストークス沈降、v=2r²Δρg/(9μ))。
        // シーンJSONは`scenes/d7-wind-low-re.json`として出荷する(残タスク完遂の
        // シーンギャラリー増分B2、上記F1と分けた理由は同所参照)。
        {
            let radius: f64 = 0.01;
            let fluid_density: f64 = 0.5;
            let viscosity: f64 = 1.0;
            let json = include_str!("../../../scenes/d7-wind-low-re.json");

            let steps = 240; // 2秒(既定dt)
            let result = run_headless_scenario(json, steps).expect("valid scenario JSON");
            let measured = *result.probe_histories[0]
                .last()
                .expect("history should not be empty");

            let steel_density = 7850.0;
            let delta_rho = steel_density - fluid_density;
            let analytic = 2.0 * radius * radius * delta_rho * 9.80665 / (9.0 * viscosity);
            let rel_err = (measured - analytic).abs() / analytic;
            assert!(
                rel_err < 0.02,
                "F3: measured={measured} analytic={analytic} rel_err={rel_err:.4}"
            );
        }
    }

    /// D9(冷めるコーヒー): `demos.rs`の`d9_cooling_coffee_matches_newton_cooling_
    /// exponential_decay`と同じ構成(単一の熱ノード、対流のみ)をシーンJSON
    /// (`Scenario::thermal`+`ProbeJson::NodeTemp`、本増分で追加したスキーマ拡張)
    /// 経由で再現し、ニュートン冷却の指数減衰$T=T_{env}+(T_0-T_{env})e^{-t/\tau}$
    /// ($\tau=C/(hA)$)と一致することを確認する。剛体は登場しない(`bodies`は
    /// 空配列)ため`Scenario::bodies`の`#[serde(default)]`をそのまま使う。
    ///
    /// シーンJSONは`scenes/d9-cooling-coffee.json`として出荷する(残タスク完遂の
    /// シーンギャラリー増分B3、`scenes/index.json`マニフェスト参照)。全値が
    /// リテラルなので焼き込みに計算は不要——このRust側の`ambient`/`c`/`h`/`area`/
    /// `t0`/`dt`はJSONに焼き込んだ値と同じものを、解析解(ニュートン冷却の
    /// 指数減衰)の計算に使う。
    #[test]
    fn run_headless_scenario_cooling_coffee_matches_newton_cooling_exponential_decay() {
        let ambient: f64 = 293.15;
        let c: f64 = 100.0;
        let h: f64 = 10.0;
        let area: f64 = 1.0;
        let t0: f64 = 350.0; // 約77°C(熱いコーヒー相当)
        let dt: f64 = 0.008333333;
        let tau = c / (h * area);
        let steps = (2.0 * tau / dt) as u32;

        let json = include_str!("../../../scenes/d9-cooling-coffee.json");

        let result = run_headless_scenario(json, steps).expect("valid scenario JSON");
        let measured = *result.probe_histories[0]
            .last()
            .expect("history should not be empty");

        let t_elapsed = steps as f64 * dt;
        let analytic = ambient + (t0 - ambient) * (-t_elapsed / tau).exp();
        let rel_err = (measured - analytic).abs() / (t0 - ambient);
        assert!(
            rel_err < 0.01,
            "T1: measured={measured} analytic={analytic} rel_err={rel_err:.4}"
        );
    }

    /// D11(振り子と時計、M3小振幅周期部分のみ): `demos.rs`の
    /// `d11_pendulum_matches_small_amplitude_period_and_double_pendulum_replay_is_
    /// deterministic`のM3部分(単振り子の小振幅周期)を、`Scenario::joints`
    /// (`DistanceJoint`、本増分で追加したスキーマ拡張)+`body_pos_x`/`body_pos_y`
    /// の2プローブ(`body_pos_x`も本増分で追加、振れ角の再構成に必要)経由で再現する。
    /// 二重振り子のリプレイ決定論部分は`Scenario`に無関係な純粋な`World`直接操作の
    /// 検証のため対象外(`demos.rs`側で既にGreen)。
    #[test]
    fn run_headless_scenario_pendulum_matches_small_amplitude_period() {
        // シーンJSONは`scenes/d11-pendulum.json`として出荷する(残タスク完遂の
        // シーンギャラリー増分、`scenes/index.json`マニフェスト参照)。小振幅
        // (θ0=0.05 rad)の初期位置はJSON側に焼き込み済み——このRust側の`length`/
        // `dt`は期待周期(解析解)の計算に使う。
        // `demos.rs`側はdt=1/2000だが、そのままだと1周期分のstep数(約4800)が
        // プローブのリングバッファ容量(`DEFAULT_PROBE_CAPACITY`=600)を超え、
        // ゼロ交差走査に必要な先頭付近のサンプルが上書きされてしまう。
        // 既定dt(1/120)なら1周期あたり約240stepで容量内に収まる。
        let length: f64 = 1.0;
        let theta0: f64 = 0.05; // 小振幅(rad)、JSON側にも同じ値が焼き込まれている。
        let dt: f64 = 0.008333333;
        let pivot_x = 0.0;
        let pivot_y = 0.0;
        let bob_x = theta0.sin() * length;
        let bob_y = -theta0.cos() * length;

        let json = include_str!("../../../scenes/d11-pendulum.json");

        let analytic_period = 2.0 * std::f64::consts::PI * (length / 9.80665_f64).sqrt();
        let steps = (1.2 * analytic_period / dt) as u32;
        let result = run_headless_scenario(json, steps).expect("valid scenario JSON");
        let xs = &result.probe_histories[0];
        let ys = &result.probe_histories[1];

        let angle = |x: f64, y: f64| -> f64 { (x - pivot_x).atan2(pivot_y - y) };
        let mut prev_angle = angle(bob_x, bob_y);
        let mut prev_t = 0.0;
        let mut crossings = Vec::new();
        for (step, (&x, &y)) in xs.iter().zip(ys.iter()).enumerate() {
            let t = (step + 1) as f64 * dt;
            let a = angle(x, y);
            if prev_angle.signum() != a.signum() && prev_angle != 0.0 {
                let frac = -prev_angle / (a - prev_angle);
                crossings.push(prev_t + frac * (t - prev_t));
                if crossings.len() >= 2 {
                    break;
                }
            }
            prev_angle = a;
            prev_t = t;
        }
        assert!(crossings.len() >= 2, "should observe two zero crossings");
        let measured_period = 2.0 * (crossings[1] - crossings[0]);
        let rel_err = (measured_period - analytic_period).abs() / analytic_period;
        assert!(
            rel_err < 0.01,
            "M3: measured_period={measured_period} analytic_period={analytic_period} rel_err={rel_err:.4}"
        );
    }

    /// D26(帯電風船): `demos.rs`の`d26_charged_balloon_sticks_to_wall_via_image_
    /// charge_force_matching_inverse_square_law`と同じ2構成(定性: 鏡像力のみで
    /// 壁へ到達する。逆二乗則: 初期距離2倍で初期加速度1/4)を`Scenario::couplings`
    /// (`CouplingJson::ImageChargeForce`、本増分で追加したスキーマ拡張)経由で
    /// 再現する。
    #[test]
    fn run_headless_scenario_charged_balloon_sticks_to_wall_matching_inverse_square_law() {
        let charge = 1.0e-7; // 摩擦帯電した風船オーダー

        // 定性: 壁から離れた位置で静止させた帯電風船が鏡像力のみで壁(x=0)へ到達する。
        // シーンJSONは`scenes/d26-balloon-qualitative.json`として出荷する(残タスク
        // 完遂のシーンギャラリー増分B2)。逆二乗則側(下の`initial_acceleration_at`)
        // は1step目の速度のみを見る数値検定でしかなく、初期距離0.1と0.2のどちらも
        // Scene Viewで動かしても「風船が壁へ寄っていく」という見た目上は定性側と
        // 区別が付かない(動きを目視する意味のある別デモにならない)ため、
        // ギャラリーへは定性側の1ファイルのみを切り出した。
        {
            let json = include_str!("../../../scenes/d26-balloon-qualitative.json");
            let result = run_headless_scenario(json, 6000).expect("valid scenario JSON");
            let final_x = *result.probe_histories[0]
                .last()
                .expect("history should not be empty");
            assert!(
                final_x <= 0.03,
                "D26 pass criterion (image charge qualitative): charged balloon should be \
                 pulled to the wall by the image charge force: final_x={final_x}"
            );
        }

        // 逆二乗則: 初期距離を2倍にすると初期加速度(=1step目の速度変化/dt)は1/4になる。
        let initial_acceleration_at = |initial_x: f64| -> f64 {
            let dt: f64 = 0.008333333;
            let json = format!(
                r#"
            {{
              "name": "d26-balloon-inverse-square",
              "world": {{ "gravity": 0.0, "dt": {dt} }},
              "bodies": [
                {{ "shape": {{ "sphere": {{ "radius": 0.01 }} }},
                  "material": "木材(松)",
                  "position": [{initial_x}, 0, 0], "name": "balloon" }}
              ],
              "couplings": [
                {{ "image_charge_force": {{ "body": "balloon", "charge": {charge},
                  "plane_normal": [1, 0, 0], "plane_d": 0 }} }}
              ],
              "probes": [ {{ "body_speed": "balloon" }} ]
            }}
            "#
            );
            let result = run_headless_scenario(&json, 1).expect("valid scenario JSON");
            let speed = *result.probe_histories[0]
                .last()
                .expect("history should not be empty");
            speed / dt
        };

        let a_near = initial_acceleration_at(0.1);
        let a_far = initial_acceleration_at(0.2);
        let ratio = a_near / a_far;
        let rel_err = (ratio - 4.0).abs() / 4.0;
        assert!(
            rel_err < 1e-6,
            "D26 pass criterion (inverse square): doubling distance should quarter the \
             initial acceleration: a_near={a_near} a_far={a_far} ratio={ratio}"
        );
    }

    /// D25(ブラウン運動): `demos.rs`の
    /// `d25_brownian_motion_ensemble_mean_squared_displacement_matches_6dt`と同じ
    /// 物理パラメータ(1μmポリスチレン球相当、水の粘性、`sim_coupling::
    /// BrownianForce`、本増分で追加した`CouplingJson::BrownianForce`スキーマ拡張)
    /// を使うが、以下2点を縮約する: (1)アンサンブルサイズをN=2000→300へ縮小
    /// (シーンJSON文字列を`format!`で毎粒子分生成するコスト・パース時間を抑える
    /// ため、統計誤差の増加分は許容誤差を広げて吸収する)。(2)3D MSD($6Dt$、
    /// x/y/z全成分)ではなくx軸1成分のみのMSD($\langle\Delta x^2\rangle=2Dt$、
    /// 等方性より3D MSDの1/3)を検証する——`BodyPosZ`プローブが無いため
    /// (`BodyPosX`はD11向けに追加済み)。ヘッドレスランナーは「起点」と「終点」の
    /// 2時点の位置差分を直接読むAPIを持たない(`probe_histories`は最終値のみ
    /// 実用的に使える)ため、同一の決定論的シード付きシナリオを異なる`steps`
    /// (ウォームアップ終了時点/測定終了時点)で2回実行し、それぞれの`body_pos_x`
    /// プローブの最終値を起点/終点として扱う(同一シードなので共通のstep区間の
    /// 軌道は厳密に一致する、決定論性を利用したテクニック)。
    #[test]
    fn run_headless_scenario_brownian_motion_ensemble_mean_squared_displacement_matches_2dt() {
        let water_like_density = 1050.0; // ポリスチレン球相当
        let radius: f64 = 1.0e-6;
        let volume = 4.0 / 3.0 * std::f64::consts::PI * radius.powi(3);
        let mass = water_like_density * volume;
        let viscosity: f64 = 1.002e-3; // 水の粘性(20℃)
        let temperature: f64 = 293.15;

        let gamma = 6.0 * std::f64::consts::PI * viscosity * radius;
        let tau = mass / gamma;
        let dt = tau / 50.0;
        let warmup_steps = (10.0 * tau / dt) as u32;
        let measurement_steps = (50.0 * tau / dt) as u32;
        let n_particles: usize = 300;

        let mut bodies_json = String::new();
        let mut couplings_json = String::new();
        let mut probes_json = String::new();
        for i in 0..n_particles {
            if i > 0 {
                bodies_json.push(',');
                couplings_json.push(',');
                probes_json.push(',');
            }
            let x = i as f64 * 1.0e-3;
            bodies_json.push_str(&format!(
                r#"{{ "shape": {{ "sphere": {{ "radius": {radius} }} }},
                  "material": "鋼(炭素鋼)", "mass_override": {mass},
                  "position": [{x}, 0, 0], "name": "p{i}" }}"#
            ));
            couplings_json.push_str(&format!(
                r#"{{ "brownian_force": {{ "body": "p{i}", "radius": {radius},
                  "viscosity": {viscosity}, "thermal_node": 0, "seed": 1, "stream": {i} }} }}"#
            ));
            probes_json.push_str(&format!(r#"{{ "body_pos_x": "p{i}" }}"#));
        }

        let json = format!(
            r#"
        {{
          "name": "d25-brownian",
          "world": {{ "gravity": 0.0, "dt": {dt} }},
          "thermal": {{ "ambient_temperature": {temperature},
            "nodes": [ {{ "temperature": {temperature}, "heat_capacity": 1000.0 }} ] }},
          "bodies": [{bodies_json}],
          "couplings": [{couplings_json}],
          "probes": [{probes_json}]
        }}
        "#
        );

        let origin_result =
            run_headless_scenario(&json, warmup_steps).expect("valid scenario JSON");
        let final_result = run_headless_scenario(&json, warmup_steps + measurement_steps)
            .expect("valid scenario JSON");

        let t = measurement_steps as f64 * dt;
        let msd: f64 = (0..n_particles)
            .map(|i| {
                let origin_x = *origin_result.probe_histories[i]
                    .last()
                    .expect("history should not be empty");
                let final_x = *final_result.probe_histories[i]
                    .last()
                    .expect("history should not be empty");
                (final_x - origin_x).powi(2)
            })
            .sum::<f64>()
            / n_particles as f64;

        let diffusion_coefficient = 1.380649e-23 * temperature / gamma;
        let expected = 2.0 * diffusion_coefficient * t; // 1軸成分のみ
        let rel_err = (msd - expected).abs() / expected;
        assert!(
            rel_err < 0.3,
            "D25 pass criterion (S4 MSD, 1-axis component, N={n_particles}): \
             msd={msd:e} expected={expected:e} rel_err={rel_err:.4}"
        );
    }

    /// D21(磁石遊び、銅管落下): `demos.rs`の
    /// `d21_magnet_play_copper_tube_drop_reaches_analytic_terminal_velocity`と
    /// 同じ構成(渦電流ブレーキ、`Circuit`+`InductionCoupling`、本増分で追加した
    /// `Scenario::circuit`+`CouplingJson::InductionCoupling`スキーマ拡張)を
    /// シーンJSON経由で再現し、終端速度が解析解$v_{term}=mgR/(B\ell)^2$と
    /// rel<0.02一致することを確認する。
    #[test]
    fn run_headless_scenario_copper_tube_drop_reaches_analytic_terminal_velocity() {
        let mass: f64 = 0.01;
        let length: f64 = 0.1;
        let b: f64 = 0.5;
        let r: f64 = 1.0;
        let gravity: f64 = 9.80665;
        let dt: f64 = 0.001;
        let tau = mass * r / (b * length).powi(2);
        let steps = (5.0 * tau / dt) as u32;

        // シーンJSONは`scenes/d21-copper-tube-drop.json`として出荷する(残タスク
        // 完遂のシーンギャラリー増分B2)。全値がリテラルなので焼き込みに計算は
        // 不要。
        let json = include_str!("../../../scenes/d21-copper-tube-drop.json");

        let result = run_headless_scenario(json, steps).expect("valid scenario JSON");
        let measured_v = *result.probe_histories[0]
            .last()
            .expect("history should not be empty");

        let expected_v_term = mass * gravity * r / (b * length).powi(2);
        let rel_err = (measured_v - expected_v_term).abs() / expected_v_term;
        assert!(
            rel_err < 0.02,
            "D21 pass criterion (eddy current terminal velocity): measured_v={measured_v} \
             expected_v_term={expected_v_term} rel_err={rel_err:.4}"
        );
    }

    /// D34(太陽系儀、A1軌道半径保存のみ): `demos.rs`の
    /// `d34_solar_system_single_planet_matches_keplers_third_law_and_conserves_
    /// energy_and_angular_momentum`のA1部分(20周回後も円軌道半径が保たれる)を
    /// `Scenario::astro`(本増分で追加したスキーマ拡張、`sim_astro::NBodySystem`)
    /// +`AstroPosX`/`AstroPosY`プローブ経由で再現する。A2(エネルギー・角運動量
    /// 保存)は検証しない——`ProbeTarget`が位置成分のみ対応で速度を読めないため
    /// 角運動量($L=r\times v$)を再構成できない(`demos.rs`側で既にGreen)。
    ///
    /// シーンJSONは`scenes/d34-solar-system-single-planet.json`として出荷する
    /// (残タスク完遂のシーンギャラリー増分B3)。JSON側の`world.dt`・初期速度
    /// (円軌道速度$v_{circ}=\sqrt{GM_{sun}/r}$)は、`period/steps_per_orbit`
    /// (`period`は下記と同じケプラー第3法則の式)を計算した結果をそのまま
    /// 焼き込んだリテラル値——このテスト自体は`steps_per_orbit`/`orbits`から
    /// 総step数を、`r`から解析解(円軌道半径)を計算するだけで、`dt`/`v_circ`の
    /// 計算式自体は不要になったため削除した(B2で確立した「解析解の期待値を
    /// 計算するロジックはRust側に残す」規律どおり——ここでの期待値は`final_r`と
    /// 比較する`r`のみで、初期条件の`v_circ`は期待値ではない)。
    #[test]
    fn run_headless_scenario_solar_system_single_planet_preserves_circular_orbit_radius() {
        let r: f64 = 1.496e11; // 1 AU相当(JSON側の`astro.bodies[1].position`と同じ値)。
        let steps_per_orbit = 1000u32;
        let orbits = 20u32;

        let json = include_str!("../../../scenes/d34-solar-system-single-planet.json");

        let steps = steps_per_orbit * orbits;
        let result = run_headless_scenario(json, steps).expect("valid scenario JSON");
        let final_x = *result.probe_histories[0]
            .last()
            .expect("history should not be empty");
        let final_y = *result.probe_histories[1]
            .last()
            .expect("history should not be empty");
        let final_r = (final_x * final_x + final_y * final_y).sqrt();

        let rel_r_err = (final_r - r).abs() / r;
        assert!(
            rel_r_err < 0.01,
            "A1: circular orbit radius should be preserved: final_r={final_r} r={r} \
             rel_err={rel_r_err:.4}"
        );
    }

    /// D35(軌道投入): `demos.rs`の
    /// `d35_orbital_insertion_elliptical_period_matches_keplers_third_law`と
    /// 同じ構成(円軌道速度の0.9倍の初速で楕円軌道を作り、vis-vivaから導いた
    /// 長半径によるケプラー第3法則の周期分だけ進めると出発点(位置・速度とも)
    /// 付近に戻ることを確認)を、D34向けに追加した`Scenario::astro`+
    /// `AstroPosX`/`AstroPosY`プローブに加え、本増分で追加した`AstroVelX`/
    /// `AstroVelY`プローブ(速度も出発点へ戻ることの確認に必要)経由で再現する。
    ///
    /// シーンJSONは`scenes/d35-orbital-insertion.json`として出荷する(残タスク
    /// 完遂のシーンギャラリー増分B3)。JSON側の`world.dt`は
    /// `analytic_period/steps_per_period`(`analytic_period`は下のケプラー第3
    /// 法則の式)を計算した結果をそのまま焼き込んだリテラル値——`semi_major_axis`/
    /// `analytic_period`自体はこのテストの期待値計算(`pos_err`/`vel_err`)には
    /// 使わないため、D34と同じ理由(モジュールdoc参照)で削除した。初期速度`v0`
    /// (=円軌道速度の0.9倍)は期待値計算(`vel_err`)にも使うためRust側に残す。
    #[test]
    fn run_headless_scenario_orbital_insertion_elliptical_period_matches_keplers_third_law() {
        let mass_central: f64 = 1.989e30;
        let r0: f64 = 1.496e11; // 1 AU相当(JSON側の`astro.bodies[1].position`と同じ値)。
        let g = sim_astro::GRAVITATIONAL_CONSTANT;
        let gm = g * mass_central;
        let v_circ = (gm / r0).sqrt();
        let v0 = v_circ * 0.9; // 円軌道より遅い初速 → 楕円軌道(出発点が遠地点)

        let steps_per_period = 4000u32;

        let json = include_str!("../../../scenes/d35-orbital-insertion.json");

        let result = run_headless_scenario(json, steps_per_period).expect("valid scenario JSON");
        let final_x = *result.probe_histories[0]
            .last()
            .expect("history should not be empty");
        let final_y = *result.probe_histories[1]
            .last()
            .expect("history should not be empty");
        let final_vx = *result.probe_histories[2]
            .last()
            .expect("history should not be empty");
        let final_vy = *result.probe_histories[3]
            .last()
            .expect("history should not be empty");

        let pos_err = ((final_x - r0).powi(2) + final_y.powi(2)).sqrt() / r0;
        let vel_err = (final_vx.powi(2) + (final_vy - v0).powi(2)).sqrt() / v0;
        assert!(
            pos_err < 0.01,
            "A3 + Kepler's third law: elliptical orbit should close after the analytic period: \
             pos_err={pos_err:.4} final_x={final_x} final_y={final_y}"
        );
        assert!(
            vel_err < 0.01,
            "A3 + Kepler's third law: velocity should also return to its initial value: \
             vel_err={vel_err:.4} final_vx={final_vx} final_vy={final_vy}"
        );
    }

    /// **増分G1** D8(散乱の再現): 球50個の接触だらけの落下を`scenes/d8-scatter.json`
    /// 経由で走らせ、同じJSONからの2回の実行が`state_hash`までビット一致することを確認する。
    ///
    /// `demos.rs`の`d8_scattered_spheres_with_same_seed_reproduce_identical_state_hash`
    /// との違いを明記しておく。あちらは**実行時に`SimRng`で散乱位置を生成**して
    /// 「同じシード→同じ配置→同じハッシュ」を見ている。こちらは静的なシーンJSONなので
    /// 散乱位置は黄金角スパイラルで**事前に焼き込んである**——つまりここで担保するのは
    /// 「シーン読み込み経路(JSONパース→`World::from_scenario`→300step)が決定的である」
    /// ことであり、散乱の乱数再現性そのものではない。ギャラリーが必要とするのは前者である。
    ///
    /// **正直な限界**: このシーンには確率的な物理項(ブラウン力など)が無いため、
    /// JSONの`seed`を42→43に書き換えても`state_hash`は**変わらない**(実測で確認:
    /// 3回とも 5936827133798999545)。`seed`が実際に結果を左右する経路はD25
    /// (ブラウン運動)側であり、このシーンでは`seed`は事実上不活性である。
    /// したがってハッシュ一致は「乱数を含めた再現性」ではなく「決定的積分の再現性」の
    /// 主張に留まる——弱いほうの主張だけを書く。
    #[test]
    fn run_headless_scenario_scatter_is_bit_reproducible() {
        let json = include_str!("../../../scenes/d8-scatter.json");
        let steps = 300u32;
        let a = run_headless_scenario(json, steps).expect("valid scenario JSON");
        let b = run_headless_scenario(json, steps).expect("valid scenario JSON");
        assert_eq!(
            a.final_state_hash, b.final_state_hash,
            "同じシーンJSONの2回の実行は`state_hash`までビット一致すべき"
        );

        // **ハッシュ一致が空虚でないこと**: 50球が実際に落ちて床で静止し、
        // どれも床をすり抜けていない(半径0.2、接触slopぶんの余裕を見る)。
        // 何も起きていないシーンならハッシュ一致は自明になってしまう。
        let scenario = Scenario::from_json(json).expect("valid scenario JSON");
        let (mut world, ids) = World::from_scenario_with_body_ids(&scenario).expect("valid world");
        assert_eq!(ids.len(), 51, "床1枚 + 球50個");
        for _ in 0..steps {
            world.step();
        }
        const SPHERE_RADIUS: f64 = 0.2;
        const CONTACT_SLOP: f64 = 0.05; // 実測の最終 min_y は 0.19614(半径0.2に対し貫入0.0039)。
        let position = &world.mechanics().bodies.position;
        let min_y = ids[1..]
            .iter()
            .map(|id| position[id.index as usize].y)
            .fold(f64::INFINITY, f64::min);
        assert!(
            min_y > SPHERE_RADIUS - CONTACT_SLOP,
            "どの球も床をすり抜けていないこと: min_y={min_y}"
        );
        // 300step(2.5秒)後には落下しきって静止している(s0は実測で速さ0)。
        let final_speed = *a.probe_histories[0].last().expect("履歴が空でない");
        assert!(
            final_speed < 0.1,
            "2.5秒後には静止しているべき: final_speed={final_speed}"
        );
    }

    /// **増分G1** D36(スイングバイ): 双曲線フライバイを`scenes/d36-swingby.json`
    /// 経由で解析解と突き合わせる。
    ///
    /// シーンは**近点から**始める配置にしてある——探査機の位置は惑星から+x方向に
    /// `r_p = 5e6 m`、相対速度は+y方向(位置ベクトルと直交)なので、この点が
    /// 定義上そのまま近点になる。相対速度の大きさ `v_p = 7189.993045893716 m/s` は
    /// 無限遠速度がちょうど `v_inf = 5000 m/s` になるよう逆算して焼き込んだ値
    /// (`v_p = sqrt(v_inf^2 + 2GM/r_p)`)。ここから離心率と漸近真近点角が閉形式で出る:
    ///
    /// - `e = r_p * v_p^2 / GM - 1 = 2.8729397662571174`
    /// - `nu_inf = arccos(-1/e) = 110.36965034969745°`(近点方向=+x から測った角度)
    ///
    /// 近点で相対速度は+y(=+xから90°)を向いており、無限遠では漸近線に平行=
    /// `nu_inf` を向く。つまり**近点から無限遠までの偏向は `nu_inf - 90° = 20.37°`**
    /// (全偏向 `2*arcsin(1/e) = 40.74°` の半分)。
    ///
    /// 実測(1e5秒 = 20,000ステップ後、r = 5.10e8 m ≒ 近点の102倍):
    /// 速度方向 110.36749°(解析解との相対誤差 **2.0e-5**)、
    /// 相対速さ 5026.0909 m/s に対し同じrでのvis-viva `sqrt(v_inf^2 + 2GM/r)` は
    /// 5026.0809 m/s(相対誤差 **2.0e-6**)。
    #[test]
    fn run_headless_scenario_swingby_deflection_matches_hyperbolic_analytic_solution() {
        let gm = sim_astro::GRAVITATIONAL_CONSTANT * 1.0e24; // JSON側の惑星質量。
        let r_p: f64 = 5.0e6; // JSON側の初期相対距離(=近点距離)。
        let v_inf: f64 = 5000.0; // JSON側の初期相対速度はこれを与えるよう逆算済み。
        let v_p = (v_inf * v_inf + 2.0 * gm / r_p).sqrt();
        let eccentricity = r_p * v_p * v_p / gm - 1.0;
        let nu_inf = (-1.0 / eccentricity).acos().to_degrees();

        let json = include_str!("../../../scenes/d36-swingby.json");
        let steps = 20_000u32; // dt=5s → 1e5秒。近点から近点距離の約100倍まで飛ばす。
        let result = run_headless_scenario(json, steps).expect("valid scenario JSON");
        let last = |i: usize| *result.probe_histories[i].last().expect("履歴が空でない");
        // プローブ0..3が探査機、4..7が惑星(惑星自身も+y方向へ 20 km/s で動いている
        // ため、双曲線軌道の量は**相対**座標で見る必要がある)。
        let (rx, ry) = (last(0) - last(4), last(1) - last(5));
        let (vx, vy) = (last(2) - last(6), last(3) - last(7));
        let r = rx.hypot(ry);
        let v = vx.hypot(vy);

        assert!(
            r > 50.0 * r_p,
            "漸近的な向きを見るには十分遠方まで飛ばす必要がある: r/r_p={}",
            r / r_p
        );

        // ①偏向: 相対速度の向きが漸近真近点角と一致する。
        let angle = vy.atan2(vx).to_degrees();
        let angle_rel_err = (angle - nu_inf).abs() / nu_inf;
        assert!(
            angle_rel_err < 1.0e-4,
            "双曲線フライバイの漸近方向が解析解と一致すべき: \
             angle={angle} nu_inf={nu_inf} rel_err={angle_rel_err:e}"
        );

        // ②エネルギー保存: 到達した距離でのvis-viva速度と一致する
        // (無限遠ではないので `v_inf` そのものではなく `sqrt(v_inf^2 + 2GM/r)` と比べる)。
        let vis_viva = (v_inf * v_inf + 2.0 * gm / r).sqrt();
        let speed_rel_err = (v - vis_viva).abs() / vis_viva;
        assert!(
            speed_rel_err < 1.0e-4,
            "相対速さがvis-vivaと一致すべき: v={v} vis_viva={vis_viva} rel_err={speed_rel_err:e}"
        );

        // ③スイングバイであること: 惑星に対する速さは(散逸が無いので)保存される
        // 一方、**慣性系での速さは変化する**——これが重力アシストの定義そのもの。
        // この配置では探査機は惑星の進行方向へ向かって近点を通るため**減速**する。
        // 解析的な漸近値は `|v_inf*(cos nu_inf, sin nu_inf) + (0, 20000)|`。
        let planet_speed: f64 = 20_000.0; // JSON側の惑星速度(+y)。
        let asymptotic_inertial = (v_inf * nu_inf.to_radians().cos())
            .hypot(v_inf * nu_inf.to_radians().sin() + planet_speed);
        let initial_inertial = v_p + planet_speed; // 近点では両者とも+y向き。
        let final_inertial = last(2).hypot(last(3));
        assert!(
            final_inertial < initial_inertial - 2_000.0,
            "スイングバイで慣性系の速さが変化しているべき: \
             initial={initial_inertial} final={final_inertial}"
        );
        // 有限距離(r は近点の約100倍)なので漸近値からは0.1%ほどずれる。
        assert!(
            (final_inertial - asymptotic_inertial).abs() / asymptotic_inertial < 3.0e-3,
            "慣性系の速さが解析的な漸近値に近づくべき: \
             final={final_inertial} asymptotic={asymptotic_inertial}"
        );
    }

    /// D2(弾道): 45°射出の真空放物運動を`body_pos_y`/`body_speed`の2プローブのみで検証する。
    /// このテストを書いた時点では`ProbeTarget`に水平位置を直接読める種別が無かったため
    /// (D11向けに追加した`BodyPosX`は本テストより後発)、`demos.rs`の
    /// `d2_ballistic_range_matches_45_degree_formula_and_drag_shortens_range`のように
    /// 着地x座標を直接assertすることはできなかった。代わりに同じ真空弾道物理から導出できる
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

        // シーンJSONは`scenes/d2-ballistic.json`として出荷する(残タスク完遂の
        // シーンギャラリー増分B2)。`linear_velocity`の[vx, vy]は
        // `v0*theta.cos()`/`v0*theta.sin()`を事前計算してそのまま焼き込んである
        // (Rustの`{}`は往復可能な最短表現を出すため、下の期待値計算と1ビットも
        // 変わらない)。
        let json = include_str!("../../../scenes/d2-ballistic.json");

        // プローブ履歴はリングバッファ(容量`DEFAULT_PROBE_CAPACITY`=600、`run_headless_scenario`
        // 参照)なので、着地ステップ(解析値T≈2.885s→step≈346)より前の区間が上書きされて
        // インデックスと絶対時刻の対応がずれないよう、stepsは容量以下に収める。
        let steps = 500;
        let result = run_headless_scenario(json, steps).expect("valid scenario JSON");
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

        // シーンJSONは`scenes/d1-free-fall.json`として出荷する(残タスク完遂の
        // シーンギャラリー増分B2)。全値がリテラルなので焼き込みに計算は不要。
        let json = include_str!("../../../scenes/d1-free-fall.json");

        let steps = 400; // 解析落下時間T≈2.019sに対し十分な余裕(dt=1/120で約243step)
        let result = run_headless_scenario(json, steps).expect("valid scenario JSON");
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
        // dt=1.0/240.0(反発の数値精度のため既定よりやや細かく、D1弾道と同じ理由)は
        // `scenes/d3-bounce.json`に焼き込み済み。
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

        // シーンJSONは`scenes/d3-bounce.json`として出荷する(残タスク完遂の
        // シーンギャラリー増分B2)。`dt`(=1.0/240.0)・`position.y`(=drop_height+radius)
        // は事前計算した値をそのまま焼き込んである(Rustの`{}`は往復可能な最短表現を
        // 出すため、下の期待値計算と1ビットも変わらない)。
        let json = include_str!("../../../scenes/d3-bounce.json");

        let steps = 500; // リングバッファ容量600以下(D1/D2の増分で確立した配慮)。
        let result = run_headless_scenario(json, steps).expect("valid scenario JSON");
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

    /// **残タスク完遂のシーンギャラリー増分**: リポジトリ直下の`scenes/index.json`
    /// マニフェストに載る全ファイルが実際に`Scenario::from_json`でパースでき、
    /// `World::from_scenario`+60step実行が成功することを確認する(壊れたアセットを
    /// 出荷しないためのゲート——各シーンJSONは`include_str!`経由でこのテストと同じ
    /// ファイルを見ているため、シーン自体の正しさは`run_headless_scenario_*`の各
    /// テストが個別に検証済みだが、本テストは「マニフェストに載っている全ファイルが
    /// 実在し壊れていないこと」をマニフェスト側から検証する)。
    #[test]
    fn all_scenes_in_the_gallery_manifest_parse_and_run_for_sixty_steps() {
        let manifest_json = include_str!("../../../scenes/index.json");
        let manifest: serde_json::Value =
            serde_json::from_str(manifest_json).expect("scenes/index.json must be valid JSON");
        let scenes = manifest["scenes"]
            .as_array()
            .expect("scenes/index.json must have a top-level \"scenes\" array");
        assert!(
            !scenes.is_empty(),
            "the gallery manifest should not be empty"
        );

        for entry in scenes {
            let file = entry["file"]
                .as_str()
                .expect("each manifest entry must have a \"file\" string");
            let scene_json = std::fs::read_to_string(format!(
                "{}/../../scenes/{file}",
                env!("CARGO_MANIFEST_DIR")
            ))
            .unwrap_or_else(|e| panic!("scenes/{file} listed in the manifest must exist: {e}"));

            let scenario = Scenario::from_json(&scene_json)
                .unwrap_or_else(|e| panic!("scenes/{file} must parse as a valid Scenario: {e:?}"));
            let mut world = World::from_scenario(&scenario)
                .unwrap_or_else(|e| panic!("scenes/{file} must build a valid World: {e:?}"));
            for _ in 0..60 {
                world.step();
            }
        }
    }
}
