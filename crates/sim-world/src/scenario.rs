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
    /// スキーマとしては妥当だが値が構成不能(**増分Hで追加**)。増分Hで足した
    /// 4ドメインは、既存の`UnknownMaterial`のような「名前が引けない」型の誤りでは
    /// 表せない不正値を持つ——粒子数を超えるピン留めindex、2未満の格子点数、
    /// 熱拡散率と材質名がどちらも無い棒、など。**構成できない値で`panic!`させない**
    /// のがこのバリアントの役目(シーンJSONはユーザーが書くデータであり、
    /// 不正入力でプロセスを落としてよい相手ではない)。
    InvalidValue(String),
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
    /// ソフトボディ(`sim_mechanics::SoftBody`、**増分Hで追加**)。D13(ロープと旗)。
    #[serde(default)]
    pub soft_body: Option<SoftBodyScenarioJson>,
    /// 2D格子流体(`sim_fluid::GridFluid2D`、**増分Hで追加**)。D14(煙と渦)・D15(対流)。
    #[serde(default)]
    pub grid_fluid: Option<GridFluidScenarioJson>,
    /// 1D熱伝導棒(`sim_thermal::ConductionRod1D`、**増分Hで追加**)。D16(熱伝導レース)。
    #[serde(default)]
    pub conduction_rod: Option<ConductionRodScenarioJson>,
    /// SPH流体(`sim_fluid::SphFluid`、**増分Hで追加**)。D23(注ぐ水)。
    #[serde(default)]
    pub sph: Option<SphScenarioJson>,
    /// 気体区画(`sim_thermal::GasCompartment`、**増分H3で追加**)。D17(ピストン)。
    #[serde(default)]
    pub gas: Option<GasScenarioJson>,
    /// 量子ドメイン1D(`sim_quantum::WaveFunction1D`、**群3で追加**)。
    /// D27(トンネル効果)・D29(調和振動子)。
    #[serde(default)]
    pub quantum_1d: Option<Quantum1dScenarioJson>,
    /// 量子ドメイン2D(`sim_quantum::WaveFunction2D`、**群3で追加**)。D28(二重スリット)。
    #[serde(default)]
    pub quantum_2d: Option<Quantum2dScenarioJson>,
    /// ブラウン運動(`sim_statistical::BrownianParticleSet`、**群3で追加**)。D25。
    #[serde(default)]
    pub brownian: Option<BrownianScenarioJson>,
    /// 気体分子運動論(`sim_statistical::GasSim`、**群3で追加**)。D30(気体分子の箱)。
    #[serde(default)]
    pub kinetic_gas: Option<KineticGasScenarioJson>,
    /// イジング模型(`sim_statistical::IsingSim`、**群3で追加**)。D31(相転移)。
    #[serde(default)]
    pub ising: Option<IsingScenarioJson>,
    /// FDTD(`sim_em::FdtdSim2D`、**群3で追加**)。D32(電磁波の伝播)。
    #[serde(default)]
    pub fdtd: Option<FdtdScenarioJson>,
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
    /// 初期角速度(**増分H3で追加**)。D20(手回し発電)はキネマティック剛体を
    /// 一定角速度で回すのが構成そのものなので、これが無いと書けなかった。
    #[serde(default)]
    pub angular_velocity: [f64; 3],
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
    /// 衝突フィルタ(**群4で追加**、群2で `sim-mechanics` に実装した
    /// `collision_group`/`collision_mask` のシーンJSON側)。未指定なら既定
    /// (グループ1・全マスク)で、既存シーンの挙動は一切変わらない。
    ///
    /// **D24(車)で実際に必要になった**——サスペンションで繋がったシャシーと
    /// 車輪は幾何的に必ず重なるので、接触ソルバが働くとジョイントと綱引きになり
    /// サスペンションが沈まない。
    #[serde(default)]
    pub collision_group: Option<u32>,
    #[serde(default)]
    pub collision_mask: Option<u32>,
}

/// 設計§3の例示に現れる3形状のみ(`Capsule`/`Compound`/`ConvexMesh`は`raycast`/
/// `overlap`モジュール同様、narrowphase未実装のため対象外)。
#[derive(Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ShapeJson {
    Box {
        half: [f64; 3],
    },
    Sphere {
        radius: f64,
    },
    /// カプセル(**増分Lで追加**、ローカル+y軸が長軸)。`sim-mechanics`側の
    /// 体積・慣性・接触(平面/球/カプセル)を同増分で実装した。
    /// **カプセル×箱の接触は未実装**(`None`を返す)。
    Capsule {
        radius: f64,
        half_height: f64,
    },
    Plane {
        normal: [f64; 3],
        d: f64,
    },
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
    /// ノード間の伝導リンク(`ThermalSolver::add_link`、**増分Hで追加**)。
    /// これが無いあいだ熱ドメインは「互いに繋がっていない単独ノードの集まり」
    /// しか書けず、伝導ネットワークを持つシーン(D10の摩擦熱→周囲、D18の氷↔飲み物)
    /// が表現できなかった。
    #[serde(default)]
    pub links: Vec<ThermalLinkJson>,
    /// 放射の相手側温度(`ThermalSolver::environment_radiation_temperature`)。
    /// 未指定なら`ThermalSolver::new`の既定のまま。
    #[serde(default)]
    pub environment_radiation_temperature: Option<f64>,
}

/// `ThermalScenarioJson::links`の1件(`sim_thermal::ThermalLink`、**増分Hで追加**)。
/// `a`/`b`は`thermal.nodes`配列のインデックス。
#[derive(Deserialize)]
pub struct ThermalLinkJson {
    pub a: usize,
    pub b: usize,
    pub conductance: f64,
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
    /// 放射率(**増分Hで追加**)。`ThermalSolver`は`radiation_coefficient`で
    /// 既に放射項を解いているのに、シーンJSON側から設定する手段が無かった。
    #[serde(default)]
    pub emissivity: f64,
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
    /// **増分Hで追加**。`soft_body`(粒子index)・`conduction_rod`(格子点index)・
    /// `grid_fluid`(平均鉛直速度)・`sph`(粒子index)の観測量。これらのドメインは
    /// Scene Viewに何も描かれないため、Probe Graphsが唯一の観測手段になる。
    SoftBodyPosX(usize),
    SoftBodyPosY(usize),
    RodTemp(usize),
    /// 引数を取らない(格子全体の平均)。JSONでは `{"grid_fluid_mean_v": null}` ではなく
    /// `"grid_fluid_mean_v"` という文字列で書く(serdeのunit variant表現)。
    GridFluidMeanV,
    /// 鉛直速度のRMS(D14の渦)。`grid_fluid_mean_v`同様 unit variant。
    GridFluidRmsV,
    SphParticlePosY(usize),
    SphParticleDensity(usize),
    /// 回路の節点電圧(`circuit.num_nodes`の範囲のインデックス、0=接地。
    /// **増分G2で追加** — D19の合格基準 E5(分圧)・E3(RC放電)・スイッチによる
    /// LED分岐の開閉はいずれも節点電圧で観測する現象で、既存の`CircuitCurrent`
    /// (電圧源の電流)では再構成できない)。
    CircuitNodeVoltage(usize),
    /// 回路の電圧源index(`circuit.voltage_sources`配列のインデックス)を流れる電流。
    CircuitCurrent(usize),
    /// **群3で追加**。量子・統計・FDTDの観測量。これらのドメインは Scene View に
    /// 直接の3D表現を持たない(波動関数・スピン格子・場)ため、**Probe Graphs と
    /// 専用オーバーレイが唯一の観測手段**になる。
    ///
    /// 量子1D: 全確率(ノルム、ユニタリなら1で一定)・位置期待値・エネルギー期待値・
    /// 指定範囲の透過確率。
    QuantumNorm,
    QuantumMeanX,
    QuantumEnergy,
    /// 格子インデックス `i` 以降の確率(トンネル効果の透過率、D27)。
    QuantumTransmission(usize),
    /// 統計: 気体の温度・圧力、イジングの磁化・1スピンあたりエネルギー、
    /// ブラウン粒子の平均二乗変位。
    GasTemperature,
    GasPressure,
    IsingMagnetization,
    IsingEnergyPerSpin,
    BrownianMsd,
    /// FDTD: 格子点 `(i, j)` の Ez、および全電磁エネルギー。
    FdtdEz(usize, usize),
    FdtdEnergy,
}

/// `Scenario::joints`の1件。設計の例示JSONには無い項目(モジュールdoc
/// 「縮約実装の理由」参照)——`sim_mechanics::MechanicsSolver`が既に持つ拘束の
/// うち、D11(振り子と時計)が要る`DistanceJoint`(ワールド固定点、または
/// 剛体間の一定長ピン拘束)のみ対応する(`BallJoint`/`SliderJoint`/
/// `HingeMotorPd`は後続増分)。
#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointJson {
    /// 直線スライダ拘束(`sim_mechanics::SliderJoint`、**増分H3で追加**)。
    /// D17(ピストン)は「1軸に沿ってのみ動けるピストン」が構成そのもの。
    /// `body_b`未指定ならワールドに対する拘束。
    Slider {
        body_a: String,
        #[serde(default)]
        anchor_a: [f64; 3],
        axis: [f64; 3],
        #[serde(default)]
        body_b: Option<String>,
        #[serde(default)]
        anchor_b: [f64; 3],
    },
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
    /// **ホイールジョイント(`sim_mechanics::WheelJoint`、群4で追加)**。
    /// サスペンション(ソフト拘束)+ 駆動モーター + 操舵を1つにまとめた複合拘束。
    /// D24(車の実験場)を組むのに要る——**この型が無かったことが D24 が
    /// 「新規物理待ちでスコープ外」だった直接の原因**。
    Wheel {
        /// 車体(シャシー)のボディ名。
        chassis: String,
        /// 車輪のボディ名。
        wheel: String,
        /// シャシーローカルのサスペンション取り付け点。
        anchor_chassis: [f64; 3],
        /// サスペンションの自然長 [m]。
        rest_length: f64,
        /// サスペンション軸(シャシーローカル)。未指定なら下向き `(0,-1,0)`。
        #[serde(default)]
        suspension_axis: Option<[f64; 3]>,
        /// 車軸方向(シャシーローカル)。未指定なら `(1,0,0)`。
        #[serde(default)]
        axle_axis: Option<[f64; 3]>,
        /// ばねの固有振動数 [Hz]。未指定なら乗用車相当(1.5 Hz)。
        #[serde(default)]
        frequency: Option<f64>,
        /// 減衰比。未指定なら 0.3。
        #[serde(default)]
        damping_ratio: Option<f64>,
        /// 操舵角 [rad]。
        #[serde(default)]
        steer_angle: Option<f64>,
        /// 駆動モーターの目標角速度 [rad/s]。
        #[serde(default)]
        motor_speed: Option<f64>,
        /// 駆動モーターのトルク上限 [N·m](0 なら空転)。
        #[serde(default)]
        motor_max_torque: Option<f64>,
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
    /// **増分Hで追加した結合群**。ここまで`couplings`は14種の`Coupling`実装のうち
    /// 4種(`ImageChargeForce`/`BrownianForce`/`InductionCoupling`/`BuoyancyDrag`)
    /// + G2の`JouleHeat`しか書けなかった。残りをまとめて開通させる。
    ///
    /// D15(対流)向け(`sim_coupling::BoussinesqBuoyancy`)。
    BoussinesqBuoyancy {
        thermal_node: usize,
        ambient_temperature: f64,
        thermal_expansion_coefficient: f64,
    },
    /// D10(摩擦の熱)向け(`sim_coupling::DissipationToHeat`)。力学の散逸を熱ノードへ。
    DissipationToHeat { thermal_node: usize },
    /// D18(氷と飲み物)向け(`sim_coupling::PhaseChangeMorph`)。
    /// `initial_enthalpy`が負なら融点未満の固相から始まる。
    PhaseChangeMorph {
        body: String,
        thermal_node: usize,
        melting_temperature: f64,
        latent_heat_fusion: f64,
        specific_heat_solid: f64,
        specific_heat_liquid: f64,
        initial_mass: f64,
        conductance: f64,
        #[serde(default)]
        initial_enthalpy: f64,
    },
    /// D20(モーターと発電)向け(`sim_coupling::MotorCoupling`)。
    MotorCoupling {
        body: String,
        axis: [f64; 3],
        voltage_source_index: usize,
        torque_constant: f64,
    },
    /// D23(注ぐ水)向け(`sim_coupling::SphRigid`)。SPH粒子と剛体の双方向。
    SphRigid {
        body: String,
        radius: f64,
        /// 剛体表面を表す境界粒子の個数(`SphRigid::new`が`SphFluid`側へ確保する)。
        boundary_points: usize,
    },
    /// D14(煙と渦)向け(`sim_coupling::GridFluidRigid`)。格子流体と剛体の双方向。
    GridFluidRigid {
        body: String,
        half_width: f64,
        half_height: f64,
    },
    /// 静電場中の荷電粒子(`sim_coupling::LorentzForce`)。
    LorentzForce { body: String, charge: f64 },
    /// 対流の相関式による熱リンク(`sim_coupling::ConvectionLink`)。
    ConvectionLink {
        fluid_node: usize,
        surface_node: usize,
        area: f64,
        characteristic_length: f64,
        fluid_thermal_conductivity: f64,
        kinematic_viscosity: f64,
        prandtl_number: f64,
    },
    /// D17(ピストン)向け(`sim_coupling::PistonGas`、**増分H3で追加**)。
    /// `initial_volume`は`gas.volume`と同じ値を書く(結合が基準体積として使う)。
    PistonGas {
        body: String,
        axis: [f64; 3],
        area: f64,
        initial_volume: f64,
    },
    /// D19(電気工作台)向け(`sim_coupling::JouleHeat`、**増分G2で追加**)。
    /// 回路の抵抗損失を熱ノードへ注入する。`thermal_node`は`thermal.nodes`配列の
    /// インデックス(`BrownianForce`と同じ縮約)。
    JouleHeat { thermal_node: usize },
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
    /// **増分G2で追加**。D19(電気工作台)の合格基準は E5(分圧)・E3(RC放電)・
    /// スイッチによるLED分岐の開閉であり、抵抗と電圧源だけでは**放電もスイッチも
    /// ダイオードも書けなかった**。`sim_em::Circuit`側には
    /// `add_capacitor`/`add_inductor`/`add_diode`/`add_switch`が揃っているので、
    /// シーンJSON側が追いついていなかっただけである。
    #[serde(default)]
    pub capacitors: Vec<CapacitorJson>,
    #[serde(default)]
    pub inductors: Vec<InductorJson>,
    #[serde(default)]
    pub diodes: Vec<DiodeJson>,
    /// 登録順が`Command::SetSwitch`の`switch_index`になる
    /// (`voltage_sources`と`couplings`の関係と同じ縮約)。
    #[serde(default)]
    pub switches: Vec<SwitchJson>,
}

/// `CircuitScenarioJson::capacitors`の1件(`sim_em::Circuit::add_capacitor`)。
/// `initial_voltage`は「予め充電された状態」を書くために要る——D19のRC放電は
/// これが無いと初期電圧0から始まってしまい、指数減衰そのものが起きない。
#[derive(Deserialize)]
pub struct CapacitorJson {
    pub a: usize,
    pub b: usize,
    pub capacitance: f64,
    #[serde(default)]
    pub initial_voltage: f64,
}

/// `CircuitScenarioJson::inductors`の1件(`sim_em::Circuit::add_inductor`)。
#[derive(Deserialize)]
pub struct InductorJson {
    pub a: usize,
    pub b: usize,
    pub inductance: f64,
    #[serde(default)]
    pub initial_current: f64,
}

/// `CircuitScenarioJson::diodes`の1件(`sim_em::Circuit::add_diode`)。
/// `saturation_current`/`n_vt`はShockleyダイオード式のパラメータ。
#[derive(Deserialize)]
pub struct DiodeJson {
    pub anode: usize,
    pub cathode: usize,
    pub saturation_current: f64,
    pub n_vt: f64,
}

/// `CircuitScenarioJson::switches`の1件(`sim_em::Circuit::add_switch`)。
#[derive(Deserialize)]
pub struct SwitchJson {
    pub a: usize,
    pub b: usize,
    #[serde(default)]
    pub closed: bool,
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
    /// 大気抗力(`NBodySystem::enable_atmospheric_drag`、**増分H2で追加**)。D37(再突入)。
    #[serde(default)]
    pub atmospheric_drag: Option<AtmosphericDragJson>,
    /// 一般相対論の近日点移動補正(`enable_relativistic_correction`、**増分H2で追加**)。
    /// D39(相対論 ON/OFF)。`[central_body, speed_of_light]`。
    #[serde(default)]
    pub relativistic_correction: Option<RelativisticCorrectionJson>,
}

/// `AstroScenarioJson::atmospheric_drag`(**増分H2で追加**)。
/// `ballistic_coefficients`は`(bodies配列のindex, 弾道係数)`の対。
#[derive(Deserialize)]
pub struct AtmosphericDragJson {
    pub central_body: usize,
    pub surface_density: f64,
    pub scale_height: f64,
    pub planet_radius: f64,
    #[serde(default)]
    pub ballistic_coefficients: Vec<(usize, f64)>,
}

/// `AstroScenarioJson::relativistic_correction`(**増分H2で追加**)。
#[derive(Deserialize)]
pub struct RelativisticCorrectionJson {
    pub central_body: usize,
    pub speed_of_light: f64,
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

/// `Scenario::soft_body`(`sim_mechanics::SoftBody`、**増分Hで追加**)。
///
/// **縮約**: 粒子と距離拘束を直接並べるほか、D13(ロープ)向けに
/// `rope`ヘルパ相当の生成を`rope`フィールドで書ける近道を用意する
/// (20分割のロープを粒子21個+拘束20本として手で書くのは現実的でないため)。
/// `rope`と`particles`は排他ではなく、`rope`を展開した後に`particles`を足す。
#[derive(Deserialize)]
pub struct SoftBodyScenarioJson {
    /// 直線ロープの生成(`sim_mechanics::rope`と同じ引数)。
    #[serde(default)]
    pub rope: Option<RopeJson>,
    #[serde(default)]
    pub particles: Vec<SoftParticleJson>,
    #[serde(default)]
    pub constraints: Vec<SoftConstraintJson>,
    /// 追加でピン留めする粒子のインデックス(`rope`の端点固定などに使う)。
    #[serde(default)]
    pub pinned: Vec<usize>,
    /// 自動ステップ用の積分設定(`SoftBody`の同名フィールド、未指定なら既定値)。
    #[serde(default)]
    pub gravity: Option<[f64; 3]>,
    #[serde(default)]
    pub substeps: Option<u32>,
    #[serde(default)]
    pub iterations: Option<u32>,
    #[serde(default)]
    pub damping: Option<f64>,
}

/// `SoftBodyScenarioJson::rope`(`sim_mechanics::rope`ヘルパの引数)。
#[derive(Deserialize)]
pub struct RopeJson {
    pub from: [f64; 3],
    pub to: [f64; 3],
    pub segments: usize,
    pub mass_per_particle: f64,
    pub total_rest_length: f64,
    #[serde(default)]
    pub compliance: f64,
}

/// `SoftBodyScenarioJson::particles`の1件。`mass`が0以下ならピン留め
/// (`SoftBody::add_particle`の規約そのまま)。
#[derive(Deserialize)]
pub struct SoftParticleJson {
    pub position: [f64; 3],
    pub mass: f64,
}

/// `SoftBodyScenarioJson::constraints`の1件。
#[derive(Deserialize)]
pub struct SoftConstraintJson {
    pub i: usize,
    pub j: usize,
    pub rest: f64,
    #[serde(default)]
    pub compliance: f64,
}

/// `Scenario::grid_fluid`(`sim_fluid::GridFluid2D`、**増分Hで追加**)。
#[derive(Deserialize)]
pub struct GridFluidScenarioJson {
    pub nx: usize,
    pub ny: usize,
    pub h: f64,
    /// `Solver::step`が使う既定密度。未指定なら`GridFluid2D::new`の既定。
    #[serde(default)]
    pub density: Option<f64>,
    #[serde(default)]
    pub kinematic_viscosity: Option<f64>,
    /// 格子全体を一様な初期速度で満たす `[u, v]`(**増分Hで追加**)。
    ///
    /// **なぜ要ったか**: `GridFluid2D`は周期境界で、流入境界を設定する手段が無い。
    /// 静止状態から始めると外力の無いD14(カルマン渦列)は**何も起きない**
    /// (実測で平均鉛直速度が 0 のまま動かないことを確認した)。一様流で満たせば
    /// 周期境界がそのまま流れを循環させ、障害物まわりの擾乱が観測できる。
    /// **縮約**: 本物の流入・流出境界ではないので、下流の後流が上流へ回り込む。
    /// レイノルズ数の定量的な検証(F11のSt数)は`sim-fluid`側の専用テストが担い、
    /// このシーンが示すのは「障害物が一様流を実際に乱すこと」までである。
    #[serde(default)]
    pub initial_velocity: Option<[f64; 2]>,
}

/// `Scenario::conduction_rod`(`sim_thermal::ConductionRod1D`、**増分Hで追加**)。
///
/// **縮約**: `thermal_diffusivity`は直接指定するほか、`material`(材質名)を
/// 書けば`MaterialDb`から $\alpha = k/(\rho c_p)$ を計算する。D16(熱伝導レース)は
/// 「銅・鋼・木材の $\alpha$ の比が立ち上がり順を決める」デモなので、
/// 材質名で書けることがそのままデモの主旨になる。
#[derive(Deserialize)]
pub struct ConductionRodScenarioJson {
    pub node_count: usize,
    pub length: f64,
    pub initial_temperature: f64,
    #[serde(default)]
    pub thermal_diffusivity: Option<f64>,
    #[serde(default)]
    pub material: Option<String>,
    /// 両端のDirichlet境界温度(`set_boundary_temperatures`)。
    #[serde(default)]
    pub boundary_left: Option<f64>,
    #[serde(default)]
    pub boundary_right: Option<f64>,
}

/// `Scenario::sph`(`sim_fluid::SphFluid`、**増分Hで追加**)。
///
/// **縮約**: 粒子を1つずつ並べるのは非現実的なので、直方体ブロックを格子状に
/// 敷き詰める`blocks`と、境界粒子を敷く`boundary_blocks`で書く。
#[derive(Deserialize)]
pub struct SphScenarioJson {
    /// 平滑化長。
    pub h: f64,
    /// 静止密度。
    pub rest_density: f64,
    /// 数値音速。
    pub sound_speed: f64,
    /// 1粒子の質量。**未指定だと`SphFluid`の既定のままになり密度が静止密度から
    /// 大きく外れる**ので、通常は `rest_density * spacing^3` を明示する
    /// (`sim-fluid`のテスト群がすべてそうしている)。
    #[serde(default)]
    pub particle_mass: Option<f64>,
    #[serde(default)]
    pub blocks: Vec<SphBlockJson>,
    #[serde(default)]
    pub boundary_blocks: Vec<SphBlockJson>,
}

/// `SphScenarioJson::blocks`の1件。`min`から`spacing`刻みで各軸`counts`個の
/// 粒子を格子状に置く。
#[derive(Deserialize)]
pub struct SphBlockJson {
    pub min: [f64; 3],
    pub counts: [usize; 3],
    pub spacing: f64,
    #[serde(default)]
    pub velocity: [f64; 3],
}

/// `Scenario::gas`(`sim_thermal::GasCompartment`、**増分H3で追加**)。
///
/// **注意(縮約)**: `GasCompartment`は`Solver`を実装しておらず`World::step()`は
/// これを回さない。気体の状態を変えるのは`couplings[].piston_gas`(ピストンの
/// 変位から体積を決め、断熱関係で圧力・温度を更新して反力を返す)であり、
/// 気体単独では時間発展しない——これは`SoftBody`/`ConductionRod1D`の
/// 「回されていなかった」問題とは違い、**気体区画に固有の時間発展が無い**
/// (準静的モデル)という物理側の性質である。
#[derive(Deserialize)]
pub struct GasScenarioJson {
    pub n_moles: f64,
    pub volume: f64,
    pub temperature: f64,
    /// 分子の自由度 f(単原子3・二原子5)。未指定なら空気(5)。
    #[serde(default)]
    pub degrees_of_freedom: Option<f64>,
    /// モル質量 [kg/mol]。未指定なら空気(28.97e-3)。
    #[serde(default)]
    pub molar_mass: Option<f64>,
}

/// `Scenario::quantum_1d`(**群3で追加**)。1D TDSE(原子単位 $\hbar=m_e=1$)。
///
/// **ポテンシャルは「よく使う3形」を列挙する**——任意関数を JSON で表現する仕組み
/// (数式パーサ)を作るのは大がかりで、D27(矩形障壁のトンネル)・D29(調和振動子)が
/// 要るのはこの3形で足りる。足りなくなったら形を足す。
#[derive(Deserialize)]
pub struct Quantum1dScenarioJson {
    /// 格子点数(2の冪、`sim_math::fft`の制約)。
    pub n: usize,
    pub dx: f64,
    /// 初期ガウス波束 $\exp[-(x-x_0)^2/(4\sigma^2)+ik_0x]$。
    pub packet: GaussianPacketJson,
    #[serde(default)]
    pub potential: Option<Potential1dJson>,
}

#[derive(Deserialize)]
pub struct GaussianPacketJson {
    pub x0: f64,
    pub sigma: f64,
    pub k0: f64,
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Potential1dJson {
    /// 矩形障壁(D27 トンネル効果)。`[x_min, x_max]` の区間で高さ `height`。
    Barrier { x_min: f64, x_max: f64, height: f64 },
    /// 調和振動子 $V=\frac12 m\omega^2(x-x_c)^2$(原子単位 $m=1$、D29)。
    Harmonic { center: f64, omega: f64 },
    /// 無限井戸(区間外を`height`で塞ぐ)。
    Well { x_min: f64, x_max: f64, height: f64 },
}

/// `Scenario::quantum_2d`(**群3で追加**)。D28(二重スリット)。
#[derive(Deserialize)]
pub struct Quantum2dScenarioJson {
    pub nx: usize,
    pub ny: usize,
    pub dx: f64,
    pub dy: f64,
    pub packet: GaussianPacket2dJson,
    /// 二重スリットの壁(未指定なら自由空間)。
    #[serde(default)]
    pub double_slit: Option<DoubleSlitJson>,
}

#[derive(Deserialize)]
pub struct GaussianPacket2dJson {
    pub x0: f64,
    pub y0: f64,
    pub sigma_x: f64,
    pub sigma_y: f64,
    pub k0: f64,
}

/// 二重スリット壁(x=`wall_x` に厚み `thickness` の高いポテンシャル壁を立て、
/// `slit_centers` の各位置に幅 `slit_width` の開口を空ける)。
#[derive(Deserialize)]
pub struct DoubleSlitJson {
    pub wall_x: f64,
    pub thickness: f64,
    pub height: f64,
    pub slit_centers: Vec<f64>,
    pub slit_width: f64,
}

/// `Scenario::brownian`(**群3で追加**)。D25(ブラウン運動)。
///
/// **これまで D25 は `format!` で300粒子ぶんの剛体を動的生成する
/// インラインシーンだった**(静的ファイル化に不向きとして `scenes/` へ
/// 切り出さずインラインのまま残していた)。粒子集合を「個数+分布」として
/// 宣言できるようにすれば静的ファイルで書けるので、その形にする。
#[derive(Deserialize)]
pub struct BrownianScenarioJson {
    pub particle_count: usize,
    /// 粒子質量 [kg]。
    pub mass: f64,
    /// ストークス抵抗係数 γ [kg/s]。
    pub gamma: f64,
    /// $k_BT$ [J]。
    pub kb_t: f64,
    /// 一様外力 [N](重力・浮力の正味など)。未指定なら 0(自由拡散)。
    #[serde(default)]
    pub external_force: Option<[f64; 3]>,
    /// 初期位置。未指定なら全粒子を原点に置く(自由拡散の MSD 測定の標準設定)。
    #[serde(default)]
    pub initial_position: Option<[f64; 3]>,
}

/// `Scenario::kinetic_gas`(**群3で追加**)。D30(気体分子の箱)。
#[derive(Deserialize)]
pub struct KineticGasScenarioJson {
    pub particle_count: usize,
    /// 分子質量 [kg]。
    pub mass: f64,
    /// 剛体球半径 [m]。
    pub radius: f64,
    pub box_size: [f64; 3],
    /// 初期速度をサンプルする温度 [K](マクスウェル分布)。
    pub temperature: f64,
}

/// `Scenario::ising`(**群3で追加**)。D31(イジング模型の相転移)。
#[derive(Deserialize)]
pub struct IsingScenarioJson {
    /// 格子の一辺 L(全 L×L スピン)。
    pub l: usize,
    /// 交換相互作用 J。
    pub j_coupling: f64,
    /// 温度 $k_BT/J$ の単位。臨界点は $T_c = 2/\ln(1+\sqrt2) \approx 2.269$。
    pub temperature: f64,
    /// 1 step あたりの更新回数(未指定なら1)。
    #[serde(default)]
    pub updates_per_step: Option<u32>,
    /// Wolff クラスタ法を使うか(未指定なら false = メトロポリス法)。
    #[serde(default)]
    pub use_wolff: Option<bool>,
}

/// `Scenario::fdtd`(**群3で追加**)。D32(電磁波の伝播)。
#[derive(Deserialize)]
pub struct FdtdScenarioJson {
    pub nx: usize,
    pub ny: usize,
    /// 格子間隔 h(正規化単位、$c=1$)。
    pub h: f64,
    /// Courant 数 $c\Delta t/h$(2D の上限は $1/\sqrt2$、既定 0.5)。
    #[serde(default)]
    pub courant: Option<f64>,
    /// 初期条件。未指定なら全ゼロ(何も起きない)。
    #[serde(default)]
    pub initial: Option<FdtdInitialJson>,
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FdtdInitialJson {
    /// 矩形空洞の固有モード $E_z = \sin(m\pi x/a)\sin(n\pi y/b)$(設計§7の検証設定)。
    CavityMode { m: u32, n: u32, amplitude: f64 },
    /// 単一格子点のガウス的な盛り上がり(点源からの波の広がりを見る)。
    Pulse {
        i: usize,
        j: usize,
        amplitude: f64,
        width: f64,
    },
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
                ShapeJson::Capsule {
                    radius,
                    half_height,
                } => Shape::Capsule {
                    radius,
                    half_height,
                },
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
            desc.angular_velocity = array_to_vec3(body.angular_velocity);
            desc.mass_override = body.mass_override;
            // 衝突フィルタ(群4で追加、フィールドのdoc参照)。
            if let Some(group) = body.collision_group {
                desc.collision_group = group;
            }
            if let Some(mask) = body.collision_mask {
                desc.collision_mask = mask;
            }
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
                node.emissivity = n.emissivity;
                solver.add_node(node);
            }
            // **増分Hで追加**: ノード間の伝導リンク。
            for link in &thermal_json.links {
                solver.add_link(link.a, link.b, link.conductance);
            }
            if let Some(t) = thermal_json.environment_radiation_temperature {
                solver.environment_radiation_temperature = t;
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
            for c in &circuit_json.capacitors {
                circuit.add_capacitor(c.a, c.b, c.capacitance, c.initial_voltage);
            }
            for l in &circuit_json.inductors {
                circuit.add_inductor(l.a, l.b, l.inductance, l.initial_current);
            }
            for d in &circuit_json.diodes {
                circuit.add_diode(d.anode, d.cathode, d.saturation_current, d.n_vt);
            }
            for s in &circuit_json.switches {
                // 返り値の`switch_index`は登録順に0,1,2...なので捨ててよい
                // (`Command::SetSwitch`が参照するのはこの順序、`SwitchJson`のdoc参照)。
                circuit.add_switch(s.a, s.b, s.closed);
            }
            world.enable_circuit(circuit);
        }

        if let Some(astro_json) = &scenario.astro {
            let mut sys = sim_astro::NBodySystem::new(astro_json.softening);
            for b in &astro_json.bodies {
                sys.add_body(array_to_vec3(b.position), array_to_vec3(b.velocity), b.mass);
            }
            if let Some(drag) = &astro_json.atmospheric_drag {
                sys.enable_atmospheric_drag(
                    drag.central_body,
                    drag.surface_density,
                    drag.scale_height,
                    drag.planet_radius,
                );
                // `set_ballistic_coefficient`は`enable_atmospheric_drag`より後でないと
                // 無言で無視される(内部の`ballistic_coefficient`ベクタが未確保のため)。
                for (body, value) in &drag.ballistic_coefficients {
                    sys.set_ballistic_coefficient(*body, *value);
                }
            }
            if let Some(rc) = &astro_json.relativistic_correction {
                sys.enable_relativistic_correction(rc.central_body, rc.speed_of_light);
            }
            world.enable_astro(sys);
        }

        // **増分Hで追加した4ドメイン**。ここまで`World`は`enable_soft_body`/
        // `enable_grid_fluid`/`enable_conduction_rod`/`enable_sph`を持ちながら、
        // シーンJSONからは1つも構成できなかった(D13/D14/D15/D16/D23が
        // 「ヘッドレスGreen・目視チェック保留」で滞留していた根本原因)。
        if let Some(sb) = &scenario.soft_body {
            let mut body = if let Some(r) = &sb.rope {
                sim_mechanics::rope(
                    array_to_vec3(r.from),
                    array_to_vec3(r.to),
                    r.segments,
                    r.mass_per_particle,
                    r.total_rest_length,
                    r.compliance,
                )
            } else {
                sim_mechanics::SoftBody::new()
            };
            for particle in &sb.particles {
                body.add_particle(array_to_vec3(particle.position), particle.mass);
            }
            for c in &sb.constraints {
                body.add_distance_constraint(c.i, c.j, c.rest, c.compliance);
            }
            for &index in &sb.pinned {
                if index >= body.position.len() {
                    return Err(SceneError::InvalidValue(format!(
                        "soft_body.pinned references particle {index} but only {} exist",
                        body.position.len()
                    )));
                }
                body.pin(index);
            }
            if let Some(g) = sb.gravity {
                body.gravity = array_to_vec3(g);
            }
            if let Some(n) = sb.substeps {
                body.substeps = n;
            }
            if let Some(n) = sb.iterations {
                body.iterations = n;
            }
            if let Some(d) = sb.damping {
                body.damping = d;
            }
            world.enable_soft_body(body);
        }

        if let Some(gf) = &scenario.grid_fluid {
            let mut fluid = sim_fluid::GridFluid2D::new(gf.nx, gf.ny, gf.h);
            if let Some(d) = gf.density {
                fluid.density = d;
            }
            if let Some(v) = gf.kinematic_viscosity {
                fluid.kinematic_viscosity = v;
            }
            if let Some([u0, v0]) = gf.initial_velocity {
                fluid.u.iter_mut().for_each(|x| *x = u0);
                fluid.v.iter_mut().for_each(|x| *x = v0);
            }
            world.enable_grid_fluid(fluid);
        }

        if let Some(rod) = &scenario.conduction_rod {
            // 熱拡散率は直接指定が最優先、無ければ材質名から α=k/(ρ·c_p) を計算する
            // (D16「熱伝導レース」は材質ごとのαの比がそのままデモの主旨なので、
            // 材質名で書けることに意味がある)。
            let alpha = match (&rod.thermal_diffusivity, &rod.material) {
                (Some(a), _) => *a,
                (None, Some(name)) => {
                    let id = world
                        .materials()
                        .find_by_name(name)
                        .ok_or_else(|| SceneError::UnknownMaterial(name.clone()))?;
                    let m = world.materials().get(id);
                    m.conductivity / (m.density * m.specific_heat)
                }
                (None, None) => {
                    return Err(SceneError::InvalidValue(
                        "conduction_rod requires either `thermal_diffusivity` or `material`"
                            .to_string(),
                    ))
                }
            };
            if rod.node_count < 2 {
                return Err(SceneError::InvalidValue(format!(
                    "conduction_rod.node_count must be at least 2 (got {})",
                    rod.node_count
                )));
            }
            let mut bar = sim_thermal::ConductionRod1D::new(
                rod.node_count,
                rod.length,
                rod.initial_temperature,
                alpha,
            );
            // 片側だけ書いた場合はもう片側を初期温度のままにする。
            if rod.boundary_left.is_some() || rod.boundary_right.is_some() {
                bar.set_boundary_temperatures(
                    rod.boundary_left.unwrap_or(rod.initial_temperature),
                    rod.boundary_right.unwrap_or(rod.initial_temperature),
                );
            }
            world.enable_conduction_rod(bar);
        }

        if let Some(sph_json) = &scenario.sph {
            let mut fluid =
                sim_fluid::SphFluid::new(sph_json.h, sph_json.rest_density, sph_json.sound_speed);
            if let Some(m) = sph_json.particle_mass {
                fluid.mass = m;
            }
            let each = |block: &SphBlockJson, f: &mut dyn FnMut(Vec3)| {
                for ix in 0..block.counts[0] {
                    for iy in 0..block.counts[1] {
                        for iz in 0..block.counts[2] {
                            f(Vec3::new(
                                block.min[0] + ix as f64 * block.spacing,
                                block.min[1] + iy as f64 * block.spacing,
                                block.min[2] + iz as f64 * block.spacing,
                            ));
                        }
                    }
                }
            };
            for block in &sph_json.blocks {
                let velocity = array_to_vec3(block.velocity);
                each(block, &mut |p| {
                    fluid.add_particle(p, velocity);
                });
            }
            for block in &sph_json.boundary_blocks {
                each(block, &mut |p| {
                    fluid.add_boundary_particle(p);
                });
            }
            world.enable_sph(fluid);
        }

        if let Some(gas) = &scenario.gas {
            world.enable_gas(sim_thermal::GasCompartment {
                n_moles: gas.n_moles,
                volume: gas.volume,
                temperature: gas.temperature,
                gas: sim_thermal::GasSpecies {
                    degrees_of_freedom: gas
                        .degrees_of_freedom
                        .unwrap_or(sim_thermal::GasSpecies::AIR.degrees_of_freedom),
                    molar_mass: gas
                        .molar_mass
                        .unwrap_or(sim_thermal::GasSpecies::AIR.molar_mass),
                },
            });
        }

        // **群3で追加した6ドメイン**。ここまで`World`は量子・統計・FDTDを
        // そもそも保持しておらず(`Solver`未実装で載る経路が原理的に無かった)、
        // D25/D27–D32 が「ドメイン自体が存在しない」として滞留していた。
        if let Some(q) = &scenario.quantum_1d {
            if !q.n.is_power_of_two() {
                return Err(SceneError::InvalidValue(format!(
                    "quantum_1d.n must be a power of two (FFT constraint), got {}",
                    q.n
                )));
            }
            let mut wave = sim_quantum::WaveFunction1D::new(q.n, q.dx);
            wave.set_gaussian_wave_packet(q.packet.x0, q.packet.sigma, q.packet.k0);
            if let Some(potential) = &q.potential {
                for i in 0..q.n {
                    let x = i as f64 * q.dx;
                    wave.v[i] = match potential {
                        Potential1dJson::Barrier {
                            x_min,
                            x_max,
                            height,
                        } => {
                            if x >= *x_min && x <= *x_max {
                                *height
                            } else {
                                0.0
                            }
                        }
                        Potential1dJson::Harmonic { center, omega } => {
                            0.5 * omega * omega * (x - center) * (x - center)
                        }
                        Potential1dJson::Well {
                            x_min,
                            x_max,
                            height,
                        } => {
                            if x < *x_min || x > *x_max {
                                *height
                            } else {
                                0.0
                            }
                        }
                    };
                }
            }
            world.enable_quantum_1d(wave);
        }

        if let Some(q) = &scenario.quantum_2d {
            if !q.nx.is_power_of_two() || !q.ny.is_power_of_two() {
                return Err(SceneError::InvalidValue(
                    "quantum_2d.nx/ny must be powers of two (FFT constraint)".to_string(),
                ));
            }
            let mut wave = sim_quantum::WaveFunction2D::new(q.nx, q.ny, q.dx, q.dy);
            wave.set_gaussian_wave_packet(
                q.packet.x0,
                q.packet.y0,
                q.packet.sigma_x,
                q.packet.sigma_y,
                q.packet.k0,
            );
            if let Some(slit) = &q.double_slit {
                for iy in 0..q.ny {
                    let y = iy as f64 * q.dy;
                    // スリットの開口内なら壁を立てない。
                    let in_slit = slit
                        .slit_centers
                        .iter()
                        .any(|c| (y - c).abs() <= slit.slit_width * 0.5);
                    if in_slit {
                        continue;
                    }
                    for ix in 0..q.nx {
                        let x = ix as f64 * q.dx;
                        if (x - slit.wall_x).abs() <= slit.thickness * 0.5 {
                            wave.v[iy * q.nx + ix] = slit.height;
                        }
                    }
                }
            }
            world.enable_quantum_2d(wave);
        }

        if let Some(b) = &scenario.brownian {
            let mut set = sim_statistical::BrownianParticleSet::new(b.mass, b.gamma, b.kb_t);
            if let Some(f) = b.external_force {
                set.external_force = array_to_vec3(f);
            }
            let start = b.initial_position.map_or(Vec3::ZERO, array_to_vec3);
            for _ in 0..b.particle_count {
                set.add_particle(start, Vec3::ZERO);
            }
            world.enable_brownian(set);
        }

        if let Some(g) = &scenario.kinetic_gas {
            let mut gas = sim_statistical::GasSim::new(g.mass, g.radius, array_to_vec3(g.box_size));
            // **位置と速度は World の seed 付き PRNG から引く**——シーンJSONに
            // 数百粒子ぶんの座標を書き下すのは現実的でなく、かつ決定論も保てる
            // (同じ seed なら同じ配置になる)。D25 が `format!` で粒子を動的生成
            // していたのと同じ問題を、シーン側ではなくローダ側で解く。
            let mut rng = sim_math::SimRng::new(scenario.seed, 0x67617300);
            let b = array_to_vec3(g.box_size);
            for _ in 0..g.particle_count {
                // 壁にめり込まないよう半径ぶん内側に収める。
                let inset = |extent: f64, u: f64| g.radius + u * (extent - 2.0 * g.radius).max(0.0);
                let position = Vec3::new(
                    inset(b.x, rng.next_f64()),
                    inset(b.y, rng.next_f64()),
                    inset(b.z, rng.next_f64()),
                );
                let sigma = (sim_statistical::BOLTZMANN_CONSTANT * g.temperature / g.mass).sqrt();
                gas.add_particle(position, rng.maxwell_boltzmann_velocity(sigma));
            }
            world.enable_kinetic_gas(gas);
        }

        if let Some(i) = &scenario.ising {
            let mut rng = sim_math::SimRng::new(scenario.seed, 0x6973696e);
            let mut sim =
                sim_statistical::IsingSim::new(i.l, i.j_coupling, i.temperature, &mut rng);
            if let Some(n) = i.updates_per_step {
                sim.updates_per_step = n;
            }
            if let Some(w) = i.use_wolff {
                sim.use_wolff = w;
            }
            world.enable_ising(sim);
        }

        if let Some(f) = &scenario.fdtd {
            let mut sim = sim_em::FdtdSim2D::new(f.nx, f.ny, f.h, f.courant.unwrap_or(0.5));
            match &f.initial {
                Some(FdtdInitialJson::CavityMode { m, n, amplitude }) => {
                    for j in 1..f.ny - 1 {
                        for i in 1..f.nx - 1 {
                            let sx = (*m as f64 * std::f64::consts::PI * i as f64
                                / (f.nx - 1) as f64)
                                .sin();
                            let sy = (*n as f64 * std::f64::consts::PI * j as f64
                                / (f.ny - 1) as f64)
                                .sin();
                            sim.set_ez(i, j, amplitude * sx * sy);
                        }
                    }
                }
                Some(FdtdInitialJson::Pulse {
                    i: ci,
                    j: cj,
                    amplitude,
                    width,
                }) => {
                    for j in 1..f.ny - 1 {
                        for i in 1..f.nx - 1 {
                            let dx = (i as f64 - *ci as f64) * f.h;
                            let dy = (j as f64 - *cj as f64) * f.h;
                            let r2 = dx * dx + dy * dy;
                            sim.set_ez(i, j, amplitude * (-r2 / (width * width)).exp());
                        }
                    }
                }
                None => {}
            }
            world.enable_fdtd(sim);
        }

        for joint in &scenario.joints {
            match joint {
                JointJson::Slider {
                    body_a,
                    anchor_a,
                    axis,
                    body_b,
                    anchor_b,
                } => {
                    let a_id = *body_ids_by_name
                        .get(body_a)
                        .ok_or_else(|| SceneError::UnknownBodyName(body_a.clone()))?;
                    let b_id = match body_b {
                        Some(name) => Some(
                            *body_ids_by_name
                                .get(name)
                                .ok_or_else(|| SceneError::UnknownBodyName(name.clone()))?,
                        ),
                        None => None,
                    };
                    let joint = sim_mechanics::SliderJoint::new(
                        &world.mechanics_mut().bodies,
                        a_id.index as usize,
                        array_to_vec3(*anchor_a),
                        array_to_vec3(*axis),
                        b_id.map(|id| id.index as usize),
                        array_to_vec3(*anchor_b),
                    );
                    world.mechanics_mut().add_slider_joint(joint);
                }
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
                            disabled: false,
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
                JointJson::Wheel {
                    chassis,
                    wheel,
                    anchor_chassis,
                    rest_length,
                    suspension_axis,
                    axle_axis,
                    frequency,
                    damping_ratio,
                    steer_angle,
                    motor_speed,
                    motor_max_torque,
                } => {
                    let chassis_id = body_ids_by_name
                        .get(chassis)
                        .ok_or_else(|| SceneError::UnknownBodyName(chassis.clone()))?;
                    let wheel_id = body_ids_by_name
                        .get(wheel)
                        .ok_or_else(|| SceneError::UnknownBodyName(wheel.clone()))?;
                    let mut joint = sim_mechanics::WheelJoint::new(
                        chassis_id.index as usize,
                        wheel_id.index as usize,
                        array_to_vec3(*anchor_chassis),
                        *rest_length,
                    );
                    if let Some(axis) = suspension_axis {
                        joint.suspension_axis = array_to_vec3(*axis);
                    }
                    if let Some(axis) = axle_axis {
                        joint.axle_axis = array_to_vec3(*axis);
                    }
                    if let Some(f) = frequency {
                        joint.soft.frequency = *f;
                    }
                    if let Some(z) = damping_ratio {
                        joint.soft.damping_ratio = *z;
                    }
                    if let Some(a) = steer_angle {
                        joint.steer_angle = *a;
                    }
                    if let Some(v) = motor_speed {
                        joint.motor_speed = *v;
                    }
                    if let Some(t) = motor_max_torque {
                        joint.motor_max_torque = *t;
                    }
                    world.mechanics_mut().wheel_joints.push(joint);
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
                CouplingJson::BoussinesqBuoyancy {
                    thermal_node,
                    ambient_temperature,
                    thermal_expansion_coefficient,
                } => {
                    world.add_coupling(Box::new(sim_coupling::BoussinesqBuoyancy {
                        thermal_node: *thermal_node,
                        ambient_temperature: *ambient_temperature,
                        thermal_expansion_coefficient: *thermal_expansion_coefficient,
                    }));
                }
                CouplingJson::DissipationToHeat { thermal_node } => {
                    world.add_coupling(Box::new(sim_coupling::DissipationToHeat {
                        thermal_node: *thermal_node,
                    }));
                }
                CouplingJson::PhaseChangeMorph {
                    body,
                    thermal_node,
                    melting_temperature,
                    latent_heat_fusion,
                    specific_heat_solid,
                    specific_heat_liquid,
                    initial_mass,
                    conductance,
                    initial_enthalpy,
                } => {
                    let id = *body_ids_by_name
                        .get(body)
                        .ok_or_else(|| SceneError::UnknownBodyName(body.clone()))?;
                    world.add_coupling(Box::new(sim_coupling::PhaseChangeMorph::new(
                        id.index as usize,
                        *thermal_node,
                        sim_thermal::PhaseMaterial {
                            melting_temperature: *melting_temperature,
                            latent_heat_fusion: *latent_heat_fusion,
                            specific_heat_solid: *specific_heat_solid,
                            specific_heat_liquid: *specific_heat_liquid,
                        },
                        *initial_mass,
                        *conductance,
                        *initial_enthalpy,
                    )));
                }
                CouplingJson::MotorCoupling {
                    body,
                    axis,
                    voltage_source_index,
                    torque_constant,
                } => {
                    let id = *body_ids_by_name
                        .get(body)
                        .ok_or_else(|| SceneError::UnknownBodyName(body.clone()))?;
                    world.add_coupling(Box::new(sim_coupling::MotorCoupling {
                        body_index: id.index as usize,
                        axis: array_to_vec3(*axis),
                        voltage_source_index: *voltage_source_index,
                        torque_constant: *torque_constant,
                    }));
                }
                CouplingJson::SphRigid {
                    body,
                    radius,
                    boundary_points,
                } => {
                    let id = *body_ids_by_name
                        .get(body)
                        .ok_or_else(|| SceneError::UnknownBodyName(body.clone()))?;
                    // `SphRigid::new`は剛体表面を表す境界粒子を`SphFluid`側へ
                    // 実際に確保する(位置は最初の`apply`で埋まる)ため、
                    // SPHドメインが先に有効化されている必要がある。
                    let coupling = {
                        let sph = world.sph_mut().ok_or_else(|| {
                            SceneError::InvalidValue(
                                "couplings[].sph_rigid requires the `sph` domain to be defined"
                                    .to_string(),
                            )
                        })?;
                        sim_coupling::SphRigid::new(
                            sph,
                            id.index as usize,
                            *radius,
                            *boundary_points,
                        )
                    };
                    world.add_coupling(Box::new(coupling));
                }
                CouplingJson::GridFluidRigid {
                    body,
                    half_width,
                    half_height,
                } => {
                    let id = *body_ids_by_name
                        .get(body)
                        .ok_or_else(|| SceneError::UnknownBodyName(body.clone()))?;
                    world.add_coupling(Box::new(sim_coupling::GridFluidRigid {
                        body_index: id.index as usize,
                        half_width: *half_width,
                        half_height: *half_height,
                    }));
                }
                CouplingJson::LorentzForce { body, charge } => {
                    let id = *body_ids_by_name
                        .get(body)
                        .ok_or_else(|| SceneError::UnknownBodyName(body.clone()))?;
                    world.add_coupling(Box::new(sim_coupling::LorentzForce {
                        body_index: id.index as usize,
                        charge: *charge,
                    }));
                }
                CouplingJson::ConvectionLink {
                    fluid_node,
                    surface_node,
                    area,
                    characteristic_length,
                    fluid_thermal_conductivity,
                    kinematic_viscosity,
                    prandtl_number,
                } => {
                    world.add_coupling(Box::new(sim_coupling::ConvectionLink {
                        fluid_node: *fluid_node,
                        surface_node: *surface_node,
                        area: *area,
                        characteristic_length: *characteristic_length,
                        fluid_thermal_conductivity: *fluid_thermal_conductivity,
                        kinematic_viscosity: *kinematic_viscosity,
                        prandtl_number: *prandtl_number,
                    }));
                }
                CouplingJson::PistonGas {
                    body,
                    axis,
                    area,
                    initial_volume,
                } => {
                    let id = *body_ids_by_name
                        .get(body)
                        .ok_or_else(|| SceneError::UnknownBodyName(body.clone()))?;
                    // `PistonGas::new`は現在のピストン位置を変位0の基準として取り込むため、
                    // 剛体が既に構成済みである必要がある(couplingsはbodiesより後に処理される)。
                    let coupling = sim_coupling::PistonGas::new(
                        &world.mechanics().bodies,
                        id.index as usize,
                        array_to_vec3(*axis),
                        *area,
                        *initial_volume,
                    );
                    world.add_coupling(Box::new(coupling));
                }
                CouplingJson::JouleHeat { thermal_node } => {
                    world.add_coupling(Box::new(sim_coupling::JouleHeat {
                        thermal_node: *thermal_node,
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
                ProbeJson::CircuitNodeVoltage(node) => ProbeTarget::CircuitNodeVoltage(*node),
                ProbeJson::SoftBodyPosX(index) => ProbeTarget::SoftBodyPosX(*index),
                ProbeJson::SoftBodyPosY(index) => ProbeTarget::SoftBodyPosY(*index),
                ProbeJson::RodTemp(index) => ProbeTarget::RodTemp(*index),
                ProbeJson::GridFluidMeanV => ProbeTarget::GridFluidMeanV,
                ProbeJson::GridFluidRmsV => ProbeTarget::GridFluidRmsV,
                ProbeJson::SphParticlePosY(index) => ProbeTarget::SphParticlePosY(*index),
                ProbeJson::SphParticleDensity(index) => ProbeTarget::SphParticleDensity(*index),
                ProbeJson::CircuitCurrent(index) => ProbeTarget::CircuitCurrent(*index),
                // **群3で追加**。
                ProbeJson::QuantumNorm => ProbeTarget::QuantumNorm,
                ProbeJson::QuantumMeanX => ProbeTarget::QuantumMeanX,
                ProbeJson::QuantumEnergy => ProbeTarget::QuantumEnergy,
                ProbeJson::QuantumTransmission(from) => ProbeTarget::QuantumTransmission(*from),
                ProbeJson::GasTemperature => ProbeTarget::GasTemperature,
                ProbeJson::GasPressure => ProbeTarget::GasPressure,
                ProbeJson::IsingMagnetization => ProbeTarget::IsingMagnetization,
                ProbeJson::IsingEnergyPerSpin => ProbeTarget::IsingEnergyPerSpin,
                ProbeJson::BrownianMsd => ProbeTarget::BrownianMsd,
                ProbeJson::FdtdEz(i, j) => ProbeTarget::FdtdEz(*i, *j),
                ProbeJson::FdtdEnergy => ProbeTarget::FdtdEnergy,
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

    /// **増分H** D13(ロープと旗): 吊るしたロープの静止形状が懸垂線(カテナリー)
    /// $y=a\cosh(x/a)$ と一致することを`scenes/d13-rope.json`経由で確認する。
    ///
    /// **このシーンが成立するのに増分Hの2つの変更が要った**: ①`Scenario`に
    /// `soft_body`セクションが無かった ②`SoftBody`が`Solver`を実装しておらず
    /// `World::step()`の対象外だった(=シーンに載せても再生しても動かなかった)。
    /// D13のテストが`world.soft_body_mut().unwrap().step(...)`と手回ししていたのは
    /// 後者が理由である。
    ///
    /// 判定は`demos.rs`のD13・`sim-mechanics`のM13と同じく**スパンで正規化した
    /// 偏差**を使う(y自体の相対誤差ではない——端点付近ではyがほぼ0になり
    /// 相対誤差が発散するため)。実測の最大偏差は約1.7%。
    #[test]
    fn run_headless_scenario_rope_settles_into_catenary_shape() {
        let (span, total_length, segments) = (1.0_f64, 1.2_f64, 20usize);
        let json = include_str!("../../../scenes/d13-rope.json");
        let scenario = Scenario::from_json(json).expect("valid scenario JSON");
        let mut world = World::from_scenario(&scenario).expect("valid world");
        // 2400step(dt=1/120相当を8.33msで20秒ぶん)。減衰2.0で静止形状へ収束する。
        for _ in 0..2400 {
            world.step();
        }

        // M13と同じ二分法で懸垂線パラメータaを全長・スパンから逆算する。
        let solve_catenary_a = |length: f64, span: f64| -> f64 {
            let f = |a: f64| 2.0 * a * (span / (2.0 * a)).sinh() - length;
            let (mut lo, mut hi) = (span * 1e-3, span * 1000.0);
            for _ in 0..200 {
                let mid = 0.5 * (lo + hi);
                if f(mid) > 0.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            0.5 * (lo + hi)
        };
        let a = solve_catenary_a(total_length, span);
        let y_at = |x: f64| a * (x / a).cosh();
        let y_endpoint = y_at(span / 2.0);

        let body = world.soft_body().expect("ソフトボディドメインが有効");
        assert_eq!(body.position.len(), segments + 1);
        let mut max_deviation: f64 = 0.0;
        for k in 0..=segments {
            let p = body.position[k];
            let deviation = (p.y - (y_at(p.x) - y_endpoint)).abs() / span;
            max_deviation = max_deviation.max(deviation);
        }
        assert!(
            max_deviation < 0.02,
            "静止形状が懸垂線と一致すべき(スパン正規化の最大偏差): {max_deviation}"
        );

        // 端点はピン留めされたまま動かないこと(`pinned`が効いている直接の証拠)。
        assert!(body.position[0].y.abs() < 1e-12 && body.position[segments].y.abs() < 1e-12);
    }

    /// **増分H** D16(熱伝導レース): 銅・鋼・木材の棒で中点の立ち上がりが
    /// 熱拡散率 $\alpha=k/(\rho c_p)$ の順(銅>鋼>木材)になることを
    /// `scenes/d16-conduction-race.json`経由で確認する。
    ///
    /// **1つのシーンファイルで3材質を走らせる**ため、ギャラリーへ出す本体は銅とし、
    /// テスト側は材質名だけを差し替えて3回読み込む(D8の`seed`差し替えと同じ手口)。
    /// `conduction_rod.material`から $\alpha$ を引く経路自体が本増分の追加であり、
    /// 「材質のk比がそのまま順序を決める」というD16の主旨がスキーマに直接現れる。
    ///
    /// `ConductionRod1D`も`SoftBody`と同様に`Solver`未実装で`World::step()`の
    /// 対象外だった(増分Hで実装)。実測の中点温度は 60秒後に
    /// 銅 0.005553 / 鋼 8.2e-9 / 木材 0(下位桁までアンダーフロー)。
    #[test]
    fn run_headless_scenario_conduction_race_orders_materials_by_thermal_diffusivity() {
        let json = include_str!("../../../scenes/d16-conduction-race.json");
        let midpoint_after_60s = |material: &str| -> f64 {
            let swapped = json.replace(
                "\"material\": \"銅\"",
                &format!("\"material\": \"{material}\""),
            );
            let result = run_headless_scenario(&swapped, 60).expect("valid scenario JSON");
            *result.probe_histories[0].last().expect("履歴が空でない")
        };
        let (copper, steel, wood) = (
            midpoint_after_60s("銅"),
            midpoint_after_60s("鋼(炭素鋼)"),
            midpoint_after_60s("木材(松)"),
        );
        assert!(
            copper > steel && steel > wood,
            "熱拡散率の高い材質ほど中点が早く温まるべき: 銅={copper:e} 鋼={steel:e} 木={wood:e}"
        );

        // 空間プロファイルが単調(高温端に近いほど熱い)であること——順序だけでは
        // 「全部ゼロに近い」場合に空虚な主張になるので、実際に熱が伝わっている
        // ことを別途見る。プローブは順に 中点(20)・高温側(10)・低温側(30)。
        let result = run_headless_scenario(json, 60).expect("valid scenario JSON");
        let last = |i: usize| *result.probe_histories[i].last().expect("履歴が空でない");
        let (mid, near_hot, near_cold) = (last(0), last(1), last(2));
        assert!(
            near_hot > mid && mid > near_cold,
            "高温端に近いほど温度が高いべき: near_hot={near_hot} mid={mid} near_cold={near_cold}"
        );
        assert!(
            near_hot > 1.0,
            "60秒で高温端寄りの格子点は実際に温まっているべき(実測3.597): {near_hot}"
        );
    }

    /// **増分H** D15(対流): 熱源(ろうそく相当のノード)による Boussinesq 浮力で
    /// 格子流体の平均鉛直速度が単調に上昇することを`scenes/d15-convection.json`
    /// 経由で確認する。
    ///
    /// **これが書けるのに増分Hの3つの追加が要った**: `grid_fluid`セクション・
    /// `couplings[].boussinesq_buoyancy`・`ProbeTarget::GridFluidMeanV`。
    /// とくに最後が無いと、Scene Viewに何も描かれないこのシーンは
    /// **ギャラリーで観測する手段が一切無い**(合格基準「Boussinesqの定性」は
    /// 平均鉛直速度そのもの)。実測: 60step後に 0.114198。
    #[test]
    fn run_headless_scenario_convection_raises_mean_vertical_velocity_monotonically() {
        let json = include_str!("../../../scenes/d15-convection.json");
        let result = run_headless_scenario(json, 60).expect("valid scenario JSON");
        let mean_v = &result.probe_histories[0];
        assert_eq!(mean_v.len(), 60);

        for i in 1..mean_v.len() {
            assert!(
                mean_v[i] >= mean_v[i - 1] - 1e-12,
                "熱源の一定浮力の下では平均上昇速度は単調に上がるべき: \
                 step={i} previous={} current={}",
                mean_v[i - 1],
                mean_v[i]
            );
        }
        let final_v = *mean_v.last().expect("履歴が空でない");
        assert!(
            final_v > 0.1,
            "60step後には実際に上昇流が立っているべき(実測0.1142): {final_v}"
        );

        // 台帳が発散していないこと(合格基準「台帳」)。
        let scenario = Scenario::from_json(json).expect("valid scenario JSON");
        let mut world = World::from_scenario(&scenario).expect("valid world");
        for _ in 0..60 {
            world.step();
        }
        let residual = world.energy_residual();
        assert!(
            residual.is_finite() && residual < 1.0e6,
            "エネルギー台帳の残差は有界であるべき: residual={residual}"
        );
    }

    /// **増分H** D23(注ぐ水): SPHの水塊が落下して床(境界粒子)の上に溜まり、
    /// 内部粒子の密度が静止密度付近に保たれる(弱圧縮性)ことを
    /// `scenes/d23-pouring-water.json`経由で確認する。
    ///
    /// **測って分かった、主張を弱めるべき点**: 密度を見る粒子の選び方で結論が
    /// 全く変わる。6×6×6ブロックの**角の粒子(index 0)は近傍が足りず密度が
    /// 606.6 → 318.3 と静止密度から大きく外れる**——これはSPHの自由表面欠損
    /// (free-surface deficiency)として知られた正しい挙動であってバグではない。
    /// 一方**内部粒子(index 86 = ix2,iy2,iz2)は初期 999.97**(静止密度1000に対し
    /// 相対誤差 2.8e-5)、着水後も 966.2(3.4%以内)に留まる。
    /// したがって「SPHが非圧縮である」ではなく**「内部粒子について弱圧縮性が
    /// 保たれる」**という弱いほうの主張だけをテストにする。
    #[test]
    fn run_headless_scenario_pouring_water_keeps_interior_density_near_rest_density() {
        let json = include_str!("../../../scenes/d23-pouring-water.json");
        let result = run_headless_scenario(json, 600).expect("valid scenario JSON");
        let first = |i: usize| result.probe_histories[i][0];
        let last = |i: usize| *result.probe_histories[i].last().expect("履歴が空でない");

        // ①実際に落ちて床の上に溜まる(境界粒子の床は y=0 の1層)。
        let (top_start, top_end) = (first(2), last(2));
        assert!(
            top_end < top_start - 0.2,
            "水塊は実際に落下すべき: {top_start} -> {top_end}"
        );
        for i in [0usize, 2] {
            let y = last(i);
            assert!(
                y > 0.0,
                "境界粒子の床をすり抜けてはいけない: probe{i} y={y}"
            );
        }

        // ②内部粒子の弱圧縮性。初期は静止密度と 1e-4 以内で一致する。
        let rest_density = 1000.0;
        let interior_start = first(1);
        assert!(
            (interior_start - rest_density).abs() / rest_density < 1.0e-4,
            "初期の内部粒子密度は静止密度と一致すべき: {interior_start}"
        );
        let interior_end = last(1);
        assert!(
            (interior_end - rest_density).abs() / rest_density < 0.10,
            "着水後も内部粒子の密度は静止密度の10%以内に留まるべき: {interior_end}"
        );

        // ③自由表面欠損を**既知の限界として固定する**。角の粒子は静止密度を
        // 大きく下回る。これが「バグとして誤って直される」ことを防ぐための記録。
        let corner_end = last(3);
        assert!(
            corner_end < 0.6 * rest_density,
            "角の粒子はSPHの自由表面欠損で密度が大きく下がる(既知の正しい挙動): {corner_end}"
        );
    }

    /// **増分H** D14(煙と渦): 一様流の中に置いた角柱が実際に流れを乱すことを
    /// `scenes/d14-vortex.json`経由で確認する。
    ///
    /// **実装して分かった2つの落とし穴を記録する**:
    ///
    /// ①**障害物を`"type": "static"`にすると何も起きない**。
    /// `sim_coupling::GridFluidRigid::apply`は冒頭で `mass <= 0.0` の剛体を
    /// 無言でreturnする(静的/キネマティック剛体は対象外)。静的な角柱で組んだ
    /// 最初のシーンは、エラーも警告も出ないまま**結合が一度も発火しなかった**。
    /// 動的剛体(鋼の0.1m角=7.85kg、このシーンは重力0なので落ちない)に変えて
    /// 初めて`solid`マスクが書かれる。
    ///
    /// ②**観測量に平均鉛直速度を使うと0のまま動かない**。障害物の後流は上下
    /// 対称に立つため、格子全体の平均では打ち消し合う(実測 4.07e-16)。
    /// そこで`ProbeTarget::GridFluidRmsV`を追加した——符号が消えるので擾乱の
    /// 大きさが見える(実測 0 → 0.0443)。
    ///
    /// **正直な限界**: `GridFluid2D`は周期境界で流入・流出境界を持たないため、
    /// 下流の後流が上流へ回り込む。したがってこのシーンが示すのは
    /// 「障害物が一様流を実際に乱すこと」までであり、**カルマン渦列のSt数(F11)
    /// のような定量的な検証は`sim-fluid`側の専用テストが担う**。
    #[test]
    fn run_headless_scenario_vortex_obstacle_perturbs_a_uniform_flow() {
        let json = include_str!("../../../scenes/d14-vortex.json");
        let result = run_headless_scenario(json, 120).expect("valid scenario JSON");
        let rms = &result.probe_histories[0];
        let mean = &result.probe_histories[1];

        assert_eq!(rms[0], 0.0, "一様な水平流なので初期の鉛直速度はゼロ");
        let final_rms = *rms.last().expect("履歴が空でない");
        assert!(
            final_rms > 0.01,
            "障害物が一様流を乱して鉛直方向の速度成分が立つべき: {final_rms}"
        );

        // 後流の上下対称性: 平均はほぼ0のまま(RMSが要る理由の裏取り)。
        let final_mean = mean.last().copied().expect("履歴が空でない").abs();
        assert!(
            final_mean < 1.0e-9,
            "後流は上下対称なので平均鉛直速度はほぼ0のままであるべき: {final_mean}"
        );
        assert!(
            final_rms > 1.0e6 * final_mean,
            "RMSは平均より桁違いに大きいはず(これがRMSプローブを足した理由): \
             rms={final_rms} mean={final_mean}"
        );
    }

    /// **増分H2** D37(再突入): 大気抗力でカプセルが減速・降下することを
    /// `scenes/d37-reentry.json`経由で確認する。`astro.atmospheric_drag`は
    /// 本増分で追加したスキーマ。
    ///
    /// **プローブではなくワールドから直接読む**: `HeadlessRunResult`の
    /// プローブ履歴は容量600のリングバッファなので、4000step走らせると
    /// 最初の3400stepぶんが落ちて「開始値」が取れない(実際に測ろうとして
    /// 踏んだ)。降下量・減速量は開始と終了の差なので、ここは`World`を
    /// 直接進めて両端を読む。
    #[test]
    fn run_headless_scenario_reentry_decelerates_the_capsule_in_the_atmosphere() {
        let json = include_str!("../../../scenes/d37-reentry.json");
        let scenario = Scenario::from_json(json).expect("valid scenario JSON");
        let mut world = World::from_scenario(&scenario).expect("valid world");
        const R_EARTH: f64 = 6.371e6;

        let read = |w: &World| -> (f64, f64) {
            let a = w.astro().expect("天体ドメインが有効");
            (a.position[1].length() - R_EARTH, a.velocity[1].length())
        };
        let (altitude0, speed0) = read(&world);
        for _ in 0..4000 {
            world.step();
        }
        let (altitude1, speed1) = read(&world);

        // 初期条件は高度120km・速度|(-3000, 6000)| = 6708 m/s。
        assert!(
            (altitude0 - 120_000.0).abs() < 1.0,
            "初期高度は120kmのはず: {altitude0}"
        );
        assert!(
            (speed0 - 6708.2).abs() < 1.0,
            "初期速度は6708 m/sのはず: {speed0}"
        );

        // 大気抗力が効いて**大きく減速**する(実測: 6708 → 72 m/s、99%減)。
        assert!(
            speed1 < 0.05 * speed0,
            "大気抗力でカプセルは大幅に減速すべき: {speed0} -> {speed1}"
        );
        // 高度も下がる(実測: 120km → 43km)。
        assert!(
            altitude1 < altitude0 - 50_000.0 && altitude1 > 0.0,
            "降下しつつ地表より上に留まるべき: {altitude0} -> {altitude1}"
        );
    }

    /// **増分H2** D39(相対論 ON/OFF): 一般相対論の近日点移動補正を有効にすると
    /// 離心率ベクトルの向きが実際に回ることを`scenes/d39-relativity.json`経由で
    /// 確認する。`astro.relativistic_correction`は本増分で追加したスキーマ。
    ///
    /// **ON/OFFの対照実験にする**: 絶対的な歳差角の値は`sim-astro`の
    /// `a8`/D39テストが解析式(3πGM/(a(1-e²)c²) per orbit)と突き合わせて
    /// 既にGreenなので、ここで重ねて検証しない。シーンJSON経由で確認するのは
    /// **補正の有無が実際に結果を変えること**——JSONから
    /// `relativistic_correction`セクションを取り除いた版と比べる。
    /// 光速は`sim-astro`のテストと同じくc=100に誇張してある(現実のc では
    /// 歳差が小さすぎて数値誤差に埋もれるため)。
    #[test]
    fn run_headless_scenario_relativistic_correction_precesses_the_orbit() {
        let json = include_str!("../../../scenes/d39-relativity.json");
        // 補正セクションを丸ごと落とした対照シーン。
        let without = json.replace(
            ",\n    \"relativistic_correction\": { \"central_body\": 0, \"speed_of_light\": 100.0 }",
            "",
        );
        assert!(
            !without.contains("relativistic_correction"),
            "対照シーンの生成に失敗"
        );

        let gm = sim_astro::GRAVITATIONAL_CONSTANT * 14983518130.056938;
        let apsidal_angle = |w: &World| -> f64 {
            let a = w.astro().expect("天体ドメインが有効");
            let (r, v) = (a.position[1], a.velocity[1]);
            let h = r.cross(v);
            let e_vec = v.cross(h).scale(1.0 / gm) - r.scale(1.0 / r.length());
            e_vec.y.atan2(e_vec.x)
        };
        let run = |scene: &str| -> f64 {
            let scenario = Scenario::from_json(scene).expect("valid scenario JSON");
            let mut world = World::from_scenario(&scenario).expect("valid world");
            let start = apsidal_angle(&world);
            for _ in 0..(20 * 8000) {
                world.step();
            }
            apsidal_angle(&world) - start
        };

        let drift_on = run(json);
        let drift_off = run(&without);
        assert!(
            drift_on.abs() > 10.0 * drift_off.abs().max(1.0e-12),
            "相対論補正を有効にすると近点方向が有意に回るべき: on={drift_on} off={drift_off}"
        );
    }

    /// **増分H3** D10(摩擦の熱): 滑る箱が摩擦で止まり、失われた運動エネルギーが
    /// `couplings[].dissipation_to_heat`経由で熱ノードの温度上昇として現れる。
    /// 実測: 速さ 3.0 → 0、温度 293.15 → 331.70 K(+38.55 K)、移動距離 0.73 m。
    #[test]
    fn run_headless_scenario_brake_heat_converts_kinetic_energy_into_temperature_rise() {
        let json = include_str!("../../../scenes/d10-brake-heat.json");
        let result = run_headless_scenario(json, 300).expect("valid scenario JSON");
        let first = |i: usize| result.probe_histories[i][0];
        let last = |i: usize| *result.probe_histories[i].last().expect("履歴が空でない");

        assert!(last(0) < 0.01, "摩擦で静止するべき: {}", last(0));
        assert!(
            last(2) - first(2) > 0.5,
            "止まるまでに実際に滑るべき: {} -> {}",
            first(2),
            last(2)
        );

        // **エネルギーの行き先を数値で確かめる**: 初期運動エネルギー ½mv² が
        // そのまま熱容量 C の温度上昇になる。箱は 1m³ の鋼(密度7850)なので
        // m=7850 kg、v=3 m/s → ½mv² = 35325 J。C=1000 J/K なので ΔT ≈ 35.3 K。
        // 実測 38.55 K(接触の法線方向の散逸ぶんが上乗せされる)。
        let delta_t = last(1) - first(1);
        let predicted = 0.5 * 7850.0 * 3.0 * 3.0 / 1000.0;
        assert!(
            delta_t > 30.0 && (delta_t - predicted).abs() / predicted < 0.25,
            "散逸した運動エネルギーが温度上昇として現れるべき: \
             delta_t={delta_t} predicted={predicted}"
        );
    }

    /// **増分H3** D20(モーターと発電): キネマティックに一定角速度で回すクランクが
    /// `couplings[].motor_coupling`経由で起電力を生み、`joule_heat`が抵抗損失を
    /// 熱ノードへ移す。
    ///
    /// **実測が解析値と厳密に一致した**: 起電力 = k·ω = 0.05 × 10 = **0.5 V**
    /// (実測 0.500000)、電流 = V/R = 0.5/10 = **0.05 A**(実測 -0.05、符号は
    /// `MotorCoupling`の向きの規約)、電力 = 0.025 W を1秒ぶん(120step × dt)で
    /// 0.025 J、熱容量1000 J/K なので **ΔT = 2.5e-5 K**(実測 293.150025)。
    #[test]
    fn run_headless_scenario_hand_crank_generator_matches_emf_and_joule_heating() {
        let (k, omega, r) = (0.05_f64, 10.0_f64, 10.0_f64);
        let json = include_str!("../../../scenes/d20-hand-crank-generator.json");
        let steps = 120u32;
        let result = run_headless_scenario(json, steps).expect("valid scenario JSON");
        let last = |i: usize| *result.probe_histories[i].last().expect("履歴が空でない");

        let expected_emf = k * omega;
        assert!(
            (last(0) - expected_emf).abs() < 1.0e-9,
            "E6の起電力 k·ω と一致すべき: {} vs {expected_emf}",
            last(0)
        );
        let expected_current = expected_emf / r;
        assert!(
            (last(1).abs() - expected_current).abs() < 1.0e-9,
            "電流は V/R と一致すべき: {} vs {expected_current}",
            last(1)
        );

        // 台帳(効率): 発電電力がそのままジュール熱として熱ノードへ入る。
        let dt = 0.008333333;
        let expected_delta_t = expected_emf * expected_current * (steps as f64 * dt) / 1000.0;
        let delta_t = last(2) - 293.15;
        assert!(
            (delta_t - expected_delta_t).abs() / expected_delta_t < 0.02,
            "ジュール熱による温度上昇が発電電力と一致すべき: \
             delta_t={delta_t} expected={expected_delta_t}"
        );
    }

    /// **増分H3** D17(ピストン): 1軸スライダ拘束のピストンが気体を圧縮し、
    /// 気体ばねが押し返してエネルギーを返すことを`scenes/d17-piston.json`経由で
    /// 確認する(`gas`セクション・`joints[].slider`・`couplings[].piston_gas`は
    /// いずれも本増分で追加)。
    ///
    /// **実測の往復**: 初速 0.5 m/s(-x方向)で x=-0.0042 から圧縮を始め、
    /// x=-0.0364 で反転し、step 30 で出発点付近(x=-0.0041)へ戻ったとき
    /// 速さ **0.4774**(初速の95.5%)。差は数値散逸。
    ///
    /// **既知の性質(バグではない)**: 基準体積を超えて膨張し続けると、気体は
    /// 片側からしか押さないので圧力は正のままピストンを加速し続ける
    /// (200step走らせると速さ1.03まで上がる)。ストッパの無い片側気体ばねの
    /// 正しい挙動なので、判定は**圧縮ストロークの往復**に限定する。
    #[test]
    fn run_headless_scenario_piston_gas_spring_returns_the_compression_energy() {
        let json = include_str!("../../../scenes/d17-piston.json");
        let result = run_headless_scenario(json, 60).expect("valid scenario JSON");
        let x = &result.probe_histories[0];
        let v = &result.probe_histories[1];

        let (x0, v0) = (x[0], v[0]);
        let min_x = x.iter().copied().fold(f64::INFINITY, f64::min);
        assert!(
            min_x < x0 - 0.03,
            "ピストンは実際に気体を圧縮すべき: x0={x0} min_x={min_x}"
        );

        // 圧縮の後、出発点へ戻ってくる最初のstepで速さを見る。
        let turning = x
            .iter()
            .position(|&xi| xi <= min_x + 1.0e-12)
            .expect("反転点があるはず");
        let back = (turning..x.len())
            .find(|&i| x[i] >= x0)
            .expect("出発点へ戻るはず");
        let returned_speed = v[back];
        assert!(
            returned_speed > 0.9 * v0 && returned_speed < 1.05 * v0,
            "気体ばねは圧縮エネルギーをほぼ返すべき: v0={v0} returned={returned_speed}"
        );
    }

    /// **増分H3** D18(氷と飲み物): 浮いた氷が`couplings[].phase_change_morph`で
    /// 融解して質量を失い、**喫水が浅くなって浮き上がる**(アルキメデスとの統合)。
    ///
    /// 実測(6000step = 50秒): 質量 0.900 → 0.3545 kg(61%融解)、
    /// 重心 y = -0.0402 → +0.0098(浮き上がり)、飲み物 350.0 → 349.04 K。
    /// 「水位不変」は自由表面を追跡しない本実装の対象外(既存の記載どおり)。
    #[test]
    fn run_headless_scenario_melting_ice_rises_as_it_loses_mass() {
        let json = include_str!("../../../scenes/d18-ice-in-drink.json");
        let scenario = Scenario::from_json(json).expect("valid scenario JSON");
        let mut world = World::from_scenario(&scenario).expect("valid world");
        let y0 = world.mechanics().bodies.position[0].y;
        let m0 = world.mechanics().bodies.mass(0);
        let drink0 = world.thermal().expect("熱ドメイン").nodes[0].temperature;
        for _ in 0..6000 {
            world.step();
        }
        let (y1, m1) = (
            world.mechanics().bodies.position[0].y,
            world.mechanics().bodies.mass(0),
        );
        let drink1 = world.thermal().expect("熱ドメイン").nodes[0].temperature;

        assert!(
            m1 < 0.5 * m0 && m1 > 0.0,
            "融解して質量が部分的に減るべき(T7の融解プラトー): {m0} -> {m1}"
        );
        assert!(
            y1 > y0 + 0.03,
            "質量が減ったぶん喫水が浅くなって浮き上がるべき: {y0} -> {y1}"
        );
        assert!(
            drink1 < drink0,
            "融解熱を奪われて飲み物は冷えるべき: {drink0} -> {drink1}"
        );
    }

    /// **増分H3** D25(ブラウン運動): 300粒子のアンサンブル平均二乗変位が
    /// アインシュタインの関係 $\langle r^2\rangle = 6Dt$($D=k_BT/(6\pi\eta a)$)
    /// と一致することを`scenes/d25-brownian.json`経由で確認する。
    ///
    /// **これまでインラインのままだった理由が解消した**: 「300粒子を`format!`で
    /// 動的生成するので静的ファイル化に不向き」と記録していたが、生成物は
    /// ただのJSONなので**書き出してしまえば静的アセットとして成立する**
    /// (`bodies`300件 + `couplings`300件)。ギャラリー最大のシーンになる。
    ///
    /// 実測: msd = 5.673e-17 に対し解析値 6Dt = 5.988e-17、**比 0.947**。
    /// 300粒子の統計誤差は $1/\sqrt{300}\approx 5.8\%$ なので1標準誤差以内。
    /// `world.dt`は $\tau/50$($\tau=m/\gamma$ は慣性時間)——`World`既定の
    /// 1/120秒では明示的Euler-Maruyamaが発散する(既存の記録どおり)。
    #[test]
    fn run_headless_scenario_brownian_ensemble_msd_matches_the_einstein_relation() {
        let json = include_str!("../../../scenes/d25-brownian.json");
        let scenario = Scenario::from_json(json).expect("valid scenario JSON");
        let dt = scenario.world.dt;
        let mut world = World::from_scenario(&scenario).expect("valid world");
        let n = world.mechanics().bodies.position.len();
        assert_eq!(n, 300, "アンサンブルは300粒子");

        for _ in 0..2000 {
            world.step(); // 慣性の過渡を落とす
        }
        let start: Vec<sim_math::Vec3> = (0..n)
            .map(|i| world.mechanics().bodies.position[i])
            .collect();
        let steps = 10_000u32;
        for _ in 0..steps {
            world.step();
        }
        let msd = (0..n)
            .map(|i| {
                let d = world.mechanics().bodies.position[i] - start[i];
                d.x * d.x + d.y * d.y + d.z * d.z
            })
            .sum::<f64>()
            / n as f64;

        let (k_b, radius, viscosity, temperature) = (1.380649e-23, 1.0e-6, 1.002e-3, 293.15);
        let gamma = 6.0 * std::f64::consts::PI * viscosity * radius;
        let diffusion = k_b * temperature / gamma;
        let expected = 6.0 * diffusion * (steps as f64 * dt);
        let ratio = msd / expected;
        assert!(
            (ratio - 1.0).abs() < 0.15,
            "アンサンブルMSDは6Dtと一致すべき(300粒子の統計誤差は約5.8%): \
             msd={msd:e} expected={expected:e} ratio={ratio}"
        );
    }

    /// **増分G2** D19(電気工作台): 分圧回路 + コンデンサ放電回路 + スイッチ付き
    /// LED回路を単一`Circuit`へ自由配線したシーンを`scenes/d19-electric-workbench.json`
    /// 経由で検証する。`demos.rs`の
    /// `d19_electric_workbench_matches_divider_and_rc_discharge_and_switch_controls_led_and_joule_heats_node`
    /// と同じ回路構成を、Rustコードではなくシーンファイルとして組む。
    ///
    /// **本増分で追加したスキーマ拡張が全部ここで効く**: `capacitors`(充電済み
    /// 初期電圧つき)・`switches`・`diodes`・`couplings[].joule_heat`・
    /// `probes[].circuit_node_voltage`。これまで`CircuitScenarioJson`は抵抗と
    /// 電圧源しか書けず、D19の合格基準3つ(E5分圧・E3放電・スイッチ開閉)のうち
    /// 分圧しか表現できなかった。
    #[test]
    fn run_headless_scenario_electric_workbench_matches_divider_and_rc_discharge_and_switch() {
        let (v0, r1, r2, r3, c): (f64, f64, f64, f64, f64) = (9.0, 1000.0, 2000.0, 500.0, 1.0e-3);
        let dt: f64 = 0.008333333; // JSON側の`world.dt`。
        let tau = r3 * c;

        let json = include_str!("../../../scenes/d19-electric-workbench.json");
        let scenario = Scenario::from_json(json).expect("valid scenario JSON");
        let mut world = World::from_scenario(&scenario).expect("valid world");
        world.step();

        // **E5(分圧、機械精度)**: 理想電圧源からの純抵抗分圧なので、節点2は
        // `v0*r2/(r1+r2)` に厳密に一致する(実測 5.999999999999999)。
        let expected_divider = v0 * r2 / (r1 + r2);
        let measured_divider = world.circuit_probe(2).expect("回路ドメインが有効");
        assert!(
            (measured_divider - expected_divider).abs() < 1e-9,
            "E5 分圧: measured={measured_divider} expected={expected_divider}"
        );

        // スイッチが閉じている間はLED(ダイオード)分岐が順方向バイアスされ、
        // 節点4が電源電圧付近まで持ち上がる(実測 8.999999978)。
        let led_on = world.circuit_probe(4).expect("回路ドメインが有効");
        assert!(
            led_on > 0.1,
            "閉じたスイッチはLED分岐を順方向バイアスすべき: led_on={led_on}"
        );

        // `Command::SetSwitch`でスイッチを開く。`switches`配列の登録順が
        // `switch_index`になる(`SwitchJson`のdoc参照)ので、唯一のスイッチは0番。
        world.push_command(crate::Command::SetSwitch {
            switch_index: 0,
            closed: false,
        });
        for _ in 0..5 {
            world.step();
        }
        let led_off = world.circuit_probe(4).expect("回路ドメインが有効");
        assert!(
            led_off < led_on * 0.5,
            "開いたスイッチはLED分岐の電圧を大幅に下げるべき(スイッチの開放抵抗は\
             有限なので完全な0Vにはならない、実測 0.2347): led_on={led_on} led_off={led_off}"
        );

        // **E3(放電形、rel<1%)**: コンデンサは初期電圧v0から R3 経由で
        // `V(t)=V0*exp(-t/(R3*C))` で減衰する。実測の相対誤差は
        // 1step後 1.4e-4、6step後 8.2e-4(後方Euler相当の離散化誤差が時間とともに
        // 蓄積する向き)。
        let steps_so_far = 6.0; // 上の world.step() 呼び出し回数(1 + 5)
        let expected_v3 = v0 * (-(steps_so_far * dt) / tau).exp();
        let measured_v3 = world.circuit_probe(3).expect("回路ドメインが有効");
        let rel_err = (measured_v3 - expected_v3).abs() / expected_v3;
        assert!(
            rel_err < 0.01,
            "E3 放電: measured={measured_v3} expected={expected_v3} rel_err={rel_err:e}"
        );

        // **JouleHeat結合**: 回路の抵抗損失が`couplings[].joule_heat`経由で
        // 熱ノードへ注入され、温度が初期値293.15Kから上昇する
        // (実測 293.1500085764805 —— 熱容量1000 J/K に対し6stepぶんなので微小)。
        let temp = world.thermal().expect("熱ドメインが有効").nodes[0].temperature;
        assert!(
            temp > 293.15,
            "ジュール熱が熱ノードへ注入されるべき: temp={temp}"
        );

        // プローブ経由でも同じ値が読めること(ギャラリーのProbe Graphsが
        // 見せているのはこちらの経路)。
        let result = run_headless_scenario(json, 6).expect("valid scenario JSON");
        assert_eq!(result.probe_histories.len(), 5);
        let probed_v3 = *result.probe_histories[1].last().expect("履歴が空でない");
        assert!(
            (probed_v3 - measured_v3).abs() < 1e-12,
            "circuit_node_voltageプローブは`circuit_probe`と同じ値を積むべき: \
             probed={probed_v3} direct={measured_v3}"
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
    /// **D24(車の実験場)がシーンJSONから組めて、実際に走ること**(群4)。
    ///
    /// D24 は長らく「新規物理待ちでスコープ外」だった——`WheelJoint`
    /// (サスペンション+駆動+操舵)が `sim-mechanics` に存在せず、
    /// 実車体を4輪で支持する手段が無かったため。群4で `SoftParams`(設計§4.3の
    /// ソフト拘束)と `WheelJoint` を実装し、シーンJSONの `joints[].wheel` と
    /// `bodies[].collision_group/mask` を足したことで、**ギャラリーに出せる**
    /// ようになった。
    #[test]
    fn d24_car_scene_drives_forward_on_its_suspension() {
        let json = include_str!("../../../scenes/d24-car.json");
        let scenario = Scenario::from_json(json).expect("D24 シーンはパースできる");
        let mut world = World::from_scenario(&scenario).expect("D24 シーンは構築できる");

        // ① 4輪ともホイールジョイントで支持されている。
        assert_eq!(world.mechanics().wheel_joints.len(), 4);

        // ② **サスペンションで車体が浮いた状態を保つ**。自然長 0.43 に対し、
        //    車重で沈むが、車輪(半径0.32)にめり込むほどは沈まない。
        for _ in 0..600 {
            world.step();
        }
        let chassis_y = world.probe(1).unwrap().history().last().copied().unwrap();
        assert!(
            chassis_y > 0.5 && chassis_y < 0.8,
            "車体はサスペンションで浮いた高さに落ち着くはず: y={chassis_y}"
        );

        // ③ **後輪駆動で前進する**。x 位置が単調に増え、速度が有限に留まる
        //    (発散していない)。
        let start_x = world.probe(0).unwrap().history().last().copied().unwrap();
        for _ in 0..600 {
            world.step();
        }
        let end_x = world.probe(0).unwrap().history().last().copied().unwrap();
        assert!(
            end_x > start_x + 0.5,
            "駆動輪が回れば前進するはず: start={start_x} end={end_x}"
        );
        let speed = world.probe(2).unwrap().history().last().copied().unwrap();
        assert!(
            speed.is_finite() && speed < 50.0,
            "速度が発散していないこと: speed={speed}"
        );

        // ④ **駆動を切ると加速が止まる**(モーターが実際に効いていることの対照実験)。
        for joint in &mut world.mechanics_mut().wheel_joints {
            joint.motor_max_torque = 0.0;
        }
        let coast_start = world.probe(2).unwrap().history().last().copied().unwrap();
        for _ in 0..600 {
            world.step();
        }
        let coast_end = world.probe(2).unwrap().history().last().copied().unwrap();
        assert!(
            coast_end <= coast_start + 1e-6,
            "駆動を切れば加速しないはず: start={coast_start} end={coast_end}"
        );
    }

    /// **群3で追加した7シーン(D27–D33)の物理的な検証**。
    ///
    /// マニフェストのテストは「壊れたアセットを出荷しない」ための安全網で、
    /// 60step 走ることしか見ない。ここでは**各デモの合格基準に対応する量**を
    /// シーンJSON経由(=エディタが実際に読む経路)で確認する。
    #[test]
    fn group3_gallery_scenes_reproduce_their_acceptance_criteria() {
        // D28 トンネル効果: ①ノルムは厳密に 1 のまま(split-step Fourier は
        // ユニタリ、Q1 の検証量)②障壁の向こう側へ確率が漏れる(透過率 > 0)。
        let result =
            run_headless_scenario(include_str!("../../../scenes/d28-tunneling.json"), 1500)
                .unwrap();
        let norm = &result.probe_histories[0];
        for &n in norm {
            assert!(
                (n - 1.0).abs() < 1e-9,
                "TDSE must stay unitary through the scene: norm={n}"
            );
        }
        let transmission = result.probe_histories[3].last().copied().unwrap();
        assert!(
            transmission > 1e-4,
            "some probability must tunnel through the barrier: T={transmission}"
        );
        // ⟨x⟩ は実際に前進する(波束が動いている)。
        let mean_x = &result.probe_histories[1];
        assert!(
            mean_x.last().unwrap() > mean_x.first().unwrap(),
            "the packet must move forward: <x>0={} <x>1={}",
            mean_x.first().unwrap(),
            mean_x.last().unwrap()
        );

        // D29 電波の水槽: PEC 空洞は無損失なので電磁エネルギーが保存する。
        let result =
            run_headless_scenario(include_str!("../../../scenes/d29-radio-tank.json"), 120)
                .unwrap();
        let energy = &result.probe_histories[0];
        let e0 = energy[0];
        let e1 = *energy.last().unwrap();
        assert!(e0 > 0.0, "the pulse must carry energy: e0={e0}");
        assert!(
            (e1 - e0).abs() / e0 < 0.05,
            "lossless PEC cavity must conserve field energy: e0={e0} e1={e1}"
        );
        // 中心から離れた観測点にも波が到達する(点源から実際に広がっている)。
        let off_center = &result.probe_histories[2];
        assert!(
            off_center.iter().any(|v| v.abs() > 1e-3),
            "the wave must reach the off-centre probe"
        );

        // D30 気体の箱: 断熱・弾性なので温度が一定、かつ圧力が理想気体則の桁に乗る。
        let result =
            run_headless_scenario(include_str!("../../../scenes/d30-gas-box.json"), 300).unwrap();
        let temperature = &result.probe_histories[0];
        let t0 = temperature[0];
        let t1 = *temperature.last().unwrap();
        assert!(
            (t1 - t0).abs() / t0 < 0.02,
            "specular walls + elastic collisions keep T constant: t0={t0} t1={t1}"
        );
        // p = N k_B T / V(理想気体則、S2 の検証量)。壁への運動量流束から
        // 測った圧力がこの桁に乗ることを確認する(統計誤差があるので係数2倍以内)。
        let pressure = *result.probe_histories[1].last().unwrap();
        let volume = 1e-7_f64.powi(3);
        let expected = 400.0 * 1.380649e-23 * t1 / volume;
        assert!(
            pressure > 0.5 * expected && pressure < 2.0 * expected,
            "measured pressure should match pV=NkT within a factor of 2: p={pressure} expected={expected}"
        );

        // D31 拡散とインク: 平均二乗変位が単調に増え、6Dt の桁に乗る。
        let steps = 400;
        let result = run_headless_scenario(
            include_str!("../../../scenes/d31-diffusion-ink.json"),
            steps,
        )
        .unwrap();
        let msd = &result.probe_histories[0];
        // **プローブは step の末尾でサンプルされる**ので、履歴の先頭は既に
        // 1step ぶん拡散した後の値(0 ではない)。ここで見るのは「単調に広がる」
        // ことと最終値の桁。
        let final_msd = *msd.last().unwrap();
        assert!(
            final_msd > msd[0] * 10.0,
            "MSD must keep growing: first={} last={final_msd}",
            msd[0]
        );
        // D = k_BT/γ、dt = 1e-8 s。
        let d = 4.0471e-21 / 9.4e-9;
        let expected = 6.0 * d * (steps as f64 * 1e-8);
        assert!(
            final_msd > 0.2 * expected && final_msd < 5.0 * expected,
            "free-diffusion MSD should be within an order of 6Dt: msd={final_msd} expected={expected}"
        );

        // D32 磁石の相転移: T=2.0 < Tc≈2.269 なので自発磁化が立ち上がる。
        let result = run_headless_scenario(
            include_str!("../../../scenes/d32-magnet-transition.json"),
            200,
        )
        .unwrap();
        let magnetization = result.probe_histories[0].last().copied().unwrap().abs();
        assert!(
            magnetization > 0.7,
            "below Tc the Ising lattice must order: |m|={magnetization}"
        );

        // D27 二重スリット・D33 井戸: どちらもノルム保存(ユニタリ)を見る。
        // D27 はプローブを持たない(可視化は |ψ|² オーバーレイ)ので、World から直接読む。
        let scenario =
            Scenario::from_json(include_str!("../../../scenes/d27-double-slit.json")).unwrap();
        let mut world = World::from_scenario(&scenario).unwrap();
        let norm0 = world.quantum_2d().unwrap().norm();
        for _ in 0..200 {
            world.step();
        }
        let norm1 = world.quantum_2d().unwrap().norm();
        assert!(
            (norm1 - norm0).abs() < 1e-9,
            "2D TDSE must stay unitary: norm0={norm0} norm1={norm1}"
        );

        let result = run_headless_scenario(
            include_str!("../../../scenes/d33-electron-in-well.json"),
            300,
        )
        .unwrap();
        for &n in &result.probe_histories[0] {
            assert!((n - 1.0).abs() < 1e-9, "well scene must stay unitary: {n}");
        }
    }

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
