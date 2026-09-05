//! wasm-bindgen バインディング。設計: docs/00-foundation/05-rust-wasm-platform.md §3。
//!
//! Phase 0 は同文書のシグネチャ例(`WasmWorld::from_scene_json/step/time/
//! body_transforms_f32/observables_json/state_hash/push_command_json`)を
//! 「箱1個が落ちる」規模に縮小したものを公開する。シーンJSON・コマンドキュー・
//! 観測値JSONはシーン記述(docs/20-integration/04-world-api.md §3)が実装され次第、
//! Phase A 以降で追加する。
//!
//! **複数ボディ対応(ワークストリームD増分)**: エディタのHierarchy/Inspector
//! (docs/23-frontend/01-editor.md §1.1・§1.3)が複数ボディを列挙・選択できることを
//! 実際に検証するため、床の静的平面(`Shape::Plane`)を追加した(箱は床の上で静止
//! するようになる、以前の「永遠に落下し続ける」挙動からの意図的な変更)。
//! `sim_world::World`自体は汎用的な「全ボディ列挙」APIを持たないため(`BodyId`は
//! 世代付きindexで、削除済みスロットとの区別に`World`内部の世代情報が必要)、
//! `WasmWorld`が自ら構築した固定2体(床・箱)+スポーンパレット(§6)で動的に
//! 追加されたボディ(`SpawnedBodyMeta`、`spawn_sphere`/`spawn_box`)をindexで
//! 列挙する縮約実装とした(シーンJSON経由で任意個のボディを構築できるように
//! なれば、`from_scenario`のボディリストをそのまま列挙する形に置き換える)。

use std::collections::{HashMap, VecDeque};

use js_sys::{Float32Array, Float64Array, Uint8Array};
use sim_core::FrameId;
use sim_math::{Quat, Vec3};
use sim_mechanics::{BallJoint, BodyType, HingeMotorPd, RigidBodyDesc, Shape};
use sim_world::{BodyId, Command, ProbeTarget, World, WorldOptions};
use wasm_bindgen::prelude::*;

mod component_schema;

/// Timelineパネルのスナップショットリングバッファ(設計docs/00-foundation/
/// 04-architecture.md §「巻き戻しのスナップショット予算」: 既定1s間隔・
/// リングバッファN=8面・直近8s分)。1s間隔は`dt`から算出する
/// (`WasmWorld::new`で`1.0/dt`を四捨五入)。
const SNAPSHOT_RING_CAPACITY: usize = 8;

/// 分圧回路(`Command::SetSwitch`実証用、`WasmWorld::new`参照)の分圧点ノード番号。
const CIRCUIT_DIVIDER_NODE: usize = 2;

/// 熱ノード(`Command::SetHeatSource`実証用、`WasmWorld::new`参照)のindex
/// (単一ノードのみのため常に0)。
const THERMAL_HEATER_NODE: usize = 0;

/// この境界crateの内部エラー型。**`JsValue`の構築を最外周(wasm-bindgenが
/// exportする`pub fn`)1点だけに閉じ込めるために導入した**。
///
/// **なぜ要ったか(テスト不能性という実害)**: 以前は各`*_impl`が
/// `Result<T, JsValue>`を直接返し、エラーは`JsValue::from_str(...)`で
/// 組み立てていた。ところが`wasm_bindgen::JsValue`はネイティブ
/// (非wasm32)ターゲットでは**値を構築した時点でプロセスごとabort(SIGABRT)
/// する**——unwindするパニックではないため`#[should_panic]`でも
/// `std::panic::catch_unwind`でも捕捉できず、`Result::is_err()`で受けても
/// 手遅れ(`Err`の中身を作る段階で既に落ちている)。このcrateのテストは
/// ほぼ全てネイティブの`#[cfg(test)] mod tests`で動くため、結果として
/// **147か所あるエラー構築のどれ一つとして`Err`パスを検証できなかった**
/// (本モジュール末尾のテストに「Errパスの検証にはwasm-bindgen-testが要る、
/// よって対象外」という趣旨のコメントが繰り返し現れていたのはこのため)。
///
/// `WasmError`は素のRust型なのでネイティブで自由に構築・比較でき、
/// `assert!(matches!(err, WasmError::BodyIndexOutOfRange { .. }))`のように
/// **エラーの種別そのもの**を検証できる。文字列を両側で組み立てて
/// 突き合わせる(同じ`format!`式が同じ文字列を作ることしか示さない)形を
/// 避けるため、意図的に判別可能なenumにした。
///
/// **JS側から見た挙動は一切変わらない**: `Display`が出す文字列は、以前
/// `JsValue::from_str`へ渡していたメッセージと1バイトも違わない。最外周の
/// `pub fn`が`impl From<WasmError> for JsValue`(下記)を通して同じ
/// `JsValue`文字列へ変換するため、wasm-bindgenがthrowするJS例外の中身も同じ。
#[derive(Clone, Debug, PartialEq)]
pub enum WasmError {
    // --- シーンJSON経路(`sim_world::SceneError`をそのまま保持する) ---
    // 以前はいずれも`format!("{e:?}")`で文字列へ潰していた。`SceneError`は
    // `Clone + Debug + PartialEq`なので、そのまま持てば
    // `matches!(err, WasmError::ScenarioParse(SceneError::UnknownMaterial(_)))`
    // のような一段深い検証まで書ける(`Display`は従来どおり`{e:?}`を出す)。
    /// `sim_world::Scenario::from_json`の失敗。
    ScenarioParse(sim_world::SceneError),
    /// `World::from_scenario_with_body_ids`の失敗。
    WorldBuild(sim_world::SceneError),
    /// `World::append_scenario_bodies`の失敗(`import_scene_json`経路)。
    AppendScenarioBodies(sim_world::SceneError),
    /// `World::add_scenario_probes`の失敗。
    ScenarioProbes(sim_world::SceneError),
    /// `sim_world::run_headless_scenario`の失敗(検証パネル経路)。
    HeadlessRun(sim_world::SceneError),
    /// `sim_world::to_scenario`の失敗(`export_scene_json`/
    /// `bookmark_export_scene_json`経路)。**生状態に非有限値がある**
    /// (=シミュレーションが発散した)ときだけ起きる
    /// ——`sim_world::to_scenario`のdoc参照。下の`ScenarioSerializeFailed`が
    /// `serde_json`側の失敗であるのに対し、こちらは書き出す前の中身の異常である。
    ScenarioExport(sim_world::SceneError),
    /// `enable_quantum_1d_domain`/`enable_quantum_2d_domain`——プリセットUIが
    /// 組み立てた`psi_re`/`psi_im`/`v`が`sim_world::build_quantum_{1d,2d}_wave_from_raw`
    /// の検証(2の冪長・配列長一致)を通らない。シーンJSON経路と同じ`SceneError`を
    /// そのまま保持する(上の5変種と同じ理由)。
    QuantumRawStateInvalid(sim_world::SceneError),

    // --- serde_jsonのシリアライズ/デシリアライズ失敗(`serde_json::Error`は
    // `PartialEq`を持たないため、`Display`済みの文字列で保持する) ---
    /// `body_shape_json_at`のシリアライズ失敗。
    ShapeSerializeFailed(String),
    /// `spawn_shape_json`が受け取った形状JSONが`ShapeJson`として読めない
    /// (**Prefabの任意形状対応で追加**)。上の3つと違い**実際に踏める**
    /// ——JS側が組み立てた文字列がそのまま渡ってくるため。
    ShapeParseFailed(String),
    /// `export_scene_json`/`bookmark_export_scene_json`のシリアライズ失敗。
    ScenarioSerializeFailed(String),
    /// `run_headless_scenario_json`の結果シリアライズ失敗。
    HeadlessResultSerializeFailed(String),
    /// `sketch_extrude_shape_json`が受け取ったリクエストJSONが読めない
    /// (**D1(スケッチ・押し出し)で追加**)。`ShapeParseFailed`と同じく
    /// JS側が組み立てた文字列がそのまま渡ってくるため**実際に踏める**。
    SketchRequestParseFailed(String),

    // --- index範囲外(いずれも「JS側から渡ってくる値」であり、生indexアクセスに
    // 使う前に必ずここで弾く。`try_body_id_at`のdoc参照) ---
    /// `try_body_id_at`——`self.bodies`の範囲外。
    BodyIndexOutOfRange { index: usize, count: usize },
    /// `try_body_meta_at`——同じく範囲外だが、**メッセージに件数を含めない**
    /// 従来の文面をそのまま保つため`BodyIndexOutOfRange`とは別変種にした。
    BodyMetaIndexOutOfRange { index: usize },
    /// 範囲内だが`World`側で既に死んでいる(削除済み、または巻き戻した
    /// スナップショットより後に作られた)`BodyId`を指している。
    BodyNoLongerExists { index: usize },
    /// `try_imported_probe_handle_at`——`imported_probe_handles`の範囲外。
    ImportedProbeIndexOutOfRange { index: usize, count: usize },
    /// `try_imported_probe_at`——handleに対応する`World::probe`が無い
    /// (World側にprobe削除経路が無いため、実際には到達しない想定)。
    ImportedProbeHandleMissing { handle: usize },
    /// `circuit_element_label_at`——回路素子の通し番号が範囲外。
    CircuitElementIndexOutOfRange { index: usize, count: usize },
    /// `try_thermal_node_index`——熱ノードindexが範囲外(熱ドメイン自体が
    /// 無効なら`count==0`になり、同じ変種で表される)。
    ThermalNodeIndexOutOfRange { index: usize, count: usize },
    /// `try_voltage_source_index`——電圧源indexが範囲外(回路ドメインが
    /// 無効なら`count==0`)。
    VoltageSourceIndexOutOfRange { index: usize, count: usize },
    /// `check_frame_index`——フレームindexが範囲外。
    FrameIndexOutOfRange { index: usize, count: usize },
    /// `try_snapshot_at`/`restore_snapshot`——スナップショットindexが範囲外。
    SnapshotIndexOutOfRange { index: usize, count: usize },
    /// `try_bookmark_at`/`restore_bookmark`——ブックマークindexが範囲外。
    BookmarkIndexOutOfRange { index: usize, count: usize },
    /// `coupling_supported_params`——結合の登録indexが範囲外(**Task#9**)。
    CouplingIndexOutOfRange { index: usize, count: usize },

    // --- 材質 ---
    /// `MaterialDb::find_by_name`が引けなかった材質名。
    UnknownMaterial(String),
    /// `derive_material`——派生先の名前が既に存在する。
    MaterialAlreadyExists(String),

    // --- ドメインが有効でない(適用対象が無い) ---
    /// 回路ドメインが無効(`circuit_element_label_at`)。
    CircuitDomainNotEnabled,
    /// SPH流体ドメインが無効(`add_sph_rigid_coupling`)。
    SphDomainNotEnabled,
    /// 格子流体ドメインが無効(`add_grid_fluid_rigid_coupling`/
    /// `add_boussinesq_buoyancy_coupling`)。
    GridFluidDomainNotEnabled,
    /// 気体区画が無効(`add_piston_gas_coupling`)。
    GasCompartmentNotEnabled,

    // --- 値そのものが不正 ---
    /// `set_dt`——dtが正の有限値でない。
    InvalidDt,
    /// `derive_material`——密度が正の有限値でない。
    InvalidDensity,
    /// `push_set_body_mass`——質量が正の有限値でない。
    InvalidMass,
    /// `set_body_scale_xyz_at`——スケール成分が正の有限値でない。
    InvalidScaleComponent,
    /// `push_set_body_type`——Dynamic/Static/Kinematic以外の名前。
    UnknownBodyType(String),
    /// `push_set_gravity_field`——uniform/point_source/zero以外の`kind`名。
    UnknownGravityFieldKind,
    /// `sketch_extrude_shape_json`——union/subtract/intersect以外のブーリアン演算名
    /// (**D1で追加**)。
    UnknownBooleanOp(String),
    /// `sketch_extrude_shape_json`——押し出す断面が無い(妥当なスケッチが
    /// 1枚も無い、またはブーリアン合成の結果が空になった)。
    SketchProfileEmpty,
    /// `sketch_extrude_shape_json`——断面はあるが押し出せない(深さが正の
    /// 有限値でない、三角形分割が1枚も作れない)。
    SketchExtrudeFailed,

    // --- 対象に対してその操作が成り立たない ---
    /// 床(index 0)は削除できない。
    CannotRemoveFloor,
    /// 削除済みボディは複製できない(位置が引けない)。
    CannotDuplicateRemovedBody,
    /// `set_motor_target_at`——そのボディはヒンジモーターを持たない。
    BodyHasNoHingeMotor { index: usize },
    /// 床(index 0)にスケールハンドルは無い(`set_body_scale_at`系)。
    GroundHasNoScaleHandle,

    // --- `apply_component`/`read_component`のディスパッチ ---
    /// `apply_component`のpayloadがJSONとして読めない。
    ApplyComponentInvalidJson(String),
    /// `apply_component`が知らないkind名。
    UnknownApplyComponentKind(String),
    /// `read_component`が知らないkind名。
    UnknownReadComponentKind(String),
}

/// **ここで出す文字列は、以前`JsValue::from_str`へ渡していたものと
/// 1バイトも違わない**(`WasmError`のdoc「JS側から見た挙動は一切変わらない」)。
/// 文面を整えたくなっても変えてはいけない——フロントエンドがメッセージを
/// そのままConsoleパネルへ出しており、リファクタで表示が変わるのは
/// 「振る舞いを変えない」という本改修の前提に反する。
impl std::fmt::Display for WasmError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            // シーンJSON経路は従来どおり`SceneError`のDebug表現をそのまま出す
            // (`format!("{e:?}")`を`JsValue::from_str`へ渡していた形と同じ)。
            WasmError::ScenarioParse(e)
            | WasmError::WorldBuild(e)
            | WasmError::AppendScenarioBodies(e)
            | WasmError::ScenarioProbes(e)
            | WasmError::HeadlessRun(e)
            | WasmError::ScenarioExport(e)
            | WasmError::QuantumRawStateInvalid(e) => write!(f, "{e:?}"),

            WasmError::ShapeSerializeFailed(e) => write!(f, "failed to serialize shape: {e}"),
            WasmError::ShapeParseFailed(e) => write!(f, "failed to parse shape json: {e}"),
            WasmError::ScenarioSerializeFailed(e) => write!(f, "failed to serialize scenario: {e}"),
            WasmError::HeadlessResultSerializeFailed(e) => {
                write!(f, "failed to serialize result: {e}")
            }
            WasmError::SketchRequestParseFailed(e) => {
                write!(f, "failed to parse sketch request json: {e}")
            }

            WasmError::BodyIndexOutOfRange { index, count } => {
                write!(f, "body index {index} out of range (body_count={count})")
            }
            WasmError::BodyMetaIndexOutOfRange { index } => {
                write!(f, "body index {index} out of range")
            }
            WasmError::BodyNoLongerExists { index } => write!(
                f,
                "body index {index} no longer exists in the current World state \
                 (removed, or created after the currently restored Timeline snapshot)"
            ),
            WasmError::ImportedProbeIndexOutOfRange { index, count } => write!(
                f,
                "imported probe index {index} out of range (imported_probe_count={count})"
            ),
            WasmError::ImportedProbeHandleMissing { handle } => write!(
                f,
                "imported probe handle {handle} has no matching World::probe (World-side removal is not implemented, this should not happen)"
            ),
            WasmError::CircuitElementIndexOutOfRange { index, count } => write!(
                f,
                "circuit element index {index} out of range (circuit_element_count={count})"
            ),
            WasmError::ThermalNodeIndexOutOfRange { index, count } => write!(
                f,
                "thermal node index {index} out of range (thermal node count={count}, \
                 is the thermal domain enabled in this scene?)"
            ),
            WasmError::VoltageSourceIndexOutOfRange { index, count } => write!(
                f,
                "voltage source index {index} out of range (voltage source count={count}, \
                 is the circuit domain enabled in this scene?)"
            ),
            WasmError::FrameIndexOutOfRange { index, count } => {
                write!(f, "frame index {index} out of range (frame_count={count})")
            }
            WasmError::SnapshotIndexOutOfRange { index, count } => write!(
                f,
                "snapshot index {index} out of range (snapshot_count={count})"
            ),
            WasmError::BookmarkIndexOutOfRange { index, count } => write!(
                f,
                "bookmark index {index} out of range (bookmark_count={count})"
            ),
            // **Task#9で新設した変種**。上のdocが釘を刺す「文面を変えては
            // いけない」対象は、以前`JsValue::from_str`へ渡していた既存
            // メッセージのこと——新設の変種に対応する旧文面は存在しないので、
            // 既存の範囲外系と同じ書式に揃える。
            WasmError::CouplingIndexOutOfRange { index, count } => write!(
                f,
                "coupling index {index} out of range (coupling_count={count})"
            ),

            WasmError::UnknownMaterial(name) => write!(f, "unknown material: {name}"),
            WasmError::MaterialAlreadyExists(name) => write!(f, "material already exists: {name}"),

            WasmError::CircuitDomainNotEnabled => {
                write!(f, "circuit domain is not enabled in the current world")
            }
            WasmError::SphDomainNotEnabled => write!(
                f,
                "SPH fluid domain is not enabled (spawn a fluid block first via \"+ 流体\")"
            ),
            WasmError::GridFluidDomainNotEnabled => write!(
                f,
                "grid fluid domain is not enabled (call enable_grid_fluid_2d_domain first)"
            ),
            WasmError::GasCompartmentNotEnabled => write!(
                f,
                "gas compartment is not enabled (call enable_gas_compartment first)"
            ),

            WasmError::InvalidDt => write!(f, "dt must be a positive finite number"),
            WasmError::InvalidDensity => write!(f, "density must be a positive finite number"),
            WasmError::InvalidMass => write!(f, "mass must be a positive finite number"),
            WasmError::InvalidScaleComponent => write!(f, "scale components must be positive"),
            // `{kind:?}`(引用符付き)は旧実装の`{other:?}`——`other`は`&str`
            // だった——と同じ出力になる(`String`のDebugは`str`のDebugと同一)。
            WasmError::UnknownBodyType(kind) => write!(
                f,
                "unknown body type {kind:?} (expected Dynamic/Static/Kinematic)"
            ),
            WasmError::UnknownGravityFieldKind => write!(
                f,
                "unknown gravity field kind (expected uniform/point_source/zero)"
            ),
            // スケッチ系の3つは**ユーザーの操作にそのまま起因する**(描き方が
            // 悪ければ普通に踏む)ので、他のwasm内部エラーと違って日本語で
            // 「次に何をすればよいか」まで書く——`CannotRemoveFloor`と同じ方針。
            WasmError::UnknownBooleanOp(op) => write!(
                f,
                "未知のブーリアン演算「{op}」(union/subtract/intersect のいずれか)"
            ),
            WasmError::SketchProfileEmpty => write!(
                f,
                "押し出せる断面がありません(閉じたスケッチが1枚も無いか、ブーリアン合成の結果が空になりました)"
            ),
            WasmError::SketchExtrudeFailed => write!(
                f,
                "押し出しに失敗しました(深さは正の値を指定してください)"
            ),

            WasmError::CannotRemoveFloor => write!(f, "床は削除できません(シーンの基準面)"),
            WasmError::CannotDuplicateRemovedBody => write!(f, "cannot duplicate a removed body"),
            WasmError::BodyHasNoHingeMotor { index } => {
                write!(f, "body index {index} has no hinge motor")
            }
            WasmError::GroundHasNoScaleHandle => {
                write!(f, "Ground is static and has no scale handle")
            }

            WasmError::ApplyComponentInvalidJson(e) => {
                write!(f, "apply_component: invalid JSON payload: {e}")
            }
            WasmError::UnknownApplyComponentKind(kind) => {
                write!(f, "apply_component: unknown kind \"{kind}\"")
            }
            WasmError::UnknownReadComponentKind(kind) => {
                write!(f, "read_component: unknown kind \"{kind}\"")
            }
        }
    }
}

impl std::error::Error for WasmError {}

/// **`JsValue`を作ってよい唯一の場所**。wasm-bindgenがexportする`pub fn`が
/// `?`もしくは`.map_err(JsValue::from)`でこれを通す——それ以外の場所で
/// `JsValue::from_str`を呼ぶと、`WasmError`のdocが述べたネイティブテスト
/// 不能性が再発する。
///
/// 変換後の`JsValue`は従来と同じ「メッセージ文字列そのもの」で、
/// wasm-bindgenはこれを通常の(捕捉可能な)JS例外としてthrowする
/// ——TypeScript側の呼び出し規約は`try_body_id_at`のdocが述べたまま変わらない。
impl From<WasmError> for JsValue {
    fn from(e: WasmError) -> JsValue {
        JsValue::from_str(&e.to_string())
    }
}

/// スポーンパレット(設計docs/23-frontend/01-editor.md §6「形状×材質を選んで
/// クリック配置」)で追加したボディの記録。Shapeは`World::mechanics().bodies.
/// shape_of`で実クエリできるが(`body_shape_label_at`参照)、Materialは`World`が
/// ボディからMaterialIdを引く公開APIを持たないため、スポーンした側(この構造体)
/// が構築時の材質名をそのまま覚えておく縮約実装のまま(固定2体(床・箱)の
/// ハードコードと同じ発想)。
struct SpawnedBodyMeta {
    id: BodyId,
    label: String,
    material_label: String,
    /// Scale Gizmo(`set_body_scale_at`参照)がスケール係数を掛ける基準形状
    /// (スポーン時の寸法、以後`World`側の実形状が変わってもこの基準は不変)。
    base_shape: Shape,
    /// 振り子スポーン(`spawn_pendulum`)が追加したDistanceJointの
    /// `World::distance_joint_anchor_points`用index。球/箱スポーンでは`None`
    /// (拘束オーバーレイ対象外)。
    constraint_joint_index: Option<usize>,
    /// モーターアームスポーン(`spawn_motor_arm`)が追加した`HingeMotorPd`の
    /// `MechanicsSolver::hinge_motors`内index(`Command::SetMotorTarget`の
    /// `hinge_motor_index`引数、`set_motor_target_at`参照)。振り子/球/箱
    /// スポーンでは`None`。
    hinge_motor_index: Option<usize>,
    /// `remove_body_at`で削除済みか(群2)。`Vec`から要素を取り除くとフロントの
    /// ボディindexが総ずれするため、削除は**このフラグで表す**
    /// (`World::remove_body`が下層スロットを詰めないのと同じ方針)。
    removed: bool,
}

/// `SpawnedBodyMeta::label`の既定接頭辞(`Sphere_3`の`Sphere`の部分)。
/// 固定レシピのスポナー(`spawn_sphere_impl`等)がリテラルで書いているのと
/// 同じ綴りを、任意形状スポナー(`spawn_shape_json_impl`)が形状から引く
/// ために切り出した。`body_shape_kind_at_impl`が返す小文字スネークケースの
/// **種別名とは別物**——あちらはJS側との外部規約なので、表示用ラベルの
/// 都合で綴りを変えられない。
fn shape_label_prefix(shape: &Shape) -> &'static str {
    match shape {
        Shape::Sphere { .. } => "Sphere",
        Shape::Box { .. } => "Box",
        Shape::Capsule { .. } => "Capsule",
        Shape::Plane { .. } => "Plane",
        Shape::Compound { .. } => "Compound",
        Shape::ConvexMesh { .. } => "ConvexMesh",
    }
}

/// **B16「ゼロコピーのメモリビューAPIへ統一」**: `apply_component`のdoc
/// 「正直な適用範囲」が対象外と明言した、毎フレーム呼ばれる型付き配列の
/// 読み出し系(`body_position_at_f32`等、レンダリングループのホットパス)向けの
/// 永続バッファ集。JSON文字列化を避けて`Float32Array`/`Float64Array`/
/// `Uint8Array`を直接返す設計自体はそのまま(そこは変えない狙いの取り組みでは
/// ない)——変えたのは**戻り値の作り方**。
///
/// 従来は呼び出しのたびに`Float32Array::from(&values[..])`等で**新しいJS側
/// 配列オブジェクトを割り当ててコピー**していた。60fpsのレンダリングループで
/// 本構造体が持つ約20種のアクセサを毎フレーム呼ぶと、使い捨ての型付き配列が
/// フレームごとに何十個も生成され、素通りするだけのGC圧になっていた。
///
/// この構造体のフィールドはアクセサ1個につき1本の永続`Vec`——各アクセサは
/// 呼び出しのたびに対応するフィールドを`clear`してから値を書き直し(値自体は
/// 毎フレーム変わるのでここは避けられない)、最後に`js_sys::XxxArray::view(&buf)`
/// (`unsafe`)でWasm線形メモリを直接指す一時的なビューを返す。JS側は新しい
/// 配列オブジェクトを割り当てずに済む代わりに、**返された配列を次にWasmへ
/// 呼び出す前に読み切る**という約束を守らなければならない
/// (`demo/src/main.ts`の呼び出し箇所のコメント参照)——`view()`はRustの`Vec`が
/// 指すメモリをエイリアスするだけなので、その後Wasm側で該当`Vec`が
/// (次にこのメソッドを呼んだときの`clear`+書き込みや、無関係な別の呼び出しで
/// Wasmのリニアメモリ自体が成長・移動することも含め)再確保されると、ビューは
/// 古い(場合によっては解放済みの)メモリを指したままになり、JS側は不定な値を
/// 読むか最悪OOBアクセスになる。
///
/// この不変条件を守るため、各アクセサ内では**バッファへの書き込みを完全に
/// 終えてから`view()`を呼び、`view()`の戻り値を関数の最後の式にする**
/// (=`view()`を作った後、同じ呼び出し内でその`buf`に触れる処理を続けない)。
#[derive(Default)]
struct HotPathViewBuffers {
    /// `frame_rotation_at_f32`用。
    frame_rotation: Vec<f32>,
    /// `frame_world_position_f32`用。
    frame_world_position: Vec<f32>,
    /// `frame_world_rotation_f32`用。
    frame_world_rotation: Vec<f32>,
    /// `body_position_at_f32`用。
    body_position: Vec<f32>,
    /// `body_velocity_at_f32`用。
    body_velocity: Vec<f32>,
    /// `body_rotation_at_f32`用。
    body_rotation: Vec<f32>,
    /// `constraint_anchor_points_at`用。
    constraint_anchor_points: Vec<f32>,
    /// `quantum_1d_density_f32`用。
    quantum_1d_density: Vec<f32>,
    /// `quantum_2d_density_f32`用。
    quantum_2d_density: Vec<f32>,
    /// `ising_spins_u8`用。
    ising_spins: Vec<u8>,
    /// `kinetic_gas_positions_f32`用。
    kinetic_gas_positions: Vec<f32>,
    /// `brownian_positions_f32`用。
    brownian_positions: Vec<f32>,
    /// `soft_body_positions_f32`用。
    soft_body_positions: Vec<f32>,
    /// `astro_positions_f32`用。
    astro_positions: Vec<f32>,
    /// `conduction_rod_temperatures_f32`用。
    conduction_rod_temperatures: Vec<f32>,
    /// `fluid_particle_positions_f32`用。
    fluid_particle_positions: Vec<f32>,
    /// `fluid_boundary_positions_f32`用。
    fluid_boundary_positions: Vec<f32>,
    /// `grid_fluid_3d_smoke_points_f32`用。
    grid_fluid_3d_smoke: Vec<f32>,
    /// `y_probe_history_f64`用。
    y_probe_history: Vec<f64>,
    /// `speed_probe_history_f64`用。
    speed_probe_history: Vec<f64>,
    /// `contact_points_f32`用。
    contact_points: Vec<f32>,
    /// `imported_probe_history_f64`用。
    imported_probe_history: Vec<f64>,
}

#[wasm_bindgen]
pub struct WasmWorld {
    inner: World,
    /// 全ボディの記録(**2026-07-27の残タスク完遂セッションで統合**:
    /// 以前は固定2体(`ground_body`/`box_body`)を専用フィールド、それ以降を
    /// `spawned: Vec<SpawnedBodyMeta>`という別々の表現にしていたが、シーン
    /// ギャラリー(`from_scene_json`)が任意個・任意種別のボディを一括構築できる
    /// ようにするため単一の`Vec`へ統合した。index 0=Ground・index 1=Box_1という
    /// 既存の意味は`new()`がこの順でpushすることでそのまま維持される)。
    bodies: Vec<SpawnedBodyMeta>,
    /// Probe Graphsパネルの既定2系列(箱のy座標・速さ)。**2026-07-28のD9/D34/D35
    /// 増分で`Option`化**: `WasmWorld::new`(既定シーン)は常に箱を持つため必ず
    /// `Some`だが、`from_scene_json`が読み込むギャラリーシーンは力学ボディを
    /// 1つも持たないことがある(D9=熱のみ、D34/D35=天体のみ)。ボディが無い
    /// シーンでは登録しようがないため`None`にする——`y_probe_history_f64`/
    /// `speed_probe_history_f64`はこの場合に空配列を返す(モジュールdoc参照)。
    y_probe: Option<usize>,
    speed_probe: Option<usize>,
    /// 分圧回路のスイッチ(`sim_em::Circuit::add_switch`が返すindex、
    /// `set_circuit_switch_closed`参照)。
    circuit_switch_index: usize,
    /// 自由配線回路エディタが`circuit_editor_add_dc_motor`で追加したDCモーターの
    /// ハンドル(登録順、`circuit_editor_set_motor_speed`/
    /// `circuit_editor_motor_current`のindexに使う)。固定デモ回路はモーターを
    /// 持たないため、`WasmWorld::new`の時点では空。
    circuit_editor_motors: Vec<sim_em::MotorHandle>,
    snapshot_interval_steps: u64,
    snapshots: VecDeque<World>,
    /// **巻き戻して眺めている位置**(`None` = 最新にいる)。
    ///
    /// 巻き戻した先より後のスナップショットは、そこから**進めた瞬間に**
    /// 実際の未来ではなくなる。以前は巻き戻したその場で捨てていたが、
    /// それだと止めたまま前後に行き来できず、スクラバを左へ引いて離すと
    /// つまみが右端へ戻る(=動いていないように見える)ことになっていた
    /// ——利用者役が2人続けて「マウスで動かせない」と書いた原因。
    /// 記録は残したまま位置だけ覚えておき、実際に進めるときに切り捨てる。
    restored_to: Option<usize>,
    bookmarks: Vec<(String, World)>,
    /// `spawn_fluid_block`を呼んだ回数(複数の水塊を並べてスポーンする際、
    /// 塊どうしが重ならないようX方向のオフセットを決めるのに使う。Hierarchyの
    /// 「Fluids」概要表示にも使う、`fluid_spawn_count`のdoc参照)。
    fluid_spawn_count: u32,
    /// 直近の`import_scene_json`呼び出しが`scenario.probes`から作成したプローブの
    /// ハンドル(`World::probe`用、`scenario.probes`と同じ順)。予測→実験ミニ
    /// パネル(`imported_probe_value_at`のdoc参照)がインポートしたシナリオの
    /// プローブ値を読むために使う。新規Importのたびに置き換わる(縮約実装、
    /// 複数回インポートした場合は最後のシナリオの`probes`のみ対象)。
    imported_probe_handles: Vec<usize>,
    /// 直近の`import_scene_json`が**JSONに書かれていたのに取り込まなかった**
    /// セクション名(QA不具合5)。Importは`materials`/`bodies`/`probes`しか
    /// 見ないため、それ以外は黙って捨てられていた——D10をImportしても
    /// `coupling_count`は0のままで、UI上は「2件のボディを追加しました」と
    /// 出るだけで**結合が落ちたことがユーザーに伝わらなかった**。
    /// `read_component("last_import_skipped_sections", "")`で読める。
    last_import_skipped_sections: Vec<String>,
    /// ホットパスの型付き配列アクセサが使い回す永続バッファ集
    /// (`HotPathViewBuffers`のdoc、B16参照)。
    view_buffers: HotPathViewBuffers,
}

/// シーンJSON Import が**書かれていたのに取り込まなかった**セクション名を列挙する
/// (QA不具合5)。
///
/// Import(`WasmWorld::import_scene_json`)が見るのは
/// `materials`/`bodies`(`World::append_scenario_bodies`)と`probes`だけで、
/// 残りは黙って捨てられる。捨てたことをユーザーへ伝えられるように、
/// **JSON に実際に書かれていたセクションのうち適用しなかったもの**を返す。
///
/// **なぜ「全部取り込む」ようにしないのか(正直な記録)**: Import は実行中の
/// ワールドへの「追加」であり、既に有効なドメインを無条件に上書きするのは
/// 意図しない挙動になりうる(`append_scenario_bodies`のdoc参照)。さらに
/// `couplings`は熱ノードindexや回路ノードindexを**そのシーンの番号体系で**
/// 参照するため、既存ワールドへ足すには番号の再割り当てが要る——
/// 「Import は現在のワールドへマージなのか差し替えなのか」という設計判断を
/// 伴う話で、この修正の範囲を超える。まずは**黙って落とさない**ことだけを
/// 保証する。
fn skipped_import_sections(scenario: &sim_world::Scenario) -> Vec<String> {
    let mut skipped: Vec<String> = Vec::new();
    let mut note = |present: bool, name: &str| {
        if present {
            skipped.push(name.to_string());
        }
    };
    note(!scenario.fluids.is_empty(), "fluids");
    note(scenario.thermal.is_some(), "thermal");
    note(!scenario.joints.is_empty(), "joints");
    note(!scenario.couplings.is_empty(), "couplings");
    note(scenario.circuit.is_some(), "circuit");
    note(scenario.astro.is_some(), "astro");
    note(scenario.soft_body.is_some(), "soft_body");
    note(scenario.grid_fluid.is_some(), "grid_fluid");
    note(scenario.conduction_rod.is_some(), "conduction_rod");
    note(scenario.sph.is_some(), "sph");
    note(scenario.gas.is_some(), "gas");
    note(scenario.brownian.is_some(), "brownian");
    note(scenario.kinetic_gas.is_some(), "kinetic_gas");
    note(scenario.ising.is_some(), "ising");
    note(scenario.fdtd.is_some(), "fdtd");
    skipped
}

#[wasm_bindgen]
impl WasmWorld {
    #[wasm_bindgen(constructor)]
    pub fn new(gravity: f64, dt: f64, initial_height: f64) -> WasmWorld {
        let options = WorldOptions {
            gravity,
            dt,
            seed: 0,
        };
        let mut inner = World::new(options);
        let concrete = inner
            .materials()
            .find_by_name("コンクリート")
            .expect("standard DB has concrete");
        let mut ground_desc = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: sim_math::Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            concrete,
        );
        ground_desc.body_type = BodyType::Static;
        let ground_body = inner.create_body(ground_desc);
        let ground_shape = Shape::Plane {
            normal: sim_math::Vec3::new(0.0, 1.0, 0.0),
            d: 0.0,
        };

        let steel = inner
            .materials()
            .find_by_name("鋼(炭素鋼)")
            .expect("standard DB has steel");
        let box_shape = Shape::Box {
            half_extents: sim_math::Vec3::new(0.5, 0.5, 0.5),
        };
        let mut desc = RigidBodyDesc::dynamic(box_shape.clone(), steel);
        desc.transform.position = sim_math::Vec3::new(0.0, initial_height, 0.0);
        let box_body = inner.create_body(desc);
        let y_probe = Some(inner.add_probe(ProbeTarget::BodyPosY(box_body)));
        let speed_probe = Some(inner.add_probe(ProbeTarget::BodySpeed(box_body)));

        // 分圧回路(`Command::SetSwitch`の実証用、`sim-world`の
        // `set_switch_command_closes_switch_and_changes_circuit_state`と同じ
        // 構成)。node 0=GND、1=電源(10V)、2=分圧点。スイッチは負荷抵抗(200Ω)と
        // 並列に接続し、閉じると分圧点をGNDへ短絡する(開: 6.67V、閉: 0V)。
        let mut circuit = sim_em::Circuit::new(3);
        circuit.add_voltage_source(1, sim_em::GROUND, 10.0);
        circuit.add_resistor(1, CIRCUIT_DIVIDER_NODE, 100.0);
        let circuit_switch_index = circuit.add_switch(CIRCUIT_DIVIDER_NODE, sim_em::GROUND, false);
        circuit.add_resistor(CIRCUIT_DIVIDER_NODE, sim_em::GROUND, 200.0);
        inner.enable_circuit(circuit);

        // 熱ノード(`Command::SetHeatSource`の実証用、`sim-world`の
        // `set_heat_source_command_raises_temperature_for_one_step_only`と
        // 同じ「1step分だけ効く」縮約セマンティクス)。ニュートン冷却
        // (`sim-thermal`のT1テストと同じc=100J/K・h=10W/(m^2K)・area=1m^2、
        // 時定数τ=c/(hA)=10s)ありの単一ノード、初期温度=周囲温度。
        let ambient_temperature = 293.15;
        let mut thermal = sim_thermal::ThermalSolver::new(ambient_temperature);
        let mut heater_node = sim_thermal::ThermalNode::new(ambient_temperature, 100.0);
        heater_node.convection_coefficient = 10.0;
        heater_node.area = 1.0;
        thermal.add_node(heater_node);
        inner.enable_thermal(thermal);

        let snapshot_interval_steps = (1.0 / dt).round().max(1.0) as u64;
        let bodies = vec![
            SpawnedBodyMeta {
                id: ground_body,
                label: "Ground".to_string(),
                material_label: "コンクリート".to_string(),
                base_shape: ground_shape,
                constraint_joint_index: None,
                hinge_motor_index: None,
                removed: false,
            },
            SpawnedBodyMeta {
                id: box_body,
                label: "Box_1".to_string(),
                material_label: "鋼(炭素鋼)".to_string(),
                base_shape: box_shape,
                constraint_joint_index: None,
                hinge_motor_index: None,
                removed: false,
            },
        ];
        WasmWorld {
            inner,
            bodies,
            y_probe,
            speed_probe,
            circuit_switch_index,
            circuit_editor_motors: Vec::new(),
            snapshot_interval_steps,
            snapshots: VecDeque::with_capacity(SNAPSHOT_RING_CAPACITY),
            restored_to: None,
            bookmarks: Vec::new(),
            fluid_spawn_count: 0,
            imported_probe_handles: Vec::new(),
            last_import_skipped_sections: Vec::new(),
            view_buffers: HotPathViewBuffers::default(),
        }
    }

    /// **残タスク完遂のシーンギャラリー増分**でシーンJSONから丸ごと`WasmWorld`を
    /// 構築するコンストラクタ。既存の`import_scene_json`(実行中ワールドへの
    /// 「追加」、`fluids`/`thermal`/`circuit`/`astro`セクションは対象外、
    /// `append_scenario_bodies`のdoc参照)とは異なり、`World::from_scenario`
    /// (`sim_world::Scenario`の**全セクション**を構成する)でワールドそのものを
    /// 差し替える。D1–D43のシーンJSON(ヘッドレスランナーが使うのと同じ
    /// スキーマ、`scenes/`ディレクトリのシーンギャラリー参照)をエディタへそのまま
    /// 読み込んで視覚的に確認できるようにするのが狙い(設計のワークストリームD
    /// 項目13)。既定コンストラクタ(`new`)固有のデモ用備品(分圧回路・ヒーター
    /// ノード)はこの経路では作らない——`circuit_switch_index`は無害なプレース
    /// ホルダ(0)のままにする(呼び出し側UIは分圧回路専用の「回路スイッチ」
    /// チェックボックスをこの経路では表示しない前提、`circuit_editor_reset`の
    /// doc「回路スイッチ(閉)チェックボックスを無効化する責任を負う」と同じ規約)。
    /// `y_probe`/`speed_probe`(Probe Graphsパネルの既定2系列)はシーン最初の
    /// ボディへ向ける——シーン定義プローブ(`scenario.probes`)がある場合は
    /// フロントエンド側がそちらを優先して表示する想定(後続増分)。
    ///
    /// **2026-07-28のD9/D34/D35増分で「ボディが1つも無いシーンを拒否する」
    /// ガードを撤廃した**: 元々この関数は`ids.first()`が`None`(=`bodies`が
    /// 空配列)なら`Err`を返していた——`y_probe`/`speed_probe`が「必ず先頭
    /// ボディへ向ける」設計だったための制約だが、これはD1–D26(いずれも
    /// `mechanics.bodies`を持つ)だけを相手にしていた頃の名残に過ぎない。
    /// D9(冷めるコーヒー、`thermal`+`probes`のみ)・D34/D35(太陽系儀/軌道投入、
    /// `astro`の質点のみ)は力学剛体を1つも持たない正当なシーンであり、
    /// シーン定義プローブ(増分B1で配線済みの`scenario.probes`/
    /// `imported_probe_*`)が既に系列を提供するため、`y_probe`/`speed_probe`が
    /// 無くても何も失われない。したがって「先頭ボディが無ければ登録しない」
    /// (`Option::None`)という設計にした——既定シーン(`WasmWorld::new`)は
    /// 常に箱を持つため挙動は変わらない。
    ///
    /// wasm-bindgenへ露出する薄い殻——実体は`from_scene_json_impl`側にあり、
    /// ここは`WasmError`を`JsValue`へ移すだけ(`WasmError`のdoc参照)。
    pub fn from_scene_json(json: String) -> Result<WasmWorld, JsValue> {
        Self::from_scene_json_impl(&json).map_err(JsValue::from)
    }

    /// `from_scene_json`の実体(ネイティブテスト可能な`Result<_, WasmError>`版)。
    fn from_scene_json_impl(json: &str) -> Result<WasmWorld, WasmError> {
        let scenario = sim_world::Scenario::from_json(json).map_err(WasmError::ScenarioParse)?;
        let (mut inner, ids) =
            World::from_scenario_with_body_ids(&scenario).map_err(WasmError::WorldBuild)?;
        let (y_probe, speed_probe) = match ids.first() {
            Some(&first_id) => (
                Some(inner.add_probe(ProbeTarget::BodyPosY(first_id))),
                Some(inner.add_probe(ProbeTarget::BodySpeed(first_id))),
            ),
            None => (None, None),
        };

        let mut body_ids_by_name: HashMap<String, BodyId> = HashMap::new();
        for (body, id) in scenario.bodies.iter().zip(ids.iter()) {
            if let Some(name) = &body.name {
                body_ids_by_name.insert(name.clone(), *id);
            }
        }
        let imported_probe_handles = inner
            .add_scenario_probes(&scenario, &body_ids_by_name)
            .map_err(WasmError::ScenarioProbes)?;

        let bodies = scenario
            .bodies
            .iter()
            .zip(ids.iter())
            .enumerate()
            .map(|(index, (body, &id))| {
                let label = body.name.clone().unwrap_or_else(|| format!("Body_{index}"));
                let shape = sim_world::shape_json_to_shape(&body.shape);
                SpawnedBodyMeta {
                    id,
                    label,
                    material_label: body.material.clone(),
                    base_shape: shape,
                    constraint_joint_index: None,
                    hinge_motor_index: None,
                    removed: false,
                }
            })
            .collect();

        let snapshot_interval_steps = (1.0 / scenario.world.dt).round().max(1.0) as u64;
        Ok(WasmWorld {
            inner,
            bodies,
            y_probe,
            speed_probe,
            circuit_switch_index: 0,
            circuit_editor_motors: Vec::new(),
            snapshot_interval_steps,
            snapshots: VecDeque::with_capacity(SNAPSHOT_RING_CAPACITY),
            restored_to: None,
            bookmarks: Vec::new(),
            fluid_spawn_count: 0,
            imported_probe_handles,
            last_import_skipped_sections: Vec::new(),
            view_buffers: HotPathViewBuffers::default(),
        })
    }

    /// Hierarchyパネルが列挙するボディ数(モジュールdoc「複数ボディ対応」参照)。
    fn body_count_impl(&self) -> usize {
        self.bodies.len()
    }

    /// `index`をボディIDへ解決する。範囲外なら`WasmError`を返す
    /// (**2026-07-27の監査で修正**: 以前は`panic!`していたが、この`index`は
    /// JS側から渡される値でありシーン再読み込み後の古い参照や単純な入力ミスで
    /// 容易に範囲外になり得る。wasmの`panic`はモジュール全体を使用不能にする
    /// ため——`console_error_panic_hook`を導入していない現状では捕捉不能な
    /// wasmトラップとしてJSに伝わり、以後同じ`WasmWorld`インスタンスへの呼び出しが
    /// 全て失敗し得る——`Result`によるエラー返却へ置き換えた。最外周の`pub fn`が
    /// `WasmError`を`JsValue`へ変換し、wasm-bindgenは
    /// `Result<T, JsValue>`を返すexport関数を、成功時は`T`をそのまま返し失敗時は
    /// 通常の(捕捉可能な)JS例外をthrowする形にバインドするため、TypeScript側の
    /// 呼び出し規約は変わらない)。
    ///
    /// **安定ID(世代付き)の生存確認もここで行う(統合エディタ実装計画、
    /// docs/reviews/2026-08-14-editor-implementation-plan.md 参照)**:
    /// `self.bodies`は`index`が範囲内であることしか保証しない。Timelineの
    /// 巻き戻し(`restore_snapshot`)は`self.inner`だけを過去の`World`へ
    /// 差し替え、`self.bodies`(フロント向けのindexテーブル)はそのまま残す
    /// ——巻き戻した時点より後に作られたボディを指す`meta.id`が
    /// `self.bodies`側には生き続ける。この`meta.id`をそのまま
    /// `mechanics().bodies.position[id.index as usize]`のような生indexアクセスに
    /// 使うと、`generations`(延いては`RigidBodySet`の各`Vec`)がその`index`より
    /// 短くなっていて**範囲外パニックでモジュール全体が使用不能になる**
    /// (`remove_body_at`で削除済みのボディも同様に世代が進んでいるため、
    /// 同じ理由でここに引っかかる——`removed`フラグと意味的に一致する)。
    /// `World::is_body_alive`(`is_valid`の公開版)で確認し、生存していなければ
    /// このまま範囲外と同じ`Err`にする。
    fn try_body_id_at(&self, index: usize) -> Result<BodyId, WasmError> {
        let id = self.bodies.get(index).map(|meta| meta.id).ok_or_else(|| {
            WasmError::BodyIndexOutOfRange {
                index,
                count: self.body_count_impl(),
            }
        })?;
        if !self.inner.is_body_alive(id) {
            return Err(WasmError::BodyNoLongerExists { index });
        }
        Ok(id)
    }

    /// `index`が範囲外なら`WasmError`を返す`self.bodies[index]`の
    /// フォールブル版(`try_body_id_at`と同じ理由でResult化、共通ヘルパとして
    /// 切り出した)。
    fn try_body_meta_at(&self, index: usize) -> Result<&SpawnedBodyMeta, WasmError> {
        self.bodies
            .get(index)
            .ok_or(WasmError::BodyMetaIndexOutOfRange { index })
    }

    /// Hierarchyパネル表示用のラベル。
    fn body_label_at_impl(&self, index: usize) -> Result<String, WasmError> {
        Ok(self.try_body_meta_at(index)?.label.clone())
    }

    /// `index`番目のボディが静的(Static)かどうか。InspectorがTransformの速度欄を
    /// 意味のある形で表示するための補助(静的ボディは速度が常に0で自明なため)。
    /// `World::mechanics().bodies.body_type`を実クエリする(以前は「index==0
    /// (固定の床)のみ静的」という決め打ちだったが、シーンJSON Import
    /// (`import_scene_json`)で任意のindexに静的ボディが追加され得るようになった
    /// ため、実際の`BodyType`を見るクエリに置き換えた——`body_shape_label_at`が
    /// 既に辿った同じ理由)。
    fn body_is_static_at_impl(&self, index: usize) -> Result<bool, WasmError> {
        let id = self.try_body_id_at(index)?;
        Ok(matches!(
            self.inner.mechanics().bodies.body_type[id.index as usize],
            BodyType::Static
        ))
    }

    /// Inspector表示用のShape文字列。`World::mechanics().bodies.shape_of`で
    /// 実際の現在の形状(Scale Gizmoで変更済みなら変更後の寸法)を読み、
    /// フォーマットする(以前は`SpawnedBodyMeta`にスポーン時の文字列を固定で
    /// 覚えておく縮約実装だったが、Scale Gizmoで寸法が変わりうるようになった
    /// ため、常に最新の値を返す実クエリに置き換えた)。
    fn body_shape_label_at_impl(&self, index: usize) -> Result<String, WasmError> {
        let id = self.try_body_id_at(index)?;
        Ok(
            match self.inner.mechanics().bodies.shape_of(id.index as usize) {
                Shape::Sphere { radius } => format!("Sphere({radius:.4})"),
                Shape::Box { half_extents } => format!(
                    "Box({:.4},{:.4},{:.4})",
                    half_extents.x, half_extents.y, half_extents.z
                ),
                Shape::Plane { normal, d } => {
                    format!(
                        "Plane(normal=({},{},{}), d={d})",
                        normal.x, normal.y, normal.z
                    )
                }
                other => format!("{other:?}"),
            },
        )
    }

    /// Prefab機能(設計docs/23-frontend/01-editor.md §6「Prefabs: 再利用可能な
    /// Body/Joint/Circuit組(自作シーンから...「Prefabとして保存」)。他シーンへ
    /// ドラッグで再利用」の縮約実装——Bodyの形状/材質のみ対象、Joint/Circuit組は
    /// 対象外)向けに、`index`番目のボディの形状の種類を返す
    /// ("sphere"/"box"/"capsule"/"plane"/"compound"/"convex_mesh")。
    /// **寸法そのものは`body_shape_json_at`から読む**——以前は平坦なf64配列を
    /// 返す`body_shape_params_f64_at`と対で使っていたが、Compound/ConvexMeshを
    /// 表現できず両形状だけ別経路になっていたため、寸法の読み出しを
    /// `body_shape_json_at`へ一本化して当該メソッドは削除した
    /// (`body_shape_json_at_impl`のdoc参照)。こちらは「Planeか否か」など
    /// 種類だけで足りる判定が残っているため存続する。
    /// **2026-08-14修正**: `Shape::Capsule`が`_ => "other"`に落ちて
    /// "capsule"を返さない実バグがあった(フロント`duplicate()`の
    /// `kind === "capsule"`分岐が到達不能だった)。Compound/ConvexMeshの
    /// UI作成経路を追加するのと合わせ、6種すべてを網羅する完全一致へ修正。
    fn body_shape_kind_at_impl(&self, index: usize) -> Result<String, WasmError> {
        let id = self.try_body_id_at(index)?;
        Ok(
            match self.inner.mechanics().bodies.shape_of(id.index as usize) {
                Shape::Sphere { .. } => "sphere".to_string(),
                Shape::Box { .. } => "box".to_string(),
                Shape::Capsule { .. } => "capsule".to_string(),
                Shape::Plane { .. } => "plane".to_string(),
                Shape::Compound { .. } => "compound".to_string(),
                Shape::ConvexMesh { .. } => "convex_mesh".to_string(),
            },
        )
    }

    /// `index`番目のボディの、入れ子構造も含めた完全な形状記述をシーンJSON
    /// 形式(`ShapeJson`を`serde_json`でシリアライズした文字列)で返す。
    /// **6形状すべてを無損失に表現できる唯一の読み出し口**であり、フロント側は
    /// 複製(`duplicate()`)もPrefabキャプチャもここ1本から形状を読む。
    ///
    /// **以前は`compound`/`convex_mesh`専用の迂回路だった**: 元々は平坦な
    /// f64配列を返す`body_shape_params_f64_at`(sphere→`[radius]`、
    /// box→`[hx,hy,hz]`、capsule→`[radius,half_height]`)が主経路で、
    /// 可変長の入れ子構造(Compound)や頂点群(ConvexMesh)を平坦な配列で
    /// 表現できないぶんだけこちらが補っていた。結果としてフロントの
    /// `duplicate()`は「sphere/box/capsuleは配列、compound/convex_meshはJSON」
    /// という**形状の種類ごとに分かれた2経路**を抱え、Prefabキャプチャに
    /// 至っては配列側しか持たないまま球/箱だけの機能に留まっていた
    /// (Compound/ConvexMeshのボディを作ってもPrefab化できない実質的な欠落)。
    /// 表現力で劣る側を残す理由が無いため`body_shape_params_f64_at`は削除し、
    /// 読み出しをこの1本へ統合した(シーンJSON importと同じ
    /// `meshFromShapeJson`がそのまま使える形で返す)。書き戻し側の対は
    /// `spawn_shape_json`。
    fn body_shape_json_at_impl(&self, index: usize) -> Result<String, WasmError> {
        let id = self.try_body_id_at(index)?;
        let shape = self.inner.mechanics().bodies.shape_of(id.index as usize);
        let shape_json = sim_world::shape_to_shape_json(shape);
        serde_json::to_string(&shape_json)
            .map_err(|e| WasmError::ShapeSerializeFailed(e.to_string()))
    }

    /// Inspector表示用の材質名。
    fn body_material_label_at_impl(&self, index: usize) -> Result<String, WasmError> {
        Ok(self.try_body_meta_at(index)?.material_label.clone())
    }

    /// Projectドロワー Materials タブ(設計docs/23-frontend/01-editor.md §1.6
    /// 「Materials: MaterialDbプリセット一覧」)向けに、指定した材質名の主要物性値を
    /// `[density, friction, restitution, specific_heat, conductivity]`の順で返す。
    /// 未知の名前なら`WasmError::UnknownMaterial`を返す(呼び出し側UIが
    /// `SPAWN_MATERIALS`等の既知の名前だけを渡す前提だが、**2026-07-27の監査で
    /// 修正**: 以前は`panic!`していた——`try_body_id_at`のdocと同じ理由で
    /// Result化した)。
    ///
    /// **`Vec<f64>`を返す(以前は`Float64Array`だった)**: 唯一の呼び出し元である
    /// `read_component`は受け取った`Float64Array`を直後に`.to_vec()`で戻して
    /// JSONへ流し込んでいた——JS側へ型付き配列として渡る経路はどこにも無く、
    /// 往復は純粋な無駄だった。加えて`Float64Array`はネイティブターゲットで
    /// 構築できないため、この往復が`read_component`の成功パスまで
    /// ネイティブテスト不能にしていた(`WasmError`のdoc参照)。JSから見た
    /// `read_component`の戻り値(JSON配列文字列)は変わらない。
    /// (この記述は元々`body_shape_params_f64_at_impl`のdocにあり、同メソッドを
    /// 削除した際にここへ移した——同じ経緯を辿った`Vec<f64>`返しはこれが最後。)
    fn material_properties_f64_impl(&self, name: &str) -> Result<Vec<f64>, WasmError> {
        let id = self
            .inner
            .materials()
            .find_by_name(name)
            .ok_or_else(|| WasmError::UnknownMaterial(name.to_string()))?;
        let m = self.inner.materials().get(id);
        Ok(vec![
            m.density,
            m.friction,
            m.restitution,
            m.specific_heat,
            m.conductivity,
        ])
    }

    /// スポーンパレット(設計docs/23-frontend/01-editor.md §6)——球を`material_name`
    /// (`MaterialDb::standard`が持つ名前)で`(x,y,z)`に配置する。新しいボディの
    /// index(`body_count`と同じ体系)を返す。未知の材質名なら
    /// `WasmError::UnknownMaterial`を返す(呼び出し側UIが既知の名前だけを
    /// 選択肢にする前提だが、
    /// `material_properties_f64`のdocと同じ理由でResult化した)。
    fn spawn_sphere_impl(
        &mut self,
        x: f64,
        y: f64,
        z: f64,
        radius: f64,
        material_name: String,
    ) -> Result<usize, WasmError> {
        let material = self
            .inner
            .materials()
            .find_by_name(&material_name)
            .ok_or_else(|| WasmError::UnknownMaterial(material_name.clone()))?;
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius }, material);
        desc.transform.position = sim_math::Vec3::new(x, y, z);
        let id = self.inner.create_body(desc);
        let index = self.body_count_impl();
        let label = format!("Sphere_{index}");
        self.bodies.push(SpawnedBodyMeta {
            id,
            label,
            material_label: material_name,
            base_shape: Shape::Sphere { radius },
            constraint_joint_index: None,
            hinge_motor_index: None,
            removed: false,
        });
        Ok(index)
    }

    /// **Hierarchy 右クリックメニューの「削除」(群2)**。設計
    /// docs/23-frontend/01-editor.md §1.1 が求める「右クリックでコンテキストメニュー
    /// (複製・削除・親付け・プレハブ化・アイソレート表示)」の削除。
    ///
    /// **`self.bodies` から要素を取り除かない**——フロントエンドのボディ index は
    /// この `Vec` の位置そのもので、詰めると Scene View のメッシュ対応表・
    /// 選択状態・Console のイベント行が一斉にずれる。`World::remove_body` が
    /// 下層スロットを詰めないのと同じ「無効化に留める」方針で揃える。
    /// 以後 `body_is_removed_at` が `true` を返し、フロント側が Hierarchy の行と
    /// Scene View のメッシュを隠す。
    fn remove_body_at_impl(&mut self, index: usize) -> Result<(), WasmError> {
        let id = self.try_body_id_at(index)?;
        if index == 0 {
            return Err(WasmError::CannotRemoveFloor);
        }
        self.inner.remove_body(id);
        self.bodies[index].removed = true;
        Ok(())
    }

    /// `index`番目のボディが `remove_body_at` で削除済みか。
    fn body_is_removed_at_impl(&self, index: usize) -> Result<bool, WasmError> {
        Ok(self.try_body_meta_at(index)?.removed)
    }

    /// **Hierarchy 右クリックメニューの「複製」(群2)**。元のボディの形状・材質を
    /// そのまま使い、`offset` だけずらした位置に新しいボディを作る。
    ///
    /// **形状は現在の実形状ではなく `base_shape`(スポーン時の寸法)を複製する**
    /// ——Scale Gizmo の倍率は `base_shape × scale` として保持されており、複製後の
    /// ボディも同じ規約(`set_body_scale_at`)に乗せる必要があるため。倍率まで
    /// 引き継ぎたい場合は複製後に改めてスケールを掛ける(既知の限界)。
    fn duplicate_body_at_impl(&mut self, index: usize, offset: f64) -> Result<usize, WasmError> {
        let meta = self.try_body_meta_at(index)?;
        let base_shape = meta.base_shape.clone();
        let material_label = meta.material_label.clone();
        let source_id = meta.id;
        let material = self
            .inner
            .materials()
            .find_by_name(&material_label)
            .ok_or_else(|| WasmError::UnknownMaterial(material_label.clone()))?;
        let position = self
            .inner
            .body_position(source_id)
            .ok_or(WasmError::CannotDuplicateRemovedBody)?;
        let mut desc = RigidBodyDesc::dynamic(base_shape.clone(), material);
        desc.transform.position = position + sim_math::Vec3::new(offset, 0.0, 0.0);
        let id = self.inner.create_body(desc);
        let new_index = self.body_count_impl();
        let label = format!("{}_copy_{new_index}", self.bodies[index].label);
        self.bodies.push(SpawnedBodyMeta {
            id,
            label,
            material_label,
            base_shape,
            constraint_joint_index: None,
            hinge_motor_index: None,
            removed: false,
        });
        Ok(new_index)
    }

    /// スポーンパレット——カプセル(**増分Lで追加**、ローカル+y軸が長軸)。
    /// `spawn_sphere`と同じ規約。`sim-mechanics`側の体積・慣性・接触
    /// (平面/球/カプセル)を同増分で実装した。
    /// **カプセル×箱の接触は未実装**なので、箱と並べても衝突しない
    /// (パニックはしない、`collision.rs`の該当arm参照)。
    fn spawn_capsule_impl(
        &mut self,
        x: f64,
        y: f64,
        z: f64,
        radius: f64,
        half_height: f64,
        material_name: String,
    ) -> Result<usize, WasmError> {
        let material = self
            .inner
            .materials()
            .find_by_name(&material_name)
            .ok_or_else(|| WasmError::UnknownMaterial(material_name.clone()))?;
        let shape = Shape::Capsule {
            radius,
            half_height,
        };
        let mut desc = RigidBodyDesc::dynamic(shape.clone(), material);
        desc.transform.position = sim_math::Vec3::new(x, y, z);
        let id = self.inner.create_body(desc);
        let index = self.body_count_impl();
        let label = format!("Capsule_{index}");
        self.bodies.push(SpawnedBodyMeta {
            id,
            label,
            material_label: material_name,
            base_shape: shape,
            constraint_joint_index: None,
            hinge_motor_index: None,
            removed: false,
        });
        Ok(index)
    }

    /// **残タスク完遂の縦串⑤前後で追加**——`Shape::Compound`をUIから作る経路
    /// (Task#7で`todo!()`を埋めた後もスポーンパレットに無く、シーンJSON
    /// import経由でしか到達できなかった既知の欠落)。L字形(縦棒0.5×2.0×0.5+
    /// 横棒1.0×0.5×0.5、縦棒下端に接続)を既定の複合形状として提供する
    /// ——`World::create_body`が実際に呼ばれる`compound_body_can_be_created_
    /// and_settles_on_the_ground_without_panicking`のL字形と同じ構成(既に
    /// 「地面に落ちて静止する」ところまでヘッドレスで検証済みの形)。
    fn spawn_compound_l_shape_impl(
        &mut self,
        x: f64,
        y: f64,
        z: f64,
        material_name: String,
    ) -> Result<usize, WasmError> {
        let material = self
            .inner
            .materials()
            .find_by_name(&material_name)
            .ok_or_else(|| WasmError::UnknownMaterial(material_name.clone()))?;
        let shape = Shape::Compound {
            children: vec![
                (
                    sim_math::Transform {
                        position: sim_math::Vec3::new(0.0, 0.75, 0.0),
                        rotation: sim_math::Quat::IDENTITY,
                    },
                    Shape::Box {
                        half_extents: sim_math::Vec3::new(0.25, 1.0, 0.25),
                    },
                ),
                (
                    sim_math::Transform {
                        position: sim_math::Vec3::new(0.25, -0.25, 0.0),
                        rotation: sim_math::Quat::IDENTITY,
                    },
                    Shape::Box {
                        half_extents: sim_math::Vec3::new(0.5, 0.25, 0.25),
                    },
                ),
            ],
        };
        let mut desc = RigidBodyDesc::dynamic(shape.clone(), material);
        desc.transform.position = sim_math::Vec3::new(x, y, z);
        let id = self.inner.create_body(desc);
        let index = self.body_count_impl();
        let label = format!("Compound_{index}");
        self.bodies.push(SpawnedBodyMeta {
            id,
            label,
            material_label: material_name,
            base_shape: shape,
            constraint_joint_index: None,
            hinge_motor_index: None,
            removed: false,
        });
        Ok(index)
    }

    /// **残タスク完遂の縦串⑤前後で追加**——`Shape::ConvexMesh`をUIから作る
    /// 経路(`spawn_compound_l_shape`と同じ理由)。頂点が立方体の8隅そのもの
    /// (`half`半辺)という、`Shape::Box`と体積・慣性が完全一致する構成
    /// (`convex_mesh_of_a_cubes_corners_matches_the_equivalent_box`と同じ)を
    /// 既定として提供する——ConvexMeshは接触生成が`None`(すり抜け、既知の
    /// 限界、モジュールdoc参照)なので、他の形状と重ねると実際にすり抜ける。
    fn spawn_convex_mesh_cube_impl(
        &mut self,
        x: f64,
        y: f64,
        z: f64,
        half: f64,
        material_name: String,
    ) -> Result<usize, WasmError> {
        let material = self
            .inner
            .materials()
            .find_by_name(&material_name)
            .ok_or_else(|| WasmError::UnknownMaterial(material_name.clone()))?;
        let mut vertices = Vec::with_capacity(8);
        for &sx in &[-1.0, 1.0] {
            for &sy in &[-1.0, 1.0] {
                for &sz in &[-1.0, 1.0] {
                    vertices.push(sim_math::Vec3::new(sx * half, sy * half, sz * half));
                }
            }
        }
        let shape = Shape::ConvexMesh { vertices };
        let mut desc = RigidBodyDesc::dynamic(shape.clone(), material);
        desc.transform.position = sim_math::Vec3::new(x, y, z);
        let id = self.inner.create_body(desc);
        let index = self.body_count_impl();
        let label = format!("ConvexMesh_{index}");
        self.bodies.push(SpawnedBodyMeta {
            id,
            label,
            material_label: material_name,
            base_shape: shape,
            constraint_joint_index: None,
            hinge_motor_index: None,
            removed: false,
        });
        Ok(index)
    }

    /// **任意の形状をそのまま配置する汎用スポナー**——`shape_json`
    /// (`body_shape_json_at`が返すのと同じ`ShapeJson`のJSON表現)を
    /// `shape_json_to_shape`で`Shape`へ戻し、`material_name`で`(x,y,z)`へ
    /// 置く。新しいボディのindexを返す規約は`spawn_sphere`と同じ。
    ///
    /// **なぜ要ったか(Prefabの実質的な欠落)**: 既存の`spawn_*`はどれも
    /// **固定レシピ**である——`spawn_box`は立方体1辺、`spawn_compound_l_shape`は
    /// L字、`spawn_convex_mesh_cube`は立方体の8隅と、寸法の自由度こそあれ
    /// 「どんな形状か」は呼び出し名で決まってしまう。そのためPrefab
    /// (`body_shape_kind_at`+形状パラメータをキャプチャして再スポーンする機能)は
    /// 球/箱しか扱えず、ユーザーがCompound/ConvexMeshのボディを組んでも
    /// 「Prefabとして保存」が黙って何もしない状態だった。ここが埋まることで
    /// キャプチャ側(`body_shape_json_at`)と再スポーン側が**同じ`ShapeJson`
    /// という1つの語彙**で対になり、6形状すべてが往復する。
    ///
    /// **`Plane`も受け付ける**(弾かない)——形状JSONの語彙をここで狭めると
    /// 「読めるが書き戻せない」非対称が復活するため。無限平面を動剛体として
    /// 増やすことに意味が無いのは事実だが、それはUI側の判断
    /// (`captureBody`が`plane`を弾く)として持たせている。
    fn spawn_shape_json_impl(
        &mut self,
        shape_json: &str,
        x: f64,
        y: f64,
        z: f64,
        material_name: &str,
    ) -> Result<usize, WasmError> {
        let parsed: sim_world::ShapeJson = serde_json::from_str(shape_json)
            .map_err(|e| WasmError::ShapeParseFailed(e.to_string()))?;
        let shape = sim_world::shape_json_to_shape(&parsed);
        let material = self
            .inner
            .materials()
            .find_by_name(material_name)
            .ok_or_else(|| WasmError::UnknownMaterial(material_name.to_string()))?;
        let mut desc = RigidBodyDesc::dynamic(shape.clone(), material);
        desc.transform.position = sim_math::Vec3::new(x, y, z);
        let id = self.inner.create_body(desc);
        let index = self.body_count_impl();
        // 既定ラベルは固定レシピのスポナーと同じ体系(`Sphere_3`等)に揃える
        // ——Hierarchyの行がスポーン経路によって別の命名になると、同じ形状の
        // ボディが由来だけで違って見える。
        let label = format!("{}_{index}", shape_label_prefix(&shape));
        self.bodies.push(SpawnedBodyMeta {
            id,
            label,
            material_label: material_name.to_string(),
            base_shape: shape,
            constraint_joint_index: None,
            hinge_motor_index: None,
            removed: false,
        });
        Ok(index)
    }

    /// 材料派生(**増分Lで追加**)。既存材料`base_name`を基に、密度だけを
    /// 差し替えた新しい材料`new_name`を`MaterialDb`へ追加する
    /// (シーンJSONの`materials[].extends`と同じ仕組みを実行時に開く)。
    ///
    /// **縮約**: 派生できるのは密度のみ。**増分C9**でシーンJSON側の
    /// `MaterialOverride`は全物性の上書きに広がったが、こちらは密度のままにした。
    /// 理由は`apply_component`のペイロード規約——数値フィールドは`f("...")`で
    /// 取り出し、欠けていれば`0.0`になる(省略可能な数値は`component_schema`の
    /// `nullable`が示す通り「0.0をセンチネルに使う」のが既存の約束)。ところが
    /// 摩擦・反発・放射率は**0.0が実在する物性値**(標準表の「水」「空気」がまさに
    /// そう)なので、この規約では「未指定」と「0.0を指定」を区別できない。
    /// 区別するには省略可能な数値用のアクセサと`component_schema`の型語彙を
    /// 新設することになり、wasm境界の規約そのものへの変更になるため、この増分の
    /// 射程外とする。
    ///
    /// なお表現力の食い違いによる書き出し不能は起きない——ここで作った材料
    /// (密度違い+`source = "editor derived"`)は`export::export_materials`が
    /// `extends`+`density`(+差分の`source`)として無損失に書き出す。
    fn derive_material_impl(
        &mut self,
        base_name: String,
        new_name: String,
        density: f64,
    ) -> Result<(), WasmError> {
        let base_id = self
            .inner
            .materials()
            .find_by_name(&base_name)
            .ok_or_else(|| WasmError::UnknownMaterial(base_name.clone()))?;
        if self.inner.materials().find_by_name(&new_name).is_some() {
            return Err(WasmError::MaterialAlreadyExists(new_name.clone()));
        }
        if !(density.is_finite() && density > 0.0) {
            return Err(WasmError::InvalidDensity);
        }
        let mut derived = self.inner.materials().get(base_id).clone();
        // `Material::name`は`&'static str`なので、実行時に作った文字列を入れるには
        // リークさせる必要がある(材料は一度作ったらワールドの寿命の間保持され、
        // 削除する経路も無いため、実質的な漏れにはならない)。
        derived.name = Box::leak(new_name.into_boxed_str());
        derived.density = density;
        derived.source = "editor derived";
        self.inner.materials_mut().push(derived);
        Ok(())
    }

    /// スポーンパレット——箱(半辺長`half_extent`の立方体)を`material_name`で
    /// `(x,y,z)`に配置する。`spawn_sphere`と同じ規約。
    fn spawn_box_impl(
        &mut self,
        x: f64,
        y: f64,
        z: f64,
        half_extent: f64,
        material_name: String,
    ) -> Result<usize, WasmError> {
        let material = self
            .inner
            .materials()
            .find_by_name(&material_name)
            .ok_or_else(|| WasmError::UnknownMaterial(material_name.clone()))?;
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: sim_math::Vec3::new(half_extent, half_extent, half_extent),
            },
            material,
        );
        desc.transform.position = sim_math::Vec3::new(x, y, z);
        let id = self.inner.create_body(desc);
        let index = self.body_count_impl();
        let label = format!("Box_{index}");
        self.bodies.push(SpawnedBodyMeta {
            id,
            label,
            material_label: material_name,
            base_shape: Shape::Box {
                half_extents: sim_math::Vec3::new(half_extent, half_extent, half_extent),
            },
            constraint_joint_index: None,
            hinge_motor_index: None,
            removed: false,
        });
        Ok(index)
    }

    /// シーンJSON Import(設計docs/23-frontend/01-editor.md §1.6「Scenes: シーン
    /// JSON…Export/Import」——Exportは既に実装済み、これがImport側)。`json`を
    /// `sim_world::Scenario`としてパースし、`World::append_scenario_bodies`
    /// (`fluids`/`probes`セクションは対象外、そのdoc参照)で現在のワールドへ
    /// ボディを追加する。D1–D43のシーンJSONファイル(ヘッドレスランナーが使うのと
    /// 同じスキーマ)をそのままエディタへ読み込んで視覚的に確認できるようにする
    /// のが狙い(設計のワークストリームD項目13)。追加した各ボディを`spawn_sphere`/
    /// `spawn_box`と同じ`SpawnedBodyMeta`として登録するため、Hierarchy/Inspector/
    /// Scene Viewから見てスポーンパレットで追加したボディと区別が付かない。
    /// 返り値は追加したボディ数(呼び出し側はこの数だけ`body_count()`の末尾から
    /// メッシュを生成すればよい)。パース/検証エラーは`WasmError`として返す
    /// (最外周の`pub fn`が`JsValue`のメッセージ文字列へ変換する)。
    ///
    /// wasm-bindgenへ露出する薄い殻——実体は`import_scene_json_impl`側。
    pub fn import_scene_json(&mut self, json: String) -> Result<usize, JsValue> {
        self.import_scene_json_impl(&json).map_err(JsValue::from)
    }

    /// `import_scene_json`の実体(ネイティブテスト可能な`Result<_, WasmError>`版)。
    fn import_scene_json_impl(&mut self, json: &str) -> Result<usize, WasmError> {
        let scenario = sim_world::Scenario::from_json(json).map_err(WasmError::ScenarioParse)?;
        let ids = self
            .inner
            .append_scenario_bodies(&scenario)
            .map_err(WasmError::AppendScenarioBodies)?;

        for (body, id) in scenario.bodies.iter().zip(ids.iter()) {
            let index = self.body_count_impl();
            let label = body
                .name
                .clone()
                .unwrap_or_else(|| format!("Imported_{index}"));
            let shape = sim_world::shape_json_to_shape(&body.shape);
            self.bodies.push(SpawnedBodyMeta {
                id: *id,
                label,
                material_label: body.material.clone(),
                base_shape: shape,
                constraint_joint_index: None,
                hinge_motor_index: None,
                removed: false,
            });
        }

        // `scenario.probes`(予測→実験ミニパネルが参照する`probe_index`は
        // この配列の並びに対応する)を、`append_scenario_bodies`とは別に
        // `World::add_scenario_probes`で設定する(`append_scenario_bodies`自体は
        // probesを対象外とする設計のまま、そのdoc参照)。
        let mut body_ids_by_name: HashMap<String, BodyId> = HashMap::new();
        for (body, id) in scenario.bodies.iter().zip(ids.iter()) {
            if let Some(name) = &body.name {
                body_ids_by_name.insert(name.clone(), *id);
            }
        }
        self.imported_probe_handles = self
            .inner
            .add_scenario_probes(&scenario, &body_ids_by_name)
            .map_err(WasmError::ScenarioProbes)?;

        self.last_import_skipped_sections = skipped_import_sections(&scenario);

        Ok(scenario.bodies.len())
    }

    /// 直近の`import_scene_json`が取り込まなかったセクション名(QA不具合5)。
    /// `read_component("last_import_skipped_sections", "")`の実体。
    fn last_import_skipped_sections_impl(&self) -> &[String] {
        &self.last_import_skipped_sections
    }

    /// 予測→実験ミニパネル(設計docs/23-frontend/01-editor.md §5)向けに、直近の
    /// `import_scene_json`が`scenario.probes`から作成したプローブの現在値を返す。
    /// `probe_index`は`scenario.probes`配列内でのインデックス(`prediction_prompts
    /// [].probe_index`と同じ添字系)。範囲外、または該当プローブの履歴がまだ
    /// 1件も無い(1stepも進んでいない)場合は0.0を返す。
    fn imported_probe_value_at_impl(&self, probe_index: usize) -> f64 {
        self.imported_probe_handles
            .get(probe_index)
            .and_then(|&handle| self.inner.probe(handle))
            .and_then(|probe| probe.history().last().copied())
            .unwrap_or(0.0)
    }

    /// `index`を`imported_probe_handles`内の`World::probe`ハンドルへ解決する。
    /// 範囲外なら`WasmError`(`try_body_id_at`のdocと同じ理由でResult化)。
    fn try_imported_probe_handle_at(&self, index: usize) -> Result<usize, WasmError> {
        self.imported_probe_handles
            .get(index)
            .copied()
            .ok_or_else(|| WasmError::ImportedProbeIndexOutOfRange {
                index,
                count: self.imported_probe_count_impl(),
            })
    }

    /// `handle`(`World::probe`用index)を実際の`Probe`へ解決する。
    /// `imported_probe_handles`に積まれたhandleは`add_scenario_probes`が
    /// `World::add_probe`から受け取った直後のものであり、`World`側でprobeが
    /// 削除される経路が現状無いため、この解決が失敗することは実際には
    /// 想定していない(それでも`Result`化するのはQ5の全面Result化方針を
    /// 一貫させるため——`try_body_id_at`のdoc参照)。
    fn try_imported_probe_at(&self, handle: usize) -> Result<&sim_world::Probe, WasmError> {
        self.inner
            .probe(handle)
            .ok_or(WasmError::ImportedProbeHandleMissing { handle })
    }

    /// Probe Graphsパネル(設計docs/23-frontend/01-editor.md §1.4、docs/
    /// 21-verification/03-demo-scenarios.md「UI共通仕様 2. Probeグラフ」)向けに、
    /// 直近の`from_scene_json`/`import_scene_json`が`scenario.probes`から作成した
    /// プローブの本数を返す(`imported_probe_label_at`/`imported_probe_history_f64`
    /// の`index`引数の有効範囲は`0..imported_probe_count()`)。
    fn imported_probe_count_impl(&self) -> usize {
        self.imported_probe_handles.len()
    }

    /// 格子流体の速度場を、Scene Viewのベクトル表示用に平坦化して返す
    /// (**増分Lで追加**)。1セルあたり4要素
    /// `[world_x, world_y, u, v]` を並べる(セル中心のワールド座標と速度成分)。
    /// 格子流体ドメインが無効なら空配列。
    ///
    /// **追加した理由**: 設計 docs/23-frontend/01-editor.md §1.2 の
    /// 「流体場オーバーレイ」のうち、SPHの粒子表示は実装済みだったが
    /// **格子流体(`GridFluid2D`)の速度場は表示手段が無かった**。D14(渦)・
    /// D15(対流)はどちらも格子流体だけのシーンで、Scene Viewに何も描かれず
    /// Probe Graphsでしか観測できない状態だった。
    ///
    /// **縮約**: `GridFluid2D`は2D(x-y平面)なのでz=0の平面上に描く。
    /// セル数が多いと矢印が密になりすぎるため`stride`で間引く
    /// (1なら全セル、2なら1つ飛ばし)。
    pub fn grid_fluid_velocity_field_f32(&self, stride: usize) -> Vec<f32> {
        let Some(grid) = self.inner.grid_fluid() else {
            return Vec::new();
        };
        let step = stride.max(1);
        let mut out = Vec::new();
        for j in (0..grid.ny).step_by(step) {
            for i in (0..grid.nx).step_by(step) {
                // セル中心のワールド座標(格子は原点から+x/+y方向へ h 刻み)。
                let x = (i as f64 + 0.5) * grid.h;
                let y = (j as f64 + 0.5) * grid.h;
                out.push(x as f32);
                out.push(y as f32);
                out.push(grid.u_at(i as i64, j as i64) as f32);
                out.push(grid.v_at(i as i64, j as i64) as f32);
            }
        }
        out
    }

    /// 3D格子流体の**煙**を、点群として返す。
    ///
    /// 1 点あたり 4 要素 `[x, y, z, 濃さ]`。`stride` でセルを間引き、`threshold`
    /// 以下の薄いセルは飛ばす(全セルを返すと数万点になり、ほとんどが空気)。
    ///
    /// **なぜ要ったか**: 「煙が流れる(3D)」は 3D の舞台が最初から最後まで
    /// 真っ暗だった——剛体も粒子も無く、煙は格子の中の数値としてしか存在して
    /// いなかったため。「まん中の 3D を見てください」と案内している隣で何も
    /// 映らない、という一番がっかりする画面になっていた(利用者役③の観察)。
    /// 濃さをそのまま渡して、点として描けるようにする。読み出すだけで、計算には
    /// 触らない。
    pub fn grid_fluid_3d_smoke_points_f32(
        &mut self,
        stride: usize,
        threshold: f32,
    ) -> Float32Array {
        let buf = &mut self.view_buffers.grid_fluid_3d_smoke;
        buf.clear();
        if let Some(grid) = self.inner.grid_fluid_3d() {
            let step = stride.max(1);
            for k in (0..grid.nz).step_by(step) {
                for j in (0..grid.ny).step_by(step) {
                    for i in (0..grid.nx).step_by(step) {
                        let density = grid.smoke_density[i + grid.nx * (j + grid.ny * k)] as f32;
                        if density <= threshold || !density.is_finite() {
                            continue;
                        }
                        buf.push(((i as f64 + 0.5) * grid.h) as f32);
                        buf.push(((j as f64 + 0.5) * grid.h) as f32);
                        buf.push(((k as f64 + 0.5) * grid.h) as f32);
                        buf.push(density);
                    }
                }
            }
        }
        // SAFETY: `fluid_particle_positions_f32`と同じ(`HotPathViewBuffers`のdoc参照)。
        unsafe { Float32Array::view(buf) }
    }

    /// 3D格子流体ドメインの概要(**群9で追加**)。無ければ空文字列。
    ///
    /// **縮約**: 3Dの速度場・煙場をブラウザで可視化する経路(ボリュームレンダリング等)は
    /// 無いので、Hierarchy に出す概要文字列だけを返す。設計
    /// docs/23-frontend/01-editor.md §1.1 の「シーングラフツリー(…Fluids)」に
    /// **ドメインが載っていること自体は見える**ようにする——これが無いと、
    /// シーンに3D格子流体があってもエディタ上で存在を確認する手段が全く無い。
    fn grid_fluid_3d_summary_impl(&self) -> String {
        let Some(grid) = self.inner.grid_fluid_3d() else {
            return String::new();
        };
        let smoke: f64 = grid.smoke_density.iter().sum();
        format!(
            "GridFluid3D ({}x{}x{}, h={:.3}m, 煙={:.2})",
            grid.nx, grid.ny, grid.nz, grid.h, smoke
        )
    }

    /// 重力加速度の大きさ [m/s^2] を実行時に変更する(**群2で追加**)。
    ///
    /// **「Unityのように物理法則を試せる」ことの中心**——重力を変えて挙動が
    /// どう変わるかを見るのは、このツールの最も基本的な使い方である。
    /// `MechanicsSolver.gravity`は元々公開フィールドで実行時に変更可能だったが、
    /// **wasm側にsetterが無かったためフロントエンドから触れなかった**
    /// (`WorldOptions`はコンストラクタでしか受け取らない、と誤解していた)。
    /// **重力場の抽象化増分での変化**: 実体は`MechanicsSolver::set_gravity`
    /// (旧公開フィールドへの代入と同じ意味を保つアクセサ)になった。
    /// 一様場に対する挙動は完全に同一。
    fn set_gravity_impl(&mut self, gravity: f64) {
        self.inner.mechanics_mut().set_gravity(gravity);
    }

    /// 現在の重力加速度の大きさ [m/s^2](非`Uniform`な重力場では0.0、
    /// `MechanicsSolver::gravity`のdoc参照)。
    fn gravity_impl(&self) -> f64 {
        self.inner.mechanics().gravity()
    }

    /// 重力の向きを実行時に変更する(**残タスク完遂増分**、レビュー指摘
    /// 「見送らず対応すること」への対応)。ゼロベクトルは
    /// `MechanicsSolver::set_gravity_direction`が既定の下向きへ安全に
    /// フォールバックする(壊れた入力で重力が消えない)。
    fn set_gravity_direction_impl(&mut self, x: f64, y: f64, z: f64) {
        self.inner
            .mechanics_mut()
            .set_gravity_direction(Vec3::new(x, y, z));
    }

    /// 現在の重力の向き(正規化済み単位ベクトル)を`[x, y, z]`で返す
    /// (非`Uniform`な重力場では既定の下向き、
    /// `MechanicsSolver::gravity_direction`のdoc参照)。
    fn gravity_direction_impl(&self) -> Float64Array {
        let d = self.inner.mechanics().gravity_direction();
        Float64Array::from(&[d.x, d.y, d.z][..])
    }

    /// 重力**場**を差し替える(**重力場の抽象化増分**)。
    /// `kind`が`"uniform"`なら`magnitude`+`(x,y,z)`、`"point_source"`なら
    /// `(center_x, center_y, center_z)`+`mu`、`"zero"`なら追加の引数を見ない
    /// (`sim_mechanics::GravityField`のdoc参照)。
    ///
    /// **既存の`set_gravity`/`set_gravity_direction`との違い**: あちらは即時に
    /// 効き`commandLog`に残らない(移行前からの挙動、後方互換のためそのまま)。
    /// こちらは`Command::SetGravityField`として積まれ、**次stepの先頭で適用され
    /// 記録される**——重力場の変更は以降の全stepを変えるため、記録されないと
    /// リプレイが一致しない(同Commandのdoc参照)。
    #[allow(clippy::too_many_arguments)]
    fn push_set_gravity_field_impl(
        &mut self,
        kind: &str,
        magnitude: f64,
        x: f64,
        y: f64,
        z: f64,
        center_x: f64,
        center_y: f64,
        center_z: f64,
        mu: f64,
    ) -> Result<(), WasmError> {
        let field = match kind {
            "uniform" => sim_mechanics::GravityField::Uniform {
                magnitude,
                direction: Vec3::new(x, y, z),
            },
            "point_source" => sim_mechanics::GravityField::PointSource {
                center: Vec3::new(center_x, center_y, center_z),
                mu,
            },
            "zero" => sim_mechanics::GravityField::Zero,
            _ => return Err(WasmError::UnknownGravityFieldKind),
        };
        self.inner
            .push_command(sim_world::Command::SetGravityField { field });
        Ok(())
    }

    /// 現在の重力場をJSONで返す(`push_set_gravity_field`が受け取るのと同じ
    /// `kind`名を使う——読んだものをそのまま書き戻せるようにするため)。
    fn gravity_field_impl(&self) -> String {
        match self.inner.mechanics().gravity_field() {
            sim_mechanics::GravityField::Uniform {
                magnitude,
                direction,
            } => serde_json::json!({
                "kind": "uniform",
                "magnitude": magnitude,
                "direction": [direction.x, direction.y, direction.z],
            })
            .to_string(),
            sim_mechanics::GravityField::PointSource { center, mu } => serde_json::json!({
                "kind": "point_source",
                "center": [center.x, center.y, center.z],
                "mu": mu,
            })
            .to_string(),
            sim_mechanics::GravityField::Zero => serde_json::json!({ "kind": "zero" }).to_string(),
        }
    }

    /// タイムステップ [s] を実行時に変更する(**群2で追加**)。
    ///
    /// **決定論を壊しうる操作**なので、フロントエンドはEditモードでのみ
    /// 呼び、変更を`commandLog`へ記録する(`SimClock::set_dt`のdoc参照)。
    fn set_dt_impl(&mut self, dt: f64) -> Result<(), WasmError> {
        if !(dt.is_finite() && dt > 0.0) {
            return Err(WasmError::InvalidDt);
        }
        self.inner.clock_mut().set_dt(dt);
        Ok(())
    }

    /// 現在のタイムステップ [s]。
    fn dt_impl(&self) -> f64 {
        self.inner.dt()
    }

    /// Settingsの環境パネル(**残タスク完遂の縦串③増分**)——大気(密度・
    /// 動粘性・風)を設定する。`sim_fluid::Atmosphere`は既に`wind: Vec3`を
    /// 持っていた(P1スケッチどおり)が、UIから設定する手段が無かった。
    /// `World::set_environment`経由なので重力・水域・周囲温度は変えない。
    fn set_atmosphere_impl(
        &mut self,
        density: f64,
        viscosity: f64,
        wind_x: f64,
        wind_y: f64,
        wind_z: f64,
    ) {
        let mut env = self.inner.environment();
        env.atmosphere = Some(sim_fluid::Atmosphere {
            density,
            viscosity,
            wind: Vec3::new(wind_x, wind_y, wind_z),
        });
        self.inner.set_environment(env);
    }

    /// 大気を無効化する(密度0の真空という意味ではなく「大気ドメイン自体を
    /// 評価しない」——`BuoyancyDrag`等の抗力・浮力の大気項が効かなくなる)。
    fn clear_atmosphere_impl(&mut self) {
        let mut env = self.inner.environment();
        env.atmosphere = None;
        self.inner.set_environment(env);
    }

    /// 大気密度[kg/m^3]。大気ドメインが無効なら`NaN`(`heater_node_temperature`
    /// と同じ「無ければNaN」規約)。
    fn atmosphere_density_impl(&self) -> f64 {
        self.inner
            .environment()
            .atmosphere
            .map(|a| a.density)
            .unwrap_or(f64::NAN)
    }

    /// 大気の動粘性係数[m^2/s]。大気ドメインが無効なら`NaN`。
    fn atmosphere_viscosity_impl(&self) -> f64 {
        self.inner
            .environment()
            .atmosphere
            .map(|a| a.viscosity)
            .unwrap_or(f64::NAN)
    }

    /// 風速ベクトル`[x,y,z]`。大気ドメインが無効なら`[NaN,NaN,NaN]`。
    fn atmosphere_wind_impl(&self) -> Float64Array {
        let wind = match self.inner.environment().atmosphere {
            Some(a) => [a.wind.x, a.wind.y, a.wind.z],
            None => [f64::NAN, f64::NAN, f64::NAN],
        };
        Float64Array::from(&wind[..])
    }

    /// Settingsの環境パネル——静的水域(水位・密度)を設定する
    /// (`World::add_fluid_region`の薄い写像、`set_environment`経由なので
    /// 重力・大気・周囲温度は変えない)。
    ///
    /// **縮約(流体領域の一般化に際しての正直な記録)**: `World`側は複数領域・
    /// 形状つきの`fluids: Vec<FluidRegion>`を持てるようになったが、このJS向け
    /// APIは**登録済み領域を1つの水平半空間で置き換える**移行前の形のままに
    /// してある——Settingsの環境パネルが「水位・密度」の2フィールドしか持たない
    /// 単一領域フォームであり、複数領域・AABBを編集するUIが無いためである。
    /// 複数領域を持つシーンはシーンJSON(`fluids`セクション)から読み込む形で
    /// 扱い、このパネルからは触らない。UI側に領域リストの編集器を足すのは
    /// 後続増分。
    fn set_water_region_impl(&mut self, water_level: f64, density: f64) {
        let mut env = self.inner.environment();
        env.fluids = vec![sim_fluid::FluidRegion::new(water_level, density)];
        self.inner.set_environment(env);
    }

    /// 静的水域を無効化する(登録済みの領域を全て取り除く)。
    fn clear_water_region_impl(&mut self) {
        let mut env = self.inner.environment();
        env.fluids.clear();
        self.inner.set_environment(env);
    }

    /// 静的水域の水位[m]。無効なら`NaN`。複数領域があるときは**先頭**を返す
    /// (`set_water_region_impl`のdocの縮約)。
    fn water_level_impl(&self) -> f64 {
        self.inner
            .environment()
            .fluids
            .first()
            .map(|w| w.water_level)
            .unwrap_or(f64::NAN)
    }

    /// 静的水域の密度[kg/m^3]。無効なら`NaN`(`water_level_impl`と同じ縮約)。
    fn water_density_impl(&self) -> f64 {
        self.inner
            .environment()
            .fluids
            .first()
            .map(|w| w.density)
            .unwrap_or(f64::NAN)
    }

    /// エネルギー台帳の残差(**増分Kで追加**)。Consoleの発散警告バッジが使う
    /// ——保存則が壊れた(発散した)ことは、残差が有限でなくなるか急激に増える
    /// ことに現れる。
    fn energy_residual_impl(&self) -> f64 {
        self.inner.energy_residual()
    }

    /// 全剛体の速さの最大値(**増分Kで追加**)。ConsoleのCFL警告バッジが使う
    /// ——CFL条件の目安 `v·dt/L` を出すのに要る。
    fn max_body_speed_impl(&self) -> f64 {
        (0..self.inner.mechanics().bodies.position.len())
            .map(|i| self.inner.mechanics().bodies.linear_velocity[i].length())
            .fold(0.0_f64, f64::max)
    }

    /// 登録済み結合の件数。
    fn coupling_count_impl(&self) -> usize {
        self.inner.coupling_count()
    }

    /// 結合の内省情報を改行区切りで返す(**群1で追加**)。1行1件で
    /// `種別\t説明\tドメイン\t関連ボディ(カンマ区切り)` のタブ区切り。
    ///
    /// **`body_index`を渡すとその剛体に作用する結合だけに絞る**(負値なら全件)。
    /// 設計 docs/23-frontend/01-editor.md §1.3 の Coupling コンポーネントは
    /// 「種別・関連する Body/Fluid/Circuit 参照」を要求しており、
    /// **選択中のオブジェクトのコンポーネントとして出す**のが本来の形なので、
    /// 絞り込みをこのAPIの既定の使い方にする。
    ///
    /// **なぜJSONではなくタブ区切りか**: `sim-wasm`は`serde_json`を依存に
    /// 持たない(バイナリサイズを抑える既存の方針)。行・列の区切りだけで
    /// 表現できる平坦なデータなので、自前のJSON組み立てより素直である。
    fn coupling_info_text_impl(&self, body_index: i32) -> String {
        let infos = if body_index < 0 {
            self.inner.couplings()
        } else {
            match self.try_body_id_at(body_index as usize) {
                Ok(id) => self.inner.couplings_for_body(id),
                Err(_) => Vec::new(),
            }
        };
        infos
            .iter()
            .map(|c| {
                let domains: Vec<String> = c.domains.iter().map(|d| format!("{d:?}")).collect();
                let bodies: Vec<String> = c.bodies.iter().map(|b| b.to_string()).collect();
                // 5番目のindex列(**残タスク完遂の縦串⑤増分**で追加)は
                // `push_set_coupling_control_surface_deflection`が参照する
                // `World::couplings()`の登録indexそのもの——既存の4列パースは
                // 先頭4つだけを読むため後方互換(末尾への追加のみ)。
                format!(
                    "{}\t{}\t{}\t{}\t{}",
                    c.kind.name(),
                    c.description,
                    domains.join("+"),
                    bodies.join(","),
                    c.index
                )
            })
            .collect::<Vec<_>>()
            .join("\n")
    }

    /// `CouplingKind`の1行説明(Inspectorのツールチップ用、**群1で追加**)。
    fn coupling_kind_summary_impl(&self, kind_name: String) -> String {
        self.inner
            .couplings()
            .iter()
            .find(|c| c.kind.name() == kind_name)
            .map(|c| c.kind.summary().to_string())
            .unwrap_or_default()
    }

    /// 登録index`coupling_index`の結合が、実行時に変更できるパラメータの
    /// 名前一覧(JSON配列文字列、**Task#9**)。
    ///
    /// **これが無かった間の縮約**: Coupling registryで実行時に書き換えられる
    /// パラメータは`push_set_coupling_control_surface_deflection`(舵角)ただ
    /// 一つだが、**それが「どの結合に対して有効か」を知る手段が無かった**。
    /// `Command::SetCouplingParam`は範囲外indexも翼以外の結合も無言で
    /// 無視する(あちらのdoc参照——毎フレーム舵角を送っても安全なように、
    /// あえてそう設計した)ので、フロントエンドから見ると**送っても何も
    /// 起きないのか、送り先が間違っているのかが区別できない**。結果として
    /// 舵角スライダーは「翼として追加した結合のindexを利用者が覚えている」
    /// 前提でしか出せなかった。`Coupling::supported_params`をここへ通すことで、
    /// 選択中の結合にスライダーを出してよいかが副作用なしで決まる。
    ///
    /// `coupling_info_text`/`coupling_kind_summary`と同じく結合index/種別を
    /// 鍵にした内省の系列だが、**戻り値はタブ区切りではなくJSON配列**にした
    /// ——`coupling_info_text`のdocが「自前のJSON組み立てより素直」として
    /// タブ区切りを選んだ理由は「`sim-wasm`が`serde_json`を依存に持たない」
    /// ことだったが、その前提はシーンJSON Import/Exportの追加で既に失効して
    /// いる(現在は`serde_json`が依存にある)。可変長の名前リストは配列として
    /// 表すほうが呼び出し側の解釈が一意になる。
    ///
    /// 範囲外indexは`Err`(空配列ではない)——「そんな結合は無い」と
    /// 「その結合に変更可能なパラメータが無い」は区別されるべき情報であり、
    /// 空配列に潰すと前者が後者に化けて原因究明を妨げるため。
    fn coupling_supported_params_impl(&self, coupling_index: usize) -> Result<String, WasmError> {
        let couplings = self.inner.couplings_raw();
        let coupling = couplings
            .get(coupling_index)
            .ok_or(WasmError::CouplingIndexOutOfRange {
                index: coupling_index,
                count: couplings.len(),
            })?;
        let names: Vec<&'static str> = coupling
            .supported_params()
            .iter()
            .map(|p| p.name())
            .collect();
        Ok(serde_json::json!(names).to_string())
    }

    /// ジョイントの内省情報を改行区切りで返す(**群1で追加**)。1行1件で
    /// `種別\t接続\t軸/長さ/目標角\t無効か` のタブ区切り。
    /// `body_index`が非負ならその剛体に接続されたものだけに絞る。
    ///
    /// **これが無かった間の縮約**: フロントエンドは`constraint_anchor_points_at`で
    /// アンカー2点しか取れず、**種別も接続先も軸もモータ設定も見えなかった**。
    fn joint_info_text_impl(&self, body_index: i32) -> String {
        let joints = if body_index < 0 {
            self.inner.joints()
        } else {
            match self.try_body_id_at(body_index as usize) {
                Ok(id) => self.inner.joints_for_body(id),
                Err(_) => Vec::new(),
            }
        };
        joints
            .iter()
            .map(|j| {
                let connection = match j.body_b {
                    Some(b) => format!("body#{} ↔ body#{}", j.body_a, b),
                    None => format!("body#{} ↔ ワールド固定点", j.body_a),
                };
                let mut detail = Vec::new();
                if let Some(l) = j.length {
                    detail.push(format!("length={l}"));
                }
                if let Some(a) = j.axis {
                    detail.push(format!("axis=({:.3}, {:.3}, {:.3})", a.x, a.y, a.z));
                }
                if let Some(t) = j.motor_target {
                    detail.push(format!("target={:.3}rad", t));
                }
                format!(
                    "{}\t{}\t{}\t{}",
                    j.kind.name(),
                    connection,
                    detail.join(" "),
                    if j.disabled { "無効" } else { "有効" }
                )
            })
            .collect::<Vec<_>>()
            .join("\n")
    }

    /// 有効な近似・縮約の一覧を改行区切りで返す(**増分Kで追加**、
    /// Inspectorの「近似バッジ」用。`World::active_approximations`のdoc参照)。
    /// 1行1件、`名前\t理由\t出典\tオフ可否` のタブ区切り(**群1で拡張**)。
    /// 設計 §1.3 の `ApproximationBadge` が要求する「名前・出典・オフ可否」を
    /// すべて渡す(以前は名前だけの文字列だった)。
    fn active_approximations_text_impl(&self) -> String {
        self.inner
            .active_approximations()
            .iter()
            .map(|a| {
                format!(
                    "{}\t{}\t{}\t{}",
                    a.name,
                    a.reason,
                    a.doc,
                    if a.can_disable { "1" } else { "0" }
                )
            })
            .collect::<Vec<_>>()
            .join("\n")
    }

    /// 現在の回路に実際に配線されている素子の本数(**増分G2で追加**)。
    /// `circuit_element_label_at`の`index`引数の有効範囲は
    /// `0..circuit_element_count()`。回路ドメインが無効なら0。
    ///
    /// **追加した理由**: フロントエンドのCircuitタブは固定デモ回路の図を
    /// ハードコードで描いており、シーンギャラリーから別の回路を読み込んでも
    /// **その図が残って実際とは違う値を表示し続けていた**(D19を読み込んでも
    /// 「10V 電源 / 100Ω / 200Ω」のまま。実際は9V / 1kΩ / 2kΩ + コンデンサ +
    /// スイッチ + ダイオード)。実際の素子を列挙する手段が無かったのが原因。
    fn circuit_element_count_impl(&self) -> usize {
        self.inner.circuit().map_or(0, |c| {
            c.voltage_sources().len()
                + c.resistors().len()
                + c.capacitors().len()
                + c.inductors().len()
                + c.diodes().len()
                + c.switches().len()
        })
    }

    /// `index`番目の回路素子の人間可読ラベル。並び順は
    /// 電圧源→抵抗→コンデンサ→インダクタ→ダイオード→スイッチ
    /// (`circuit_element_count`の加算順と同じ)。スイッチは現在の開閉状態も出す
    /// ——`Command::SetSwitch`で実行中に変わる唯一の素子であり、
    /// 表示が実態と乖離しないことがこのAPIを足した動機そのものだから。
    fn circuit_element_label_at_impl(&self, index: usize) -> Result<String, WasmError> {
        let circuit = self
            .inner
            .circuit()
            .ok_or(WasmError::CircuitDomainNotEnabled)?;
        let node = |n: usize| {
            if n == sim_em::GROUND {
                "GND".to_string()
            } else {
                format!("N{n}")
            }
        };
        let mut i = index;
        for (k, (a, b, v)) in circuit.voltage_sources().iter().enumerate() {
            if i == 0 {
                return Ok(format!("V{k}: {} → {} {v} V", node(*b), node(*a)));
            }
            i -= 1;
        }
        for (a, b, r) in circuit.resistors() {
            if i == 0 {
                return Ok(format!("R: {} – {} {r} Ω", node(*a), node(*b)));
            }
            i -= 1;
        }
        for (a, b, c) in circuit.capacitors() {
            if i == 0 {
                return Ok(format!("C: {} – {} {c} F", node(*a), node(*b)));
            }
            i -= 1;
        }
        for (a, b, l) in circuit.inductors() {
            if i == 0 {
                return Ok(format!("L: {} – {} {l} H", node(*a), node(*b)));
            }
            i -= 1;
        }
        for (a, k, _, _) in circuit.diodes() {
            if i == 0 {
                return Ok(format!("D: {} → {}", node(*a), node(*k)));
            }
            i -= 1;
        }
        for (k, (a, b, closed)) in circuit.switches().iter().enumerate() {
            if i == 0 {
                let state = if *closed { "閉" } else { "開" };
                return Ok(format!("SW{k}: {} – {} ({state})", node(*a), node(*b)));
            }
            i -= 1;
        }
        Err(WasmError::CircuitElementIndexOutOfRange {
            index,
            count: self.circuit_element_count_impl(),
        })
    }

    /// `index`番目のインポート済みプローブの人間可読ラベル(Probe Graphsパネルの
    /// 凡例表示用)。`probe_target_label`のdoc参照。
    fn imported_probe_label_at_impl(&self, index: usize) -> Result<String, WasmError> {
        let handle = self.try_imported_probe_handle_at(index)?;
        let probe = self.try_imported_probe_at(handle)?;
        Ok(self.probe_target_label(probe.target))
    }

    /// `index`番目のインポート済みプローブの観測履歴(古い順、`y_probe_history_f64`
    /// と同じ実装パターン)。既定シーン専用の`y_probe_history_f64`/
    /// `speed_probe_history_f64`とは独立に、シーンJSONが宣言した任意本数の
    /// プローブをProbe Graphsパネルへ配線するために追加した
    /// (docs/22-roadmap/02-feature-checklist.md 増分B1)。
    ///
    /// `Float64Array`の構築はネイティブターゲットでは行えない(モジュール末尾の
    /// テストdoc参照)ため、**index検証と値の取り出しだけを
    /// `imported_probe_history_impl`へ切り出した**——`Err`パスをネイティブで
    /// 検証できるようにするためで、JS側から見た戻り値の中身は従来と同一。
    ///
    /// **B16(ゼロコピー化)**: 戻り値は`self.view_buffers.imported_probe_history`を
    /// エイリアスする一時的なビュー(`HotPathViewBuffers`のdoc参照)。呼び出し側は
    /// 値を読み切ってから次のWasm呼び出しへ進むこと。
    pub fn imported_probe_history_f64(&mut self, index: usize) -> Result<Float64Array, JsValue> {
        let values = self.imported_probe_history_impl(index)?;
        let buf = &mut self.view_buffers.imported_probe_history;
        buf.clear();
        buf.extend_from_slice(&values);
        // SAFETY: `buf`への書き込みはここまでで完了しており、このビューを
        // 構築した後は関数を抜けるだけ(`HotPathViewBuffers`のdoc参照)。
        Ok(unsafe { Float64Array::view(buf) })
    }

    /// `imported_probe_history_f64`の実体(`Float64Array`化する前の生の履歴)。
    fn imported_probe_history_impl(&self, index: usize) -> Result<Vec<f64>, WasmError> {
        let handle = self.try_imported_probe_handle_at(index)?;
        let probe = self.try_imported_probe_at(handle)?;
        Ok(probe.history().copied().collect())
    }

    /// 全プローブ履歴が占めるメモリの概算[byte]
    /// (`World::probe_history_bytes_estimate`の素通し、
    /// `read_component("probe_history_bytes_estimate", "")`から引く)。
    ///
    /// **なぜ露出するのか**: プローブ履歴は固定容量のリングバッファをやめて
    /// 可変長になった(`sim_world::Probe`のdoc参照)。**測ったデータを黙って
    /// 捨てない**のがその要点なので、コア側は上限を課さない——代わりに
    /// フロントエンドがこの値を見て、自分で決めた軟らかい上限に近づいたら
    /// 「長時間実行中です」と警告できるようにする。**警告するかどうか・
    /// どこを閾値にするかは呼び出し側の判断**であり、ここでは事実だけを返す。
    fn probe_history_bytes_estimate_impl(&self) -> usize {
        self.inner.probe_history_bytes_estimate()
    }

    /// 指定した(インポート済みシーンの)プローブの蓄積済みサンプル数。
    /// `probe_history_bytes_estimate`の内訳を系列ごとに見たいとき用。
    fn imported_probe_history_len_impl(&self, index: usize) -> Result<usize, WasmError> {
        let handle = self.try_imported_probe_handle_at(index)?;
        Ok(self.try_imported_probe_at(handle)?.len())
    }

    /// `ProbeTarget`の11変種をProbe Graphsパネル表示用の人間可読ラベルへ変換する
    /// (`imported_probe_label_at`から使う)。既存の既定シーン向け固定ラベル
    /// (`demo/src/main.ts`が`y_probe_history_f64`/`speed_probe_history_f64`に
    /// 振っている"BodyPosY"/"BodySpeed"という表記)に揃え、ボディを指す変種は
    /// 括弧内に表示名を添える(例: `"BodyPosY(box)"`)。
    ///
    /// **縮約実装(正直な記録)**: ボディを指す変種(`BodyPosY`/`BodyPosX`/
    /// `BodySpeed`)の表示名は、`self.bodies`(`WasmWorld`がボディごとに覚えている
    /// `SpawnedBodyMeta`)を線形探索して`BodyId`が一致するエントリを探し、その
    /// `label`を使う——`World`自体は`BodyId`から表示名を逆引きする公開APIを
    /// 持たないため(`body_label_at`が`try_body_id_at`経由でindex→`BodyId`の
    /// 一方向にしか対応していないのと対称の制約)。一致が見つからない場合は
    /// `format!("body#{}", id.index)`という**index表記への縮約フォールバック**を
    /// 使う——`imported_probe_handles`に積まれる`BodyId`は常に
    /// `append_scenario_bodies`が直前に`self.bodies`へpushしたものである
    /// (`import_scene_json`/`from_scene_json`のdoc参照)ため、実際にはこの
    /// フォールバックは踏まれない想定だが、将来ボディ削除が実装された場合の
    /// 保険として残す。
    fn probe_target_label(&self, target: ProbeTarget) -> String {
        let body_label = |id: BodyId| -> String {
            self.bodies
                .iter()
                .find(|meta| meta.id == id)
                .map(|meta| meta.label.clone())
                .unwrap_or_else(|| format!("body#{}", id.index))
        };
        match target {
            ProbeTarget::BodyPosY(id) => format!("BodyPosY({})", body_label(id)),
            ProbeTarget::BodyPosX(id) => format!("BodyPosX({})", body_label(id)),
            ProbeTarget::BodySpeed(id) => format!("BodySpeed({})", body_label(id)),
            ProbeTarget::NodeTemp(idx) => format!("NodeTemp[{idx}]"),
            ProbeTarget::AstroPosX(idx) => format!("AstroPosX[{idx}]"),
            ProbeTarget::AstroPosY(idx) => format!("AstroPosY[{idx}]"),
            ProbeTarget::AstroVelX(idx) => format!("AstroVelX[{idx}]"),
            ProbeTarget::AstroVelY(idx) => format!("AstroVelY[{idx}]"),
            ProbeTarget::CircuitCurrent(idx) => format!("CircuitCurrent[{idx}]"),
            ProbeTarget::CircuitNodeVoltage(node) => format!("CircuitV[{node}]"),
            ProbeTarget::SoftBodyPosX(idx) => format!("SoftBodyPosX[{idx}]"),
            ProbeTarget::SoftBodyPosY(idx) => format!("SoftBodyPosY[{idx}]"),
            ProbeTarget::RodTemp(idx) => format!("RodTemp[{idx}]"),
            ProbeTarget::GridFluidMeanV => "GridFluidMeanV".to_string(),
            ProbeTarget::GridFluidRmsV => "GridFluidRmsV".to_string(),
            ProbeTarget::SphParticlePosY(idx) => format!("SphPosY[{idx}]"),
            ProbeTarget::SphParticleDensity(idx) => format!("SphDensity[{idx}]"),
            // **群3で追加**した量子・統計・FDTDの観測量。
            ProbeTarget::QuantumNorm => "QuantumNorm".to_string(),
            ProbeTarget::QuantumMeanX => "Quantum⟨x⟩".to_string(),
            ProbeTarget::QuantumEnergy => "Quantum⟨H⟩".to_string(),
            ProbeTarget::QuantumTransmission(from) => format!("Quantum透過率[i≥{from}]"),
            ProbeTarget::GasTemperature => "GasT [K]".to_string(),
            ProbeTarget::GasPressure => "GasP [Pa]".to_string(),
            ProbeTarget::IsingMagnetization => "Ising磁化".to_string(),
            ProbeTarget::IsingEnergyPerSpin => "IsingE/N".to_string(),
            ProbeTarget::BrownianMsd => "Brownian⟨Δx²⟩".to_string(),
            ProbeTarget::FdtdEz(i, j) => format!("FdtdEz[{i},{j}]"),
            ProbeTarget::FdtdEnergy => "Fdtdエネルギー".to_string(),
            ProbeTarget::LedgerKinetic => "LedgerKinetic".to_string(),
            ProbeTarget::StateHashDigest => "StateHashDigest".to_string(),
        }
    }

    /// **群3で追加したドメインの可視化用アクセサ**。
    ///
    /// チェックリストは D27–D33 を閉じる際、解禁には「`World`への新ドメイン追加」と
    /// 「**専用の可視化パネル**(波動関数の$|\psi|^2$分布・スピン格子・速度
    /// ヒストグラム等は Scene View の剛体描画では表現できない)」の両方が要ると
    /// 書いていた。前者は群3の`Solver`実装で済み、ここが後者。
    ///
    /// いずれも**表示に必要な最小限の配列を`Float32Array`で返す**——`f64`のまま
    /// 渡すと転送量が倍になるうえ、描画側(Three.js/Canvas)は`f32`しか使わない。
    ///
    /// 量子1D: 確率密度 $|\psi(x)|^2$(格子点数ぶん)。
    ///
    /// **B16(ゼロコピー化)**: 戻り値は`self.view_buffers.quantum_1d_density`を
    /// エイリアスする一時的なビュー(`HotPathViewBuffers`のdoc参照)。呼び出し側は
    /// 値を読み切ってから次のWasm呼び出しへ進むこと。
    pub fn quantum_1d_density_f32(&mut self) -> Float32Array {
        let buf = &mut self.view_buffers.quantum_1d_density;
        buf.clear();
        if let Some(q) = self.inner.quantum_1d() {
            buf.extend(q.psi.iter().map(|p| p.norm_sq() as f32));
        }
        // SAFETY: `buf`への書き込みはここまでで完了しており、このビューを
        // 構築した後は関数を抜けるだけ(`HotPathViewBuffers`のdoc参照)。
        unsafe { Float32Array::view(buf) }
    }

    /// 量子1Dのポテンシャル $V(x)$(密度と同じ格子点数)。障壁・井戸の位置を
    /// 密度と重ねて描くために要る。
    pub fn quantum_1d_potential_f32(&self) -> Float32Array {
        let values: Vec<f32> = self
            .inner
            .quantum_1d()
            .map(|q| q.v.iter().map(|&v| v as f32).collect())
            .unwrap_or_default();
        Float32Array::from(values.as_slice())
    }

    pub fn quantum_1d_dx(&self) -> f64 {
        self.inner.quantum_1d().map_or(0.0, |q| q.dx)
    }

    /// 量子2D: 確率密度 $|\psi(x,y)|^2$ を行優先で返す(`nx*ny`要素)。
    ///
    /// **B16(ゼロコピー化)**: `quantum_1d_density_f32`と同じ規約
    /// (`self.view_buffers.quantum_2d_density`をエイリアスする一時的なビュー)。
    pub fn quantum_2d_density_f32(&mut self) -> Float32Array {
        let buf = &mut self.view_buffers.quantum_2d_density;
        buf.clear();
        if let Some(q) = self.inner.quantum_2d() {
            buf.extend(q.psi.iter().map(|p| p.norm_sq() as f32));
        }
        // SAFETY: `quantum_1d_density_f32`と同じ(`HotPathViewBuffers`のdoc参照)。
        unsafe { Float32Array::view(buf) }
    }

    /// 量子2Dのポテンシャル(スリット壁の位置を density と重ねて描くため)。
    pub fn quantum_2d_potential_f32(&self) -> Float32Array {
        let values: Vec<f32> = self
            .inner
            .quantum_2d()
            .map(|q| q.v.iter().map(|&v| v as f32).collect())
            .unwrap_or_default();
        Float32Array::from(values.as_slice())
    }

    /// 量子2Dの格子サイズ `[nx, ny]`(0要素なら未有効化)。
    pub fn quantum_2d_size(&self) -> Vec<u32> {
        self.inner
            .quantum_2d()
            .map(|q| vec![q.nx as u32, q.ny as u32])
            .unwrap_or_default()
    }

    /// イジング模型のスピン格子(+1 → 1、-1 → 0 の`u8`、`l*l`要素)。
    ///
    /// **B16(ゼロコピー化)**: 以前は`Vec<u8>`を返し、wasm-bindgenの標準変換が
    /// 新しい`Uint8Array`へコピーしていた。他の19メソッドと同じ規約
    /// (`self.view_buffers.ising_spins`をエイリアスする一時的なビュー、
    /// `HotPathViewBuffers`のdoc参照)へ揃えるため、明示的に`Uint8Array`を返す
    /// 形にした。
    pub fn ising_spins_u8(&mut self) -> Uint8Array {
        let buf = &mut self.view_buffers.ising_spins;
        buf.clear();
        if let Some(i) = self.inner.ising() {
            buf.extend(i.spins.iter().map(|&s| u8::from(s > 0)));
        }
        // SAFETY: `quantum_1d_density_f32`と同じ(`HotPathViewBuffers`のdoc参照)。
        unsafe { Uint8Array::view(buf) }
    }

    pub fn ising_size(&self) -> usize {
        self.inner.ising().map_or(0, |i| i.l)
    }

    /// 気体分子の位置(`[x,y,z]`の並び)。粒子数が多いので`stride`で間引ける
    /// (格子流体オーバーレイと同じ方針)。
    ///
    /// **B16(ゼロコピー化)**: `quantum_1d_density_f32`と同じ規約
    /// (`self.view_buffers.kinetic_gas_positions`をエイリアスする一時的な
    /// ビュー)。
    pub fn kinetic_gas_positions_f32(&mut self, stride: usize) -> Float32Array {
        let stride = stride.max(1);
        let buf = &mut self.view_buffers.kinetic_gas_positions;
        buf.clear();
        if let Some(g) = self.inner.kinetic_gas() {
            buf.extend(
                g.position
                    .iter()
                    .step_by(stride)
                    .flat_map(|p| [p.x as f32, p.y as f32, p.z as f32]),
            );
        }
        // SAFETY: `quantum_1d_density_f32`と同じ(`HotPathViewBuffers`のdoc参照)。
        unsafe { Float32Array::view(buf) }
    }

    /// 気体分子の速さのヒストグラム(設計 docs/21-verification/03-demo-scenarios.md
    /// D30「圧力計・温度計・**ヒストグラム**」)。`bins`本の等幅ビンに数え上げ、
    /// **各ビンの粒子数**を返す。マクスウェル分布との比較はフロント側で行う。
    pub fn kinetic_gas_speed_histogram_f32(&self, bins: usize, max_speed: f64) -> Float32Array {
        let bins = bins.max(1);
        let mut counts = vec![0.0f32; bins];
        if let Some(g) = self.inner.kinetic_gas() {
            if max_speed > 0.0 {
                for v in &g.velocity {
                    let speed = v.length();
                    let bin = ((speed / max_speed) * bins as f64).floor() as usize;
                    if bin < bins {
                        counts[bin] += 1.0;
                    }
                }
            }
        }
        Float32Array::from(counts.as_slice())
    }

    /// 気体分子の最大速さ(ヒストグラムのレンジ決定に使う)。
    pub fn kinetic_gas_max_speed(&self) -> f64 {
        self.inner.kinetic_gas().map_or(0.0, |g| g.max_speed())
    }

    /// ブラウン粒子の位置(`[x,y,z]`の並び)。
    ///
    /// **B16(ゼロコピー化)**: `kinetic_gas_positions_f32`と同じ規約
    /// (`self.view_buffers.brownian_positions`をエイリアスする一時的なビュー)。
    pub fn brownian_positions_f32(&mut self, stride: usize) -> Float32Array {
        let stride = stride.max(1);
        let buf = &mut self.view_buffers.brownian_positions;
        buf.clear();
        if let Some(b) = self.inner.brownian() {
            buf.extend(
                b.position
                    .iter()
                    .step_by(stride)
                    .flat_map(|p| [p.x as f32, p.y as f32, p.z as f32]),
            );
        }
        // SAFETY: `quantum_1d_density_f32`と同じ(`HotPathViewBuffers`のdoc参照)。
        unsafe { Float32Array::view(buf) }
    }

    /// FDTD の Ez 場(行優先、`nx*ny`要素)。
    pub fn fdtd_ez_f32(&self) -> Float32Array {
        let values: Vec<f32> = self
            .inner
            .fdtd()
            .map(|f| {
                let mut out = Vec::with_capacity(f.nx() * f.ny());
                for j in 0..f.ny() {
                    for i in 0..f.nx() {
                        out.push(f.ez(i, j) as f32);
                    }
                }
                out
            })
            .unwrap_or_default();
        Float32Array::from(values.as_slice())
    }

    /// FDTD の格子サイズ `[nx, ny]`(0要素なら未有効化)。
    pub fn fdtd_size(&self) -> Vec<u32> {
        self.inner
            .fdtd()
            .map(|f| vec![f.nx() as u32, f.ny() as u32])
            .unwrap_or_default()
    }

    /// ソフトボディ粒子の位置(`[x,y,z]`の並び、**群3で追加**)。
    /// **D13(ロープと旗)は Scene View に何も描かれていなかった**——ソフトボディは
    /// 剛体ではないので `bodyMeshes` の同期対象外で、Probe Graphs でしか
    /// 観測できない状態だった。
    ///
    /// **B16(ゼロコピー化)**: `quantum_1d_density_f32`と同じ規約
    /// (`self.view_buffers.soft_body_positions`をエイリアスする一時的な
    /// ビュー)。
    pub fn soft_body_positions_f32(&mut self) -> Float32Array {
        let buf = &mut self.view_buffers.soft_body_positions;
        buf.clear();
        if let Some(b) = self.inner.soft_body() {
            buf.extend(
                b.position
                    .iter()
                    .flat_map(|p| [p.x as f32, p.y as f32, p.z as f32]),
            );
        }
        // SAFETY: `quantum_1d_density_f32`と同じ(`HotPathViewBuffers`のdoc参照)。
        unsafe { Float32Array::view(buf) }
    }

    /// ソフトボディの距離拘束のペア(`[i, j]`の並び)。線分として描くために要る。
    pub fn soft_body_constraint_pairs_u32(&self) -> Vec<u32> {
        self.inner
            .soft_body()
            .map(|b| {
                b.constraints
                    .iter()
                    .flat_map(|c| [c.i as u32, c.j as u32])
                    .collect()
            })
            .unwrap_or_default()
    }

    /// 天体の位置(`[x,y,z]`の並び、**群3で追加**)。
    /// **D34/D35/D36 も Scene View には何も描かれていなかった**——天体は
    /// `RigidBodySet` とは別の質点集合で、剛体メッシュの同期対象外だった。
    ///
    /// **B16(ゼロコピー化)**: `quantum_1d_density_f32`と同じ規約
    /// (`self.view_buffers.astro_positions`をエイリアスする一時的なビュー)。
    pub fn astro_positions_f32(&mut self) -> Float32Array {
        let buf = &mut self.view_buffers.astro_positions;
        buf.clear();
        if let Some(a) = self.inner.astro() {
            buf.extend(
                a.position
                    .iter()
                    .flat_map(|p| [p.x as f32, p.y as f32, p.z as f32]),
            );
        }
        // SAFETY: `quantum_1d_density_f32`と同じ(`HotPathViewBuffers`のdoc参照)。
        unsafe { Float32Array::view(buf) }
    }

    /// 天体の質量(相対的な描画サイズを決めるために使う)。
    pub fn astro_masses_f64(&self) -> Float64Array {
        let values: Vec<f64> = self
            .inner
            .astro()
            .map(|a| a.mass.clone())
            .unwrap_or_default();
        Float64Array::from(values.as_slice())
    }

    /// 1D熱伝導棒の温度分布(**群3で追加**)。D16 も Scene View には何も
    /// 描かれず Probe Graphs のみだった。
    ///
    /// **B16(ゼロコピー化)**: `quantum_1d_density_f32`と同じ規約
    /// (`self.view_buffers.conduction_rod_temperatures`をエイリアスする
    /// 一時的なビュー)。
    pub fn conduction_rod_temperatures_f32(&mut self) -> Float32Array {
        let buf = &mut self.view_buffers.conduction_rod_temperatures;
        buf.clear();
        if let Some(r) = self.inner.conduction_rod() {
            buf.extend(r.temperature.iter().map(|&t| t as f32));
        }
        // SAFETY: `quantum_1d_density_f32`と同じ(`HotPathViewBuffers`のdoc参照)。
        unsafe { Float32Array::view(buf) }
    }

    /// ドメイン別のエネルギー内訳(`World::energy_report`、**群3で追加**)。
    /// タブ区切り4列 `名前\t合計\t単位\t保存性(1/0)` を改行区切りで返す。
    /// **単位と保存性を必ず添える**——SI と原子単位/正規化単位が混在しうるため、
    /// 数値だけ出すと合計してよいように見えてしまう(`energy_report`のdoc参照)。
    fn energy_report_text_impl(&self) -> String {
        self.inner
            .energy_report()
            .iter()
            .map(|d| {
                format!(
                    "{}\t{}\t{}\t{}",
                    d.domain,
                    d.energy.total(),
                    d.unit,
                    u8::from(d.conservative)
                )
            })
            .collect::<Vec<_>>()
            .join("\n")
    }

    /// スポーンパレット——振り子(拘束オーバーレイの実証用)。ワールド固定点
    /// `(pivot_x, pivot_y, pivot_z)`から`DistanceJoint`(`World::
    /// add_distance_joint_to_world_point`)で距離`arm_length`に保たれる球を
    /// 配置する。鉛直から30度傾いた位置(`pivot`から`arm_length`だけ離れた
    /// 点)を初期位置とすることで、静止した自明な平衡状態ではなく実際に
    /// 重力で振り子運動が始まる。
    fn spawn_pendulum_impl(
        &mut self,
        pivot_x: f64,
        pivot_y: f64,
        pivot_z: f64,
        arm_length: f64,
        material_name: String,
    ) -> Result<usize, WasmError> {
        const BOB_RADIUS: f64 = 0.3;
        let material = self
            .inner
            .materials()
            .find_by_name(&material_name)
            .ok_or_else(|| WasmError::UnknownMaterial(material_name.clone()))?;
        let pivot = sim_math::Vec3::new(pivot_x, pivot_y, pivot_z);
        let initial_angle_from_vertical = std::f64::consts::PI / 6.0; // 30度
        let initial_offset = sim_math::Vec3::new(
            arm_length * initial_angle_from_vertical.sin(),
            -arm_length * initial_angle_from_vertical.cos(),
            0.0,
        );
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: BOB_RADIUS }, material);
        desc.transform.position = pivot + initial_offset;
        let id = self.inner.create_body(desc);
        let joint_index = self.inner.add_distance_joint_to_world_point(
            id,
            sim_math::Vec3::ZERO,
            pivot,
            arm_length,
        );
        let index = self.body_count_impl();
        let label = format!("Pendulum_{index}");
        self.bodies.push(SpawnedBodyMeta {
            id,
            label,
            material_label: material_name,
            base_shape: Shape::Sphere { radius: BOB_RADIUS },
            constraint_joint_index: Some(joint_index),
            hinge_motor_index: None,
            removed: false,
        });
        Ok(index)
    }

    /// スポーンパレット——モーターアーム(`Command::SetMotorTarget`の実証用)。
    /// ワールド固定点`(pivot_x, pivot_y, pivot_z)`へ`BallJoint`でピン留めした
    /// 棒状の箱を、Z軸まわりの`HingeMotorPd`(PD位置サーボ)で角度制御する。
    /// 初期状態は目標角0(鉛直にぶら下がる姿勢)。
    fn spawn_motor_arm_impl(
        &mut self,
        pivot_x: f64,
        pivot_y: f64,
        pivot_z: f64,
        material_name: String,
    ) -> Result<usize, WasmError> {
        const HALF_EXTENTS: sim_math::Vec3 = sim_math::Vec3 {
            x: 0.1,
            y: 0.6,
            z: 0.1,
        };
        let material = self
            .inner
            .materials()
            .find_by_name(&material_name)
            .ok_or_else(|| WasmError::UnknownMaterial(material_name.clone()))?;
        let pivot = sim_math::Vec3::new(pivot_x, pivot_y, pivot_z);
        let anchor_local_top = sim_math::Vec3::new(0.0, HALF_EXTENTS.y, 0.0);
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: HALF_EXTENTS,
            },
            material,
        );
        // 目標角0(鉛直)の姿勢: body中心はpivotから真下にHALF_EXTENTS.yだけ離れた点。
        desc.transform.position = pivot - anchor_local_top;
        desc.mass_override = Some(5.0); // sim-mechanicsの参照テストと同じ質量(kp/kd/torque_maxの既定値がこの慣性で検証済み)。
        let id = self.inner.create_body(desc);
        self.inner.mechanics_mut().add_ball_joint(BallJoint {
            body_a: id.index as usize,
            anchor_a: anchor_local_top,
            body_b: None,
            anchor_b: pivot,
            disabled: false,
        });
        let hinge_motor_index = self.inner.mechanics().hinge_motors.len();
        self.inner.mechanics_mut().add_hinge_motor(HingeMotorPd {
            body: id.index as usize,
            axis: sim_math::Vec3::new(0.0, 0.0, 1.0),
            reference_rotation: sim_math::Quat::IDENTITY,
            theta_target: 0.0,
            kp: 20.0,
            kd: 2.0,
            torque_max: 50.0,
            limit: None,
            disabled: false,
        });
        let index = self.body_count_impl();
        let label = format!("MotorArm_{index}");
        self.bodies.push(SpawnedBodyMeta {
            id,
            label,
            material_label: material_name,
            base_shape: Shape::Box {
                half_extents: HALF_EXTENTS,
            },
            constraint_joint_index: None,
            hinge_motor_index: Some(hinge_motor_index),
            removed: false,
        });
        Ok(index)
    }

    /// `Command::SetMotorTarget`(モジュールdoc参照、`theta_target`は
    /// ラジアン)を、`index`番目のボディが持つヒンジモーターへ送る。
    /// モーターを持たないボディに呼ぶと`WasmError`を返す(呼び出し側UIが
    /// モーターを持つボディだけに対して呼ぶ前提だが、`try_body_id_at`のdocと
    /// 同じ理由でResult化した)。
    fn set_motor_target_at_impl(
        &mut self,
        index: usize,
        theta_target: f64,
    ) -> Result<(), WasmError> {
        let hinge_motor_index = self.try_body_meta_at(index)?.hinge_motor_index;
        let hinge_motor_index =
            hinge_motor_index.ok_or(WasmError::BodyHasNoHingeMotor { index })?;
        self.inner.push_command(Command::SetMotorTarget {
            hinge_motor_index,
            theta_target,
        });
        Ok(())
    }

    /// 分圧回路(`WasmWorld::new`参照)の分圧点電圧[V]。`Command::SetSwitch`の
    /// 効果をUIから確認するための読み取り専用クエリ。
    fn circuit_divider_voltage_impl(&self) -> f64 {
        self.inner
            .circuit_probe(CIRCUIT_DIVIDER_NODE)
            .unwrap_or(0.0)
    }

    /// `Command::SetSwitch`——分圧回路のスイッチの開閉を変更する。閉じると
    /// 分圧点がGNDへ短絡され`circuit_divider_voltage`がほぼ0になる。
    fn set_circuit_switch_closed_impl(&mut self, closed: bool) {
        self.inner.push_command(Command::SetSwitch {
            switch_index: self.circuit_switch_index,
            closed,
        });
    }

    /// 自由配線回路エディタ(設計docs/23-frontend/01-editor.md §6「回路エディタ
    /// サブモード」(D19)の縮約実装——専用のグラフィカルなノード配線UIではなく、
    /// Circuitタブのフォームベースの操作でノード/素子を追加していく形とした)
    /// 向けに、既存の固定デモ回路(`WasmWorld::new`が構築したもの)を`num_nodes`
    /// 個のノードを持つ空の回路で置き換える。以後`circuit_editor_add_*`で
    /// ユーザーが自由に素子を追加していく。置き換え後は固定デモの
    /// `circuit_switch_index`が新回路のスイッチ数を超えて無効になり得るため、
    /// 呼び出し側(`main.ts`)は既存の「回路スイッチ(閉)」チェックボックスを
    /// 無効化する責任を負う(`set_circuit_switch_closed`をこの後に呼ぶと
    /// パニックし得る)。
    fn circuit_editor_reset_impl(&mut self, num_nodes: usize) {
        self.inner.enable_circuit(sim_em::Circuit::new(num_nodes));
    }

    /// 自由配線回路エディタ——ノード`a`・`b`間に抵抗`resistance`[Ω]を追加する。
    /// `circuit_editor_reset`より前に呼ぶと(回路が未有効化)何もしない。
    fn circuit_editor_add_resistor_impl(&mut self, a: usize, b: usize, resistance: f64) {
        if let Some(circuit) = self.inner.circuit_mut() {
            circuit.add_resistor(a, b, resistance);
        }
    }

    /// 自由配線回路エディタ——ノード`a`(正極)・`b`(負極)間に独立電圧源
    /// `voltage`[V]を追加する。
    fn circuit_editor_add_voltage_source_impl(&mut self, a: usize, b: usize, voltage: f64) {
        if let Some(circuit) = self.inner.circuit_mut() {
            circuit.add_voltage_source(a, b, voltage);
        }
    }

    /// 自由配線回路エディタ——ノード`a`・`b`間に理想スイッチを追加する。返り値は
    /// このスイッチのindex(`circuit_editor_set_switch_closed`用)。回路が
    /// 未有効化なら0を返す(縮約実装、呼び出し側は`circuit_editor_reset`を
    /// 先に呼ぶ前提)。
    fn circuit_editor_add_switch_impl(&mut self, a: usize, b: usize, closed: bool) -> usize {
        self.inner
            .circuit_mut()
            .map_or(0, |circuit| circuit.add_switch(a, b, closed))
    }

    /// 自由配線回路エディタ——`circuit_editor_add_switch`が返したindexのスイッチの
    /// 開閉状態を変更する(既存の`set_circuit_switch_closed`と異なりCommandキューを
    /// 経由しない即時変更——自由配線回路の構築/操作全体がEditモード的な直接操作
    /// として設計されているため、`spawn_sphere`等と同じ即時反映の扱いとした)。
    fn circuit_editor_set_switch_closed_impl(&mut self, index: usize, closed: bool) {
        if let Some(circuit) = self.inner.circuit_mut() {
            circuit.set_switch_closed(index, closed);
        }
    }

    /// 自由配線回路エディタ——ノード`a`・`b`間にコンデンサ`capacitance`[F]を
    /// 追加する。`initial_voltage`[V]は充電済みの状態から始めたい場合に使う
    /// (`sim_em::Circuit::add_capacitor`のdoc参照)。
    fn circuit_editor_add_capacitor_impl(
        &mut self,
        a: usize,
        b: usize,
        capacitance: f64,
        initial_voltage: f64,
    ) {
        if let Some(circuit) = self.inner.circuit_mut() {
            circuit.add_capacitor(a, b, capacitance, initial_voltage);
        }
    }

    /// 自由配線回路エディタ——ノード`a`・`b`間にインダクタ`inductance`[H]を
    /// 追加する。
    fn circuit_editor_add_inductor_impl(
        &mut self,
        a: usize,
        b: usize,
        inductance: f64,
        initial_current: f64,
    ) {
        if let Some(circuit) = self.inner.circuit_mut() {
            circuit.add_inductor(a, b, inductance, initial_current);
        }
    }

    /// 自由配線回路エディタ——アノード`anode`・カソード`cathode`間にダイオードを
    /// 追加する(Shockleyダイオード式、`saturation_current`・`n_vt`は
    /// `sim_em::Circuit::add_diode`のdoc参照)。
    fn circuit_editor_add_diode_impl(
        &mut self,
        anode: usize,
        cathode: usize,
        saturation_current: f64,
        n_vt: f64,
    ) {
        if let Some(circuit) = self.inner.circuit_mut() {
            circuit.add_diode(anode, cathode, saturation_current, n_vt);
        }
    }

    /// 自由配線回路エディタ——ノード`a`・`b`間にDCモーター(巻線抵抗+
    /// 巻線インダクタンス+逆起電力源の直列等価回路、`sim_em::Circuit::
    /// add_dc_motor`のdoc参照)を追加する。内部ノードは自動的に2つ確保して
    /// `num_nodes`を伸ばす(呼び出し側が内部ノード番号を管理する必要が無いように
    /// するため——他の`circuit_editor_add_*`と違い、モーターは単純な2端子素子
    /// ではなく内部状態を持つため)。返り値は`circuit_editor_set_motor_speed`/
    /// `circuit_editor_motor_current`用のindex。回路が未有効化なら0を返す
    /// (`circuit_editor_add_switch`と同じ縮約——ただしモーターも登録しないため
    /// 以後の速度設定/電流読み出しは無害な無視になる)。
    fn circuit_editor_add_dc_motor_impl(
        &mut self,
        a: usize,
        b: usize,
        winding_resistance: f64,
        winding_inductance: f64,
        back_emf_constant: f64,
    ) -> usize {
        let Some(circuit) = self.inner.circuit_mut() else {
            return 0;
        };
        let n1 = circuit.add_nodes(2);
        let n2 = n1 + 1;
        let handle = circuit.add_dc_motor(
            a,
            b,
            (n1, n2),
            winding_resistance,
            winding_inductance,
            back_emf_constant,
        );
        self.circuit_editor_motors.push(handle);
        self.circuit_editor_motors.len() - 1
    }

    /// 自由配線回路エディタ——`circuit_editor_add_dc_motor`が返したindexの
    /// モーターの角速度[rad/s]を設定する(逆起電力を反映)。
    fn circuit_editor_set_motor_speed_impl(&mut self, index: usize, angular_velocity: f64) {
        let Some(handle) = self.circuit_editor_motors.get(index).copied() else {
            return;
        };
        if let Some(circuit) = self.inner.circuit_mut() {
            circuit.set_motor_speed(handle, angular_velocity);
        }
    }

    /// 自由配線回路エディタ——`circuit_editor_add_dc_motor`が返したindexの
    /// モーターを流れる電流[A](符号は`a`から`b`へ流れる向きが正)。
    fn circuit_editor_motor_current_impl(&self, index: usize) -> f64 {
        let Some(handle) = self.circuit_editor_motors.get(index).copied() else {
            return 0.0;
        };
        self.inner
            .circuit()
            .map_or(0.0, |circuit| circuit.motor_current(handle))
    }

    /// 自由配線回路エディタ——任意ノードの電圧(既存の固定デモ専用
    /// `circuit_divider_voltage`の一般化、`World::circuit_probe`をそのまま使う)。
    fn circuit_node_voltage_impl(&self, node: usize) -> f64 {
        self.inner.circuit_probe(node).unwrap_or(0.0)
    }

    /// 熱ノード(`WasmWorld::new`参照)の現在温度[K]。`from_scene_json`で読み込んだ
    /// シーンが熱ドメインを持たない場合`thermal`は`None`になるため、
    /// `circuit_divider_voltage`と同じ`unwrap_or`パターンでNaNを返す(HUD側は
    /// `toFixed`でそのまま表示できる、値が無いことが伝わればよい縮約実装)。
    ///
    /// **2026-07-28のD9増分で以下の記述を訂正**: 以前は「現状のギャラリー
    /// シーンは全て力学のみ」と書いていたが、D9(冷めるコーヒー)は`thermal`
    /// セクションに単一ノード(index 0)を持つため、D9を読み込むとこの関数は
    /// **HUDの固定デモ用ヒーターノードではなくD9自身のコーヒーノードの実温度**
    /// を返すようになる(`THERMAL_HEATER_NODE`=0が両者で一致するため、
    /// たまたま意味のある値が出る——コード変更は不要だが、doc上の前提が古く
    /// なっていたため訂正した)。同様に、既定シーンの「ヒーター」トグル
    /// (`push_heat_source`)はD9を読み込んだ状態でPlay中に操作すると、D9の
    /// コーヒーノードへ実際に追加の熱を加える(既定シーン専用に作られたUIが
    /// ギャラリーシーンの実ドメインへ意図せず干渉し得るという、既知の限定)。
    fn heater_node_temperature_impl(&self) -> f64 {
        self.inner
            .thermal()
            .map(|thermal| thermal.nodes[THERMAL_HEATER_NODE].temperature)
            .unwrap_or(f64::NAN)
    }

    /// `Command::SetHeatSource`——熱ノードへ`watts`ワットの熱源を1step分だけ
    /// 与える(モジュールdoc「1step分だけ効く」縮約セマンティクス参照)。
    /// 継続加熱するには呼び出し側が毎stepの直前に再度呼ぶ必要がある
    /// (`main.ts`の`frame()`ループ参照)。
    fn push_heat_source_impl(&mut self, watts: f64) {
        self.inner.push_command(Command::SetHeatSource {
            node: THERMAL_HEATER_NODE,
            watts,
        });
    }

    /// Scene Viewの拘束オーバーレイ(設計docs/23-frontend/01-editor.md §1.2
    /// 「拘束」)向けに、`index`番目のボディが持つ拘束(DistanceJoint)の
    /// アンカー点2点を`[ax,ay,az,bx,by,bz]`(f32)で返す。拘束を持たない
    /// ボディ(床・箱・スポーンした球/箱)なら空配列を返す。
    ///
    /// `imported_probe_history_f64`と同じ理由で、実体(index検証+アンカー点の
    /// 取り出し)を`constraint_anchor_points_impl`へ切り出してある。
    /// 拘束を持たないボディは`None`——従来どおり長さ0の`Float32Array`になる。
    ///
    /// **B16(ゼロコピー化)**: 戻り値は`self.view_buffers.constraint_anchor_points`を
    /// エイリアスする一時的なビュー(`HotPathViewBuffers`のdoc参照)。呼び出し側は
    /// 値を読み切ってから次のWasm呼び出しへ進むこと。
    pub fn constraint_anchor_points_at(&mut self, index: usize) -> Result<Float32Array, JsValue> {
        let points = self.constraint_anchor_points_impl(index)?;
        let buf = &mut self.view_buffers.constraint_anchor_points;
        buf.clear();
        if let Some(points) = points {
            buf.extend_from_slice(&points);
        }
        // SAFETY: `buf`への書き込みはここまでで完了しており、このビューを
        // 構築した後は関数を抜けるだけ(`HotPathViewBuffers`のdoc参照)。
        Ok(unsafe { Float32Array::view(buf) })
    }

    /// `constraint_anchor_points_at`の実体。拘束を持たないボディなら`None`。
    fn constraint_anchor_points_impl(&self, index: usize) -> Result<Option<[f32; 6]>, WasmError> {
        let joint_index = self.try_body_meta_at(index)?.constraint_joint_index;
        let Some(joint_index) = joint_index else {
            return Ok(None);
        };
        let (a, b) = self
            .inner
            .distance_joint_anchor_points(joint_index)
            .expect("constraint_joint_index recorded at spawn time must stay valid");
        Ok(Some([
            a.x as f32, a.y as f32, a.z as f32, b.x as f32, b.y as f32, b.z as f32,
        ]))
    }

    /// wasm境界を`schema`/`read`/`apply`の3メソッドへ畳む取り組み
    /// (**残タスク完遂増分**、Task#8)の第一弾。Joint(5種)・Coupling
    /// (14種の追加+操縦面舵角の実行時変更)・熱ノード追加/流体・気体ドメイン
    /// 有効化(3種)、計25個の「追加/設定」系メソッドをこの1メソッドへ畳んだ
    /// (対になる内省系5個は`read_component`、利用可能kind一覧は
    /// `component_schema`)。実装そのものは変えていない——各`pub fn ○○`を
    /// `fn ○○_impl`(非公開ヘルパー)へ改名し、ここから`match kind`で
    /// 呼ぶだけ(ロジックの一字一句は不変、wasm-bindgenが生成するJS向け
    /// シグネチャの本数だけが減る)。
    ///
    /// `payload`はJSONオブジェクト文字列(フィールド名は元のメソッドの引数名と
    /// 一致、例: `add_distance_joint`なら`{"body_a":0,"ax":0,...}`)。戻り値は
    /// JSONオブジェクト文字列——作成系は`{"index":N}`、それ以外は`{}`。
    ///
    /// **正直な適用範囲**: 毎フレーム呼ばれる型付き配列の読み出し系
    /// (`body_position_at_f32`等、レンダリングループのホットパス)はこの
    /// 取り組みの対象外のまま残す——JSON文字列への都度変換は60fpsの
    /// レンダリングループでは明白な性能後退であり、`schema/read/apply`化の
    /// そもそもの目的(重複ボイラープレートの削減)とは無関係な代償を
    /// 払うことになるため。残る「追加/設定/内省」系メソッド(body系・
    /// environment系・circuit editor系等)は今後の増分で同じ2メソッドへ
    /// 引き続き畳んでいく。
    ///
    /// wasm-bindgenへ露出する薄い殻——`match kind`のディスパッチ本体は
    /// `apply_component_impl`側にある。**25個の`_impl`が返す`WasmError`が
    /// そのまま`?`で通り抜けてここまで来る**ため、それぞれの失敗条件を
    /// ネイティブテストから`apply_component_impl`経由で叩ける。
    pub fn apply_component(&mut self, kind: &str, payload: &str) -> Result<String, JsValue> {
        self.apply_component_impl(kind, payload)
            .map_err(JsValue::from)
    }

    /// `apply_component`の実体(ネイティブテスト可能な`Result<_, WasmError>`版)。
    fn apply_component_impl(&mut self, kind: &str, payload: &str) -> Result<String, WasmError> {
        let v: serde_json::Value = serde_json::from_str(payload)
            .map_err(|e| WasmError::ApplyComponentInvalidJson(e.to_string()))?;
        let f = |key: &str| -> f64 { v.get(key).and_then(|x| x.as_f64()).unwrap_or(0.0) };
        let u = |key: &str| -> usize { v.get(key).and_then(|x| x.as_u64()).unwrap_or(0) as usize };
        let i = |key: &str| -> i32 { v.get(key).and_then(|x| x.as_i64()).unwrap_or(0) as i32 };
        let s = |key: &str| -> String {
            v.get(key)
                .and_then(|x| x.as_str())
                .unwrap_or("")
                .to_string()
        };
        let b = |key: &str| -> bool { v.get(key).and_then(|x| x.as_bool()).unwrap_or(false) };
        match kind {
            "add_distance_joint" => {
                let index = self.add_distance_joint_impl(
                    u("body_a"),
                    f("ax"),
                    f("ay"),
                    f("az"),
                    i("body_b"),
                    f("bx"),
                    f("by"),
                    f("bz"),
                    f("length"),
                )?;
                Ok(format!("{{\"index\":{index}}}"))
            }
            "add_ball_joint" => {
                let index = self.add_ball_joint_impl(
                    u("body_a"),
                    f("ax"),
                    f("ay"),
                    f("az"),
                    i("body_b"),
                    f("bx"),
                    f("by"),
                    f("bz"),
                )?;
                Ok(format!("{{\"index\":{index}}}"))
            }
            "add_slider_joint" => {
                let index = self.add_slider_joint_impl(
                    u("body_a"),
                    f("ax"),
                    f("ay"),
                    f("az"),
                    f("axis_x"),
                    f("axis_y"),
                    f("axis_z"),
                    i("body_b"),
                    f("bx"),
                    f("by"),
                    f("bz"),
                )?;
                Ok(format!("{{\"index\":{index}}}"))
            }
            "add_wheel_joint" => {
                let index = self.add_wheel_joint_impl(
                    u("chassis"),
                    u("wheel"),
                    f("acx"),
                    f("acy"),
                    f("acz"),
                    f("rest_length"),
                    f("frequency"),
                    f("damping_ratio"),
                    f("steer_angle"),
                    f("motor_speed"),
                    f("motor_max_torque"),
                )?;
                Ok(format!("{{\"index\":{index}}}"))
            }
            "add_hinge_motor_joint" => {
                let index = self.add_hinge_motor_joint_impl(
                    u("body"),
                    f("axis_x"),
                    f("axis_y"),
                    f("axis_z"),
                    f("theta_target"),
                    f("kp"),
                    f("kd"),
                    f("torque_max"),
                )?;
                Ok(format!("{{\"index\":{index}}}"))
            }
            "add_image_charge_force_coupling" => {
                self.add_image_charge_force_coupling_impl(
                    u("body"),
                    f("charge"),
                    f("plane_normal_x"),
                    f("plane_normal_y"),
                    f("plane_normal_z"),
                    f("plane_d"),
                )?;
                Ok("{}".to_string())
            }
            "add_lorentz_force_coupling" => {
                self.add_lorentz_force_coupling_impl(u("body"), f("charge"))?;
                Ok("{}".to_string())
            }
            "add_buoyancy_drag_coupling" => {
                self.add_buoyancy_drag_coupling_impl(
                    u("body"),
                    f("water_level"),
                    f("water_density"),
                )?;
                Ok("{}".to_string())
            }
            "add_dissipation_to_heat_coupling" => {
                self.add_dissipation_to_heat_coupling_impl(u("thermal_node"))?;
                Ok("{}".to_string())
            }
            "add_joule_heat_coupling" => {
                self.add_joule_heat_coupling_impl(u("thermal_node"))?;
                Ok("{}".to_string())
            }
            "add_brownian_force_coupling" => {
                self.add_brownian_force_coupling_impl(
                    u("body"),
                    f("radius"),
                    f("viscosity"),
                    u("thermal_node"),
                    v.get("seed").and_then(|x| x.as_u64()).unwrap_or(0),
                    v.get("stream").and_then(|x| x.as_u64()).unwrap_or(0),
                )?;
                Ok("{}".to_string())
            }
            "add_motor_coupling" => {
                self.add_motor_coupling_impl(
                    u("body"),
                    f("axis_x"),
                    f("axis_y"),
                    f("axis_z"),
                    u("voltage_source_index"),
                    f("torque_constant"),
                )?;
                Ok("{}".to_string())
            }
            "add_induction_coupling" => {
                self.add_induction_coupling_impl(
                    u("body"),
                    u("voltage_source_index"),
                    f("length"),
                    f("magnetic_field"),
                    f("axis_x"),
                    f("axis_y"),
                    f("axis_z"),
                )?;
                Ok("{}".to_string())
            }
            "add_thermal_node" => {
                let index = self.add_thermal_node_impl(f("temperature"), f("heat_capacity"));
                Ok(format!("{{\"index\":{index}}}"))
            }
            "enable_grid_fluid_2d_domain" => {
                self.enable_grid_fluid_2d_domain_impl();
                Ok("{}".to_string())
            }
            "enable_gas_compartment" => {
                self.enable_gas_compartment_impl();
                Ok("{}".to_string())
            }
            "enable_quantum_1d_domain" => {
                self.enable_quantum_1d_domain_impl(&s("psi_re"), &s("psi_im"), &s("v"), f("dx"))?;
                Ok("{}".to_string())
            }
            "enable_quantum_2d_domain" => {
                self.enable_quantum_2d_domain_impl(
                    &s("psi_re"),
                    &s("psi_im"),
                    &s("v"),
                    u("nx"),
                    u("ny"),
                    f("dx"),
                    f("dy"),
                )?;
                Ok("{}".to_string())
            }
            "add_sph_rigid_coupling" => {
                self.add_sph_rigid_coupling_impl(u("body"), f("radius"), u("boundary_points"))?;
                Ok("{}".to_string())
            }
            "add_grid_fluid_rigid_coupling" => {
                self.add_grid_fluid_rigid_coupling_impl(
                    u("body"),
                    f("half_width"),
                    f("half_height"),
                )?;
                Ok("{}".to_string())
            }
            "add_piston_gas_coupling" => {
                self.add_piston_gas_coupling_impl(
                    u("body"),
                    f("axis_x"),
                    f("axis_y"),
                    f("axis_z"),
                    f("area"),
                    f("initial_volume"),
                )?;
                Ok("{}".to_string())
            }
            "add_wing_lift_coupling" => {
                let index = self.add_wing_lift_coupling_impl(
                    u("body"),
                    f("wing_area"),
                    f("chord_x"),
                    f("chord_y"),
                    f("chord_z"),
                    f("span_x"),
                    f("span_y"),
                    f("span_z"),
                    f("atmosphere_density"),
                    f("atmosphere_viscosity"),
                )?;
                Ok(format!("{{\"index\":{index}}}"))
            }
            "add_magnus_lift_coupling" => {
                let index = self.add_magnus_lift_coupling_impl(
                    u("body"),
                    f("radius"),
                    f("atmosphere_density"),
                    f("atmosphere_viscosity"),
                )?;
                Ok(format!("{{\"index\":{index}}}"))
            }
            "push_set_coupling_control_surface_deflection" => {
                self.push_set_coupling_control_surface_deflection_impl(
                    u("coupling_index"),
                    f("deflection_radians"),
                );
                Ok("{}".to_string())
            }
            "add_boussinesq_buoyancy_coupling" => {
                self.add_boussinesq_buoyancy_coupling_impl(
                    u("thermal_node"),
                    f("ambient_temperature"),
                    f("thermal_expansion_coefficient"),
                )?;
                Ok("{}".to_string())
            }
            "add_convection_link_coupling" => {
                self.add_convection_link_coupling_impl(
                    u("fluid_node"),
                    u("surface_node"),
                    f("area"),
                    f("characteristic_length"),
                    v.get("mode").and_then(|x| x.as_u64()).unwrap_or(3) as u32,
                    f("fluid_thermal_conductivity"),
                    f("kinematic_viscosity"),
                    f("prandtl_number"),
                    f("thermal_expansion_coefficient"),
                )?;
                Ok("{}".to_string())
            }
            "add_phase_change_morph_coupling" => {
                self.add_phase_change_morph_coupling_impl(
                    u("body"),
                    u("thermal_node"),
                    f("melting_temperature"),
                    f("latent_heat_fusion"),
                    f("specific_heat_solid"),
                    f("specific_heat_liquid"),
                    f("initial_mass"),
                    f("conductance"),
                    f("initial_enthalpy"),
                )?;
                Ok("{}".to_string())
            }
            "set_gravity" => {
                self.set_gravity_impl(f("gravity"));
                Ok("{}".to_string())
            }
            "set_gravity_direction" => {
                self.set_gravity_direction_impl(f("x"), f("y"), f("z"));
                Ok("{}".to_string())
            }
            "push_set_gravity_field" => {
                self.push_set_gravity_field_impl(
                    &s("kind"),
                    f("magnitude"),
                    f("x"),
                    f("y"),
                    f("z"),
                    f("center_x"),
                    f("center_y"),
                    f("center_z"),
                    f("mu"),
                )?;
                Ok("{}".to_string())
            }
            "set_dt" => {
                self.set_dt_impl(f("dt"))?;
                Ok("{}".to_string())
            }
            "set_atmosphere" => {
                self.set_atmosphere_impl(
                    f("density"),
                    f("viscosity"),
                    f("wind_x"),
                    f("wind_y"),
                    f("wind_z"),
                );
                Ok("{}".to_string())
            }
            "clear_atmosphere" => {
                self.clear_atmosphere_impl();
                Ok("{}".to_string())
            }
            "set_water_region" => {
                self.set_water_region_impl(f("water_level"), f("density"));
                Ok("{}".to_string())
            }
            "clear_water_region" => {
                self.clear_water_region_impl();
                Ok("{}".to_string())
            }
            "set_body_position_at" => {
                self.set_body_position_at_impl(u("index"), f("x"), f("y"), f("z"))?;
                Ok("{}".to_string())
            }
            "set_body_rotation_at" => {
                self.set_body_rotation_at_impl(u("index"), f("x"), f("y"), f("z"), f("w"))?;
                Ok("{}".to_string())
            }
            "add_body_probes" => {
                let first = self.add_body_probes_impl(u("index"))?;
                Ok(format!("{{\"index\":{first}}}"))
            }
            "set_body_mass_at" => {
                self.set_body_mass_at_impl(u("index"), f("mass"))?;
                Ok("{}".to_string())
            }
            "set_body_scale_at" => {
                self.set_body_scale_at_impl(u("index"), f("scale"))?;
                Ok("{}".to_string())
            }
            "set_body_scale_xyz_at" => {
                let applied =
                    self.set_body_scale_xyz_at_impl(u("index"), f("sx"), f("sy"), f("sz"))?;
                Ok(format!("{{\"applied\":{applied}}}"))
            }
            "push_apply_force" => {
                self.push_apply_force_impl(u("body_index"), f("fx"), f("fy"), f("fz"))?;
                Ok("{}".to_string())
            }
            "push_set_body_mass" => {
                self.push_set_body_mass_impl(u("body_index"), f("mass"))?;
                Ok("{}".to_string())
            }
            "push_set_body_type" => {
                self.push_set_body_type_impl(u("body_index"), s("kind"))?;
                Ok("{}".to_string())
            }
            "push_set_collision_filter" => {
                self.push_set_collision_filter_impl(
                    u("body_index"),
                    u("group") as u32,
                    u("mask") as u32,
                )?;
                Ok("{}".to_string())
            }
            "push_grab" => {
                self.push_grab_impl(u("body_index"), f("target_x"), f("target_y"), f("target_z"))?;
                Ok("{}".to_string())
            }
            "push_move_grab" => {
                self.push_move_grab_impl(
                    u("body_index"),
                    f("target_x"),
                    f("target_y"),
                    f("target_z"),
                )?;
                Ok("{}".to_string())
            }
            "push_release" => {
                self.push_release_impl(u("body_index"))?;
                Ok("{}".to_string())
            }
            "spawn_sphere" => {
                let index = self.spawn_sphere_impl(
                    f("x"),
                    f("y"),
                    f("z"),
                    f("radius"),
                    s("material_name"),
                )?;
                Ok(format!("{{\"index\":{index}}}"))
            }
            "spawn_capsule" => {
                let index = self.spawn_capsule_impl(
                    f("x"),
                    f("y"),
                    f("z"),
                    f("radius"),
                    f("half_height"),
                    s("material_name"),
                )?;
                Ok(format!("{{\"index\":{index}}}"))
            }
            "spawn_compound_l_shape" => {
                let index =
                    self.spawn_compound_l_shape_impl(f("x"), f("y"), f("z"), s("material_name"))?;
                Ok(format!("{{\"index\":{index}}}"))
            }
            "spawn_convex_mesh_cube" => {
                let index = self.spawn_convex_mesh_cube_impl(
                    f("x"),
                    f("y"),
                    f("z"),
                    f("half"),
                    s("material_name"),
                )?;
                Ok(format!("{{\"index\":{index}}}"))
            }
            "spawn_box" => {
                let index = self.spawn_box_impl(
                    f("x"),
                    f("y"),
                    f("z"),
                    f("half_extent"),
                    s("material_name"),
                )?;
                Ok(format!("{{\"index\":{index}}}"))
            }
            "spawn_shape_json" => {
                let index = self.spawn_shape_json_impl(
                    &s("shape_json"),
                    f("x"),
                    f("y"),
                    f("z"),
                    &s("material_name"),
                )?;
                Ok(format!("{{\"index\":{index}}}"))
            }
            "remove_body_at" => {
                self.remove_body_at_impl(u("index"))?;
                Ok("{}".to_string())
            }
            "duplicate_body_at" => {
                let index = self.duplicate_body_at_impl(u("index"), f("offset"))?;
                Ok(format!("{{\"index\":{index}}}"))
            }
            "derive_material" => {
                self.derive_material_impl(s("base_name"), s("new_name"), f("density"))?;
                Ok("{}".to_string())
            }
            "set_circuit_switch_closed" => {
                self.set_circuit_switch_closed_impl(b("closed"));
                Ok("{}".to_string())
            }
            "circuit_editor_reset" => {
                self.circuit_editor_reset_impl(u("num_nodes"));
                Ok("{}".to_string())
            }
            "circuit_editor_add_resistor" => {
                self.circuit_editor_add_resistor_impl(u("a"), u("b"), f("resistance"));
                Ok("{}".to_string())
            }
            "circuit_editor_add_voltage_source" => {
                self.circuit_editor_add_voltage_source_impl(u("a"), u("b"), f("voltage"));
                Ok("{}".to_string())
            }
            "circuit_editor_add_switch" => {
                let index = self.circuit_editor_add_switch_impl(u("a"), u("b"), b("closed"));
                Ok(format!("{{\"index\":{index}}}"))
            }
            "circuit_editor_set_switch_closed" => {
                self.circuit_editor_set_switch_closed_impl(u("index"), b("closed"));
                Ok("{}".to_string())
            }
            "circuit_editor_add_capacitor" => {
                self.circuit_editor_add_capacitor_impl(
                    u("a"),
                    u("b"),
                    f("capacitance"),
                    f("initial_voltage"),
                );
                Ok("{}".to_string())
            }
            "circuit_editor_add_inductor" => {
                self.circuit_editor_add_inductor_impl(
                    u("a"),
                    u("b"),
                    f("inductance"),
                    f("initial_current"),
                );
                Ok("{}".to_string())
            }
            "circuit_editor_add_diode" => {
                self.circuit_editor_add_diode_impl(
                    u("anode"),
                    u("cathode"),
                    f("saturation_current"),
                    f("n_vt"),
                );
                Ok("{}".to_string())
            }
            "circuit_editor_add_dc_motor" => {
                let index = self.circuit_editor_add_dc_motor_impl(
                    u("a"),
                    u("b"),
                    f("winding_resistance"),
                    f("winding_inductance"),
                    f("back_emf_constant"),
                );
                Ok(format!("{{\"index\":{index}}}"))
            }
            "circuit_editor_set_motor_speed" => {
                self.circuit_editor_set_motor_speed_impl(u("index"), f("angular_velocity"));
                Ok("{}".to_string())
            }
            "push_heat_source" => {
                self.push_heat_source_impl(f("watts"));
                Ok("{}".to_string())
            }
            "add_rotating_frame" => {
                let index = self.add_rotating_frame_impl(f("angular_velocity_z"));
                Ok(format!("{{\"index\":{index}}}"))
            }
            "add_child_frame" => {
                let index = self.add_child_frame_impl(
                    u("parent_index"),
                    f("origin_offset_x"),
                    f("origin_offset_y"),
                    f("origin_offset_z"),
                    f("angular_velocity_z"),
                )?;
                Ok(format!("{{\"index\":{index}}}"))
            }
            "set_motor_target_at" => {
                self.set_motor_target_at_impl(u("index"), f("theta_target"))?;
                Ok("{}".to_string())
            }
            "spawn_pendulum" => {
                let index = self.spawn_pendulum_impl(
                    f("pivot_x"),
                    f("pivot_y"),
                    f("pivot_z"),
                    f("arm_length"),
                    s("material_name"),
                )?;
                Ok(format!("{{\"index\":{index}}}"))
            }
            "spawn_motor_arm" => {
                let index = self.spawn_motor_arm_impl(
                    f("pivot_x"),
                    f("pivot_y"),
                    f("pivot_z"),
                    s("material_name"),
                )?;
                Ok(format!("{{\"index\":{index}}}"))
            }
            "spawn_fluid_block" => {
                self.spawn_fluid_block_impl();
                Ok("{}".to_string())
            }
            "restore_snapshot" => {
                self.restore_snapshot_impl(u("index"))?;
                Ok("{}".to_string())
            }
            "add_bookmark" => {
                self.add_bookmark_impl(s("label"));
                Ok("{}".to_string())
            }
            "restore_bookmark" => {
                self.restore_bookmark_impl(u("index"))?;
                Ok("{}".to_string())
            }
            _ => Err(WasmError::UnknownApplyComponentKind(kind.to_string())),
        }
    }

    /// `apply_component`と対になる汎用の内省(読み取り専用)メソッド
    /// (Task#8第一弾)。`arg`はkindごとに意味が異なる単純な文字列
    /// (JSONではない——数値ならその文字列表現、不要なら空文字列)。
    /// 戻り値は元のメソッドが返していたのと同じ文字列(数値系は
    /// `to_string()`、テキスト系はそのまま)——呼び出し側の解釈は変えていない。
    ///
    /// `apply_component`と同じく、実体は`read_component_impl`側。
    pub fn read_component(&self, kind: &str, arg: &str) -> Result<String, JsValue> {
        self.read_component_impl(kind, arg).map_err(JsValue::from)
    }

    /// `read_component`の実体(ネイティブテスト可能な`Result<_, WasmError>`版)。
    fn read_component_impl(&self, kind: &str, arg: &str) -> Result<String, WasmError> {
        match kind {
            "coupling_count" => Ok(self.coupling_count_impl().to_string()),
            "coupling_info_text" => {
                let body_index: i32 = arg.parse().unwrap_or(-1);
                Ok(self.coupling_info_text_impl(body_index))
            }
            "coupling_kind_summary" => Ok(self.coupling_kind_summary_impl(arg.to_string())),
            "coupling_supported_params" => {
                let index: usize = arg.parse().unwrap_or(0);
                self.coupling_supported_params_impl(index)
            }
            "joint_info_text" => {
                let body_index: i32 = arg.parse().unwrap_or(-1);
                Ok(self.joint_info_text_impl(body_index))
            }
            "thermal_node_count" => Ok(self.thermal_node_count_impl().to_string()),
            "gravity" => Ok(self.gravity_impl().to_string()),
            "gravity_direction" => {
                Ok(serde_json::json!(self.gravity_direction_impl().to_vec()).to_string())
            }
            "gravity_field" => Ok(self.gravity_field_impl()),
            "dt" => Ok(self.dt_impl().to_string()),
            "atmosphere_density" => Ok(self.atmosphere_density_impl().to_string()),
            "atmosphere_viscosity" => Ok(self.atmosphere_viscosity_impl().to_string()),
            "atmosphere_wind" => {
                Ok(serde_json::json!(self.atmosphere_wind_impl().to_vec()).to_string())
            }
            "water_level" => Ok(self.water_level_impl().to_string()),
            "water_density" => Ok(self.water_density_impl().to_string()),
            "body_mass_at" => {
                let index: usize = arg.parse().unwrap_or(0);
                Ok(self.body_mass_at_impl(index)?.to_string())
            }
            "body_position_at_f64" => {
                let index: usize = arg.parse().unwrap_or(0);
                Ok(serde_json::json!(self.body_position_at_f64_impl(index)?.to_vec()).to_string())
            }
            "last_import_skipped_sections" => {
                Ok(serde_json::json!(self.last_import_skipped_sections_impl()).to_string())
            }
            "body_type_at" => {
                let index: usize = arg.parse().unwrap_or(0);
                self.body_type_at_impl(index)
            }
            "body_collision_group_at" => {
                let index: usize = arg.parse().unwrap_or(0);
                Ok(self.body_collision_group_at_impl(index)?.to_string())
            }
            "body_collision_mask_at" => {
                let index: usize = arg.parse().unwrap_or(0);
                Ok(self.body_collision_mask_at_impl(index)?.to_string())
            }
            "body_count" => Ok(self.body_count_impl().to_string()),
            "body_label_at" => {
                let index: usize = arg.parse().unwrap_or(0);
                self.body_label_at_impl(index)
            }
            "body_is_static_at" => {
                let index: usize = arg.parse().unwrap_or(0);
                Ok(self.body_is_static_at_impl(index)?.to_string())
            }
            "body_shape_label_at" => {
                let index: usize = arg.parse().unwrap_or(0);
                self.body_shape_label_at_impl(index)
            }
            "body_shape_kind_at" => {
                let index: usize = arg.parse().unwrap_or(0);
                self.body_shape_kind_at_impl(index)
            }
            "body_shape_json_at" => {
                let index: usize = arg.parse().unwrap_or(0);
                self.body_shape_json_at_impl(index)
            }
            "body_material_label_at" => {
                let index: usize = arg.parse().unwrap_or(0);
                self.body_material_label_at_impl(index)
            }
            "body_is_removed_at" => {
                let index: usize = arg.parse().unwrap_or(0);
                Ok(self.body_is_removed_at_impl(index)?.to_string())
            }
            "material_properties_f64" => {
                Ok(serde_json::json!(self.material_properties_f64_impl(arg)?).to_string())
            }
            "circuit_element_count" => Ok(self.circuit_element_count_impl().to_string()),
            "circuit_element_label_at" => {
                let index: usize = arg.parse().unwrap_or(0);
                self.circuit_element_label_at_impl(index)
            }
            "circuit_divider_voltage" => Ok(self.circuit_divider_voltage_impl().to_string()),
            "circuit_editor_motor_current" => {
                let index: usize = arg.parse().unwrap_or(0);
                Ok(self.circuit_editor_motor_current_impl(index).to_string())
            }
            "circuit_node_voltage" => {
                let node: usize = arg.parse().unwrap_or(0);
                Ok(self.circuit_node_voltage_impl(node).to_string())
            }
            "heater_node_temperature" => Ok(self.heater_node_temperature_impl().to_string()),
            "time" => Ok(self.time_impl().to_string()),
            "step_count" => Ok(self.step_count_impl().to_string()),
            "state_hash" => Ok(self.state_hash_impl()),
            "energy_residual" => Ok(self.energy_residual_impl().to_string()),
            "max_body_speed" => Ok(self.max_body_speed_impl().to_string()),
            "active_approximations_text" => Ok(self.active_approximations_text_impl()),
            "imported_probe_count" => Ok(self.imported_probe_count_impl().to_string()),
            "imported_probe_label_at" => {
                let index: usize = arg.parse().unwrap_or(0);
                self.imported_probe_label_at_impl(index)
            }
            "imported_probe_value_at" => {
                let index: usize = arg.parse().unwrap_or(0);
                Ok(self.imported_probe_value_at_impl(index).to_string())
            }
            // **プローブ履歴の使用量**(`probe_history_bytes_estimate_impl`のdoc参照)。
            // 上限を課さない代わりの観測手段で、警告を出すかどうかは
            // フロントエンドの判断に委ねる。
            "probe_history_bytes_estimate" => {
                Ok(self.probe_history_bytes_estimate_impl().to_string())
            }
            "imported_probe_history_len" => {
                let index: usize = arg.parse().unwrap_or(0);
                Ok(self.imported_probe_history_len_impl(index)?.to_string())
            }
            "frame_count" => Ok(self.frame_count_impl().to_string()),
            "frame_parent_index" => {
                let index: usize = arg.parse().unwrap_or(0);
                Ok(self.frame_parent_index_impl(index)?.to_string())
            }
            "grid_fluid_3d_summary" => Ok(self.grid_fluid_3d_summary_impl()),
            "energy_report_text" => Ok(self.energy_report_text_impl()),
            "fluid_spawn_count" => Ok(self.fluid_spawn_count_impl().to_string()),
            "fluid_particle_count" => Ok(self.fluid_particle_count_impl().to_string()),
            "snapshot_count" => Ok(self.snapshot_count_impl().to_string()),
            "snapshot_time_at" => {
                let index: usize = arg.parse().unwrap_or(0);
                Ok(self.snapshot_time_at_impl(index)?.to_string())
            }
            "bookmark_count" => Ok(self.bookmark_count_impl().to_string()),
            "bookmark_label_at" => {
                let index: usize = arg.parse().unwrap_or(0);
                self.bookmark_label_at_impl(index)
            }
            "bookmark_time_at" => {
                let index: usize = arg.parse().unwrap_or(0);
                Ok(self.bookmark_time_at_impl(index)?.to_string())
            }
            "bookmark_export_scene_json" => {
                let index: usize = arg.parse().unwrap_or(0);
                self.bookmark_export_scene_json_impl(index)
            }
            "export_scene_json" => self.export_scene_json_impl(),
            _ => Err(WasmError::UnknownReadComponentKind(kind.to_string())),
        }
    }

    /// `apply_component`/`read_component`が受け付けるkindの一覧
    /// (JSON文字列、Task#8第一弾 → **Task#9で`apply`側を構造化した**)。
    /// フロントエンドが動的に把握できるようにする3つ目のメソッド。
    ///
    /// **形が変わった理由**: Task#8当時のこのメソッドは`apply`/`read`とも
    /// 単なるkind名の文字列配列で、docには「パラメータのスキーマ自体は元の
    /// UIフォーム側にすでにtitleツールチップとして存在するため、二重管理を
    /// 避けてここでは持たない」と書いてあった。ところが**その「元のUIフォーム」
    /// こそが問題**だった——`demo/src/main.ts`は19種類ほどのフォームを手書きし、
    /// 入力欄のラベルは「Body」「Anchor」「Axis」「Param」という汎用文字列で、
    /// 各フィールドが選択中のkindにとって何を意味するのか(単位・既定値・
    /// 負値の意味)は**titleツールチップにしか存在しなかった**。二重管理の
    /// 回避ではなく、意味がホバーの中だけに閉じ込められていたのである。
    ///
    /// そこで`apply`側は`{ kind, fields: [{ name, type, unit, default,
    /// nullable, min, max }] }`の配列を返すようにした(`component_schema`
    /// モジュールのdoc参照)。これはフォームを手書きする代わりにスキーマから
    /// 生成する後続増分の土台であり、本増分自体はフロントエンドに触れない。
    ///
    /// `read`側は従来どおりkind名の文字列配列——あちらの`arg`は
    /// 「kindごとに意味が異なる単純な文字列」(`read_component`のdoc)で
    /// あって名前付きフィールドの集合ではないため、同じ構造化が意味を成さない。
    pub fn component_schema(&self) -> String {
        serde_json::json!({
            "apply": component_schema::apply_schema(),
            "read": [
                "coupling_count", "coupling_info_text", "coupling_kind_summary",
                "coupling_supported_params",
                "joint_info_text", "thermal_node_count",
                "gravity", "gravity_direction", "gravity_field", "dt",
                "atmosphere_density", "atmosphere_viscosity", "atmosphere_wind",
                "water_level", "water_density",
                "body_mass_at", "body_position_at_f64", "body_type_at",
                "body_collision_group_at", "body_collision_mask_at",
                "body_count", "body_label_at", "body_is_static_at",
                "body_shape_label_at", "body_shape_kind_at", "body_shape_json_at",
                "body_material_label_at", "body_is_removed_at",
                "material_properties_f64",
                "circuit_element_count", "circuit_element_label_at",
                "circuit_divider_voltage", "circuit_editor_motor_current",
                "circuit_node_voltage", "heater_node_temperature",
                "time", "step_count", "state_hash", "energy_residual",
                "max_body_speed", "active_approximations_text",
                "last_import_skipped_sections",
                "imported_probe_count", "imported_probe_label_at",
                "imported_probe_value_at", "probe_history_bytes_estimate",
                "imported_probe_history_len", "frame_count", "frame_parent_index",
                "grid_fluid_3d_summary", "energy_report_text",
                "fluid_spawn_count", "fluid_particle_count",
                "snapshot_count", "snapshot_time_at",
                "bookmark_count", "bookmark_label_at", "bookmark_time_at",
                "bookmark_export_scene_json", "export_scene_json"
            ]
        })
        .to_string()
    }

    /// Inspectorの Add Component(**残タスク完遂の縦串①増分**、
    /// `sim_world::JointDesc::Distance`の薄い写像)——`body_a`(`body_b`が負なら
    /// ワールド固定点)を`length`で結ぶ距離拘束を追加する。返り値は
    /// `joint_info_text`が0始まりで振る種別内indexと同じ体系(種別ごとに別配列
    /// なので、他種別と共有しない)。
    #[allow(clippy::too_many_arguments)]
    fn add_distance_joint_impl(
        &mut self,
        body_a: usize,
        ax: f64,
        ay: f64,
        az: f64,
        body_b: i32,
        bx: f64,
        by: f64,
        bz: f64,
        length: f64,
    ) -> Result<usize, WasmError> {
        let body_a_id = self.try_body_id_at(body_a)?;
        let body_b_id = if body_b < 0 {
            None
        } else {
            Some(self.try_body_id_at(body_b as usize)?)
        };
        Ok(self.inner.create_joint(sim_world::JointDesc::Distance {
            body_a: body_a_id,
            anchor_a: Vec3::new(ax, ay, az),
            body_b: body_b_id,
            anchor_b: Vec3::new(bx, by, bz),
            length,
        }))
    }

    /// Add Component——`sim_world::JointDesc::Ball`の薄い写像。3自由度の
    /// 球面拘束(ドア蝶番・振り子等)を追加する。
    #[allow(clippy::too_many_arguments)]
    fn add_ball_joint_impl(
        &mut self,
        body_a: usize,
        ax: f64,
        ay: f64,
        az: f64,
        body_b: i32,
        bx: f64,
        by: f64,
        bz: f64,
    ) -> Result<usize, WasmError> {
        let body_a_id = self.try_body_id_at(body_a)?;
        let body_b_id = if body_b < 0 {
            None
        } else {
            Some(self.try_body_id_at(body_b as usize)?)
        };
        Ok(self.inner.create_joint(sim_world::JointDesc::Ball {
            body_a: body_a_id,
            anchor_a: Vec3::new(ax, ay, az),
            body_b: body_b_id,
            anchor_b: Vec3::new(bx, by, bz),
        }))
    }

    /// Add Component——`sim_world::JointDesc::Slider`の薄い写像。`axis`方向の
    /// 並進のみを許す拘束(ピストン等)を追加する。
    #[allow(clippy::too_many_arguments)]
    fn add_slider_joint_impl(
        &mut self,
        body_a: usize,
        ax: f64,
        ay: f64,
        az: f64,
        axis_x: f64,
        axis_y: f64,
        axis_z: f64,
        body_b: i32,
        bx: f64,
        by: f64,
        bz: f64,
    ) -> Result<usize, WasmError> {
        let body_a_id = self.try_body_id_at(body_a)?;
        let body_b_id = if body_b < 0 {
            None
        } else {
            Some(self.try_body_id_at(body_b as usize)?)
        };
        Ok(self.inner.create_joint(sim_world::JointDesc::Slider {
            body_a: body_a_id,
            anchor_a: Vec3::new(ax, ay, az),
            axis: Vec3::new(axis_x, axis_y, axis_z),
            body_b: body_b_id,
            anchor_b: Vec3::new(bx, by, bz),
        }))
    }

    /// Add Component——`sim_world::JointDesc::Wheel`の薄い写像(D24車と同じ
    /// 車輪拘束)。`suspension_axis`/`axle_axis`はUIフォームに出さず
    /// `WheelJoint::new`と同じ既定値(下向きサスペンション・横向き車軸、
    /// 乗用車として最も普通の配置)を使う——2本のVec3を追加でUIに出すより、
    /// 「普通の車」を作れることを優先した縮約。操舵・駆動が要るなら
    /// `motor_speed`/`motor_max_torque`/`steer_angle`で足りる。
    #[allow(clippy::too_many_arguments)]
    fn add_wheel_joint_impl(
        &mut self,
        chassis: usize,
        wheel: usize,
        acx: f64,
        acy: f64,
        acz: f64,
        rest_length: f64,
        frequency: f64,
        damping_ratio: f64,
        steer_angle: f64,
        motor_speed: f64,
        motor_max_torque: f64,
    ) -> Result<usize, WasmError> {
        let chassis_id = self.try_body_id_at(chassis)?;
        let wheel_id = self.try_body_id_at(wheel)?;
        let default = sim_mechanics::WheelJoint::new(0, 0, Vec3::ZERO, rest_length);
        Ok(self.inner.create_joint(sim_world::JointDesc::Wheel {
            chassis: chassis_id,
            wheel: wheel_id,
            anchor_chassis: Vec3::new(acx, acy, acz),
            rest_length,
            suspension_axis: default.suspension_axis,
            axle_axis: default.axle_axis,
            frequency,
            damping_ratio,
            steer_angle,
            motor_speed,
            motor_max_torque,
        }))
    }

    /// Add Component——`sim_world::JointDesc::HingeMotor`の薄い写像
    /// (ドア・回転ハッチ等、PD制御で目標角へ駆動する1軸ヒンジ)。
    /// `reference_rotation`は追加時点の`body`の姿勢を基準に取る
    /// (`JointDesc::HingeMotor`のdoc参照、`None`と同じ挙動)。`limit`は
    /// UIフォームでは常に無制限(`None`)——角度制限が要る操作(ドアが
    /// 90°で止まる等)は縦串①の対象外として残す。
    #[allow(clippy::too_many_arguments)]
    fn add_hinge_motor_joint_impl(
        &mut self,
        body: usize,
        axis_x: f64,
        axis_y: f64,
        axis_z: f64,
        theta_target: f64,
        kp: f64,
        kd: f64,
        torque_max: f64,
    ) -> Result<usize, WasmError> {
        let body_id = self.try_body_id_at(body)?;
        Ok(self.inner.create_joint(sim_world::JointDesc::HingeMotor {
            body: body_id,
            axis: Vec3::new(axis_x, axis_y, axis_z),
            reference_rotation: None,
            theta_target,
            kp,
            kd,
            torque_max,
            limit: None,
        }))
    }

    /// Inspectorの Add Coupling(**残タスク完遂の縦串②増分**)——
    /// `sim_coupling::ImageChargeForce`の薄い写像。対象剛体を点電荷近似し、
    /// 平板(法線・符号付き距離)への鏡像力(D26帯電風船と同じ物理)を追加する。
    /// 剛体参照だけで完結する結合(他ドメインの参照を要らない)なので、
    /// 縦串②で最初に配線した3種の1つ。
    fn add_image_charge_force_coupling_impl(
        &mut self,
        body: usize,
        charge: f64,
        plane_normal_x: f64,
        plane_normal_y: f64,
        plane_normal_z: f64,
        plane_d: f64,
    ) -> Result<(), WasmError> {
        let id = self.try_body_id_at(body)?;
        self.inner
            .add_coupling(Box::new(sim_coupling::ImageChargeForce {
                body_index: id.index as usize,
                charge,
                plane_normal: Vec3::new(plane_normal_x, plane_normal_y, plane_normal_z),
                plane_d,
            }));
        Ok(())
    }

    /// Add Coupling——`sim_coupling::LorentzForce`の薄い写像。対象剛体に電荷を
    /// 持たせ、`em_electrostatics`ドメインの電場からのローレンツ力を注入する
    /// (電場が無い/点電荷が無ければ力はゼロ、パニックはしない)。
    fn add_lorentz_force_coupling_impl(
        &mut self,
        body: usize,
        charge: f64,
    ) -> Result<(), WasmError> {
        let id = self.try_body_id_at(body)?;
        self.inner
            .add_coupling(Box::new(sim_coupling::LorentzForce {
                body_index: id.index as usize,
                charge,
            }));
        Ok(())
    }

    /// Add Coupling——`sim_coupling::BuoyancyDrag`の薄い写像。静的水域による
    /// 浮力・抗力をCoupling registry経由で剛体単位に適用する(`fluids[].
    /// static_water`の埋め込み経路とは別物、`BuoyancyDrag`のRustdoc参照)。
    /// `atmosphere`/`lift`(揚力)はUIフォームに出さず`None`のまま——
    /// 大気場は縦串③(環境と大気の場)の対象、揚力は縦串⑤(飛行機の物理)の
    /// 対象として別途配線する。
    fn add_buoyancy_drag_coupling_impl(
        &mut self,
        body: usize,
        water_level: f64,
        water_density: f64,
    ) -> Result<(), WasmError> {
        let id = self.try_body_id_at(body)?;
        self.inner
            .add_coupling(Box::new(sim_coupling::BuoyancyDrag {
                body_index: id.index as usize,
                water: Some(sim_fluid::FluidRegion::new(water_level, water_density)),
                atmosphere: None,
                lift: None,
            }));
        Ok(())
    }

    /// 熱ノードindexが有効(熱ドメインが有効、かつ範囲内)かを確認する
    /// (残り5種のAdd Couplingが参照する`thermal_node`向け共通ガード)。
    fn try_thermal_node_index(&self, index: usize) -> Result<usize, WasmError> {
        let count = self.inner.thermal().map(|t| t.nodes.len()).unwrap_or(0);
        if index >= count {
            return Err(WasmError::ThermalNodeIndexOutOfRange { index, count });
        }
        Ok(index)
    }

    /// 電圧源indexが有効(回路ドメインが有効、かつ範囲内)かを確認する
    /// (MotorCoupling/InductionCouplingが参照する`voltage_source_index`向け)。
    fn try_voltage_source_index(&self, index: usize) -> Result<usize, WasmError> {
        let count = self
            .inner
            .circuit()
            .map(|c| c.voltage_sources().len())
            .unwrap_or(0);
        if index >= count {
            return Err(WasmError::VoltageSourceIndexOutOfRange { index, count });
        }
        Ok(index)
    }

    /// Add Coupling——`sim_coupling::DissipationToHeat`の薄い写像
    /// (`to_single_node`、剛体↔熱ノード対応表は空=全量を`thermal_node`へ)。
    /// D10(摩擦の熱)向け。**熱ドメインが有効なシーンでのみ意味を持つ**——
    /// 既定の起動シーンは熱ノードを1つ(index 0)持つが、Add Componentで
    /// 一から組んだシーンには熱ドメインを追加する手段がまだ無い(縦串③の
    /// 対象外として残した既知の限界、`docs/22-roadmap/03-editor-todo.md`参照)。
    fn add_dissipation_to_heat_coupling_impl(
        &mut self,
        thermal_node: usize,
    ) -> Result<(), WasmError> {
        let node = self.try_thermal_node_index(thermal_node)?;
        self.inner
            .add_coupling(Box::new(sim_coupling::DissipationToHeat {
                thermal_node: node,
                body_links: Vec::new(),
            }));
        Ok(())
    }

    /// Add Coupling——`sim_coupling::JouleHeat`の薄い写像
    /// (`to_single_node`、抵抗↔熱ノード対応表は空=回路全体の損失を
    /// `thermal_node`へ)。D19(電気工作台)向け、熱・回路の両ドメインが
    /// 有効なシーンでのみ意味を持つ(`add_dissipation_to_heat_coupling`と
    /// 同じ既知の限界)。
    fn add_joule_heat_coupling_impl(&mut self, thermal_node: usize) -> Result<(), WasmError> {
        let node = self.try_thermal_node_index(thermal_node)?;
        self.inner
            .add_coupling(Box::new(sim_coupling::JouleHeat::to_single_node(node)));
        Ok(())
    }

    /// Add Coupling——`sim_coupling::BrownianForce`の薄い写像。D25
    /// (ブラウン運動)向け、熱ドメインが有効なシーンでのみ意味を持つ
    /// (同上の既知の限界)。`seed`/`stream`はPRNGの乱数系列
    /// (`SimRng::new`、同じ値なら同じ揺らぎが再現される)。
    #[allow(clippy::too_many_arguments)]
    fn add_brownian_force_coupling_impl(
        &mut self,
        body: usize,
        radius: f64,
        viscosity: f64,
        thermal_node: usize,
        seed: u64,
        stream: u64,
    ) -> Result<(), WasmError> {
        let id = self.try_body_id_at(body)?;
        let node = self.try_thermal_node_index(thermal_node)?;
        self.inner
            .add_coupling(Box::new(sim_coupling::BrownianForce::new(
                id.index as usize,
                radius,
                viscosity,
                node,
                seed,
                stream,
            )));
        Ok(())
    }

    /// Add Coupling——`sim_coupling::MotorCoupling`の薄い写像。D20
    /// (モーターと発電)向け、回路ドメインが有効なシーンでのみ意味を持つ
    /// (`add_dissipation_to_heat_coupling`と同じ既知の限界、対象は熱でなく
    /// 回路)。
    fn add_motor_coupling_impl(
        &mut self,
        body: usize,
        axis_x: f64,
        axis_y: f64,
        axis_z: f64,
        voltage_source_index: usize,
        torque_constant: f64,
    ) -> Result<(), WasmError> {
        let id = self.try_body_id_at(body)?;
        let source = self.try_voltage_source_index(voltage_source_index)?;
        self.inner
            .add_coupling(Box::new(sim_coupling::MotorCoupling {
                body_index: id.index as usize,
                axis: Vec3::new(axis_x, axis_y, axis_z),
                voltage_source_index: source,
                torque_constant,
            }));
        Ok(())
    }

    /// Add Coupling——`sim_coupling::InductionCoupling`の薄い写像。D21
    /// (磁石遊び・銅管落下)向け、回路ドメインが有効なシーンでのみ意味を持つ
    /// (同上の既知の限界)。
    #[allow(clippy::too_many_arguments)]
    fn add_induction_coupling_impl(
        &mut self,
        body: usize,
        voltage_source_index: usize,
        length: f64,
        magnetic_field: f64,
        axis_x: f64,
        axis_y: f64,
        axis_z: f64,
    ) -> Result<(), WasmError> {
        let id = self.try_body_id_at(body)?;
        let source = self.try_voltage_source_index(voltage_source_index)?;
        self.inner
            .add_coupling(Box::new(sim_coupling::InductionCoupling {
                body_index: id.index as usize,
                voltage_source_index: source,
                length,
                magnetic_field,
                axis: Vec3::new(axis_x, axis_y, axis_z),
            }));
        Ok(())
    }

    /// **残タスク完遂の縦串②残り6種を解禁する増分**——レビュー指摘
    /// (「やり遂げて欲しい」「対応できていますか？出来ていなければ対応して」)
    /// への対応。熱ドメインが無効なら既定の周囲温度293.15K(他コードの既定と
    /// 揃える、`ThermalSolver::new`の呼び出し例参照)で自動的に有効化してから
    /// ノードを追加する——「Add Componentで一から組んだシーンには熱ドメインを
    /// 追加する手段が無い」という、上の`add_dissipation_to_heat_coupling`の
    /// docに書かれていたギャップそのものを埋める。戻り値は追加したノードの
    /// index(他のCoupling系メソッドが受け取る`thermal_node`引数にそのまま渡せる)。
    fn add_thermal_node_impl(&mut self, temperature: f64, heat_capacity: f64) -> usize {
        if self.inner.thermal().is_none() {
            self.inner
                .enable_thermal(sim_thermal::ThermalSolver::new(293.15));
        }
        self.inner
            .thermal_mut()
            .expect("just enabled above")
            .add_node(sim_thermal::ThermalNode::new(temperature, heat_capacity))
    }

    /// Inspector/Hierarchy向けの熱ノード数(熱ドメイン未有効なら0)。
    fn thermal_node_count_impl(&self) -> usize {
        self.inner.thermal().map(|t| t.nodes.len()).unwrap_or(0)
    }

    /// 格子流体(2D)ドメインを既定パラメータ(D14/D15など既存ギャラリーシーンの
    /// 値と同オーダー)で有効化する——`GridFluidRigid`/`BoussinesqBuoyancy`結合が
    /// 参照する`world.grid_fluid`はシーンJSON経由でしか作れず、UIから一から
    /// 組む経路が無かった既知の欠落を埋める。既に有効なら何もしない(冪等、
    /// 複数回押しても壊れない)。
    fn enable_grid_fluid_2d_domain_impl(&mut self) {
        if self.inner.grid_fluid().is_none() {
            self.inner
                .enable_grid_fluid(sim_fluid::GridFluid2D::new(32, 32, 0.05));
        }
    }

    /// 気体区画(`PistonGas`が参照する`world.gas`)を既定パラメータ(空気1mol、
    /// 体積1L、常温)で有効化する——`enable_grid_fluid_2d_domain`と同じ理由・
    /// 同じ冪等性。
    fn enable_gas_compartment_impl(&mut self) {
        if self.inner.gas().is_none() {
            self.inner.enable_gas(sim_thermal::GasCompartment {
                n_moles: 1.0,
                volume: 0.001,
                temperature: 293.15,
                gas: sim_thermal::GasSpecies::AIR,
            });
        }
    }

    /// 量子ドメイン(1D)をプリセットUI由来の生状態で有効化する——エディタの
    /// 「＋ 量子ドメイン」フォーム(`demo/src/main.ts`)がガウス波束・ポテンシャル形状
    /// をTypeScript側で計算し、`psi_re`/`psi_im`/`v`(いずれも
    /// `raw_bytes::encode_f64_le_base64_finite`と同じLE+base64)として渡してくる。
    ///
    /// **検証はシーンJSON経路と完全に同じ**(`sim_world::build_quantum_1d_wave_from_raw`
    /// を直接呼ぶ——2の冪長・配列長一致のチェックをここで再実装すると、シーンJSON側の
    /// 検証が変わったときにここだけ取り残される)。`enable_grid_fluid_2d_domain_impl`と
    /// 違って**冪等ではなく常に上書きする**——こちらは呼び出しごとに異なるプリセット
    /// (パラメータ違い)を渡す前提の操作であり、「既に有効なら何もしない」と
    /// 「フォームの値を反映する」が両立しないため(`World::enable_quantum_1d`のdoc
    /// 参照、単に`Option`を差し替えるだけで元々冪等性を持たない)。
    fn enable_quantum_1d_domain_impl(
        &mut self,
        psi_re: &str,
        psi_im: &str,
        v: &str,
        dx: f64,
    ) -> Result<(), WasmError> {
        let wave = sim_world::build_quantum_1d_wave_from_raw(psi_re, psi_im, v, dx)
            .map_err(WasmError::QuantumRawStateInvalid)?;
        self.inner.enable_quantum_1d(wave);
        Ok(())
    }

    /// `enable_quantum_1d_domain_impl`の2D版。`nx`/`ny`は2の冪(FFTの制約、
    /// `build_quantum_2d_wave_from_raw`が検証する)。
    #[allow(clippy::too_many_arguments)]
    fn enable_quantum_2d_domain_impl(
        &mut self,
        psi_re: &str,
        psi_im: &str,
        v: &str,
        nx: usize,
        ny: usize,
        dx: f64,
        dy: f64,
    ) -> Result<(), WasmError> {
        let wave = sim_world::build_quantum_2d_wave_from_raw(psi_re, psi_im, v, nx, ny, dx, dy)
            .map_err(WasmError::QuantumRawStateInvalid)?;
        self.inner.enable_quantum_2d(wave);
        Ok(())
    }

    /// Add Coupling——`sim_coupling::SphRigid`の薄い写像。D23(注ぐ水)向け。
    /// SPHドメインが無効だと`SphRigid::new`が境界粒子を確保する先が無いため、
    /// 先に「＋ 流体 (SPH 水塊)」(`spawn_fluid_block`)でSPH流体を有効化して
    /// もらう必要がある——無効なら明示的に`Err`を返す(他のCoupling系
    /// メソッドの「無言で無効化けより失敗として伝わる」方針と同じ)。
    fn add_sph_rigid_coupling_impl(
        &mut self,
        body: usize,
        radius: f64,
        boundary_points: usize,
    ) -> Result<(), WasmError> {
        let id = self.try_body_id_at(body)?;
        let sph = self.inner.sph_mut().ok_or(WasmError::SphDomainNotEnabled)?;
        let coupling = sim_coupling::SphRigid::new(sph, id.index as usize, radius, boundary_points);
        self.inner.add_coupling(Box::new(coupling));
        Ok(())
    }

    /// Add Coupling——`sim_coupling::GridFluidRigid`の薄い写像。D14(煙と渦)向け。
    /// 格子流体ドメインが無効なら明示的に`Err`(`enable_grid_fluid_2d_domain`を
    /// 先に呼ぶ必要がある、`add_sph_rigid_coupling`と同じ方針)。
    fn add_grid_fluid_rigid_coupling_impl(
        &mut self,
        body: usize,
        half_width: f64,
        half_height: f64,
    ) -> Result<(), WasmError> {
        let id = self.try_body_id_at(body)?;
        if self.inner.grid_fluid().is_none() {
            return Err(WasmError::GridFluidDomainNotEnabled);
        }
        self.inner
            .add_coupling(Box::new(sim_coupling::GridFluidRigid::new(
                id.index as usize,
                half_width,
                half_height,
            )));
        Ok(())
    }

    /// Add Coupling——`sim_coupling::PistonGas`の薄い写像。D17(ピストン)向け。
    /// 気体区画が無効なら明示的に`Err`(`enable_gas_compartment`を先に呼ぶ
    /// 必要がある)。基準体積(`initial_volume`)はUIから明示的に渡す
    /// (シーンJSON側の`gas.volume`と同じ値を渡す規約、`CouplingJson::
    /// PistonGas`のdoc参照)。
    fn add_piston_gas_coupling_impl(
        &mut self,
        body: usize,
        axis_x: f64,
        axis_y: f64,
        axis_z: f64,
        area: f64,
        initial_volume: f64,
    ) -> Result<(), WasmError> {
        let id = self.try_body_id_at(body)?;
        if self.inner.gas().is_none() {
            return Err(WasmError::GasCompartmentNotEnabled);
        }
        let axis = Vec3::new(axis_x, axis_y, axis_z);
        let coupling = sim_coupling::PistonGas::new(
            &self.inner.mechanics().bodies,
            id.index as usize,
            axis,
            area,
            initial_volume,
        );
        self.inner.add_coupling(Box::new(coupling));
        Ok(())
    }

    /// Add Coupling——翼(薄翼理論)による揚力を持つ`sim_coupling::BuoyancyDrag`を
    /// 追加する(**残タスク完遂の縦串⑤増分**、`LiftModel::Wing`は物理コア側に
    /// 既に実装済みだったが、Add Couplingフォームでは`None`固定で到達できな
    /// かった)。`chord_local`/`span_local`は剛体ローカル座標(毎step姿勢で
    /// ワールドへ回す)。同じ剛体に複数回呼べる——主翼・水平尾翼(エレベーター)・
    /// 垂直尾翼(ラダー)・補助翼(エルロン)をそれぞれ別の翼として追加し、
    /// 戻り値のindexを`push_set_coupling_control_surface_deflection`で
    /// 個別に操作する、という使い方を想定している。
    /// 戻り値は`World::couplings()`の`index`(`CouplingInfo::index`と同じ並び)。
    #[allow(clippy::too_many_arguments)]
    fn add_wing_lift_coupling_impl(
        &mut self,
        body: usize,
        wing_area: f64,
        chord_x: f64,
        chord_y: f64,
        chord_z: f64,
        span_x: f64,
        span_y: f64,
        span_z: f64,
        atmosphere_density: f64,
        atmosphere_viscosity: f64,
    ) -> Result<usize, WasmError> {
        let id = self.try_body_id_at(body)?;
        let index = self
            .inner
            .add_coupling(Box::new(sim_coupling::BuoyancyDrag {
                body_index: id.index as usize,
                water: None,
                atmosphere: Some(sim_fluid::Atmosphere::still(
                    atmosphere_density,
                    atmosphere_viscosity,
                )),
                lift: Some(sim_coupling::LiftModel::Wing {
                    area: wing_area,
                    chord_local: Vec3::new(chord_x, chord_y, chord_z),
                    span_local: Vec3::new(span_x, span_y, span_z),
                    control_surface_deflection: 0.0,
                }),
            }));
        Ok(index)
    }

    /// Add Coupling——回転球のマグヌス効果による揚力を持つ`sim_coupling::
    /// BuoyancyDrag`を追加する(`add_wing_lift_coupling`と同じ理由で解禁)。
    fn add_magnus_lift_coupling_impl(
        &mut self,
        body: usize,
        radius: f64,
        atmosphere_density: f64,
        atmosphere_viscosity: f64,
    ) -> Result<usize, WasmError> {
        let id = self.try_body_id_at(body)?;
        let index = self
            .inner
            .add_coupling(Box::new(sim_coupling::BuoyancyDrag {
                body_index: id.index as usize,
                water: None,
                atmosphere: Some(sim_fluid::Atmosphere::still(
                    atmosphere_density,
                    atmosphere_viscosity,
                )),
                lift: Some(sim_coupling::LiftModel::MagnusSphere { radius }),
            }));
        Ok(index)
    }

    /// 登録済みの翼(`add_wing_lift_coupling`が返したindex)の操縦面舵角[rad]を
    /// 実行時に変更する(**残タスク完遂の縦串⑤増分**)。`Command::
    /// SetCouplingParam`(`CouplingParam::ControlSurfaceDeflection`)を積み、
    /// 次stepの先頭で適用される——他の`push_*`系メソッドと同じCommand経由の
    /// 規約(Playモード中でもリプレイ再現性と決定論が壊れない)。範囲外index・
    /// 翼以外のCouplingを指しても(無効な入力として無言で無視される、
    /// `Command`のdoc参照)エラーにはならない——UIから毎フレーム舵角を送っても
    /// 安全なように、あえて`Result`ではなく無条件に成功する設計にした。
    fn push_set_coupling_control_surface_deflection_impl(
        &mut self,
        coupling_index: usize,
        deflection_radians: f64,
    ) {
        self.inner.push_command(Command::SetCouplingParam {
            coupling_index,
            param: sim_coupling::CouplingParam::ControlSurfaceDeflection,
            value: deflection_radians,
        });
    }

    /// Add Coupling——`sim_coupling::BoussinesqBuoyancy`の薄い写像。
    /// 格子流体ドメインが無効なら明示的に`Err`(適用対象が無いため、
    /// `add_grid_fluid_rigid_coupling`と同じ方針)。
    fn add_boussinesq_buoyancy_coupling_impl(
        &mut self,
        thermal_node: usize,
        ambient_temperature: f64,
        thermal_expansion_coefficient: f64,
    ) -> Result<(), WasmError> {
        let node = self.try_thermal_node_index(thermal_node)?;
        if self.inner.grid_fluid().is_none() {
            return Err(WasmError::GridFluidDomainNotEnabled);
        }
        self.inner
            .add_coupling(Box::new(sim_coupling::BoussinesqBuoyancy {
                thermal_node: node,
                ambient_temperature,
                thermal_expansion_coefficient,
            }));
        Ok(())
    }

    /// Add Coupling——`sim_coupling::ConvectionLink`の薄い写像。流体の物性値
    /// (熱伝導率・動粘性・プラントル数・自然対流の体膨張係数)もUIから明示的に
    /// 指定できる(**残タスク完遂増分**、レビュー指摘「縮約禁止令」への対応
    /// ——以前は`ConvectionLink::default()`(空気20℃)固定だった)。
    /// `thermal_expansion_coefficient`は`<=0`なら`None`(理想気体近似
    /// $\beta=1/T_{film}$、`ConvectionLink`のdoc参照)として扱う——負の体膨張
    /// 係数は物理的に無意味なので、`Option`をUIのフラットな数値入力へ写す
    /// ための素直な符号化とした。`mode`は`0..=3`の整数(NaturalVerticalPlate/
    /// NaturalSphere/ForcedSphere/ForcedFlatPlate、UIのtitleツールチップ参照)。
    #[allow(clippy::too_many_arguments)]
    fn add_convection_link_coupling_impl(
        &mut self,
        fluid_node: usize,
        surface_node: usize,
        area: f64,
        characteristic_length: f64,
        mode: u32,
        fluid_thermal_conductivity: f64,
        kinematic_viscosity: f64,
        prandtl_number: f64,
        thermal_expansion_coefficient: f64,
    ) -> Result<(), WasmError> {
        let fluid_node = self.try_thermal_node_index(fluid_node)?;
        let surface_node = self.try_thermal_node_index(surface_node)?;
        let mode = match mode {
            0 => sim_coupling::ConvectionMode::NaturalVerticalPlate,
            1 => sim_coupling::ConvectionMode::NaturalSphere,
            2 => sim_coupling::ConvectionMode::ForcedSphere,
            _ => sim_coupling::ConvectionMode::ForcedFlatPlate,
        };
        self.inner
            .add_coupling(Box::new(sim_coupling::ConvectionLink {
                fluid_node,
                surface_node,
                area,
                characteristic_length,
                mode,
                fluid_thermal_conductivity,
                kinematic_viscosity,
                prandtl_number,
                thermal_expansion_coefficient: (thermal_expansion_coefficient > 0.0)
                    .then_some(thermal_expansion_coefficient),
            }));
        Ok(())
    }

    /// Add Coupling——`sim_coupling::PhaseChangeMorph`の薄い写像。D18(氷と
    /// 飲み物)向け。材質(融点・融解潜熱・固相/液相比熱)もUIから明示的に
    /// 指定できる(**残タスク完遂増分**、レビュー指摘「縮約禁止令」への対応
    /// ——以前は氷/水の定数に固定していた)。融解質量のSPH粒子生成
    /// (`melt_spawn`)はこのメソッドでは`None`のまま(既定、移行前と同じ挙動
    /// ——粒子生成のばらつき半径・乱数シードまで含めると汎用フォームの
    /// 枠を明確に超えるため、こちらは専用フォームが要る妥当な残作業として
    /// 正直に残す)。
    #[allow(clippy::too_many_arguments)]
    fn add_phase_change_morph_coupling_impl(
        &mut self,
        body: usize,
        thermal_node: usize,
        melting_temperature: f64,
        latent_heat_fusion: f64,
        specific_heat_solid: f64,
        specific_heat_liquid: f64,
        initial_mass: f64,
        conductance: f64,
        initial_enthalpy: f64,
    ) -> Result<(), WasmError> {
        let id = self.try_body_id_at(body)?;
        let node = self.try_thermal_node_index(thermal_node)?;
        let material = sim_thermal::PhaseMaterial {
            melting_temperature,
            latent_heat_fusion,
            specific_heat_solid,
            specific_heat_liquid,
        };
        let coupling = sim_coupling::PhaseChangeMorph::new(
            id.index as usize,
            node,
            material,
            initial_mass,
            conductance,
            initial_enthalpy,
        );
        self.inner.add_coupling(Box::new(coupling));
        Ok(())
    }

    /// フレーム軸オーバーレイ(設計docs/23-frontend/01-editor.md §1.3「フレーム
    /// サブモード」の土台)向けに、ROOTの子として指定角速度(z軸まわり)で自転する
    /// フレームを追加する(`World::add_frame`+`sim_core::FrameTree::step`が毎step
    /// 自動的に回転を進める)。返り値はこのフレームの`FrameId`(`frame_rotation_
    /// at_f32`に渡すindex)。
    fn add_rotating_frame_impl(&mut self, angular_velocity_z: f64) -> usize {
        let id = self.inner.add_frame(
            FrameId::ROOT,
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::new(0.0, 0.0, angular_velocity_z),
        );
        id.0 as usize
    }

    /// `frame_index`が有効な範囲内かを検証する(**2026-07-27の監査で追加**:
    /// `sim_core::FrameTree::frame`は範囲外の`FrameId`に対し生スライスindexで
    /// パニックする——`try_body_id_at`のdocと同じ理由で、フレームindexを扱う
    /// 各wasm公開メソッドの入口でここを通す)。フレームは`add_frame`/
    /// `add_child_frame`で単調増加するのみ(削除が無い)ため、
    /// `frame_index < frame_count()`が有効性の必要十分条件になる。
    fn check_frame_index(&self, frame_index: usize) -> Result<(), WasmError> {
        if frame_index < self.frame_count_impl() {
            Ok(())
        } else {
            Err(WasmError::FrameIndexOutOfRange {
                index: frame_index,
                count: self.frame_count_impl(),
            })
        }
    }

    /// `frame_index`番目のフレームの現在の姿勢(親フレームからの相対回転)を
    /// クォータニオン`[x, y, z, w]`(f32)で返す。
    /// `imported_probe_history_f64`と同じ理由で、index検証と値の取り出しは
    /// `frame_rotation_at_impl`側(ネイティブテスト可能)。
    ///
    /// **B16(ゼロコピー化)**: 戻り値は`self.view_buffers.frame_rotation`を
    /// エイリアスする一時的なビュー(`HotPathViewBuffers`のdoc参照)。呼び出し側は
    /// 値を読み切ってから次のWasm呼び出しへ進むこと。
    pub fn frame_rotation_at_f32(&mut self, frame_index: usize) -> Result<Float32Array, JsValue> {
        let rotation = self.frame_rotation_at_impl(frame_index)?;
        let buf = &mut self.view_buffers.frame_rotation;
        buf.clear();
        buf.extend_from_slice(&rotation);
        // SAFETY: `buf`への書き込みはここまでで完了しており、このビューを
        // 構築した後は関数を抜けるだけ(`HotPathViewBuffers`のdoc参照)。
        Ok(unsafe { Float32Array::view(buf) })
    }

    /// `frame_rotation_at_f32`の実体。
    fn frame_rotation_at_impl(&self, frame_index: usize) -> Result<[f32; 4], WasmError> {
        self.check_frame_index(frame_index)?;
        let rotation = self
            .inner
            .frames()
            .frame(FrameId(frame_index as u32))
            .rotation_in_parent;
        Ok([
            rotation.x as f32,
            rotation.y as f32,
            rotation.z as f32,
            rotation.w as f32,
        ])
    }

    /// 全フレーム数(ROOT含む、`sim_core::FrameTree::frame_count`の素通し)。
    /// フレーム階層ドリルインUI(Hierarchyの「Frames」サブツリー)がフレーム
    /// 一覧を列挙するために使う。
    fn frame_count_impl(&self) -> usize {
        self.inner.frames().frame_count()
    }

    /// `frame_index`番目のフレームの親のindex。ROOT自身(index 0)は親を
    /// 持たないため`-1`を返す(フレーム階層ドリルインUIがツリー構造を
    /// 組み立てるための情報)。
    fn frame_parent_index_impl(&self, frame_index: usize) -> Result<i32, WasmError> {
        self.check_frame_index(frame_index)?;
        Ok(
            match self
                .inner
                .frames()
                .frame(FrameId(frame_index as u32))
                .parent
            {
                Some(parent) => parent.0 as i32,
                None => -1,
            },
        )
    }

    /// `frame_index`番目のフレームのROOT(ワールド)座標系での位置
    /// (`sim_core::FrameTree::transform_to_root`)。`frame_rotation_at_f32`
    /// (親フレームからの相対回転のみ、単一のROOT直下フレームを想定していた
    /// 旧API)と異なり、複数フレームが親子関係を持つ場合(フレーム階層
    /// ドリルインUI)でも階層を遡って合成した実際のワールド位置を返す。
    ///
    /// **B16(ゼロコピー化)**: `frame_rotation_at_f32`と同じ規約
    /// (`self.view_buffers.frame_world_position`をエイリアスする一時的な
    /// ビュー)。
    pub fn frame_world_position_f32(
        &mut self,
        frame_index: usize,
    ) -> Result<Float32Array, JsValue> {
        let position = self.frame_world_position_impl(frame_index)?;
        let buf = &mut self.view_buffers.frame_world_position;
        buf.clear();
        buf.extend_from_slice(&position);
        // SAFETY: `frame_rotation_at_f32`と同じ(`HotPathViewBuffers`のdoc参照)。
        Ok(unsafe { Float32Array::view(buf) })
    }

    /// `frame_world_position_f32`の実体(`frame_rotation_at_impl`と同じ規約)。
    fn frame_world_position_impl(&self, frame_index: usize) -> Result<[f32; 3], WasmError> {
        self.check_frame_index(frame_index)?;
        let position = self
            .inner
            .frames()
            .transform_to_root(FrameId(frame_index as u32))
            .position;
        Ok([position.x as f32, position.y as f32, position.z as f32])
    }

    /// `frame_index`番目のフレームのROOT(ワールド)座標系での姿勢
    /// (`frame_world_position_f32`と同じ理由で`transform_to_root`を使う)。
    ///
    /// **B16(ゼロコピー化)**: `frame_rotation_at_f32`と同じ規約
    /// (`self.view_buffers.frame_world_rotation`をエイリアスする一時的な
    /// ビュー)。
    pub fn frame_world_rotation_f32(
        &mut self,
        frame_index: usize,
    ) -> Result<Float32Array, JsValue> {
        let rotation = self.frame_world_rotation_impl(frame_index)?;
        let buf = &mut self.view_buffers.frame_world_rotation;
        buf.clear();
        buf.extend_from_slice(&rotation);
        // SAFETY: `frame_rotation_at_f32`と同じ(`HotPathViewBuffers`のdoc参照)。
        Ok(unsafe { Float32Array::view(buf) })
    }

    /// `frame_world_rotation_f32`の実体(`frame_rotation_at_impl`と同じ規約)。
    fn frame_world_rotation_impl(&self, frame_index: usize) -> Result<[f32; 4], WasmError> {
        self.check_frame_index(frame_index)?;
        let rotation = self
            .inner
            .frames()
            .transform_to_root(FrameId(frame_index as u32))
            .rotation;
        Ok([
            rotation.x as f32,
            rotation.y as f32,
            rotation.z as f32,
            rotation.w as f32,
        ])
    }

    /// フレーム階層ドリルインUI(設計docs/23-frontend/01-editor.md §1.3
    /// 「フレームサブモード」)向けに、任意の既存フレーム(`parent_index`、
    /// 0=ROOT)の子として新規フレームを追加する(`add_rotating_frame`の
    /// 一般化——親をROOT固定ではなく任意に選べる)。`origin_offset_*`は
    /// 親フレーム内での原点位置(Scene View上でネストしたフレームが重ならない
    /// よう、呼び出し側が親からのオフセットを指定する)。返り値は新規フレームの
    /// index(`frame_world_position_f32`/`frame_world_rotation_f32`に渡す)。
    fn add_child_frame_impl(
        &mut self,
        parent_index: usize,
        origin_offset_x: f64,
        origin_offset_y: f64,
        origin_offset_z: f64,
        angular_velocity_z: f64,
    ) -> Result<usize, WasmError> {
        // `World::add_frame`は範囲外の親`FrameId`に対し`assert!`でパニックする
        // (`sim_core::FrameTree::add_frame`)ため、`try_body_id_at`のdocと同じ
        // 理由でここも事前に検証する。
        self.check_frame_index(parent_index)?;
        let id = self.inner.add_frame(
            FrameId(parent_index as u32),
            Vec3::new(origin_offset_x, origin_offset_y, origin_offset_z),
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::new(0.0, 0.0, angular_velocity_z),
        );
        Ok(id.0 as usize)
    }

    /// 流体場オーバーレイ(設計docs/23-frontend/01-editor.md §1.3「流体場」の土台)
    /// 向けに、小さな水塊(3×3×3粒子)+**それを受け止める容器**(3層の床と
    /// 4面の側壁、`SphFluid::add_boundary_particle`)を追加する。既にSPH流体が
    /// 有効なら(`World::sph_mut`)そこへ粒子を追加する(複数回スポーンすると
    /// 水塊が増えていく、`fluid_spawn_count`でX方向にずらして重なりを避ける)。
    /// まだ無効なら新規`SphFluid`を構築して有効化する(初回のみ)。
    ///
    /// **QA不具合2で書き換えた点**。以前は境界が「1層の床」だけで壁が無く、
    /// $h=0.15$ に対し格子間隔 $\Delta x=0.1$ という設計§4.1・§9 の規約
    /// ($\Delta x = h/2$)を外れた組み合わせだった。着水した水は横へ薄く広がって
    /// 1〜2粒子厚の膜になり、膜はカーネル欠損で密度推定が静止密度を割るため
    /// Tait 状態方程式の圧力が 0 にクランプされ、境界の反発力
    /// ($2p_i/\rho_i^2$ に比例)がまったく立たない——支えを失った水は毎step
    /// 少しずつ沈み、**床を抜けて落ち続けた**(実測: 2 秒で 27 粒子中 20 個が
    /// 床より 0.5 m 下)。
    ///
    /// そこで (1) $h = 2\Delta x$ として規約を満たし、(2) 床を3層にし、
    /// (3) **水塊がちょうど収まる大きさの側壁4面**を足した。容器を水塊より
    /// 大きくしてはいけない——広い容器では同じ水量が薄い膜になってしまい、
    /// 壁を足した意味が無くなる(内側をちょうど 3×3 格子にして、水が
    /// 3 粒子ぶんの深さを保てるようにしている)。
    fn spawn_fluid_block_impl(&mut self) {
        let rho0: f64 = 1000.0;
        let dx: f64 = 0.1;
        // 設計§4.1・§9 の規約 Δx = h/2。
        let h: f64 = 2.0 * dx;
        let n: i32 = 3;
        // 容器の壁の厚み(層数)。設計§4.1・§9 の既定。
        let layers: i32 = 3;
        let floor_y = -dx;

        let spawn_index = self.fluid_spawn_count;
        self.fluid_spawn_count += 1;
        // **落差を Δx(0.1 m)に抑える**(QA不具合2)。以前は y=2.0 から
        // 床(y=-0.1)まで 2.1 m 落としていて着水速度が 6.4 m/s になり、
        // 人工音速 20 m/s に対して Mach 0.32 だった。WCSPH が弱圧縮の近似として
        // 成り立つのは流速が人工音速より十分小さいとき(設計§2.2)だけなので、
        // この落差では**状態方程式が着水を止めるだけの圧力を作れず**、
        // 容器を足しても水が突き抜けた(実測 27 粒子中 5〜12 個)。
        //
        // **3×3×3 = 27 粒子という小さな塊は SPH としてそもそも際どい**——
        // すべての粒子が自由表面粒子で、密度推定が静止密度を割る粒子が常にある
        // (実測 606.6)。落差を大きく取ると着水時に過圧縮して圧力が 10 万 Pa
        // 級に跳ね、**塊が容器から弾き飛ばされて**近傍ゼロ(密度 = 自己寄与
        // 318.3 のみ)の粒子が生まれ、以後は何にも捕まらず落ち続けた。
        // 「流体オーバーレイを見せる」のが目的の追加なので、静かに置いて
        // 少しだけ落ちる(=容器に受け止められるのが見える)挙動にする。
        let drop = dx;
        let origin = Vec3::new(3.0 + spawn_index as f64 * 1.5, floor_y + dx + drop, 0.0);

        // **人工音速は落差から決める**。静水圧平衡試験も `c_s = 10 u_max`
        // (`u_max = sqrt(2gH)`)という同じ規則を使っている(設計§2.2・§9)。
        // 落差から決めれば着水時の Mach 数が 0.1 に収まる。
        let drop_height = (origin.y - floor_y).max(dx);
        let u_max = (2.0 * 9.80665 * drop_height).sqrt();
        let c_s: f64 = 10.0 * u_max;

        let mut particle_positions = Vec::with_capacity((n * n * n) as usize);
        for ix in 0..n {
            for iy in 0..n {
                for iz in 0..n {
                    particle_positions
                        .push(origin + Vec3::new(ix as f64 * dx, iy as f64 * dx, iz as f64 * dx));
                }
            }
        }

        // 容器は格子に載せて組む。内側は水塊とちょうど同じ 3×3 格子
        // (x,z のインデックス 0..n)で、その外側 `layers` 層が壁になる。
        let mut boundary_positions = Vec::new();
        let at = |ix: i32, iy: i32, iz: i32| {
            Vec3::new(
                origin.x + ix as f64 * dx,
                floor_y + iy as f64 * dx,
                origin.z + iz as f64 * dx,
            )
        };
        let lo = -layers; // 壁の外端。
        let hi = n - 1 + layers; // 反対側の壁の外端。
                                 // 床(`layers`層、壁の下まで敷いて角を埋める)。
        for ix in lo..=hi {
            for iz in lo..=hi {
                for l in 0..layers {
                    boundary_positions.push(at(ix, -l, iz));
                }
            }
        }
        // 側壁4面(床の1段上から、**着水の飛沫が収まる高さ**まで)。
        //
        // 以前は `n + layers`(= 6Δx、y=+0.5)で「水塊がすべて収まる高さ」しか
        // 無かった。ところがこのシーンで実際に起きるのは貫通ではなく**射出**で、
        // 着水の過圧縮で弾かれた粒子が縁を越え、容器の外側へ落ちていた
        // (実測: y=+0.80 ≒ 9Δx まで上がり、壁の footprint 内にいながら
        // 上端 y=+0.5 を飛び越していた)。
        //
        // この余裕の無さが、プラットフォーム間で結果が変わっていた原因である。
        // 力学の step 経路には `sin`/`cos`/`exp`/`atan2` が多数あり、これらは
        // Rust std が各OSの libm へ委譲するため IEEE-754 の保証が及ばず 1 ULP
        // 差が出る。その差が「縁を越えるか越えないか」を分けていた
        // (Linux/macOS では収まり、Windows では 1 粒子が越えていた)。
        //
        // 初期位置へ相対摂動を与えて 48 回ずつ漏れを掃引した実測:
        //   壁高 6Δx : 摂動 1e-6 で 28/48 失敗
        //   壁高 12Δx: 摂動 1e-15〜1e-6 のいずれでも 0/48
        // 飛沫の到達高さ(≒3n Δx)を覆う `3n + layers` を採る。
        let wall_top = 3 * n + layers;
        for iy in 1..=wall_top {
            for ix in lo..=hi {
                for iz in lo..=hi {
                    let outside_x = ix < 0 || ix > n - 1;
                    let outside_z = iz < 0 || iz > n - 1;
                    if outside_x || outside_z {
                        boundary_positions.push(at(ix, iy, iz));
                    }
                }
            }
        }

        if let Some(sph) = self.inner.sph_mut() {
            for p in particle_positions {
                sph.add_particle(p, Vec3::ZERO);
            }
            for b in boundary_positions {
                sph.add_boundary_particle(b);
            }
        } else {
            let mut sph = sim_fluid::SphFluid::new(h, rho0, c_s);
            sph.mass = rho0 * dx.powi(3);
            for p in particle_positions {
                sph.add_particle(p, Vec3::ZERO);
            }
            for b in boundary_positions {
                sph.add_boundary_particle(b);
            }
            self.inner.enable_sph(sph);
        }
    }

    /// `spawn_fluid_block`を呼んだ回数(Hierarchyの「Fluids」概要表示用)。
    fn fluid_spawn_count_impl(&self) -> u32 {
        self.fluid_spawn_count
    }

    /// 流体粒子数(境界粒子は含まない、`fluid_particle_positions_f32`と同じ体系)。
    /// 流体ドメインが有効でなければ0。
    fn fluid_particle_count_impl(&self) -> usize {
        self.inner.sph().map_or(0, |s| s.position.len())
    }

    /// 全流体粒子の位置をフラットな`[x0,y0,z0,x1,y1,z1,...]`(f32)で返す
    /// (毎フレーム粒子数分`body_position_at_f32`相当を個別呼び出しするのは
    /// wasm境界越えのオーバーヘッドが大きいため、1回のクエリにまとめた)。
    ///
    /// **B16(ゼロコピー化)**: `quantum_1d_density_f32`と同じ規約
    /// (`self.view_buffers.fluid_particle_positions`をエイリアスする一時的な
    /// ビュー)。
    pub fn fluid_particle_positions_f32(&mut self) -> Float32Array {
        let buf = &mut self.view_buffers.fluid_particle_positions;
        buf.clear();
        if let Some(sph) = self.inner.sph() {
            buf.extend(
                sph.position
                    .iter()
                    .flat_map(|p| [p.x as f32, p.y as f32, p.z as f32]),
            );
        }
        // SAFETY: `quantum_1d_density_f32`と同じ(`HotPathViewBuffers`のdoc参照)。
        unsafe { Float32Array::view(buf) }
    }

    /// **境界粒子**の位置をフラットな`[x0,y0,z0,...]`(f32)で返す。
    ///
    /// 水を受け止めている器は、これまで**画面のどこにも描かれていなかった**
    /// ——「水のかたまりが落ちて、容器に溜まります」と書いてある隣で、真っ暗な
    /// 空間に水色の塊が浮いているだけに見えた(利用者役①の観察)。器は物理側に
    /// 境界粒子として実在するので、その位置をそのまま渡して描けるようにする。
    /// 読み出すだけで、計算には一切触らない。
    ///
    /// 規約は`fluid_particle_positions_f32`と同じ(一時的なビュー)。
    pub fn fluid_boundary_positions_f32(&mut self) -> Float32Array {
        let buf = &mut self.view_buffers.fluid_boundary_positions;
        buf.clear();
        if let Some(sph) = self.inner.sph() {
            buf.extend(
                sph.boundary_position
                    .iter()
                    .flat_map(|p| [p.x as f32, p.y as f32, p.z as f32]),
            );
        }
        // SAFETY: `fluid_particle_positions_f32`と同じ。
        unsafe { Float32Array::view(buf) }
    }

    /// `index`番目のボディの位置 [x, y, z](f32)。
    /// `imported_probe_history_f64`と同じ理由で、index検証と値の取り出しは
    /// `body_position_at_impl`側(ネイティブテスト可能)。
    ///
    /// **B16(ゼロコピー化)**: 以前の`Float32Array`の組み立て方
    /// (`new_with_length`+`set_index`)から、`self.view_buffers.body_position`を
    /// エイリアスする一時的なビューを返す形へ変えた(`HotPathViewBuffers`の
    /// doc参照)。呼び出し側は値を読み切ってから次のWasm呼び出しへ進むこと。
    pub fn body_position_at_f32(&mut self, index: usize) -> Result<Float32Array, JsValue> {
        let p = self.body_position_at_impl(index)?;
        let buf = &mut self.view_buffers.body_position;
        buf.clear();
        buf.extend_from_slice(&p);
        // SAFETY: `buf`への書き込みはここまでで完了しており、このビューを
        // 構築した後は関数を抜けるだけ(`HotPathViewBuffers`のdoc参照)。
        Ok(unsafe { Float32Array::view(buf) })
    }

    /// `body_position_at_f32`の実体。
    fn body_position_at_impl(&self, index: usize) -> Result<[f32; 3], WasmError> {
        let id = self.try_body_id_at(index)?;
        let p = self
            .inner
            .body_position(id)
            .expect("body is created in new() and never removed");
        Ok([p.x as f32, p.y as f32, p.z as f32])
    }

    /// `index`番目のボディの位置 [x, y, z]を**f64のまま**返す
    /// (`read_component("body_position_at_f64", "<index>")`の実体、QA不具合6)。
    ///
    /// `body_position_at_f32`はレンダリングループのホットパスなので f32 の
    /// `Float32Array`を返す。ところが f32 の刻みは粒子列の端(x = 0.299 m)で
    /// **3.0×10⁻⁸ m** あり、D25(ブラウン運動)の 2×10⁻⁹ m オーダーの変位は
    /// この量子化ノイズに完全に埋もれる——UI から計算したアンサンブル MSD が
    /// 解析解 $6Dt$ の 4.4 倍になっていたのは、物理ではなく**量子化誤差を
    /// 測っていた**ためだった。等分配則($\langle v^2\rangle$、速度は
    /// 10⁻³ m/s オーダー)は f32 でも足りるが、拡散の定量検証には届かない。
    ///
    /// **描画には使わない**。`read_component`は1呼び出しごとに JSON 文字列を
    /// 作るので、粒子数ぶん毎フレーム回すホットパスは
    /// `body_position_at_f32`のままにする(`apply_component`のdoc
    /// 「正直な適用範囲」と同じ線引き)。精密な測定・検証のときだけこちらを使う。
    fn body_position_at_f64_impl(&self, index: usize) -> Result<[f64; 3], WasmError> {
        let id = self.try_body_id_at(index)?;
        let p = self
            .inner
            .body_position(id)
            .expect("body is created in new() and never removed");
        Ok([p.x, p.y, p.z])
    }

    /// `index`番目のボディの速度 [vx, vy, vz](f32)。
    ///
    /// **B16(ゼロコピー化)**: `body_position_at_f32`と同じ規約
    /// (`self.view_buffers.body_velocity`をエイリアスする一時的なビュー)。
    pub fn body_velocity_at_f32(&mut self, index: usize) -> Result<Float32Array, JsValue> {
        let v = self.body_velocity_at_impl(index)?;
        let buf = &mut self.view_buffers.body_velocity;
        buf.clear();
        buf.extend_from_slice(&v);
        // SAFETY: `body_position_at_f32`と同じ(`HotPathViewBuffers`のdoc参照)。
        Ok(unsafe { Float32Array::view(buf) })
    }

    /// `body_velocity_at_f32`の実体(`body_position_at_impl`と同じ規約)。
    fn body_velocity_at_impl(&self, index: usize) -> Result<[f32; 3], WasmError> {
        let id = self.try_body_id_at(index)?;
        let v = self
            .inner
            .body_velocity(id)
            .expect("body is created in new() and never removed");
        Ok([v.x as f32, v.y as f32, v.z as f32])
    }

    /// `index`番目のボディの姿勢クォータニオン [x, y, z, w](f32)。
    ///
    /// **B16(ゼロコピー化)**: `body_position_at_f32`と同じ規約
    /// (`self.view_buffers.body_rotation`をエイリアスする一時的なビュー)。
    pub fn body_rotation_at_f32(&mut self, index: usize) -> Result<Float32Array, JsValue> {
        let q = self.body_rotation_at_impl(index)?;
        let buf = &mut self.view_buffers.body_rotation;
        buf.clear();
        buf.extend_from_slice(&q);
        // SAFETY: `body_position_at_f32`と同じ(`HotPathViewBuffers`のdoc参照)。
        Ok(unsafe { Float32Array::view(buf) })
    }

    /// `body_rotation_at_f32`の実体(`body_position_at_impl`と同じ規約)。
    fn body_rotation_at_impl(&self, index: usize) -> Result<[f32; 4], WasmError> {
        let id = self.try_body_id_at(index)?;
        let q = self
            .inner
            .body_rotation(id)
            .expect("body is created in new() and never removed");
        Ok([q.x as f32, q.y as f32, q.z as f32, q.w as f32])
    }

    /// Editモードの回転Gizmo向けの直接編集(`set_body_position_at`の姿勢版、
    /// 同じくCommandキューを経由しない直接書き換え)。
    fn set_body_rotation_at_impl(
        &mut self,
        index: usize,
        x: f64,
        y: f64,
        z: f64,
        w: f64,
    ) -> Result<(), WasmError> {
        let id = self.try_body_id_at(index)?;
        self.inner.mechanics_mut().bodies.rotation[id.index as usize] =
            sim_math::Quat { x, y, z, w };
        Ok(())
    }

    /// Scale Gizmo(縮約実装、`sim_world::World::set_body_shape`のdoc参照)——
    /// ボディの形状をスポーン時の寸法(`base_shape`、床は対象外)の`scale`倍に
    /// 置き換え、質量・慣性を再計算する。`scale`はドラッグ開始時点からの
    /// 相対値ではなく、常に基準形状からの絶対倍率(Translate/Rotate Gizmoの
    /// 「ドラッグ開始値+差分」ではなく「基準値×絶対倍率」という設計、複数回の
    /// ドラッグを重ねても誤差が蓄積しない)。
    fn set_body_scale_at_impl(&mut self, index: usize, scale: f64) -> Result<(), WasmError> {
        let id = self.try_body_id_at(index)?;
        if index == 0 {
            return Err(WasmError::GroundHasNoScaleHandle);
        }
        let base_shape = self.try_body_meta_at(index)?.base_shape.clone();
        let scaled_shape = match base_shape {
            Shape::Sphere { radius } => Shape::Sphere {
                radius: radius * scale,
            },
            Shape::Box { half_extents } => Shape::Box {
                half_extents: half_extents.scale(scale),
            },
            other => other,
        };
        self.inner.set_body_shape(id, scaled_shape);
        Ok(())
    }

    /// **軸別スケール(群2)**。設計 docs/23-frontend/01-editor.md §1.2 の Gizmo は
    /// Transform を直接編集するもので、スケールも当然軸別に効く。これまでは
    /// 単一倍率(`set_body_scale_at`)しか無く、**細長い箱を作れなかった**
    /// (斜面・板・棒はどれも非等方な箱で表すのが自然なので、実際に困る)。
    ///
    /// **球には軸別スケールが効かない**——`Shape::Sphere` は半径1つで表され、
    /// 楕円体は `Shape` に存在しないため。設計の形状一覧(Sphere/Box/Plane/
    /// Capsule/Convex)に楕円体が無い以上、ここで勝手に作るより「効かない」
    /// ことを返り値で正直に伝える(`false` を返す)。球は `set_body_scale_at`
    /// の等方スケールを使う。
    fn set_body_scale_xyz_at_impl(
        &mut self,
        index: usize,
        sx: f64,
        sy: f64,
        sz: f64,
    ) -> Result<bool, WasmError> {
        let id = self.try_body_id_at(index)?;
        if index == 0 {
            return Err(WasmError::GroundHasNoScaleHandle);
        }
        for s in [sx, sy, sz] {
            if !s.is_finite() || s <= 0.0 {
                return Err(WasmError::InvalidScaleComponent);
            }
        }
        let base_shape = self.try_body_meta_at(index)?.base_shape.clone();
        let scaled_shape = match base_shape {
            Shape::Box { half_extents } => Shape::Box {
                half_extents: sim_math::Vec3::new(
                    half_extents.x * sx,
                    half_extents.y * sy,
                    half_extents.z * sz,
                ),
            },
            // 球・カプセル・平面は軸別に変形できない(モジュールdoc参照)。
            _ => return Ok(false),
        };
        self.inner.set_body_shape(id, scaled_shape);
        Ok(true)
    }

    /// 1 world step。1s相当のstep数ごとにTimelineスナップショットを
    /// リングバッファへ記録する(モジュールdoc「スナップショットリングバッファ」
    /// 参照、既存の`World::snapshot`をそのまま使う)。
    pub fn step(&mut self) {
        // 巻き戻した位置から**実際に進める**ときが、記録済みの未来を捨てる
        // 瞬間である(`restored_to`のdoc参照)。ここから先は新しい時間の筋。
        if let Some(index) = self.restored_to.take() {
            self.snapshots.truncate(index + 1);
        }
        self.inner.step();
        if self
            .inner
            .step_count()
            .is_multiple_of(self.snapshot_interval_steps)
        {
            if self.snapshots.len() >= SNAPSHOT_RING_CAPACITY {
                self.snapshots.pop_front();
            }
            self.snapshots.push_back(self.inner.snapshot());
        }
    }

    /// Timelineスクラバが表示できるスナップショット数(モジュールdoc参照)。
    fn snapshot_count_impl(&self) -> usize {
        self.snapshots.len()
    }

    /// `index`が有効なスナップショットindexかを検証する(**2026-07-27の監査で
    /// 追加**: `VecDeque`の生indexアクセスは範囲外でパニックする、
    /// `try_body_id_at`のdocと同じ理由)。
    fn try_snapshot_at(&self, index: usize) -> Result<&World, WasmError> {
        self.snapshots
            .get(index)
            .ok_or(WasmError::SnapshotIndexOutOfRange {
                index,
                count: self.snapshots.len(),
            })
    }

    /// `index`番目のスナップショットの記録時刻(秒、古い順)。
    fn snapshot_time_at_impl(&self, index: usize) -> Result<f64, WasmError> {
        Ok(self.try_snapshot_at(index)?.time())
    }

    /// Timelineスクラバ操作: `index`番目のスナップショットへ巻き戻す(既存の
    /// `World::restore`をそのまま使う)。
    ///
    /// 巻き戻した時点より後のスナップショットは、**そこから進めた瞬間に**
    /// 実際の未来ではなくなる——破棄するのはその瞬間であって、巻き戻した
    /// 時点ではない(`restored_to`のdoc参照)。止めたまま前後に行き来する
    /// 操作を壊さないための区別である。
    fn restore_snapshot_impl(&mut self, index: usize) -> Result<(), WasmError> {
        // `try_snapshot_at`は`&self`(disjointでない全体借用)を取るため、
        // その戻り値を保持したまま`&mut self.inner`は取れない。フィールドへ
        // 直接アクセスして借用チェッカに`snapshots`と`inner`が別フィールド
        // であることを見せる。
        let snapshot = self
            .snapshots
            .get(index)
            .ok_or(WasmError::SnapshotIndexOutOfRange {
                index,
                count: self.snapshots.len(),
            })?;
        self.inner.restore(snapshot);
        // ここでは捨てない——止めたまま前後に行き来できるようにする
        // (`restored_to`のdoc参照)。捨てるのは次に進めるとき。
        self.restored_to = Some(index);
        Ok(())
    }

    /// Timelineのブックマーク(設計docs/23-frontend/01-editor.md §1.4
    /// 「ブックマーク: 任意時点にラベル付けし、後で戻れる」)。リングバッファの
    /// 退避に晒されない別領域へ、現在時点のスナップショットをラベル付きで保存する
    /// (既存の`World::snapshot`をそのまま使う)。数の上限は設けない(縮約実装、
    /// シーンJSONと一緒に出す「共有」用途は未実装)。
    fn add_bookmark_impl(&mut self, label: String) {
        self.bookmarks.push((label, self.inner.snapshot()));
    }

    fn bookmark_count_impl(&self) -> usize {
        self.bookmarks.len()
    }

    /// `index`が有効なブックマークindexかを検証する(**2026-07-27の監査で
    /// 追加**: `Vec`の生indexアクセスは範囲外でパニックする、
    /// `try_body_id_at`のdocと同じ理由)。
    fn try_bookmark_at(&self, index: usize) -> Result<&(String, World), WasmError> {
        self.bookmarks
            .get(index)
            .ok_or(WasmError::BookmarkIndexOutOfRange {
                index,
                count: self.bookmarks.len(),
            })
    }

    fn bookmark_label_at_impl(&self, index: usize) -> Result<String, WasmError> {
        Ok(self.try_bookmark_at(index)?.0.clone())
    }

    fn bookmark_time_at_impl(&self, index: usize) -> Result<f64, WasmError> {
        Ok(self.try_bookmark_at(index)?.1.time())
    }

    /// ブックマークのエクスポート(設計docs/23-frontend/01-editor.md §6
    /// 「保存・共有: シーンJSON+Replay+ブックマークを単一ファイルとして
    /// エクスポート」)。`sim_world::to_scenario`(**残タスク完遂の縦串④
    /// 増分で手書き実装を置き換えた**——Task#4当時「手書きの
    /// `export_scene_json` を置き換える」が残タスクとして明記されていた。
    /// 検証タブでprobeが1本も出ない実バグ(旧実装は`world`/`bodies`しか
    /// 書き出さず、probes/joints/couplings/thermal/circuit/astro/gas は
    /// 常に欠落していた)を追う過程で発覚した)。
    fn bookmark_export_scene_json_impl(&self, index: usize) -> Result<String, WasmError> {
        let (label, snapshot) = self.try_bookmark_at(index)?;
        let scenario = sim_world::to_scenario(snapshot, &format!("bookmark-{label}"))
            .map_err(WasmError::ScenarioExport)?;
        serde_json::to_string(&scenario)
            .map_err(|e| WasmError::ScenarioSerializeFailed(e.to_string()))
    }

    /// **現在の状態をシーンJSONとして書き出す(群2)**。単一ファイルExport
    /// (設計 §6「シーンJSON+Replay+ブックマークを単一ファイルとして
    /// エクスポート」)が使う。`bookmark_export_scene_json`と同じく
    /// `sim_world::to_scenario`を経由する(旧実装は`world`/`bodies`だけの
    /// 手書きシリアライズだった、上のdoc参照)。
    fn export_scene_json_impl(&self) -> Result<String, WasmError> {
        let scenario =
            sim_world::to_scenario(&self.inner, "current").map_err(WasmError::ScenarioExport)?;
        serde_json::to_string(&scenario)
            .map_err(|e| WasmError::ScenarioSerializeFailed(e.to_string()))
    }

    /// ブックマークへ巻き戻す。`restore_snapshot`と異なり、ブックマーク自体は
    /// 巻き戻し後も残す(いつでも同じブックマークへ再度戻れるように)。ただし
    /// リングバッファ側のスナップショットは、もはや実際の未来を表さないため
    /// 全て破棄する(新しいタイムラインがそこから再開する)。
    fn restore_bookmark_impl(&mut self, index: usize) -> Result<(), WasmError> {
        // `restore_snapshot`と同じ理由でフィールドへ直接アクセスする
        // (借用チェッカに`bookmarks`と`inner`が別フィールドであることを見せる)。
        let (_, snapshot) =
            self.bookmarks
                .get(index)
                .ok_or(WasmError::BookmarkIndexOutOfRange {
                    index,
                    count: self.bookmarks.len(),
                })?;
        self.inner.restore(snapshot);
        self.snapshots.clear();
        Ok(())
    }

    fn time_impl(&self) -> f64 {
        self.inner.time()
    }

    fn step_count_impl(&self) -> u64 {
        self.inner.step_count()
    }

    /// 決定論検証・UI 表示用の状態ハッシュ(16進文字列)。
    fn state_hash_impl(&self) -> String {
        format!("{:016x}", self.inner.state_hash())
    }

    /// 箱のy座標プローブの観測履歴(古い順)。エディタのProbe Graphsパネル
    /// (設計docs/23-frontend/01-editor.md §1.4)デモ用。
    ///
    /// **2026-07-28のD9/D34/D35増分**: `y_probe`が`None`(`from_scene_json`が
    /// 力学ボディを持たないシーン——D9/D34/D35——を読み込んだ場合)なら空配列を
    /// 返す(`.expect`していた旧実装はここでパニックしていた——既定シーン
    /// (`WasmWorld::new`)は必ず`Some`のため挙動は変わらない)。
    ///
    /// **B16(ゼロコピー化)**: `quantum_1d_density_f32`と同じ規約
    /// (`self.view_buffers.y_probe_history`をエイリアスする一時的なビュー)。
    pub fn y_probe_history_f64(&mut self) -> Float64Array {
        let buf = &mut self.view_buffers.y_probe_history;
        buf.clear();
        if let Some(handle) = self.y_probe {
            let probe = self
                .inner
                .probe(handle)
                .expect("y_probe, once Some, is registered and never removed");
            buf.extend(probe.history().copied());
        }
        // SAFETY: `quantum_1d_density_f32`と同じ(`HotPathViewBuffers`のdoc参照)。
        unsafe { Float64Array::view(buf) }
    }

    /// 箱の速さ(`ProbeTarget::BodySpeed`)プローブの観測履歴(古い順)。
    /// y座標プローブと同じProbe Graphsパネルに2系列目として表示するデモ用。
    /// `y_probe_history_f64`と同じ理由で`speed_probe`が`None`なら空配列を返す。
    ///
    /// **B16(ゼロコピー化)**: `y_probe_history_f64`と同じ規約
    /// (`self.view_buffers.speed_probe_history`をエイリアスする一時的な
    /// ビュー)。
    pub fn speed_probe_history_f64(&mut self) -> Float64Array {
        let buf = &mut self.view_buffers.speed_probe_history;
        buf.clear();
        if let Some(handle) = self.speed_probe {
            let probe = self
                .inner
                .probe(handle)
                .expect("speed_probe, once Some, is registered and never removed");
            buf.extend(probe.history().copied());
        }
        // SAFETY: `quantum_1d_density_f32`と同じ(`HotPathViewBuffers`のdoc参照)。
        unsafe { Float64Array::view(buf) }
    }

    /// エディタのPlayモード操作(設計docs/23-frontend/01-editor.md §4「介入は全て
    /// Commandとしてキューに積まれ、次ステップ先頭で適用される」)、`index`番目の
    /// ボディに力を加えるCommandをキューに積む。重心への加力(トルク無し、
    /// `point=None`)。**2026-07-27の残タスク完遂セッションで`body_index`引数を
    /// 追加**(以前は常に固定の箱(index 1)決め打ちだったが、シーンギャラリー
    /// (`from_scene_json`)で任意のボディが存在するようになったため、呼び出し側が
    /// 対象を選べるようにした)。
    fn push_apply_force_impl(
        &mut self,
        body_index: usize,
        fx: f64,
        fy: f64,
        fz: f64,
    ) -> Result<(), WasmError> {
        let body = self.try_body_id_at(body_index)?;
        self.inner.push_command(Command::ApplyForce {
            body,
            force: sim_math::Vec3::new(fx, fy, fz),
            point: None,
        });
        Ok(())
    }

    /// **Inspector の RigidBody Component を編集可能にする(群2)**。設計
    /// docs/23-frontend/01-editor.md §1.3 は「各 Component は World API の `Desc` 型と
    /// 1:1 対応。編集は次ステップ先頭で Command として適用される」と定めているが、
    /// これまで Inspector は**全フィールドが読み取り専用**で、質量も Body type も
    /// 衝突フィルタも触れなかった(そもそも衝突フィルタは `RigidBodySet` に
    /// 概念自体が無かった——群2で追加)。
    ///
    /// いずれも`Command`経由なので、**Playモード中でも決定論とリプレイ再現性を
    /// 保ったまま**編集できる(Gizmoドラッグのような直接書き換えとは違う)。
    fn body_mass_at_impl(&self, index: usize) -> Result<f64, WasmError> {
        let id = self.try_body_id_at(index)?;
        Ok(self.inner.mechanics().bodies.mass(id.index as usize))
    }

    /// Body type を表す文字列(`"Dynamic"`/`"Static"`/`"Kinematic"`)。
    fn body_type_at_impl(&self, index: usize) -> Result<String, WasmError> {
        let id = self.try_body_id_at(index)?;
        Ok(
            match self.inner.mechanics().bodies.body_type[id.index as usize] {
                BodyType::Dynamic => "Dynamic",
                BodyType::Static => "Static",
                BodyType::Kinematic => "Kinematic",
            }
            .to_string(),
        )
    }

    fn body_collision_group_at_impl(&self, index: usize) -> Result<u32, WasmError> {
        let id = self.try_body_id_at(index)?;
        Ok(self.inner.mechanics().bodies.collision_group[id.index as usize])
    }

    fn body_collision_mask_at_impl(&self, index: usize) -> Result<u32, WasmError> {
        let id = self.try_body_id_at(index)?;
        Ok(self.inner.mechanics().bodies.collision_mask[id.index as usize])
    }

    fn push_set_body_mass_impl(&mut self, body_index: usize, mass: f64) -> Result<(), WasmError> {
        if mass <= 0.0 || !mass.is_finite() {
            return Err(WasmError::InvalidMass);
        }
        let body = self.try_body_id_at(body_index)?;
        self.inner.push_command(Command::SetBodyMass { body, mass });
        Ok(())
    }

    /// 指定したボディの**高さと速さを記録し始める**(観測点を2本足す)。
    ///
    /// **なぜ要ったか**: 観測点はシーンJSONが宣言したものしか無く、エディタで
    /// 自分が置いた物には一本も付かなかった。そのため自分で組み立てた場面では
    /// グラフが永久に「まだデータがありません」のままで、CSVボタンも押せない
    /// ——用意された実験では動くだけに、壊れているとしか読めなかった
    /// (利用者役の観察)。
    ///
    /// 追加した観測点は`imported_probe_handles`へ積む。シーンJSONが宣言した
    /// ものと同じ扱いになり、既存の読み出し(`imported_probe_*`)がそのまま
    /// 使えるためである。戻り値は最初のハンドル(高さの方)。
    fn add_body_probes_impl(&mut self, index: usize) -> Result<usize, WasmError> {
        let id = self.try_body_id_at(index)?;
        let y = self.inner.add_probe(sim_world::ProbeTarget::BodyPosY(id));
        let speed = self.inner.add_probe(sim_world::ProbeTarget::BodySpeed(id));
        self.imported_probe_handles.push(y);
        self.imported_probe_handles.push(speed);
        Ok(y)
    }

    /// 質量を**その場で**変える(`set_body_position_at`等と同じ「Edit中の直接
    /// 設定」の一員)。
    ///
    /// **なぜ要ったか**: 質量の変更は`Command`(`push_set_body_mass`)しか
    /// 経路が無く、Commandは**次stepの先頭**で適用される。Editモードはstepが
    /// 進まないので、Inspectorに質量を打ち込んでも永久に何も起きなかった
    /// ——「10と入れたのに元の重さのまま落ちてくる」という、いちばん信用を
    /// 失う壊れ方をしていた(利用者役の観察)。
    ///
    /// 適用する処理は`Command::SetBodyMass`の腕と**同一**
    /// (`RigidBodySet::set_mass`)なので、Editで打つのとPlay中にCommandで
    /// 送るのとで結果は変わらない。決定論とリプレイ再現性の観点でも、
    /// 「Play中の介入はCommand」という取り決めは崩していない——これはPlayに
    /// 入る前の初期条件づくりであり、`set_body_position_at`が既にそうである
    /// のと同じ位置付けである。
    fn set_body_mass_at_impl(&mut self, index: usize, mass: f64) -> Result<(), WasmError> {
        if mass <= 0.0 || !mass.is_finite() {
            return Err(WasmError::InvalidMass);
        }
        let id = self.try_body_id_at(index)?;
        self.inner
            .mechanics_mut()
            .bodies
            .set_mass(id.index as usize, mass);
        Ok(())
    }

    /// Body type を切り替える。**Dynamic へ戻すときの質量をこちら側で確保する**
    /// のが要点——`Static` 化すると `inv_mass = 0`(無限質量)になり元の質量は
    /// 復元できないため、切替前の値を読んで `Command` に載せる。
    /// 既に非 Dynamic で質量が 0 になっているボディを Dynamic へ戻す場合は、
    /// 形状と材質密度から `create_body` と同じ式で計算し直す。
    fn push_set_body_type_impl(
        &mut self,
        body_index: usize,
        kind: String,
    ) -> Result<(), WasmError> {
        let body = self.try_body_id_at(body_index)?;
        let body_type = match kind.as_str() {
            "Dynamic" => BodyType::Dynamic,
            "Static" => BodyType::Static,
            "Kinematic" => BodyType::Kinematic,
            other => return Err(WasmError::UnknownBodyType(other.to_string())),
        };
        let idx = body.index as usize;
        let bodies = &self.inner.mechanics().bodies;
        let mut mass = bodies.mass(idx);
        if mass <= 0.0 {
            let material = self.inner.materials().get(bodies.material[idx]);
            mass = bodies.shape_of(idx).volume().unwrap_or(0.0) * material.density;
        }
        self.inner.push_command(Command::SetBodyType {
            body,
            body_type,
            mass,
        });
        Ok(())
    }

    fn push_set_collision_filter_impl(
        &mut self,
        body_index: usize,
        group: u32,
        mask: u32,
    ) -> Result<(), WasmError> {
        let body = self.try_body_id_at(body_index)?;
        self.inner
            .push_command(Command::SetCollisionFilter { body, group, mask });
        Ok(())
    }

    /// Scene ViewでのドラッグでD&D的に`index`番目のボディをつかむ(設計§1.2
    /// 「Gizmo」に相当する最小デモ、`Command::Grab`——重心(`anchor_local=
    /// Vec3::ZERO`)をワールド座標`target`へ剛にピン留めする)。`push_apply_force`
    /// と同じ理由で`body_index`引数を追加した。
    fn push_grab_impl(
        &mut self,
        body_index: usize,
        target_x: f64,
        target_y: f64,
        target_z: f64,
    ) -> Result<(), WasmError> {
        let body = self.try_body_id_at(body_index)?;
        self.inner.push_command(Command::Grab {
            body,
            anchor_local: sim_math::Vec3::ZERO,
            target: sim_math::Vec3::new(target_x, target_y, target_z),
        });
        Ok(())
    }

    /// ドラッグ中の`Command::MoveGrab`(既存のgrabの目標点をマウス位置へ追従させる)。
    /// `push_grab`と同じ`body_index`引数。
    fn push_move_grab_impl(
        &mut self,
        body_index: usize,
        target_x: f64,
        target_y: f64,
        target_z: f64,
    ) -> Result<(), WasmError> {
        let body = self.try_body_id_at(body_index)?;
        self.inner.push_command(Command::MoveGrab {
            body,
            target: sim_math::Vec3::new(target_x, target_y, target_z),
        });
        Ok(())
    }

    /// ドラッグ終了時の`Command::Release`(grabを解除、以後は通常の物理に戻る)。
    /// `push_grab`と同じ`body_index`引数。
    fn push_release_impl(&mut self, body_index: usize) -> Result<(), WasmError> {
        let body = self.try_body_id_at(body_index)?;
        self.inner.push_command(Command::Release { body });
        Ok(())
    }

    /// Editモードの直接編集(設計docs/23-frontend/01-editor.md §4「Editモード:
    /// シーンの直接編集が可能…Scene View gizmo ドラッグ」)。Playモードのドラッグ
    /// (`push_grab`/`push_move_grab`、Command経由・シミュレーション実行中でも
    /// 決定的に記録される)とは異なり、Commandキューを経由せず`RigidBodySet`の
    /// 位置を直接書き換える——設計が「実行中の直接編集は不可」とする境界どおり、
    /// この操作はシミュレーションが進行していない(Editモード中は呼び出し側が
    /// `step()`を呼ばない)ことを前提とする。`World`自体はBodyId経由の位置setterを
    /// 公開していないため(`mechanics_mut().bodies.position`はP1設計が定める
    /// `RigidBodySet`のSoAレイアウト、`docs/10-mechanics/01-rigid-body.md` §4)、
    /// ここで直接アクセスする。
    fn set_body_position_at_impl(
        &mut self,
        index: usize,
        x: f64,
        y: f64,
        z: f64,
    ) -> Result<(), WasmError> {
        let id = self.try_body_id_at(index)?;
        // Gizmo が掴んでいるのは**形状**なので、形状のローカル原点が
        // 指定座標へ来るように置く(群11、`RigidBodySet`型doc参照)。
        self.inner
            .mechanics_mut()
            .bodies
            .set_origin_position(id.index as usize, sim_math::Vec3::new(x, y, z));
        Ok(())
    }

    /// Scene View オーバーレイ(設計docs/23-frontend/01-editor.md §1.2「接触点」)向けに、
    /// 直近stepの接触点ワールド座標を`[x0,y0,z0,x1,y1,z1,...]`のフラット配列で返す
    /// (既存の`World::contact_points`をそのまま使う)。
    ///
    /// **B16(ゼロコピー化)**: `quantum_1d_density_f32`と同じ規約
    /// (`self.view_buffers.contact_points`をエイリアスする一時的なビュー)。
    pub fn contact_points_f32(&mut self) -> Float32Array {
        let points = self.inner.contact_points();
        let buf = &mut self.view_buffers.contact_points;
        buf.clear();
        buf.extend(
            points
                .iter()
                .flat_map(|p| [p.x as f32, p.y as f32, p.z as f32]),
        );
        // SAFETY: `quantum_1d_density_f32`と同じ(`HotPathViewBuffers`のdoc参照)。
        unsafe { Float32Array::view(buf) }
    }

    /// Consoleパネル(設計docs/23-frontend/01-editor.md §1.5「`SolverDiagnostics`の
    /// 発散警告・…イベントをフィルタ表示」)向けに、前回呼び出し以降に発生した
    /// イベント(既存の`World::drain_events`)を1行1件のテキストへ整形して返す
    /// (空行区切り、`level::message`形式——フロントエンドのフィルタタブが
    /// levelで分けるための単純な区切り、JSONは使わない縮約実装)。この2体デモでは
    /// 箱が床に着地/跳ね返るたびに`ContactStarted`/`ContactEnded`が実際に発生する。
    pub fn drain_events_text(&mut self) -> String {
        self.inner
            .drain_events()
            .iter()
            .map(|e| {
                let level = match e.kind {
                    sim_core::EventKind::FuseBlown
                    | sim_core::EventKind::SolverDiverged
                    | sim_core::EventKind::JointBroken => "warnings",
                    _ => "info",
                };
                // **接触イベントの`source`はボディ対を符号化している**
                // (`sim_mechanics::MechanicsSolver::emit_contact_events`の
                // `source_id = |a, b| SourceId((a << 32) | b)`)。生の`u64`
                // (例: ボディ(1,1)なら4294967297)は人間には読めないうえ、
                // フロントエンドが復号すると符号化の知識が2箇所に分かれる。
                // ここで復号して`bodies=a,b`として出し、**Consoleがイベント行から
                // 発生源ボディを選択できる**ようにする(設計docs/23-frontend/
                // 01-editor.md §1.5「クリックでTimeline/Scene Viewと連動」の
                // オブジェクト側、増分E4)。
                //
                // **`SourceId`の意味は生産者ごとに異なる**(例:
                // `sim_thermal`の`SolverDiverged`は`SourceId(0)`固定)ため、
                // 復号は接触イベントに限定する。他の種別は生値のまま出す。
                let detail = match e.kind {
                    sim_core::EventKind::ContactStarted | sim_core::EventKind::ContactEnded => {
                        let a = (e.source.0 >> 32) as usize;
                        let b = (e.source.0 & 0xFFFF_FFFF) as usize;
                        format!("bodies={a},{b}")
                    }
                    _ => format!("source={}", e.source.0),
                };
                format!("{level}::step={} {:?} ({detail})", e.step, e.kind)
            })
            .collect::<Vec<_>>()
            .join("\n")
    }
}

/// 検証パネル(**残タスク完遂の縦串④増分**、`sim_world::run_headless_scenario`の
/// 薄い写像)——シーンJSONを`steps`回実行し、結果をJSON文字列で返す
/// (`WasmWorld`のメソッドではなく自由関数——`run_headless_scenario`自身が
/// 呼び出しのたびに独立した新しい`World`を構築するため、既存の`self.inner`
/// とは無関係)。
///
/// `final_state_hash`は`u64`のままJSONへ出すとJSの`Number`精度(2^53)を
/// 超えて壊れる(`WasmWorld::state_hash`が16進文字列を返すのと同じ理由)ため、
/// ここでも16進文字列に変換して返す。
///
/// wasm-bindgenへ露出する薄い殻——実体は`run_headless_scenario_json_impl`側
/// (`WasmError`のdoc参照)。
#[wasm_bindgen]
pub fn run_headless_scenario_json(json: &str, steps: u32) -> Result<String, JsValue> {
    run_headless_scenario_json_impl(json, steps).map_err(JsValue::from)
}

/// `run_headless_scenario_json`の実体(ネイティブテスト可能な
/// `Result<_, WasmError>`版)。
fn run_headless_scenario_json_impl(json: &str, steps: u32) -> Result<String, WasmError> {
    #[derive(serde::Serialize)]
    struct HeadlessRunResultJson {
        final_state_hash: String,
        final_time: f64,
        probe_histories: Vec<Vec<f64>>,
    }
    let result = sim_world::run_headless_scenario(json, steps).map_err(WasmError::HeadlessRun)?;
    let json_result = HeadlessRunResultJson {
        final_state_hash: format!("{:016x}", result.final_state_hash),
        final_time: result.final_time,
        probe_histories: result.probe_histories,
    };
    serde_json::to_string(&json_result)
        .map_err(|e| WasmError::HeadlessResultSerializeFailed(e.to_string()))
}

// ---------------------------------------------------------------------------
// スケッチ・押し出し(D1)
// ---------------------------------------------------------------------------

/// エディタのスケッチ1枚(構築平面上の点列 + 直前までの結果との合成方法)。
///
/// `points`は構築平面上の`[x, z]`(**構築平面は地面 y=0**——既存の
/// スポーンパレットのクリック位置決めと同じ平面、`main.ts`側のdoc参照)。
/// `op`は`"union"`/`"subtract"`/`"intersect"`で、**リストの最初の1枚では
/// 無視される**(合成相手がまだ無いため)。
#[derive(serde::Deserialize)]
struct SketchProfileJson {
    #[serde(default)]
    op: String,
    points: Vec<[f64; 2]>,
}

/// `sketch_extrude_shape_json`の入力。
#[derive(serde::Deserialize)]
struct SketchExtrudeRequestJson {
    profiles: Vec<SketchProfileJson>,
    depth: f64,
}

/// `sketch_extrude_shape_json`の出力。
///
/// `shape`はそのまま`apply_component("spawn_shape_json", …)`の`shape_json`へ
/// 渡せる`ShapeJson`。通常は新タグ`mesh`1つだが、**断面に穴があるときは
/// `mesh`を子に持つ`compound`**になる(`sim_mechanics::extrude_region`が
/// 穴を通る線で切り分けたパーツをそのまま子にする、下記`_impl`のコメント参照)。
/// `origin`/`rest_height`は「描いたとおりの
/// 位置に、地面へちょうど載る高さで置く」ための配置情報で、
/// スポーン位置は`(origin[0], rest_height, origin[1])`になる
/// (`sim_mechanics::extrude_region`が重心を原点へ寄せる規約と対になっている)。
#[derive(serde::Serialize)]
struct SketchExtrudeResultJson {
    shape: sim_world::ShapeJson,
    origin: [f64; 2],
    rest_height: f64,
    /// 押し出し前の断面積[m²](UIの確認表示用)。
    profile_area: f64,
    /// 出来上がった角柱の体積[m³](= `profile_area * depth`)。
    volume: f64,
}

/// **スケッチ→ブーリアン合成→押し出し**を1回のwasm呼び出しで行い、
/// 出来たメッシュを`ShapeJson`(タグ`mesh`、穴があれば`mesh`の`compound`)
/// として返す(D1)。
///
/// ## なぜRust側に置いたか
///
/// 幾何の数値的に厄介な部分(多角形ブーリアンの交点計算・内外判定・
/// 縫い合わせ、耳刈り、穴のブリッジ)を**1箇所にまとめ、ネイティブの
/// `cargo test`で解析的に固定できる**ため(`sim_mechanics::sketch`の
/// テスト群)。TypeScript側に置くと同じ幾何をPlaywright越しにしか
/// 検証できず、面積・体積の数値が合っているかを解析解と突き合わせる
/// 回帰テストが書けない。
///
/// **wasm往復のコストは問題にならない**——この呼び出しは「押し出し」
/// ボタン1回につき1回だけで、スケッチ編集中(点を置くたび)には走らない。
/// 描画プレビューはTypeScript側が点列をそのまま線で結ぶだけで済む。
///
/// wasm-bindgenへ露出する薄い殻——実体は`sketch_extrude_shape_json_impl`側。
#[wasm_bindgen]
pub fn sketch_extrude_shape_json(request_json: &str) -> Result<String, JsValue> {
    sketch_extrude_shape_json_impl(request_json).map_err(JsValue::from)
}

/// `sketch_extrude_shape_json`の実体(ネイティブテスト可能な
/// `Result<_, WasmError>`版)。
fn sketch_extrude_shape_json_impl(request_json: &str) -> Result<String, WasmError> {
    let request: SketchExtrudeRequestJson = serde_json::from_str(request_json)
        .map_err(|e| WasmError::SketchRequestParseFailed(e.to_string()))?;

    // 各スケッチを妥当なCCWループへ正規化し、順に合成する。最初の1枚が
    // 土台で、2枚目以降が自分の`op`で土台へ効く(CADの作図順そのまま)。
    let mut region: Vec<sim_mechanics::Loop2> = Vec::new();
    let mut combined_any = false;
    for profile in &request.profiles {
        let Some(outline) = sim_mechanics::normalize_loop(&profile.points) else {
            // 面積を持たないスケッチ(点が2つ以下・全点が共線)は黙って捨てる
            // ——描きかけの点列がUI側で紛れ込んでも操作が止まらないようにする。
            continue;
        };
        if !combined_any {
            region = vec![outline];
            combined_any = true;
            continue;
        }
        let op = match profile.op.as_str() {
            "subtract" => sim_mechanics::BooleanOp::Subtract,
            "intersect" => sim_mechanics::BooleanOp::Intersect,
            "union" | "" => sim_mechanics::BooleanOp::Union,
            other => return Err(WasmError::UnknownBooleanOp(other.to_string())),
        };
        region = sim_mechanics::polygon_boolean(&region, &[outline], op);
        if region.is_empty() {
            // 「重なっていない図形の積」のように、合成結果が空になることは
            // 正当に起きる。押し出す面が無いことをそのままユーザーへ返す。
            return Err(WasmError::SketchProfileEmpty);
        }
    }
    if !combined_any {
        return Err(WasmError::SketchProfileEmpty);
    }

    let centroid = sim_mechanics::region_centroid(&region).ok_or(WasmError::SketchProfileEmpty)?;
    let parts = sim_mechanics::extrude_region(&region, request.depth)
        .ok_or(WasmError::SketchExtrudeFailed)?;

    let to_mesh_json = |part: &sim_mechanics::ExtrudedMesh| sim_world::ShapeJson::Mesh {
        vertices: part.vertices.iter().map(|v| [v.x, v.y, v.z]).collect(),
        triangles: part.triangles.clone(),
    };
    // **穴の無い断面(大多数)は`mesh`1つ**。穴があると`extrude_region`が
    // 穴を通る線で切り分けたパーツを返すので、それらを`compound`の子として
    // まとめる(`sim_mechanics::sketch::split_into_hole_free_regions`のdoc参照
    // ——1つのメッシュに繋ぐと近似凸分解から見た配置が元と同じになり、
    // 穴が塞がってしまう)。子はすべて同じローカル原点なので変換は恒等。
    let shape = if parts.len() == 1 {
        to_mesh_json(&parts[0])
    } else {
        sim_world::ShapeJson::Compound {
            children: parts
                .iter()
                .map(|part| sim_world::CompoundChildJson {
                    position: [0.0, 0.0, 0.0],
                    rotation: [0.0, 0.0, 0.0, 1.0],
                    shape: Box::new(to_mesh_json(part)),
                })
                .collect(),
        }
    };

    let profile_area = sim_mechanics::region_area(&region);
    let result = SketchExtrudeResultJson {
        shape,
        origin: centroid,
        rest_height: request.depth * 0.5,
        profile_area,
        volume: profile_area * request.depth,
    };
    serde_json::to_string(&result).map_err(|e| WasmError::ShapeSerializeFailed(e.to_string()))
}

/// **2026-07-27の監査で追加**: このcrate(JS/WASM境界、1280行)にテストが
/// 1本も無かった(Rustワークスペース最大の未テスト面)ため、Q5(wasm境界の
/// パニック除去)の作業とあわせて最小限のユニットテストを追加する。
///
/// **かつての制約と、それを`WasmError`導入で解いた経緯**:
/// `js_sys::Float32Array`/`Float64Array`と`wasm_bindgen::JsValue`はいずれも、
/// 実際のwasmホスト(ブラウザ/Node)無しでは**値を構築すること自体ができない**。
/// 実験の結果、`Float32Array::new_with_length`はネイティブターゲットで
/// 「cannot call wasm-bindgen imported functions on non-wasm targets」と
/// (unwind可能な)パニックを起こす一方、`JsValue::from_str`は**unwindしない
/// プロセスabort(SIGABRT)** を起こすことを確認していた——各`*_impl`が
/// `Result<T, JsValue>`を返していた頃は、その`Err`分岐を
/// `assert!(result.is_err())`のような形で検証しようとするテストが
/// `Err`を構築した時点で**テストプロセスごと**abortしていた(該当テストの
/// `#[should_panic]`でも`catch_unwind`でも捕捉できない)。そのため本モジュールは
/// 長らく**成功パスのみ**に限定され、随所に「Errパスの検証には
/// wasm-bindgen-testが要るため対象外」というコメントが残っていた。
///
/// **`WasmError`(モジュール冒頭)の導入でこの制約は解けた**: `*_impl`が
/// 返すのは素のRust enumになり、`JsValue`の構築はwasm-bindgenがexportする
/// `pub fn`——ネイティブテストからは呼ばない最外周——1点だけに寄せた。
/// エラーパスは`*_impl`を直接呼び、
/// `assert!(matches!(err, WasmError::BodyIndexOutOfRange { .. }))`のように
/// **種別で**検証する(メッセージ文字列を両側で組み立てて突き合わせても、
/// 同じ`format!`式が同じ文字列を作ることしか示さない)。
///
/// **今も残る制約**: `Float32Array`/`Float64Array`そのものはネイティブで
/// 構築できないままなので、それらを返す`pub fn`(`body_position_at_f32`等)の
/// 戻り値の中身は検証しない——ただしindex検証を担う`*_impl`
/// (`body_position_at_impl`等)は素のRust配列を返すため、成功値もエラーも
/// ここで検証できる。
#[cfg(test)]
mod tests {
    use super::*;

    fn new_world() -> WasmWorld {
        WasmWorld::new(-9.80665, 1.0 / 60.0, 5.0)
    }

    /// エラーパス検証の共通ヘルパ——`Result`が`Err`であることと、その
    /// **中身が期待した`WasmError`変種そのものであること**を確かめる。
    /// 変種まで見るのは、たとえば「範囲外index」を期待したテストが実際には
    /// 別の理由(材質名の誤りなど)で失敗していても素通りしてしまうのを
    /// 防ぐため。`WasmError`は`PartialEq`なので値まで含めて一致を見る。
    #[track_caller]
    fn assert_err_is<T: std::fmt::Debug>(result: Result<T, WasmError>, expected: WasmError) {
        match result {
            Ok(v) => panic!("expected Err({expected:?}), got Ok({v:?})"),
            Err(e) => assert_eq!(e, expected, "wrong WasmError variant"),
        }
    }

    /// `assert_err_is`の緩い版——`count`のように将来増減しうる付随情報まで
    /// 固定したくない場合に、変種だけをパターンで検証する。
    ///
    /// `Ok`側の値は表示しない(`T: Debug`を要求しない)——`WasmWorld`自身が
    /// `Debug`を実装しておらず、`from_scene_json_impl`の`Err`検証にも使いたい
    /// ため。失敗時に知りたいのは「どの変種が来たか」であって成功値ではない。
    macro_rules! assert_err_matches {
        ($result:expr, $pattern:pat) => {
            match $result {
                Ok(_) => panic!("expected Err({}), got Ok(..)", stringify!($pattern)),
                Err(e) => assert!(
                    matches!(e, $pattern),
                    "expected {}, got {:?}",
                    stringify!($pattern),
                    e
                ),
            }
        };
    }

    /// 検証パネル(**残タスク完遂の縦串④増分**)——`run_headless_scenario_json`が
    /// D1(自由落下)を60step実行し、`final_state_hash`(16進文字列)・`final_time`・
    /// probe履歴(自由落下なので単調に下がるはず)を含むJSONを返すこと。
    #[test]
    fn run_headless_scenario_json_reports_free_fall_probe_history() {
        let json = include_str!("../../../scenes/d1-free-fall.json");
        let result_json =
            run_headless_scenario_json_impl(json, 60).expect("D1 must run headlessly");
        let parsed: serde_json::Value =
            serde_json::from_str(&result_json).expect("result must be valid JSON");
        let hash = parsed["final_state_hash"]
            .as_str()
            .expect("final_state_hash must be a hex string");
        assert_eq!(hash.len(), 16, "expected 16 hex digits, got {hash}");
        assert!(parsed["final_time"].as_f64().unwrap() > 0.0);
        let history = parsed["probe_histories"][0]
            .as_array()
            .expect("D1 has exactly one probe (body_pos_y)");
        assert_eq!(history.len(), 60);
        let first = history[0].as_f64().unwrap();
        let last = history[59].as_f64().unwrap();
        assert!(
            last < first,
            "a body in free fall must have a lower y at the end than at the start"
        );
    }

    /// 固定2体(床・箱)のラベル・材質・静的判定が期待どおりであること。
    #[test]
    fn fixed_bodies_have_expected_labels_and_materials() {
        let world = new_world();
        assert_eq!(world.body_count_impl(), 2);
        assert_eq!(world.body_label_at_impl(0).unwrap(), "Ground");
        assert_eq!(world.body_label_at_impl(1).unwrap(), "Box_1");
        assert_eq!(
            world.body_material_label_at_impl(0).unwrap(),
            "コンクリート"
        );
        assert_eq!(world.body_material_label_at_impl(1).unwrap(), "鋼(炭素鋼)");
        assert!(world.body_is_static_at_impl(0).unwrap());
        assert!(!world.body_is_static_at_impl(1).unwrap());
    }

    /// 自由配線回路エディタへ追加したコンデンサ・インダクタ・ダイオード・
    /// DCモーターの4種(回路素子4種をUIエディタに追加、モジュールdoc参照)が
    /// パニックせず追加でき、`num_nodes`・電流読み出しが期待どおり動くこと。
    #[test]
    fn circuit_editor_add_capacitor_inductor_diode_and_dc_motor_succeed() {
        let mut world = new_world();
        world.circuit_editor_reset_impl(3);
        // ノード1-2間にコンデンサ(初期電圧5V)。
        world.circuit_editor_add_capacitor_impl(1, 2, 1e-3, 5.0);
        // ノード2-0間にインダクタ。
        world.circuit_editor_add_inductor_impl(2, 0, 1e-3, 0.0);
        // ノード1-0間にダイオード。
        world.circuit_editor_add_diode_impl(1, 0, 1e-12, 0.026);

        // DCモーターは内部ノードを2つ自動確保するので、num_nodesが3→5に伸びる。
        let motor_index = world.circuit_editor_add_dc_motor_impl(1, 0, 2.0, 1e-3, 0.1);
        assert_eq!(motor_index, 0);
        world.circuit_editor_set_motor_speed_impl(motor_index, 10.0);
        // 速度設定直後、逆起電力による電流は有限値のはず(NaN/panicしない)。
        let current = world.circuit_editor_motor_current_impl(motor_index);
        assert!(current.is_finite());

        // 未知indexへの操作は無害に無視される(パニックしない)。
        world.circuit_editor_set_motor_speed_impl(999, 1.0);
        assert_eq!(world.circuit_editor_motor_current_impl(999), 0.0);
    }

    /// Inspectorの Add Component(**残タスク完遂の縦串①増分**)——5種の
    /// `add_*_joint`がスポーンしたボディを相手にパニックせず成功し、
    /// `joint_info_text`(Inspectorの読み取り側)にそれぞれ現れること。
    /// Wheel は D24 車と同じ「駆動あり」パラメータで追加し、
    /// `JointKind::Wheel`が`World::joints()`に無かった既存の欠落
    /// (追加はできても内省層に出ず Inspector から見えなかった)も
    /// あわせて解消したことを確認する。
    #[test]
    fn add_joint_methods_succeed_and_are_visible_in_joint_info_text() {
        let mut world = new_world();
        let chassis = world
            .spawn_box_impl(0.0, 1.0, 0.0, 1.0, "鋼(炭素鋼)".to_string())
            .unwrap();
        let wheel = world
            .spawn_sphere_impl(1.0, 0.3, 1.0, 0.3, "ゴム(天然)".to_string())
            .unwrap();
        let anchor = world
            .spawn_sphere_impl(-2.0, 1.0, 0.0, 0.2, "鋼(炭素鋼)".to_string())
            .unwrap();

        world
            .add_distance_joint_impl(chassis, 0.0, 0.0, 0.0, anchor as i32, 0.0, 0.0, 0.0, 2.0)
            .expect("distance joint between two live bodies must succeed");
        world
            .add_ball_joint_impl(chassis, 0.0, 0.0, 0.0, -1, 0.0, 2.0, 0.0)
            .expect("ball joint to a world-fixed point must succeed");
        world
            .add_slider_joint_impl(chassis, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1, 0.0, 3.0, 0.0)
            .expect("slider joint must succeed");
        world
            .add_wheel_joint_impl(
                chassis, wheel, 1.0, 0.0, 1.0, 0.4, 2.5, 0.7, 0.0, 12.0, 200.0,
            )
            .expect("wheel joint must succeed");
        world
            .add_hinge_motor_joint_impl(anchor, 0.0, 1.0, 0.0, 1.0, 5.0, 0.5, 10.0)
            .expect("hinge motor joint must succeed");

        let text = world.joint_info_text_impl(-1);
        for kind in [
            "DistanceJoint",
            "BallJoint",
            "SliderJoint",
            "WheelJoint",
            "HingeMotorPd",
        ] {
            assert!(
                text.contains(kind),
                "joint_info_text must report a {kind} line, got:\n{text}"
            );
        }
        // **`WasmError`導入で検証できるようになったErrパス**(以前はここに
        // 「Errパスの検証にはwasm-bindgen-testが要るため対象外」と書いてあった)。
        // Joint 5種はいずれも`try_body_id_at`を通るので、範囲外indexで
        // `BodyIndexOutOfRange`になる。
        let out_of_range = world.body_count_impl();
        assert_err_is(
            world.add_distance_joint_impl(out_of_range, 0.0, 0.0, 0.0, -1, 0.0, 0.0, 0.0, 2.0),
            WasmError::BodyIndexOutOfRange {
                index: out_of_range,
                count: out_of_range,
            },
        );
        assert_err_matches!(
            world.add_ball_joint_impl(out_of_range, 0.0, 0.0, 0.0, -1, 0.0, 2.0, 0.0),
            WasmError::BodyIndexOutOfRange { .. }
        );
        assert_err_matches!(
            world.add_slider_joint_impl(
                out_of_range,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                -1,
                0.0,
                3.0,
                0.0
            ),
            WasmError::BodyIndexOutOfRange { .. }
        );
        assert_err_matches!(
            world.add_wheel_joint_impl(
                out_of_range,
                wheel,
                1.0,
                0.0,
                1.0,
                0.4,
                2.5,
                0.7,
                0.0,
                12.0,
                200.0
            ),
            WasmError::BodyIndexOutOfRange { .. }
        );
        assert_err_matches!(
            world.add_hinge_motor_joint_impl(out_of_range, 0.0, 1.0, 0.0, 1.0, 5.0, 0.5, 10.0),
            WasmError::BodyIndexOutOfRange { .. }
        );
        // `add_wheel_joint`は2つ目のボディindexも`try_body_id_at`を通る
        // (片側だけ検証して満足しないための確認)。
        assert_err_matches!(
            world.add_wheel_joint_impl(
                chassis,
                out_of_range,
                1.0,
                0.0,
                1.0,
                0.4,
                2.5,
                0.7,
                0.0,
                12.0,
                200.0
            ),
            WasmError::BodyIndexOutOfRange { .. }
        );
    }

    /// Settingsの環境パネル(**残タスク完遂の縦串③増分**)——大気・水域の
    /// 設定/解除が期待どおり反映され、未設定なら`NaN`を返すこと。
    #[test]
    fn set_and_clear_atmosphere_and_water_region_round_trip() {
        let mut world = new_world();
        assert!(world.atmosphere_density_impl().is_nan());
        assert!(world.water_level_impl().is_nan());

        world.set_atmosphere_impl(1.225, 1.5e-5, 2.0, 0.0, -1.0);
        assert_eq!(world.atmosphere_density_impl(), 1.225);
        assert_eq!(world.atmosphere_viscosity_impl(), 1.5e-5);
        // `atmosphere_wind`は`Float64Array`を返すため、このテストモジュール
        // 冒頭のdoc comment(`Float32Array`/`Float64Array`はネイティブでは
        // 構築できない)どおり、ここでは呼ばない。

        world.set_water_region_impl(0.0, 1000.0);
        assert_eq!(world.water_level_impl(), 0.0);
        assert_eq!(world.water_density_impl(), 1000.0);

        world.clear_atmosphere_impl();
        assert!(world.atmosphere_density_impl().is_nan());
        // 大気を消しても水域は無事(`set_environment`が両方を毎回まとめて
        // 書き直すため、片方だけ変えるつもりが他方を巻き込まないことの確認)。
        assert_eq!(world.water_level_impl(), 0.0);

        world.clear_water_region_impl();
        assert!(world.water_level_impl().is_nan());
    }

    /// **Task#8第二弾の回帰テスト**: 環境系(重力・dt・大気・水域)15個も
    /// `apply_component`/`read_component`経由で操作できることを確認する。
    /// `gravity_direction`/`atmosphere_wind`は`Float64Array`を返すため
    /// (このテストモジュール冒頭のdoc comment参照)、ここでは呼ばない
    /// ——スカラーを返すkindのみで往復を確認する。
    #[test]
    fn apply_component_and_read_component_change_environment_via_generic_dispatch() {
        let mut world = new_world();

        world
            .apply_component_impl("set_gravity", r#"{"gravity":1.62}"#)
            .expect("set_gravity via apply_component must succeed");
        assert_eq!(
            world.read_component_impl("gravity", "").unwrap(),
            1.62_f64.to_string()
        );

        world
            .apply_component_impl("set_dt", r#"{"dt":0.02}"#)
            .expect("set_dt via apply_component must succeed");
        assert_eq!(
            world.read_component_impl("dt", "").unwrap(),
            0.02_f64.to_string()
        );
        // **`WasmError`導入で検証できるようになったErrパス**——`set_dt`は
        // 「正の有限値」だけを受ける。0・負・非有限のいずれも`InvalidDt`。
        for bad in ["0.0", "-0.01"] {
            assert_err_is(
                world.apply_component_impl("set_dt", &format!(r#"{{"dt":{bad}}}"#)),
                WasmError::InvalidDt,
            );
        }
        // 非有限値はJSONに書けない(serde_jsonが`Infinity`/`NaN`を受け付けない)
        // ため、`set_dt_impl`を直接叩いて`is_finite`側のガードも通す。
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_err_is(world.set_dt_impl(bad), WasmError::InvalidDt);
        }
        // 弾かれた後もdtは直前の正常値のまま(Errが状態を壊していない)。
        assert_eq!(
            world.read_component_impl("dt", "").unwrap(),
            0.02_f64.to_string()
        );
        // ディスパッチ自体の未知kindも同様に検証できる。
        assert_err_is(
            world.apply_component_impl("no_such_kind", "{}"),
            WasmError::UnknownApplyComponentKind("no_such_kind".to_string()),
        );
        assert_err_is(
            world.read_component_impl("no_such_kind", ""),
            WasmError::UnknownReadComponentKind("no_such_kind".to_string()),
        );
        // payloadがJSONとして壊れている場合(kindの解決より手前で弾かれる)。
        assert_err_matches!(
            world.apply_component_impl("set_dt", "{not json"),
            WasmError::ApplyComponentInvalidJson(_)
        );

        world
            .apply_component_impl(
                "set_atmosphere",
                r#"{"density":1.225,"viscosity":1.5e-5,"wind_x":2.0,"wind_y":0.0,"wind_z":-1.0}"#,
            )
            .expect("set_atmosphere via apply_component must succeed");
        assert_eq!(
            world.read_component_impl("atmosphere_density", "").unwrap(),
            "1.225"
        );

        world
            .apply_component_impl(
                "set_water_region",
                r#"{"water_level":0.0,"density":1000.0}"#,
            )
            .expect("set_water_region via apply_component must succeed");
        assert_eq!(world.read_component_impl("water_level", "").unwrap(), "0");
        assert_eq!(
            world.read_component_impl("water_density", "").unwrap(),
            "1000"
        );

        world
            .apply_component_impl("clear_atmosphere", "{}")
            .expect("clear_atmosphere via apply_component must succeed");
        assert!(world
            .read_component_impl("atmosphere_density", "")
            .unwrap()
            .parse::<f64>()
            .unwrap()
            .is_nan());

        world
            .apply_component_impl("clear_water_region", "{}")
            .expect("clear_water_region via apply_component must succeed");
        assert!(world
            .read_component_impl("water_level", "")
            .unwrap()
            .parse::<f64>()
            .unwrap()
            .is_nan());
    }

    /// **Task#8第三弾の回帰テスト**: ボディのGizmo直接編集・Command系(質量・
    /// body type・衝突フィルタ・grab)・その内省4個も`apply_component`/
    /// `read_component`経由で操作できることを確認する。
    #[test]
    fn apply_component_and_read_component_change_body_properties_via_generic_dispatch() {
        let mut world = new_world();
        let body = world
            .spawn_box_impl(0.0, 5.0, 0.0, 0.5, "アルミニウム".to_string())
            .unwrap();

        world
            .apply_component_impl(
                "set_body_position_at",
                &format!(r#"{{"index":{body},"x":1.0,"y":2.0,"z":3.0}}"#),
            )
            .expect("set_body_position_at via apply_component must succeed");
        world
            .apply_component_impl(
                "set_body_rotation_at",
                &format!(r#"{{"index":{body},"x":0.0,"y":0.0,"z":0.0,"w":1.0}}"#),
            )
            .expect("set_body_rotation_at via apply_component must succeed");
        world
            .apply_component_impl(
                "set_body_scale_at",
                &format!(r#"{{"index":{body},"scale":2.0}}"#),
            )
            .expect("set_body_scale_at via apply_component must succeed");

        // 自分で置いた物にも観測点を足せる(`add_body_probes_impl`のdoc参照)。
        let before: usize = world
            .read_component_impl("imported_probe_count", "")
            .unwrap()
            .parse()
            .unwrap();
        world
            .apply_component_impl("add_body_probes", &format!(r#"{{"index":{body}}}"#))
            .expect("add_body_probes via apply_component must succeed");
        let after: usize = world
            .read_component_impl("imported_probe_count", "")
            .unwrap()
            .parse()
            .unwrap();
        assert_eq!(after, before + 2, "高さと速さの2本が足される");
        assert!(
            world
                .read_component_impl("imported_probe_label_at", &before.to_string())
                .unwrap()
                .starts_with("BodyPosY"),
            "1本目は高さ"
        );
        assert!(
            world
                .apply_component_impl("add_body_probes", r#"{"index":9999}"#)
                .is_err(),
            "存在しないボディは弾く"
        );

        // 質量の直接設定は**stepを挟まずに**効く(Editモードでも打った値が
        // 反映される、`set_body_mass_at_impl`のdoc参照)。
        world
            .apply_component_impl(
                "set_body_mass_at",
                &format!(r#"{{"index":{body},"mass":7.0}}"#),
            )
            .expect("set_body_mass_at via apply_component must succeed");
        assert_eq!(
            world
                .read_component_impl("body_mass_at", &body.to_string())
                .unwrap(),
            7.0_f64.to_string(),
            "set_body_mass_at must apply immediately (no step)"
        );
        assert!(
            world
                .apply_component_impl(
                    "set_body_mass_at",
                    &format!(r#"{{"index":{body},"mass":0.0}}"#)
                )
                .is_err(),
            "set_body_mass_at must reject a non-positive mass"
        );

        let result = world
            .apply_component_impl(
                "push_set_body_mass",
                &format!(r#"{{"body_index":{body},"mass":5.0}}"#),
            )
            .expect("push_set_body_mass via apply_component must succeed");
        assert_eq!(result, "{}");
        world.step();
        assert_eq!(
            world
                .read_component_impl("body_mass_at", &body.to_string())
                .unwrap(),
            5.0_f64.to_string()
        );

        world
            .apply_component_impl(
                "push_set_collision_filter",
                &format!(r#"{{"body_index":{body},"group":2,"mask":4}}"#),
            )
            .expect("push_set_collision_filter via apply_component must succeed");
        world.step();
        assert_eq!(
            world
                .read_component_impl("body_collision_group_at", &body.to_string())
                .unwrap(),
            "2"
        );
        assert_eq!(
            world
                .read_component_impl("body_collision_mask_at", &body.to_string())
                .unwrap(),
            "4"
        );

        world
            .apply_component_impl(
                "push_set_body_type",
                &format!(r#"{{"body_index":{body},"kind":"Static"}}"#),
            )
            .expect("push_set_body_type via apply_component must succeed");
        world.step();
        assert_eq!(
            world
                .read_component_impl("body_type_at", &body.to_string())
                .unwrap(),
            "Static"
        );

        world
            .apply_component_impl(
                "push_grab",
                &format!(r#"{{"body_index":{body},"target_x":1.0,"target_y":1.0,"target_z":1.0}}"#),
            )
            .expect("push_grab via apply_component must succeed");
        world
            .apply_component_impl(
                "push_move_grab",
                &format!(r#"{{"body_index":{body},"target_x":2.0,"target_y":1.0,"target_z":1.0}}"#),
            )
            .expect("push_move_grab via apply_component must succeed");
        world
            .apply_component_impl("push_release", &format!(r#"{{"body_index":{body}}}"#))
            .expect("push_release via apply_component must succeed");
        world
            .apply_component_impl(
                "push_apply_force",
                &format!(r#"{{"body_index":{body},"fx":0.0,"fy":0.0,"fz":0.0}}"#),
            )
            .expect("push_apply_force via apply_component must succeed");
        world.step();
    }

    /// **Task#8第四弾の回帰テスト**: ボディのスポーン/削除/複製/材料派生
    /// 9個と、その内省9個も`apply_component`/`read_component`経由で操作できる
    /// ことを確認する。`material_properties_f64`は以前「`Float64Array`を返す
    /// 実装がネイティブでSIGABRTする」ため除外していたが、`_impl`を
    /// `Vec<f64>`返しへ直した(往復は元々無駄だった、
    /// `material_properties_f64_impl`のdoc参照)ので今は検証対象に含める。
    /// **形状パラメータの内省は`body_shape_json_at`1本**——同じ経緯で
    /// 検証対象に入っていた`body_shape_params_f64_at`は、Compound/ConvexMeshを
    /// 表現できず読み出しを2経路に割っていたため削除した
    /// (`body_shape_json_at_impl`のdoc参照)。
    #[test]
    fn apply_component_and_read_component_spawn_and_introspect_bodies_via_generic_dispatch() {
        let mut world = new_world();
        assert_eq!(world.read_component_impl("body_count", "").unwrap(), "2");

        let result = world
            .apply_component_impl(
                "spawn_sphere",
                r#"{"x":1.0,"y":2.0,"z":3.0,"radius":0.4,"material_name":"アルミニウム"}"#,
            )
            .expect("spawn_sphere via apply_component must succeed");
        assert_eq!(result, "{\"index\":2}");
        assert_eq!(world.read_component_impl("body_count", "").unwrap(), "3");
        assert_eq!(
            world
                .read_component_impl("body_shape_kind_at", "2")
                .unwrap(),
            "sphere"
        );
        assert_eq!(
            world.read_component_impl("body_label_at", "2").unwrap(),
            "Sphere_2"
        );
        assert_eq!(
            world
                .read_component_impl("body_material_label_at", "2")
                .unwrap(),
            "アルミニウム"
        );
        assert_eq!(
            world.read_component_impl("body_is_static_at", "2").unwrap(),
            "false"
        );
        assert_eq!(
            world
                .read_component_impl("body_is_removed_at", "2")
                .unwrap(),
            "false"
        );
        assert!(world
            .read_component_impl("body_shape_label_at", "2")
            .unwrap()
            .starts_with("Sphere"));
        let shape_json = world
            .read_component_impl("body_shape_json_at", "2")
            .unwrap();
        assert!(shape_json.contains("sphere"), "actual: {shape_json}");

        let result = world
            .apply_component_impl(
                "spawn_box",
                r#"{"x":0.0,"y":2.0,"z":0.0,"half_extent":0.5,"material_name":"アルミニウム"}"#,
            )
            .expect("spawn_box via apply_component must succeed");
        assert_eq!(result, "{\"index\":3}");

        let result = world
            .apply_component_impl(
                "spawn_capsule",
                r#"{"x":2.0,"y":2.0,"z":0.0,"radius":0.3,"half_height":0.5,"material_name":"アルミニウム"}"#,
            )
            .expect("spawn_capsule via apply_component must succeed");
        assert_eq!(result, "{\"index\":4}");

        let result = world
            .apply_component_impl(
                "spawn_compound_l_shape",
                r#"{"x":3.0,"y":5.0,"z":0.0,"material_name":"アルミニウム"}"#,
            )
            .expect("spawn_compound_l_shape via apply_component must succeed");
        assert_eq!(result, "{\"index\":5}");

        let result = world
            .apply_component_impl(
                "spawn_convex_mesh_cube",
                r#"{"x":4.0,"y":5.0,"z":0.0,"half":0.5,"material_name":"アルミニウム"}"#,
            )
            .expect("spawn_convex_mesh_cube via apply_component must succeed");
        assert_eq!(result, "{\"index\":6}");
        assert_eq!(world.read_component_impl("body_count", "").unwrap(), "7");

        let result = world
            .apply_component_impl("duplicate_body_at", r#"{"index":2,"offset":1.0}"#)
            .expect("duplicate_body_at via apply_component must succeed");
        assert_eq!(result, "{\"index\":7}");
        assert_eq!(world.read_component_impl("body_count", "").unwrap(), "8");

        world
            .apply_component_impl("remove_body_at", r#"{"index":7}"#)
            .expect("remove_body_at via apply_component must succeed");
        assert_eq!(
            world
                .read_component_impl("body_is_removed_at", "7")
                .unwrap(),
            "true"
        );

        world
            .apply_component_impl(
                "derive_material",
                r#"{"base_name":"アルミニウム","new_name":"軽量アルミニウム(Task8第四弾テスト)","density":1500.0}"#,
            )
            .expect("derive_material via apply_component must succeed");

        // 任意形状スポナー——固定レシピの5種と違い、形状そのものを
        // `body_shape_json_at`と同じ`ShapeJson`表現で受ける(9個目のスポーン)。
        let result = world
            .apply_component_impl(
                "spawn_shape_json",
                r#"{"shape_json":"{\"capsule\":{\"radius\":0.2,\"half_height\":0.6}}","x":6.0,"y":3.0,"z":0.0,"material_name":"アルミニウム"}"#,
            )
            .expect("spawn_shape_json via apply_component must succeed");
        assert_eq!(result, "{\"index\":8}");
        assert_eq!(
            world
                .read_component_impl("body_shape_kind_at", "8")
                .unwrap(),
            "capsule"
        );
        assert_eq!(
            world.read_component_impl("body_label_at", "8").unwrap(),
            "Capsule_8"
        );

        // `Vec<f64>`返しへ直した内省(上のdoc参照)。
        let props = world
            .read_component_impl("material_properties_f64", "アルミニウム")
            .unwrap();
        assert!(
            props.starts_with('[') && props.ends_with(']'),
            "actual: {props}"
        );

        // === ここから下は`WasmError`導入で初めて検証できるようになったErrパス ===

        // 未知の材質名: スポーン6種・材料派生・材質物性の内省が同じ
        // `UnknownMaterial`を返す(材質名は`find_by_name`が引けなかった名前
        // そのものが載る)。
        let bogus = "存在しない材質";
        assert_err_is(
            world.spawn_sphere_impl(0.0, 1.0, 0.0, 0.3, bogus.to_string()),
            WasmError::UnknownMaterial(bogus.to_string()),
        );
        assert_err_is(
            world.spawn_box_impl(0.0, 1.0, 0.0, 0.3, bogus.to_string()),
            WasmError::UnknownMaterial(bogus.to_string()),
        );
        assert_err_is(
            world.spawn_capsule_impl(0.0, 1.0, 0.0, 0.3, 0.5, bogus.to_string()),
            WasmError::UnknownMaterial(bogus.to_string()),
        );
        assert_err_is(
            world.spawn_compound_l_shape_impl(0.0, 1.0, 0.0, bogus.to_string()),
            WasmError::UnknownMaterial(bogus.to_string()),
        );
        assert_err_is(
            world.spawn_convex_mesh_cube_impl(0.0, 1.0, 0.0, 0.5, bogus.to_string()),
            WasmError::UnknownMaterial(bogus.to_string()),
        );
        // 任意形状スポナーも同じ規約——**形状JSONの妥当性より材質名の解決が
        // 後**である点まで含めて確かめる(形状は読めているのに材質で落ちる)。
        assert_err_is(
            world.spawn_shape_json_impl(r#"{"sphere":{"radius":0.3}}"#, 0.0, 1.0, 0.0, bogus),
            WasmError::UnknownMaterial(bogus.to_string()),
        );
        assert_err_is(
            world.material_properties_f64_impl(bogus),
            WasmError::UnknownMaterial(bogus.to_string()),
        );
        assert_err_is(
            world.derive_material_impl(bogus.to_string(), "派生先".to_string(), 1500.0),
            WasmError::UnknownMaterial(bogus.to_string()),
        );
        // 失敗したスポーンはボディを1つも増やしていない(Errが状態を汚していない)。
        assert_eq!(world.read_component_impl("body_count", "").unwrap(), "9");

        // 派生先の名前が既存とぶつかる / 密度が正の有限値でない。
        assert_err_is(
            world.derive_material_impl(
                "アルミニウム".to_string(),
                "軽量アルミニウム(Task8第四弾テスト)".to_string(),
                1500.0,
            ),
            WasmError::MaterialAlreadyExists("軽量アルミニウム(Task8第四弾テスト)".to_string()),
        );
        for bad_density in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert_err_is(
                world.derive_material_impl(
                    "アルミニウム".to_string(),
                    format!("密度不正{bad_density}"),
                    bad_density,
                ),
                WasmError::InvalidDensity,
            );
        }

        // 範囲外index: 内省系は`try_body_id_at`(件数付き)か
        // `try_body_meta_at`(件数無し)のどちらかを通る——**文面が違うので
        // 別変種**になっていることまで確かめる(`WasmError`のdoc参照)。
        let count = world.body_count_impl();
        assert_err_is(
            world.body_is_static_at_impl(count),
            WasmError::BodyIndexOutOfRange {
                index: count,
                count,
            },
        );
        assert_err_is(
            world.body_shape_kind_at_impl(count),
            WasmError::BodyIndexOutOfRange {
                index: count,
                count,
            },
        );
        assert_err_is(
            world.body_shape_json_at_impl(count),
            WasmError::BodyIndexOutOfRange {
                index: count,
                count,
            },
        );
        assert_err_is(
            world.body_shape_label_at_impl(count),
            WasmError::BodyIndexOutOfRange {
                index: count,
                count,
            },
        );
        assert_err_is(
            world.body_label_at_impl(count),
            WasmError::BodyMetaIndexOutOfRange { index: count },
        );
        assert_err_is(
            world.body_material_label_at_impl(count),
            WasmError::BodyMetaIndexOutOfRange { index: count },
        );
        assert_err_is(
            world.body_is_removed_at_impl(count),
            WasmError::BodyMetaIndexOutOfRange { index: count },
        );
        assert_err_is(
            world.duplicate_body_at_impl(count, 1.0),
            WasmError::BodyMetaIndexOutOfRange { index: count },
        );
        assert_err_is(
            world.remove_body_at_impl(count),
            WasmError::BodyIndexOutOfRange {
                index: count,
                count,
            },
        );

        // 床(index 0)は削除できない。
        assert_err_is(world.remove_body_at_impl(0), WasmError::CannotRemoveFloor);
        assert_eq!(
            world
                .read_component_impl("body_is_removed_at", "0")
                .unwrap(),
            "false",
            "拒否された削除が床の状態を変えていないこと"
        );

        // 削除済みボディ(index 7、上で`remove_body_at`済み)。`World`側の世代が
        // 進んでいるため`try_body_id_at`は`BodyNoLongerExists`を返す——
        // 「範囲内だが生きていない」を「範囲外」と取り違えていないことの確認。
        assert_err_is(
            world.body_is_static_at_impl(7),
            WasmError::BodyNoLongerExists { index: 7 },
        );
        // 一方`try_body_meta_at`しか通らない内省は、削除済みでも`Ok`のまま
        // (`self.bodies`の行は残す設計、`remove_body_at`のdoc参照)。
        assert!(world.body_is_removed_at_impl(7).unwrap());
        // 複製は`body_position`が引けないため`CannotDuplicateRemovedBody`。
        assert_err_is(
            world.duplicate_body_at_impl(7, 1.0),
            WasmError::CannotDuplicateRemovedBody,
        );
    }

    /// **Task#8第五弾の回帰テスト**: 自由配線回路エディタ12個(適用系)と、
    /// 固定デモ回路のスイッチ・ヒーター、その内省6個も`apply_component`/
    /// `read_component`経由で操作できることを確認する。
    #[test]
    fn apply_component_and_read_component_wire_a_circuit_via_generic_dispatch() {
        let mut world = new_world();

        // 固定デモ回路のスイッチ(WasmWorld::newが積む分圧回路)。
        world
            .apply_component_impl("set_circuit_switch_closed", r#"{"closed":true}"#)
            .expect("set_circuit_switch_closed via apply_component must succeed");
        world.step();
        let divider_voltage: f64 = world
            .read_component_impl("circuit_divider_voltage", "")
            .unwrap()
            .parse()
            .unwrap();
        assert!(divider_voltage.abs() < 1e-6, "actual: {divider_voltage}");

        // ヒーター。
        world
            .apply_component_impl("push_heat_source", r#"{"watts":2000.0}"#)
            .expect("push_heat_source via apply_component must succeed");
        world.step();
        assert!(
            world
                .read_component_impl("heater_node_temperature", "")
                .unwrap()
                .parse::<f64>()
                .unwrap()
                > 293.15
        );

        // 自由配線回路エディタ: 既定回路数を0にリセットしてから3ノードの
        // 単純な回路(電圧源+抵抗+スイッチ+コンデンサ+インダクタ+ダイオード+
        // DCモーター)を組む。
        world
            .apply_component_impl("circuit_editor_reset", r#"{"num_nodes":3}"#)
            .expect("circuit_editor_reset via apply_component must succeed");
        assert_eq!(
            world
                .read_component_impl("circuit_element_count", "")
                .unwrap(),
            "0"
        );

        world
            .apply_component_impl(
                "circuit_editor_add_voltage_source",
                r#"{"a":1,"b":0,"voltage":10.0}"#,
            )
            .expect("circuit_editor_add_voltage_source via apply_component must succeed");
        world
            .apply_component_impl(
                "circuit_editor_add_resistor",
                r#"{"a":1,"b":2,"resistance":100.0}"#,
            )
            .expect("circuit_editor_add_resistor via apply_component must succeed");
        assert_eq!(
            world
                .read_component_impl("circuit_element_count", "")
                .unwrap(),
            "2"
        );
        let label = world
            .read_component_impl("circuit_element_label_at", "0")
            .unwrap();
        assert!(label.contains("10"), "actual: {label}");

        let result = world
            .apply_component_impl(
                "circuit_editor_add_switch",
                r#"{"a":2,"b":0,"closed":false}"#,
            )
            .expect("circuit_editor_add_switch via apply_component must succeed");
        assert_eq!(result, "{\"index\":0}");
        world
            .apply_component_impl(
                "circuit_editor_set_switch_closed",
                r#"{"index":0,"closed":true}"#,
            )
            .expect("circuit_editor_set_switch_closed via apply_component must succeed");

        world
            .apply_component_impl(
                "circuit_editor_add_capacitor",
                r#"{"a":1,"b":2,"capacitance":1e-3,"initial_voltage":0.0}"#,
            )
            .expect("circuit_editor_add_capacitor via apply_component must succeed");
        world
            .apply_component_impl(
                "circuit_editor_add_inductor",
                r#"{"a":1,"b":2,"inductance":1e-3,"initial_current":0.0}"#,
            )
            .expect("circuit_editor_add_inductor via apply_component must succeed");
        world
            .apply_component_impl(
                "circuit_editor_add_diode",
                r#"{"anode":1,"cathode":2,"saturation_current":1e-12,"n_vt":0.026}"#,
            )
            .expect("circuit_editor_add_diode via apply_component must succeed");

        let result = world
            .apply_component_impl(
                "circuit_editor_add_dc_motor",
                r#"{"a":1,"b":2,"winding_resistance":1.0,"winding_inductance":1e-3,"back_emf_constant":0.05}"#,
            )
            .expect("circuit_editor_add_dc_motor via apply_component must succeed");
        assert_eq!(result, "{\"index\":0}");
        world
            .apply_component_impl(
                "circuit_editor_set_motor_speed",
                r#"{"index":0,"angular_velocity":10.0}"#,
            )
            .expect("circuit_editor_set_motor_speed via apply_component must succeed");
        world.step();
        // モーター電流の内省(値そのものは検証しない、往復できることのみ確認)。
        let _ = world
            .read_component_impl("circuit_editor_motor_current", "0")
            .unwrap()
            .parse::<f64>()
            .unwrap();
        let _ = world
            .read_component_impl("circuit_node_voltage", "1")
            .unwrap()
            .parse::<f64>()
            .unwrap();
    }

    /// **Task#8第六弾の回帰テスト**: フレーム(`add_rotating_frame`/
    /// `add_child_frame`)・ヒンジモーター(`set_motor_target_at`)の適用系3個と、
    /// 時刻/step/ハッシュ/エネルギー/近似バッジ/インポート済みprobe/frameの
    /// 内省11個も`apply_component`/`read_component`経由で操作できることを
    /// 確認する。
    #[test]
    fn apply_component_and_read_component_wire_frames_and_read_misc_info_via_generic_dispatch() {
        let mut world = new_world();

        assert_eq!(world.read_component_impl("step_count", "").unwrap(), "0");
        assert_eq!(world.read_component_impl("time", "").unwrap(), "0");
        let hash_before = world.read_component_impl("state_hash", "").unwrap();
        assert_eq!(hash_before.len(), 16);
        world.step();
        assert_eq!(world.read_component_impl("step_count", "").unwrap(), "1");
        let _ = world
            .read_component_impl("energy_residual", "")
            .unwrap()
            .parse::<f64>()
            .unwrap();
        let _ = world
            .read_component_impl("max_body_speed", "")
            .unwrap()
            .parse::<f64>()
            .unwrap();
        // 既定シーンの各ソルバが自己申告する近似・縮約(タブ区切り、複数行)。
        assert!(!world
            .read_component_impl("active_approximations_text", "")
            .unwrap()
            .is_empty());
        assert_eq!(
            world
                .read_component_impl("imported_probe_count", "")
                .unwrap(),
            "0"
        );

        let result = world
            .apply_component_impl("add_rotating_frame", r#"{"angular_velocity_z":1.0}"#)
            .expect("add_rotating_frame via apply_component must succeed");
        assert_eq!(result, "{\"index\":1}");
        assert_eq!(world.read_component_impl("frame_count", "").unwrap(), "2");
        assert_eq!(
            world
                .read_component_impl("frame_parent_index", "1")
                .unwrap(),
            "0"
        );

        let result = world
            .apply_component_impl(
                "add_child_frame",
                r#"{"parent_index":1,"origin_offset_x":0.0,"origin_offset_y":0.0,"origin_offset_z":0.0,"angular_velocity_z":0.5}"#,
            )
            .expect("add_child_frame via apply_component must succeed");
        assert_eq!(result, "{\"index\":2}");
        assert_eq!(
            world
                .read_component_impl("frame_parent_index", "2")
                .unwrap(),
            "1"
        );

        // `set_motor_target_at`の成功パス——`spawn_motor_arm`(モーターを実際に
        // 持つボディを作る)を使う。
        let motor_arm = world
            .spawn_motor_arm_impl(0.0, 2.0, 0.0, "アルミニウム".to_string())
            .expect("spawn_motor_arm must succeed");
        world
            .apply_component_impl(
                "set_motor_target_at",
                &format!(r#"{{"index":{motor_arm},"theta_target":0.5}}"#),
            )
            .expect("set_motor_target_at via apply_component must succeed");

        // **`WasmError`導入で検証できるようになったErrパス**(以前はここに
        // 「SIGABRTするため成功パスのみ確認する」と書いてあった)。
        // モーターを持たないボディ(index 1 = 既定シーンの箱)へ呼ぶと
        // `BodyHasNoHingeMotor`——`BodyMetaIndexOutOfRange`ではないことまで見る。
        assert_err_is(
            world.set_motor_target_at_impl(1, 0.5),
            WasmError::BodyHasNoHingeMotor { index: 1 },
        );
        // 範囲外indexなら`try_body_meta_at`側で先に弾かれる。
        let count = world.body_count_impl();
        assert_err_is(
            world.set_motor_target_at_impl(count, 0.5),
            WasmError::BodyMetaIndexOutOfRange { index: count },
        );

        // フレームindexの範囲外(`check_frame_index`を通る4経路)。
        let frames = world.frame_count_impl();
        for result in [
            world.frame_parent_index_impl(frames).map(|_| ()),
            world.frame_rotation_at_impl(frames).map(|_| ()),
            world.frame_world_position_impl(frames).map(|_| ()),
            world.frame_world_rotation_impl(frames).map(|_| ()),
        ] {
            assert_err_is(
                result,
                WasmError::FrameIndexOutOfRange {
                    index: frames,
                    count: frames,
                },
            );
        }
        // 親フレームが範囲外なら子フレームも作れない(`World::add_frame`の
        // `assert!`へ落ちる前にここで弾く、`add_child_frame_impl`のコメント参照)。
        assert_err_is(
            world.add_child_frame_impl(frames, 0.0, 0.0, 0.0, 0.5),
            WasmError::FrameIndexOutOfRange {
                index: frames,
                count: frames,
            },
        );
        assert_eq!(
            world.frame_count_impl(),
            frames,
            "拒否された追加がフレーム数を増やしていないこと"
        );
    }

    /// **Task#8第七弾の回帰テスト**: スポーン2種(振り子・モーターアームは
    /// 第六弾で既に`_impl`直呼びで使っているため、ここでは振り子のみ改めて
    /// `apply_component`経由で確認)・SPH流体スポーン・スナップショット・
    /// ブックマーク・シーンJSONエクスポートも`apply_component`/
    /// `read_component`経由で操作できることを確認する。
    #[test]
    fn apply_component_and_read_component_spawn_pendulum_snapshot_and_bookmark_via_generic_dispatch(
    ) {
        let mut world = new_world();

        let result = world
            .apply_component_impl(
                "spawn_pendulum",
                r#"{"pivot_x":0.0,"pivot_y":5.0,"pivot_z":0.0,"arm_length":1.0,"material_name":"アルミニウム"}"#,
            )
            .expect("spawn_pendulum via apply_component must succeed");
        assert_eq!(result, "{\"index\":2}");

        world
            .apply_component_impl("spawn_fluid_block", "{}")
            .expect("spawn_fluid_block via apply_component must succeed");
        assert_eq!(
            world.read_component_impl("fluid_spawn_count", "").unwrap(),
            "1"
        );
        assert!(
            world
                .read_component_impl("fluid_particle_count", "")
                .unwrap()
                .parse::<usize>()
                .unwrap()
                > 0
        );
        // 3D格子流体・エネルギー内訳は既定シーンでは無効/空でも、往復できる
        // ことだけを確認する(文字列が返る、パニックしない)。
        let _ = world
            .read_component_impl("grid_fluid_3d_summary", "")
            .unwrap();
        let _ = world.read_component_impl("energy_report_text", "").unwrap();

        assert_eq!(
            world.read_component_impl("snapshot_count", "").unwrap(),
            "0"
        );
        world.step();
        // **`WasmError`導入で検証できるようになったErrパス**(以前はここに
        // 「無効indexで呼ばない、SIGABRTするため」と書いてあった)。
        // スナップショットは`snapshot_interval_steps`ごとにしか積まれない
        // (`WasmWorld::step`参照)ので、1step後の`snapshot_count`は0のまま
        // ——つまりindex 0 が既に範囲外であり、`count: 0`まで検証できる。
        assert_eq!(world.snapshot_count_impl(), 0);
        assert_err_is(
            world.snapshot_time_at_impl(0),
            WasmError::SnapshotIndexOutOfRange { index: 0, count: 0 },
        );
        assert_err_is(
            world.restore_snapshot_impl(0),
            WasmError::SnapshotIndexOutOfRange { index: 0, count: 0 },
        );
        // ブックマークもまだ1件も無いので同様に範囲外。
        assert_err_is(
            world.bookmark_label_at_impl(0),
            WasmError::BookmarkIndexOutOfRange { index: 0, count: 0 },
        );
        assert_err_is(
            world.bookmark_time_at_impl(0),
            WasmError::BookmarkIndexOutOfRange { index: 0, count: 0 },
        );
        assert_err_is(
            world.bookmark_export_scene_json_impl(0),
            WasmError::BookmarkIndexOutOfRange { index: 0, count: 0 },
        );
        assert_err_is(
            world.restore_bookmark_impl(0),
            WasmError::BookmarkIndexOutOfRange { index: 0, count: 0 },
        );

        world
            .apply_component_impl("add_bookmark", r#"{"label":"Task8第七弾テスト"}"#)
            .expect("add_bookmark via apply_component must succeed");
        assert_eq!(
            world.read_component_impl("bookmark_count", "").unwrap(),
            "1"
        );
        assert_eq!(
            world.read_component_impl("bookmark_label_at", "0").unwrap(),
            "Task8第七弾テスト"
        );
        let _ = world
            .read_component_impl("bookmark_time_at", "0")
            .unwrap()
            .parse::<f64>()
            .unwrap();
        let exported = world
            .read_component_impl("bookmark_export_scene_json", "0")
            .unwrap();
        assert!(exported.contains("\"bodies\""), "actual: {exported}");

        world
            .apply_component_impl("restore_bookmark", r#"{"index":0}"#)
            .expect("restore_bookmark via apply_component must succeed");

        let current = world.read_component_impl("export_scene_json", "").unwrap();
        assert!(current.contains("\"bodies\""), "actual: {current}");

        // ブックマークを1件積んだ後の範囲外(境界のindex==countがErrで、
        // index==count-1がOkであること——off-by-oneを取り違えていない確認)。
        let bookmarks = world.bookmark_count_impl();
        assert_eq!(bookmarks, 1);
        assert!(world.bookmark_label_at_impl(bookmarks - 1).is_ok());
        assert_err_is(
            world.bookmark_label_at_impl(bookmarks),
            WasmError::BookmarkIndexOutOfRange {
                index: bookmarks,
                count: bookmarks,
            },
        );
    }

    /// **残タスク完遂増分**(レビュー指摘「見送らず対応すること」への対応):
    /// `set_gravity_direction`が既定の下向きから変更でき、正規化されること。
    /// `gravity_direction()`は`Float64Array`を返すためここでは呼ばない
    /// (本テストモジュール冒頭のdoc comment参照)——`self.inner`経由で
    /// `MechanicsSolver::gravity_direction`を直接読む。
    #[test]
    fn set_gravity_direction_changes_and_normalizes_the_stored_direction() {
        let mut world = new_world();
        assert_eq!(
            world.inner.mechanics().gravity_direction(),
            sim_math::Vec3::new(0.0, -1.0, 0.0)
        );

        world.set_gravity_direction_impl(3.0, 0.0, 0.0);
        let direction = world.inner.mechanics().gravity_direction();
        assert!((direction.length() - 1.0).abs() < 1e-12);
        assert!((direction.x - 1.0).abs() < 1e-12);

        // ゼロベクトルは既定の下向きへ安全にフォールバックする。
        world.set_gravity_direction_impl(0.0, 0.0, 0.0);
        assert_eq!(
            world.inner.mechanics().gravity_direction(),
            sim_math::Vec3::new(0.0, -1.0, 0.0)
        );
    }

    /// **重力場の抽象化増分**: 新しい`push_set_gravity_field` kindが3種の場を
    /// 構築でき、**Commandとして次stepの先頭で適用され`commandLog`へ記録される**
    /// こと(`sim_world::Command::SetGravityField`のdoc「黙ってリプレイされない
    /// 変更は決定論のバグ」)。読み戻しは`gravity_field` kindで行い、
    /// 書いた`kind`名がそのまま返ることも確かめる(往復できる形にしてある)。
    #[test]
    fn push_set_gravity_field_builds_every_kind_and_is_recorded_as_a_command() {
        let mut world = new_world();

        world
            .apply_component_impl(
                "push_set_gravity_field",
                r#"{"kind":"point_source","center_x":0.0,"center_y":-100.0,"center_z":0.0,"mu":4.0e14}"#,
            )
            .unwrap();
        // Commandなのでstepするまで効かない。
        assert!(matches!(
            world.inner.mechanics().gravity_field(),
            sim_mechanics::GravityField::Uniform { .. }
        ));
        world.step();
        assert_eq!(
            world.inner.mechanics().gravity_field(),
            sim_mechanics::GravityField::PointSource {
                center: Vec3::new(0.0, -100.0, 0.0),
                mu: 4.0e14,
            }
        );
        assert_eq!(world.inner.command_log().len(), 1);
        let read_back: serde_json::Value =
            serde_json::from_str(&world.read_component_impl("gravity_field", "").unwrap()).unwrap();
        assert_eq!(read_back["kind"], "point_source");
        assert_eq!(read_back["mu"], 4.0e14);

        world
            .apply_component_impl("push_set_gravity_field", r#"{"kind":"zero"}"#)
            .unwrap();
        world.step();
        assert_eq!(
            world.inner.mechanics().gravity_field(),
            sim_mechanics::GravityField::Zero
        );
        let read_back: serde_json::Value =
            serde_json::from_str(&world.read_component_impl("gravity_field", "").unwrap()).unwrap();
        assert_eq!(read_back["kind"], "zero");

        // `uniform`は向きを正規化して保持する(既存の`set_gravity_direction`と
        // 同じ不変条件、`MechanicsSolver::set_gravity_field`のdoc参照)。
        world
            .apply_component_impl(
                "push_set_gravity_field",
                r#"{"kind":"uniform","magnitude":1.62,"x":0.0,"y":0.0,"z":5.0}"#,
            )
            .unwrap();
        world.step();
        assert_eq!(
            world.inner.mechanics().gravity_field(),
            sim_mechanics::GravityField::Uniform {
                magnitude: 1.62,
                direction: Vec3::new(0.0, 0.0, 1.0),
            }
        );
        let read_back: serde_json::Value =
            serde_json::from_str(&world.read_component_impl("gravity_field", "").unwrap()).unwrap();
        assert_eq!(read_back["kind"], "uniform");
        assert_eq!(read_back["magnitude"], 1.62);

        // 知らない`kind`は弾く(黙って既定の場にしない)。
        assert_err_is(
            world.apply_component_impl("push_set_gravity_field", r#"{"kind":"radial"}"#),
            WasmError::UnknownGravityFieldKind,
        );
    }

    /// **重力場の抽象化増分**: 既存の`set_gravity`/`set_gravity_direction`/
    /// `gravity`/`gravity_direction`が**移行前と1ビットも変わらず**動くこと
    /// (フロントエンドは本増分の対象外なので、ここが壊れると即座に実害になる)。
    /// あわせて、非`Uniform`な場での縮約(`MechanicsSolver::gravity`のdoc)も固定する。
    #[test]
    fn legacy_gravity_kinds_keep_working_and_degrade_predictably_for_non_uniform_fields() {
        let mut world = new_world();
        world
            .apply_component_impl("set_gravity", r#"{"gravity":3.71}"#)
            .unwrap();
        assert_eq!(world.read_component_impl("gravity", "").unwrap(), "3.71");
        world
            .apply_component_impl("set_gravity_direction", r#"{"x":2.0,"y":0.0,"z":0.0}"#)
            .unwrap();
        // 大きさは向きの変更で保たれる。
        assert_eq!(world.read_component_impl("gravity", "").unwrap(), "3.71");
        assert_eq!(
            world.inner.mechanics().gravity_direction(),
            Vec3::new(1.0, 0.0, 0.0)
        );

        // 点源場では、スカラー2つのAPIは「一様場として見た値」=(0.0, 下向き)
        // へ縮退する(自然対流のレイリー数はその前提を失うため意図的に無効化
        // される。**浮力は重力追従増分でこの縮約から外れ**、
        // `GravityField::up_and_magnitude_at`経由で点源場でも効く)。
        world
            .apply_component_impl(
                "push_set_gravity_field",
                r#"{"kind":"point_source","center_x":0.0,"center_y":0.0,"center_z":0.0,"mu":1.0e5}"#,
            )
            .unwrap();
        world.step();
        assert_eq!(world.read_component_impl("gravity", "").unwrap(), "0");
        assert_eq!(
            world.inner.mechanics().gravity_direction(),
            Vec3::new(0.0, -1.0, 0.0)
        );
        // スカラーAPIで書き戻すと一様場を選んだことになる(同docの規則)。
        world
            .apply_component_impl("set_gravity", r#"{"gravity":9.80665}"#)
            .unwrap();
        assert_eq!(
            world.inner.mechanics().gravity_field(),
            sim_mechanics::GravityField::Uniform {
                magnitude: 9.80665,
                direction: Vec3::new(0.0, -1.0, 0.0),
            }
        );
    }

    /// Inspectorの Add Coupling(**残タスク完遂の縦串②増分**)——3種
    /// (ImageChargeForce/LorentzForce/BuoyancyDrag、いずれも剛体参照だけで
    /// 完結する)がパニックせず追加でき、`coupling_count`/`coupling_info_text`
    /// (Inspectorの読み取り側)に反映されること。
    #[test]
    fn add_coupling_methods_succeed_and_are_visible_in_coupling_info_text() {
        let mut world = new_world();
        let body = world
            .spawn_sphere_impl(0.0, 5.0, 0.0, 0.3, "鋼(炭素鋼)".to_string())
            .unwrap();

        world
            .add_image_charge_force_coupling_impl(body, 1e-6, 1.0, 0.0, 0.0, 2.0)
            .expect("image charge force coupling must succeed");
        world
            .add_lorentz_force_coupling_impl(body, 1e-6)
            .expect("lorentz force coupling must succeed");
        world
            .add_buoyancy_drag_coupling_impl(body, 0.0, 1000.0)
            .expect("buoyancy drag coupling must succeed");

        assert_eq!(world.coupling_count_impl(), 3);
        let text = world.coupling_info_text_impl(-1);
        for kind in ["ImageChargeForce", "LorentzForce", "BuoyancyDrag"] {
            assert!(
                text.contains(kind),
                "coupling_info_text must report a {kind} line, got:\n{text}"
            );
        }

        // **`WasmError`導入で検証できるようになったErrパス**(以前はここに
        // 「死んだボディを指すと`Err`」とだけ書いて確認していなかった)。
        // 3種とも`try_body_id_at`を通るので、範囲外indexは`BodyIndexOutOfRange`。
        let count = world.body_count_impl();
        assert_err_is(
            world.add_image_charge_force_coupling_impl(count, 1e-6, 1.0, 0.0, 0.0, 2.0),
            WasmError::BodyIndexOutOfRange {
                index: count,
                count,
            },
        );
        assert_err_matches!(
            world.add_lorentz_force_coupling_impl(count, 1e-6),
            WasmError::BodyIndexOutOfRange { .. }
        );
        assert_err_matches!(
            world.add_buoyancy_drag_coupling_impl(count, 0.0, 1000.0),
            WasmError::BodyIndexOutOfRange { .. }
        );

        // 「死んだボディを指すと`Err`」——範囲内だが削除済みのボディは
        // `BodyNoLongerExists`(範囲外とは別変種)になる。
        let doomed = world
            .spawn_sphere_impl(0.0, 6.0, 0.0, 0.3, "鋼(炭素鋼)".to_string())
            .unwrap();
        world.remove_body_at_impl(doomed).unwrap();
        assert_err_is(
            world.add_lorentz_force_coupling_impl(doomed, 1e-6),
            WasmError::BodyNoLongerExists { index: doomed },
        );
        assert_eq!(
            world.coupling_count_impl(),
            3,
            "拒否された追加がcouplingを増やしていないこと"
        );
    }

    /// Add Coupling——熱・回路ドメインを参照する5種
    /// (DissipationToHeat/JouleHeat/BrownianForce/MotorCoupling/
    /// InductionCoupling)。`new_world()`(既定の起動シーンと同じ構成)は
    /// 熱ノード1個(index 0)・電圧源1個(index 0)を最初から持つため、
    /// それらを参照して成功することを確認する。**範囲外indexで`Err`になること
    /// も実行時に確認する**(`WasmError`導入前は「wasm-bindgen-testが要るため
    /// 対象外」としていた箇所、モジュール冒頭のテストdoc参照)。
    #[test]
    fn add_thermal_and_circuit_coupling_methods_succeed_with_valid_indices_and_reject_invalid_ones()
    {
        let mut world = new_world();
        let body = world
            .spawn_sphere_impl(0.0, 5.0, 0.0, 0.3, "鋼(炭素鋼)".to_string())
            .unwrap();

        world
            .add_dissipation_to_heat_coupling_impl(0)
            .expect("dissipation to heat with valid thermal node must succeed");
        world
            .add_joule_heat_coupling_impl(0)
            .expect("joule heat with valid thermal node must succeed");
        world
            .add_brownian_force_coupling_impl(body, 0.05, 1e-3, 0, 1, 2)
            .expect("brownian force with valid thermal node must succeed");
        world
            .add_motor_coupling_impl(body, 0.0, 1.0, 0.0, 0, 0.1)
            .expect("motor coupling with valid voltage source must succeed");
        world
            .add_induction_coupling_impl(body, 0, 0.5, 1.0, 0.0, 1.0, 0.0)
            .expect("induction coupling with valid voltage source must succeed");
        assert_eq!(world.coupling_count_impl(), 5);

        let text = world.coupling_info_text_impl(-1);
        for kind in [
            "DissipationToHeat",
            "JouleHeat",
            "BrownianForce",
            "MotorCoupling",
            "InductionCoupling",
        ] {
            assert!(
                text.contains(kind),
                "coupling_info_text must report a {kind} line, got:\n{text}"
            );
        }

        // === Errパス(以前は「wasm-bindgen-testが要るため対象外」だった) ===

        // 熱ノードindexの範囲外(`try_thermal_node_index`)。既定シーンの
        // 熱ノードは1個なので index 1 が最初の範囲外。`count`まで載ることで
        // 「熱ドメイン自体が無効(count==0)」と区別できる。
        let thermal_nodes = world.thermal_node_count_impl();
        assert_eq!(thermal_nodes, 1);
        assert_err_is(
            world.add_dissipation_to_heat_coupling_impl(thermal_nodes),
            WasmError::ThermalNodeIndexOutOfRange {
                index: thermal_nodes,
                count: thermal_nodes,
            },
        );
        assert_err_matches!(
            world.add_joule_heat_coupling_impl(thermal_nodes),
            WasmError::ThermalNodeIndexOutOfRange { .. }
        );
        assert_err_matches!(
            world.add_brownian_force_coupling_impl(body, 0.05, 1e-3, thermal_nodes, 1, 2),
            WasmError::ThermalNodeIndexOutOfRange { .. }
        );

        // 電圧源indexの範囲外(`try_voltage_source_index`)。熱ノードとは
        // **別の変種**になることを見る——両者を取り違えると、UIには
        // 見当違いの「熱ドメインは有効か?」という案内が出てしまう。
        assert_err_matches!(
            world.add_motor_coupling_impl(body, 0.0, 1.0, 0.0, 99, 0.1),
            WasmError::VoltageSourceIndexOutOfRange { index: 99, .. }
        );
        assert_err_matches!(
            world.add_induction_coupling_impl(body, 99, 0.5, 1.0, 0.0, 1.0, 0.0),
            WasmError::VoltageSourceIndexOutOfRange { index: 99, .. }
        );

        // 剛体indexの範囲外(熱・回路indexが正しくてもこちらで弾かれる)。
        let count = world.body_count_impl();
        assert_err_matches!(
            world.add_brownian_force_coupling_impl(count, 0.05, 1e-3, 0, 1, 2),
            WasmError::BodyIndexOutOfRange { .. }
        );
        assert_err_matches!(
            world.add_motor_coupling_impl(count, 0.0, 1.0, 0.0, 0, 0.1),
            WasmError::BodyIndexOutOfRange { .. }
        );
        assert_err_matches!(
            world.add_induction_coupling_impl(count, 0, 0.5, 1.0, 0.0, 1.0, 0.0),
            WasmError::BodyIndexOutOfRange { .. }
        );

        assert_eq!(
            world.coupling_count_impl(),
            5,
            "拒否された追加がcouplingを増やしていないこと"
        );
    }

    /// **残タスク完遂の縦串②残り6種を解禁する増分**——レビュー指摘
    /// (「やり遂げて欲しい」)への対応。`add_thermal_node`/
    /// `enable_grid_fluid_2d_domain`/`enable_gas_compartment`(いずれも
    /// UIから一から組んだシーンにドメインを作る新設経路)を先に呼んでから、
    /// PhaseChangeMorph/SphRigid/GridFluidRigid/ConvectionLink/PistonGas/
    /// BoussinesqBuoyancyの6種すべてが成功パスで追加できることを確認する。
    #[test]
    fn remaining_six_coupling_kinds_succeed_once_their_domains_are_created_via_new_wasm_methods() {
        let mut world = new_world();
        let body = world
            .spawn_sphere_impl(0.0, 5.0, 0.0, 0.3, "鋼(炭素鋼)".to_string())
            .unwrap();

        // 熱ノードをUIから新規作成(index 1、既定シーンのindex 0は別ノード)。
        let node = world.add_thermal_node_impl(273.15, 100.0);
        assert_eq!(node, 1);
        assert_eq!(world.thermal_node_count_impl(), 2);

        // **ドメイン未有効のErrパス**(以前は「wasm-bindgen-testが要るため
        // 対象外」としていた箇所、モジュール冒頭のテストdoc参照)。この時点では
        // SPH・格子流体・気体区画のいずれもまだ有効化していない——**下で
        // 有効化する前にここで確かめるのが要点**で、3ドメインがそれぞれ
        // 固有の変種を返す(取り違えるとUIに出る復旧手順の案内が食い違う)。
        assert_err_is(
            world.add_sph_rigid_coupling_impl(body, 0.2, 12),
            WasmError::SphDomainNotEnabled,
        );
        assert_err_is(
            world.add_grid_fluid_rigid_coupling_impl(body, 0.3, 0.3),
            WasmError::GridFluidDomainNotEnabled,
        );
        assert_err_is(
            world.add_boussinesq_buoyancy_coupling_impl(node, 293.15, 3.4e-3),
            WasmError::GridFluidDomainNotEnabled,
        );
        assert_err_is(
            world.add_piston_gas_coupling_impl(body, 0.0, 1.0, 0.0, 0.01, 0.001),
            WasmError::GasCompartmentNotEnabled,
        );
        // 剛体indexの検証はドメイン判定より手前で走る(`try_body_id_at`が先)
        // ——両方不正なときに「ドメインが無い」と誤って案内しないこと。
        let out_of_range = world.body_count_impl();
        assert_err_matches!(
            world.add_sph_rigid_coupling_impl(out_of_range, 0.2, 12),
            WasmError::BodyIndexOutOfRange { .. }
        );
        assert_err_matches!(
            world.add_grid_fluid_rigid_coupling_impl(out_of_range, 0.3, 0.3),
            WasmError::BodyIndexOutOfRange { .. }
        );
        assert_err_matches!(
            world.add_piston_gas_coupling_impl(out_of_range, 0.0, 1.0, 0.0, 0.01, 0.001),
            WasmError::BodyIndexOutOfRange { .. }
        );
        // 同じく`add_boussinesq_buoyancy`は熱ノード判定が格子流体判定より先。
        assert_err_matches!(
            world.add_boussinesq_buoyancy_coupling_impl(99, 293.15, 3.4e-3),
            WasmError::ThermalNodeIndexOutOfRange { index: 99, .. }
        );
        assert_eq!(
            world.coupling_count_impl(),
            0,
            "拒否された追加がcouplingを1件も増やしていないこと"
        );

        world
            .add_phase_change_morph_coupling_impl(
                body, node, 273.15, 334_000.0, 2100.0, 4186.0, 1.0, 10.0, -50_000.0,
            )
            .expect("phase change morph must succeed once a thermal node exists");

        world.spawn_fluid_block_impl(); // SPHドメインを有効化。
        world
            .add_sph_rigid_coupling_impl(body, 0.2, 12)
            .expect("sph rigid must succeed once the SPH domain exists");

        world.enable_grid_fluid_2d_domain_impl();
        world
            .add_grid_fluid_rigid_coupling_impl(body, 0.3, 0.3)
            .expect("grid fluid rigid must succeed once the grid fluid domain exists");
        world
            .add_boussinesq_buoyancy_coupling_impl(node, 293.15, 3.4e-3)
            .expect("boussinesq buoyancy must succeed once the grid fluid domain exists");
        world
            .add_convection_link_coupling_impl(0, node, 0.01, 0.05, 3, 0.026, 1.5e-5, 0.71, 0.0)
            .expect("convection link must succeed with valid thermal node indices");

        world.enable_gas_compartment_impl();
        world
            .add_piston_gas_coupling_impl(body, 0.0, 1.0, 0.0, 0.01, 0.001)
            .expect("piston gas must succeed once the gas compartment exists");

        assert_eq!(world.coupling_count_impl(), 6);
        let text = world.coupling_info_text_impl(-1);
        for kind in [
            "PhaseChangeMorph",
            "SphRigid",
            "GridFluidRigid",
            "BoussinesqBuoyancy",
            "ConvectionLink",
            "PistonGas",
        ] {
            assert!(
                text.contains(kind),
                "coupling_info_text must report a {kind} line, got:\n{text}"
            );
        }

        // 熱ノードindexの範囲外は、ドメインを全て有効化した後でも
        // `ThermalNodeIndexOutOfRange`のまま(`PhaseChangeMorph`と
        // `ConvectionLink`はどちらも`try_thermal_node_index`を通る)。
        let nodes = world.thermal_node_count_impl();
        assert_err_is(
            world.add_phase_change_morph_coupling_impl(
                body, nodes, 273.15, 334_000.0, 2100.0, 4186.0, 1.0, 10.0, -50_000.0,
            ),
            WasmError::ThermalNodeIndexOutOfRange {
                index: nodes,
                count: nodes,
            },
        );
        assert_err_matches!(
            world.add_convection_link_coupling_impl(
                nodes, node, 0.01, 0.05, 3, 0.026, 1.5e-5, 0.71, 0.0
            ),
            WasmError::ThermalNodeIndexOutOfRange { .. }
        );
        assert_err_matches!(
            world.add_convection_link_coupling_impl(
                0, nodes, 0.01, 0.05, 3, 0.026, 1.5e-5, 0.71, 0.0
            ),
            WasmError::ThermalNodeIndexOutOfRange { .. }
        );
        assert_eq!(world.coupling_count_impl(), 6);
    }

    /// **残タスク完遂の縦串⑤増分**(飛行機の物理: 揚力の配線+操縦面Command)。
    /// `add_wing_lift_coupling`/`add_magnus_lift_coupling`が成功パスで追加でき、
    /// `coupling_info_text`にBuoyancyDragとして反映され(揚力自体は`LiftModel`の
    /// 内部状態でありkind名には出ない、他の`lift`フィールドを持つ結合と同じ)、
    /// `push_set_coupling_control_surface_deflection`が(範囲外indexでも)
    /// パニックせず1step適用できることを確認する。
    #[test]
    fn wing_and_magnus_lift_couplings_succeed_and_control_surface_deflection_applies_without_panicking(
    ) {
        let mut world = new_world();
        let body = world
            .spawn_box_impl(0.0, 5.0, 0.0, 0.5, "アルミニウム".to_string())
            .unwrap();

        let wing_index = world
            .add_wing_lift_coupling_impl(body, 2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.225, 1.81e-5)
            .expect("wing lift coupling must succeed");
        assert_eq!(wing_index, 0);

        let magnus_body = world
            .spawn_sphere_impl(3.0, 5.0, 0.0, 0.3, "アルミニウム".to_string())
            .unwrap();
        let magnus_index = world
            .add_magnus_lift_coupling_impl(magnus_body, 0.3, 1.225, 1.81e-5)
            .expect("magnus lift coupling must succeed");
        assert_eq!(magnus_index, 1);

        assert_eq!(world.coupling_count_impl(), 2);
        let text = world.coupling_info_text_impl(-1);
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(
            lines.len(),
            2,
            "both lift couplings are BuoyancyDrag under the hood, got:\n{text}"
        );
        assert!(lines.iter().all(|l| l.starts_with("BuoyancyDrag\t")));

        // 操縦面の舵角をCommand経由で設定し、1step進めてもパニックしないこと
        // (無効なindexを渡しても無言で無視される、Commandのdoc参照)。
        world.push_set_coupling_control_surface_deflection_impl(wing_index, 0.1);
        world.push_set_coupling_control_surface_deflection_impl(9999, 0.1);
        world.step();

        // 揚力2種も`try_body_id_at`を通る(範囲外indexで`Err`)。
        let count = world.body_count_impl();
        assert_err_is(
            world.add_wing_lift_coupling_impl(
                count, 2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.225, 1.81e-5,
            ),
            WasmError::BodyIndexOutOfRange {
                index: count,
                count,
            },
        );
        assert_err_is(
            world.add_magnus_lift_coupling_impl(count, 0.3, 1.225, 1.81e-5),
            WasmError::BodyIndexOutOfRange {
                index: count,
                count,
            },
        );
        assert_eq!(
            world.coupling_count_impl(),
            2,
            "拒否された追加がcouplingを増やしていないこと"
        );
    }

    /// 特殊スポーン2種(振り子・モーターアーム)も未知の材質名を拒否すること。
    /// どちらも内部でJoint/モーターを組み立てるため、材質解決の失敗が
    /// **途中まで作った状態を残さない**(ボディ数が増えない)ことまで見る。
    #[test]
    fn spawn_pendulum_and_motor_arm_reject_unknown_material_without_leaving_partial_state() {
        let mut world = new_world();
        let before = world.body_count_impl();
        let bogus = "存在しない材質";

        assert_err_is(
            world.spawn_pendulum_impl(0.0, 5.0, 0.0, 1.0, bogus.to_string()),
            WasmError::UnknownMaterial(bogus.to_string()),
        );
        assert_err_is(
            world.spawn_motor_arm_impl(0.0, 2.0, 0.0, bogus.to_string()),
            WasmError::UnknownMaterial(bogus.to_string()),
        );
        assert_eq!(world.body_count_impl(), before);
        assert!(
            world.joint_info_text_impl(-1).is_empty(),
            "材質解決に失敗したスポーンがJointだけ作って終わっていないこと"
        );

        // 既知の材質なら両方成功する(上のErrが材質名以外の理由ではないこと)。
        world
            .spawn_pendulum_impl(0.0, 5.0, 0.0, 1.0, "アルミニウム".to_string())
            .expect("known material must succeed");
        world
            .spawn_motor_arm_impl(0.0, 2.0, 0.0, "アルミニウム".to_string())
            .expect("known material must succeed");
        assert!(world.body_count_impl() > before);
    }

    /// **Task#8第一弾の回帰テスト**: `apply_component`/`read_component`が
    /// 実際にJoint/Coupling/熱ノードの追加・内省を代替できることを、旧来の
    /// 専用メソッド(`_impl`版)を直接呼ぶテストとは別に、JSON経由の呼び出し
    /// 規約そのものを通して確認する——フロントエンドが実際に使うのと同じ
    /// 呼び出し方(kind文字列+JSONペイロード)。
    #[test]
    fn apply_component_and_read_component_add_a_joint_and_a_coupling_via_generic_dispatch() {
        let mut world = new_world();
        let body_a = world
            .spawn_box_impl(0.0, 5.0, 0.0, 0.5, "アルミニウム".to_string())
            .unwrap();
        let body_b = world
            .spawn_box_impl(2.0, 5.0, 0.0, 0.5, "アルミニウム".to_string())
            .unwrap();

        let result = world
            .apply_component_impl(
                "add_distance_joint",
                &format!(
                    r#"{{"body_a":{body_a},"ax":0,"ay":0,"az":0,"body_b":{body_b},"bx":0,"by":0,"bz":0,"length":2.0}}"#
                ),
            )
            .expect("add_distance_joint via apply_component must succeed");
        assert_eq!(result, "{\"index\":0}");
        let joint_text = world
            .read_component_impl("joint_info_text", "-1")
            .expect("joint_info_text via read_component must succeed");
        assert!(
            joint_text.starts_with("DistanceJoint\t"),
            "actual: {joint_text}"
        );

        let result = world
            .apply_component_impl(
                "add_lorentz_force_coupling",
                &format!(r#"{{"body":{body_a},"charge":1e-6}}"#),
            )
            .expect("add_lorentz_force_coupling via apply_component must succeed");
        assert_eq!(result, "{}");
        let count = world
            .read_component_impl("coupling_count", "")
            .expect("coupling_count via read_component must succeed");
        assert_eq!(count, "1");

        // 熱ノード追加はindexを返す作成系オペレーション。
        let result = world
            .apply_component_impl(
                "add_thermal_node",
                r#"{"temperature":293.15,"heat_capacity":100.0}"#,
            )
            .expect("add_thermal_node via apply_component must succeed");
        assert_eq!(result, "{\"index\":1}"); // 既定シーンが既にnode 0を持つ
        let node_count = world
            .read_component_impl("thermal_node_count", "")
            .expect("thermal_node_count via read_component must succeed");
        assert_eq!(node_count, "2");

        // **未知のkindは両メソッドともErrになる(無言で無視しない)**。
        // 以前は「`JsValue`のネイティブ構築がSIGABRTするため呼ばない」として
        // Playwright任せにしていたが、`WasmError`導入でここで直接確かめられる
        // ——`apply`側と`read`側が**別の変種**になる(どちらのディスパッチで
        // 落ちたのかがエラーだけで分かる)ことまで見る。
        assert_err_is(
            world.apply_component_impl("no_such_apply_kind", "{}"),
            WasmError::UnknownApplyComponentKind("no_such_apply_kind".to_string()),
        );
        assert_err_is(
            world.read_component_impl("no_such_read_kind", ""),
            WasmError::UnknownReadComponentKind("no_such_read_kind".to_string()),
        );
        // `apply`で有効なkindを`read`へ渡しても(その逆も)通らない。
        assert_err_is(
            world.read_component_impl("add_thermal_node", ""),
            WasmError::UnknownReadComponentKind("add_thermal_node".to_string()),
        );
        assert_err_is(
            world.apply_component_impl("coupling_count", "{}"),
            WasmError::UnknownApplyComponentKind("coupling_count".to_string()),
        );
        // payloadがJSONでなければkindの解決より手前で弾かれる。
        assert_err_matches!(
            world.apply_component_impl("add_thermal_node", "not json at all"),
            WasmError::ApplyComponentInvalidJson(_)
        );

        // `component_schema`が畳んだ代表的なkind(このテストで実際に使った
        // ものと、他の増分で畳んだ環境系)を過不足なく列挙していることを
        // 確認する——増分ごとに畳む本数が増えるため、正確な総数ではなく
        // 「畳んだはずのkindが必ず入っている」ことだけを検証する。
        let schema: serde_json::Value = serde_json::from_str(&world.component_schema())
            .expect("component_schema must produce valid JSON");
        // **Task#9で`apply`側の要素が文字列からオブジェクトになった**ため、
        // kind名は`"kind"`キーから取り出す(フィールドスキーマ自体の検証は
        // `component_schema_*`の各テスト)。
        let apply_kinds: Vec<&str> = schema["apply"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v["kind"].as_str().unwrap())
            .collect();
        let read_kinds: Vec<&str> = schema["read"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_str().unwrap())
            .collect();
        for kind in [
            "add_distance_joint",
            "add_lorentz_force_coupling",
            "add_thermal_node",
            "set_gravity",
            "set_atmosphere",
            "set_water_region",
        ] {
            assert!(apply_kinds.contains(&kind), "missing apply kind: {kind}");
        }
        for kind in [
            "coupling_count",
            "joint_info_text",
            "thermal_node_count",
            "gravity",
            "atmosphere_density",
            "water_level",
        ] {
            assert!(read_kinds.contains(&kind), "missing read kind: {kind}");
        }
    }

    /// `component_schema`の`apply`表から、あるkindの`fields`配列を取り出す
    /// (以下のスキーマ検証テスト共通のヘルパ)。
    fn apply_fields_of(kind: &str) -> serde_json::Value {
        let world = new_world();
        let schema: serde_json::Value = serde_json::from_str(&world.component_schema())
            .expect("component_schema must produce valid JSON");
        schema["apply"]
            .as_array()
            .expect("apply must be an array")
            .iter()
            .find(|entry| entry["kind"] == kind)
            .unwrap_or_else(|| panic!("component_schema has no apply kind: {kind}"))["fields"]
            .clone()
    }

    /// **スキーマとディスパッチの同期を守る要のテスト**(Task#9)。
    /// `component_schema`が載せるkindは`apply_component_impl`の`match kind`の
    /// 写像でしかないので、両者がずれると生成されたフォームが
    /// `UnknownApplyComponentKind`を踏むpayloadを送る。
    ///
    /// ①スキーマ上の全kindがディスパッチに実在すること(未知kindのときだけ
    /// 出る`UnknownApplyComponentKind`が返らないことで見る——空payloadは
    /// index範囲外などの別の`Err`にはなり得るが、それはkindが存在する証拠に
    /// なるので構わない)、②件数が一致すること(ディスパッチ側にだけ足された
    /// kindを検出する)、③kind名に重複が無いこと、を見る。
    /// **止めたまま前後に行き来できる**(`restored_to`のdoc参照)。
    ///
    /// 以前は巻き戻したその場で後ろの記録を捨てていたため、左へ引いて離すと
    /// つまみが右端へ戻り、利用者からは「マウスで動かせない」と見えていた。
    /// 記録を捨てるのは**そこから進めたとき**である。
    #[test]
    fn restoring_a_snapshot_keeps_the_recorded_future_until_stepping_again() {
        // dt = 0.1s なので 1s 間隔 = 10 step ごとに記録される。
        let mut world = WasmWorld::new(-9.80665, 0.1, 50.0);
        for _ in 0..50 {
            world.step();
        }
        let recorded = world.snapshot_count_impl();
        assert!(recorded >= 4, "記録が足りない: {recorded}");

        // 巻き戻しても記録は残る——前後に行き来できる。
        world.restore_snapshot_impl(1).expect("有効なindex");
        assert_eq!(
            world.snapshot_count_impl(),
            recorded,
            "巻き戻しただけでは記録を捨てない"
        );
        let back = world.read_component_impl("time", "").unwrap();
        world
            .restore_snapshot_impl(recorded - 1)
            .expect("有効なindex");
        assert_ne!(
            world.read_component_impl("time", "").unwrap(),
            back,
            "先へも戻れる"
        );

        // 進めた瞬間に、そこから先の記録は実際の未来ではなくなるので捨てる。
        world.restore_snapshot_impl(1).expect("有効なindex");
        world.step();
        assert_eq!(
            world.snapshot_count_impl(),
            2,
            "巻き戻した位置から進めたら、そこから先は新しい時間の筋になる"
        );
    }

    #[test]
    fn component_schema_covers_every_apply_kind() {
        let schema: serde_json::Value = serde_json::from_str(&new_world().component_schema())
            .expect("component_schema must produce valid JSON");
        let entries = schema["apply"].as_array().expect("apply must be an array");

        for entry in entries {
            let kind = entry["kind"].as_str().expect("kind must be a string");
            // **kindごとに新しい`WasmWorld`を使う**——空payloadでの試し打ちは
            // 実際にその操作を実行する(`circuit_editor_reset`ならノード0個の
            // 回路で既存回路を置き換える)ので、1つのworldを使い回すと後続の
            // kindが前のkindの壊した状態を踏む(実際に`circuit_editor_add_
            // dc_motor`が`内部ノードはGND以外`のassertで落ちた)。
            let mut world = new_world();
            let result = world.apply_component_impl(kind, "{}");
            assert!(
                !matches!(result, Err(WasmError::UnknownApplyComponentKind(_))),
                "component_schema lists an apply kind the dispatch does not know: {kind}"
            );
            assert!(
                entry["fields"].is_array(),
                "apply kind {kind} must carry a fields array"
            );
        }

        let mut names: Vec<&str> = entries
            .iter()
            .map(|e| e["kind"].as_str().unwrap())
            .collect();
        names.sort_unstable();
        let unique = names.len();
        names.dedup();
        assert_eq!(names.len(), unique, "apply kind名が重複している");

        // `apply_component_impl`の`match kind`のarm数。**ディスパッチへkindを
        // 足したらこの数と`component_schema`の表の両方を更新すること**——
        // ここが落ちるのは「スキーマに載せ忘れた」ことの検出である。
        const APPLY_KIND_COUNT: usize = 78;
        assert_eq!(
            entries.len(),
            APPLY_KIND_COUNT,
            "apply kindの件数がディスパッチと食い違っている"
        );
    }

    /// 代表的な4系統(Joint・Coupling・スポーン・環境設定)について、
    /// **`_impl`メソッドの本体を手で読んで書き起こしたフィールド一覧と
    /// 完全一致する**ことを見る(Task#9)。名前・型・単位・既定値・
    /// `nullable`・値域まで丸ごと突き合わせるので、引数を1つ足したのに
    /// 表を直し忘れれば必ず落ちる。
    #[test]
    fn component_schema_reports_hand_checked_fields_for_representative_kinds() {
        // ① Joint——`add_distance_joint_impl`。アンカーは剛体ローカル座標[m]、
        // `body_b`だけがi32で、負値が「ワールド固定点」を意味する(`nullable`)。
        assert_eq!(
            apply_fields_of("add_distance_joint"),
            serde_json::json!([
                {"name":"body_a","type":"usize","unit":null,"default":0,"nullable":false,"min":null,"max":null},
                {"name":"ax","type":"f64","unit":"m","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"ay","type":"f64","unit":"m","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"az","type":"f64","unit":"m","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"body_b","type":"i32","unit":null,"default":0,"nullable":true,"min":null,"max":null},
                {"name":"bx","type":"f64","unit":"m","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"by","type":"f64","unit":"m","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"bz","type":"f64","unit":"m","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"length","type":"f64","unit":"m","default":0.0,"nullable":false,"min":null,"max":null},
            ])
        );

        // ② Coupling——`add_convection_link_coupling_impl`。この増分で拾えた
        // 一番きわどい2点が両方入っている: `mode`だけディスパッチの
        // フォールバックが`unwrap_or(3)`(他の`u(...)`の0ではない)であること、
        // `thermal_expansion_coefficient`は`<=0`が「理想気体近似」を意味する
        // センチネル(`nullable`)であること。
        assert_eq!(
            apply_fields_of("add_convection_link_coupling"),
            serde_json::json!([
                {"name":"fluid_node","type":"usize","unit":null,"default":0,"nullable":false,"min":null,"max":null},
                {"name":"surface_node","type":"usize","unit":null,"default":0,"nullable":false,"min":null,"max":null},
                {"name":"area","type":"f64","unit":"m^2","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"characteristic_length","type":"f64","unit":"m","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"mode","type":"usize","unit":null,"default":3,"nullable":false,"min":0.0,"max":3.0},
                {"name":"fluid_thermal_conductivity","type":"f64","unit":"W/(m·K)","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"kinematic_viscosity","type":"f64","unit":"m^2/s","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"prandtl_number","type":"f64","unit":null,"default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"thermal_expansion_coefficient","type":"f64","unit":"1/K","default":0.0,"nullable":true,"min":null,"max":null},
            ])
        );

        // ③ スポーン——`spawn_capsule_impl`。材質は文字列(列挙値を表現できない
        // 既知の限界は`component_schema`モジュールの`s()`のdoc参照)。
        assert_eq!(
            apply_fields_of("spawn_capsule"),
            serde_json::json!([
                {"name":"x","type":"f64","unit":"m","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"y","type":"f64","unit":"m","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"z","type":"f64","unit":"m","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"radius","type":"f64","unit":"m","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"half_height","type":"f64","unit":"m","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"material_name","type":"string","unit":null,"default":"","nullable":false,"min":null,"max":null},
            ])
        );

        // ④ 環境設定——`set_atmosphere_impl`。`viscosity`は`sim_fluid::
        // Atmosphere`と同じ**動粘性係数**[m^2/s](力学的粘性[Pa·s]ではない)。
        assert_eq!(
            apply_fields_of("set_atmosphere"),
            serde_json::json!([
                {"name":"density","type":"f64","unit":"kg/m^3","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"viscosity","type":"f64","unit":"m^2/s","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"wind_x","type":"f64","unit":"m/s","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"wind_y","type":"f64","unit":"m/s","default":0.0,"nullable":false,"min":null,"max":null},
                {"name":"wind_z","type":"f64","unit":"m/s","default":0.0,"nullable":false,"min":null,"max":null},
            ])
        );

        // 引数を取らないkindは空配列(`null`や欠落ではない——フォーム生成側が
        // 「フィールド0個のフォーム」として一様に扱えるように)。
        assert_eq!(apply_fields_of("clear_atmosphere"), serde_json::json!([]));
        assert_eq!(apply_fields_of("spawn_fluid_block"), serde_json::json!([]));

        // 実際に検証される値域だけを載せる方針の確認——`set_dt_impl`は
        // 「正の有限値」を要求して`WasmError::InvalidDt`で弾く。
        assert_eq!(
            apply_fields_of("set_dt"),
            serde_json::json!([
                {"name":"dt","type":"f64","unit":"s","default":0.0,"nullable":false,"min":0.0,"max":null},
            ])
        );
        // 一方`set_body_scale_at_impl`は`scale`を検証しない(軸別版だけが
        // 検証する)ので`min`を載せない。発明した境界を書かない方針の対。
        assert_eq!(
            apply_fields_of("set_body_scale_at")[1]["min"],
            serde_json::Value::Null
        );
        assert_eq!(
            apply_fields_of("set_body_scale_xyz_at")[1]["min"],
            serde_json::json!(0.0)
        );
    }

    /// **`default`は「意味のある既定値」ではなく「省略時に実際に起きること」**
    /// (Task#9、`FieldDefault`のdoc)であることを、スキーマの宣言と
    /// `apply_component_impl`の実挙動を突き合わせて確かめる。
    ///
    /// ここを取り違えるのが一番怖い——`body_b`の「意味のある既定」は
    /// -1(ワールド固定点)だが、**省略すると実際にはボディ0(既定シーンでは床)
    /// へ繋がる**。スキーマが-1を既定と称すると、生成されたフォームが
    /// 「未入力なら宙に固定される」つもりで床に繋ぐ拘束を作ってしまう。
    #[test]
    fn component_schema_defaults_match_what_omitting_the_field_actually_does() {
        // `body_b`——スキーマの既定は0、かつ`nullable`(負値がセンチネル)。
        let body_b_field = apply_fields_of("add_distance_joint")[4].clone();
        assert_eq!(body_b_field["name"], "body_b");
        assert_eq!(body_b_field["default"], serde_json::json!(0));
        assert_eq!(body_b_field["nullable"], serde_json::json!(true));

        // 省略すると宣言どおりボディ0へ繋がる(ワールド固定点にはならない)。
        let mut world = new_world();
        let index = world
            .spawn_sphere_impl(0.0, 5.0, 0.0, 0.3, "鋼(炭素鋼)".to_string())
            .expect("spawn_sphere must succeed");
        world
            .apply_component_impl(
                "add_distance_joint",
                &format!("{{\"body_a\":{index},\"length\":1.0}}"),
            )
            .expect("add_distance_joint must succeed");
        let text = world.joint_info_text_impl(-1);
        assert!(
            text.contains(&format!("body#{index} ↔ body#0")),
            "body_bを省いたらボディ0へ繋がるはず: {text}"
        );

        // センチネル-1を明示したときだけワールド固定点になる。
        world
            .apply_component_impl(
                "add_distance_joint",
                &format!("{{\"body_a\":{index},\"body_b\":-1,\"length\":1.0}}"),
            )
            .expect("add_distance_joint must succeed");
        let text = world.joint_info_text_impl(-1);
        assert!(
            text.contains("ワールド固定点"),
            "body_b=-1はワールド固定点になるはず: {text}"
        );

        // `mode`——スキーマの既定は3(`unwrap_or(3)`)。省略すると
        // `ConvectionMode::ForcedFlatPlate`になることを`describe`経由で見る。
        let mut world = new_world();
        world
            .apply_component_impl("add_thermal_node", "{\"temperature\":300.0}")
            .expect("add_thermal_node must succeed");
        world
            .apply_component_impl("add_thermal_node", "{\"temperature\":300.0}")
            .expect("add_thermal_node must succeed");
        world
            .apply_component_impl(
                "add_convection_link_coupling",
                "{\"fluid_node\":0,\"surface_node\":1,\"area\":1.0}",
            )
            .expect("add_convection_link_coupling must succeed");
        assert!(
            world
                .coupling_info_text_impl(-1)
                .contains("ForcedFlatPlate"),
            "modeを省いたら既定3=ForcedFlatPlateになるはず"
        );
    }

    /// 結合indexを鍵にした`supported_params`の内省(Task#9)。
    /// **副作用のある`set_scalar_param`を叩いて戻り値を見る**という従来の
    /// 唯一の手段を置き換えるものなので、①舵角を持つ`BuoyancyDrag`が
    /// `ControlSurfaceDeflection`を1つだけ挙げること、②持たない結合
    /// (`LorentzForce`)が空配列を返すこと、③範囲外indexは空配列ではなく
    /// `Err`になること(「そんな結合は無い」と「変更可能なパラメータが無い」
    /// を潰さない)を見る。
    #[test]
    fn coupling_supported_params_reports_runtime_adjustable_params_per_coupling() {
        let mut world = new_world();
        let body = world
            .spawn_sphere_impl(0.0, 5.0, 0.0, 0.3, "鋼(炭素鋼)".to_string())
            .expect("spawn_sphere must succeed");

        // index 0: 翼(`LiftModel::Wing`)としての`BuoyancyDrag`。
        world
            .apply_component_impl(
                "add_wing_lift_coupling",
                &format!(
                    "{{\"body\":{body},\"wing_area\":2.0,\"chord_x\":1.0,\"span_z\":1.0,\
                      \"atmosphere_density\":1.225,\"atmosphere_viscosity\":1.5e-5}}"
                ),
            )
            .expect("add_wing_lift_coupling must succeed");
        // index 1: 実行時に変更できるパラメータを持たない結合。
        world
            .apply_component_impl(
                "add_lorentz_force_coupling",
                &format!("{{\"body\":{body},\"charge\":1.0}}"),
            )
            .expect("add_lorentz_force_coupling must succeed");

        assert_eq!(
            world
                .read_component_impl("coupling_supported_params", "0")
                .expect("coupling_supported_params must succeed"),
            "[\"ControlSurfaceDeflection\"]"
        );
        assert_eq!(
            world
                .read_component_impl("coupling_supported_params", "1")
                .expect("coupling_supported_params must succeed"),
            "[]"
        );

        // 範囲外は`Err`(件数まで含めて突き合わせる)。
        assert_err_is(
            world.read_component_impl("coupling_supported_params", "2"),
            WasmError::CouplingIndexOutOfRange { index: 2, count: 2 },
        );

        // `component_schema`の`read`一覧にも載っている(フロントエンドが
        // このkindの存在を動的に知れる)。
        let schema: serde_json::Value = serde_json::from_str(&world.component_schema())
            .expect("component_schema must produce valid JSON");
        assert!(
            schema["read"]
                .as_array()
                .unwrap()
                .iter()
                .any(|v| v == "coupling_supported_params"),
            "read一覧にcoupling_supported_paramsが無い"
        );
    }

    /// `export_scene_json`が`sim_world::to_scenario`経由に置き換わったこと
    /// (**残タスク完遂の縦串④増分**)の回帰テスト——旧実装は`world`/`bodies`
    /// しか書き出さず、既定シーンが持つ2本のprobe(`y_probe`/`speed_probe`、
    /// `WasmWorld::new`のdoc参照)もWheelJointのような結合も一切
    /// エクスポートされなかった(検証タブでprobeが1本も出ない実バグとして
    /// 発覚)。ここではprobesが正しく2本、既定シーンのボディ名(Ground/Box_1)
    /// で書き出され、`Scenario::from_json`で読み戻せることを確認する。
    #[test]
    fn export_scene_json_round_trips_the_default_scenes_two_probes() {
        let world = new_world();
        let json = world
            .export_scene_json_impl()
            .expect("export_scene_json must succeed");
        let parsed: serde_json::Value =
            serde_json::from_str(&json).expect("export_scene_json must produce valid JSON");
        let probes = parsed["probes"]
            .as_array()
            .expect("exported scene must have a probes array");
        assert_eq!(
            probes.len(),
            2,
            "expected the default scene's y_probe/speed_probe, got: {json}"
        );
        // 読み戻せること自体も確認する(往復可能でなければ検証タブの
        // スイープ実行に使えない)。
        let reloaded = sim_world::Scenario::from_json(&json)
            .expect("exported scene JSON must be re-parseable as a Scenario");
        assert_eq!(reloaded.probes.len(), 2);
    }

    /// `spawn_sphere`/`spawn_box`が正しい材質名で成功し、`body_count`が
    /// 増分どおりに伸び、新しいボディのラベル・形状種別・材質が読めること
    /// (Q5でResult化した成功パスの回帰テスト)。
    #[test]
    fn spawn_sphere_and_box_succeed_and_extend_body_count() {
        let mut world = new_world();
        let sphere_index = world
            .spawn_sphere_impl(1.0, 2.0, 3.0, 0.5, "コンクリート".to_string())
            .expect("known material name must succeed");
        assert_eq!(sphere_index, 2);
        assert_eq!(world.body_count_impl(), 3);
        assert_eq!(
            world.body_shape_kind_at_impl(sphere_index).unwrap(),
            "sphere"
        );
        assert_eq!(
            world.body_material_label_at_impl(sphere_index).unwrap(),
            "コンクリート"
        );

        let box_index = world
            .spawn_box_impl(0.0, 0.0, 0.0, 0.25, "鋼(炭素鋼)".to_string())
            .expect("known material name must succeed");
        assert_eq!(box_index, 3);
        assert_eq!(world.body_count_impl(), 4);
        assert_eq!(world.body_shape_kind_at_impl(box_index).unwrap(), "box");
    }

    /// **残タスク完遂の縦串⑤前後で追加**——`spawn_compound_l_shape`/
    /// `spawn_convex_mesh_cube`(Compound/ConvexMeshをUIから作る経路)が
    /// 成功し、`body_shape_kind_at`が正しい種別文字列を返し、
    /// `body_shape_json_at`が有効なシーンJSON形状(`convex_mesh`タグを
    /// 含む)を返すこと。
    #[test]
    fn spawn_compound_and_convex_mesh_succeed_and_are_introspectable() {
        let mut world = new_world();
        let compound_index = world
            .spawn_compound_l_shape_impl(1.0, 2.0, 3.0, "コンクリート".to_string())
            .expect("known material name must succeed");
        assert_eq!(
            world.body_shape_kind_at_impl(compound_index).unwrap(),
            "compound"
        );
        let compound_json = world.body_shape_json_at_impl(compound_index).unwrap();
        assert!(compound_json.starts_with(r#"{"compound":"#));
        let parsed: sim_world::ShapeJson =
            serde_json::from_str(&compound_json).expect("must be a valid ShapeJson");
        assert!(matches!(parsed, sim_world::ShapeJson::Compound { .. }));

        let convex_index = world
            .spawn_convex_mesh_cube_impl(0.0, 0.0, 0.0, 0.3, "鋼(炭素鋼)".to_string())
            .expect("known material name must succeed");
        assert_eq!(
            world.body_shape_kind_at_impl(convex_index).unwrap(),
            "convex_mesh"
        );
        let convex_json = world.body_shape_json_at_impl(convex_index).unwrap();
        assert!(convex_json.starts_with(r#"{"convex_mesh":"#));
        let parsed: sim_world::ShapeJson =
            serde_json::from_str(&convex_json).expect("must be a valid ShapeJson");
        assert!(matches!(parsed, sim_world::ShapeJson::ConvexMesh { .. }));
    }

    /// **Prefabの任意形状対応で追加**——`spawn_shape_json`が
    /// `body_shape_json_at`と**無損失に対になる**こと。
    ///
    /// これがPrefab機能の実体そのものである: ユーザーが組んだボディの形状を
    /// `body_shape_json_at`で読み(キャプチャ)、後から`spawn_shape_json`で
    /// 同じ形状のボディを作る(再スポーン)。**Compound(入れ子)と
    /// ConvexMesh(頂点群)を必ず含める**——固定レシピのスポナーしか無かった
    /// 頃にPrefab化できなかったのがまさにこの2形状であり、平坦なf64配列では
    /// 表現できずに落ちていた情報(子の変換・頂点座標)が本当に往復すること
    /// をここで固定する。
    ///
    /// 比較は**JSON文字列そのもの**で行う。`Shape`は`PartialEq`を持たない
    /// うえ、間に挟まる`shape_json_to_shape`/`shape_to_shape_json`の
    /// どちらか片方だけが情報を落としても`Shape`同士の目視比較では
    /// 気付きにくいためである(文字列一致なら子の順序・座標まで丸ごと縛れる)。
    #[test]
    fn spawn_shape_json_round_trips_every_shape_including_compound_and_convex_mesh() {
        let mut world = new_world();

        // ① キャプチャ元を固定レシピのスポナーで作り、その形状JSONを読む。
        let compound_source = world
            .spawn_compound_l_shape_impl(1.0, 5.0, 0.0, "コンクリート".to_string())
            .expect("known material name must succeed");
        let compound_json = world.body_shape_json_at_impl(compound_source).unwrap();
        let convex_source = world
            .spawn_convex_mesh_cube_impl(2.0, 5.0, 0.0, 0.3, "鋼(炭素鋼)".to_string())
            .expect("known material name must succeed");
        let convex_json = world.body_shape_json_at_impl(convex_source).unwrap();

        // ② 読んだJSONをそのまま渡して再スポーンし、読み直したJSONが
        //    1バイトも違わないこと(=往復で情報が落ちていないこと)。
        let compound_spawned = world
            .spawn_shape_json_impl(&compound_json, -1.0, 5.0, 0.0, "コンクリート")
            .expect("captured compound must respawn");
        assert_eq!(
            world.body_shape_json_at_impl(compound_spawned).unwrap(),
            compound_json
        );
        assert_eq!(
            world.body_shape_kind_at_impl(compound_spawned).unwrap(),
            "compound"
        );
        // ラベルは固定レシピのスポナーと同じ体系(`shape_label_prefix`)。
        assert_eq!(
            world.body_label_at_impl(compound_spawned).unwrap(),
            format!("Compound_{compound_spawned}")
        );
        assert_eq!(
            world.body_material_label_at_impl(compound_spawned).unwrap(),
            "コンクリート"
        );

        let convex_spawned = world
            .spawn_shape_json_impl(&convex_json, -2.0, 5.0, 0.0, "鋼(炭素鋼)")
            .expect("captured convex mesh must respawn");
        assert_eq!(
            world.body_shape_json_at_impl(convex_spawned).unwrap(),
            convex_json
        );
        assert_eq!(
            world.body_shape_kind_at_impl(convex_spawned).unwrap(),
            "convex_mesh"
        );
        assert_eq!(
            world.body_label_at_impl(convex_spawned).unwrap(),
            format!("ConvexMesh_{convex_spawned}")
        );

        // ③ 残る4形状も同じ1つの経路で作れる(固定レシピのスポナーが
        //    受け付けない寸法——非立方体の箱——も通ることまで見る:
        //    `spawn_box`は`half_extent`1つしか取らず立方体しか作れなかった)。
        for (shape_json, expected_kind, expected_prefix) in [
            (r#"{"sphere":{"radius":0.42}}"#, "sphere", "Sphere"),
            (r#"{"box":{"half":[0.1,0.4,0.9]}}"#, "box", "Box"),
            (
                r#"{"capsule":{"radius":0.2,"half_height":0.7}}"#,
                "capsule",
                "Capsule",
            ),
            (
                r#"{"plane":{"normal":[0.0,1.0,0.0],"d":0.0}}"#,
                "plane",
                "Plane",
            ),
        ] {
            let index = world
                .spawn_shape_json_impl(shape_json, 0.0, 3.0, 0.0, "コンクリート")
                .expect("valid ShapeJson must spawn");
            assert_eq!(world.body_shape_kind_at_impl(index).unwrap(), expected_kind);
            assert_eq!(
                world.body_shape_json_at_impl(index).unwrap(),
                shape_json,
                "{expected_kind}の形状JSONが往復していない"
            );
            assert_eq!(
                world.body_label_at_impl(index).unwrap(),
                format!("{expected_prefix}_{index}")
            );
        }

        // ④ 走らせても落ちない(再構築した`Shape`が`World`の積分・接触へ
        //    そのまま乗ること——形状JSONを読めるだけでは足りない)。
        for _ in 0..60 {
            world.step();
        }

        // ⑤ Errパス: 壊れたJSON・`ShapeJson`として解釈できないJSON・未知の
        //    材質名がそれぞれ別の変種で返る(`WasmError`のdoc参照)。
        assert_err_matches!(
            world.spawn_shape_json_impl("{ this is not json", 0.0, 0.0, 0.0, "コンクリート"),
            WasmError::ShapeParseFailed(_)
        );
        assert_err_matches!(
            world.spawn_shape_json_impl(
                r#"{"pyramid":{"height":1.0}}"#,
                0.0,
                0.0,
                0.0,
                "コンクリート"
            ),
            WasmError::ShapeParseFailed(_)
        );
        assert_err_is(
            world.spawn_shape_json_impl(
                r#"{"sphere":{"radius":0.3}}"#,
                0.0,
                0.0,
                0.0,
                "存在しない材質",
            ),
            WasmError::UnknownMaterial("存在しない材質".to_string()),
        );
    }

    /// D1(スケッチ・押し出し)のテスト用: 軸並行な長方形のスケッチ点列。
    fn sketch_rect(x0: f64, z0: f64, x1: f64, z1: f64) -> String {
        format!("[[{x0},{z0}],[{x1},{z0}],[{x1},{z1}],[{x0},{z1}]]")
    }

    /// **D1**: スケッチ→ブーリアン合成→押し出しが、期待どおりの断面積・体積を
    /// 持つ`mesh`タグの形状JSONを返すこと。
    ///
    /// 2m角の正方形から、その右上に1m²だけ重なる正方形を引く(=L字、3 m²)。
    /// 凸包で近似していたら 4 m² になってしまう数値なので、**押し出しが
    /// 凹みを保っている**ことがこの1つの数字で分かる。
    #[test]
    fn sketch_extrude_subtract_produces_a_mesh_shape_with_the_concave_area() {
        let request = format!(
            r#"{{"depth":0.5,"profiles":[
                {{"op":"union","points":{}}},
                {{"op":"subtract","points":{}}}
            ]}}"#,
            sketch_rect(0.0, 0.0, 2.0, 2.0),
            sketch_rect(1.0, 1.0, 3.0, 3.0)
        );
        let result_json = sketch_extrude_shape_json_impl(&request).expect("押し出せる");
        let result: serde_json::Value = serde_json::from_str(&result_json).unwrap();

        assert!(
            (result["profile_area"].as_f64().unwrap() - 3.0).abs() < 1e-9,
            "4-1=3 m²(実際: {})",
            result["profile_area"]
        );
        assert!((result["volume"].as_f64().unwrap() - 1.5).abs() < 1e-9);
        // 地面へちょうど載る高さ = 深さの半分(`extrude_region`が重心を
        // ローカル原点へ寄せる規約と対になっている)。
        assert!((result["rest_height"].as_f64().unwrap() - 0.25).abs() < 1e-12);

        // 形状は新タグ`mesh`で、`ShapeJson`として読み直せる。
        let shape_json = serde_json::to_string(&result["shape"]).unwrap();
        assert!(
            shape_json.starts_with(r#"{"mesh":"#),
            "新しいJSONタグは`mesh`(実際: {})",
            &shape_json[..shape_json.len().min(40)]
        );
        let parsed: sim_world::ShapeJson =
            serde_json::from_str(&shape_json).expect("ShapeJsonとして読める");
        assert!(matches!(parsed, sim_world::ShapeJson::Mesh { .. }));
    }

    /// **D1**: `mesh`形状JSONが`spawn_shape_json`(=シーンJSONと同じ
    /// `shape_json_to_shape`)を通って**実際に物理を持つ剛体になる**こと。
    ///
    /// 1. 質量が「断面積 × 深さ × 密度」に一致する(=`Shape::from_triangle_mesh`
    ///    の近似凸分解が凹みを保った質量特性を返している)。凸包で近似して
    ///    しまうと 4/3 倍に過大評価されるので、この比較は実際に効く。
    /// 2. 床の上に置いて60step走らせても沈まない・落ちない(=接触生成まで
    ///    通っている。`Compound`へ分解された場合も`Compound`のnarrowphaseに
    ///    そのまま乗る)。
    #[test]
    fn mesh_shape_json_spawns_a_rigid_body_with_the_concave_mass_and_rests_on_the_floor() {
        // **`new_world()`ヘルパは使えない**——あちらは`gravity`に負値
        // (-9.80665)を渡しており、`WorldOptions::gravity`は「下向きの大きさ」
        // なので実際には**上向き**の重力になる(既存テストはどれも落下の
        // 向きを見ていないため露見していなかった)。床に載ることを見る
        // このテストだけは正しい向きの重力で世界を作る。
        let mut world = WasmWorld::new(9.80665, 1.0 / 60.0, 5.0);

        // 密度の基準: 体積1 m³ の箱の質量がそのまま密度[kg/m³]になる。
        let reference = world
            .spawn_shape_json_impl(
                r#"{"box":{"half":[0.5,0.5,0.5]}}"#,
                20.0,
                20.0,
                20.0,
                "コンクリート",
            )
            .expect("箱はスポーンできる");
        let density = world.body_mass_at_impl(reference).unwrap();

        let request = format!(
            r#"{{"depth":0.5,"profiles":[
                {{"op":"union","points":{}}},
                {{"op":"subtract","points":{}}}
            ]}}"#,
            sketch_rect(0.0, 0.0, 2.0, 2.0),
            sketch_rect(1.0, 1.0, 3.0, 3.0)
        );
        let result: serde_json::Value =
            serde_json::from_str(&sketch_extrude_shape_json_impl(&request).unwrap()).unwrap();
        let shape_json = serde_json::to_string(&result["shape"]).unwrap();
        let rest_height = result["rest_height"].as_f64().unwrap();

        // **床から0.5m浮かせて落とす**——最初から接地させて置くと「接触が
        // 一度も生成されていないのに、たまたま動かなかっただけ」でも通って
        // しまう。落として静止するところまで見れば接触生成を実際に踏む。
        let drop_height = rest_height + 0.5;
        let index = world
            .spawn_shape_json_impl(&shape_json, -12.0, drop_height, -12.0, "コンクリート")
            .expect("meshタグの形状JSONがスポーンできる");

        // ① 質量。近似凸分解のパーツはわずかに重なりうる(`decompose`の
        //    モジュールdoc)ので5%の余裕を持たせるが、凸包近似の 2.0 m³
        //    (=+33%)とは明確に区別できる幅にしてある。
        let mass = world.body_mass_at_impl(index).unwrap();
        let expected = 1.5 * density;
        assert!(
            (mass / expected - 1.0).abs() < 0.05,
            "L字断面(3 m²)×深さ0.5m = 1.5 m³ 相当の質量のはず\
             (期待 {expected:.1} kg、実際 {mass:.1} kg、凸包近似なら {:.1} kg)",
            2.0 * density
        );

        // ② 床の上で静止する。1step目で崩れる形状(接触が生成されず
        //    すり抜ける、逆に沈む)なら、この高さ比較で必ず落ちる。
        for _ in 0..180 {
            world.step();
        }
        let y = world.body_position_at_f64_impl(index).unwrap()[1];
        assert!(
            (y - rest_height).abs() < 0.05,
            "0.5m落として床にちょうど載るはず(期待 {rest_height:.3} m、実際 {y:.3} m)"
        );
    }

    /// **D1**: 穴の空く減算(内側にすっぽり入る四角形を引く)は、`mesh`を子に
    /// 持つ`compound`として返り、スポーンすると**穴のぶんだけ軽い**剛体になる。
    ///
    /// 穴あき断面を1つのメッシュとして渡すと近似凸分解が働かず穴が塞がる
    /// (`sim_mechanics::sketch::split_into_hole_free_regions`のdoc)。ここは
    /// その回避がwasm境界を越えて効いていることの確認である。
    #[test]
    fn sketch_extrude_with_a_hole_returns_a_compound_of_mesh_parts() {
        let mut world = WasmWorld::new(9.80665, 1.0 / 60.0, 5.0);
        let reference = world
            .spawn_shape_json_impl(
                r#"{"box":{"half":[0.5,0.5,0.5]}}"#,
                30.0,
                30.0,
                30.0,
                "コンクリート",
            )
            .unwrap();
        let density = world.body_mass_at_impl(reference).unwrap();

        // 外形 4×4 m² から中央の 1×1 m² を引く → 15 m²、深さ0.2m → 3 m³。
        let request = format!(
            r#"{{"depth":0.2,"profiles":[
                {{"points":{}}},
                {{"op":"subtract","points":{}}}
            ]}}"#,
            sketch_rect(0.0, 0.0, 4.0, 4.0),
            sketch_rect(1.5, 1.5, 2.5, 2.5)
        );
        let result: serde_json::Value =
            serde_json::from_str(&sketch_extrude_shape_json_impl(&request).unwrap()).unwrap();
        assert!(
            (result["profile_area"].as_f64().unwrap() - 15.0).abs() < 1e-9,
            "16-1=15 m²"
        );
        let children = result["shape"]["compound"]["children"]
            .as_array()
            .expect("穴あき断面は compound で返る");
        assert!(children.len() >= 2, "穴を通る線で切り分けられている");
        assert!(
            children.iter().all(|c| c["shape"]["mesh"].is_object()),
            "子はいずれも`mesh`タグ(=それぞれが近似凸分解を通る)"
        );

        let index = world
            .spawn_shape_json_impl(
                &serde_json::to_string(&result["shape"]).unwrap(),
                -20.0,
                result["rest_height"].as_f64().unwrap(),
                -20.0,
                "コンクリート",
            )
            .expect("compound of mesh がスポーンできる");
        let mass = world.body_mass_at_impl(index).unwrap();
        assert!(
            (mass / (3.0 * density) - 1.0).abs() < 0.02,
            "15 m² × 0.2 m = 3 m³ 相当の質量のはず(穴が塞がると 3.2 m³ 相当。\
             期待 {:.1} kg、実際 {mass:.1} kg)",
            3.0 * density
        );
    }

    /// **D1**: スケッチ押し出しのErrパス。いずれもユーザーの操作から素直に
    /// 踏めるので、`WasmError`の変種まで固定しておく。
    #[test]
    fn sketch_extrude_reports_distinct_errors_for_each_bad_input() {
        assert_err_matches!(
            sketch_extrude_shape_json_impl("{ this is not json"),
            WasmError::SketchRequestParseFailed(_)
        );
        // 閉じたスケッチが1枚も無い(点2つは多角形にならない)。
        assert_err_is(
            sketch_extrude_shape_json_impl(
                r#"{"depth":0.5,"profiles":[{"points":[[0,0],[1,0]]}]}"#,
            ),
            WasmError::SketchProfileEmpty,
        );
        // 重なっていない2枚の積 → 断面が空になる。
        let disjoint = format!(
            r#"{{"depth":0.5,"profiles":[{{"points":{}}},{{"op":"intersect","points":{}}}]}}"#,
            sketch_rect(0.0, 0.0, 1.0, 1.0),
            sketch_rect(5.0, 5.0, 6.0, 6.0)
        );
        assert_err_is(
            sketch_extrude_shape_json_impl(&disjoint),
            WasmError::SketchProfileEmpty,
        );
        // 未知の演算名。
        let unknown = format!(
            r#"{{"depth":0.5,"profiles":[{{"points":{}}},{{"op":"xor","points":{}}}]}}"#,
            sketch_rect(0.0, 0.0, 1.0, 1.0),
            sketch_rect(0.5, 0.5, 1.5, 1.5)
        );
        assert_err_is(
            sketch_extrude_shape_json_impl(&unknown),
            WasmError::UnknownBooleanOp("xor".to_string()),
        );
        // 深さ0(押し出す厚みが無い)。
        let flat = format!(
            r#"{{"depth":0.0,"profiles":[{{"points":{}}}]}}"#,
            sketch_rect(0.0, 0.0, 1.0, 1.0)
        );
        assert_err_is(
            sketch_extrude_shape_json_impl(&flat),
            WasmError::SketchExtrudeFailed,
        );
    }

    /// フレーム階層: ROOTの子フレームを追加でき、親indexが正しく読めること。
    #[test]
    fn add_child_frame_succeeds_and_reports_correct_parent() {
        let mut world = new_world();
        assert_eq!(world.frame_count_impl(), 1); // ROOTのみ。
        let child = world
            .add_child_frame_impl(0, 1.0, 0.0, 0.0, 0.5)
            .expect("ROOT is always a valid parent");
        assert_eq!(child, 1);
        assert_eq!(world.frame_count_impl(), 2);
        assert_eq!(world.frame_parent_index_impl(child).unwrap(), 0);
        assert_eq!(world.frame_parent_index_impl(0).unwrap(), -1); // ROOTは親を持たない。
    }

    /// ブックマークの追加・ラベル/時刻の読み取り・巻き戻しが成功パスで
    /// 期待どおり動くこと。
    #[test]
    fn bookmark_add_and_restore_round_trips_successfully() {
        let mut world = new_world();
        world.step();
        let time_at_bookmark = world.time_impl();
        world.add_bookmark_impl("test-bookmark".to_string());
        assert_eq!(world.bookmark_count_impl(), 1);
        assert_eq!(world.bookmark_label_at_impl(0).unwrap(), "test-bookmark");
        assert!((world.bookmark_time_at_impl(0).unwrap() - time_at_bookmark).abs() < 1e-12);

        world.step();
        world.step();
        assert!(world.time_impl() > time_at_bookmark);

        world
            .restore_bookmark_impl(0)
            .expect("bookmark 0 must exist");
        assert!((world.time_impl() - time_at_bookmark).abs() < 1e-12);
    }

    /// `＋ 追加 → 流体` で足した水塊が容器に受け止められること(QA不具合2)。
    ///
    /// 以前は境界が「1層の床」だけで壁が無く、着水した水は横へ薄く広がって
    /// 膜になり、カーネル欠損で圧力が 0 にクランプされて支えを失い、
    /// **床を抜けて落ち続けた**(実測: 2 秒で 27 粒子中 20 個が床より 0.5 m 下)。
    /// 床を3層+側壁4面の容器にして、Δx = h/2 の規約も満たすようにした。
    #[test]
    fn a_spawned_fluid_block_is_caught_by_its_container_instead_of_leaking_through() {
        let mut world = new_world();
        world.spawn_fluid_block_impl();
        let sph = world.inner.sph().expect("SPHドメインが有効になる");
        assert_eq!(sph.position.len(), 27, "水塊は 3×3×3 粒子");
        // Δx = h/2(設計§4.1・§9)。
        let dx = (sph.mass / sph.rho0).cbrt();
        assert!(
            (sph.h - 2.0 * dx).abs() < 1e-12,
            "格子間隔は h/2 であるべき: h={} dx={dx}",
            sph.h
        );
        // 床は3層(最上層 y=-dx、以下 -2dx, -3dx)。
        let floor_y = -dx;
        let deepest_boundary = sph
            .boundary_position
            .iter()
            .map(|p| p.y)
            .fold(f64::MAX, f64::min);
        assert!(
            (deepest_boundary - (floor_y - 2.0 * dx)).abs() < 1e-9,
            "床は3層であるべき: 最下層 y={deepest_boundary}"
        );
        // 側壁があること(床より上に境界粒子が居る)。
        assert!(
            sph.boundary_position
                .iter()
                .any(|p| p.y > floor_y + 0.5 * dx),
            "容器の側壁が無い(床だけだと水が横へ広がって膜になり漏れる)"
        );

        // 2 秒ぶん進めて、1 粒子も容器を抜けないこと。
        for _ in 0..240 {
            world.inner.step();
        }
        let sph = world.inner.sph().expect("SPHドメインが有効");
        let escaped: Vec<f64> = sph
            .position
            .iter()
            .map(|p| p.y)
            .filter(|y| *y < floor_y - 2.0 * dx)
            .collect();
        assert!(
            escaped.is_empty(),
            "{} / {} 個が容器の底(y={})を抜けた: {escaped:?}",
            escaped.len(),
            sph.position.len(),
            floor_y - 2.0 * dx,
        );
    }

    /// 上のテストは**ある1つの初期条件**で漏れないことしか見ていない。ところが
    /// このシーンで実際に起きた失敗は「Linux と macOS では緑、Windows だけ赤」
    /// というプラットフォーム依存だった。原因は力学の step 経路にある
    /// `sin`/`cos`/`exp`/`atan2` で、Rust std はこれらを各OSの libm へ委譲し、
    /// IEEE-754 はこれらに正確な丸めを要求していないため 1 ULP の差が正当に出る。
    /// その差が数百ステップで増幅し、着水の飛沫が容器の縁を「越えるか越えないか」
    /// を分けていた。**単一の初期条件を回すテストでは原理的に検出できない**。
    ///
    /// そこで初期位置へ微小な相対摂動を与え、**どの初期条件でも容器から粒子が
    /// 失われない**という、余裕そのものを見る。摂動 1e-6 は倍精度の丸め差より
    /// はるかに大きいので、OS差の上界として機能する。
    ///
    /// この判定は較正済みで、壁が低かった頃(`wall_top = n + layers`)は摂動 1e-6
    /// で48回中28回落ち、飛沫を覆う高さ(`3n + layers`)にしてからは48回中0回になる
    /// ——つまり本テストは旧実装をきちんと落とす。
    #[test]
    fn spawned_fluid_block_stays_in_its_container_under_tiny_perturbations() {
        let dx: f64 = 0.1;
        let floor_y = -dx;
        let escape_y = floor_y - 2.0 * dx;
        // 相対摂動の幅。倍精度の丸め(~1e-16)よりはるかに大きく取る。
        let perturb = 1e-6;

        for seed in 1..=6u64 {
            let mut world = new_world();
            world.spawn_fluid_block_impl();

            // 決定論的な xorshift で ±perturb の相対摂動を与える(壁時計非依存)。
            let mut state = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
            let mut next = move || {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                (state >> 11) as f64 / (1u64 << 52) as f64 - 1.0
            };
            {
                let sph = world.inner.sph_mut().expect("SPHドメインが有効");
                for p in sph.position.iter_mut() {
                    *p = Vec3::new(
                        p.x * (1.0 + perturb * next()),
                        p.y * (1.0 + perturb * next()),
                        p.z + perturb * next(),
                    );
                }
            }

            for _ in 0..240 {
                world.inner.step();
            }

            let sph = world.inner.sph().expect("SPHドメインが有効");
            let escaped: Vec<f64> = sph
                .position
                .iter()
                .map(|p| p.y)
                .filter(|y| *y < escape_y)
                .collect();
            assert!(
                escaped.is_empty(),
                "seed={seed}: {} / {} 個が容器(底 y={escape_y})から失われた: {escaped:?}",
                escaped.len(),
                sph.position.len(),
            );
        }
    }

    /// Import が捨てたセクションを申告すること(QA不具合5)。
    ///
    /// D10(摩擦の熱)は`thermal`と`couplings`を持つが、Import は`bodies`と
    /// `probes`しか取り込まない。**黙って落とすのをやめる**のがこの修正の
    /// 目的なので、「落としたものが名前で分かる」ことを直接押さえる。
    #[test]
    fn import_scene_json_reports_the_sections_it_did_not_apply() {
        let mut world = new_world();
        let json = include_str!("../../../scenes/d10-brake-heat.json");
        let before = world.body_count_impl();
        let added = world
            .import_scene_json_impl(json)
            .expect("D10 の JSON は Import できる");

        // ボディは実際に増える(ここは従来どおり動いている)。
        assert_eq!(added, 2);
        assert_eq!(world.body_count_impl(), before + 2);

        // `thermal`と`couplings`はどちらも D10 に書かれているが取り込まれない。
        let skipped = world.last_import_skipped_sections_impl().to_vec();
        assert!(
            skipped.contains(&"thermal".to_string()) && skipped.contains(&"couplings".to_string()),
            "D10 の thermal / couplings が申告されるべき: {skipped:?}"
        );
        // 実際に結合は入っていない(申告の内容が事実と一致していること)。
        assert_eq!(world.coupling_count_impl(), 0);

        // `read_component`経由(UI が読む経路)でも JSON 配列で出る。
        let text = world
            .read_component_impl("last_import_skipped_sections", "")
            .expect("last_import_skipped_sections via read_component must succeed");
        let parsed: Vec<String> = serde_json::from_str(&text).expect("JSON 文字列配列であること");
        assert_eq!(parsed, skipped);

        // 書かれていないセクションは申告しない(D10 に回路も流体も無い)。
        assert!(
            !skipped.contains(&"circuit".to_string()) && !skipped.contains(&"fluids".to_string()),
            "書かれていないセクションを申告してはいけない: {skipped:?}"
        );
    }

    /// `body_position_at_f64`が f32 では表現できない刻みを保つこと(QA不具合6)。
    ///
    /// D25(ブラウン運動)の粒子列の端は x = 0.299 m にあり、そこでの f32 の
    /// 刻みは約 3.0×10⁻⁸ m。ブラウン変位は 2×10⁻⁹ m オーダーなので、f32 では
    /// **変位そのものが 0 に丸められる**。同じ座標を f64 で読めば残ることを
    /// 「0.299 m に 2 nm を足して読み戻す」形で確かめる——f32 経路では
    /// 足す前と足した後が同じ値になり、f64 経路では違う値になる。
    #[test]
    fn body_position_at_f64_resolves_displacements_that_f32_quantizes_away() {
        let mut world = new_world();
        let base_x = 0.299; // D25 の粒子列の端。
        let nanometers = 2.0e-9; // D25 のブラウン変位のオーダー。
        world
            .set_body_position_at_impl(1, base_x, 0.0, 0.0)
            .unwrap();
        let f32_before = world.body_position_at_impl(1).unwrap()[0];
        let f64_before = world.body_position_at_f64_impl(1).unwrap()[0];

        world
            .set_body_position_at_impl(1, base_x + nanometers, 0.0, 0.0)
            .unwrap();
        let f32_after = world.body_position_at_impl(1).unwrap()[0];
        let f64_after = world.body_position_at_f64_impl(1).unwrap()[0];

        // f32 では 2 nm の差が消える(これが MSD が解析解の 4.4 倍になった原因)。
        assert_eq!(
            f32_before, f32_after,
            "f32 経路は 2 nm の変位を量子化で失うはず(この前提が崩れたら\
             不具合6の説明ごと見直す): before={f32_before} after={f32_after}"
        );
        // f64 なら残る(0.299 における f64 の刻みは約 5.5×10⁻¹⁷ なので、
        // 差そのものの相対誤差で見る)。
        let recovered = (f64_after - f64_before) / nanometers;
        assert!(
            (recovered - 1.0).abs() < 1e-6,
            "f64 経路は 2 nm の変位を保つべき: before={f64_before} after={f64_after} \
             (復元した変位の比 {recovered})"
        );

        // `read_component`経由(UI が実際に使う経路)でも同じ値が JSON 配列で出る。
        let text = world
            .read_component_impl("body_position_at_f64", "1")
            .expect("body_position_at_f64 via read_component must succeed");
        let parsed: [f64; 3] = serde_json::from_str(&text).expect("JSON 配列であること");
        assert_eq!(parsed[0], f64_after);

        // Errパス: 他の body 系と同じく`try_body_id_at`を通る。
        let count = world.body_count_impl();
        assert_eq!(
            world.body_position_at_f64_impl(count),
            Err(WasmError::BodyIndexOutOfRange {
                index: count,
                count
            })
        );
    }

    /// 位置/姿勢の直接編集(Gizmo相当)が成功パスで正しく反映され、範囲外
    /// indexでは`Err`になること。**書き込んだ値の読み戻しに`Float32Array`は
    /// 要らない**——index検証と値の取り出しを担う`body_position_at_impl`/
    /// `body_rotation_at_impl`は素のRust配列を返すため(モジュール冒頭の
    /// テストdoc「今も残る制約」参照)、ここで実際に往復を確かめられる。
    #[test]
    fn set_body_position_and_rotation_succeed_for_a_valid_body() {
        let mut world = new_world();
        world.set_body_position_at_impl(1, 7.0, 8.0, 9.0).unwrap();
        world
            .set_body_rotation_at_impl(1, 0.0, 0.0, 0.0, 1.0)
            .unwrap();
        assert_eq!(world.body_position_at_impl(1).unwrap(), [7.0, 8.0, 9.0]);
        assert_eq!(
            world.body_rotation_at_impl(1).unwrap(),
            [0.0, 0.0, 0.0, 1.0]
        );
        let _ = world.body_velocity_at_impl(1).unwrap();

        // Errパス: 読み書きどちらも`try_body_id_at`を通る。
        let count = world.body_count_impl();
        let out_of_range = WasmError::BodyIndexOutOfRange {
            index: count,
            count,
        };
        assert_err_is(
            world.set_body_position_at_impl(count, 7.0, 8.0, 9.0),
            out_of_range.clone(),
        );
        assert_err_is(
            world.set_body_rotation_at_impl(count, 0.0, 0.0, 0.0, 1.0),
            out_of_range.clone(),
        );
        assert_err_is(world.body_position_at_impl(count), out_of_range.clone());
        assert_err_is(world.body_velocity_at_impl(count), out_of_range.clone());
        assert_err_is(world.body_rotation_at_impl(count), out_of_range);
    }

    /// Scale Gizmo(`set_body_scale_at`/`set_body_scale_xyz_at`)がスポーンした
    /// 球のスケールを成功パスで受理し、**床(index 0)と不正なスケール成分を
    /// 拒否する**こと。後者2つは`WasmError`導入で初めて検証できるようになった。
    #[test]
    fn set_body_scale_at_succeeds_for_a_spawned_body() {
        let mut world = new_world();
        let sphere_index = world
            .spawn_sphere_impl(0.0, 0.0, 0.0, 0.5, "コンクリート".to_string())
            .expect("known material name must succeed");
        world.set_body_scale_at_impl(sphere_index, 2.0).unwrap();
        assert_eq!(
            world.body_shape_kind_at_impl(sphere_index).unwrap(),
            "sphere"
        );
        // 球に軸別スケールは効かない(`false`を返すが`Err`ではない、
        // `set_body_scale_xyz_at_impl`のdoc参照)——「効かない」と「失敗」を
        // 取り違えていないことの確認。
        assert!(!world
            .set_body_scale_xyz_at_impl(sphere_index, 1.0, 2.0, 3.0)
            .unwrap());

        // 床(index 0)にはスケールハンドルが無い(両メソッドとも同じ変種)。
        assert_err_is(
            world.set_body_scale_at_impl(0, 2.0),
            WasmError::GroundHasNoScaleHandle,
        );
        assert_err_is(
            world.set_body_scale_xyz_at_impl(0, 2.0, 2.0, 2.0),
            WasmError::GroundHasNoScaleHandle,
        );
        // 軸別スケールは各成分が正の有限値であることを要求する。
        for (sx, sy, sz) in [
            (0.0, 1.0, 1.0),
            (1.0, -1.0, 1.0),
            (1.0, 1.0, f64::NAN),
            (f64::INFINITY, 1.0, 1.0),
        ] {
            assert_err_is(
                world.set_body_scale_xyz_at_impl(sphere_index, sx, sy, sz),
                WasmError::InvalidScaleComponent,
            );
        }
        // 範囲外index(スケール判定より手前の`try_body_id_at`で弾かれる)。
        let count = world.body_count_impl();
        assert_err_is(
            world.set_body_scale_at_impl(count, 2.0),
            WasmError::BodyIndexOutOfRange {
                index: count,
                count,
            },
        );
        assert_err_matches!(
            world.set_body_scale_xyz_at_impl(count, 2.0, 2.0, 2.0),
            WasmError::BodyIndexOutOfRange { .. }
        );
    }

    /// **残タスク完遂のシーンギャラリー増分**: `WasmWorld::from_scene_json`が
    /// シーンギャラリーの実アセット(`scenes/d4-box-stack.json`、4体——地面+
    /// 3段の箱)を正しく読み込み、`step()`を回して静止させられることを確認する
    /// (`sim-world::scenario`側の`run_headless_scenario_settles_a_stacked_box_
    /// tower_matching_d4_pass_criterion`と同じシーンJSONファイルを読む——
    /// アセットが壊れれば両方のテストが連動してRedになる)。
    #[test]
    fn from_scene_json_loads_the_d4_box_stack_gallery_asset_and_settles() {
        let json = include_str!("../../../scenes/d4-box-stack.json");
        let mut world = WasmWorld::from_scene_json_impl(json)
            .expect("scenes/d4-box-stack.json must be a valid scene");
        assert_eq!(world.body_count_impl(), 4); // 地面+3段の箱。
                                                // 地面(JSON側に"name"フィールドが無い)は`format!("Body_{index}")`の
                                                // フォールバックラベルになる。
        assert_eq!(world.body_label_at_impl(0).unwrap(), "Body_0");
        for _ in 0..1200 {
            world.step();
        }
        for index in 1..4 {
            let label = world.body_label_at_impl(index).unwrap();
            assert!(
                label.starts_with("box"),
                "body {index} should be named box1/box2/box3, got {label:?}"
            );
        }
    }

    /// 範囲外の`body_index`で`push_apply_force`/`push_grab`/`push_move_grab`/
    /// `push_release`等を呼ぶと`Err`を返すこと(**Q5と同じ理由でResult化した
    /// 対象**、シーンギャラリーで任意のシーンを読み込んだ後にNudge/Grab UIが
    /// 古い`body_index`を渡しても`panic!`しないことの検証)。
    ///
    /// **この`Err`検証こそがQ5の主張そのものだったが、`JsValue`を構築した
    /// 時点でネイティブテストがabortするため長らく書けず、成功パスだけを
    /// 確認していた**(モジュール冒頭のテストdoc参照)。`WasmError`導入で
    /// 本来検証したかった側をようやく書けるようになった。
    #[test]
    fn push_commands_accept_an_explicit_body_index_for_a_valid_body() {
        let mut world = new_world();
        world.push_apply_force_impl(1, 0.0, 1.0, 0.0).unwrap();
        world.push_grab_impl(1, 0.0, 1.0, 0.0).unwrap();
        world.push_move_grab_impl(1, 0.0, 1.0, 0.0).unwrap();
        world.push_release_impl(1).unwrap();
        world.push_set_body_mass_impl(1, 2.5).unwrap();
        world
            .push_set_body_type_impl(1, "Static".to_string())
            .unwrap();
        world.push_set_collision_filter_impl(1, 0b10, 0b01).unwrap();

        // 範囲外index——Command系7経路すべてが`try_body_id_at`を通る。
        let count = world.body_count_impl();
        let out_of_range = WasmError::BodyIndexOutOfRange {
            index: count,
            count,
        };
        assert_err_is(
            world.push_apply_force_impl(count, 0.0, 1.0, 0.0),
            out_of_range.clone(),
        );
        assert_err_is(
            world.push_grab_impl(count, 0.0, 1.0, 0.0),
            out_of_range.clone(),
        );
        assert_err_is(
            world.push_move_grab_impl(count, 0.0, 1.0, 0.0),
            out_of_range.clone(),
        );
        assert_err_is(world.push_release_impl(count), out_of_range.clone());
        assert_err_is(
            world.push_set_body_mass_impl(count, 2.5),
            out_of_range.clone(),
        );
        assert_err_is(
            world.push_set_body_type_impl(count, "Static".to_string()),
            out_of_range.clone(),
        );
        assert_err_is(
            world.push_set_collision_filter_impl(count, 0b10, 0b01),
            out_of_range.clone(),
        );

        // 内省系4つも同じ経路。
        assert_err_is(world.body_mass_at_impl(count), out_of_range.clone());
        assert_err_is(world.body_type_at_impl(count), out_of_range.clone());
        assert_err_is(
            world.body_collision_group_at_impl(count),
            out_of_range.clone(),
        );
        assert_err_is(world.body_collision_mask_at_impl(count), out_of_range);

        // **質量の検証はindex検証より手前**(`push_set_body_mass_impl`は
        // 質量を先に見る)——範囲外indexかつ不正質量なら`InvalidMass`が勝つ。
        for bad_mass in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert_err_is(
                world.push_set_body_mass_impl(1, bad_mass),
                WasmError::InvalidMass,
            );
            assert_err_is(
                world.push_set_body_mass_impl(count, bad_mass),
                WasmError::InvalidMass,
            );
        }

        // Dynamic/Static/Kinematic以外のbody type名。メッセージには`{:?}`
        // (引用符付き)で名前が載る規約なので、変種にも生の名前を保持する。
        assert_err_is(
            world.push_set_body_type_impl(1, "Rigid".to_string()),
            WasmError::UnknownBodyType("Rigid".to_string()),
        );
        assert_err_is(
            world.push_set_body_type_impl(1, String::new()),
            WasmError::UnknownBodyType(String::new()),
        );
        // 大文字小文字は区別する(UIの選択肢と1対1で対応させる規約)。
        assert_err_is(
            world.push_set_body_type_impl(1, "static".to_string()),
            WasmError::UnknownBodyType("static".to_string()),
        );

        // 拘束アンカーの内省(拘束を持たないボディは`None`、範囲外は
        // `try_body_meta_at`側の変種)。
        assert_eq!(world.constraint_anchor_points_impl(1).unwrap(), None);
        assert_err_is(
            world.constraint_anchor_points_impl(count),
            WasmError::BodyMetaIndexOutOfRange { index: count },
        );
    }

    /// **増分B1(シーン定義プローブをProbe Graphsパネルへ配線)**: 既定シーン
    /// (`WasmWorld::new`)は`scenario.probes`を持たない(`imported_probe_handles`は
    /// `Vec::new()`のまま、`new()`のdoc参照)ため、`imported_probe_count()`は
    /// 常に0を返すこと。フロントエンド側(`demo/main.ts`)が「プローブ0本の
    /// シーンでも壊れない」ことを要求するケースの最も単純な実例でもある。
    #[test]
    fn imported_probe_count_is_zero_for_the_default_scene() {
        let world = new_world();
        assert_eq!(world.imported_probe_count_impl(), 0);
    }

    /// **プローブ履歴が固定容量を超えても切り詰められないこと**を、wasm境界の
    /// 内省経路(`read_component`)越しに確認する(`sim_world::Probe`のdoc参照)。
    ///
    /// 旧・固定容量は6000だった。既定シーンは2本のプローブを持つので、
    /// 8000step走らせれば両系列とも旧容量を超える。
    #[test]
    fn probe_history_grows_past_the_old_fixed_capacity_and_reports_its_size() {
        const OLD_FIXED_CAPACITY: usize = 6000;
        const STEPS: usize = 8000;
        const _: () = assert!(STEPS > OLD_FIXED_CAPACITY);

        let mut world = new_world();
        assert_eq!(
            world
                .read_component_impl("probe_history_bytes_estimate", "")
                .unwrap(),
            "0"
        );
        for _ in 0..STEPS {
            world.step();
        }
        // 既定シーンのプローブ2本 × STEPSサンプル × f64。
        assert_eq!(
            world
                .read_component_impl("probe_history_bytes_estimate", "")
                .unwrap(),
            (2 * STEPS * std::mem::size_of::<f64>()).to_string()
        );

        // `component_schema`が新しい2つの内省kindを申告していること
        // (フロントエンドはこの一覧を見て何を読めるか決める)。
        let schema = world.component_schema();
        assert!(schema.contains("probe_history_bytes_estimate"));
        assert!(schema.contains("imported_probe_history_len"));
    }

    /// `imported_probe_history_len`が系列ごとの内訳を返すこと
    /// (`probe_history_bytes_estimate`の合計の内訳)。
    #[test]
    fn imported_probe_history_len_reports_each_series_length() {
        let json = include_str!("../../../scenes/d6-floating-box-f4.json");
        let mut world = WasmWorld::from_scene_json_impl(json)
            .expect("scenes/d6-floating-box-f4.json must be a valid scene");
        for _ in 0..7000 {
            world.step();
        }
        assert_eq!(
            world
                .read_component_impl("imported_probe_history_len", "0")
                .unwrap(),
            "7000"
        );
        // 範囲外のindexはエラー(黙って0を返さない)。
        assert!(world
            .read_component_impl("imported_probe_history_len", "9")
            .is_err());
    }

    /// **エディタでシーンを保存しても合格基準が消えないこと**
    /// (`sim_world::Scenario::pass_criteria`のdoc参照)。
    ///
    /// これが移行前の実害そのものである: 手で`pass_criteria`を書いたシーンを
    /// 読み込み、`export_scene_json`(=エディタの保存)で書き戻すと、
    /// 検証タブの合格基準が丸ごと落ちていた——`from_scenario`が読まず
    /// `to_scenario`が常に空を返していたため。
    #[test]
    fn export_scene_json_keeps_the_author_written_pass_criteria_and_prompts() {
        let json = r#"{
            "name": "authored",
            "world": { "gravity": 9.80665, "dt": 0.008333333 },
            "bodies": [
                { "name": "ball", "shape": { "sphere": { "radius": 0.2 } },
                  "material": "鋼(炭素鋼)", "position": [0, 5, 0] }
            ],
            "probes": [ { "body_pos_y": "ball" } ],
            "prediction_prompts": [
                { "question": "落下1秒後の高さは?", "probe_index": 0,
                  "expected_value": 0.0967 }
            ],
            "pass_criteria": [
                { "probe_index": 0, "operator": "le", "threshold": 0.25 }
            ]
        }"#;
        let mut world =
            WasmWorld::from_scene_json_impl(json).expect("authored scene must be valid");
        for _ in 0..60 {
            world.step();
        }
        let exported = world
            .export_scene_json_impl()
            .expect("export_scene_json must succeed");
        let parsed: serde_json::Value =
            serde_json::from_str(&exported).expect("must produce valid JSON");

        let criteria = parsed["pass_criteria"]
            .as_array()
            .expect("pass_criteria must survive the editor save path");
        assert_eq!(criteria.len(), 1);
        assert_eq!(criteria[0]["probe_index"], 0);
        assert_eq!(criteria[0]["operator"], "le");
        assert_eq!(criteria[0]["threshold"], 0.25);

        let prompts = parsed["prediction_prompts"]
            .as_array()
            .expect("prediction_prompts must survive the editor save path");
        assert_eq!(prompts.len(), 1);
        assert_eq!(prompts[0]["question"], "落下1秒後の高さは?");
    }

    /// D6(浮き沈み、`scenes/d6-floating-box-f4.json`)は`probes`に
    /// `{ "body_pos_y": "box" }`を1本だけ持つ。`imported_probe_count`/
    /// `imported_probe_label_at`がこれを正しく反映し、ラベルにボディ名"box"が
    /// 含まれること(`probe_target_label`のdoc参照)。
    #[test]
    fn imported_probe_label_reflects_the_d6_floating_box_scene_probe() {
        let json = include_str!("../../../scenes/d6-floating-box-f4.json");
        let world = WasmWorld::from_scene_json_impl(json)
            .expect("scenes/d6-floating-box-f4.json must be a valid scene");
        assert_eq!(world.imported_probe_count_impl(), 1);
        assert_eq!(
            world.imported_probe_label_at_impl(0).unwrap(),
            "BodyPosY(box)"
        );
    }

    /// **「JS側から見た挙動は一切変わらない」ことの固定テスト**
    /// (`WasmError`のdoc参照)。`Display`が出す文字列は、リファクタ前に
    /// `JsValue::from_str`へ渡していたメッセージと1バイトも違ってはならない
    /// ——フロントエンドはこの文字列をそのままConsoleパネルへ出しており、
    /// 内部リファクタで表示が変わるのは本改修の前提に反する。
    ///
    /// **期待値はリテラルで直書きする**(`Display`と同じ`format!`式を書き直す
    /// のではなく)。同じ式を両側に書いても「同じ式は同じ文字列を作る」しか
    /// 言えず、文面が変わってしまったことを検出できないため。
    ///
    /// **本テストが全変種を舐めることの副次的な意味(正直な記録)**: 39変種の
    /// うち5つは、実際に踏ませる呼び出しをネイティブテストから書けない——
    /// `ShapeSerializeFailed`/`ScenarioSerializeFailed`/
    /// `HeadlessResultSerializeFailed`は`serde_json::to_string`の失敗だが、
    /// 対象の型(`ShapeJson`/`Scenario`/内部の結果構造体)はいずれも
    /// 「mapのキーが文字列でない」等の失敗要因を持たず事実上infallibleであり、
    /// `ImportedProbeHandleMissing`は`World`側にprobe削除経路が無いため
    /// 到達しない(その変種のdoc参照)。これら5つについては、本テストの
    /// 文面固定が唯一の回帰防御になる。
    ///
    /// `ScenarioExport`(**状態スナップショット移行で追加**)も実際に踏ませるには
    /// 発散した`World`が要るので、ここでは文面だけを固定する。
    #[test]
    fn display_reproduces_the_exact_messages_that_used_to_reach_jsvalue() {
        use sim_world::SceneError;

        // シーンJSON経路は`SceneError`のDebug表現をそのまま出す
        // (旧`format!("{e:?}")`と同じ)。5変種とも同じ規約。
        assert_eq!(
            WasmError::ScenarioParse(SceneError::JsonParse("boom".to_string())).to_string(),
            r#"JsonParse("boom")"#
        );
        assert_eq!(
            WasmError::WorldBuild(SceneError::UnknownMaterial("鋼".to_string())).to_string(),
            r#"UnknownMaterial("鋼")"#
        );
        assert_eq!(
            WasmError::AppendScenarioBodies(SceneError::UnknownBodyName("b".to_string()))
                .to_string(),
            r#"UnknownBodyName("b")"#
        );
        assert_eq!(
            WasmError::ScenarioProbes(SceneError::InvalidValue("v".to_string())).to_string(),
            r#"InvalidValue("v")"#
        );
        assert_eq!(
            WasmError::HeadlessRun(SceneError::JsonParse("boom".to_string())).to_string(),
            r#"JsonParse("boom")"#
        );
        assert_eq!(
            WasmError::ScenarioExport(SceneError::InvalidValue(
                "sph.raw_state.position: 非有限値NaNがindex 3にある(発散の疑い)".to_string()
            ))
            .to_string(),
            r#"InvalidValue("sph.raw_state.position: 非有限値NaNがindex 3にある(発散の疑い)")"#
        );

        let cases: Vec<(WasmError, &str)> = vec![
            (
                WasmError::ShapeSerializeFailed("boom".to_string()),
                "failed to serialize shape: boom",
            ),
            // シリアライズ側と違い、こちらは`spawn_shape_json`へ壊れた文字列を
            // 渡せば実際に踏める(その変種のdoc参照)。
            (
                WasmError::ShapeParseFailed("boom".to_string()),
                "failed to parse shape json: boom",
            ),
            (
                WasmError::ScenarioSerializeFailed("boom".to_string()),
                "failed to serialize scenario: boom",
            ),
            (
                WasmError::HeadlessResultSerializeFailed("boom".to_string()),
                "failed to serialize result: boom",
            ),
            (
                WasmError::BodyIndexOutOfRange { index: 7, count: 3 },
                "body index 7 out of range (body_count=3)",
            ),
            // 件数を含まない文面は`BodyIndexOutOfRange`とは別変種として
            // 保たれている(`try_body_meta_at`の従来の文面)。
            (
                WasmError::BodyMetaIndexOutOfRange { index: 7 },
                "body index 7 out of range",
            ),
            (
                WasmError::BodyNoLongerExists { index: 7 },
                "body index 7 no longer exists in the current World state (removed, or created after the currently restored Timeline snapshot)",
            ),
            (
                WasmError::ImportedProbeIndexOutOfRange { index: 7, count: 3 },
                "imported probe index 7 out of range (imported_probe_count=3)",
            ),
            (
                WasmError::ImportedProbeHandleMissing { handle: 7 },
                "imported probe handle 7 has no matching World::probe (World-side removal is not implemented, this should not happen)",
            ),
            (
                WasmError::CircuitElementIndexOutOfRange { index: 7, count: 3 },
                "circuit element index 7 out of range (circuit_element_count=3)",
            ),
            (
                WasmError::ThermalNodeIndexOutOfRange { index: 7, count: 3 },
                "thermal node index 7 out of range (thermal node count=3, is the thermal domain enabled in this scene?)",
            ),
            (
                WasmError::VoltageSourceIndexOutOfRange { index: 7, count: 3 },
                "voltage source index 7 out of range (voltage source count=3, is the circuit domain enabled in this scene?)",
            ),
            (
                WasmError::FrameIndexOutOfRange { index: 7, count: 3 },
                "frame index 7 out of range (frame_count=3)",
            ),
            (
                WasmError::SnapshotIndexOutOfRange { index: 7, count: 3 },
                "snapshot index 7 out of range (snapshot_count=3)",
            ),
            (
                WasmError::BookmarkIndexOutOfRange { index: 7, count: 3 },
                "bookmark index 7 out of range (bookmark_count=3)",
            ),
            (
                WasmError::UnknownMaterial("謎".to_string()),
                "unknown material: 謎",
            ),
            (
                WasmError::MaterialAlreadyExists("謎".to_string()),
                "material already exists: 謎",
            ),
            (
                WasmError::CircuitDomainNotEnabled,
                "circuit domain is not enabled in the current world",
            ),
            (
                WasmError::SphDomainNotEnabled,
                "SPH fluid domain is not enabled (spawn a fluid block first via \"+ 流体\")",
            ),
            (
                WasmError::GridFluidDomainNotEnabled,
                "grid fluid domain is not enabled (call enable_grid_fluid_2d_domain first)",
            ),
            (
                WasmError::GasCompartmentNotEnabled,
                "gas compartment is not enabled (call enable_gas_compartment first)",
            ),
            (WasmError::InvalidDt, "dt must be a positive finite number"),
            (
                WasmError::InvalidDensity,
                "density must be a positive finite number",
            ),
            (
                WasmError::InvalidMass,
                "mass must be a positive finite number",
            ),
            (
                WasmError::InvalidScaleComponent,
                "scale components must be positive",
            ),
            // 旧実装の`{other:?}`(`other`は`&str`)と同じ引用符付き表示。
            (
                WasmError::UnknownBodyType("Rigid".to_string()),
                "unknown body type \"Rigid\" (expected Dynamic/Static/Kinematic)",
            ),
            (
                WasmError::CannotRemoveFloor,
                "床は削除できません(シーンの基準面)",
            ),
            (
                WasmError::CannotDuplicateRemovedBody,
                "cannot duplicate a removed body",
            ),
            (
                WasmError::BodyHasNoHingeMotor { index: 7 },
                "body index 7 has no hinge motor",
            ),
            (
                WasmError::GroundHasNoScaleHandle,
                "Ground is static and has no scale handle",
            ),
            (
                WasmError::ApplyComponentInvalidJson("boom".to_string()),
                "apply_component: invalid JSON payload: boom",
            ),
            (
                WasmError::UnknownApplyComponentKind("zap".to_string()),
                "apply_component: unknown kind \"zap\"",
            ),
            (
                WasmError::UnknownReadComponentKind("zap".to_string()),
                "read_component: unknown kind \"zap\"",
            ),
        ];
        for (error, expected) in cases {
            assert_eq!(error.to_string(), expected, "variant {error:?}");
        }
    }

    /// **`WasmError`導入の主眼**——シーンJSON経路の`Err`をネイティブで
    /// 検証する。以前は`JsValue`のネイティブ構築がプロセスごとabortしていた
    /// ため、「不正なシーンJSONを拒否する」という**このcrateの最も外向きの
    /// 契約**が1本もテストされていなかった。
    ///
    /// `SceneError`をそのまま保持する設計(`WasmError`のdoc)により、
    /// 「JSONとして壊れている」と「JSONは読めたが材料名が解決できない」を
    /// **区別して**検証できる——文字列を突き合わせるだけでは区別が付かない。
    #[test]
    fn scene_json_paths_reject_broken_and_unresolvable_scenes_with_the_right_error_kind() {
        use sim_world::SceneError;

        // (1) JSONとして壊れている——`Scenario::from_json`が`JsonParse`を返し、
        //     `WasmError::ScenarioParse`で包まれる。
        assert_err_matches!(
            WasmWorld::from_scene_json_impl("{ this is not json"),
            WasmError::ScenarioParse(SceneError::JsonParse(_))
        );
        assert_err_matches!(
            run_headless_scenario_json_impl("{ this is not json", 1),
            WasmError::HeadlessRun(SceneError::JsonParse(_))
        );
        let mut world = new_world();
        assert_err_matches!(
            world.import_scene_json_impl("{ this is not json"),
            WasmError::ScenarioParse(SceneError::JsonParse(_))
        );

        // (2) JSONとしては妥当だが材料名が解決できない——`World`構築側で
        //     `UnknownMaterial`になる。`from_scene_json`は`WorldBuild`、
        //     `import_scene_json`は`AppendScenarioBodies`と、**どちらの
        //     構築経路で落ちたのかが変種で分かる**(同じ`SceneError`を
        //     別の文脈で受けているため、文字列だけでは区別できない)。
        let unknown_material = r#"{
            "name": "unknown-material",
            "world": { "gravity": 9.80665, "dt": 0.008333333 },
            "bodies": [
              { "shape": { "sphere": { "radius": 0.3 } },
                "material": "存在しない材質",
                "position": [0.0, 20.0, 0.0],
                "name": "ball" }
            ]
        }"#;
        assert_err_matches!(
            WasmWorld::from_scene_json_impl(unknown_material),
            WasmError::WorldBuild(SceneError::UnknownMaterial(_))
        );
        assert_err_matches!(
            world.import_scene_json_impl(unknown_material),
            WasmError::AppendScenarioBodies(SceneError::UnknownMaterial(_))
        );
        assert_err_matches!(
            run_headless_scenario_json_impl(unknown_material, 1),
            WasmError::HeadlessRun(SceneError::UnknownMaterial(_))
        );
        // 材料名は`SceneError`側にそのまま載っている(UIがどの名前を直せば
        // よいか示せる)ことまで確かめる。
        match WasmWorld::from_scene_json_impl(unknown_material).err() {
            Some(WasmError::WorldBuild(SceneError::UnknownMaterial(name))) => {
                assert_eq!(name, "存在しない材質");
            }
            other => panic!("expected WorldBuild(UnknownMaterial), got {other:?}"),
        }

        // (3) ボディは解決できるが`probes`が存在しないボディ名を指す。
        //     **同じ壊れ方でも2経路で落ちる場所が違う**のがここで見える:
        //     `from_scene_json`は`World::from_scenario_with_body_ids`が
        //     probesまで含めて構築するため`WorldBuild`で落ち、
        //     `import_scene_json`は`append_scenario_bodies`がprobesを対象外と
        //     する設計(そのdoc参照)なので、後続の`add_scenario_probes`まで
        //     進んでから`ScenarioProbes`で落ちる。`SceneError`の中身
        //     (`UnknownBodyName`)は同じなので、**文字列を突き合わせる形の
        //     検証ではこの差を捉えられない**——変種を分けた甲斐がここに出る。
        let unknown_probe_body = r#"{
            "name": "unknown-probe-body",
            "world": { "gravity": 9.80665, "dt": 0.008333333 },
            "bodies": [
              { "shape": { "sphere": { "radius": 0.3 } },
                "material": "鋼(炭素鋼)",
                "position": [0.0, 20.0, 0.0],
                "name": "ball" }
            ],
            "probes": [ { "body_pos_y": "そんなボディは無い" } ]
        }"#;
        assert_err_matches!(
            WasmWorld::from_scene_json_impl(unknown_probe_body),
            WasmError::WorldBuild(SceneError::UnknownBodyName(_))
        );
        assert_err_matches!(
            world.import_scene_json_impl(unknown_probe_body),
            WasmError::ScenarioProbes(SceneError::UnknownBodyName(_))
        );

        // 一連の失敗を経ても、`import_scene_json`を呼び続けた`world`は
        // 依然として使える(Errがワールドを壊していない)。
        let valid = include_str!("../../../scenes/d1-free-fall.json");
        let added = world
            .import_scene_json_impl(valid)
            .expect("a valid scene must still import after the failures above");
        assert_eq!(added, 1);
    }

    /// インポート済みプローブのindex範囲外(`try_imported_probe_handle_at`)。
    /// 既定シーンはプローブを1本も持たないので index 0 が既に範囲外——
    /// `count: 0`まで含めて検証できる。
    #[test]
    fn imported_probe_accessors_reject_out_of_range_indices() {
        let world = new_world();
        assert_eq!(world.imported_probe_count_impl(), 0);
        assert_err_is(
            world.imported_probe_label_at_impl(0),
            WasmError::ImportedProbeIndexOutOfRange { index: 0, count: 0 },
        );
        assert_err_is(
            world.imported_probe_history_impl(0),
            WasmError::ImportedProbeIndexOutOfRange { index: 0, count: 0 },
        );

        // プローブを1本持つシーンでは境界がずれる(index 0はOk、1はErr)。
        let json = include_str!("../../../scenes/d6-floating-box-f4.json");
        let world = WasmWorld::from_scene_json_impl(json)
            .expect("scenes/d6-floating-box-f4.json must be a valid scene");
        assert_eq!(world.imported_probe_count_impl(), 1);
        assert!(world.imported_probe_label_at_impl(0).is_ok());
        assert_eq!(
            world.imported_probe_history_impl(0).unwrap(),
            Vec::<f64>::new()
        );
        assert_err_is(
            world.imported_probe_label_at_impl(1),
            WasmError::ImportedProbeIndexOutOfRange { index: 1, count: 1 },
        );
        assert_err_is(
            world.imported_probe_history_impl(1),
            WasmError::ImportedProbeIndexOutOfRange { index: 1, count: 1 },
        );
    }

    /// 回路素子ラベルの2つの失敗(**回路ドメインが無効**と**素子indexが
    /// 範囲外**)が別々の変種になること。既定シーン(`WasmWorld::new`)は
    /// 分圧回路を持つので後者を、回路を持たないシーンJSONで前者を確かめる
    /// ——「回路が無い」と「番号が大きすぎる」はUIの復旧手順が違うため、
    /// 取り違えると誤った案内になる。
    #[test]
    fn circuit_element_label_distinguishes_a_missing_domain_from_an_out_of_range_index() {
        let world = new_world();
        let count = world.circuit_element_count_impl();
        assert!(count > 0, "既定シーンは分圧回路を持つ");
        assert!(world.circuit_element_label_at_impl(0).is_ok());
        assert!(world.circuit_element_label_at_impl(count - 1).is_ok());
        assert_err_is(
            world.circuit_element_label_at_impl(count),
            WasmError::CircuitElementIndexOutOfRange {
                index: count,
                count,
            },
        );

        // 回路ドメインを持たないシーン(D1=自由落下)では、同じ呼び出しが
        // `CircuitDomainNotEnabled`になる。
        let json = include_str!("../../../scenes/d1-free-fall.json");
        let world = WasmWorld::from_scene_json_impl(json)
            .expect("scenes/d1-free-fall.json must be a valid scene");
        assert_eq!(world.circuit_element_count_impl(), 0);
        assert_err_is(
            world.circuit_element_label_at_impl(0),
            WasmError::CircuitDomainNotEnabled,
        );
    }

    /// D11(振り子、`scenes/d11-pendulum.json`)は`probes`に`body_pos_x`/
    /// `body_pos_y`の2本(いずれも"bob")を持つ。両方が順序どおり・正しいラベルで
    /// 読めること(Probe Graphsパネルが複数系列を束ねられることの前提)。
    #[test]
    fn imported_probe_labels_reflect_the_d11_pendulum_scene_probes_in_order() {
        let json = include_str!("../../../scenes/d11-pendulum.json");
        let world = WasmWorld::from_scene_json_impl(json)
            .expect("scenes/d11-pendulum.json must be a valid scene");
        assert_eq!(world.imported_probe_count_impl(), 2);
        assert_eq!(
            world.imported_probe_label_at_impl(0).unwrap(),
            "BodyPosX(bob)"
        );
        assert_eq!(
            world.imported_probe_label_at_impl(1).unwrap(),
            "BodyPosY(bob)"
        );
    }

    /// **残タスク完遂の増分B3(D9/D34/D35)**: `from_scene_json`が力学ボディを
    /// 1つも持たないシーン(D9=`thermal`のみ、D34/D35=`astro`のみ)を実際に
    /// 拒否せず読み込めること(以前の`ids.first()`が`None`なら`Err`を返す
    /// ガードの撤廃、モジュールdoc参照)。あわせて`body_count()`が0であること、
    /// シーン定義プローブ(`imported_probe_count`)が各シーンの`probes`本数
    /// どおりであること(D9=1本、D34=2本、D35=4本)、`step()`を呼んでも
    /// パニックしないことを確認する。
    ///
    /// **正直な制約**: `y_probe`/`speed_probe`が`None`になったときの
    /// `y_probe_history_f64`/`speed_probe_history_f64`(空の`Float64Array`を
    /// 返す経路、かつては`.expect(...)`でパニックしていた箇所そのもの)は
    /// ここでは検証できない——モジュールdoc「正直な制約」のとおり
    /// `Float64Array::new_with_length`はネイティブターゲットでは(空配列
    /// 生成であっても)`cannot call wasm-bindgen imported functions on
    /// non-wasm targets`でパニックする。したがってこの経路は実ブラウザでの
    /// Playwright確認(`docs/22-roadmap/02-feature-checklist.md`参照)でのみ
    /// 検証した。
    #[test]
    fn from_scene_json_loads_bodyless_thermal_and_astro_gallery_scenes_without_panicking() {
        fn check_bodyless_scene(path: &str, json: &str, expected_imported_probes: usize) {
            let mut world = WasmWorld::from_scene_json_impl(json)
                .unwrap_or_else(|e| panic!("{path} must be a valid (bodyless) scene: {e:?}"));
            assert_eq!(
                world.body_count_impl(),
                0,
                "{path} should define zero bodies"
            );
            assert_eq!(
                world.imported_probe_count_impl(),
                expected_imported_probes,
                "{path} probe count mismatch"
            );
            world.step();
            world.step();
        }

        check_bodyless_scene(
            "scenes/d9-cooling-coffee.json",
            include_str!("../../../scenes/d9-cooling-coffee.json"),
            1,
        );
        check_bodyless_scene(
            "scenes/d34-solar-system-single-planet.json",
            include_str!("../../../scenes/d34-solar-system-single-planet.json"),
            2,
        );
        check_bodyless_scene(
            "scenes/d35-orbital-insertion.json",
            include_str!("../../../scenes/d35-orbital-insertion.json"),
            4,
        );
    }

    /// **量子ドメインのプリセットUI経路(B9)**: `demo/src/main.ts`のプリセット関数が
    /// TypeScript側で計算するのと同じ式(`WaveFunction1D::set_gaussian_wave_packet`
    /// と同じガウス波束、調和振動子ポテンシャル$V(x)=\frac12\omega^2(x-x_c)^2$)を
    /// ここでも複製し、base64+LEでエンコードした`payload`を`apply_component`経由で
    /// 送る——**エディタが実際に送るのと同じ形のJSON**が`enable_quantum_1d_domain`を
    /// 通ることと、そのノルム・ポテンシャルが式どおりに復元されることを確認する。
    #[test]
    fn enable_quantum_1d_domain_accepts_a_normalized_gaussian_wave_packet_and_harmonic_potential() {
        let n: usize = 64;
        let dx = 0.1;
        let center = n as f64 * dx * 0.5;
        let sigma = 0.5;
        let k0 = 2.0;
        let omega = 1.0;

        let mut psi_re = vec![0.0; n];
        let mut psi_im = vec![0.0; n];
        for i in 0..n {
            let x = i as f64 * dx;
            let envelope = (-(x - center).powi(2) / (4.0 * sigma * sigma)).exp();
            psi_re[i] = envelope * (k0 * x).cos();
            psi_im[i] = envelope * (k0 * x).sin();
        }
        // Σ|ψ|²dx=1になるよう正規化(`WaveFunction1D::set_gaussian_wave_packet`と
        // 同じ離散ノルム規約、モジュールdoc「構築レシピ」の§規約1参照)。
        let norm_before: f64 = psi_re
            .iter()
            .zip(psi_im.iter())
            .map(|(re, im)| re * re + im * im)
            .sum::<f64>()
            * dx;
        let scale = 1.0 / norm_before.sqrt();
        for value in psi_re.iter_mut().chain(psi_im.iter_mut()) {
            *value *= scale;
        }

        let mut v = vec![0.0; n];
        for (i, v_i) in v.iter_mut().enumerate() {
            let x = i as f64 * dx - center;
            *v_i = 0.5 * omega * omega * x * x;
        }

        let payload = serde_json::json!({
            "psi_re": sim_world::encode_f64_le_base64_finite(&psi_re).unwrap(),
            "psi_im": sim_world::encode_f64_le_base64_finite(&psi_im).unwrap(),
            "v": sim_world::encode_f64_le_base64_finite(&v).unwrap(),
            "dx": dx,
        })
        .to_string();

        let mut world = new_world();
        assert!(
            world.inner.quantum_1d().is_none(),
            "quantum_1d must not be enabled before the preset call"
        );
        world
            .apply_component_impl("enable_quantum_1d_domain", &payload)
            .expect("a valid preset payload must enable the quantum_1d domain");

        let wave = world
            .inner
            .quantum_1d()
            .expect("quantum_1d domain must be enabled after enable_quantum_1d_domain");
        assert_eq!(wave.len(), n);
        assert!((wave.norm() - 1.0).abs() < 1e-9, "norm={}", wave.norm());
        for i in [0usize, n / 4, n / 2, 3 * n / 4, n - 1] {
            let x = i as f64 * dx - center;
            let expected = 0.5 * omega * omega * x * x;
            assert!(
                (wave.v[i] - expected).abs() < 1e-9,
                "i={i} v={} expected={expected}",
                wave.v[i]
            );
        }
    }

    /// 2D版(B9)——`WaveFunction2D::set_gaussian_wave_packet`と同じ2Dガウス波束、
    /// ポテンシャルは`scenes/d27-double-slit.json`と同じ二重スリット障壁(壁の高さ・
    /// スリット2本の幅と間隔)を複製する。ノルムと、障壁/スリット位置でのポテンシャル
    /// 値が期待どおりであることを確認する。
    #[test]
    fn enable_quantum_2d_domain_accepts_a_normalized_wave_packet_and_double_slit_potential() {
        let nx: usize = 64;
        let ny: usize = 64;
        let dx = 0.1;
        let dy = 0.1;
        let x0 = 1.0;
        let y0 = ny as f64 * dy * 0.5;
        let sigma_x = 0.5;
        let sigma_y = 2.0;
        let k0 = 3.0;

        let mut psi_re = vec![0.0; nx * ny];
        let mut psi_im = vec![0.0; nx * ny];
        for iy in 0..ny {
            for ix in 0..nx {
                let x = ix as f64 * dx;
                let y = iy as f64 * dy;
                let envelope = (-(x - x0).powi(2) / (4.0 * sigma_x * sigma_x)
                    - (y - y0).powi(2) / (4.0 * sigma_y * sigma_y))
                    .exp();
                let idx = iy * nx + ix;
                psi_re[idx] = envelope * (k0 * x).cos();
                psi_im[idx] = envelope * (k0 * x).sin();
            }
        }
        let norm_before: f64 = psi_re
            .iter()
            .zip(psi_im.iter())
            .map(|(re, im)| re * re + im * im)
            .sum::<f64>()
            * dx
            * dy;
        let scale = 1.0 / norm_before.sqrt();
        for value in psi_re.iter_mut().chain(psi_im.iter_mut()) {
            *value *= scale;
        }

        // 二重スリット障壁: x方向1セル厚の壁、y方向に2本の隙間(`scenes/
        // d27-double-slit.json`と同じ構成、`sim_quantum::schrodinger2d`のQ6テスト
        // 参照)。
        let barrier_ix = nx / 2;
        let v0 = 60.0;
        let slit_half_width = 3.0 * dy;
        let slit_separation = 10.0 * dy;
        let mut v = vec![0.0; nx * ny];
        for iy in 0..ny {
            let y = iy as f64 * dy - y0;
            let in_slit1 = (y - slit_separation * 0.5).abs() < slit_half_width;
            let in_slit2 = (y + slit_separation * 0.5).abs() < slit_half_width;
            if !in_slit1 && !in_slit2 {
                v[iy * nx + barrier_ix] = v0;
            }
        }

        let payload = serde_json::json!({
            "psi_re": sim_world::encode_f64_le_base64_finite(&psi_re).unwrap(),
            "psi_im": sim_world::encode_f64_le_base64_finite(&psi_im).unwrap(),
            "v": sim_world::encode_f64_le_base64_finite(&v).unwrap(),
            "nx": nx,
            "ny": ny,
            "dx": dx,
            "dy": dy,
        })
        .to_string();

        let mut world = new_world();
        world
            .apply_component_impl("enable_quantum_2d_domain", &payload)
            .expect("a valid preset payload must enable the quantum_2d domain");

        let wave = world
            .inner
            .quantum_2d()
            .expect("quantum_2d domain must be enabled after enable_quantum_2d_domain");
        assert_eq!(wave.nx, nx);
        assert_eq!(wave.ny, ny);
        assert!((wave.norm() - 1.0).abs() < 1e-9, "norm={}", wave.norm());

        // 障壁の壁部分は`v0`、スリットの隙間はゼロ。
        assert_eq!(wave.v[barrier_ix], v0);
        let slit1_iy = (y0 + slit_separation * 0.5) / dy;
        assert_eq!(wave.v[slit1_iy as usize * nx + barrier_ix], 0.0);
    }

    /// 非2の冪グリッドは`WasmError::QuantumRawStateInvalid`として明示的に`Err`を
    /// 返し、パニックしない(タスク仕様「invalid grid sizes ... are rejected with
    /// a clear error rather than panicking or silently truncating」)。検証は
    /// `sim_world::build_quantum_1d_wave_from_raw`——シーンJSON経路
    /// (`from_scenario_with_body_ids`)と全く同じ関数を通る。
    #[test]
    fn enable_quantum_1d_domain_rejects_a_non_power_of_two_grid_size_without_panicking() {
        let n = 100; // 2の冪ではない
        let zeros = vec![0.0; n];
        let payload = serde_json::json!({
            "psi_re": sim_world::encode_f64_le_base64_finite(&zeros).unwrap(),
            "psi_im": sim_world::encode_f64_le_base64_finite(&zeros).unwrap(),
            "v": sim_world::encode_f64_le_base64_finite(&zeros).unwrap(),
            "dx": 0.1,
        })
        .to_string();

        let mut world = new_world();
        assert_err_matches!(
            world.apply_component_impl("enable_quantum_1d_domain", &payload),
            WasmError::QuantumRawStateInvalid(sim_world::SceneError::InvalidValue(_))
        );
        assert!(
            world.inner.quantum_1d().is_none(),
            "a rejected preset must not leave a partially-built domain behind"
        );
    }

    /// 2D版——`nx`は2の冪だが`ny`がそうでない、という片方だけ壊れたケースも
    /// 弾かれることを確認する(`build_quantum_2d_wave_from_raw`は両方を検証する)。
    #[test]
    fn enable_quantum_2d_domain_rejects_a_non_power_of_two_ny_without_panicking() {
        let nx = 64;
        let ny = 100; // 2の冪ではない
        let zeros = vec![0.0; nx * ny];
        let payload = serde_json::json!({
            "psi_re": sim_world::encode_f64_le_base64_finite(&zeros).unwrap(),
            "psi_im": sim_world::encode_f64_le_base64_finite(&zeros).unwrap(),
            "v": sim_world::encode_f64_le_base64_finite(&zeros).unwrap(),
            "nx": nx,
            "ny": ny,
            "dx": 0.1,
            "dy": 0.1,
        })
        .to_string();

        let mut world = new_world();
        assert_err_matches!(
            world.apply_component_impl("enable_quantum_2d_domain", &payload),
            WasmError::QuantumRawStateInvalid(sim_world::SceneError::InvalidValue(_))
        );
        assert!(world.inner.quantum_2d().is_none());
    }

    /// 配列長が食い違うペイロード(`psi_re`と`v`の長さが違う)も、2の冪長チェックを
    /// 通過した後の2段目の検証で弾かれる(`build_quantum_1d_wave_from_raw`の
    /// 「psi_re/psi_im/vの長さが揃っていない」分岐)。
    #[test]
    fn enable_quantum_1d_domain_rejects_mismatched_array_lengths_without_panicking() {
        let n = 64;
        let psi = vec![0.0; n];
        let mismatched_v = vec![0.0; n / 2];
        let payload = serde_json::json!({
            "psi_re": sim_world::encode_f64_le_base64_finite(&psi).unwrap(),
            "psi_im": sim_world::encode_f64_le_base64_finite(&psi).unwrap(),
            "v": sim_world::encode_f64_le_base64_finite(&mismatched_v).unwrap(),
            "dx": 0.1,
        })
        .to_string();

        let mut world = new_world();
        assert_err_matches!(
            world.apply_component_impl("enable_quantum_1d_domain", &payload),
            WasmError::QuantumRawStateInvalid(sim_world::SceneError::InvalidValue(_))
        );
    }

    /// `enable_quantum_1d_domain`/`enable_quantum_2d_domain`は`enable_grid_fluid_
    /// 2d_domain`と違って**冪等ではない**——呼ぶたびに渡された生状態で上書きする
    /// (`enable_quantum_1d_domain_impl`のdoc参照)。2回目の呼び出しが1回目と違う
    /// グリッド長を渡しても、古い状態を引きずらず新しい状態にきれいに置き換わる
    /// ことを確認する。
    #[test]
    fn enable_quantum_1d_domain_overwrites_a_previously_enabled_domain_with_a_different_size() {
        let make_payload = |n: usize, dx: f64| {
            let zeros = vec![0.0; n];
            serde_json::json!({
                "psi_re": sim_world::encode_f64_le_base64_finite(&zeros).unwrap(),
                "psi_im": sim_world::encode_f64_le_base64_finite(&zeros).unwrap(),
                "v": sim_world::encode_f64_le_base64_finite(&zeros).unwrap(),
                "dx": dx,
            })
            .to_string()
        };

        let mut world = new_world();
        world
            .apply_component_impl("enable_quantum_1d_domain", &make_payload(32, 0.2))
            .expect("first preset call must succeed");
        assert_eq!(world.inner.quantum_1d().unwrap().len(), 32);

        world
            .apply_component_impl("enable_quantum_1d_domain", &make_payload(128, 0.05))
            .expect("second preset call must succeed and replace the domain");
        let wave = world.inner.quantum_1d().unwrap();
        assert_eq!(wave.len(), 128);
        assert_eq!(wave.dx, 0.05);
    }
}
