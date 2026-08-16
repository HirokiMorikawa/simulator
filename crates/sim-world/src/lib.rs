//! World facade。設計: docs/00-foundation/04-architecture.md §1.1、
//!       docs/20-integration/04-world-api.md。
//!
//! Phase A 時点では `create_body` による複数剛体の構築 + `MechanicsSolver` 駆動を
//! 正式な `RigidBodySet` 経由で提供する縮小版。フル API(joint/circuit/fluid region/
//! Coupling、コマンドキュー、スナップショット、シーン JSON)は後続の増分で
//! docs/20-integration/04-world-api.md §2 に沿って拡張する。
//!
//! `create_body`/`remove_body`/`body_position` は `sim_core::BodyId`(世代付き index)を
//! 使う(設計 docs/00-foundation/04-architecture.md §3「削除済み ID へのアクセスは
//! `None`」)。世代は `World` 層で管理する — `sim_mechanics::RigidBodySet` 自体はまだ
//! スロットの削除・再利用に対応していないため(密な `Vec` ベースで、削除は配列の
//! 詰め直しか tombstone 化を要する大きめの改修になる)、`remove_body` は下層スロットを
//! 「無効化」(`BodyType::Static` 化 + 遠方(y=-1e9)へ退避 + 速度ゼロ化)するに留め、世代
//! カウンタだけを正式にインクリメントして以後のアクセスを `None` にする。
//! **ジョイント・結合の連鎖削除(設計 §2 の `remove_body` 完全仕様)は群2で実装した**
//! ——以前は「`World` がまだジョイント・Coupling を保持していないため対象外」と
//! 書いてあったが、その前提は既に成り立っていない(`mechanics.joints`/`ball_joints`/
//! `slider_joints`/`hinge_motors` と `couplings` を保持している)。削除された剛体を
//! 参照するジョイントは `disabled` にし、参照する `Coupling` は
//! `Coupling::referenced_bodies()`(群1の内省層)で特定して取り除く。
//! `EnergyLedger`(docs/00-foundation/04-architecture.md §1.1.2(2))は P1 で導入済み:
//! シーン構築(`create_body` 呼び出し列)が終わり最初の `step()` が呼ばれた時点の
//! 合計エネルギーを基準点として、以後毎 step 後に記帳する(構築途中の`create_body`
//! 呼び出し自体は台帳上の「エネルギーの出現」として扱わない)。
//!
//! **全ドメイン合成(ワークストリームB増分)**: `mechanics` は常時有効な正典ドメインとして
//! 保持し、`thermal`(`sim_thermal::ThermalSolver`)・`em_electrostatics`
//! (`sim_em::PointChargeSystem`)・`astro`(`sim_astro::NBodySystem`)・`circuit`
//! (`sim_em::Circuit`、回路のMNAソルバ。`Solver`トレイト実装は`sim-coupling::JouleHeat`
//! 増分で追加済み)は`Option`として追加した(シーンが使う分だけ`enable_*`で有効化、設計
//! 「Solverトレイトの共通契約」docs/00-foundation/04-architecture.md §1.2に既に準拠している
//! 型をそのまま接続)。`sim_fluid::SphFluid`・`sim_fluid::GridFluid2D`にも`Solver`トレイトを
//! 実装し`sph`/`grid_fluid`ドメインとして同様に接続した(`sph.rs`/`grid_fluid.rs`の
//! `Solver`実装のdoc参照。これで`GridFluidRigid`/`ConvectionLink`/`BoussinesqBuoyancy`/
//! `SphRigid`各Couplingが要求する「決定的sub-step経由での流体状態進行」の前提が揃った)。
//! `step()`は有効なドメインを固定順(mechanics→thermal→em→
//! astro→circuit→sph→grid_fluid、`state_hash`も同順)で順に進める。各ドメインは
//! `orchestrator::sub_step_count`(設計§1.3の
//! 決定的sub-step数算出、`max_stable_dt()`から算出)に従いsub-stepする — Lie-Trotter
//! operator splitting自体(pre/post couplingを挟むパイプライン、
//! docs/20-integration/01-coupling-matrix.md §4)の**pre/post 2相分離は群5で実装完了**
//! ——`step()`は「pre 相(全結合)→ 全ドメインsub-step → post 相(全結合)」の順に走る。
//! 各結合がどちらの相に載るかは`sim-coupling`側の実装が決める(既定は post、
//! 各モジュールdoc参照)。登録済み`Coupling`(`add_coupling`)を
//! 自動適用するレジストリも実装済み(`couplings`フィールド、`apply_coupling`のdoc参照)。シーンJSON`couplings`セクションからの自動解決・排他結合検査
//! (`sim-coupling::validate_exclusive_couplings`)との接続は未実装(`scenario`モジュール
//! doc参照)。`quantum`/`statistical`は
//! 専用シーンでのみ有効化する設計方針のため見送る。`gas`
//! (`sim_thermal::GasCompartment`、断熱圧縮の`PistonGas`結合が使う)・`conduction_rod`
//! (`sim_thermal::ConductionRod1D`、D16「熱伝導レース」が使う)は`Solver`を実装しない —
//! `step()`の自動走査対象ではなく、呼び出し側が
//! `apply_coupling`/`conduction_rod_mut().step(dt)`を明示的に呼んで状態を進める。

mod demos;
mod export;
mod integration_scenarios;
mod orchestrator;
mod overlap;
mod raw_bytes;
mod raycast;
mod scenario;

pub use export::{shape_to_shape_json, to_scenario};
pub use raw_bytes::{
    decode_base64, decode_bool_bitpacked_base64, decode_f64_le_base64, decode_i8_base64,
    decode_vec3_le_base64, encode_base64, encode_bool_bitpacked_base64, encode_f64_le_base64,
    encode_f64_le_base64_finite, encode_i8_base64, encode_vec3_le_base64, RawBytesError,
};
pub use scenario::{
    run_headless_scenario, shape_json_to_shape, BodyScenarioDesc, CompoundChildJson,
    HeadlessRunResult, MaterialOverride, PassCriterionJson, PassCriterionOperator,
    PhaseChangeOverrideJson, PredictionPromptJson, Scenario, SceneError, ShapeJson,
    WorldScenarioOptions,
};

use sim_core::{EnergyLedger, EventQueue, MaterialDb, Solver, SolverContext, StateHasher};
use sim_math::{SimRng, Vec3};
use sim_mechanics::{BodyType, MechanicsSolver, RigidBodyDesc, Shape};

// 下流crate(sim-wasm等)が別途sim-core依存を追加しなくてもBodyIdを使えるよう、
// Worldの公開APIとしてそのまま再エクスポートする。
pub use sim_core::BodyId;

/// World 生成オプション。剛体はここでは作らず `create_body` で追加する。
pub struct WorldOptions {
    pub gravity: f64,
    pub dt: f64,
    pub seed: u64,
}

impl Default for WorldOptions {
    fn default() -> Self {
        WorldOptions {
            gravity: 9.80665,
            dt: 1.0 / 120.0,
            seed: 0,
        }
    }
}

/// 実行中の状態変更コマンド(設計§1「書き込みは規律」— 実行中の変更は
/// シーン構築時のcreate系と本コマンドの2経路のみ、docs/20-integration/04-world-api.md
/// §2)。次`step()`の先頭で適用され、`command_log()`に記録される(リプレイ検証用)。
///
/// **縮約実装の理由**: 設計が例示する5種(`ApplyForce`・`SetMotorTarget`・`SetSwitch`・
/// `SetHeatSource`・`Grab`/`MoveGrab`/`Release`)を全て実装する。`SetMotorTarget`・
/// `SetSwitch`は`World`にJointId/CircuitId管理が無く、`sim_mechanics::
/// MechanicsSolver::hinge_motors`・`sim_em::Circuit`の switches が生indexで管理
/// されている(削除操作が無くID再利用の懸念が無いため`BodyId`のような世代管理までは
/// 導入していない)ことを踏まえ、`hinge_motor_index`/`switch_index`という生indexを
/// 直接引数に取る縮約版とする。`SetHeatSource`は`ApplyForce`と同じ「1step分だけ効く」
/// 縮約セマンティクス(設計が意図する可能性のある「変更するまで持続するダイヤル」では
/// ない、継続加熱には毎stepの再push が必要)を採る — `ThermalNode::heat_accum`が
/// 毎step末尾でクリアされる既存の設計(`sim-thermal`のT4テスト参照)にそのまま
/// 乗せられるため。`SetMotorTarget`は設計の例示(`{joint, velocity}`)とは異なり
/// `theta_target`(角度)を設定する — 実装済みの`HingeMotorPd`が速度ではなく角度目標の
/// PD位置サーボ(`joint`モジュールdoc参照)であるため、設計の例示する変数名ではなく
/// 実装済みのモーターが実際に持つパラメータをそのまま公開する(こちらも継続的な状態
/// 変更、一度設定すると次に変更するまで持続する — `HingeMotorPd`自体が
/// `MechanicsSolver::step()`内で毎step自動適用される永続的な構成要素であるため、
/// `SetHeatSource`とは異なり1step限りの効果ではない)。`Grab`/`MoveGrab`/`Release`
/// (マウスでつかむ)は、設計が示唆する「ばね拘束」ではなく`sim_mechanics::
/// BallJoint`(動く目標点へのワールド固定点、`joint`モジュールdoc参照)による
/// 剛な(rigid)ピン拘束として実装する — `DistanceJoint`(`length=0`)は方向ベクトルの
/// 正規化がゼロ距離近傍で退化し目標点付近で拘束が効かなくなる(実装検証中に
/// 発見、掴んだ対象が目標点周りで収束せず振動し続ける形で顕在化した)ため使わず、
/// ワールド座標軸沿いの3本の独立スカラー拘束(ゼロ距離でも退化しない)を持つ
/// `BallJoint`を採用した。専用のばね(soft constraint、未実装)ではなく既存の
/// Baumgarte安定化されたPGS拘束をそのまま流用する縮約(掴んだ瞬間に対象が目標点へ
/// 強く引き寄せられる、真のばねより硬い挙動になりうることを承知の上での簡略化)。
/// 1剛体につき同時に1つのgrabを想定し(`grab_joints`マップで剛体index→
/// `mechanics.ball_joints`indexを対応付け)、`Release`は`BallJoint::disabled`を
/// 立てて無効化する(密な`Vec`からの実削除はしない、`RigidBodySet`の削除と同じ
/// 「無効化に留める」方針、`joint`モジュールdoc参照)。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Command {
    /// 剛体`body`のワールド座標`point`(`None`なら重心、トルクなし)に`force`を加える。
    ApplyForce {
        body: BodyId,
        force: Vec3,
        point: Option<Vec3>,
    },
    /// ヒンジモーター(`MechanicsSolver::add_hinge_motor`を呼んだ順のindex)の目標角度を
    /// 変更する(モジュールdoc参照、設計の`velocity`ではなく実装済みの`theta_target`)。
    SetMotorTarget {
        hinge_motor_index: usize,
        theta_target: f64,
    },
    /// 回路のスイッチ(`sim_em::Circuit::add_switch`が返すindex)の開閉を変更する
    /// (`World`は単一`circuit`ドメイン前提のため`CircuitId`引数は省略、`circuit_probe`
    /// と同じ縮約)。
    SetSwitch { switch_index: usize, closed: bool },
    /// 熱ノード`node`に`watts`ワットの熱源を1step分だけ与える(モジュールdoc「1step分
    /// だけ効く」縮約参照)。
    SetHeatSource { node: usize, watts: f64 },
    /// 剛体`body`のローカル座標`anchor_local`をワールド座標`target`へピン拘束する
    /// (モジュールdoc「`Grab`系」参照)。既に同じ`body`をgrab中なら前のgrabを
    /// 無効化してから新設する。
    Grab {
        body: BodyId,
        anchor_local: Vec3,
        target: Vec3,
    },
    /// `body`の既存grabの目標点を`target`へ更新する(grab中でなければ無視)。
    MoveGrab { body: BodyId, target: Vec3 },
    /// `body`の既存grabを解除する(grab中でなければ無視)。
    Release { body: BodyId },
    /// 剛体`body`の質量を`mass`[kg]へ変更する(Inspector の RigidBody Component、
    /// 設計 docs/23-frontend/01-editor.md §1.3「編集は次ステップ先頭で Command として
    /// 適用される」)。形状は変えないので密度が暗黙に動く(`RigidBodySet::set_mass`)。
    SetBodyMass { body: BodyId, mass: f64 },
    /// 剛体`body`の種別を切り替える(Dynamic/Static/Kinematic)。Dynamic へ戻すときに
    /// 使う質量を`mass`で明示する——**Static 化で inv_mass=0 になると元の質量が
    /// 失われる**ため、復元値はコマンド側が持つ必要がある。
    SetBodyType {
        body: BodyId,
        body_type: sim_mechanics::BodyType,
        mass: f64,
    },
    /// 剛体`body`の衝突フィルタ(設計 docs/10-mechanics/02-collision-detection.md §4.1)
    /// を設定する。broadphase で双方向 AND を取るため、片側の変更だけで
    /// ペアを落とせる。
    SetCollisionFilter { body: BodyId, group: u32, mask: u32 },
    /// 登録済みCoupling(`World::couplings()`が返す`CouplingInfo::index`)へ
    /// 実行時パラメータを設定する(**残タスク完遂の縦串⑤増分**、操縦面の
    /// 舵角変更が最初の用途)。Coupling registryは元々「追加のみ・実行時
    /// パラメータ変更不可」だったため、この Command がその唯一の書き換え経路。
    /// 範囲外indexや対応しないパラメータは無言で無視する(他のCommandの
    /// `is_valid`ガードと同じ「無効な入力は無視する」方針)。
    SetCouplingParam {
        coupling_index: usize,
        param: sim_coupling::CouplingParam,
        value: f64,
    },
    /// 重力場(`sim_mechanics::GravityField`)を差し替える(**重力場の抽象化増分**)。
    ///
    /// **なぜCommandにしたか**: 重力の変更は以降の全stepの結果を変える。
    /// `command_log()`に残らない変更は、同じシーン・同じseed・同じログから
    /// 再生しても軌跡が一致しない——**黙ってリプレイされない変更は決定論のバグ**
    /// である(モジュールdoc「実行中の変更はcreate系と本コマンドの2経路のみ」)。
    ///
    /// **既存の状況を正直に書いておく**: 移行前から存在する
    /// `MechanicsSolver::set_gravity`/`set_gravity_direction`への直接呼び出し
    /// (`World::set_environment`と`sim-wasm`の`set_gravity`/
    /// `set_gravity_direction` kind)は**Commandを経由せず即時に効き、
    /// この`command_log()`には残らない**。現状それらの記録を担っているのは
    /// フロントエンド側の別台帳(`demo/src/main.ts`の`CommandLogEntry::
    /// SetGravity`/`SetGravityDirection`、Replayタブが読む)であり、
    /// **物理コアだけを使う経路(ヘッドレス実行・Rust APIの直接利用)では
    /// 記録されない**という穴が残っている。本増分ではそこには手を付けていない
    /// ——Command化すると適用が1step遅れる挙動変更になり、既にその即時性に
    /// 依存している既存フロントエンド(本増分の対象外)を壊すため。
    /// 新しい経路(`sim-wasm`の`push_set_gravity_field` kind)だけが、
    /// フロントエンドに依存せず物理コア側で記録される。
    SetGravityField { field: sim_mechanics::GravityField },
}

/// `World::energy_report`の1ドメイン分(**群3で追加**、`energy_report`のdoc参照)。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DomainEnergy {
    /// ドメイン名(表示用の安定した識別子)。
    pub domain: &'static str,
    pub energy: sim_core::EnergyBreakdown,
    /// エネルギーの単位。SI以外のドメインがあるため明示する。
    pub unit: &'static str,
    /// 閉じた保存系か(`false`なら保存しないのが正しい挙動)。
    pub conservative: bool,
    /// `World::total_energy()`の合計に含まれているか。
    pub in_total: bool,
}

/// `raycast`/`overlap_sphere`のフィルタ(設計docs/20-integration/04-world-api.md §2
/// が引数に取る`Filter`、増分F1で追加)。
///
/// **設計は`Filter`の中身を定義していない**ため、クエリの用途から必要最小限を
/// 決めた: ①静的/動的の別で絞る(例: 「地面以外に当たったか」)②特定のボディを
/// 除外する(例: 自分自身を無視してレイを飛ばす)③**衝突グループのマスク**
/// (群2で追加。`RigidBodySet`に`collision_group`/`collision_mask`が入り、設計
/// docs/10-mechanics/02-collision-detection.md §4.1 のビットANDが実体を持ったため、
/// 「その概念自体が無いので対象外」という以前の記述は解消した)。
///
/// `default()`は**何も除外しない**ので、フィルタ不要な呼び出しはそれを渡せばよい。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct QueryFilter {
    /// 静的ボディ(床・壁等)を除外する。
    pub exclude_static: bool,
    /// 動的ボディを除外する(静的な地形だけを拾いたい場合)。
    pub exclude_dynamic: bool,
    /// 明示的に除外するボディ。**世代も一致した場合のみ除外する**——削除後に
    /// index が再利用された別のボディを巻き添えにしないため(`BodyId`が
    /// 世代付きである理由そのもの)。
    pub exclude: Vec<BodyId>,
    /// 衝突グループのマスク。`Some(m)`なら`m & body.collision_group != 0`の
    /// ボディだけを拾う。`None`(既定)は絞らない。
    pub collision_mask: Option<u32>,
}

/// `raycast`の結果(`raycast`モジュールdoc参照)。生の`RigidBodySet`indexではなく
/// 世代付き`BodyId`を返す(削除済み剛体の再利用indexと取り違えないため)。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    pub body: BodyId,
    pub point: Vec3,
    pub normal: Vec3,
    pub distance: f64,
}

/// `World::sample_fluid`の結果(設計docs/20-integration/04-world-api.md §2)。
/// 縮約実装の理由は`sample_fluid`のdoc参照(SPHは温度場を持たないため`temperature`は
/// 対象外)。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FluidSample {
    pub velocity: Vec3,
    pub pressure: f64,
}

/// `Probe`が毎stepサンプルする観測対象(設計docs/20-integration/04-world-api.md §2.1
/// `ProbeTarget`)。
///
/// **縮約実装の理由**: 設計の例示(`BodyPosY`・`Bodyspeed`・`NodeTemp`・
/// `CircuitCurrent`・`LedgerKinetic`・`StateHashDigest`)のうち、`NodeTemp`は
/// `NodeId`型が未整備なため熱ドメインの`ThermalNode`indexへ、`CircuitCurrent`は
/// `CircuitId`型が未整備なため回路の電圧源indexへ、それぞれ縮約する(いずれも
/// 現時点で`World`が単一の熱/回路ドメインしか保持しないため実害はない)。
/// `LedgerKinetic`はエネルギー台帳自体が種別別の内訳を持たないため、
/// `mechanics`ドメインの運動エネルギー(`EnergyBreakdown::kinetic`)と解釈する。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ProbeTarget {
    BodyPosY(BodyId),
    /// 設計の例示には無い項目——D11(振り子と時計)のように、鉛直位置だけでは
    /// 振れ角(周期の判定に使う量)を再構成できないシナリオに対応するため追加。
    BodyPosX(BodyId),
    BodySpeed(BodyId),
    /// 熱ドメインの`ThermalNode`index(モジュールdoc「縮約実装の理由」参照)。
    NodeTemp(usize),
    /// 天体ドメイン(`sim_astro::NBodySystem`)の`position`配列index(D34太陽系儀
    /// のような軌道半径の再構成に使う、`BodyPosX`と同じ理由で追加)。
    AstroPosX(usize),
    AstroPosY(usize),
    /// `sim_astro::NBodySystem`の`velocity`配列index(D35軌道投入のような
    /// 「出発点(位置・速度とも)へ戻る」判定に使う、`position`版と対称に追加)。
    AstroVelX(usize),
    AstroVelY(usize),
    /// 回路の電圧源index(モジュールdoc「縮約実装の理由」参照)。
    CircuitCurrent(usize),
    /// **増分Hで追加した4ドメインの観測量**。ここまで`ProbeTarget`は力学・熱ノード・
    /// 天体・回路しか観測できず、`soft_body`/`grid_fluid`/`conduction_rod`/`sph`は
    /// **シーンに載せても一切グラフに出せなかった**。Scene Viewに何も描かれない
    /// ドメインではProbe Graphsが唯一の観測手段なので、これが無いとギャラリーに
    /// 出す意味自体が無い。
    ///
    /// ソフトボディの粒子index。
    SoftBodyPosX(usize),
    SoftBodyPosY(usize),
    /// 1D熱伝導棒の格子点index。
    RodTemp(usize),
    /// 格子流体の平均鉛直速度(D15対流の「上昇流が立つ」の観測量そのもの)。
    GridFluidMeanV,
    /// 格子流体の鉛直速度のRMS。**D14(渦)にはこちらが要る**——一様な水平流を
    /// 障害物が乱すと上下対称に渦が立つため`GridFluidMeanV`は打ち消し合って0の
    /// ままになる(実測で確認)。RMSなら符号が消えるので擾乱の大きさが見える。
    GridFluidRmsV,
    /// SPH粒子index。
    SphParticlePosY(usize),
    /// SPH粒子index の密度(F10ダム崩壊・D23注ぐ水の非圧縮性の目安)。
    SphParticleDensity(usize),
    /// 回路のノードindex(`circuit_probe(node)`と同じ量をプローブ履歴に積む、
    /// **増分G2で追加**)。D19(電気工作台)は分圧比・RC放電の指数減衰・LED分岐の
    /// スイッチ開閉という**3つとも節点電圧で観測する現象**であり、電流だけを見る
    /// `CircuitCurrent`では合格基準(E5/E3)のどれも再構成できなかった。
    CircuitNodeVoltage(usize),
    /// **群3で追加した6ドメインの観測量**。量子(波動関数)・統計(スピン格子・
    /// 分子集合)・FDTD(場)はいずれも Scene View に直接の3D表現を持たないため、
    /// **Probe Graphs が主要な観測手段**になる。
    ///
    /// 量子1D: 全確率(ユニタリなら 1 で一定 = Q1 の検証量そのもの)。
    QuantumNorm,
    QuantumMeanX,
    QuantumEnergy,
    /// 格子インデックス `i` 以降に存在する確率(トンネル効果の透過率、D27)。
    QuantumTransmission(usize),
    GasTemperature,
    GasPressure,
    IsingMagnetization,
    IsingEnergyPerSpin,
    /// ブラウン粒子の平均二乗変位(原点からの、S4 の検証量)。
    BrownianMsd,
    FdtdEz(usize, usize),
    FdtdEnergy,
    LedgerKinetic,
    /// `state_hash()`をグラフ表示用に`f64`へ変換した値(厳密な数値変換ではなく、
    /// UI上でハッシュの変化を視覚化するためのダイジェスト、設計§2.1「UIのグラフ」)。
    StateHashDigest,
}

/// ジョイントの種別(**群1で追加**、`JointInfo`のdoc参照)。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    Distance,
    Ball,
    Slider,
    Wheel,
    HingeMotor,
}

impl JointKind {
    pub fn name(self) -> &'static str {
        match self {
            JointKind::Distance => "DistanceJoint",
            JointKind::Ball => "BallJoint",
            JointKind::Slider => "SliderJoint",
            JointKind::Wheel => "WheelJoint",
            JointKind::HingeMotor => "HingeMotorPd",
        }
    }
}

/// ジョイント1件の内省情報(**群1で追加**)。
///
/// **これが無かった間の縮約**: フロントエンドはジョイントを
/// `constraint_anchor_points_at`(スポーン時に覚えた`constraint_joint_index`から
/// アンカー2点を返すだけ)でしか見られず、**種別も接続先も制限もモータ設定も
/// 取り出せなかった**。設計 docs/23-frontend/01-editor.md §1.3 の Joint
/// コンポーネントは「種別(Ball/Hinge/Slider/…)・接続 Body ID・軸・制限・モータ」を
/// 要求している。`MechanicsSolver`は4種のジョイントを別々の`Vec`で保持しているので、
/// ここで種別タグを付けて1本の列挙へまとめる。
#[derive(Clone, Debug)]
pub struct JointInfo {
    /// 種別ごとの`Vec`内のindex(`kind`と組で一意)。
    pub index: usize,
    pub kind: JointKind,
    pub body_a: usize,
    /// `None`ならワールド固定点への拘束。
    pub body_b: Option<usize>,
    pub anchor_a: Vec3,
    pub anchor_b: Vec3,
    /// 軸を持つ種別(Slider/HingeMotor/Wheel、Wheelは操舵反映前の`axle_axis`)
    /// のみ`Some`。
    pub axis: Option<Vec3>,
    /// `DistanceJoint`の拘束長、または`WheelJoint`のサスペンション自然長。
    pub length: Option<f64>,
    /// モータの目標角(`HingeMotorPd`)、または`WheelJoint`のモータ角速度
    /// (`motor_max_torque > 0`の時のみ`Some`、駆動なしの車輪では`None`)。
    pub motor_target: Option<f64>,
    /// 無効化されているか(`BallJoint::disabled`)。
    pub disabled: bool,
}

/// 結合1件の内省情報(**群1で追加**、`World::couplings`のdoc参照)。
///
/// 設計 docs/23-frontend/01-editor.md §1.3 の Coupling コンポーネントが要求する
/// 「種別・関連する Body/Fluid/Circuit 参照」をそのまま表現する。
#[derive(Clone, Debug)]
pub struct CouplingInfo {
    /// 登録順のindex(`World::couplings`の並びと一致)。
    pub index: usize,
    pub kind: sim_coupling::CouplingKind,
    /// パラメータ込みの人間可読表現。
    pub description: String,
    /// この結合が跨るドメイン。
    pub domains: &'static [sim_core::DomainId],
    /// 読み書きする剛体index。
    pub bodies: Vec<usize>,
    /// 読み書きする熱ノードindex。
    pub thermal_nodes: Vec<usize>,
    /// 読み書きする電圧源index。
    pub voltage_sources: Vec<usize>,
}

/// 任意の観測量を毎stepサンプルして`history`に積む軽量プローブ
/// (設計docs/20-integration/04-world-api.md §2.1「測って遊ぶの中心機能」)。
///
/// **履歴は可変長(切り詰めない)**。以前は固定容量の`RingBuffer`
/// (`DEFAULT_PROBE_CAPACITY`、600 → QA不具合9で6000へ拡大)で、容量を超えると
/// **無言で先頭が捨てられて**いた。窓を広げてもこれは先延ばしにしかならない
/// ——長時間走らせれば必ず当たるし、当たったことは呼び出し側に一切通知されず、
/// 「グラフの左端が実は0秒ではない」という形で静かに誤読を生む。
/// **測ったデータを黙って捨てない**ほうを不変条件に選び、上限を外した。
///
/// 代わりに使用量を問い合わせる口を用意してある(`Probe::len`・
/// `World::probe_history_bytes_estimate`)。呼び出し側(将来のフロントエンド)は
/// これを見て自分で決めた軟らかい上限に近づいたら警告を出せる——
/// **こちら側で勝手に上限を課したり捨てたりはしない**。
#[derive(Clone)]
pub struct Probe {
    pub target: ProbeTarget,
    history: Vec<f64>,
}

impl Probe {
    /// 古い順(サンプル順)の観測履歴。
    pub fn history(&self) -> impl Iterator<Item = &f64> {
        self.history.iter()
    }

    /// 蓄積済みサンプル数(= このプローブが登録されてからの`step()`回数)。
    /// `World::probe_history_bytes_estimate`の内訳を見たい呼び出し側が使う。
    pub fn len(&self) -> usize {
        self.history.len()
    }

    pub fn is_empty(&self) -> bool {
        self.history.is_empty()
    }
}

/// シミュレートされた環境そのもの。世界時刻の一意性は `clock`
/// (docs/00-foundation/04-architecture.md §1.1.2(4))、状態オーナーシップの一意性は
/// `mechanics`(正典状態)が保持することで満たす(同 §1.1.2(1))。
///
/// `Clone`を導出できるのは、全フィールド(`mechanics`・`thermal`等の各ドメインソルバ、
/// `materials`・`rng`・`events`・`ledger`・`generations`)が既にClone可能なため
/// (このワークストリームBの増分で各ドメインcrateに`#[derive(Clone)]`を追加した)。
/// `snapshot`/`restore`(設計docs/20-integration/04-world-api.md §2、
/// docs/20-integration/02-determinism-replay.md §6「スナップショット再開時の
/// リプレイ一致」)はこの`Clone`実装をそのまま使う縮約実装 — 差分スナップショット
/// (メモリ効率化)は後続増分。
#[derive(Clone)]
pub struct World {
    clock: sim_core::SimClock,
    mechanics: MechanicsSolver,
    /// 熱ドメイン(モジュールdoc「全ドメイン合成」参照、シーンが使う場合のみ`Some`)。
    thermal: Option<sim_thermal::ThermalSolver>,
    /// 電磁気ドメイン(静電、モジュールdoc参照、シーンが使う場合のみ`Some`)。
    em_electrostatics: Option<sim_em::PointChargeSystem>,
    /// 天体ドメイン(モジュールdoc参照、シーンが使う場合のみ`Some`)。
    astro: Option<sim_astro::NBodySystem>,
    /// 回路ドメイン(モジュールdoc参照、シーンが使う場合のみ`Some`)。
    circuit: Option<sim_em::Circuit>,
    /// 気体区画ドメイン(`sim_coupling::PistonGas`が読み書きする、シーンが使う場合のみ
    /// `Some`)。**これは今も`Solver`未実装で正しい**——`GasCompartment`は
    /// 状態変数(モル数・体積・温度)を持つだけで自律的な時間発展を持たず、
    /// ピストンの動きに追従して`apply_coupling`が更新する従属量だからである
    /// (`conduction_rod`/`soft_body`とは事情が違う。あちらは自律的に発展する
    /// のに`Solver`が無かっただけで、増分Hで実装した)。
    gas: Option<sim_thermal::GasCompartment>,
    /// 1D格子熱伝導ドメイン(D16「熱伝導レース」が使う、シーンが使う場合のみ`Some`)。
    /// **増分Hで`Solver`を実装したので`step()`が自動的にsub-stepする**
    /// (それまでは`conduction_rod_mut().step(dt)`を呼び出し側が明示的に
    /// 呼ぶ必要があり、シーンに載せても再生では一切動かなかった)。
    conduction_rod: Option<sim_thermal::ConductionRod1D>,
    /// SPH流体ドメイン(シーンが使う場合のみ`Some`)。`sim_fluid::SphFluid`に
    /// `Solver`トレイトを実装済み(モジュールdoc「全ドメイン合成」参照)のため、
    /// `thermal`/`em_electrostatics`/`astro`/`circuit`と同じ固定順で`step()`が
    /// 自動的にsub-stepする。
    sph: Option<sim_fluid::SphFluid>,
    /// 格子(Eulerian)流体ドメイン(シーンが使う場合のみ`Some`)。`sim_fluid::GridFluid2D`に
    /// `Solver`トレイトを実装済み(モジュールdoc「全ドメイン合成」参照)のため、`sph`と
    /// 同じ固定順で`step()`が自動的にsub-stepする。
    grid_fluid: Option<sim_fluid::GridFluid2D>,
    /// 3D格子流体(**群9で追加**、`sim_fluid::GridFluid3D`)。固定順序では
    /// `grid_fluid`(2D)の直後に置く。
    grid_fluid_3d: Option<sim_fluid::GridFluid3D>,
    /// ソフトボディ(XPBDロープ、D13「ロープと旗」が使う、シーンが使う場合のみ`Some`)。
    /// **増分Hで`Solver`を実装したので`step()`が自動的にsub-stepする**——
    /// `step`が複数引数(`dt, gravity, n_sub, n_iter, damping`)を取り
    /// `Solver::step(dt, ctx)`のシグネチャに素直に適合しなかったのを、
    /// 積分設定を`SoftBody`のフィールドへ移すことで解消した。
    /// **群3で描画も繋いだ**——それまで`RigidBodySet`の剛体ではないため
    /// Scene Viewに一切現れず、Probe Graphsが唯一の観測手段だった。
    soft_body: Option<sim_mechanics::SoftBody>,
    /// **量子ドメイン(群3で追加)**。設計 docs/00-foundation/04-architecture.md §1.2 は
    /// 量子を7ドメインの1つとして数えているのに、`sim-quantum`は`Solver`を実装しておらず
    /// **`World`に載る経路が原理的に存在しなかった**——D27–D29(トンネル効果・二重
    /// スリット・調和振動子)がシーンギャラリーに出せなかった直接の原因。
    /// 群3で`WaveFunction1D`/`WaveFunction2D`に`Solver`を実装して接続した。
    ///
    /// 1Dと2Dを**別フィールドで持つ**のは、両者が別の型で相互変換できないため
    /// (enumで1つにまとめると、1D用のプローブから2Dの状態を読む経路が塞がる)。
    quantum_1d: Option<sim_quantum::WaveFunction1D>,
    quantum_2d: Option<sim_quantum::WaveFunction2D>,
    /// **統計ドメイン(群3で追加)**。`quantum_*`と同じ理由。D25/D30/D31が対象。
    brownian: Option<sim_statistical::BrownianParticleSet>,
    kinetic_gas: Option<sim_statistical::GasSim>,
    ising: Option<sim_statistical::IsingSim>,
    /// **FDTD(電磁波)ドメイン(群3で追加)**。`PointChargeSystem`(静電)と
    /// `Circuit`(回路)は既に`Solver`を実装済みだったが、**波動だけ未実装**で
    /// `World`に載らなかった(D32「電磁波の伝播」が出せなかった原因)。
    fdtd: Option<sim_em::FdtdSim2D>,
    /// 直近の`step()`で、いずれかのドメインが sub-step 数の上限
    /// (`orchestrator::MAX_SUB_STEPS_PER_FRAME`)に当たったか(**群3で追加**)。
    /// 当たったフレームはそのドメインが**要求された時間ぶん正しくは進んでいない**
    /// ので、`active_approximations()`が申告する。
    sub_step_cap_reached: bool,
    materials: MaterialDb,
    rng: SimRng,
    events: EventQueue,
    /// 最初の `step()` で遅延初期化する(構築フェーズの `create_body` を
    /// 台帳の基準点計算に含めないため)。
    ledger: Option<EnergyLedger>,
    /// `BodyId` の世代管理(`RigidBodySet` のインデックスに対応、モジュールdoc参照)。
    generations: Vec<u32>,
    /// `remove_body`で削除済みのスロットか(`generations`と対、`body_ids()`が
    /// 生存ボディだけを列挙するのに使う)。`RigidBodySet`のスロット自体は
    /// `remove_body`のdoc参照の通り解放されない(`BodyType::Static`化+遠方退避)ため、
    /// 座標だけからは削除済みかどうかを判別できない——`World → Scenario`逆写像
    /// (`export`モジュール)が削除済みの亡霊ボディを書き出さないために追加した。
    removed: Vec<bool>,
    /// `push_command`で積まれ、次`step()`の先頭で適用されるコマンドの待ち行列
    /// (`Command`のdoc参照)。
    pending_commands: Vec<Command>,
    /// 適用済みコマンドの記録(`step_count`と対、リプレイ検証用、設計§2「記録されリプレイ
    /// 可能」)。
    command_log: Vec<(u64, Command)>,
    /// 登録済みプローブ(`Probe`のdoc参照)。`step()`末尾で毎step全プローブをサンプルする。
    probes: Vec<Probe>,
    /// `Command::Grab`が作った`BallJoint`の、剛体index→`mechanics.ball_joints`
    /// indexの対応(`Command::MoveGrab`/`Release`が同じ剛体を再度参照するために使う、
    /// `Command`のdoc参照)。1剛体につき同時に1つのgrabのみを想定する(再`Grab`は
    /// 前のgrabを`disabled`化してから新設)。
    grab_joints: std::collections::HashMap<u32, usize>,
    /// `step()`が排出した全イベントの履歴(`drain_events`のdoc参照)。
    event_log: sim_math::RingBuffer<sim_core::Event>,
    /// `add_coupling`で登録され、`step()`が毎フレーム自動的に`.apply()`を呼ぶ
    /// `Coupling`のレジストリ(`apply_coupling`のdoc参照。登録順=決定論的な適用順)。
    couplings: Vec<Box<dyn sim_coupling::Coupling>>,
    /// 時間加速の2レジーム(設計docs/20-integration/06-regime-switching.md §2、
    /// `sim_astro::TimeRegime`をそのまま使う)。既定`Local`は「これまでの`step()`の
    /// 挙動(有効な全ドメインを毎stepまとめて進める)」と完全に一致するため、この
    /// フィールド追加自体は既存シナリオの挙動を一切変えない。`Astro`に切り替えると
    /// `astro`ドメインのみを独立時間軸`dt_astro`で進め、他の全ドメイン(`mechanics`
    /// 含む)は凍結する(設計§1「天体レンジ…ローカル物理ソルバは停止」)。
    ///
    /// **縮約実装の理由**: 設計が定めるプロトコル全体(切替のCommand化・ヒステリシス付き
    /// 自動切替・World時刻の天体時刻への従属化・Astro中のスナップショット間隔の天体時間
    /// 基準化・切替を跨ぐリプレイ一致のCIゲート)はここでは実装しない。本増分は
    /// 「レジームがWorldの状態として存在し、`step()`がそれに応じて正しくドメインの
    /// 進行/凍結を分岐する」という土台のみを提供する(`set_time_regime`は直接呼び出し、
    /// Command経由ではない)。`Local`の`steps_per_frame`(通常レンジの時間倍率)も
    /// まだ`step()`から参照されない(フロントエンド側の`MAX_STEPS_PER_FRAME`相当の
    /// responsibilityは既に呼び出し側にあるため)。
    time_regime: sim_astro::TimeRegime,
    /// フレーム階層(設計docs/20-integration/05-frame-hierarchy.md、`sim_core::FrameTree`の
    /// doc参照)。`World`は常にROOTのみの木を持つ(空のシーンでも`Default`が使える)。
    /// 自動レジーム切替(`auto_regime_switch`)が地表フレームをここに追加する。
    frames: sim_core::FrameTree,
    /// 閾値ベースの自動レジーム切替設定(設計docs/20-integration/06-regime-switching.md §1
    /// 「天体→局所への切替は、中心天体からの距離が閾値を下回った時点で自動的に発生する」
    /// の縮約実装、`configure_auto_regime_switch`のdoc参照)。`Some`の間、`step()`の
    /// Astroレジーム分岐が毎step終端でこの設定を見て閾値判定する。切替が起きると`None`に
    /// 戻し(1回のみ発火、再トリガ防止)、以後は通常のLocalレジーム挙動に戻る。
    auto_regime_switch: Option<AutoRegimeSwitchConfig>,
    /// **著者向けメタデータ**(`Scenario::prediction_prompts`/`pass_criteria`、
    /// `author_metadata`アクセサのdoc参照)。**物理には一切影響しない**——
    /// `step()`も`state_hash()`もこの2つに触れない。シーンJSONから読み込んだ値を
    /// そのまま抱えておくためだけの場所である。
    prediction_prompts: Vec<scenario::PredictionPromptJson>,
    pass_criteria: Vec<scenario::PassCriterionJson>,
}

/// `World::configure_auto_regime_switch`が使う自動切替の設定(モジュールdoc「全ドメイン
/// 合成」・`auto_regime_switch`フィールドdoc参照)。
///
/// **縮約実装の理由**: 設計が定めるヒステリシス(往復切替の抑制)・切替のCommand化・
/// 切替を跨いだリプレイ一致のCIゲートはここでは実装しない。本増分は「天体レンジ中に
/// 中心天体からの距離を毎step監視し、閾値を下回った瞬間に既存の手動ハンドオフ手順
/// (`switching_from_astro_to_local_hands_off_orbital_state_via_frame_conversion`と
/// 同じフレーム変換)を`World::step()`内部で自動的に実行する」という土台のみを提供する。
#[derive(Clone, Copy, Debug)]
pub struct AutoRegimeSwitchConfig {
    /// `astro`ドメイン(`sim_astro::NBodySystem`)内での、切替対象ボディのインデックス。
    pub astro_body_index: usize,
    /// 距離判定の基準にする中心天体の`astro`ドメイン内インデックス。
    pub central_body_index: usize,
    /// この距離(中心天体からの距離)を下回るとLocalへ切り替える。
    pub threshold_distance: f64,
    /// フレーム変換先の地表フレーム(`World::add_frame`で事前に`frames`へ追加しておく)。
    pub surface_frame: sim_core::FrameId,
    /// 変換後の状態を書き込む、あらかじめ`create_body`で作成済みのLocalボディ。
    pub local_body: BodyId,
}

/// `event_log`の容量(設計は`subscribe`/`drain_events`の容量を規定しないため、
/// UIが1フレームで捌ける件数の同オーダーの値を採用)。**`Probe`の履歴とは
/// 事情が違う**——あちらは「測ったデータ」なので切り詰めをやめた(`Probe`の
/// doc参照)が、こちらは通知であり、消費されなかった古い通知を無限に
/// 溜め続ける意味は無い。
const EVENT_LOG_CAPACITY: usize = 1024;

const STREAM_DIAG: u64 = 0;
/// エネルギー台帳の代表エネルギー(ゼロ初期エネルギー対策の下限)。設計
/// docs/21-verification/02-conservation-laws.md §2 の E_scale。シーンごとの代表値を求める
/// API はまだ無いため、P1 では固定値 1 J とする(将来シーン記述に応じて拡張)。
const ENERGY_SCALE_FLOOR: f64 = 1.0;

/// 1ドメインをOrchestratorの決定的sub-step数(`orchestrator::sub_step_count`)に従って
/// フレームdt分進める。フィールドを個別の引数として受け取ることで、呼び出し側で
/// `&mut self.<domain>` と `&mut self.rng`/`&mut self.events` の disjoint borrow が
/// 同時に成立する(構造体メソッド越しだと借用チェッカに見えなくなるため、あえて自由関数
/// にしている)。
/// 1ドメインを`frame_dt`ぶん進める(必要なら`max_stable_dt`以下へ均等分割)。
///
/// **戻り値は「sub-step数の上限で打ち切ったか」**(群3で追加、
/// `orchestrator::MAX_SUB_STEPS_PER_FRAME`のdoc参照)。打ち切った場合は
/// そのドメインは要求された時間ぶん**正しくは進んでいない**ので、
/// `World`が近似バッジとして申告する。
fn run_domain_substeps<S: Solver>(
    solver: &mut S,
    frame_dt: f64,
    materials: &MaterialDb,
    rng: &mut SimRng,
    events: &mut EventQueue,
) -> bool {
    let (n, capped) = orchestrator::sub_step_count_capped(frame_dt, solver.max_stable_dt());
    let sub_dt = orchestrator::sub_step_dt(frame_dt, n);
    for _ in 0..n {
        let mut ctx = SolverContext {
            materials,
            rng: &mut *rng,
            events: &mut *events,
        };
        solver.step(sub_dt, &mut ctx);
    }
    capped
}

/// UIのAdd Component(設計docs/23-frontend/01-editor.md §1.3)や
/// `sim-wasm`のschema/read/apply(統合エディタ実装計画、縦串①)が任意の
/// ジョイントを追加するための記述(設計 docs/20-integration/04-world-api.md
/// §2 `create_joint(desc: JointDesc) -> JointId`)。`scenario::JointJson`と
/// 同じ5種だが、名前解決を経ず`BodyId`を直接使う点が違う——シーンJSON読み込み
/// (`from_scenario`)は文字列名からボディを引く必要があるが、実行中の
/// Inspector・wasm境界は既に`BodyId`を持っているため、名前解決を経由させる
/// 必要が無い。
#[derive(Clone, Debug)]
pub enum JointDesc {
    Distance {
        body_a: BodyId,
        anchor_a: Vec3,
        body_b: Option<BodyId>,
        anchor_b: Vec3,
        length: f64,
    },
    Ball {
        body_a: BodyId,
        anchor_a: Vec3,
        body_b: Option<BodyId>,
        anchor_b: Vec3,
    },
    Slider {
        body_a: BodyId,
        anchor_a: Vec3,
        axis: Vec3,
        body_b: Option<BodyId>,
        anchor_b: Vec3,
    },
    Wheel {
        chassis: BodyId,
        wheel: BodyId,
        anchor_chassis: Vec3,
        rest_length: f64,
        suspension_axis: Vec3,
        axle_axis: Vec3,
        frequency: f64,
        damping_ratio: f64,
        steer_angle: f64,
        motor_speed: f64,
        motor_max_torque: f64,
    },
    HingeMotor {
        body: BodyId,
        axis: Vec3,
        /// `None`なら現在の`body`の姿勢を基準に取る(`HingeMotorPd::new`と同じ)。
        reference_rotation: Option<sim_math::Quat>,
        theta_target: f64,
        kp: f64,
        kd: f64,
        torque_max: f64,
        limit: Option<(f64, f64)>,
    },
}

/// 環境(重力・大気・水域・周囲温度)をまとめて記述する(設計
/// docs/20-integration/04-world-api.md §2「重力ベクトル・大気・水域・
/// 周囲温度を`EnvironmentDesc`として第一級にする」)。
///
/// **レビュー指摘(「見送らず対応すること」)を受けて重力の向き
/// (`gravity_direction`)も追加した**——`sim_mechanics::MechanicsSolver::
/// gravity_direction`のdocが示すとおり、影響範囲は自由体への直接の重力
/// 積分とポテンシャルエネルギー計算のみ(浮力・大気抗力は向きに依存しない、
/// `sim-fluid`crateの水面モデルが水平面固定であることに由来する既存の
/// 制約で、本増分の対象外)。
///
/// **重力場の抽象化増分**: `gravity_field`を追加した。`Some`ならそちらが
/// 優先され、`None`なら従来どおり`gravity`+`gravity_direction`から一様場を
/// 組み立てる——シーンJSONの`world.gravity_field`と**まったく同じ優先規則**
/// (`scenario::WorldScenarioOptions::gravity_field`参照)。表現が二重になるが、
/// 既存の呼び出し側が`gravity`/`gravity_direction`だけを読み書きし続けられる
/// ことを優先した。`environment()`は常に`gravity_field: Some(..)`を埋めるので、
/// 読んでそのまま書き戻す往復では非一様な場も無損失で保たれる。
///
/// **流体領域の一般化に伴い`Copy`ではなくなった**——`fluids`が`Vec`になったため。
#[derive(Clone, Debug)]
pub struct EnvironmentDesc {
    /// 重力の大きさ [m/s^2](非`Uniform`な場では0.0、
    /// `sim_mechanics::MechanicsSolver::gravity`のdoc参照)。
    /// `gravity_field`が`Some`のときは無視される。
    pub gravity: f64,
    /// 重力の向き(単位ベクトルとして正規化される、既定は下向き`(0,-1,0)`)。
    /// `gravity_field`が`Some`のときは無視される。
    pub gravity_direction: sim_math::Vec3,
    /// 重力場そのもの(`sim_mechanics::GravityField`)。`Some`なら
    /// `gravity`/`gravity_direction`より優先する(構造体docの優先規則)。
    pub gravity_field: Option<sim_mechanics::GravityField>,
    pub atmosphere: Option<sim_fluid::Atmosphere>,
    /// 流体領域一覧(`sim_mechanics::MechanicsSolver::fluids`をそのまま写す)。
    /// **移行前は`water: Option<StaticWaterRegion>`**で、水域は高々1つだった。
    /// `set_environment`はこの`Vec`で丸ごと置き換える(空なら水域なし)。
    pub fluids: Vec<sim_fluid::FluidRegion>,
    /// `None`なら熱ドメインが無効(`enable_thermal`未呼び出し)、または
    /// 変更しない。
    pub ambient_temperature: Option<f64>,
}

impl World {
    pub fn new(options: WorldOptions) -> World {
        World {
            clock: sim_core::SimClock::new(options.dt),
            mechanics: MechanicsSolver::new(options.gravity),
            thermal: None,
            em_electrostatics: None,
            astro: None,
            circuit: None,
            gas: None,
            conduction_rod: None,
            sph: None,
            grid_fluid: None,
            grid_fluid_3d: None,
            soft_body: None,
            quantum_1d: None,
            quantum_2d: None,
            brownian: None,
            kinetic_gas: None,
            ising: None,
            fdtd: None,
            sub_step_cap_reached: false,
            materials: MaterialDb::standard(),
            rng: SimRng::new(options.seed, STREAM_DIAG),
            events: EventQueue::new(),
            ledger: None,
            generations: Vec::new(),
            removed: Vec::new(),
            pending_commands: Vec::new(),
            command_log: Vec::new(),
            probes: Vec::new(),
            grab_joints: std::collections::HashMap::new(),
            event_log: sim_math::RingBuffer::new(EVENT_LOG_CAPACITY),
            couplings: Vec::new(),
            time_regime: sim_astro::TimeRegime::Local { steps_per_frame: 1 },
            frames: sim_core::FrameTree::new(),
            auto_regime_switch: None,
            prediction_prompts: Vec::new(),
            pass_criteria: Vec::new(),
        }
    }

    /// 予測→実験ミニパネルのヒント(`Scenario::prediction_prompts`)。
    ///
    /// **なぜ`World`が持つのか**: この2つ(と`pass_criteria`)は長らく
    /// 「`from_scenario`は読まない・`to_scenario`は常に空を返す」だった。
    /// つまり**エディタでシーンを保存するたびに消えていた**——手で
    /// `pass_criteria`を書いたシーンを読み込み、`export_scene_json`で書き戻すと
    /// 検証タブの合格基準が丸ごと落ちる。物理に影響しないことと、
    /// 実行時状態として保持しなくてよいことは別の話である。
    ///
    /// **物理から完全に隔離されている**: `step()`はこのフィールドを読まないし、
    /// `state_hash()`にも混ざらない(決定論replayに影響しない)。`Clone`
    /// (=`snapshot`/`restore`)には素直について回る。
    pub fn prediction_prompts(&self) -> &[scenario::PredictionPromptJson] {
        &self.prediction_prompts
    }

    /// 検証タブの合格基準(`Scenario::pass_criteria`)。`prediction_prompts`の
    /// doc参照(同じ扱い)。
    pub fn pass_criteria(&self) -> &[scenario::PassCriterionJson] {
        &self.pass_criteria
    }

    /// 著者向けメタデータを差し替える(`prediction_prompts`のdoc参照)。
    /// `World::append_scenario_bodies`がシーンJSONから読んだ値を入れるほか、
    /// エディタが編集した結果を書き戻す口としても使う。
    pub fn set_author_metadata(
        &mut self,
        prediction_prompts: Vec<scenario::PredictionPromptJson>,
        pass_criteria: Vec<scenario::PassCriterionJson>,
    ) {
        self.prediction_prompts = prediction_prompts;
        self.pass_criteria = pass_criteria;
    }

    /// 著者向けメタデータを末尾へ追加する(`append_scenario_bodies`が使う——
    /// あちらは実行中ワールドへの「追加」なので、既存のシーンのメタデータを
    /// 消してはならない)。
    ///
    /// `probe_index`は**取り込むシーンの`probes`配列内の位置**なので、
    /// 既に登録済みのプローブ本数(`probe_offset`)ぶんずらしてから積む——
    /// `add_scenario_probes`が同じオフセットで末尾へ追加するため、これで
    /// 両者の指す先が一致する。新規構築(`from_scenario`、プローブ0本)なら
    /// オフセット0で恒等になり、往復は無損失である。
    pub fn append_author_metadata(
        &mut self,
        prediction_prompts: &[scenario::PredictionPromptJson],
        pass_criteria: &[scenario::PassCriterionJson],
        probe_offset: usize,
    ) {
        for p in prediction_prompts {
            self.prediction_prompts
                .push(scenario::PredictionPromptJson {
                    question: p.question.clone(),
                    probe_index: p.probe_index + probe_offset,
                    expected_value: p.expected_value,
                });
        }
        for c in pass_criteria {
            self.pass_criteria.push(scenario::PassCriterionJson {
                probe_index: c.probe_index + probe_offset,
                operator: c.operator,
                threshold: c.threshold,
            });
        }
    }

    /// 登録済みプローブの本数(`append_author_metadata`の`probe_offset`に使う)。
    pub fn probe_count(&self) -> usize {
        self.probes.len()
    }

    /// 現在のレジーム(設計docs/20-integration/06-regime-switching.md §2)。
    pub fn time_regime(&self) -> sim_astro::TimeRegime {
        self.time_regime
    }

    /// レジームを切り替える(`time_regime`フィールドdoc参照。直接呼び出しであり、
    /// Command経由での記録・リプレイは未実装——設計が求める「加速率の変更はコマンド」
    /// (§1)自体は後続増分)。
    pub fn set_time_regime(&mut self, regime: sim_astro::TimeRegime) {
        self.time_regime = regime;
    }

    /// フレーム階層(`frames`フィールドdoc参照)への読み取りアクセス。
    pub fn frames(&self) -> &sim_core::FrameTree {
        &self.frames
    }

    /// `frames`に新規フレームを追加する(`sim_core::FrameTree::add_frame`の素通し、
    /// 自動レジーム切替が地表フレームを追加するために使う)。
    #[allow(clippy::too_many_arguments)]
    pub fn add_frame(
        &mut self,
        parent: sim_core::FrameId,
        origin_in_parent: Vec3,
        rotation_in_parent: sim_math::Quat,
        velocity_in_parent: Vec3,
        angular_velocity_in_parent: Vec3,
    ) -> sim_core::FrameId {
        self.frames.add_frame(
            parent,
            origin_in_parent,
            rotation_in_parent,
            velocity_in_parent,
            angular_velocity_in_parent,
        )
    }

    /// 閾値ベースの自動レジーム切替を設定する(`AutoRegimeSwitchConfig`のdoc参照)。
    /// `config.local_body`は呼び出し側があらかじめ`create_body`で作成しておく必要がある
    /// (切替発火時に`World::step()`内部から新規`create_body`を呼ぶのは、既存の手動
    /// ハンドオフ手順が示す「シーン構築はcreate_body、実行中の状態変更はCommand/内部
    /// 書き込み」という役割分担から外れるため避けた)。
    pub fn configure_auto_regime_switch(&mut self, config: AutoRegimeSwitchConfig) {
        self.auto_regime_switch = Some(config);
    }

    /// Astroレジーム中、毎step終端で中心天体からの距離を判定し、閾値を下回っていれば
    /// 既存の手動ハンドオフ手順と同じフレーム変換でLocalボディへ状態を書き込み、
    /// レジームをLocalへ切り替える(`auto_regime_switch`フィールドdoc参照)。
    fn check_auto_regime_switch(&mut self) {
        let Some(config) = self.auto_regime_switch else {
            return;
        };
        let Some(astro) = &self.astro else {
            return;
        };
        let central_position = astro.position[config.central_body_index];
        let orbital_position = astro.position[config.astro_body_index];
        let orbital_velocity = astro.velocity[config.astro_body_index];
        if (orbital_position - central_position).length() > config.threshold_distance {
            return;
        }

        let (local_position, local_velocity) = sim_astro::astro_to_local_state(
            &self.frames,
            config.surface_frame,
            orbital_position,
            orbital_velocity,
        );

        if self.is_valid(config.local_body) {
            let index = config.local_body.index as usize;
            self.mechanics
                .bodies
                .set_origin_position(index, local_position);
            self.mechanics.bodies.linear_velocity[index] = local_velocity;
            self.mechanics.bodies.still_time[index] = 0.0;
            self.mechanics.bodies.asleep[index] = false;
        }
        self.time_regime = sim_astro::TimeRegime::Local { steps_per_frame: 1 };
        self.auto_regime_switch = None;
    }

    /// プローブを登録する(`Probe`のdoc参照)。返すハンドルは`probe`/`probe_history`が
    /// 使う(現時点では単なるベクタindex、`Vec`が縮まないため安定)。
    ///
    /// **容量引数は取らない**(`Probe`のdoc参照)——履歴は可変長で、
    /// 切り詰めは起こらない。
    pub fn add_probe(&mut self, target: ProbeTarget) -> usize {
        self.probes.push(Probe {
            target,
            history: Vec::new(),
        });
        self.probes.len() - 1
    }

    pub fn probe(&self, handle: usize) -> Option<&Probe> {
        self.probes.get(handle)
    }

    /// 指定プローブの蓄積済みサンプル数(`Probe::len`の素通し、
    /// 未登録ハンドルなら`None`)。
    pub fn probe_history_len(&self, handle: usize) -> Option<usize> {
        self.probes.get(handle).map(|p| p.len())
    }

    /// 全プローブ履歴が占めるメモリの概算[byte](`f64`×総サンプル数)。
    ///
    /// **上限を課さない代わりの観測手段**(`Probe`のdoc参照)。
    /// `Vec`の予約分(容量-長さ)は数えない概算だが、桁を誤らせるほどの差は
    /// 出ない(`Vec`の伸長は倍々なので最悪2倍)。呼び出し側はこれを見て
    /// **自分で決めた軟らかい上限**に近づいたら警告できる——
    /// `World`側が勝手に捨てることは無い。
    pub fn probe_history_bytes_estimate(&self) -> usize {
        self.probes.iter().map(|p| p.len()).sum::<usize>() * std::mem::size_of::<f64>()
    }

    /// `target`が指す観測量の現在値を読む(`step()`末尾の毎stepサンプルと同じロジック)。
    /// 対象が無効(削除済み`BodyId`・未有効化ドメインのインデックス範囲外)の場合は`0.0`
    /// (パニックしない、設計の不変条件)。
    fn sample_probe_target(&self, target: ProbeTarget) -> f64 {
        match target {
            ProbeTarget::BodyPosY(id) => self.body_position(id).map_or(0.0, |p| p.y),
            ProbeTarget::BodyPosX(id) => self.body_position(id).map_or(0.0, |p| p.x),
            ProbeTarget::BodySpeed(id) => self.body_velocity(id).map_or(0.0, |v| v.length()),
            ProbeTarget::NodeTemp(idx) => self
                .thermal
                .as_ref()
                .and_then(|t| t.nodes.get(idx))
                .map_or(0.0, |n| n.temperature),
            ProbeTarget::AstroPosX(idx) => self
                .astro
                .as_ref()
                .and_then(|a| a.position.get(idx))
                .map_or(0.0, |p| p.x),
            ProbeTarget::AstroPosY(idx) => self
                .astro
                .as_ref()
                .and_then(|a| a.position.get(idx))
                .map_or(0.0, |p| p.y),
            ProbeTarget::AstroVelX(idx) => self
                .astro
                .as_ref()
                .and_then(|a| a.velocity.get(idx))
                .map_or(0.0, |v| v.x),
            ProbeTarget::AstroVelY(idx) => self
                .astro
                .as_ref()
                .and_then(|a| a.velocity.get(idx))
                .map_or(0.0, |v| v.y),
            ProbeTarget::CircuitCurrent(idx) => {
                self.circuit.as_ref().map_or(0.0, |c| c.source_current(idx))
            }
            ProbeTarget::CircuitNodeVoltage(idx) => self.circuit_probe(idx).unwrap_or(0.0),
            ProbeTarget::SoftBodyPosX(idx) => self
                .soft_body
                .as_ref()
                .and_then(|b| b.position.get(idx))
                .map_or(0.0, |p| p.x),
            ProbeTarget::SoftBodyPosY(idx) => self
                .soft_body
                .as_ref()
                .and_then(|b| b.position.get(idx))
                .map_or(0.0, |p| p.y),
            ProbeTarget::RodTemp(idx) => self
                .conduction_rod
                .as_ref()
                .and_then(|r| r.temperature.get(idx))
                .copied()
                .unwrap_or(0.0),
            ProbeTarget::GridFluidMeanV => self.grid_fluid.as_ref().map_or(0.0, |g| {
                if g.v.is_empty() {
                    0.0
                } else {
                    g.v.iter().sum::<f64>() / g.v.len() as f64
                }
            }),
            ProbeTarget::GridFluidRmsV => self.grid_fluid.as_ref().map_or(0.0, |g| {
                if g.v.is_empty() {
                    0.0
                } else {
                    (g.v.iter().map(|x| x * x).sum::<f64>() / g.v.len() as f64).sqrt()
                }
            }),
            ProbeTarget::SphParticlePosY(idx) => self
                .sph
                .as_ref()
                .and_then(|s| s.position.get(idx))
                .map_or(0.0, |p| p.y),
            ProbeTarget::SphParticleDensity(idx) => self
                .sph
                .as_ref()
                .and_then(|s| s.density.get(idx))
                .copied()
                .unwrap_or(0.0),
            // **群3で追加**。ドメインが無効なら 0 を返す(既存の全プローブと同じ規約
            // ——「無効なドメインを観測したらエラー」ではなく静かに 0)。
            ProbeTarget::QuantumNorm => self.quantum_1d.as_ref().map_or(0.0, |q| q.norm()),
            ProbeTarget::QuantumMeanX => self.quantum_1d.as_ref().map_or(0.0, |q| q.mean_x()),
            ProbeTarget::QuantumEnergy => self.quantum_1d.as_ref().map_or(0.0, |q| q.energy()),
            ProbeTarget::QuantumTransmission(from) => self.quantum_1d.as_ref().map_or(0.0, |q| {
                let total = q.norm();
                if total == 0.0 {
                    return 0.0;
                }
                let tail: f64 = q.psi.iter().skip(from).map(|p| p.norm_sq()).sum::<f64>() * q.dx;
                tail / total
            }),
            ProbeTarget::GasTemperature => self
                .kinetic_gas
                .as_ref()
                .filter(|g| g.particle_count() > 0)
                .map_or(0.0, |g| g.temperature()),
            ProbeTarget::GasPressure => self
                .kinetic_gas
                .as_ref()
                .filter(|g| g.particle_count() > 0)
                .map_or(0.0, |g| g.pressure()),
            ProbeTarget::IsingMagnetization => {
                self.ising.as_ref().map_or(0.0, |i| i.magnetization())
            }
            ProbeTarget::IsingEnergyPerSpin => {
                self.ising.as_ref().map_or(0.0, |i| i.energy_per_spin())
            }
            ProbeTarget::BrownianMsd => self.brownian.as_ref().map_or(0.0, |b| {
                if b.position.is_empty() {
                    0.0
                } else {
                    b.position.iter().map(|p| p.length_sq()).sum::<f64>() / b.position.len() as f64
                }
            }),
            ProbeTarget::FdtdEz(i, j) => self.fdtd.as_ref().map_or(0.0, |f| {
                if i < f.nx() && j < f.ny() {
                    f.ez(i, j)
                } else {
                    0.0
                }
            }),
            ProbeTarget::FdtdEnergy => self.fdtd.as_ref().map_or(0.0, |f| f.total_energy()),
            ProbeTarget::LedgerKinetic => self.mechanics.total_energy().kinetic,
            ProbeTarget::StateHashDigest => self.state_hash() as f64,
        }
    }

    /// コマンドを次`step()`の先頭適用待ちの列に積む(`Command`のdoc参照)。
    pub fn push_command(&mut self, cmd: Command) {
        self.pending_commands.push(cmd);
    }

    /// 適用済みコマンドの記録(`(step_count, command)`の対、`Command`のdoc参照)。
    pub fn command_log(&self) -> &[(u64, Command)] {
        &self.command_log
    }

    /// 待ち行列の全コマンドを、次数の物理更新前に(このstepの`step_count`で記録しつつ)
    /// 適用する(設計§1「次ステップ先頭で適用・記録」)。無効な`BodyId`を参照する
    /// コマンドは黙って無視する(削除済みIDへのアクセスは`None`、設計の不変条件)。
    fn apply_pending_commands(&mut self) {
        let step = self.clock.step_count();
        let dt = self.clock.dt();
        for cmd in std::mem::take(&mut self.pending_commands) {
            match cmd {
                Command::ApplyForce { body, force, point } => {
                    if self.is_valid(body) {
                        let idx = body.index as usize;
                        // 外力は「新情報」なのでasleep状態を解除する(そうしないと
                        // `sleep::update_sleep_state`が力適用・速度積分ごと止めており、
                        // 力を積んでも一切反映されない、実装検証中に発見)。
                        self.mechanics.bodies.asleep[idx] = false;
                        self.mechanics.bodies.force_accum[idx] =
                            self.mechanics.bodies.force_accum[idx] + force;
                        if let Some(p) = point {
                            let r = p - self.mechanics.bodies.position[idx];
                            self.mechanics.bodies.torque_accum[idx] =
                                self.mechanics.bodies.torque_accum[idx] + r.cross(force);
                        }
                    }
                }
                Command::SetMotorTarget {
                    hinge_motor_index,
                    theta_target,
                } => {
                    if let Some(motor) = self.mechanics.hinge_motors.get_mut(hinge_motor_index) {
                        motor.theta_target = theta_target;
                        // ApplyForceと同じ理由でasleep状態を解除する(新しい目標角度は
                        // 新情報であり、休眠中の剛体はPDトルクを適用しても速度積分が
                        // 止まっているため一切動かない)。
                        self.mechanics.bodies.asleep[motor.body] = false;
                    }
                }
                Command::SetSwitch {
                    switch_index,
                    closed,
                } => {
                    if let Some(circuit) = &mut self.circuit {
                        circuit.set_switch_closed(switch_index, closed);
                    }
                }
                Command::SetHeatSource { node, watts } => {
                    if let Some(thermal) = &mut self.thermal {
                        if let Some(n) = thermal.nodes.get_mut(node) {
                            n.heat_accum += watts * dt;
                        }
                    }
                }
                Command::Grab {
                    body,
                    anchor_local,
                    target,
                } => {
                    if self.is_valid(body) {
                        let idx = body.index as usize;
                        // 既存grabがあれば先に無効化してから新設する(モジュールdoc
                        // 「1剛体につき同時に1つのgrab」参照)。
                        if let Some(&old_joint_index) = self.grab_joints.get(&body.index) {
                            self.mechanics.ball_joints[old_joint_index].disabled = true;
                        }
                        self.mechanics.bodies.asleep[idx] = false;
                        self.mechanics.ball_joints.push(sim_mechanics::BallJoint {
                            body_a: idx,
                            anchor_a: anchor_local,
                            body_b: None,
                            anchor_b: target,
                            disabled: false,
                        });
                        let new_joint_index = self.mechanics.ball_joints.len() - 1;
                        self.grab_joints.insert(body.index, new_joint_index);
                    }
                }
                Command::MoveGrab { body, target } => {
                    if let Some(&joint_index) = self.grab_joints.get(&body.index) {
                        self.mechanics.ball_joints[joint_index].anchor_b = target;
                        if self.is_valid(body) {
                            let idx = body.index as usize;
                            self.mechanics.bodies.asleep[idx] = false;
                        }
                    }
                }
                Command::Release { body } => {
                    if let Some(joint_index) = self.grab_joints.remove(&body.index) {
                        self.mechanics.ball_joints[joint_index].disabled = true;
                        // grab中に静止し続けていた剛体はasleep化している可能性が高く、
                        // 起こさないと重力も含め力適用・速度積分ごと止まったまま
                        // (`ApplyForce`/`SetMotorTarget`と同じ理由、実装検証中に発見)。
                        if self.is_valid(body) {
                            self.mechanics.bodies.asleep[body.index as usize] = false;
                        }
                    }
                }
                Command::SetBodyMass { body, mass } => {
                    if self.is_valid(body) {
                        self.mechanics.bodies.set_mass(body.index as usize, mass);
                    }
                }
                Command::SetBodyType {
                    body,
                    body_type,
                    mass,
                } => {
                    if self.is_valid(body) {
                        self.mechanics
                            .bodies
                            .set_body_type(body.index as usize, body_type, mass);
                    }
                }
                Command::SetCollisionFilter { body, group, mask } => {
                    if self.is_valid(body) {
                        self.mechanics.bodies.set_collision_filter(
                            body.index as usize,
                            group,
                            mask,
                        );
                        // フィルタ変更で新たに触れるようになった相手を拾うため、
                        // 静止仮定を解除する(`set_shape` と同じ理由)。
                        self.mechanics.bodies.asleep[body.index as usize] = false;
                    }
                }
                Command::SetCouplingParam {
                    coupling_index,
                    param,
                    value,
                } => {
                    if let Some(coupling) = self.couplings.get_mut(coupling_index) {
                        coupling.set_scalar_param(param, value);
                    }
                }
                Command::SetGravityField { field } => {
                    // 重力が変われば、静止仮定で眠っている剛体も動き出しうる。
                    // `SetCollisionFilter`等と同じ理由で全Dynamic剛体を起こす
                    // (起こさないと`sleep::update_sleep_state`が力適用・速度積分
                    // ごと止めたままになり、新しい重力場が効かない)。
                    self.mechanics.set_gravity_field(field);
                    for i in 0..self.mechanics.bodies.len() {
                        self.mechanics.bodies.asleep[i] = false;
                    }
                }
            }
            self.command_log.push((step, cmd));
        }
    }

    /// 熱ドメインを有効化する(モジュールdoc「全ドメイン合成」参照)。
    pub fn enable_thermal(&mut self, solver: sim_thermal::ThermalSolver) {
        self.thermal = Some(solver);
    }

    pub fn thermal(&self) -> Option<&sim_thermal::ThermalSolver> {
        self.thermal.as_ref()
    }

    pub fn thermal_mut(&mut self) -> Option<&mut sim_thermal::ThermalSolver> {
        self.thermal.as_mut()
    }

    /// 電磁気(静電)ドメインを有効化する。
    pub fn enable_em_electrostatics(&mut self, solver: sim_em::PointChargeSystem) {
        self.em_electrostatics = Some(solver);
    }

    pub fn em_electrostatics(&self) -> Option<&sim_em::PointChargeSystem> {
        self.em_electrostatics.as_ref()
    }

    pub fn em_electrostatics_mut(&mut self) -> Option<&mut sim_em::PointChargeSystem> {
        self.em_electrostatics.as_mut()
    }

    /// 天体ドメインを有効化する。
    pub fn enable_astro(&mut self, solver: sim_astro::NBodySystem) {
        self.astro = Some(solver);
    }

    pub fn astro(&self) -> Option<&sim_astro::NBodySystem> {
        self.astro.as_ref()
    }

    pub fn astro_mut(&mut self) -> Option<&mut sim_astro::NBodySystem> {
        self.astro.as_mut()
    }

    /// 回路ドメインを有効化する。
    pub fn enable_circuit(&mut self, circuit: sim_em::Circuit) {
        self.circuit = Some(circuit);
    }

    pub fn circuit(&self) -> Option<&sim_em::Circuit> {
        self.circuit.as_ref()
    }

    pub fn circuit_mut(&mut self) -> Option<&mut sim_em::Circuit> {
        self.circuit.as_mut()
    }

    /// 気体区画ドメインを有効化する(`sim_coupling::PistonGas`が使う、断熱圧縮シナリオ)。
    pub fn enable_gas(&mut self, gas: sim_thermal::GasCompartment) {
        self.gas = Some(gas);
    }

    pub fn gas(&self) -> Option<&sim_thermal::GasCompartment> {
        self.gas.as_ref()
    }

    pub fn gas_mut(&mut self) -> Option<&mut sim_thermal::GasCompartment> {
        self.gas.as_mut()
    }

    /// 1D格子熱伝導ドメインを有効化する(D16「熱伝導レース」が使う)。
    pub fn enable_conduction_rod(&mut self, rod: sim_thermal::ConductionRod1D) {
        self.conduction_rod = Some(rod);
    }

    pub fn conduction_rod(&self) -> Option<&sim_thermal::ConductionRod1D> {
        self.conduction_rod.as_ref()
    }

    pub fn conduction_rod_mut(&mut self) -> Option<&mut sim_thermal::ConductionRod1D> {
        self.conduction_rod.as_mut()
    }

    /// SPH流体ドメインを有効化する(`step()`が自動的にsub-stepする、モジュールdoc参照)。
    pub fn enable_sph(&mut self, sph: sim_fluid::SphFluid) {
        self.sph = Some(sph);
    }

    pub fn sph(&self) -> Option<&sim_fluid::SphFluid> {
        self.sph.as_ref()
    }

    pub fn sph_mut(&mut self) -> Option<&mut sim_fluid::SphFluid> {
        self.sph.as_mut()
    }

    /// 格子流体ドメインを有効化する(`step()`が自動的にsub-stepする、モジュールdoc参照)。
    pub fn enable_grid_fluid(&mut self, grid_fluid: sim_fluid::GridFluid2D) {
        self.grid_fluid = Some(grid_fluid);
    }

    pub fn grid_fluid(&self) -> Option<&sim_fluid::GridFluid2D> {
        self.grid_fluid.as_ref()
    }

    pub fn grid_fluid_mut(&mut self) -> Option<&mut sim_fluid::GridFluid2D> {
        self.grid_fluid.as_mut()
    }

    /// 3D格子流体ドメインを有効にする(**群9で追加**)。
    pub fn enable_grid_fluid_3d(&mut self, grid_fluid_3d: sim_fluid::GridFluid3D) {
        self.grid_fluid_3d = Some(grid_fluid_3d);
    }

    pub fn grid_fluid_3d(&self) -> Option<&sim_fluid::GridFluid3D> {
        self.grid_fluid_3d.as_ref()
    }

    pub fn grid_fluid_3d_mut(&mut self) -> Option<&mut sim_fluid::GridFluid3D> {
        self.grid_fluid_3d.as_mut()
    }

    /// ソフトボディ(XPBDロープ)ドメインを有効化する(`conduction_rod_mut().step(dt)`と
    /// 同じ理由で呼び出し側が明示的に`soft_body_mut().step(...)`を呼ぶ必要がある、
    /// モジュールdoc参照)。
    pub fn enable_soft_body(&mut self, soft_body: sim_mechanics::SoftBody) {
        self.soft_body = Some(soft_body);
    }

    pub fn soft_body(&self) -> Option<&sim_mechanics::SoftBody> {
        self.soft_body.as_ref()
    }

    pub fn soft_body_mut(&mut self) -> Option<&mut sim_mechanics::SoftBody> {
        self.soft_body.as_mut()
    }

    /// **量子ドメイン(1D TDSE)を有効化する(群3、フィールドのdoc参照)**。
    /// `Solver`を実装したので`step()`が他ドメインと同じ固定順で自動sub-stepする。
    ///
    /// **単位系が違う点に注意**: 量子は原子単位($\hbar=m_e=1$)、他ドメインはSI。
    /// `total_energy()`の合計は物理的に意味を持たない(`WaveFunction1D::total_energy`
    /// のdoc参照)。混在シーンを禁止はしない——D27–D29 のように量子だけのシーンでは
    /// 何の問題も無く、禁止するとそれらが載せられなくなるため。
    pub fn enable_quantum_1d(&mut self, wave: sim_quantum::WaveFunction1D) {
        self.quantum_1d = Some(wave);
    }

    pub fn quantum_1d(&self) -> Option<&sim_quantum::WaveFunction1D> {
        self.quantum_1d.as_ref()
    }

    pub fn quantum_1d_mut(&mut self) -> Option<&mut sim_quantum::WaveFunction1D> {
        self.quantum_1d.as_mut()
    }

    /// 量子ドメイン(2D TDSE、二重スリット用)を有効化する。
    pub fn enable_quantum_2d(&mut self, wave: sim_quantum::WaveFunction2D) {
        self.quantum_2d = Some(wave);
    }

    pub fn quantum_2d(&self) -> Option<&sim_quantum::WaveFunction2D> {
        self.quantum_2d.as_ref()
    }

    pub fn quantum_2d_mut(&mut self) -> Option<&mut sim_quantum::WaveFunction2D> {
        self.quantum_2d.as_mut()
    }

    /// ブラウン運動ドメインを有効化する(群3)。外力は
    /// `BrownianParticleSet::external_force`(一様場)で与える。
    pub fn enable_brownian(&mut self, brownian: sim_statistical::BrownianParticleSet) {
        self.brownian = Some(brownian);
    }

    pub fn brownian(&self) -> Option<&sim_statistical::BrownianParticleSet> {
        self.brownian.as_ref()
    }

    pub fn brownian_mut(&mut self) -> Option<&mut sim_statistical::BrownianParticleSet> {
        self.brownian.as_mut()
    }

    /// 気体分子運動論(剛体球MD)ドメインを有効化する(群3)。
    pub fn enable_kinetic_gas(&mut self, gas: sim_statistical::GasSim) {
        self.kinetic_gas = Some(gas);
    }

    pub fn kinetic_gas(&self) -> Option<&sim_statistical::GasSim> {
        self.kinetic_gas.as_ref()
    }

    pub fn kinetic_gas_mut(&mut self) -> Option<&mut sim_statistical::GasSim> {
        self.kinetic_gas.as_mut()
    }

    /// イジング模型ドメインを有効化する(群3)。**モンテカルロには物理時間が無い**
    /// ため、`step()`の`dt`は無視され`updates_per_step`回の更新が行われる
    /// (`IsingSim`の`Solver`実装のdoc参照)。
    pub fn enable_ising(&mut self, ising: sim_statistical::IsingSim) {
        self.ising = Some(ising);
    }

    pub fn ising(&self) -> Option<&sim_statistical::IsingSim> {
        self.ising.as_ref()
    }

    pub fn ising_mut(&mut self) -> Option<&mut sim_statistical::IsingSim> {
        self.ising.as_mut()
    }

    /// FDTD(電磁波)ドメインを有効化する(群3)。
    pub fn enable_fdtd(&mut self, fdtd: sim_em::FdtdSim2D) {
        self.fdtd = Some(fdtd);
    }

    pub fn fdtd(&self) -> Option<&sim_em::FdtdSim2D> {
        self.fdtd.as_ref()
    }

    pub fn fdtd_mut(&mut self) -> Option<&mut sim_em::FdtdSim2D> {
        self.fdtd.as_mut()
    }

    /// 流体場の点`p`での観測(設計docs/20-integration/04-world-api.md §2
    /// `sample_fluid(p) -> FluidSample`)。
    ///
    /// **縮約実装の理由**: 設計は速度・圧力・温度を返すが、SPHは温度場を持たない
    /// (`温度`は対象外)。カーネル補間ではなく最近傍粒子の値をそのまま返す縮約
    /// (`p`が粒子分布から離れているほど代表性が下がる点に注意、真のカーネル補間は
    /// 後続増分)。SPHドメインが未有効化、または粒子が1つも無い場合は`None`。
    pub fn sample_fluid(&self, p: Vec3) -> Option<FluidSample> {
        let sph = self.sph.as_ref()?;
        if sph.position.is_empty() {
            return None;
        }

        // **増分Jで真のカーネル補間へ移行した**。それまでは最近傍粒子の値を
        // そのまま返す縮約で、粒子間では値が階段状に不連続に飛んでいた
        // (同じ水塊の中でもサンプル点が少し動くだけで別粒子の値に切り替わる)。
        // SPHの場の補間は $A(x)=\sum_j \frac{m_j}{\rho_j} A_j W(|x-x_j|,h)$ であり、
        // **密度計算に使っているのと同じカーネル**(`sim_fluid::sph_kernel`)で
        // 重み付けしないと、密度と場のサンプリングが整合しない。
        let h = sph.h;
        let mut velocity = Vec3::ZERO;
        let mut pressure = 0.0;
        // シェパード補正の分母。自由表面付近では $\sum_j \frac{m_j}{\rho_j}W$ が
        // 1を大きく下回る(近傍が足りない=D23で実測した自由表面欠損と同じ現象)ため、
        // これで割って規格化しないと**表面付近の値が系統的に小さく出る**。
        let mut weight_sum = 0.0;
        for j in 0..sph.position.len() {
            let r = (sph.position[j] - p).length();
            if r > h {
                continue; // cubic splineの台はr<=h。
            }
            let density = sph.density[j];
            if density <= 0.0 {
                continue;
            }
            let w = sim_fluid::sph_kernel(r, h) * sph.mass / density;
            velocity = velocity + sph.velocity[j].scale(w);
            pressure += sph.pressure[j] * w;
            weight_sum += w;
        }

        if weight_sum <= 1.0e-12 {
            // 近傍粒子が1つも無い点(流体の外)。**最近傍へフォールバックしない**
            // ——遠く離れた点にもっともらしい値を返すより、「そこに流体は無い」と
            // 答えるほうが正直である。
            return None;
        }
        Some(FluidSample {
            velocity: velocity.scale(1.0 / weight_sum),
            pressure: pressure / weight_sum,
        })
    }

    /// 現在のタイムステップ [s](**群2で追加**、エディタのSettingsが表示する)。
    pub fn dt(&self) -> f64 {
        self.clock.dt()
    }

    /// クロックへの可変アクセス(**群2で追加**)。`dt`を実行時に変えるための窓口
    /// ——決定論への影響は`SimClock::set_dt`のdoc参照。
    pub fn clock_mut(&mut self) -> &mut sim_core::SimClock {
        &mut self.clock
    }

    /// 中央PRNG(`rng`フィールド)の**現在の内部状態**
    /// (`sim_math::SimRngState`のdoc参照)。
    ///
    /// **なぜ`seed`ではないのか**: `WorldOptions::seed`は`SimRng::new`が状態へ
    /// 畳み込んだ時点で失われ、`World`はどこにも保持していない。`state_hash`は
    /// PRNGを含まないので決定論replayの一致には効かないが、**エクスポート→
    /// 再インポート後も同じ乱数列を続けたい**(ブラウン運動・気体分子運動論の
    /// 衝突判定など、`SolverContext::rng`を引くドメインを含むシーン)場合は
    /// 種ではなく**今のストリーム位置**を戻す必要がある。
    /// `Scenario::rng_state`がこの値を書き出す。
    pub fn rng_state(&self) -> sim_math::SimRngState {
        self.rng.raw_state()
    }

    /// 中央PRNGの内部状態を差し替える(`rng_state`の逆、`Scenario::rng_state`の
    /// 復元経路が使う)。
    pub fn set_rng_state(&mut self, state: sim_math::SimRngState) {
        self.rng.set_raw_state(state);
    }

    /// 全ドメインが読む物性データベース(設計 §1.1.5)。`create_body` に渡す
    /// `MaterialId` の解決に使う。
    pub fn materials(&self) -> &MaterialDb {
        &self.materials
    }

    /// 力学ソルバへの不変アクセス(`mechanics_mut`の読み取り専用版、Inspectorの
    /// Shape表示など、`RigidBodySet::shape_of`のような読み取りだけで済む
    /// クエリ向け)。
    pub fn mechanics(&self) -> &MechanicsSolver {
        &self.mechanics
    }

    /// 材料DBへの可変アクセス(`from_scenario`の`extends`派生材料の追加用、設計§1
    /// 「シーン構築時の設定はコマンド規律の対象外」)。
    pub fn materials_mut(&mut self) -> &mut MaterialDb {
        &mut self.materials
    }

    /// 剛体を追加する。設計 docs/20-integration/04-world-api.md §2 の `create_body`。
    pub fn create_body(&mut self, desc: RigidBodyDesc) -> BodyId {
        let index = self.mechanics.create_body(desc, &self.materials);
        debug_assert_eq!(
            index,
            self.generations.len(),
            "RigidBodySet is expected to only grow (no slot reuse yet, module doc)"
        );
        self.generations.push(0);
        self.removed.push(false);
        BodyId {
            index: index as u32,
            generation: 0,
        }
    }

    /// エディタのScale Gizmo(縮約実装、`sim_mechanics::RigidBodySet::set_shape`の
    /// doc参照)向けに、既存ボディの形状を置き換え、質量・慣性を`create_body`と
    /// 同じ規約で再計算する。無効な`id`なら何もしない(`remove_body`と同じ
    /// 不変条件)。
    pub fn set_body_shape(&mut self, id: BodyId, shape: Shape) {
        if !self.is_valid(id) {
            return;
        }
        self.mechanics
            .bodies
            .set_shape(id.index as usize, shape, &self.materials);
    }

    fn is_valid(&self, id: BodyId) -> bool {
        (id.index as usize) < self.generations.len()
            && self.generations[id.index as usize] == id.generation
    }

    /// `id`が現在の`World`でまだ生存しているか(`is_valid`の公開版)。
    ///
    /// **なぜ要るか**: `sim-wasm::WasmWorld`は`BodyId`をフロント向けの安定
    /// index(`self.bodies: Vec<SpawnedBodyMeta>`の位置)へ対応付けて保持するが、
    /// Timeline の巻き戻し(`restore_snapshot`、`World::restore`のdoc参照)は
    /// `self.inner`だけを過去の`World`へ差し替え、`self.bodies`はそのまま残す。
    /// そのため「巻き戻した時点より後に作られたボディ」を指す`BodyId`が
    /// `self.bodies`側には生き続ける——これを`is_valid`で確認せずに
    /// `mechanics().bodies.position[id.index as usize]`のような生indexアクセスを
    /// すると、`generations`(延いては`RigidBodySet`の各`Vec`)がその`index`より
    /// 短くなっていて**範囲外パニック**になる(wasmのパニックはモジュール全体を
    /// 使用不能にする、`try_body_id_at`のdoc参照)。
    pub fn is_body_alive(&self, id: BodyId) -> bool {
        self.is_valid(id)
    }

    /// ボディを削除する。世代カウンタをインクリメントし、以後この `id` (と古い世代の
    /// 再利用)へのアクセスは `None` になる(設計の不変条件)。下層の `RigidBodySet`
    /// スロット自体はまだ真に解放されない(モジュールdoc参照) — 無効化として
    /// `BodyType::Static` 化・遠方への退避・速度ゼロ化を行い、実質的な影響を無くす。
    ///
    /// **連鎖削除(群2)**: 削除した剛体を参照するジョイントと結合を切り離す。
    /// これをやらないと、遠方(y=-1e9)へ飛ばした剛体にジョイントで繋がった相手が
    /// **一緒に引きずられて飛んでいく**(拘束は「無効化」を知らないため)。
    /// ジョイントは index のずれを避けるため `disabled` フラグで無効化し、
    /// `Coupling` は `referenced_bodies()`(群1の内省層)で参照を判定して除去する。
    ///
    /// **`Coupling` は index ベースの参照しか持たない**ため、削除で index が
    /// 再利用されることはない(スロットを真に解放しない設計のおかげで、
    /// 「別のボディを指し始める」危険はそもそも生じない)。
    pub fn remove_body(&mut self, id: BodyId) {
        if !self.is_valid(id) {
            return;
        }
        let idx = id.index as usize;
        self.generations[idx] += 1;
        self.removed[idx] = true;
        self.mechanics.bodies.body_type[idx] = BodyType::Static;
        self.mechanics.bodies.position[idx] = Vec3::new(0.0, -1.0e9, 0.0);
        self.mechanics.bodies.linear_velocity[idx] = Vec3::ZERO;
        self.mechanics.bodies.angular_velocity[idx] = Vec3::ZERO;

        // ジョイントの連鎖削除。`body_b: Option<usize>` の `None` はワールド固定点。
        let touches = |a: usize, b: Option<usize>| a == idx || b == Some(idx);
        for joint in &mut self.mechanics.joints {
            if touches(joint.body_a, joint.body_b) {
                joint.disabled = true;
            }
        }
        for joint in &mut self.mechanics.ball_joints {
            if touches(joint.body_a, joint.body_b) {
                joint.disabled = true;
            }
        }
        for joint in &mut self.mechanics.slider_joints {
            if touches(joint.body_a, joint.body_b) {
                joint.disabled = true;
            }
        }
        for motor in &mut self.mechanics.hinge_motors {
            if motor.body == idx {
                motor.disabled = true;
            }
        }
        // grab 中だったなら追跡表からも落とす(残すと `MoveGrab` が
        // 無効化済みジョイントを触り続ける)。
        self.grab_joints.remove(&id.index);

        // 結合の連鎖削除(群1の `Coupling::referenced_bodies()` を使う)。
        self.couplings
            .retain(|c| !c.referenced_bodies().contains(&idx));
    }

    /// 直接可変アクセス(抗力・浮力の周囲媒質設定など)。設計が定める
    /// 「書き込みはコマンド経由」規律の対象は実行中の状態変更であり、
    /// シーン構築時の設定はこの限りでない(§1 設計原則)。
    pub fn mechanics_mut(&mut self) -> &mut MechanicsSolver {
        &mut self.mechanics
    }

    /// 有効な全ドメインの合計エネルギー(固定順、モジュールdoc参照)。
    fn total_energy(&self) -> sim_core::EnergyBreakdown {
        let mut total = self.mechanics.total_energy();
        if let Some(t) = &self.thermal {
            total = total + t.total_energy();
        }
        if let Some(e) = &self.em_electrostatics {
            total = total + e.total_energy();
        }
        if let Some(a) = &self.astro {
            total = total + a.total_energy();
        }
        if let Some(c) = &self.circuit {
            total = total + c.total_energy();
        }
        if let Some(s) = &self.sph {
            total = total + s.total_energy();
        }
        if let Some(g) = &self.grid_fluid {
            total = total + g.total_energy();
        }
        if let Some(g) = &self.grid_fluid_3d {
            total = total + g.total_energy();
        }
        // **群3で追加**: `soft_body`(弾性)・`conduction_rod`(熱)・`kinetic_gas`
        // (運動)。いずれもSI単位で閉じた保存系なので合計に入れてよい。
        // 増分Hで`Solver`を実装して`step()`へ繋いだ時点で入れるべきだったが
        // 漏れており、**ロープや熱伝導棒のエネルギーが台帳から丸ごと抜けていた**。
        if let Some(b) = &self.soft_body {
            total = total + b.total_energy();
        }
        if let Some(r) = &self.conduction_rod {
            total = total + r.total_energy();
        }
        if let Some(g) = &self.kinetic_gas {
            total = total + g.total_energy();
        }
        total
    }

    /// ドメインごとのエネルギー内訳(**群3で追加**)。
    ///
    /// **なぜ `total_energy()` に全ドメインを足さないのか**——`EnergyLedger` は
    /// 「合計エネルギーが保存しているか」をCIゲートとして検算する仕組みだが、
    /// 群3で載せたドメインには**そもそも足してはいけないものがある**:
    ///
    /// - **量子(原子単位 $\hbar=m_e=1$)・FDTD(正規化単位 $c=\varepsilon_0=\mu_0=1$)**:
    ///   SI単位の力学・熱と**単位系が違う**。数値を足しても物理量にならない。
    /// - **イジング模型**: $J$ は無次元スケール。加えて正準集団のサンプリングなので
    ///   エネルギーは**揺らぐのが正しい**。
    /// - **ブラウン運動**: ランジュバン熱浴と絶えずエネルギーをやり取りする開放系。
    ///   保存しないのが正しい挙動。
    ///
    /// これらを合計に混ぜると、**台帳の残差が「バグの兆候」ではなく「仕様」に
    /// なってしまい、保存則ゲートが死ぬ**。そこで合計からは外し、代わりに
    /// このメソッドで**ドメインごとに単位と保存性を明示して**返す。
    /// エディタの Inspector / HUD はこちらを使う。
    pub fn energy_report(&self) -> Vec<DomainEnergy> {
        let mut report = vec![DomainEnergy {
            domain: "Mechanics",
            energy: self.mechanics.total_energy(),
            unit: "J",
            conservative: true,
            in_total: true,
        }];
        let mut push = |domain, energy, unit, conservative, in_total| {
            report.push(DomainEnergy {
                domain,
                energy,
                unit,
                conservative,
                in_total,
            });
        };
        if let Some(t) = &self.thermal {
            push("Thermal", t.total_energy(), "J", true, true);
        }
        if let Some(e) = &self.em_electrostatics {
            push("Electrostatics", e.total_energy(), "J", true, true);
        }
        if let Some(a) = &self.astro {
            push("Astro", a.total_energy(), "J", true, true);
        }
        if let Some(c) = &self.circuit {
            push("Circuit", c.total_energy(), "J", true, true);
        }
        if let Some(x) = &self.sph {
            push("SPH", x.total_energy(), "J", true, true);
        }
        if let Some(g) = &self.grid_fluid {
            push("GridFluid", g.total_energy(), "J", true, true);
        }
        if let Some(g) = &self.grid_fluid_3d {
            push("GridFluid3D", g.total_energy(), "J", true, true);
        }
        if let Some(b) = &self.soft_body {
            push("SoftBody", b.total_energy(), "J", true, true);
        }
        if let Some(r) = &self.conduction_rod {
            push("ConductionRod", r.total_energy(), "J", true, true);
        }
        if let Some(g) = &self.kinetic_gas {
            push("KineticGas", g.total_energy(), "J", true, true);
        }
        if let Some(q) = &self.quantum_1d {
            push("Quantum1D", q.total_energy(), "Ha (原子単位)", true, false);
        }
        if let Some(q) = &self.quantum_2d {
            push("Quantum2D", q.total_energy(), "Ha (原子単位)", true, false);
        }
        if let Some(b) = &self.brownian {
            push("Brownian", b.total_energy(), "J", false, false);
        }
        if let Some(i) = &self.ising {
            push(
                "Ising",
                i.total_energy(),
                "J (無次元スケール)",
                false,
                false,
            );
        }
        if let Some(f) = &self.fdtd {
            // `FdtdSim2D`はスカラーを返す inherent `total_energy()` を持ち、
            // そちらがトレイトメソッドより優先されるため明示的に呼び分ける。
            push(
                "FDTD",
                sim_core::Solver::total_energy(f),
                "正規化単位 (c=1)",
                true,
                false,
            );
        }
        report
    }

    /// 1 world step(固定 dt)。docs/20-integration/04-world-api.md §2 の `step()`。
    /// 有効な全ドメインを固定順(mechanics→thermal→em→astro、モジュールdoc参照)で進める。
    pub fn step(&mut self) {
        if self.ledger.is_none() {
            self.ledger = Some(EnergyLedger::new(self.total_energy().total()));
        }
        self.apply_pending_commands();
        let dt = self.clock.dt();

        // **pre 相(増分Jで追加)**。設計 docs/20-integration/01-coupling-matrix.md
        // §1.3 が求める pre/post の2相分離のうち、ドメインソルバを進める**前**に
        // 呼ぶ側。今stepの積分へ効かせたい注入型の結合をここへ移せるようにする。
        // **既定実装は何もしない**ので、既存の全結合の挙動は変わらない
        // (`Coupling::apply_pre`のdoc参照)。
        for index in 0..self.couplings.len() {
            let mut coupling = std::mem::replace(
                &mut self.couplings[index],
                Box::new(sim_coupling::NoopCoupling),
            );
            {
                let mut states = sim_coupling::DomainStates {
                    mechanics: &mut self.mechanics,
                    thermal: self.thermal.as_mut(),
                    em_circuit: self.circuit.as_mut(),
                    em_electrostatics: self.em_electrostatics.as_mut(),
                    gas: self.gas.as_mut(),
                    grid_fluid: self.grid_fluid.as_mut(),
                    grid_fluid_3d: self.grid_fluid_3d.as_mut(),
                    sph: self.sph.as_mut(),
                };
                coupling.apply_pre(&mut states, dt);
            }
            self.couplings[index] = coupling;
        }

        match self.time_regime {
            sim_astro::TimeRegime::Local { .. } => {
                let mut capped = run_domain_substeps(
                    &mut self.mechanics,
                    dt,
                    &self.materials,
                    &mut self.rng,
                    &mut self.events,
                );
                if let Some(t) = &mut self.thermal {
                    capped |= run_domain_substeps(
                        t,
                        dt,
                        &self.materials,
                        &mut self.rng,
                        &mut self.events,
                    );
                }
                if let Some(e) = &mut self.em_electrostatics {
                    capped |= run_domain_substeps(
                        e,
                        dt,
                        &self.materials,
                        &mut self.rng,
                        &mut self.events,
                    );
                }
                if let Some(a) = &mut self.astro {
                    capped |= run_domain_substeps(
                        a,
                        dt,
                        &self.materials,
                        &mut self.rng,
                        &mut self.events,
                    );
                }
                if let Some(c) = &mut self.circuit {
                    capped |= run_domain_substeps(
                        c,
                        dt,
                        &self.materials,
                        &mut self.rng,
                        &mut self.events,
                    );
                }
                if let Some(s) = &mut self.sph {
                    capped |= run_domain_substeps(
                        s,
                        dt,
                        &self.materials,
                        &mut self.rng,
                        &mut self.events,
                    );
                }
                if let Some(g) = &mut self.grid_fluid {
                    capped |= run_domain_substeps(
                        g,
                        dt,
                        &self.materials,
                        &mut self.rng,
                        &mut self.events,
                    );
                }
                if let Some(g) = &mut self.grid_fluid_3d {
                    capped |= run_domain_substeps(
                        g,
                        dt,
                        &self.materials,
                        &mut self.rng,
                        &mut self.events,
                    );
                }
                // **増分Hで追加**。`soft_body`と`conduction_rod`はここに無かったため、
                // `enable_soft_body`/`enable_conduction_rod`で載せても`World::step()`
                // では一切動かなかった(D13/D16のテストがドメインを直接取り出して
                // 手で`step`を呼んでいたのはこのため)。シーンギャラリーへ出すには
                // 自動ステップが要るので、両者に`Solver`を実装した上でここへ繋ぐ。
                if let Some(b) = &mut self.soft_body {
                    capped |= run_domain_substeps(
                        b,
                        dt,
                        &self.materials,
                        &mut self.rng,
                        &mut self.events,
                    );
                }
                if let Some(r) = &mut self.conduction_rod {
                    capped |= run_domain_substeps(
                        r,
                        dt,
                        &self.materials,
                        &mut self.rng,
                        &mut self.events,
                    );
                }
                // **群3で追加**。量子・統計・FDTDは`Solver`未実装だったため
                // `World`に載る経路が原理的に無かった(D25/D27–D32が全て
                // 「ドメイン自体が存在しない」として滞留していた)。
                // 順序は他ドメインと同じく**固定**(決定論のため)。
                if let Some(q) = &mut self.quantum_1d {
                    capped |= run_domain_substeps(
                        q,
                        dt,
                        &self.materials,
                        &mut self.rng,
                        &mut self.events,
                    );
                }
                if let Some(q) = &mut self.quantum_2d {
                    capped |= run_domain_substeps(
                        q,
                        dt,
                        &self.materials,
                        &mut self.rng,
                        &mut self.events,
                    );
                }
                if let Some(b) = &mut self.brownian {
                    capped |= run_domain_substeps(
                        b,
                        dt,
                        &self.materials,
                        &mut self.rng,
                        &mut self.events,
                    );
                }
                if let Some(g) = &mut self.kinetic_gas {
                    capped |= run_domain_substeps(
                        g,
                        dt,
                        &self.materials,
                        &mut self.rng,
                        &mut self.events,
                    );
                }
                if let Some(i) = &mut self.ising {
                    capped |= run_domain_substeps(
                        i,
                        dt,
                        &self.materials,
                        &mut self.rng,
                        &mut self.events,
                    );
                }
                if let Some(f) = &mut self.fdtd {
                    capped |= run_domain_substeps(
                        f,
                        dt,
                        &self.materials,
                        &mut self.rng,
                        &mut self.events,
                    );
                }
                self.sub_step_cap_reached = capped;
            }
            // 天体レンジ(設計docs/20-integration/06-regime-switching.md §1「天体専用
            // レジームに切り替え、天体状態のみを独立時間軸で進める…ローカル物理ソルバは
            // 停止(状態は凍結保存)」)。`astro`以外の全ドメインはこのstep呼び出しを
            // 完全にスキップする(状態はそのまま保持され、次にLocalへ戻ったときの
            // 初期条件になる)。
            sim_astro::TimeRegime::Astro {
                dt_astro,
                steps_per_frame,
            } => {
                if let Some(a) = &mut self.astro {
                    for _ in 0..steps_per_frame {
                        let mut ctx = SolverContext {
                            materials: &self.materials,
                            rng: &mut self.rng,
                            events: &mut self.events,
                        };
                        a.step(dt_astro, &mut ctx);
                    }
                }
                self.check_auto_regime_switch();
            }
        }
        // フレーム自身の回転運動学(`sim_core::FrameTree::step`のdoc参照、
        // フレーム軸オーバーレイの土台)。惑星の自転のような「どのレジームが
        // 走っていても物理的に進み続ける」性質の量のため、レジーム分岐の外側で
        // 毎step無条件に進める(自動レジーム切替の判定は上記で既に完了しているため、
        // この時点で`self.frames`を進めても`check_auto_regime_switch`が読んだ状態には
        // 影響しない)。
        self.frames.step(dt);
        // 登録済み全Couplingを1回ずつ適用する(登録順、`apply_coupling`のdocが説明する
        // 「post」型結合(前stepで確定した量を読む)と同じタイミング — 呼び出し側が
        // 毎stepの後に手動で`apply_coupling`を呼んでいた既存の使い方をそのまま
        // `World`内部に移しただけで、タイミング上の意味は変えていない)。
        for coupling in self.couplings.iter_mut() {
            let mut states = sim_coupling::DomainStates {
                mechanics: &mut self.mechanics,
                thermal: self.thermal.as_mut(),
                em_circuit: self.circuit.as_mut(),
                em_electrostatics: self.em_electrostatics.as_mut(),
                gas: self.gas.as_mut(),
                grid_fluid: self.grid_fluid.as_mut(),
                grid_fluid_3d: self.grid_fluid_3d.as_mut(),
                sph: self.sph.as_mut(),
            };
            coupling.apply_post(&mut states, dt);
        }
        // このstepで発行された全イベントを排出し、Event::step(発行元ドメインは
        // ワールド全体のstep_countを知らないため0で埋めている、`sim-mechanics::
        // MechanicsSolver::emit_contact_events`のdoc参照)を正しい値へ上書きしてから
        // `event_log`に記録する(`drain_events`のdoc参照)。
        let step_count = self.clock.step_count();
        for mut e in self.events.drain_sorted() {
            e.step = step_count;
            self.event_log.push(e);
        }
        let total = self.total_energy().total();
        self.ledger
            .as_mut()
            .expect("initialized above")
            .record(total, ENERGY_SCALE_FLOOR);
        self.clock.advance();

        // 登録済み全プローブを毎stepサンプルする(設計§2.1「測って遊ぶの中心機能」)。
        // まず不変借用でサンプル値を集め(`self.probes.iter()`と`self.sample_probe_target`
        // はどちらも共有借用なので同時に成立する)、その後で可変借用に切り替えて
        // `history`へ積む(`self`全体への不変・可変借用が重ならないようにするため2段階
        // にしている)。
        let samples: Vec<f64> = self
            .probes
            .iter()
            .map(|p| self.sample_probe_target(p.target))
            .collect();
        for (probe, sample) in self.probes.iter_mut().zip(samples) {
            probe.history.push(sample);
        }
    }

    /// 直近の記帳残差(設計 docs/21-verification/02-conservation-laws.md §2)。
    /// トレンド監視指標であり、単発のバグ検出には使わない(ドメイン別保存則テストが担う)。
    /// `step()` を一度も呼んでいない場合は 0。
    pub fn energy_residual(&self) -> f64 {
        self.ledger.as_ref().map_or(0.0, |l| l.latest_residual())
    }

    /// 登録済み`Coupling`の件数。
    pub fn coupling_count(&self) -> usize {
        self.couplings.len()
    }

    /// 登録済み結合を内省用に列挙する(**群1で追加**)。
    ///
    /// **これが無かった間の縮約**: `Coupling`トレイトが種別を名乗る手段を持たず、
    /// Inspectorは「種別: —(トレイトが名前を持たないため非表示)」という表示に
    /// 留まっていた。設計 docs/23-frontend/01-editor.md §1.3 が Coupling
    /// コンポーネントに要求する「**種別**・関連する Body/Fluid/Circuit 参照」を
    /// 満たすため、`CouplingKind`+`describe()`+`referenced_*()`を
    /// トレイトへ追加し、ここで集約する。
    pub fn couplings(&self) -> Vec<CouplingInfo> {
        self.couplings
            .iter()
            .enumerate()
            .map(|(index, c)| CouplingInfo {
                index,
                kind: c.kind(),
                description: c.describe(),
                domains: c.domain_ids(),
                bodies: c.referenced_bodies(),
                thermal_nodes: c.referenced_thermal_nodes(),
                voltage_sources: c.referenced_voltage_sources(),
            })
            .collect()
    }

    /// `Coupling`の生の登録列(`World → Scenario`逆写像がパラメータ込みで
    /// 各`Coupling`を読み戻すために使う。`couplings()`が返す`CouplingInfo`は
    /// `describe()`の人間可読文字列しか持たないため、そこからは再構成できない)。
    pub fn couplings_raw(&self) -> &[Box<dyn sim_coupling::Coupling>] {
        &self.couplings
    }

    /// 登録済み結合への可変アクセス(`couplings_raw`の可変版)。
    /// `Coupling::restore_raw_state`(`sim_coupling::CouplingRawState`のdoc参照)を
    /// `from_scenario`が呼ぶために要る——`add_coupling`はindexを返すだけで、
    /// 積んだ後の結合へ触れる口が無かった。
    pub fn couplings_raw_mut(&mut self) -> &mut [Box<dyn sim_coupling::Coupling>] {
        &mut self.couplings
    }

    /// 全ジョイントを種別タグ付きで列挙する(**群1で追加**、`JointInfo`のdoc参照)。
    pub fn joints(&self) -> Vec<JointInfo> {
        let m = &self.mechanics;
        let mut out = Vec::new();
        for (index, j) in m.joints.iter().enumerate() {
            out.push(JointInfo {
                index,
                kind: JointKind::Distance,
                body_a: j.body_a,
                body_b: j.body_b,
                anchor_a: j.anchor_a,
                anchor_b: j.anchor_b,
                axis: None,
                length: Some(j.length),
                motor_target: None,
                disabled: false,
            });
        }
        for (index, j) in m.ball_joints.iter().enumerate() {
            out.push(JointInfo {
                index,
                kind: JointKind::Ball,
                body_a: j.body_a,
                body_b: j.body_b,
                anchor_a: j.anchor_a,
                anchor_b: j.anchor_b,
                axis: None,
                length: None,
                motor_target: None,
                disabled: j.disabled,
            });
        }
        for (index, j) in m.slider_joints.iter().enumerate() {
            out.push(JointInfo {
                index,
                kind: JointKind::Slider,
                body_a: j.body_a,
                body_b: j.body_b,
                anchor_a: j.anchor_a,
                anchor_b: j.anchor_b,
                axis: Some(j.axis_a),
                length: None,
                motor_target: None,
                disabled: false,
            });
        }
        for (index, j) in m.wheel_joints.iter().enumerate() {
            out.push(JointInfo {
                index,
                kind: JointKind::Wheel,
                body_a: j.chassis,
                body_b: Some(j.wheel),
                anchor_a: j.anchor_chassis,
                anchor_b: Vec3::ZERO,
                axis: Some(j.axle_axis),
                length: Some(j.rest_length),
                motor_target: (j.motor_max_torque > 0.0).then_some(j.motor_speed),
                disabled: j.disabled,
            });
        }
        for (index, j) in m.hinge_motors.iter().enumerate() {
            out.push(JointInfo {
                index,
                kind: JointKind::HingeMotor,
                body_a: j.body,
                body_b: None,
                anchor_a: Vec3::ZERO,
                anchor_b: Vec3::ZERO,
                axis: Some(j.axis),
                length: None,
                motor_target: Some(j.theta_target),
                disabled: false,
            });
        }
        out
    }

    /// 特定の剛体に接続されたジョイントだけを返す(Inspectorが選択中ボディで絞る)。
    pub fn joints_for_body(&self, body: BodyId) -> Vec<JointInfo> {
        let index = body.index as usize;
        self.joints()
            .into_iter()
            .filter(|j| j.body_a == index || j.body_b == Some(index))
            .collect()
    }

    /// 特定の剛体に作用する結合だけを返す(Inspectorが選択中ボディで絞るため)。
    pub fn couplings_for_body(&self, body: BodyId) -> Vec<CouplingInfo> {
        let index = body.index as usize;
        self.couplings()
            .into_iter()
            .filter(|c| c.bodies.contains(&index))
            .collect()
    }

    /// 現在のワールドで有効になっている**縮約・近似の一覧**。
    /// Inspectorの「近似バッジ」に出す。
    ///
    /// **群1で「Worldからの推測」を「各ソルバの自己申告の集約」へ置き換えた**。
    /// 以前はここで「どのドメインが有効か」を見て固定文字列を並べていたため、
    /// ①ソルバ側で近似を変えてもここを直さないと表示が古いまま
    /// ②同じドメインでも設定によって効いている近似が違う(格子流体の粘性0など)
    /// ことを表現できない、という問題があった。`Solver::approximations()`
    /// (既定は空)を各ソルバが実装し、ここは集めるだけにする。
    ///
    /// **`MechanicsSolver`のように申告がまだ空のソルバもある**——その場合は
    /// 何も出さない(嘘を並べるより出さない方を選ぶ、既存の方針どおり)。
    pub fn active_approximations(&self) -> Vec<sim_core::Approximation> {
        let mut out = Vec::new();
        out.extend(self.mechanics.approximations());
        if let Some(t) = &self.thermal {
            out.extend(t.approximations());
        }
        if let Some(e) = &self.em_electrostatics {
            out.extend(e.approximations());
        }
        if let Some(a) = &self.astro {
            out.extend(a.approximations());
        }
        if let Some(c) = &self.circuit {
            out.extend(c.approximations());
        }
        if let Some(sph) = &self.sph {
            out.extend(sph.approximations());
        }
        if let Some(g) = &self.grid_fluid {
            out.extend(g.approximations());
        }
        if let Some(g) = &self.grid_fluid_3d {
            out.extend(g.approximations());
        }
        if let Some(b) = &self.soft_body {
            out.extend(b.approximations());
        }
        if let Some(r) = &self.conduction_rod {
            out.extend(r.approximations());
        }
        // **群3で追加**した6ドメイン。
        if let Some(q) = &self.quantum_1d {
            out.extend(q.approximations());
        }
        if let Some(q) = &self.quantum_2d {
            out.extend(q.approximations());
        }
        if let Some(b) = &self.brownian {
            out.extend(b.approximations());
        }
        if let Some(g) = &self.kinetic_gas {
            out.extend(g.approximations());
        }
        if let Some(i) = &self.ising {
            out.extend(i.approximations());
        }
        if let Some(f) = &self.fdtd {
            out.extend(f.approximations());
        }
        // **単位系の混在を World の側で申告する(群3)**。個々のソルバは自分の
        // 単位しか知らないので、「SI と原子単位/正規化単位が同じシーンに同居して
        // いる」という事実はここでしか言えない——`total_energy()` の合計から
        // 除外しているとはいえ、混ぜて使っていること自体が近似である。
        let has_si = self.thermal.is_some()
            || self.em_electrostatics.is_some()
            || self.astro.is_some()
            || self.circuit.is_some()
            || self.sph.is_some()
            || self.grid_fluid.is_some()
            || self.grid_fluid_3d.is_some()
            || !self.mechanics.bodies.is_empty();
        let has_non_si = self.quantum_1d.is_some()
            || self.quantum_2d.is_some()
            || self.ising.is_some()
            || self.fdtd.is_some();
        if has_si && has_non_si {
            out.push(sim_core::Approximation {
                name: "単位系の混在",
                reason: "SI単位のドメインと、原子単位(量子)/正規化単位(FDTD)/無次元\
                         (イジング)のドメインが同じシーンに載っている。エネルギーの\
                         合計は取らない(World::energy_report がドメインごとに分けて出す)。",
                doc: "docs/00-foundation/04-architecture.md",
                can_disable: false,
            });
        }
        // sub-step 上限に当たったフレームは、そのドメインが要求された時間ぶん
        // 正しく進んでいない(群3、`orchestrator::MAX_SUB_STEPS_PER_FRAME`のdoc参照)。
        if self.sub_step_cap_reached {
            out.push(sim_core::Approximation {
                name: "sub-step 上限に到達",
                reason: "あるドメインの安定 dt がフレーム dt より桁違いに小さく、\
                         1フレームあたりの sub-step 数が上限(1000)で打ち切られた。\
                         そのドメインは要求された時間ぶん正しくは進んでいない\
                         ——フレーム dt を下げる必要がある。",
                doc: "docs/00-foundation/04-architecture.md",
                can_disable: false,
            });
        }
        // 結合の適用順序に起因する近似はソルバではなく`World`の責務なので、
        // ここでしか申告できない(どのソルバの近似でもない)。
        //
        // **群5の実測にもとづく整理**。旧「結合の1step遅れ」バッジは
        // 誘導・モーター・SPH剛体・格子流体剛体の4種を一括で申告していたが、
        // 実際に測ったところ内訳が違っていた:
        //  - 誘導・モーターは**本当に1step遅れていた**(post 相で設定した起電力を回路が
        //    読むのは次step)→ `apply_pre`へ移して解消済み。
        //  - SPH剛体・格子流体剛体の**反作用力は元から遅れていなかった**(post 相は
        //    全ドメインソルバの後なので今stepの値を読む)→ doc側の誤記だったので訂正。
        //    ただし境界形状(境界粒子位置・solidマスク)は下記のとおり今も遅れる。
        //  - 外力注入型(浮力抗力/クーロン/鏡像/ブラウン)は`apply_pre`へ移し、注入した
        //    速度がその step の位置積分に乗るようにした。
        let stale_geometry = self.couplings.iter().any(|c| {
            matches!(
                c.kind(),
                sim_coupling::CouplingKind::SphRigid | sim_coupling::CouplingKind::GridFluidRigid
            )
        });
        if stale_geometry {
            out.push(sim_core::Approximation {
                name: "流体が見る剛体境界が1力学step古い",
                reason: "SPH剛体・格子流体剛体が流体へ渡す境界形状(境界粒子位置・solid\
                         マスク)は、書き込みと流体ステップの間に力学ステップが挟まるため\
                         v*dt だけ遅れる。pre 相へ移しても遅れ量は変わらない(実測で確認)\
                         ——pre/post の2相しか無いAPIとドメイン固定順序に由来する。",
                doc: "docs/20-integration/01-coupling-matrix.md",
                can_disable: false,
            });
        }
        if self
            .couplings
            .iter()
            .any(|c| matches!(c.kind(), sim_coupling::CouplingKind::BoussinesqBuoyancy))
        {
            out.push(sim_core::Approximation {
                name: "熱起因の浮力の位置応答が1step遅れる",
                reason: "`BoussinesqBuoyancy`は**今stepで確定した温度**を読むため post 相に\
                         置いている(pre 相にすると前stepの温度を読むことになる)。その結果、\
                         注入された速度がその step の位置積分には乗らず、位置応答だけが\
                         1step遅れる。固定ドメイン順序(力学→熱)に由来するので、\
                         どちらか一方の遅れは避けられない。",
                doc: "docs/20-integration/01-coupling-matrix.md",
                can_disable: false,
            });
        }
        out
    }

    pub fn energy_residual_history(&self) -> &[f64] {
        self.ledger.as_ref().map_or(&[], |l| l.residual_history())
    }

    pub fn time(&self) -> f64 {
        self.clock.time()
    }

    pub fn step_count(&self) -> u64 {
        self.clock.step_count()
    }

    /// 現在の全状態のスナップショットを取る(設計docs/20-integration/04-world-api.md §2、
    /// 型doc「`Clone`を導出できる理由」参照)。
    pub fn snapshot(&self) -> World {
        self.clone()
    }

    /// スナップショットから状態を復元する(設計docs/20-integration/02-determinism-replay.md
    /// §6「スナップショット再開時のリプレイ一致」— 復元直後に`step()`を続けても、
    /// スナップショットを取らず通しで実行した場合と`state_hash()`が一致することを
    /// テストで検証する)。
    pub fn restore(&mut self, snapshot: &World) {
        *self = snapshot.clone();
    }

    /// 設計 docs/00-foundation/04-architecture.md §3「削除済み ID へのアクセスは `None`
    /// (パニックしない)」。
    pub fn body_position(&self, id: BodyId) -> Option<Vec3> {
        if !self.is_valid(id) {
            return None;
        }
        // 「ボディはどこにあるか」の答えは**形状のローカル原点**
        // (生成時に指定した`transform.position`と同じ点)。`bodies.position`
        // は重心なので`origin_position`を通す(群11、`RigidBodySet`型doc参照)。
        Some(self.mechanics.bodies.origin_position(id.index as usize))
    }

    /// `body_position`と同じ不変条件の速度版(`Probe::BodySpeed`が読む)。
    pub fn body_velocity(&self, id: BodyId) -> Option<Vec3> {
        if !self.is_valid(id) {
            return None;
        }
        Some(self.mechanics.bodies.linear_velocity[id.index as usize])
    }

    /// `body_position`と同じ不変条件の姿勢(クォータニオン)版。
    pub fn body_rotation(&self, id: BodyId) -> Option<sim_math::Quat> {
        if !self.is_valid(id) {
            return None;
        }
        Some(self.mechanics.bodies.rotation[id.index as usize])
    }

    /// `body_position`と同じ不変条件の角速度版(`export`モジュールの
    /// `World → Scenario`逆写像が`BodyScenarioDesc::angular_velocity`を
    /// 読み戻すのに使う)。
    pub fn body_angular_velocity(&self, id: BodyId) -> Option<Vec3> {
        if !self.is_valid(id) {
            return None;
        }
        Some(self.mechanics.bodies.angular_velocity[id.index as usize])
    }

    /// 現在生存している(`remove_body`されていない)全`BodyId`をindex昇順で返す
    /// (`export`モジュールの`World → Scenario`逆写像向け。`RigidBodySet`のスロットは
    /// 削除後も解放されない(`remove_body`のdoc参照)ため、`generations`/`removed`を
    /// 経由しないと削除済みの亡霊ボディまで書き出してしまう)。
    pub fn body_ids(&self) -> Vec<BodyId> {
        self.generations
            .iter()
            .enumerate()
            .filter(|(idx, _)| !self.removed[*idx])
            .map(|(idx, gen)| BodyId {
                index: idx as u32,
                generation: *gen,
            })
            .collect()
    }

    /// `body`のローカル座標`anchor_local`をワールド固定点`anchor_world`から
    /// 距離`length`に保つ`sim_mechanics::DistanceJoint`(`body_b: None`、
    /// モジュールdoc「Distance は単振り子(質量無しの棒/紐)」)を追加する。
    /// 振り子スポーン(`sim-wasm::spawn_pendulum`)向け。返り値は
    /// `distance_joint_anchor_points`で問い合わせる際に使う`joints`内の
    /// index。無効な`body`でも(`create_body`直後の呼び出しのみを想定して
    /// いるため)検証はしない。
    pub fn add_distance_joint_to_world_point(
        &mut self,
        body: BodyId,
        anchor_local: Vec3,
        anchor_world: Vec3,
        length: f64,
    ) -> usize {
        let index = self.mechanics.joints.len();
        self.mechanics
            .add_distance_joint(sim_mechanics::DistanceJoint {
                body_a: body.index as usize,
                anchor_a: anchor_local,
                body_b: None,
                anchor_b: anchor_world,
                length,
                disabled: false,
            });
        index
    }

    /// `joint_index`番目のDistanceJointの現在のワールド座標アンカー点2点
    /// `(anchor_a_world, anchor_b_world)`を返す(`body_b`が`None`なら
    /// `anchor_b`はそのままワールド座標)。`sim_mechanics::joint`内部の
    /// `world_anchor`と同じ式(`position + rotation.rotate(anchor_local)`)を
    /// 使う。Scene Viewの拘束オーバーレイ(設計docs/23-frontend/01-editor.md
    /// §1.2)がジョイントを結ぶ線を描くためのクエリ。範囲外の`joint_index`
    /// なら`None`。
    pub fn distance_joint_anchor_points(&self, joint_index: usize) -> Option<(Vec3, Vec3)> {
        let joint = self.mechanics.joints.get(joint_index)?;
        let body_a = joint.body_a;
        let anchor_world = |body: usize, anchor_local: Vec3| {
            // `sim_mechanics::joint::world_anchor` と同じ式。群11でアンカーは
            // **形状ローカル**、`position`は**重心**になったため、両者の差
            // (`center_of_mass`)を引いてから回す。
            let from_com = anchor_local - self.mechanics.bodies.center_of_mass[body];
            self.mechanics.bodies.position[body]
                + self.mechanics.bodies.rotation[body].rotate(from_com)
        };
        let anchor_a_world = anchor_world(body_a, joint.anchor_a);
        let anchor_b_world = match joint.body_b {
            Some(body_b) => anchor_world(body_b, joint.anchor_b),
            None => joint.anchor_b,
        };
        Some((anchor_a_world, anchor_b_world))
    }

    /// `JointDesc`からジョイントを1本作り、対応する種別のVec内でのindexを返す
    /// (`JointInfo::index`・`joints_for_body`と同じ体系。種別が違えば同じ数値の
    /// indexが重複しうるので、識別には種別と組で使うこと)。
    ///
    /// `add_distance_joint_to_world_point`と同じ理由で`body`の生存確認はしない
    /// (呼び出し側——`sim-wasm`の場合は生存確認済みの`BodyId`を渡す前提、
    /// `try_body_id_at`のdoc参照)。
    pub fn create_joint(&mut self, desc: JointDesc) -> usize {
        match desc {
            JointDesc::Distance {
                body_a,
                anchor_a,
                body_b,
                anchor_b,
                length,
            } => {
                let index = self.mechanics.joints.len();
                self.mechanics
                    .add_distance_joint(sim_mechanics::DistanceJoint {
                        body_a: body_a.index as usize,
                        anchor_a,
                        body_b: body_b.map(|id| id.index as usize),
                        anchor_b,
                        length,
                        disabled: false,
                    });
                index
            }
            JointDesc::Ball {
                body_a,
                anchor_a,
                body_b,
                anchor_b,
            } => {
                let index = self.mechanics.ball_joints.len();
                self.mechanics.add_ball_joint(sim_mechanics::BallJoint {
                    body_a: body_a.index as usize,
                    anchor_a,
                    body_b: body_b.map(|id| id.index as usize),
                    anchor_b,
                    disabled: false,
                });
                index
            }
            JointDesc::Slider {
                body_a,
                anchor_a,
                axis,
                body_b,
                anchor_b,
            } => {
                let index = self.mechanics.slider_joints.len();
                let joint = sim_mechanics::SliderJoint::new(
                    &self.mechanics.bodies,
                    body_a.index as usize,
                    anchor_a,
                    axis,
                    body_b.map(|id| id.index as usize),
                    anchor_b,
                );
                self.mechanics.add_slider_joint(joint);
                index
            }
            JointDesc::Wheel {
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
                let index = self.mechanics.wheel_joints.len();
                let mut joint = sim_mechanics::WheelJoint::new(
                    chassis.index as usize,
                    wheel.index as usize,
                    anchor_chassis,
                    rest_length,
                );
                joint.suspension_axis = suspension_axis;
                joint.axle_axis = axle_axis;
                joint.soft.frequency = frequency;
                joint.soft.damping_ratio = damping_ratio;
                joint.steer_angle = steer_angle;
                joint.motor_speed = motor_speed;
                joint.motor_max_torque = motor_max_torque;
                self.mechanics.wheel_joints.push(joint);
                index
            }
            JointDesc::HingeMotor {
                body,
                axis,
                reference_rotation,
                theta_target,
                kp,
                kd,
                torque_max,
                limit,
            } => {
                let index = self.mechanics.hinge_motors.len();
                let reference_rotation = reference_rotation
                    .unwrap_or(self.mechanics.bodies.rotation[body.index as usize]);
                self.mechanics.add_hinge_motor(sim_mechanics::HingeMotorPd {
                    body: body.index as usize,
                    axis,
                    reference_rotation,
                    theta_target,
                    kp,
                    kd,
                    torque_max,
                    limit,
                    disabled: false,
                });
                index
            }
        }
    }

    /// 流体領域を1つ**追加する**(設計 docs/20-integration/04-world-api.md §2
    /// `add_fluid_region(desc: FluidDesc) -> FluidId`)。集中定数の浮力領域
    /// (`sim_fluid::FluidRegion`)のみを対象とする——SPH・格子流体は
    /// `enable_sph`/`enable_grid_fluid`が別に受け持つ(モジュールdoc参照)。
    ///
    /// **移行前は「置き換え」だった**(`World`は水域を高々1つしか持てなかった)。
    /// 領域が`Vec`になったので素直な追加になり、戻り値でindex(設計の`FluidId`
    /// 相当)を返す。重なった領域の決着規則は
    /// `sim_mechanics::MechanicsSolver::fluids`のdoc参照——**追加した順が
    /// そのまま優先順位**になる。全消しは`clear_fluid_regions`。
    pub fn add_fluid_region(&mut self, region: sim_fluid::FluidRegion) -> usize {
        self.mechanics.fluids.push(region);
        self.mechanics.fluids.len() - 1
    }

    /// 登録済みの流体領域一覧(追加順=優先順)。
    pub fn fluid_regions(&self) -> &[sim_fluid::FluidRegion] {
        &self.mechanics.fluids
    }

    /// 流体領域を全て取り除く。
    pub fn clear_fluid_regions(&mut self) {
        self.mechanics.fluids.clear();
    }

    /// 現在の環境設定を`EnvironmentDesc`として読む(Inspectorの環境パネルが
    /// 表示・編集フォームの初期値に使う)。
    pub fn environment(&self) -> EnvironmentDesc {
        EnvironmentDesc {
            gravity: self.mechanics.gravity(),
            gravity_direction: self.mechanics.gravity_direction(),
            gravity_field: Some(self.mechanics.gravity_field()),
            atmosphere: self.mechanics.atmosphere,
            fluids: self.mechanics.fluids.clone(),
            ambient_temperature: self.thermal.as_ref().map(|t| t.ambient_temperature),
        }
    }

    /// `EnvironmentDesc`をまとめて適用する(Inspectorの環境パネルの確定操作)。
    /// `ambient_temperature`は熱ドメインが有効な場合のみ反映する(`None`のままの
    /// 熱ドメインを勝手に`enable_thermal`しない——ドメインの有効化は
    /// シーン構築の責務であり、環境設定の責務ではないため)。
    pub fn set_environment(&mut self, desc: EnvironmentDesc) {
        // `gravity_field`が`Some`ならそれが唯一の情報源(構造体docの優先規則)。
        // `None`のときだけ、スカラー2つから一様場を組み立てる従来の経路を通る。
        match desc.gravity_field {
            Some(field) => self.mechanics.set_gravity_field(field),
            None => {
                self.mechanics.set_gravity(desc.gravity);
                self.mechanics.set_gravity_direction(desc.gravity_direction);
            }
        }
        self.mechanics.atmosphere = desc.atmosphere;
        self.mechanics.fluids = desc.fluids;
        if let Some(t) = desc.ambient_temperature {
            if let Some(thermal) = self.thermal.as_mut() {
                thermal.ambient_temperature = t;
            }
        }
    }

    /// 直近stepで検出された接触点のワールド座標一覧(設計docs/23-frontend/
    /// 01-editor.md §1.2 Scene View オーバーレイ「接触点」向け)。法線・貫入量は
    /// このオーバーレイの用途(接触位置のマーカー表示)には不要なため座標のみ返す
    /// 縮約実装(`sim_mechanics::MechanicsSolver::last_manifolds`をそのまま使う)。
    pub fn contact_points(&self) -> Vec<Vec3> {
        self.mechanics
            .last_manifolds
            .iter()
            .flat_map(|m| m.points.iter().map(|p| p.world_point))
            .collect()
    }

    /// `filter`が`index`のボディを受け入れるか。
    fn filter_accepts(&self, filter: &QueryFilter, index: usize) -> bool {
        // `body_type`はメソッドではなく`Vec<BodyType>`フィールド。
        let is_static = self.mechanics.bodies.body_type[index] == sim_mechanics::BodyType::Static;
        if filter.exclude_static && is_static {
            return false;
        }
        if filter.exclude_dynamic && !is_static {
            return false;
        }
        // 衝突フィルタ(設計 docs/10-mechanics/02-collision-detection.md §4.1)。
        // **クエリ側は単方向 AND** ——「このマスクで見えるものを拾う」という
        // 問い合わせであり、broadphase の接触ペア(双方向 AND、運動量保存のため)
        // とは意味論が違う。既定の `mask: None` は何も絞らない。
        if let Some(mask) = filter.collision_mask {
            if (mask & self.mechanics.bodies.collision_group[index]) == 0 {
                return false;
            }
        }
        !filter
            .exclude
            .iter()
            .any(|id| id.index as usize == index && self.generations[index] == id.generation)
    }

    /// レイキャストクエリ(設計docs/20-integration/04-world-api.md §2
    /// `raycast(origin, dir, max, filter)`、`raycast`モジュールdoc参照)。
    ///
    /// **増分F1で`filter`引数を追加した**。`QueryFilter::default()`は全て受け入れる
    /// ので、フィルタ不要な呼び出しはそれを渡せばよい。
    ///
    /// **縮約(正直な記録)**: フィルタは受理判定を**ヒット後**に適用する。
    /// `raycast::raycast`は最近傍1件だけを返す実装なので、「最近傍が除外対象
    /// だった場合に次の候補を返す」ことはできず`None`になる。除外対象を跨いだ
    /// 背後のボディを取りたい場合は、`raycast`側に「全ヒットを距離順に返す」
    /// APIを足す必要があり、それは後続増分の対象とする。
    pub fn raycast(
        &self,
        origin: Vec3,
        dir: Vec3,
        max_distance: f64,
        filter: &QueryFilter,
    ) -> Option<RayHit> {
        raycast::raycast(&self.mechanics.bodies, origin, dir, max_distance)
            .filter(|hit| self.filter_accepts(filter, hit.body_index))
            .map(|hit| RayHit {
                body: BodyId {
                    index: hit.body_index as u32,
                    generation: self.generations[hit.body_index],
                },
                point: hit.point,
                normal: hit.normal,
                distance: hit.distance,
            })
    }

    /// 球オーバーラップクエリ(設計docs/20-integration/04-world-api.md §2
    /// `overlap_sphere(center, r, filter)`、`overlap`モジュールdoc参照)。
    /// `raycast`と違い全件を返すので、フィルタは素直に絞り込みとして効く。
    pub fn overlap_sphere(&self, center: Vec3, r: f64, filter: &QueryFilter) -> Vec<BodyId> {
        overlap::overlap_sphere(&self.mechanics.bodies, center, r)
            .into_iter()
            .filter(|&index| self.filter_accepts(filter, index))
            .map(|index| BodyId {
                index: index as u32,
                generation: self.generations[index],
            })
            .collect()
    }

    /// 回路ノード`node`の電位(電圧計、設計docs/20-integration/04-world-api.md §2
    /// `circuit_probe(id, node)`)。設計は複数回路を`CircuitId`で選ぶが、`World`は
    /// 現時点で単一の`circuit`ドメインしか持たないため`id`引数は省略する(縮約実装、
    /// 複数回路対応時に`CircuitId`を導入して拡張する)。回路ドメインが未有効化なら
    /// `None`。
    pub fn circuit_probe(&self, node: usize) -> Option<f64> {
        self.circuit.as_ref().map(|c| c.node_voltage(node))
    }

    /// `step()`が排出した全イベントを取り出しつつ`event_log`を空にする(設計
    /// docs/20-integration/04-world-api.md §2「イベント購読」)。
    ///
    /// **縮約実装の理由**: 設計の`subscribe(kind, sub) -> Subscription` +
    /// `drain_events(sub) -> Vec<Event>`は複数の独立した購読者(`SubscriberId`ごとに
    /// 別々の未読カーソル・`EventKind`フィルタ)を想定するが、現時点でイベントの
    /// 消費者(フロントエンド等)が存在しないため、`SubscriberId`/`Subscription`型は
    /// まだ導入せず、単一の共有履歴(`event_log`、固定容量`RingBuffer`)を全消費者が
    /// 共有する縮約版とする(`kind`によるフィルタも呼び出し側が`Vec`をフィルタする)。
    /// 複数購読者・フィルタ登録が必要になった時点で`SubscriberId`ごとの独立カーソルへ
    /// 拡張する。イベントの生産者は現時点で`sim_mechanics::MechanicsSolver::
    /// emit_contact_events`(`ContactStarted`/`ContactEnded`)のみ。
    pub fn drain_events(&mut self) -> Vec<sim_core::Event> {
        self.event_log.drain().collect()
    }

    /// `sim_coupling::Coupling`を`World`が保持する実ドメインに対して1回適用する。
    ///
    /// **縮約実装の理由**: 設計は`Coupling`を`World::step()`内部のLie-Trotter
    /// operator splittingパイプライン(pre/postの2相、docs/20-integration/
    /// 01-coupling-matrix.md §4)へ自動的に組み込む想定であり、プログラムから登録した
    /// `Coupling`を毎stepの後に自動適用するレジストリ自体は`add_coupling`/`couplings`
    /// フィールドとして実装済み(このメソッドのdoc下部参照)。**pre/post 2相への分離も
    /// 群5で完了**(`step()`のdoc参照)。シーンJSON`couplings`セクションからの
    /// 自動解決・排他結合検査(`sim-coupling::validate_exclusive_couplings`)との
    /// 接続は未実装(`from_scenario`のモジュールdoc参照)。
    /// 本メソッドは、`add_coupling`によるレジストリ登録より前から存在する、呼び出し側が
    /// 呼び出し頻度・タイミングを明示的に管理する下位のプリミティブとして残している
    /// (統合シナリオテストの一部・レジストリ自体の内部実装が使う)。`step()`の後に
    /// 呼ぶ場合、`DissipationToHeat`・`JouleHeat`の
    /// ように直近stepで確定した量(`last_contact_dissipation`・`resistor_power`等)を
    /// 読むCoupling(design上の"post")は正しく機能するが、`BrownianForce`・
    /// `LorentzForce`のように力・速度を注入し同stepの位置積分に反映されるべき
    /// Coupling(design上の"pre")は、その注入が次の`step()`まで反映されない
    /// 1step遅れが生じる。**`step()`経由(レジストリ経由)ならこの遅れは無い**
    /// ——群5でこれらの結合を`apply_pre`へ移し、`step()`がドメインソルバの前に
    /// pre 相を呼ぶようにしたため(各`sim-coupling`実装のモジュールdoc参照)。
    /// この下位プリミティブを直接使う呼び出し側だけが上記の注意点を負う。
    pub fn apply_coupling(&mut self, coupling: &mut dyn sim_coupling::Coupling, dt: f64) {
        let mut states = sim_coupling::DomainStates {
            mechanics: &mut self.mechanics,
            thermal: self.thermal.as_mut(),
            em_circuit: self.circuit.as_mut(),
            em_electrostatics: self.em_electrostatics.as_mut(),
            gas: self.gas.as_mut(),
            grid_fluid: self.grid_fluid.as_mut(),
            grid_fluid_3d: self.grid_fluid_3d.as_mut(),
            sph: self.sph.as_mut(),
        };
        coupling.apply(&mut states, dt);
    }

    /// `Coupling`をレジストリに登録する。以後`step()`が毎フレーム自動的に、登録順で
    /// 1回ずつ`.apply()`を呼ぶ(`apply_coupling`のdocが説明する「post」型タイミングと
    /// 同じ — `step()`内の全ドメインsub-step完了後)。呼び出し側が毎stepの後に手動で
    /// `apply_coupling`を呼ぶ手間を無くす、Coupling registryの縮約版(シーンJSON
    /// `couplings`セクションからの自動解決・排他結合検査との接続は後続増分、
    /// `scenario`モジュールdoc参照)。
    /// 戻り値はこのCouplingの登録index(`CouplingInfo::index`と同じ並び、
    /// `Command::SetCouplingParam`が参照するindex——**残タスク完遂の縦串⑤増分**
    /// で追加、それまでは戻り値`()`で呼び出し側は捨てるだけだった)。
    pub fn add_coupling(&mut self, coupling: Box<dyn sim_coupling::Coupling>) -> usize {
        self.couplings.push(coupling);
        self.couplings.len() - 1
    }

    /// 全状態(clock + 有効な全ドメイン)を決定的順序(ドメイン登録順固定:
    /// mechanics→thermal→em→astro→circuit、
    /// 設計docs/20-integration/02-determinism-replay.md §3)で
    /// ハッシュする。各`Option`ドメインは有効/無効自体も書き込む(構造の異なる2つのWorldが
    /// 偶然衝突するリスクを減らす)。
    pub fn state_hash(&self) -> u64 {
        let mut hasher = StateHasher::new();
        hasher.write_u64(self.clock.step_count());
        hasher.write_f64(self.clock.time());
        self.mechanics.state_hash(&mut hasher);
        hasher.write_u64(self.thermal.is_some() as u64);
        if let Some(t) = &self.thermal {
            t.state_hash(&mut hasher);
        }
        hasher.write_u64(self.em_electrostatics.is_some() as u64);
        if let Some(e) = &self.em_electrostatics {
            e.state_hash(&mut hasher);
        }
        hasher.write_u64(self.astro.is_some() as u64);
        if let Some(a) = &self.astro {
            a.state_hash(&mut hasher);
        }
        hasher.write_u64(self.circuit.is_some() as u64);
        if let Some(c) = &self.circuit {
            c.state_hash(&mut hasher);
        }
        hasher.write_u64(self.sph.is_some() as u64);
        if let Some(s) = &self.sph {
            s.state_hash(&mut hasher);
        }
        hasher.write_u64(self.grid_fluid.is_some() as u64);
        if let Some(g) = &self.grid_fluid {
            g.state_hash(&mut hasher);
        }
        hasher.write_u64(self.grid_fluid_3d.is_some() as u64);
        if let Some(g) = &self.grid_fluid_3d {
            g.state_hash(&mut hasher);
        }
        // **増分Hで追加**。`World::step()`が回すようになった以上、状態ハッシュにも
        // 含めないと決定論の検証(D8のようなハッシュ一致)がこの2ドメインの
        // 差分を見逃す。
        hasher.write_u64(self.soft_body.is_some() as u64);
        if let Some(b) = &self.soft_body {
            b.state_hash(&mut hasher);
        }
        hasher.write_u64(self.conduction_rod.is_some() as u64);
        if let Some(r) = &self.conduction_rod {
            r.state_hash(&mut hasher);
        }
        // **群3で追加**。同じ理由——`step()`が回す以上、ハッシュにも含めないと
        // 決定論の検証がこの6ドメインの差分を見逃す。
        hasher.write_u64(self.quantum_1d.is_some() as u64);
        if let Some(q) = &self.quantum_1d {
            q.state_hash(&mut hasher);
        }
        hasher.write_u64(self.quantum_2d.is_some() as u64);
        if let Some(q) = &self.quantum_2d {
            q.state_hash(&mut hasher);
        }
        hasher.write_u64(self.brownian.is_some() as u64);
        if let Some(b) = &self.brownian {
            b.state_hash(&mut hasher);
        }
        hasher.write_u64(self.kinetic_gas.is_some() as u64);
        if let Some(g) = &self.kinetic_gas {
            g.state_hash(&mut hasher);
        }
        hasher.write_u64(self.ising.is_some() as u64);
        if let Some(i) = &self.ising {
            i.state_hash(&mut hasher);
        }
        hasher.write_u64(self.fdtd.is_some() as u64);
        if let Some(f) = &self.fdtd {
            f.state_hash(&mut hasher);
        }
        match self.time_regime {
            sim_astro::TimeRegime::Local { steps_per_frame } => {
                hasher.write_u64(0);
                hasher.write_u64(steps_per_frame as u64);
            }
            sim_astro::TimeRegime::Astro {
                dt_astro,
                steps_per_frame,
            } => {
                hasher.write_u64(1);
                hasher.write_f64(dt_astro);
                hasher.write_u64(steps_per_frame as u64);
            }
        }
        hasher.finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_math::{Quat, Transform};
    use sim_mechanics::{BodyType, Shape};

    const INITIAL_HEIGHT: f64 = 10.0;

    /// Phase 0 相当の「箱1個が落ちる」シーンを構築する(鋼の箱、高さ `INITIAL_HEIGHT`)。
    fn create_falling_box(world: &mut World) -> BodyId {
        let steel = world
            .materials()
            .find_by_name("鋼(炭素鋼)")
            .expect("standard DB has steel");
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(0.5, 0.5, 0.5),
            },
            steel,
        );
        desc.body_type = BodyType::Dynamic;
        desc.transform = Transform {
            position: Vec3::new(0.0, INITIAL_HEIGHT, 0.0),
            rotation: sim_math::Quat::IDENTITY,
        };
        world.create_body(desc)
    }

    /// y=0 の静的な無限平面を敷く(衝突フィルタのテストで「床をすり抜けるか」を
    /// 見るのに使う)。
    fn add_ground_plane(world: &mut World) -> BodyId {
        let steel = world.materials().find_by_name("鋼(炭素鋼)").unwrap();
        let mut floor = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            steel,
        );
        floor.body_type = BodyType::Static;
        world.create_body(floor)
    }

    #[test]
    fn box_falls_and_test_is_green() {
        let mut world = World::new(WorldOptions::default());
        let idx = create_falling_box(&mut world);
        let y0 = world.body_position(idx).unwrap().y;
        for _ in 0..120 {
            world.step();
        }
        assert!(world.body_position(idx).unwrap().y < y0);
        assert_eq!(world.step_count(), 120);
    }

    /// 複数剛体: create_body を複数回呼んでも各 body が独立に扱えること。
    #[test]
    fn multiple_bodies_are_independently_addressable() {
        let mut world = World::new(WorldOptions::default());
        let a = create_falling_box(&mut world);
        let steel = world.materials().find_by_name("鋼(炭素鋼)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.2 }, steel);
        desc.body_type = BodyType::Static; // 静止参照点(比較用)
        desc.transform.position = Vec3::new(3.0, 2.0, 0.0);
        let b = world.create_body(desc);

        for _ in 0..60 {
            world.step();
        }
        assert!(
            world.body_position(a).unwrap().y < INITIAL_HEIGHT,
            "a should fall"
        );
        assert_eq!(
            world.body_position(b).unwrap(),
            Vec3::new(3.0, 2.0, 0.0),
            "static body must not move"
        );
    }

    /// 世代付き`BodyId`(設計 docs/00-foundation/04-architecture.md §3)の不変条件:
    /// 削除済み ID へのアクセスは `None`(パニックしない)。同じインデックスへの新規
    /// `create_body`(現時点では `RigidBodySet` がスロット再利用に未対応のため実際には
    /// 新しいインデックスになるが、`is_valid`の世代比較ロジック自体はどちらの場合も
    /// 正しく機能する)。
    #[test]
    fn removed_body_id_returns_none_and_does_not_panic() {
        let mut world = World::new(WorldOptions::default());
        let a = create_falling_box(&mut world);
        assert!(world.body_position(a).is_some());

        world.remove_body(a);
        assert!(
            world.body_position(a).is_none(),
            "removed body id must resolve to None, not panic"
        );

        // 削除後も他のボディ・ステップ実行は正常に動作する(パニックしない)。
        let b = create_falling_box(&mut world);
        for _ in 0..10 {
            world.step();
        }
        assert!(world.body_position(b).is_some());
        assert!(world.body_position(a).is_none());
    }

    /// `is_body_alive`(`sim-wasm`の`try_body_id_at`がTimeline巻き戻し後の
    /// 生存確認に使う、モジュールdoc参照)が、スナップショットより後に作られた
    /// ボディを正しく「もう存在しない」と判定すること。また、そのボディの
    /// `index`が復元後の`RigidBodySet`の各`Vec`の範囲外になること(=
    /// 生存確認をせずに生indexアクセスすると範囲外パニックになりうること)も
    /// 確認する——`is_body_alive`がこの危険を防ぐ根拠そのもの。
    #[test]
    fn is_body_alive_detects_bodies_that_did_not_exist_yet_at_an_earlier_snapshot() {
        let mut world = World::new(WorldOptions::default());
        let a = create_falling_box(&mut world);
        let snapshot = world.snapshot();

        let b = create_falling_box(&mut world);
        assert!(world.is_body_alive(a));
        assert!(world.is_body_alive(b));

        world.restore(&snapshot);

        assert!(
            world.is_body_alive(a),
            "body created before the snapshot must still be alive after restore"
        );
        assert!(
            !world.is_body_alive(b),
            "body created after the snapshot must not be alive after restoring to it"
        );
        // これが `is_body_alive` の確認を省いて `mechanics().bodies.position\
        // [b.index as usize]` のような生indexアクセスをすると危険な理由:
        // 復元後の `RigidBodySet` は `b` を一度も知らないので、そのindexは
        // 配列長の範囲外になる。
        assert!(
            b.index as usize >= world.mechanics().bodies.position.len(),
            "the restored World's RigidBodySet must be shorter than the removed body's index"
        );
    }

    /// `create_joint`が`JointDesc`の5種すべてを正しい種別のVecへ積むこと
    /// (`World::joints()`の内省で種別・パラメータを確認する)。
    #[test]
    fn create_joint_adds_each_joint_desc_variant_to_its_kind() {
        let mut world = World::new(WorldOptions::default());
        let a = create_falling_box(&mut world);
        let b = create_falling_box(&mut world);

        world.create_joint(JointDesc::Distance {
            body_a: a,
            anchor_a: Vec3::ZERO,
            body_b: Some(b),
            anchor_b: Vec3::ZERO,
            length: 2.0,
        });
        world.create_joint(JointDesc::Ball {
            body_a: a,
            anchor_a: Vec3::ZERO,
            body_b: None,
            anchor_b: Vec3::new(0.0, 5.0, 0.0),
        });
        world.create_joint(JointDesc::Slider {
            body_a: a,
            anchor_a: Vec3::ZERO,
            axis: Vec3::new(0.0, 0.0, 1.0),
            body_b: None,
            anchor_b: Vec3::ZERO,
        });
        world.create_joint(JointDesc::Wheel {
            chassis: a,
            wheel: b,
            anchor_chassis: Vec3::new(0.0, -0.3, 0.0),
            rest_length: 0.4,
            suspension_axis: Vec3::new(0.0, -1.0, 0.0),
            axle_axis: Vec3::new(1.0, 0.0, 0.0),
            frequency: 2.0,
            damping_ratio: 0.3,
            steer_angle: 0.1,
            motor_speed: 1.0,
            motor_max_torque: 10.0,
        });
        world.create_joint(JointDesc::HingeMotor {
            body: a,
            axis: Vec3::new(0.0, 1.0, 0.0),
            reference_rotation: None,
            theta_target: 0.5,
            kp: 1.0,
            kd: 0.1,
            torque_max: 5.0,
            limit: None,
        });

        let kinds: Vec<JointKind> = world.joints().iter().map(|j| j.kind).collect();
        assert!(kinds.contains(&JointKind::Distance));
        assert!(kinds.contains(&JointKind::Ball));
        assert!(kinds.contains(&JointKind::Slider));
        assert!(kinds.contains(&JointKind::HingeMotor));
        assert_eq!(world.mechanics().wheel_joints.len(), 1);
        assert_eq!(world.mechanics().wheel_joints[0].motor_speed, 1.0);
    }

    /// `add_fluid_region`が`MechanicsSolver::fluids`へ**追加**すること
    /// (**移行前は置き換えだった**)。indexが登録順=優先順を表し、
    /// `clear_fluid_regions`で全消しできる。
    #[test]
    fn add_fluid_region_appends_regions_in_registration_order() {
        let mut world = World::new(WorldOptions::default());
        assert_eq!(
            world.add_fluid_region(sim_fluid::FluidRegion::new(1.0, 1000.0)),
            0
        );
        assert_eq!(
            world.add_fluid_region(sim_fluid::FluidRegion::new(2.0, 1000.0)),
            1
        );

        let regions = world.fluid_regions();
        assert_eq!(regions.len(), 2);
        assert_eq!(regions[0].water_level, 1.0);
        assert_eq!(regions[1].water_level, 2.0);
        // `mechanics()`側と同じものを見ている。
        assert_eq!(world.mechanics().fluids.len(), 2);

        world.clear_fluid_regions();
        assert!(world.fluid_regions().is_empty());
    }

    /// `environment`/`set_environment`が重力(大きさ・向き)・大気・水域・
    /// 周囲温度を往復できること。向きの往復確認は**残タスク完遂増分**
    /// (レビュー指摘「見送らず対応すること」への対応で`gravity_direction`を
    /// 追加した際に拡張)。
    #[test]
    fn environment_desc_round_trips_gravity_atmosphere_water_and_ambient_temperature() {
        let mut world = World::new(WorldOptions::default());
        world.enable_thermal(sim_thermal::ThermalSolver::new(293.15));

        let desc = EnvironmentDesc {
            gravity: 3.71, // 火星の重力
            gravity_direction: sim_math::Vec3::new(1.0, 0.0, 0.0),
            // `None` = 従来のスカラー2つ経路(構造体docの優先規則)。
            gravity_field: None,
            atmosphere: Some(sim_fluid::Atmosphere::still(0.02, 1.1e-5)),
            // **流体領域の一般化**: 複数領域・形状・水温をまとめて往復する。
            fluids: vec![
                sim_fluid::FluidRegion::new(-1.0, 1000.0),
                sim_fluid::FluidRegion::aabb(
                    sim_math::Vec3::new(-1.0, -2.0, -1.0),
                    sim_math::Vec3::new(1.0, 0.5, 1.0),
                    0.0,
                    1200.0,
                )
                .with_temperature(277.0),
            ],
            ambient_temperature: Some(210.0),
        };
        world.set_environment(desc.clone());

        let read_back = world.environment();
        assert_eq!(read_back.gravity, 3.71);
        assert_eq!(
            read_back.gravity_direction,
            sim_math::Vec3::new(1.0, 0.0, 0.0)
        );
        assert_eq!(read_back.atmosphere.unwrap().density, 0.02);
        assert_eq!(read_back.fluids, desc.fluids);
        assert_eq!(read_back.fluids[0].water_level, -1.0);
        assert_eq!(read_back.fluids[1].temperature, Some(277.0));
        assert_eq!(read_back.ambient_temperature, Some(210.0));
        assert_eq!(world.thermal().unwrap().ambient_temperature, 210.0);
    }

    /// **重力場の抽象化増分**: `EnvironmentDesc`が`GravityField`の3種すべてを
    /// 往復できること、そして`gravity_field: Some(..)`がスカラー2つ
    /// (`gravity`/`gravity_direction`)より優先されること(構造体docの優先規則)。
    #[test]
    fn environment_desc_round_trips_every_gravity_field_kind_and_field_wins() {
        let fields = [
            sim_mechanics::GravityField::Uniform {
                magnitude: 1.62, // 月の重力
                direction: sim_math::Vec3::new(0.0, -1.0, 0.0),
            },
            sim_mechanics::GravityField::PointSource {
                center: sim_math::Vec3::new(1.0, 2.0, 3.0),
                mu: 4.0e5,
            },
            sim_mechanics::GravityField::Zero,
        ];
        for field in fields {
            let mut world = World::new(WorldOptions::default());
            world.set_environment(EnvironmentDesc {
                // わざと場と矛盾する値を入れる——優先規則が効いていれば
                // これらは無視されるはず。
                gravity: 99.0,
                gravity_direction: sim_math::Vec3::new(1.0, 0.0, 0.0),
                gravity_field: Some(field),
                atmosphere: None,
                fluids: Vec::new(),
                ambient_temperature: None,
            });
            assert_eq!(world.environment().gravity_field, Some(field));
            assert_eq!(world.mechanics().gravity_field(), field);
        }
    }

    /// **重力場の抽象化増分**: `Command::SetGravityField`が次stepの先頭で適用され、
    /// `command_log()`へ記録されること(同variantのdoc「黙ってリプレイされない
    /// 変更は決定論のバグ」)。
    #[test]
    fn set_gravity_field_command_applies_and_is_recorded_in_the_command_log() {
        let mut world = World::new(WorldOptions::default());
        let field = sim_mechanics::GravityField::PointSource {
            center: sim_math::Vec3::ZERO,
            mu: 3.986e14,
        };
        world.push_command(Command::SetGravityField { field });
        // Commandは「次step先頭」で効く——push直後はまだ既定の一様場のまま。
        assert!(matches!(
            world.mechanics().gravity_field(),
            sim_mechanics::GravityField::Uniform { .. }
        ));
        world.step();
        assert_eq!(world.mechanics().gravity_field(), field);
        assert_eq!(world.command_log().len(), 1);
        assert_eq!(world.command_log()[0].1, Command::SetGravityField { field });
    }

    /// `Shape::Compound`の`todo!()`穴埋め(統合エディタ実装計画の縦串①)の
    /// エンドツーエンド検証: L字形(2つの箱を組んだ)のCompoundボディを作り、
    /// 地面(y=0の平面)へ落として`step`を回す——質量照会でパニックしない
    /// (これがtodo!()の直接の症状だった)ことと、接触解決を経て床の上に
    /// 静止することの両方を確認する。
    #[test]
    fn compound_body_can_be_created_and_settles_on_the_ground_without_panicking() {
        let mut world = World::new(WorldOptions::default());
        let steel = world.materials().find_by_name("鋼(炭素鋼)").unwrap();
        let concrete = world.materials().find_by_name("コンクリート").unwrap();

        let mut ground = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            concrete,
        );
        ground.body_type = BodyType::Static;
        world.create_body(ground);

        // L字形: 縦棒(0.5×2.0×0.5)+横棒(1.0×0.5×0.5、縦棒の下端に接続)。
        let l_shape = Shape::Compound {
            children: vec![
                (
                    Transform {
                        position: Vec3::new(0.0, 0.75, 0.0),
                        rotation: sim_math::Quat::IDENTITY,
                    },
                    Shape::Box {
                        half_extents: Vec3::new(0.25, 1.0, 0.25),
                    },
                ),
                (
                    Transform {
                        position: Vec3::new(0.25, -0.25, 0.0),
                        rotation: sim_math::Quat::IDENTITY,
                    },
                    Shape::Box {
                        half_extents: Vec3::new(0.5, 0.25, 0.25),
                    },
                ),
            ],
        };
        let mut desc = RigidBodyDesc::dynamic(l_shape, steel);
        desc.transform.position = Vec3::new(0.0, 5.0, 0.0);
        // `create_body`が内部で`shape.volume()`/`unit_mass_inertia_diagonal()`を
        // 呼ぶ——todo!()が残っていればここで即パニックしていた。
        let body = world.create_body(desc);

        // L字形は非対称なので、着地後にわずかに揺れてから静止するまで
        // 単純な箱よりも時間がかかる(実測: 約12秒)。余裕を見て20秒回す。
        for _ in 0..1200 {
            world.step();
        }

        let final_speed = world.body_velocity(body).unwrap().length();
        assert!(
            final_speed < 0.05,
            "10秒後には静止しているはず(速さ={final_speed})"
        );

        // **群11で期待値を意図的に更新**。
        //
        // 移行前はここで「静止時の本体原点は概ね y=0.5」(=横棒で直立した
        // まま)を要求していた。しかしそれは**重心オフセットが未実装だった
        // ことによる見かけの安定**だった:
        //   ① 慣性テンソルをローカル原点まわりで計算していたため、真の重心
        //      まわりの値より大きく(平行軸定理のぶん)、回転しにくかった。
        //   ② 重力・接触力のトルクをローカル原点まわりに立てていたため、
        //      重心が x=+0.083 にずれていることによる転倒モーメントが
        //      そもそも発生しなかった。
        // 群11で①②を正した結果、**4.5mの自由落下 → 剛なコンクリートでの
        // バウンド**という高エネルギーな着地では、L字は跳ねたあと横倒しに
        // なって静止する(実測 tilt≈1.574 rad ≒ 90°)。
        //
        // これが「物理が壊れた」のではなく「正しくなった」ことは、静的な
        // つり合いを別途確認して裏付けた——解析的な静止高さ y=0.5 へそっと
        // 置くと、L字は tilt≈1e-4 rad で直立したまま静止する
        // (`compound_l_shape_placed_at_rest_stays_upright`)。重心 x=0.083 は
        // 横棒の支持多角形 x∈[-0.25,0.75] の内側なので直立は安定であり、
        // 横倒しはバウンドの動力学の結果であって静的な誤りではない。
        //
        // したがってここでは姿勢に依存しない不変量——**形状の最下点が床の
        // 上に乗っていること**——だけを要求する。
        let lowest = lowest_world_y_of_compound(&world, body);
        assert!(
            (-0.01..0.05).contains(&lowest),
            "姿勢によらず形状の最下点が床に接しているはず(最下点y={lowest})"
        );
    }

    /// 複合剛体のワールド空間での最下点の y。姿勢に依存しない「床に乗っている」
    /// 判定に使う(部品はすべて`Box`前提の簡易版、テスト専用)。
    fn lowest_world_y_of_compound(world: &World, body: BodyId) -> f64 {
        let bodies = &world.mechanics().bodies;
        let idx = body.index as usize;
        let xf = bodies.shape_transform(idx);
        let Shape::Compound { children } = bodies.shape_of(idx) else {
            panic!("expected a compound shape");
        };
        let mut lowest = f64::INFINITY;
        for (child_xf, child) in children {
            let Shape::Box { half_extents } = child else {
                continue;
            };
            let world_child = xf.compose(*child_xf);
            for sx in [-1.0, 1.0] {
                for sy in [-1.0, 1.0] {
                    for sz in [-1.0, 1.0] {
                        let corner = world_child.apply_point(Vec3::new(
                            sx * half_extents.x,
                            sy * half_extents.y,
                            sz * half_extents.z,
                        ));
                        lowest = lowest.min(corner.y);
                    }
                }
            }
        }
        lowest
    }

    /// **複合剛体の静的つり合い**(群11、重心オフセット導入の裏付け)。
    ///
    /// L字形の重心は x=+0.0833 にずれるが、これは接地する横棒の支持多角形
    /// x∈[-0.25,0.75] の**内側**なので、直立姿勢は静的に安定でなければ
    /// ならない。解析的な静止高さ(横棒の底面 y=-0.5 が床に接する = 原点 y=0.5)
    /// へ初速ゼロで置き、そのまま直立して静止し続けることを確認する。
    ///
    /// これは`compound_body_can_be_created_and_settles_on_the_ground_without_panicking`
    /// が高所落下で横倒しになることの対照実験——「静的には安定、動的な
    /// バウンドでのみ転ぶ」ことを示し、転倒が実装の誤りでないことを固定する。
    #[test]
    fn compound_l_shape_placed_at_rest_stays_upright() {
        let mut world = World::new(WorldOptions::default());
        let steel = world.materials().find_by_name("鋼(炭素鋼)").unwrap();
        let concrete = world.materials().find_by_name("コンクリート").unwrap();

        let mut ground = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            concrete,
        );
        ground.body_type = BodyType::Static;
        world.create_body(ground);

        let l_shape = Shape::Compound {
            children: vec![
                (
                    Transform {
                        position: Vec3::new(0.0, 0.75, 0.0),
                        rotation: sim_math::Quat::IDENTITY,
                    },
                    Shape::Box {
                        half_extents: Vec3::new(0.25, 1.0, 0.25),
                    },
                ),
                (
                    Transform {
                        position: Vec3::new(0.25, -0.25, 0.0),
                        rotation: sim_math::Quat::IDENTITY,
                    },
                    Shape::Box {
                        half_extents: Vec3::new(0.5, 0.25, 0.25),
                    },
                ),
            ],
        };
        // 重心がローカル原点からずれていること(=このテストが意味を持つこと)。
        let com = l_shape.center_of_mass();
        assert!(
            (com.x - 0.0833333333333333).abs() < 1e-12 && (com.y - 0.4166666666666667).abs() < 1e-9,
            "L字の重心は解析値 (1/12, 5/12, 0) のはず: {com:?}"
        );

        let mut desc = RigidBodyDesc::dynamic(l_shape, steel);
        // 横棒の底面(本体原点から-0.5)がちょうど床に接する高さ。
        desc.transform.position = Vec3::new(0.0, 0.5, 0.0);
        let body = world.create_body(desc);

        for _ in 0..600 {
            world.step();
        }

        let idx = body.index as usize;
        let rotation = world.mechanics().bodies.rotation[idx];
        let tilt = 2.0 * rotation.w.abs().min(1.0).acos();
        assert!(
            tilt < 1e-2,
            "支持多角形の内側に重心があるので直立は安定のはず: tilt={tilt:.3e} rad"
        );
        let final_y = world.body_position(body).unwrap().y;
        assert!(
            (final_y - 0.5).abs() < 0.01,
            "解析的な静止高さ y=0.5 のままのはず(y={final_y})"
        );
    }

    /// 未知(存在しない index)の`BodyId`も`None`(パニックしない)。
    #[test]
    fn unknown_body_id_returns_none() {
        let world = World::new(WorldOptions::default());
        let bogus = BodyId {
            index: 999,
            generation: 0,
        };
        assert!(world.body_position(bogus).is_none());
    }

    /// 全ドメイン合成(モジュールdoc参照): mechanics(箱の自由落下)とthermal
    /// (2ノード熱平衡、`sim_thermal`のT2テストと同じ構成)を同一Worldで同時に有効化し、
    /// 1つの`step()`呼び出しで両方が(結合なしで)独立に正しく進行することを検証する。
    #[test]
    fn multiple_domains_step_independently_in_the_same_world() {
        let mut world = World::new(WorldOptions::default());
        let box_id = create_falling_box(&mut world);
        let y0 = world.body_position(box_id).unwrap().y;

        let (c1, c2) = (50.0, 200.0);
        let (t1_0, t2_0) = (400.0, 250.0);
        let mut thermal = sim_thermal::ThermalSolver::new(293.15);
        let idx1 = thermal.add_node(sim_thermal::ThermalNode::new(t1_0, c1));
        let idx2 = thermal.add_node(sim_thermal::ThermalNode::new(t2_0, c2));
        thermal.add_link(idx1, idx2, 5.0);
        world.enable_thermal(thermal);
        let expected_teq = (c1 * t1_0 + c2 * t2_0) / (c1 + c2);

        // 熱の時定数 tau = 1/(conductance*(1/C1+1/C2)) = 8s。Worldの既定dt(1/120、力学の
        // 安定刻みに合わせる、Orchestrator未実装のため両ドメインで共有)では、
        // sim-thermal単体のT2テストのような大きなdt(0.5s)は使えないため、その分ステップ数を
        // 増やして同じ物理時間(20*tau=160s)を確保する。
        let steps = (160.0 / WorldOptions::default().dt) as u32;
        for _ in 0..steps {
            world.step();
        }

        assert!(
            world.body_position(box_id).unwrap().y < y0,
            "mechanics domain should still evolve independently"
        );
        // World既定dt(1/120)はsim-thermal単体のT2テスト(dt=0.5)よりずっと小さいため、
        // 同じ物理時間を確保するのに必要なステップ数がはるかに多く、各ステップのPCG
        // 収束許容(tol_rel=1e-10)由来の累積誤差もその分大きくなる(実装検証中に1e-5では
        // 僅かに超過(~1e-4)することを確認したため、許容を1e-3に緩めた)。
        let t1 = world.thermal().unwrap().nodes[idx1].temperature;
        let t2 = world.thermal().unwrap().nodes[idx2].temperature;
        assert!(
            (t1 - expected_teq).abs() < 1e-3,
            "T1={t1} vs Teq={expected_teq}"
        );
        assert!(
            (t2 - expected_teq).abs() < 1e-3,
            "T2={t2} vs Teq={expected_teq}"
        );
    }

    /// 全ドメイン合成: 回路ドメイン(モジュールdoc参照)を有効化し、力学(箱の自由落下)と
    /// 同一Worldで独立に進行することを確認する(RC回路の過渡応答が理論値`V0(1-e^{-t/RC})`
    /// に一致することも合わせて検証、sim-em `e3_rc_transient` テストと同じ構成)。
    #[test]
    fn circuit_domain_steps_independently_in_the_same_world() {
        let mut world = World::new(WorldOptions::default());
        let box_id = create_falling_box(&mut world);
        let y0 = world.body_position(box_id).unwrap().y;

        let (v0, r, c) = (5.0, 1000.0, 1.0e-3);
        let mut circuit = sim_em::Circuit::new(3);
        circuit.add_voltage_source(1, sim_em::GROUND, v0);
        circuit.add_resistor(1, 2, r);
        circuit.add_capacitor(2, sim_em::GROUND, c, 0.0);
        world.enable_circuit(circuit);

        let tau = r * c;
        let steps = (5.0 * tau / WorldOptions::default().dt) as u32;
        for _ in 0..steps {
            world.step();
        }

        assert!(
            world.body_position(box_id).unwrap().y < y0,
            "mechanics domain should still evolve independently"
        );
        let t = steps as f64 * WorldOptions::default().dt;
        let expected_v = v0 * (1.0 - (-t / tau).exp());
        let measured_v = world.circuit().unwrap().node_voltage(2);
        assert!(
            (measured_v - expected_v).abs() / v0 < 1e-3,
            "measured_v={measured_v} expected_v={expected_v}"
        );
    }

    /// SPH流体ドメイン(`sim_fluid::SphFluid`の`Solver`実装、モジュールdoc「全ドメイン
    /// 合成」参照): 他ドメインと同様に`step()`が自動的にsub-stepし、孤立した1粒子が
    /// 重力で落下することを確認する。`sample_fluid`が最近傍粒子の速度・圧力を返す
    /// **増分J: `sample_fluid`が真のカーネル補間になったこと**。
    ///
    /// それまでは最近傍粒子の値をそのまま返す縮約で、**粒子間で値が階段状に
    /// 不連続に飛んでいた**(サンプル点が中点を跨いだ瞬間に別粒子の値へ切り替わる)。
    /// SPHの場の補間 $A(x)=\sum_j \frac{m_j}{\rho_j}A_j W(|x-x_j|,h)$ へ移した。
    ///
    /// 判定は**2粒子の中点での連続性**で行う。速度の違う2粒子を距離hより近く
    /// 置き、その間をサンプル点で走査する:
    /// - 最近傍方式なら中点で値が飛ぶ(片方の速度→もう片方の速度へ不連続)
    /// - カーネル補間なら中点は両者の中間値になり、走査は滑らかに変化する
    #[test]
    fn sample_fluid_interpolates_smoothly_between_particles() {
        let mut world = World::new(WorldOptions::default());
        let h = 0.4;
        let mut sph = sim_fluid::SphFluid::new(h, 1000.0, 20.0);
        sph.mass = 1.0;
        // 距離0.2(< h)に2粒子。速度は +x と -x で正反対。
        sph.add_particle(Vec3::new(-0.1, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        sph.add_particle(Vec3::new(0.1, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0));
        world.enable_sph(sph);
        world.step(); // 密度を確定させる(補間の分母に要る)

        // 中点は対称なので速度がちょうど打ち消し合ってほぼ0になる。
        let mid = world
            .sample_fluid(Vec3::ZERO)
            .expect("2粒子の中点は台の内側");
        assert!(
            mid.velocity.x.abs() < 1.0e-9,
            "対称な2粒子の中点では速度が打ち消し合うべき(最近傍方式ならどちらかの\
             ±1付近の値が出る): {}",
            mid.velocity.x
        );

        // 走査して不連続な飛びが無いこと(最近傍方式なら中点で2.0の跳躍が出る)。
        let n = 200;
        let mut previous: Option<f64> = None;
        let mut max_jump: f64 = 0.0;
        for k in 0..=n {
            let x = -0.1 + 0.2 * k as f64 / n as f64;
            let sample = world
                .sample_fluid(Vec3::new(x, 0.0, 0.0))
                .expect("粒子間は台の内側");
            if let Some(p) = previous {
                max_jump = max_jump.max((sample.velocity.x - p).abs());
            }
            previous = Some(sample.velocity.x);
        }
        assert!(
            max_jump < 0.05,
            "カーネル補間なら隣接サンプル間の変化は滑らかなはず(最近傍方式では\
             中点で約2.0の跳躍が出る): max_jump={max_jump}"
        );
    }

    /// **増分J: 流体の外をサンプルすると`None`を返す**。カーネルの台(r<=h)に
    /// 粒子が1つも無い点には「そこに流体は無い」と答える——最近傍へ
    /// フォールバックして遠く離れた点にもっともらしい値を返すより正直である。
    #[test]
    fn sample_fluid_returns_none_outside_the_kernel_support() {
        let mut world = World::new(WorldOptions::default());
        let mut sph = sim_fluid::SphFluid::new(0.1, 1000.0, 20.0);
        sph.mass = 1.0;
        sph.add_particle(Vec3::ZERO, Vec3::ZERO);
        world.enable_sph(sph);
        world.step();

        assert!(
            world.sample_fluid(Vec3::ZERO).is_some(),
            "粒子の位置は台の内側"
        );
        assert!(
            world.sample_fluid(Vec3::new(10.0, 0.0, 0.0)).is_none(),
            "台の外は None を返すべき"
        );
    }

    /// **群1: 近似バッジがソルバの自己申告であり、設定に追従すること**。
    ///
    /// 以前は`World`が「どのドメインが有効か」から固定文字列を組み立てていたため、
    /// **同じドメインなら設定が違っても同じバッジ**しか出せなかった。
    /// `Solver::approximations()`へ移したので、格子流体の粘性を0にすると
    /// 「粘性拡散をスキップ」が増える——**設定依存の近似を表現できる**ことを固定する。
    #[test]
    fn approximations_are_self_reported_and_follow_solver_settings() {
        let build = |viscosity: f64| -> Vec<sim_core::Approximation> {
            let mut world = World::new(WorldOptions::default());
            let mut grid = sim_fluid::GridFluid2D::new(8, 8, 0.1);
            grid.kinematic_viscosity = viscosity;
            world.enable_grid_fluid(grid);
            world.active_approximations()
        };

        let viscous = build(1.0e-4);
        let inviscid = build(0.0);
        let names = |v: &[sim_core::Approximation]| -> Vec<&'static str> {
            v.iter().map(|a| a.name).collect()
        };
        assert!(
            names(&viscous).contains(&"2D・周期境界"),
            "格子流体の常時の近似は申告されるべき: {:?}",
            names(&viscous)
        );
        assert!(
            !names(&viscous).contains(&"粘性拡散をスキップ"),
            "粘性があるならスキップの申告は出ないはず: {:?}",
            names(&viscous)
        );
        assert!(
            names(&inviscid).contains(&"粘性拡散をスキップ"),
            "粘性0なら設定依存の近似が増えるべき(自己申告にした目的そのもの): {:?}",
            names(&inviscid)
        );

        // 設計§1.3 が要求する「出典」と「オフ可否」が全件に入っていること。
        for a in &inviscid {
            assert!(a.doc.starts_with("docs/"), "出典を持つべき: {a:?}");
            assert!(!a.reason.is_empty(), "理由を持つべき: {a:?}");
        }
        // 現状オフにできる機構が無いものは`can_disable=false`で申告される
        // (UIが「オフにできます」という嘘のトグルを出さないため)。
        // semi-Lagrangian移流は移流スキーム自体なので外しようがない。
        assert!(inviscid
            .iter()
            .any(|a| a.name == "semi-Lagrangian移流" && !a.can_disable));
        // 逆に、**群7で`GridBoundary::Channel`、群9で固体境界が入って実際に切り替えられる
        // ようになった**境界の申告は`can_disable=true`になる(嘘のトグルの逆——
        // 切り替えられるのに「できない」と申告し続けるのも同じく嘘である)。
        assert!(inviscid
            .iter()
            .any(|a| a.name == "2D・周期境界" && a.can_disable));
    }

    /// **群1: 結合の内省が「捏造でない」こと**——`referenced_bodies()`が申告する
    /// 剛体が、**実際にその結合から力を受ける剛体と一致する**ことを確認する。
    ///
    /// これが内省層の要である。種別名や説明文は実装者が好きに書けてしまうが、
    /// 参照ボディは**実測と突き合わせられる**: 結合を適用する前後で速度が変わった
    /// 剛体を集め、申告値と一致するかを見る。申告だけ増やして実際には触っていない
    /// (あるいは逆に、申告に無い剛体を勝手に動かす)結合をこれで弾ける。
    #[test]
    fn coupling_referenced_bodies_match_the_bodies_actually_affected() {
        let mut world = World::new(WorldOptions {
            gravity: 0.0,
            ..WorldOptions::default()
        });
        let steel = world.materials().find_by_name("鋼(炭素鋼)").unwrap();
        // 3体用意し、真ん中(index 1)だけに結合を張る。**互いに接触しない位置へ離す**
        // ——群5で外力注入型の結合を pre 相(力学ステップの前)へ移した結果、原点に
        // 3体重ねたままだと結合で動かした剛体の速度が同stepの接触解決で隣へ伝わり、
        // 「実測」に間接的に動いた剛体まで混ざってしまう(結合の申告漏れではなく
        // テストの測り方の問題。移行前は注入が力学ステップの後だったため隠れていた)。
        for i in 0..3 {
            let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.1 }, steel);
            desc.transform.position = Vec3::new(i as f64 * 5.0, 0.0, 0.0);
            world.create_body(desc);
        }
        // 一様電場を与えれば、結合を張ったボディだけが力を受ける。
        let electrostatics = sim_em::PointChargeSystem::new(sim_em::UniformField {
            e: Vec3::new(0.0, 1.0e6, 0.0),
            b: Vec3::ZERO,
        });
        world.enable_em_electrostatics(electrostatics);
        world.add_coupling(Box::new(sim_coupling::LorentzForce {
            body_index: 1,
            charge: 1.0e-6,
        }));

        let before: Vec<Vec3> = (0..3)
            .map(|i| world.mechanics().bodies.linear_velocity[i])
            .collect();
        world.step();
        let moved: Vec<usize> = (0..3)
            .filter(|&i| {
                let v = world.mechanics().bodies.linear_velocity[i];
                (v - before[i]).length() > 1.0e-15
            })
            .collect();

        let infos = world.couplings();
        assert_eq!(infos.len(), 1);
        assert_eq!(infos[0].kind, sim_coupling::CouplingKind::LorentzForce);
        assert_eq!(
            infos[0].bodies, moved,
            "referenced_bodies()は実際に力を受けた剛体と一致すべき: \
             申告={:?} 実測={:?}",
            infos[0].bodies, moved
        );
        // 説明文にパラメータが入っていること(件数だけの縮約からの脱却)。
        assert!(
            infos[0].description.contains("body#1"),
            "describe()はパラメータを含むべき: {}",
            infos[0].description
        );
        // 選択ボディでの絞り込みが効くこと。
        let ids: Vec<BodyId> = (0..3)
            .map(|i| BodyId {
                index: i,
                generation: 0,
            })
            .collect();
        assert_eq!(world.couplings_for_body(ids[1]).len(), 1);
        assert!(world.couplings_for_body(ids[0]).is_empty());
    }

    /// **群1: 種別が一意で、跨るドメインが空でないこと**。`CouplingKind`を
    /// enum にした目的(UIが種別で分岐・フィルタできる)が成立する前提を固定する。
    #[test]
    fn every_coupling_kind_is_distinct_and_declares_its_domains() {
        use sim_coupling::{Coupling, CouplingKind};
        let couplings: Vec<Box<dyn Coupling>> = vec![
            Box::new(sim_coupling::DissipationToHeat::to_single_node(0)),
            Box::new(sim_coupling::JouleHeat::to_single_node(0)),
            Box::new(sim_coupling::LorentzForce {
                body_index: 0,
                charge: 1.0,
            }),
            Box::new(sim_coupling::MotorCoupling {
                body_index: 0,
                axis: Vec3::new(0.0, 1.0, 0.0),
                voltage_source_index: 0,
                torque_constant: 0.05,
            }),
            Box::new(sim_coupling::BoussinesqBuoyancy {
                thermal_node: 0,
                ambient_temperature: 293.15,
                thermal_expansion_coefficient: 3.4e-3,
            }),
        ];
        let mut seen = std::collections::HashSet::new();
        for c in &couplings {
            let kind = c.kind();
            assert!(seen.insert(kind), "種別は一意であるべき: {kind:?}");
            assert!(
                !c.domain_ids().is_empty(),
                "跨るドメインを宣言すべき: {kind:?}"
            );
            assert!(
                c.describe().starts_with(kind.name()),
                "describe()は種別名で始めるべき: {}",
                c.describe()
            );
            assert!(!kind.summary().is_empty());
        }
        assert_ne!(CouplingKind::JouleHeat, CouplingKind::DissipationToHeat);
    }

    /// **群1: ジョイントの列挙**。4種のジョイントが種別タグ付きで1本の列挙に
    /// 並び、選択ボディで絞れること。Inspectorの Joint コンポーネントが
    /// アンカー2点だけの縮約表示だったのを、種別・接続先・軸まで出せるようにした。
    #[test]
    fn joints_are_enumerated_with_kind_and_filtered_by_body() {
        let mut world = World::new(WorldOptions::default());
        let steel = world.materials().find_by_name("鋼(炭素鋼)").unwrap();
        let a = world.create_body(RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.1 }, steel));
        let b = world.create_body(RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.1 }, steel));

        world.add_distance_joint_to_world_point(a, Vec3::ZERO, Vec3::new(0.0, 2.0, 0.0), 1.0);
        world
            .mechanics_mut()
            .add_ball_joint(sim_mechanics::BallJoint {
                body_a: a.index as usize,
                anchor_a: Vec3::ZERO,
                body_b: Some(b.index as usize),
                anchor_b: Vec3::ZERO,
                disabled: false,
            });

        let joints = world.joints();
        assert_eq!(joints.len(), 2);
        let kinds: Vec<JointKind> = joints.iter().map(|j| j.kind).collect();
        assert!(kinds.contains(&JointKind::Distance) && kinds.contains(&JointKind::Ball));
        // DistanceJoint はワールド固定点なので body_b が None。
        let distance = joints
            .iter()
            .find(|j| j.kind == JointKind::Distance)
            .unwrap();
        assert_eq!(distance.body_b, None);
        assert_eq!(distance.length, Some(1.0));

        // bはBallJointだけに繋がる。
        assert_eq!(world.joints_for_body(b).len(), 1);
        assert_eq!(world.joints_for_body(a).len(), 2);
    }

    /// **増分J: 結合の pre/post 2相分離が実際に働くこと**。
    ///
    /// 設計 docs/20-integration/01-coupling-matrix.md §1.3 が求める2相のうち、
    /// pre 相は**ドメインソルバを進める前**に呼ばれなければ意味が無い
    /// (注入した力が今stepの積分に効かないなら1step遅れのままで、分離した意味が
    /// 消える)。そこで pre 相で速度を書き換える結合を仕込み、**同じstepの
    /// 積分がその値を使って位置を進めた**ことを位置の変化から確認する。
    #[test]
    fn coupling_pre_phase_runs_before_the_domain_solvers() {
        /// pre 相で剛体0の速度を +x 方向 `speed` に固定する検査用の結合。
        #[derive(Clone)]
        struct PinVelocity {
            speed: f64,
        }
        impl sim_coupling::Coupling for PinVelocity {
            fn kind(&self) -> sim_coupling::CouplingKind {
                sim_coupling::CouplingKind::Noop
            }
            fn domain_ids(&self) -> &'static [sim_core::DomainId] {
                &[sim_core::DomainId::Mechanics]
            }
            fn describe(&self) -> String {
                "PinVelocity(検査用)".to_string()
            }
            fn apply(&mut self, _world: &mut sim_coupling::DomainStates, _dt: f64) {}
            fn apply_pre(&mut self, world: &mut sim_coupling::DomainStates, _dt: f64) {
                world.mechanics.bodies.linear_velocity[0] = Vec3::new(self.speed, 0.0, 0.0);
            }
        }

        let mut world = World::new(WorldOptions {
            gravity: 0.0,
            ..WorldOptions::default()
        });
        let steel = world.materials().find_by_name("鋼(炭素鋼)").unwrap();
        let desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.1 }, steel);
        world.create_body(desc);

        let speed = 3.0;
        world.add_coupling(Box::new(PinVelocity { speed }));
        let dt = WorldOptions::default().dt;
        world.step();

        // pre 相で書いた速度が**同じstep**の積分に使われていれば、位置は speed*dt
        // だけ進む。post 相(従来の`apply`)だけなら、このstepの積分は初速0で
        // 行われるので位置は動かない。
        let x = world.mechanics().bodies.position[0].x;
        assert!(
            (x - speed * dt).abs() < 1.0e-9,
            "pre 相はソルバより前に走り、その速度が今stepの積分に効くべき: \
             x={x} expected={}",
            speed * dt
        );
    }

    /// ことも合わせて確認する。
    #[test]
    fn sph_domain_steps_automatically_and_sample_fluid_reads_nearest_particle() {
        let mut world = World::new(WorldOptions::default());
        let mut sph = sim_fluid::SphFluid::new(0.04, 1000.0, 20.0);
        sph.mass = 1.0;
        let particle_start = Vec3::new(0.0, 10.0, 0.0);
        sph.add_particle(particle_start, Vec3::ZERO);
        world.enable_sph(sph);

        let dt = WorldOptions::default().dt;
        for _ in 0..60 {
            world.step();
        }

        let particle_now = world.sph().unwrap().position[0];
        assert!(
            particle_now.y < particle_start.y,
            "isolated SPH particle should fall under gravity: particle_now={particle_now:?}"
        );
        let expected_vy = -9.80665 * 60.0 * dt;
        let measured_vy = world.sph().unwrap().velocity[0].y;
        let rel_err = (measured_vy - expected_vy).abs() / expected_vy.abs();
        assert!(
            rel_err < 0.01,
            "measured_vy={measured_vy} expected_vy={expected_vy} rel_err={rel_err:.4}"
        );

        let sample = world.sample_fluid(particle_now).unwrap();
        assert!(
            (sample.velocity.y - measured_vy).abs() < 1e-9,
            "sample_fluid should read the nearest particle's velocity"
        );
    }

    /// 格子流体ドメインが`step()`で自動的にsub-stepされ、粘性拡散により運動エネルギーが
    /// 単調に減衰すること(`total_energy`にも反映されること)。
    #[test]
    fn grid_fluid_domain_steps_automatically_and_dissipates_kinetic_energy() {
        let mut world = World::new(WorldOptions::default());
        let nx = 8;
        let ny = 8;
        let h = 1.0 / nx as f64;
        let mut fluid = sim_fluid::GridFluid2D::new(nx, ny, h);
        fluid.kinematic_viscosity = 0.05;
        let k = 2.0 * std::f64::consts::PI;
        for j in 0..ny as i64 {
            for i in 0..=nx as i64 {
                let idx =
                    (i.rem_euclid(nx as i64)) as usize + nx * (j.rem_euclid(ny as i64)) as usize;
                let x = i as f64 * h;
                let y = (j as f64 + 0.5) * h;
                fluid.u[idx] = -(k * x).cos() * (k * y).sin();
            }
        }
        for j in 0..=ny as i64 {
            for i in 0..nx as i64 {
                let idx =
                    (i.rem_euclid(nx as i64)) as usize + nx * (j.rem_euclid(ny as i64)) as usize;
                let x = (i as f64 + 0.5) * h;
                let y = j as f64 * h;
                fluid.v[idx] = (k * x).sin() * (k * y).cos();
            }
        }
        world.enable_grid_fluid(fluid);

        let energy_before = world.total_energy().total();
        for _ in 0..30 {
            world.step();
        }
        let energy_after = world.total_energy().total();

        assert!(
            energy_after < energy_before,
            "viscous grid fluid should lose kinetic energy: before={energy_before} after={energy_after}"
        );
        assert!(world.grid_fluid().is_some());
    }

    /// `BoussinesqBuoyancy`をレジストリ経由(`add_coupling`)で`grid_fluid`ドメインに
    /// 接続し、暖かい熱源によって格子流体の平均鉛直速度が単調に上昇することを確認する
    /// (`sim_coupling::BoussinesqBuoyancy`単体テストの解析的挙動が`World`経由でも
    /// 機能することの確認)。
    #[test]
    fn boussinesq_buoyancy_coupling_raises_grid_fluid_mean_vertical_velocity_via_world() {
        let mut world = World::new(WorldOptions::default());
        let ambient = 293.15;
        let mut thermal = sim_thermal::ThermalSolver::new(ambient);
        let node = thermal.add_node(sim_thermal::ThermalNode::new(313.15, 1000.0));
        world.enable_thermal(thermal);
        world.enable_grid_fluid(sim_fluid::GridFluid2D::new(8, 8, 0.1));
        world.add_coupling(Box::new(sim_coupling::BoussinesqBuoyancy {
            thermal_node: node,
            ambient_temperature: ambient,
            thermal_expansion_coefficient: 3.4e-3,
        }));

        let mean_v = |w: &World| -> f64 {
            let fluid = w.grid_fluid().unwrap();
            fluid.v.iter().sum::<f64>() / fluid.v.len() as f64
        };
        let v_before = mean_v(&world);
        for _ in 0..10 {
            world.step();
        }
        let v_after = mean_v(&world);

        assert!(
            v_after > v_before,
            "warm thermal node should raise grid_fluid's mean vertical velocity via \
             BoussinesqBuoyancy: v_before={v_before} v_after={v_after}"
        );
    }

    /// `ConvectionLink`をレジストリ経由で`grid_fluid`+`thermal`に接続し、流速のある
    /// 流体ノードから受熱面ノードへ熱が移動する(受熱面の温度が単調に上昇する)ことを
    /// `World`経由で確認する。
    #[test]
    fn convection_link_coupling_warms_surface_node_from_flowing_fluid_via_world() {
        let mut world = World::new(WorldOptions::default());
        let mut thermal = sim_thermal::ThermalSolver::new(293.15);
        let fluid_node = thermal.add_node(sim_thermal::ThermalNode::new(350.0, 1000.0));
        let surface_node = thermal.add_node(sim_thermal::ThermalNode::new(293.15, 1000.0));
        world.enable_thermal(thermal);

        let mut fluid = sim_fluid::GridFluid2D::new(8, 8, 0.1);
        for u in fluid.u.iter_mut() {
            *u = 2.0;
        }
        world.enable_grid_fluid(fluid);

        world.add_coupling(Box::new(sim_coupling::ConvectionLink {
            fluid_node,
            surface_node,
            area: 0.05,
            characteristic_length: 0.2,
            fluid_thermal_conductivity: 0.026,
            kinematic_viscosity: 1.5e-5,
            prandtl_number: 0.71,
            mode: sim_coupling::ConvectionMode::ForcedFlatPlate,
            thermal_expansion_coefficient: None,
        }));

        let surface_temp_before = world.thermal().unwrap().nodes[surface_node].temperature;
        for _ in 0..10 {
            world.step();
        }
        let surface_temp_after = world.thermal().unwrap().nodes[surface_node].temperature;

        assert!(
            surface_temp_after > surface_temp_before,
            "surface node should warm up from the flowing hot fluid via ConvectionLink: \
             before={surface_temp_before} after={surface_temp_after}"
        );
    }

    /// `GridFluidRigid`をレジストリ経由で`mechanics`+`grid_fluid`に接続し、一様な流れ
    /// (u=1.0)の中に置いた軽い剛体が、流れと同じ+x方向に押し流されることを`World`経由で
    /// 確認する(圧力積分によるマスキング手法自体の定量的な物理的妥当性は
    /// `sim_fluid::GridFluidRigidBox2D`(X2)の既存テストが既に検証済みなので、ここでは
    /// `World`のCouplingレジストリ経由での配線 — mechanicsボディの位置・速度が
    /// `grid_fluid`のセル種別マスクに反映され、圧力反力がボディに戻ってくること — を
    /// 定性的に確認する、`SphRigid`実装検証時に確立した「動的な定量検証はSPH/格子流体
    /// 特有の縁効果に弱い」という教訓を踏まえた判断)。
    #[test]
    fn grid_fluid_rigid_coupling_pushes_a_light_body_downstream_via_world() {
        let mut world = World::new(WorldOptions {
            gravity: 0.0,
            ..WorldOptions::default()
        });

        let mut fluid = sim_fluid::GridFluid2D::new(16, 16, 0.1);
        for u in fluid.u.iter_mut() {
            *u = 1.0;
        }
        world.enable_grid_fluid(fluid);

        let steel = world
            .materials()
            .find_by_name("鋼(炭素鋼)")
            .expect("standard DB has steel");
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.05 }, steel);
        desc.mass_override = Some(0.02);
        desc.transform.position = Vec3::new(0.8, 0.8, 0.0);
        let body = world.create_body(desc);

        world.add_coupling(Box::new(sim_coupling::GridFluidRigid::new(
            body.index as usize,
            0.15,
            0.15,
        )));

        let x_before = world.body_position(body).unwrap().x;

        // まず1step進めて、固体マスクが剛体位置へ追従していることを確認する。固体表現は
        // `cell_type`へ一本化されたので、移行前のように`solid_box()`で矩形を読み戻すの
        // ではなく、**Solidセルのx範囲の中心**が剛体のx位置と一致することを見る。
        //
        // 20step後ではなくここで見るのは、この軽い剛体(質量0.02kg)が一様流に押されて
        // **格子の外**(x≈76、格子は 0..1.6)まで飛んでいくため——外に出た後は
        // ラスタライズされるセルが1つも無く、マスク追従の主張が空になる。
        // (移行前の`solid_box()`は「セルを1つも塗らなくても矩形は保持される」ので、
        // 20step後でも読み戻せてしまい、この検査は事実上空振りしていた。)
        world.step();
        {
            let grid = world.grid_fluid().unwrap();
            let body_pos = world.body_position(body).unwrap();
            let mut solid_x: Vec<f64> = Vec::new();
            for j in 0..grid.ny {
                for i in 0..grid.nx {
                    if grid.cell_type()[i + grid.nx * j] == sim_fluid::CellType::Solid {
                        solid_x.push((i as f64 + 0.5) * grid.h);
                    }
                }
            }
            assert!(
                !solid_x.is_empty(),
                "GridFluidRigid should have rasterized the body into Solid cells: \
                 body_pos={body_pos:?}"
            );
            let min_x = solid_x.iter().cloned().fold(f64::INFINITY, f64::min);
            let max_x = solid_x.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            let center_x = 0.5 * (min_x + max_x);
            assert!(
                (center_x - body_pos.x).abs() < grid.h,
                "solid cells should be centered on the body's x position: \
                 center_x={center_x} body_pos.x={} h={}",
                body_pos.x,
                grid.h
            );
        }

        for _ in 0..19 {
            world.step();
        }
        let x_after = world.body_position(body).unwrap().x;
        let vx_after = world.body_velocity(body).unwrap().x;

        assert!(
            x_after > x_before,
            "body should be pushed in the +x direction by the uniform +x flow: \
             x_before={x_before} x_after={x_after}"
        );
        assert!(
            vx_after.is_finite() && vx_after > 0.0,
            "body's x-velocity should be finite and positive (downstream): vx_after={vx_after}"
        );
    }

    /// `BuoyancyDrag`をレジストリ経由(`add_coupling`)で剛体に接続し、`demos.rs`のD6
    /// (F4部分、密度比0.6の直立直方体)と同じ釣り合い喫水深さの近傍で有界に留まる
    /// ことを確認する(既存の`MechanicsSolver.fluids`埋め込み経路(D6が使う)と同じ
    /// 物理式(`sim_fluid::{submerged_box_below_plane, buoyancy_force}`)を使うが、
    /// `mechanics_mut().fluids`は設定しない独立経路)。
    ///
    /// D6のF4部分は埋め込み経路(`apply_forces`内でmechanicsの各sub-stepごとに
    /// 浮力を再評価)のため実質的に減衰項なしでも密着した釣り合いに留まるが、この
    /// Coupling経由の経路はフレーム粒度の縮約(Couplingレジストリはフレームごとに
    /// 1回だけ適用され、mechanicsの内部sub-stepごとには再評価されない。群5で
    /// `BuoyancyDrag`を pre 相へ移したので**stepをまたぐ遅れは無い**が、sub-step
    /// 粒度での再評価が無い点は変わらない)であり、かつ抗力(減衰)を伴わない純粋な浮力の
    /// 復元力のみのため、実装検証中に測定したところ振幅約0.02(side比2%)の非減衰
    /// 振動として恒久的に持続することを発見した(F4のように単調に釣り合いへ収束
    /// するのではなく、有界な振動に留まることを確認する検証に切り替えた、X2の
    /// 「有界であること」の確認と同種の判断)。許容誤差はこの実測振幅に十分な
    /// マージンを持たせてrel<0.03(side比)とした。
    #[test]
    fn buoyancy_drag_coupling_stays_bounded_near_embedded_path_equilibrium_waterline_via_world() {
        let water_density = 998.2;
        let half = 0.5;
        let side = 2.0 * half;
        let ratio = 0.6;

        let mut world = World::new(WorldOptions::default());
        let body_material = world.materials_mut().push(sim_core::Material {
            name: "test-buoyancy-drag-coupling-floating-body",
            density: ratio * water_density,
            friction: 0.0,
            restitution: 0.0,
            youngs_modulus: None,
            specific_heat: 1000.0,
            conductivity: 1.0,
            emissivity: 0.5,
            melting: None,
            resistivity: None,
            relative_permittivity: 1.0,
            refractive_index: None,
            source: "test fixture",
            uncertainty: 0.0,
        });
        let h_sub = ratio * side;
        let equilibrium_y = -h_sub + half;
        let mut desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(half, half, half),
            },
            body_material,
        );
        desc.transform.position = Vec3::new(0.0, equilibrium_y, 0.0);
        let box_id = world.create_body(desc);

        world.add_coupling(Box::new(sim_coupling::BuoyancyDrag {
            body_index: box_id.index as usize,
            water: Some(sim_fluid::FluidRegion::new(0.0, water_density)),
            atmosphere: None,
            lift: None,
        }));

        let mut max_drift: f64 = 0.0;
        for _ in 0..2000 {
            world.step();
            let y = world.body_position(box_id).unwrap().y;
            assert!(y.is_finite(), "solver diverged: y={y}");
            max_drift = max_drift.max((y - equilibrium_y).abs());
        }
        assert!(
            max_drift / side < 0.03,
            "max_drift={max_drift} equilibrium_y={equilibrium_y} side={side}"
        );
    }

    /// `add_coupling`で登録した`Coupling`が`step()`ごとに自動適用され(呼び出し側が
    /// `apply_coupling`を手動で呼ばなくても)、`snapshot`/`restore`(`#[derive(Clone)]`
    /// 経由)を跨いでもレジストリごと正しく複製・継続することを確認する
    /// (`sim-coupling::Coupling`にdyn-safeな`CouplingClone`を追加した増分の検証)。
    #[test]
    fn add_coupling_is_applied_automatically_every_step_and_survives_snapshot_restore() {
        let build = |world: &mut World| {
            let steel = world
                .materials()
                .find_by_name("鋼(炭素鋼)")
                .expect("standard DB has steel");
            let mut floor_desc = RigidBodyDesc::dynamic(
                Shape::Plane {
                    normal: Vec3::new(0.0, 1.0, 0.0),
                    d: 0.0,
                },
                steel,
            );
            floor_desc.body_type = BodyType::Static;
            world.create_body(floor_desc);

            let mut box_desc = RigidBodyDesc::dynamic(
                Shape::Box {
                    half_extents: Vec3::new(0.5, 0.5, 0.5),
                },
                steel,
            );
            box_desc.transform = Transform {
                position: Vec3::new(0.0, 0.5, 0.0),
                rotation: Quat::IDENTITY,
            };
            box_desc.linear_velocity = Vec3::new(3.0, 0.0, 0.0);
            world.create_body(box_desc);

            let mut thermal = sim_thermal::ThermalSolver::new(293.15);
            let node = thermal.add_node(sim_thermal::ThermalNode::new(293.15, 1000.0));
            world.enable_thermal(thermal);
            world.add_coupling(Box::new(sim_coupling::DissipationToHeat::to_single_node(
                node,
            )));
        };

        let mut world = World::new(WorldOptions::default());
        build(&mut world);
        for _ in 0..60 {
            world.step();
        }
        let initial_temp = world.thermal().unwrap().nodes[0].temperature;
        assert!(
            initial_temp > 293.15,
            "registered DissipationToHeat coupling should have raised the thermal node's \
             temperature without any manual apply_coupling call: initial_temp={initial_temp}"
        );

        let straight_run_hash = {
            let mut w = world.clone();
            for _ in 0..60 {
                w.step();
            }
            w.state_hash()
        };

        let snapshot = world.snapshot();
        let mut restored = World::new(WorldOptions::default());
        restored.restore(&snapshot);
        for _ in 0..60 {
            restored.step();
        }
        assert_eq!(
            straight_run_hash,
            restored.state_hash(),
            "restored world's coupling registry should keep applying identically to the \
             original after restore"
        );
    }

    /// 決定論テスト(階層1): 同一初期条件 → 同数ステップ後のハッシュが一致する。
    /// 設計: docs/20-integration/02-determinism-replay.md §5/§6。
    #[test]
    fn determinism_same_scenario_twice_matches_hash() {
        let run = || {
            let mut world = World::new(WorldOptions::default());
            create_falling_box(&mut world);
            for _ in 0..300 {
                world.step();
            }
            world.state_hash()
        };
        let hash_a = run();
        let hash_b = run();
        assert_eq!(hash_a, hash_b);
    }

    /// 決定論テスト(階層1): スナップショット再開時のリプレイ一致
    /// (設計docs/20-integration/02-determinism-replay.md §6)。同一シナリオを
    /// 300step通しで実行した場合と、150step時点でスナップショットを取り、
    /// (スナップショットが単なる巻き戻し先ではなく実際に状態を保持していることを
    /// 検証するため)さらに50step進めて状態を変えた上でスナップショットへ復元し、
    /// 残り150stepを続けた場合とで、最終`state_hash()`が一致することを確認する。
    #[test]
    fn determinism_snapshot_restore_replay_matches_uninterrupted_run() {
        let straight_run_hash = {
            let mut world = World::new(WorldOptions::default());
            create_falling_box(&mut world);
            for _ in 0..300 {
                world.step();
            }
            world.state_hash()
        };

        let mut world = World::new(WorldOptions::default());
        create_falling_box(&mut world);
        for _ in 0..150 {
            world.step();
        }
        let snapshot = world.snapshot();
        let hash_at_snapshot = world.state_hash();

        // スナップショット取得後も別途進め、復元前の状態をスナップショットと異なる
        // ものにする(復元が実際に巻き戻すことを検証する対照)。
        for _ in 0..50 {
            world.step();
        }
        assert_ne!(
            hash_at_snapshot,
            world.state_hash(),
            "world should have diverged from the snapshot after 50 more steps"
        );

        world.restore(&snapshot);
        assert_eq!(
            hash_at_snapshot,
            world.state_hash(),
            "restore should bring the hash back to exactly the snapshot point"
        );

        for _ in 0..150 {
            world.step();
        }
        assert_eq!(straight_run_hash, world.state_hash());
    }

    /// エネルギー台帳: 接触なし自由落下では semi-implicit Euler が定数外力(一様重力)に対して
    /// 1 step あたり厳密に `-0.5 m g^2 dt^2` の力学的エネルギー損失を持つ(周期軌道でないため
    /// symplectic 特有の有界誤差ではなく、線形ドリフトになる — 既知の積分器由来のドリフトで
    /// あり不明な漏れではない)。E(0)=m g h0 が ENERGY_SCALE_FLOOR を大きく上回るので
    /// residual の scale は E(0) に決まり、質量 m が式から消える:
    /// residual(N) = N * 0.5 * g * dt^2 / h0。台帳の記帳がこの解析予測と一致することを検証する。
    /// (台帳は最初の `step()` で遅延初期化するため、`create_body` はここでは計上されない。)
    #[test]
    fn energy_ledger_residual_matches_analytic_symplectic_drift() {
        let options = WorldOptions::default();
        let (g, dt) = (options.gravity, options.dt);
        let n = 100u32;

        let mut world = World::new(options);
        create_falling_box(&mut world);
        for _ in 0..n {
            world.step();
        }

        let expected = n as f64 * 0.5 * g * dt * dt / INITIAL_HEIGHT;
        let measured = world.energy_residual();
        assert!(
            (measured - expected).abs() / expected < 1e-6,
            "measured={measured} expected={expected}"
        );
        assert_eq!(world.energy_residual_history().len(), n as usize);
        // 外力なし・接触なしの単調な力学的エネルギー減少なので残差は単調非減少のはず。
        assert!(world.energy_residual_history()[0] <= measured);
    }

    /// `Command::ApplyForce`(重心、`point: None`): 設計§1「実行中の変更はコマンド経由」
    /// (docs/20-integration/04-world-api.md §2)。重力なしのWorldで1step分の力を加え、
    /// semi-implicit Eulerの速度更新 `Δv=(F/m)dt` に一致することを確認する。
    #[test]
    fn apply_force_command_at_center_of_mass_matches_semi_implicit_euler_velocity_update() {
        let options = WorldOptions {
            gravity: 0.0,
            ..WorldOptions::default()
        };
        let dt = options.dt;
        let mut world = World::new(options);
        let box_id = create_falling_box(&mut world);
        let mass = world.mechanics_mut().bodies.mass(box_id.index as usize);

        let force = Vec3::new(10.0, 0.0, 0.0);
        world.push_command(Command::ApplyForce {
            body: box_id,
            force,
            point: None,
        });
        world.step();

        let expected_v = force.scale(dt / mass);
        let measured_v = world.mechanics_mut().bodies.linear_velocity[box_id.index as usize];
        assert!(
            (measured_v - expected_v).length() < 1e-9,
            "measured_v={measured_v:?} expected_v={expected_v:?}"
        );
        // 重心への力なのでトルクは生じない。
        assert_eq!(
            world.mechanics_mut().bodies.angular_velocity[box_id.index as usize],
            Vec3::ZERO
        );
        assert_eq!(world.command_log().len(), 1);
        assert_eq!(
            world.command_log()[0].0,
            0,
            "applied during the first step (step_count=0 at apply time)"
        );

        // 力は1stepのみ有効(force_accumはstep末尾でクリアされる)— もう1step進めても
        // 力なしの慣性運動(等速直線運動)になるはず。
        let v_after_first_step = measured_v;
        world.step();
        let v_after_second_step =
            world.mechanics_mut().bodies.linear_velocity[box_id.index as usize];
        assert!(
            (v_after_second_step - v_after_first_step).length() < 1e-9,
            "force must not persist beyond the step it was applied in"
        );
    }

    /// `Command::ApplyForce`(重心以外の`point`): トルクが生じ角速度がゼロでなくなることを
    /// 確認する(設計§2 `ApplyForce{body, force, point}`)。
    #[test]
    fn apply_force_command_off_center_produces_angular_velocity() {
        let options = WorldOptions {
            gravity: 0.0,
            ..WorldOptions::default()
        };
        let mut world = World::new(options);
        let box_id = create_falling_box(&mut world);
        let position = world.body_position(box_id).unwrap();

        world.push_command(Command::ApplyForce {
            body: box_id,
            force: Vec3::new(0.0, 0.0, 10.0),
            point: Some(position + Vec3::new(0.5, 0.0, 0.0)),
        });
        world.step();

        let omega = world.mechanics_mut().bodies.angular_velocity[box_id.index as usize];
        assert!(
            omega.length() > 0.0,
            "off-center force should induce rotation: omega={omega:?}"
        );
    }

    /// `Command::SetMotorTarget`(設計§2「`SetMotorTarget{joint, velocity}`」、モジュールdoc
    /// 「実装済みの`theta_target`を公開する」参照)。ヒンジモーターの目標角度を実行中に
    /// 変更すると、PD制御(`HingeMotorPd::apply`)により剛体の角度が新しい目標へ収束する
    /// ことを確認する(`sim-mechanics`のPD位置サーボ自体は別途単体テスト済み、ここでは
    /// `World`経由のCommandが正しく`hinge_motors[i].theta_target`まで届くことを検証)。
    #[test]
    fn set_motor_target_command_changes_hinge_motor_target_angle_at_runtime() {
        let mut world = World::new(WorldOptions {
            gravity: 0.0,
            ..WorldOptions::default()
        });
        let steel = world.materials().find_by_name("鋼(炭素鋼)").unwrap();
        // `entity_layer_hinge_motor_maintains_crouch_pose_for_60s_with_ground_contact`
        // (crates/sim-mechanics/src/solver.rs)と同じ形状・質量(kp/kd/torque_maxの既定値は
        // この慣性モーメントで検証済み — 小さい球のような軽い慣性だとPD制御が過大な角速度を
        // 要求し発振するため合わせる)。
        let mut desc = sim_mechanics::RigidBodyDesc::dynamic(
            sim_mechanics::Shape::Box {
                half_extents: Vec3::new(0.05, 0.4, 0.05),
            },
            steel,
        );
        desc.mass_override = Some(5.0);
        world.create_body(desc);

        world
            .mechanics_mut()
            .add_hinge_motor(sim_mechanics::HingeMotorPd {
                body: 0,
                axis: Vec3::new(0.0, 0.0, 1.0),
                reference_rotation: Quat::IDENTITY,
                theta_target: 0.0,
                kp: 20.0,
                kd: 2.0,
                torque_max: 50.0,
                limit: None,
                disabled: false,
            });

        for _ in 0..60 {
            world.step();
        }
        let theta_before = {
            let mechanics = world.mechanics_mut();
            mechanics.hinge_motors[0].measure_angle(&mechanics.bodies)
        };
        assert!(
            theta_before.abs() < 0.05,
            "should stay near the initial target 0: theta_before={theta_before}"
        );

        let new_target = std::f64::consts::FRAC_PI_4;
        world.push_command(Command::SetMotorTarget {
            hinge_motor_index: 0,
            theta_target: new_target,
        });
        for _ in 0..300 {
            world.step();
        }
        let theta_after = {
            let mechanics = world.mechanics_mut();
            mechanics.hinge_motors[0].measure_angle(&mechanics.bodies)
        };
        assert!(
            (theta_after - new_target).abs() < 0.05,
            "should converge to the new target: theta_after={theta_after} new_target={new_target}"
        );
    }

    /// 無効な`BodyId`(削除済み)を参照する`ApplyForce`はパニックせず黙って無視される
    /// (設計§1「削除済みIDへのアクセスはNone」の不変条件、Command版)。
    #[test]
    fn apply_force_command_with_removed_body_id_is_silently_ignored() {
        let mut world = World::new(WorldOptions::default());
        let box_id = create_falling_box(&mut world);
        world.remove_body(box_id);

        world.push_command(Command::ApplyForce {
            body: box_id,
            force: Vec3::new(100.0, 0.0, 0.0),
            point: None,
        });
        world.step(); // パニックしないことの確認そのものがテスト。
        assert_eq!(world.command_log().len(), 1);
    }

    /// `Command::SetSwitch`(設計§2「`SetSwitch{circuit, element, closed}`」、`World`は
    /// 単一`circuit`ドメイン前提のため`circuit`引数は省略)。分圧回路の負荷抵抗と並列に
    /// 置いたスイッチを閉じると、`sim_em::circuit`の単体テストと同じ理屈で分圧点の電圧が
    /// ほぼ0まで落ちることを確認する。
    #[test]
    fn set_switch_command_closes_switch_and_changes_circuit_state() {
        let mut world = World::new(WorldOptions::default());
        let mut circuit = sim_em::Circuit::new(3); // 0=GND, 1=電源, 2=分圧点
        circuit.add_voltage_source(1, sim_em::GROUND, 10.0);
        circuit.add_resistor(1, 2, 100.0);
        let switch = circuit.add_switch(2, sim_em::GROUND, false);
        circuit.add_resistor(2, sim_em::GROUND, 200.0);
        world.enable_circuit(circuit);

        world.step();
        let v_open = world.circuit_probe(2).unwrap();
        assert!(
            (v_open - 10.0 * 200.0 / 300.0).abs() / (10.0 * 200.0 / 300.0) < 0.01,
            "switch open: v_open={v_open}"
        );

        world.push_command(Command::SetSwitch {
            switch_index: switch,
            closed: true,
        });
        world.step();
        let v_closed = world.circuit_probe(2).unwrap();
        assert!(
            v_closed.abs() < 1e-3,
            "switch closed should short node 2 to GND, got {v_closed}"
        );
    }

    /// `World::drain_events`(設計docs/20-integration/04-world-api.md §2「イベント
    /// 購読」、モジュールdoc「縮約実装の理由」参照)。跳ねる球の`ContactStarted`/
    /// `ContactEnded`(`sim_mechanics::MechanicsSolver::emit_contact_events`が
    /// 発行する`World`最初のイベント)が、`World::step()`経由でも正しい`step`値
    /// (発行元ドメインが埋めた`0`ではなく、実際に発生した`World`のstep_count)で
    /// 排出されることを確認する。
    #[test]
    fn drain_events_surfaces_contact_started_and_ended_with_correct_step() {
        let mut world = World::new(WorldOptions::default());
        let steel = world.materials().find_by_name("鋼(炭素鋼)").unwrap();

        let mut floor = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            steel,
        );
        floor.body_type = BodyType::Static;
        world.create_body(floor);

        let mut ball = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.5 }, steel);
        ball.transform.position = Vec3::new(0.0, 2.0, 0.0);
        world.create_body(ball);

        let mut all_events = Vec::new();
        let mut max_step_seen = 0u64;
        for _ in 0..300 {
            world.step();
            max_step_seen = world.step_count();
            all_events.extend(world.drain_events());
        }

        let started = all_events
            .iter()
            .filter(|e| e.kind == sim_core::EventKind::ContactStarted)
            .count();
        let ended = all_events
            .iter()
            .filter(|e| e.kind == sim_core::EventKind::ContactEnded)
            .count();
        assert!(started >= 1, "should observe at least one ContactStarted");
        assert!(ended >= 1, "should observe at least one ContactEnded");
        for e in &all_events {
            assert!(
                e.step >= 1 && e.step <= max_step_seen,
                "event step should be a real World step_count, not the domain's placeholder 0: e.step={} max_step_seen={max_step_seen}",
                e.step
            );
        }
    }

    /// `Command::SetHeatSource`(設計§2「`SetHeatSource{node, watts}`」)。モジュールdoc
    /// 「1step分だけ効く」縮約(`ApplyForce`と同じ)どおり、1回のpushで1step分の
    /// $Q=watts \cdot dt$ だけ温度が上昇し、2step目以降は追加のpushなしには温度が
    /// 変化しない(外部熱源が持続しない)ことを確認する。
    #[test]
    fn set_heat_source_command_raises_temperature_for_one_step_only() {
        let mut world = World::new(WorldOptions::default());
        let mut thermal = sim_thermal::ThermalSolver::new(293.15);
        let node = thermal.add_node(sim_thermal::ThermalNode::new(293.15, 10.0));
        world.enable_thermal(thermal);

        let watts = 500.0;
        let dt = WorldOptions::default().dt;
        world.push_command(Command::SetHeatSource { node, watts });
        world.step();

        let expected_t1 = 293.15 + watts * dt / 10.0;
        let t1 = world.thermal().unwrap().nodes[node].temperature;
        assert!(
            (t1 - expected_t1).abs() < 1e-6,
            "t1={t1} expected_t1={expected_t1}"
        );

        world.step(); // 追加のpushなし。
        let t2 = world.thermal().unwrap().nodes[node].temperature;
        assert!(
            (t2 - t1).abs() < 1e-9,
            "temperature must not keep rising without re-pushing the command: t1={t1} t2={t2}"
        );
    }

    /// **群3: 量子・統計・FDTD が `World::step()` で実際に動くこと**。
    ///
    /// これらは長らく `Solver` 未実装で **`World` に載る経路が原理的に存在しなかった**
    /// (`enable_*` が無いのでシーンJSONからもエディタからも到達できず、D25/D27–D32 が
    /// 「ドメイン自体が無い」として滞留していた)。載っただけで満足しないよう、
    /// **各ドメインが持つ保存量・特徴量で「実際に正しく進んだこと」**を確認する。
    #[test]
    fn group3_domains_actually_advance_inside_world_step() {
        // ① 量子1D: split-step Fourier はユニタリなのでノルムが厳密に保存する
        //    (Q1 と同じ検証量)。かつ波束は実際に動く(⟨x⟩ が変化する)。
        let mut world = World::new(WorldOptions::default());
        let mut wave = sim_quantum::WaveFunction1D::new(128, 0.1);
        wave.set_gaussian_wave_packet(4.0, 0.5, 5.0);
        let norm0 = wave.norm();
        let mean_x0 = wave.mean_x();
        world.enable_quantum_1d(wave);
        for _ in 0..50 {
            world.step();
        }
        let q = world.quantum_1d().unwrap();
        assert!(
            (q.norm() - norm0).abs() < 1e-9,
            "TDSE must be unitary: norm0={norm0} norm={}",
            q.norm()
        );
        assert!(
            (q.mean_x() - mean_x0).abs() > 1e-3,
            "the wave packet must actually move: <x>0={mean_x0} <x>={}",
            q.mean_x()
        );

        // ② 気体分子運動論: 壁は鏡面反射(断熱)・衝突は弾性なのでエネルギー保存。
        //
        // **フレームdtを分子気体の時間スケールに合わせる**。既定の 1/120 s だと
        // `max_stable_dt`(最速粒子が半径ぶん動く時間 ~1e-13 s)との比が 10¹⁰ になり、
        // sub-step 上限(1000)で毎フレーム打ち切られて意味のある積分にならない
        // ——**実際に `World::step()` が返ってこなくなり、上限機構を入れる契機になった**。
        let mut world = World::new(WorldOptions {
            dt: 1.0e-12,
            ..Default::default()
        });
        let mut gas = sim_statistical::GasSim::new(4.65e-26, 1.5e-10, Vec3::new(1e-7, 1e-7, 1e-7));
        let mut rng = SimRng::new(11, 0);
        for _ in 0..200 {
            let p = Vec3::new(
                rng.next_f64() * 1e-7,
                rng.next_f64() * 1e-7,
                rng.next_f64() * 1e-7,
            );
            gas.add_particle(p, rng.maxwell_boltzmann_velocity(300.0));
        }
        let e0 = gas.kinetic_energy();
        world.enable_kinetic_gas(gas);
        for _ in 0..100 {
            world.step();
        }
        let e1 = world.kinetic_gas().unwrap().kinetic_energy();
        assert!(
            (e1 - e0).abs() / e0 < 1e-9,
            "hard-sphere gas with specular walls must conserve energy: e0={e0} e1={e1}"
        );

        // ③ FDTD: PEC空洞は無損失なので電磁エネルギーが保存する(既存のクレート内
        //    テストと同じ検証量を、`World::step()` 経由で成り立つことで確認する)。
        //    **`step_dt` の一般化が効いているかもここで見る**——`World` の dt
        //    (1/120 s)は FDTD の Courant dt とは無関係なので、`max_stable_dt` に
        //    従った sub-step が正しく効かないとここで発散する。
        let mut world = World::new(WorldOptions::default());
        let (nx, ny) = (33, 33);
        let mut fdtd = sim_em::FdtdSim2D::new(nx, ny, 0.02, 0.5);
        for j in 1..ny - 1 {
            for i in 1..nx - 1 {
                let sx = (std::f64::consts::PI * i as f64 / (nx - 1) as f64).sin();
                let sy = (std::f64::consts::PI * j as f64 / (ny - 1) as f64).sin();
                fdtd.set_ez(i, j, sx * sy);
            }
        }
        let e0 = fdtd.total_energy();
        world.enable_fdtd(fdtd);
        for _ in 0..20 {
            world.step();
        }
        let e1 = world.fdtd().unwrap().total_energy();
        assert!(
            (e1 - e0).abs() / e0 < 0.05,
            "lossless PEC cavity must conserve field energy: e0={e0} e1={e1}"
        );

        // ④ イジング模型: 高温では磁化が 0 付近に留まる(無秩序相)。
        //    `dt` に依存しないこと(モンテカルロには物理時間が無い)も同時に見る。
        let mut world = World::new(WorldOptions::default());
        let mut ising = sim_statistical::IsingSim::new(16, 1.0, 50.0, &mut SimRng::new(3, 0));
        ising.updates_per_step = 2;
        world.enable_ising(ising);
        for _ in 0..40 {
            world.step();
        }
        let m = world.ising().unwrap().magnetization().abs();
        assert!(m < 0.3, "high-T Ising must stay disordered: |m|={m}");

        // ⑤ ブラウン運動: 外力ゼロの自由拡散で、粒子は実際に広がる。
        let mut world = World::new(WorldOptions {
            dt: 1.0e-8,
            ..Default::default()
        });
        let mut brownian =
            sim_statistical::BrownianParticleSet::new(5.5e-16, 9.4e-9, 1.380649e-23 * 293.15);
        for _ in 0..500 {
            brownian.add_particle(Vec3::ZERO, Vec3::ZERO);
        }
        world.enable_brownian(brownian);
        for _ in 0..200 {
            world.step();
        }
        let b = world.brownian().unwrap();
        let msd: f64 =
            b.position.iter().map(|p| p.length_sq()).sum::<f64>() / b.position.len() as f64;
        // ⟨Δx²⟩ = 6Dt。統計誤差があるので桁で確認する(厳密値は S4 テストが担保)。
        let expected = 6.0 * b.diffusion_coefficient() * (200.0 * 1.0e-8);
        assert!(
            msd > 0.2 * expected && msd < 5.0 * expected,
            "free diffusion MSD should be within an order of 6Dt: msd={msd} expected={expected}"
        );
    }

    /// **群3: `energy_report` が単位系と保存性を正直に分けて出すこと**。
    /// **`total_energy()` の合計に非SI・非保存ドメインを混ぜてはいけない**
    /// ——混ぜると `EnergyLedger` の残差が「バグの兆候」ではなく「仕様」になり、
    /// CIの保存則ゲートが機能しなくなる。
    #[test]
    fn energy_report_separates_non_si_and_non_conservative_domains() {
        let mut world = World::new(WorldOptions::default());
        create_falling_box(&mut world);
        let mut wave = sim_quantum::WaveFunction1D::new(64, 0.1);
        wave.set_gaussian_wave_packet(2.0, 0.4, 3.0);
        world.enable_quantum_1d(wave);
        world.enable_ising(sim_statistical::IsingSim::new(
            8,
            1.0,
            2.0,
            &mut SimRng::new(1, 0),
        ));
        world.step();

        let report = world.energy_report();
        let quantum = report.iter().find(|d| d.domain == "Quantum1D").unwrap();
        assert!(!quantum.in_total, "原子単位の量子はSI合計に入れない");
        assert_eq!(quantum.unit, "Ha (原子単位)");
        let ising = report.iter().find(|d| d.domain == "Ising").unwrap();
        assert!(!ising.in_total);
        assert!(!ising.conservative, "正準集団のサンプリングは保存しない");
        let mechanics = report.iter().find(|d| d.domain == "Mechanics").unwrap();
        assert!(mechanics.in_total && mechanics.conservative);

        // 合計は `in_total` のドメインだけの和に一致する。
        let expected: f64 = report
            .iter()
            .filter(|d| d.in_total)
            .map(|d| d.energy.total())
            .sum();
        assert!(
            (world.total_energy().total() - expected).abs() < 1e-9,
            "total_energy must equal the sum of in_total domains"
        );

        // 単位系が混在していることを近似バッジで申告する。
        let approximations = world.active_approximations();
        assert!(
            approximations.iter().any(|a| a.name == "単位系の混在"),
            "mixing SI and atomic units must be reported: {:?}",
            approximations.iter().map(|a| a.name).collect::<Vec<_>>()
        );
    }

    /// **`remove_body` の連鎖削除(群2)**。削除した剛体にジョイントで繋がった
    /// 相手が、遠方(y=-1e9)へ退避させられた側に**引きずられて飛んでいかない**
    /// ことを確認する。連鎖削除が無いと拘束はそのまま解かれ続けるため、
    /// 相手は 1e9 m のオーダーで落ちる——「無効化しただけ」では足りないことが
    /// 数値としてはっきり出る。
    #[test]
    fn remove_body_detaches_joints_so_the_partner_is_not_dragged_away() {
        let mut world = World::new(WorldOptions::default());
        let anchor = create_falling_box(&mut world);
        let hanging = create_falling_box(&mut world);
        // 2体を長さ1のDistanceJointで繋ぐ。
        world
            .mechanics_mut()
            .add_distance_joint(sim_mechanics::DistanceJoint {
                body_a: anchor.index as usize,
                anchor_a: Vec3::ZERO,
                body_b: Some(hanging.index as usize),
                anchor_b: Vec3::ZERO,
                length: 1.0,
                disabled: false,
            });
        for _ in 0..10 {
            world.step();
        }
        let before = world.body_position(hanging).unwrap();

        world.remove_body(anchor);
        for _ in 0..60 {
            world.step();
        }
        let after = world.body_position(hanging).unwrap();
        // 自由落下しているので下がってはいるが、退避先(-1e9)へは連れていかれない。
        assert!(
            after.y > -100.0,
            "partner must not be dragged toward the removed body's parking spot: \
             before={before:?} after={after:?}"
        );
        // 削除済み ID へのアクセスは None(設計の不変条件)。
        assert!(world.body_position(anchor).is_none());
    }

    /// **Inspector の編集を Command として適用する(群2)**。設計
    /// docs/23-frontend/01-editor.md §1.3「編集は次ステップ先頭で Command として
    /// 適用される」の実体。3つとも「観測可能な物理の変化」で確認する:
    ///
    /// - `SetBodyMass`: 質量を変えても**自由落下の加速度は変わらない**
    ///   (ガリレオ)——質量が実際に効いていることは、代わりに同じ力を加えたときの
    ///   加速度が $a = F/m$ で反比例することで示す。
    /// - `SetBodyType`: Static 化すると重力下でも一切動かない。
    /// - `SetCollisionFilter`: 床と別グループへ移すと**床をすり抜けて落ち続ける**。
    #[test]
    fn inspector_edit_commands_change_mass_body_type_and_collision_filter() {
        // ① SetBodyMass: 同じ力に対する加速度が a = F/m に従う。
        let force = 100.0;
        let mut accelerations = Vec::new();
        for mass in [1.0, 4.0] {
            let mut world = World::new(WorldOptions::default());
            let body = create_falling_box(&mut world);
            world.push_command(Command::SetBodyMass { body, mass });
            world.step(); // 質量変更をこのstepの先頭で適用。
            let v0 = world.body_velocity(body).unwrap().x;
            let steps = 10;
            for _ in 0..steps {
                world.push_command(Command::ApplyForce {
                    body,
                    force: Vec3::new(force, 0.0, 0.0),
                    point: None,
                });
                world.step();
            }
            let v1 = world.body_velocity(body).unwrap().x;
            let dt = WorldOptions::default().dt;
            accelerations.push((v1 - v0) / (steps as f64 * dt));
        }
        // 質量4倍 → 加速度1/4(重力はx方向に効かないので純粋に F/m)。
        let ratio = accelerations[0] / accelerations[1];
        assert!(
            (ratio - 4.0).abs() < 1e-6,
            "a ∝ 1/m should hold: a(m=1)={} a(m=4)={} ratio={ratio}",
            accelerations[0],
            accelerations[1]
        );

        // ② SetBodyType: Static 化すると重力下でも動かない。
        let mut world = World::new(WorldOptions::default());
        let body = create_falling_box(&mut world);
        let start = world.body_position(body).unwrap();
        world.push_command(Command::SetBodyType {
            body,
            body_type: BodyType::Static,
            mass: 1.0,
        });
        for _ in 0..120 {
            world.step();
        }
        let held = world.body_position(body).unwrap();
        assert!(
            (held - start).length() < 1e-12,
            "static body must not move at all: start={start:?} held={held:?}"
        );

        // ③ SetCollisionFilter: 床(既定グループ 1)と互いに不可視にすると
        //    床で止まらず自由落下し続ける。
        let mut baseline = World::new(WorldOptions::default());
        add_ground_plane(&mut baseline);
        let falling = create_falling_box(&mut baseline);
        for _ in 0..400 {
            baseline.step();
        }
        let rested_y = baseline.body_position(falling).unwrap().y;
        assert!(
            rested_y > 0.0,
            "sanity: the box must rest on the ground without a filter (y={rested_y})"
        );

        let mut filtered = World::new(WorldOptions::default());
        add_ground_plane(&mut filtered);
        let ghost = create_falling_box(&mut filtered);
        filtered.push_command(Command::SetCollisionFilter {
            body: ghost,
            group: 0b10,
            mask: 0b10,
        });
        for _ in 0..400 {
            filtered.step();
        }
        let fallen_y = filtered.body_position(ghost).unwrap().y;
        assert!(
            fallen_y < -1.0,
            "filtered body must pass through the ground (y={fallen_y}, baseline={rested_y})"
        );
    }

    /// `Command::Grab`/`MoveGrab`/`Release`(設計§2「マウスでつかむ」、モジュールdoc
    /// 「`length=0`のピン拘束」参照)。落下中の箱を`Grab`すると重力に反して目標点付近に
    /// 保持され、`MoveGrab`で目標点を動かすと追従し、`Release`すると再び自由落下する
    /// (重力で加速し始める)ことを確認する。
    #[test]
    fn grab_move_grab_release_pin_and_release_a_falling_body() {
        let mut world = World::new(WorldOptions::default());
        let box_id = create_falling_box(&mut world);
        let start_y = world.body_position(box_id).unwrap().y;

        let target1 = Vec3::new(0.0, start_y, 0.0);
        world.push_command(Command::Grab {
            body: box_id,
            anchor_local: Vec3::ZERO,
            target: target1,
        });
        for _ in 0..120 {
            world.step();
        }
        let pos_grabbed = world.body_position(box_id).unwrap();
        assert!(
            (pos_grabbed - target1).length() < 0.05,
            "grab should pin the body near the target despite gravity: pos_grabbed={pos_grabbed:?} target1={target1:?}"
        );

        let target2 = Vec3::new(2.0, start_y, 0.0);
        world.push_command(Command::MoveGrab {
            body: box_id,
            target: target2,
        });
        for _ in 0..300 {
            world.step();
        }
        let pos_moved = world.body_position(box_id).unwrap();
        assert!(
            (pos_moved - target2).length() < 0.05,
            "move_grab should pull the body to the new target: pos_moved={pos_moved:?} target2={target2:?}"
        );

        world.push_command(Command::Release { body: box_id });
        world.step();
        let v_after_one_step = world.body_velocity(box_id).unwrap();
        for _ in 0..30 {
            world.step();
        }
        let v_after_more_steps = world.body_velocity(box_id).unwrap();
        assert!(
            v_after_more_steps.y < v_after_one_step.y - 0.1,
            "released body should resume free fall (accelerating downward): \
             v_after_one_step={v_after_one_step:?} v_after_more_steps={v_after_more_steps:?}"
        );
    }

    /// `World::raycast`(設計docs/20-integration/04-world-api.md §2、`raycast`
    /// モジュールdoc参照): `RayHit::body`が生インデックスではなく世代付き`BodyId`を
    /// 正しく返すことを確認する(削除済みindexの再利用と取り違えないための不変条件)。
    #[test]
    fn raycast_returns_body_id_with_correct_generation() {
        let mut world = World::new(WorldOptions::default());
        let box_id = create_falling_box(&mut world);
        let y0 = world.body_position(box_id).unwrap().y;

        let hit = world
            .raycast(
                Vec3::new(0.0, y0 + 10.0, 0.0),
                Vec3::new(0.0, -1.0, 0.0),
                100.0,
                &QueryFilter::default(),
            )
            .expect("ray straight down should hit the box");
        assert_eq!(hit.body, box_id);
        // 箱の半径0.5の上面までの距離(重心からの10mからさらに半径分近い)。
        assert!(
            (hit.distance - 9.5).abs() < 1e-9,
            "distance={}",
            hit.distance
        );
    }

    /// **増分F1**: `QueryFilter`が実際に絞り込むこと。`exclude`(自分自身を無視して
    /// レイを飛ばす典型的な用途)と`exclude_dynamic`(静的な地形だけ拾う)を確認する。
    #[test]
    fn query_filter_excludes_bodies_by_id_and_by_static_dynamic_kind() {
        let mut world = World::new(WorldOptions::default());
        let steel = world.materials().find_by_name("鋼(炭素鋼)").unwrap();
        let mut ground_desc = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            steel,
        );
        ground_desc.body_type = sim_mechanics::BodyType::Static;
        let ground = world.create_body(ground_desc);
        let box_id = create_falling_box(&mut world);
        let y0 = world.body_position(box_id).unwrap().y;
        let origin = Vec3::new(0.0, y0 + 10.0, 0.0);
        let down = Vec3::new(0.0, -1.0, 0.0);

        // フィルタ無しなら手前の箱に当たる。
        let hit = world
            .raycast(origin, down, 100.0, &QueryFilter::default())
            .expect("フィルタ無しでは箱に当たる");
        assert_eq!(hit.body, box_id);

        // 箱を除外すると、`raycast`が最近傍1件しか返さない縮約(doc参照)により
        // **背後の地面ではなく`None`になる**。この挙動自体を固定する。
        let excluded = QueryFilter {
            exclude: vec![box_id],
            ..QueryFilter::default()
        };
        assert!(
            world.raycast(origin, down, 100.0, &excluded).is_none(),
            "最近傍が除外対象なら次の候補は返さない(縮約、raycastのdoc参照)"
        );

        // overlap_sphere は全件を返すので、フィルタは素直に絞り込みとして効く。
        let both = world.overlap_sphere(Vec3::new(0.0, y0, 0.0), 2.0, &QueryFilter::default());
        assert!(both.contains(&box_id), "フィルタ無しなら箱が含まれる");

        let dynamic_only = world.overlap_sphere(
            Vec3::new(0.0, y0, 0.0),
            2.0,
            &QueryFilter {
                exclude_static: true,
                ..QueryFilter::default()
            },
        );
        assert!(dynamic_only.contains(&box_id));
        assert!(
            !dynamic_only.contains(&ground),
            "exclude_static は静的な地面を落とすべき"
        );

        let static_only = world.overlap_sphere(
            Vec3::new(0.0, y0, 0.0),
            2.0,
            &QueryFilter {
                exclude_dynamic: true,
                ..QueryFilter::default()
            },
        );
        assert!(
            !static_only.contains(&box_id),
            "exclude_dynamic は動的な箱を落とすべき"
        );
    }

    /// `World::overlap_sphere`(設計docs/20-integration/04-world-api.md §2、`overlap`
    /// モジュールdoc参照): 重なる剛体の`BodyId`(世代付き)を正しく返すことを確認する。
    #[test]
    fn overlap_sphere_returns_body_ids_of_overlapping_bodies_only() {
        let mut world = World::new(WorldOptions::default());
        let box_id = create_falling_box(&mut world);
        let position = world.body_position(box_id).unwrap();

        let near_hits = world.overlap_sphere(position, 0.1, &QueryFilter::default());
        assert_eq!(near_hits, vec![box_id]);

        let far_hits = world.overlap_sphere(
            position + Vec3::new(1000.0, 0.0, 0.0),
            0.1,
            &QueryFilter::default(),
        );
        assert!(far_hits.is_empty());
    }

    /// `Probe`(設計docs/20-integration/04-world-api.md §2.1「測って遊ぶの中心機能」):
    /// `BodyPosY`が箱の自由落下を毎stepサンプルし、履歴が単調減少することを確認する。
    /// **step数ぶんのサンプルが1つも欠けずに残る**ことも併せて見る(履歴は可変長、
    /// `Probe`のdoc参照)。
    #[test]
    fn probe_body_pos_y_samples_falling_box_every_step() {
        let mut world = World::new(WorldOptions::default());
        let box_id = create_falling_box(&mut world);
        let handle = world.add_probe(ProbeTarget::BodyPosY(box_id));

        for _ in 0..30 {
            world.step();
        }

        let probe = world.probe(handle).unwrap();
        let history: Vec<f64> = probe.history().copied().collect();
        assert_eq!(history.len(), 30, "30step分がそのまま残る(切り詰めない)");
        // 単調減少(自由落下、接触前)。
        for pair in history.windows(2) {
            assert!(
                pair[0] > pair[1],
                "history should be monotonically decreasing: {history:?}"
            );
        }
        // 最後のサンプルは直近のbody_position()と一致するはず。
        let final_y = world.body_position(box_id).unwrap().y;
        assert!((history.last().unwrap() - final_y).abs() < 1e-12);
    }

    /// **旧・固定容量(`DEFAULT_PROBE_CAPACITY`=6000)を大きく超えても1サンプルも
    /// 失われないこと**。以前はここで先頭が無言に捨てられ、「グラフの左端が実は
    /// 0秒ではない」という形で静かに誤読を生んでいた(`Probe`のdoc参照)。
    ///
    /// 判定は本数だけでなく**先頭のサンプルが残っているか**まで見る——
    /// リングバッファに戻ると真っ先に消えるのがそこだからである。
    #[test]
    fn probe_history_keeps_every_sample_far_past_the_old_fixed_capacity() {
        const OLD_FIXED_CAPACITY: usize = 6000;
        const STEPS: usize = 10_000;

        let mut world = World::new(WorldOptions::default());
        let box_id = create_falling_box(&mut world);
        let handle = world.add_probe(ProbeTarget::BodyPosY(box_id));

        // 1step目のサンプル(切り詰めが起きれば最初に失われる値)を控えておく。
        world.step();
        let first_sample = *world.probe(handle).unwrap().history().next().unwrap();

        for _ in 1..STEPS {
            world.step();
        }

        // 旧容量を超える長さで走らせていること(この関係が崩れたらこのテストは
        // 何も検証していない)。定数どうしの比較なのでコンパイル時に見る。
        const _: () = assert!(STEPS > OLD_FIXED_CAPACITY);

        let probe = world.probe(handle).unwrap();
        assert_eq!(
            probe.len(),
            STEPS,
            "旧容量({OLD_FIXED_CAPACITY})を超えても切り詰めない"
        );
        assert_eq!(probe.history().count(), STEPS);
        assert_eq!(
            *probe.history().next().unwrap(),
            first_sample,
            "先頭のサンプルが残っていること(リングバッファなら最初に消える値)"
        );

        // メモリ使用量の問い合わせ口(上限を課さない代わりの観測手段)。
        assert_eq!(world.probe_history_len(handle), Some(STEPS));
        assert_eq!(world.probe_history_len(handle + 1), None);
        assert_eq!(
            world.probe_history_bytes_estimate(),
            STEPS * std::mem::size_of::<f64>()
        );
    }

    /// `probe_history_bytes_estimate`が**全プローブの合計**であること。
    #[test]
    fn probe_history_bytes_estimate_sums_every_probe() {
        let mut world = World::new(WorldOptions::default());
        let box_id = create_falling_box(&mut world);
        world.add_probe(ProbeTarget::BodyPosY(box_id));
        world.add_probe(ProbeTarget::BodySpeed(box_id));
        assert_eq!(world.probe_history_bytes_estimate(), 0);

        for _ in 0..50 {
            world.step();
        }
        assert_eq!(
            world.probe_history_bytes_estimate(),
            2 * 50 * std::mem::size_of::<f64>()
        );
    }

    /// `ProbeTarget::LedgerKinetic`・`StateHashDigest`が無効なindex/id無しでも
    /// パニックせず妥当な値をサンプルすることを確認する(常時有効なmechanicsドメイン
    /// のみに依存するターゲット)。
    #[test]
    fn probe_ledger_kinetic_and_state_hash_digest_sample_without_panicking() {
        let mut world = World::new(WorldOptions::default());
        create_falling_box(&mut world);
        let kinetic_handle = world.add_probe(ProbeTarget::LedgerKinetic);
        let hash_handle = world.add_probe(ProbeTarget::StateHashDigest);

        for _ in 0..5 {
            world.step();
        }

        let kinetic_history: Vec<f64> = world
            .probe(kinetic_handle)
            .unwrap()
            .history()
            .copied()
            .collect();
        assert_eq!(kinetic_history.len(), 5);
        assert!(
            kinetic_history.last().unwrap() > &0.0,
            "falling box should have kinetic energy"
        );

        let hash_history: Vec<f64> = world
            .probe(hash_handle)
            .unwrap()
            .history()
            .copied()
            .collect();
        assert_eq!(hash_history.len(), 5);
    }

    /// `World::circuit_probe`(設計docs/20-integration/04-world-api.md §2
    /// `circuit_probe(id, node)`、`World`単一回路への縮約は同メソッドのdoc参照):
    /// 回路ドメイン未有効化なら`None`、有効化後は`Circuit::node_voltage`と一致することを
    /// 確認する。
    #[test]
    fn circuit_probe_reads_node_voltage_when_circuit_domain_enabled() {
        let mut world = World::new(WorldOptions::default());
        assert_eq!(
            world.circuit_probe(1),
            None,
            "no circuit domain enabled yet"
        );

        let mut circuit = sim_em::Circuit::new(2);
        circuit.add_voltage_source(1, sim_em::GROUND, 5.0);
        circuit.add_resistor(1, sim_em::GROUND, 100.0);
        world.enable_circuit(circuit);
        world.step();

        let probed = world.circuit_probe(1).unwrap();
        let expected = world.circuit().unwrap().node_voltage(1);
        assert_eq!(probed, expected);
    }

    /// レジーム切替(設計docs/20-integration/06-regime-switching.md §1)の土台:
    /// `Astro`レジーム中は`mechanics`が完全に凍結され(1step たりとも進まない)、
    /// `Local`へ戻すと再び進行することを確認する。
    #[test]
    fn astro_regime_freezes_mechanics_and_local_regime_resumes_it() {
        let mut world = World::new(WorldOptions::default());
        let steel = world.materials().find_by_name("鋼(炭素鋼)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.5 }, steel);
        desc.transform.position = Vec3::new(0.0, 50.0, 0.0);
        desc.linear_velocity = Vec3::new(1.0, 0.0, 0.0);
        let body = world.create_body(desc);

        world.enable_astro(sim_astro::NBodySystem::new(0.0));
        world.set_time_regime(sim_astro::TimeRegime::Astro {
            dt_astro: 0.1,
            steps_per_frame: 1,
        });

        let frozen_position = world.body_position(body).unwrap();
        let frozen_velocity = world.body_velocity(body).unwrap();
        for _ in 0..10 {
            world.step();
        }
        assert_eq!(
            world.body_position(body).unwrap(),
            frozen_position,
            "mechanics must not advance at all while in Astro regime"
        );
        assert_eq!(world.body_velocity(body).unwrap(), frozen_velocity);

        world.set_time_regime(sim_astro::TimeRegime::Local { steps_per_frame: 1 });
        for _ in 0..10 {
            world.step();
        }
        assert_ne!(
            world.body_position(body).unwrap(),
            frozen_position,
            "mechanics must resume advancing once back in Local regime"
        );
    }

    /// 再突入(D37)の最小骨格: 天体ドメイン(`NBodySystem`)で周回するカプセルの
    /// 軌道状態を、既存の`sim_astro::astro_to_local_state`(設計§3.2、フレーム変換)で
    /// 地表フレームのローカル座標・速度に変換し、その状態でLocal物理側に
    /// `RigidBody`を新設して引き継がせる。数値は現実の軌道力学の再現を狙った
    /// ものではなく(縮約実装、実際のD37合格基準は後続増分)、レジーム切替+
    /// フレーム変換+ローカル物理再開という一連の配線が実際に機能することの検証。
    #[test]
    fn switching_from_astro_to_local_hands_off_orbital_state_via_frame_conversion() {
        let mut world = World::new(WorldOptions::default());

        let mut astro = sim_astro::NBodySystem::new(0.0);
        astro.add_body(Vec3::ZERO, Vec3::ZERO, 1.0e15);
        let capsule_index =
            astro.add_body(Vec3::new(1000.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0), 1.0);
        world.enable_astro(astro);
        world.set_time_regime(sim_astro::TimeRegime::Astro {
            dt_astro: 1.0,
            steps_per_frame: 5,
        });
        for _ in 0..5 {
            world.step();
        }

        let (orbital_position, orbital_velocity) = {
            let a = world.astro().expect("astro enabled above");
            (a.position[capsule_index], a.velocity[capsule_index])
        };

        let mut frames = sim_core::FrameTree::new();
        let surface_frame = frames.add_frame(
            sim_core::FrameId::ROOT,
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::new(0.0, 0.0, 7.292e-5),
        );
        let (local_position, local_velocity) = sim_astro::astro_to_local_state(
            &frames,
            surface_frame,
            orbital_position,
            orbital_velocity,
        );

        let steel = world.materials().find_by_name("鋼(炭素鋼)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 1.0 }, steel);
        desc.transform.position = local_position;
        desc.linear_velocity = local_velocity;
        let capsule_body = world.create_body(desc);

        world.set_time_regime(sim_astro::TimeRegime::Local { steps_per_frame: 1 });

        assert_eq!(
            world.body_position(capsule_body).unwrap(),
            local_position,
            "handed-off body must start exactly at the frame-converted position"
        );
        assert_eq!(world.body_velocity(capsule_body).unwrap(), local_velocity);

        for _ in 0..10 {
            world.step();
        }
        assert_ne!(
            world.body_position(capsule_body).unwrap(),
            local_position,
            "local physics must continue evolving the handed-off body"
        );
    }

    /// 閾値ベース自動レジーム切替(`AutoRegimeSwitchConfig`のdoc参照): Astroレジーム中、
    /// 中心天体からの距離が閾値を下回った時点で`step()`が自動的に(既存の手動ハンドオフ
    /// 手順と同じフレーム変換で)Localボディへ状態を書き込み、レジームをLocalへ切り替える
    /// ことを確認する。`dt_astro: 0.0`により天体状態を切替判定の瞬間に固定し(積分器の
    /// 詳細に依存しない)、`sim_astro::astro_to_local_state`を直接呼んだ期待値と厳密一致
    /// することを確認する(既存の手動ハンドオフテストと同じ変換式、自動化されたことのみが違い)。
    #[test]
    fn auto_regime_switch_triggers_when_distance_crosses_threshold_and_hands_off_state() {
        let mut world = World::new(WorldOptions::default());

        let mut astro = sim_astro::NBodySystem::new(0.0);
        astro.add_body(Vec3::ZERO, Vec3::ZERO, 1.0e15);
        let orbital_position0 = Vec3::new(1000.0, 0.0, 0.0);
        let orbital_velocity0 = Vec3::new(0.0, 10.0, 0.0);
        let capsule_index = astro.add_body(orbital_position0, orbital_velocity0, 1.0);
        world.enable_astro(astro);

        let surface_frame = world.add_frame(
            sim_core::FrameId::ROOT,
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::new(0.0, 0.0, 7.292e-5),
        );

        let (expected_position, expected_velocity) = sim_astro::astro_to_local_state(
            world.frames(),
            surface_frame,
            orbital_position0,
            orbital_velocity0,
        );

        let steel = world.materials().find_by_name("鋼(炭素鋼)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 1.0 }, steel);
        desc.transform.position = Vec3::new(0.0, 12345.0, 0.0); // プレースホルダー(切替時に上書きされる)
        let capsule_body = world.create_body(desc);

        world.configure_auto_regime_switch(AutoRegimeSwitchConfig {
            astro_body_index: capsule_index,
            central_body_index: 0,
            threshold_distance: 1500.0, // 実際の距離(1000)は既に閾値未満なので初回stepで即発火
            surface_frame,
            local_body: capsule_body,
        });

        world.set_time_regime(sim_astro::TimeRegime::Astro {
            dt_astro: 0.0,
            steps_per_frame: 1,
        });
        world.step();

        assert_eq!(
            world.time_regime(),
            sim_astro::TimeRegime::Local { steps_per_frame: 1 },
            "crossing the threshold must auto-switch the regime to Local"
        );
        assert_eq!(
            world.body_position(capsule_body).unwrap(),
            expected_position
        );
        assert_eq!(
            world.body_velocity(capsule_body).unwrap(),
            expected_velocity
        );

        for _ in 0..10 {
            world.step();
        }
        assert_ne!(
            world.body_position(capsule_body).unwrap(),
            expected_position,
            "after auto hand-off, local physics must continue evolving the body"
        );
    }

    /// 閾値を上回っている間は自動切替が発火しないこと(誤発火防止の裏取り)。
    #[test]
    fn auto_regime_switch_does_not_trigger_while_still_above_threshold_distance() {
        let mut world = World::new(WorldOptions::default());

        let mut astro = sim_astro::NBodySystem::new(0.0);
        astro.add_body(Vec3::ZERO, Vec3::ZERO, 1.0e15);
        let capsule_index =
            astro.add_body(Vec3::new(1000.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0), 1.0);
        world.enable_astro(astro);

        let surface_frame = world.add_frame(
            sim_core::FrameId::ROOT,
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::new(0.0, 0.0, 7.292e-5),
        );

        let steel = world.materials().find_by_name("鋼(炭素鋼)").unwrap();
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 1.0 }, steel);
        let placeholder = Vec3::new(0.0, 12345.0, 0.0);
        desc.transform.position = placeholder;
        let capsule_body = world.create_body(desc);

        world.configure_auto_regime_switch(AutoRegimeSwitchConfig {
            astro_body_index: capsule_index,
            central_body_index: 0,
            threshold_distance: 500.0, // 実際の距離(1000)は閾値を上回っているため発火しない
            surface_frame,
            local_body: capsule_body,
        });

        world.set_time_regime(sim_astro::TimeRegime::Astro {
            dt_astro: 0.0,
            steps_per_frame: 1,
        });
        world.step();

        assert_eq!(
            world.time_regime(),
            sim_astro::TimeRegime::Astro {
                dt_astro: 0.0,
                steps_per_frame: 1
            },
            "must stay in Astro while still above the threshold distance"
        );
        assert_eq!(
            world.body_position(capsule_body).unwrap(),
            placeholder,
            "local body must be untouched until the threshold is actually crossed"
        );
    }

    /// フレーム軸オーバーレイの土台: `World::step()`が毎step`self.frames`
    /// (`sim_core::FrameTree::step`)を実際に進めることを確認する。角速度
    /// $\omega_z$を持つフレームを`add_frame`で追加し、`Local`レジームで複数step
    /// 進めた後、フレームの回転が既知の解析回転(z軸まわり角度$\omega_z t$)に
    /// 一致することを確認する(`FrameTree::step`単体テストと同じ検証をWorld
    /// 経由で行う)。
    #[test]
    fn world_step_advances_frame_rotation_by_its_angular_velocity() {
        let dt = WorldOptions::default().dt;
        let mut world = World::new(WorldOptions::default());
        let omega_z = 1.0; // rad/s
        let spinning = world.add_frame(
            sim_core::FrameId::ROOT,
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::new(0.0, 0.0, omega_z),
        );

        let steps = 100u32;
        for _ in 0..steps {
            world.step();
        }

        let elapsed = dt * steps as f64;
        let expected_rotation = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), omega_z * elapsed);
        let probe = Vec3::new(1.0, 0.0, 0.0);
        let actual = world
            .frames()
            .frame(spinning)
            .rotation_in_parent
            .rotate(probe);
        let expected = expected_rotation.rotate(probe);
        let rel_err = (actual - expected).length() / expected.length();
        // `FrameTree::step`は一次積分(`Quat::integrate_angular_velocity`)のため、
        // Worldの既定dt(細かいdtを使う`FrameTree::step`単体テストより粗い)では
        // 離散化誤差がやや大きくなる。ここでは配線(WorldがちゃんとFrameTree::step
        // を毎step呼んでいること)の検証が主眼なので、rel<1e-4を採用する。
        assert!(
            rel_err < 1e-4,
            "frame rotation should match analytic rotation after {steps} world steps: \
             actual={actual:?} expected={expected:?} rel_err={rel_err:e}"
        );
    }

    /// Scale Gizmo(`set_body_shape`)向けの検証: 立方体の半辺長を2倍にすると、
    /// 体積は2^3=8倍になるため質量も8倍になり(密度は不変)、mass()から
    /// それが正しく読み取れること・`shape_of`が実際に新しい寸法を返すことを
    /// 確認する。
    #[test]
    fn set_body_shape_rescales_mass_by_volume_ratio_and_updates_shape_query() {
        let mut world = World::new(WorldOptions::default());
        let idx = create_falling_box(&mut world);
        let mass_before = world.mechanics_mut().bodies.mass(idx.index as usize);

        world.set_body_shape(
            idx,
            Shape::Box {
                half_extents: Vec3::new(1.0, 1.0, 1.0),
            },
        );

        let mass_after = world.mechanics_mut().bodies.mass(idx.index as usize);
        assert!(
            (mass_after / mass_before - 8.0).abs() < 1e-9,
            "doubling half_extents in all 3 axes must scale volume (and thus mass) by 2^3=8, got ratio {}",
            mass_after / mass_before
        );

        match world.mechanics_mut().bodies.shape_of(idx.index as usize) {
            Shape::Box { half_extents } => {
                assert_eq!(*half_extents, Vec3::new(1.0, 1.0, 1.0));
            }
            other => panic!("expected Box shape, got {other:?}"),
        }
    }

    /// `set_body_shape`は無効な(削除済み/範囲外の)`BodyId`に対しては静かに
    /// 無視する(`remove_body`と同じ不変条件、パニックしない)。
    #[test]
    fn set_body_shape_on_invalid_body_id_is_a_no_op() {
        let mut world = World::new(WorldOptions::default());
        let idx = create_falling_box(&mut world);
        world.remove_body(idx);

        // Must not panic.
        world.set_body_shape(idx, Shape::Sphere { radius: 5.0 });
    }

    /// Scale Gizmoが静止済み(asleep)のボディへ形状変更を適用した場合の回帰
    /// テスト。バグ再現手順: 箱を床に着地・静止させてasleepにしてから、その場で
    /// 大きく拡大する——`set_shape`が`still_time`/`asleep`をリセットしなければ、
    /// asleep同士(静的な床+asleepな箱)の接触は`MechanicsSolver::
    /// manifold_is_active`により再解決されず、拡大後の新しい半辺長が床へ
    /// 深く干渉したまま物理的に一切動かなくなる(見た目上は形状だけ変わって
    /// 位置が追従しない、というエディタ上の実バグとして発見)。`set_shape`の
    /// 修正後は、次stepで確実に起床・再接触解決され、新しい半辺長ぶんだけ
    /// 押し上げられて正しい高さに収束する。
    #[test]
    fn set_body_shape_wakes_a_sleeping_body_so_it_resolves_the_new_interpenetration() {
        let mut world = World::new(WorldOptions::default());
        let steel = world
            .materials()
            .find_by_name("鋼(炭素鋼)")
            .expect("standard DB has steel");
        let mut floor_desc = RigidBodyDesc::dynamic(
            Shape::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
            steel,
        );
        floor_desc.body_type = BodyType::Static;
        world.create_body(floor_desc);

        let mut box_desc = RigidBodyDesc::dynamic(
            Shape::Box {
                half_extents: Vec3::new(0.5, 0.5, 0.5),
            },
            steel,
        );
        box_desc.transform = Transform {
            position: Vec3::new(0.0, 2.0, 0.0),
            rotation: Quat::IDENTITY,
        };
        let idx = world.create_body(box_desc);

        // Let it fall, land, and go to sleep (SLEEP_TIME_THRESHOLD=0.5s well
        // within this many steps at dt=1/120s).
        for _ in 0..600 {
            world.step();
        }
        let settled_y = world.body_position(idx).unwrap().y;
        assert!(
            (settled_y - 0.5).abs() < 0.05,
            "box should have settled near half_extent=0.5, got {settled_y}"
        );

        // Enlarge in place: the new half_extent=1.5 now interpenetrates the
        // floor deeply (old rest position only clears half_extent=0.5).
        world.set_body_shape(
            idx,
            Shape::Box {
                half_extents: Vec3::new(1.5, 1.5, 1.5),
            },
        );

        for _ in 0..600 {
            world.step();
        }
        let resettled_y = world.body_position(idx).unwrap().y;
        assert!(
            (resettled_y - 1.5).abs() < 0.05,
            "after enlarging in place, the body must wake up and resettle near the \
             new half_extent=1.5 (not stay frozen at the old half_extent=0.5 rest \
             height), got {resettled_y}"
        );
    }

    /// 拘束オーバーレイ(振り子スポーン)向けの検証: ワールド固定点へDistanceJointで
    /// つないだ球が、鉛直から水平にずらして開始すると重力により振り子運動
    /// (固定点からの距離をほぼ一定に保ちながら往復)することと、
    /// `distance_joint_anchor_points`が固定点側は常に一定・可動体側は実際の球の
    /// 現在位置に一致し続けることを確認する。
    #[test]
    fn distance_joint_to_world_point_makes_a_pendulum_that_swings_at_constant_radius() {
        let mut world = World::new(WorldOptions::default());
        let steel = world
            .materials()
            .find_by_name("鋼(炭素鋼)")
            .expect("standard DB has steel");
        let mut desc = RigidBodyDesc::dynamic(Shape::Sphere { radius: 0.2 }, steel);
        let pivot = Vec3::new(0.0, 5.0, 0.0);
        let arm_length = 2.0;
        // Start displaced 90 degrees from vertical (horizontal arm) so gravity
        // actually drives a swing rather than leaving it at a trivial equilibrium.
        desc.transform.position = pivot + Vec3::new(arm_length, 0.0, 0.0);
        let bob = world.create_body(desc);
        let joint_index =
            world.add_distance_joint_to_world_point(bob, Vec3::ZERO, pivot, arm_length);

        let (anchor_a0, anchor_b0) = world.distance_joint_anchor_points(joint_index).unwrap();
        assert_eq!(anchor_a0, world.body_position(bob).unwrap());
        assert_eq!(anchor_b0, pivot);

        let mut max_radius_error: f64 = 0.0;
        for _ in 0..300 {
            world.step();
            let (anchor_a, anchor_b) = world.distance_joint_anchor_points(joint_index).unwrap();
            assert_eq!(
                anchor_a,
                world.body_position(bob).unwrap(),
                "anchor_a must track the swinging body's live position every step"
            );
            assert_eq!(
                anchor_b, pivot,
                "the fixed world-point anchor must never move"
            );
            let radius = (anchor_a - anchor_b).length();
            max_radius_error = max_radius_error.max((radius - arm_length).abs());
        }
        assert!(
            max_radius_error < 0.05,
            "distance joint must keep the bob at ~constant radius {arm_length} from the \
             pivot throughout the swing (max deviation observed: {max_radius_error})"
        );

        // It must actually have swung (moved substantially from the starting
        // horizontal displacement), not stayed frozen in place.
        let final_position = world.body_position(bob).unwrap();
        assert!(
            (final_position - (pivot + Vec3::new(arm_length, 0.0, 0.0))).length() > 0.5,
            "the pendulum must actually swing under gravity, got final position {final_position:?}"
        );
    }
}
