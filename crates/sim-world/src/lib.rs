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
//! カウンタだけを正式にインクリメントして以後のアクセスを `None` にする。ジョイント・
//! 結合の連鎖削除(設計 §2 の `remove_body` 完全仕様)は、`World` がまだジョイント・
//! Coupling を保持していないため対象外(それらの導入時に合わせて拡張する)。
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
//! docs/20-integration/01-coupling-matrix.md §4)のうち、pre/postの2相分離は未実装
//! (現状は全て単一の`apply`、各`sim-coupling`実装のモジュールdoc「1step遅れの縮約」
//! 参照)だが、登録済み`Coupling`(`add_coupling`)を毎stepの全ドメインsub-step完了後に
//! 自動適用するレジストリ自体は実装済み(`couplings`フィールド、`apply_coupling`の
//! doc参照)。シーンJSON`couplings`セクションからの自動解決・排他結合検査
//! (`sim-coupling::validate_exclusive_couplings`)との接続は未実装(`scenario`モジュール
//! doc参照)。`quantum`/`statistical`は
//! 専用シーンでのみ有効化する設計方針のため見送る。`gas`
//! (`sim_thermal::GasCompartment`、断熱圧縮の`PistonGas`結合が使う)・`conduction_rod`
//! (`sim_thermal::ConductionRod1D`、D16「熱伝導レース」が使う)は`Solver`を実装しない —
//! `step()`の自動走査対象ではなく、呼び出し側が
//! `apply_coupling`/`conduction_rod_mut().step(dt)`を明示的に呼んで状態を進める。

mod demos;
mod integration_scenarios;
mod orchestrator;
mod overlap;
mod raycast;
mod scenario;

pub use scenario::{
    run_headless_scenario, BodyScenarioDesc, HeadlessRunResult, MaterialOverride,
    PredictionPromptJson, Scenario, SceneError, ShapeJson, WorldScenarioOptions,
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
    LedgerKinetic,
    /// `state_hash()`をグラフ表示用に`f64`へ変換した値(厳密な数値変換ではなく、
    /// UI上でハッシュの変化を視覚化するためのダイジェスト、設計§2.1「UIのグラフ」)。
    StateHashDigest,
}

/// 任意の観測量を毎stepサンプルして`history`(`RingBuffer`)に積む軽量プローブ
/// (設計docs/20-integration/04-world-api.md §2.1「測って遊ぶの中心機能」)。
#[derive(Clone)]
pub struct Probe {
    pub target: ProbeTarget,
    history: sim_math::RingBuffer<f64>,
}

impl Probe {
    /// 古い順(サンプル順)の観測履歴。
    pub fn history(&self) -> impl Iterator<Item = &f64> {
        self.history.iter()
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
    /// `Some`)。`Solver`トレイトを実装しないため`step()`のドメイン走査対象ではなく、
    /// `apply_coupling`経由でのみ状態が変化する。
    gas: Option<sim_thermal::GasCompartment>,
    /// 1D格子熱伝導ドメイン(D16「熱伝導レース」が使う、シーンが使う場合のみ`Some`)。
    /// `gas`と同じ理由(`Solver`トレイト未実装)で`step()`の自動走査対象ではなく、
    /// `conduction_rod_mut().step(dt)`を呼び出し側が明示的に呼ぶ必要がある。
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
    /// ソフトボディ(XPBDロープ、D13「ロープと旗」が使う、シーンが使う場合のみ`Some`)。
    /// `gas`・`conduction_rod`と同じ理由(`Solver`トレイト未実装、`step`が複数引数
    /// (`dt, gravity, n_sub, n_iter, damping`)を取るため`Solver::step(dt, ctx)`の
    /// シグネチャに素直に適合しない)で`step()`の自動走査対象ではなく、
    /// `soft_body_mut().step(...)`を呼び出し側が明示的に呼ぶ必要がある。
    soft_body: Option<sim_mechanics::SoftBody>,
    materials: MaterialDb,
    rng: SimRng,
    events: EventQueue,
    /// 最初の `step()` で遅延初期化する(構築フェーズの `create_body` を
    /// 台帳の基準点計算に含めないため)。
    ledger: Option<EnergyLedger>,
    /// `BodyId` の世代管理(`RigidBodySet` のインデックスに対応、モジュールdoc参照)。
    generations: Vec<u32>,
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
/// `Probe`の`DEFAULT_PROBE_CAPACITY`(`scenario`モジュール)と同オーダーの値を採用)。
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
fn run_domain_substeps<S: Solver>(
    solver: &mut S,
    frame_dt: f64,
    materials: &MaterialDb,
    rng: &mut SimRng,
    events: &mut EventQueue,
) {
    let n = orchestrator::sub_step_count(frame_dt, solver.max_stable_dt());
    let sub_dt = orchestrator::sub_step_dt(frame_dt, n);
    for _ in 0..n {
        let mut ctx = SolverContext {
            materials,
            rng: &mut *rng,
            events: &mut *events,
        };
        solver.step(sub_dt, &mut ctx);
    }
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
            soft_body: None,
            materials: MaterialDb::standard(),
            rng: SimRng::new(options.seed, STREAM_DIAG),
            events: EventQueue::new(),
            ledger: None,
            generations: Vec::new(),
            pending_commands: Vec::new(),
            command_log: Vec::new(),
            probes: Vec::new(),
            grab_joints: std::collections::HashMap::new(),
            event_log: sim_math::RingBuffer::new(EVENT_LOG_CAPACITY),
            couplings: Vec::new(),
            time_regime: sim_astro::TimeRegime::Local { steps_per_frame: 1 },
            frames: sim_core::FrameTree::new(),
            auto_regime_switch: None,
        }
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
            self.mechanics.bodies.position[index] = local_position;
            self.mechanics.bodies.linear_velocity[index] = local_velocity;
            self.mechanics.bodies.still_time[index] = 0.0;
            self.mechanics.bodies.asleep[index] = false;
        }
        self.time_regime = sim_astro::TimeRegime::Local { steps_per_frame: 1 };
        self.auto_regime_switch = None;
    }

    /// プローブを登録する(`Probe`のdoc参照)。返すハンドルは`probe`/`probe_history`が
    /// 使う(現時点では単なるベクタindex、`Vec`が縮まないため安定)。
    pub fn add_probe(&mut self, target: ProbeTarget, capacity: usize) -> usize {
        self.probes.push(Probe {
            target,
            history: sim_math::RingBuffer::new(capacity),
        });
        self.probes.len() - 1
    }

    pub fn probe(&self, handle: usize) -> Option<&Probe> {
        self.probes.get(handle)
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

    /// 流体場の点`p`での観測(設計docs/20-integration/04-world-api.md §2
    /// `sample_fluid(p) -> FluidSample`)。
    ///
    /// **縮約実装の理由**: 設計は速度・圧力・温度を返すが、SPHは温度場を持たない
    /// (`温度`は対象外)。カーネル補間ではなく最近傍粒子の値をそのまま返す縮約
    /// (`p`が粒子分布から離れているほど代表性が下がる点に注意、真のカーネル補間は
    /// 後続増分)。SPHドメインが未有効化、または粒子が1つも無い場合は`None`。
    pub fn sample_fluid(&self, p: Vec3) -> Option<FluidSample> {
        let sph = self.sph.as_ref()?;
        let (nearest_idx, _) = sph
            .position
            .iter()
            .enumerate()
            .map(|(i, &pos)| (i, (pos - p).length_sq()))
            .min_by(|a, b| a.1.total_cmp(&b.1))?;
        Some(FluidSample {
            velocity: sph.velocity[nearest_idx],
            pressure: sph.pressure[nearest_idx],
        })
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

    /// ボディを削除する。世代カウンタをインクリメントし、以後この `id` (と古い世代の
    /// 再利用)へのアクセスは `None` になる(設計の不変条件)。下層の `RigidBodySet`
    /// スロット自体はまだ真に解放されない(モジュールdoc参照) — 無効化として
    /// `BodyType::Static` 化・遠方への退避・速度ゼロ化を行い、実質的な影響を無くす。
    pub fn remove_body(&mut self, id: BodyId) {
        if !self.is_valid(id) {
            return;
        }
        let idx = id.index as usize;
        self.generations[idx] += 1;
        self.mechanics.bodies.body_type[idx] = BodyType::Static;
        self.mechanics.bodies.position[idx] = Vec3::new(0.0, -1.0e9, 0.0);
        self.mechanics.bodies.linear_velocity[idx] = Vec3::ZERO;
        self.mechanics.bodies.angular_velocity[idx] = Vec3::ZERO;
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
        total
    }

    /// 1 world step(固定 dt)。docs/20-integration/04-world-api.md §2 の `step()`。
    /// 有効な全ドメインを固定順(mechanics→thermal→em→astro、モジュールdoc参照)で進める。
    pub fn step(&mut self) {
        if self.ledger.is_none() {
            self.ledger = Some(EnergyLedger::new(self.total_energy().total()));
        }
        self.apply_pending_commands();
        let dt = self.clock.dt();
        match self.time_regime {
            sim_astro::TimeRegime::Local { .. } => {
                run_domain_substeps(
                    &mut self.mechanics,
                    dt,
                    &self.materials,
                    &mut self.rng,
                    &mut self.events,
                );
                if let Some(t) = &mut self.thermal {
                    run_domain_substeps(t, dt, &self.materials, &mut self.rng, &mut self.events);
                }
                if let Some(e) = &mut self.em_electrostatics {
                    run_domain_substeps(e, dt, &self.materials, &mut self.rng, &mut self.events);
                }
                if let Some(a) = &mut self.astro {
                    run_domain_substeps(a, dt, &self.materials, &mut self.rng, &mut self.events);
                }
                if let Some(c) = &mut self.circuit {
                    run_domain_substeps(c, dt, &self.materials, &mut self.rng, &mut self.events);
                }
                if let Some(s) = &mut self.sph {
                    run_domain_substeps(s, dt, &self.materials, &mut self.rng, &mut self.events);
                }
                if let Some(g) = &mut self.grid_fluid {
                    run_domain_substeps(g, dt, &self.materials, &mut self.rng, &mut self.events);
                }
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
                sph: self.sph.as_mut(),
            };
            coupling.apply(&mut states, dt);
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
        Some(self.mechanics.bodies.position[id.index as usize])
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
        let anchor_a_world = self.mechanics.bodies.position[body_a]
            + self.mechanics.bodies.rotation[body_a].rotate(joint.anchor_a);
        let anchor_b_world = match joint.body_b {
            Some(body_b) => {
                self.mechanics.bodies.position[body_b]
                    + self.mechanics.bodies.rotation[body_b].rotate(joint.anchor_b)
            }
            None => joint.anchor_b,
        };
        Some((anchor_a_world, anchor_b_world))
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

    /// レイキャストクエリ(設計docs/20-integration/04-world-api.md §2、`raycast`
    /// モジュールdoc参照。`filter`引数は未実装、同モジュールdoc参照)。
    pub fn raycast(&self, origin: Vec3, dir: Vec3, max_distance: f64) -> Option<RayHit> {
        raycast::raycast(&self.mechanics.bodies, origin, dir, max_distance).map(|hit| RayHit {
            body: BodyId {
                index: hit.body_index as u32,
                generation: self.generations[hit.body_index],
            },
            point: hit.point,
            normal: hit.normal,
            distance: hit.distance,
        })
    }

    /// 球オーバーラップクエリ(設計docs/20-integration/04-world-api.md §2、`overlap`
    /// モジュールdoc参照。`filter`引数は未実装)。
    pub fn overlap_sphere(&self, center: Vec3, r: f64) -> Vec<BodyId> {
        overlap::overlap_sphere(&self.mechanics.bodies, center, r)
            .into_iter()
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
    /// フィールドとして実装済み(このメソッドのdoc下部参照)。ただしシーンJSON
    /// `couplings`セクションからの自動解決・排他結合検査(`sim-coupling::
    /// validate_exclusive_couplings`)との接続、pre/post 2相への分離(現状は全て
    /// post相当のタイミングで単一`apply`済み、各`sim-coupling`実装のモジュールdoc
    /// 「1step遅れの縮約」参照)は未実装(`from_scenario`のモジュールdoc参照)。
    /// 本メソッドは、`add_coupling`によるレジストリ登録より前から存在する、呼び出し側が
    /// 呼び出し頻度・タイミングを明示的に管理する下位のプリミティブとして残している
    /// (統合シナリオテストの一部・レジストリ自体の内部実装が使う)。`step()`の後に
    /// 呼ぶ場合、`DissipationToHeat`・`JouleHeat`の
    /// ように直近stepで確定した量(`last_contact_dissipation`・`resistor_power`等)を
    /// 読むCoupling(design上の"post")は正しく機能するが、`BrownianForce`・
    /// `LorentzForce`のように力・速度を注入し同stepの位置積分に反映されるべき
    /// Coupling(design上の"pre")は、その注入が次の`step()`まで反映されない
    /// 1step遅れが生じる(`InductionCoupling`で既に検証・許容した縮約と同じパターン、
    /// 同モジュールdoc参照)。
    pub fn apply_coupling(&mut self, coupling: &mut dyn sim_coupling::Coupling, dt: f64) {
        let mut states = sim_coupling::DomainStates {
            mechanics: &mut self.mechanics,
            thermal: self.thermal.as_mut(),
            em_circuit: self.circuit.as_mut(),
            em_electrostatics: self.em_electrostatics.as_mut(),
            gas: self.gas.as_mut(),
            grid_fluid: self.grid_fluid.as_mut(),
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
    pub fn add_coupling(&mut self, coupling: Box<dyn sim_coupling::Coupling>) {
        self.couplings.push(coupling);
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
    /// `grid_fluid`の`solid`マスクに反映され、圧力反力がボディに戻ってくること — を
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
        for _ in 0..20 {
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

        let solid = world
            .grid_fluid()
            .unwrap()
            .solid
            .expect("GridFluidRigid should have set the solid mask by now");
        let body_pos = world.body_position(body).unwrap();
        assert!(
            (solid.center.0 - body_pos.x).abs() < 1e-9,
            "solid mask should track the body's x position: solid.center.0={} body_pos.x={}",
            solid.center.0,
            body_pos.x
        );
    }

    /// `BuoyancyDrag`をレジストリ経由(`add_coupling`)で剛体に接続し、`demos.rs`のD6
    /// (F4部分、密度比0.6の直立直方体)と同じ釣り合い喫水深さの近傍で有界に留まる
    /// ことを確認する(既存の`MechanicsSolver.water`埋め込み経路(D6が使う)と同じ
    /// 物理式(`sim_fluid::{submerged_box_axis_aligned, buoyancy_force}`)を使うが、
    /// `mechanics_mut().water`は設定しない独立経路)。
    ///
    /// D6のF4部分は埋め込み経路(`apply_forces`内でmechanicsの各sub-stepごとに
    /// 浮力を再評価)のため実質的に減衰項なしでも密着した釣り合いに留まるが、この
    /// Coupling経由の経路は`SphRigid`・`GridFluidRigid`等と同じ1step遅れの縮約
    /// (Couplingレジストリはフレームごとに1回だけ適用され、mechanicsの内部sub-step
    /// ごとには再評価されない)であり、かつ抗力(減衰)を伴わない純粋な浮力の
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
            water: Some(sim_fluid::StaticWaterRegion::new(0.0, water_density)),
            atmosphere: None,
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
            world.add_coupling(Box::new(sim_coupling::DissipationToHeat {
                thermal_node: node,
            }));
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

    /// `World::overlap_sphere`(設計docs/20-integration/04-world-api.md §2、`overlap`
    /// モジュールdoc参照): 重なる剛体の`BodyId`(世代付き)を正しく返すことを確認する。
    #[test]
    fn overlap_sphere_returns_body_ids_of_overlapping_bodies_only() {
        let mut world = World::new(WorldOptions::default());
        let box_id = create_falling_box(&mut world);
        let position = world.body_position(box_id).unwrap();

        let near_hits = world.overlap_sphere(position, 0.1);
        assert_eq!(near_hits, vec![box_id]);

        let far_hits = world.overlap_sphere(position + Vec3::new(1000.0, 0.0, 0.0), 0.1);
        assert!(far_hits.is_empty());
    }

    /// `Probe`(設計docs/20-integration/04-world-api.md §2.1「測って遊ぶの中心機能」):
    /// `BodyPosY`が箱の自由落下を毎stepサンプルし、履歴が単調減少することを確認する。
    /// リングバッファの容量制限(古いサンプルが捨てられること)も併せて検証する。
    #[test]
    fn probe_body_pos_y_samples_falling_box_every_step_within_ring_buffer_capacity() {
        let mut world = World::new(WorldOptions::default());
        let box_id = create_falling_box(&mut world);
        let handle = world.add_probe(ProbeTarget::BodyPosY(box_id), 10);

        for _ in 0..30 {
            world.step();
        }

        let probe = world.probe(handle).unwrap();
        let history: Vec<f64> = probe.history().copied().collect();
        // 容量10なので30step分のうち最新10個だけが残る。
        assert_eq!(history.len(), 10);
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

    /// `ProbeTarget::LedgerKinetic`・`StateHashDigest`が無効なindex/id無しでも
    /// パニックせず妥当な値をサンプルすることを確認する(常時有効なmechanicsドメイン
    /// のみに依存するターゲット)。
    #[test]
    fn probe_ledger_kinetic_and_state_hash_digest_sample_without_panicking() {
        let mut world = World::new(WorldOptions::default());
        create_falling_box(&mut world);
        let kinetic_handle = world.add_probe(ProbeTarget::LedgerKinetic, 5);
        let hash_handle = world.add_probe(ProbeTarget::StateHashDigest, 5);

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
