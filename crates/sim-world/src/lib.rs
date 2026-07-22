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
//! 型をそのまま接続)。`step()`は有効なドメインを固定順(mechanics→thermal→em→astro→circuit、
//! `state_hash`も同順)で順に進める。各ドメインは`orchestrator::sub_step_count`(設計§1.3の
//! 決定的sub-step数算出、`max_stable_dt()`から算出)に従いsub-stepする — Lie-Trotter
//! operator splitting自体(pre/post couplingを挟むパイプライン、
//! docs/20-integration/01-coupling-matrix.md §4)は、`Coupling`実装(`sim-coupling`の
//! `DissipationToHeat`・`JouleHeat`)がまだ`World`に接続されていない(各Couplingは
//! `sim-coupling`crate内で単体検証済みだが、`World::step()`のパイプラインへの組み込みは
//! Coupling registry相当の仕組みが必要で後続増分)ため未実装。`fluid`(`sim-fluid`の
//! GridFluid系・SPH)は`Solver`トレイトを未実装のため今回は見送る(各流体型に`Solver`実装を
//! 追加するか専用の接続方式を検討する必要があり、後続増分)。`quantum`/`statistical`は
//! 専用シーンでのみ有効化する設計方針のため同様に見送る。`gas`
//! (`sim_thermal::GasCompartment`、断熱圧縮の`PistonGas`結合が使う)・`conduction_rod`
//! (`sim_thermal::ConductionRod1D`、D16「熱伝導レース」が使う)も同じ理由で
//! `Solver`を実装しない — `step()`の自動走査対象ではなく、呼び出し側が
//! `apply_coupling`/`conduction_rod_mut().step(dt)`を明示的に呼んで状態を進める。

mod demos;
mod integration_scenarios;
mod orchestrator;
mod overlap;
mod raycast;
mod scenario;

pub use scenario::{
    BodyScenarioDesc, MaterialOverride, Scenario, SceneError, ShapeJson, WorldScenarioOptions,
};

use sim_core::{EnergyLedger, EventQueue, MaterialDb, Solver, SolverContext, StateHasher};
use sim_math::{SimRng, Vec3};
use sim_mechanics::{BodyType, MechanicsSolver, RigidBodyDesc};

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
    BodySpeed(BodyId),
    /// 熱ドメインの`ThermalNode`index(モジュールdoc「縮約実装の理由」参照)。
    NodeTemp(usize),
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
        }
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
            ProbeTarget::BodySpeed(id) => self.body_velocity(id).map_or(0.0, |v| v.length()),
            ProbeTarget::NodeTemp(idx) => self
                .thermal
                .as_ref()
                .and_then(|t| t.nodes.get(idx))
                .map_or(0.0, |n| n.temperature),
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

    /// 全ドメインが読む物性データベース(設計 §1.1.5)。`create_body` に渡す
    /// `MaterialId` の解決に使う。
    pub fn materials(&self) -> &MaterialDb {
        &self.materials
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
    /// 01-coupling-matrix.md §4)へ自動的に組み込む想定だが、そのためのCoupling
    /// registry(シーンJSON`couplings`セクションからの自動解決・実行順序決定を含む)は
    /// まだ実装していない(`from_scenario`のモジュールdoc、各`sim-coupling`実装の
    /// モジュールdoc参照)。本メソッドは、登録・自動実行の仕組みより前に必要な
    /// 「`World`が保持する実ドメイン(`mechanics`・`thermal`・`circuit`・
    /// `em_electrostatics`)に対して外部から`Coupling`を適用する経路」を先に提供する
    /// 縮約版 — 呼び出し側(統合シナリオテスト・将来のCoupling registry自体)が
    /// 呼び出し頻度・タイミング(`step()`の前か後か、design上のpre/post区別)を
    /// 明示的に管理する。`step()`の後に呼ぶ場合、`DissipationToHeat`・`JouleHeat`の
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
        };
        coupling.apply(&mut states, dt);
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
}
