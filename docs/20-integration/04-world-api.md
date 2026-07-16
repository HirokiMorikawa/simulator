# 横断 04. World API — 公開インターフェースとシーン記述

crate: `sim-world`。エンティティ層・デモ UI・テストが使う唯一の入口。
WASM 境界([00-foundation/05](../00-foundation/05-rust-wasm-platform.md) §3)はこの API の薄い写像。

## 1. 設計原則

- **読み取りは自由、書き込みは規律**: 状態クエリは常時可。状態変更は
  (a) シーン構築時の create 系、(b) 実行中の Command(次ステップ先頭で適用・記録)の 2 経路のみ。
  step 中のコールバックからの変更は禁止(コマンド化)—決定論とリプレイの要
  ([02-determinism-replay.md](02-determinism-replay.md) §4.2)。
- **ID は世代付き**: 削除済み ID へのアクセスは `None`(パニックしない)。

## 2. API(Rust シグネチャ)

```rust
impl World {
    // ── 構築 ─────────────────────────────────────────
    pub fn new(options: WorldOptions) -> World;
    pub fn from_scenario(s: &Scenario) -> Result<World, SceneError>;  // validator 込み
    pub fn create_body(&mut self, desc: RigidBodyDesc) -> BodyId;
    pub fn create_joint(&mut self, desc: JointDesc) -> JointId;
    pub fn create_soft_body(&mut self, desc: SoftBodyDesc) -> SoftBodyId;
    pub fn create_circuit(&mut self, desc: CircuitDesc) -> CircuitId;
    pub fn add_fluid_region(&mut self, desc: FluidDesc) -> FluidId;
    pub fn add_coupling(&mut self, desc: CouplingDesc) -> CouplingId;
    pub fn remove_body(&mut self, id: BodyId);   // 依存 (ジョイント・結合) も連鎖削除+イベント

    // ── 時間 ─────────────────────────────────────────
    pub fn step(&mut self);                       // 1 world step (固定 dt)
    pub fn time(&self) -> f64;
    pub fn step_count(&self) -> u64;

    // ── コマンド (実行中の介入。記録されリプレイ可能) ──
    pub fn push_command(&mut self, cmd: Command);
    // Command 例: ApplyForce{body, force, point}, SetMotorTarget{joint, velocity},
    //   SetSwitch{circuit, element, closed}, SetHeatSource{node, watts},
    //   Grab{body, anchor}/MoveGrab/Release (マウスでつかむ = ソフト拘束)

    // ── クエリ (読み取り) ─────────────────────────────
    pub fn body(&self, id: BodyId) -> Option<BodyView>;          // 位置・速度・温度…
    pub fn joint(&self, id: JointId) -> Option<JointView>;       // 角度・角速度・拘束力
    pub fn raycast(&self, origin: Vec3, dir: Vec3, max: f64, filter: Filter) -> Option<RayHit>;
    pub fn overlap_sphere(&self, center: Vec3, r: f64, filter: Filter) -> Vec<BodyId>;
    pub fn sample_fluid(&self, p: Vec3) -> FluidSample;          // 速度・圧力・温度
    pub fn circuit_probe(&self, id: CircuitId, node: NodeId) -> f64;  // 電位 (電圧計)

    // ── 観測・検証 ────────────────────────────────────
    pub fn energy_ledger(&self) -> &EnergyLedger;                // [12-thermal/01] §4
    pub fn total_momentum(&self) -> (Vec3, Vec3);                // P, L
    pub fn entropy_estimate(&self) -> f64;
    pub fn state_hash(&self) -> u64;
    pub fn diagnostics(&self) -> &SolverDiagnostics;             // 発散・CFL違反・警告
    pub fn approximations(&self) -> Vec<ApproximationNote>;      // このシーンの近似一覧 (UI表示)

    // ── イベント購読 ──────────────────────────────────
    pub fn subscribe(&mut self, kind: EventKind, sub: SubscriberId) -> Subscription;
    pub fn drain_events(&mut self, sub: SubscriberId) -> Vec<Event>;
    // EventKind: ContactStarted/Ended, JointBroken, PhaseChanged, Discharge,
    //   FuseBlown, SolverDiverged, ...

    // ── 記録・再現 ────────────────────────────────────
    pub fn snapshot(&self) -> Snapshot;                          // 全状態 (再開可能)
    pub fn restore(s: &Snapshot) -> World;
    pub fn start_replay_recording(&mut self) -> ReplayRecorder;
}
```

### 2.1 観測ストリーム(グラフ用)

```rust
/// 任意の観測量を毎ステップ記録する軽量プローブ (UI のグラフ・CSV エクスポート)
pub struct Probe { pub target: ProbeTarget, pub history: RingBuffer<f64> }
// ProbeTarget: BodyPosY(id), Bodyspeed(id), NodeTemp(id), CircuitCurrent(..),
//   LedgerKinetic, StateHashDigest, ...
```

「測って遊ぶ」の中心機能。全デモ UI はこの Probe を使う(専用コードでの直接読み出しを避け、
観測手段自体を汎用化する)。

## 3. シーン記述(JSON)

```jsonc
{
  "name": "buoyancy-basic",
  "seed": 42,
  "world": { "gravity": 9.80665, "ambient_temperature": 293.15, "dt": 0.008333333 },
  "materials": [ { "extends": "wood", "name": "light-wood", "density": 400.0 } ],
  "bodies": [
    { "shape": { "box": { "half": [0.1, 0.1, 0.1] } }, "material": "light-wood",
      "position": [0, 2, 0], "name": "crate" },
    { "shape": { "plane": { "normal": [0,1,0], "d": 0 } }, "type": "static", "material": "concrete" }
  ],
  "fluids": [ { "static_water": { "aabb": [[-5,-2,-5],[5,0.5,5]], "temperature": 288 } } ],
  "couplings": [ "buoyancy_drag", "dissipation_to_heat", "convection" ],
  "probes": [ { "body_pos_y": "crate" }, { "ledger": "thermal" } ]
}
```

- validator: 参照整合(名前解決)、排他結合検査([01-coupling-matrix.md](01-coupling-matrix.md) §2.2)、
  単位・範囲の妥当性(負の質量等)をロード時に拒否し、位置つきエラーを返す。
- `extends` による材料派生 = 「密度だけ変えた木」など検証遊びの基本操作。

## 4. バージョニング

- シーン JSON に `format_version`。エンジンは後方互換ローダを維持(検証済みシナリオ資産を
  腐らせない)。
- Replay はエンジンバージョン厳密一致を要求(数値互換はバージョン間で保証しないため。
  [02-determinism-replay.md](02-determinism-replay.md) §4.2)。

## 5. 検証

- API 経由のみで全デモシナリオ([21-verification/03](../21-verification/03-demo-scenarios.md))が
  構築できること(特権アクセスの不在の証明)。
- validator の拒否ケーステスト(二重浮力・壊れた参照・NaN 入力)。
- Command リプレイの一致([02](02-determinism-replay.md) §6)。
- Probe のオーバーヘッド < 1%(性能予算)。
