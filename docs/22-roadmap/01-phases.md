# ロードマップ — 実装フェーズ分割と完了条件

設計レビュー承認後の実装計画。各フェーズは「動くデモ + 通るテスト」で閉じる
(縦切り: 全レイヤを薄く通す)。デモ番号は [21-verification/03](../21-verification/03-demo-scenarios.md)、
テスト番号は [21-verification/01](../21-verification/01-analytic-tests.md)。

## Phase 0 — 骨格(1 スプリント想定)

- Cargo ワークスペース([00-foundation/05](../00-foundation/05-rust-wasm-platform.md) §2)、CI
  (fmt/clippy/test/wasm ビルド)、demo の Vite + Three.js 雛形、wasm 境界の疎通
  (箱 1 個が落ちるだけの World)。
- **完了条件**: `cargo test` 緑 / ブラウザで箱が落ちる / 決定論スモーク(ハッシュ 2 回一致)。

## Phase 1 — 剛体力学コア + 地球環境の基本

- sim-math(線形代数・PRNG)、剛体(球/箱/平面)、総当たり broadphase、
  接触ソルバ(反発・摩擦・Baumgarte・warm start)、重力・抗力・静的水域浮力、
  ThermalNode(冷却・散逸記帳)、EnergyLedger、Scenario/Replay、Probe。
- **完了条件**: テスト M1–M9, M12, F1–F6, T1, T2 + 保存則 CI + デモ D1–D10 合格。

## Phase 2 — 力学の拡充

- SAP broadphase、Box-Box(SAT)、カプセル・複合形状、split impulse、スリープ、
  転がり摩擦、慣性の一般化(Jacobi 固有分解)。
- **完了条件**: M6 が split impulse 精度で、M10/M11、スタック 8 段、性能予算(500 体 3 ms)。

## Phase 3 — 拘束・流体格子・熱ネットワーク

- ジョイント一式(Ball/Hinge/Distance/Fixed + limit/motor + ソフト拘束 + ブロックソルバ)、
  ラグドール、XPBD(ロープ・布)、格子流体(移流・投影・Boussinesq・剛体結合)、
  熱伝導ネットワーク・温度場・相変化(エンタルピー法)、GasCompartment + ピストン結合。
- **完了条件**: M3/M4/M13/M14, F7–F9, F11, T3, T5, T7 + デモ D11–D18。

## Phase 4 — 電磁気・光学・SPH・乗り物・ブラウン

- 回路(MNA + 素子一式)、モーター/発電結合、静場・磁気力・誘導、
  幾何光学(レイトレーサ・分散・黒体)、WCSPH(境界粒子・剛体双方向)、
  車両(WheelJoint + 簡易タイヤ)、船、ランジュバン(BAOAB)、蒸発冷却。
- **完了条件**: E1–E7, E9–E12, F10, S4–S6, T8 + デモ D19–D26。

## Phase 5 — 量子・統計・波動・高度化

- シュレディンガーソルバ(1D/2D + FFT)、FDTD(2D + PML)、気体分子デモ(熱壁・測定)、
  イジング(メトロポリス)、GJK/EPA・CCD、渦電流・温度依存物性、「なぜ?」ページ群。
- **完了条件**: Q1–Q6, E8/E13, S1–S3, S7–S9 + デモ D27–D33。

## Phase 6+ — 拡張(優先度はレビューで)

歩行制御・飛行機・熱機関(オットー)・FLIP 流体・音響・地形(高さ場)・並列化(rayon/wasm)・
WebGPU 流体・速度依存反発係数・破壊/塑性。

## 横断ルール

- **各フェーズの Definition of Done**: (1) 対象テスト全緑 + 収束次数 ◆、
  (2) 決定論 CI(全シーン 2 回実行一致 + スナップショット再開一致)、
  (3) 保存則 CI(residual 閾値 [21-verification/02](../21-verification/02-conservation-laws.md) §2)、
  (4) 性能予算([00-foundation/05](../00-foundation/05-rust-wasm-platform.md) §5)、
  (5) 設計書との突合(実装が設計から乖離したら**設計書を先に改訂**)。
- フェーズ内の実装順は「math → ソルバ単体(ネイティブテスト)→ World 統合 → wasm/デモ」の
  縦切りを守る。
- 依存: P1→P2→P3 は直列。P4 の回路/光学/SPH/ブラウンは相互独立(並行可)。P5 の各項も独立。

## リスクと備え

| リスク | 備え |
|---|---|
| WASM 性能不足(流体・SPH) | 規模を落とす設定を既定に。並列化(P6)を前倒し検討 |
| 接触ソルバの安定性調整の長期化 | Box2D 系の実証済み定数から開始([10-mechanics/03](../10-mechanics/03-contact-solver.md) §9)。デモ D4 を常時回帰 |
| クロスプラットフォーム決定論の穴 | 保証範囲を「同一バイナリ」に限定して出発([20-integration/02](../20-integration/02-determinism-replay.md) §5) |
| スコープ肥大(物理全域ゆえ) | フェーズ完了条件を数値で固定。新要望はまず本ロードマップの改訂 PR にする |
