# ロードマップ — TDD による実装フェーズ(v2)

設計レビュー承認後の実装計画。**テスト駆動開発(TDD)** を採用し、
**全ドメインのテストを先に書き(Red)→ 依存順に実装(Green)→ 結合検証**の 3 段 + レンダリングの
Phase D で構成する。実装順(ドメインの依存順)は v1 から維持する。

## 開発体制の前提

本計画は **AI 主導の一気通貫開発**を前提とする。

- 人月ベースの規模見積り・途中経過の到達性計測は導入しない。到達の確認は開発完了時の
  全テスト緑・全デモ(D1–D43)合格で行う。進行状態の記録は
  [02-feature-checklist.md](02-feature-checklist.md) が唯一の記録である。
- 工程は本計画の現行構成を維持し、複雑化させない。

## 全体構造

```
Phase A (Red)   全ドメインの型/トレイト スケルトン + 全テスト先行記述  → 全テスト Red
Phase B (Green) 依存順にドメイン実装 + 性能3本柱を同時適用             → ドメインテスト Green
Phase C (結合)  ドメイン間結合・全体シナリオ・決定論/保存則/性能 CI      → 結合テスト Green
Phase D         物理正確フルパストレ + リアルタイムプレビュー(全物理後) → レンダリング検証
```

デモ 33+10 本は Phase B の**ドメイン内スモークテスト**として実装中に使い、多ドメインデモは Phase C で検証
([21-verification/03-demo-scenarios.md](../21-verification/03-demo-scenarios.md))。

## Phase 0 — 骨格

- Cargo ワークスペース([00-foundation/05](../00-foundation/05-rust-wasm-platform.md) §2、sim-astro/sim-render 含む)、
  CI(fmt/clippy/test/wasm ビルド/決定論スモーク)、demo の Vite + Three.js 雛形、wasm 境界の疎通。
- **完了条件**: 箱 1 個が落ちる最小 World が cargo test 緑 + ブラウザ表示 + ハッシュ 2 回一致。

## Phase A — テスト先行(Red)

- **全ドメインの型・トレイトのスケルトン**を定義(`Solver`/`Constraint`/`Integrator`/各ドメインの状態型)。
  中身は `todo!()`。コンパイルは通る。
- **全テストを記述**: [21-verification/01-analytic-tests.md](../21-verification/01-analytic-tests.md) の全表
  (力学 M・流体 F・熱 T・電磁 E・量子 Q・統計 S・天体 A・レンダ R)+ 各ドメイン §7 のユニットテスト +
  保存則([21-verification/02-conservation-laws.md](../21-verification/02-conservation-laws.md))+ 決定論テスト。
- **完了条件**: 全テストが記述され、全て Red(未実装)であること。テスト自体のレビュー完了。

## Phase B — 実装(Green)、依存順

依存順に実装し、各ドメインの担当テストを Green にする。**各ドメイン実装時に性能 3 本柱
([00-foundation/06-performance-strategy.md](../00-foundation/06-performance-strategy.md): アルゴリズム上位化・
SIMD/並列化・データ局所性)を同時適用**する(スカラー参照実装 → 最適化版 + 参照一致テスト)。

実装ウェーブ(旧 Phase 1〜5 = 実装順、各文書 §8・結合行列の P 表記に対応):

| ウェーブ | ドメイン/機能 | Green にするテスト |
|---|---|---|
| **math** | 線形代数・場・積分器・PRNG | 数学基盤テスト、収束次数 ◆ |
| **P1** 力学基礎 | 剛体・総当たり衝突・接触ソルバ・摩擦・重力・抗力・浮力・熱ノード・台帳 | M1–M9,M12, F1–F6, T1,T2 |
| **P2** 力学拡充 | SAP/BVH・Box-Box(SAT)・split impulse・スリープ・転がり摩擦 | M6(精度),M10,M11 |
| **P3** 拘束・流体・熱 | ジョイント・XPBD・格子流体・熱伝導網・相変化・気体区画 | M3,M4,M13,M14, F7–F9,F11, T3,T5,T7 |
| **P4** 電磁・光・SPH・車両・ブラウン | 回路・モーター・静場・光学・WCSPH・車両・ランジュバン | E1–E7,E9–E12, F10, S4–S6, T8 |
| **P5** 量子・統計・波動 | シュレディンガー・FDTD・気体分子・イジング・GJK/EPA | Q1–Q6, E8,E13, S1–S3,S7–S9 |
| **Pα** 天体 | N体重力・軌道・再突入・相対論オプトイン・宇宙機 | A1–A10 |

- ウェーブ内は「math → ソルバ単体(ネイティブテスト)→ World 統合 → wasm/デモ」の縦切り。
- 依存: math→P1→P2→P3 は直列。P4 の回路/光学/SPH/ブラウンは相互独立(並行可)。P5・Pα の各項も独立。
- **完了条件**: 各ウェーブの担当テスト全緑 + 収束次数 ◆ + 対応スモークデモ動作。

## Phase C — 結合・全体検証

- **ドメイン間結合**([20-integration/01-coupling-matrix.md](../20-integration/01-coupling-matrix.md))の実装と検算
  (保存量の対記帳、排他結合の validator)。
- **多ドメイン統合シナリオ**(ブレーキ発熱・手回し発電・氷と飲み物・断熱圧縮・再突入)。
- **CI ゲート**: 決定論(全シーン 2 回実行一致 + スナップショット再開一致 = 階層 1、
  スレッド数変更・wasm⇔ネイティブの許容誤差照合 = 階層 2)、
  保存則(residual 閾値 [21-verification/02](../21-verification/02-conservation-laws.md) §2)、性能ベンチ回帰。
- **完了条件**: 全結合テスト緑、全デモ(D1–D39)の合格基準達成、性能予算監視が緑。

## Phase D — レンダリング(全物理完了後)

- **物理正確フルパストレ**([17-rendering/02-path-tracing.md](../17-rendering/02-path-tracing.md))+
  リアルタイムプレビュー(Three.js)。BVH → BSDF → NEE → 分光・屈折・コースティクス →
  参加媒質(大気・水・煙)→ カメラ。CPU 実装先行、GPU 後([00-foundation/06](../00-foundation/06-performance-strategy.md) §3)。
- **完了条件**: R1–R7 + デモ D40–D43。写実性レベル(現状「物理正確フルパストレ」で確定。
  さらなる詳細化が要るときはユーザーに確認)。

## Phase E+ — 拡張(優先度はレビューで)

歩行制御・飛行機・熱機関(オットー)・FLIP 流体・DFSPH・音響・地形(高さ場)・
FMM(大規模 N 体)・GPU 本格化(WebGPU)・破壊/塑性・速度依存反発係数・地磁気の地理分布。

## 横断ルール(Definition of Done)

各フェーズ/ウェーブの完了条件:
1. 対象テスト全緑 + 収束次数 ◆。
2. 決定論 CI(同一シード一致・スナップショット再開一致 = ビット一致の階層 1。
   スレッド数変更・wasm⇔ネイティブは許容誤差照合の階層 2 —
   [20-integration/02-determinism-replay.md](../20-integration/02-determinism-replay.md) §5)。
3. 保存則 CI(residual 閾値内)。
4. 性能予算監視([00-foundation/05](../00-foundation/05-rust-wasm-platform.md) §5)+ ベンチ回帰なし。
5. 実装が設計から乖離したら**設計書を先に改訂**する。

## リスクと備え

| リスク | 備え |
|---|---|
| Phase A のテスト量が大きい | ドメインごとにテストを分割記述、スケルトンと並行。テスト自体もレビュー対象 |
| WASM/性能不足(流体・SPH・パストレ) | 規模を落とす既定 + GPU(CPU 優先で隔離設計、[00-foundation/06](../00-foundation/06-performance-strategy.md) §3) |
| 接触ソルバ安定性の調整長期化 | Box2D 系実証定数から開始、デモ D4 を常時回帰 |
| クロスプラットフォーム決定論の穴 | 「同一バイナリ」保証で出発、GPU は許容誤差内([20-integration/02](../20-integration/02-determinism-replay.md) §5) |
| 天体スケールの座標精度 | floating origin フレーム([00-foundation/02](../00-foundation/02-scale-ladder.md) §2.2) |
| スコープ肥大 | 完了条件を数値で固定。新要望はまず本ロードマップの改訂 PR にする |
