# 機能群一覧・実装チェック表 — 中断・再開のための進行記録

目的: 本プロジェクトは AI 主導の一気通貫開発で進めるため、トークン制限・セッション中断で
作業が途切れた際に**再開地点を機械的に特定できる**必要がある。本表はプロジェクト全体の
機能群を単一のチェック表として列挙し、進行状態の**唯一の記録**とする。

## 運用ルール

1. 項目が完了するたび `[ ]` を `[x]` にし、**その作業と同じコミットに含める**
   (表の更新が遅れると再開地点がずれる)。
2. [現在地](#現在地) 節は作業の開始時・中断時に更新する。「作業中」は常に 1 項目だけにする。
3. **再開手順**: (1) 現在地を読む → (2) 作業中項目の実状態をコード・テスト実行で確認する
   (チェック表は自己申告であり、実状態が正) → (3) 未チェックの最初の項目から続行する。
4. 項目の増減は設計書改訂と同じ扱いとする(実装が本表と乖離したら本表を先に直す)。
5. 各項目の内容定義は括弧内の参照文書が正。本表は索引であり仕様を再定義しない。

## 現在地

- **フェーズ**: 実装(Phase 0 完了。math ウェーブは状態非依存の全項目が Green。Phase A 着手済み:
  `sim-core` の `Solver`トレイト・`MaterialDb`・`EventQueue`・`FrameId`。`sim-mechanics` の
  `RigidBodySet`/`Shape`(Sphere/Box/Plane)/`MechanicsSolver`/衝突検出(broadphase総当たり+
  Sphere-Sphere/Sphere-Plane/Box-Plane/Sphere-Box narrowphase)/sequential impulses 接触ソルバ
  (反発+Baumgarte+箱近似クーロン摩擦)を実装し M1・M5–M9 を Green 化。`sim-world` の `World` を
  Phase 0 の `FallingBody` から `MechanicsSolver` 経由に移行。`sim-thermal` に集中熱容量ノード網
  (ニュートン冷却+線形化放射+陰的Euler/PCG)を実装し T1・T2 を Green 化。`sim-core` に
  `EnergyLedger` を実装し `sim-world::World` に配線、解析ドリフト予測との一致で検証。`sim-fluid`
  に集中定数抗力モデル(`aero`、Schiller-Naumann補正付き球抗力)と集中定数浮力モデル
  (`buoyancy`、直立直方体限定)を実装し `sim-mechanics` の `apply_forces` に配線、F1–F6 を
  Green 化。P1 の担当解析解テストは M12・M15 を除き全て Green。力学ドメインの角運動量・
  回転運動エネルギー保存則テストを追加(陽的ジャイロのドリフト率を実測・文書化)。
  M11(中間軸不安定)の解析成長率比較は簡易線形化では検証できないことを数値実験で確認、
  P2着手時に再検討する課題として記録。`sim-world::World` を Phase 0 の「箱1個固定」構成から
  `create_body`(複数剛体を任意に追加できる、docs/20-integration/04-world-api.md §2 の縮小版)
  経由の構成へ一般化。`EnergyLedger` は最初の `step()` まで遅延初期化(シーン構築中の
  `create_body` 呼び出しを台帳の基準点計算に含めないため)。`sim-wasm::WasmWorld` の
  JS向け公開シグネチャ(コンストラクタ・`body_position_f32()`)は内部実装のみ追随させ不変に
  維持、`demo` のビルドで確認。P2 の Box-Box(SAT)を実装(15軸判定+面クリップ+辺×辺、
  単体テスト4件 Green)。続けて warm starting(feature_idベース)+ 軸選択ヒステリシス
  (相対5%)を実装 — 同一サイズの箱スタックでA面軸/B面軸の重なり量が完全一致し浮動小数点
  誤差で軸がフリップして warm start を破壊する問題を発見・ヒステリシスで解消し、4段目の
  速度残差が約1.65cm/sから約0.41cm/sへ改善したが、M12はなお Green にならない(2段目の
  貫入が約9.5mmでslop超過)。原因は split impulse 未実装と特定し、実装後に再挑戦する課題
  として記録)
- **作業中**: 力学(`sim-mechanics`)P1 最後の残り — 最小CCD(M15、意図的に後回し。
  下記 §2/§3 の P1 行)。P2 では split impulse(M12 Green化に必要)か
  他ドメインの Phase A スケルトンへ
- **次**: 力学 P1 の残りを詰めたら流体・熱・電磁・量子・統計・天体・レンダリングの型スケルトンへ
  → World/Coupling 拡張、の順にスケルトンと Phase A テスト記述を進める(下記 §2)。
  math ウェーブ(`sim-math` の `Vec3`/`Quat`/`Mat3`/`Transform`/`SimRng`/積分器カタログの汎用部分/
  `Grid3`/`MacGrid`/`GridSampler`/トライリニア・Catmull-Rom補間/勾配・ラプラシアン/`pcg`/`ParticleSet`/
  `SpatialHash`)は依存が無く低リスクなため、Phase A の Red 段階を経ずに直接実装 + テストで Green 化した。
  ただし `RigidIntegrator` トレイト(P1、`RigidBodySet` に依存)・陰的 Euler の具体的な線形系(場は
  揃ったが熱ドメインの温度場が必要)・IC(0)前処理(格子ラプラシアンの疎パターンが必要)・
  leapfrog(Yee)・split-step Fourier・XPBD・semi-Lagrangian・BAOAB は状態型を持つ各ドメイン crate が
  P1–P5 で実装する(sim-math には汎用プリミティブのみ置く)。
  他ドメインは設計通り Phase A(型スケルトン + 全テスト記述・Red確認)から進める。

## 0. 設計フェーズ残作業

決定事項:

- [x] C-1 決定論水準の決定(案 1 緩和を採用 — [20-integration/02](../20-integration/02-determinism-replay.md) §5)
- [x] C-3 位置表現(保持フレーム)の決定(フレーム ID + ローカル座標 f64 を採用 — [../00-foundation/02-scale-ladder.md](../00-foundation/02-scale-ladder.md) §2.2)
- [x] C-5 最小 CCD の方式選定(speculative contact を採用 — [../10-mechanics/02-collision-detection.md](../10-mechanics/02-collision-detection.md) §4.6)

改訂 PR:

- [x] PR-1 対応不要判断(A-1/B-1/B-3/D)の反映(vision §4、phases.md 開発体制の前提)
- [x] PR-2 テスト表の実装可能性監査・長時間級ルール・Boris pusher・Wolff 必須化・F11 注記
- [x] PR-3 決定論方針の反映・台帳再定義・sub-iteration 規則 + stiff 検出テスト行
- [x] PR-4 位置表現の決定反映・最小 CCD 標準機能化 + 検証テスト行・立位保持基準の置換
- [x] PR-5 性能構成規則・wasm 配布戦略・巻き戻しコスト
- [x] PR-6 新設文書: UI/フロントエンド設計・フレーム階層詳細設計・レジーム切替プロトコル
- [x] PR-7 実装の難所の詳細化(全ドメイン文書横断 — 難所一覧は [../00-foundation/01-vision.md](../00-foundation/01-vision.md) §4.1)
- [x] 実装開始ゲート通過(vision §4: レビュー承認。ユーザー指示により2026-07-19承認)

## 1. Phase 0 — 骨格

- [x] Cargo ワークスペース(05-rust-wasm-platform §2、sim-astro/sim-render 含む)
- [x] CI 最小構成(fmt / clippy / test / wasm ビルド / 決定論スモーク)
- [x] demo の Vite + Three.js 雛形
- [x] wasm 境界の疎通
- [x] 最小 World: 箱 1 個が落ちて cargo test 緑 + ブラウザ表示 + ハッシュ 2 回一致

## 2. Phase A — テスト先行(Red)

型・トレイトのスケルトン(中身 `todo!()`、コンパイル可):

- [x] math(Vec3/Quat/Mat3・場・`Integrator`・SimRng)— Red を経ず直接 Green 化済み(§3参照)
- [x] 力学(剛体状態・`Solver`/`Constraint`・衝突型)— `RigidBodySet`/`BodyType`/`Shape`(Sphere/Box/
      Plane 実装、Capsule/Compound/ConvexMesh は型のみ)/`MechanicsSolver`(`Solver`実装)まで完了。
      `Constraint`(ジョイント)型は P3 で追加
- [ ] 流体(MAC 格子・SPH 粒子)
- [ ] 熱(熱ノード・相変化)
- [ ] 電磁(回路 MNA・静場・FDTD・光学)
- [ ] 量子(TDSE)
- [ ] 統計(気体分子・イジング・ランジュバン)
- [ ] 天体(N 体・軌道・フレーム階層)
- [ ] レンダリング(パストレ骨格)
- [ ] World / Coupling / 台帳 / スナップショット — `sim-core` 側の共通基盤(`Solver`トレイト・
      `SolverContext`・`EventQueue`・`MaterialDb`)は先行実装済み(`crates/sim-core/src/{solver,material}.rs`)。
      `World`本体の拡張・`Coupling`トレイト・`EnergyLedger`・スナップショットは未着手

テスト記述(定義は [21-verification/01-analytic-tests.md](../21-verification/01-analytic-tests.md)、
Green 管理は [§8](#8-解析解テスト-green-管理表) で行う):

- [ ] 力学 M1–M15 を記述、全 Red 確認
- [ ] 流体 F1–F11 を記述、全 Red 確認
- [ ] 熱 T1–T8 を記述、全 Red 確認
- [ ] 電磁 E1–E13 を記述、全 Red 確認
- [ ] 量子 Q1–Q6 を記述、全 Red 確認
- [ ] 統計 S1–S9 を記述、全 Red 確認
- [ ] 天体 A1–A10 を記述、全 Red 確認
- [ ] レンダリング R1–R7 を記述、全 Red 確認
- [ ] 結合 stiff 検出 X1–X2 を記述、全 Red 確認
- [ ] 各ドメイン文書 §7 のユニットテストを記述
- [ ] 保存則テスト(21-verification/02)を記述(力学ドメインの角運動量・回転運動エネルギー
      保存は Green 実装済み — `crates/sim-mechanics/tests/conservation.rs`。陽的ジャイロ積分の
      ドリフト率を実測・文書化(dt=1/120・1秒で |L|≈0.52%、KE≈0.79%、許容2%)。他ドメイン・
      他保存量は未記述)
- [ ] 決定論テスト(20-integration/02 §6)を記述
- [ ] テスト自体のレビュー完了(Phase A 完了条件)

## 3. Phase B — 実装ウェーブ(Green)

### math ウェーブ

- [x] 線形代数(Vec3/Quat/Mat3/テンソル)
- [x] 場(MAC / セル中心格子・補間)— `Grid3<T>`/`BoundaryRule`/`GridSampler`(Clamp/Constant/
      ZeroGradient/Periodic)、トライリニア・Catmull-Rom 補間、勾配・ラプラシアン(一様係数・流束形式の
      変係数版)、`MacGrid`+発散、PCG(`pcg`、Jacobi 前処理。IC(0) は P3 で具体的な格子ステンシルと
      併せて実装)、`ParticleSet`、`SpatialHash`(Teschner ハッシュ、総当たり一致テスト済み) —
      `crates/sim-math/src/{grid,pcg,particles}.rs`
- [x] 積分器カタログ: 状態非依存の汎用部分は Green
      (explicit/semi-implicit Euler・velocity Verlet・RK4=`BallisticIntegrator`・Boris pusher、
      `crates/sim-math/src/integrators.rs`)。ドメイン状態型が要る残り(XPBD・Euler–Maruyama/BAOAB・
      陰的 Euler・semi-Lagrangian・leapfrog・split-step Fourier)と `RigidIntegrator` トレイトは
      各ドメイン crate の P1–P5 実装時に追加する
- [x] 決定論 PRNG(SimRng)・分布サンプリング(PCG-XSH-RR 64/32、公式参照ベクタ一致 —
      docs/01-math/04-random.md §1/§3/§5)
- [x] 数学基盤テスト・収束次数 ◆ Green(sim-math 全体で47テスト。ドメイン結合が要る残りの積分器・
      IC(0) は各ドメイン crate の担当ウェーブで追加テストする)

### P1 — 力学基礎

- [x] 剛体(状態・慣性テンソル・力/トルク API)— `crates/sim-mechanics/src/{body,shape,solver}.rs`
- [x] 総当たり衝突・接触ソルバ(sequential impulses)— `crates/sim-mechanics/src/{collision,contact}.rs`。
      narrowphase は Sphere-Sphere/Sphere-Plane/Box-Plane/Sphere-Box(Phase1の4組)+
      Box-Box(SAT、15軸+Sutherland-Hodgmanクリップ、`collision.rs::box_box`)。
      軸選択のヒステリシス(相対5%、`collision::AxisCache`)+ warm starting
      (feature_idベース、`contact::WarmStartCache`。feature_idは軸選択+参照面上の象限から
      安定的に組み立てる、post-clipのインデックスは使わない)を実装。マニフォールド持続化
      (§4.7 の移動量2mmチェック)・split impulse は未実装(多段スタックで貫入が slop を
      超える既知の制限、下記 M12 参照)
- [x] 摩擦(クーロン・摩擦円錐)— 箱近似(2接線独立クランプ、`contact.rs::solve_tangent`)、
      `MaterialDb::friction_pair`(幾何平均+ペア表)を実接触ソルバで使用
- [ ] 最小 CCD(弾丸級の speculative contact)
- [x] 位置表現 = フレーム ID + ローカル座標(`sim_core::FrameId`、単一ルートフレームで運用中。
      フル階層は Pα)
- [x] 重力(実装済み)。抗力(球、Schiller-Naumann補正付き、`sim-fluid::aero`+
      `MechanicsSolver::apply_forces`)を実装、F1–F3 Green化。浮力(直立直方体、
      `sim-fluid::buoyancy`+`MechanicsSolver.water`)を実装、F4–F6 Green化(一般姿勢の
      凸多面体切断・球冠体積・水中抗力は Phase 3)
- [x] 熱ノード(基礎)— `crates/sim-thermal/src/lib.rs`。集中熱容量ノード網 + ニュートン冷却
      (対流)+ 放射(線形化、Picard 1回)+ 陰的Euler(matrix-free PCG、`sim_math::pcg`)
- [x] エネルギー台帳(残差トレンド監視)— `crates/sim-core/src/ledger.rs::EnergyLedger`
      (docs/00-foundation/04-architecture.md §1.1.2(2)、docs/21-verification/02-conservation-laws.md
      §2 の residual 式)。`sim-world::World` に配線し毎 step 後に mechanics 合計エネルギーを記帳。
      解析予測(接触なし自由落下の semi-implicit Euler 線形ドリフト)と記帳値が一致することを
      `crates/sim-world/src/lib.rs::tests::energy_ledger_residual_matches_analytic_symplectic_drift`
      で検証
- [ ] 担当テスト Green: M1–M9, M12, M15, F1–F6, T1, T2(M1・M5–M9・F1–F6・T1・T2 Green。
      M12 は Box-Box(SAT)+ warm starting + 軸選択ヒステリシス実装後も未 Green:
      軸フリップ由来の不安定化はヒステリシスで解消し速度が改善した(4段目時点の実測が
      約1.65cm/sから約0.41cm/sへ改善)が、なお貫入がslop(5mm)を超える(2段目で約9.5mm)。
      原因は split impulse 未実装(Baumgarteのみ)によるもので、split impulse 実装後に
      再挑戦する。M15=最小CCD待ち)

### P2 — 力学拡充

- [x] Box-Box(SAT)— `crates/sim-mechanics/src/collision.rs::box_box`。15軸分離判定
      (面3+3、辺×辺9)+ 面接触は参照面への Sutherland-Hodgman クリップ(最大4点、
      設計 §4.4 の縮約は簡易版: 面積最大化でなく深度降順で上位4点)+ 辺×辺接触は
      2線分の最近点1点。退化ケース(平行辺の軸除外・クリップ0点フォールバック)を実装
- [x] 軸選択ヒステリシス(相対5%、`collision::AxisCache`)— 設計 §4.4・§9。同一サイズの
      箱が積み重なるとA面軸/B面軸の重なり量が理論上完全一致し、浮動小数点誤差で
      ステップごとに選択軸がフリップして warm start の feature_id 対応を破壊する
      (実測: ヒステリシスなしでは warm starting がむしろ速度残差を悪化させた)ことを発見・
      修正
- [x] Warm starting(feature_idベース、`contact::WarmStartCache`)— 設計 §4.4。マニフォールド
      持続化(§4.7 の移動量チェックによる再利用判定)は未実装、feature_id 自体は軸選択+
      参照面象限から安定的に算出(post-clipインデックスは不安定なため不使用)
- [ ] SAP / BVH(broadphase)
- [ ] split impulse・スリープ・転がり摩擦(M12 Green化に必要)
- [ ] 担当テスト Green: M6(精度), M10, M11(M10 は固定ピボット回転がジョイント実装
      (P3、docs/10-mechanics/05-joints-constraints.md)に依存。M11 は簡易線形化で解析成長率との
      比較を試みたが、非線形フィードバック(ωx・ωzの積がωyへ2λ倍のレートで再結合)により
      線形近似が数値実験で想定より早く破綻することを確認済み — 正しい検証には Jacobi 楕円関数
      による厳密解、または慎重な多重スケール摂動法が必要で、P2着手時に再検討する)

### P3 — 拘束・流体・熱

- [ ] ジョイント・拘束(ヤコビアン)
- [ ] XPBD(布・ロープ)
- [ ] 格子流体(MAC・semi-Lagrangian・投影法)
- [ ] 熱伝導網・相変化(エンタルピー法)・気体区画
- [ ] 並列リダクション(同一スレッド数で決定的 — C-1 案 1)
- [ ] 担当テスト Green: M3, M4, M13, M14, F7–F9, F11, T3, T5, T7

### P4 — 電磁・光・SPH・車両・ブラウン

- [ ] 回路(MNA・非線形素子収束)
- [ ] モーター結合(sub-iteration 決定的算出)
- [ ] 静電場・静磁場
- [ ] 幾何光学
- [ ] WCSPH
- [ ] 車両(Pacejka)
- [ ] ランジュバン(ブラウン運動)
- [ ] エンティティ受け入れ: 関節 PD 静的姿勢維持
- [ ] 担当テスト Green: E1–E7, E9–E12, F10, S4–S6, T8

### P5 — 量子・統計・波動

- [ ] シュレディンガー(split-step)
- [ ] FDTD(Yee・PML)
- [ ] 気体分子運動
- [ ] イジング(Metropolis + Wolff 必須)
- [ ] GJK / EPA・フル CCD
- [ ] 担当テスト Green: Q1–Q6, E8, E13, S1–S3, S7–S9

### Pα — 天体

- [ ] N 体重力(Barnes-Hut・シンプレクティック)
- [ ] 軌道・再突入・宇宙機
- [ ] フレーム階層・floating origin
- [ ] レジーム切替(時間加速)
- [ ] 1PN 補正(オプトイン)
- [ ] 担当テスト Green: A1–A10

## 4. Phase C — 結合・全体検証

- [ ] 結合行列の実装(保存量の対記帳・排他結合 validator)
- [ ] 統合シナリオ: ブレーキ発熱
- [ ] 統合シナリオ: 手回し発電
- [ ] 統合シナリオ: 氷と飲み物
- [ ] 統合シナリオ: 断熱圧縮
- [ ] 統合シナリオ: 再突入
- [ ] CI ゲート: 決定論(2 回実行一致・スナップショット再開一致 = 階層 1、スレッド数変更・wasm⇔ネイティブは許容誤差 = 階層 2 — C-1 案 1)
- [ ] CI ゲート: 保存則 residual
- [ ] CI ゲート: 性能ベンチ回帰(構成規則)
- [ ] 全デモ D1–D39 合格([§7](#7-デモ合格管理表-d1d43))

## 5. Phase D — レンダリング

- [ ] BVH(レイ交差)
- [ ] BSDF・NEE
- [ ] 分光・屈折・コースティクス
- [ ] 参加媒質(大気・水・煙)
- [ ] 物理カメラ・トーンマッピング
- [ ] 担当テスト Green: R1–R7
- [ ] デモ D40–D43 合格

## 6. フロントエンド(設計は [../23-frontend/01-editor.md](../23-frontend/01-editor.md) が正)

Unity 風統合エディタ:

- [ ] Toolbar: 再生制御(▶/⏸/⏭)+ 時間倍率スライダー + 状態ハッシュ表示
- [ ] Scene View: Three.js 3D ビューポート + Gizmo(移動/回転/スケール)+ ピック
- [ ] Scene View オーバーレイ(接触点/速度/力/拘束/流体場/フレーム軸、切替可)
- [ ] Hierarchy: シーングラフツリー(Bodies/Joints/Circuits/Fluids/Probes/Frames)、双方向選択
- [ ] Inspector: Component ビュー(Transform/RigidBody/Joint/Circuit/FluidRegion/Coupling/Probe/近似バッジ)
- [ ] Timeline: 再生スクラバ + Play モードバッジ + ブックマーク
- [ ] Console: イベント・診断ログ(発散・CFL 警告・シーンクラス/スロー再生バッジ)+ フィルタ + クリック→時刻/オブジェクト連動
- [ ] Probe Graphs パネル: 複数系列・対数軸・CSV エクスポート
- [ ] Project ドロワー: Scenes/Materials/Prefabs/Replays
- [ ] Edit / Play モードの切替と編集ロック
- [ ] Command 系(Grab/MoveGrab/Release/SetMotorTarget/…)と入力列記録
- [ ] レイアウトプリセット(Default / Physics-focus / Circuit-focus / Astro)
- [ ] 回路エディタ(Scene View サブモード、D19 自由配線)
- [ ] フレームサブモード(L5 ドリルイン)
- [ ] 予測→実験ミニパネル(シーン側オプトイン)
- [ ] シーン編集・スポーン・材料派生
- [ ] シーン + Replay + ブックマークのエクスポート/インポート
- [ ] Undo / Redo(Edit モードのみ)
- [ ] ヘッドレスランナー(Probe assert・CI 基盤)

## 7. デモ合格管理表(D1–D43)

定義は [21-verification/03-demo-scenarios.md](../21-verification/03-demo-scenarios.md)。
「合格」= 合格基準のヘッドレステスト Green + 目視チェック。

Phase 1(P1〜P2 スモーク):

- [ ] D1 落下時計
- [ ] D2 弾道
- [ ] D3 バウンド比べ
- [ ] D4 積み木
- [ ] D5 斜面
- [ ] D6 浮き沈み
- [ ] D7 風と終端速度
- [ ] D8 散乱の再現
- [ ] D9 冷めるコーヒー
- [ ] D10 摩擦の熱

Phase 2〜3:

- [ ] D11 振り子と時計
- [ ] D12 ラグドール階段
- [ ] D13 ロープと旗
- [ ] D14 煙と渦
- [ ] D15 対流
- [ ] D16 熱伝導レース
- [ ] D17 ピストン
- [ ] D18 氷と飲み物

Phase 4:

- [ ] D19 電気工作台
- [ ] D20 モーターと発電
- [ ] D21 磁石遊び
- [ ] D22 光学ベンチ
- [ ] D23 注ぐ水(SPH)
- [ ] D24 車の実験場
- [ ] D25 ブラウン運動
- [ ] D26 帯電風船

Phase 5:

- [ ] D27 二重スリット(電子)
- [ ] D28 トンネル効果
- [ ] D29 電波の水槽
- [ ] D30 気体の箱
- [ ] D31 拡散とインク
- [ ] D32 磁石の相転移
- [ ] D33 井戸の中の電子

Pα:

- [ ] D34 太陽系儀
- [ ] D35 軌道投入
- [ ] D36 スイングバイ
- [ ] D37 再突入
- [ ] D38 潮汐
- [ ] D39 相対論 ON/OFF

Phase D:

- [ ] D40 光の実験室
- [ ] D41 材質ギャラリー
- [ ] D42 空と大気
- [ ] D43 カメラ

## 8. 解析解テスト Green 管理表

定義・許容誤差は [21-verification/01-analytic-tests.md](../21-verification/01-analytic-tests.md)。
記述(Red)の管理は §2、Green 化はここでチェックする。長時間級(通常 CI 外)の行は
PR-2 の監査で確定後、末尾に「(長時間級)」を付記すること。

力学(M、担当: P1〜P3):

- [x] M1
- [ ] M2
- [ ] M3
- [ ] M4
- [x] M5
- [x] M6(閾値0.5m/sの固定減算により有限衝突速度では厳密1e-9は達成できないため、検証は
      反発閾値0・細かいdtで理想化した設定で実施。既定パラメータでの精密一致はP2 split impulse後)
- [x] M7
- [x] M8
- [x] M9
- [ ] M10
- [ ] M11
- [ ] M12
- [ ] M13
- [ ] M14
- [ ] M15

流体(F、担当: P1/P3/P4):

- [x] F1 — `crates/sim-mechanics/tests/p1_analytic.rs::f1_terminal_velocity_matches_high_re_drag_formula`
- [x] F2 — `crates/sim-mechanics/tests/p1_analytic.rs::f2_raindrop_terminal_velocity_matches_gunn_kinzer_measurement`
- [x] F3 — `crates/sim-mechanics/tests/p1_analytic.rs::f3_stokes_settling_matches_analytic_formula`
      (媒質密度を無視できるほど小さく取り Δρ≈ρ_particle として隔離検証。F3 は気中沈降シナリオ
      であり `MechanicsSolver::water` を設定しないため浮力機構とは独立)
- [x] F4 — `crates/sim-mechanics/tests/p1_analytic.rs::f4_cube_waterline_depth_matches_density_ratio`
- [x] F5 — `crates/sim-mechanics/tests/p1_analytic.rs::f5_floating_body_heave_period_matches_analytic_formula`
- [x] F6 — `crates/sim-fluid/src/buoyancy.rs::tests::f6_hydrostatic_pressure_matches_rho_g_h`(代数検算)
- [ ] F7
- [ ] F8
- [ ] F9
- [ ] F10
- [ ] F11

熱(T、担当: P1/P3/P4):

- [x] T1 — `crates/sim-thermal/src/lib.rs::tests::t1_newton_cooling_matches_analytic_decay`
- [x] T2 — `crates/sim-thermal/src/lib.rs::tests::t2_two_node_equilibrium_matches_weighted_average`
- [ ] T3
- [ ] T4
- [ ] T5
- [ ] T6
- [ ] T7
- [ ] T8

電磁(E、担当: P4/P5):

- [ ] E1
- [ ] E2
- [ ] E3
- [ ] E4
- [ ] E5
- [ ] E6
- [ ] E7
- [ ] E8
- [ ] E9
- [ ] E10
- [ ] E11
- [ ] E12
- [ ] E13

量子(Q、担当: P5):

- [ ] Q1
- [ ] Q2
- [ ] Q3
- [ ] Q4
- [ ] Q5
- [ ] Q6

統計(S、担当: P4/P5):

- [ ] S1
- [ ] S2
- [ ] S3
- [ ] S4
- [ ] S5
- [ ] S6
- [ ] S7(L=256 フル版は長時間級)
- [ ] S8(L=256 フル版は長時間級)
- [ ] S9

天体(A、担当: Pα):

- [ ] A1
- [ ] A2(10⁶ 周フル版は長時間級)
- [ ] A3
- [ ] A4
- [ ] A5
- [ ] A6
- [ ] A7
- [ ] A8
- [ ] A9
- [ ] A10

レンダリング(R、担当: Phase D):

- [ ] R1
- [ ] R2
- [ ] R3
- [ ] R4
- [ ] R5
- [ ] R6
- [ ] R7

結合 stiff 検出(X、担当: P4/Phase C):

- [ ] X1
- [ ] X2
