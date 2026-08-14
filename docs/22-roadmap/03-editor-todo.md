# 統合エディタ TODO(棚卸し版)

出典: [../reviews/2026-08-14-editor-implementation-plan.md](../reviews/2026-08-14-editor-implementation-plan.md)
の TODO 17件に、同ドキュメント §3「意図的にやらないこと」で除外されていた4件を
条件付きでスコープへ編入したもの。「やらない」を無条件の除外ではなく、
着手条件を明記したバックログとして扱う。

## 目的

UIで自由に物体・環境を編集し、複雑なシナリオを組んで検証できる状態にする。
物理エンジン本体(7ドメイン・644テスト)は[02-feature-checklist.md](02-feature-checklist.md)
の通り完成済みだが、「ユーザー自身が物を配置し、編集し、保存して検証できる」
エディタ側の配線が欠けている。ソルバが持つ51種類の物理要素のうちUIから作れるのは
9種類のみ、結合14種は1つも作れない——物理は完成しており、欠けているのは配線だけ、
というのが現行の残タスクの中心。

運用ルール: 項目が完了するたび `[ ]` を `[x]` にし、その作業と同じコミットに含める。
末尾4件(§3由来)は着手条件が満たされるまで着手しない。

## TODO

- [x] 物差しとなるE2Eテストを2本置く(**執筆時点で両方グリーン**)
  `crates/sim-world/tests/editor_acceptance.rs`(crate境界の外側=統合テストとして
  新設。単体テストと違い非公開項目に触れられないため「特権アクセス不在」の証明に
  なる)。①D24車(WheelJoint・操舵駆動モータあり)をsave→load→60step実行した
  state_hashが元の実行と一致することを確認。②`scenes/index.json`列挙の全シーンを
  `Scenario::from_json`+`World::from_scenario`(公開API)のみで構築・60step実行
  できることを確認。当初「今は必ず落ちる」想定だったが、先行タスク
  (`World→Scenario`逆写像・安定ID・部品作成メソッド)が既に土台を作っていたため
  書いた時点で両方PASS——以後の退行検知として維持する。
- [x] 縮約監査スクリプトを作る(`scripts/audit_reductions.py`)
  コード中に自己申告された「縮約」を機械集計する。実行結果: 385件
  (crates 267・docs 118)。
- [x] `Scenario` に `Serialize` を実装する(55構造体すべて)
- [ ] `World → Scenario` の逆写像を実装する(**一部完了**)
  bodies/joints/couplings/fluids/thermal/circuit/probes を全ドメイン無損失で書き戻し、
  手書きの `export_scene_json` を置き換える。`sim-world::export::to_scenario` として
  world options・materials・bodies・joints(Distance/Ball/Slider/Wheel/HingeMotor)・
  couplings(14種)・probes・thermal・circuit・astro・gas を実装済み(state_hashが
  reload直後・stepping後の両方で一致することをテストで確認)。
  残: `grid_fluid`/`soft_body`/`sph`/`quantum_1d`/`quantum_2d`/`brownian`/
  `kinetic_gas`/`ising`/`fdtd` ——シーンJSON側が「構築レシピ」形式(波束の中心・
  分散、SPH粒子を敷き詰める直方体ブロック等)で状態スナップショットを表現できない
  ため、生値スナップショット形式のスキーマ拡張が別途要る。
- [x] 安定ID(世代付き)をwasm境界まで通す
  監査の結果、JS向けのindex自体(`self.bodies: Vec<SpawnedBodyMeta>`の位置)は
  削除でもシフトしないため署名としては既に安定していた。実際の欠陥は
  「そこから解決した`BodyId`を、その後generation確認なしにWorldの生配列へ
  直接インデックスしていた」こと(Timeline巻き戻し後に範囲外パニックで
  モジュール全体が壊れる、再現・回帰テストとも確認済み)。52箇所すべてが経由する
  単一の解決点 `try_body_id_at` に `World::is_body_alive`(世代確認)を追加して
  修正——52本の署名を書き換えるより的確で、JS側の呼び出し規約も変えずに済む。
- [x] World APIに部品作成メソッドを実装する
  `add_coupling`/`add_probe`は既存。未実装だった`create_joint`(JointDesc、
  BodyId直接参照)・`add_fluid_region`・`EnvironmentDesc`(重力・大気・水域・
  周囲温度をまとめて読み書き)を追加。Inspector UI・wasm境界からの配線は
  未着手(Add Component/schema-read-applyの各タスクで行う)。
- [x] `Shape` の `todo!()` 4箇所を埋める(Compound/ConvexMesh)(**一部近似**)
  AABB・接触生成・体積・慣性テンソル。「質量を訊くとpanicする」は両形状とも
  解消。Compoundは接触生成(narrowphase)まで含めてフル実装
  (部品ごとに既存の解析的ペア関数へ再帰分解、L字形ボディが地面に落ちて
  静止するところまでE2Eテストで確認)。ConvexMeshは面情報を持たない
  (頂点列のみ)ため、体積/慣性/AABBはAABB近似で対応、接触生成(実際に
  衝突する)は「外部クレート実質ゼロ」の方針で3D凸包の自前実装が要り
  範囲外——`None`(すり抜け)を返す既知の限界として明記。
- [ ] wasm境界を `schema`/`read`/`apply` の3メソッドへ畳む(現状118本・2,798行)
- [x] Inspectorに Add Component とスキーマ駆動フォームを実装する(**Jointの5種のみ**)
  `World::joints()`/`JointKind`にWheelJointが無く(追加はできても内省層に
  一切出ず、Inspectorから見えなかった既存の欠落)を先に修正。
  `WasmWorld::add_distance_joint`/`add_ball_joint`/`add_slider_joint`/
  `add_wheel_joint`/`add_hinge_motor_joint`(`JointDesc`5種の薄い写像)を
  新設し、InspectorにAdd Jointフォームを追加(種別ごとの専用フォームでは
  なく、自由配線回路エディタと同じ「種別セレクト+汎用Body/Anchor/Axis/
  Param欄、使うフィールドはtitleツールチップで示す」縮約)。Rust側テスト・
  Playwrightでの実UI経由操作(Ball/Wheel追加→`joint_info_text`に反映)の
  両方で確認、QA16/16・スモーク23/23維持。Coupling(14種、縦串②)・
  FluidRegion/Environment(縦串③)は対象外——スキーマ駆動フォームの
  汎用化(wasm境界のschema/read/apply化)と一体に進める方が手戻りが少ない
  ため、Jointだけを先行させた。
- [x] 形状描画をShape記述に一本化する(**一部完了**)
  `demo/src/main.ts` の `sceneImportRef.current`/`sceneGalleryRef.current` に
  2箇所コピーされていた形状パーサを `meshFromShapeJson()` 1関数に集約。
  副次的に実バグを発見・修正: `ImportedShapeJson` がCapsuleを型に持たず、
  カプセル形状のボディが常に0.3mの球として描かれていた(計画書指摘の不具合)
  ——capsule variantを追加し`CapsuleGeometry`で正しく描画。未知形状は
  黙って球を出さずconsole.warnで警告するよう変更。Compound/ConvexMeshの
  メッシュ生成は対象外——シーンJSONスキーマ・`body_shape_kind_at`とも
  これらの形状を表現できず、UIから作る経路も無いため現状到達不能・
  テスト不能(縮約として明記)。
- [x] 縦串①(ジョイント)の受け入れテストを緑にする
  `demo/tests/acceptance-d24.spec.ts`。D24相当の車(シャシー+車輪4個+
  WheelJoint4本)をScene View/Inspectorの**UI操作だけ**(スポーンパレット・
  Position/Scale/Mass/Collisionフィールド・Add Jointフォーム)で組み立て、
  60step実行した`state_hash`が既存の`d24-car.json`をそのまま実行した結果と
  完全一致することを確認。唯一UI操作でない例外は開始状態——既定の起動シーンは
  回路・熱ドメインの実演セットアップを最初から積んでおり、それを消して
  「床だけ」から始める「新規シーン」ボタンが無いため、シーンギャラリーの
  差し替えロジックをテスト専用に露出した`window.__loadSceneJson`でD24の
  groundのみのシーンへ差し替えてから、残り全部をUIで組み立てる
  (詳細は同ファイル冒頭のdocコメント)。

  **副次的に発見・修正した実バグ2件**(このテストを緑にする過程で発覚——
  どちらも「フィールドを1つずつ順にfillすると、値が未確定のうちに
  毎フレーム更新が割り込んで消える」という同型のレース):
  1. 球のスケール入力: 3欄(X/Y/Z)の一致を等方スケール適用の条件にしていたが、
     欄を1つずつ確定するUIでは残り2欄が古い値のままなので毎回不一致になり、
     一度も適用されなかった(球の半径をUIから正確な数値で指定する手段が
     事実上無かった)。変更された欄の値だけを信じるよう修正。
  2. Collision group/mask: 変更のたびに両欄をDOMから読み直していたが、
     `updateInspectorRigidBodyFields`の毎フレーム更新(未適用の欄をフォーカスが
     外れた瞬間に実際の値へ戻す、設計どおりの挙動)がその間に割り込むと、
     片方の欄が古い値に戻った状態でもう片方の変更と組んで送られていた
     (実測: groupが常に1のまま送られ、意図した2/4が反映されなかった)。
     各欄の変更をローカル変数へ確定し、DOMを読み直さないよう修正。
- [x] 回路素子4種をUIエディタに追加する(コンデンサ・インダクタ・ダイオード・DCモータ)
  `sim_em::Circuit`は既に7種そろっていたので、自由配線回路エディタ
  (`circuit_editor_*`)へUI+wasm境界の配線を追加。DCモーターは内部ノードを
  自動確保する`Circuit::add_nodes`を新設。wasm実ビルド+`tsc`+Playwright
  スモーク23件で検証済み。
- [x] QA報告の不具合9件を修正する
  ([2026-08-04-editor-qa.md](../reviews/2026-08-04-editor-qa.md) の既知不具合)。
  再現スクリプト(`demo/tests/qa/qa-defects.mjs`)が0/16→16/16 PASSへ転じたことを
  確認済み。Playwrightスモーク23件・Rust側テストも無傷。
- [x] 結合14種を縦串②として配線する(**8種**)
  `WasmWorld::add_image_charge_force_coupling`/`add_lorentz_force_coupling`/
  `add_buoyancy_drag_coupling`/`add_dissipation_to_heat_coupling`/
  `add_joule_heat_coupling`/`add_brownian_force_coupling`/
  `add_motor_coupling`/`add_induction_coupling`(`sim_coupling`8種の薄い
  写像)を新設し、InspectorにAdd Couplingフォームを追加(Add Jointと同じ
  「種別セレクト+汎用Param欄」縮約)。**レビューで「3種だけで大丈夫か」と
  指摘を受け、剛体参照だけで完結する3種(ImageChargeForce・LorentzForce・
  BuoyancyDrag)に加え、熱ノード・電圧源を`usize`のindexで参照するだけの
  5種(DissipationToHeat・JouleHeat・BrownianForce・MotorCoupling・
  InductionCoupling)へ拡張した**——既定の起動シーンが熱ノード1個・電圧源
  1個(いずれもindex 0)を最初から持つため、それらを参照するだけならUIから
  熱ノード自体を作る手段が無くても実用になる。範囲外indexを渡すと
  `try_thermal_node_index`/`try_voltage_source_index`が明示的に`Err`を返す
  (熱・回路ドメインが無いシーンで追加しようとした時に、無言で無効な状態に
  なるより失敗として伝わる)。
  残り6種(PhaseChangeMorph/SphRigid/GridFluidRigid/ConvectionLink/
  PistonGas/BoussinesqBuoyancy)は熱ノード**自体を作る**か、SPH/格子流体
  ドメインをUIから作れるようにならないと意味を持たないため対象外——
  対応ドメインのUI作成(別途)が先に要る。
  Rust側テスト・Playwrightでの実UI経由操作(8種すべて追加→
  `coupling_info_text`に反映)の両方で確認、QA16/16・スモーク24/24維持。
- [x] 環境と大気の場を縦串③として実装する(**大気・水域のみ、重力ベクトル化は対象外**)
  Settingsに「環境(大気・水域)」パネルを追加。`sim_fluid::Atmosphere`は
  既に`density`/`viscosity`/`wind: Vec3`(風の場)を持っていた
  (物理コアは無変更)が、UIから設定する手段が無かった——
  `WasmWorld::set_atmosphere`/`clear_atmosphere`/`set_water_region`/
  `clear_water_region`(いずれも`World::environment()`/`set_environment`
  経由、Task#6で実装済みのAPI)を新設して配線。ISA標準大気(高度依存密度)は
  国際標準大気の気圧公式($\rho(h)=\rho_0(1-Lh/T_0)^{gM/(RL)-1}$)を
  フロントエンドJS側で計算し、高度入力→密度欄へ書き込むボタンとして実装
  (物理コアには触れない——1つの数値をJSで計算するだけなので「物理コアへの
  変更」ではない)。`BuoyancyDrag`結合(縦串②)を持つ剛体に効く。
  **重力ベクトル化(任意方向の重力場)は対象外**——現状
  `MechanicsSolver::gravity: f64`(下向き固定スカラー)の変更はワークスペース
  全体の呼び出し規約・全テストへ波及する物理コア変更であり、
  末尾の「物理コアへの変更を再評価する」の着手条件(新規/既存ドメインの
  拡張が具体的に必要になった時点)に該当するため、配線(エディタ側)の
  完成を優先する本方針どおり見送った。
  検証: Rust側新規テスト(大気・水域の設定/解除の往復)、cargo test
  --workspace全緑、fmt/clippyクリーン。Playwrightで実UI経由(Settings→
  大気有効化→密度/動粘性/風入力→ISA密度ボタン→水域有効化)の一連の
  操作が`atmosphere_density`等へ正しく反映されることを確認、
  QA16/16・スモーク24/24とも維持。
- [ ] 検証機能(合格基準・掃引・差分)を縦串④として実装する
- [ ] 飛行機の物理を縦串⑤として実装する
  `BuoyancyDrag::apply_pre` の力積分方式変更、推力Coupling、操縦面Command。
  着陸装置は既存の `WheelJoint` + Pacejka を流用。
- [ ] 3D CADモデリング(スケッチ・押し出し・ブーリアン)を実装する
  着手条件: 縦串⑤(飛行機)で凸分解の欠如が実際に不便になった時点。
  凸分解が必須になり「外部クレート実質ゼロ」の方針では自前実装が要る
  (物理1ドメイン分の作業量)。
- [ ] 汎用ECS/プラグイン機構の要否を再評価する
  着手条件: ドキュメント+スキーマ方式(現行方針)では表現できない要件が
  具体的に出てきた時点。それまでは「要るのはドキュメントとスキーマだけ」の
  判断を維持する。
- [ ] Unity機能パリティ相当の追加機能を再評価する
  着手条件: 「複雑なシナリオを組んで検証できる」という目標に対して、
  具体的に不足している機能がユーザーから指摘された時点。
- [ ] 物理コア(7ドメイン・644テスト)への変更を再評価する
  着手条件: 新規ドメイン、または既存ドメインの拡張が具体的に必要になった時点。
  それまでは配線(エディタ側)の完成を優先する。
