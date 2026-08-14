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
- [x] 形状描画をShape記述に一本化する(**全部完了**)
  `demo/src/main.ts` の `sceneImportRef.current`/`sceneGalleryRef.current` に
  2箇所コピーされていた形状パーサを `meshFromShapeJson()` 1関数に集約。
  副次的に実バグを発見・修正: `ImportedShapeJson` がCapsuleを型に持たず、
  カプセル形状のボディが常に0.3mの球として描かれていた(計画書指摘の不具合)
  ——capsule variantを追加し`CapsuleGeometry`で正しく描画。未知形状は
  黙って球を出さずconsole.warnで警告するよう変更。

  **レビュー指摘(「一部完了ではなく全部完了となるよう」「UIから作る経路が
  ないから〜について、出来るようにする前提で開発を推進してください」
  「縮約させないよう、あるべき姿を検討し実装すること」)を受けて、
  Compound/ConvexMeshも含めて完遂した:**
  - シーンJSONスキーマ(`ShapeJson`)に`Compound`/`ConvexMesh`を追加し、
    JSON⇄`Shape`の変換を`shape_json_to_shape`/`shape_to_shape_json`として
    共有関数化(`sim-world`)。これに伴い、`sim-wasm`の`import_scene_json`/
    `from_scene_json`に**元から存在していた非exhaustive match(Compound/
    ConvexMeshのアーム欠落、コンパイラが指摘する実バグ)**と、
    `export.rs`の`export_bodies`にあった`unreachable!()`(Task#7完了後も
    残っていた古い前提の実バグ)を副次的に発見・修正。
  - `WasmWorld::spawn_compound_l_shape`/`spawn_convex_mesh_cube`を新設し、
    ツールバー「＋ 追加」メニューとScene View右クリックメニューの両方に
    「＋ 複合形状 (L字)」「＋ 凸包メッシュ」を追加——UIから実際に作れる
    経路がこれで存在する(以前は無く、シーンJSON importでしか到達
    できなかった既知の欠落だった)。
  - `meshFromShapeJson()`にCompound(空ジオメトリの入れ物へ子メッシュを
    再帰的に載せる「carrier mesh」)とConvexMesh(`three/examples/jsm`の
    `ConvexGeometry`で頂点群から見た目上の凸包を計算)の描画分岐を追加。
    ConvexMeshの接触判定が`None`(すり抜け)なのは`sim-mechanics`側の
    既知の限界(Task#58参照、レビューで指摘されていない)のままだが、
    描画は本物の凸包として正しく出る。
  - `body_shape_kind_at`が`Shape::Capsule`を`_ => "other"`に落として
    いた**副次的に発見した実バグ**(フロント`duplicate()`の
    `kind === "capsule"`分岐が到達不能だった)も合わせて修正し、
    sphere/box/capsule/plane/compound/convex_meshの6種を網羅する
    完全一致にした。
  - 複製(`duplicate()`)がCompound/ConvexMeshでも動くよう、新設した
    `body_shape_json_at`(実際の形状をシーンJSON形式で読み直す、
    `shape_to_shape_json`を再利用)経由でメッシュを再構築——スポーン時の
    既定形状だと決め打ちしない。
  - **副次的に発見・修正した実バグ**: `ShapeJson::ConvexMesh`のJSONタグが
    列挙型全体の`#[serde(rename_all = "lowercase")]`だけでは単語区切りが
    消えて`"convexmesh"`になり、フロント側が前提としていた`"convex_mesh"`
    キーと食い違って**エクスポートJSON経由の凸包メッシュ描画が常に
    フォールバック(0.3mの球)へ落ちる**状態だった。`#[serde(rename =
    "convex_mesh")]`を明示して修正し、タグを固定する回帰テストも追加。
  - Playwrightで実UI経由(ツールバー/右クリック双方のメニューからの
    スポーン→Hierarchy反映確認→N step実行でクラッシュしないこと→
    複製してもクラッシュしないこと)を検証する受け入れテストを追加
    (「テスト不能」への直接の反証)。Rust側もスポーン成功パス・
    `body_shape_kind_at`/`body_shape_json_at`の往復を単体テストで確認。
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
- [x] 結合14種を縦串②として配線する(**14種すべて**)
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

  **レビュー指摘(「やり遂げて欲しい」「対応できていますか？出来ていなければ
  対応して」)を受けて、残り6種(PhaseChangeMorph/SphRigid/GridFluidRigid/
  ConvectionLink/PistonGas/BoussinesqBuoyancy)も完遂した:**
  「熱ノード自体を作る手段が無い」「SPH/格子流体/気体ドメインをUIから
  作れない」という、対象外にしていた理由そのものを埋めた——
  - `WasmWorld::add_thermal_node(temperature, heat_capacity) -> usize`
    (熱ドメインが無効なら周囲温度293.15Kで自動的に有効化してから追加、
    Settingsの「ドメイン」パネルから呼べる)。
  - `WasmWorld::enable_grid_fluid_2d_domain()`/`enable_gas_compartment()`
    (いずれも既定パラメータ・冪等)を新設し、同じく「ドメイン」パネルへ
    ボタンとして配置。SPHドメインは既存の「＋ 流体 (SPH 水塊)」
    (`spawn_fluid_block`)をそのまま流用——別のSPH作成経路を増やさない。
  - `add_phase_change_morph_coupling`・`add_sph_rigid_coupling`・
    `add_grid_fluid_rigid_coupling`・`add_boussinesq_buoyancy_coupling`・
    `add_convection_link_coupling`・`add_piston_gas_coupling`を新設し、
    Add Couplingフォームの種別セレクトへ追加。対応ドメインが無効な状態で
    呼ぶと明示的に`Err`を返す(他8種と同じ方針)。
  - Rust側単体テスト(6種すべて成功パスで追加でき`coupling_info_text`に
    反映されること)、Playwrightで実UI経由(Settingsの「ドメイン」パネルで
    熱ノード・格子流体・気体を有効化→「＋ 流体」でSPHを有効化→Add
    Couplingフォームで6種すべて追加→Inspectorに反映)の受け入れテストを
    追加して確認、QA16/16・スモーク26/26維持。

  **レビュー指摘(「ﾋﾟﾋﾟﾋﾟｯ縮約禁止令発令中」「設定できるようになってますか？
  諦めていませんか？」)を受けて、上記の時点で残っていた2件の縮約
  (`PhaseChangeMorph`の材質固定・`ConvectionLink`の流体物性値固定)も
  解消した:**
  - `add_phase_change_morph_coupling`のシグネチャに`melting_temperature`/
    `latent_heat_fusion`/`specific_heat_solid`/`specific_heat_liquid`
    (材質の4物性値)を追加し、氷/水の定数への固定をやめた——Axisの3欄
    (元々MotorCoupling等の回転軸専用だった汎用欄)を融点・融解潜熱・
    固相比熱に、Param欄の1つを液相比熱に割り当てることで、専用フォームを
    新設せず汎用Add Couplingフォームの枠内に収めた。
  - `add_convection_link_coupling`のシグネチャに`fluid_thermal_conductivity`/
    `kinematic_viscosity`/`prandtl_number`/`thermal_expansion_coefficient`
    (流体物性値)を追加し、`ConvectionLink::default()`(空気20℃)固定を
    やめた——同じくAxisの3欄を熱伝導率・動粘性・プラントル数に割り当てた。
    `thermal_expansion_coefficient`は`Option<f64>`なので、UIの数値欄1個で
    表現するため「0以下なら`None`(理想気体近似)」という符号化にした。
  - Rust側テスト・Playwrightテストとも新シグネチャに合わせて更新、
    実際に材質/流体物性値を明示指定して追加できることを確認。
- [x] 環境と大気の場を縦串③として実装する(**大気・水域・重力の向き**)
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

  **レビュー指摘(「見送らず対応すること」)を受けて、重力ベクトル化
  (任意方向の重力場)も完遂した:**
  - 影響範囲を精査した結果、`MechanicsSolver::gravity: f64`を丸ごとベクトル型
    へ置き換える(全呼び出し規約変更・シーンJSONスキーマ破壊的変更)必要は
    無いと判明——`gravity`(大きさ)はそのまま残し、新設`gravity_direction:
    Vec3`(既定`(0,-1,0)`)を追加する**加算的な**変更で済んだ。自由体への
    重力積分・ポテンシャルエネルギー計算は`gravity * gravity_direction`
    (ベクトル)を使うよう更新、`MechanicsSolver::new`のシグネチャは不変
    (全既存呼び出し元・644テストとも無改修で動く)。
  - **正直な適用範囲**: 浮力(`sim_fluid::buoyancy_force`)・大気抗力は
    重力の向きに依存しない——`StaticWaterRegion`の水面がワールドy座標の
    水平面として定義されるモデル(本増分より前からの`sim-fluid`crateの
    設計上の制約)のため。重力の向きを変えても浮力は`+y`のまま働く。
    これは新たに導入した縮約ではなく、既存の設計上の境界を正直に記録した
    もの。
  - `EnvironmentDesc`に`gravity_direction`を追加(Task#6が元々想定していた
    位置)、`WasmWorld::set_gravity_direction`/`gravity_direction`を新設し
    Settingsパネルへ3軸入力を追加。シーンJSON(`WorldScenarioOptions::
    gravity_direction: Option<[f64;3]>`)にも追加——`#[serde(default)]`で
    未指定時は既定の下向きになり、既存の全シーンファイルと完全後方互換。
    Replayタブの決定論記録(`CommandLogEntry::SetGravityDirection`)にも
    `SetGravity`と同じ理由で対応(記録しても再生しないと最も気づきにくい
    バグになる、既存の`SetGravity`のdoc参照)。
  - 検証: Rust側新規テスト(`+x`向きの重力下で自由落下がm1と同じ解析解に
    軸を入れ替えて従うこと、シーンJSONの後方互換・明示指定の両方、
    `EnvironmentDesc`往復)、Playwrightで実UI経由(Settings→重力の向きを
    x=1,y=0,z=0へ変更→`world.gravity_direction()`に反映)を確認。
    cargo test --workspace全緑、fmt/clippyクリーン、QA16/16・スモーク26/26維持。
  検証(大気・水域分): Rust側新規テスト(大気・水域の設定/解除の往復)、
  cargo test --workspace全緑、fmt/clippyクリーン。Playwrightで実UI経由(Settings→
  大気有効化→密度/動粘性/風入力→ISA密度ボタン→水域有効化)の一連の
  操作が`atmosphere_density`等へ正しく反映されることを確認、
  QA16/16・スモーク24/24とも維持。
- [x] 検証機能(合格基準・掃引・差分)を縦串④として実装する
  Project ドロワーに「Validation」タブを追加。`sim_world::run_headless_scenario`
  をwasm境界へ`run_headless_scenario_json`として公開(自由関数——呼ぶたびに
  独立した新しい`World`を構築するため`WasmWorld`のメソッドではない)。
  現在のシーンをベースJSONとして読み込み、パラメータのJSONパス(ドット区切り、
  例`world.gravity`)+値のリストを指定してN回ヘッドレス実行、結果を
  テーブル(パラメータ値・final_time・final_state_hash・probe最終値・
  合格基準の可否)と、probe履歴を実行ごとに重ね書きしたcanvasグラフで表示する。
  **縮約**: 「合格基準」をシーンJSONスキーマの一部にはしていない
  (`Scenario`のスキーマ拡張は別途判断が要るため対象外)——probe index・
  比較演算子・しきい値をタブのUI状態として持ち、ヘッドレス実行結果に対して
  フロントエンド側で評価する形で代替した。

  **副次的に発見・修正した実バグ**(このタブの実装中に発覚): `WasmWorld::
  export_scene_json`/`bookmark_export_scene_json`が、Task#4で`sim_world::
  to_scenario`を実装した後も手書きの旧実装(`world`/`bodies`の位置・姿勢・
  速度のみを素朴な文字列整形で書き出す)のまま残っており、**probes・joints・
  couplings・thermal・circuit・astro・gasが常に欠落**していた——Task#4の
  TODO本文に「手書きのexport_scene_jsonを置き換える」と残タスクとして
  明記されていたが、着手されていなかった。検証タブでprobeが1本も出ない
  ことから発覚し、`to_scenario`経由に置き換えて修正した(単一ファイル
  Export・ブックマークExport等、既存のシーン保存機能全体の欠落も同時に
  解消される)。
  検証: Rust側新規テスト(`export_scene_json`が既定シーンの2本のprobeを
  含めて書き出し、読み戻せることを確認/`run_headless_scenario_json`が
  D1自由落下を正しく実行することを確認)、cargo test --workspace全緑、
  fmt/clippyクリーン。Playwrightで実UI経由(Validationタブ→パス+値+step数
  入力→スイープ実行)にテーブル・グラフが実データで表示されることを確認、
  QA16/16・スモーク24/24とも維持(単一ファイルExportのスモークテストが
  `export_scene_json`書き換えの回帰検知として機能することも確認)。
- [x] 飛行機の物理を縦串⑤として実装する
  推力: 新しいCoupling/Commandを物理コアへ足さず、既存の`Command::ApplyForce`
  (wasm `push_apply_force`)をそのまま再利用して実装(`ThrustState`、
  Inspectorの Thrust セクション)。Playモード中、毎stepローカル軸をボディの
  姿勢でワールドへ回し、スロットル×最大推力を`ApplyForce`で送る。
  着陸装置: 既存の`WheelJoint`+Pacejkaタイヤモデルがそのまま使える(Add Joint
  フォームの"Wheel"種別、追加のRust側実装は不要と確認済み)。

  **レビュー指摘(「これについては、コア変更してもオッケー」)を受けて、
  揚力の配線と操縦面Commandも物理コア変更を含めて完遂した:**
  - `sim_coupling::BuoyancyDrag::lift`(`LiftModel::Wing`薄翼理論+失速・
    `MagnusSphere`マグヌス効果)は物理コア側に実装済みだったがAdd Coupling
    フォームで`None`固定のままだったため、`WasmWorld::add_wing_lift_coupling`/
    `add_magnus_lift_coupling`を新設して解禁した(種別セレクトへ
    `wing_lift`/`magnus_lift`として追加、Axis欄を翼弦`chord_local`に流用)。
    同じ剛体に複数回呼べるので、主翼・水平尾翼・垂直尾翼・補助翼をそれぞれ
    別の翼として追加できる。
  - 操縦面(エルロン・エレベーター・ラダー)Commandは、Coupling registryが
    元々「追加のみ・実行時パラメータ変更不可」だったギャップを埋める
    **物理コア変更**として実装した: `sim_coupling::Coupling`トレイトへ
    `set_scalar_param(CouplingParam, f64) -> bool`(既定`false`の空実装、
    ほとんどのCouplingは対応不要)を追加し、`BuoyancyDrag`が
    `CouplingParam::ControlSurfaceDeflection`を受けて`LiftModel::Wing`の
    `chord_local`を`span_local`軸まわりに追加回転させる(新設フィールド
    `control_surface_deflection`、既定0)。`World`に`Command::
    SetCouplingParam { coupling_index, param, value }`を新設し、
    `World::add_coupling`の戻り値を`usize`(登録index)化——他の`Command`
    (`ApplyForce`等)と同じ「次stepの先頭で適用・リプレイ再現性を保つ」
    経路。wasm `push_set_coupling_control_surface_deflection`+
    Inspectorの各`BuoyancyDrag`行に舵角入力欄(度→ラジアン変換)を追加。
  - **正直な設計判断**: `BuoyancyDrag::apply_pre`の力積分方式変更(直接
    速度キック方式)は、実装時点で具体的な数値不安定性の証拠が無かったため
    **見送った**——「TODOに書いた懸念」を実測せずに変更すると、D6/D26等の
    既存シーンへ影響しうる物理コア変更を検証なしに行うことになり、それ自体が
    「あるべき姿を検討せず変更した」縮約になりかねないため。Rust側テストで
    舵角による揚力変化が機体回転による揚力変化と解析的に一致すること
    (`control_surface_deflection_matches_an_equivalent_body_rotation`)、
    Playwrightで実UI経由(翼揚力/マグヌス揚力の追加→舵角変更→N step実行)
    にクラッシュしないことを確認済み——数値不安定性の兆候は出ていない。
    将来、高角速度・高舵角の実シナリオで具体的な問題が観測された場合に
    改めて対処する。
  - 検証: Rust側新規テスト(舵角の解析的検証、`set_scalar_param`の既定`false`
    確認)、cargo test --workspace全緑、fmt/clippyクリーン。Playwrightで
    実UI経由(WingLift/MagnusLift追加→Inspectorの舵角欄操作→N step実行)を
    確認、QA16/16・スモーク27/27維持。
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
- [x] 条件付き項目4件の着手可否を評価する
  縦串①〜④が完了した時点での評価(結論: **4件とも未着手のまま維持**、
  ただし4件目は次の縦串⑤で部分的に交差する)。
  1. **3D CADモデリング**: 未着手。縦串⑤(飛行機)にまだ着手していないため
     「凸分解の欠如が不便になった」という着手条件そのものが発生していない。
  2. **汎用ECS/プラグイン機構**: 未着手。縦串②〜④(結合8種・環境パネル・
     検証タブ)は、いずれも既存の「Rust構造体+wasm薄い写像+フォーム」
     パターンだけで実装でき、ドキュメント+スキーマ方式で表現できない要件は
     一度も出てこなかった——これは着手条件の不成立を積極的に裏付ける実例。
  3. **Unity機能パリティ**: 未着手。ユーザーから具体的な不足機能の指摘は
     無い。
  4. **物理コア変更**: 未着手だが、**次の縦串⑤(飛行機の物理)の一部が
     この着手条件に直接抵触する**——`BuoyancyDrag::apply_pre` の力積分
     方式変更は既存ドメイン(浮力・抗力)の拡張そのものであり、D6(浮く箱)・
     D26(帯電風船)等の既存シーンの挙動に影響しうる。縦串⑤に着手する際は、
     この力積分変更だけを明確に切り分け、それ以外の純粋加算的な部分
     (推力Coupling・操縦面Command・着陸装置=既存WheelJoint+Pacejkaの流用)
     を先に進める。
