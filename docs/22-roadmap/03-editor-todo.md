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

- [ ] 失敗するE2Eテストを2本置く
  ①「編集→保存→読込→実行→state_hashが一致する」をジョイント・結合を含むシーン
  (D24車)で。②「scenes/の43本がDesc APIだけで構築できる」(特権アクセスの不在の証明)。
- [ ] 縮約監査スクリプトを作る
  コード中に自己申告された「縮約」(crates側262箇所、docs側114箇所)を機械集計する。
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
- [ ] Inspectorに Add Component とスキーマ駆動フォームを実装する
- [ ] 形状描画をShape記述に一本化する
  `demo/src/main.ts` に2箇所コピーされている形状パーサを1関数に集約。
  Capsuleを任意寸法で描画、Compound/ConvexMeshのメッシュ生成を追加。
  未知形状は黙って球を出さず警告を出す。
- [ ] 縦串①(ジョイント)の受け入れテストを緑にする
  D24相当の車をUIのみで組み立て、保存し、読み直して実行し、`state_hash` が
  既存のD24シーンJSON実行結果と一致することを確認する。
- [x] 回路素子4種をUIエディタに追加する(コンデンサ・インダクタ・ダイオード・DCモータ)
  `sim_em::Circuit`は既に7種そろっていたので、自由配線回路エディタ
  (`circuit_editor_*`)へUI+wasm境界の配線を追加。DCモーターは内部ノードを
  自動確保する`Circuit::add_nodes`を新設。wasm実ビルド+`tsc`+Playwright
  スモーク23件で検証済み。
- [x] QA報告の不具合9件を修正する
  ([2026-08-04-editor-qa.md](../reviews/2026-08-04-editor-qa.md) の既知不具合)。
  再現スクリプト(`demo/tests/qa/qa-defects.mjs`)が0/16→16/16 PASSへ転じたことを
  確認済み。Playwrightスモーク23件・Rust側テストも無傷。
- [ ] 結合14種を縦串②として配線する
- [ ] 環境と大気の場を縦串③として実装する
  重力ベクトル化、ISA標準大気(高度依存密度)、風の場
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
