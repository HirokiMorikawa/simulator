# フロントエンド 01. エディタ — Unity 風統合ワークベンチ

demo パッケージ(Vite + TypeScript + Three.js、
[00-foundation/05](../00-foundation/05-rust-wasm-platform.md) §6)。ブラウザ内で動く**単一の
統合エディタ**として構成する — Unity Editor のパネルレイアウトを参照点にし、Hierarchy /
Inspector / Scene view / Console / Timeline / Project の 6 領域からなる。

**設計原則**: エディタは **World API([20-integration/04](../20-integration/04-world-api.md))
のみ**を使う(エンティティ層と同じ制約 — エンジン内部への特権アクセスなし。必要な機能が
API に無ければ API 設計の欠陥として扱う)。描画は Three.js のリアルタイムプレビュー
(Phase D のパストレは別途 [17-rendering/01](../17-rendering/01-rendering-architecture.md))。
43 デモ([21-verification/03](../21-verification/03-demo-scenarios.md))は本エディタで開ける
「サンプルシーン」として供給する — デモごとの専用シェルは作らない。

## 1. パネル構成

Unity Editor のドッキングモデルを踏襲した 6 パネル + トップツールバー。ブラウザ 1 ページ内で
リサイズ・タブ化・切り離しができる(既定レイアウトを含めて 3 種のプリセットを持つ)。

```
┌──────────────────────────────────────────────────────────────────┐
│ Toolbar: [▶ ⏸ ⏭] [x1▽] [Scene ▽] [Layout ▽] [Hash: 0xa8b1…] [⚙] │
├─────────────────┬─────────────────────────────┬──────────────────┤
│ Hierarchy       │                             │ Inspector        │
│ - World Root    │       Scene View (3D)       │ [Selected obj]   │
│   └ Bodies      │                             │  Transform       │
│     ├ Ball_1    │       (Three.js viewport,   │  RigidBody       │
│     ├ Ramp      │        gizmos, grid,        │   mass: 1.0 kg   │
│   └ Joints      │        camera controls)     │   material: wood │
│   └ Circuits    │                             │  Contact         │
│   └ Fluids      │                             │  Probe (attach)  │
│   └ Probes      │                             │                  │
├─────────────────┴─────────────────────────────┴──────────────────┤
│ Timeline (replay scrubber) : ├──●───────┤    Play mode: [Rec][●] │
├──────────────────────────────────────────────────────────────────┤
│ Console                        │ Probe Graphs (docked)           │
│ [WARN] CFL violation @ t=1.2   │ [chart: BodyPos.y over time]    │
│ [INFO] Contact begin (Ball,Fl) │ [chart: KineticE / Total]       │
├─────────────────────────────────┴─────────────────────────────────┤
│ Project (bottom drawer): Scenes | Materials | Prefabs | Replays  │
└──────────────────────────────────────────────────────────────────┘
```

### 1.1 Hierarchy — シーングラフのツリー

- 現在ロード中の World の中身をツリー表示: Bodies / Joints / Circuits / Fluids /
  Probes / Frames(フレーム階層)/ Materials(参照)。
- ツリーは折り畳み可、複数選択可、右クリックでコンテキストメニュー
  (複製・削除・親付け・プレハブ化・アイソレート表示)。
- 選択は Scene View と Inspector に連動(Unity と同じ双方向)。
- 実行中(Play モード)の追加/削除も表示 — 実行中の直接編集は不可(§4 参照)。

### 1.2 Scene View — 3D ビューポート

- Three.js カメラ操作: 中クリック回転・右クリックパン・ホイールでズーム。
  ショートカットは W(移動)/E(回転)/R(スケール)/Q(選択のみ)。
- **Gizmo**: 選択中オブジェクトの Transform を直接ドラッグで編集
  (実行停止中のみ有効)。座標系は World / Local 切替可。
- **オーバーレイ表示**(切替可能):
  接触点・接触法線(矢印)/ 速度ベクトル / 力矢印 / 拘束線 /
  流体格子の速度場矢印 / 電磁場の等電位面 / 質量重心 / フレーム軸。
- **ピック**: クリックで body/joint/probe を選択。Alt-クリックで下層(重なった裏)を選択。
- **サブモード**: 選択オブジェクトが Circuit の場合は 2D 回路エディタサブモードに切替
  (§3)。フレーム(L5)にドリルイン可(選択フレームのローカル座標系で表示)。
- グリッド・スナップ(既定 10 cm、変更可)。

### 1.3 Inspector — Component ビュー

Unity の GameObject-Component モデルを物理エンジンに写像する:

- **選択オブジェクト**が上部に表示、その下に**Component 群**をアコーディオンで並べる。
- 各 Component は World API の `Desc` 型と 1:1 対応。編集は次ステップ先頭で
  Command として適用される(実行中は編集ロック — §4)。

代表 Component の例:

| Component | 対応 API | 主なフィールド |
|---|---|---|
| Transform | `RigidBodyDesc.position/rotation/frame` | Position、Rotation、Frame ID(フレーム階層) |
| RigidBody | `RigidBodyDesc` | Shape、Mass、Material、Body type(Dynamic/Static/Kinematic)、Collision group/mask |
| Joint | `JointDesc` | 種別(Ball/Hinge/Slider/…)・接続 Body ID・軸・制限・モータ |
| Circuit | `CircuitDesc` | ノード・素子リスト(サブエディタで編集、§3) |
| FluidRegion | `FluidDesc` | 領域 AABB・解像度・境界条件・温度 |
| Coupling | `CouplingDesc` | 種別・関連する Body/Fluid/Circuit 参照 |
| Probe | `Probe`([20-integration/04](../20-integration/04-world-api.md) §2.1) | Target(BodyPos.y など)、色、履歴長 |
| ApproximationBadge | `approximations()` の項目 | 近似の名前・出典・オフ可否 |

- 材料(`MaterialId`)は下部の Project → Materials からドラッグアンドドロップで割り当て。
- 「Add Component」ボタンで既存 API から作れる Desc を追加。API に無いものは追加できない
  (World API-only 制約の UI 側の担保)。

### 1.4 Timeline — 再生・リプレイのスクラバ

- 上部: 現在時刻・step 数・実効時間倍率・State Hash。
- 下部: リプレイのタイムライン(録画中は赤丸、録画済みは目盛りつき)。
  スクラブでスナップショットに巻き戻し([00-foundation/04](../00-foundation/04-architecture.md) §5、
  1 s 間隔リングバッファ)。
- **ブックマーク**: 任意時点にラベル付けし、後で戻れる。共有時にシーン JSON と一緒に出す。
- **Play モード バッジ**: Edit(編集可)/ Playing(実行中)/ Paused(一時停止・観察のみ)/
  Replaying(記録再生中)を明示。

### 1.5 Console — イベント・診断・ログ

- カテゴリ別タブ: All / Errors / Warnings / Info / Contacts / Events。
- `SolverDiagnostics` の発散警告・CFL 違反・シーンクラス(S/M/L)昇降格
  ([00-foundation/05](../00-foundation/05-rust-wasm-platform.md) §5.1)、`SolverDiverged`・
  `FuseBlown` などのイベントをフィルタ表示。
- 各ログエントリは発生ステップと発生源 ID を持ち、クリックで Timeline をその時刻へ、
  Scene View で発生源を選択、Hierarchy をハイライトする(Unity のログ→クリックで
  該当 GameObject 選択と同じ)。

### 1.6 Project — アセットブラウザ(下部ドロワー)

Unity の Project ウィンドウに相当する、シーン外のアセット群:

- **Scenes**: シーン JSON([20-integration/04](../20-integration/04-world-api.md) §3)。
  ダブルクリックで開く。デモ D1〜D43 のスターターシーンもここに並ぶ。
- **Materials**: MaterialDb のプリセット。派生(`extends`)で新規作成可。
  Inspector から D&D で割り当て。
- **Prefabs**: 再利用可能な Body / Joint / Circuit 組(自作シーンから右クリック → 
  「Prefab として保存」)。他シーンへドラッグで再利用。
- **Replays**: 録画済みリプレイ(シナリオ + 入力列 + チェックポイント、
  [20-integration/02](../20-integration/02-determinism-replay.md) §4.2)。
  ダブルクリックで Replay モードで開く。

## 2. トップツールバー

- **再生制御**: 再生(▶)/ 一時停止(⏸)/ 1 step(⏭)/ 指定 step 進める(数値入力)。
- **時間倍率スライダー**: 1/8×・1/4×・1/2×・1×・2×・4×・8×・…・128×
  ([20-integration/06](../20-integration/06-regime-switching.md) の通常レンジ)。
  性能予算に達しなければ実効倍率を隣に赤字で併記(静かに遅くしない)。
- **シーン選択ドロップダウン**: Project → Scenes の現在シーン切替。未保存編集がある場合は確認。
- **レイアウトプリセット**: Default / Physics-focus(Console 展開)/ Circuit-focus
  (回路サブモード + プローブ大)/ Astro(タイムライン重視)。
- **状態ハッシュ表示**: 常時 8 桁を表示、クリックでフル 64 bit ハッシュをコピー。
- **Settings**(⚙): レンダリング品質・グリッド・ショートカット・PRNG シードの一括変更。

## 3. 回路エディタ(Scene View のサブモード)

Unity の Shader Graph のように、**同じエディタ内のモード切替**として実装する。
Scene View のツールバーに `3D | Circuit | Frame` の 3 モードタブがあり、Circuit を選ぶと
選択中の Circuit ノードに対する 2D ブレッドボード画面に切り替わる。

- **配線モデル**: 回路はノードグラフ(部品 = 2 端子/多端子素子、配線 = 理想導体ノード結合)。
  内部表現は `CircuitDesc`([13-electromagnetism/02](../13-electromagnetism/02-circuits.md))と
  1:1 対応 — エディタ操作は `CircuitDesc` の編集であり、World の再構築(create_circuit)
  として適用する。
- **UI**: 2D 盤面(ブレッドボード風グリッド)。部品パレット(電池・抵抗・LED・コンデンサ・
  コイル・スイッチ・モーター端子・計器)からドラッグ配置、端子間をクリックで配線。
  配線はノード自動割当(接触した配線同士は同一ノードへ結合)。
- **計器**: 電圧計(`circuit_probe`)・電流計(素子の電流クエリ)は **Probe を貼り付ける**
  形で置き、Console の Probe Graphs パネルへ自動接続。
- **3D 世界との接続**: 盤面はシーン内の「工作台」オブジェクトに対応。モーター端子・
  導体棒等の電磁⇔力学素子([13-electromagnetism/05](../13-electromagnetism/05-em-mechanics-coupling.md))は
  3D 側の実体とリンク表示する(Circuit → 3D モード切替時に対応先をハイライト)。
- 検証との接続: エディタで組んだ回路がそのまま E3/E4/E5 の検証シーンになる(プリセット
  読込 = シーン JSON)。短絡・過電流はヒューズイベント(`FuseBlown`)で表現し、
  ジュール熱は熱ドメインへ(D19 の合格基準)。

## 4. Edit / Play モードと決定論

Unity と同じく **Edit モード**と **Play モード**を明確に分ける — 決定論・リプレイの
一貫性を守るための境界:

- **Edit モード**: シーンの直接編集が可能(Hierarchy 追加/削除、Inspector 直接編集、
  Scene View gizmo ドラッグ)。編集は「編集済みシーン JSON」の生成であり、
  Play を押した瞬間の状態が実行の初期条件になる。
- **Play モード**: 直接編集は不可。介入は全て **Command**(`Grab` / `MoveGrab` / `Release`
  / `SetMotorTarget` / `SetSwitch` / `SetHeatSource` / `ApplyForce` / …)としてキューに
  積まれ、次ステップ先頭で決定的順序で適用される
  ([20-integration/04](../20-integration/04-world-api.md) §1)。Command は全て
  Replay に記録される。
- **Pause**: Play モードの派生。時刻を進めずに観察のみ許可(Scene View の視点変更、
  Probe グラフのスクロール、Console の遡り)。編集は不可。
- **Play → Edit の戻り**: 実行後の状態を新規シーン JSON として保存する選択肢を出す
  (「編集後の状態」を新しいシナリオとして残せる)。

## 5. 予測 → 実験(オプションのミニパネル)

「検証して遊ぶ」の教材モード。Inspector 隣に切替可能なミニパネルとして提供する
(必須ではなく、シーン側の宣言でオンオフする):

- シーンに `prediction_prompts`(問い + 期待値の解析式)が定義されていると、
  Scene View に「予測を書く」オーバーレイが出る。
- ユーザーが数値/式を入力 → Play 開始 → Probe グラフに**予測線が重ね描き**される →
  停止条件(Probe の閾値)で自動一時停止 → 実測 vs 予測 vs 解析解の比較表を表示。
- 予測を書かずにそのまま Play することもできる — 自由モードが既定。

Unity のような広い操作性を優先し、この機能はデモ側のオプトインとする(全画面遷移を強制しない)。

## 6. シーン編集・スポーン

- **スポーンパレット**: Toolbar の「+」または Scene View の右クリックメニューから、
  形状(球・箱・カプセル)× 材質を選んでクリック配置(`create_body`)。
- **つかむ・投げる**: Play モードでマウスドラッグ = `Grab` / `MoveGrab` / `Release`
  Command(ソフト拘束)。実行中の介入はすべてコマンド経由でリプレイに記録される。
- **Undo / Redo**: Edit モードのみ。編集操作はシーン JSON の差分として保持。
- **保存・共有**: File → Export で「シーン JSON + Replay + ブックマーク」を単一ファイル
  としてエクスポート(数値は f64 を損なわない表現、
  [20-integration/02](../20-integration/02-determinism-replay.md) §4.1)。

## 7. ヘッドレスモードと CI

- エディタ本体とは別に、**ヘッドレスランナー**を同一 npm ワークスペースで提供する:
  シーン JSON + 入力列 + Probe assert を受け取り、Play を回して合格判定を返す。
- 43 デモ(D1〜D43)の合格基準はこのヘッドレスランナーで実行される
  (ネイティブ `cargo test` と wasm node 両方)。合格 = ヘッドレス Green + 目視チェック。
- エディタ UI 自体のテストは最小(スモーク)に留める — 検証の重心は物理側(World API 経由)
  にあり、UI は薄い写像を保つ。

## 8. 実装フェーズ対応

- Phase 0: Vite + TypeScript + Three.js 雛形、wasm 疎通、Scene View + Toolbar の最小版
  (既存の Phase 0 骨格に対応、[22-roadmap/01](../22-roadmap/01-phases.md))。
- Phase B・math ウェーブ後: Hierarchy + Inspector + Console + Timeline を最小実装
  (最初は Body/Transform だけ) → 各ウェーブで Component と Probe 対応を追加。
- P4: 回路サブモード(D19)。Pα: Frame サブモード・レジーム切替バッジ。
- Phase C: シーン編集・共有・Prefab の完成、全デモのヘッドレス CI。

## 9. 検証

- エディタが World API のみで実装されている(特権アクセス不在 —
  [20-integration/04](../20-integration/04-world-api.md) §5 と同じ検証)。
- 「Edit で編集 → Save → Load → Play → Replay 保存 → 別セッションで Load → 同一 State Hash」
  の E2E スモークがビット一致(階層 1、
  [20-integration/02](../20-integration/02-determinism-replay.md) §5)。
- 回路エディタ: 組んだ `CircuitDesc` の MNA 解が回路解析の手計算ケースと一致(E5 と同基準)。
- Gizmo ドラッグ結果が Inspector 数値と一致(座標系変換の往復精度)。
