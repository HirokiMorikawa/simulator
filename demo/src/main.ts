import * as THREE from "three";
import init, { WasmWorld } from "../pkg/sim_wasm.js";
import "./style.css";

// 統合エディタ(docs/23-frontend/01-editor.md)の骨格増分。
//
// **縮約実装の理由**: このファイルはドッキングレイアウトの骨格(§1)と、既存の
// Phase 0 デモ(床の上に箱が落ちて静止する)を Scene View パネルへ配線するところ
// までを扱う。Hierarchy/Inspector/Scene Viewは`WasmWorld`の複数ボディ列挙API
// (`body_count`/`body_label_at`/`body_position_at_f32`/`body_velocity_at_f32`)に
// 接続済みで、Hierarchyでのクリック・Scene Viewでのクリックピック(`THREE.
// Raycaster`、Alt-クリックで裏側を選択)のどちらからでも同じ選択状態
// (`selectBody`)を通じてInspectorが更新される(設計§1.2/§1.3が求める双方向
// 選択)。設計§4のEdit/Playモード分離を実装した——既定はEditモード(Unityと同じ
// 起動時挙動)で、Toolbarの Edit/Play トグルで切り替える。Editモードでは
// Scene Viewに選択中(かつ非静的)ボディのTranslate Gizmo(X赤/Y緑/Z青の
// 3軸ハンドル)が表示され、軸ハンドルをドラッグするとその軸方向にのみ
// `WasmWorld::set_body_position_at`(Commandキューを経由しない、
// `RigidBodySet`位置の直接書き換え)で位置を編集できる。同時にRotate Gizmo
// (X/Y/Z軸周りのリングハンドル)も表示され、リングをドラッグするとドラッグ
// 開始点からの画面上の角度差をそのままワールド軸周りの回転角として
// `WasmWorld::set_body_rotation_at`で適用する(Blenderのようなビュー平面
// トラックボールではなく単純な単一軸回転、スケールハンドルはこの2体デモに
// 意味のある対象が無いため未実装)。Gizmoドラッグ(位置のみ)開始のたびに
// 直前の位置/姿勢をUndoスタックへ積み、ToolbarのUndoボタン(Editモードかつ
// スタックが空でない場合のみ有効)で1件ずつ取り消せる(設計§6「Undo/Redo:
// Editモードのみ」、縮約実装によりシーンJSON差分ではなく単純な位置/姿勢スタック)。
// Redoボタンも実装済み(Undo時に取り消し前の値をRedoスタックへ積み、新規の
// Gizmoドラッグ開始でRedoスタックは破棄する標準的な意味論)。再生/ステップ/
// Nudgeボタンは無効化される(シミュレーションは
// 進行しない)。Playモードでは
// Gizmoは非表示になり、箱への直接ドラッグがCommandキュー経由
// (`Command::Grab/MoveGrab/Release`、`push_grab`/`push_move_grab`/
// `push_release`)の物理的な"つかむ"操作になり、再生/ステップ/Nudgeボタンが
// 有効になる。Shape/Materialは`world.body_shape_label_at`/
// `body_material_label_at`(スポーンパレットで追加したボディも含めて実際に
// クエリできる、`sim-wasm`側が構築時の値を覚えておく縮約実装)で取得する。
// Toolbarのスポーンパレット(設計§6「形状×材質を選んでクリック配置」)から
// 球/箱を追加できる(縮約実装によりカプセルは対象外、材質は代表的な4種のみ)。
// Scene Viewオーバーレイ(設計§1.2)は
// 選択中ボディの速度ベクトル(矢印)+ 接触点(既存の`World::contact_points`が
// 返す直近stepの接触点ワールド座標に小球マーカーを表示、この2体デモでは
// 着地/跳ね返りのたびに実際に現れる)+ 力(Nudgeボタンでキューに積む
// `Command::ApplyForce`の力ベクトルを、クリックした瞬間だけ短時間矢印表示——
// 一般の力の可視化(接触力・拘束反力の継続的な蓄積)には対応するWorld側の
// クエリが無いため対象外)を実装(いずれも切替可、Toolbarのチェックボックス。
// 拘束・流体場・フレーム軸のオーバーレイは対象外)。Consoleは
// `World::drain_events`(既存API)が返す実イベント(この2体デモでは箱の着地/
// 跳ね返りのたびに発生する`ContactStarted`/`ContactEnded`)をAll/Errors/
// Warnings/Infoタブでフィルタ表示し(設計§1.5)、イベント行(step番号を含む)を
// クリックするとその時刻に最も近いTimelineスナップショットへジャンプする
// (設計§1.5「クリックでTimeline/Scene Viewと連動」の時刻側、`jumpToStepRef`
// 経由でConsole/Scene View間を疎結合に配線)。Projectは静的な
// プレースホルダ内容のまま。TimelineはWorld::snapshot/restoreによる
// スナップショットリングバッファ(1s間隔・N=8面)でスクラブ・巻き戻しができ、
// 任意時点をブックマーク(`add_bookmark`/`restore_bookmark`、リングバッファの
// 退避を受けない別領域)として名前付きで保存・復元できる。Gizmoのスケール
// ハンドル・回転のUndo・残りのオーバーレイ種別(力・拘束・流体場・フレーム軸)・
// Commandキュー残り(SetMotorTarget/SetSwitch/SetHeatSource未配線)・
// 回路サブモードは全て後続増分。

const GRAVITY = 9.80665;
const DT = 1.0 / 120.0;
const INITIAL_HEIGHT = 10.0;
const BOX_HALF_EXTENT = 0.5;
const MAX_STEPS_PER_FRAME = 240;
const BODY_INDEX_GROUND = 0;
const BODY_INDEX_BOX = 1;

// スポーンパレット(設計docs/23-frontend/01-editor.md §6)で選べる材質。
// `sim_core::MaterialDb::standard`が持つ名前の一部(密度・反発特性が異なる
// ものを選び、着地の見た目が分かりやすいように)。
const SPAWN_MATERIALS = ["鋼(炭素鋼)", "アルミニウム", "木材(松)", "ゴム(天然)"];
const SPAWN_HEIGHT = 12.0;
const SPAWN_SPHERE_RADIUS = 0.4;
const SPAWN_BOX_HALF_EXTENT = 0.4;
const PENDULUM_PIVOT_HEIGHT = 6.0;
const PENDULUM_ARM_LENGTH = 2.0;

function setUpLayoutPresetSwitcher() {
  const app = document.getElementById("app")!;
  const select = document.getElementById("select-layout") as HTMLSelectElement;
  select.addEventListener("change", () => {
    app.dataset.layout = select.value;
  });
}

// Consoleパネル(設計docs/23-frontend/01-editor.md §1.5「SolverDiagnostics の
// 発散警告・…イベントをフィルタ表示」)。`world.drain_events_text`(既存の
// `World::drain_events`をそのまま使う、`sim_core::EventKind`)が返す
// `level::message`形式の行を実際のログとして追記し、タブでlevel別に絞り込む。
// 縮約実装の理由: クリックでTimeline/Scene Viewと連動させる機能・Contacts/
// Eventsタブ(設計は6タブだが本デモはAll/Errors/Warnings/Infoの4タブのみ)は
// 対象外。
const CONSOLE_LOG_CAPACITY = 200;

// クリック→時刻連動(設計docs/23-frontend/01-editor.md §1.5「クリックで
// Timeline/Scene Viewと連動」)。イベント行は`drain_events_text`が
// `step={N}`を埋め込むため、それを`data-step`属性に持たせておき、クリック時に
// `jumpToStepRef.current`(`setUpSceneView`がworld生成後に設定するコールバック)
// へその時刻へジャンプさせる。Console自体は世界(`world`)より先に構築されるため、
// 直接コールバックを渡せず可変の参照オブジェクト越しに後から配線する。
type JumpToStepRef = { current: ((step: number) => void) | null };

// Projectドロワー Materials タブ(設計§1.6「Materials: MaterialDbプリセット一覧」)向け。
// Console/jumpToStepRefと同じ理由(worldより先にパネルが構築される)で、
// 可変の参照オブジェクト越しに`setUpSceneView`がworld生成後にコールバックを配線する。
type MaterialProperties = {
  name: string;
  density: number;
  friction: number;
  restitution: number;
  specificHeat: number;
  conductivity: number;
};
type MaterialsRef = { current: (() => MaterialProperties[]) | null };

// Projectドロワー Circuit タブ(設計docs/23-frontend/01-editor.md §1.7「回路
// エディタサブモード」の縮約実装——自由配線ではなく、既存の固定トポロジー
// (分圧回路)のトポロジー表示+ライブ電圧読み取りのみ)。MaterialsRefと同じ理由
// (worldより先にパネルが構築される)で、可変の参照オブジェクト越しにコールバックを
// 後から配線する。
type CircuitRef = { current: (() => number) | null };

// Projectドロワー Scenes タブ(設計docs/23-frontend/01-editor.md §1.6「Scenes:
// シーンJSON…Export/Import」)。Exportは現在のボディ一覧を(Inspector表示と同じ
// 人間可読なJSONへ)書き出すのみで、`sim_world::Scenario`スキーマとは形式が違う
// (Exportは表示専用、往復(round-trip)は想定しない)。Import(本増分で追加)は
// 逆に`Scenario`スキーマ(ヘッドレスランナー・D1–D43のシーンJSONと同じもの)を
// 読み、`world.import_scene_json`(`World::append_scenario_bodies`のwasm薄い
// ラッパー)で現在のワールドへボディを追加する——Exportした自分自身のJSONを
// そのままImportし直せるわけではない(スキーマが異なるため)が、設計書・
// ヘッドレスランナーのテストが使うシーンJSONファイルをエディタへ読み込んで
// 視覚的に確認できるようにする、というワークストリームDの狙い(項目13)には
// この非対称な形で十分応える。MaterialsRefと同じ理由(worldより先にパネルが
// 構築される)で、可変の参照オブジェクト越しにコールバックを後から配線する。
type SceneBodyExport = {
  index: number;
  label: string;
  shape: string;
  material: string;
  position: [number, number, number];
  isStatic: boolean;
};
type SceneExportRef = { current: (() => SceneBodyExport[]) | null };
type SceneImportRef = { current: ((json: string) => number) | null };

// シーンギャラリー(設計docs/23-frontend/01-editor.md §1.6「Scenes」、ワーク
// ストリームD項目13「D1–D43の全シーンをエディタから読み込み可能にし、視覚的な
// 合否確認を可能にする」)。Importが既存ワールドへボディを「追加」するのに対し、
// ギャラリーは`WasmWorld::from_scene_json`(`World::from_scenario`ベース、
// fluids/thermal/circuit/astro/joints/probesの全セクションが効く)で**ワールド
// 自体を丸ごと差し替える**——D9(熱のみ)やD34(天体のみ)のような非力学
// シーンも正しく成立させるため。
type SceneGalleryRef = { current: ((json: string) => void) | null };

// Replay再生実行(設計docs/23-frontend/01-editor.md §1.6「Replays」)。記録済みの
// `commandLog`を、既定シーン(床+箱のみ、`WasmWorld`のコンストラクタが構築する
// もの)を持つ新規`WasmWorld`へステップ番号どおりに再送し、実際に同じ入力列を
// 再実行できることを検証する(縮約実装: Scene Viewでのライブな視覚的再生では
// なく、ヘッドレスに再実行して最終状態を報告するテキストベースの検証——
// `world`をライブで差し替えるとScene View/Hierarchy/Inspector等の大部分の
// 配線をworldへの可変参照へ作り直す必要があり影響範囲が大きいため見送った)。
// スポーン/Importでボディが増えているとスポーン由来のCommand(モーター等)を
// 再現できない(`sceneChanged`で明示)。
type ReplayVerifyResult = {
  totalSteps: number;
  commandCount: number;
  sceneChanged: boolean;
  finalStateHash: string;
  finalBoxPosition: [number, number, number];
  liveStateHash: string;
  liveBoxPosition: [number, number, number];
  matches: boolean;
};
type ReplayVerifyRef = { current: (() => ReplayVerifyResult) | null };

// 自由配線回路エディタ(設計docs/23-frontend/01-editor.md §6「回路エディタ
// サブモード」(D19)の縮約実装——専用のグラフィカルなノード配線UIではなく、
// Circuitタブのフォームベースの操作でノード/素子を追加していく形とした、
// `sim-wasm`の`circuit_editor_*`メソッド群のdoc参照)向けに、`world`への
// 直接アクセスを持たない`setUpProjectDrawer`から呼べるようにするための
// コールバック群。`setSwitchClosed`以外は全て即時反映(Command経由ではない、
// `circuit_editor_set_switch_closed`のdoc参照)。
type CircuitEditorRef = {
  current: {
    reset: (numNodes: number) => void;
    addResistor: (a: number, b: number, resistance: number) => void;
    addVoltageSource: (a: number, b: number, voltage: number) => void;
    addSwitch: (a: number, b: number, closed: boolean) => number;
    setSwitchClosed: (index: number, closed: boolean) => void;
    nodeVoltage: (node: number) => number;
  } | null;
};

// 自由配線回路が有効化されたかどうかの共有フラグ。`setUpProjectDrawer`(リセット
// ボタン押下時)がtrueにし、`setUpSceneView`の既存の固定デモ用「回路スイッチ
// (閉)」チェックボックスのハンドラがこれを見て自身を無効化する——リセット後は
// 固定デモの`circuit_switch_index`が新回路のスイッチ数を超えて無効になり得る
// (`circuit_editor_reset`のdoc参照)ため、パニックを避ける。
type CircuitFreeWiringState = { active: boolean };

// Prefabs(設計docs/23-frontend/01-editor.md §6「Prefabs: 再利用可能な
// Body/Joint/Circuit組(自作シーンから右クリック→「Prefabとして保存」)。
// 他シーンへドラッグで再利用」の縮約実装——Bodyの形状/材質のみ対象、
// Joint/Circuit組・ドラッグ&ドロップ・複数シーンをまたいだ永続化は対象外、
// ブラウザセッション内のみ保持)。`setUpProjectDrawer`から`world`への
// 直接アクセスを持たないため、他のRef同様コールバック経由で配線する。
type PrefabDefinition = { name: string; kind: string; params: number[]; material: string };
type PrefabRef = {
  current: {
    captureSelectedBody: () => Omit<PrefabDefinition, "name"> | null;
    spawn: (prefab: PrefabDefinition) => void;
  } | null;
};

// Import側のシーンJSONパース(`sim_world::scenario::ShapeJson`のJSON表現と同じ
// タグ付きオブジェクト形)。`world.import_scene_json`はボディの追加自体は行うが
// (返り値は追加件数のみ)、Scene Viewが各ボディに対応するThree.jsメッシュを
// 生成するには形状の種類・寸法が要る。`body_shape_label_at`は表示専用の整形
// 済み文字列(`Sphere(0.3000)`等)を返すのみで往復(round-trip)を想定していない
// ため、そこから数値を文字列パースするのではなく、Import時にJSがそもそも
// 持っている生のシーンJSONを(Rust側の`serde_json`とは独立に)そのまま
// `JSON.parse`して形状情報を読む。
type ImportedShapeJson =
  | { box: { half: [number, number, number] } }
  | { sphere: { radius: number } }
  | { plane: { normal: [number, number, number]; d: number } };
type ImportedBodyJson = { shape: ImportedShapeJson };
// 予測→実験ミニパネル(設計docs/23-frontend/01-editor.md §5)向け。
// `sim_world::scenario::PredictionPromptJson`のJSON表現と同じ形(物理には
// 影響しないメタデータのため、Rust側で検証済みの値としてではなく、Importに
// 渡した生のJSONをJSが独立に読む——他のImportedShapeJson等と同じ設計)。
type ImportedPredictionPromptJson = { question: string; probe_index: number; expected_value: number };
type ImportedScenarioJson = { bodies?: ImportedBodyJson[]; prediction_prompts?: ImportedPredictionPromptJson[] };

// シーンギャラリー(`SceneGalleryRef`のdoc参照)向けのアセット読み込み。
// リポジトリ直下`scenes/`(ヘッドレスランナーのテストが読むのと同じファイル、
// `crates/sim-world/src/scenario.rs`の`all_scenes_in_the_gallery_manifest_
// parse_and_run_for_sixty_steps`が壊れたアセットの出荷を防ぐ)を`import.meta.glob`
// でバンドルする(`vite.config.ts`の`server.fs.allow: [".."]`によりdemo/外の
// ディレクトリを参照できる、ビルド後の出力にも静的にバンドルされる)。
type SceneGalleryManifestEntry = {
  file: string;
  demo: string;
  title: string;
  description: string;
  domains: string[];
};
const sceneGalleryFiles = import.meta.glob("../../scenes/*.json", {
  query: "?raw",
  import: "default",
  eager: true,
}) as Record<string, string>;

function sceneGalleryFileContent(file: string): string | null {
  const entry = Object.entries(sceneGalleryFiles).find(([path]) => path.endsWith(`/${file}`));
  return entry ? entry[1] : null;
}

function sceneGalleryManifest(): SceneGalleryManifestEntry[] {
  const indexJson = sceneGalleryFileContent("index.json");
  if (!indexJson) return [];
  return (JSON.parse(indexJson) as { scenes: SceneGalleryManifestEntry[] }).scenes;
}

function setUpConsole(jumpToStepRef: JumpToStepRef): (eventsText: string) => void {
  const log = document.getElementById("console-log")!;
  const tabs = document.querySelectorAll<HTMLButtonElement>(".console-tab");
  let activeLevel = "all";

  function applyFilter() {
    for (const li of log.children) {
      const level = (li as HTMLElement).dataset.level;
      (li as HTMLElement).style.display = activeLevel === "all" || level === activeLevel ? "" : "none";
    }
  }

  tabs.forEach((tab) => {
    tab.addEventListener("click", () => {
      tabs.forEach((t) => t.classList.toggle("active", t === tab));
      activeLevel = tab.dataset.tab!;
      applyFilter();
    });
  });

  log.addEventListener("click", (event) => {
    const li = (event.target as HTMLElement).closest("li");
    const step = li?.dataset.step;
    if (step && jumpToStepRef.current) {
      jumpToStepRef.current(Number(step));
    }
  });

  const initial = document.createElement("li");
  initial.dataset.level = "info";
  initial.textContent = "[INFO] World起動 — SolverDiagnostics接続済み(ContactStarted/ContactEndedを表示)";
  log.appendChild(initial);

  return (eventsText: string) => {
    if (!eventsText) return;
    for (const line of eventsText.split("\n")) {
      const [level, message] = line.split("::", 2);
      const li = document.createElement("li");
      li.dataset.level = level;
      const stepMatch = message.match(/step=(\d+)/);
      if (stepMatch) {
        li.dataset.step = stepMatch[1];
        li.classList.add("console-entry-clickable");
        li.title = "クリックでその時点のTimelineへジャンプ";
      }
      li.textContent = `[${level.toUpperCase()}] ${message}`;
      log.appendChild(li);
    }
    while (log.children.length > CONSOLE_LOG_CAPACITY) {
      log.removeChild(log.firstChild!);
    }
    applyFilter();
    log.scrollTop = log.scrollHeight;
  };
}

// Hierarchyパネル(設計docs/23-frontend/01-editor.md §1.1「シーングラフツリー
// (Bodies/Joints/Circuits/Fluids/Probes/Frames)」)。`world.body_count`/
// `body_label_at`から実際のボディ一覧を組み立て、クリックで`onSelect`を呼ぶ
// (選択はInspector・Scene Viewと連動、設計が求める双方向選択)。戻り値の関数は
// Scene Viewピッキング(`onSelect`を経由せず見た目のハイライトだけ更新したい
// 場合)向けに、外部からハイライトだけを同期させる手段として公開する。
// Bodiesの兄弟としてJointsサブツリーも実装済み(振り子スポーンが追加した
// DistanceJointのみが対象、`constraint_anchor_points_at`で判定)。
// Circuits/Fluids/Probes/Framesはこれらのドメインが未接続のため未対応。
function setUpHierarchy(
  world: WasmWorld,
  onSelect: (index: number) => void,
  selectedFrameIndex: number,
  onSelectFrame: (frameIndex: number) => void,
): (index: number) => void {
  const tree = document.getElementById("hierarchy-tree")!;
  tree.innerHTML = "";
  const root = document.createElement("li");
  root.textContent = "World Root";
  const bodies = document.createElement("ul");
  bodies.className = "tree-nested";
  const bodyItem = document.createElement("li");
  bodyItem.textContent = "Bodies";
  const list = document.createElement("ul");
  list.className = "tree-nested";

  const count = world.body_count();
  const items: HTMLLIElement[] = [];

  function highlight(index: number) {
    items.forEach((it, i) => it.classList.toggle("selected", i === index));
  }

  for (let i = 0; i < count; i++) {
    const item = document.createElement("li");
    item.textContent = world.body_label_at(i);
    item.classList.add("tree-selectable");
    item.addEventListener("click", () => {
      highlight(i);
      onSelect(i);
    });
    items.push(item);
    list.appendChild(item);
  }
  highlight(BODY_INDEX_BOX);

  bodyItem.appendChild(list);
  bodies.appendChild(bodyItem);

  // Joints(設計§1.1「シーングラフツリー(Bodies/Joints/Circuits/Fluids/
  // Probes/Frames)」)。振り子スポーン(`spawn_pendulum`)が追加した
  // DistanceJointのみが対象(`constraint_anchor_points_at`が空でないボディ)。
  // クリックすると対応するボディを選択する(Joints専用のInspector表示は
  // 未実装のため、現状はBodies側の選択と同じ経路を再利用する)。
  const jointList = document.createElement("ul");
  jointList.className = "tree-nested";
  let jointCount = 0;
  for (let i = 0; i < count; i++) {
    if (world.constraint_anchor_points_at(i).length < 6) continue;
    jointCount += 1;
    const item = document.createElement("li");
    item.textContent = `DistanceJoint (${world.body_label_at(i)})`;
    item.classList.add("tree-selectable");
    item.addEventListener("click", () => {
      highlight(i);
      onSelect(i);
    });
    jointList.appendChild(item);
  }
  if (jointCount > 0) {
    const jointItem = document.createElement("li");
    jointItem.textContent = "Joints";
    jointItem.appendChild(jointList);
    bodies.appendChild(jointItem);
  }

  // Frames(設計§1.1「シーングラフツリー(...Frames)」、フレーム階層ドリルイン
  // UI)。index 0はROOT(常に存在、`add_child_frame`の既定の親候補だが、それ自体は
  // クリック可能な項目として列挙しない)。1以上の各フレームについて、
  // `frame_parent_index`(親子関係)から再帰的にネストした`<ul>`を組み立てる——
  // 「階層ドリルイン」の名のとおり、クリックしたフレームを選択すると
  // 「+ フレーム」ボタンがそのフレームの子として次のフレームを追加するようになる。
  const frameCount = world.frame_count();
  if (frameCount > 1) {
    function buildFrameSubtree(parentIndex: number): HTMLUListElement {
      const ul = document.createElement("ul");
      ul.className = "tree-nested";
      for (let i = 1; i < frameCount; i++) {
        if (world.frame_parent_index(i) !== parentIndex) continue;
        const item = document.createElement("li");
        item.textContent = `Frame ${i}`;
        item.classList.add("tree-selectable");
        item.classList.toggle("selected", i === selectedFrameIndex);
        item.addEventListener("click", (event) => {
          event.stopPropagation();
          onSelectFrame(i);
        });
        item.appendChild(buildFrameSubtree(i));
        ul.appendChild(item);
      }
      return ul;
    }
    const frameItem = document.createElement("li");
    frameItem.textContent = "Frames";
    frameItem.appendChild(buildFrameSubtree(0));
    bodies.appendChild(frameItem);
  }

  // Fluids(設計§1.1「シーングラフツリー(...Fluids)」)。個々の粒子や塊単位の
  // 選択・ドリルインまでは実装せず(SPH粒子は`RigidBodySet`のような個別ID体系を
  // 持たないため)、スポーンした水塊の数+総粒子数の概要表示のみとする
  // (縮約実装、`spawn_fluid_block`が複数回スポーンで水塊を追加できるように
  // なったことを受けての最小限のHierarchy反映)。
  const fluidSpawnCount = world.fluid_spawn_count();
  if (fluidSpawnCount > 0) {
    const fluidItem = document.createElement("li");
    fluidItem.textContent = `Fluids (${fluidSpawnCount}塊, ${world.fluid_particle_count()}粒子)`;
    bodies.appendChild(fluidItem);
  }

  root.appendChild(bodies);
  tree.appendChild(root);
  return highlight;
}

// Inspectorパネル(設計docs/23-frontend/01-editor.md §1.3)。選択中ボディの
// Shape/Material(`world.body_shape_label_at`/`body_material_label_at`、
// スポーンパレットで追加したボディも含めて実際にクエリできる)+ Transform
// (毎フレーム実データで更新、`updateInspectorTransformFields`)を表示する。
function renderInspectorFor(world: WasmWorld, index: number): void {
  const body = document.getElementById("inspector-body")!;
  const label = world.body_label_at(index);
  const staticBadge = world.body_is_static_at(index) ? ' <span class="badge">Static</span>' : "";
  body.innerHTML = `
    <div class="inspector-component">
      <h3>${label}${staticBadge}</h3>
      <div class="inspector-field"><span>Shape</span><span>${world.body_shape_label_at(index)}</span></div>
    </div>
    <div class="inspector-component">
      <h3>Transform</h3>
      <div class="inspector-field"><span>Position</span><span id="inspector-position">—</span></div>
      <div class="inspector-field"><span>Rotation</span><span id="inspector-rotation">—</span></div>
      <div class="inspector-field"><span>Velocity</span><span id="inspector-velocity">—</span></div>
    </div>
    <div class="inspector-component">
      <h3>RigidBody</h3>
      <div class="inspector-field"><span>Material</span><span>${world.body_material_label_at(index)}</span></div>
    </div>
  `;
}

function updateInspectorTransformFields(
  position: THREE.Vector3,
  rotation: THREE.Euler,
  velocity: THREE.Vector3,
): void {
  const positionField = document.getElementById("inspector-position");
  const rotationField = document.getElementById("inspector-rotation");
  const velocityField = document.getElementById("inspector-velocity");
  if (!positionField || !rotationField || !velocityField) return; // 選択切替の再描画中は一時的に無い。
  positionField.textContent = `${position.x.toFixed(3)}, ${position.y.toFixed(3)}, ${position.z.toFixed(3)}`;
  const toDeg = (rad: number) => THREE.MathUtils.radToDeg(rad).toFixed(1);
  rotationField.textContent = `${toDeg(rotation.x)}°, ${toDeg(rotation.y)}°, ${toDeg(rotation.z)}°`;
  velocityField.textContent = `${velocity.x.toFixed(3)}, ${velocity.y.toFixed(3)}, ${velocity.z.toFixed(3)}`;
}

// 入力列記録(設計docs/23-frontend/01-editor.md §1.6「Replays」、Command系の
// 実行記録)。Commandをキューへ積む離散的なUI操作(Nudge・Grab開始/Release・
// モーター目標切替・回路スイッチ切替)のたびに1件記録する。ヒーターは毎step
// 再送する継続的な内部動作(縮約実装、モジュールdoc参照)のため、切替の
// 瞬間だけを記録し、各subStepの再送そのものは記録しない(記録が単調に
// 膨れ上がるのを避ける、ユーザーの実際の操作単位に対応させる設計)。
//
// 各コマンドの実際のパラメータを判別共用体として構造化データのまま保持する
// (以前は呼び出し側が組み立てた表示用文字列`detail`のみを保持していたが、
// Replay再生実行(`replayCommandLogHeadless`)には実際のパラメータそのものが
// 要るため、この増分で構造化した——表示用の文字列化は`formatCommandLogDetail`
// に一本化)。
type CommandLogEntry =
  | {
      t: number;
      step: number;
      kind: "Grab";
      bodyIndex: number;
      targetX: number;
      targetY: number;
      targetZ: number;
    }
  | { t: number; step: number; kind: "Release"; bodyIndex: number }
  | { t: number; step: number; kind: "ApplyForce"; bodyIndex: number; fx: number; fy: number; fz: number }
  | {
      t: number;
      step: number;
      kind: "SetMotorTarget";
      bodyIndex: number;
      bodyLabel: string;
      targetAngle: number;
    }
  | { t: number; step: number; kind: "SetSwitch"; closed: boolean }
  | { t: number; step: number; kind: "SetHeatSource"; on: boolean; watts: number };
const commandLog: CommandLogEntry[] = [];

// `Omit<Union, K>`はTypeScriptでは判別共用体を分配せず、各variant固有の
// フィールド(targetX/fx/closed等)が消えてしまう既知の挙動のため、分配版の
// Omitを自前で定義する(`T extends any ? ... : never`は条件型がunion型の各
// メンバーへ分配して適用される性質を利用する標準的なパターン)。
type DistributiveOmit<T, K extends keyof T> = T extends unknown ? Omit<T, K> : never;

function pushCommandLog(world: WasmWorld, entry: DistributiveOmit<CommandLogEntry, "t" | "step">) {
  commandLog.push({ ...entry, t: world.time(), step: Number(world.step_count()) } as CommandLogEntry);
}

function formatCommandLogDetail(entry: CommandLogEntry): string {
  switch (entry.kind) {
    case "Grab":
      return `body=Box_1 anchor=(${entry.targetX.toFixed(3)},${entry.targetY.toFixed(3)},${entry.targetZ.toFixed(3)})`;
    case "Release":
      return "body=Box_1";
    case "ApplyForce":
      return `body=Box_1 force=(${entry.fx},${entry.fy},${entry.fz})`;
    case "SetMotorTarget":
      return `body=${entry.bodyLabel} theta_target=${entry.targetAngle.toFixed(3)}`;
    case "SetSwitch":
      return `closed=${entry.closed}`;
    case "SetHeatSource":
      return `toggled ${entry.on ? "on" : "off"} (${entry.watts}W)`;
  }
}

function setUpProjectDrawer(
  materialsRef: MaterialsRef,
  circuitRef: CircuitRef,
  sceneExportRef: SceneExportRef,
  sceneImportRef: SceneImportRef,
  replayVerifyRef: ReplayVerifyRef,
  circuitEditorRef: CircuitEditorRef,
  circuitFreeWiringState: CircuitFreeWiringState,
  prefabRef: PrefabRef,
  sceneGalleryRef: SceneGalleryRef,
) {
  const body = document.getElementById("project-body")!;
  const tabs = document.querySelectorAll<HTMLButtonElement>(".project-tab");
  const staticContentByTab: Record<string, string> = {};
  let circuitTabRefreshIntervalId: number | null = null;
  const prefabs: PrefabDefinition[] = [];

  // 自由配線回路エディタの状態(タブ切替でDOMは再構築されるが、実際に構築した
  // 回路自体はwasm側に残るため、この一覧はタブ再訪時の表示復元用)。
  type FreeWiringComponent =
    | { kind: "resistor"; a: number; b: number; resistance: number }
    | { kind: "voltage_source"; a: number; b: number; voltage: number }
    | { kind: "switch"; a: number; b: number; index: number; closed: boolean };
  let freeWiringNumNodes = 0;
  const freeWiringComponents: FreeWiringComponent[] = [];

  function renderScenesTab() {
    body.innerHTML = "";
    if (!sceneExportRef.current) {
      body.textContent = "Scenes: 読み込み中...";
      return;
    }

    // シーンギャラリー(`SceneGalleryRef`のdoc参照)。ヘッドレスランナー・
    // D1–D43のテストと同じシーンJSONをワンクリックで読み込む。ここでの
    // 読み込みは既存シーンへの追加(下のImport)ではなく、ワールド自体の
    // 差し替え(`from_scene_json`)である点が異なる。
    const galleryHeading = document.createElement("h4");
    galleryHeading.textContent = "シーンギャラリー(ワールドを差し替えて読み込み)";
    body.appendChild(galleryHeading);
    const galleryList = document.createElement("ul");
    galleryList.className = "scene-gallery-list";
    for (const entry of sceneGalleryManifest()) {
      const item = document.createElement("li");
      const label = document.createElement("div");
      label.textContent = `${entry.demo}: ${entry.title} — ${entry.description} [${entry.domains.join(", ")}]`;
      item.appendChild(label);
      const loadButton = document.createElement("button");
      loadButton.textContent = "読み込み";
      loadButton.dataset.sceneFile = entry.file;
      loadButton.addEventListener("click", () => {
        const json = sceneGalleryFileContent(entry.file);
        if (!json || !sceneGalleryRef.current) return;
        sceneGalleryRef.current(json);
      });
      item.appendChild(loadButton);
      galleryList.appendChild(item);
    }
    body.appendChild(galleryList);

    const note = document.createElement("p");
    note.textContent = "現在のシーン(ボディ一覧)をJSONへエクスポートする(人間可読な表示専用の形式)。";
    body.appendChild(note);

    const bodies = sceneExportRef.current();
    const exportButton = document.createElement("button");
    exportButton.textContent = `Export current scene (${bodies.length}件, JSON)`;
    exportButton.addEventListener("click", () => {
      const latestBodies = sceneExportRef.current ? sceneExportRef.current() : bodies;
      const blob = new Blob([JSON.stringify(latestBodies, null, 2)], { type: "application/json" });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = "scene.json";
      a.click();
      URL.revokeObjectURL(url);
    });
    body.appendChild(exportButton);

    const list = document.createElement("ul");
    for (const b of bodies) {
      const item = document.createElement("li");
      const [x, y, z] = b.position;
      item.textContent = `${b.label}: ${b.shape} / ${b.material} @ (${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)})${b.isStatic ? " [static]" : ""}`;
      list.appendChild(item);
    }
    body.appendChild(list);

    const importNote = document.createElement("p");
    importNote.textContent =
      "シーンJSON(sim_world::Scenarioスキーマ、ヘッドレスランナー・D1–D43のテストと同じ形式)を読み込み、現在のシーンへボディを追加する。";
    body.appendChild(importNote);

    const importInput = document.createElement("input");
    importInput.type = "file";
    importInput.accept = "application/json,.json";
    body.appendChild(importInput);

    const importStatus = document.createElement("p");
    body.appendChild(importStatus);

    importInput.addEventListener("change", () => {
      const file = importInput.files?.[0];
      if (!file || !sceneImportRef.current) return;
      file
        .text()
        .then((text) => {
          const count = sceneImportRef.current!(text);
          importStatus.textContent = `${count}件のボディを追加しました。`;
          renderScenesTab();
        })
        .catch((err: unknown) => {
          importStatus.textContent = `Import失敗: ${err}`;
        });
    });
  }

  function renderCircuitTab() {
    body.innerHTML = "";
    let circuitFreeWiringRefresh: (() => void) | null = null;
    const topology = document.createElement("pre");
    topology.className = "circuit-topology";
    topology.textContent = [
      "分圧回路(固定トポロジー):",
      "",
      "  Node1 (10V 電源) --[100Ω]-- Node2 --[200Ω]-- GND",
      "                                  |",
      "                              [スイッチ]",
      "                                  |",
      "                                 GND",
      "",
      circuitFreeWiringState.active
        ? "(自由配線回路への置き換え、またはシーンギャラリーからの読み込みにより、このデモ回路・チェックボックスは無効です)"
        : "スイッチの開閉は画面上部の「回路スイッチ(閉)」チェックボックスで操作する。",
    ].join("\n");
    body.appendChild(topology);

    const voltageLine = document.createElement("div");
    voltageLine.id = "circuit-tab-voltage";
    voltageLine.className = "inspector-field";
    body.appendChild(voltageLine);

    const switchCheckbox = document.getElementById("toggle-circuit-switch") as HTMLInputElement | null;

    // 自由配線回路エディタ(設計docs/23-frontend/01-editor.md §6「回路エディタ
    // サブモード」(D19)の縮約実装、`CircuitEditorRef`のdoc参照)。専用の
    // グラフィカルなノード配線UIではなく、ノード番号を直接指定するフォームで
    // 素子を追加していく形とした(Scene View内の別サブモードは大掛かりな
    // 追加実装が要るため見送った——`sim-em::Circuit`自体は任意のノード対応
    // 素子を既に自由に組める設計であり、本増分はそこへの配線が主眼)。
    const editorHeading = document.createElement("h4");
    editorHeading.textContent = "自由配線回路エディタ";
    body.appendChild(editorHeading);

    const resetForm = document.createElement("div");
    const nodeCountInput = document.createElement("input");
    nodeCountInput.type = "number";
    nodeCountInput.min = "2";
    nodeCountInput.value = "3";
    nodeCountInput.title = "GND(node 0)を含むノード総数";
    const resetButton = document.createElement("button");
    resetButton.textContent = "リセット(新規回路)";
    resetButton.addEventListener("click", () => {
      if (!circuitEditorRef.current) return;
      const numNodes = Math.max(2, Math.trunc(Number(nodeCountInput.value) || 2));
      circuitEditorRef.current.reset(numNodes);
      circuitFreeWiringState.active = true;
      if (switchCheckbox) {
        switchCheckbox.checked = false;
        switchCheckbox.disabled = true;
      }
      freeWiringNumNodes = numNodes;
      freeWiringComponents.length = 0;
      renderCircuitTab();
    });
    resetForm.append("ノード数(GND含む): ", nodeCountInput, resetButton);
    body.appendChild(resetForm);

    if (freeWiringNumNodes > 0) {
      const addForm = document.createElement("div");
      const aInput = document.createElement("input");
      aInput.type = "number";
      aInput.min = "0";
      aInput.value = "0";
      aInput.title = "ノードA(0=GND)";
      aInput.id = "circuit-editor-node-a";
      const bInput = document.createElement("input");
      bInput.type = "number";
      bInput.min = "0";
      bInput.value = "1";
      bInput.title = "ノードB(0=GND)";
      bInput.id = "circuit-editor-node-b";
      const kindSelect = document.createElement("select");
      kindSelect.id = "circuit-editor-kind";
      for (const [value, label] of [
        ["resistor", "抵抗 [Ω]"],
        ["voltage_source", "電圧源 [V] (A=正極)"],
        ["switch", "スイッチ"],
      ]) {
        const option = document.createElement("option");
        option.value = value;
        option.textContent = label;
        kindSelect.appendChild(option);
      }
      const valueInput = document.createElement("input");
      valueInput.type = "number";
      valueInput.value = "100";
      valueInput.id = "circuit-editor-value";
      const addButton = document.createElement("button");
      addButton.textContent = "素子を追加";
      addButton.addEventListener("click", () => {
        if (!circuitEditorRef.current) return;
        const a = Math.trunc(Number(aInput.value) || 0);
        const b = Math.trunc(Number(bInput.value) || 0);
        const value = Number(valueInput.value) || 0;
        if (kindSelect.value === "resistor") {
          circuitEditorRef.current.addResistor(a, b, value);
          freeWiringComponents.push({ kind: "resistor", a, b, resistance: value });
        } else if (kindSelect.value === "voltage_source") {
          circuitEditorRef.current.addVoltageSource(a, b, value);
          freeWiringComponents.push({ kind: "voltage_source", a, b, voltage: value });
        } else {
          const index = circuitEditorRef.current.addSwitch(a, b, false);
          freeWiringComponents.push({ kind: "switch", a, b, index, closed: false });
        }
        renderCircuitTab();
      });
      addForm.append("A: ", aInput, " B: ", bInput, " ", kindSelect, " 値: ", valueInput, addButton);
      body.appendChild(addForm);

      const componentList = document.createElement("ul");
      for (const c of freeWiringComponents) {
        const item = document.createElement("li");
        if (c.kind === "resistor") {
          item.textContent = `抵抗 ${c.a}-${c.b}: ${c.resistance}Ω`;
        } else if (c.kind === "voltage_source") {
          item.textContent = `電圧源 ${c.a}(+)-${c.b}(-): ${c.voltage}V`;
        } else {
          const switchCheckboxItem = document.createElement("input");
          switchCheckboxItem.type = "checkbox";
          switchCheckboxItem.checked = c.closed;
          switchCheckboxItem.addEventListener("change", () => {
            c.closed = switchCheckboxItem.checked;
            circuitEditorRef.current?.setSwitchClosed(c.index, c.closed);
          });
          item.textContent = `スイッチ ${c.a}-${c.b}: `;
          item.appendChild(switchCheckboxItem);
        }
        componentList.appendChild(item);
      }
      body.appendChild(componentList);

      const voltageTable = document.createElement("div");
      voltageTable.id = "circuit-editor-voltages";
      body.appendChild(voltageTable);

      function refreshFreeWiringVoltages() {
        if (!circuitEditorRef.current) return;
        const lines: string[] = [];
        for (let node = 0; node < freeWiringNumNodes; node++) {
          lines.push(`Node${node}: ${circuitEditorRef.current.nodeVoltage(node).toFixed(3)}V`);
        }
        voltageTable.textContent = lines.join(" / ");
      }
      refreshFreeWiringVoltages();
      circuitFreeWiringRefresh = refreshFreeWiringVoltages;
    } else {
      circuitFreeWiringRefresh = null;
    }

    function refresh() {
      if (!circuitRef.current) {
        voltageLine.textContent = "Node2電圧: 読み込み中...";
      } else {
        const voltage = circuitRef.current();
        const switchState = switchCheckbox?.checked ? "閉" : "開";
        voltageLine.textContent = circuitFreeWiringState.active
          ? "固定デモ回路は自由配線回路に置き換え済みです"
          : `Node2電圧: ${voltage.toFixed(3)} V (スイッチ: ${switchState})`;
      }
      circuitFreeWiringRefresh?.();
    }
    refresh();

    if (circuitTabRefreshIntervalId !== null) {
      window.clearInterval(circuitTabRefreshIntervalId);
    }
    circuitTabRefreshIntervalId = window.setInterval(refresh, 200);
  }

  function renderPrefabsTab() {
    body.innerHTML = "";
    const note = document.createElement("p");
    note.textContent = "選択中のボディの形状+材質をPrefabとして保存し、後で同じ形状+材質のボディを再スポーンできる。";
    body.appendChild(note);

    const saveForm = document.createElement("div");
    const nameInput = document.createElement("input");
    nameInput.type = "text";
    nameInput.placeholder = "Prefab名";
    const saveButton = document.createElement("button");
    saveButton.textContent = "選択中のボディをPrefabとして保存";
    saveButton.addEventListener("click", () => {
      if (!prefabRef.current) return;
      const captured = prefabRef.current.captureSelectedBody();
      if (!captured) return; // 未対応の形状(Plane等)は保存しない。
      const name = nameInput.value.trim() || `Prefab_${prefabs.length + 1}`;
      prefabs.push({ name, ...captured });
      nameInput.value = "";
      renderPrefabsTab();
    });
    saveForm.append(nameInput, saveButton);
    body.appendChild(saveForm);

    const list = document.createElement("ul");
    for (const prefab of prefabs) {
      const item = document.createElement("li");
      item.textContent = `${prefab.name} (${prefab.kind}, ${prefab.material}) `;
      const spawnButton = document.createElement("button");
      spawnButton.textContent = "スポーン";
      spawnButton.addEventListener("click", () => {
        prefabRef.current?.spawn(prefab);
      });
      item.appendChild(spawnButton);
      list.appendChild(item);
    }
    body.appendChild(list);
  }

  function renderMaterialsTab() {
    if (!materialsRef.current) {
      body.textContent = "Materials: 読み込み中...";
      return;
    }
    const props = materialsRef.current();
    body.innerHTML = "";
    const table = document.createElement("table");
    table.className = "materials-table";
    const header = table.insertRow();
    for (const label of ["Material", "density [kg/m^3]", "friction", "restitution", "specific heat [J/(kg・K)]", "conductivity [W/(m・K)]"]) {
      const th = document.createElement("th");
      th.textContent = label;
      header.appendChild(th);
    }
    for (const p of props) {
      const row = table.insertRow();
      for (const value of [
        p.name,
        p.density.toFixed(1),
        p.friction.toFixed(3),
        p.restitution.toFixed(3),
        p.specificHeat.toFixed(1),
        p.conductivity.toFixed(3),
      ]) {
        const td = row.insertCell();
        td.textContent = value;
      }
    }
    body.appendChild(table);
  }

  function renderReplaysTab() {
    body.innerHTML = "";
    const exportButton = document.createElement("button");
    exportButton.textContent = `Export (${commandLog.length}件, JSON)`;
    exportButton.addEventListener("click", () => {
      const blob = new Blob([JSON.stringify(commandLog, null, 2)], { type: "application/json" });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = "command_log.json";
      a.click();
      URL.revokeObjectURL(url);
    });
    body.appendChild(exportButton);

    const replayButton = document.createElement("button");
    replayButton.textContent = "▶ Replay実行(検証)";
    replayButton.title =
      "記録済みコマンドを既定シーン(床+箱)の新規Worldへ再送し、最終状態が現在のシーンと一致するか検証する";
    const replayStatus = document.createElement("p");
    replayButton.addEventListener("click", () => {
      if (!replayVerifyRef.current) return;
      const result = replayVerifyRef.current();
      const boxText = (p: [number, number, number]) => `(${p[0].toFixed(3)}, ${p[1].toFixed(3)}, ${p[2].toFixed(3)})`;
      replayStatus.textContent =
        `${result.commandCount}件のコマンドを${result.totalSteps}stepにわたって再生。` +
        `再生後Box_1位置=${boxText(result.finalBoxPosition)} (現在のシーン: ${boxText(result.liveBoxPosition)})。` +
        (result.sceneChanged
          ? "現在のシーンはスポーン/Importで初期状態から変更されているため一致は期待できません。"
          : result.matches
            ? "state_hashが一致——決定論的に同じ結果を再現しました。"
            : "state_hashが一致しませんでした。");
    });
    body.appendChild(replayButton);
    body.appendChild(replayStatus);

    const list = document.createElement("ul");
    for (const entry of commandLog) {
      const item = document.createElement("li");
      item.textContent = `[step ${entry.step}, t=${entry.t.toFixed(3)}s] ${entry.kind}: ${formatCommandLogDetail(entry)}`;
      list.appendChild(item);
    }
    body.appendChild(list);
  }

  function show(tab: string) {
    tabs.forEach((t) => t.classList.toggle("active", t.dataset.tab === tab));
    if (tab !== "circuit" && circuitTabRefreshIntervalId !== null) {
      window.clearInterval(circuitTabRefreshIntervalId);
      circuitTabRefreshIntervalId = null;
    }
    if (tab === "scenes") {
      renderScenesTab();
      return;
    }
    if (tab === "materials") {
      renderMaterialsTab();
      return;
    }
    if (tab === "replays") {
      renderReplaysTab();
      return;
    }
    if (tab === "circuit") {
      renderCircuitTab();
      return;
    }
    if (tab === "prefabs") {
      renderPrefabsTab();
      return;
    }
    body.textContent = staticContentByTab[tab] ?? "";
  }
  tabs.forEach((tab) => {
    tab.addEventListener("click", () => show(tab.dataset.tab!));
  });
  show("scenes");
}

// Probe Graphsパネル(設計docs/23-frontend/01-editor.md §1.4「Probeグラフ:
// シーン定義の観測量を時系列表示」)のデモ。複数系列(箱のy座標・箱の速さ)を
// 各系列独立の自動スケーリングで重ね描きする(値のレンジが大きく異なる系列
// (m単位のy座標 vs m/s単位の速さ)を同一軸に正規化すると見づらいため、
// 系列ごとに独立してmin/maxを取り0..canvas高さへ正規化する設計)。縮約実装の
// 理由: 対数軸・CSVエクスポート(design§1.4のフル仕様)は後続増分。
type ProbeSeries = { label: string; color: string; history: Float64Array };

function setUpProbeGraph(): (series: ProbeSeries[]) => void {
  const canvas = document.getElementById("probe-canvas") as HTMLCanvasElement;
  const ctx = canvas.getContext("2d")!;

  return (series: ProbeSeries[]) => {
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }
    ctx.clearRect(0, 0, w, h);
    ctx.font = "11px monospace";

    let legendY = 12;
    for (const s of series) {
      if (s.history.length < 2) continue;

      let min = Infinity;
      let max = -Infinity;
      for (const v of s.history) {
        if (v < min) min = v;
        if (v > max) max = v;
      }
      const range = max - min > 1e-9 ? max - min : 1.0;

      ctx.strokeStyle = s.color;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i = 0; i < s.history.length; i++) {
        const x = (i / (s.history.length - 1)) * w;
        const y = h - ((s.history[i] - min) / range) * h;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();

      ctx.fillStyle = s.color;
      ctx.fillText(`${s.label}: max=${max.toFixed(2)} min=${min.toFixed(2)}`, 4, legendY);
      legendY += 12;
    }
  };
}

async function setUpSceneView(
  updateProbeGraph: (series: ProbeSeries[]) => void,
  appendConsoleEntries: (eventsText: string) => void,
  jumpToStepRef: JumpToStepRef,
  materialsRef: MaterialsRef,
  circuitRef: CircuitRef,
  sceneExportRef: SceneExportRef,
  sceneImportRef: SceneImportRef,
  replayVerifyRef: ReplayVerifyRef,
  circuitEditorRef: CircuitEditorRef,
  circuitFreeWiringState: CircuitFreeWiringState,
  prefabRef: PrefabRef,
  sceneGalleryRef: SceneGalleryRef,
) {
  await init();
  let world = new WasmWorld(GRAVITY, DT, INITIAL_HEIGHT);
  circuitEditorRef.current = {
    reset: (numNodes: number) => world.circuit_editor_reset(numNodes),
    addResistor: (a, b, resistance) => world.circuit_editor_add_resistor(a, b, resistance),
    addVoltageSource: (a, b, voltage) => world.circuit_editor_add_voltage_source(a, b, voltage),
    addSwitch: (a, b, closed) => world.circuit_editor_add_switch(a, b, closed),
    setSwitchClosed: (index, closed) => world.circuit_editor_set_switch_closed(index, closed),
    nodeVoltage: (node) => world.circuit_node_voltage(node),
  };
  materialsRef.current = () =>
    SPAWN_MATERIALS.map((name) => {
      const [density, friction, restitution, specificHeat, conductivity] = world.material_properties_f64(name);
      return { name, density, friction, restitution, specificHeat, conductivity };
    });
  circuitRef.current = () => world.circuit_divider_voltage();
  sceneExportRef.current = () => {
    const count = world.body_count();
    const bodies: SceneBodyExport[] = [];
    for (let i = 0; i < count; i++) {
      const pos = world.body_position_at_f32(i);
      bodies.push({
        index: i,
        label: world.body_label_at(i),
        shape: world.body_shape_label_at(i),
        material: world.body_material_label_at(i),
        position: [pos[0], pos[1], pos[2]],
        isStatic: world.body_is_static_at(i),
      });
    }
    return bodies;
  };

  const host = document.getElementById("scene-view-canvas-host")!;

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x111111);

  const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 1000);
  camera.position.set(6, 4, 10);
  camera.lookAt(0, 3, 0);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  host.appendChild(renderer.domElement);

  function resize() {
    const w = host.clientWidth;
    const h = host.clientHeight;
    camera.aspect = w / Math.max(h, 1);
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  }
  window.addEventListener("resize", resize);
  resize();

  scene.add(new THREE.AmbientLight(0xffffff, 0.5));
  const sun = new THREE.DirectionalLight(0xffffff, 1.0);
  sun.position.set(5, 10, 5);
  scene.add(sun);

  const box = new THREE.Mesh(
    new THREE.BoxGeometry(BOX_HALF_EXTENT * 2, BOX_HALF_EXTENT * 2, BOX_HALF_EXTENT * 2),
    new THREE.MeshStandardMaterial({ color: 0xffa500 }),
  );
  scene.add(box);

  // 床(静的平面、`WasmWorld::new`が`BODY_INDEX_GROUND`として構築するコンクリート面)。
  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(20, 20),
    new THREE.MeshStandardMaterial({ color: 0x555555 }),
  );
  ground.rotation.x = -Math.PI / 2;
  scene.add(ground);

  // 全ボディのThree.jsメッシュ(bodyIndexで引ける、`render()`が毎フレーム
  // 位置/姿勢を反映させるために使う)。**残タスク完遂のシーンギャラリー増分**で
  // `box`/`ground`(既定シーン固有の決め打ち)とスポーンパレットで追加した
  // メッシュを統合した——シーンギャラリー(`loadScene`)が任意のシーンJSONを
  // 読み込むと、既定シーンには無いボディ構成(例: D11振り子は球1個のみで
  // index 1に「箱」は存在しない)になり得るため、「index 1は常に箱」という
  // 決め打ちを解く必要があった。
  const bodyMeshes = new Map<number, THREE.Mesh>();
  bodyMeshes.set(BODY_INDEX_GROUND, ground);
  bodyMeshes.set(BODY_INDEX_BOX, box);

  const grid = new THREE.GridHelper(20, 20, 0x444444, 0x222222);
  scene.add(grid);

  // Scene View オーバーレイ(設計docs/23-frontend/01-editor.md §1.2「速度ベクトル」、
  // 切替可)の最小デモ: 選択中ボディの速度ベクトルを矢印で表示する。縮約実装の
  // 理由: 接触点・力・拘束・流体場・フレーム軸のオーバーレイは対象外、速度のみ。
  const VELOCITY_OVERLAY_SCALE = 0.3; // 矢印長 = 速さ[m/s] * この係数[m]。
  const velocityArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(),
    1,
    0xffee00,
  );
  velocityArrow.visible = false;
  scene.add(velocityArrow);
  const velocityOverlayToggle = document.getElementById("toggle-velocity-overlay") as HTMLInputElement;
  const velocityDirection = new THREE.Vector3();

  // 接触点オーバーレイ(設計docs/23-frontend/01-editor.md §1.2「接触点」、切替可)。
  // `World::contact_points`(既存の`MechanicsSolver::last_manifolds`をそのまま
  // 使う)が返す直近stepの接触点ワールド座標に、小さな球マーカーを重ねて表示する。
  // 縮約実装の理由: マーカーの固定プール(`CONTACT_MARKER_POOL_SIZE`個)を使い回す
  // だけで、法線・貫入量の可視化(矢印やインパルス強度の色分け等)は対象外。
  const CONTACT_MARKER_POOL_SIZE = 8;
  const CONTACT_MARKER_RADIUS = 0.06;
  const contactOverlayToggle = document.getElementById("toggle-contact-overlay") as HTMLInputElement;
  const contactMarkerGeometry = new THREE.SphereGeometry(CONTACT_MARKER_RADIUS, 10, 8);
  const contactMarkerMaterial = new THREE.MeshBasicMaterial({ color: 0xff2222 });
  const contactMarkers: THREE.Mesh[] = [];
  for (let i = 0; i < CONTACT_MARKER_POOL_SIZE; i++) {
    const marker = new THREE.Mesh(contactMarkerGeometry, contactMarkerMaterial);
    marker.visible = false;
    scene.add(marker);
    contactMarkers.push(marker);
  }

  // 力オーバーレイ(設計docs/23-frontend/01-editor.md §1.2「力」、切替可)。
  // 縮約実装の理由: 一般の力の可視化(接触力・拘束反力等の継続的な蓄積量)には
  // `World`側の対応するクエリが無いため対象外。Nudgeボタン(`Command::
  // ApplyForce`、1step分だけ効く既知の力)をクリックした瞬間にだけ、その
  // 力ベクトルを短時間(`FORCE_OVERLAY_DURATION_MS`)矢印表示する——実際に
  // Commandとして適用される値をそのまま可視化するため、UIの見た目と実際の
  // 物理入力が一致する。
  const FORCE_OVERLAY_DURATION_MS = 500;
  const FORCE_OVERLAY_SCALE = 1.0 / 300_000.0; // 矢印長 = 力[N] * この係数[m]。
  const forceOverlayToggle = document.getElementById("toggle-force-overlay") as HTMLInputElement;
  const forceArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(), 1, 0xff8800);
  forceArrow.visible = false;
  scene.add(forceArrow);
  let forceOverlayHideAtMs = 0;

  // 拘束オーバーレイ(設計docs/23-frontend/01-editor.md §1.2「拘束」)。振り子
  // スポーン(`spawn_pendulum`)が追加したDistanceJointの2つのアンカー点
  // (固定ピボット・可動体側)を結ぶ線を毎フレーム描画する(`render()`内、
  // `constraintLines`ループ参照)。拘束を持たないボディ(球/箱スポーン・
  // 床・箱)は対象外。
  const constraintOverlayToggle = document.getElementById("toggle-constraint-overlay") as HTMLInputElement;

  // フレーム軸オーバーレイ + 階層ドリルインUI(設計docs/23-frontend/01-editor.md
  // §1.3「フレームサブモード」)。ROOTの子としてz軸まわりに自転するフレームを
  // 1つ既定で追加し(`World::add_frame`+`sim_core::FrameTree::step`が毎step
  // 自動的に回転を進める、`sim-world`側の増分参照)、以後Hierarchyの「Frames」
  // サブツリーで選択したフレームの子として「+ フレーム」ボタンからネストした
  // フレームを追加できる(`add_child_frame`——`add_rotating_frame`の一般化、
  // 親をROOT固定ではなく任意に選べる)。各フレームは`frame_world_position_f32`/
  // `frame_world_rotation_f32`(`FrameTree::transform_to_root`、階層を遡って
  // 合成したワールド姿勢)で毎フレーム更新する専用の`THREE.AxesHelper`を持つ。
  const frameOverlayToggle = document.getElementById("toggle-frame-overlay") as HTMLInputElement;
  const FRAME_AXIS_ANGULAR_VELOCITY = 1.0; // rad/s(任意値、回転が目視できる速さ)
  const FRAME_CHILD_OFFSET = 1.5; // 新規子フレームの親からのローカルオフセット(x軸方向)
  const frameAxesHelpers = new Map<number, THREE.AxesHelper>();
  let selectedFrameIndex = 0; // 0=ROOT(既定の親)。Hierarchyでフレームを選ぶと更新される。

  function createFrameAxesHelper(frameIndex: number) {
    const helper = new THREE.AxesHelper(2.0);
    scene.add(helper);
    frameAxesHelpers.set(frameIndex, helper);
  }

  const initialFrameIndex = world.add_child_frame(0, 0, 3, 0, FRAME_AXIS_ANGULAR_VELOCITY);
  createFrameAxesHelper(initialFrameIndex);
  selectedFrameIndex = initialFrameIndex;

  // 流体場オーバーレイ(設計docs/23-frontend/01-editor.md §1.3「流体場」の土台)。
  // 「+ 流体」ボタンでSPH流体塊(`world.spawn_fluid_block`)をスポーンすると、
  // 粒子位置をTHREE.Pointsで毎フレーム反映する(粒子数は固定なので、スポーン時に
  // 一度だけBufferAttributeを確保しrender()内で内容だけ更新する)。
  const fluidGeometry = new THREE.BufferGeometry();
  const fluidMaterial = new THREE.PointsMaterial({ color: 0x3399ff, size: 0.08 });
  const fluidPoints = new THREE.Points(fluidGeometry, fluidMaterial);
  fluidPoints.visible = false;
  scene.add(fluidPoints);
  let fluidPositionAttribute: THREE.BufferAttribute | null = null;

  function showForceOverlay(origin: THREE.Vector3, force: THREE.Vector3) {
    const magnitude = force.length();
    if (magnitude < 1e-6) return;
    forceArrow.position.copy(origin);
    forceArrow.setDirection(force.clone().divideScalar(magnitude));
    const length = magnitude * FORCE_OVERLAY_SCALE;
    forceArrow.setLength(Math.max(length, 0.3), Math.min(0.3, length * 0.3), Math.min(0.2, length * 0.2));
    forceOverlayHideAtMs = performance.now() + FORCE_OVERLAY_DURATION_MS;
  }

  // 正式なGizmo(設計docs/23-frontend/01-editor.md §1.2「Gizmo: 選択中オブジェクトの
  // Transformを直接ドラッグで編集」、§4「Scene View gizmo ドラッグ」)。縮約実装の
  // 理由: 移動(Translate)のみ(回転/スケールはこの2体デモに意味のある対象が無い
  // ——箱は軸並行のまま、床は静的平面——ため後続増分)。X(赤)/Y(緑)/Z(青)の3本の
  // 矢印を選択中ボディの位置に表示し、Editモードでのみ表示・操作可能(設計§4
  // 「Editモード…Scene View gizmo ドラッグ」「Playモード: 直接編集は不可」の
  // 境界どおり、Playモードでは非表示)。静的ボディ(床)を選択中は編集対象として
  // 意味が無いため非表示にする。
  const GIZMO_AXIS_LENGTH = 1.2;
  const GIZMO_HEAD_LENGTH = 0.28;
  const GIZMO_SHAFT_RADIUS = 0.03;
  const GIZMO_HEAD_RADIUS = 0.09;
  const GIZMO_AXES: { axis: THREE.Vector3; color: number; name: "x" | "y" | "z" }[] = [
    { axis: new THREE.Vector3(1, 0, 0), color: 0xff4444, name: "x" },
    { axis: new THREE.Vector3(0, 1, 0), color: 0x44ff44, name: "y" },
    { axis: new THREE.Vector3(0, 0, 1), color: 0x4488ff, name: "z" },
  ];
  const gizmoGroup = new THREE.Group();
  const gizmoHandleMeshes: { mesh: THREE.Mesh; axisName: "x" | "y" | "z" }[] = [];
  for (const { axis, color, name } of GIZMO_AXES) {
    const shaftLength = GIZMO_AXIS_LENGTH - GIZMO_HEAD_LENGTH;
    const material = new THREE.MeshBasicMaterial({ color });
    const shaft = new THREE.Mesh(
      new THREE.CylinderGeometry(GIZMO_SHAFT_RADIUS, GIZMO_SHAFT_RADIUS, shaftLength, 8),
      material,
    );
    shaft.position.y = shaftLength / 2;
    const head = new THREE.Mesh(new THREE.ConeGeometry(GIZMO_HEAD_RADIUS, GIZMO_HEAD_LENGTH, 8), material);
    head.position.y = shaftLength + GIZMO_HEAD_LENGTH / 2;
    const axisGroup = new THREE.Group();
    axisGroup.add(shaft, head);
    axisGroup.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis);
    gizmoGroup.add(axisGroup);
    gizmoHandleMeshes.push({ mesh: shaft, axisName: name }, { mesh: head, axisName: name });
  }
  gizmoGroup.visible = false;
  scene.add(gizmoGroup);

  // Rotate Gizmo(設計§1.2「Gizmo: 移動/回転/スケール」の回転部分)。X(赤)/Y(緑)/
  // Z(青)の3本のリングを選択中ボディの位置に表示し、Translate Gizmoと同じく
  // Editモードかつ非静的ボディ選択時のみ表示・操作可能。リングをドラッグすると、
  // ドラッグ開始点との画面上の角度差をそのままワールド軸周りの回転角として
  // 適用する単純な実装(Blenderのようなビュー平面トラックボールではなく、
  // 選択軸周りの単純回転)。
  const ROTATION_RING_RADIUS = 1.0;
  const ROTATION_RING_TUBE_RADIUS = 0.03;
  const rotationGizmoGroup = new THREE.Group();
  const rotationHandleMeshes: { mesh: THREE.Mesh; axisName: "x" | "y" | "z" }[] = [];
  for (const { axis, color, name } of GIZMO_AXES) {
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(ROTATION_RING_RADIUS, ROTATION_RING_TUBE_RADIUS, 8, 48),
      new THREE.MeshBasicMaterial({ color }),
    );
    // TorusGeometryは既定でXY平面上(穴の軸はZ)にあるため、穴の軸を`axis`へ合わせる。
    ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), axis);
    rotationGizmoGroup.add(ring);
    rotationHandleMeshes.push({ mesh: ring, axisName: name });
  }
  rotationGizmoGroup.visible = false;
  scene.add(rotationGizmoGroup);

  // Scale Gizmo(設計§1.2「Gizmo: 移動/回転/スケール」のスケール部分)。単一の
  // 立方体ハンドル(黄)を選択中ボディの対角オフセット位置に表示し、Translate/
  // Rotate Gizmoと同じくEditモードかつ非静的ボディ選択時のみ表示・操作可能。
  // 縮約実装の理由: Blenderのような軸別スケールではなく単一の一様スケールのみ
  // (X/Y/Zハンドル無し)。ドラッグ開始点からハンドルまでの画面上の距離と、
  // ドラッグ中の現在距離の比をそのまま一様スケール係数の相対変化として使う
  // (Rotate Gizmoの「角度差をそのまま回転角に使う」設計と同じ発想)。
  // `sim-wasm::set_body_scale_at`はスポーン時の寸法からの絶対倍率を受け取る
  // ため、フロント側で現在のスケール値(`currentScale`、既定1.0)をボディごとに
  // 保持し、ドラッグ開始時の値に距離比を掛けた絶対値を毎回渡す。
  const SCALE_HANDLE_OFFSET = 1.6;
  const SCALE_HANDLE_SIZE = 0.16;
  const SCALE_MIN = 0.2;
  const SCALE_MAX = 4.0;
  const scaleGizmoGroup = new THREE.Group();
  const scaleHandleMesh = new THREE.Mesh(
    new THREE.BoxGeometry(SCALE_HANDLE_SIZE, SCALE_HANDLE_SIZE, SCALE_HANDLE_SIZE),
    new THREE.MeshBasicMaterial({ color: 0xffff00 }),
  );
  scaleHandleMesh.position.set(SCALE_HANDLE_OFFSET, SCALE_HANDLE_OFFSET, SCALE_HANDLE_OFFSET);
  scaleGizmoGroup.add(scaleHandleMesh);
  scaleGizmoGroup.visible = false;
  scene.add(scaleGizmoGroup);
  const currentScale = new Map<number, number>();

  // モーターアーム(`Command::SetMotorTarget`の実証用、設計docs/20-integration/
  // 04-world-api.md §2「Commandキュー」)。`motorArmBodies`はスポーン時に登録
  // されたモーター付きボディのindex集合(それ以外のボディへ`set_motor_target_at`
  // を呼ぶとRust側がパニックするため、UI側でも呼び先を絞る)。
  const MOTOR_TARGET_LOW = 0.0;
  const MOTOR_TARGET_HIGH = Math.PI / 2;
  const motorArmBodies = new Set<number>();
  const currentMotorTarget = new Map<number, number>();

  let selectedBodyIndex = BODY_INDEX_BOX;
  function selectBody(index: number) {
    selectedBodyIndex = index;
    renderInspectorFor(world, index);
    highlightHierarchy(index);
    motorToggleButton.disabled = mode === "edit" || !motorArmBodies.has(index);
  }
  // フレーム階層ドリルインUI(Hierarchyの「Frames」サブツリー): クリックした
  // フレームを以後の「+ フレーム」ボタンの親候補にする(`selectedFrameIndex`は
  // このスコープの外側で宣言済み)。
  function selectFrame(frameIndex: number) {
    selectedFrameIndex = frameIndex;
    highlightHierarchy = setUpHierarchy(world, selectBody, selectedFrameIndex, selectFrame);
  }
  let highlightHierarchy = setUpHierarchy(world, selectBody, selectedFrameIndex, selectFrame);
  renderInspectorFor(world, selectedBodyIndex);

  // Scene Viewピック(設計docs/23-frontend/01-editor.md §1.2「クリックでbody/
  // joint/probeを選択。Alt-クリックで下層(重なった裏)を選択」)。Playモードでは
  // 箱を直接ドラッグして`Command::Grab/MoveGrab/Release`(設計§4「ドラッグ系は
  // Commandキュー経由」)でワールド座標の目標点へ剛にピン留めする物理的な
  // "つかむ"操作(移動量が閾値未満なら通常のクリック選択、pointerdown/move/upの
  // 3イベントで判別)。Editモードでは箱本体への直接ドラッグは行わず(設計§4の
  // 「Editモード…Scene View gizmo ドラッグ」どおりGizmo経由のみ)、Gizmoの
  // 軸ハンドルをドラッグすると`set_body_position_at`でその軸方向にのみ位置を
  // 直接書き換える。
  let pickables: { mesh: THREE.Object3D; bodyIndex: number }[] = [
    { mesh: ground, bodyIndex: BODY_INDEX_GROUND },
    { mesh: box, bodyIndex: BODY_INDEX_BOX },
  ];
  // 拘束オーバーレイ(設計docs/23-frontend/01-editor.md §1.2「拘束」)向けの
  // THREE.Line(振り子スポーンごとに1本、`world.constraint_anchor_points_at`が
  // 返す2点を毎フレーム反映する)。
  const constraintLines = new Map<number, THREE.Line>();
  const raycaster = new THREE.Raycaster();
  const pointerNdc = new THREE.Vector2();
  const dragPlane = new THREE.Plane();
  const dragPlaneHit = new THREE.Vector3();
  const cameraDirection = new THREE.Vector3();
  const DRAG_THRESHOLD_PX = 4;

  function updatePointerNdc(event: PointerEvent) {
    const rect = renderer.domElement.getBoundingClientRect();
    pointerNdc.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    pointerNdc.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  }

  function hitTest(event: PointerEvent, wantBack: boolean) {
    updatePointerNdc(event);
    raycaster.setFromCamera(pointerNdc, camera);
    const hits = raycaster.intersectObjects(pickables.map((p) => p.mesh));
    const hit = hits[wantBack && hits.length > 1 ? 1 : 0];
    if (!hit) return null;
    const picked = pickables.find((p) => p.mesh === hit.object);
    return picked ? { picked, worldPoint: hit.point } : null;
  }

  function hitGizmo(event: PointerEvent): "x" | "y" | "z" | null {
    if (!gizmoGroup.visible) return null;
    updatePointerNdc(event);
    raycaster.setFromCamera(pointerNdc, camera);
    const hits = raycaster.intersectObjects(gizmoHandleMeshes.map((h) => h.mesh));
    if (!hits.length) return null;
    const handle = gizmoHandleMeshes.find((h) => h.mesh === hits[0].object);
    return handle ? handle.axisName : null;
  }

  function hitRotationGizmo(event: PointerEvent): "x" | "y" | "z" | null {
    if (!rotationGizmoGroup.visible) return null;
    updatePointerNdc(event);
    raycaster.setFromCamera(pointerNdc, camera);
    const hits = raycaster.intersectObjects(rotationHandleMeshes.map((h) => h.mesh));
    if (!hits.length) return null;
    const handle = rotationHandleMeshes.find((h) => h.mesh === hits[0].object);
    return handle ? handle.axisName : null;
  }

  function hitScaleGizmo(event: PointerEvent): boolean {
    if (!scaleGizmoGroup.visible) return false;
    updatePointerNdc(event);
    raycaster.setFromCamera(pointerNdc, camera);
    return raycaster.intersectObject(scaleHandleMesh).length > 0;
  }

  const AXIS_VECTORS: Record<"x" | "y" | "z", THREE.Vector3> = {
    x: new THREE.Vector3(1, 0, 0),
    y: new THREE.Vector3(0, 1, 0),
    z: new THREE.Vector3(0, 0, 1),
  };

  let dragStartScreen: { x: number; y: number } | null = null;
  let pointerDownHit: ReturnType<typeof hitTest> = null;
  let pointerDownGizmoAxis: "x" | "y" | "z" | null = null;
  let pointerDownRotationAxis: "x" | "y" | "z" | null = null;
  let pointerDownScaleHit = false;
  let isDragging = false;
  let dragMode: "grab" | "gizmo" | "rotate" | "scale" | null = null;
  // 現在grab中のボディ(**残タスク完遂のシーンギャラリー増分**で`BODY_INDEX_BOX`
  // 決め打ちから一般化した——ピックしたボディをそのままgrab対象にする、
  // `push_grab`/`push_move_grab`/`push_release`のdoc参照)。
  let grabbedBodyIndex = BODY_INDEX_BOX;
  const gizmoAxisDir = new THREE.Vector3();
  const gizmoDragStartPosition = new THREE.Vector3();
  let gizmoDragStartScalar = 0;
  let rotateAxisDir = new THREE.Vector3();
  let rotateStartQuat = new THREE.Quaternion();
  let rotateCenterScreen = { x: 0, y: 0 };
  let rotateStartAngle = 0;
  let scaleCenterScreen = { x: 0, y: 0 };
  let scaleDragStartDistance = 0;
  let scaleDragStartValue = 1.0;

  // Undo/Redo(Editモードのみ、設計docs/23-frontend/01-editor.md §6「Undo/Redo:
  // Editモードのみ。編集操作はシーンJSONの差分として保持」)。縮約実装の理由:
  // シーンJSON差分ではなく、Gizmoドラッグ開始直前の位置/姿勢を積むだけの単純な
  // 2本のスタック(Undo/Redo双方)。ドラッグ(Translate/Rotateいずれも)開始の
  // たびに直前の値をUndoスタックへ1件積み、新規ドラッグはRedoスタックを破棄する
  // (標準的なUndo/Redoの意味論)。
  const EDIT_UNDO_STACK_CAPACITY = 20;
  type EditUndoEntry =
    | { bodyIndex: number; kind: "position"; position: THREE.Vector3 }
    | { bodyIndex: number; kind: "rotation"; rotation: THREE.Quaternion }
    | { bodyIndex: number; kind: "scale"; scale: number };
  const editUndoStack: EditUndoEntry[] = [];
  const editRedoStack: EditUndoEntry[] = [];

  function captureCurrentEntry(bodyIndex: number, kind: "position" | "rotation" | "scale"): EditUndoEntry {
    if (kind === "position") {
      const p = world.body_position_at_f32(bodyIndex);
      return { bodyIndex, kind: "position", position: new THREE.Vector3(p[0], p[1], p[2]) };
    }
    if (kind === "rotation") {
      const r = world.body_rotation_at_f32(bodyIndex);
      return { bodyIndex, kind: "rotation", rotation: new THREE.Quaternion(r[0], r[1], r[2], r[3]) };
    }
    return { bodyIndex, kind: "scale", scale: currentScale.get(bodyIndex) ?? 1.0 };
  }

  function projectToScreen(worldPos: THREE.Vector3): { x: number; y: number } {
    const ndc = worldPos.clone().project(camera);
    const rect = renderer.domElement.getBoundingClientRect();
    return {
      x: rect.left + ((ndc.x + 1) / 2) * rect.width,
      y: rect.top + ((1 - ndc.y) / 2) * rect.height,
    };
  }

  renderer.domElement.addEventListener("pointerdown", (event) => {
    dragStartScreen = { x: event.clientX, y: event.clientY };
    isDragging = false;
    dragMode = null;
    if (mode === "edit") {
      pointerDownGizmoAxis = hitGizmo(event);
      pointerDownRotationAxis = pointerDownGizmoAxis ? null : hitRotationGizmo(event);
      pointerDownScaleHit =
        !pointerDownGizmoAxis && !pointerDownRotationAxis && hitScaleGizmo(event);
      pointerDownHit =
        pointerDownGizmoAxis || pointerDownRotationAxis || pointerDownScaleHit
          ? null
          : hitTest(event, event.altKey);
    } else {
      pointerDownGizmoAxis = null;
      pointerDownRotationAxis = null;
      pointerDownScaleHit = false;
      pointerDownHit = hitTest(event, event.altKey);
    }
  });

  renderer.domElement.addEventListener("pointermove", (event) => {
    if (!dragStartScreen) return;
    const dx = event.clientX - dragStartScreen.x;
    const dy = event.clientY - dragStartScreen.y;
    if (!isDragging) {
      if (Math.hypot(dx, dy) < DRAG_THRESHOLD_PX) return;
      if (pointerDownRotationAxis) {
        isDragging = true;
        dragMode = "rotate";
        rotateAxisDir.copy(AXIS_VECTORS[pointerDownRotationAxis]);
        const r = world.body_rotation_at_f32(selectedBodyIndex);
        rotateStartQuat.set(r[0], r[1], r[2], r[3]);
        rotateCenterScreen = projectToScreen(rotationGizmoGroup.position);
        rotateStartAngle = Math.atan2(
          event.clientY - rotateCenterScreen.y,
          event.clientX - rotateCenterScreen.x,
        );
        editUndoStack.push({
          bodyIndex: selectedBodyIndex,
          kind: "rotation",
          rotation: rotateStartQuat.clone(),
        });
        if (editUndoStack.length > EDIT_UNDO_STACK_CAPACITY) editUndoStack.shift();
        editRedoStack.length = 0;
        undoButton.disabled = mode !== "edit";
        redoButton.disabled = true;
      } else if (pointerDownGizmoAxis) {
        isDragging = true;
        dragMode = "gizmo";
        gizmoAxisDir.copy(AXIS_VECTORS[pointerDownGizmoAxis]);
        gizmoDragStartPosition.copy(gizmoGroup.position);
        gizmoDragStartScalar = gizmoAxisDir.dot(gizmoDragStartPosition);
        editUndoStack.push({
          bodyIndex: selectedBodyIndex,
          kind: "position",
          position: gizmoDragStartPosition.clone(),
        });
        if (editUndoStack.length > EDIT_UNDO_STACK_CAPACITY) editUndoStack.shift();
        editRedoStack.length = 0;
        undoButton.disabled = mode !== "edit";
        redoButton.disabled = true;
        camera.getWorldDirection(cameraDirection);
        let planeNormal = cameraDirection
          .clone()
          .sub(gizmoAxisDir.clone().multiplyScalar(cameraDirection.dot(gizmoAxisDir)));
        if (planeNormal.lengthSq() < 1e-9) {
          planeNormal = new THREE.Vector3().crossVectors(gizmoAxisDir, new THREE.Vector3(0, 1, 0));
          if (planeNormal.lengthSq() < 1e-9) {
            planeNormal.crossVectors(gizmoAxisDir, new THREE.Vector3(1, 0, 0));
          }
        }
        planeNormal.normalize();
        dragPlane.setFromNormalAndCoplanarPoint(planeNormal, gizmoDragStartPosition);
      } else if (pointerDownScaleHit) {
        isDragging = true;
        dragMode = "scale";
        scaleCenterScreen = projectToScreen(scaleGizmoGroup.position);
        scaleDragStartDistance = Math.max(
          Math.hypot(event.clientX - scaleCenterScreen.x, event.clientY - scaleCenterScreen.y),
          10,
        );
        scaleDragStartValue = currentScale.get(selectedBodyIndex) ?? 1.0;
        editUndoStack.push({
          bodyIndex: selectedBodyIndex,
          kind: "scale",
          scale: scaleDragStartValue,
        });
        if (editUndoStack.length > EDIT_UNDO_STACK_CAPACITY) editUndoStack.shift();
        editRedoStack.length = 0;
        undoButton.disabled = mode !== "edit";
        redoButton.disabled = true;
      } else {
        if (mode !== "play" || !pointerDownHit) return;
        isDragging = true;
        dragMode = "grab";
        grabbedBodyIndex = pointerDownHit.picked.bodyIndex;
        selectBody(grabbedBodyIndex);
        camera.getWorldDirection(cameraDirection);
        dragPlane.setFromNormalAndCoplanarPoint(cameraDirection, pointerDownHit.worldPoint);
        const p = world.body_position_at_f32(grabbedBodyIndex);
        world.push_grab(grabbedBodyIndex, p[0], p[1], p[2]);
        pushCommandLog(world, { kind: "Grab", bodyIndex: grabbedBodyIndex, targetX: p[0], targetY: p[1], targetZ: p[2] });
      }
    }
    if (dragMode === "rotate") {
      // 回転はドラッグ平面ではなく画面上の角度差(中心=Gizmo位置の画面投影)を
      // そのままワールド軸周りの回転角として使う(モジュールdoc参照)。
      const currentAngle = Math.atan2(
        event.clientY - rotateCenterScreen.y,
        event.clientX - rotateCenterScreen.x,
      );
      const deltaAngle = currentAngle - rotateStartAngle;
      const deltaQuat = new THREE.Quaternion().setFromAxisAngle(rotateAxisDir, deltaAngle);
      const newQuat = deltaQuat.multiply(rotateStartQuat);
      world.set_body_rotation_at(selectedBodyIndex, newQuat.x, newQuat.y, newQuat.z, newQuat.w);
      return;
    }
    if (dragMode === "scale") {
      // ドラッグ開始点からハンドルまでの画面上の距離との比を、そのまま
      // ドラッグ開始時点のスケール値への相対倍率として使う(モジュールdoc参照)。
      const currentDistance = Math.hypot(
        event.clientX - scaleCenterScreen.x,
        event.clientY - scaleCenterScreen.y,
      );
      const factor = Math.min(
        Math.max(scaleDragStartValue * (currentDistance / scaleDragStartDistance), SCALE_MIN),
        SCALE_MAX,
      );
      world.set_body_scale_at(selectedBodyIndex, factor);
      currentScale.set(selectedBodyIndex, factor);
      renderInspectorFor(world, selectedBodyIndex);
      return;
    }
    updatePointerNdc(event);
    raycaster.setFromCamera(pointerNdc, camera);
    if (!raycaster.ray.intersectPlane(dragPlane, dragPlaneHit)) return;
    if (dragMode === "gizmo") {
      const t = gizmoAxisDir.dot(dragPlaneHit);
      const delta = t - gizmoDragStartScalar;
      const newPos = gizmoDragStartPosition.clone().addScaledVector(gizmoAxisDir, delta);
      world.set_body_position_at(selectedBodyIndex, newPos.x, newPos.y, newPos.z);
    } else if (dragMode === "grab") {
      world.push_move_grab(grabbedBodyIndex, dragPlaneHit.x, dragPlaneHit.y, dragPlaneHit.z);
    }
  });

  renderer.domElement.addEventListener("pointerup", () => {
    if (isDragging) {
      if (dragMode === "grab") {
        world.push_release(grabbedBodyIndex);
        pushCommandLog(world, { kind: "Release", bodyIndex: grabbedBodyIndex });
      }
    } else if (pointerDownHit) {
      selectBody(pointerDownHit.picked.bodyIndex);
    }
    isDragging = false;
    dragMode = null;
    dragStartScreen = null;
    pointerDownHit = null;
    pointerDownGizmoAxis = null;
    pointerDownRotationAxis = null;
    pointerDownScaleHit = false;
  });

  const hud = document.getElementById("hud")!;
  const hashDisplay = document.getElementById("hash-display")!;
  const timelineTime = document.getElementById("timeline-time")!;
  const timelineStep = document.getElementById("timeline-step")!;
  const playButton = document.getElementById("btn-play") as HTMLButtonElement;
  const stepButton = document.getElementById("btn-step") as HTMLButtonElement;
  const nudgeButton = document.getElementById("btn-nudge") as HTMLButtonElement;
  const motorToggleButton = document.getElementById("btn-motor-toggle") as HTMLButtonElement;
  // 分圧回路のスイッチ(`Command::SetSwitch`実証用、設計docs/20-integration/
  // 04-world-api.md §2「Commandキュー」)。`WasmWorld::new`が既に分圧回路
  // (電源10V→100Ω→分圧点、分圧点→200Ω→GND、分圧点↔GNDにスイッチ)を構築
  // 済みなので、フロントエンドは切替のみを担う。HUDの`circuit V`行が
  // 実際の分圧点電圧(開: 約6.67V、閉: 約0V)を毎フレーム表示する。
  const circuitSwitchToggle = document.getElementById("toggle-circuit-switch") as HTMLInputElement;
  // ヒーター(`Command::SetHeatSource`実証用)。モジュールdoc「1step分だけ効く」
  // 縮約セマンティクスのとおり、継続加熱するには毎stepの直前に再度
  // `push_heat_source`を呼ぶ必要がある(`frame()`ループ内、`world.step()`の
  // 直前で呼ぶ)。HUDの`heater T`行が熱ノードの現在温度(ニュートン冷却あり、
  // 時定数τ=10s)を毎フレーム表示する。
  const HEATER_WATTS = 2000.0;
  const heaterToggle = document.getElementById("toggle-heater") as HTMLInputElement;
  const undoButton = document.getElementById("btn-undo") as HTMLButtonElement;
  const redoButton = document.getElementById("btn-redo") as HTMLButtonElement;

  // 時間倍率(設計docs/23-frontend/01-editor.md §1.1「Toolbar: 時間倍率スライダー」)。
  // dt自体は固定(`DT`、物理の決定論性はステップ幅に依存するため)のまま、
  // 1描画フレームあたりに進める実時間(`frameSeconds`)をこの倍率でスケールする
  // ことで、シミュレーションの見かけの再生速度のみを変える(縮約実装:
  // スライダーではなくセレクトボックス、離散値×0.5/×1/×2/×5のみ)。
  const timescaleSelect = document.getElementById("select-timescale") as HTMLSelectElement;
  let timeScale = Number.parseFloat(timescaleSelect.value);
  timescaleSelect.addEventListener("change", () => {
    timeScale = Number.parseFloat(timescaleSelect.value);
  });

  // Edit/Play モードの分離(設計§4「Edit モード: シーンの直接編集が可能…Play を
  // 押した瞬間の状態が実行の初期条件になる」「Play モード: 直接編集は不可。
  // 介入は全て Command」)。既定はEditモード(Unityと同じ起動時挙動、Playを
  // 押すまでシミュレーションは進まない)。
  type Mode = "edit" | "play";
  let mode: Mode = "edit";
  let playing = false;
  const modeEditButton = document.getElementById("btn-mode-edit") as HTMLButtonElement;
  const modePlayButton = document.getElementById("btn-mode-play") as HTMLButtonElement;

  function setMode(next: Mode) {
    mode = next;
    playing = next === "play";
    playButton.textContent = playing ? "⏸" : "▶";
    playButton.disabled = mode === "edit";
    stepButton.disabled = mode === "edit";
    nudgeButton.disabled = mode === "edit";
    motorToggleButton.disabled = mode === "edit" || !motorArmBodies.has(selectedBodyIndex);
    // 自由配線回路エディタでリセット済みなら、モード切替に関わらず無効のまま
    // (`circuitFreeWiringState`のdoc参照——`circuit_switch_index`が新回路の
    // スイッチ数を超えて無効になり得るため、再有効化してはならない)。
    circuitSwitchToggle.disabled = mode === "edit" || circuitFreeWiringState.active;
    heaterToggle.disabled = mode === "edit";
    undoButton.disabled = mode !== "edit" || editUndoStack.length === 0;
    redoButton.disabled = mode !== "edit" || editRedoStack.length === 0;
    modeEditButton.classList.toggle("active", mode === "edit");
    modePlayButton.classList.toggle("active", mode === "play");
  }
  modeEditButton.addEventListener("click", () => setMode("edit"));
  modePlayButton.addEventListener("click", () => setMode("play"));
  setMode("edit");

  function applyEditEntry(entry: EditUndoEntry) {
    if (entry.kind === "position") {
      world.set_body_position_at(entry.bodyIndex, entry.position.x, entry.position.y, entry.position.z);
    } else if (entry.kind === "rotation") {
      world.set_body_rotation_at(
        entry.bodyIndex,
        entry.rotation.x,
        entry.rotation.y,
        entry.rotation.z,
        entry.rotation.w,
      );
    } else {
      world.set_body_scale_at(entry.bodyIndex, entry.scale);
      currentScale.set(entry.bodyIndex, entry.scale);
      renderInspectorFor(world, entry.bodyIndex);
    }
  }

  undoButton.addEventListener("click", () => {
    if (mode !== "edit") return;
    const entry = editUndoStack.pop();
    if (!entry) return;
    editRedoStack.push(captureCurrentEntry(entry.bodyIndex, entry.kind));
    if (editRedoStack.length > EDIT_UNDO_STACK_CAPACITY) editRedoStack.shift();
    applyEditEntry(entry);
    undoButton.disabled = editUndoStack.length === 0;
    redoButton.disabled = editRedoStack.length === 0;
    render();
  });

  redoButton.addEventListener("click", () => {
    if (mode !== "edit") return;
    const entry = editRedoStack.pop();
    if (!entry) return;
    editUndoStack.push(captureCurrentEntry(entry.bodyIndex, entry.kind));
    if (editUndoStack.length > EDIT_UNDO_STACK_CAPACITY) editUndoStack.shift();
    applyEditEntry(entry);
    redoButton.disabled = editRedoStack.length === 0;
    undoButton.disabled = editUndoStack.length === 0;
    render();
  });

  // スポーンパレット(設計docs/23-frontend/01-editor.md §6「形状×材質を選んで
  // クリック配置(`create_body`)」)。Toolbarの「+ 球」/「+ 箱」ボタンで、床の
  // 中心付近・上空(`SPAWN_HEIGHT`)へ新規ボディを配置する。落下位置が重ならない
  // よう、配置のたびにx/zを少しずつずらす(スポーン回数から決定的に算出)。
  const spawnMaterialSelect = document.getElementById("select-spawn-material") as HTMLSelectElement;
  for (const material of SPAWN_MATERIALS) {
    const option = document.createElement("option");
    option.value = material;
    option.textContent = material;
    spawnMaterialSelect.appendChild(option);
  }

  // `sceneBaseBodyCount`: 現在の`world`が最初から持つボディ数(既定シーンは
  // 床+箱の2体、`loadScene`でギャラリーシーンへ差し替えた後はそのシーンの
  // ボディ数)。以後のスポーンパレット操作による「これまでのスポーン数」の
  // 基準点として使う。
  let sceneBaseBodyCount = 2;
  function nextSpawnPosition(): { x: number; z: number } {
    const n = world.body_count() - sceneBaseBodyCount; // これまでのスポーン数
    const angle = n * 2.4; // 黄金角に近い値、重ならないようばらけさせる
    const radius = 1.5 + n * 0.3;
    return { x: Math.cos(angle) * radius, z: Math.sin(angle) * radius };
  }

  function addSpawnedMesh(bodyIndex: number, mesh: THREE.Mesh) {
    scene.add(mesh);
    pickables.push({ mesh, bodyIndex });
    bodyMeshes.set(bodyIndex, mesh);
    highlightHierarchy = setUpHierarchy(world, selectBody, selectedFrameIndex, selectFrame);
    selectBody(bodyIndex);
  }

  // シーンJSON Import(`SceneImportRef`のdoc参照)。`world.import_scene_json`が
  // 検証+ボディ追加を行い、追加件数を返す(検証エラーはJS例外として投げられる、
  // 呼び出し側のProjectドロワーが`.catch`で拾う)。形状ごとのメッシュ生成は
  // Importに渡した生のシーンJSONを(Rust側とは独立に)自前で`JSON.parse`して
  // 読む(`ImportedShapeJson`のdoc参照)。Plane形状は既存のスポーンパレットに
  // 対応物が無いため、大きな平板メッシュで代用し、位置は物理的な平面の定義
  // (`normal・p=d`、剛体の`transform.position`ではなく`normal`/`d`自体)に
  // 合わせて`normal.scale(d)`に置く。
  // 予測→実験ミニパネル(設計docs/23-frontend/01-editor.md §5、`prediction_prompts`
  // を宣言したシーンJSONをImportした場合のみ表示するオプトイン——シーン側に
  // 無ければ既定でスキップされる、モジュール冒頭の設計ノート「全画面遷移を強制
  // しない」に対応)。停止条件による自動一時停止・Probeグラフへの予測線重ね描き
  // は対象外(縮約実装)——予測入力+実測/予測/解析解の比較表のみ実装する。
  let currentPredictionPrompts: ImportedPredictionPromptJson[] = [];
  const predictionPanel = document.getElementById("prediction-panel")!;

  function renderPredictionPanel() {
    predictionPanel.innerHTML = "";
    if (currentPredictionPrompts.length === 0) {
      predictionPanel.style.display = "none";
      return;
    }
    predictionPanel.style.display = "block";
    const heading = document.createElement("h3");
    heading.textContent = "予測 → 実験";
    predictionPanel.appendChild(heading);
    currentPredictionPrompts.forEach((prompt, i) => {
      const questionLine = document.createElement("div");
      questionLine.className = "inspector-field";
      questionLine.textContent = prompt.question;
      predictionPanel.appendChild(questionLine);

      const guessRow = document.createElement("div");
      guessRow.className = "inspector-field";
      const guessLabel = document.createElement("span");
      guessLabel.textContent = "あなたの予測";
      const guessInput = document.createElement("input");
      guessInput.type = "number";
      guessInput.id = `prediction-guess-${i}`;
      guessRow.append(guessLabel, guessInput);
      predictionPanel.appendChild(guessRow);

      const resultLine = document.createElement("div");
      resultLine.className = "inspector-field";
      resultLine.id = `prediction-result-${i}`;
      predictionPanel.appendChild(resultLine);
    });
  }

  function updatePredictionResults() {
    currentPredictionPrompts.forEach((prompt, i) => {
      const resultLine = document.getElementById(`prediction-result-${i}`);
      if (!resultLine) return;
      const guessInput = document.getElementById(`prediction-guess-${i}`) as HTMLInputElement | null;
      const actual = world.imported_probe_value_at(prompt.probe_index);
      const guessText = guessInput?.value ? Number(guessInput.value).toFixed(3) : "(未入力)";
      resultLine.textContent = `実測=${actual.toFixed(3)} / 予測=${guessText} / 解析解=${prompt.expected_value.toFixed(3)}`;
    });
  }

  sceneImportRef.current = (json: string) => {
    const count = world.import_scene_json(json);
    const parsed = JSON.parse(json) as ImportedScenarioJson;
    const bodies = parsed.bodies ?? [];
    currentPredictionPrompts = parsed.prediction_prompts ?? [];
    renderPredictionPanel();
    const total = world.body_count();
    const startIndex = total - count;

    for (let i = 0; i < count; i++) {
      const bodyIndex = startIndex + i;
      const shape = bodies[i]?.shape;
      if (shape && "plane" in shape) {
        const [nx, ny, nz] = shape.plane.normal;
        const normal = new THREE.Vector3(nx, ny, nz).normalize();
        const mesh = new THREE.Mesh(
          new THREE.PlaneGeometry(20, 20),
          new THREE.MeshStandardMaterial({ color: 0x777755, side: THREE.DoubleSide }),
        );
        mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), normal);
        mesh.position.copy(normal.multiplyScalar(shape.plane.d));
        addSpawnedMesh(bodyIndex, mesh);
        continue;
      }

      let mesh: THREE.Mesh;
      if (shape && "sphere" in shape) {
        mesh = new THREE.Mesh(
          new THREE.SphereGeometry(shape.sphere.radius, 16, 12),
          new THREE.MeshStandardMaterial({ color: 0xffaa00 }),
        );
      } else if (shape && "box" in shape) {
        const [hx, hy, hz] = shape.box.half;
        mesh = new THREE.Mesh(
          new THREE.BoxGeometry(hx * 2, hy * 2, hz * 2),
          new THREE.MeshStandardMaterial({ color: 0xffaa00 }),
        );
      } else {
        // 形状情報が読めない(想定外のJSON構造)場合のフォールバック。
        mesh = new THREE.Mesh(
          new THREE.SphereGeometry(0.3, 16, 12),
          new THREE.MeshStandardMaterial({ color: 0xffaa00 }),
        );
      }
      const pos = world.body_position_at_f32(bodyIndex);
      mesh.position.set(pos[0], pos[1], pos[2]);
      const rot = world.body_rotation_at_f32(bodyIndex);
      mesh.quaternion.set(rot[0], rot[1], rot[2], rot[3]);
      addSpawnedMesh(bodyIndex, mesh);
    }
    return count;
  };

  // シーンギャラリー(`SceneGalleryRef`のdoc参照)。Importと異なり`world`
  // そのものを`WasmWorld.from_scene_json`で差し替える——D9/D34のような
  // 非力学シーンも`fluids`/`thermal`/`circuit`/`astro`セクション込みで
  // 正しく構成されるようにするため(`import_scene_json`は`fluids`/`probes`
  // 以外を見ない設計上の非対称、モジュール冒頭のdoc参照)。
  //
  // 差し替えに伴い、旧`world`に紐づいていた以下の状態を掃除する(**残タスク
  // 完遂の増分**で発見した、旧worldのボディ/フレーム数を前提にした箇所):
  // - `bodyMeshes`/`pickables`/`constraintLines`: 旧ボディのメッシュ・拘束線。
  // - `motorArmBodies`/`currentMotorTarget`/`currentScale`: 旧ボディのindex
  //   キーが新worldでは無関係な(あるいは存在しない)ボディを指してしまう。
  // - `frameAxesHelpers`/`selectedFrameIndex`: 起動時に追加した既定の回転
  //   フレーム(`initialFrameIndex`)は新worldには存在しないため、参照すると
  //   `frame_world_position_f32`がJS例外を投げてrender()ループが壊れる
  //   ——ROOT(常に存在するindex 0)へリセットする。ギャラリーシーンの
  //   フレーム階層自体は現状スコープ外(正直な限界として記録)。
  // - `fluidPositionAttribute`: 旧worldで流体をスポーンしていた場合の粒子
  //   バッファ、新worldの流体粒子数と食い違うと壊れるため無効化する。
  // - `editUndoStack`/`commandLog`: 旧ボディのindexを記録しているため無効化。
  // - `circuitFreeWiringState.active`: 現状のギャラリーシーン(D4/D5/D6/D11)は
  //   いずれも回路を持たないため、固定デモ回路のスイッチ(`circuit_switch_index`
  //   はプレースホルダ値0、`from_scene_json`のdoc参照)を無効化する
  //   (自由配線回路エディタと同じ保護、`circuitFreeWiringState`のdoc参照)。
  sceneGalleryRef.current = (json: string) => {
    const parsed = JSON.parse(json) as ImportedScenarioJson;
    const bodies = parsed.bodies ?? [];

    for (const mesh of bodyMeshes.values()) {
      scene.remove(mesh);
      mesh.geometry.dispose();
      (mesh.material as THREE.Material).dispose();
    }
    bodyMeshes.clear();
    for (const line of constraintLines.values()) {
      scene.remove(line);
    }
    constraintLines.clear();
    for (const helper of frameAxesHelpers.values()) {
      scene.remove(helper);
    }
    frameAxesHelpers.clear();
    selectedFrameIndex = 0; // ROOT。ギャラリーシーンのフレーム階層は対象外。
    pickables = [];
    motorArmBodies.clear();
    currentMotorTarget.clear();
    currentScale.clear();
    editUndoStack.length = 0;
    commandLog.length = 0;
    fluidPositionAttribute = null;
    fluidPoints.visible = false;
    circuitFreeWiringState.active = true;
    circuitSwitchToggle.disabled = true;

    world = WasmWorld.from_scene_json(json);
    sceneBaseBodyCount = bodies.length;

    currentPredictionPrompts = parsed.prediction_prompts ?? [];
    renderPredictionPanel();

    for (let bodyIndex = 0; bodyIndex < bodies.length; bodyIndex++) {
      const shape = bodies[bodyIndex]?.shape;
      if (shape && "plane" in shape) {
        const [nx, ny, nz] = shape.plane.normal;
        const normal = new THREE.Vector3(nx, ny, nz).normalize();
        const mesh = new THREE.Mesh(
          new THREE.PlaneGeometry(20, 20),
          new THREE.MeshStandardMaterial({ color: 0x777755, side: THREE.DoubleSide }),
        );
        mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), normal);
        mesh.position.copy(normal.multiplyScalar(shape.plane.d));
        addSpawnedMesh(bodyIndex, mesh);
        continue;
      }

      let mesh: THREE.Mesh;
      if (shape && "sphere" in shape) {
        mesh = new THREE.Mesh(
          new THREE.SphereGeometry(shape.sphere.radius, 16, 12),
          new THREE.MeshStandardMaterial({ color: 0xffaa00 }),
        );
      } else if (shape && "box" in shape) {
        const [hx, hy, hz] = shape.box.half;
        mesh = new THREE.Mesh(
          new THREE.BoxGeometry(hx * 2, hy * 2, hz * 2),
          new THREE.MeshStandardMaterial({ color: 0xffaa00 }),
        );
      } else {
        mesh = new THREE.Mesh(
          new THREE.SphereGeometry(0.3, 16, 12),
          new THREE.MeshStandardMaterial({ color: 0xffaa00 }),
        );
      }
      const pos = world.body_position_at_f32(bodyIndex);
      mesh.position.set(pos[0], pos[1], pos[2]);
      const rot = world.body_rotation_at_f32(bodyIndex);
      mesh.quaternion.set(rot[0], rot[1], rot[2], rot[3]);
      addSpawnedMesh(bodyIndex, mesh);

      if (world.constraint_anchor_points_at(bodyIndex).length >= 6) {
        const lineGeometry = new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(),
          new THREE.Vector3(),
        ]);
        const line = new THREE.Line(lineGeometry, new THREE.LineBasicMaterial({ color: 0xffaa00 }));
        scene.add(line);
        constraintLines.set(bodyIndex, line);
      }
    }
    selectBody(0);
  };

  // Replay再生実行(`ReplayVerifyRef`のdoc参照)。記録済み`commandLog`を、
  // 既定シーン(床+箱のみ)を持つ新規`WasmWorld`へステップ番号どおりに再送する。
  // Grab/Release/ApplyForce/SetSwitch/SetHeatSourceはWasmWorldのコンストラクタが
  // 必ず用意する固定ボディ/回路/熱ノードが対象なので常に再現できるが、
  // SetMotorTarget(スポーンしたモーターアームが対象)は新規Worldにその
  // ボディが存在しないため`bodyIndex`が範囲外なら無視する(縮約実装、既知の
  // 限定——`sceneChanged`で呼び出し側に伝える)。MoveGrab(ドラッグ中の連続更新)
  // は元々記録していないため、再生されるのはGrabの初期アンカー位置のみ。
  replayVerifyRef.current = () => {
    const replayWorld = new WasmWorld(GRAVITY, DT, INITIAL_HEIGHT);
    const totalSteps = Number(world.step_count());
    const sceneChanged = world.body_count() !== 2;

    const commandsByStep = new Map<number, CommandLogEntry[]>();
    for (const entry of commandLog) {
      const list = commandsByStep.get(entry.step) ?? [];
      list.push(entry);
      commandsByStep.set(entry.step, list);
    }

    let heaterOn = false;
    let heaterWatts = 0;
    for (let s = 0; s < totalSteps; s++) {
      for (const entry of commandsByStep.get(s) ?? []) {
        switch (entry.kind) {
          case "Grab":
            if (entry.bodyIndex < replayWorld.body_count()) {
              replayWorld.push_grab(entry.bodyIndex, entry.targetX, entry.targetY, entry.targetZ);
            }
            break;
          case "Release":
            if (entry.bodyIndex < replayWorld.body_count()) {
              replayWorld.push_release(entry.bodyIndex);
            }
            break;
          case "ApplyForce":
            if (entry.bodyIndex < replayWorld.body_count()) {
              replayWorld.push_apply_force(entry.bodyIndex, entry.fx, entry.fy, entry.fz);
            }
            break;
          case "SetMotorTarget":
            if (entry.bodyIndex < replayWorld.body_count()) {
              replayWorld.set_motor_target_at(entry.bodyIndex, entry.targetAngle);
            }
            break;
          case "SetSwitch":
            replayWorld.set_circuit_switch_closed(entry.closed);
            break;
          case "SetHeatSource":
            heaterOn = entry.on;
            heaterWatts = entry.watts;
            break;
        }
      }
      if (heaterOn) replayWorld.push_heat_source(heaterWatts);
      replayWorld.step();
    }

    const finalBoxPos = replayWorld.body_position_at_f32(BODY_INDEX_BOX);
    // ライブ側の`world`はシーンギャラリー経由で任意のシーンに差し替わっている
    // 可能性があり、index 1(既定シーンの箱)が存在しないことがある——`sceneChanged`
    // が真の時点で`matches`は`false`確定なので、位置の意味自体が無いプレース
    // ホルダで安全に済ませる(**残タスク完遂のシーンギャラリー増分**で追加した
    // ガード、以前は既定シーン以外あり得なかったため無条件アクセスで安全だった)。
    const liveBoxPos = sceneChanged ? new Float32Array(3) : world.body_position_at_f32(BODY_INDEX_BOX);
    const finalStateHash = replayWorld.state_hash();
    const liveStateHash = world.state_hash();
    return {
      totalSteps,
      commandCount: commandLog.length,
      sceneChanged,
      finalStateHash,
      finalBoxPosition: [finalBoxPos[0], finalBoxPos[1], finalBoxPos[2]],
      liveStateHash,
      liveBoxPosition: [liveBoxPos[0], liveBoxPos[1], liveBoxPos[2]],
      matches: !sceneChanged && finalStateHash === liveStateHash,
    };
  };

  // Prefabs(`PrefabRef`のdoc参照)。球/箱のみ対応(スポーンパレットの
  // `spawn_sphere`/`spawn_box`自体がこの2形状しか受け付けないのと同じ制約)。
  // `spawn_box`は立方体のみ(単一`half_extent`引数)のため、非立方体の箱を
  // captureした場合は`params[0]`(半径/1軸目のhalf_extent)のみを使う
  // (既知の簡略化)。
  prefabRef.current = {
    captureSelectedBody: () => {
      const kind = world.body_shape_kind_at(selectedBodyIndex);
      if (kind !== "sphere" && kind !== "box") return null;
      const params = Array.from(world.body_shape_params_f64_at(selectedBodyIndex));
      const material = world.body_material_label_at(selectedBodyIndex);
      return { kind, params, material };
    },
    spawn: (prefab) => {
      const { x, z } = nextSpawnPosition();
      if (prefab.kind === "sphere") {
        const radius = prefab.params[0] ?? SPAWN_SPHERE_RADIUS;
        const bodyIndex = world.spawn_sphere(x, SPAWN_HEIGHT, z, radius, prefab.material);
        const mesh = new THREE.Mesh(
          new THREE.SphereGeometry(radius, 16, 12),
          new THREE.MeshStandardMaterial({ color: 0x6699ff }),
        );
        addSpawnedMesh(bodyIndex, mesh);
      } else if (prefab.kind === "box") {
        const halfExtent = prefab.params[0] ?? SPAWN_BOX_HALF_EXTENT;
        const bodyIndex = world.spawn_box(x, SPAWN_HEIGHT, z, halfExtent, prefab.material);
        const mesh = new THREE.Mesh(
          new THREE.BoxGeometry(halfExtent * 2, halfExtent * 2, halfExtent * 2),
          new THREE.MeshStandardMaterial({ color: 0x66cc66 }),
        );
        addSpawnedMesh(bodyIndex, mesh);
      }
    },
  };

  document.getElementById("btn-spawn-sphere")!.addEventListener("click", () => {
    const { x, z } = nextSpawnPosition();
    const material = spawnMaterialSelect.value;
    const bodyIndex = world.spawn_sphere(x, SPAWN_HEIGHT, z, SPAWN_SPHERE_RADIUS, material);
    const mesh = new THREE.Mesh(
      new THREE.SphereGeometry(SPAWN_SPHERE_RADIUS, 16, 12),
      new THREE.MeshStandardMaterial({ color: 0x6699ff }),
    );
    addSpawnedMesh(bodyIndex, mesh);
  });

  document.getElementById("btn-spawn-box")!.addEventListener("click", () => {
    const { x, z } = nextSpawnPosition();
    const material = spawnMaterialSelect.value;
    const bodyIndex = world.spawn_box(x, SPAWN_HEIGHT, z, SPAWN_BOX_HALF_EXTENT, material);
    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(SPAWN_BOX_HALF_EXTENT * 2, SPAWN_BOX_HALF_EXTENT * 2, SPAWN_BOX_HALF_EXTENT * 2),
      new THREE.MeshStandardMaterial({ color: 0x66cc66 }),
    );
    addSpawnedMesh(bodyIndex, mesh);
  });

  document.getElementById("btn-spawn-pendulum")!.addEventListener("click", () => {
    const { x, z } = nextSpawnPosition();
    const material = spawnMaterialSelect.value;
    const bodyIndex = world.spawn_pendulum(x, PENDULUM_PIVOT_HEIGHT, z, PENDULUM_ARM_LENGTH, material);
    const mesh = new THREE.Mesh(
      new THREE.SphereGeometry(0.3, 16, 12),
      new THREE.MeshStandardMaterial({ color: 0xff66cc }),
    );
    addSpawnedMesh(bodyIndex, mesh);
    const lineGeometry = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(),
      new THREE.Vector3(),
    ]);
    const line = new THREE.Line(lineGeometry, new THREE.LineBasicMaterial({ color: 0xffaa00 }));
    scene.add(line);
    constraintLines.set(bodyIndex, line);
  });

  document.getElementById("btn-spawn-motor")!.addEventListener("click", () => {
    const { x, z } = nextSpawnPosition();
    const material = spawnMaterialSelect.value;
    const bodyIndex = world.spawn_motor_arm(x, PENDULUM_PIVOT_HEIGHT, z, material);
    motorArmBodies.add(bodyIndex);
    currentMotorTarget.set(bodyIndex, MOTOR_TARGET_LOW);
    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(0.2, 1.2, 0.2),
      new THREE.MeshStandardMaterial({ color: 0x66ffcc }),
    );
    addSpawnedMesh(bodyIndex, mesh);
  });

  document.getElementById("btn-spawn-fluid")!.addEventListener("click", () => {
    world.spawn_fluid_block();
    const count = world.fluid_particle_count();
    fluidPositionAttribute = new THREE.BufferAttribute(new Float32Array(count * 3), 3);
    fluidGeometry.setAttribute("position", fluidPositionAttribute);
    fluidPoints.visible = true;
    highlightHierarchy = setUpHierarchy(world, selectBody, selectedFrameIndex, selectFrame);
  });

  // フレーム階層ドリルインUI: Hierarchyで選択中のフレーム(既定はROOTでは
  // なく、起動時に追加した既定のフレーム——`selectedFrameIndex`の初期値)の
  // 子として新規フレームを追加する。追加した新規フレームをそのまま選択状態に
  // することで、連続クリックすると親→子→孫…と鎖状にネストしたフレームを
  // 手軽に組み立てられる(選択を変えれば任意のフレームの下に分岐させることも
  // できる)。
  document.getElementById("btn-add-frame")!.addEventListener("click", () => {
    const newFrameIndex = world.add_child_frame(
      selectedFrameIndex,
      FRAME_CHILD_OFFSET,
      0,
      0,
      FRAME_AXIS_ANGULAR_VELOCITY,
    );
    createFrameAxesHelper(newFrameIndex);
    selectFrame(newFrameIndex);
  });

  playButton.addEventListener("click", () => {
    if (mode !== "play") return;
    playing = !playing;
    playButton.textContent = playing ? "⏸" : "▶";
  });
  stepButton.addEventListener("click", () => {
    if (mode === "play" && !playing) {
      world.step();
      appendConsoleEntries(world.drain_events_text());
      render();
    }
  });

  // Timeline スクラバ(設計docs/00-foundation/04-architecture.md「巻き戻しの
  // スナップショット予算」既定1s間隔・リングバッファN=8面)。ドラッグ中
  // (`scrubbing`)は`render()`側からスクラバのmax/valueを触らない——そうしないと
  // 毎フレームの「最新に追従」更新がユーザーのドラッグ位置を上書きしてしまう。
  const scrubber = document.getElementById("timeline-scrubber") as HTMLInputElement;
  const playModeBadge = document.getElementById("play-mode-badge")!;
  let scrubbing = false;
  scrubber.addEventListener("pointerdown", () => {
    scrubbing = true;
    playing = false;
    playButton.textContent = "▶";
  });
  scrubber.addEventListener("input", () => {
    world.restore_snapshot(Number(scrubber.value));
    render();
  });
  scrubber.addEventListener("pointerup", () => {
    scrubbing = false;
  });

  // Timelineブックマーク(設計docs/23-frontend/01-editor.md §1.4「ブックマーク:
  // 任意時点にラベル付けし、後で戻れる」)。リングバッファの退避を受けない別領域
  // (`add_bookmark`/`restore_bookmark`)に保存する。縮約実装の理由: シーンJSONと
  // 一緒に出す「共有」用途(設計の記述)は未実装、ブラウザ内での往復のみ。
  const bookmarkLabelInput = document.getElementById("bookmark-label") as HTMLInputElement;
  const addBookmarkButton = document.getElementById("btn-add-bookmark") as HTMLButtonElement;
  const bookmarkList = document.getElementById("bookmark-list")!;

  function renderBookmarkList() {
    bookmarkList.innerHTML = "";
    const count = world.bookmark_count();
    for (let i = 0; i < count; i++) {
      const item = document.createElement("span");
      const chip = document.createElement("button");
      chip.className = "bookmark-chip";
      chip.textContent = `${world.bookmark_label_at(i)} (${world.bookmark_time_at(i).toFixed(1)}s)`;
      chip.addEventListener("click", () => {
        playing = false;
        playButton.textContent = "▶";
        world.restore_bookmark(i);
        render();
      });
      item.appendChild(chip);

      // ブックマークのエクスポート(設計§6「保存・共有: シーンJSON+Replay+
      // ブックマークを単一ファイルとしてエクスポート」の縮約実装)。`World`が
      // `Serialize`を持たないため内部状態のバイト単位の保存ではなく、シーンJSON
      // Importへそのまま読み込める`Scenario`互換JSONとして剛体の観測可能な状態
      // (位置・姿勢・速度)のみを書き出す(流体/熱/回路ドメインの状態は対象外、
      // `bookmark_export_scene_json`のdoc参照)。
      const exportButton = document.createElement("button");
      exportButton.textContent = "⬇";
      exportButton.title = "このブックマークをシーンJSONとしてエクスポート(Importで読み込み可能)";
      exportButton.addEventListener("click", () => {
        const json = world.bookmark_export_scene_json(i);
        const blob = new Blob([json], { type: "application/json" });
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = `bookmark_${world.bookmark_label_at(i)}.json`;
        a.click();
        URL.revokeObjectURL(url);
      });
      item.appendChild(exportButton);

      bookmarkList.appendChild(item);
    }
  }

  addBookmarkButton.addEventListener("click", () => {
    const label = bookmarkLabelInput.value.trim() || `t=${world.time().toFixed(1)}s`;
    world.add_bookmark(label);
    bookmarkLabelInput.value = "";
    renderBookmarkList();
  });

  // Consoleのイベント行クリック→Timelineジャンプ(設計docs/23-frontend/
  // 01-editor.md §1.5「クリックでTimeline/Scene Viewと連動」)。イベント行に
  // 埋め込まれたstep番号の時刻に最も近いスナップショットへ巻き戻す(スナップショット
  // は1s間隔のため厳密なstep一致ではなく最近傍、`restore_snapshot`と同じ挙動)。
  jumpToStepRef.current = (step: number) => {
    const count = world.snapshot_count();
    if (count === 0) return;
    const targetTime = step * DT;
    let bestIndex = 0;
    let bestDiff = Infinity;
    for (let i = 0; i < count; i++) {
      const diff = Math.abs(world.snapshot_time_at(i) - targetTime);
      if (diff < bestDiff) {
        bestDiff = diff;
        bestIndex = i;
      }
    }
    playing = false;
    playButton.textContent = "▶";
    world.restore_snapshot(bestIndex);
    render();
  };

  // 設計§4「Playモードでの介入は全てCommandとしてキューに積まれ、次ステップ先頭で
  // 適用される」の最小デモ: 直接オブジェクトの状態を書き換えるのではなく、
  // `push_apply_force`(Command::ApplyForceをキューに積む`sim-wasm`側の新API)を
  // 呼ぶだけで、実際の力の適用は次の`world.step()`側が担う。
  // 既定シーンの箱(鋼(炭素鋼)1m^3、密度約7850kg/m^3)を目視で分かる程度に動かす
  // (1クリックでΔv≈0.4m/s程度)よう較正した値。**残タスク完遂のシーンギャラリー
  // 増分**で`selectedBodyIndex`(選択中のボディ)へ効くよう一般化した——質量が
  // 異なるボディでは体感速度変化も変わる(質量ごとの再較正は対象外)。
  const NUDGE_FORCE_NEWTONS = 400_000.0;
  nudgeButton.addEventListener("click", () => {
    world.push_apply_force(selectedBodyIndex, 0.0, NUDGE_FORCE_NEWTONS, 0.0);
    pushCommandLog(world, { kind: "ApplyForce", bodyIndex: selectedBodyIndex, fx: 0.0, fy: NUDGE_FORCE_NEWTONS, fz: 0.0 });
    if (forceOverlayToggle.checked) {
      const p = world.body_position_at_f32(selectedBodyIndex);
      showForceOverlay(new THREE.Vector3(p[0], p[1], p[2]), new THREE.Vector3(0.0, NUDGE_FORCE_NEWTONS, 0.0));
    }
  });

  motorToggleButton.addEventListener("click", () => {
    if (mode !== "play" || !motorArmBodies.has(selectedBodyIndex)) return;
    const current = currentMotorTarget.get(selectedBodyIndex) ?? MOTOR_TARGET_LOW;
    const next = current === MOTOR_TARGET_LOW ? MOTOR_TARGET_HIGH : MOTOR_TARGET_LOW;
    world.set_motor_target_at(selectedBodyIndex, next);
    currentMotorTarget.set(selectedBodyIndex, next);
    pushCommandLog(world, {
      kind: "SetMotorTarget",
      bodyIndex: selectedBodyIndex,
      bodyLabel: world.body_label_at(selectedBodyIndex),
      targetAngle: next,
    });
  });

  circuitSwitchToggle.addEventListener("change", () => {
    // 自由配線回路エディタでリセットした後は`circuit_switch_index`(固定デモの
    // スイッチ)が新回路のスイッチ数を超えて無効になり得るため、この経路は
    // 無効化する(`circuitFreeWiringState`のdoc参照、チェックボックス自体も
    // リセット時に`disabled`にする)。
    if (circuitFreeWiringState.active) return;
    world.set_circuit_switch_closed(circuitSwitchToggle.checked);
    pushCommandLog(world, { kind: "SetSwitch", closed: circuitSwitchToggle.checked });
  });

  heaterToggle.addEventListener("change", () => {
    // ヒーター自体の`Command::SetHeatSource`は`frame()`ループが毎subStep
    // 再送する(モジュールdoc「1step分だけ効く」縮約セマンティクス参照)ため
    // ここでは記録しない——ユーザーが行った「切替」という離散操作のみ記録する
    // (Replay再生実行はこの`on`/`watts`から再送区間を再構成する)。
    pushCommandLog(world, { kind: "SetHeatSource", on: heaterToggle.checked, watts: HEATER_WATTS });
  });

  const inspectorPosition = new THREE.Vector3();
  const inspectorRotationQuat = new THREE.Quaternion();
  const inspectorRotation = new THREE.Euler();
  const inspectorVelocity = new THREE.Vector3();

  function render() {
    updatePredictionResults();

    // 全ボディのメッシュ位置/姿勢/スケールを同期する(**残タスク完遂の
    // シーンギャラリー増分**で`box`だけの決め打ち同期を統合、`bodyMeshes`の
    // doc参照)。**Plane形状は同期から除外する**——`Shape::Plane`の世界座標での
    // 向き/位置は`normal`/`d`で定義され、剛体の`Transform.position`/
    // `rotation`とは独立(`RigidBodyDesc::dynamic`は常に`rotation:
    // Quat::IDENTITY`を設定するため、同期すればPlaneメッシュ生成時に
    // `normal`から計算した向き(`sceneImportRef`のPlane分岐参照)が単位回転で
    // 上書きされてしまう——統合の際に発見し、床メッシュの見た目が壊れる前に
    // 気付いて対処した)。Planeは静的なので同期しなくても正しい。
    for (const [bodyIndex, mesh] of bodyMeshes) {
      if (world.body_shape_kind_at(bodyIndex) === "plane") continue;
      const sp = world.body_position_at_f32(bodyIndex);
      mesh.position.set(sp[0], sp[1], sp[2]);
      const sr = world.body_rotation_at_f32(bodyIndex);
      mesh.quaternion.set(sr[0], sr[1], sr[2], sr[3]);
      mesh.scale.setScalar(currentScale.get(bodyIndex) ?? 1.0);
    }

    for (const [bodyIndex, line] of constraintLines) {
      if (!constraintOverlayToggle.checked) {
        line.visible = false;
        continue;
      }
      const anchors = world.constraint_anchor_points_at(bodyIndex);
      if (anchors.length < 6) {
        line.visible = false;
        continue;
      }
      const positions = line.geometry.attributes.position as THREE.BufferAttribute;
      positions.setXYZ(0, anchors[0], anchors[1], anchors[2]);
      positions.setXYZ(1, anchors[3], anchors[4], anchors[5]);
      positions.needsUpdate = true;
      line.visible = true;
    }

    if (frameOverlayToggle.checked) {
      for (const [frameIndex, helper] of frameAxesHelpers) {
        const pos = world.frame_world_position_f32(frameIndex);
        const rot = world.frame_world_rotation_f32(frameIndex);
        helper.position.set(pos[0], pos[1], pos[2]);
        helper.quaternion.set(rot[0], rot[1], rot[2], rot[3]);
        helper.visible = true;
      }
    } else {
      for (const helper of frameAxesHelpers.values()) {
        helper.visible = false;
      }
    }

    if (fluidPositionAttribute) {
      const positions = world.fluid_particle_positions_f32();
      (fluidPositionAttribute.array as Float32Array).set(positions);
      fluidPositionAttribute.needsUpdate = true;
    }

    const selectedPosition = world.body_position_at_f32(selectedBodyIndex);
    const selectedRotation = world.body_rotation_at_f32(selectedBodyIndex);
    const selectedVelocity = world.body_velocity_at_f32(selectedBodyIndex);
    inspectorPosition.set(selectedPosition[0], selectedPosition[1], selectedPosition[2]);
    inspectorRotationQuat.set(selectedRotation[0], selectedRotation[1], selectedRotation[2], selectedRotation[3]);
    inspectorRotation.setFromQuaternion(inspectorRotationQuat);
    inspectorVelocity.set(selectedVelocity[0], selectedVelocity[1], selectedVelocity[2]);
    updateInspectorTransformFields(inspectorPosition, inspectorRotation, inspectorVelocity);
    updateProbeGraph([
      { label: "BodyPosY", color: "#9cf", history: world.y_probe_history_f64() },
      { label: "BodySpeed", color: "#fc6", history: world.speed_probe_history_f64() },
    ]);

    const speed = inspectorVelocity.length();
    if (velocityOverlayToggle.checked && speed > 1e-6) {
      velocityDirection.copy(inspectorVelocity).divideScalar(speed);
      velocityArrow.position.copy(inspectorPosition);
      velocityArrow.setDirection(velocityDirection);
      velocityArrow.setLength(
        Math.max(speed * VELOCITY_OVERLAY_SCALE, 0.2),
        Math.min(0.2, speed * VELOCITY_OVERLAY_SCALE * 0.3),
        Math.min(0.15, speed * VELOCITY_OVERLAY_SCALE * 0.2),
      );
      velocityArrow.visible = true;
    } else {
      velocityArrow.visible = false;
    }

    if (contactOverlayToggle.checked) {
      const contactPoints = world.contact_points_f32();
      const count = Math.min(contactPoints.length / 3, CONTACT_MARKER_POOL_SIZE);
      for (let i = 0; i < CONTACT_MARKER_POOL_SIZE; i++) {
        if (i < count) {
          contactMarkers[i].position.set(contactPoints[i * 3], contactPoints[i * 3 + 1], contactPoints[i * 3 + 2]);
          contactMarkers[i].visible = true;
        } else {
          contactMarkers[i].visible = false;
        }
      }
    } else {
      for (const marker of contactMarkers) marker.visible = false;
    }

    forceArrow.visible = forceOverlayToggle.checked && performance.now() < forceOverlayHideAtMs;

    const showGizmo = mode === "edit" && !world.body_is_static_at(selectedBodyIndex);
    gizmoGroup.visible = showGizmo;
    rotationGizmoGroup.visible = showGizmo;
    scaleGizmoGroup.visible = showGizmo;
    if (showGizmo) {
      gizmoGroup.position.copy(inspectorPosition);
      rotationGizmoGroup.position.copy(inspectorPosition);
      scaleGizmoGroup.position.copy(inspectorPosition);
    }

    const hashFull = world.state_hash();
    hud.textContent = [
      `t = ${world.time().toFixed(3)} s`,
      `step = ${world.step_count().toString()}`,
      `y = ${selectedPosition[1].toFixed(4)} m`,
      `circuit V = ${world.circuit_divider_voltage().toFixed(3)} V`,
      `heater T = ${world.heater_node_temperature().toFixed(2)} K`,
    ].join("\n");
    timelineTime.textContent = `t = ${world.time().toFixed(3)} s`;
    timelineStep.textContent = `step = ${world.step_count().toString()}`;
    hashDisplay.textContent = `hash: ${hashFull.slice(0, 8)}`;
    hashDisplay.title = hashFull;
    playModeBadge.textContent = mode === "edit" ? "Edit" : playing ? "Playing" : "Paused";

    if (!scrubbing) {
      const latestIndex = Math.max(world.snapshot_count() - 1, 0);
      scrubber.max = String(latestIndex);
      scrubber.value = String(latestIndex);
    }

    renderer.render(scene, camera);
  }
  hashDisplay.addEventListener("click", () => {
    navigator.clipboard?.writeText(world.state_hash()).catch(() => {});
  });

  let accumulator = 0;
  let lastTimeMs = performance.now();

  function frame(nowMs: number) {
    const frameSeconds = Math.min((nowMs - lastTimeMs) / 1000, 0.25);
    lastTimeMs = nowMs;

    if (mode === "play" && playing) {
      accumulator += frameSeconds * timeScale;
      let steps = 0;
      while (accumulator >= DT && steps < MAX_STEPS_PER_FRAME) {
        if (heaterToggle.checked) world.push_heat_source(HEATER_WATTS);
        world.step();
        accumulator -= DT;
        steps += 1;
      }
      appendConsoleEntries(world.drain_events_text());
    }

    render();
    requestAnimationFrame(frame);
  }

  requestAnimationFrame(frame);
}

function main() {
  setUpLayoutPresetSwitcher();
  const updateProbeGraph = setUpProbeGraph();
  const jumpToStepRef: JumpToStepRef = { current: null };
  const appendConsoleEntries = setUpConsole(jumpToStepRef);
  const materialsRef: MaterialsRef = { current: null };
  const circuitRef: CircuitRef = { current: null };
  const sceneExportRef: SceneExportRef = { current: null };
  const sceneImportRef: SceneImportRef = { current: null };
  const replayVerifyRef: ReplayVerifyRef = { current: null };
  const circuitEditorRef: CircuitEditorRef = { current: null };
  const circuitFreeWiringState: CircuitFreeWiringState = { active: false };
  const prefabRef: PrefabRef = { current: null };
  const sceneGalleryRef: SceneGalleryRef = { current: null };
  setUpProjectDrawer(
    materialsRef,
    circuitRef,
    sceneExportRef,
    sceneImportRef,
    replayVerifyRef,
    circuitEditorRef,
    circuitFreeWiringState,
    prefabRef,
    sceneGalleryRef,
  );
  setUpSceneView(
    updateProbeGraph,
    appendConsoleEntries,
    jumpToStepRef,
    materialsRef,
    circuitRef,
    sceneExportRef,
    sceneImportRef,
    replayVerifyRef,
    circuitEditorRef,
    circuitFreeWiringState,
    prefabRef,
    sceneGalleryRef,
  ).catch((err) => {
    const hud = document.getElementById("hud");
    if (hud) hud.textContent = `エラー: ${String(err)}`;
    console.error(err);
  });
}

main();
