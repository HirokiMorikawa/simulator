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
// `RigidBodySet`位置の直接書き換え)で位置を編集できる(回転/スケールの
// ハンドルは、この2体デモに意味のある対象が無いため未実装)。再生/ステップ/
// Nudgeボタンは無効化される(シミュレーションは進行しない)。Playモードでは
// Gizmoは非表示になり、箱への直接ドラッグがCommandキュー経由
// (`Command::Grab/MoveGrab/Release`、`push_grab`/`push_move_grab`/
// `push_release`)の物理的な"つかむ"操作になり、再生/ステップ/Nudgeボタンが
// 有効になる。Shape/Materialは`sim-wasm`側に対応するクエリAPIが無いため
// (World API-only制約)、Phase 0デモが実際に構築する内容と一致させた固定の
// ルックアップテーブル(`BODY_META`)を使う。Scene Viewオーバーレイ(設計§1.2)は
// 選択中ボディの速度ベクトルを矢印表示するもの(切替可、Toolbarのチェック
// ボックス)のみ実装(接触点・力・拘束・流体場・フレーム軸は対象外)。Consoleは
// `World::drain_events`(既存API)が返す実イベント(この2体デモでは箱の着地/
// 跳ね返りのたびに発生する`ContactStarted`/`ContactEnded`)をAll/Errors/
// Warnings/Infoタブでフィルタ表示する(設計§1.5)。Projectは静的な
// プレースホルダ内容のまま。TimelineはWorld::snapshot/restoreによる
// スナップショットリングバッファ(1s間隔・N=8面)でスクラブ・巻き戻しができ、
// 任意時点をブックマーク(`add_bookmark`/`restore_bookmark`、リングバッファの
// 退避を受けない別領域)として名前付きで保存・復元できる。Gizmoの回転/
// スケールハンドル・オーバーレイ残り・Commandキュー残り(SetMotorTarget/
// SetSwitch/SetHeatSource未配線)・回路サブモードは全て後続増分。

const GRAVITY = 9.80665;
const DT = 1.0 / 120.0;
const INITIAL_HEIGHT = 10.0;
const BOX_HALF_EXTENT = 0.5;
const MAX_STEPS_PER_FRAME = 240;
const BODY_INDEX_GROUND = 0;
const BODY_INDEX_BOX = 1;

// `sim-wasm`のWorld API-only制約(Shape/Materialのクエリが無い)により、
// `WasmWorld::new`が実際に構築する内容と一致させた固定のルックアップテーブル。
const BODY_META: Record<number, { shape: string; material: string }> = {
  [BODY_INDEX_GROUND]: { shape: "Plane(normal=(0,1,0), d=0)", material: "コンクリート" },
  [BODY_INDEX_BOX]: { shape: `Box(${BOX_HALF_EXTENT},${BOX_HALF_EXTENT},${BOX_HALF_EXTENT})`, material: "鋼(炭素鋼)" },
};

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

function setUpConsole(): (eventsText: string) => void {
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

// Hierarchyパネル(設計docs/23-frontend/01-editor.md §1.1)。`world.body_count`/
// `body_label_at`から実際のボディ一覧を組み立て、クリックで`onSelect`を呼ぶ
// (選択はInspector・Scene Viewと連動、設計が求める双方向選択)。戻り値の関数は
// Scene Viewピッキング(`onSelect`を経由せず見た目のハイライトだけ更新したい
// 場合)向けに、外部からハイライトだけを同期させる手段として公開する。
function setUpHierarchy(world: WasmWorld, onSelect: (index: number) => void): (index: number) => void {
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
  root.appendChild(bodies);
  tree.appendChild(root);
  return highlight;
}

// Inspectorパネル(設計docs/23-frontend/01-editor.md §1.3)。選択中ボディの
// Shape/Material(`BODY_META`、World API-only制約により固定値)+ Transform
// (毎フレーム実データで更新、`updateInspectorTransformFields`)を表示する。
function renderInspectorFor(world: WasmWorld, index: number): void {
  const body = document.getElementById("inspector-body")!;
  const meta = BODY_META[index] ?? { shape: "?", material: "?" };
  const label = world.body_label_at(index);
  const staticBadge = world.body_is_static_at(index) ? ' <span class="badge">Static</span>' : "";
  body.innerHTML = `
    <div class="inspector-component">
      <h3>${label}${staticBadge}</h3>
      <div class="inspector-field"><span>Shape</span><span>${meta.shape}</span></div>
    </div>
    <div class="inspector-component">
      <h3>Transform</h3>
      <div class="inspector-field"><span>Position</span><span id="inspector-position">—</span></div>
      <div class="inspector-field"><span>Velocity</span><span id="inspector-velocity">—</span></div>
    </div>
    <div class="inspector-component">
      <h3>RigidBody</h3>
      <div class="inspector-field"><span>Material</span><span>${meta.material}</span></div>
    </div>
  `;
}

function updateInspectorTransformFields(position: THREE.Vector3, velocity: THREE.Vector3): void {
  const positionField = document.getElementById("inspector-position");
  const velocityField = document.getElementById("inspector-velocity");
  if (!positionField || !velocityField) return; // 選択切替の再描画中は一時的に無い。
  positionField.textContent = `${position.x.toFixed(3)}, ${position.y.toFixed(3)}, ${position.z.toFixed(3)}`;
  velocityField.textContent = `${velocity.x.toFixed(3)}, ${velocity.y.toFixed(3)}, ${velocity.z.toFixed(3)}`;
}

function setUpProjectDrawer() {
  const body = document.getElementById("project-body")!;
  const tabs = document.querySelectorAll<HTMLButtonElement>(".project-tab");
  const contentByTab: Record<string, string> = {
    scenes: "Scenes: (D1–D43 サンプルシーンの読み込みは後続増分)",
    materials: "Materials: MaterialDb プリセット一覧は後続増分",
    prefabs: "Prefabs: 未実装",
    replays: "Replays: 未実装",
  };
  function show(tab: string) {
    body.textContent = contentByTab[tab] ?? "";
    tabs.forEach((t) => t.classList.toggle("active", t.dataset.tab === tab));
  }
  tabs.forEach((tab) => {
    tab.addEventListener("click", () => show(tab.dataset.tab!));
  });
  show("scenes");
}

// Probe Graphsパネル(設計docs/23-frontend/01-editor.md §1.4「Probeグラフ:
// シーン定義の観測量を時系列表示」)の最小デモ。1系列(箱のy座標)の折れ線を
// canvas 2Dで描画する。縮約実装の理由: 複数系列の重ね描き・対数軸・CSV
// エクスポート(design§1.4のフル仕様)は後続増分、ここでは単一系列の自動
// スケーリング折れ線のみ。
function setUpProbeGraph(): (history: Float64Array) => void {
  const canvas = document.getElementById("probe-canvas") as HTMLCanvasElement;
  const ctx = canvas.getContext("2d")!;

  return (history: Float64Array) => {
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }
    ctx.clearRect(0, 0, w, h);
    if (history.length < 2) return;

    let min = Infinity;
    let max = -Infinity;
    for (const v of history) {
      if (v < min) min = v;
      if (v > max) max = v;
    }
    const range = max - min > 1e-9 ? max - min : 1.0;

    ctx.strokeStyle = "#9cf";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    for (let i = 0; i < history.length; i++) {
      const x = (i / (history.length - 1)) * w;
      const y = h - ((history[i] - min) / range) * h;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();

    ctx.fillStyle = "#888";
    ctx.font = "11px monospace";
    ctx.fillText(`BodyPosY: max=${max.toFixed(2)} min=${min.toFixed(2)}`, 4, 12);
  };
}

async function setUpSceneView(
  updateProbeGraph: (history: Float64Array) => void,
  appendConsoleEntries: (eventsText: string) => void,
) {
  await init();
  const world = new WasmWorld(GRAVITY, DT, INITIAL_HEIGHT);

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

  let selectedBodyIndex = BODY_INDEX_BOX;
  function selectBody(index: number) {
    selectedBodyIndex = index;
    renderInspectorFor(world, index);
    highlightHierarchy(index);
  }
  const highlightHierarchy = setUpHierarchy(world, selectBody);
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
  const pickables: { mesh: THREE.Object3D; bodyIndex: number }[] = [
    { mesh: ground, bodyIndex: BODY_INDEX_GROUND },
    { mesh: box, bodyIndex: BODY_INDEX_BOX },
  ];
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

  const AXIS_VECTORS: Record<"x" | "y" | "z", THREE.Vector3> = {
    x: new THREE.Vector3(1, 0, 0),
    y: new THREE.Vector3(0, 1, 0),
    z: new THREE.Vector3(0, 0, 1),
  };

  let dragStartScreen: { x: number; y: number } | null = null;
  let pointerDownHit: ReturnType<typeof hitTest> = null;
  let pointerDownGizmoAxis: "x" | "y" | "z" | null = null;
  let isDragging = false;
  let dragMode: "grab" | "gizmo" | null = null;
  const gizmoAxisDir = new THREE.Vector3();
  const gizmoDragStartPosition = new THREE.Vector3();
  let gizmoDragStartScalar = 0;

  renderer.domElement.addEventListener("pointerdown", (event) => {
    dragStartScreen = { x: event.clientX, y: event.clientY };
    isDragging = false;
    dragMode = null;
    if (mode === "edit") {
      pointerDownGizmoAxis = hitGizmo(event);
      pointerDownHit = pointerDownGizmoAxis ? null : hitTest(event, event.altKey);
    } else {
      pointerDownGizmoAxis = null;
      pointerDownHit = hitTest(event, event.altKey);
    }
  });

  renderer.domElement.addEventListener("pointermove", (event) => {
    if (!dragStartScreen) return;
    const dx = event.clientX - dragStartScreen.x;
    const dy = event.clientY - dragStartScreen.y;
    if (!isDragging) {
      if (Math.hypot(dx, dy) < DRAG_THRESHOLD_PX) return;
      if (pointerDownGizmoAxis) {
        isDragging = true;
        dragMode = "gizmo";
        gizmoAxisDir.copy(AXIS_VECTORS[pointerDownGizmoAxis]);
        gizmoDragStartPosition.copy(gizmoGroup.position);
        gizmoDragStartScalar = gizmoAxisDir.dot(gizmoDragStartPosition);
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
      } else {
        if (mode !== "play" || !pointerDownHit || pointerDownHit.picked.bodyIndex !== BODY_INDEX_BOX) return;
        isDragging = true;
        dragMode = "grab";
        camera.getWorldDirection(cameraDirection);
        dragPlane.setFromNormalAndCoplanarPoint(cameraDirection, pointerDownHit.worldPoint);
        const p = world.body_position_at_f32(BODY_INDEX_BOX);
        world.push_grab(p[0], p[1], p[2]);
      }
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
      world.push_move_grab(dragPlaneHit.x, dragPlaneHit.y, dragPlaneHit.z);
    }
  });

  renderer.domElement.addEventListener("pointerup", () => {
    if (isDragging) {
      if (dragMode === "grab") {
        world.push_release();
      }
    } else if (pointerDownHit) {
      selectBody(pointerDownHit.picked.bodyIndex);
    }
    isDragging = false;
    dragMode = null;
    dragStartScreen = null;
    pointerDownHit = null;
    pointerDownGizmoAxis = null;
  });

  const hud = document.getElementById("hud")!;
  const hashDisplay = document.getElementById("hash-display")!;
  const timelineTime = document.getElementById("timeline-time")!;
  const timelineStep = document.getElementById("timeline-step")!;
  const playButton = document.getElementById("btn-play") as HTMLButtonElement;
  const stepButton = document.getElementById("btn-step") as HTMLButtonElement;
  const nudgeButton = document.getElementById("btn-nudge") as HTMLButtonElement;

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
    modeEditButton.classList.toggle("active", mode === "edit");
    modePlayButton.classList.toggle("active", mode === "play");
  }
  modeEditButton.addEventListener("click", () => setMode("edit"));
  modePlayButton.addEventListener("click", () => setMode("play"));
  setMode("edit");

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
      const chip = document.createElement("button");
      chip.className = "bookmark-chip";
      chip.textContent = `${world.bookmark_label_at(i)} (${world.bookmark_time_at(i).toFixed(1)}s)`;
      chip.addEventListener("click", () => {
        playing = false;
        playButton.textContent = "▶";
        world.restore_bookmark(i);
        render();
      });
      bookmarkList.appendChild(chip);
    }
  }

  addBookmarkButton.addEventListener("click", () => {
    const label = bookmarkLabelInput.value.trim() || `t=${world.time().toFixed(1)}s`;
    world.add_bookmark(label);
    bookmarkLabelInput.value = "";
    renderBookmarkList();
  });

  // 設計§4「Playモードでの介入は全てCommandとしてキューに積まれ、次ステップ先頭で
  // 適用される」の最小デモ: 直接オブジェクトの状態を書き換えるのではなく、
  // `push_apply_force`(Command::ApplyForceをキューに積む`sim-wasm`側の新API)を
  // 呼ぶだけで、実際の力の適用は次の`world.step()`側が担う。
  // 箱は鋼(炭素鋼)1m^3(密度約7850kg/m^3)相当のため質量が大きく、1step
  // (dt=1/120s)だけ働く力ではΔv=F*dt/mが小さくなりがち。目視で分かる程度の
  // 速度変化(1クリックでΔv≈0.4m/s程度)になるよう十分大きな値を選んだ。
  const NUDGE_FORCE_NEWTONS = 400_000.0;
  nudgeButton.addEventListener("click", () => {
    world.push_apply_force(0.0, NUDGE_FORCE_NEWTONS, 0.0);
  });

  const inspectorPosition = new THREE.Vector3();
  const inspectorVelocity = new THREE.Vector3();

  function render() {
    const p = world.body_position_at_f32(BODY_INDEX_BOX);
    box.position.set(p[0], p[1], p[2]);

    const selectedPosition = world.body_position_at_f32(selectedBodyIndex);
    const selectedVelocity = world.body_velocity_at_f32(selectedBodyIndex);
    inspectorPosition.set(selectedPosition[0], selectedPosition[1], selectedPosition[2]);
    inspectorVelocity.set(selectedVelocity[0], selectedVelocity[1], selectedVelocity[2]);
    updateInspectorTransformFields(inspectorPosition, inspectorVelocity);
    updateProbeGraph(world.y_probe_history_f64());

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

    const showGizmo = mode === "edit" && !world.body_is_static_at(selectedBodyIndex);
    gizmoGroup.visible = showGizmo;
    if (showGizmo) {
      gizmoGroup.position.copy(inspectorPosition);
    }

    const hashFull = world.state_hash();
    hud.textContent = [
      `t = ${world.time().toFixed(3)} s`,
      `step = ${world.step_count().toString()}`,
      `y = ${p[1].toFixed(4)} m`,
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
      accumulator += frameSeconds;
      let steps = 0;
      while (accumulator >= DT && steps < MAX_STEPS_PER_FRAME) {
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
  const appendConsoleEntries = setUpConsole();
  setUpProjectDrawer();
  setUpSceneView(updateProbeGraph, appendConsoleEntries).catch((err) => {
    const hud = document.getElementById("hud");
    if (hud) hud.textContent = `エラー: ${String(err)}`;
    console.error(err);
  });
}

main();
