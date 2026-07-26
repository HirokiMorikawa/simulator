import * as THREE from "three";
import init, { WasmWorld } from "../pkg/sim_wasm.js";
import "./style.css";

// 統合エディタ(docs/23-frontend/01-editor.md)の骨格増分。
//
// **縮約実装の理由**: このファイルはドッキングレイアウトの骨格(§1)と、既存の
// Phase 0 デモ(箱 1 個の落下)を Scene View パネルへ配線するところまでを扱う。
// Hierarchy/Inspector/Console/Project は現時点では静的なプレースホルダ内容
// (World API 経由の実データ接続は後続増分、`sim-wasm` 側がまだ `body_transforms`
// 以外のクエリ API を持たないため)。Gizmo・オーバーレイ・ピック・Command キュー・
// Edit/Play モードの分離(§4)・回路サブモード(§3)は全て後続増分。
// 再生/一時停止/1step ボタンだけは、既存の固定 dt アキュムレータの制御として
// 素朴に配線した(新しい World API を要さないため、骨格増分でも実装できる)。

const GRAVITY = 9.80665;
const DT = 1.0 / 120.0;
const INITIAL_HEIGHT = 10.0;
const BOX_HALF_EXTENT = 0.5;
const MAX_STEPS_PER_FRAME = 240;

function setUpLayoutPresetSwitcher() {
  const app = document.getElementById("app")!;
  const select = document.getElementById("select-layout") as HTMLSelectElement;
  select.addEventListener("change", () => {
    app.dataset.layout = select.value;
  });
}

function setUpConsolePlaceholder() {
  const log = document.getElementById("console-log")!;
  const entry = document.createElement("li");
  entry.textContent =
    "[INFO] World API 接続待ち — SolverDiagnostics の配線は後続増分(docs/23-frontend/01-editor.md §1.5)";
  log.appendChild(entry);
}

function setUpHierarchyPlaceholder() {
  const tree = document.getElementById("hierarchy-tree")!;
  const root = document.createElement("li");
  root.textContent = "World Root";
  const bodies = document.createElement("ul");
  bodies.className = "tree-nested";
  const bodyItem = document.createElement("li");
  bodyItem.textContent = "Bodies";
  const boxItem = document.createElement("ul");
  boxItem.className = "tree-nested";
  const box = document.createElement("li");
  box.textContent = "Box_1";
  boxItem.appendChild(box);
  bodyItem.appendChild(boxItem);
  bodies.appendChild(bodyItem);
  root.appendChild(bodies);
  tree.appendChild(root);
}

/// Inspector骨格を組み立てる。Transformの位置/速度は`updateInspectorTransform`で
/// 毎フレーム実データ(`WasmWorld::body_position_f32`/`body_velocity_f32`)へ
/// 更新される。Shape/Materialは`sim-wasm`側にまだ対応するクエリAPIが無いため
/// (設計上、World APIに無い機能はエディタ側からも追加できない——「World API-only
/// 制約」docs/23-frontend/01-editor.md §1.3)、Phase 0デモが実際に構築する内容と
/// 一致させた固定値のまま(後続増分でAPIが追加され次第、実データに置き換える)。
function setUpInspectorSkeleton(): (position: THREE.Vector3, velocity: THREE.Vector3) => void {
  const body = document.getElementById("inspector-body")!;
  body.innerHTML = `
    <div class="inspector-component">
      <h3>Box_1</h3>
      <div class="inspector-field"><span>Shape</span><span>Box(0.5,0.5,0.5)</span></div>
    </div>
    <div class="inspector-component">
      <h3>Transform</h3>
      <div class="inspector-field"><span>Position</span><span id="inspector-position">—</span></div>
      <div class="inspector-field"><span>Velocity</span><span id="inspector-velocity">—</span></div>
    </div>
    <div class="inspector-component">
      <h3>RigidBody</h3>
      <div class="inspector-field"><span>Material</span><span>鋼(炭素鋼)</span></div>
    </div>
  `;
  const positionField = document.getElementById("inspector-position")!;
  const velocityField = document.getElementById("inspector-velocity")!;
  return (position, velocity) => {
    positionField.textContent = `${position.x.toFixed(3)}, ${position.y.toFixed(3)}, ${position.z.toFixed(3)}`;
    velocityField.textContent = `${velocity.x.toFixed(3)}, ${velocity.y.toFixed(3)}, ${velocity.z.toFixed(3)}`;
  };
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

function setUpConsoleTabs() {
  const tabs = document.querySelectorAll<HTMLButtonElement>(".console-tab");
  tabs.forEach((tab) => {
    tab.addEventListener("click", () => {
      tabs.forEach((t) => t.classList.toggle("active", t === tab));
    });
  });
}

async function setUpSceneView(updateInspectorTransform: (position: THREE.Vector3, velocity: THREE.Vector3) => void) {
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

  const grid = new THREE.GridHelper(20, 20, 0x444444, 0x222222);
  scene.add(grid);

  const hud = document.getElementById("hud")!;
  const hashDisplay = document.getElementById("hash-display")!;
  const timelineTime = document.getElementById("timeline-time")!;
  const timelineStep = document.getElementById("timeline-step")!;
  const playButton = document.getElementById("btn-play") as HTMLButtonElement;
  const stepButton = document.getElementById("btn-step") as HTMLButtonElement;

  // Edit/Play モードの正式な分離(設計§4、Command キュー経由の介入)は後続増分。
  // ここでは単に「ステップ実行中かどうか」のトグルとして再生ボタンを配線する。
  let playing = true;
  playButton.textContent = "⏸";
  playButton.addEventListener("click", () => {
    playing = !playing;
    playButton.textContent = playing ? "⏸" : "▶";
  });
  stepButton.addEventListener("click", () => {
    if (!playing) {
      world.step();
      render();
    }
  });

  const inspectorPosition = new THREE.Vector3();
  const inspectorVelocity = new THREE.Vector3();

  function render() {
    const p = world.body_position_f32();
    box.position.set(p[0], p[1], p[2]);

    const v = world.body_velocity_f32();
    inspectorPosition.set(p[0], p[1], p[2]);
    inspectorVelocity.set(v[0], v[1], v[2]);
    updateInspectorTransform(inspectorPosition, inspectorVelocity);

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

    if (playing) {
      accumulator += frameSeconds;
      let steps = 0;
      while (accumulator >= DT && steps < MAX_STEPS_PER_FRAME) {
        world.step();
        accumulator -= DT;
        steps += 1;
      }
    }

    render();
    requestAnimationFrame(frame);
  }

  requestAnimationFrame(frame);
}

function main() {
  setUpLayoutPresetSwitcher();
  setUpHierarchyPlaceholder();
  const updateInspectorTransform = setUpInspectorSkeleton();
  setUpConsolePlaceholder();
  setUpConsoleTabs();
  setUpProjectDrawer();
  setUpSceneView(updateInspectorTransform).catch((err) => {
    const hud = document.getElementById("hud");
    if (hud) hud.textContent = `エラー: ${String(err)}`;
    console.error(err);
  });
}

main();
