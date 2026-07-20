import * as THREE from "three";
import init, { WasmWorld } from "../pkg/sim_wasm.js";

// Phase 0 完了条件(docs/22-roadmap/01-phases.md):
// 「箱 1 個が落ちる最小 World が cargo test 緑 + ブラウザ表示 + ハッシュ 2 回一致」。
// このファイルは「ブラウザ表示」を満たす最小デモ。

const GRAVITY = 9.80665;
const DT = 1.0 / 120.0;
const INITIAL_HEIGHT = 10.0;
const BOX_HALF_EXTENT = 0.5;

async function main() {
  await init();
  const world = new WasmWorld(GRAVITY, DT, INITIAL_HEIGHT);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x111111);

  const camera = new THREE.PerspectiveCamera(
    50,
    window.innerWidth / window.innerHeight,
    0.1,
    1000,
  );
  camera.position.set(6, 4, 10);
  camera.lookAt(0, 3, 0);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(window.innerWidth, window.innerHeight);
  document.body.appendChild(renderer.domElement);

  scene.add(new THREE.AmbientLight(0xffffff, 0.5));
  const sun = new THREE.DirectionalLight(0xffffff, 1.0);
  sun.position.set(5, 10, 5);
  scene.add(sun);

  const box = new THREE.Mesh(
    new THREE.BoxGeometry(
      BOX_HALF_EXTENT * 2,
      BOX_HALF_EXTENT * 2,
      BOX_HALF_EXTENT * 2,
    ),
    new THREE.MeshStandardMaterial({ color: 0xffa500 }),
  );
  scene.add(box);

  const grid = new THREE.GridHelper(20, 20, 0x444444, 0x222222);
  scene.add(grid);

  window.addEventListener("resize", () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });

  const hud = document.getElementById("hud")!;

  // 固定 dt アキュムレータ: 描画フレームレートと物理刻みを分離し、
  // 決定論(docs/20-integration/02-determinism-replay.md §2「可変タイムステップ」の禁止)
  // を守ったまま可変フレームレートの画面表示を行う。
  let accumulator = 0;
  let lastTimeMs = performance.now();
  const MAX_STEPS_PER_FRAME = 240;

  function frame(nowMs: number) {
    const frameSeconds = Math.min((nowMs - lastTimeMs) / 1000, 0.25);
    lastTimeMs = nowMs;
    accumulator += frameSeconds;

    let steps = 0;
    while (accumulator >= DT && steps < MAX_STEPS_PER_FRAME) {
      world.step();
      accumulator -= DT;
      steps += 1;
    }

    const p = world.body_position_f32();
    box.position.set(p[0], p[1], p[2]);

    hud.textContent = [
      `t = ${world.time().toFixed(3)} s`,
      `step = ${world.step_count().toString()}`,
      `y = ${p[1].toFixed(4)} m`,
      `hash = ${world.state_hash()}`,
    ].join("\n");

    renderer.render(scene, camera);
    requestAnimationFrame(frame);
  }

  requestAnimationFrame(frame);
}

main().catch((err) => {
  const hud = document.getElementById("hud");
  if (hud) hud.textContent = `エラー: ${String(err)}`;
  console.error(err);
});
