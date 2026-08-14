// 統合エディタの手動 QA ハーネス共通部(docs/reviews/2026-08-04-editor-qa.md)。
//
// **demo/tests/smoke.spec.ts との違い**: スモークテストは「起動して wasm が
// 初期化され、主要操作でクラッシュしない」配線の健全性だけを守る。こちらは
// **実際にマウス/キーで操作した結果として物理量を読み出し、解析解と突き合わせる**
// ことを目的にする。Rust 側のテスト関数は一切呼ばない——UI から到達できる
// 値だけを使うのが、この QA の存在意義そのものである。
//
// Playwright のテストランナーには載せていない。判定が「解析解との誤差」であり
// CI のノイズ床(描画 fps に依存する step 進行など)と相性が悪いためで、
// 実行は README のとおり手動で行う。
import { chromium } from "playwright-core";

export const URL = process.env.QA_URL ?? "http://127.0.0.1:5199/";
export const OUT = process.env.QA_OUT ?? "/tmp/simulator-qa";

/// 開発コンテナに事前インストール済みの Chromium を使う。`@playwright/test` が
/// 要求するビルド番号と一致しないことがあるため、実行ファイルを明示する
/// (demo/playwright.config.ts の `PLAYWRIGHT_CHROMIUM_PATH` と同じ理由)。
/// SwiftShader を有効にしないと Three.js の WebGL コンテキストが取れない。
export async function launch() {
  const browser = await chromium.launch({
    executablePath: process.env.PLAYWRIGHT_CHROMIUM_PATH ?? "/opt/pw-browsers/chromium",
    args: [
      "--no-sandbox",
      "--use-gl=angle",
      "--use-angle=swiftshader",
      "--enable-unsafe-swiftshader",
      "--disable-gpu-sandbox",
    ],
  });
  const page = await browser.newPage({ viewport: { width: 1600, height: 1000 } });
  const errors = [];
  page.on("console", (m) => {
    if (m.type() === "error" || m.type() === "warning") errors.push(`[${m.type()}] ${m.text()}`);
  });
  page.on("pageerror", (e) => errors.push(`[pageerror] ${e.message}`));
  return { browser, page, errors };
}

/// wasm の初期化完了は HUD に `t = ` が現れることで判定する(main.ts の
/// `render()` が初回に回るまで HUD は空のまま)。
export async function boot(page) {
  await page.goto(URL, { waitUntil: "load" });
  await page.waitForFunction(
    () => {
      const h = document.getElementById("hud");
      return h && /t = /.test(h.textContent || "");
    },
    { timeout: 45000 },
  );
  await page.waitForTimeout(500);
}

export function results() {
  const rows = [];
  return {
    rows,
    check(id, name, ok, detail) {
      const verdict = ok ? "PASS" : "FAIL";
      rows.push({ id, name, verdict, detail });
      console.log(`${verdict}  ${id}  ${name}  — ${detail}`);
    },
    summary() {
      const fail = rows.filter((r) => r.verdict === "FAIL");
      console.log(`\n${rows.length - fail.length}/${rows.length} PASS`);
      return fail.length;
    },
  };
}

/// シーンギャラリーからの読み込み。**ツールバーの `<select>` を実際に操作する**
/// (`change` を発火させる)ので、UI 経由という前提を崩さない。
export async function loadScene(page, fileFragment) {
  const value = await page.evaluate((frag) => {
    const sel = document.getElementById("select-scene");
    const opt = Array.from(sel.options).find((o) => o.value.includes(frag));
    if (!opt) return null;
    sel.value = opt.value;
    sel.dispatchEvent(new Event("change"));
    return opt.value;
  }, fileFragment);
  if (!value) throw new Error(`scene not found in gallery: ${fileFragment}`);
  await page.waitForTimeout(600);
  return value;
}

/// **Play モードへ入って即座に一時停止する**。
///
/// `setMode("play")` は `playing = true` にするため、モード切替だけでは
/// 自由走行が始まってしまい、`⏭` の N step 送りが効かない
/// (`stepButton` のハンドラは `!playing` を要求する)うえ、
/// 待ち時間ぶんの step が測定に混入して再現性が失われる。
/// 2つの click を**同じ tick 内で**呼ぶことで、1 step も進めずに停止状態にする。
export async function enterPlayPaused(page) {
  await page.evaluate(() => {
    document.getElementById("btn-mode-play").click();
    document.getElementById("btn-play").click();
  });
  await page.waitForTimeout(120);
  return await page.evaluate(() => ({
    playButton: document.getElementById("btn-play").textContent,
    step: Number(window.__world.step_count()),
  }));
}

/// `⏭`(N step 送り)で決まった step 数だけ進める。停止中のみ有効。
export async function stepN(page, n) {
  await page.locator("#input-step-count").fill(String(n));
  await page.locator("#btn-step").click();
  await page.waitForTimeout(250);
}

/// シーン JSON の `probes` が宣言した系列の履歴。
/// **`DEFAULT_PROBE_CAPACITY = 600` のリングバッファ**なので、
/// 600 step を超えると先頭が失われ、添字と時刻の対応が崩れる点に注意
/// (crates/sim-world/src/scenario.rs:35)。
export function probes(page) {
  return page.evaluate(() => {
    const w = window.__world;
    const out = [];
    for (let i = 0; i < w.imported_probe_count(); i += 1) {
      out.push({ label: w.imported_probe_label_at(i), h: Array.from(w.imported_probe_history_f64(i)) });
    }
    return out;
  });
}

export function worldState(page) {
  return page.evaluate(() => ({
    t: window.__world.time(),
    step: Number(window.__world.step_count()),
    dt: Number(window.__world.read_component("dt", "")),
    hash: window.__world.state_hash(),
    residual: window.__world.energy_residual(),
  }));
}

/// ワールド座標を Scene View のクライアント座標へ投影する。
/// ギズモのハンドルを掴む座標を求めるのに使う。
export function projectToScreen(page, x, y, z) {
  return page.evaluate(([x, y, z]) => {
    const scene = window.__scene;
    const camera = window.__camera;
    scene.updateMatrixWorld(true);
    camera.updateMatrixWorld(true);
    const proto = scene.children.find((o) => o.isMesh);
    const V = proto.position.constructor;
    const rect = document.querySelector("#scene-view-canvas-host canvas").getBoundingClientRect();
    const v = new V(x, y, z).project(camera);
    return [rect.left + ((v.x + 1) / 2) * rect.width, rect.top + ((1 - v.y) / 2) * rect.height];
  }, [x, y, z]);
}
