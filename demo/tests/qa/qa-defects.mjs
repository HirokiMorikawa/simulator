// 2026-08-04 の QA で見つかった不具合の再現スクリプト。
// docs/reviews/2026-08-04-editor-qa.md §5 の各項目に 1:1 で対応する。
//
// **このスクリプトは、不具合が残っているあいだ FAIL する**。修正が入れば
// PASS に転じるので、そのまま回帰確認に使える(だから他の 2 本と違い、
// FAIL を「異常」ではなく「未修正」として読む)。
import fs from "fs";
import { launch, boot, results, loadScene, enterPlayPaused, stepN, OUT } from "./qa-lib.mjs";

fs.mkdirSync(OUT, { recursive: true });
const { browser, page, errors } = await launch();
const r = results();
await boot(page);

// ---------------------------------------------------------------- 不具合 1
// ツールバーが横にあふれ、右側のコントロールに到達できない。
await page.setViewportSize({ width: 1280, height: 800 });
await page.waitForTimeout(400);
const toolbar = await page.evaluate(() => {
  const tb = document.getElementById("toolbar");
  const out = {};
  for (const id of ["btn-settings", "select-scene", "select-layout", "hash-display"]) {
    const b = document.getElementById(id).getBoundingClientRect();
    out[id] = b.left >= 0 && b.right <= window.innerWidth;
  }
  const tall = Array.from(tb.querySelectorAll("label, span"))
    .filter((el) => el.children.length === 0 && el.textContent.trim() && el.getBoundingClientRect().height > 34)
    .map((el) => `${el.textContent.trim()}(h=${Math.round(el.getBoundingClientRect().height)})`);
  return { reachable: out, scrollWidth: tb.scrollWidth, clientWidth: tb.clientWidth, overflowX: getComputedStyle(tb).overflowX, tall };
});
const unreachable = Object.entries(toolbar.reachable).filter(([, ok]) => !ok).map(([id]) => id);
r.check("F1-1", "1280px でツールバーの全コントロールに届く", unreachable.length === 0,
  `画面外: ${unreachable.join(", ") || "なし"}(内容幅 ${toolbar.scrollWidth}px / 表示幅 ${toolbar.clientWidth}px、overflow-x=${toolbar.overflowX})`);
const scrolled = await page.evaluate(() => {
  const tb = document.getElementById("toolbar");
  tb.scrollLeft = 9999;
  return tb.scrollLeft;
});
r.check("F1-2", "はみ出し分を横スクロールで救える", scrolled > 0, `scrollLeft = ${scrolled}px`);
r.check("F1-3", "ツールバーのラベルが折り返さない", toolbar.tall.length === 0, `縦に伸びた要素: ${toolbar.tall.join(", ") || "なし"}`);
await page.setViewportSize({ width: 1600, height: 1000 });
await page.waitForTimeout(300);

// ---------------------------------------------------------------- 不具合 2
// frameCameraOnContent() が静的な床(20 m)を含むため、対象が極小になる/床下に潜る。
const cameraY = () => page.evaluate(() => window.__camera.position.y);
const belowGround = [];
for (const scene of ["d11-pendulum", "d12-ragdoll", "d13-rope"]) {
  await loadScene(page, scene);
  await page.waitForTimeout(500);
  const y = await cameraY();
  if (y < 0) belowGround.push(`${scene}(y=${y.toFixed(2)})`);
}
r.check("F2-1", "シーン読み込み後にカメラが地面より下へ潜らない", belowGround.length === 0,
  `地面下: ${belowGround.join(", ") || "なし"}`);

await loadScene(page, "d4-box-stack");
await page.waitForTimeout(500);
const occupancy = await page.evaluate(() => {
  const w = window.__world;
  const c = window.__camera;
  const s = window.__scene;
  s.updateMatrixWorld(true);
  c.updateMatrixWorld(true);
  const V = s.children.find((o) => o.isMesh).position.constructor;
  let minY = 9;
  let maxY = -9;
  for (let i = 0; i < w.body_count(); i += 1) {
    if (w.body_is_removed_at(i) || w.body_is_static_at(i)) continue;
    const p = w.body_position_at_f32(i);
    const v = new V(p[0], p[1], p[2]).project(c);
    minY = Math.min(minY, v.y);
    maxY = Math.max(maxY, v.y);
  }
  return (maxY - minY) / 2;
});
r.check("F2-2", "D4 積み木が画面の 15% 以上を占める", occupancy > 0.15,
  `動的ボディの画面占有(縦)= ${(occupancy * 100).toFixed(1)} %`);

// ---------------------------------------------------------------- 不具合 3
// 起動直後の既定シーンに自動フレーミングが掛からない。
await page.reload();
await boot(page);
const defaultFramed = await page.evaluate(() => {
  const c = window.__camera;
  const s = window.__scene;
  s.updateMatrixWorld(true);
  c.updateMatrixWorld(true);
  const box = s.children.find((o) => o.isMesh && o.geometry?.type === "BoxGeometry");
  const rect = document.querySelector("#scene-view-canvas-host canvas").getBoundingClientRect();
  const v = box.position.clone().project(c);
  const y = rect.top + ((1 - v.y) / 2) * rect.height;
  return { y: Math.round(y), top: Math.round(rect.top), bottom: Math.round(rect.top + rect.height) };
});
r.check("F3-1", "起動直後の既定シーンで箱が画面内にある",
  defaultFramed.y > defaultFramed.top && defaultFramed.y < defaultFramed.bottom,
  `箱の投影 y = ${defaultFramed.y} px(キャンバス ${defaultFramed.top}〜${defaultFramed.bottom})`);

// ---------------------------------------------------------------- 不具合 4
// シーン切替で Console がクリアされない。
await loadScene(page, "d8-scatter");
await enterPlayPaused(page);
await stepN(page, 600);
await loadScene(page, "d1-free-fall"); // ボディ1体・床なし = 接触が起きえない
await enterPlayPaused(page);
await stepN(page, 300);
const consoleState = await page.evaluate(() => ({
  contactLines: Array.from(document.querySelectorAll("#console-log li"))
    .filter((li) => /bodies=\d+,\d+/.test(li.textContent)).length,
  bodyCount: window.__world.body_count(),
}));
r.check("F4-1", "シーン切替で Console が引き継がれない", consoleState.contactLines === 0,
  `接触が起きえないシーン(body_count=${consoleState.bodyCount})に残る接触ログ ${consoleState.contactLines} 件`);

// ---------------------------------------------------------------- 不具合 5
// ⏭ が再生中は無反応(ボタンは有効のまま)。
await page.locator("#btn-mode-play").click();
await page.waitForTimeout(300);
const stepEnabledWhilePlaying = !(await page.locator("#btn-step").isDisabled());
const s0 = await page.evaluate(() => Number(window.__world.step_count()));
await page.locator("#input-step-count").fill("1");
await page.locator("#btn-step").click();
await page.waitForTimeout(100);
await page.locator("#input-step-count").fill("500");
await page.locator("#btn-step").click();
await page.waitForTimeout(100);
const s1 = await page.evaluate(() => Number(window.__world.step_count()));
r.check("F5-1", "再生中の ⏭ が無効化されている(空振りしない)", !stepEnabledWhilePlaying,
  `再生中もボタンは有効=${stepEnabledWhilePlaying}。1 と 500 を要求しても合計 ${s1 - s0} step(自由走行分のみ)`);

// ---------------------------------------------------------------- 不具合 6
// Unity 標準のショートカットが未実装。
await page.reload();
await boot(page);
await page.locator("#hierarchy-tree .tree-body").nth(1).click();
await page.keyboard.press("f");
await page.waitForTimeout(300);
const px = () => page.evaluate(() => window.__world.body_position_at_f32(1)[0]);
const x0 = await px();
const handle = await page.evaluate(() => {
  const s = window.__scene;
  const c = window.__camera;
  s.updateMatrixWorld(true);
  c.updateMatrixWorld(true);
  const b = s.children.find((o) => o.isMesh && o.geometry?.type === "BoxGeometry");
  const rect = document.querySelector("#scene-view-canvas-host canvas").getBoundingClientRect();
  const v = b.position.clone().setX(b.position.x + 0.6).project(c);
  return [rect.left + ((v.x + 1) / 2) * rect.width, rect.top + ((1 - v.y) / 2) * rect.height];
});
await page.mouse.move(handle[0], handle[1]);
await page.mouse.down();
await page.mouse.move(handle[0] + 90, handle[1], { steps: 12 });
await page.mouse.up();
await page.waitForTimeout(300);
const x1 = await px();
await page.keyboard.press("Control+z");
await page.waitForTimeout(300);
r.check("F6-1", "Ctrl+Z で Undo できる", Math.abs((await px()) - x0) < 1e-3,
  `x: ${x0.toFixed(3)} → ドラッグ ${x1.toFixed(3)} → Ctrl+Z 後 ${(await px()).toFixed(3)}`);

const n0 = await page.locator("#hierarchy-tree .tree-body").count();
await page.keyboard.press("Delete");
await page.waitForTimeout(400);
const n1 = await page.locator("#hierarchy-tree .tree-body").count();
r.check("F6-2", "Delete キーで選択中のボディを削除できる", n1 === n0 - 1, `${n0} → ${n1} 体`);
await page.keyboard.press("Control+d");
await page.waitForTimeout(400);
r.check("F6-3", "Ctrl+D で複製できる", (await page.locator("#hierarchy-tree .tree-body").count()) === n1 + 1,
  `${n1} → ${await page.locator("#hierarchy-tree .tree-body").count()} 体`);
await page.locator("#btn-mode-play").click();
await page.waitForTimeout(200);
const play0 = await page.locator("#btn-play").textContent();
await page.keyboard.press("Space");
await page.waitForTimeout(300);
r.check("F6-4", "Space で再生/一時停止", play0 !== (await page.locator("#btn-play").textContent()),
  `再生ボタン "${play0}" → "${await page.locator("#btn-play").textContent()}"`);

// ---------------------------------------------------------------- 不具合 7
// X キー(Gizmo の World/Local)が未実装。README とツールチップは実装済みと書いている。
await page.locator("#btn-mode-edit").click();
await page.keyboard.press("w");
const space0 = await page.locator("#btn-gizmo-space").getAttribute("data-space");
await page.keyboard.press("x");
await page.waitForTimeout(200);
const space1 = await page.locator("#btn-gizmo-space").getAttribute("data-space");
r.check("F7-1", "X キーで Gizmo の World/Local を切り替えられる", space0 !== space1, `${space0} → ${space1}`);

// ---------------------------------------------------------------- 不具合 8
// 該当ドメインを持たないシーンで HUD が NaN を出す。
await loadScene(page, "d1-free-fall");
await page.waitForTimeout(400);
const hud = await page.locator("#hud").textContent();
r.check("F8-1", "HUD に NaN が出ない", !/NaN/.test(hud), `HUD = "${hud.replace(/\n/g, " / ")}"`);

// ---------------------------------------------------------------- 不具合 9
// Probe 履歴が 600 サンプルで無言に打ち切られ、時間軸の表示も無い。
await enterPlayPaused(page);
await stepN(page, 900);
const probeState = await page.evaluate(() => ({
  length: window.__world.imported_probe_history_f64(0).length,
  step: Number(window.__world.step_count()),
  panelText: document.getElementById("probe-graphs").textContent.replace(/\s+/g, " ").trim(),
}));
r.check("F9-1", "Probe 履歴が step 数ぶん残る(打ち切りが無い)", probeState.length >= probeState.step,
  `${probeState.step} step 実行後の履歴長 = ${probeState.length} サンプル`);
r.check("F9-2", "Probe パネルに時刻の目盛りがある", /\d+(\.\d+)?\s*s/.test(probeState.panelText),
  `パネル文言 = "${probeState.panelText.slice(0, 80)}"`);

console.log("\n===== ブラウザコンソール =====");
console.log([...new Set(errors)].join("\n") || "(なし)");
r.summary();
console.log("\n(FAIL = 未修正の不具合。docs/reviews/2026-08-04-editor-qa.md を参照)");
await browser.close();
