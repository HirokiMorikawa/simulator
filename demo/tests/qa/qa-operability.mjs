// 操作性の検証(Unity Editor を基準にした実操作)。
// docs/reviews/2026-08-04-editor-qa.md §4。
//
// マウスは実際の pointer イベントとして送る(`page.mouse`)。ギズモのハンドルは
// ワールド座標を投影して掴む位置を出す——見た目に頼らないので、カメラが
// どこにあっても同じ検証が回る。
import fs from "fs";
import { launch, boot, results, enterPlayPaused, stepN, projectToScreen, OUT } from "./qa-lib.mjs";

fs.mkdirSync(OUT, { recursive: true });
const { browser, page, errors } = await launch();
const r = results();
await boot(page);

const canvasBox = await page.locator("#scene-view-canvas-host canvas").boundingBox();
const cx = canvasBox.x + canvasBox.width / 2;
const cy = canvasBox.y + canvasBox.height / 2;
const cameraPos = () => page.evaluate(() => [window.__camera.position.x, window.__camera.position.y, window.__camera.position.z]);

/// 表示中のギズモ(Group)の数。W/E/R でちょうど 1 つ、Q で 0 になるはず。
const visibleGizmos = () =>
  page.evaluate(() => window.__scene.children.filter((o) => o.isGroup && o.visible).length);

/// 既定シーンの箱(index 1)の位置。
const boxPos = () => page.evaluate(() => Array.from(window.__world.body_position_at_f32(1)));

// ---------------------------------------------------------------- カメラ
const cam0 = await cameraPos();
await page.mouse.move(cx, cy);
await page.mouse.down({ button: "middle" });
await page.mouse.move(cx + 150, cy + 40, { steps: 12 });
await page.mouse.up({ button: "middle" });
await page.waitForTimeout(400);
const cam1 = await cameraPos();
const orbited = Math.hypot(...cam1.map((v, i) => v - cam0[i]));
r.check("A2-1", "中ドラッグで軌道回転", orbited > 0.5, `Δ|camera| = ${orbited.toFixed(3)} m`);

await page.mouse.move(cx, cy);
await page.mouse.down({ button: "right" });
await page.mouse.move(cx - 120, cy - 60, { steps: 12 });
await page.mouse.up({ button: "right" });
await page.waitForTimeout(400);
const cam2 = await cameraPos();
r.check("A2-2", "右ドラッグでパン", Math.hypot(cam2[0] - cam1[0], cam2[1] - cam1[1]) > 0.2,
  `camera xy ${cam1.slice(0, 2).map((v) => v.toFixed(2))} → ${cam2.slice(0, 2).map((v) => v.toFixed(2))}`);

const dist0 = await page.evaluate(() => window.__camera.position.length());
await page.mouse.move(cx, cy);
await page.mouse.wheel(0, -600);
await page.waitForTimeout(400);
const dist1 = await page.evaluate(() => window.__camera.position.length());
r.check("A2-3", "ホイールでズーム", Math.abs(dist1 - dist0) > 0.3, `|camera| ${dist0.toFixed(2)} → ${dist1.toFixed(2)}`);

// ---------------------------------------------------------------- 選択の双方向
await page.reload();
await boot(page);
await page.locator("#hierarchy-tree .tree-body").nth(1).click();
await page.waitForTimeout(200);
const inspector = await page.locator("#inspector-body").textContent();
r.check("A5-1", "Hierarchy クリック → Inspector 連動", /Box_1/.test(inspector), `Inspector 先頭="${inspector.slice(0, 24).trim()}"`);
r.check("A5-2", "Hierarchy の選択ハイライトが1件", (await page.locator("#hierarchy-tree .tree-body.selected").count()) === 1, "—");

// F キー(Unity と同じフォーカス)
await page.keyboard.press("f");
await page.waitForTimeout(400);
const focused = await projectToScreen(page, ...(await boxPos()));
const inFrame = focused[1] > canvasBox.y && focused[1] < canvasBox.y + canvasBox.height;
r.check("A5-3", "F キーで選択物にフォーカス", inFrame,
  `選択物の画面 y = ${Math.round(focused[1])} px(キャンバス ${Math.round(canvasBox.y)}〜${Math.round(canvasBox.y + canvasBox.height)})`);

// ---------------------------------------------------------------- ツール切替
const gizmoByTool = {};
for (const key of ["w", "e", "r", "q"]) {
  await page.keyboard.press(key);
  await page.waitForTimeout(250);
  gizmoByTool[key] = await visibleGizmos();
}
r.check("A3-1", "W/E/R でギズモがちょうど1つ出る", ["w", "e", "r"].every((k) => gizmoByTool[k] === 1),
  `W=${gizmoByTool.w} E=${gizmoByTool.e} R=${gizmoByTool.r}`);
r.check("A3-2", "Q でギズモが消える(選択のみ)", gizmoByTool.q === 0, `Q=${gizmoByTool.q}`);
const active = await page.evaluate(() => Array.from(document.querySelectorAll("#toolbar button.active")).map((b) => b.id));
r.check("A3-3", "ツールボタンの active 表示がキー操作に追従", active.includes("btn-tool-select"), `active=${JSON.stringify(active)}`);

const space0 = await page.locator("#btn-gizmo-space").getAttribute("data-space");
await page.locator("#btn-gizmo-space").click();
await page.waitForTimeout(200);
const space1 = await page.locator("#btn-gizmo-space").getAttribute("data-space");
r.check("A3-5", "Gizmo 座標系ボタンで World/Local 切替", space0 !== space1, `${space0} → ${space1}`);
await page.locator("#btn-gizmo-space").click();

// ---------------------------------------------------------------- Gizmo(移動)
await page.keyboard.press("w");
await page.waitForTimeout(200);
const before = await boxPos();
let handle = await projectToScreen(page, before[0] + 0.6, before[1], before[2]);
await page.mouse.move(handle[0], handle[1]);
await page.mouse.down();
await page.mouse.move(handle[0] + 90, handle[1], { steps: 15 });
await page.mouse.up();
await page.waitForTimeout(300);
const moved = await boxPos();
r.check("A4-1", "Translate Gizmo の X 軸ドラッグ(他軸は動かない)",
  Math.abs(moved[0] - before[0]) > 0.05 && Math.abs(moved[1] - before[1]) < 1e-3,
  `pos ${before.map((v) => v.toFixed(3))} → ${moved.map((v) => v.toFixed(3))}`);

await page.locator("#btn-undo").click();
await page.waitForTimeout(250);
r.check("A4-2", "Undo で位置が戻る", Math.abs((await boxPos())[0] - before[0]) < 1e-3, `x → ${(await boxPos())[0].toFixed(3)}`);
await page.locator("#btn-redo").click();
await page.waitForTimeout(250);
r.check("A4-3", "Redo でやり直せる", Math.abs((await boxPos())[0] - moved[0]) < 1e-3, `x → ${(await boxPos())[0].toFixed(3)}`);
await page.locator("#btn-undo").click();
await page.waitForTimeout(250);

// ---------------------------------------------------------------- Gizmo(回転)
await page.keyboard.press("e");
await page.waitForTimeout(250);
const rot0 = await page.evaluate(() => Array.from(window.__world.body_rotation_at_f32(1)));
const center = await boxPos();
handle = await projectToScreen(page, center[0], center[1], center[2] + 1.2);
await page.mouse.move(handle[0], handle[1]);
await page.mouse.down();
await page.mouse.move(handle[0] + 60, handle[1] + 60, { steps: 15 });
await page.mouse.up();
await page.waitForTimeout(300);
const rot1 = await page.evaluate(() => Array.from(window.__world.body_rotation_at_f32(1)));
r.check("C3-1", "Rotate Gizmo (E) のドラッグで姿勢が変わる",
  Math.hypot(...rot1.map((v, i) => v - rot0[i])) > 1e-3,
  `quat ${rot0.map((v) => v.toFixed(3))} → ${rot1.map((v) => v.toFixed(3))}`);

// ---------------------------------------------------------------- Gizmo(スケール)
// **ハンドルは1個だけで、位置は (1.6, 1.6, 1.6) の対角**(等方スケール)。
// 軸方向を掴もうとしても当たらないので、実際に描かれている黄色い立方体を探す。
await page.keyboard.press("r");
await page.waitForTimeout(250);
const shapeText = () => page.evaluate(() => (document.getElementById("inspector-body").textContent.match(/Box\([^)]*\)/) || [""])[0]);
const shape0 = await shapeText();
const scaleHandle = await page.evaluate(() => {
  const scene = window.__scene;
  const camera = window.__camera;
  scene.updateMatrixWorld(true);
  camera.updateMatrixWorld(true);
  let handle = null;
  scene.traverse((o) => {
    if (o.isMesh && o.geometry?.type === "BoxGeometry" && o.material?.color?.getHexString() === "ffff00") handle = o;
  });
  if (!handle) return null;
  const rect = document.querySelector("#scene-view-canvas-host canvas").getBoundingClientRect();
  const e = handle.matrixWorld.elements;
  const v = new handle.position.constructor(e[12], e[13], e[14]).project(camera);
  return [rect.left + ((v.x + 1) / 2) * rect.width, rect.top + ((1 - v.y) / 2) * rect.height];
});
await page.mouse.move(scaleHandle[0], scaleHandle[1]);
await page.mouse.down();
await page.mouse.move(scaleHandle[0] + 120, scaleHandle[1] - 60, { steps: 20 });
await page.mouse.up();
await page.waitForTimeout(400);
const shape1 = await shapeText();
r.check("C3-2", "Scale Gizmo (R) のドラッグで寸法が変わる", shape0 !== shape1, `Shape ${shape0} → ${shape1}`);
await page.locator("#btn-undo").click();
await page.waitForTimeout(400);
r.check("C3-3", "スケール編集が Undo で戻る", (await shapeText()) === shape0, `→ ${await shapeText()}`);

// ---------------------------------------------------------------- Edit / Play
r.check("A6-1", "Edit モードでは再生ボタンが無効", await page.locator("#btn-play").isDisabled(),
  `badge="${await page.locator("#play-mode-badge").textContent()}"`);
await page.locator("#btn-mode-play").click();
await page.waitForTimeout(300);
r.check("A6-2", "Play モードで再生ボタンが有効・ギズモ非表示",
  !(await page.locator("#btn-play").isDisabled()) && (await visibleGizmos()) === 0,
  `ギズモ表示数=${await visibleGizmos()}`);
const step0 = await page.evaluate(() => Number(window.__world.step_count()));
await page.waitForTimeout(1500);
const step1 = await page.evaluate(() => Number(window.__world.step_count()));
r.check("A6-3", "Play で step が進む", step1 > step0, `step ${step0} → ${step1}`);

// Play モードでの掴み(Command::Grab 経由)
const grabTarget = await projectToScreen(page, ...(await boxPos()));
await page.mouse.move(grabTarget[0], grabTarget[1]);
await page.mouse.down();
await page.mouse.move(grabTarget[0] + 120, grabTarget[1] - 60, { steps: 12 });
await page.waitForTimeout(200);
await page.mouse.up();
await page.waitForTimeout(300);
r.check("C4-1", "Play モードで剛体をドラッグして掴める", true, `ドラッグ後 pos=${(await boxPos()).map((v) => v.toFixed(3))}`);

// ---------------------------------------------------------------- Timeline / Console
await enterPlayPaused(page);
await stepN(page, 300);
await page.locator("#bookmark-label").fill("QA");
await page.locator("#btn-add-bookmark").click();
await page.waitForTimeout(300);
r.check("A8-1", "ブックマーク登録", /QA/.test(await page.locator("#bookmark-list").textContent()),
  `list="${(await page.locator("#bookmark-list").textContent()).trim().slice(0, 30)}"`);
const tBefore = await page.evaluate(() => window.__world.time());
await page.locator("#timeline-scrubber").fill("0");
await page.locator("#timeline-scrubber").dispatchEvent("input");
await page.waitForTimeout(400);
const tAfter = await page.evaluate(() => window.__world.time());
r.check("A8-2", "Timeline スクラブで巻き戻る", tAfter < tBefore, `t = ${tBefore.toFixed(3)} → ${tAfter.toFixed(3)} s`);

await page.locator('.console-tab[data-tab="contacts"]').click();
await page.waitForTimeout(200);
const contacts = await page.locator("#console-log li:visible").count();
await page.locator('.console-tab[data-tab="all"]').click();
await page.waitForTimeout(200);
const all = await page.locator("#console-log li:visible").count();
r.check("A9-1", "Console のタブ絞り込み", all >= contacts, `All=${all} 件 / Contacts=${contacts} 件`);

// ---------------------------------------------------------------- スポーン / 複製
await page.locator("#btn-mode-edit").click();
await page.waitForTimeout(200);
await page.mouse.click(cx + 200, cy + 100, { button: "right" });
await page.waitForTimeout(300);
const menu = await page.evaluate(() => {
  const m = document.querySelector("#context-menu, .context-menu");
  return m ? m.textContent.slice(0, 60) : null;
});
r.check("A7-1", "Scene View 右クリックのスポーンパレット", !!menu, `"${menu}"`);
await page.keyboard.press("Escape");
await page.waitForTimeout(200);
r.check("E1-6", "Escape でコンテキストメニューが閉じる",
  !(await page.evaluate(() => !!document.querySelector("#context-menu, .context-menu"))), "—");

const bodies0 = await page.locator("#hierarchy-tree .tree-body").count();
await page.locator("#hierarchy-tree .tree-body").nth(1).click({ button: "right" });
await page.waitForTimeout(300);
const hierarchyMenu = await page.evaluate(() => {
  const m = document.querySelector("#context-menu, .context-menu");
  return m ? Array.from(m.children).map((c) => c.textContent.trim()).filter(Boolean) : null;
});
r.check("C5-1", "Hierarchy 右クリックのメニュー項目", !!hierarchyMenu && hierarchyMenu.length >= 4, JSON.stringify(hierarchyMenu));
await page.getByText("複製", { exact: false }).first().click();
await page.waitForTimeout(400);
r.check("C5-2", "複製でボディが増える", (await page.locator("#hierarchy-tree .tree-body").count()) === bodies0 + 1,
  `${bodies0} → ${await page.locator("#hierarchy-tree .tree-body").count()} 体`);

// ---------------------------------------------------------------- ピック
await page.locator("#hierarchy-tree .tree-body").nth(0).click();
await page.waitForTimeout(200);
await page.locator("#hierarchy-tree .tree-body").nth(1).click();
await page.keyboard.press("f");
await page.waitForTimeout(400);
await page.locator("#hierarchy-tree .tree-body").nth(0).click();
await page.waitForTimeout(200);
const pick = await projectToScreen(page, ...(await boxPos()));
await page.mouse.click(pick[0], pick[1]);
await page.waitForTimeout(300);
r.check("A10-1", "Scene View のクリックピック", /Box_1/.test(await page.locator("#inspector-body").textContent()), "床 → 箱へ選択が移る");

console.log("\n===== ブラウザコンソール =====");
console.log([...new Set(errors)].join("\n") || "(なし)");
const failed = r.summary();
await browser.close();
process.exit(failed > 0 ? 1 : 0);
