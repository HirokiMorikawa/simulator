import { expect, test } from "@playwright/test";
import { collectPageErrors, waitForWorld } from "./helpers";

// **カメラのパンと Rotate Gizmo を「実際にマウスで操作して」守るテスト**
// (QA不具合 A2-2 / C3-1、docs/reviews/2026-08-04-coupling-qa.md の続き)。
//
// この 2 件は `qa-operability.mjs` の probe 判定では「動いていない」ことしか
// 分からず、**なぜ動かないか**は分からなかった。原因はどちらも
// 「掴めていない/イベントが届いていない」種類で、値の比較ではなく
// **操作そのものを再現する**テストでしか回帰を防げない。
//
// smoke.spec.ts が配線の健全性だけを見るのに対し、ここは
// 「右ドラッグで視点が動く」「リングを掴んで回すと姿勢が変わる」という
// **操作の成立**を見る。物理の正しさは Rust 側が担保するので触らない。

/** Scene View キャンバスの中心座標。 */
async function canvasCenter(page: import("@playwright/test").Page) {
  const box = await page.locator("#scene-view-canvas-host canvas").boundingBox();
  if (!box) throw new Error("Scene View のキャンバスが見つからない");
  return { cx: box.x + box.width / 2, cy: box.y + box.height / 2 };
}

const cameraPosition = (page: import("@playwright/test").Page) =>
  page.evaluate(() => {
    const c = (window as unknown as { __camera: { position: { x: number; y: number; z: number } } }).__camera;
    return [c.position.x, c.position.y, c.position.z] as [number, number, number];
  });

const distance = (a: [number, number, number], b: [number, number, number]) =>
  Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

/** 右ボタンで (dx, dy) だけドラッグする。 */
async function rightDrag(
  page: import("@playwright/test").Page,
  from: { cx: number; cy: number },
  dx: number,
  dy: number,
) {
  await page.mouse.move(from.cx, from.cy);
  await page.mouse.down({ button: "right" });
  await page.mouse.move(from.cx + dx, from.cy + dy, { steps: 12 });
  await page.mouse.up({ button: "right" });
  // enableDamping があるので、パンが収束するまで数フレーム待つ。
  await page.waitForTimeout(700);
}

const paletteOpen = (page: import("@playwright/test").Page) =>
  page.evaluate(() => !!document.querySelector("#context-menu, .context-menu"));

test("右ドラッグでカメラがパンし、連続して何度でもパンできる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);
  const center = await canvasCenter(page);

  // **同じ操作を 3 回続ける**のが要点(QA不具合 A2-2 の核心)。
  // 修正前は 1 回目だけ動き、2 回目以降は一切動かなかった
  // (実測 0.41 → 0.015 → 0.0003 m)——右ボタンを押した瞬間にスポーン
  // パレットが開き、カーソル直下に残ったメニューが次の右押しを
  // 奪ってキャンバスの `pointerdown` が発火しなくなるため。
  const displacements: number[] = [];
  for (let i = 0; i < 3; i += 1) {
    const before = await cameraPosition(page);
    await rightDrag(page, center, -120, -60);
    const after = await cameraPosition(page);
    displacements.push(distance(before, after));
    // ドラッグ中・直後にパレットが開いていてはならない。
    expect(
      await paletteOpen(page),
      `${i + 1} 回目の右ドラッグでスポーンパレットが開いた`,
    ).toBe(false);
  }

  for (const [i, d] of displacements.entries()) {
    expect(d, `${i + 1} 回目の右ドラッグでカメラが動かなかった(変位 ${d} m)`).toBeGreaterThan(0.1);
  }
  expect(errors).toEqual([]);
});

test("右クリック(ドラッグなし)ならスポーンパレットが開く", async ({ page }) => {
  await page.goto("/");
  await waitForWorld(page);
  const center = await canvasCenter(page);

  // パンと取り違えないための対に なる検証。移動量が閾値未満なら
  // 「クリック」なのでパレットを出す(判定は `pointerup` で行っている)。
  expect(await paletteOpen(page)).toBe(false);
  await page.mouse.click(center.cx, center.cy, { button: "right" });
  await page.waitForTimeout(300);
  expect(await paletteOpen(page)).toBe(true);
  await expect(page.locator("#context-menu, .context-menu")).toContainText("ここに箱を配置");
});

test("Rotate Gizmo のリングを掴んでドラッグすると姿勢が変わる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  // 既定シーンの箱(index 1)を選び、E で Rotate Gizmo を出す。
  await page.locator("#hierarchy-tree .tree-body").nth(1).click();
  await page.keyboard.press("e");
  await page.waitForTimeout(300);

  const rotationOf = () =>
    page.evaluate(() =>
      Array.from(
        (window as unknown as { __world: { body_rotation_at_f32: (i: number) => Float32Array } }).__world.body_rotation_at_f32(1),
      ),
    );
  const before = await rotationOf();

  // **リング上の点をワールド座標から投影して掴む**(見た目に頼らない)。
  // リング半径は Translate Gizmo の矢印長と揃えてあるので、選択物の中心から
  // z 方向へその距離だけ離れた点が X/Y リングの上に乗る。
  const handle = await page.evaluate(() => {
    const w = window as unknown as {
      __world: { body_position_at_f32: (i: number) => Float32Array };
      __scene: { updateMatrixWorld: (f: boolean) => void; children: { isMesh?: boolean; position: { constructor: new (x: number, y: number, z: number) => { project: (c: unknown) => { x: number; y: number } } } }[] };
      __camera: { updateMatrixWorld: (f: boolean) => void };
    };
    const p = w.__world.body_position_at_f32(1);
    w.__scene.updateMatrixWorld(true);
    w.__camera.updateMatrixWorld(true);
    const proto = w.__scene.children.find((o) => o.isMesh)!;
    const V = proto.position.constructor;
    const rect = document.querySelector("#scene-view-canvas-host canvas")!.getBoundingClientRect();
    // 1.2 = GIZMO_AXIS_LENGTH(= リング半径)。
    const v = new V(p[0], p[1], p[2] + 1.2).project(w.__camera);
    return [
      rect.left + ((v.x + 1) / 2) * rect.width,
      rect.top + ((1 - v.y) / 2) * rect.height,
    ] as [number, number];
  });

  await page.mouse.move(handle[0], handle[1]);
  await page.mouse.down();
  await page.mouse.move(handle[0] + 60, handle[1] + 60, { steps: 15 });
  await page.mouse.up();
  await page.waitForTimeout(300);

  const after = await rotationOf();
  const delta = Math.hypot(...after.map((v, i) => v - before[i]));
  expect(delta, `リングをドラッグしても姿勢が変わらない(quat 差 ${delta})`).toBeGreaterThan(1e-3);
  expect(errors).toEqual([]);
});
