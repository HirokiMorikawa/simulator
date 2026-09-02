import { expect, test } from "@playwright/test";
import { collectPageErrors, waitForWorld } from "./helpers";

// 増分「UI 品質の底上げ」で入れた操作系のテスト。
//
// ここが守るのは**操作が実際に届くか**である。見た目(色・余白・角丸)は
// テストの対象にしない——スクリーンショット比較は些細な差分で落ちてメンテナンス
// コストだけが残ることが多く、この規模のプロジェクトでは割に合わない。一方、
// 「掴んで動かせる」「キーだけで到達できる」「押しても何も起きないボタンが無い」
// は落ちたら壊れているので、こちらをテストにする。
//
// 対象は 3 系統:
//   1. パネルのリサイズ(設計 docs/23-frontend/01-editor.md §1「リサイズ…ができる」)
//   2. キーボードのみでの操作(QA 報告書 2026-08-04-editor-qa.md §5 が
//      「未検証」と明記していた領域)
//   3. 空状態と絞り込み(「何も無い」と「壊れている」を画面で区別できること)

test("スプリッターで Hierarchy の幅が変わり、再読み込み後も保たれ、ダブルクリックで戻る", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  const width = () =>
    page.evaluate(
      () => document.getElementById("hierarchy")!.getBoundingClientRect().width,
    );
  const before = await width();

  const splitter = page.locator("#splitter-left");
  const box = (await splitter.boundingBox())!;
  await page.mouse.move(box.x + box.width / 2, box.y + 200);
  await page.mouse.down();
  await page.mouse.move(box.x + box.width / 2 + 90, box.y + 200, { steps: 8 });
  await page.mouse.up();

  const dragged = await width();
  expect(dragged).toBeGreaterThan(before + 60);

  // 再読み込みしても保たれる(localStorage)。パネルの大きさは作業ごとに
  // 決まるもので、開くたびに直すのは苦痛なので残す。
  await page.reload();
  await waitForWorld(page);
  expect(await width()).toBeCloseTo(dragged, 0);

  // ダブルクリックで既定へ戻る。
  await page.locator("#splitter-left").dblclick();
  await expect.poll(width).toBeCloseTo(before, 0);

  expect(errors).toEqual([]);
});

test("スプリッターはキーボードでも動かせる(マウスを使わない操作)", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  const width = () =>
    page.evaluate(
      () => document.getElementById("inspector")!.getBoundingClientRect().width,
    );
  const before = await width();

  await page.locator("#splitter-right").focus();
  // Inspector はガターより右にあるので、左キーで**広がる**(`data-invert`)。
  await page.keyboard.press("ArrowLeft");
  await page.keyboard.press("ArrowLeft");
  const widened = await width();
  expect(widened).toBeGreaterThan(before);

  // Home で既定へ戻る。
  await page.keyboard.press("Home");
  await expect.poll(width).toBeCloseTo(before, 0);

  expect(errors).toEqual([]);
});

test("`?` でショートカット一覧が開き、Esc で閉じる", async ({ page }) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  const overlay = page.locator("#shortcut-overlay");
  await expect(overlay).toBeHidden();

  await page.keyboard.press("?");
  await expect(overlay).toBeVisible();
  // 実装済みのショートカットが一覧に載っていること(QA不具合7は「title には
  // 書いてあるが keydown に case が無い」という食い違いだった。一覧を同じ
  // ファイルに置いたので、両者のずれはここで気づける)。
  await expect(overlay).toContainText("移動ギズモ");
  await expect(overlay).toContainText("選択中のボディへカメラを寄せる");

  await page.keyboard.press("Escape");
  await expect(overlay).toBeHidden();

  // ツールバーのボタンからも開ける(キーを知らない利用者の入口)。
  await page.click("#btn-shortcuts");
  await expect(overlay).toBeVisible();

  expect(errors).toEqual([]);
});

test("Hierarchy を上下キーで辿ると選択が Inspector へ連動する", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  await page.locator("#hierarchy-tree").focus();
  await page.keyboard.press("ArrowUp");
  await expect(page.locator("#hierarchy-tree .tree-body.selected")).toHaveText(
    "Ground",
  );
  await expect(page.locator("#inspector-body h3").first()).toContainText("Ground");

  await page.keyboard.press("ArrowDown");
  await expect(page.locator("#hierarchy-tree .tree-body.selected")).toHaveText(
    "Box_1",
  );
  await expect(page.locator("#inspector-body h3").first()).toContainText("Box_1");

  // 選択状態は `aria-selected` にも出る(読み上げが class を読めないため)。
  await expect(
    page.locator("#hierarchy-tree .tree-body.selected"),
  ).toHaveAttribute("aria-selected", "true");

  expect(errors).toEqual([]);
});

test("Console / Project のタブは左右キーで移動でき、aria-selected が追従する", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  await page.locator('.console-tab[data-tab="all"]').focus();
  await page.keyboard.press("ArrowRight");
  await expect(page.locator(".console-tab.active")).toHaveText("Errors");
  await expect(page.locator(".console-tab.active")).toHaveAttribute(
    "aria-selected",
    "true",
  );
  await expect(page.locator('.console-tab[data-tab="all"]')).toHaveAttribute(
    "aria-selected",
    "false",
  );

  await page.keyboard.press("End");
  await expect(page.locator(".console-tab.active")).toHaveText("Events");

  expect(errors).toEqual([]);
});

test("Probe Graphs は履歴が無いあいだ空状態を出し、CSV ボタンを押せなくする", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  // 起動直後は 1 サンプルも無い。黒い矩形ではなく「何をすれば線が出るか」を出す。
  await expect(page.locator("#probe-empty")).toBeVisible();
  await expect(page.locator("#probe-canvas")).toBeHidden();
  // 押しても何も起きないボタンは「壊れている」と読まれる。
  await expect(page.locator("#btn-probe-csv")).toBeDisabled();

  // step を送れば線が出て、CSV も押せるようになる。
  await page.click("#btn-mode-play");
  await page.click("#btn-play"); // 一時停止して `⏭` を有効にする
  await page.fill("#input-step-count", "20");
  await page.click("#btn-step");

  await expect(page.locator("#probe-canvas")).toBeVisible();
  await expect(page.locator("#probe-empty")).toBeHidden();
  await expect(page.locator("#btn-probe-csv")).toBeEnabled();

  expect(errors).toEqual([]);
});

test("シーンギャラリーは検索とドメインで絞り込める", async ({ page }) => {
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  await page.click('.project-tab[data-tab="scenes"]');
  const cards = page.locator(".scene-gallery-list > li:not([hidden])");
  const total = await cards.count();
  expect(total).toBeGreaterThan(20); // 43 本(将来増減しても意味が変わらない下限)
  await expect(page.locator(".scene-gallery-count")).toContainText(
    `${total} / ${total}`,
  );

  // 番号で引ける。
  await page.fill(".scene-gallery-search", "D27");
  await expect(cards).toHaveCount(1);
  await expect(cards.first()).toContainText("二重スリット");

  // 題名・説明の日本語でも引ける。
  await page.fill(".scene-gallery-search", "振り子");
  expect(await cards.count()).toBeGreaterThan(0);

  // 一致が無いときは空状態を出す(黙って空欄にしない)。
  await page.fill(".scene-gallery-search", "存在しないシーン名");
  await expect(cards).toHaveCount(0);
  await expect(
    page.locator("#project-body .empty-state"),
  ).toBeVisible();

  // ドメインのチップで絞る。
  await page.fill(".scene-gallery-search", "");
  await page.locator(".scene-domain-chip", { hasText: "quantum" }).click();
  const filtered = await cards.count();
  expect(filtered).toBeGreaterThan(0);
  expect(filtered).toBeLessThan(total);
  for (const card of await cards.all()) {
    await expect(card).toContainText("quantum");
  }

  // カード全体がボタンなので、そのまま押して読み込める。
  await cards.first().locator("button").click();
  await expect(page.locator("#field-panel")).toBeVisible();

  expect(errors).toEqual([]);
});

test("起動中は読み込みオーバーレイが出て、World ができたら消える", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  // wasm の取得をわざと遅くして、オーバーレイが出ている瞬間を捕まえる。
  // 実機では 1〜3 秒かかる区間で、以前はここが**空のパネルが並ぶだけ**の
  // 画面だった(読み込み中なのか壊れているのか区別が付かない)。
  await page.route("**/*.wasm", async (route) => {
    await new Promise((resolve) => setTimeout(resolve, 1500));
    await route.continue();
  });
  await page.goto("/", { waitUntil: "commit" });

  const overlay = page.locator("#boot-overlay");
  await expect(overlay).toBeVisible();
  await expect(overlay).toContainText("読み込んでいます");

  await waitForWorld(page);
  await expect(overlay).toBeHidden();

  expect(errors).toEqual([]);
});

test("状態ハッシュのコピーは成功をトーストで伝える", async ({
  page,
  context,
}) => {
  // 設計 §2「クリックでフル 64 bit ハッシュをコピー」。押しても画面が一切
  // 変わらないと、コピーできたのか押し損ねたのか分からない。
  await context.grantPermissions(["clipboard-read", "clipboard-write"]);
  const errors = collectPageErrors(page);
  await page.goto("/");
  await waitForWorld(page);

  await page.click("#hash-display");
  await expect(page.locator('.toast[data-kind="success"]')).toContainText(
    "状態ハッシュをコピーしました",
  );
  // 実際にクリップボードへ入っている(トーストが嘘をついていない)。
  const copied = await page.evaluate(() => navigator.clipboard.readText());
  expect(copied).toMatch(/^[0-9a-f]{16}$/);

  expect(errors).toEqual([]);
});
