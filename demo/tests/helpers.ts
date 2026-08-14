import { expect, type Page } from "@playwright/test";

// スモークテスト・受け入れテストで共有するヘルパ(元は smoke.spec.ts 内に
// あったが、Playwright は *.spec.ts 同士の import を許さない
// 「should not import test file」ため、テストファイルではないこのモジュールへ
// 切り出した)。

/** ページ全体で発生した未捕捉例外を集める。favicon の 404 は除外する。 */
export function collectPageErrors(page: Page): string[] {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(String(e)));
  return errors;
}

/** wasm 初期化を待つ(Hierarchy にボディが並ぶまで)。 */
export async function waitForWorld(page: Page) {
  await expect(page.locator("#hierarchy-tree .tree-selectable").first()).toBeVisible({
    timeout: 30_000,
  });
}

/**
 * ツールバーの「＋ 追加」メニューから項目を選ぶ(群2)。
 * スポーン系8個のボタンはツールバーを3行ぶんの高さに膨らませていたため
 * 1つのメニューへ畳んだ。個々のボタンは `hidden` で DOM に残してあるが、
 * **`hidden` な要素は Playwright からクリックできない**ので、テストも
 * 実ユーザーと同じくメニュー経由で操作する。
 */
export async function addViaMenu(page: Page, label: string) {
  await page.click("#btn-add");
  await page.locator("#context-menu button", { hasText: label }).first().click();
}
