import { expect, test, type Page } from "@playwright/test";
import { collectPageErrors } from "./helpers";
import { GUIDED_CATEGORIES } from "../src/guided-catalog";

// かんたんモード(`src/guided.ts`)の E2E。
//
// **ここが守るもの**: 「中の仕組みを知らない人が 3 手で見たい現象へ到達できる」
// という、このモードの存在理由そのもの。具体的には
//   ① カテゴリを選ぶ → ② 実験を選ぶ → ③「うごかす」を押す
// の 3 クリックで、シーンが読み込まれ・再生が始まり・数値が動くこと。
// カタログの全実験(カテゴリごとに 1 テスト)について同じことを確認するので、
// 「一覧には出るのに選ぶと何も起きない」実験が混ざったまま出荷されることはない。
//
// 既定の storageState(`playwright.config.ts`)は統合エディタ(pro)なので、
// **初めて開いた人**を再現するためにここでは空の storageState を使う。
test.use({ storageState: { cookies: [], origins: [] } });

/** 起動 → 起動オーバーレイが消える(wasm 初期化完了)まで待つ。 */
async function bootGuided(page: Page) {
  await page.goto("/");
  await expect(page.locator("#boot-overlay")).toBeHidden({ timeout: 30_000 });
}

/** ガイドパネルが出している「経過した時間」を秒で読む(表示は単位が変わる)。 */
async function elapsedSeconds(page: Page): Promise<number> {
  const raw = await page
    .locator("#guided-readout-time")
    .getAttribute("data-seconds");
  return Number.parseFloat(raw ?? "0") || 0;
}

/** ①②③ を順に押して 1 つの実験を動かす。 */
async function runThreeSteps(
  page: Page,
  categoryId: string,
  experimentId: string,
) {
  await page.click(`.guided-card[data-category-id="${categoryId}"]`);
  await page.click(`.guided-card[data-experiment-id="${experimentId}"]`);
  await page.click("#btn-guided-start");
}

test("初めて開くと3ステップのチューザが出る", async ({ page }) => {
  const errors = collectPageErrors(page);
  await bootGuided(page);

  await expect(page.locator("#guided-chooser")).toBeVisible();
  // ①が現在地、②③はまだ押せない(手順が一目で分かること自体が要件)。
  await expect(page.locator('.guided-step[data-step="1"]')).toHaveAttribute(
    "data-state",
    "current",
  );
  await expect(page.locator('.guided-step[data-step="2"]')).toBeDisabled();
  // カテゴリはカタログの数だけ並ぶ。
  await expect(page.locator(".guided-card[data-category-id]")).toHaveCount(
    GUIDED_CATEGORIES.length,
  );
  // 統合エディタ側のパネルは出さない(初見の人に見せる情報量を絞る)。
  await expect(page.locator("#hierarchy")).toBeHidden();
  await expect(page.locator("#project-drawer")).toBeHidden();
  expect(errors).toEqual([]);
});

test("3クリック(カテゴリ→実験→うごかす)でシミュレーションが動き出す", async ({
  page,
}) => {
  const errors = collectPageErrors(page);
  await bootGuided(page);

  await runThreeSteps(page, "drop", "d1-free-fall");

  // チューザは閉じ、選んだ実験がバーに出る。
  await expect(page.locator("#guided-chooser")).toBeHidden();
  await expect(page.locator("#guided-bar")).toContainText("ボールを落とす");
  // 「とめる」に変わっている = 自動で走り始めている(押す手数を増やさない)。
  await expect(page.locator("#btn-guided-play")).toHaveAttribute(
    "data-playing",
    "true",
  );
  // 見どころといまの数値が横に出ている。
  await expect(page.locator("#guided-panel")).toContainText("ここを見てください");
  await expect(page.locator("#guided-panel")).toContainText("ボールの高さ");

  // 時間が実際に進む。
  await expect.poll(() => elapsedSeconds(page), { timeout: 15_000 }).toBeGreaterThan(0);
  expect(errors).toEqual([]);
});

test("「とめる」「はじめから」が効く", async ({ page }) => {
  const errors = collectPageErrors(page);
  await bootGuided(page);
  await runThreeSteps(page, "drop", "d1-free-fall");

  await expect.poll(() => elapsedSeconds(page), { timeout: 15_000 }).toBeGreaterThan(0);

  await page.click("#btn-guided-play"); // とめる
  await expect(page.locator("#btn-guided-play")).toHaveAttribute(
    "data-playing",
    "false",
  );
  const stopped = await elapsedSeconds(page);
  await page.waitForTimeout(500);
  expect(await elapsedSeconds(page)).toBe(stopped);

  // 「はじめから」で t=0 の世界を作り直し、また走り出す。
  await page.click("#btn-guided-restart");
  await expect(page.locator("#btn-guided-play")).toHaveAttribute(
    "data-playing",
    "true",
  );
  expect(errors).toEqual([]);
});

test("つまみを動かすと、その設定でやり直す", async ({ page }) => {
  const errors = collectPageErrors(page);
  await bootGuided(page);
  await runThreeSteps(page, "drop", "d1-free-fall");
  await expect.poll(() => elapsedSeconds(page), { timeout: 15_000 }).toBeGreaterThan(0);

  // 「落とす高さ」を 5m にすると、作り直した直後の高さがその値になる。
  const slider = page.locator("#guided-panel #guided-knob-height");
  await slider.fill("5");
  await slider.dispatchEvent("change");

  const height = page.locator('#guided-panel dd[data-probe="0"]');
  await expect(height).not.toHaveText("—");
  // 20m から落とすシーンが 5m から始まっている(=つまみが物理へ届いている)。
  await expect
    .poll(
      async () => Number.parseFloat((await height.textContent()) ?? "99"),
      { timeout: 10_000 },
    )
    .toBeLessThan(6);
  expect(errors).toEqual([]);
});

test("かんたん ⇄ くわしい編集画面 を行き来でき、選択が残る", async ({ page }) => {
  const errors = collectPageErrors(page);
  await bootGuided(page);
  await runThreeSteps(page, "drop", "d1-free-fall");

  await page.click("#btn-guided-pro");
  await expect(page.locator("#toolbar")).toBeVisible();
  await expect(page.locator("#hierarchy")).toBeVisible();
  await expect(page.locator("#guided-bar")).toBeHidden();

  // 再読み込みしても統合エディタのまま(毎回チューザを通らされない)。
  await page.reload();
  await expect(page.locator("#boot-overlay")).toBeHidden({ timeout: 30_000 });
  await expect(page.locator("#toolbar")).toBeVisible();
  await expect(page.locator("#guided-chooser")).toBeHidden();

  // ツールバーの「🔰 かんたんモード」で戻れる。
  await page.click("#btn-simple-mode");
  await expect(page.locator("#guided-bar")).toBeVisible();
  await expect(page.locator("#toolbar")).toBeHidden();
  expect(errors).toEqual([]);
});

test("統合エディタへ戻すと、進む速さの主導権も返る", async ({ page }) => {
  const errors = collectPageErrors(page);
  await bootGuided(page);
  // かんたんモードは「1秒あたりの step 数」で世界を進める。統合エディタへ
  // 切り替えたらそれを返さないと、**時間倍率を変えても何も起きない**という
  // 説明のつかない状態になる(かんたんモードを一度でも通った人だけが踏む)。
  await runThreeSteps(page, "drop", "d1-free-fall");
  await expect.poll(() => elapsedSeconds(page), { timeout: 15_000 }).toBeGreaterThan(0);

  await page.click("#btn-guided-pro");
  await page.selectOption("#select-timescale", "0.125");
  await expect
    .poll(
      async () => {
        const text =
          (await page.locator("#timescale-effective").textContent()) ?? "×9";
        return Number.parseFloat(text.replace("×", ""));
      },
      { timeout: 10_000 },
    )
    .toBeLessThan(0.5);
  expect(errors).toEqual([]);
});

test("3Dに何も描かれない実験では、グラフが自動で開く", async ({ page }) => {
  const errors = collectPageErrors(page);
  await bootGuided(page);

  // D9(冷めるコーヒー)は剛体を1つも持たない——3D ビューは空のままなので、
  // 何も知らない人には「壊れている」ようにしか見えない。かんたんモードは
  // グラフを自動で開き、「下のグラフに現れます」と明示する。
  await runThreeSteps(page, "heat", "d9-cooling-coffee");
  await expect(page.locator("#probe-graphs")).toBeVisible();
  await expect(page.locator("#guided-panel")).toContainText("下のグラフ");

  // 温度が実際に下がる(単位はケルビンではなく ℃ で出す)。
  const temperature = page.locator('#guided-panel dd[data-probe="0"]');
  await expect(temperature).toContainText("℃", { timeout: 15_000 });
  expect(errors).toEqual([]);
});

test("dt の桁が極端なシーンでも、待たずに現象が進む", async ({ page }) => {
  const errors = collectPageErrors(page);
  await bootGuided(page);

  // D34(太陽系儀)は 1 step が 31555 秒。従来の「時間倍率」では上限の ×128 でも
  // 1 step に 4 分かかり、選んでも永遠に何も起きなかった。かんたんモードは
  // 「1 秒あたりの step 数」で進めるので、数秒で公転が見える。
  await runThreeSteps(page, "space", "d34-solar-system");
  await expect
    .poll(() => elapsedSeconds(page), { timeout: 20_000 })
    .toBeGreaterThan(1_000_000); // 秒。数秒待てば十数日ぶんは進む。
  // 表示は「31554896.93 秒」ではなく、桁に合った単位で書く。
  await expect(page.locator("#guided-readout-time")).toContainText("日");
  expect(errors).toEqual([]);
});

// カタログの全実験が実際に動くことを、カテゴリごとに確認する。
// 「一覧には出るのに選ぶと何も起きない/例外が出る」を出荷しないための網。
for (const category of GUIDED_CATEGORIES) {
  test(`カテゴリ「${category.title}」の実験がすべて動く`, async ({ page }) => {
    const errors = collectPageErrors(page);
    await bootGuided(page);

    for (const experiment of category.experiments) {
      // 初回はチューザが開いた状態で始まる(初めて開いた人の状態)。
      if (await page.locator("#guided-chooser").isHidden()) {
        await page.click("#btn-guided-choose");
      }
      await runThreeSteps(page, category.id, experiment.id);
      await expect(page.locator("#guided-bar")).toContainText(experiment.title);
      // 読み込めていれば時間が進む(プローブが無い実験もあるので時間で見る)。
      await expect
        .poll(() => elapsedSeconds(page), { timeout: 15_000 })
        .toBeGreaterThan(0);
    }
    expect(errors).toEqual([]);
  });
}
